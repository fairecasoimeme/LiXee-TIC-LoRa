/*****************************************************************************
 *
 * MODULE:             app_lora_crypt
 *
 * COMPONENT:          app_lora_crypt.c
 *
 * DESCRIPTION:        AES-128-CTR encryption + CMAC MIC for LoRa TIC packets
 *
 *                     Le header (type + seq) reste en clair.
 *                     Le payload (bytes [2..N-1]) est chiffre AES-128-CTR.
 *                     Un MIC de 2 octets (CMAC tronque) est ajoute en fin.
 *
 *                     Nonce CTR (16 bytes) :
 *                       [0..7]  = MAC emetteur
 *                       [8..14] = 0x00
 *                       [15]    = seq
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "fsl_clock.h"

#include "app_lora_crypt.h"

#if (ZIGBEE_USE_FRAMEWORK != 0)
#include "SecLib.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define AES_BLOCK_SIZE  16

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vBuildNonce(uint8 *pu8Nonce, uint8 *pu8MAC, uint8 u8Seq);
PRIVATE void vComputeMIC(uint8 *pu8Data, uint8 u8Len, uint8 *pu8Key, uint8 *pu8MIC);
PRIVATE void vAesCtrCrypt(uint8 *pu8Data, uint8 u8Len, uint8 *pu8Key, uint8 *pu8Nonce);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_u8LoraCryptEncrypt
 *
 * DESCRIPTION:
 * Encrypt a LoRa TIC packet in-place and append 2-byte MIC.
 *
 * Input:  [type][seq][payload_0..payload_N]
 * Output: [type][seq][encrypted_0..encrypted_N][MIC_hi][MIC_lo]
 *
 * Steps:
 *   1) Compute MIC over cleartext (type + seq + payload)
 *   2) Encrypt payload bytes [2..u8Len-1] with AES-128-CTR
 *   3) Append 2-byte MIC at end
 *
 * RETURNS: new total packet length (u8Len + 2)
 *
 ****************************************************************************/
PUBLIC uint8 APP_u8LoraCryptEncrypt(uint8 *pu8Packet, uint8 u8Len,
                                     uint8 *pu8Key, uint8 *pu8MAC, uint8 u8Seq)
{
#if (ZIGBEE_USE_FRAMEWORK != 0)
    uint8 au8MIC[AES_BLOCK_SIZE];
    uint8 au8Nonce[AES_BLOCK_SIZE];

    /* Enable AES clock */
    CLOCK_EnableClock(kCLOCK_Aes);

    /* 1) Compute MIC over cleartext (full packet including header) */
    vComputeMIC(pu8Packet, u8Len, pu8Key, au8MIC);

    /* 2) Build nonce and encrypt payload (skip header bytes 0,1) */
    vBuildNonce(au8Nonce, pu8MAC, u8Seq);

    uint8 u8PayloadLen = u8Len - 2; /* bytes to encrypt (skip type+seq) */
    if (u8PayloadLen > 0)
    {
        vAesCtrCrypt(&pu8Packet[2], u8PayloadLen, pu8Key, au8Nonce);
    }

    /* 3) Append truncated MIC (2 bytes) */
    pu8Packet[u8Len]     = au8MIC[0];
    pu8Packet[u8Len + 1] = au8MIC[1];

    /* Disable AES clock */
    CLOCK_DisableClock(kCLOCK_Aes);

    return u8Len + LORA_CRYPT_MIC_SIZE;
#else
    /* No encryption available — pass through */
    return u8Len;
#endif
}

/****************************************************************************
 *
 * NAME: APP_bLoraCryptDecrypt
 *
 * DESCRIPTION:
 * Decrypt a LoRa TIC packet in-place and verify MIC.
 *
 * Steps:
 *   1) Extract MIC from last 2 bytes
 *   2) Decrypt payload bytes [2..u8Len-3] with AES-128-CTR
 *   3) Compute expected MIC over decrypted cleartext
 *   4) Compare MICs
 *
 * RETURNS: TRUE if MIC valid, FALSE if tampered/wrong key
 *
 ****************************************************************************/
PUBLIC bool_t APP_bLoraCryptDecrypt(uint8 *pu8Packet, uint8 u8Len,
                                     uint8 *pu8Key, uint8 *pu8MAC, uint8 u8Seq)
{
#if (ZIGBEE_USE_FRAMEWORK != 0)
    if (u8Len < (2 + LORA_CRYPT_MIC_SIZE))
        return FALSE;

    uint8 au8Nonce[AES_BLOCK_SIZE];
    uint8 au8MIC[AES_BLOCK_SIZE];

    uint8 u8DataLen = u8Len - LORA_CRYPT_MIC_SIZE; /* length without MIC */
    uint8 u8RecvMIC0 = pu8Packet[u8DataLen];
    uint8 u8RecvMIC1 = pu8Packet[u8DataLen + 1];

    /* Enable AES clock */
    CLOCK_EnableClock(kCLOCK_Aes);

    /* 1) Decrypt payload (CTR is symmetric) */
    vBuildNonce(au8Nonce, pu8MAC, u8Seq);

    uint8 u8PayloadLen = u8DataLen - 2;
    if (u8PayloadLen > 0)
    {
        vAesCtrCrypt(&pu8Packet[2], u8PayloadLen, pu8Key, au8Nonce);
    }

    /* 2) Compute expected MIC over decrypted cleartext */
    vComputeMIC(pu8Packet, u8DataLen, pu8Key, au8MIC);

    /* Disable AES clock */
    CLOCK_DisableClock(kCLOCK_Aes);

    /* 3) Compare */
    if (au8MIC[0] == u8RecvMIC0 && au8MIC[1] == u8RecvMIC1)
        return TRUE;

    return FALSE;
#else
    return TRUE; /* No crypto — always accept */
#endif
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vBuildNonce
 *
 * DESCRIPTION:
 * Build 16-byte CTR nonce from MAC address and sequence number.
 *   [0..7]  = MAC (8 bytes)
 *   [8..14] = 0x00
 *   [15]    = seq
 *
 ****************************************************************************/
PRIVATE void vBuildNonce(uint8 *pu8Nonce, uint8 *pu8MAC, uint8 u8Seq)
{
    memcpy(pu8Nonce, pu8MAC, 8);
    memset(&pu8Nonce[8], 0, 7);
    pu8Nonce[15] = u8Seq;
}

/****************************************************************************
 *
 * NAME: vComputeMIC
 *
 * DESCRIPTION:
 * Compute AES-128-CMAC over data, store full 16-byte tag.
 * Caller uses only first 2 bytes as truncated MIC.
 *
 ****************************************************************************/
PRIVATE void vComputeMIC(uint8 *pu8Data, uint8 u8Len, uint8 *pu8Key, uint8 *pu8MIC)
{
#if (ZIGBEE_USE_FRAMEWORK != 0)
    AES_128_CMAC(pu8Data, (uint32)u8Len, pu8Key, pu8MIC);
#else
    memset(pu8MIC, 0, 16);
#endif
}

/****************************************************************************
 *
 * NAME: vAesCtrCrypt
 *
 * DESCRIPTION:
 * AES-128-CTR encrypt/decrypt in-place.
 * SecLib AES_128_CTR requires input length multiple of 16.
 * We pad with zeros, encrypt, then keep only the relevant bytes.
 *
 ****************************************************************************/
PRIVATE void vAesCtrCrypt(uint8 *pu8Data, uint8 u8Len, uint8 *pu8Key, uint8 *pu8Nonce)
{
#if (ZIGBEE_USE_FRAMEWORK != 0)
    uint8 au8Padded[32];  /* Max 2 AES blocks = 32 bytes (enough for 19B payload) */
    uint8 au8Output[32];
    uint8 au8Counter[AES_BLOCK_SIZE];

    /* Pad input to multiple of 16 */
    uint8 u8PaddedLen = ((u8Len + AES_BLOCK_SIZE - 1) / AES_BLOCK_SIZE) * AES_BLOCK_SIZE;

    memset(au8Padded, 0, u8PaddedLen);
    memcpy(au8Padded, pu8Data, u8Len);

    /* Copy nonce as initial counter */
    memcpy(au8Counter, pu8Nonce, AES_BLOCK_SIZE);

    /* CTR encrypt */
    AES_128_CTR(au8Padded, (uint32)u8PaddedLen, au8Counter, pu8Key, au8Output);

    /* Copy back only the actual data bytes */
    memcpy(pu8Data, au8Output, u8Len);
#endif
}

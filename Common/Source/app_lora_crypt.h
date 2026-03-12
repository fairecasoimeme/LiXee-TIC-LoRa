/*****************************************************************************
 *
 * MODULE:             app_lora_crypt
 *
 * COMPONENT:          app_lora_crypt.h
 *
 * DESCRIPTION:        AES-128-CTR encryption + CMAC MIC for LoRa TIC packets
 *
 *                     Chiffrement applicatif des paquets DATA :
 *                     - AES-128-CTR pour le payload (octets [2..N-3])
 *                     - AES-128-CMAC tronque 2 octets comme MIC
 *                     - Header (type + seq) reste en clair
 *
 *                     Nonce CTR = MAC(8) + 0x00000000 + 0x000000 + seq(1)
 *
 ****************************************************************************/

#ifndef APP_LORA_CRYPT_H_
#define APP_LORA_CRYPT_H_

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* MIC size appended to encrypted packets */
#define LORA_CRYPT_MIC_SIZE    2

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Encrypt a LoRa TIC packet in-place and append 2-byte MIC
 *
 * Input packet format:  [type][seq][payload...]
 * Output packet format: [type][seq][encrypted_payload...][MIC_hi][MIC_lo]
 *
 * @param pu8Packet  Buffer containing the packet (must have room for +2 MIC)
 * @param u8Len      Original packet length (including type+seq header)
 * @param pu8Key     AES-128 key (16 bytes)
 * @param pu8MAC     Device MAC address (8 bytes, for nonce)
 * @param u8Seq      Sequence number (byte[1] of packet, used in nonce)
 * @return           New packet length (u8Len + LORA_CRYPT_MIC_SIZE)
 */
PUBLIC uint8 APP_u8LoraCryptEncrypt(uint8 *pu8Packet, uint8 u8Len,
                                     uint8 *pu8Key, uint8 *pu8MAC, uint8 u8Seq);

/**
 * @brief Decrypt a LoRa TIC packet in-place and verify MIC
 *
 * @param pu8Packet  Buffer containing the encrypted packet with MIC
 * @param u8Len      Total received length (including MIC)
 * @param pu8Key     AES-128 key (16 bytes)
 * @param pu8MAC     Emitter MAC address (8 bytes, for nonce)
 * @param u8Seq      Sequence number (byte[1] of packet)
 * @return           TRUE if MIC valid, FALSE if tampered/wrong key
 */
PUBLIC bool_t APP_bLoraCryptDecrypt(uint8 *pu8Packet, uint8 u8Len,
                                     uint8 *pu8Key, uint8 *pu8MAC, uint8 u8Seq);

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* APP_LORA_CRYPT_H_ */

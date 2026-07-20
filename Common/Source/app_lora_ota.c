/*****************************************************************************
 *
 * MODULE:             app_lora_ota
 *
 * COMPONENT:          app_lora_ota.c
 *
 * DESCRIPTION:        LoRa OTA firmware update receiver-side state machine.
 *                     Implements the protocol but flash write is stubbed
 *                     (Phase 3 = ecriture reelle dans slot B).
 *
 * STATUS: Phase 1-2 (skeleton + protocol). Flash write a venir.
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include <setjmp.h>
#include "dbg.h"
#include "ZTimer.h"

/* Required by FunctionLib's exception handling for flash ops */
jmp_buf *exception_buf;

#include "app_lora_ota.h"
#include "app_lora_pair.h"
#include "app_lora_tic.h"
#include "app_lora_crypt.h"
#include "sx1280.h"
#include "sx1280_hal.h"
#include "sx1280_config.h"
#include "fsl_gpio.h"
#include "OtaSupport.h"
#include "fsl_reset.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define TRACE_LORA_OTA  TRUE

#define LORA_TIC_PROTOCOL_VERSION_OTA  0x01

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsOtaContext_t sOta;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vBitmapSet(uint16 idx);
PRIVATE bool_t bBitmapIsSet(uint16 idx);
PRIVATE void vBitmapClear(void);
PRIVATE bool_t bIsBitmapComplete(void);

/****************************************************************************/
/***        Public Functions                                              ***/
/****************************************************************************/

PUBLIC void APP_vLoraOtaInit(void)
{
    memset(&sOta, 0, sizeof(sOta));
    sOta.state = OTA_STATE_IDLE;
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: Init done, state=IDLE\r\n");
}

PUBLIC eOtaState_t APP_eLoraOtaGetState(void)
{
    return sOta.state;
}

PUBLIC void APP_vLoraOtaEnterListen(void)
{
    if (!APP_bLoraPairIsPaired()) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: cannot enter listen, not paired\r\n");
        return;
    }
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: entering LISTEN mode (5 min timeout)\r\n");

    /* Stop pending normal TX activity (caller must also disable data TX timer) */
    memset(&sOta, 0, sizeof(sOta));
    sOta.state = OTA_STATE_LISTENING;
    sOta.u32LastBlockTime = 0;  /* TODO : utiliser un vrai millis() */

    /* Configure SX1280 in continuous RX */
    SX1280_SetStandby(SX1280_STANDBY_RC);
    SX1280_HAL_WaitOnBusy();
    SX1280_SetDioIrqParams(
        SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR,
        SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR);
    /* SetRx count=0xFFFF = mode RX continu (no timeout) sur SX1280 */
    SX1280_SetRx(0xFFFF);
}

PUBLIC void APP_vLoraOtaExitListen(void)
{
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: exiting LISTEN, back to data TX mode\r\n");
    SX1280_SetStandby(SX1280_STANDBY_RC);
    SX1280_HAL_WaitOnBusy();
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
    sOta.state = OTA_STATE_IDLE;
    /* Caller doit re-activer le timer LoRa data TX */
}

PUBLIC bool_t APP_bLoraOtaProcessPacket(uint8 *buf, uint16 len)
{
    if (len < 1) return FALSE;

    uint8 type = buf[0] & 0x0F;

    switch (type)
    {
    case LORA_TIC_PKT_OTA_START:
        APP_bLoraOtaOnStart(buf, len);
        return TRUE;

    case LORA_TIC_PKT_OTA_BLOCK:
        APP_bLoraOtaOnBlock(buf, len);
        return TRUE;

    case LORA_TIC_PKT_OTA_END:
        APP_bLoraOtaOnEnd(buf, len);
        return TRUE;

    default:
        return FALSE;  /* pas un paquet OTA */
    }
}

PUBLIC bool_t APP_bLoraOtaOnStart(uint8 *buf, uint16 len)
{
    if (len < LORA_OTA_START_SIZE) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: START too short (%d, need %d)\r\n",
                    len, LORA_OTA_START_SIZE);
        APP_vLoraOtaSendAck(0, LORA_OTA_STATUS_OUT_OF_RANGE);
        return FALSE;
    }

    /* Parse packet : [0]=type [1]=seq [2-3]=version [4-7]=size [8-11]=crc [12-13]=total_blocks [14]=block_size */
    uint8  seq          = buf[1];
    uint16 newVer       = ((uint16)buf[2] << 8) | buf[3];
    uint32 fwSize       = ((uint32)buf[4] << 24) | ((uint32)buf[5] << 16) |
                          ((uint32)buf[6] << 8)  | buf[7];
    uint32 fwCrc32      = ((uint32)buf[8] << 24) | ((uint32)buf[9] << 16) |
                          ((uint32)buf[10] << 8) | buf[11];
    uint16 totalBlocks  = ((uint16)buf[12] << 8) | buf[13];
    uint16 blockSize    = buf[14];

    DBG_vPrintf(TRACE_LORA_OTA,
        "LORA_OTA: START seq=%d ver=0x%04X size=%lu crc=0x%08lX blocks=%d block_size=%d\r\n",
        seq, newVer, fwSize, fwCrc32, totalBlocks, blockSize);

    /* Validation de base */
    if (totalBlocks > LORA_OTA_MAX_BLOCKS) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: too many blocks (%d > %d)\r\n",
                    totalBlocks, LORA_OTA_MAX_BLOCKS);
        APP_vLoraOtaSendAck(0, LORA_OTA_STATUS_FW_TOO_BIG);
        return FALSE;
    }
    if (blockSize != LORA_OTA_BLOCK_SIZE) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: unsupported block_size %d\r\n", blockSize);
        APP_vLoraOtaSendAck(0, LORA_OTA_STATUS_OUT_OF_RANGE);
        return FALSE;
    }

    /* OtaSupport NXP : start image (allocates and erases slot B) */
    otaResult_t otaRes = OTA_StartImage(fwSize);
    if (otaRes != gOtaSuccess_c) {
        DBG_vPrintf(TRACE_LORA_OTA,
            "LORA_OTA: OTA_StartImage failed: %d (size=%lu)\r\n",
            otaRes, fwSize);
        APP_vLoraOtaSendAck(0, LORA_OTA_STATUS_FLASH_ERROR);
        return FALSE;
    }
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: OTA_StartImage OK, slot B prepared\r\n");

    /* Reset context */
    memset(&sOta, 0, sizeof(sOta));
    sOta.state            = OTA_STATE_RECEIVING;
    sOta.u16NewVersion    = newVer;
    sOta.u32FirmwareSize  = fwSize;
    sOta.u32FirmwareCrc32 = fwCrc32;
    sOta.u16TotalBlocks   = totalBlocks;
    sOta.u16BlockSize     = blockSize;
    sOta.u16BlocksReceived = 0;
    sOta.u16ExpectedBlock = 0;  /* On attend le block 0 en premier */
    sOta.u32LastBlockTime = 0;
    sOta.u8LastSeq        = seq;

    APP_vLoraOtaSendAck(0xFFFF, LORA_OTA_STATUS_OK);
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: state=RECEIVING, expecting blocks 0..%d sequential\r\n",
                totalBlocks - 1);
    return TRUE;
}

PUBLIC bool_t APP_bLoraOtaOnBlock(uint8 *buf, uint16 len)
{
    if (sOta.state != OTA_STATE_RECEIVING) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: BLOCK received but state=%d, ignoring\r\n",
                    sOta.state);
        APP_vLoraOtaSendAck(0xFFFF, LORA_OTA_STATUS_NOT_STARTED);
        return FALSE;
    }
    if (len < LORA_OTA_BLOCK_HDR_SIZE) {
        return FALSE;
    }

    /* Parse : [0]=type [1]=seq [2-3]=block_idx [4..N]=data */
    uint16 blockIdx = ((uint16)buf[2] << 8) | buf[3];
    uint16 dataLen  = len - LORA_OTA_BLOCK_HDR_SIZE;

    if (blockIdx >= sOta.u16TotalBlocks) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: block_idx %d out of range (max %d)\r\n",
                    blockIdx, sOta.u16TotalBlocks);
        APP_vLoraOtaSendAck(blockIdx, LORA_OTA_STATUS_OUT_OF_RANGE);
        return FALSE;
    }

    /* Sequentiel uniquement : si le block n'est pas le suivant attendu,
     * on NACK avec l'indice attendu pour que le gateway resend depuis la. */
    if (blockIdx != sOta.u16ExpectedBlock) {
        DBG_vPrintf(TRACE_LORA_OTA,
            "LORA_OTA: block %d received, expected %d → NACK\r\n",
            blockIdx, sOta.u16ExpectedBlock);
        APP_vLoraOtaSendAck(sOta.u16ExpectedBlock, LORA_OTA_STATUS_OUT_OF_RANGE);
        return FALSE;
    }

    /* Push to flash slot B via OtaSupport NXP (sequential)
     *
     * IMPORTANT : le dernier bloc peut etre partiellement utile.
     * Le firmware fait u32FirmwareSize bytes, qui n'est generalement pas
     * un multiple exact de u16BlockSize (64). Le gateway envoie quand meme
     * 64 bytes en padding 0xFF, mais on doit ecrire uniquement les bytes
     * utiles dans le slot, sinon OTA_PushImageChunk retourne
     * gOtaInvalidParam_c (5) car il deborde la zone allouee. */
    uint16 u16WriteLen = dataLen;
    {
        uint32 u32ByteOffset = (uint32)blockIdx * (uint32)sOta.u16BlockSize;
        uint32 u32Remaining  = sOta.u32FirmwareSize - u32ByteOffset;
        if ((uint32)u16WriteLen > u32Remaining) {
            u16WriteLen = (uint16)u32Remaining;
        }
    }

    uint32 u32ImageLength = 0;
    uint32 u32ImageOffset = 0;
    otaResult_t otaRes = OTA_PushImageChunkBlocking(
        &buf[4], u16WriteLen, &u32ImageLength, &u32ImageOffset);
    if (otaRes != gOtaSuccess_c) {
        DBG_vPrintf(TRACE_LORA_OTA,
            "LORA_OTA: OTA_PushImageChunk failed: %d (block %d, offset %lu)\r\n",
            otaRes, blockIdx, u32ImageOffset);
        APP_vLoraOtaSendAck(blockIdx, LORA_OTA_STATUS_FLASH_ERROR);
        sOta.state = OTA_STATE_FAILED;
        OTA_CancelImage();
        return FALSE;
    }

    /* Track received */
    vBitmapSet(blockIdx);
    sOta.u16BlocksReceived++;
    sOta.u16ExpectedBlock = blockIdx + 1;
    sOta.u16LastBlockIdx = blockIdx;
    sOta.u32LastBlockTime = 0;  /* TODO : utiliser un vrai millis() */

    /* Print progress every 50 blocks */
    if ((sOta.u16BlocksReceived % 50) == 0) {
        uint32 pct = (uint32)sOta.u16BlocksReceived * 100UL / sOta.u16TotalBlocks;
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: progress %lu%% (%d/%d blocks)\r\n",
                    pct, sOta.u16BlocksReceived, sOta.u16TotalBlocks);
    }

    APP_vLoraOtaSendAck(blockIdx, LORA_OTA_STATUS_OK);
    return TRUE;
}

PUBLIC bool_t APP_bLoraOtaOnEnd(uint8 *buf, uint16 len)
{
    if (sOta.state != OTA_STATE_RECEIVING) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: END but state=%d\r\n", sOta.state);
        return FALSE;
    }
    if (len < LORA_OTA_END_SIZE) {
        return FALSE;
    }

    uint32 fullCrc = ((uint32)buf[2] << 24) | ((uint32)buf[3] << 16) |
                     ((uint32)buf[4] << 8)  | buf[5];

    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: END recv=%d/%d expected_crc=0x%08lX\r\n",
                sOta.u16BlocksReceived, sOta.u16TotalBlocks, fullCrc);

    sOta.state = OTA_STATE_FINALIZING;

    /* Verifier qu'on a recu tous les blocks */
    if (!bIsBitmapComplete()) {
        DBG_vPrintf(TRACE_LORA_OTA,
            "LORA_OTA: missing blocks (%d/%d received, expecting block %d next)\r\n",
            sOta.u16BlocksReceived, sOta.u16TotalBlocks, sOta.u16ExpectedBlock);
        APP_vLoraOtaSendStatus();
        sOta.state = OTA_STATE_RECEIVING;
        return FALSE;
    }

    /* Validate consistency between START and END CRC */
    if (sOta.u32FirmwareCrc32 != fullCrc) {
        DBG_vPrintf(TRACE_LORA_OTA,
            "LORA_OTA: CRC mismatch START(0x%08lX) vs END(0x%08lX)\r\n",
            sOta.u32FirmwareCrc32, fullCrc);
        APP_vLoraOtaSendAck(0, LORA_OTA_STATUS_FW_BAD_CRC);
        OTA_CancelImage();
        sOta.state = OTA_STATE_FAILED;
        return FALSE;
    }

    /* OtaSupport CommitImage : verifie l'integrite + inscrit dans la table OTA.
     * Le bootloader saura prendre le slot B au prochain reboot. */
    otaResult_t otaRes = OTA_CommitImage(NULL);
    if (otaRes != gOtaSuccess_c) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: OTA_CommitImage failed: %d\r\n", otaRes);
        APP_vLoraOtaSendAck(0, LORA_OTA_STATUS_FW_BAD_CRC);
        OTA_CancelImage();
        sOta.state = OTA_STATE_FAILED;
        return FALSE;
    }

    /* Set boot flag : le bootloader prendra slot B au prochain reset */
    OTA_SetNewImageFlag();

    sOta.state = OTA_STATE_COMMITTING;
    APP_vLoraOtaSendAck(0xFFFF, LORA_OTA_STATUS_OK);
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: *** COMMIT OK *** rebooting in 2 sec...\r\n");

    /* Petit delai pour laisser le ACK partir, puis reboot */
    for (volatile uint32 i = 0; i < 12000000UL; i++) { /* ~1-2 sec @ 12 MHz */ }

    /* Reset du SOC : redemarrage sur slot B */
    NVIC_SystemReset();

    /* unreachable */
    return TRUE;
}

PUBLIC void APP_vLoraOtaSendAck(uint16 u16BlockIdx, uint8 u8Status)
{
    /* Build OTA_ACK packet: 7 bytes raw + 2 bytes MIC = 9 bytes wire */
    uint8 au8Pkt[16];
    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_OTA << 4) | LORA_TIC_PKT_OTA_ACK;
    au8Pkt[1] = sOta.u8LastSeq;
    au8Pkt[2] = (u16BlockIdx >> 8) & 0xFF;
    au8Pkt[3] = u16BlockIdx & 0xFF;
    au8Pkt[4] = u8Status;
    au8Pkt[5] = 0x00;  /* reserved */
    au8Pkt[6] = 0x00;  /* reserved */

    /* Encrypt + add MIC */
    uint8 u8EncLen = APP_u8LoraCryptEncrypt(au8Pkt, LORA_OTA_ACK_SIZE,
                                             APP_pu8LoraPairGetKey(),
                                             APP_pu8LoraPairGetMAC(),
                                             au8Pkt[1]);

    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: TX ACK block=%d status=0x%02X (%d bytes wire)\r\n",
                u16BlockIdx, u8Status, u8EncLen);

    APP_bLoraTicTransmitRaw(au8Pkt, u8EncLen);
}

PUBLIC void APP_vLoraOtaSendStatus(void)
{
    /* Build OTA_STATUS packet: 8 bytes raw + 2 bytes MIC = 10 bytes wire */
    uint8 au8Pkt[16];
    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_OTA << 4) | LORA_TIC_PKT_OTA_STATUS;
    au8Pkt[1] = sOta.u8LastSeq;
    au8Pkt[2] = (uint8)sOta.state;
    au8Pkt[3] = (sOta.u16BlocksReceived >> 8) & 0xFF;
    au8Pkt[4] = sOta.u16BlocksReceived & 0xFF;
    au8Pkt[5] = (sOta.u16LastBlockIdx >> 8) & 0xFF;
    au8Pkt[6] = sOta.u16LastBlockIdx & 0xFF;
    au8Pkt[7] = 0x00;  /* reserved */

    uint8 u8EncLen = APP_u8LoraCryptEncrypt(au8Pkt, LORA_OTA_STATUS_SIZE,
                                             APP_pu8LoraPairGetKey(),
                                             APP_pu8LoraPairGetMAC(),
                                             au8Pkt[1]);

    DBG_vPrintf(TRACE_LORA_OTA,
        "LORA_OTA: TX STATUS state=%d recv=%d/%d (%d bytes wire)\r\n",
        sOta.state, sOta.u16BlocksReceived, sOta.u16TotalBlocks, u8EncLen);

    APP_bLoraTicTransmitRaw(au8Pkt, u8EncLen);
}

/* Process a single RX_DONE event from SX1280.
 * Reads payload, decrypts, dispatches to OTA handlers. */
PRIVATE void vProcessOtaRxDone(void)
{
    uint8 au8RxBuf[LORA_OTA_BLOCK_FULL_SIZE + 4];  /* max OTA packet + MIC */
    uint8 u8Len = 0;
    int8  i8Rssi = 0, i8Snr = 0;

    SX1280_GetRxPayload(au8RxBuf, &u8Len, &i8Rssi, &i8Snr);

    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: RX %d bytes RSSI=%d SNR=%d\r\n",
                u8Len, i8Rssi, i8Snr);

    /* Decrypt + verify MIC.
     * Le module crypt verifie aussi le MIC (CMAC tronque 2 bytes).
     * Si KO, on jette le paquet silencieusement.
     * IMPORTANT : pour OTA, le MAC source est celui du gateway, pas de notre device.
     * Le pairing actuel n'echange pas le MAC du gateway, donc on utilise notre MAC
     * comme reference (ce qui marche tant que la cle est unique par device). */
    uint8 u8Seq = au8RxBuf[1];
    if (!APP_bLoraCryptDecrypt(au8RxBuf, u8Len,
                                APP_pu8LoraPairGetKey(),
                                APP_pu8LoraPairGetMAC(),
                                u8Seq))
    {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: decrypt/MIC failed\r\n");
        return;
    }

    /* MIC stripped, len -= 2 */
    uint16 u16PlainLen = u8Len - 2;
    APP_bLoraOtaProcessPacket(au8RxBuf, u16PlainLen);
}

/* Restaure la radio en RX continu apres un TX d'ACK.
 * IMPORTANT : vReconfigureLoRaForTx (appele par bTransmitAndWait) reconfigure
 * DIO1 IRQ pour TX_DONE+TIMEOUT et payloadLength=N reel du paquet TX.
 * Pour la reception du prochain OTA_BLOCK il faut RESTAURER :
 *   - DIO1 IRQ = RX_DONE + CRC_ERROR
 *   - payloadLength = 255 (max, en EXPLICIT le header donne la taille reelle
 *     mais certains chipsets utilisent ce champ comme limite)
 * Sans ca, l'OTA_BLOCK arrive sur la radio mais DIO1 ne se leve jamais
 * → bloc ignore, pas d'ACK genere → gateway timeout. */
PRIVATE void vRestoreRxModeForOta(void)
{
    /* 1) Restaurer payloadLength = 255 */
    SX1280_LoRaPacketParams_t pktParams;
    pktParams.preambleLength = 0x18;
    pktParams.headerType     = SX1280_LORA_HEADER_EXPLICIT;
    pktParams.payloadLength  = 255;
    pktParams.crc            = SX1280_LORA_CRC_ON;
    pktParams.invertIQ       = SX1280_LORA_IQ_NORMAL;
    SX1280_SetPacketParams(&pktParams);

    /* 2) Restaurer IRQ DIO1 = RX_DONE + CRC_ERROR */
    SX1280_SetDioIrqParams(
        SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR,
        SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR);

    /* 3) Clear toutes les IRQ pendantes avant de redonner la main au RX */
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

    /* 4) RX continu (count=0xFFFF est traite comme continu par le SX1280) */
    SX1280_SetRx(0xFFFF);
}

PUBLIC void APP_vLoraOtaTick(void)
{
    /* Polling DIO1 pour RX_DONE quand en mode LISTENING ou RECEIVING */
    if (sOta.state == OTA_STATE_LISTENING || sOta.state == OTA_STATE_RECEIVING)
    {
        /* Check DIO1 (RX_DONE) */
        uint8 u8Dio1 = GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_DIO1_PIN);
        if (u8Dio1) {
            uint16 u16Irq = SX1280_GetIrqStatus();
            SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

            if (u16Irq & SX1280_IRQ_RX_DONE) {
                vProcessOtaRxDone();
                /* Re-arm RX si toujours en mode listen/recv.
                 * vProcessOtaRxDone a probablement TX un ACK, ce qui a tout
                 * casse cote IRQ + payloadLength → restaurer entierement. */
                if (sOta.state == OTA_STATE_LISTENING ||
                    sOta.state == OTA_STATE_RECEIVING) {
                    vRestoreRxModeForOta();
                }
            } else if (u16Irq & SX1280_IRQ_CRC_ERROR) {
                DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: CRC error in RX, re-arm\r\n");
                vRestoreRxModeForOta();
            }
        }
    }
}

/* ====================================================================== */
/* ===== Boot-time button detection + dedicated OTA loop ============== */
/* ====================================================================== */

#include "app.h"
#include "fsl_iocon.h"

PUBLIC bool_t APP_bLoraOtaButtonHeldAtBoot(void)
{
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: checking button (PIO%d)...\r\n", APP_BOARD_SW0_PIN);

    /* Init GPIO + IOCON pour PIO0 (USERINTERFACE = bouton sur OM15081) */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Pull-up sur la pin (le bouton tire LOW quand appuye) */
    IOCON_PinMuxSet(IOCON, 0, APP_BOARD_SW0_PIN,
                    IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);

    gpio_pin_config_t in_cfg = { kGPIO_DigitalInput, 0 };
    GPIO_PinInit(GPIO, 0, APP_BOARD_SW0_PIN, &in_cfg);

    /* Petit delai pour laisser la pin se stabiliser */
    for (volatile uint32 i = 0; i < 60000; i++);  /* ~5ms */

    /* Read une fois pour debug : si pas LOW, on n'attend pas plus */
    uint8 firstRead = GPIO_PinRead(GPIO, 0, APP_BOARD_SW0_PIN);
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: button pin first read = %d (0=pressed, 1=released)\r\n", firstRead);

    if (firstRead != 0) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: button NOT pressed at boot, normal mode\r\n");
        return FALSE;
    }

    /* Bouton apparemment pressé : echantillonner ~200ms pour confirmer
     * et eviter les rebonds / glitches au boot. */
    int held = 0, total = 0;
    for (int i = 0; i < 200; i++) {
        if (GPIO_PinRead(GPIO, 0, APP_BOARD_SW0_PIN) == 0) held++;
        total++;
        for (volatile uint32 d = 0; d < 12000; d++);  /* ~1ms */
    }

    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: button sampled %d/%d LOW\r\n", held, total);

    bool_t result = (held > (total * 9 / 10));  /* >90% LOW = appui confirme */
    if (result) {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: button held → ENTERING OTA MODE\r\n");
    } else {
        DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: button bouncy/released, normal mode\r\n");
    }
    return result;
}

PUBLIC void APP_vLoraOtaRunListenLoop(void)
{
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: *** OTA RECOVERY MODE ***\r\n");
    DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: Listening on op_channel for new firmware...\r\n");

    APP_vLoraOtaEnterListen();

    /* Tight loop : poll DIO1 + tick.
     * En cas de OTA_END reussi, on reboote dans APP_bLoraOtaOnEnd() (NVIC_SystemReset).
     * Sinon : timeout 5 minutes ou power-cycle. */
    uint32 u32IterCounter = 0;
    while (1) {
        APP_vLoraOtaTick();

        /* Check timeout 5 min : @12 MHz CPU, 1ms ≈ 12000 cycles
         * mais on est dans une boucle simple, donc compteur approximatif */
        u32IterCounter++;
        if (u32IterCounter > 30000000UL) {  /* ~5 min en boucle simple, approximatif */
            DBG_vPrintf(TRACE_LORA_OTA, "LORA_OTA: 5 min timeout, reboot\r\n");
            NVIC_SystemReset();
        }

        /* Petit delai pour ne pas spammer */
        for (volatile uint32 d = 0; d < 100; d++);
    }
}

/****************************************************************************/
/***        Local Functions (bitmap helpers)                              ***/
/****************************************************************************/

PRIVATE void vBitmapSet(uint16 idx)
{
    if (idx >= LORA_OTA_MAX_BLOCKS) return;
    sOta.au8BlockBitmap[idx >> 3] |= (1 << (idx & 7));
}

PRIVATE bool_t bBitmapIsSet(uint16 idx)
{
    if (idx >= LORA_OTA_MAX_BLOCKS) return FALSE;
    return (sOta.au8BlockBitmap[idx >> 3] & (1 << (idx & 7))) != 0;
}

PRIVATE void vBitmapClear(void)
{
    memset(sOta.au8BlockBitmap, 0, LORA_OTA_BITMAP_SIZE);
}

PRIVATE bool_t bIsBitmapComplete(void)
{
    for (uint16 i = 0; i < sOta.u16TotalBlocks; i++) {
        if (!bBitmapIsSet(i)) return FALSE;
    }
    return TRUE;
}

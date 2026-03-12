/*****************************************************************************
 *
 * MODULE:             app_lora_pair
 *
 * COMPONENT:          app_lora_pair.c
 *
 * DESCRIPTION:        LoRa pairing protocol — handshake + AES-128 key exchange
 *
 *                     Emitter state machine :
 *                       Boot PDM vide → PAIRING auto → PAIRED (cle recue)
 *                       Boot PDM ok   → PAIRED direct (TX donnees)
 *                       PAIRED + appui long → PAIRING (efface cle + re-pairing)
 *
 *                     Handshake (500ms cycle) :
 *                       1) TX PAIR_REQUEST (MAC 8 octets)
 *                       2) RX 200ms → attend PAIR_RESPONSE (cle AES 16 octets)
 *                       3) TX PAIR_CONFIRM (preuve = AES-ECB(MAC, key))
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "ZQueue.h"
#include "ZTimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "PDM.h"
#include "PDM_IDs.h"

#include "app_lora_pair.h"
#include "app_lora_tic.h"
#include "app_lora_crypt.h"
#include "App_Linky.h"
#include "app_main.h"
#include "app_events.h"
#include "app_blink_led.h"

#include "sx1280.h"
#include "sx1280_hal.h"
#include "sx1280_config.h"

/* ZPS for IEEE MAC address */
#include "zps_apl_zdo.h"

/* SecLib for AES-ECB (pairing proof) */
#if (ZIGBEE_USE_FRAMEWORK != 0)
#include "SecLib.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_LORA_PAIR
    #define TRACE_LORA_PAIR  TRUE
#else
    #define TRACE_LORA_PAIR  TRUE
#endif

/* LED pin */
#define LORA_LED_PIN        10
#define LORA_GPIO_PORT      SX1280_GPIO_PORT

/* DIO1 pour poll RX_DONE */
#define LORA_DIO1_PIN       SX1280_DIO1_PIN

/* Protocol version */
#define LORA_TIC_PROTOCOL_VERSION_PAIR  0x01

/* Flag HAL SPI silent (defini dans sx1280_hal.c) */
extern volatile uint8_t g_u8SX1280_HalSilent;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/* Pairing state */
PRIVATE eLoraPairState_t ePairState = LORA_PAIR_UNPAIRED;

/* Persistent pairing data */
PRIVATE tsLoraPairingData sPairingData;

/* Pairing timeout counter (incremented every 500ms tick) */
PRIVATE uint8 u8PairTickCount = 0;

/* Pairing sequence number */
PRIVATE uint8 u8PairSeq = 0;

/* LED blink counter */
PRIVATE uint8 u8LedBlinkCount = 0;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vLoadPairingFromPDM(void);
PRIVATE void vSavePairingToPDM(void);
PRIVATE void vErasePairingFromPDM(void);
PRIVATE void vReadMAC(uint8 *pu8MAC);
PRIVATE void vSendPairRequest(void);
PRIVATE bool_t bListenForPairResponse(void);
PRIVATE void vSendPairConfirm(void);
PRIVATE void vReconfigureForPairTx(void);


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_vLoraPairInit
 *
 * DESCRIPTION:
 * Initialize pairing module. Load state from PDM.
 *
 ****************************************************************************/
PUBLIC void APP_vLoraPairInit(void)
{
    /* Read MAC address from ZPS layer */
    vReadMAC(sPairingData.au8MAC);

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: MAC=");
    int i;
    for (i = 0; i < LORA_MAC_SIZE; i++)
        DBG_vPrintf(TRACE_LORA_PAIR, "%02X", sPairingData.au8MAC[i]);
    DBG_vPrintf(TRACE_LORA_PAIR, "\r\n");

    /* Try to load pairing data from PDM */
    vLoadPairingFromPDM();

    if (sPairingData.u32Magic == LORA_PAIR_MAGIC)
    {
        ePairState = LORA_PAIR_PAIRED;
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Paired (key loaded from PDM)\r\n");

        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Key=");
        for (i = 0; i < LORA_AES_KEY_SIZE; i++)
            DBG_vPrintf(TRACE_LORA_PAIR, "%02X", sPairingData.au8NetKey[i]);
        DBG_vPrintf(TRACE_LORA_PAIR, "\r\n");
    }
    else
    {
        /* Not paired (PDM empty/erased) → auto-pairing at boot */
        ePairState = LORA_PAIR_PAIRING;
        u8PairTickCount = 0;
        u8PairSeq = 0;
        u8LedBlinkCount = 0;

        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Not paired — auto-entering PAIRING mode\r\n");

        /* LED fast blink (250ms period: 100ms ON + 150ms OFF → visible 4Hz) */
        vStartBlinkTimer(250);

        /* Start pairing timer (300ms ticks — TX PAIR_REQUEST + RX listen) */
        ZTIMER_eStart(u8TimerLoraPair, ZTIMER_TIME_MSEC(LORA_PAIR_TX_INTERVAL_MS));
    }
}

/****************************************************************************
 *
 * NAME: APP_vLoraPairOnButton
 *
 * DESCRIPTION:
 * Handle button events for pairing.
 *   - Short press (BUTTON_DOWN) → ignored
 *   - Long press (LEAVE_AND_RESET) → enter pairing mode (erase key if paired)
 *
 ****************************************************************************/
PUBLIC void APP_vLoraPairOnButton(APP_teEventType eEvent)
{
    if (eEvent == APP_E_EVENT_LEAVE_AND_RESET)
    {
        /* Long press: enter pairing mode (erase key if was paired) */
        if (ePairState == LORA_PAIR_PAIRING)
        {
            /* Already pairing, ignore */
            DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Long press — already in pairing mode\r\n");
            return;
        }

        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Long press — entering pairing mode\r\n");

        if (ePairState == LORA_PAIR_PAIRED)
        {
            /* Erase existing pairing first */
            DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Erasing existing pairing data\r\n");
            ZTIMER_eStop(u8TimerLoRa);
            memset(sPairingData.au8NetKey, 0, LORA_AES_KEY_SIZE);
            sPairingData.u32Magic = 0;
            vErasePairingFromPDM();
        }

        ePairState = LORA_PAIR_PAIRING;
        u8PairTickCount = 0;
        u8PairSeq = 0;
        u8LedBlinkCount = 0;

        /* LED fast blink (150ms) for pairing mode */
        vStartBlinkTimer(150);

        /* Start pairing timer (300ms ticks) */
        ZTIMER_eStart(u8TimerLoraPair, ZTIMER_TIME_MSEC(LORA_PAIR_TX_INTERVAL_MS));
        return;
    }

    /* Short press: ignored (pairing is long press only) */
    if (eEvent == APP_E_EVENT_BUTTON_DOWN)
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Short press — ignored (use long press to pair)\r\n");
    }
}

/****************************************************************************
 *
 * NAME: APP_bLoraPairIsPaired
 *
 * DESCRIPTION:
 * Returns TRUE if device is paired and data TX is allowed.
 *
 ****************************************************************************/
PUBLIC bool_t APP_bLoraPairIsPaired(void)
{
    return (ePairState == LORA_PAIR_PAIRED);
}

/****************************************************************************
 *
 * NAME: APP_pu8LoraPairGetKey
 *
 ****************************************************************************/
PUBLIC uint8* APP_pu8LoraPairGetKey(void)
{
    return sPairingData.au8NetKey;
}

/****************************************************************************
 *
 * NAME: APP_pu8LoraPairGetMAC
 *
 ****************************************************************************/
PUBLIC uint8* APP_pu8LoraPairGetMAC(void)
{
    return sPairingData.au8MAC;
}

/****************************************************************************
 *
 * NAME: APP_eLoraPairGetState
 *
 ****************************************************************************/
PUBLIC eLoraPairState_t APP_eLoraPairGetState(void)
{
    return ePairState;
}

/****************************************************************************
 *
 * NAME: APP_cbTimerLoraPair
 *
 * DESCRIPTION:
 * Timer callback for pairing cycle (called every 500ms).
 *   1) Send PAIR_REQUEST
 *   2) Listen 200ms for PAIR_RESPONSE
 *   3) If received → send PAIR_CONFIRM, save, go PAIRED
 *   4) If timeout 30s → go UNPAIRED
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerLoraPair(void *pvParam)
{
    if (ePairState != LORA_PAIR_PAIRING)
        return;

    /* ---- PAIRING: TX/RX cycle (300ms, loops forever until paired) ---- */
    u8PairTickCount++;

    /* 1) Send PAIR_REQUEST (~50ms TX time at SF10) */
    vSendPairRequest();

    /* 2) Listen for PAIR_RESPONSE (200ms) */
    if (bListenForPairResponse())
    {
        /* 3) Got response! Send PAIR_CONFIRM */
        vSendPairConfirm();

        /* 4) Save to PDM and go PAIRED */
        sPairingData.u32Magic = LORA_PAIR_MAGIC;
        vSavePairingToPDM();

        ePairState = LORA_PAIR_PAIRED;

        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: *** PAIRED SUCCESSFULLY ***\r\n");
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Key=");
        int i;
        for (i = 0; i < LORA_AES_KEY_SIZE; i++)
            DBG_vPrintf(TRACE_LORA_PAIR, "%02X", sPairingData.au8NetKey[i]);
        DBG_vPrintf(TRACE_LORA_PAIR, "\r\n");

        /* Stop pairing timer */
        ZTIMER_eStop(u8TimerLoraPair);

        /* Switch LED from fast pairing blink to default TIC blink (3s)
         * App_Linky will adjust to 1s if TIC error */
        vStartBlinkTimer(3000);

        /* Start TIC acquisition + LoRa data TX */
        vAPP_LinkySensorSample();
        APP_vLoRaTicStart();
        return;
    }

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: No response, retry %d\r\n",
                u8PairTickCount);

    /* Restart pairing timer for next tick */
    ZTIMER_eStart(u8TimerLoraPair, ZTIMER_TIME_MSEC(LORA_PAIR_TX_INTERVAL_MS));
}

/****************************************************************************
 *
 * NAME: APP_vLoraPairErase
 *
 ****************************************************************************/
PUBLIC void APP_vLoraPairErase(void)
{
    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Erasing pairing data...\r\n");

    /* Stop timers */
    ZTIMER_eStop(u8TimerLoRa);
    ZTIMER_eStop(u8TimerLoraPair);

    /* Clear in-memory data */
    memset(sPairingData.au8NetKey, 0, LORA_AES_KEY_SIZE);
    sPairingData.u32Magic = 0;

    /* Erase from PDM */
    vErasePairingFromPDM();

    /* Return to UNPAIRED — user must long press again to start pairing */
    ePairState = LORA_PAIR_UNPAIRED;

    /* Stop LED blink */
    vStopBlinkTimer();

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Pairing erased, now UNPAIRED\r\n");
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vLoadPairingFromPDM
 *
 ****************************************************************************/
PRIVATE void vLoadPairingFromPDM(void)
{
    uint16 u16BytesRead = 0;
    PDM_teStatus eStatus;

    eStatus = PDM_eReadDataFromRecord(PDM_ID_LORA_PAIRING,
                                       &sPairingData,
                                       sizeof(tsLoraPairingData),
                                       &u16BytesRead);

    if (eStatus != PDM_E_STATUS_OK || u16BytesRead != sizeof(tsLoraPairingData))
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: PDM read failed (status=%d, read=%d)\r\n",
                    eStatus, u16BytesRead);
        sPairingData.u32Magic = 0;
    }
}

/****************************************************************************
 *
 * NAME: vSavePairingToPDM
 *
 ****************************************************************************/
PRIVATE void vSavePairingToPDM(void)
{
    PDM_teStatus eStatus;

    eStatus = PDM_eSaveRecordData(PDM_ID_LORA_PAIRING,
                                   &sPairingData,
                                   sizeof(tsLoraPairingData));

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: PDM save status=%d\r\n", eStatus);
}

/****************************************************************************
 *
 * NAME: vErasePairingFromPDM
 *
 ****************************************************************************/
PRIVATE void vErasePairingFromPDM(void)
{
    PDM_vDeleteDataRecord(PDM_ID_LORA_PAIRING);
}

/****************************************************************************
 *
 * NAME: vReadMAC
 *
 * DESCRIPTION:
 * Read the 64-bit IEEE MAC address from the ZPS stack.
 * Stores as big-endian 8 bytes.
 *
 ****************************************************************************/
PRIVATE void vReadMAC(uint8 *pu8MAC)
{
    uint64 u64Addr = ZPS_u64AplZdoGetIeeeAddr();

    /* Store big-endian */
    pu8MAC[0] = (uint8)(u64Addr >> 56);
    pu8MAC[1] = (uint8)(u64Addr >> 48);
    pu8MAC[2] = (uint8)(u64Addr >> 40);
    pu8MAC[3] = (uint8)(u64Addr >> 32);
    pu8MAC[4] = (uint8)(u64Addr >> 24);
    pu8MAC[5] = (uint8)(u64Addr >> 16);
    pu8MAC[6] = (uint8)(u64Addr >> 8);
    pu8MAC[7] = (uint8)(u64Addr & 0xFF);
}

/****************************************************************************
 *
 * NAME: vReconfigureForPairTx
 *
 * DESCRIPTION:
 * Quick reconfigure SX1280 for TX during pairing
 * (no full HW reset — just set IRQ for TX_DONE)
 *
 ****************************************************************************/
PRIVATE void vReconfigureForPairTx(void)
{
    /* Set standby */
    SX1280_SetStandby(SX1280_STANDBY_RC);
    SX1280_HAL_WaitOnBusy();

    /* IRQ: TX_DONE + TIMEOUT on DIO1 */
    SX1280_SetDioIrqParams(
        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,
        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);
}

/****************************************************************************
 *
 * NAME: vSendPairRequest
 *
 * DESCRIPTION:
 * Build and transmit PAIR_REQUEST packet (11 bytes):
 *   [0]   0x13 (v1, pair_request)
 *   [1]   sequence
 *   [2-9] MAC address (8 bytes)
 *   [10]  channel (0 = default)
 *
 ****************************************************************************/
PRIVATE void vSendPairRequest(void)
{
    uint8 au8Pkt[LORA_PAIR_REQUEST_SIZE];

    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_PAIR << 4) | LORA_TIC_PKT_PAIR_REQUEST;
    au8Pkt[1] = u8PairSeq++;
    memcpy(&au8Pkt[2], sPairingData.au8MAC, LORA_MAC_SIZE);
    au8Pkt[10] = 0; /* channel = default */

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: TX PAIR_REQUEST seq=%d\r\n", au8Pkt[1]);

    /* Silence SPI debug during TX */
    g_u8SX1280_HalSilent = 1;

    vReconfigureForPairTx();

    /* Send payload */
    SX1280_SendPayload(au8Pkt, LORA_PAIR_REQUEST_SIZE);

    /* Poll DIO1 for TX_DONE (max 500ms) */
    uint32 u32Start = 0;
    bool_t bDio1 = FALSE;
    for (u32Start = 0; u32Start < 6000000; u32Start++) /* ~500ms */
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
        {
            bDio1 = TRUE;
            break;
        }
        ZTIMER_vTask();  /* LED blink pendant le poll TX */
    }

    /* Clear IRQ */
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
    SX1280_SetStandby(SX1280_STANDBY_RC);

    g_u8SX1280_HalSilent = 0;

    if (!bDio1)
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: PAIR_REQUEST TX timeout\r\n");
}

/****************************************************************************
 *
 * NAME: bListenForPairResponse
 *
 * DESCRIPTION:
 * Switch to RX mode and listen for 200ms for a PAIR_RESPONSE.
 * Returns TRUE if valid PAIR_RESPONSE received.
 *
 ****************************************************************************/
PRIVATE bool_t bListenForPairResponse(void)
{
    g_u8SX1280_HalSilent = 1;

    /* Reconfigure IRQ for RX */
    SX1280_SetDioIrqParams(
        SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR,
        SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR);

    /* Start RX with timeout */
    SX1280_SetRx(LORA_PAIR_RX_TIMEOUT_MS);

    /* Poll DIO1 for RX_DONE or timeout */
    uint32 u32Wait;
    bool_t bDio1 = FALSE;
    for (u32Wait = 0; u32Wait < 3600000; u32Wait++) /* ~300ms max */
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
        {
            bDio1 = TRUE;
            break;
        }
        ZTIMER_vTask();  /* LED blink pendant le poll RX */
    }

    if (!bDio1)
    {
        SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
        SX1280_SetStandby(SX1280_STANDBY_RC);
        g_u8SX1280_HalSilent = 0;
        return FALSE;
    }

    /* Check IRQ status */
    uint16 u16Irq = SX1280_GetIrqStatus();
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

    if (!(u16Irq & SX1280_IRQ_RX_DONE))
    {
        SX1280_SetStandby(SX1280_STANDBY_RC);
        g_u8SX1280_HalSilent = 0;
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: RX IRQ=0x%04X (no RX_DONE)\r\n", u16Irq);
        return FALSE;
    }

    /* Read payload */
    uint8 au8RxBuf[32];
    uint8 u8RxLen = 0;
    int8 i8Rssi = 0, i8Snr = 0;

    SX1280_GetRxPayload(au8RxBuf, &u8RxLen, &i8Rssi, &i8Snr);
    SX1280_SetStandby(SX1280_STANDBY_RC);
    g_u8SX1280_HalSilent = 0;

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: RX len=%d RSSI=%d SNR=%d\r\n",
                u8RxLen, i8Rssi, i8Snr);

    /* Validate PAIR_RESPONSE */
    if (u8RxLen != LORA_PAIR_RESPONSE_SIZE)
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Wrong size (expected %d, got %d)\r\n",
                    LORA_PAIR_RESPONSE_SIZE, u8RxLen);
        return FALSE;
    }

    uint8 u8Type = au8RxBuf[0] & 0x0F;
    if (u8Type != LORA_TIC_PKT_PAIR_RESPONSE)
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Wrong type (expected 0x04, got 0x%02X)\r\n", u8Type);
        return FALSE;
    }

    uint8 u8Status = au8RxBuf[18];
    if (u8Status != 0x00)
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Response status=0x%02X (rejected)\r\n", u8Status);
        return FALSE;
    }

    /* Extract AES key (bytes 2..17) */
    memcpy(sPairingData.au8NetKey, &au8RxBuf[2], LORA_AES_KEY_SIZE);

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: PAIR_RESPONSE received! Key extracted.\r\n");

    return TRUE;
}

/****************************************************************************
 *
 * NAME: vSendPairConfirm
 *
 * DESCRIPTION:
 * Build and transmit PAIR_CONFIRM packet (15 bytes):
 *   [0]     0x15 (v1, pair_confirm)
 *   [1]     sequence
 *   [2-9]   MAC address
 *   [10-13] AES proof = first 4 bytes of AES-128-ECB(MAC padded, netKey)
 *   [14]    status 0x00 = OK
 *
 ****************************************************************************/
PRIVATE void vSendPairConfirm(void)
{
    uint8 au8Pkt[LORA_PAIR_CONFIRM_SIZE];

    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_PAIR << 4) | LORA_TIC_PKT_PAIR_CONFIRM;
    au8Pkt[1] = u8PairSeq++;
    memcpy(&au8Pkt[2], sPairingData.au8MAC, LORA_MAC_SIZE);

    /* AES proof: ECB encrypt MAC (padded to 16B) with netKey */
    {
        uint8 au8Input[16];
        uint8 au8Output[16];

        memset(au8Input, 0, 16);
        memcpy(au8Input, sPairingData.au8MAC, LORA_MAC_SIZE);

#if (ZIGBEE_USE_FRAMEWORK != 0)
        /* Enable AES clock */
        CLOCK_EnableClock(kCLOCK_Aes);
        AES_128_ECB_Encrypt(au8Input, 16, sPairingData.au8NetKey, au8Output);
        CLOCK_DisableClock(kCLOCK_Aes);
#else
        /* Fallback: just copy MAC as proof (no encryption) */
        memcpy(au8Output, au8Input, 16);
#endif

        memcpy(&au8Pkt[10], au8Output, 4);
    }

    au8Pkt[14] = 0x00; /* status OK */

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: TX PAIR_CONFIRM proof=%02X%02X%02X%02X\r\n",
                au8Pkt[10], au8Pkt[11], au8Pkt[12], au8Pkt[13]);

    g_u8SX1280_HalSilent = 1;

    vReconfigureForPairTx();
    SX1280_SendPayload(au8Pkt, LORA_PAIR_CONFIRM_SIZE);

    /* Poll DIO1 for TX_DONE (max 500ms) */
    uint32 u32Start;
    for (u32Start = 0; u32Start < 6000000; u32Start++)
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
            break;
        ZTIMER_vTask();  /* LED blink pendant le poll TX */
    }

    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
    SX1280_SetStandby(SX1280_STANDBY_RC);
    g_u8SX1280_HalSilent = 0;
}

/* LED management:
 *   PAIRING: vStartBlinkTimer(150) — very fast blink
 *   PAIRED:  managed by App_Linky.c — 1s (TIC error) / 3s (TIC OK)
 *   UNPAIRED: LED off
 */

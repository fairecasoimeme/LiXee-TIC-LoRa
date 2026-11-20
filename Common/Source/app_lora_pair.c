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
#include "app_lora_channels.h"

/* Horloge ms (meme source que ZTIMER) pour animer la LED pendant les
 * boucles de poll d'appairage sans reentrer dans ZTIMER_vTask(). */
#include "fsl_os_abstraction.h"
#ifdef WATCHDOG_ALLOWED
#include "fsl_wwdt.h"
extern uint8_t wdt_update_count;
#endif

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

/* seq de la derniere PAIR_REQUEST emise : le recepteur le renvoie en echo
 * dans sa PAIR_RESPONSE, ce qui ecarte une reponse destinee a un autre
 * emetteur (cf. bListenForPairResponse). */
PRIVATE uint8 u8LastReqSeq = 0;

/* Instant de reception de la PAIR_RESPONSE (ms) : origine du calendrier des
 * PAIR_CONFIRM (cf. LORA_PAIR_CONFIRM_FIRST_MS). */
PRIVATE uint32 u32PairRespMs = 0;

/* Nombre de ticks avant expiration de la fenetre d'appairage */
PRIVATE uint8 u8PairMaxTicks = 0;

/* Appairage en vigueur avant un appui long, conserve tel quel (RAM et PDM)
 * jusqu'au succes du nouvel appairage. Restaure si la fenetre expire. */
PRIVATE tsLoraPairingData sPrevPairing;
PRIVATE bool_t bHasPrevPairing = FALSE;

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
PRIVATE bool_t bSendPairConfirm(uint8 u8Seq);
PRIVATE void vSendPairConfirms(void);
PRIVATE void vPairWaitUntil(uint32 u32TargetMs);
PRIVATE void vReconfigureForPairTx(uint8 u8Channel, uint8 u8SF, int8 i8PowerDbm);
PRIVATE void vPollLedBlink(void);
PRIVATE void vStartPairingWindow(uint32 u32WindowMs);
PRIVATE void vEndPairingWindow(void);


/****************************************************************************
 *
 * NAME: vPollLedBlink
 *
 * DESCRIPTION:
 * Appelee dans les boucles d'attente de l'echange d'appairage (callback
 * APP_cbTimerLoraPair, ZTIMER_vTask() interdit). Ces boucles bloquent les
 * timers de blink : la LED est donc pilotee ici en direct, et maintenue
 * eteinte pour laisser toute la reserve d'energie a la radio.
 *
 ****************************************************************************/
PRIVATE void vPollLedBlink(void)
{
    /* LED eteinte pendant tout l'echange (PAIR_REQUEST -> PAIR_CONFIRM) :
     * elle puise dans la meme reserve que la radio (cf. LORA_PAIR_LED_*).
     * Les flashs reprennent entre deux tentatives, via le timer de blink. */
    GPIO_PinWrite(GPIO, LORA_GPIO_PORT, LORA_LED_PIN, 0);
}


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

        /* Validate stored channel; fallback to PAIR_CHANNEL if invalid (old PDM record) */
        if (!APP_bLoraChannelIsValid(sPairingData.u8OpChannel))
        {
            DBG_vPrintf(TRACE_LORA_PAIR,
                "LORA_PAIR: PDM has invalid op_channel %d, fallback to PAIR_CHANNEL %d\r\n",
                sPairingData.u8OpChannel, LORA_PAIR_CHANNEL);
            sPairingData.u8OpChannel = LORA_PAIR_CHANNEL;
        }

        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: Paired (key loaded from PDM, op_channel=%d)\r\n",
            sPairingData.u8OpChannel);

        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Key=");
        for (i = 0; i < LORA_AES_KEY_SIZE; i++)
            DBG_vPrintf(TRACE_LORA_PAIR, "%02X", sPairingData.au8NetKey[i]);
        DBG_vPrintf(TRACE_LORA_PAIR, "\r\n");

        /* Set RF frequency to operational channel */
        APP_vLoraChannelSet(sPairingData.u8OpChannel);
    }
    else
    {
        /* Not paired (PDM empty/erased) → auto-pairing at boot on PAIR_CHANNEL,
         * pendant une fenetre bornee (cf. LORA_PAIR_WINDOW_BOOT_MS). */
        bHasPrevPairing = FALSE;

        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Not paired — auto-entering PAIRING mode on ch %d (%lu s)\r\n",
                    LORA_PAIR_CHANNEL, (uint32)(LORA_PAIR_WINDOW_BOOT_MS / 1000));

        vStartPairingWindow(LORA_PAIR_WINDOW_BOOT_MS);
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
            /* On NE TOUCHE PAS a l'appairage en cours (RAM ni PDM) : il est
             * mis de cote et restaure si la fenetre expire sans succes. Avant,
             * il etait efface ici, si bien qu'un appairage interrompu, ou capte
             * par une autre box, laissait l'emetteur sans cle valide. */
            DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Current pairing kept until the new one succeeds\r\n");
            ZTIMER_eStop(u8TimerLoRa);
            sPrevPairing = sPairingData;
            bHasPrevPairing = TRUE;
        }

        vStartPairingWindow(LORA_PAIR_WINDOW_BUTTON_MS);
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
 * NAME: APP_u8LoraPairGetOpChannel
 *
 ****************************************************************************/
PUBLIC uint8 APP_u8LoraPairGetOpChannel(void)
{
    if (APP_bLoraChannelIsValid(sPairingData.u8OpChannel))
        return sPairingData.u8OpChannel;
    return LORA_PAIR_CHANNEL;
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
 * NAME: APP_u8LoraPairGetSF / APP_u8LoraPairGetTxIntervalSec
 *
 * DESCRIPTION:
 * Config radio pilotable par downlink. Un enregistrement PDM ecrit par une
 * version anterieure porte 0 dans ces champs (ex-octets reserves) : on
 * retombe alors sur le defaut.
 *
 ****************************************************************************/
PUBLIC uint8 APP_u8LoraPairGetSF(void)
{
    uint8 u8SF = sPairingData.u8SpreadingFactor;
    if (u8SF < LORA_SF_MIN || u8SF > LORA_SF_MAX)
        return LORA_SF_DEFAULT;
    return u8SF;
}

PUBLIC int8 APP_i8LoraPairGetTxPowerDbm(void)
{
    int16 i16Power = (int16)sPairingData.u8TxPowerCode - LORA_TX_POWER_CODE_OFFSET;
    if (sPairingData.u8TxPowerCode == 0 ||
        i16Power < LORA_TX_POWER_MIN_DBM || i16Power > LORA_TX_POWER_MAX_DBM)
        return LORA_TX_POWER_MAX_DBM;
    return (int8)i16Power;
}

PUBLIC uint8 APP_u8LoraPairGetTxIntervalSec(void)
{
    uint8 u8Sec = sPairingData.u8TxIntervalSec;
    if (u8Sec < LORA_TX_INTERVAL_SEC_MIN || u8Sec > LORA_TX_INTERVAL_SEC_MAX)
        return LORA_TX_INTERVAL_SEC_DEFAULT;
    return u8Sec;
}

/****************************************************************************
 *
 * NAME: APP_bLoraPairSetOpChannel / SetSF / SetTxIntervalSec
 *
 * DESCRIPTION:
 * Appliquent une config en RAM uniquement. Le module downlink ne persiste
 * qu'apres confirmation du recepteur, pour pouvoir revenir en arriere si la
 * nouvelle config coupe la liaison.
 *
 ****************************************************************************/
PUBLIC bool_t APP_bLoraPairSetOpChannel(uint8 u8Channel)
{
    if (!APP_bLoraChannelIsValid(u8Channel))
        return FALSE;
    sPairingData.u8OpChannel = u8Channel;
    return TRUE;
}

PUBLIC bool_t APP_bLoraPairSetSF(uint8 u8SF)
{
    if (u8SF < LORA_SF_MIN || u8SF > LORA_SF_MAX)
        return FALSE;
    sPairingData.u8SpreadingFactor = u8SF;
    return TRUE;
}

PUBLIC bool_t APP_bLoraPairSetTxIntervalSec(uint8 u8Sec)
{
    if (u8Sec < LORA_TX_INTERVAL_SEC_MIN || u8Sec > LORA_TX_INTERVAL_SEC_MAX)
        return FALSE;
    sPairingData.u8TxIntervalSec = u8Sec;
    return TRUE;
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

    /* ---- PAIRING: TX/RX cycle, jusqu'au succes ou a la fin de la fenetre ---- */
    u8PairTickCount++;
    if (u8PairTickCount > u8PairMaxTicks)
    {
        vEndPairingWindow();
        return;
    }

    /* 1) Send PAIR_REQUEST (~250ms TX time at SF11) */
    vSendPairRequest();

    /* 2) Listen for PAIR_RESPONSE (200ms) */
    if (bListenForPairResponse())
    {
        /* 3) Got response! Send PAIR_CONFIRM */
        vSendPairConfirms();
        PRINTF("[PAIR-CB] PAIR_CONFIRM sent, now SavePairingToPDM\r\n");

        /* 4) Save to PDM and go PAIRED */
        sPairingData.u32Magic = LORA_PAIR_MAGIC;
        vSavePairingToPDM();
        PRINTF("[PAIR-CB] PDM save done\r\n");

        /* Le nouvel appairage remplace l'ancien : plus rien a restaurer */
        bHasPrevPairing = FALSE;
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
        vSetBlinkOnTime(LORA_LED_FLASH_DEFAULT_MS);
        vStartBlinkTimer(3000);

        /* Start TIC acquisition + LoRa data TX */
        vAPP_LinkySensorSample();
        APP_vLoRaTicStart();
        return;
    }

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: No response, retry %d\r\n",
                u8PairTickCount);

    /* Radio en sommeil jusqu'a la prochaine tentative : la prochaine
     * PAIR_REQUEST refait un reset et une configuration complete. */
    SX1280_SetSleep();

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
 * NAME: vStartPairingWindow
 *
 * DESCRIPTION:
 * Entre en mode appairage pour u32WindowMs au plus (cf. LORA_PAIR_WINDOW_*).
 *
 * Le seq de depart est tire au hasard : tous les emetteurs repartaient de 0,
 * si bien que deux emetteurs en appairage au meme moment emettaient les memes
 * seq et ne pouvaient pas distinguer leurs reponses (cf. u8LastReqSeq).
 *
 ****************************************************************************/
PRIVATE void vStartPairingWindow(uint32 u32WindowMs)
{
    ePairState = LORA_PAIR_PAIRING;
    u8PairTickCount = 0;
    u8PairMaxTicks = (uint8)((u32WindowMs + LORA_PAIR_TX_INTERVAL_MS - 1) / LORA_PAIR_TX_INTERVAL_MS);
    u8PairSeq = (uint8)(OSA_TimeGetMsec() ^ sPairingData.au8MAC[7] ^
                        (sPairingData.au8MAC[6] << 3));
    u8LedBlinkCount = 0;
    sPairingData.u8OpChannel = LORA_PAIR_CHANNEL;

    /* Pas de reset radio ici : c'est vSendPairRequest (au premier tick du
     * timer d'appairage) qui fait la reconfig COMPLETE sur le rendez-vous.
     * On se contente de pointer la frequence sur le canal d'appairage. */
    APP_vLoraChannelSet(LORA_PAIR_CHANNEL);

    vSetBlinkOnTime(LORA_PAIR_LED_FLASH_MS);
    vStartBlinkTimer(LORA_PAIR_LED_PERIOD_MS);

    /* La premiere PAIR_REQUEST part apres un intervalle complet : la reserve
     * se recharge d'abord, que l'on vienne d'un appui long ou d'un
     * demarrage (de meme entre deux tentatives). */
    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: first PAIR_REQUEST in %d ms (recharge)\r\n",
                LORA_PAIR_TX_INTERVAL_MS);
    ZTIMER_eStart(u8TimerLoraPair, ZTIMER_TIME_MSEC(LORA_PAIR_TX_INTERVAL_MS));
}

/****************************************************************************
 *
 * NAME: vEndPairingWindow
 *
 * DESCRIPTION:
 * Fin de fenetre sans appairage. Un emetteur deja appaire reprend son
 * appairage precedent, intact en PDM ; un emetteur sans cle repasse non
 * appaire et attend un appui long.
 *
 ****************************************************************************/
PRIVATE void vEndPairingWindow(void)
{
    ZTIMER_eStop(u8TimerLoraPair);

    if (bHasPrevPairing)
    {
        sPairingData = sPrevPairing;
        bHasPrevPairing = FALSE;
        ePairState = LORA_PAIR_PAIRED;

        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: pairing window expired, previous pairing restored (op_channel=%d SF%d)\r\n",
            sPairingData.u8OpChannel, APP_u8LoraPairGetSF());

        /* Reset + configuration complete sur la config operationnelle : la
         * radio sort peut-etre du sommeil, sur le canal de rendez-vous, et
         * APP_vLoRaTicStart teste le PLL juste apres. */
        APP_vLoRaTicReconfigureRadio(APP_u8LoraPairGetOpChannel(),
                                     APP_u8LoraPairGetSF());
        vSetBlinkOnTime(LORA_LED_FLASH_DEFAULT_MS);
        vStartBlinkTimer(3000);
        APP_vLoRaTicStart();
    }
    else
    {
        ePairState = LORA_PAIR_UNPAIRED;
        vStopBlinkTimer();
        vSetBlinkOnTime(LORA_LED_FLASH_DEFAULT_MS);
        GPIO_PinWrite(GPIO, LORA_GPIO_PORT, LORA_LED_PIN, 0);   /* LED eteinte */

        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: pairing window expired, not paired — long press to retry\r\n");
    }
}

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
 * Reconfigure le SX1280 pour un TX d'appairage sur (canal, SF) donnes.
 *
 * Le handshake (REQUEST + RESPONSE) se fait sur le rendez-vous FIXE
 * LORA_PAIR_CHANNEL / LORA_PAIR_SF, quelle que soit la config operationnelle
 * stockee : un module qui tourne en SF7/canal7 doit pouvoir y revenir pour
 * renegocier. Le CONFIRM, lui, part sur la config operationnelle assignee
 * par le recepteur, ce qui la valide au passage.
 *
 * Fait une reconfiguration COMPLETE (reset materiel inclus), comme le chemin
 * TX TIC, pour ne pas dependre de l'etat laisse par un autre module.
 *
 ****************************************************************************/
PRIVATE void vReconfigureForPairTx(uint8 u8Channel, uint8 u8SF, int8 i8PowerDbm)
{
    /* Reset + config complete sur (canal, SF) : packet type, modulation,
     * sync word, packet params, TX power, puis IRQ TX_DONE sur DIO1. */
    APP_vLoRaTicReconfigureRadio(u8Channel, u8SF);

    /* Puissance reduite pour l'appairage : le reconfig ci-dessus applique
     * la puissance des donnees (+10 dBm), on la remplace ici pour limiter
     * le pic de courant du PA (cf. LORA_PAIR_TX_POWER_DBM). */
    SX1280_SetTxParams(i8PowerDbm, SX1280_RAMP_20_US);

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
 * Build and transmit PAIR_REQUEST packet.
 *
 * Base (11 octets — retro-compatible avec les receivers antérieurs a v2.22) :
 *   [0]   0x13 (v1, pair_request)
 *   [1]   sequence
 *   [2-9] MAC address (8 bytes)
 *   [10]  channel (0 = default)
 *
 * Extension (LiXee-Box >= v2.22, active via LORA_PAIR_REQUEST_HAS_EXT) :
 *   [11-12] device_id (uint16 BE)   — nomme data/tp/<device_id>.json
 *   [13]    model_len (N)
 *   [14..]  model ASCII (N octets)  — cle dans le JSON template
 *
 * Un receiver ancien lira ses 11 octets et ignorera la queue.
 *
 ****************************************************************************/
PRIVATE void vSendPairRequest(void)
{
    uint8 au8Pkt[LORA_PAIR_REQUEST_MAX_SIZE];
    uint8 u8PktLen = LORA_PAIR_REQUEST_BASE_SIZE;

    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_PAIR << 4) | LORA_TIC_PKT_PAIR_REQUEST;
    u8LastReqSeq = u8PairSeq++;
    au8Pkt[1] = u8LastReqSeq;
    memcpy(&au8Pkt[2], sPairingData.au8MAC, LORA_MAC_SIZE);
    au8Pkt[10] = 0; /* channel = default */

#if LORA_PAIR_REQUEST_HAS_EXT
    /* Annonce du type d'objet : device_id (BE) + model_len + model ASCII */
    au8Pkt[11] = (uint8)((LORA_DEVICE_ID_ZLINKY_TIC >> 8) & 0xFF);
    au8Pkt[12] = (uint8)(LORA_DEVICE_ID_ZLINKY_TIC & 0xFF);

    static const char szModel[] = LORA_DEVICE_MODEL_ZLINKY_TIC;
    uint8 u8ModelLen = (uint8)(sizeof(szModel) - 1);  /* sans le nul terminal */
    if (u8ModelLen > LORA_DEVICE_MODEL_MAX_LEN) {
        u8ModelLen = LORA_DEVICE_MODEL_MAX_LEN;
    }
    au8Pkt[13] = u8ModelLen;
    memcpy(&au8Pkt[14], szModel, u8ModelLen);
    u8PktLen = LORA_PAIR_REQUEST_BASE_SIZE + 3 + u8ModelLen;
#endif

    DBG_vPrintf(TRACE_LORA_PAIR,
                "LORA_PAIR: TX PAIR_REQUEST seq=%d len=%d (device_id=0x%04X model=\"%s\")\r\n",
                au8Pkt[1], u8PktLen, LORA_DEVICE_ID_ZLINKY_TIC, LORA_DEVICE_MODEL_ZLINKY_TIC);

    /* Silence SPI debug during TX */
    g_u8SX1280_HalSilent = 1;

    /* Reconfig complete sur le rendez-vous fixe (canal 3 / SF11), connu des
     * deux cotes independamment de la config operationnelle. */
    vReconfigureForPairTx(LORA_PAIR_CHANNEL, LORA_PAIR_SF, LORA_PAIR_TX_POWER_DBM);

    /* Send payload */
    SX1280_SendPayload(au8Pkt, u8PktLen);

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
        /* NE PAS appeler ZTIMER_vTask() ici : vSendPairRequest est appelee
         * depuis le callback timer APP_cbTimerLoraPair, donc ce serait une
         * REENTRANCE du dispatcher ZTIMER. Le ZTIMER_vTask du SDK JN518x
         * reutilise u32Ticks comme delta ET timestamp : en reentrance il lit
         * le delta comme un timestamp -> expire TOUS les timers d'un coup et
         * declenche leurs callbacks en cascade (radio reconfiguree en plein
         * TX, pile timers corrompue) -> blocage. On anime la LED en direct. */
        vPollLedBlink();
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

    /* IRQ : RX_DONE / TIMEOUT / CRC_ERROR leves sur DIO1. PREAMBLE_DETECTED
     * et HEADER_VALID sont seulement memorises dans le registre d'IRQ, pour
     * le diagnostic : ils disent si une reponse a commence a arriver. */
    SX1280_SetDioIrqParams(
        SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR |
        SX1280_IRQ_PREAMBLE_DETECTED | SX1280_IRQ_HEADER_VALID,
        SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR);

    /* Start RX with timeout */
    SX1280_SetRx(LORA_PAIR_RX_TIMEOUT_MS);

    /* Attente de DIO1, mesuree en millisecondes. Elle couvre la fenetre de
     * detection PLUS la duree d'une reponse : avant, une boucle d'iterations
     * d'environ 300 ms pouvait abandonner une PAIR_RESPONSE (~200 ms a SF11)
     * detectee en fin de fenetre, avant sa fin de reception. */
    uint32 u32RxStartMs = OSA_TimeGetMsec();
    uint32 u32ElapsedMs = 0;
    bool_t bDio1 = FALSE;
    while (u32ElapsedMs < (LORA_PAIR_RX_TIMEOUT_MS + LORA_PAIR_RESPONSE_MAX_AIRTIME_MS))
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
        {
            bDio1 = TRUE;
            break;
        }
        /* Pas de ZTIMER_vTask() : reentrance interdite (voir vSendPairRequest). */
        vPollLedBlink();
        u32ElapsedMs = OSA_TimeGetMsec() - u32RxStartMs;
    }

    /* Check IRQ status */
    uint16 u16Irq = SX1280_GetIrqStatus();
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

    if (!bDio1)
    {
        SX1280_SetStandby(SX1280_STANDBY_RC);
        g_u8SX1280_HalSilent = 0;
        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: no PAIR_RESPONSE, DIO1 never raised in %lu ms (IRQ=0x%04X%s)\r\n",
            u32ElapsedMs, u16Irq,
            (u16Irq & SX1280_IRQ_HEADER_VALID) ? ", header seen: response cut" : "");
        return FALSE;
    }

    if (!(u16Irq & SX1280_IRQ_RX_DONE))
    {
        SX1280_SetStandby(SX1280_STANDBY_RC);
        g_u8SX1280_HalSilent = 0;
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: no PAIR_RESPONSE, IRQ=0x%04X (%s)\r\n", u16Irq,
            (u16Irq & SX1280_IRQ_CRC_ERROR) ? "CRC error: response unreadable" :
            (u16Irq & SX1280_IRQ_HEADER_VALID) ? "timeout after header: response cut" :
            (u16Irq & SX1280_IRQ_PREAMBLE_DETECTED) ? "timeout after preamble" :
            "timeout, nothing heard");
        return FALSE;
    }

    /* Origine du calendrier des PAIR_CONFIRM */
    u32PairRespMs = OSA_TimeGetMsec();

    /* Read payload */
    uint8 au8RxBuf[32];
    uint8 u8RxLen = 0;
    int8 i8Rssi = 0, i8Snr = 0;

    SX1280_GetRxPayload(au8RxBuf, sizeof(au8RxBuf), &u8RxLen, &i8Rssi, &i8Snr);
    SX1280_SetStandby(SX1280_STANDBY_RC);
    g_u8SX1280_HalSilent = 0;

    DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: RX len=%d RSSI=%d SNR=%d\r\n",
                u8RxLen, i8Rssi, i8Snr);

    /* Validate PAIR_RESPONSE. Tailles acceptees :
     *   20      = format legacy (receivers < v2.23) → SF11 par defaut
     *   21..28  = format courant (avec op_SF en [20])
     *   >= 29   = format adresse : MAC de l'emetteur vise en [21..28]
     * Une trame plus longue que le buffer est ecartee par SX1280_GetRxPayload
     * (u8RxLen > sizeof). */
    if ((u8RxLen != LORA_PAIR_RESPONSE_SIZE_LEGACY &&
         u8RxLen <  LORA_PAIR_RESPONSE_SIZE) ||
        u8RxLen > sizeof(au8RxBuf))
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Wrong size (got %d)\r\n", u8RxLen);
        return FALSE;
    }

    uint8 u8Type = au8RxBuf[0] & 0x0F;
    if (u8Type != LORA_TIC_PKT_PAIR_RESPONSE)
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: Wrong type (expected 0x04, got 0x%02X)\r\n", u8Type);
        return FALSE;
    }

    /* La reponse doit renvoyer le seq de NOTRE derniere PAIR_REQUEST. Sinon
     * c'est la reponse d'une box a un autre emetteur en appairage. */
    if (au8RxBuf[1] != u8LastReqSeq)
    {
        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: response seq=%d, our request seq=%d → not for us, ignored\r\n",
            au8RxBuf[1], u8LastReqSeq);
        return FALSE;
    }

    /* Format adresse : la reponse doit porter notre MAC */
    if (u8RxLen >= LORA_PAIR_RESPONSE_SIZE_V2 &&
        memcmp(&au8RxBuf[LORA_PAIR_RESPONSE_MAC_OFFSET], sPairingData.au8MAC, LORA_MAC_SIZE) != 0)
    {
        DBG_vPrintf(TRACE_LORA_PAIR, "LORA_PAIR: response addressed to another MAC, ignored\r\n");
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

    /* Extract operational channel (byte 19) */
    uint8 u8OpCh = au8RxBuf[19];
    if (!APP_bLoraChannelIsValid(u8OpCh))
    {
        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: receiver assigned invalid channel %d, ignoring → fallback to PAIR_CHANNEL\r\n",
            u8OpCh);
        u8OpCh = LORA_PAIR_CHANNEL;
    }
    sPairingData.u8OpChannel = u8OpCh;

    /* Extract operational SF (byte 20). Absent en format legacy → SF11.
     * APP_bLoraPairSetSF valide la borne 7..12 ; hors bornes → defaut. */
    uint8 u8OpSF = LORA_SF_DEFAULT;
    if (u8RxLen == LORA_PAIR_RESPONSE_SIZE)
        u8OpSF = au8RxBuf[20];
    if (!APP_bLoraPairSetSF(u8OpSF))
    {
        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: SF assigne invalide (%d) → defaut SF%d\r\n",
            u8OpSF, LORA_SF_DEFAULT);
        APP_bLoraPairSetSF(LORA_SF_DEFAULT);
    }

    /* Puissance d'emission adaptee au lien mesure (cf. LORA_TX_POWER_*) */
    {
        int16 i16Power = (int16)LORA_LINK_TARGET_RSSI_DBM
                       + (int16)LORA_BOX_TX_POWER_ASSUMED_DBM - (int16)i8Rssi;
        if (i16Power < LORA_TX_POWER_MIN_DBM) i16Power = LORA_TX_POWER_MIN_DBM;
        if (i16Power > LORA_TX_POWER_MAX_DBM) i16Power = LORA_TX_POWER_MAX_DBM;
        sPairingData.u8TxPowerCode = (uint8)(i16Power + LORA_TX_POWER_CODE_OFFSET);
    }

    DBG_vPrintf(TRACE_LORA_PAIR,
        "LORA_PAIR: PAIR_RESPONSE received! Key extracted, op_channel=%d op_SF=%d\r\n",
        u8OpCh, APP_u8LoraPairGetSF());
    DBG_vPrintf(TRACE_LORA_PAIR,
        "LORA_PAIR: response RSSI=%d dBm SNR=%d dB -> TX power %d dBm\r\n",
        i8Rssi, i8Snr, APP_i8LoraPairGetTxPowerDbm());

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
PRIVATE bool_t bSendPairConfirm(uint8 u8Seq)
{
    uint8 au8Pkt[LORA_PAIR_CONFIRM_SIZE];

    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_PAIR << 4) | LORA_TIC_PKT_PAIR_CONFIRM;
    au8Pkt[1] = u8Seq;
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

    PRINTF("[CONF] step 1: enter (switching to op_channel=%d SF%d)\r\n",
           sPairingData.u8OpChannel, APP_u8LoraPairGetSF());
    /* Traces SPI coupees : affichees, elles retardaient la PAIR_CONFIRM de
     * plusieurs centaines de ms et faussaient son calendrier. */
    g_u8SX1280_HalSilent = 1;

    /* Reconfig complete sur la config OPERATIONNELLE (canal + SF) assignee par
     * le recepteur : c'est la qu'il ecoute le CONFIRM. Emettre le CONFIRM sur
     * cette config la valide — si elle ne passe pas, l'appairage echoue ici et
     * on retente, plutot que de se retrouver appaire sur une config muette. */
    PRINTF("[CONF] step 2: vReconfigureForPairTx(ch=%d SF%d)\r\n",
           sPairingData.u8OpChannel, APP_u8LoraPairGetSF());
    vReconfigureForPairTx(sPairingData.u8OpChannel, APP_u8LoraPairGetSF(),
                          APP_i8LoraPairGetTxPowerDbm());
    PRINTF("[CONF] step 3: SendPayload(%d bytes)\r\n", LORA_PAIR_CONFIRM_SIZE);
    SX1280_SendPayload(au8Pkt, LORA_PAIR_CONFIRM_SIZE);
    PRINTF("[CONF] step 4: SendPayload returned\r\n");

    /* Poll DIO1 for TX_DONE - shortened with periodic status checks */
    uint32 u32Start;
    bool_t bDio1 = FALSE;
    uint32 u32StatusCheckPeriod = 100000;
    for (u32Start = 0; u32Start < 1000000; u32Start++)  /* reduced loop */
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
        {
            bDio1 = TRUE;
            break;
        }
        /* Pas de ZTIMER_vTask() : reentrance interdite (voir vSendPairRequest). */
        vPollLedBlink();
        /* Periodic status check */
        if ((u32Start % u32StatusCheckPeriod) == 0 && u32Start > 0)
        {
            uint8 u8Status = SX1280_GetStatus();
            uint16 u16Irq = SX1280_GetIrqStatus();
            uint8 dio1_now = GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN);
            PRINTF("[CONF] poll %lu: status=0x%02X irq=0x%04X DIO1=%d\r\n",
                   u32Start, u8Status, u16Irq, dio1_now);
        }
    }
    PRINTF("[CONF] step 5: DIO1 poll done (DIO1=%d, count=%lu)\r\n",
           bDio1, u32Start);

    /* Final status snapshot */
    {
        uint8 u8Status = SX1280_GetStatus();
        uint16 u16Irq = SX1280_GetIrqStatus();
        PRINTF("[CONF] final status=0x%02X irq=0x%04X\r\n", u8Status, u16Irq);
    }

    PRINTF("[CONF] step 6: ClearIrqStatus\r\n");
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
    PRINTF("[CONF] step 7: SetStandby\r\n");
    SX1280_SetStandby(SX1280_STANDBY_RC);
    PRINTF("[CONF] step 8: done, returning\r\n");
    g_u8SX1280_HalSilent = 0;
    return bDio1;
}

/****************************************************************************
 *
 * NAME: vPairWaitUntil
 *
 * DESCRIPTION:
 * Attend l'instant u32TargetMs (horloge OSA, ms), CPU en veille entre deux
 * interruptions pour laisser la reserve se recharger. Appelee depuis le
 * callback du timer d'appairage : pas de ZTIMER_vTask() (reentrance), mais
 * le watchdog est nourri, l'attente pouvant durer ~3 s.
 *
 ****************************************************************************/
PRIVATE void vPairWaitUntil(uint32 u32TargetMs)
{
    while ((int32)(u32TargetMs - OSA_TimeGetMsec()) > 0)
    {
#ifdef WATCHDOG_ALLOWED
        WWDT_Refresh(WWDT);
        wdt_update_count = 0;
#endif
        vPollLedBlink();
        __WFI();
    }
}

/****************************************************************************
 *
 * NAME: vSendPairConfirms
 *
 * DESCRIPTION:
 * Emet LORA_PAIR_CONFIRM_COUNT fois la meme PAIR_CONFIRM, aux instants
 * LORA_PAIR_CONFIRM_FIRST_MS / SECOND_MS comptes depuis la reception de la
 * PAIR_RESPONSE, la radio restant en standby entre les emissions pour
 * laisser la reserve se recharger.
 *
 ****************************************************************************/
PRIVATE void vSendPairConfirms(void)
{
    static const uint16 au16AtMs[LORA_PAIR_CONFIRM_COUNT] = {
        LORA_PAIR_CONFIRM_FIRST_MS,
        LORA_PAIR_CONFIRM_SECOND_MS
    };
    uint8 u8Seq = u8PairSeq++;   /* meme seq pour toutes les emissions */
    uint8 i;

    for (i = 0; i < LORA_PAIR_CONFIRM_COUNT; i++)
    {
        /* Radio en sommeil pendant la recharge : chaque PAIR_CONFIRM refait
         * un reset et une configuration complete. */
        SX1280_SetSleep();
        vPairWaitUntil(u32PairRespMs + au16AtMs[i]);
        uint32 u32StartMs = OSA_TimeGetMsec() - u32PairRespMs;
        bool_t bTxDone = bSendPairConfirm(u8Seq);
        uint32 u32EndMs = OSA_TimeGetMsec() - u32PairRespMs;

        /* Delais comptes depuis la reception de la PAIR_RESPONSE. Calendrier
         * fixe : debut prevu a LORA_PAIR_CONFIRM_FIRST_MS / SECOND_MS. */
        DBG_vPrintf(TRACE_LORA_PAIR,
            "LORA_PAIR: PAIR_CONFIRM %d/%d start t+%lu ms, end t+%lu ms (planned t+%d) %s\r\n",
            i + 1, LORA_PAIR_CONFIRM_COUNT, u32StartMs, u32EndMs, au16AtMs[i],
            bTxDone ? "TX_DONE" : "NO TX_DONE");
    }
}

/* LED management:
 *   PAIRING: vStartBlinkTimer(150) — very fast blink
 *   PAIRED:  managed by App_Linky.c — 1s (TIC error) / 3s (TIC OK)
 *   UNPAIRED: LED off
 */

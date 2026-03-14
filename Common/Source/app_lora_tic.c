/*****************************************************************************
 *
 * MODULE:             app_lora_tic
 *
 * COMPONENT:          app_lora_tic.c
 *
 * DESCRIPTION:        LoRa TIC protocol — periodic TX of Linky meter data
 *                     via SX1280 2.4GHz LoRa (NiceRF LORA1280-TCXO)
 *
 *                     Uniquement des etiquettes TIC brutes du Linky.
 *                     Pas de valeurs calculees (pas de puissance active, etc.)
 *
 *                     ESSENTIAL (17B) envoye chaque seconde :
 *                       SINSTS/1/2/3 + STGE + ADPS + mode
 *
 *                     EXTENDED (15-19B) en piggyback toutes les 10s :
 *                       6 sous-types en round-robin, cycle complet ~60s
 *
 *                     Max 20 octets par paquet (contrainte SF10).
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

#include "app_lora_tic.h"
#include "app_main.h"
#include "app_events.h"
#include "app_common.h"
#include "zcl_options.h"
#include "app_zcl_task.h"

#include "sx1280.h"
#include "app_lora_pair.h"
#include "sx1280_hal.h"
#include "sx1280_config.h"
#include "App_Linky.h"
#include "app_lora_pair.h"
#include "app_lora_crypt.h"

/* Flag HAL SPI : coupe les PRINTF pendant TX RF (defini dans sx1280_hal.c) */
extern volatile uint8_t g_u8SX1280_HalSilent;

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_LORA_TIC
    #define TRACE_LORA_TIC  TRUE
#else
    #define TRACE_LORA_TIC  TRUE
#endif

/* DIO1 pin pour poll TX_DONE */
#define LORA_DIO1_PIN       SX1280_DIO1_PIN
#define LORA_GPIO_PORT      SX1280_GPIO_PORT

/* LED sur PIO10 */
#define LORA_LED_PIN        10

/* Timeout TX en ms (SF10/BW406/20B ~100-150ms airtime, on met 500ms)
 * Le SX1280 a son propre timeout a 10s (SetTx 0x2710@1ms) */
#define LORA_TX_TIMEOUT_MS      500

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/* Compteur de sequence (0-255, wrap around) */
PRIVATE uint8 u8TxSequence = 0;

/* Compteur de paquets envoyes (pour stats debug) */
PRIVATE uint32 u32TxCount = 0;

/* Compteur d'echecs TX consecutifs */
PRIVATE uint32 u32ConsecutiveFailures = 0;

/* Flag SX1280 operationnel (positionne par app_start.c apres test SPI) */
PRIVATE bool_t bSX1280Ready = FALSE;

/* Flag TX pending — positionne par le callback ZTIMER, consomme dans la boucle main */
PRIVATE volatile bool_t bLoRaTxPending = FALSE;

/* Buffer TX (dimensionne pour le plus grand paquet + 2B MIC) */
PRIVATE uint8 au8TxPayload[LORA_TIC_MAX_PAYLOAD + LORA_CRYPT_MIC_SIZE];

/* Round-robin extended : compteur + sous-type courant */
PRIVATE uint8 u8EssentialCounter = 0;
PRIVATE uint8 u8ExtSubType = 0;

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/* sBaseDevice declare dans app_zcl_task.h (inclus ci-dessus) */

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void vBuildEssentialPacket(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtendedPacket(uint8 *pu8Buffer, uint8 u8SubType);
PRIVATE uint8 u8BuildExtCurrentVoltage(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtEnergy(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtEnergy2(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtVoltageStats(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtPowerMaxCfg(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtDailyEnergy(uint8 *pu8Buffer);
PRIVATE uint8 u8BuildExtComplement(uint8 *pu8Buffer);
PRIVATE void vReconfigureLoRaForTx(void);
PRIVATE bool_t bTransmitAndWait(uint8 *pu8Data, uint8 u8Len);
PRIVATE uint8 u8HexCharToNibble(uint8 u8Char);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_cbTimerLoRa
 *
 * DESCRIPTION:
 * Callback ZTIMER periodique — poste APP_E_EVENT_LORA_TX dans la queue
 * et relance le timer pour le prochain cycle.
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerLoRa(void *pvParam)
{
    /* Positionne un flag — le travail bloquant (TIC + TX) sera fait
     * dans la boucle main APRES ZTIMER_vTask(), pour ne pas bloquer
     * les autres timers (blink LED, button scan, etc.) */
    bLoRaTxPending = TRUE;
}

/****************************************************************************
 *
 * NAME: APP_vLoRaTicSetReady
 *
 * DESCRIPTION:
 * Positionne le flag de disponibilite SX1280.
 * Appele par app_start.c apres le test SPI au boot.
 *
 ****************************************************************************/
PUBLIC void APP_vLoRaTicSetReady(bool_t bReady)
{
    bSX1280Ready = bReady;
    u32ConsecutiveFailures = 0;
    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: SX1280 ready = %d\r\n", bReady);
}

/****************************************************************************
 *
 * NAME: APP_vLoRaTicPollTx
 *
 * DESCRIPTION:
 * Verifie si un TX est en attente (flag pose par APP_cbTimerLoRa).
 * Doit etre appele depuis la boucle main, APRES ZTIMER_vTask(),
 * pour que le travail bloquant (TIC acquisition + TX LoRa)
 * ne gele pas les autres timers (LED blink, button scan...).
 *
 ****************************************************************************/
PUBLIC void APP_vLoRaTicPollTx(void)
{
    if (bLoRaTxPending)
    {
        bLoRaTxPending = FALSE;
        if (APP_bLoraPairIsPaired())
        {
            APP_vLoRaTicSendPacket();
        }
    }
}

/****************************************************************************
 *
 * NAME: APP_vLoRaTicSendPacket
 *
 * DESCRIPTION:
 * Cycle de transmission LoRa TIC :
 * 1) Toujours envoyer un ESSENTIAL (17B) — SINSTS + STGE/ADPS
 * 2) Toutes les LORA_TIC_EXTENDED_INTERVAL TX, envoyer aussi un
 *    EXTENDED (15-19B) en piggyback juste apres.
 *
 * Ainsi SINSTS est disponible chaque seconde (jamais rate),
 * et toutes les autres etiquettes TIC sont remontees en ~60s.
 *
 ****************************************************************************/
PUBLIC void APP_vLoRaTicSendPacket(void)
{
    /* Skip TX si SX1280 non operationnel */
    if (!bSX1280Ready)
    {
        u32ConsecutiveFailures++;
        if (u32ConsecutiveFailures % 30 == 1)
        {
            DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: SX1280 not ready, TX skipped (%lu times)\r\n",
                        u32ConsecutiveFailures);
        }
        return;
    }

    /* Skip TX si non appaire */
    if (!APP_bLoraPairIsPaired())
    {
        return;
    }

    /* Stopper le timer LoRa pendant l'acquisition TIC + TX
     * pour eviter l'accumulation d'events dans la queue
     * (vAPP_LinkySensorSample est bloquante ~1-2s) */
    ZTIMER_eStop(u8TimerLoRa);

    /* Acquisition TIC — met a jour sBaseDevice avec les dernieres
     * donnees Linky disponibles sur l'UART */
    vAPP_LinkySensorSample();

    /* LED geree par app_blink_led / App_Linky (1s erreur, 3s OK) — pas de toggle TX */

    /* ===== 1) Toujours envoyer ESSENTIAL (17B) ===== */
    vBuildEssentialPacket(au8TxPayload);

    /* Encrypt if paired */
    uint8 u8EssTxLen = LORA_TIC_DATA_ESSENTIAL_SIZE;
    u8EssTxLen = APP_u8LoraCryptEncrypt(au8TxPayload, u8EssTxLen,
                                         APP_pu8LoraPairGetKey(),
                                         APP_pu8LoraPairGetMAC(),
                                         au8TxPayload[1]);

    /* Hex dump */
    {
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: [ESS%s] TX [%d]: ",
                    (u8EssTxLen > LORA_TIC_DATA_ESSENTIAL_SIZE) ? "+ENC" : "",
                    u8EssTxLen);
        int i;
        for (i = 0; i < u8EssTxLen; i++)
            DBG_vPrintf(TRACE_LORA_TIC, "%02X ", au8TxPayload[i]);
        DBG_vPrintf(TRACE_LORA_TIC, "\r\n");
    }

    bool_t bOk = bTransmitAndWait(au8TxPayload, u8EssTxLen);

    if (bOk)
    {
        u32TxCount++;
        u32ConsecutiveFailures = 0;
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: ESS TX #%lu OK (seq=%u)\r\n",
                    u32TxCount, u8TxSequence - 1);
    }
    else
    {
        u32ConsecutiveFailures++;
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: ESS TX FAILED (seq=%u, fails=%lu)\r\n",
                    u8TxSequence - 1, u32ConsecutiveFailures);
    }

    /* ===== 2) Toutes les N TX, piggyback EXTENDED ===== */
    u8EssentialCounter++;
    if (u8EssentialCounter >= LORA_TIC_EXTENDED_INTERVAL)
    {
        u8EssentialCounter = 0;

        uint8 u8ExtLen = u8BuildExtendedPacket(au8TxPayload, u8ExtSubType);

        /* Encrypt if paired */
        u8ExtLen = APP_u8LoraCryptEncrypt(au8TxPayload, u8ExtLen,
                                           APP_pu8LoraPairGetKey(),
                                           APP_pu8LoraPairGetMAC(),
                                           au8TxPayload[1]);

        /* Hex dump */
        {
            DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: [EXT sub=%d%s] TX [%d]: ",
                        u8ExtSubType,
                        (u8ExtLen > LORA_TIC_DATA_EXTENDED_MAX_SIZE) ? "+ENC" : "",
                        u8ExtLen);
            int i;
            for (i = 0; i < u8ExtLen; i++)
                DBG_vPrintf(TRACE_LORA_TIC, "%02X ", au8TxPayload[i]);
            DBG_vPrintf(TRACE_LORA_TIC, "\r\n");
        }

        bool_t bExtOk = bTransmitAndWait(au8TxPayload, u8ExtLen);

        if (bExtOk)
        {
            u32TxCount++;
            DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: EXT TX #%lu OK (sub=%d seq=%u)\r\n",
                        u32TxCount, u8ExtSubType, u8TxSequence - 1);
        }
        else
        {
            DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: EXT TX FAILED (sub=%d seq=%u)\r\n",
                        u8ExtSubType, u8TxSequence - 1);
        }

        u8ExtSubType = (u8ExtSubType + 1) % LORA_TIC_EXT_SUBTYPE_COUNT;
    }

    /* LED geree par app_blink_led — pas de toggle TX */

    /* Relancer le timer LoRa pour le prochain cycle
     * (intervalle = temps entre la FIN d'un TX et le DEBUT du suivant) */
    ZTIMER_eStart(u8TimerLoRa, ZTIMER_TIME_MSEC(LORA_TX_INTERVAL_MS));
}

/****************************************************************************
 *
 * NAME: APP_vLoRaTicStart
 *
 * DESCRIPTION:
 * Demarre le timer periodique LoRa TX.
 * Appele depuis APP_vInitialiseRouter apres init TIC.
 *
 ****************************************************************************/
PUBLIC void APP_vLoRaTicStart(void)
{
    if (!bSX1280Ready)
    {
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: SX1280 not ready, LoRa TX disabled\r\n");
        return;
    }

    /* Test PLL lock (SetFs) avant de demarrer les TX periodiques.
     * Si le PLL ne peut pas lock, le TX ne fonctionnera pas. */
    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: Testing PLL lock (SetFs)...\r\n");
    SX1280_SetFs();

    /* Attendre ~5ms que le PLL lock */
    for (volatile uint32 j = 0; j < 60000; j++); /* ~5ms a 12MHz */

    uint8 u8FsStatus = SX1280_GetStatus();
    uint8 u8FsMode = (u8FsStatus >> 5) & 0x07;
    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: SetFs Status=0x%02X mode=%d (%s)\r\n",
                u8FsStatus, u8FsMode,
                u8FsMode == 4 ? "FS OK" : u8FsMode == 2 ? "STDBY_RC (PLL FAIL!)" : "?");

    /* Revenir en standby */
    SX1280_SetStandby(SX1280_STANDBY_RC);

    if (u8FsMode != 4)
    {
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: WARNING - PLL lock failed, TX may not work!\r\n");
    }

    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: Starting periodic TX (interval=%dms, ext every %d)\r\n",
                LORA_TX_INTERVAL_MS, LORA_TIC_EXTENDED_INTERVAL);

    ZTIMER_eStart(u8TimerLoRa, ZTIMER_TIME_MSEC(LORA_TX_INTERVAL_MS));
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: u8HexCharToNibble
 *
 * DESCRIPTION:
 * Convertit un caractere hexadecimal ASCII en valeur 4 bits.
 * Utilise pour decoder STGE (8 chars hex -> 4 octets binaire).
 *
 ****************************************************************************/
PRIVATE uint8 u8HexCharToNibble(uint8 u8Char)
{
    if (u8Char >= '0' && u8Char <= '9') return u8Char - '0';
    if (u8Char >= 'A' && u8Char <= 'F') return u8Char - 'A' + 10;
    if (u8Char >= 'a' && u8Char <= 'f') return u8Char - 'a' + 10;
    return 0;
}

/****************************************************************************
 *
 * NAME: vBuildEssentialPacket
 *
 * DESCRIPTION:
 * Construit un paquet DATA_ESSENTIAL de 17 octets.
 * Uniquement des etiquettes TIC brutes :
 *
 *   [0]     0x11 (version 1, type essential)
 *   [1]     sequence
 *   [2-3]   SINSTS total (VA, uint16 BE) — = PAPP en Historique
 *   [4-5]   SINSTS1 (VA, uint16 BE) — = SINSTS/PAPP en mono
 *   [6-7]   SINSTS2 (VA, uint16 BE) — 0 en mono
 *   [8-9]   SINSTS3 (VA, uint16 BE) — 0 en mono
 *   [10-13] STGE (32 bits binaire, converti depuis 8 chars hex)
 *   [14-15] ADPS (uint16 BE) — 0 en mode Standard
 *   [16]    mode (au8LinkyMode: 0=Histo, 1=Std, 2=Tri)
 *
 ****************************************************************************/
PRIVATE void vBuildEssentialPacket(uint8 *pu8Buffer)
{
    /* Header */
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_ESSENTIAL;
    pu8Buffer[1] = u8TxSequence++;

    /* SINSTS total (u32 -> uint16, cap 65535)
     * En mono : = SINSTS = PAPP
     * En tri  : = SINSTS (somme des 3 phases) */
    uint32 u32Sinsts = sBaseDevice.sElectricalMeasurement.u32TotalApparentPower;
    uint16 u16Sinsts = (u32Sinsts > 65535) ? 65535 : (uint16)u32Sinsts;
    pu8Buffer[2] = (uint8)(u16Sinsts >> 8);
    pu8Buffer[3] = (uint8)(u16Sinsts & 0xFF);

    /* SINSTS1 — phase 1 (= SINSTS/PAPP en mono) */
    uint16 u16Val = sBaseDevice.sElectricalMeasurement.u16ApparentPower;
    pu8Buffer[4] = (uint8)(u16Val >> 8);
    pu8Buffer[5] = (uint8)(u16Val & 0xFF);

    /* SINSTS2 — phase 2 (0 en mono) */
    u16Val = sBaseDevice.sElectricalMeasurement.u16ApparentPowerPhB;
    pu8Buffer[6] = (uint8)(u16Val >> 8);
    pu8Buffer[7] = (uint8)(u16Val & 0xFF);

    /* SINSTS3 — phase 3 (0 en mono) */
    u16Val = sBaseDevice.sElectricalMeasurement.u16ApparentPowerPhC;
    pu8Buffer[8] = (uint8)(u16Val >> 8);
    pu8Buffer[9] = (uint8)(u16Val & 0xFF);

    /* STGE — 8 chars hex ASCII → 4 octets binaire
     * "003A1B2C" → 0x00, 0x3A, 0x1B, 0x2C
     * En Historique, au8LinkySTGE peut etre vide (= 0) */
    {
        uint8 *pSTGE = sBaseDevice.sLinkyServerCluster.au8LinkySTGE;
        int i;
        for (i = 0; i < 4; i++)
        {
            pu8Buffer[10 + i] = (u8HexCharToNibble(pSTGE[i * 2]) << 4) |
                                 u8HexCharToNibble(pSTGE[i * 2 + 1]);
        }
    }

    /* ADPS — alerte depassement (mode Historique, 0 en Standard) */
    uint16 u16Adps = sBaseDevice.sLinkyServerCluster.au16LinkyADPS;
    pu8Buffer[14] = (uint8)(u16Adps >> 8);
    pu8Buffer[15] = (uint8)(u16Adps & 0xFF);

    /* Mode Linky */
    pu8Buffer[16] = sBaseDevice.sLinkyServerCluster.au8LinkyMode;
}

/****************************************************************************
 *
 * NAME: u8BuildExtendedPacket
 *
 * DESCRIPTION:
 * Dispatch vers le builder du sous-type extended demande.
 *
 * RETURNS:
 * Taille totale du paquet en octets.
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtendedPacket(uint8 *pu8Buffer, uint8 u8SubType)
{
    switch (u8SubType)
    {
        case LORA_TIC_EXT_CURRENT_VOLTAGE: return u8BuildExtCurrentVoltage(pu8Buffer);
        case LORA_TIC_EXT_ENERGY:          return u8BuildExtEnergy(pu8Buffer);
        case LORA_TIC_EXT_ENERGY_2:        return u8BuildExtEnergy2(pu8Buffer);
        case LORA_TIC_EXT_VOLTAGE_STATS:   return u8BuildExtVoltageStats(pu8Buffer);
        case LORA_TIC_EXT_POWER_MAX_CFG:   return u8BuildExtPowerMaxCfg(pu8Buffer);
        case LORA_TIC_EXT_DAILY_ENERGY:    return u8BuildExtDailyEnergy(pu8Buffer);
        case LORA_TIC_EXT_COMPLEMENT:      return u8BuildExtComplement(pu8Buffer);
        default:                           return u8BuildExtCurrentVoltage(pu8Buffer);
    }
}

/****************************************************************************
 *
 * NAME: u8BuildExtCurrentVoltage
 *
 * DESCRIPTION:
 * Sub-type 0x00 CURRENT_VOLTAGE (15 octets) :
 * Courants et tensions efficaces par phase
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x00 (sub_type)
 *   [3-4]   IRMS1 (mA, uint16)
 *   [5-6]   IRMS2 (mA, uint16)
 *   [7-8]   IRMS3 (mA, uint16)
 *   [9-10]  URMS1 (V, uint16)
 *   [11-12] URMS2 (V, uint16)
 *   [13-14] URMS3 (V, uint16)
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtCurrentVoltage(uint8 *pu8Buffer)
{
    /* Header extended (3 octets) */
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_CURRENT_VOLTAGE;

    /* IRMS1 */
    uint16 u16Val = sBaseDevice.sElectricalMeasurement.u16RMSCurrent;
    pu8Buffer[3] = (uint8)(u16Val >> 8);
    pu8Buffer[4] = (uint8)(u16Val & 0xFF);

    /* IRMS2 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSCurrentPhB;
    pu8Buffer[5] = (uint8)(u16Val >> 8);
    pu8Buffer[6] = (uint8)(u16Val & 0xFF);

    /* IRMS3 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSCurrentPhC;
    pu8Buffer[7] = (uint8)(u16Val >> 8);
    pu8Buffer[8] = (uint8)(u16Val & 0xFF);

    /* URMS1 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSVoltage;
    pu8Buffer[9]  = (uint8)(u16Val >> 8);
    pu8Buffer[10] = (uint8)(u16Val & 0xFF);

    /* URMS2 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSVoltagePhB;
    pu8Buffer[11] = (uint8)(u16Val >> 8);
    pu8Buffer[12] = (uint8)(u16Val & 0xFF);

    /* URMS3 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSVoltagePhC;
    pu8Buffer[13] = (uint8)(u16Val >> 8);
    pu8Buffer[14] = (uint8)(u16Val & 0xFF);

    return 15;
}

/****************************************************************************
 *
 * NAME: u8BuildExtEnergy
 *
 * DESCRIPTION:
 * Sub-type 0x01 ENERGY (19 octets) :
 * Compteurs d'energie principaux
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x01 (sub_type)
 *   [3-6]   EAST Wh (uint32 BE) — energie active totale soutiree
 *   [7-10]  EAIT Wh (uint32 BE) — energie active totale injectee
 *   [11-14] EASF01 Wh (uint32 BE) — index tarif 1
 *   [15-18] EASF02 Wh (uint32 BE) — index tarif 2
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtEnergy(uint8 *pu8Buffer)
{
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_ENERGY;

    /* EAST */
    uint32 u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentSummationDelivered;
    pu8Buffer[3]  = (uint8)(u32Val >> 24);
    pu8Buffer[4]  = (uint8)(u32Val >> 16);
    pu8Buffer[5]  = (uint8)(u32Val >> 8);
    pu8Buffer[6]  = (uint8)(u32Val & 0xFF);

    /* EAIT */
    u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentSummationReceived;
    pu8Buffer[7]  = (uint8)(u32Val >> 24);
    pu8Buffer[8]  = (uint8)(u32Val >> 16);
    pu8Buffer[9]  = (uint8)(u32Val >> 8);
    pu8Buffer[10] = (uint8)(u32Val & 0xFF);

    /* EASF01 */
    u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier1SummationDelivered;
    pu8Buffer[11] = (uint8)(u32Val >> 24);
    pu8Buffer[12] = (uint8)(u32Val >> 16);
    pu8Buffer[13] = (uint8)(u32Val >> 8);
    pu8Buffer[14] = (uint8)(u32Val & 0xFF);

    /* EASF02 */
    u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier2SummationDelivered;
    pu8Buffer[15] = (uint8)(u32Val >> 24);
    pu8Buffer[16] = (uint8)(u32Val >> 16);
    pu8Buffer[17] = (uint8)(u32Val >> 8);
    pu8Buffer[18] = (uint8)(u32Val & 0xFF);

    return 19;
}

/****************************************************************************
 *
 * NAME: u8BuildExtEnergy2
 *
 * DESCRIPTION:
 * Sub-type 0x02 ENERGY_2 (19 octets) :
 * Suite des index tarifaires
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x02 (sub_type)
 *   [3-6]   EASF03 Wh (uint32 BE) — index tarif 3
 *   [7-10]  EASF04 Wh (uint32 BE) — index tarif 4
 *   [11-14] EASF05 Wh (uint32 BE) — index tarif 5
 *   [15-18] EASF06 Wh (uint32 BE) — index tarif 6
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtEnergy2(uint8 *pu8Buffer)
{
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_ENERGY_2;

    /* EASF03 */
    uint32 u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier3SummationDelivered;
    pu8Buffer[3]  = (uint8)(u32Val >> 24);
    pu8Buffer[4]  = (uint8)(u32Val >> 16);
    pu8Buffer[5]  = (uint8)(u32Val >> 8);
    pu8Buffer[6]  = (uint8)(u32Val & 0xFF);

    /* EASF04 */
    u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier4SummationDelivered;
    pu8Buffer[7]  = (uint8)(u32Val >> 24);
    pu8Buffer[8]  = (uint8)(u32Val >> 16);
    pu8Buffer[9]  = (uint8)(u32Val >> 8);
    pu8Buffer[10] = (uint8)(u32Val & 0xFF);

    /* EASF05 */
    u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier5SummationDelivered;
    pu8Buffer[11] = (uint8)(u32Val >> 24);
    pu8Buffer[12] = (uint8)(u32Val >> 16);
    pu8Buffer[13] = (uint8)(u32Val >> 8);
    pu8Buffer[14] = (uint8)(u32Val & 0xFF);

    /* EASF06 */
    u32Val = (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier6SummationDelivered;
    pu8Buffer[15] = (uint8)(u32Val >> 24);
    pu8Buffer[16] = (uint8)(u32Val >> 16);
    pu8Buffer[17] = (uint8)(u32Val >> 8);
    pu8Buffer[18] = (uint8)(u32Val & 0xFF);

    return 19;
}

/****************************************************************************
 *
 * NAME: u8BuildExtVoltageStats
 *
 * DESCRIPTION:
 * Sub-type 0x03 VOLTAGE_STATS (15 octets) :
 * Tensions moyennes et courants max par phase
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x03 (sub_type)
 *   [3-4]   UMOY1 (V, uint16)
 *   [5-6]   UMOY2 (V, uint16)
 *   [7-8]   UMOY3 (V, uint16)
 *   [9-10]  IMAX1 (mA, uint16)
 *   [11-12] IMAX2 (mA, uint16)
 *   [13-14] IMAX3 (mA, uint16)
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtVoltageStats(uint8 *pu8Buffer)
{
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_VOLTAGE_STATS;

    /* UMOY1 */
    uint16 u16Val = sBaseDevice.sElectricalMeasurement.u16AverageRMSVoltageMeasurementPeriod;
    pu8Buffer[3] = (uint8)(u16Val >> 8);
    pu8Buffer[4] = (uint8)(u16Val & 0xFF);

    /* UMOY2 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16AverageRMSVoltageMeasurementPeriodPhB;
    pu8Buffer[5] = (uint8)(u16Val >> 8);
    pu8Buffer[6] = (uint8)(u16Val & 0xFF);

    /* UMOY3 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16AverageRMSVoltageMeasurementPeriodPhC;
    pu8Buffer[7] = (uint8)(u16Val >> 8);
    pu8Buffer[8] = (uint8)(u16Val & 0xFF);

    /* IMAX1 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSCurrentMax;
    pu8Buffer[9]  = (uint8)(u16Val >> 8);
    pu8Buffer[10] = (uint8)(u16Val & 0xFF);

    /* IMAX2 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSCurrentMaxPhB;
    pu8Buffer[11] = (uint8)(u16Val >> 8);
    pu8Buffer[12] = (uint8)(u16Val & 0xFF);

    /* IMAX3 */
    u16Val = sBaseDevice.sElectricalMeasurement.u16RMSCurrentMaxPhC;
    pu8Buffer[13] = (uint8)(u16Val >> 8);
    pu8Buffer[14] = (uint8)(u16Val & 0xFF);

    return 15;
}

/****************************************************************************
 *
 * NAME: u8BuildExtPowerMaxCfg
 *
 * DESCRIPTION:
 * Sub-type 0x04 POWER_MAX_CFG (18 octets) :
 * Puissances max, config compteur
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x04 (sub_type)
 *   [3-4]   SMAXSN (VA, uint16) — puissance max soutiree jour
 *   [5-6]   SMAXIN (VA, uint16) — puissance max injectee jour
 *   [7-8]   SINSTI (VA, uint16) — puissance apparente instantanee
 *   [9-10]  CCAIN (VA, uint16) — point courbe charge
 *   [11-12] RELAIS (uint16)
 *   [13]    NTARF (uint8) — numero tarif en cours
 *   [14]    PCOUP (kVA, uint8) — seuil de coupure
 *   [15]    PREF (kVA, uint8) — puissance de reference
 *   [16]    mode (uint8) — au8LinkyMode
 *   [17]    reserved (0x00)
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtPowerMaxCfg(uint8 *pu8Buffer)
{
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_POWER_MAX_CFG;

    /* SMAXSN — puissance max soutiree du jour */
    uint16 u16Val = sBaseDevice.sLinkyServerCluster.u16LinkySMAXSN_1;
    pu8Buffer[3] = (uint8)(u16Val >> 8);
    pu8Buffer[4] = (uint8)(u16Val & 0xFF);

    /* SMAXIN — puissance max injectee du jour */
    u16Val = sBaseDevice.sLinkyServerCluster.u16LinkySMAXIN;
    pu8Buffer[5] = (uint8)(u16Val >> 8);
    pu8Buffer[6] = (uint8)(u16Val & 0xFF);

    /* SINSTI — puissance apparente instantanee */
    u16Val = sBaseDevice.sLinkyServerCluster.u16LinkySINSTI;
    pu8Buffer[7] = (uint8)(u16Val >> 8);
    pu8Buffer[8] = (uint8)(u16Val & 0xFF);

    /* CCAIN — point courbe de charge */
    u16Val = sBaseDevice.sLinkyServerCluster.u16LinkyCCAIN;
    pu8Buffer[9]  = (uint8)(u16Val >> 8);
    pu8Buffer[10] = (uint8)(u16Val & 0xFF);

    /* RELAIS */
    u16Val = sBaseDevice.sLinkyServerCluster.u16LinkyRELAIS;
    pu8Buffer[11] = (uint8)(u16Val >> 8);
    pu8Buffer[12] = (uint8)(u16Val & 0xFF);

    /* NTARF */
    pu8Buffer[13] = sBaseDevice.sLinkyServerCluster.au8LinkyNTARF;

    /* PCOUP */
    pu8Buffer[14] = (uint8)sBaseDevice.sMeterIdentification.au8LinkyMIPowerThreshold;

    /* PREF */
    pu8Buffer[15] = (uint8)sBaseDevice.sMeterIdentification.au8LinkyMIAvailablePower;

    /* Mode */
    pu8Buffer[16] = sBaseDevice.sLinkyServerCluster.au8LinkyMode;

    /* Reserved */
    pu8Buffer[17] = 0x00;

    return 18;
}

/****************************************************************************
 *
 * NAME: u8BuildExtDailyEnergy
 *
 * DESCRIPTION:
 * Sub-type 0x05 DAILY_ENERGY (19 octets) :
 * Energies journalieres J a J-3
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x05 (sub_type)
 *   [3-6]   EASD01 Wh (uint32 BE) — energie journaliere J
 *   [7-10]  EASD02 Wh (uint32 BE) — energie journaliere J-1
 *   [11-14] EASD03 Wh (uint32 BE) — energie journaliere J-2
 *   [15-18] EASD04 Wh (uint32 BE) — energie journaliere J-3
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtDailyEnergy(uint8 *pu8Buffer)
{
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_DAILY_ENERGY;

    /* EASD01 */
    uint32 u32Val = sBaseDevice.sLinkyServerCluster.u32LinkyEASD01;
    pu8Buffer[3]  = (uint8)(u32Val >> 24);
    pu8Buffer[4]  = (uint8)(u32Val >> 16);
    pu8Buffer[5]  = (uint8)(u32Val >> 8);
    pu8Buffer[6]  = (uint8)(u32Val & 0xFF);

    /* EASD02 */
    u32Val = sBaseDevice.sLinkyServerCluster.u32LinkyEASD02;
    pu8Buffer[7]  = (uint8)(u32Val >> 24);
    pu8Buffer[8]  = (uint8)(u32Val >> 16);
    pu8Buffer[9]  = (uint8)(u32Val >> 8);
    pu8Buffer[10] = (uint8)(u32Val & 0xFF);

    /* EASD03 */
    u32Val = sBaseDevice.sLinkyServerCluster.u32LinkyEASD03;
    pu8Buffer[11] = (uint8)(u32Val >> 24);
    pu8Buffer[12] = (uint8)(u32Val >> 16);
    pu8Buffer[13] = (uint8)(u32Val >> 8);
    pu8Buffer[14] = (uint8)(u32Val & 0xFF);

    /* EASD04 */
    u32Val = sBaseDevice.sLinkyServerCluster.u32LinkyEASD04;
    pu8Buffer[15] = (uint8)(u32Val >> 24);
    pu8Buffer[16] = (uint8)(u32Val >> 16);
    pu8Buffer[17] = (uint8)(u32Val >> 8);
    pu8Buffer[18] = (uint8)(u32Val & 0xFF);

    return 19;
}

/****************************************************************************
 *
 * NAME: u8BuildExtComplement
 *
 * DESCRIPTION:
 * Sub-type 0x06 COMPLEMENT (9 octets) :
 * Debut/fin pointe mobile
 *
 *   [0]     0x12 (v1, extended)
 *   [1]     sequence
 *   [2]     0x06 (sub_type)
 *   [3]     DPM1 (uint8) — debut pointe mobile 1
 *   [4]     FPM1 (uint8) — fin pointe mobile 1
 *   [5]     DPM2 (uint8) — debut pointe mobile 2
 *   [6]     FPM2 (uint8) — fin pointe mobile 2
 *   [7]     DPM3 (uint8) — debut pointe mobile 3
 *   [8]     FPM3 (uint8) — fin pointe mobile 3
 *
 ****************************************************************************/
PRIVATE uint8 u8BuildExtComplement(uint8 *pu8Buffer)
{
    pu8Buffer[0] = (LORA_TIC_PROTOCOL_VERSION << 4) | LORA_TIC_PKT_DATA_EXTENDED;
    pu8Buffer[1] = u8TxSequence++;
    pu8Buffer[2] = LORA_TIC_EXT_COMPLEMENT;

    /* DPM/FPM — debut/fin pointe mobile */
    pu8Buffer[3] = sBaseDevice.sLinkyServerCluster.u8LinkyDPM1;
    pu8Buffer[4] = sBaseDevice.sLinkyServerCluster.u8LinkyFPM1;
    pu8Buffer[5] = sBaseDevice.sLinkyServerCluster.u8LinkyDPM2;
    pu8Buffer[6] = sBaseDevice.sLinkyServerCluster.u8LinkyFPM2;
    pu8Buffer[7] = sBaseDevice.sLinkyServerCluster.u8LinkyDPM3;
    pu8Buffer[8] = sBaseDevice.sLinkyServerCluster.u8LinkyFPM3;

    return 9;
}

/****************************************************************************
 *
 * NAME: vReconfigureLoRaForTx
 *
 * DESCRIPTION:
 * Hardware reset du SX1280 suivi de la configuration LoRa COMPLETE.
 * Calibrate(0x7E) apres reset pour corriger toute derive thermique.
 *
 ****************************************************************************/
PRIVATE void vReconfigureLoRaForTx(void)
{
    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: HW Reset + Calibrate + full reconfig...\r\n");

    /* ====== HW Reset + Calibrate ======
     * Le TCXOEN est cable directement au 3.3V (pas sur un GPIO),
     * donc on ne peut pas power-cycler le TCXO par software.
     * A la place, apres le HW reset, on utilise la commande
     * Calibrate (0x89) pour forcer la recalibration des oscillateurs
     * RC internes et du PLL, corrigeant toute derive progressive.
     */
    GPIO_PinWrite(GPIO, LORA_GPIO_PORT, SX1280_NSS_PIN, 1);

    /* 1) RESET pulse LOW 1ms */
    GPIO_PinWrite(GPIO, LORA_GPIO_PORT, SX1280_RESET_PIN, 0);
    for (volatile uint32 i = 0; i < 12000; i++);  /* ~1ms @ 12MHz */
    GPIO_PinWrite(GPIO, LORA_GPIO_PORT, SX1280_RESET_PIN, 1);

    /* 2) Attendre que le TCXO soit stable apres reset (~5ms) */
    for (volatile uint32 i = 0; i < 60000; i++);  /* ~5ms @ 12MHz */

    /* Attendre BUSY LOW (max 20ms) */
    {
        uint32 u32Count = 0;
        while (GPIO_PinRead(GPIO, LORA_GPIO_PORT, SX1280_BUSY_PIN))
        {
            if (++u32Count >= 240000) break;
        }
    }

    /* ====== Full ConfigureLoRa avec delais explicites ====== */

    /* 1) Standby RC */
    SX1280_SetStandby(SX1280_STANDBY_RC);
    for (volatile uint32 i = 0; i < 12000; i++);  /* 1ms stabilisation */

    /* 2) Calibrate — force recalibration RC13M + RC64k + PLL (~3.5ms)
     * 0x7E = tous les blocs (RC64k, RC13M, PLL, ADC pulse, ADC bulk N/P)
     * Corrige la derive thermique progressive des oscillateurs internes
     * qui cause la degradation SNR au recepteur apres de nombreux TX. */
    SX1280_Calibrate(0x7E);
    /* Attendre fin calibration (~3.5ms, BUSY HIGH pendant) */
    {
        uint32 u32Count = 0;
        while (GPIO_PinRead(GPIO, LORA_GPIO_PORT, SX1280_BUSY_PIN))
        {
            if (++u32Count >= 60000) break;  /* ~5ms max */
        }
    }

    /* 3) Buffer base addresses */
    SX1280_SetBufferBaseAddress(0x00, 0x00);

    /* 4) Packet Type LoRa */
    SX1280_SetPacketType(SX1280_PACKET_TYPE_LORA);

    /* 5) CAD params (CAD_ON_8_SYMB = 0x60) */
    {
        uint8 cadParam = 0x60;
        SX1280_HAL_WriteCommand(0x88, &cadParam, 1);
    }

    /* 6) Regulator LDO */
    SX1280_SetRegulatorMode(SX1280_REGULATOR_LDO);

    /* 7) Frequence 2490 MHz + delai PLL settling */
    SX1280_SetRfFrequency(2490000000UL);
    for (volatile uint32 i = 0; i < 24000; i++);  /* 2ms PLL */

    /* 8) Modulation SF10/BW406/CR4_5 + mandatory registers */
    {
        SX1280_LoRaModParams_t modParams;
        modParams.spreadingFactor = SX1280_LORA_SF10;
        modParams.bandwidth       = SX1280_LORA_BW_406;
        modParams.codingRate      = SX1280_LORA_CR_4_5;
        SX1280_SetModulationParams(&modParams);
    }
    for (volatile uint32 i = 0; i < 12000; i++);  /* 1ms post-modulation */

    /* 9) Sync Word 0x14,0x24 = RadioLib 0x12 */
    {
        uint8 syncWord[2] = {0x14, 0x24};
        SX1280_HAL_WriteRegisters(0x0944, syncWord, 2);
    }

    /* 10) Packet params */
    {
        SX1280_LoRaPacketParams_t pktParams;
        pktParams.preambleLength = 0x18;
        pktParams.headerType     = SX1280_LORA_HEADER_EXPLICIT;
        pktParams.payloadLength  = 255;
        pktParams.crc            = SX1280_LORA_CRC_ON;
        pktParams.invertIQ       = SX1280_LORA_IQ_NORMAL;
        SX1280_SetPacketParams(&pktParams);
    }

    /* 11) TX Power +10 dBm, ramp 20us */
    SX1280_SetTxParams(10, SX1280_RAMP_20_US);

    /* 12) IRQ : TX_DONE + TIMEOUT sur DIO1 */
    SX1280_SetDioIrqParams(
        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,
        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);

    /* ====== Readback registres critiques (mode verbose uniquement) ====== */
    if (!g_u8SX1280_HalSilent)
    {
        uint8 regPktType = 0, regSF = 0, regBW = 0, regCR = 0;
        uint8 reg0925 = 0, regSW[2] = {0, 0};

        SX1280_HAL_ReadRegisters(0x0889, &regPktType, 1);
        SX1280_HAL_ReadRegisters(0x088B, &regSF, 1);
        SX1280_HAL_ReadRegisters(0x088C, &regBW, 1);
        SX1280_HAL_ReadRegisters(0x088D, &regCR, 1);
        SX1280_HAL_ReadRegisters(0x0925, &reg0925, 1);
        SX1280_HAL_ReadRegisters(0x0944, regSW, 2);

        DBG_vPrintf(TRACE_LORA_TIC,
            "LORA_TIC: Verify: PktType=0x%02X SF=0x%02X BW=0x%02X CR=0x%02X "
            "r0925=0x%02X SW=%02X%02X Status=0x%02X\r\n",
            regPktType, regSF, regBW, regCR, reg0925,
            regSW[0], regSW[1], SX1280_GetStatus());
    }
}

/****************************************************************************
 *
 * NAME: bTransmitAndWait
 *
 * DESCRIPTION:
 * Envoie le payload via SX1280 et attend la fin du TX.
 *
 * Strategie "ZERO SPI pendant TX" :
 * - Apres SendPayload, AUCUN acces SPI pendant la transmission
 *   (le polling SPI generait du bruit EMI corrompant le signal RF)
 * - Attente basee sur DIO1 GPIO (pas de SPI) + timeout DWT
 * - Apres detection DIO1 ou timeout, UNE SEULE lecture status/IRQ
 * - Si DIO1 ne fonctionne pas, fallback sur timeout + verification
 *   que le SX1280 est revenu en STDBY (= TX terminee naturellement)
 *
 * RETURNS:
 * TRUE si TX complete, FALSE si timeout ou erreur.
 *
 ****************************************************************************/
PRIVATE bool_t bTransmitAndWait(uint8 *pu8Data, uint8 u8Len)
{
    /* ===== Initialiser le compteur de cycles DWT ===== */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32 u32CpuHz = SystemCoreClock;  /* 12000000 ou 48000000 */
    uint32 u32CycPerMs = u32CpuHz / 1000;

    /* ===== Mode SILENT pendant reconfig + TX ===== */
    g_u8SX1280_HalSilent = 1;

    vReconfigureLoRaForTx();

    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: Starting TX (silent mode)...\r\n");

    DWT->CYCCNT = 0;
    SX1280_SendPayload(pu8Data, u8Len);

    /* ===== ZERO SPI pendant TX =====
     * Apres SendPayload (= SetTx), AUCUN SPI ni UART.
     * Poll DIO1 GPIO uniquement. Timeout 500ms (airtime ~100-160ms).
     */

    bool_t bDio1Detected = FALSE;
    uint32 u32WaitCycles = 500UL * u32CycPerMs;  /* 500ms max */

    while (DWT->CYCCNT < u32WaitCycles)
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
        {
            bDio1Detected = TRUE;
            break;
        }
        /* Dispatch cooperative timers (LED blink etc.) pendant le poll TX */
        ZTIMER_vTask();
    }

    uint32 u32ElapsedMs = DWT->CYCCNT / u32CycPerMs;

    /* ===== TX fini — re-activer les PRINTF HAL pour diagnostics ===== */
    g_u8SX1280_HalSilent = 0;

    uint8 u8Status = SX1280_GetStatus();
    uint16 u16Irq = SX1280_GetIrqStatus();
    uint8 u8Dio1 = GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN);
    uint8 u8Mode = (u8Status >> 5) & 0x07;

    /* GetDeviceErrors (0x17) — bit6=PLL_LOCK_ERR, bit5=XOSC_START_ERR */
    uint8 errBuf[2] = {0, 0};
    SX1280_HAL_ReadCommand(0x17, errBuf, 2);
    uint16 u16DevErrors = ((uint16)errBuf[0] << 8) | errBuf[1];

    DBG_vPrintf(TRACE_LORA_TIC,
        "LORA_TIC: Post-TX @%lums DIO1det=%d DIO1now=%d Status=0x%02X mode=%d "
        "IRQ=0x%04X DevErr=0x%04X\r\n",
        u32ElapsedMs, bDio1Detected, u8Dio1, u8Status, u8Mode, u16Irq, u16DevErrors);

    if (u16DevErrors != 0)
    {
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: *** DEVICE ERRORS: %s%s ***\r\n",
                    (u16DevErrors & 0x0040) ? "PLL_LOCK " : "",
                    (u16DevErrors & 0x0020) ? "XOSC_START " : "");
        /* Clear device errors (0x07 = ClearDeviceErrors sur SX1280) */
        uint8 clrBuf[2] = {0x00, 0x00};
        SX1280_HAL_WriteCommand(0x07, clrBuf, 2);
    }

    /* Cas 1 : DIO1 detecte — TX_DONE confirme */
    if (bDio1Detected)
    {
        SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: TX OK (DIO1 @%lums)\r\n", u32ElapsedMs);
        return TRUE;
    }

    /* Cas 2 : DIO1 pas vu mais mode STDBY — TX terminee naturellement */
    if (u8Mode == 2 || u8Mode == 3)
    {
        SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
        DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: TX OK (STDBY, no DIO1) @%lums\r\n", u32ElapsedMs);
        return TRUE;
    }

    /* Cas 3 : Vrai timeout — ni DIO1 ni retour en STDBY */
    DBG_vPrintf(TRACE_LORA_TIC, "LORA_TIC: TX TIMEOUT @%lums (mode=%d)\r\n",
                u32ElapsedMs, u8Mode);
    return FALSE;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

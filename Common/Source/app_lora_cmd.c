/*****************************************************************************
 *
 * MODULE:             app_lora_cmd
 *
 * COMPONENT:          app_lora_cmd.c
 *
 * DESCRIPTION:        Canal descendant : lecture d'attribut a la demande.
 *                     Voir app_lora_cmd.h pour le principe.
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "ZTimer.h"
#include "fsl_gpio.h"

#include "app_lora_cmd.h"
#include "app_lora_tic.h"
#include "app_lora_pair.h"
#include "app_lora_crypt.h"

#include "app_common.h"
#include "App_Linky.h"
#include "app_zcl_task.h"
#include "LixeeCluster.h"

#include "sx1280.h"
#include "sx1280_hal.h"
#include "sx1280_config.h"
#ifdef WATCHDOG_ALLOWED
#include "fsl_wwdt.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define TRACE_LORA_CMD  TRUE

#define LORA_TIC_PROTOCOL_VERSION_CMD  0x01

#define LORA_GPIO_PORT   SX1280_GPIO_PORT
#define LORA_DIO1_PIN    SX1280_DIO1_PIN

extern volatile uint8_t g_u8SX1280_HalSilent;
extern PUBLIC tsZHA_BaseDevice sBaseDevice;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/* Sequence des reponses descendantes */
PRIVATE uint8 u8RespSeq = 0;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vProcessPollRequest(uint8 *pu8Buf, uint8 u8Len);
PRIVATE int  iBuildPollValue(uint16 u16Cluster, uint16 u16Attr,
                             uint8 *pu8Type, uint8 *pu8Out);
PRIVATE void vSendPollResponse(uint16 u16Cluster, uint16 u16Attr,
                               uint8 u8Status, uint8 u8Type,
                               const uint8 *pu8Value, uint8 u8ValLen);

/* Helpers de serialisation */
PRIVATE uint8 u8PutU16(uint8 *p, uint16 v);
PRIVATE uint8 u8PutU32(uint8 *p, uint32 v);
PRIVATE uint8 u8PutAscii(uint8 *p, const uint8 *src, uint8 u8MaxLen);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void APP_vLoraCmdInit(void)
{
    u8RespSeq = 0;
    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: Init (fenetre RX %d ms, poll attribut)\r\n",
                LORA_CMD_RX_WINDOW_MS);
}

/****************************************************************************
 *
 * NAME: APP_vLoraCmdRxWindow
 *
 * DESCRIPTION:
 * Ouvre la fenetre de reception d'une requete de lecture, juste apres un TX
 * de donnees.
 *
 * La radio sort d'un TX : DIO1 est mappe sur TX_DONE et payloadLength vaut la
 * taille du dernier paquet emis. Il faut restaurer les deux avant de passer en
 * RX, sinon le paquet est bien recu par le modem mais DIO1 ne se leve jamais
 * (meme piege que sur l'OTA).
 *
 ****************************************************************************/
PUBLIC void APP_vLoraCmdRxWindow(void)
{
    if (!APP_bLoraPairIsPaired())
        return;

    g_u8SX1280_HalSilent = 1;

    /* 1) payloadLength = 255 */
    {
        SX1280_LoRaPacketParams_t pktParams;
        pktParams.preambleLength = SX1280_LORA_PREAMBLE;
        pktParams.headerType     = SX1280_LORA_HEADER_EXPLICIT;
        pktParams.payloadLength  = 255;
        pktParams.crc            = SX1280_LORA_CRC_ON;
        pktParams.invertIQ       = SX1280_LORA_IQ_NORMAL;
        SX1280_SetPacketParams(&pktParams);
    }

    /* 2) DIO1 sur RX_DONE + CRC_ERROR */
    SX1280_SetDioIrqParams(
        SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR,
        SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR);

    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

    /* 3) RX avec timeout materiel (base 1 ms) */
    SX1280_SetRx(LORA_CMD_RX_WINDOW_MS);

    /* 4) Attente DIO1, bornee par le compteur de cycles DWT.
     *    ZTIMER_vTask() garde la LED et le scan bouton vivants. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32 u32CycPerMs = SystemCoreClock / 1000;
    if (u32CycPerMs == 0) u32CycPerMs = 12000;
    uint32 u32WaitCycles = (uint32)LORA_CMD_RX_WINDOW_MS * u32CycPerMs;

    bool_t bDio1 = FALSE;
    while (DWT->CYCCNT < u32WaitCycles)
    {
        if (GPIO_PinRead(GPIO, LORA_GPIO_PORT, LORA_DIO1_PIN))
        {
            bDio1 = TRUE;
            break;
        }
        ZTIMER_vTask();
#ifdef WATCHDOG_ALLOWED
        /* Fenetre RX de 300 ms : bornee par le compteur DWT, donc la
         * nourrir ici ne masque aucun blocage (cf. bTransmitAndWait). */
        WWDT_Refresh(WWDT);
#endif
    }

    if (!bDio1)
    {
        /* Cas nominal : aucune requete en attente cote recepteur */
        SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
        SX1280_SetStandby(SX1280_STANDBY_RC);
        g_u8SX1280_HalSilent = 0;
        return;
    }

    uint16 u16Irq = SX1280_GetIrqStatus();
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

    if (!(u16Irq & SX1280_IRQ_RX_DONE))
    {
        SX1280_SetStandby(SX1280_STANDBY_RC);
        g_u8SX1280_HalSilent = 0;
        return;
    }

    /* 5) Lecture bornee : u8Len vient du header radio (source non fiable) */
    uint8 au8RxBuf[LORA_POLL_REQUEST_SIZE + LORA_CRYPT_MIC_SIZE + 4];
    uint8 u8Len = 0;
    int8  i8Rssi = 0, i8Snr = 0;

    SX1280_GetRxPayload(au8RxBuf, &u8Len, &i8Rssi, &i8Snr);
    SX1280_SetStandby(SX1280_STANDBY_RC);
    g_u8SX1280_HalSilent = 0;

    if (u8Len == 0 || u8Len > sizeof(au8RxBuf))
    {
        DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: taille RX invalide (%d)\r\n", u8Len);
        return;
    }

    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: RX %d octets RSSI=%d SNR=%d\r\n",
                u8Len, i8Rssi, i8Snr);

    /* 6) Dechiffrement + verification du MIC */
    if (!APP_bLoraCryptDecrypt(au8RxBuf, u8Len,
                               APP_pu8LoraPairGetKey(),
                               APP_pu8LoraPairGetMAC(),
                               au8RxBuf[1]))
    {
        DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: MIC invalide, ignore\r\n");
        return;
    }

    vProcessPollRequest(au8RxBuf, u8Len - LORA_CRYPT_MIC_SIZE);
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vProcessPollRequest
 *
 * DESCRIPTION:
 * Parse une requete de lecture et repond avec la valeur de l'attribut.
 *
 * Format requete : [0]=type [1]=seq [2-3]=cluster BE [4-5]=attr BE
 *
 ****************************************************************************/
PRIVATE void vProcessPollRequest(uint8 *pu8Buf, uint8 u8Len)
{
    if (u8Len < LORA_POLL_REQUEST_SIZE)
    {
        DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: requete trop courte (%d)\r\n", u8Len);
        return;
    }

    uint8 u8Type = pu8Buf[0] & 0x0F;
    if (u8Type != LORA_TIC_PKT_POLL_REQUEST)
        return;   /* pas une requete de lecture : ignore */

    uint16 u16Cluster = ((uint16)pu8Buf[2] << 8) | pu8Buf[3];
    uint16 u16Attr    = ((uint16)pu8Buf[4] << 8) | pu8Buf[5];

    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: POLL cluster=0x%04X attr=0x%04X\r\n",
                u16Cluster, u16Attr);

    uint8 au8Value[LORA_POLL_VALUE_MAX];
    uint8 u8ZclType = 0;
    int   iValLen = iBuildPollValue(u16Cluster, u16Attr, &u8ZclType, au8Value);

    if (iValLen < 0)
    {
        vSendPollResponse(u16Cluster, u16Attr, LORA_POLL_STATUS_UNKNOWN_ATTR,
                          0, NULL, 0);
        return;
    }

    vSendPollResponse(u16Cluster, u16Attr, LORA_POLL_STATUS_OK,
                      u8ZclType, au8Value, (uint8)iValLen);
}

/****************************************************************************
 *
 * NAME: iBuildPollValue
 *
 * DESCRIPTION:
 * Serialise la valeur d'un attribut (cluster, attr) dans pu8Out (big-endian
 * pour les numeriques) et renvoie sa longueur, ou -1 si l'attribut n'est pas
 * gere. *pu8Type recoit le type ZCL.
 *
 * Couvre TOUS les attributs TIC disponibles, repartis sur 4 clusters aux
 * memes identifiants que le firmware Zigbee LiXee :
 *   0xFF66 LiXee custom, 0x0702 Simple Metering, 0x0B04 Electrical
 *   Measurement, 0x0B01 Meter Identification.
 *
 ****************************************************************************/

/* Raccourcis de retour, locaux a cette fonction */
#define POLL_U8(v)    do { *pu8Type = LORA_ZCL_TYPE_UINT8;  pu8Out[0] = (uint8)(v); return 1; } while(0)
#define POLL_U16(v)   do { *pu8Type = LORA_ZCL_TYPE_UINT16; return u8PutU16(pu8Out, (uint16)(v)); } while(0)
#define POLL_S16(v)   do { *pu8Type = LORA_ZCL_TYPE_INT16;  return u8PutU16(pu8Out, (uint16)(int16)(v)); } while(0)
#define POLL_U32(v)   do { *pu8Type = LORA_ZCL_TYPE_UINT32; return u8PutU32(pu8Out, (uint32)(v)); } while(0)
#define POLL_S32(v)   do { *pu8Type = LORA_ZCL_TYPE_INT32;  return u8PutU32(pu8Out, (uint32)(int32)(v)); } while(0)
#define POLL_STR(p,n) do { *pu8Type = LORA_ZCL_TYPE_CHAR_STRING; return u8PutAscii(pu8Out, (p), (n)); } while(0)

PRIVATE int iBuildPollValue(uint16 u16Cluster, uint16 u16Attr,
                            uint8 *pu8Type, uint8 *pu8Out)
{
    tsCLD_Linky *pL = &sBaseDevice.sLinkyServerCluster;

    /* ========== Cluster Basic 0x0000 (identification de l'objet) ==========
     * Permet au recepteur de choisir son template apres l'appairage, sans
     * alourdir le PAIR_REQUEST (cf. LORA_CLUSTER_BASIC dans le .h). */
    if (u16Cluster == LORA_CLUSTER_BASIC)
    {
        static const char szModel[] = LORA_DEVICE_MODEL_ZLINKY_TIC;

        switch (u16Attr)
        {
        case LORA_ATTR_BASIC_DEVICE_ID:
            POLL_U16(LORA_DEVICE_ID_ZLINKY_TIC);
        case LORA_ATTR_BASIC_MODEL:
            POLL_STR((const uint8 *)szModel, (uint8)(sizeof(szModel) - 1));
        default:
            return -1;
        }
    }

    /* ================= Cluster LiXee 0xFF66 (donnees TIC) ================= */
    if (u16Cluster == LORA_CLUSTER_LIXEE)
    {
        switch (u16Attr)
        {
        case E_CLD_LINKY_OPTARIF:          POLL_STR(pL->au8LinkyOptarif, 16);
        case E_CLD_LINKY_DEMAIN:           POLL_STR(pL->au8LinkyDemain, 4);
        case E_CLD_LINKY_HHPHC:            POLL_U8 (pL->au8LinkyHHPHC);
        case E_CLD_LINKY_PPOT:             POLL_U8 (pL->au8LinkyPPOT);
        case E_CLD_LINKY_PEJP:             POLL_U8 (pL->au8LinkyPEJP);
        case E_CLD_LINKY_ADPS:             POLL_U16(pL->au16LinkyADPS);
        case E_CLD_LINKY_ADIR1:            POLL_U16(pL->au16LinkyADIR1);
        case E_CLD_LINKY_ADIR2:            POLL_U16(pL->au16LinkyADIR2);
        case E_CLD_LINKY_ADIR3:            POLL_U16(pL->au16LinkyADIR3);
        case E_CLD_LINKY_MOTDETAT:         POLL_STR(pL->au8LinkyMOTDETAT, 6);
        case E_CLD_LINKY_TARIFPERIOD:      POLL_STR(pL->au8LinkyTarifPeriod, 16);
        case E_CLD_LINKY_PERIODIC_SEND:    POLL_U8 (pL->au8LinkyPeriodicSend);
        case E_CLD_LINKY_STANDARD_LTARF:   POLL_STR(pL->au8LinkyLTARF, 16);
        case E_CLD_LINKY_STANDARD_NTARF:   POLL_U8 (pL->au8LinkyNTARF);
        case E_CLD_LINKY_STANDARD_DATE:    POLL_STR(pL->au8LinkyDATE, 13);
        case E_CLD_LINKY_STANDARD_EASD01:  POLL_U32(pL->u32LinkyEASD01);
        case E_CLD_LINKY_STANDARD_EASD02:  POLL_U32(pL->u32LinkyEASD02);
        case E_CLD_LINKY_STANDARD_EASD03:  POLL_U32(pL->u32LinkyEASD03);
        case E_CLD_LINKY_STANDARD_EASD04:  POLL_U32(pL->u32LinkyEASD04);
        case E_CLD_LINKY_STANDARD_SINSTI:  POLL_U16(pL->u16LinkySINSTI);
        case E_CLD_LINKY_STANDARD_SMAXIN:  POLL_U16(pL->u16LinkySMAXIN);
        case E_CLD_LINKY_STANDARD_SMAXIN_1:POLL_U16(pL->u16LinkySMAXIN_1);
        case E_CLD_LINKY_STANDARD_CCAIN:   POLL_U16(pL->u16LinkyCCAIN);
        case E_CLD_LINKY_STANDARD_CCAIN_1: POLL_U16(pL->u16LinkyCCAIN_1);
        case E_CLD_LINKY_STANDARD_SMAXSN1_1: POLL_U16(pL->u16LinkySMAXSN1_1);
        case E_CLD_LINKY_STANDARD_SMAXSN2_1: POLL_U16(pL->u16LinkySMAXSN2_1);
        case E_CLD_LINKY_STANDARD_SMAXSN3_1: POLL_U16(pL->u16LinkySMAXSN3_1);
        case E_CLD_LINKY_STANDARD_MSG1:    POLL_STR(pL->au8LinkyMSG1, 32);
        case E_CLD_LINKY_STANDARD_MSG2:    POLL_STR(pL->au8LinkyMSG2, 16);
        case E_CLD_LINKY_STANDARD_STGE:    POLL_STR(pL->au8LinkySTGE, 8); /* 8 car. hex */
        case E_CLD_LINKY_STANDARD_DPM1:    POLL_U8 (pL->u8LinkyDPM1);
        case E_CLD_LINKY_STANDARD_FPM1:    POLL_U8 (pL->u8LinkyFPM1);
        case E_CLD_LINKY_STANDARD_DPM2:    POLL_U8 (pL->u8LinkyDPM2);
        case E_CLD_LINKY_STANDARD_FPM2:    POLL_U8 (pL->u8LinkyFPM2);
        case E_CLD_LINKY_STANDARD_DPM3:    POLL_U8 (pL->u8LinkyDPM3);
        case E_CLD_LINKY_STANDARD_FPM3:    POLL_U8 (pL->u8LinkyFPM3);
        case E_CLD_LINKY_STANDARD_RELAIS:  POLL_U16(pL->u16LinkyRELAIS);
        case E_CLD_LINKY_STANDARD_NJOURF:  POLL_U8 (pL->u8LinkyNJOURF);
        case E_CLD_LINKY_STANDARD_NJOURF1: POLL_U8 (pL->u8LinkyNJOURF1);
        case E_CLD_LINKY_STANDARD_PJOURF1: POLL_STR(pL->au8LinkyPJOURF1, 32);
        case E_CLD_LINKY_STANDARD_PPOINTE: POLL_STR(pL->au8LinkyPPOINTE, 32);
        case E_CLD_LINKY_MODE:             POLL_U8 (pL->au8LinkyMode);
        case E_CLD_LINKY_STANDARD_SMAXSN_1: POLL_U16(pL->u16LinkySMAXSN_1);
        default:                           return -1;
        }
    }

    /* ============= Cluster Simple Metering 0x0702 (index Wh) =============
     * Valeurs 48 bits mais l'energie TIC tient sur 32 : on transmet 4 octets,
     * comme les paquets ENERGY. Tier N = 0x0100 + (N-1). */
    if (u16Cluster == LORA_CLUSTER_METERING)
    {
        tsCLD_SimpleMetering *pM = &sBaseDevice.sSimpleMeteringServerCluster;
        switch (u16Attr)
        {
        case 0x0000: POLL_U32(pM->u48CurrentSummationDelivered);  /* EAST/BASE */
        case 0x0001: POLL_U32(pM->u48CurrentSummationReceived);   /* EAIT */
        case 0x0100: POLL_U32(pM->u48CurrentTier1SummationDelivered);
        case 0x0101: POLL_U32(pM->u48CurrentTier2SummationDelivered);
        case 0x0102: POLL_U32(pM->u48CurrentTier3SummationDelivered);
        case 0x0103: POLL_U32(pM->u48CurrentTier4SummationDelivered);
        case 0x0104: POLL_U32(pM->u48CurrentTier5SummationDelivered);
        case 0x0105: POLL_U32(pM->u48CurrentTier6SummationDelivered);
        case 0x0106: POLL_U32(pM->u48CurrentTier7SummationDelivered);
        case 0x0107: POLL_U32(pM->u48CurrentTier8SummationDelivered);
        case 0x0108: POLL_U32(pM->u48CurrentTier9SummationDelivered);
        case 0x0109: POLL_U32(pM->u48CurrentTier10SummationDelivered);
        default:     return -1;
        }
    }

    /* ========= Cluster Electrical Measurement 0x0B04 (puissances) =========
     * IDs ZCL standard. Puissances actives/reactives signees (int). */
    if (u16Cluster == LORA_CLUSTER_ELEC_MEAS)
    {
        tsCLD_ElectricalMeasurement *pE = &sBaseDevice.sElectricalMeasurement;
        switch (u16Attr)
        {
        case 0x0304: POLL_S32(pE->i32TotalActivePower);      /* SMAXSN (jour) */
        case 0x0305: POLL_S32(pE->i32TotalReactivePower);    /* ERQ1 */
        case 0x0306: POLL_U32(pE->u32TotalApparentPower);    /* SINSTS total */
        case 0x0505: POLL_U16(pE->u16RMSVoltage);            /* URMS1 */
        case 0x0508: POLL_U16(pE->u16RMSCurrent);            /* IRMS1 / IINST1 */
        case 0x050A: POLL_U16(pE->u16RMSCurrentMax);         /* IMAX1 */
        case 0x050B: POLL_S16(pE->i16ActivePower);           /* CCASN */
        case 0x050D: POLL_S16(pE->i16ActivePowerMax);        /* SMAXSN1 / PMAX */
        case 0x050E: POLL_S16(pE->i16ReactivePower);         /* ERQ2 */
        case 0x050F: POLL_U16(pE->u16ApparentPower);         /* SINSTS1 / PAPP */
        case 0x0511: POLL_U16(pE->u16AverageRMSVoltageMeasurementPeriod);     /* UMOY1 */
        case 0x0905: POLL_U16(pE->u16RMSVoltagePhB);         /* URMS2 */
        case 0x0908: POLL_U16(pE->u16RMSCurrentPhB);         /* IRMS2 / IINST2 */
        case 0x090A: POLL_U16(pE->u16RMSCurrentMaxPhB);      /* IMAX2 */
        case 0x090B: POLL_S16(pE->i16ActivePowerPhB);        /* CCASN-1 */
        case 0x090D: POLL_S16(pE->i16ActivePowerMaxPhB);     /* SMAXSN2 */
        case 0x090E: POLL_S16(pE->i16ReactivePowerPhB);      /* ERQ3 */
        case 0x090F: POLL_U16(pE->u16ApparentPowerPhB);      /* SINSTS2 */
        case 0x0911: POLL_U16(pE->u16AverageRMSVoltageMeasurementPeriodPhB);  /* UMOY2 */
        case 0x0A05: POLL_U16(pE->u16RMSVoltagePhC);         /* URMS3 */
        case 0x0A08: POLL_U16(pE->u16RMSCurrentPhC);         /* IRMS3 / IINST3 */
        case 0x0A0A: POLL_U16(pE->u16RMSCurrentMaxPhC);      /* IMAX3 */
        case 0x0A0D: POLL_S16(pE->i16ActivePowerMaxPhC);     /* SMAXSN3 */
        case 0x0A0E: POLL_S16(pE->i16ReactivePowerPhC);      /* ERQ4 */
        case 0x0A0F: POLL_U16(pE->u16ApparentPowerPhC);      /* SINSTS3 */
        case 0x0A11: POLL_U16(pE->u16AverageRMSVoltageMeasurementPeriodPhC);  /* UMOY3 */
        default:     return -1;
        }
    }

    /* ========= Cluster Meter Identification 0x0B01 (config compteur) ======= */
    if (u16Cluster == LORA_CLUSTER_METER_ID)
    {
        switch (u16Attr)
        {
        case 0x000A: POLL_U8 (sBaseDevice.sMeterIdentification.au8LinkyMISoftwareRevision); /* VTIC */
        case 0x000C: POLL_U16(sBaseDevice.sMeterIdentification.au16LinkyMIPOD);             /* POD */
        case 0x000D: POLL_U8 (sBaseDevice.sMeterIdentification.au8LinkyMIAvailablePower);   /* PREF/ISOUSC */
        case 0x000E: POLL_U8 (sBaseDevice.sMeterIdentification.au8LinkyMIPowerThreshold);   /* PCOUP */
        default:     return -1;
        }
    }

    return -1;   /* cluster non gere */
}

#undef POLL_U8
#undef POLL_U16
#undef POLL_S16
#undef POLL_U32
#undef POLL_S32
#undef POLL_STR

/****************************************************************************
 *
 * NAME: vSendPollResponse
 *
 * DESCRIPTION:
 * Construit et emet un POLL_RESPONSE sur la config operationnelle courante.
 *
 * Format : [0]=type [1]=seq [2-3]=cluster [4-5]=attr [6]=status
 *          [7]=zcl_type [8]=len [9..]=valeur
 *
 ****************************************************************************/
PRIVATE void vSendPollResponse(uint16 u16Cluster, uint16 u16Attr,
                               uint8 u8Status, uint8 u8Type,
                               const uint8 *pu8Value, uint8 u8ValLen)
{
    uint8 au8Pkt[LORA_POLL_RESP_MAX_SIZE + LORA_CRYPT_MIC_SIZE];

    if (u8ValLen > LORA_POLL_VALUE_MAX)
        u8ValLen = LORA_POLL_VALUE_MAX;

    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_CMD << 4) | LORA_TIC_PKT_POLL_RESPONSE;
    au8Pkt[1] = u8RespSeq++;
    au8Pkt[2] = (uint8)(u16Cluster >> 8);
    au8Pkt[3] = (uint8)(u16Cluster & 0xFF);
    au8Pkt[4] = (uint8)(u16Attr >> 8);
    au8Pkt[5] = (uint8)(u16Attr & 0xFF);
    au8Pkt[6] = u8Status;
    au8Pkt[7] = u8Type;
    au8Pkt[8] = u8ValLen;
    if (u8ValLen > 0 && pu8Value != NULL)
        memcpy(&au8Pkt[LORA_POLL_RESP_HDR_SIZE], pu8Value, u8ValLen);

    uint8 u8Clear = LORA_POLL_RESP_HDR_SIZE + u8ValLen;

    uint8 u8TxLen = APP_u8LoraCryptEncrypt(au8Pkt, u8Clear,
                                            APP_pu8LoraPairGetKey(),
                                            APP_pu8LoraPairGetMAC(),
                                            au8Pkt[1]);

    DBG_vPrintf(TRACE_LORA_CMD,
                "LORA_CMD: TX RESP cluster=0x%04X attr=0x%04X status=0x%02X type=0x%02X len=%d\r\n",
                u16Cluster, u16Attr, u8Status, u8Type, u8ValLen);

    APP_bLoraTicTransmitRaw(au8Pkt, u8TxLen);
}

/****************************************************************************/
/***        Serialisation helpers                                          ***/
/****************************************************************************/

PRIVATE uint8 u8PutU16(uint8 *p, uint16 v)
{
    p[0] = (uint8)(v >> 8);
    p[1] = (uint8)(v & 0xFF);
    return 2;
}

PRIVATE uint8 u8PutU32(uint8 *p, uint32 v)
{
    p[0] = (uint8)(v >> 24);
    p[1] = (uint8)(v >> 16);
    p[2] = (uint8)(v >> 8);
    p[3] = (uint8)(v & 0xFF);
    return 4;
}

/* Copie une chaine ASCII imprimable, espaces/nuls de fin retires, bornee a
 * u8MaxLen et a LORA_POLL_VALUE_MAX. */
PRIVATE uint8 u8PutAscii(uint8 *p, const uint8 *src, uint8 u8MaxLen)
{
    if (u8MaxLen > LORA_POLL_VALUE_MAX)
        u8MaxLen = LORA_POLL_VALUE_MAX;

    uint8 u8Len = 0;
    uint8 i;
    for (i = 0; i < u8MaxLen; i++)
    {
        uint8 c = src[i];
        if (c == 0x00) break;
        if (c < 0x20 || c > 0x7E) break;
        p[i] = c;
        u8Len++;
    }
    while (u8Len > 0 && p[u8Len - 1] == ' ')
        u8Len--;

    return u8Len;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

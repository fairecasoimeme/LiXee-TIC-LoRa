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
        pktParams.preambleLength = 0x18;
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
 * Table volontairement extensible : ajouter un case suffit. Les clusters
 * standard non listes (mesure electrique 0x0B04, identification 0x0B01)
 * peuvent etre ajoutes sur le meme modele en pointant vers les champs
 * correspondants de sBaseDevice.
 *
 ****************************************************************************/
PRIVATE int iBuildPollValue(uint16 u16Cluster, uint16 u16Attr,
                            uint8 *pu8Type, uint8 *pu8Out)
{
    if (u16Cluster == LORA_CLUSTER_LIXEE)
    {
        switch (u16Attr)
        {
        case E_CLD_LINKY_OPTARIF:
            *pu8Type = LORA_ZCL_TYPE_CHAR_STRING;
            return u8PutAscii(pu8Out, sBaseDevice.sLinkyServerCluster.au8LinkyOptarif, 16);

        case E_CLD_LINKY_DEMAIN:
            *pu8Type = LORA_ZCL_TYPE_CHAR_STRING;
            return u8PutAscii(pu8Out, sBaseDevice.sLinkyServerCluster.au8LinkyDemain, 4);

        case E_CLD_LINKY_ADPS:
            *pu8Type = LORA_ZCL_TYPE_UINT16;
            return u8PutU16(pu8Out, sBaseDevice.sLinkyServerCluster.au16LinkyADPS);

        case E_CLD_LINKY_TARIFPERIOD:
            *pu8Type = LORA_ZCL_TYPE_CHAR_STRING;
            return u8PutAscii(pu8Out, sBaseDevice.sLinkyServerCluster.au8LinkyTarifPeriod, 16);

        case E_CLD_LINKY_STANDARD_LTARF:
            *pu8Type = LORA_ZCL_TYPE_CHAR_STRING;
            return u8PutAscii(pu8Out, sBaseDevice.sLinkyServerCluster.au8LinkyLTARF, 16);

        case E_CLD_LINKY_STANDARD_NTARF:
            *pu8Type = LORA_ZCL_TYPE_UINT8;
            pu8Out[0] = sBaseDevice.sLinkyServerCluster.au8LinkyNTARF;
            return 1;

        case E_CLD_LINKY_STANDARD_DATE:
            *pu8Type = LORA_ZCL_TYPE_CHAR_STRING;
            return u8PutAscii(pu8Out, sBaseDevice.sLinkyServerCluster.au8LinkyDATE, 13);

        case E_CLD_LINKY_STANDARD_EASD01:
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, sBaseDevice.sLinkyServerCluster.u32LinkyEASD01);
        case E_CLD_LINKY_STANDARD_EASD02:
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, sBaseDevice.sLinkyServerCluster.u32LinkyEASD02);
        case E_CLD_LINKY_STANDARD_EASD03:
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, sBaseDevice.sLinkyServerCluster.u32LinkyEASD03);
        case E_CLD_LINKY_STANDARD_EASD04:
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, sBaseDevice.sLinkyServerCluster.u32LinkyEASD04);

        case E_CLD_LINKY_STANDARD_SINSTI:
            *pu8Type = LORA_ZCL_TYPE_UINT16;
            return u8PutU16(pu8Out, sBaseDevice.sLinkyServerCluster.u16LinkySINSTI);
        case E_CLD_LINKY_STANDARD_SMAXIN:
            *pu8Type = LORA_ZCL_TYPE_UINT16;
            return u8PutU16(pu8Out, sBaseDevice.sLinkyServerCluster.u16LinkySMAXIN);
        case E_CLD_LINKY_STANDARD_CCAIN:
            *pu8Type = LORA_ZCL_TYPE_UINT16;
            return u8PutU16(pu8Out, sBaseDevice.sLinkyServerCluster.u16LinkyCCAIN);

        case E_CLD_LINKY_STANDARD_STGE:
            /* Stocke en 8 caracteres hex ASCII */
            *pu8Type = LORA_ZCL_TYPE_CHAR_STRING;
            return u8PutAscii(pu8Out, sBaseDevice.sLinkyServerCluster.au8LinkySTGE, 8);

        case E_CLD_LINKY_STANDARD_DPM1:
            *pu8Type = LORA_ZCL_TYPE_UINT8; pu8Out[0] = sBaseDevice.sLinkyServerCluster.u8LinkyDPM1; return 1;
        case E_CLD_LINKY_STANDARD_FPM1:
            *pu8Type = LORA_ZCL_TYPE_UINT8; pu8Out[0] = sBaseDevice.sLinkyServerCluster.u8LinkyFPM1; return 1;
        case E_CLD_LINKY_STANDARD_DPM2:
            *pu8Type = LORA_ZCL_TYPE_UINT8; pu8Out[0] = sBaseDevice.sLinkyServerCluster.u8LinkyDPM2; return 1;
        case E_CLD_LINKY_STANDARD_FPM2:
            *pu8Type = LORA_ZCL_TYPE_UINT8; pu8Out[0] = sBaseDevice.sLinkyServerCluster.u8LinkyFPM2; return 1;
        case E_CLD_LINKY_STANDARD_DPM3:
            *pu8Type = LORA_ZCL_TYPE_UINT8; pu8Out[0] = sBaseDevice.sLinkyServerCluster.u8LinkyDPM3; return 1;
        case E_CLD_LINKY_STANDARD_FPM3:
            *pu8Type = LORA_ZCL_TYPE_UINT8; pu8Out[0] = sBaseDevice.sLinkyServerCluster.u8LinkyFPM3; return 1;

        case E_CLD_LINKY_STANDARD_RELAIS:
            *pu8Type = LORA_ZCL_TYPE_UINT16;
            return u8PutU16(pu8Out, sBaseDevice.sLinkyServerCluster.u16LinkyRELAIS);

        case E_CLD_LINKY_MODE:
            *pu8Type = LORA_ZCL_TYPE_UINT8;
            pu8Out[0] = sBaseDevice.sLinkyServerCluster.au8LinkyMode;
            return 1;

        default:
            return -1;
        }
    }
    else if (u16Cluster == LORA_CLUSTER_METERING)
    {
        /* Index d'energie. Valeur stockee sur 48 bits mais TIC tient sur 32
         * (Wh) : on transmet 4 octets, comme les paquets ENERGY. */
        switch (u16Attr)
        {
        case E_CLD_SM_ATTR_ID_CURRENT_SUMMATION_DELIVERED:   /* 0x0000 EAST/BASE */
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentSummationDelivered);
        case E_CLD_SM_ATTR_ID_CURRENT_SUMMATION_RECEIVED:    /* 0x0001 EAIT */
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentSummationReceived);
        case E_CLD_SM_ATTR_ID_CURRENT_TIER_1_SUMMATION_DELIVERED:       /* 0x0100 */
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier1SummationDelivered);
        case (E_CLD_SM_ATTR_ID_CURRENT_TIER_1_SUMMATION_DELIVERED + 1): /* 0x0101 Tier2 */
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier2SummationDelivered);
        case (E_CLD_SM_ATTR_ID_CURRENT_TIER_1_SUMMATION_DELIVERED + 2): /* 0x0102 Tier3 */
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier3SummationDelivered);
        case (E_CLD_SM_ATTR_ID_CURRENT_TIER_1_SUMMATION_DELIVERED + 3): /* 0x0103 Tier4 */
            *pu8Type = LORA_ZCL_TYPE_UINT32;
            return u8PutU32(pu8Out, (uint32)sBaseDevice.sSimpleMeteringServerCluster.u48CurrentTier4SummationDelivered);
        default:
            return -1;
        }
    }

    return -1;   /* cluster non gere */
}

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

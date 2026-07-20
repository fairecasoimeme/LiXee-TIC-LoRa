/*****************************************************************************
 *
 * MODULE:             app_lora_cmd
 *
 * COMPONENT:          app_lora_cmd.c
 *
 * DESCRIPTION:        Canal de commande descendant (recepteur -> ZLinky).
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
#include "fsl_os_abstraction.h"

#include "app_lora_cmd.h"
#include "app_lora_tic.h"
#include "app_lora_pair.h"
#include "app_lora_crypt.h"
#include "app_lora_channels.h"
#include "app_lora_ota.h"

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

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/* Dernier compteur accepte. Toute commande dont le compteur n'est pas
 * strictement superieur est rejetee : c'est la protection anti-rejeu.
 * Volontairement non persiste : un reboot repart de 0, et le recepteur
 * reprend au compteur qu'il a en base. Persister imposerait une ecriture
 * PDM par commande. */
PRIVATE uint32 u32LastCounter = 0;

/* Config sauvegardee avant application d'une commande, pour pouvoir revenir
 * en arriere si le recepteur ne confirme pas. */
PRIVATE bool_t bPendingConfirm   = FALSE;
PRIVATE uint32 u32ConfirmDeadline = 0;
PRIVATE uint8  u8PrevChannel     = 0;
PRIVATE uint8  u8PrevSF          = 0;
PRIVATE uint8  u8PrevTxInterval  = 0;

/* Sequence des ACK descendants */
PRIVATE uint8 u8AckSeq = 0;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vSendCmdAck(uint8 u8CmdId, uint32 u32Counter, uint8 u8Status, uint8 u8Info);
PRIVATE void vProcessCommand(uint8 *pu8Buf, uint8 u8Len);
PRIVATE void vSnapshotConfig(void);
PRIVATE void vRestoreConfig(void);
PRIVATE void vApplyRadioConfig(void);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void APP_vLoraCmdInit(void)
{
    u32LastCounter   = 0;
    bPendingConfirm  = FALSE;
    u32ConfirmDeadline = 0;
    u8AckSeq         = 0;

    DBG_vPrintf(TRACE_LORA_CMD,
                "LORA_CMD: Init (fenetre RX %d ms, rollback %d s)\r\n",
                LORA_CMD_RX_WINDOW_MS, LORA_CMD_CONFIRM_TIMEOUT_MS / 1000);
}

PUBLIC bool_t APP_bLoraCmdIsPendingConfirm(void)
{
    return bPendingConfirm;
}

/****************************************************************************
 *
 * NAME: APP_vLoraCmdRxWindow
 *
 * DESCRIPTION:
 * Ouvre la fenetre de reception downlink juste apres un TX de donnees.
 *
 * La radio sort d'un TX : DIO1 est mappe sur TX_DONE et payloadLength vaut
 * la taille du dernier paquet emis. Il faut restaurer les deux avant de
 * passer en RX, sinon le paquet est bien recu par le modem mais DIO1 ne se
 * leve jamais — c'est le meme piege que celui rencontre sur l'OTA.
 *
 ****************************************************************************/
PUBLIC void APP_vLoraCmdRxWindow(void)
{
    /* Rien a ecouter tant qu'on n'est pas appaire */
    if (!APP_bLoraPairIsPaired())
        return;

    g_u8SX1280_HalSilent = 1;

    /* 1) payloadLength = 255 : en header explicite la taille reelle vient du
     *    header, mais ce champ sert de borne sur certains SX1280. */
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

    /* 3) RX avec timeout materiel, base 1 ms */
    SX1280_SetRx(LORA_CMD_RX_WINDOW_MS);

    /* 4) Attente DIO1, bornee par le compteur de cycles DWT.
     *    ZTIMER_vTask() garde la LED et le scan bouton vivants. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32 u32CycPerMs  = SystemCoreClock / 1000;
    if (u32CycPerMs == 0) u32CycPerMs = 12000;   /* garde-fou */
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
        /* Cas nominal : aucune commande en attente cote recepteur */
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

    /* 5) Lecture bornee : u8Len vient du header radio, donc d'une source
     *    non fiable. On refuse tout ce qui deborde du buffer. */
    uint8 au8RxBuf[LORA_CMD_MAX_SIZE + LORA_CRYPT_MIC_SIZE];
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
        /* Trame d'un autre emetteur, ou alteree : on jette sans repondre */
        DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: MIC invalide, ignore\r\n");
        return;
    }

    vProcessCommand(au8RxBuf, u8Len - LORA_CRYPT_MIC_SIZE);
}

/****************************************************************************
 *
 * NAME: APP_vLoraCmdPollRollback
 *
 * DESCRIPTION:
 * Restaure la config precedente si le recepteur n'a pas confirme dans les
 * temps. Sans ce filet, une commande de changement de canal mal recue ou
 * forgee rendrait le module injoignable jusqu'a intervention physique.
 *
 ****************************************************************************/
PUBLIC void APP_vLoraCmdPollRollback(void)
{
    if (!bPendingConfirm)
        return;

    /* Comparaison en arithmetique signee pour rester correcte au wrap
     * du compteur de millisecondes. */
    if ((int32)(OSA_TimeGetMsec() - u32ConfirmDeadline) < 0)
        return;

    DBG_vPrintf(TRACE_LORA_CMD,
                "LORA_CMD: pas de CONFIRM en %d s -> retour a la config precedente\r\n",
                LORA_CMD_CONFIRM_TIMEOUT_MS / 1000);

    vRestoreConfig();
    bPendingConfirm = FALSE;
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/* Ne sauvegarde QUE si aucune config n'est deja en attente de confirmation.
 *
 * Sans cette garde, deux commandes successives avant confirmation ecraseraient
 * la sauvegarde : un SET_CHANNEL(7) puis un SET_CHANNEL(2) memoriserait 7
 * comme "config precedente" au lieu du canal d'origine, et le retour arriere
 * ramenerait sur un canal que le recepteur n'ecoute pas forcement. On garde
 * donc toujours la derniere config CONFIRMEE comme point de retour. */
PRIVATE void vSnapshotConfig(void)
{
    if (bPendingConfirm)
        return;

    u8PrevChannel    = APP_u8LoraPairGetOpChannel();
    u8PrevSF         = APP_u8LoraPairGetSF();
    u8PrevTxInterval = APP_u8LoraPairGetTxIntervalSec();
}

PRIVATE void vRestoreConfig(void)
{
    APP_bLoraPairSetOpChannel(u8PrevChannel);
    APP_bLoraPairSetSF(u8PrevSF);
    APP_bLoraPairSetTxIntervalSec(u8PrevTxInterval);
    vApplyRadioConfig();

    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: config restauree (ch=%d SF=%d int=%ds)\r\n",
                u8PrevChannel, u8PrevSF, u8PrevTxInterval);
}

/* Rend la config effective cote radio. Le canal et le SF sont relus par
 * APP_vLoRaTicReconfigureRadio() au TX suivant ; l'intervalle est relu par
 * le timer TX. Rien a faire ici au-dela de la trace. */
PRIVATE void vApplyRadioConfig(void)
{
    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: config active ch=%d SF=%d int=%ds\r\n",
                APP_u8LoraPairGetOpChannel(),
                APP_u8LoraPairGetSF(),
                APP_u8LoraPairGetTxIntervalSec());
}

/****************************************************************************
 *
 * NAME: vSendCmdAck
 *
 * DESCRIPTION:
 * Emet un DOWNLINK_ACK (9 octets + MIC) sur la config COURANTE. Apres un
 * changement de canal ou de SF, l'ACK part donc sur la nouvelle config :
 * c'est ce qui permet au recepteur de verifier qu'il entend toujours le
 * module avant d'envoyer CONFIRM.
 *
 ****************************************************************************/
PRIVATE void vSendCmdAck(uint8 u8CmdId, uint32 u32Counter, uint8 u8Status, uint8 u8Info)
{
    uint8 au8Pkt[LORA_CMD_ACK_SIZE + LORA_CRYPT_MIC_SIZE];

    au8Pkt[0] = (LORA_TIC_PROTOCOL_VERSION_CMD << 4) | LORA_TIC_PKT_DOWNLINK_ACK;
    au8Pkt[1] = u8AckSeq++;
    au8Pkt[2] = u8CmdId;
    au8Pkt[3] = (uint8)(u32Counter >> 24);
    au8Pkt[4] = (uint8)(u32Counter >> 16);
    au8Pkt[5] = (uint8)(u32Counter >> 8);
    au8Pkt[6] = (uint8)(u32Counter & 0xFF);
    au8Pkt[7] = u8Status;
    au8Pkt[8] = u8Info;

    uint8 u8TxLen = APP_u8LoraCryptEncrypt(au8Pkt, LORA_CMD_ACK_SIZE,
                                            APP_pu8LoraPairGetKey(),
                                            APP_pu8LoraPairGetMAC(),
                                            au8Pkt[1]);

    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: TX ACK cmd=0x%02X status=0x%02X info=%d\r\n",
                u8CmdId, u8Status, u8Info);

    APP_bLoraTicTransmitRaw(au8Pkt, u8TxLen);
}

/****************************************************************************
 *
 * NAME: vProcessCommand
 *
 * DESCRIPTION:
 * Valide puis execute une commande dechiffree.
 *
 * Format : [0]=type [1]=seq [2]=cmd_id [3..6]=compteur [7..]=params
 *
 ****************************************************************************/
PRIVATE void vProcessCommand(uint8 *pu8Buf, uint8 u8Len)
{
    if (u8Len < LORA_CMD_HDR_SIZE)
    {
        DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: trame trop courte (%d)\r\n", u8Len);
        return;
    }

    uint8 u8Type = pu8Buf[0] & 0x0F;
    if (u8Type != LORA_TIC_PKT_DOWNLINK_CMD)
    {
        /* Pas une commande : trame d'un autre role, on ignore */
        return;
    }

    uint8  u8CmdId   = pu8Buf[2];
    uint32 u32Counter = ((uint32)pu8Buf[3] << 24) | ((uint32)pu8Buf[4] << 16) |
                        ((uint32)pu8Buf[5] << 8)  |  (uint32)pu8Buf[6];

    DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: RX cmd=0x%02X counter=%lu\r\n",
                u8CmdId, u32Counter);

    /* Anti-rejeu : le compteur doit progresser strictement. */
    if (u32Counter <= u32LastCounter)
    {
        DBG_vPrintf(TRACE_LORA_CMD,
                    "LORA_CMD: rejeu detecte (counter=%lu <= %lu), rejete\r\n",
                    u32Counter, u32LastCounter);
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_COUNTER, 0);
        return;
    }
    u32LastCounter = u32Counter;

    switch (u8CmdId)
    {
    case LORA_CMD_PING:
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_OK,
                    APP_u8LoraPairGetOpChannel());
        break;

    case LORA_CMD_SET_CHANNEL:
    {
        if (u8Len < LORA_CMD_HDR_SIZE + 1)
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_PARAM, 0);
            break;
        }
        uint8 u8NewCh = pu8Buf[7];

        vSnapshotConfig();
        if (!APP_bLoraPairSetOpChannel(u8NewCh))
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_PARAM,
                        APP_u8LoraPairGetOpChannel());
            break;
        }

        bPendingConfirm    = TRUE;
        u32ConfirmDeadline = OSA_TimeGetMsec() + LORA_CMD_CONFIRM_TIMEOUT_MS;
        vApplyRadioConfig();

        /* L'ACK part sur le NOUVEAU canal : si le recepteur l'entend, il
         * confirme ; sinon le rollback rendra la main sur l'ancien. */
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_PENDING_CONFIRM, u8NewCh);
        break;
    }

    case LORA_CMD_SET_SF:
    {
        if (u8Len < LORA_CMD_HDR_SIZE + 1)
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_PARAM, 0);
            break;
        }
        uint8 u8NewSF = pu8Buf[7];

        vSnapshotConfig();
        if (!APP_bLoraPairSetSF(u8NewSF))
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_PARAM,
                        APP_u8LoraPairGetSF());
            break;
        }

        bPendingConfirm    = TRUE;
        u32ConfirmDeadline = OSA_TimeGetMsec() + LORA_CMD_CONFIRM_TIMEOUT_MS;
        vApplyRadioConfig();
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_PENDING_CONFIRM, u8NewSF);
        break;
    }

    case LORA_CMD_SET_TX_INTERVAL:
    {
        if (u8Len < LORA_CMD_HDR_SIZE + 2)
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_PARAM, 0);
            break;
        }
        uint16 u16Sec = ((uint16)pu8Buf[7] << 8) | pu8Buf[8];

        vSnapshotConfig();
        if (u16Sec > 255 || !APP_bLoraPairSetTxIntervalSec((uint8)u16Sec))
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_BAD_PARAM,
                        APP_u8LoraPairGetTxIntervalSec());
            break;
        }

        bPendingConfirm    = TRUE;
        u32ConfirmDeadline = OSA_TimeGetMsec() + LORA_CMD_CONFIRM_TIMEOUT_MS;
        vApplyRadioConfig();
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_PENDING_CONFIRM,
                    (uint8)u16Sec);
        break;
    }

    case LORA_CMD_CONFIRM:
        if (!bPendingConfirm)
        {
            vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_NOTHING_PENDING, 0);
            break;
        }
        bPendingConfirm = FALSE;
        APP_vLoraPairSaveConfig();
        DBG_vPrintf(TRACE_LORA_CMD,
                    "LORA_CMD: config confirmee et persistee (ch=%d SF=%d int=%ds)\r\n",
                    APP_u8LoraPairGetOpChannel(),
                    APP_u8LoraPairGetSF(),
                    APP_u8LoraPairGetTxIntervalSec());
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_OK,
                    APP_u8LoraPairGetOpChannel());
        break;

    case LORA_CMD_ENTER_OTA:
        /* Pas de rollback ici : le mode OTA a deja son propre timeout, et
         * un retour arriere en plein transfert n'aurait pas de sens. */
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_OK, 0);
        DBG_vPrintf(TRACE_LORA_CMD, "LORA_CMD: passage en mode OTA sur commande\r\n");
        APP_vLoraOtaRunListenLoop();
        /* ne revient pas : reboot ou timeout */
        break;

    default:
        vSendCmdAck(u8CmdId, u32Counter, LORA_CMD_STATUS_UNKNOWN_CMD, 0);
        break;
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

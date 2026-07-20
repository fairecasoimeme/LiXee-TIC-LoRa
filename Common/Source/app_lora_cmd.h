/*****************************************************************************
 *
 * MODULE:             app_lora_cmd
 *
 * COMPONENT:          app_lora_cmd.h
 *
 * DESCRIPTION:        Canal de commande descendant (recepteur -> ZLinky).
 *
 *                     Modele Class A facon LoRaWAN : le ZLinky ouvre une
 *                     courte fenetre de reception juste apres chaque TX de
 *                     donnees. Le recepteur, qui vient de recevoir l'uplink,
 *                     y depose une commande en attente.
 *
 *                     Cout : ~300 ms de RX par cycle de ~7 s, soit ~1,5 mW
 *                     en moyenne. Une ecoute permanente couterait ~30 mW,
 *                     soit pres du quart du budget TIC (130 mW).
 *
 *                     Deux protections, cf. LORA_PROTOCOL.md :
 *                       - compteur monotone contre le rejeu
 *                       - retour arriere automatique a 60 s sur les commandes
 *                         qui touchent a la liaison radio (canal, SF,
 *                         intervalle) : sans confirmation du recepteur sur la
 *                         nouvelle config, l'ancienne est restauree. Protege
 *                         aussi bien de l'erreur que de l'attaque, un mauvais
 *                         canal rendant le module injoignable.
 *
 ****************************************************************************/

#ifndef APP_LORA_CMD_H_
#define APP_LORA_CMD_H_

#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Types de paquets (byte[0] bits[3:0]) */
#define LORA_TIC_PKT_DOWNLINK_CMD   0x0B  /* recepteur -> ZLinky */
#define LORA_TIC_PKT_DOWNLINK_ACK   0x0C  /* ZLinky -> recepteur */

/* Identifiants de commande (DOWNLINK_CMD byte[2]) */
#define LORA_CMD_PING               0x00  /* test de liaison, sans effet */
#define LORA_CMD_SET_CHANNEL        0x01  /* param : canal 0..7 */
#define LORA_CMD_SET_SF             0x02  /* param : SF 7..12 */
#define LORA_CMD_SET_TX_INTERVAL    0x03  /* param : secondes (uint16 BE) */
#define LORA_CMD_ENTER_OTA          0x04  /* bascule en mode OTA recovery */
#define LORA_CMD_CONFIRM            0x05  /* valide la config en attente */

/* Codes de statut (DOWNLINK_ACK byte[7]) */
#define LORA_CMD_STATUS_OK              0x00
#define LORA_CMD_STATUS_BAD_COUNTER     0x01  /* rejeu : compteur <= dernier vu */
#define LORA_CMD_STATUS_BAD_PARAM       0x02  /* valeur hors bornes */
#define LORA_CMD_STATUS_UNKNOWN_CMD     0x03
#define LORA_CMD_STATUS_PENDING_CONFIRM 0x04  /* applique, attend CONFIRM */
#define LORA_CMD_STATUS_NOTHING_PENDING 0x05  /* CONFIRM sans config en attente */

/* Tailles */
#define LORA_CMD_HDR_SIZE           7   /* type+seq+cmd_id+counter(4) */
#define LORA_CMD_MAX_SIZE           16  /* header + params, hors MIC */
#define LORA_CMD_ACK_SIZE           9   /* type+seq+cmd_id+counter(4)+status+info */

/* Fenetre de reception ouverte apres chaque TX de donnees */
#define LORA_CMD_RX_WINDOW_MS       300

/* Delai de retour arriere si le recepteur ne confirme pas */
#define LORA_CMD_CONFIRM_TIMEOUT_MS 60000

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Init du module. A appeler depuis APP_vInitialiseRouter().
 */
PUBLIC void APP_vLoraCmdInit(void);

/**
 * @brief Ouvre la fenetre de reception downlink.
 *        A appeler juste apres un TX de donnees, la radio etant encore
 *        configuree et en standby.
 *
 *        Bloquant pendant LORA_CMD_RX_WINDOW_MS au maximum ; sort des la
 *        reception d'un paquet. ZTIMER_vTask() est appele pendant l'attente
 *        pour ne pas geler la LED ni le scan bouton.
 */
PUBLIC void APP_vLoraCmdRxWindow(void);

/**
 * @brief Verifie l'echeance de retour arriere. A appeler regulierement
 *        depuis la boucle principale.
 */
PUBLIC void APP_vLoraCmdPollRollback(void);

/**
 * @brief TRUE si une config est appliquee mais pas encore confirmee.
 */
PUBLIC bool_t APP_bLoraCmdIsPendingConfirm(void);

#endif /* APP_LORA_CMD_H_ */

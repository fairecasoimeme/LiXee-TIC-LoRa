/*****************************************************************************
 *
 * MODULE:             app_lora_cmd
 *
 * COMPONENT:          app_lora_cmd.h
 *
 * DESCRIPTION:        Canal descendant (recepteur -> ZLinky) : lecture
 *                     d'attribut a la demande, facon ZCL Read Attributes.
 *
 *                     Modele Class A facon LoRaWAN : le ZLinky ouvre une
 *                     courte fenetre de reception juste apres chaque TX de
 *                     donnees. Le recepteur, qui vient de recevoir l'uplink,
 *                     y depose une requete de lecture.
 *
 *                     Le descendant ne sert QU'A interroger un attribut. Il
 *                     ne modifie aucun etat : le SF et le canal se negocient
 *                     a l'appairage (PAIR_RESPONSE), pas ici. Une lecture
 *                     etant idempotente, il n'y a ni compteur anti-rejeu ni
 *                     retour arriere a gerer — un rejeu ne fait que relire la
 *                     valeur courante.
 *
 *                     Cout : ~300 ms de RX par cycle de ~7 s, soit ~1,5 mW
 *                     en moyenne. Une ecoute permanente couterait ~30 mW,
 *                     soit pres du quart du budget TIC (130 mW).
 *
 ****************************************************************************/

#ifndef APP_LORA_CMD_H_
#define APP_LORA_CMD_H_

#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Types de paquets (byte[0] bits[3:0]) */
#define LORA_TIC_PKT_POLL_REQUEST   0x0B  /* recepteur -> ZLinky */
#define LORA_TIC_PKT_POLL_RESPONSE  0x0C  /* ZLinky -> recepteur */

/* Codes de statut (POLL_RESPONSE byte[6]) */
#define LORA_POLL_STATUS_OK             0x00
#define LORA_POLL_STATUS_UNKNOWN_ATTR   0x01  /* cluster/attribut non gere */
#define LORA_POLL_STATUS_BAD_REQUEST    0x02  /* trame malformee */

/* Types de donnees ZCL utilises dans les reponses (sous-ensemble) */
#define LORA_ZCL_TYPE_UINT8         0x20
#define LORA_ZCL_TYPE_UINT16        0x21
#define LORA_ZCL_TYPE_UINT32        0x23
#define LORA_ZCL_TYPE_UINT48        0x25
#define LORA_ZCL_TYPE_INT8          0x28
#define LORA_ZCL_TYPE_INT16         0x29
#define LORA_ZCL_TYPE_INT32         0x2B
#define LORA_ZCL_TYPE_OCTET_STRING  0x41
#define LORA_ZCL_TYPE_CHAR_STRING   0x42

/* Cluster IDs adressables — memes identifiants que le firmware Zigbee LiXee */
#define LORA_CLUSTER_LIXEE          0xFF66  /* cluster custom LiXee (TIC) */
#define LORA_CLUSTER_METERING       0x0702  /* Simple Metering (index energie) */
#define LORA_CLUSTER_ELEC_MEAS      0x0B04  /* Electrical Measurement (puissances) */
#define LORA_CLUSTER_METER_ID       0x0B01  /* Meter Identification (PREF/PCOUP/VTIC) */

/* Tailles */
#define LORA_POLL_REQUEST_SIZE      6   /* type+seq+cluster(2)+attr(2) */
#define LORA_POLL_RESP_HDR_SIZE     9   /* type+seq+cluster(2)+attr(2)+status+zcl_type+len */
#define LORA_POLL_VALUE_MAX         32  /* plus grande valeur transmise (MSG1 = 32 car.) */
#define LORA_POLL_RESP_MAX_SIZE     (LORA_POLL_RESP_HDR_SIZE + LORA_POLL_VALUE_MAX)

/* Fenetre de reception ouverte apres chaque TX de donnees */
#define LORA_CMD_RX_WINDOW_MS       300

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Init du module. A appeler depuis APP_vInitialiseRouter().
 */
PUBLIC void APP_vLoraCmdInit(void);

/**
 * @brief Ouvre la fenetre de reception d'une requete de lecture.
 *        A appeler juste apres un TX de donnees, la radio etant encore
 *        configuree et en standby.
 *
 *        Bloquant pendant LORA_CMD_RX_WINDOW_MS au maximum ; sort des la
 *        reception d'un paquet. ZTIMER_vTask() est appele pendant l'attente
 *        pour ne pas geler la LED ni le scan bouton.
 */
PUBLIC void APP_vLoraCmdRxWindow(void);

#endif /* APP_LORA_CMD_H_ */

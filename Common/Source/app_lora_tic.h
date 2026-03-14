/*****************************************************************************
 *
 * MODULE:             app_lora_tic
 *
 * COMPONENT:          app_lora_tic.h
 *
 * DESCRIPTION:        LoRa TIC protocol — periodic TX of Linky meter data
 *                     via SX1280 2.4GHz LoRa
 *
 *                     Uniquement des etiquettes TIC brutes (pas de valeurs
 *                     calculees). Max 20 octets par paquet (SF10).
 *
 ****************************************************************************/

#ifndef APP_LORA_TIC_H_
#define APP_LORA_TIC_H_

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Intervalle TX LoRa en ms (1 paquet ESSENTIAL par seconde) */
#define LORA_TX_INTERVAL_MS         1000

/* Nombre d'ESSENTIAL entre chaque EXTENDED piggyback */
#define LORA_TIC_EXTENDED_INTERVAL  10

/* Version du protocole LoRa TIC */
#define LORA_TIC_PROTOCOL_VERSION   0x01

/* Types de paquets */
#define LORA_TIC_PKT_DATA_ESSENTIAL 0x01
#define LORA_TIC_PKT_DATA_EXTENDED  0x02

/* Taille max payload (contrainte SF10 fiabilite) */
#define LORA_TIC_MAX_PAYLOAD        20

/* Taille du paquet DATA_ESSENTIAL (17 octets) */
#define LORA_TIC_DATA_ESSENTIAL_SIZE 17

/* Sous-types DATA_EXTENDED (7 sous-types, cycle complet ~70s) */
#define LORA_TIC_EXT_CURRENT_VOLTAGE  0x00  /* IRMS1/2/3 + URMS1/2/3 (15B) */
#define LORA_TIC_EXT_ENERGY           0x01  /* EAST + EAIT + EASF01 + EASF02 (19B) */
#define LORA_TIC_EXT_ENERGY_2         0x02  /* EASF03 + EASF04 + EASF05 + EASF06 (19B) */
#define LORA_TIC_EXT_VOLTAGE_STATS    0x03  /* UMOY1/2/3 + IMAX1/2/3 (15B) */
#define LORA_TIC_EXT_POWER_MAX_CFG    0x04  /* SMAXSN/IN + SINSTI + CCAIN + config (18B) */
#define LORA_TIC_EXT_DAILY_ENERGY     0x05  /* EASD01/02/03/04 (19B) */
#define LORA_TIC_EXT_COMPLEMENT       0x06  /* DPM/FPM 1-3 (9B) */
#define LORA_TIC_EXT_SUBTYPE_COUNT    7

/* Taille max d'un paquet DATA_EXTENDED (ENERGY = 19B) */
#define LORA_TIC_DATA_EXTENDED_MAX_SIZE 19

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/*
 * Format paquet DATA_ESSENTIAL (17 octets) :
 * Etiquettes TIC temps-reel : puissances apparentes + statut
 *
 * [0]     version_type   : bits[7:4]=version(1), bits[3:0]=type(1=essential)
 * [1]     sequence       : compteur 0-255 (wrap around)
 * [2-3]   SINSTS         : puissance apparente totale VA (uint16 BE)
 *                          = PAPP en mode Historique
 * [4-5]   SINSTS1        : puissance apparente phase 1 VA (uint16 BE)
 *                          = SINSTS/PAPP en monophase
 * [6-7]   SINSTS2        : puissance apparente phase 2 VA (uint16 BE)
 *                          = 0 en monophase
 * [8-9]   SINSTS3        : puissance apparente phase 3 VA (uint16 BE)
 *                          = 0 en monophase
 * [10-13] STGE           : registre statut 32 bits (binaire, converti hex)
 *                          = 0 en mode Historique
 * [14-15] ADPS           : alerte depassement puissance VA (uint16 BE)
 *                          = 0 en mode Standard
 * [16]    mode           : au8LinkyMode (0=Histo, 1=Std, 2=Tri)
 */

/*
 * Format paquet DATA_EXTENDED (taille variable, max 19B) :
 *
 * [0]     version_type   : bits[7:4]=version(1), bits[3:0]=type(2=extended)
 * [1]     sequence       : compteur 0-255 (partage avec essential)
 * [2]     sub_type       : identifiant du sous-type (0x00-0x05)
 * [3+]    payload        : variable selon sub_type
 *
 * Sub-type 0x00 CURRENT_VOLTAGE (15B total) :
 *   [3-4]   IRMS1 mA, [5-6] IRMS2, [7-8] IRMS3
 *   [9-10]  URMS1 V, [11-12] URMS2, [13-14] URMS3
 *
 * Sub-type 0x01 ENERGY (19B total) :
 *   [3-6] EAST Wh, [7-10] EAIT Wh, [11-14] EASF01 Wh, [15-18] EASF02 Wh
 *
 * Sub-type 0x02 ENERGY_2 (19B total) :
 *   [3-6] EASF03 Wh, [7-10] EASF04 Wh, [11-14] EASF05 Wh, [15-18] EASF06 Wh
 *
 * Sub-type 0x03 VOLTAGE_STATS (15B total) :
 *   [3-4] UMOY1 V, [5-6] UMOY2, [7-8] UMOY3
 *   [9-10] IMAX1 mA, [11-12] IMAX2, [13-14] IMAX3
 *
 * Sub-type 0x04 POWER_MAX_CFG (18B total) :
 *   [3-4] SMAXSN VA, [5-6] SMAXIN VA, [7-8] SINSTI VA
 *   [9-10] CCAIN VA, [11-12] RELAIS
 *   [13] NTARF, [14] PCOUP kVA, [15] PREF kVA, [16] mode, [17] reserved
 *
 * Sub-type 0x05 DAILY_ENERGY (19B total) :
 *   [3-6] EASD01 Wh, [7-10] EASD02 Wh, [11-14] EASD03 Wh, [15-18] EASD04 Wh
 *
 * Sub-type 0x06 COMPLEMENT (9B total) :
 *   [3] DPM1, [4] FPM1, [5] DPM2, [6] FPM2, [7] DPM3, [8] FPM3
 */

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Timer callback LoRa — poste APP_E_EVENT_LORA_TX dans la queue
 *        Enregistre comme callback ZTIMER dans app_main.c
 */
PUBLIC void APP_cbTimerLoRa(void *pvParam);

/**
 * @brief Construit le paquet DATA_ESSENTIAL et envoie via SX1280.
 *        Toutes les 10 TX, ajoute un DATA_EXTENDED en piggyback.
 *        Appele depuis APP_taskRouter sur event APP_E_EVENT_LORA_TX
 */
PUBLIC void APP_vLoRaTicSendPacket(void);

/**
 * @brief Demarre le timer periodique LoRa TX
 *        Appele depuis APP_vInitialiseRouter apres init TIC
 */
PUBLIC void APP_vLoRaTicStart(void);

/**
 * @brief Signale si le SX1280 est operationnel (SPI OK au boot)
 *        Positionne par app_start.c apres le test SPI
 */
PUBLIC void APP_vLoRaTicSetReady(bool_t bReady);

/**
 * @brief Verifie si un TX est en attente et l'execute.
 *        Doit etre appele dans la boucle main APRES ZTIMER_vTask()
 *        pour ne pas bloquer les autres timers pendant l'acquisition TIC.
 */
PUBLIC void APP_vLoRaTicPollTx(void);

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* APP_LORA_TIC_H_ */

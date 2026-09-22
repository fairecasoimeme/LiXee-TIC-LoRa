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
 *                     calculees). Max 22 octets par paquet.
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

/* Intervalle TX LoRa en ms (1 paquet ESSENTIAL toutes les 5 secondes) */
#define LORA_TX_INTERVAL_MS         5000

/* Nombre d'ESSENTIAL entre chaque EXTENDED piggyback
 * Avec 1 essential/5s x 4 = 1 extended toutes les 20s
 * Cycle complet des 11 sous-types extended : ~220s (~3 min 40) */
#define LORA_TIC_EXTENDED_INTERVAL  4

/* Puissance d'emission en dBm.
 * +10 dBm = limite reglementaire ETSI EU (EN 300 440) pour la bande
 * 2.4 GHz ISM. Le SX1280 accepte jusqu'a +13 dBm mais ce serait hors
 * conformite pour une commercialisation en Europe. */
/* +10 dBm : limite ETSI EU (EN 300 440) pour la bande 2.4 GHz ISM.
 * NB : un test a 0 dBm (pic PA ~27 mA au lieu de ~45 mA) n'a PAS
 * supprime les echecs TX -- ils etaient meme plus frequents. La cause
 * de ces echecs n'est donc pas le courant consomme par le PA. */
#define LORA_TIC_TX_POWER_DBM       10

/* Version du protocole LoRa TIC */
#define LORA_TIC_PROTOCOL_VERSION   0x01

/* Types de paquets */
#define LORA_TIC_PKT_DATA_ESSENTIAL 0x01
#define LORA_TIC_PKT_DATA_EXTENDED  0x02

/* Taille max payload.
 * 22 = TARIFF_LABEL avec un libelle de 16 caracteres (6 + 16).
 * A SF11/BW406 l'airtime reste ~10 ms de plus qu'un paquet de 19 : negligeable. */
#define LORA_TIC_MAX_PAYLOAD        22

/* Taille du paquet DATA_ESSENTIAL (17 octets) */
#define LORA_TIC_DATA_ESSENTIAL_SIZE 17

/* Codes tarif unifie (POWER_MAX_CFG octet [17], ex-reserved)
 *
 * La signification depend du mode Linky (POWER_MAX_CFG octet [16]) :
 *
 * Mode Standard (bit 0 de au8LinkyMode == 1) :
 *   redondant avec NTARF en [13] pour compat, mais reprend :
 *   0     = inconnu / pas encore renseigne
 *   1..10 = NTARF direct (index du tarif en cours)
 *
 * Mode Historique (bit 0 == 0) — NTARF est 0 dans ce mode :
 *   0  = inconnu
 *   1  = BASE (PTEC "TH.." ou "BASE")
 *   2  = HP (PTEC "HP..")
 *   3  = HC (PTEC "HC..")
 *   4  = EJP heures normales (PTEC "HN..")
 *   5  = EJP pointe mobile (PTEC "PM..")
 *   6  = TEMPO Bleu HC (PTEC "HCJB")
 *   7  = TEMPO Bleu HP (PTEC "HPJB")
 *   8  = TEMPO Blanc HC (PTEC "HCJW")
 *   9  = TEMPO Blanc HP (PTEC "HPJW")
 *   10 = TEMPO Rouge HC (PTEC "HCJR")
 *   11 = TEMPO Rouge HP (PTEC "HPJR")
 */
#define LORA_TARIF_UNKNOWN               0
#define LORA_TARIF_HISTO_BASE            1
#define LORA_TARIF_HISTO_HP              2
#define LORA_TARIF_HISTO_HC              3
#define LORA_TARIF_HISTO_EJP_HN          4
#define LORA_TARIF_HISTO_EJP_PM          5
#define LORA_TARIF_HISTO_TEMPO_BLEU_HC   6
#define LORA_TARIF_HISTO_TEMPO_BLEU_HP   7
#define LORA_TARIF_HISTO_TEMPO_BLANC_HC  8
#define LORA_TARIF_HISTO_TEMPO_BLANC_HP  9
#define LORA_TARIF_HISTO_TEMPO_ROUGE_HC  10
#define LORA_TARIF_HISTO_TEMPO_ROUGE_HP  11

/* Sous-types DATA_EXTENDED (11 sous-types, cycle complet ~220s) */
#define LORA_TIC_EXT_CURRENT_VOLTAGE  0x00  /* IRMS1/2/3 + URMS1/2/3 (15B) */
#define LORA_TIC_EXT_ENERGY           0x01  /* EAST + EAIT + EASF01 + EASF02 (19B) */
#define LORA_TIC_EXT_ENERGY_2         0x02  /* EASF03 + EASF04 + EASF05 + EASF06 (19B) */
#define LORA_TIC_EXT_VOLTAGE_STATS    0x03  /* UMOY1/2/3 + IMAX1/2/3 (15B) */
#define LORA_TIC_EXT_POWER_MAX_CFG    0x04  /* SMAXSN/IN + SINSTI + CCAIN + config (18B) */
#define LORA_TIC_EXT_DAILY_ENERGY     0x05  /* EASD01/02/03/04 (19B) */
#define LORA_TIC_EXT_COMPLEMENT       0x06  /* DPM/FPM 1-3 (9B) */
#define LORA_TIC_EXT_TARIFF_LABEL     0x07  /* LTARF (Std) / PTEC (Histo) en ASCII (6..22B) */
#define LORA_TIC_EXT_METER_SERIAL     0x08  /* ADSC (Std) / ADCO (Histo) en ASCII (4..16B) */
#define LORA_TIC_EXT_METER_DATE       0x09  /* DATE horodate compteur, Std seul (4..17B) */
#define LORA_TIC_EXT_TARIFF_OPTION    0x0A  /* OPTARIF + DEMAIN (Histo) + CCASN/-1 (Std) (16B) */
#define LORA_TIC_EXT_HISTO_ALERTS     0x0B  /* PEJP + PPOT + ADIR1/2/3 (Histo) (12B) */
#define LORA_TIC_EXT_SUBTYPE_COUNT    12

/* Taille max d'un paquet DATA_EXTENDED (TARIFF_LABEL = 6 + 16 = 22B) */
#define LORA_TIC_DATA_EXTENDED_MAX_SIZE 22

/* TARIFF_LABEL : longueur max du libelle ASCII transmis.
 * LTARF fait 16 caracteres en TIC Standard, PTEC 4 en Historique. */
#define LORA_TIC_TARIFF_LABEL_MAX_LEN 16
#define LORA_TIC_EXT_TARIFF_HDR_SIZE  6   /* type+seq+subtype+mode+tarif+len */

/* METER_SERIAL : numero de serie du compteur, 12 caracteres ASCII.
 * ADSC (Standard) et ADCO (Historique) sont identiques en longueur et
 * partagent le meme champ interne → pas de champ mode necessaire. */
#define LORA_TIC_SERIAL_MAX_LEN       12
#define LORA_TIC_EXT_SERIAL_HDR_SIZE  4   /* type+seq+subtype+len */

/* METER_DATE : horodate du compteur, format TIC "SAAMMJJhhmmss" (13 chars).
 *   S    = saison : 'E' ete / 'H' hiver ; minuscule ('e'/'h') = horloge en
 *          mode degrade (non synchronisee) ; espace = inconnue
 *   AA   = annee (2 chiffres), MM mois, JJ jour, hh:mm:ss heure
 * Etiquette presente en TIC Standard uniquement (absente en Historique). */
#define LORA_TIC_DATE_MAX_LEN         13
#define LORA_TIC_EXT_DATE_HDR_SIZE    4   /* type+seq+subtype+len */

/* TARIFF_OPTION : longueurs des champs ASCII a disposition fixe.
 *   OPTARIF (Historique) : "BASE", "HC..", "EJP.", "BBR(" — 4 chars
 *   DEMAIN  (Histo Tempo): "BLEU", "BLAN", "ROUG", "----" — 4 chars */
#define LORA_TIC_OPTARIF_LEN          4
#define LORA_TIC_DEMAIN_LEN           4

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
 * Format paquet DATA_EXTENDED (taille variable, max 22B) :
 *
 * [0]     version_type   : bits[7:4]=version(1), bits[3:0]=type(2=extended)
 * [1]     sequence       : compteur 0-255 (partage avec essential)
 * [2]     sub_type       : identifiant du sous-type (0x00-0x0A)
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

/**
 * @brief Transmet un paquet brut LoRa (deja construit) et attend TX_DONE.
 *        N'effectue PAS de chiffrement AES, le buffer est envoye tel quel.
 *        Reutilise par le module OTA pour envoyer ACK/STATUS apres
 *        chiffrement par AES-CTR.
 * @param pu8Data  Buffer prêt à envoyer (avec MIC si chiffré)
 * @param u8Len    Taille totale en bytes
 * @return TRUE si TX_DONE detecte, FALSE sinon
 */
PUBLIC bool_t APP_bLoraTicTransmitRaw(uint8 *pu8Data, uint8 u8Len);

/**
 * @brief Reset materiel du SX1280 + reconfiguration LoRa complete sur le
 *        canal donne (packet type, modulation SF11/BW406, sync word,
 *        packet params, TX power, IRQ TX_DONE).
 *
 *        A appeler par tout module qui doit emettre : la radio est alors
 *        dans un etat connu, sans dependre de la configuration laissee par
 *        un autre module.
 *
 * @param u8Channel  Index de canal 0..LORA_NUM_CHANNELS-1
 */
PUBLIC void APP_vLoRaTicReconfigureRadio(uint8 u8Channel, uint8 u8SF);

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* APP_LORA_TIC_H_ */

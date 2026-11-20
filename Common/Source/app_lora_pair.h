/*****************************************************************************
 *
 * MODULE:             app_lora_pair
 *
 * COMPONENT:          app_lora_pair.h
 *
 * DESCRIPTION:        LoRa pairing protocol — handshake + AES-128 key exchange
 *                     between emitter (JN5189) and receiver (ESP32)
 *
 *                     Protocol :
 *                       1) PAIR_REQUEST  (emitter → receiver) : MAC address
 *                       2) PAIR_RESPONSE (receiver → emitter) : AES-128 key
 *                       3) PAIR_CONFIRM  (emitter → receiver) : proof of key
 *
 *                     After pairing, all DATA packets are encrypted.
 *
 ****************************************************************************/

#ifndef APP_LORA_PAIR_H_
#define APP_LORA_PAIR_H_

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "app_events.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Pairing packet types (byte[0] bits[3:0]) */
#define LORA_TIC_PKT_PAIR_REQUEST      0x03
#define LORA_TIC_PKT_PAIR_RESPONSE     0x04
#define LORA_TIC_PKT_PAIR_CONFIRM      0x05

/* Packet sizes */
#define LORA_PAIR_REQUEST_BASE_SIZE    11  /* type+seq+MAC(8)+channel */
#define LORA_PAIR_REQUEST_SIZE         LORA_PAIR_REQUEST_BASE_SIZE  /* legacy alias */
#define LORA_PAIR_RESPONSE_SIZE        21  /* type+seq+key(16)+status+op_channel+op_SF */
#define LORA_PAIR_RESPONSE_SIZE_LEGACY 20  /* sans op_SF — receivers < v2.23 → SF11 par defaut */
/* PAIR_RESPONSE adressee (protocole v2) : MAC de l'emetteur vise en [21..28].
 * L'emetteur verifie que c'est le sien. Toute longueur >= 21 est acceptee ;
 * la verification du MAC s'applique des que la trame fait au moins 29 octets
 * (les octets suivants sont reserves a une evolution, ex. identifiant de box). */
#define LORA_PAIR_RESPONSE_MAC_OFFSET  21
#define LORA_PAIR_RESPONSE_SIZE_V2     (LORA_PAIR_RESPONSE_MAC_OFFSET + 8)
#define LORA_PAIR_CONFIRM_SIZE         15  /* type+seq+MAC(8)+proof(4)+status */

/* Rendez-vous d'appairage : FIXE, independant de la config operationnelle.
 * Un ZLinky qui tourne en SF7/canal7 doit pouvoir revenir ici pour renegocier.
 * Le canal est LORA_PAIR_CHANNEL (defini dans app_lora_channels.h). */
/* SF du handshake d'appairage.
 *
 * ⚠️ DOIT etre identique cote recepteur : le rendez-vous doit etre connu
 * a l'avance des deux cotes. Changer cette valeur sans changer le
 * recepteur rend l'appairage totalement IMPOSSIBLE (aucune negociation
 * n'est possible a ce stade).
 *
 * Historique des mesures terrain, a +10 dBm :
 *   PAIR_REQUEST SF11 : 45 mA x 247 ms = 11,1 mA.s
 *       -> OK sur Linky monophase, ECHOUE sur triphase (le recepteur ne
 *          recoit rien), les deux compteurs etant au MEME endroit, a la
 *          MEME distance du recepteur
 *   PAIR_REQUEST SF10 : 45 mA x 124 ms =  5,6 mA.s
 *       -> OK sur les deux
 *
 * Le budget d'alimentation de la prise TIC limite la CHARGE par emission
 * (courant x airtime), pas le courant instantane, et ce budget varie
 * selon le compteur : le seuil se situe entre ~7 et ~11 mA.s.
 *
 * SF11 est donc a la limite : il tient sur les installations favorables
 * et lache sur les autres. Pour l'avoir partout il faut de la marge
 * materielle (LDO unique : ~5 mA permanents rendus, soit ~19 % du
 * budget ; et/ou supercap plus gros). */
#define LORA_PAIR_SF                   11

/* Extension PAIR_REQUEST (LiXee-Box >= v2.22) :
 *   Format 14+N octets, retro-compatible avec les receivers antérieurs
 *   qui liront simplement 11 octets et ignoreront la suite.
 *   [11-12] device_id (uint16 BE) — nomme le template data/tp/<device_id>.json
 *   [13]    longueur du model (N)
 *   [14..]  model ASCII (N octets)
 */
/* 1 = envoie l'extension (device_id + model), 0 = format legacy 11 octets.
 *
 * TEST EN COURS (a trancher apres essais) : passe a 0 pour discriminer la
 * cause de l'echec d'appairage sur Linky triphase.
 *
 * Le LoRa quantifie par blocs de symboles, et 24 octets franchissent une
 * marche que 19 octets ne franchissent pas (SF11/BW406) :
 *
 *   PAIR_REQUEST 24 o (avec ext) : 33 symboles, 268 ms -> 12,1 mA.s  ECHOUE
 *   trame donnees  19 o          : 28 symboles, 243 ms -> 10,9 mA.s  passe
 *                                                                    en partie
 *   PAIR_REQUEST 11 o (legacy)   : 18 symboles, 193 ms ->  8,7 mA.s  ?
 *
 * Le seuil du compteur triphase est donc cerne entre 10,9 et 12,1 mA.s.
 * Si le legacy appaire, la cause est bien la charge de la trame et rien
 * d'autre -- et il faudra choisir entre l'identification automatique du
 * modele et SF11 sur ce type d'installation.
 *
 * Retro-compatible : un recepteur lit les 11 premiers octets et ignore la
 * suite, donc passer a 0 ne demande aucune modification cote LiXee-Box.
 * Seule consequence : plus de device_id ni de model, donc plus de
 * selection automatique du template data/tp/<device_id>.json. */
#define LORA_PAIR_REQUEST_HAS_EXT      0
#define LORA_DEVICE_ID_ZLINKY_TIC      0x0051  /* device_id LiXee du ZLinky — template data/tp/0051.json */
#define LORA_DEVICE_MODEL_ZLINKY_TIC   "ZLinky_TIC"  /* cle dans data/tp/0051.json cote LiXee-Box */
#define LORA_DEVICE_MODEL_MAX_LEN      32   /* garde-fou de securite (permet des noms d'objets futurs plus longs) */
#define LORA_PAIR_REQUEST_MAX_SIZE     (LORA_PAIR_REQUEST_BASE_SIZE + 3 + LORA_DEVICE_MODEL_MAX_LEN)

/* PDM record ID for pairing data */
#define PDM_ID_LORA_PAIRING            0xA110

/* Magic value to validate PDM record */
#define LORA_PAIR_MAGIC                0x4C505249  /* "LPRI" */

/* Duree maximale d'une fenetre d'appairage cote emetteur.
 *
 * Un emetteur qui reste indefiniment en appairage finit par accepter la
 * PAIR_RESPONSE de la premiere box qui ouvre son propre appairage, meme si
 * elle repond a un autre emetteur : il prend alors une cle que sa box ne
 * connait pas et semble « desappaire ». La fenetre est donc bornee :
 *   - appui long : 60 s, le temps d'ouvrir l'appairage sur la box ;
 *   - premier demarrage sans cle : 5 min, le temps d'aller jusqu'a la box.
 * A l'expiration, un emetteur deja appaire reprend son appairage precedent
 * (cle, canal, SF) ; un emetteur sans cle repasse non appaire, LED eteinte,
 * et attend un appui long. */
#define LORA_PAIR_WINDOW_BUTTON_MS     60000
#define LORA_PAIR_WINDOW_BOOT_MS       300000

/* Intervalle entre deux PAIR_REQUEST = temps de RECHARGE du supercap
 * avant chaque tentative.
 *
 * Le budget d'alimentation TIC limite la charge par emission (courant x
 * airtime) et, a SF11, on est pile au seuil sur certains compteurs. Le
 * temps de recharge devient alors determinant. Mesures terrain, a
 * charge identique (11,1 mA.s par trame, SF11 / +10 dBm) :
 *
 *   appairage, recharge 2,5 s -> ECHOUE toujours   (triphase)
 *   donnees,   recharge 5,0 s -> passe en partie   (triphase)
 *   appairage/donnees, SF10   -> OK partout
 *
 * D'ou 6000 ms : on donne a chaque PAIR_REQUEST une recharge superieure
 * a celle des donnees, qui passent deja partiellement.
 *
 * Le cout en nombre d'essais reste faible : 10 essais sur la fenetre de
 * 60 s d'un appui long (cf. LORA_PAIR_WINDOW_BUTTON_MS), et l'appairage
 * n'a besoin que d'UN succes, contrairement aux donnees ou chaque echec
 * est une trame perdue. */
#define LORA_PAIR_TX_INTERVAL_MS       6000

/* RX listen window after each PAIR_REQUEST (200ms) */
#define LORA_PAIR_RX_TIMEOUT_MS        200

/* Duree d'emission maximale d'une PAIR_RESPONSE (29 octets a SF11 : ~255 ms).
 * La fenetre ci-dessus borne la DETECTION de la reponse ; une reponse detectee
 * juste avant la fin de la fenetre se termine jusqu'a cette duree plus tard.
 * L'attente logicielle couvre donc les deux, sinon on abandonnait une reponse
 * en cours de reception. */
#define LORA_PAIR_RESPONSE_MAX_AIRTIME_MS  300

/* PAIR_CONFIRM : recharge de l'alimentation, puis deux emissions.
 *
 * Le ZLinky enchaine PAIR_REQUEST, reception de la PAIR_RESPONSE et
 * PAIR_CONFIRM. Emise ~0,65 s apres la reponse, la PAIR_CONFIRM partait
 * sur une reserve vide sur les compteurs faibles : la box la recevait
 * tronquee (erreur CRC) ou pas du tout (mesure sur un triphase).
 *
 * Les instants sont comptes depuis la reception de la PAIR_RESPONSE. La box
 * (v3.3) attend la PAIR_CONFIRM 4 s apres la fin de sa reponse ; la seconde
 * emission doit donc se terminer avant ~3,5 s, y compris a SF12 (~360 ms
 * pour 15 octets) : 3000 + 360 = 3360 ms.
 *
 * La seconde emission rattrape une premiere coupee par une chute de tension.
 * Elle est identique (meme seq) ; la box ignore une PAIR_CONFIRM en trop. */
#define LORA_PAIR_CONFIRM_FIRST_MS     1700
#define LORA_PAIR_CONFIRM_SECOND_MS    3000
#define LORA_PAIR_CONFIRM_COUNT        2

/* LED pendant l'appairage. Elle puise dans la meme reserve que la radio :
 * avant, elle restait allumee 40 % (demarrage) a 67 % (appui long) du temps,
 * et clignotait aussi pendant l'echange. Desormais : un flash de 20 ms toutes
 * les 500 ms entre deux tentatives (~4 % du temps), et LED eteinte de
 * l'emission de la PAIR_REQUEST a la fin de la derniere PAIR_CONFIRM. */
#define LORA_PAIR_LED_PERIOD_MS        500
#define LORA_PAIR_LED_FLASH_MS         20
#define LORA_LED_FLASH_DEFAULT_MS      100   /* flash hors appairage */

/* Puissance d'emission pendant l'appairage uniquement (dBm).
 *
 * On reste a +10 dBm, comme les donnees : le Linky n'est pas mobile,
 * l'appairage se fait donc a la DISTANCE OPERATIONNELLE reelle et a
 * besoin de toute la portee. Baisser la puissance ici reviendrait a
 * n'autoriser l'appairage que des installations proches.
 *
 * La charge tiree est maitrisee par le SF du rendez-vous (SF10, cf.
 * LORA_PAIR_SF) et non par la puissance : 5,6 mA.s a pleine puissance,
 * contre 11,1 mA.s a SF11 qui echouait sur certaines installations.
 *
 * Ce define reste un point de reglage si une installation resistait
 * malgre le SF10. */
#define LORA_PAIR_TX_POWER_DBM         10

/* Puissance d'emission adaptee au lien.
 *
 * Le ZLinky mesure le RSSI de la PAIR_RESPONSE et en deduit la perte de
 * trajet, en supposant que la box emet a LORA_BOX_TX_POWER_ASSUMED_DBM.
 * Il choisit ensuite la puissance qui ferait arriver ses trames a la box
 * vers LORA_LINK_TARGET_RSSI_DBM, soit ~30 dB au-dessus de la limite de
 * reception (~-120 dBm a SF11) : marge pour les evanouissements, les
 * obstacles qui bougent et l'imprecision du RSSI.
 *
 *   puissance = cible + (puissance box - RSSI recu), bornee [MIN, MAX]
 *
 * Exemples : RSSI -58 dBm → 0 dBm ; -85 dBm → +5 dBm ; -90 dBm ou moins → +10.
 *
 * La LiXee-Box emet a +10 dBm (valeur donnee par son developpeur). Si une
 * box emettait plus fort, le ZLinky sous-estimerait la perte de trajet et
 * emettrait trop faiblement : la marge en absorberait une partie.
 *
 * Le compteur ne bouge pas : la valeur est calculee a l'appairage et
 * memorisee avec lui. La PAIR_REQUEST, premiere trame de l'echange, part
 * toujours a LORA_PAIR_TX_POWER_DBM ; la PAIR_CONFIRM et toutes les
 * trames suivantes (donnees, POLL, OTA) a la puissance adaptee. */
#define LORA_TX_POWER_MIN_DBM          0
#define LORA_TX_POWER_MAX_DBM          10    /* limite ETSI EN 300 440 */
#define LORA_BOX_TX_POWER_ASSUMED_DBM  10
#define LORA_LINK_TARGET_RSSI_DBM      (-90)
#define LORA_TX_POWER_CODE_OFFSET      64    /* code PDM = dBm + 64 ; 0 = non renseigne */

/* LED patterns */
#define LORA_LED_UNPAIRED_PERIOD_MS    200   /* Fast blink 5Hz */
#define LORA_LED_PAIRING_ON_MS         100   /* Triple flash */
#define LORA_LED_PAIRING_OFF_MS        100
#define LORA_LED_PAIRING_PAUSE_MS      700

/* AES key size */
#define LORA_AES_KEY_SIZE              16

/* MAC address size */
#define LORA_MAC_SIZE                  8

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Pairing state machine */
typedef enum {
    LORA_PAIR_UNPAIRED = 0,   /* No key stored, LED blinks fast, no data TX */
    LORA_PAIR_PAIRING,        /* Button pressed, sending PAIR_REQUEST, listening */
    LORA_PAIR_PAIRED          /* Key stored, encrypted data TX active */
} eLoraPairState_t;

/* PDM persistent pairing data
 *
 * IMPORTANT : la taille de cette structure ne doit PAS changer. Les 3 octets
 * anciennement reserves portent desormais le SF et l'intervalle TX, negocies
 * a l'appairage (PAIR_RESPONSE). Un enregistrement PDM ecrit par une version
 * anterieure les lit a 0, valeur invalide qui declenche le repli sur les
 * defauts (cf. APP_u8LoraPairGetSF / GetTxIntervalSec). Les modules deja
 * appaires survivent donc a la mise a jour.
 *
 * Le SF et le canal se changent en REFAISANT un appairage (appui long), pas
 * en cours de route : ce sont des parametres reseau communs a tous les
 * ZLinky d'un meme recepteur, pas des reglages par appareil. */
typedef struct {
    uint32  u32Magic;                   /* LORA_PAIR_MAGIC = valid record */
    uint8   au8NetKey[LORA_AES_KEY_SIZE]; /* AES-128 network key */
    uint8   au8MAC[LORA_MAC_SIZE];      /* Own MAC address (for nonce) */
    uint8   u8OpChannel;                /* Operational channel (0..7), assigned by RX at pairing */
    uint8   u8SpreadingFactor;          /* SF 7..12 ; 0 = non renseigne → defaut */
    uint8   u8TxIntervalSec;            /* Intervalle TX en s ; 0 = non renseigne → defaut */
    uint8   u8TxPowerCode;              /* puissance TX : dBm + LORA_TX_POWER_CODE_OFFSET ; 0 = non renseigne → max */
} tsLoraPairingData;

/* Bornes et defauts de la config radio (negociee a l'appairage) */
#define LORA_SF_MIN                    7
#define LORA_SF_MAX                    12
#define LORA_SF_DEFAULT                11
#define LORA_TX_INTERVAL_SEC_MIN       3    /* sous 3 s le budget alim TIC ne suit plus */
#define LORA_TX_INTERVAL_SEC_MAX       240
#define LORA_TX_INTERVAL_SEC_DEFAULT   5

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Init pairing module: load PDM, determine initial state, start LED
 *        Call BEFORE APP_vLoRaTicStart() in APP_vInitialiseRouter()
 */
PUBLIC void APP_vLoraPairInit(void);

/**
 * @brief Handle button events for pairing
 *        Short press = start pairing, Long press = erase pairing
 * @param eEvent  APP_E_EVENT_BUTTON_DOWN or APP_E_EVENT_LEAVE_AND_RESET
 */
PUBLIC void APP_vLoraPairOnButton(APP_teEventType eEvent);

/**
 * @brief Returns TRUE if device is paired (data TX allowed)
 */
PUBLIC bool_t APP_bLoraPairIsPaired(void);

/**
 * @brief Get pointer to the AES-128 network key (16 bytes)
 *        Only valid when paired
 */
PUBLIC uint8* APP_pu8LoraPairGetKey(void);

/**
 * @brief Get pointer to the MAC address (8 bytes)
 *        Used for nonce construction
 */
PUBLIC uint8* APP_pu8LoraPairGetMAC(void);

/**
 * @brief Get the operational channel (0..LORA_NUM_CHANNELS-1)
 *        Returns LORA_PAIR_CHANNEL if not paired or invalid.
 */
PUBLIC uint8 APP_u8LoraPairGetOpChannel(void);

/**
 * @brief Config radio negociee a l'appairage. Chaque getter valide la valeur
 *        stockee et retombe sur le defaut si elle est hors bornes (cas d'un
 *        enregistrement PDM ecrit par une version anterieure).
 */
PUBLIC uint8 APP_u8LoraPairGetSF(void);
PUBLIC uint8 APP_u8LoraPairGetTxIntervalSec(void);

/**
 * @brief Puissance d'emission adaptee au lien (cf. LORA_TX_POWER_*). Retourne
 *        LORA_TX_POWER_MAX_DBM si elle n'a pas ete mesuree (appairage
 *        anterieur a la 1.1.4).
 */
PUBLIC int8 APP_i8LoraPairGetTxPowerDbm(void);

/**
 * @brief Applique une valeur de config en RAM. Utilise pendant le handshake
 *        pour stocker le SF assigne par le recepteur avant persistance.
 * @return TRUE si la valeur est dans les bornes et a ete appliquee
 */
PUBLIC bool_t APP_bLoraPairSetOpChannel(uint8 u8Channel);
PUBLIC bool_t APP_bLoraPairSetSF(uint8 u8SF);
PUBLIC bool_t APP_bLoraPairSetTxIntervalSec(uint8 u8Sec);

/**
 * @brief Timer callback for pairing cycle (500ms TX/RX)
 *        Registered as ZTIMER callback in app_main.c
 */
PUBLIC void APP_cbTimerLoraPair(void *pvParam);

/**
 * @brief Erase pairing data from PDM, return to UNPAIRED
 */
PUBLIC void APP_vLoraPairErase(void);

/**
 * @brief Get current pairing state
 */
PUBLIC eLoraPairState_t APP_eLoraPairGetState(void);

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* APP_LORA_PAIR_H_ */

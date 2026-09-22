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
#define LORA_PAIR_CONFIRM_SIZE         15  /* type+seq+MAC(8)+proof(4)+status */

/* Rendez-vous d'appairage : FIXE, independant de la config operationnelle.
 * Un ZLinky qui tourne en SF7/canal7 doit pouvoir revenir ici pour renegocier.
 * Le canal est LORA_PAIR_CHANNEL (defini dans app_lora_channels.h). */
#define LORA_PAIR_SF                   11  /* SF du handshake (SX1280 SF11 = 0xB0) */

/* Extension PAIR_REQUEST (LiXee-Box >= v2.22) :
 *   Format 14+N octets, retro-compatible avec les receivers antérieurs
 *   qui liront simplement 11 octets et ignoreront la suite.
 *   [11-12] device_id (uint16 BE) — nomme le template data/tp/<device_id>.json
 *   [13]    longueur du model (N)
 *   [14..]  model ASCII (N octets)
 */
#define LORA_PAIR_REQUEST_HAS_EXT      1  /* 1 = envoie l'extension, 0 = format legacy 11 octets */
#define LORA_DEVICE_ID_ZLINKY_TIC      0x0051  /* device_id LiXee du ZLinky — template data/tp/0051.json */
#define LORA_DEVICE_MODEL_ZLINKY_TIC   "ZLinky_TIC"  /* cle dans data/tp/0051.json cote LiXee-Box */
#define LORA_DEVICE_MODEL_MAX_LEN      32   /* garde-fou de securite (permet des noms d'objets futurs plus longs) */
#define LORA_PAIR_REQUEST_MAX_SIZE     (LORA_PAIR_REQUEST_BASE_SIZE + 3 + LORA_DEVICE_MODEL_MAX_LEN)

/* PDM record ID for pairing data */
#define PDM_ID_LORA_PAIRING            0xA110

/* Magic value to validate PDM record */
#define LORA_PAIR_MAGIC                0x4C505249  /* "LPRI" */

/* Pairing timeout (30 seconds) */
#define LORA_PAIR_TIMEOUT_MS           30000

/* Intervalle entre deux PAIR_REQUEST.
 *
 * ATTENTION au budget d'alimentation TIC. A SF11 un PAIR_REQUEST dure
 * ~250 ms d'airtime : a 300 ms d'intervalle on emettait ~83 % du temps,
 * alors que le mode donnees n'emet que 5 % (250 ms toutes les 5 s).
 * C'est dix fois au-dessus du minimum documente par
 * LORA_TX_INTERVAL_SEC_MIN ("sous 3 s le budget alim TIC ne suit plus").
 *
 * Constate sur le terrain : avec 300 ms, l'appairage fonctionne sur un
 * Linky monophase mais echoue systematiquement sur un triphase place au
 * MEME endroit, a la MEME distance du recepteur (donc a RF identique) --
 * la seule variable restante etant l'energie fournie par la prise TIC.
 *
 * A 2500 ms le rapport cyclique retombe a ~10 %, et la fenetre
 * d'appairage de 30 s laisse encore une douzaine de tentatives. */
#define LORA_PAIR_TX_INTERVAL_MS       2500

/* RX listen window after each PAIR_REQUEST (200ms) */
#define LORA_PAIR_RX_TIMEOUT_MS        200

/* Puissance d'emission pendant l'appairage uniquement (dBm).
 *
 * Plus basse que les donnees (+10 dBm) pour reduire le PIC de courant du
 * PA : ~27 mA a 0 dBm contre ~45 mA a +10 dBm.
 *
 * Constate sur le terrain : sur un Linky triphase, TX_DONE ne se leve
 * JAMAIS pendant l'appairage ("PAIR_REQUEST TX timeout" a chaque essai),
 * alors qu'il se leve systematiquement sur un monophase place au MEME
 * endroit, a la MEME distance du recepteur. Or TX_DONE est un evenement
 * interne au SX1280 : il ne depend ni de la distance, ni du recepteur,
 * ni de la propagation. La seule variable restante est donc l'energie
 * fournie par la prise TIC.
 *
 * Espacer les PAIR_REQUEST (300 -> 2500 ms) n'a rien change : ce n'est
 * pas le rapport cyclique moyen mais le pic instantane qui compte.
 *
 * L'appairage est une operation de mise en service, faite a proximite du
 * recepteur : la portee n'y est pas critique. Mesure en monophase lors
 * d'un appairage reussi : RSSI = -53 dBm, soit une marge enorme. */
#define LORA_PAIR_TX_POWER_DBM         0

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
    uint8   u8Reserved;                 /* alignement / usage futur */
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

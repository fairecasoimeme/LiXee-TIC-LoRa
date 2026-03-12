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
#define LORA_PAIR_REQUEST_SIZE         11  /* type+seq+MAC(8)+channel */
#define LORA_PAIR_RESPONSE_SIZE        19  /* type+seq+key(16)+status */
#define LORA_PAIR_CONFIRM_SIZE         15  /* type+seq+MAC(8)+proof(4)+status */

/* PDM record ID for pairing data */
#define PDM_ID_LORA_PAIRING            0xA110

/* Magic value to validate PDM record */
#define LORA_PAIR_MAGIC                0x4C505249  /* "LPRI" */

/* Pairing timeout (30 seconds) */
#define LORA_PAIR_TIMEOUT_MS           30000

/* Pairing TX interval (300ms between PAIR_REQUEST) */
#define LORA_PAIR_TX_INTERVAL_MS       300

/* RX listen window after each PAIR_REQUEST (200ms) */
#define LORA_PAIR_RX_TIMEOUT_MS        200

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

/* PDM persistent pairing data */
typedef struct {
    uint32  u32Magic;                   /* LORA_PAIR_MAGIC = valid record */
    uint8   au8NetKey[LORA_AES_KEY_SIZE]; /* AES-128 network key */
    uint8   au8MAC[LORA_MAC_SIZE];      /* Own MAC address (for nonce) */
} tsLoraPairingData;

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

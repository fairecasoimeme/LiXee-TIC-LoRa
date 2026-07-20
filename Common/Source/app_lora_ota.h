/*****************************************************************************
 *
 * MODULE:             app_lora_ota
 *
 * COMPONENT:          app_lora_ota.h
 *
 * DESCRIPTION:        LoRa OTA firmware update receiver-side state machine.
 *                     Receives firmware blocks via existing pairing/AES infra
 *                     and writes them to flash slot B (dual-bank).
 *                     On full firmware received + CRC valid, sets boot flag
 *                     and reboots into new firmware.
 *
 *                     Protocol (extends pairing protocol with new types) :
 *                       OTA_START  (gateway -> device)  v1+type=0x06
 *                       OTA_BLOCK  (gateway -> device)  v1+type=0x07
 *                       OTA_END    (gateway -> device)  v1+type=0x08
 *                       OTA_ACK    (device -> gateway)  v1+type=0x09
 *                       OTA_STATUS (device -> gateway)  v1+type=0x0A
 *
 ****************************************************************************/

#ifndef APP_LORA_OTA_H_
#define APP_LORA_OTA_H_

#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Packet types (byte[0] bits[3:0]) */
#define LORA_TIC_PKT_OTA_START     0x06
#define LORA_TIC_PKT_OTA_BLOCK     0x07
#define LORA_TIC_PKT_OTA_END       0x08
#define LORA_TIC_PKT_OTA_ACK       0x09
#define LORA_TIC_PKT_OTA_STATUS    0x0A

/* Block size choisi : compromis entre debit et fragmentation */
#define LORA_OTA_BLOCK_SIZE        64    /* bytes per OTA block */

/* Max blocks supported (firmware ~256KB / 64B = 4096 blocks max) */
#define LORA_OTA_MAX_BLOCKS        4096

/* Bitmap pour tracker les blocks recus : 4096 bits = 512 bytes */
#define LORA_OTA_BITMAP_SIZE       (LORA_OTA_MAX_BLOCKS / 8)

/* Status codes (pour OTA_STATUS et OTA_ACK) */
#define LORA_OTA_STATUS_OK              0x00
#define LORA_OTA_STATUS_BUSY            0x01
#define LORA_OTA_STATUS_BLOCK_BAD_CRC   0x02
#define LORA_OTA_STATUS_FLASH_ERROR     0x03
#define LORA_OTA_STATUS_OUT_OF_RANGE    0x04
#define LORA_OTA_STATUS_FW_TOO_BIG      0x05
#define LORA_OTA_STATUS_FW_BAD_CRC      0x06
#define LORA_OTA_STATUS_NOT_STARTED     0x07
#define LORA_OTA_STATUS_VERSION_OLD     0x08

/* Tailles paquets (sans MIC, qui s'ajoute apres chiffrement AES) */
#define LORA_OTA_START_SIZE        15  /* type+seq+ver(2)+size(4)+crc(4)+total_blocks(2)+block_size(1) */
#define LORA_OTA_BLOCK_HDR_SIZE    4   /* type+seq+block_idx(2) */
#define LORA_OTA_BLOCK_FULL_SIZE   (LORA_OTA_BLOCK_HDR_SIZE + LORA_OTA_BLOCK_SIZE)
#define LORA_OTA_END_SIZE          7   /* type+seq+full_crc32(4)+reserved(1) */
#define LORA_OTA_ACK_SIZE          7   /* type+seq+block_idx(2)+status(1)+reserved(2) */
#define LORA_OTA_STATUS_SIZE       8   /* type+seq+state(1)+blocks_recv(2)+last_block(2) */

/* Timeout sur reception block (si pas de block recu pendant N ms, abort) */
#define LORA_OTA_BLOCK_TIMEOUT_MS  10000   /* 10 sec */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef enum {
    OTA_STATE_IDLE = 0,         /* Pas d'OTA en cours */
    OTA_STATE_LISTENING,        /* Continuous RX, attend OTA_START */
    OTA_STATE_RECEIVING,        /* OTA_START recu, blocks en cours */
    OTA_STATE_FINALIZING,       /* OTA_END recu, validation CRC */
    OTA_STATE_COMMITTING,       /* CRC OK, ecriture flag boot, reboot imminent */
    OTA_STATE_FAILED            /* Erreur, OTA aborte */
} eOtaState_t;

/* Timeout sur le mode listening (sortie auto si pas de OTA_START recu) */
#define LORA_OTA_LISTEN_TIMEOUT_MS  300000   /* 5 minutes */

typedef struct {
    eOtaState_t state;
    uint16  u16NewVersion;
    uint32  u32FirmwareSize;
    uint32  u32FirmwareCrc32;
    uint16  u16TotalBlocks;
    uint16  u16BlockSize;
    uint16  u16BlocksReceived;
    uint16  u16LastBlockIdx;
    uint16  u16ExpectedBlock;   /* Prochain block attendu (sequentiel) */
    uint32  u32LastBlockTime;   /* timestamp dernier block recu (timeout detection) */
    uint8   au8BlockBitmap[LORA_OTA_BITMAP_SIZE];
    uint8   u8LastSeq;
} tsOtaContext_t;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Init OTA module. Called once at boot.
 */
PUBLIC void APP_vLoraOtaInit(void);

/**
 * @brief Get current OTA state
 */
PUBLIC eOtaState_t APP_eLoraOtaGetState(void);

/**
 * @brief Enter OTA_LISTENING mode :
 *        - Stops normal data TX
 *        - Configures SX1280 in continuous RX
 *        - Listens for OTA_START packets for LORA_OTA_LISTEN_TIMEOUT_MS
 *        Trigger via button (very long press) or serial command.
 *        Auto-exits on timeout or successful OTA_END.
 */
PUBLIC void APP_vLoraOtaEnterListen(void);

/**
 * @brief Exit OTA listening mode and return to normal data TX.
 */
PUBLIC void APP_vLoraOtaExitListen(void);

/**
 * @brief Process an incoming RX packet that may be OTA-related.
 *        Caller has already decrypted (via AES) and verified MIC.
 *        Inspects byte[0] to dispatch to OTA_START/BLOCK/END handlers.
 * @param buf  Decrypted packet buffer
 * @param len  Packet length without MIC
 * @return TRUE if it was an OTA packet (handled), FALSE otherwise
 */
PUBLIC bool_t APP_bLoraOtaProcessPacket(uint8 *buf, uint16 len);

/**
 * @brief Handle incoming OTA_START packet (from gateway).
 *        Initializes context, prepares slot B for write.
 * @param buf Decrypted packet data
 * @param len Packet length (without MIC)
 * @return TRUE if accepted, FALSE if rejected
 */
PUBLIC bool_t APP_bLoraOtaOnStart(uint8 *buf, uint16 len);

/**
 * @brief Handle incoming OTA_BLOCK packet.
 *        Writes block to flash slot B at correct offset.
 * @param buf Decrypted packet data
 * @param len Packet length (without MIC)
 * @return TRUE if block stored, FALSE on error (out of range, flash err, etc.)
 */
PUBLIC bool_t APP_bLoraOtaOnBlock(uint8 *buf, uint16 len);

/**
 * @brief Handle incoming OTA_END packet.
 *        Verifies firmware CRC32 over all received data.
 *        If valid : sets boot flag, schedules reboot.
 * @param buf Decrypted packet data
 * @param len Packet length (without MIC)
 * @return TRUE if commit will proceed, FALSE on error
 */
PUBLIC bool_t APP_bLoraOtaOnEnd(uint8 *buf, uint16 len);

/**
 * @brief Build and send an OTA_ACK reply
 * @param u16BlockIdx Block ack'd (or last received in case of NACK)
 * @param u8Status One of LORA_OTA_STATUS_*
 */
PUBLIC void APP_vLoraOtaSendAck(uint16 u16BlockIdx, uint8 u8Status);

/**
 * @brief Build and send an OTA_STATUS reply (progress info)
 */
PUBLIC void APP_vLoraOtaSendStatus(void);

/**
 * @brief Periodic tick to check timeouts and process RX_DONE events.
 *        Should be called every 50-100ms when in LISTENING/RECEIVING state.
 *        Wire this in a ZTIMER callback.
 */
PUBLIC void APP_vLoraOtaTick(void);

/**
 * @brief Read the user button state at boot to decide if OTA listen mode
 *        should be entered. Call AT THE VERY BEGINNING of init, before
 *        any other timer/interrupt is set up.
 * @return TRUE if the button is held (= enter OTA mode), FALSE otherwise.
 */
PUBLIC bool_t APP_bLoraOtaButtonHeldAtBoot(void);

/**
 * @brief OTA-only main loop. Sets up timer + listening + tight RX poll.
 *        Does NOT return until either:
 *        - Timeout (5 min without OTA_START)
 *        - OTA_END committed → reboot
 *        - User power-cycle
 *        Call this if APP_bLoraOtaButtonHeldAtBoot() returned TRUE.
 */
PUBLIC void APP_vLoraOtaRunListenLoop(void);

#endif /* APP_LORA_OTA_H_ */

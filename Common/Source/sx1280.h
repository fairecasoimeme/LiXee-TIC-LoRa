#ifndef SX1280_H
#define SX1280_H

#include <stdint.h>

// ===== Commandes SX1280 =====
#define SX1280_CMD_GET_STATUS               0xC0
#define SX1280_CMD_WRITE_REGISTER           0x18
#define SX1280_CMD_READ_REGISTER            0x19
#define SX1280_CMD_WRITE_BUFFER             0x1A
#define SX1280_CMD_READ_BUFFER              0x1B
#define SX1280_CMD_SET_STANDBY              0x80
#define SX1280_CMD_SET_FS                   0xC1
#define SX1280_CMD_SET_TX                   0x83
#define SX1280_CMD_SET_RX                   0x82
#define SX1280_CMD_SET_PACKET_TYPE          0x8A
#define SX1280_CMD_SET_RF_FREQUENCY         0x86
#define SX1280_CMD_SET_TX_PARAMS            0x8E
#define SX1280_CMD_SET_BUFFER_BASE_ADDR     0x8F
#define SX1280_CMD_SET_MODULATION_PARAMS    0x8B
#define SX1280_CMD_SET_PACKET_PARAMS        0x8C
#define SX1280_CMD_GET_RX_BUFFER_STATUS     0x17
#define SX1280_CMD_GET_PACKET_STATUS        0x1D
#define SX1280_CMD_SET_DIO_IRQ_PARAMS       0x8D
#define SX1280_CMD_CLEAR_IRQ_STATUS         0x97
#define SX1280_CMD_GET_IRQ_STATUS           0x15
#define SX1280_CMD_SET_REGULATOR_MODE       0x96
#define SX1280_CMD_SET_TX_CONTINUOUS_WAVE   0xD1
#define SX1280_CMD_SET_TX_CONTINUOUS_PREAMBLE 0xD2
/*
 * NOTE: Le SX1280 n'a PAS de commande GetDeviceErrors (c'est SX126x).
 * L'opcode 0x17 est GetRxBufferStatus sur SX1280.
 * Le diagnostic oscillateur utilise SetStandby(XOSC) a la place.
 */
#define SX1280_CMD_CALIBRATE                0x89

// ===== Packet Type =====
#define SX1280_PACKET_TYPE_GFSK             0x00
#define SX1280_PACKET_TYPE_LORA             0x01
#define SX1280_PACKET_TYPE_RANGING          0x02
#define SX1280_PACKET_TYPE_FLRC             0x03
#define SX1280_PACKET_TYPE_BLE              0x04

// ===== Standby modes =====
#define SX1280_STANDBY_RC                   0x00
#define SX1280_STANDBY_XOSC                 0x01

// ===== Regulator modes (SetRegulatorMode 0x96) =====
#define SX1280_REGULATOR_LDO                0x00  // LDO (defaut au power-up)
#define SX1280_REGULATOR_DCDC               0x01  // DC-DC (plus efficace, requis pour PLL)

// ===== IRQ Masks =====
#define SX1280_IRQ_TX_DONE                  0x0001
#define SX1280_IRQ_RX_DONE                  0x0002
#define SX1280_IRQ_SYNC_WORD_VALID          0x0004
#define SX1280_IRQ_SYNC_WORD_ERROR          0x0008
#define SX1280_IRQ_HEADER_VALID             0x0010
#define SX1280_IRQ_HEADER_ERROR             0x0020
#define SX1280_IRQ_CRC_ERROR                0x0040
#define SX1280_IRQ_RANGING_SLAVE_RESPONSE   0x0080
#define SX1280_IRQ_RANGING_SLAVE_REQUEST    0x0100
#define SX1280_IRQ_RANGING_MASTER_RESULT    0x0200
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT   0x0400
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED 0x0800
#define SX1280_IRQ_CAD_DONE                 0x1000
#define SX1280_IRQ_CAD_DETECTED             0x2000
#define SX1280_IRQ_RX_TX_TIMEOUT            0x4000
#define SX1280_IRQ_PREAMBLE_DETECTED        0x8000
#define SX1280_IRQ_ALL                      0xFFFF

// ===== LoRa Spreading Factors =====
#define SX1280_LORA_SF5                     0x50
#define SX1280_LORA_SF6                     0x60
#define SX1280_LORA_SF7                     0x70
#define SX1280_LORA_SF8                     0x80
#define SX1280_LORA_SF9                     0x90
#define SX1280_LORA_SF10                    0xA0
#define SX1280_LORA_SF11                    0xB0
#define SX1280_LORA_SF12                    0xC0

// ===== LoRa Bandwidths =====
#define SX1280_LORA_BW_203                  0x34  // 203 kHz
#define SX1280_LORA_BW_406                  0x26  // 406 kHz
#define SX1280_LORA_BW_812                  0x18  // 812 kHz
#define SX1280_LORA_BW_1625                 0x0A  // 1625 kHz

// ===== Longueur du preambule LoRa =====
//
// ATTENTION a l'encodage : mantisse * 2^exposant sur 1 octet.
//   bits[3:0] = mantisse (1..15), bits[7:4] = exposant (0..15)
// Une valeur brute comme 16 (0x10) donne mantisse=0, ce qui est INVALIDE
// (bug deja rencontre sur ce projet, il empechait toute reception).
//
// /!\ VALEUR BILATERALE : le preambleLength du recepteur doit etre
// INFERIEUR OU EGAL a celui qu'on emet, sinon il attend des symboles qui
// n'arrivent jamais et la detection echoue completement.
//
// Pourquoi 8 symboles : a SF11/BW406 le preambule coute 102 ms a 16
// symboles contre 62 ms a 8, soit 1,8 mA.s economises sur CHAQUE trame.
// Le budget d'alimentation de la prise TIC limite la charge par emission,
// et a 11,0 mA.s seules ~36 % des trames aboutissent sur certains
// compteurs (contre 100 % a 8,7 mA.s). La sensibilite, elle, ne depend
// que du SF/BW/CR : 8 symboles ne coute donc pas de portee, seulement un
// peu de marge de detection a tres faible SNR. LoRaWAN utilise 8
// symboles pour tous ses uplinks, c'est une valeur eprouvee.
#define SX1280_PREAMBLE_8_SYM               0x08  // 8 * 2^0 =  8 symboles
#define SX1280_PREAMBLE_16_SYM              0x18  // 8 * 2^1 = 16 symboles
#define SX1280_LORA_PREAMBLE                SX1280_PREAMBLE_8_SYM

// ===== LoRa Coding Rates =====
#define SX1280_LORA_CR_4_5                  0x01
#define SX1280_LORA_CR_4_6                  0x02
#define SX1280_LORA_CR_4_7                  0x03
#define SX1280_LORA_CR_4_8                  0x04
#define SX1280_LORA_CR_LI_4_5               0x05
#define SX1280_LORA_CR_LI_4_6               0x06
#define SX1280_LORA_CR_LI_4_7               0x07

// ===== LoRa Packet Parameter Values =====
// ATTENTION: CRC et InvertIQ ne sont PAS des booleens 0/1 !
// Ce sont des valeurs specifiques du SX1280 (confirmees par RadioLib).
#define SX1280_LORA_CRC_ON                  0x20
#define SX1280_LORA_CRC_OFF                 0x00
#define SX1280_LORA_IQ_NORMAL               0x40  // IQ standard (pas d'inversion)
#define SX1280_LORA_IQ_INVERTED             0x00  // IQ inverse

#define SX1280_LORA_HEADER_EXPLICIT         0x00
#define SX1280_LORA_HEADER_IMPLICIT         0x80

// ===== TX Ramp Times (SetTxParams) =====
// ATTENTION: seules ces valeurs sont valides. Toute autre valeur
// provoque un blocage du SX1280 (BUSY=HIGH permanent).
#define SX1280_RAMP_02_US                   0x00
#define SX1280_RAMP_04_US                   0x20
#define SX1280_RAMP_06_US                   0x40
#define SX1280_RAMP_08_US                   0x60
#define SX1280_RAMP_10_US                   0x80
#define SX1280_RAMP_12_US                   0xA0
#define SX1280_RAMP_16_US                   0xC0
#define SX1280_RAMP_20_US                   0xE0

// ===== Structures =====

/**
 * @brief Paramètres de modulation LoRa
 */
typedef struct {
    uint8_t spreadingFactor;    // SF5-SF12
    uint8_t bandwidth;          // BW 203, 406, 812, 1625 kHz
    uint8_t codingRate;         // CR 4/5 à 4/8
} SX1280_LoRaModParams_t;

/**
 * @brief Paramètres de paquet LoRa
 */
typedef struct {
    uint8_t preambleLength;     // Longueur du préambule
    uint8_t headerType;         // 0=explicit, 1=implicit
    uint8_t payloadLength;      // Longueur du payload
    uint8_t crc;                // 0=off, 1=on
    uint8_t invertIQ;           // 0=standard, 1=inverted
} SX1280_LoRaPacketParams_t;

// ===== Fonctions publiques =====

/**
 * @brief Initialise le SX1280
 */
void SX1280_Init(void);

/**
 * @brief Met le SX1280 en mode Standby
 * @param mode SX1280_STANDBY_RC ou SX1280_STANDBY_XOSC
 */
void SX1280_SetStandby(uint8_t mode);

/**
 * @brief Configure le type de paquet
 * @param packetType Type de paquet (GFSK, LoRa, etc.)
 */
void SX1280_SetPacketType(uint8_t packetType);

/**
 * @brief Configure la fréquence RF
 * @param frequency Fréquence en Hz (ex: 2400000000 pour 2.4 GHz)
 */
void SX1280_SetRfFrequency(uint32_t frequency);

/**
 * @brief Configure les paramètres de modulation LoRa
 * @param params Pointeur vers la structure de paramètres
 */
void SX1280_SetModulationParams(SX1280_LoRaModParams_t *params);

/**
 * @brief Configure les paramètres de paquet LoRa
 * @param params Pointeur vers la structure de paramètres
 */
void SX1280_SetPacketParams(SX1280_LoRaPacketParams_t *params);

/**
 * @brief Configure la puissance de transmission
 * @param power Puissance en dBm (-18 à +13 dBm)
 * @param rampTime Temps de montée (0x00 à 0x07)
 */
void SX1280_SetTxParams(int8_t power, uint8_t rampTime);

/**
 * @brief Configure les adresses de base des buffers TX et RX
 * @param txBase Adresse de base du buffer TX
 * @param rxBase Adresse de base du buffer RX
 */
void SX1280_SetBufferBaseAddress(uint8_t txBase, uint8_t rxBase);

/**
 * @brief Configure les interruptions
 * @param irqMask Masque des IRQ à activer
 * @param dio1Mask Masque des IRQ à router sur DIO1
 */
void SX1280_SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask);

/**
 * @brief Envoie un payload
 * @param payload Données à envoyer
 * @param size Taille des données
 */
void SX1280_SendPayload(uint8_t *payload, uint8_t size);

/**
 * @brief Met le SX1280 en mode réception
 * @param timeout Timeout en ms (0xFFFF = RX continu)
 */
void SX1280_SetRx(uint16_t timeout);

/**
 * @brief Lit le payload reçu
 * @param buffer Buffer pour recevoir les données
 * @param size Pointeur vers la taille reçue
 * @param rssi Pointeur vers le RSSI
 * @param snr Pointeur vers le SNR
 * @return 1 si données reçues, 0 sinon
 */
uint8_t SX1280_GetRxPayload(uint8_t *buffer, uint8_t *size, int8_t *rssi, int8_t *snr);

/**
 * @brief Lit le status des IRQ
 * @return Status des IRQ (16 bits)
 */
uint16_t SX1280_GetIrqStatus(void);

/**
 * @brief Efface les IRQ
 * @param irq Masque des IRQ à effacer
 */
void SX1280_ClearIrqStatus(uint16_t irq);

/**
 * @brief Lit le status du SX1280
 * @return Status (8 bits)
 */
uint8_t SX1280_GetStatus(void);

/**
 * @brief Configure le mode du regulateur de tension
 *        CRITIQUE : doit etre appele apres SetPacketType et AVANT SetRfFrequency.
 *        Sans cette commande, le SX1280 reste en mode LDO (defaut au power-up)
 *        et le PLL ne peut pas se verrouiller (BUSY reste HIGH indefiniment).
 *        RadioLib appelle SetRegulatorMode(DC-DC) dans sa fonction config().
 * @param mode SX1280_REGULATOR_LDO (0x00) ou SX1280_REGULATOR_DCDC (0x01)
 */
void SX1280_SetRegulatorMode(uint8_t mode);

/**
 * @brief Passe en mode Frequency Synthesis (PLL lock)
 *        Utile pour verifier que le SX1280 peut quitter le mode STDBY.
 */
void SX1280_SetFs(void);

/**
 * @brief Emet une porteuse continue (onde CW) sans modulation
 *        Test le plus simple pour verifier le chemin TX RF.
 *        Le SX1280 reste en mode TX CW jusqu'a un SetStandby.
 */
void SX1280_SetTxContinuousWave(void);

/**
 * @brief Emet un preambule LoRa continu
 *        Test intermediaire entre CW et TX normal.
 *        Le SX1280 reste en mode TX jusqu'a un SetStandby.
 */
void SX1280_SetTxContinuousPreamble(void);

/**
 * @brief Force la recalibration interne du SX1280
 *        Recalibre les oscillateurs RC (13MHz, 64kHz), le PLL et l'ADC.
 *        Utile pour corriger une derive progressive (thermique ou autre)
 *        sans avoir besoin de power-cycler le TCXO.
 *        Doit etre appele en STDBY_RC. Dure ~3.5ms.
 * @param calibParam Bitmask : bit6=RC64k, bit5=RC13M, bit4=PLL,
 *        bit3=ADCpulse, bit2=ADCbulkN, bit1=ADCbulkP. 0x7E = all.
 */
void SX1280_Calibrate(uint8_t calibParam);

#endif // SX1280_H

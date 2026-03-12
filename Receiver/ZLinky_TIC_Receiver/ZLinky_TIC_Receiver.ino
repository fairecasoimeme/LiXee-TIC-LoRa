/*
 * ZLinky TIC Receiver — SX1280 LoRa 2.4GHz
 * Recoit les paquets DATA_ESSENTIAL (17+2B) et DATA_EXTENDED (9-19+2B)
 * envoyes par le ZLinky (JN5189 + SX1280)
 *
 * Appairage : 30s au boot en mode PAIR_LISTEN
 *   1) Recoit PAIR_REQUEST (MAC emetteur)
 *   2) Genere cle AES-128, envoie PAIR_RESPONSE
 *   3) Recoit PAIR_CONFIRM (verification preuve AES-ECB)
 *   4) Sauvegarde MAC+cle en EEPROM
 *
 * Apres appairage : dechiffrement AES-128-CTR + verification MIC CMAC
 *
 * Config LoRa : 2490 MHz, SF10, BW406.25, CR4/5
 * Preamble 16sym, CRC on, IQ normal, SyncWord 0x12
 * Max 20+2 octets par paquet (SF10 + MIC)
 *
 * RadioLib : https://github.com/jgromes/RadioLib
 *
 * Modes Linky :
 *   0 = Historique monophase
 *   1 = Standard monophase
 *   2 = Historique triphase
 *   3 = Standard triphase
 *   5 = Standard monophase producteur
 *   7 = Standard triphase producteur
 */

#include <SPI.h>
#include <RadioLib.h>
#include <EEPROM.h>

/* AES-128 software implementation (portable, no external deps) */
#include "aes128.h"

// Pins : NSS=7, DIO1=5, RESET=A0, BUSY=3
SX1280 radio = new Module(7, 5, A0, 3);

volatile bool rxFlag = false;
uint32_t rxCount = 0;
uint32_t rxErrors = 0;
uint32_t missedPackets = 0;
uint8_t  lastSeq = 0;
uint8_t  consecutiveCrcErrors = 0;
uint32_t rxSinceReset = 0;
uint32_t totalResets = 0;

#define CRC_ERROR_RESET_THRESHOLD  3
#define PREVENTIVE_RESET_INTERVAL  50
#define SNR_RESET_THRESHOLD       -2.0

/* ====== Types de paquets (byte[0] & 0x0F) ====== */
#define PKT_TYPE_ESSENTIAL      0x01
#define PKT_TYPE_EXTENDED       0x02
#define PKT_TYPE_PAIR_REQUEST   0x03
#define PKT_TYPE_PAIR_RESPONSE  0x04
#define PKT_TYPE_PAIR_CONFIRM   0x05

/* Tailles paquets DATA (sans MIC) */
#define PKT_ESSENTIAL_SIZE  17
#define PKT_EXT_IV_SIZE     15  /* Current/Voltage */
#define PKT_EXT_NRG_SIZE    19  /* Energy */
#define PKT_EXT_NRG2_SIZE   19  /* Energy 2 */
#define PKT_EXT_VSTAT_SIZE  15  /* Voltage Stats */
#define PKT_EXT_PMCFG_SIZE  18  /* Power Max Config */
#define PKT_EXT_DAILY_SIZE  19  /* Daily Energy */
#define PKT_EXT_COMP_SIZE    9  /* Complement */

/* MIC size */
#define CRYPT_MIC_SIZE  2

/* Tailles paquets pairing */
#define PAIR_REQUEST_SIZE   11
#define PAIR_RESPONSE_SIZE  19
#define PAIR_CONFIRM_SIZE   15

/* Sous-types extended (byte[2]) */
#define EXT_CURRENT_VOLTAGE  0x00
#define EXT_ENERGY           0x01
#define EXT_ENERGY_2         0x02
#define EXT_VOLTAGE_STATS    0x03
#define EXT_POWER_MAX_CFG    0x04
#define EXT_DAILY_ENERGY     0x05
#define EXT_COMPLEMENT       0x06

/* AES sizes */
#define AES_KEY_SIZE    16
#define AES_BLOCK_SIZE  16
#define MAC_SIZE         8

/* ====== EEPROM layout ====== */
#define EEPROM_SIZE       64
#define EEPROM_MAGIC_ADDR  0   /* uint32 magic (4B) */
#define EEPROM_KEY_ADDR    4   /* AES key (16B) */
#define EEPROM_MAC_ADDR   20   /* Emitter MAC (8B) */
#define EEPROM_MAGIC_VAL  0x4C505249  /* "LPRI" — same as emitter */

/* ====== Pairing state machine ====== */
typedef enum {
  RX_PAIR_LISTEN = 0,  /* 30s au boot, attend PAIR_REQUEST */
  RX_PAIRED,           /* Appaire, recoit donnees chiffrees */
  RX_IDLE              /* Pas appaire, hors fenetre pairing */
} eRxPairState_t;

eRxPairState_t rxPairState = RX_IDLE;
uint32_t pairListenStart = 0;
#define PAIR_LISTEN_TIMEOUT_MS  30000

/* ====== Bouton USER (B1) sur STM32 Nucleo-64 = PC13 ====== */
#define USER_BUTTON_PIN  PC13
bool lastButtonState = HIGH;  /* Pull-up, actif LOW */
uint32_t buttonPressStart = 0;
bool buttonLongDetected = false;
#define BUTTON_LONG_PRESS_MS  3000  /* 3s appui long pour entrer en pairing */

/* Pairing data */
uint8_t netKey[AES_KEY_SIZE];
uint8_t emitterMAC[MAC_SIZE];
bool isPaired = false;

// Helpers big-endian
uint16_t getU16(const uint8_t *b) { return ((uint16_t)b[0] << 8) | b[1]; }
uint32_t getU32(const uint8_t *b) { return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]; }

void rxDone(void) {
  rxFlag = true;
}

/* Mode Linky en texte */
const char* modeStr(uint8_t mode) {
  switch (mode) {
    case 0: return "Histo Mono";
    case 1: return "Std Mono";
    case 2: return "Histo Tri";
    case 3: return "Std Tri";
    case 5: return "Std Mono Prod";
    case 7: return "Std Tri Prod";
    default: return "Inconnu";
  }
}

bool isTri(uint8_t mode) {
  return (mode == 2 || mode == 3 || mode == 7);
}

/* Full reinit du SX1280 RX */
void doFullReset(const char *reason) {
  Serial.print(F("[RX] *** Full reset: "));
  Serial.print(reason);
  Serial.print(F(" (reset #"));
  Serial.print(++totalResets);
  Serial.print(F(", after "));
  Serial.print(rxSinceReset);
  Serial.println(F(" pkts) ***"));

  consecutiveCrcErrors = 0;
  rxSinceReset = 0;

  radio.begin(2490.0, 406.25, 10, 5, 0x12, 10, 16);
  radio.setCRC(2);
  radio.explicitHeader();
  radio.invertIQ(false);
  radio.setDio1Action(rxDone);
  radio.startReceive(0xFFFF);

  Serial.println(F("[RX] Reset complete, listening...\r\n"));
}

void hexDump(const uint8_t *buf, int len) {
  for (int i = 0; i < len && i < 40; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  if (len > 40) Serial.print(F("..."));
  Serial.println();
}

void trackSequence(uint8_t seq) {
  rxCount++;
  if (rxCount > 1) {
    uint8_t expected = (lastSeq + 1) & 0xFF;
    if (seq != expected) {
      uint8_t missed = (seq - expected) & 0xFF;
      missedPackets += missed;
    }
  }
  lastSeq = seq;
}

void printStats(float rssi, float snr) {
  Serial.print(F("  Stats  : rx="));
  Serial.print(rxCount);
  Serial.print(F(" err="));
  Serial.print(rxErrors);
  Serial.print(F(" miss="));
  Serial.print(missedPackets);
  if (rxCount + missedPackets > 0) {
    Serial.print(F(" PDR="));
    Serial.print(100.0 * rxCount / (rxCount + missedPackets), 0);
    Serial.print(F("%"));
  }
  Serial.print(F("  RSSI="));
  Serial.print(rssi, 0);
  Serial.print(F(" SNR="));
  Serial.println(snr, 1);
}

/* ====================================================================== */
/* ====== EEPROM persistence ====== */
/* ====================================================================== */

void loadPairingFromEEPROM() {
  uint32_t magic = 0;
  magic  = (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR)     << 24;
  magic |= (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR + 1) << 16;
  magic |= (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR + 2) << 8;
  magic |= (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR + 3);

  if (magic == EEPROM_MAGIC_VAL) {
    for (int i = 0; i < AES_KEY_SIZE; i++)
      netKey[i] = EEPROM.read(EEPROM_KEY_ADDR + i);
    for (int i = 0; i < MAC_SIZE; i++)
      emitterMAC[i] = EEPROM.read(EEPROM_MAC_ADDR + i);
    isPaired = true;

    Serial.print(F("[PAIR] Loaded from EEPROM. MAC="));
    for (int i = 0; i < MAC_SIZE; i++) {
      if (emitterMAC[i] < 0x10) Serial.print('0');
      Serial.print(emitterMAC[i], HEX);
    }
    Serial.print(F(" Key="));
    for (int i = 0; i < AES_KEY_SIZE; i++) {
      if (netKey[i] < 0x10) Serial.print('0');
      Serial.print(netKey[i], HEX);
    }
    Serial.println();
  } else {
    isPaired = false;
    Serial.println(F("[PAIR] No valid pairing in EEPROM"));
  }
}

void savePairingToEEPROM() {
  uint32_t magic = EEPROM_MAGIC_VAL;
  EEPROM.update(EEPROM_MAGIC_ADDR,     (magic >> 24) & 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 1, (magic >> 16) & 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 2, (magic >> 8)  & 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 3,  magic        & 0xFF);

  for (int i = 0; i < AES_KEY_SIZE; i++)
    EEPROM.update(EEPROM_KEY_ADDR + i, netKey[i]);
  for (int i = 0; i < MAC_SIZE; i++)
    EEPROM.update(EEPROM_MAC_ADDR + i, emitterMAC[i]);

  Serial.println(F("[PAIR] Pairing saved to EEPROM"));
}

void erasePairingFromEEPROM() {
  for (int i = 0; i < EEPROM_SIZE; i++)
    EEPROM.update(i, 0xFF);

  isPaired = false;
  memset(netKey, 0, AES_KEY_SIZE);
  memset(emitterMAC, 0, MAC_SIZE);

  Serial.println(F("[PAIR] Pairing erased from EEPROM"));
}

/* ====================================================================== */
/* ====== AES-128-CTR decryption + CMAC MIC (aes128.h) ====== */
/* ====================================================================== */

/**
 * Build 16-byte CTR nonce: MAC(8) + 0x00(7) + seq(1)
 * Identical to emitter vBuildNonce()
 */
void buildNonce(uint8_t *nonce, const uint8_t *mac, uint8_t seq) {
  memcpy(nonce, mac, 8);
  memset(&nonce[8], 0, 7);
  nonce[15] = seq;
}

/**
 * Decrypt a LoRa TIC packet in-place and verify MIC.
 *
 * Input: [type][seq][encrypted_payload...][MIC_hi][MIC_lo]
 * Output (if OK): [type][seq][decrypted_payload...] (MIC stripped)
 *
 * Returns true if MIC valid.
 */
bool decryptPacket(uint8_t *pkt, int totalLen, const uint8_t *key, const uint8_t *mac) {
  if (totalLen < 2 + CRYPT_MIC_SIZE) return false;

  int dataLen = totalLen - CRYPT_MIC_SIZE; /* length without MIC */
  uint8_t recvMIC0 = pkt[dataLen];
  uint8_t recvMIC1 = pkt[dataLen + 1];

  /* 1) Decrypt payload (skip type+seq = 2 bytes header) */
  uint8_t nonce[16];
  uint8_t seq = pkt[1];
  buildNonce(nonce, mac, seq);

  int payloadLen = dataLen - 2;
  if (payloadLen > 0) {
    aes128_ctr_crypt(&pkt[2], (uint16_t)payloadLen, key, nonce);
  }

  /* 2) Compute expected MIC over decrypted cleartext */
  uint8_t expectedMIC[16];
  aes128_cmac(pkt, (uint16_t)dataLen, key, expectedMIC);

  /* 3) Compare first 2 bytes */
  if (expectedMIC[0] == recvMIC0 && expectedMIC[1] == recvMIC1) {
    return true;
  }

  Serial.print(F("[CRYPT] MIC mismatch: got "));
  if (recvMIC0 < 0x10) Serial.print('0');
  Serial.print(recvMIC0, HEX);
  if (recvMIC1 < 0x10) Serial.print('0');
  Serial.print(recvMIC1, HEX);
  Serial.print(F(" expected "));
  if (expectedMIC[0] < 0x10) Serial.print('0');
  Serial.print(expectedMIC[0], HEX);
  if (expectedMIC[1] < 0x10) Serial.print('0');
  Serial.println(expectedMIC[1], HEX);

  return false;
}

/* ====================================================================== */
/* ====== Pairing protocol (receiver side) ====== */
/* ====================================================================== */

/**
 * Generate pseudo-random AES-128 key
 * Uses ADC noise + micros() as entropy source.
 * Acceptable: pairing key is sent in cleartext anyway (same model as Zigbee TC key)
 */
void generateRandomKey(uint8_t *key) {
  for (int i = 0; i < AES_KEY_SIZE; i++) {
    uint32_t entropy = 0;
    for (int b = 0; b < 8; b++) {
      entropy = (entropy << 1) | (analogRead(A0) & 1);
      delayMicroseconds(10);
    }
    entropy ^= micros();
    entropy ^= analogRead(A0);
    key[i] = (uint8_t)(entropy & 0xFF);
  }
}

/**
 * Send PAIR_RESPONSE packet (19 bytes):
 *   [0]     0x14 (v1, pair_response)
 *   [1]     seq (echo from request)
 *   [2-17]  netKey[0..15] AES-128 key
 *   [18]    status 0x00 = OK
 */
bool sendPairResponse(uint8_t seq) {
  uint8_t pkt[PAIR_RESPONSE_SIZE];

  pkt[0] = 0x14; /* v1 | PAIR_RESPONSE */
  pkt[1] = seq;
  memcpy(&pkt[2], netKey, AES_KEY_SIZE);
  pkt[18] = 0x00; /* OK */

  Serial.print(F("[PAIR] TX PAIR_RESPONSE seq="));
  Serial.print(seq);
  Serial.print(F(" Key="));
  for (int i = 0; i < AES_KEY_SIZE; i++) {
    if (netKey[i] < 0x10) Serial.print('0');
    Serial.print(netKey[i], HEX);
  }
  Serial.println();

  /* Switch to TX */
  radio.clearDio1Action();
  int state = radio.transmit(pkt, PAIR_RESPONSE_SIZE);

  /* Return to RX */
  radio.setDio1Action(rxDone);
  radio.startReceive(0xFFFF);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[PAIR] PAIR_RESPONSE sent OK"));
    return true;
  } else {
    Serial.print(F("[PAIR] PAIR_RESPONSE TX failed: "));
    Serial.println(state);
    return false;
  }
}

/**
 * Handle incoming PAIR_REQUEST
 * Extract MAC, generate key, send PAIR_RESPONSE
 */
void handlePairRequest(uint8_t *buf, int len) {
  if (len != PAIR_REQUEST_SIZE) {
    Serial.print(F("[PAIR] PAIR_REQUEST wrong size: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  uint8_t channel = buf[10];

  /* Extract emitter MAC */
  memcpy(emitterMAC, &buf[2], MAC_SIZE);

  Serial.print(F("[PAIR] PAIR_REQUEST from MAC="));
  for (int i = 0; i < MAC_SIZE; i++) {
    if (emitterMAC[i] < 0x10) Serial.print('0');
    Serial.print(emitterMAC[i], HEX);
  }
  Serial.print(F(" seq="));
  Serial.print(seq);
  Serial.print(F(" ch="));
  Serial.println(channel);

  /* Generate new AES-128 key */
  generateRandomKey(netKey);

  /* Send PAIR_RESPONSE */
  sendPairResponse(seq);

  /* Now wait for PAIR_CONFIRM (will be handled in next rxFlag) */
  Serial.println(F("[PAIR] Waiting for PAIR_CONFIRM..."));
}

/**
 * Handle incoming PAIR_CONFIRM
 * Verify AES-ECB proof, save to EEPROM if valid
 */
void handlePairConfirm(uint8_t *buf, int len) {
  if (len != PAIR_CONFIRM_SIZE) {
    Serial.print(F("[PAIR] PAIR_CONFIRM wrong size: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  uint8_t rxMAC[MAC_SIZE];
  memcpy(rxMAC, &buf[2], MAC_SIZE);
  uint8_t rxProof[4];
  memcpy(rxProof, &buf[10], 4);
  uint8_t status = buf[14];

  Serial.print(F("[PAIR] PAIR_CONFIRM seq="));
  Serial.print(seq);
  Serial.print(F(" status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" proof="));
  for (int i = 0; i < 4; i++) {
    if (rxProof[i] < 0x10) Serial.print('0');
    Serial.print(rxProof[i], HEX);
  }
  Serial.println();

  /* Verify MAC matches */
  if (memcmp(rxMAC, emitterMAC, MAC_SIZE) != 0) {
    Serial.println(F("[PAIR] PAIR_CONFIRM MAC mismatch — rejected"));
    return;
  }

  /* Verify AES-ECB proof: ECB(MAC padded to 16B, netKey) */
  uint8_t ecbInput[AES_BLOCK_SIZE];
  uint8_t ecbOutput[AES_BLOCK_SIZE];

  memset(ecbInput, 0, AES_BLOCK_SIZE);
  memcpy(ecbInput, emitterMAC, MAC_SIZE);

  aes128_ecb_encrypt(ecbInput, netKey, ecbOutput);

  /* Compare first 4 bytes */
  if (memcmp(rxProof, ecbOutput, 4) != 0) {
    Serial.print(F("[PAIR] Proof mismatch! Expected: "));
    for (int i = 0; i < 4; i++) {
      if (ecbOutput[i] < 0x10) Serial.print('0');
      Serial.print(ecbOutput[i], HEX);
    }
    Serial.println();
    return;
  }

  /* SUCCESS — save pairing */
  Serial.println(F("[PAIR] *** PAIRING SUCCESSFUL ***"));
  isPaired = true;
  rxPairState = RX_PAIRED;
  savePairingToEEPROM();

  Serial.print(F("[PAIR] Paired with MAC="));
  for (int i = 0; i < MAC_SIZE; i++) {
    if (emitterMAC[i] < 0x10) Serial.print('0');
    Serial.print(emitterMAC[i], HEX);
  }
  Serial.println();
}

/* ====================================================================== */
/* ====== Parse DATA_ESSENTIAL (17B cleartext, 19B with MIC) ====== */
/* ====================================================================== */
void parseEssential(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_ESSENTIAL_SIZE) {
    Serial.print(F("[RX] Essential taille inattendue: "));
    Serial.print(len);
    Serial.print(F("B  HEX: "));
    hexDump(buf, len);
    return;
  }

  uint8_t  seq     = buf[1];
  uint16_t sinsts  = getU16(&buf[2]);
  uint16_t sinsts1 = getU16(&buf[4]);
  uint16_t sinsts2 = getU16(&buf[6]);
  uint16_t sinsts3 = getU16(&buf[8]);
  /* STGE = 4 bytes binaire [10-13] */
  uint16_t adps    = getU16(&buf[14]);
  uint8_t  mode    = buf[16];

  trackSequence(seq);

  Serial.println(F("========== LINKY ESSENTIAL =========="));

  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.print(seq);
  Serial.print(F("  Mode: "));
  Serial.println(modeStr(mode));

  Serial.print(F("  SINSTS : "));
  Serial.print(sinsts);
  Serial.println(F(" VA (total)"));

  if (isTri(mode)) {
    Serial.print(F("  SINSTS1: "));
    Serial.print(sinsts1);
    Serial.print(F("  SINSTS2: "));
    Serial.print(sinsts2);
    Serial.print(F("  SINSTS3: "));
    Serial.print(sinsts3);
    Serial.println(F(" VA"));
  } else {
    Serial.print(F("  SINSTS1: "));
    Serial.print(sinsts1);
    Serial.println(F(" VA"));
  }

  /* STGE (4 bytes binaire -> affichage hex) */
  Serial.print(F("  STGE   : "));
  for (int i = 10; i < 14; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
  }
  Serial.println();

  if (adps > 0) {
    Serial.print(F("  ADPS   : "));
    Serial.print(adps);
    Serial.println(F(" VA ***ALERTE***"));
  }

  printStats(rssi, snr);
  Serial.println(F("=====================================\r\n"));
}

/* ====== Parse EXT 0x00 : CURRENT_VOLTAGE (15B) ====== */
void parseExtCurrentVoltage(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_IV_SIZE) {
    Serial.print(F("[EXT] CurrentVoltage taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint16_t irms1 = getU16(&buf[3]);
  uint16_t irms2 = getU16(&buf[5]);
  uint16_t irms3 = getU16(&buf[7]);
  uint16_t urms1 = getU16(&buf[9]);
  uint16_t urms2 = getU16(&buf[11]);
  uint16_t urms3 = getU16(&buf[13]);

  Serial.println(F("---------- EXT: CURRENT_VOLTAGE ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  IRMS1  : "));
  Serial.print(irms1);
  Serial.print(F("  IRMS2: "));
  Serial.print(irms2);
  Serial.print(F("  IRMS3: "));
  Serial.print(irms3);
  Serial.println(F(" mA"));

  Serial.print(F("  URMS1  : "));
  Serial.print(urms1);
  Serial.print(F("  URMS2: "));
  Serial.print(urms2);
  Serial.print(F("  URMS3: "));
  Serial.print(urms3);
  Serial.println(F(" V"));

  printStats(rssi, snr);
  Serial.println(F("------------------------------------------\r\n"));
}

/* ====== Parse EXT 0x01 : ENERGY (19B) ====== */
void parseExtEnergy(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_NRG_SIZE) {
    Serial.print(F("[EXT] Energy taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint32_t east   = getU32(&buf[3]);
  uint32_t eait   = getU32(&buf[7]);
  uint32_t easf01 = getU32(&buf[11]);
  uint32_t easf02 = getU32(&buf[15]);

  Serial.println(F("---------- EXT: ENERGY ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  EAST   : "));
  Serial.print(east);
  Serial.print(F(" Wh ("));
  Serial.print(east / 1000.0, 3);
  Serial.println(F(" kWh)"));

  Serial.print(F("  EAIT   : "));
  Serial.print(eait);
  Serial.print(F(" Wh ("));
  Serial.print(eait / 1000.0, 3);
  Serial.println(F(" kWh) [injection]"));

  Serial.print(F("  EASF01 : "));
  Serial.print(easf01);
  Serial.print(F(" Wh ("));
  Serial.print(easf01 / 1000.0, 3);
  Serial.println(F(" kWh)"));

  Serial.print(F("  EASF02 : "));
  Serial.print(easf02);
  Serial.print(F(" Wh ("));
  Serial.print(easf02 / 1000.0, 3);
  Serial.println(F(" kWh)"));

  printStats(rssi, snr);
  Serial.println(F("---------------------------------\r\n"));
}

/* ====== Parse EXT 0x02 : ENERGY_2 (19B) ====== */
void parseExtEnergy2(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_NRG2_SIZE) {
    Serial.print(F("[EXT] Energy2 taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint32_t easf03 = getU32(&buf[3]);
  uint32_t easf04 = getU32(&buf[7]);
  uint32_t easf05 = getU32(&buf[11]);
  uint32_t easf06 = getU32(&buf[15]);

  Serial.println(F("---------- EXT: ENERGY_2 ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  EASF03 : "));
  Serial.print(easf03);
  Serial.print(F(" Wh ("));
  Serial.print(easf03 / 1000.0, 3);
  Serial.println(F(" kWh)"));

  Serial.print(F("  EASF04 : "));
  Serial.print(easf04);
  Serial.print(F(" Wh ("));
  Serial.print(easf04 / 1000.0, 3);
  Serial.println(F(" kWh)"));

  Serial.print(F("  EASF05 : "));
  Serial.print(easf05);
  Serial.print(F(" Wh ("));
  Serial.print(easf05 / 1000.0, 3);
  Serial.println(F(" kWh)"));

  Serial.print(F("  EASF06 : "));
  Serial.print(easf06);
  Serial.print(F(" Wh ("));
  Serial.print(easf06 / 1000.0, 3);
  Serial.println(F(" kWh)"));

  printStats(rssi, snr);
  Serial.println(F("------------------------------------\r\n"));
}

/* ====== Parse EXT 0x03 : VOLTAGE_STATS (15B) ====== */
void parseExtVoltageStats(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_VSTAT_SIZE) {
    Serial.print(F("[EXT] VoltageStats taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint16_t umoy1 = getU16(&buf[3]);
  uint16_t umoy2 = getU16(&buf[5]);
  uint16_t umoy3 = getU16(&buf[7]);
  uint16_t imax1 = getU16(&buf[9]);
  uint16_t imax2 = getU16(&buf[11]);
  uint16_t imax3 = getU16(&buf[13]);

  Serial.println(F("---------- EXT: VOLTAGE_STATS ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  UMOY1  : "));
  Serial.print(umoy1);
  Serial.print(F("  UMOY2: "));
  Serial.print(umoy2);
  Serial.print(F("  UMOY3: "));
  Serial.print(umoy3);
  Serial.println(F(" V"));

  Serial.print(F("  IMAX1  : "));
  Serial.print(imax1);
  Serial.print(F("  IMAX2: "));
  Serial.print(imax2);
  Serial.print(F("  IMAX3: "));
  Serial.print(imax3);
  Serial.println(F(" mA"));

  printStats(rssi, snr);
  Serial.println(F("----------------------------------------\r\n"));
}

/* ====== Parse EXT 0x04 : POWER_MAX_CFG (18B) ====== */
void parseExtPowerMaxCfg(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_PMCFG_SIZE) {
    Serial.print(F("[EXT] PowerMaxCfg taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint16_t smaxsn = getU16(&buf[3]);
  uint16_t smaxin = getU16(&buf[5]);
  uint16_t sinsti = getU16(&buf[7]);
  uint16_t ccain  = getU16(&buf[9]);
  uint16_t relais = getU16(&buf[11]);
  uint8_t  ntarf  = buf[13];
  uint8_t  pcoup  = buf[14];
  uint8_t  pref   = buf[15];
  uint8_t  mode   = buf[16];

  Serial.println(F("---------- EXT: POWER_MAX_CFG ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  SMAXSN : "));
  Serial.print(smaxsn);
  Serial.println(F(" VA (soutire max jour)"));

  Serial.print(F("  SMAXIN : "));
  Serial.print(smaxin);
  Serial.println(F(" VA (injecte max jour)"));

  Serial.print(F("  SINSTI : "));
  Serial.print(sinsti);
  Serial.println(F(" VA (instantane)"));

  Serial.print(F("  CCAIN  : "));
  Serial.print(ccain);
  Serial.println(F(" VA (courbe charge)"));

  Serial.print(F("  RELAIS : 0x"));
  if (relais < 0x1000) Serial.print('0');
  if (relais < 0x100)  Serial.print('0');
  if (relais < 0x10)   Serial.print('0');
  Serial.println(relais, HEX);

  Serial.print(F("  NTARF  : "));
  Serial.println(ntarf);

  Serial.print(F("  PCOUP  : "));
  Serial.print(pcoup);
  Serial.println(F(" kVA"));

  Serial.print(F("  PREF   : "));
  Serial.print(pref);
  Serial.println(F(" kVA"));

  Serial.print(F("  Mode   : "));
  Serial.println(modeStr(mode));

  printStats(rssi, snr);
  Serial.println(F("----------------------------------------\r\n"));
}

/* ====== Parse EXT 0x05 : DAILY_ENERGY (19B) ====== */
void parseExtDailyEnergy(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_DAILY_SIZE) {
    Serial.print(F("[EXT] DailyEnergy taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint32_t easd01 = getU32(&buf[3]);
  uint32_t easd02 = getU32(&buf[7]);
  uint32_t easd03 = getU32(&buf[11]);
  uint32_t easd04 = getU32(&buf[15]);

  Serial.println(F("---------- EXT: DAILY_ENERGY ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  EASD01 : "));
  Serial.print(easd01);
  Serial.print(F(" Wh ("));
  Serial.print(easd01 / 1000.0, 3);
  Serial.println(F(" kWh) [jour J]"));

  Serial.print(F("  EASD02 : "));
  Serial.print(easd02);
  Serial.print(F(" Wh ("));
  Serial.print(easd02 / 1000.0, 3);
  Serial.println(F(" kWh) [jour J-1]"));

  Serial.print(F("  EASD03 : "));
  Serial.print(easd03);
  Serial.print(F(" Wh ("));
  Serial.print(easd03 / 1000.0, 3);
  Serial.println(F(" kWh) [jour J-2]"));

  Serial.print(F("  EASD04 : "));
  Serial.print(easd04);
  Serial.print(F(" Wh ("));
  Serial.print(easd04 / 1000.0, 3);
  Serial.println(F(" kWh) [jour J-3]"));

  printStats(rssi, snr);
  Serial.println(F("---------------------------------------\r\n"));
}

/* ====== Parse EXT 0x06 : COMPLEMENT (9B) ====== */
void parseExtComplement(uint8_t *buf, int len, float rssi, float snr) {
  if (len != PKT_EXT_COMP_SIZE) {
    Serial.print(F("[EXT] Complement taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq = buf[1];
  trackSequence(seq);

  uint8_t dpm1 = buf[3];
  uint8_t fpm1 = buf[4];
  uint8_t dpm2 = buf[5];
  uint8_t fpm2 = buf[6];
  uint8_t dpm3 = buf[7];
  uint8_t fpm3 = buf[8];

  Serial.println(F("---------- EXT: COMPLEMENT ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  DPM1   : "));
  Serial.print(dpm1);
  Serial.print(F("  FPM1: "));
  Serial.println(fpm1);

  Serial.print(F("  DPM2   : "));
  Serial.print(dpm2);
  Serial.print(F("  FPM2: "));
  Serial.println(fpm2);

  Serial.print(F("  DPM3   : "));
  Serial.print(dpm3);
  Serial.print(F("  FPM3: "));
  Serial.println(fpm3);

  printStats(rssi, snr);
  Serial.println(F("-------------------------------------\r\n"));
}

/* ====================================================================== */
/* ====== Dispatch principal ====== */
/* ====================================================================== */
void parseAndPrint(uint8_t *buf, int len, float rssi, float snr) {
  if (len < 2) {
    Serial.print(F("[RX] Paquet trop court: "));
    Serial.print(len);
    Serial.print(F("B  HEX: "));
    hexDump(buf, len);
    return;
  }

  uint8_t version = (buf[0] >> 4) & 0x0F;
  uint8_t type    = buf[0] & 0x0F;

  if (version != 0x01) {
    Serial.print(F("[RX] Version inconnue: 0x"));
    Serial.println(buf[0], HEX);
    rxErrors++;
    return;
  }

  /* ---- Pairing packets (not encrypted) ---- */
  if (type == PKT_TYPE_PAIR_REQUEST) {
    if (rxPairState == RX_PAIR_LISTEN) {
      handlePairRequest(buf, len);
    } else {
      Serial.println(F("[RX] PAIR_REQUEST ignored (long press USER button to enter pairing)"));
    }
    return;
  }

  if (type == PKT_TYPE_PAIR_CONFIRM) {
    if (rxPairState == RX_PAIR_LISTEN) {
      handlePairConfirm(buf, len);
    } else {
      Serial.println(F("[RX] PAIR_CONFIRM ignored (not in listen mode)"));
    }
    return;
  }

  /* ---- Data packets (must be paired + decrypted) ---- */
  if (!isPaired) {
    Serial.print(F("[RX] Data packet ignored (not paired). type=0x"));
    Serial.print(type, HEX);
    Serial.print(F(" len="));
    Serial.print(len);
    Serial.print(F(" HEX: "));
    hexDump(buf, len);
    return;
  }

  /* Decrypt + verify MIC */
  if (len < 4) { /* minimum: type + seq + 0 payload + 2 MIC */
    Serial.print(F("[RX] Packet too short for decrypt: "));
    Serial.println(len);
    rxErrors++;
    return;
  }

  if (!decryptPacket(buf, len, netKey, emitterMAC)) {
    Serial.print(F("[RX] Decrypt/MIC failed! len="));
    Serial.print(len);
    Serial.print(F(" HEX: "));
    hexDump(buf, len);
    rxErrors++;
    return;
  }

  /* After decryption: len without MIC = actual data length */
  int dataLen = len - CRYPT_MIC_SIZE;

  if (type == PKT_TYPE_ESSENTIAL) {
    parseEssential(buf, dataLen, rssi, snr);
  }
  else if (type == PKT_TYPE_EXTENDED) {
    if (dataLen < 3) {
      Serial.print(F("[RX] Extended trop court: "));
      Serial.println(dataLen);
      rxErrors++;
      return;
    }
    uint8_t subType = buf[2];
    switch (subType) {
      case EXT_CURRENT_VOLTAGE: parseExtCurrentVoltage(buf, dataLen, rssi, snr); break;
      case EXT_ENERGY:          parseExtEnergy(buf, dataLen, rssi, snr);         break;
      case EXT_ENERGY_2:        parseExtEnergy2(buf, dataLen, rssi, snr);        break;
      case EXT_VOLTAGE_STATS:   parseExtVoltageStats(buf, dataLen, rssi, snr);   break;
      case EXT_POWER_MAX_CFG:   parseExtPowerMaxCfg(buf, dataLen, rssi, snr);    break;
      case EXT_DAILY_ENERGY:    parseExtDailyEnergy(buf, dataLen, rssi, snr);   break;
      case EXT_COMPLEMENT:      parseExtComplement(buf, dataLen, rssi, snr);    break;
      default:
        Serial.print(F("[RX] Extended sub-type inconnu: 0x"));
        Serial.println(subType, HEX);
        Serial.print(F("  HEX: "));
        hexDump(buf, dataLen);
        trackSequence(buf[1]);
        break;
    }
  }
  else {
    Serial.print(F("[RX] Type inconnu: 0x"));
    Serial.println(buf[0], HEX);
    Serial.print(F("  HEX: "));
    hexDump(buf, dataLen);
    rxErrors++;
  }
}

/* ====================================================================== */
/* ====== Setup ====== */
/* ====================================================================== */
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("\r\n[RX] ZLinky TIC Receiver SX1280"));
  Serial.println(F("[RX] 2490MHz SF10 BW406 CR4/5"));
  Serial.println(F("[RX] Essential 1s + Extended piggyback 10s"));
  Serial.println(F("[RX] AES-128-CTR + CMAC MIC enabled"));
  Serial.println(F("[RX] Max 22 octets/paquet (SF10 + MIC)\r\n"));

  /* Load pairing from EEPROM */
  loadPairingFromEEPROM();

  /* Init radio */
  int state = radio.begin(2490.0, 406.25, 10, 5, 0x12, 10, 16);
  Serial.print(F("Init: "));
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("FAIL code "));
    Serial.println(state);
    while (true) delay(1000);
  }
  Serial.println(F("OK"));

  radio.setCRC(2);
  radio.explicitHeader();
  radio.invertIQ(false);

  radio.setDio1Action(rxDone);

  state = radio.startReceive(0xFFFF);
  Serial.print(F("startReceive(continuous): "));
  Serial.println(state == RADIOLIB_ERR_NONE ? F("OK") : String(state));

  /* Determine initial state (pairing only via long press on USER button) */
  if (isPaired) {
    rxPairState = RX_PAIRED;
    Serial.println(F("[PAIR] Already paired — receiving encrypted data"));
  } else {
    rxPairState = RX_IDLE;
    Serial.println(F("[PAIR] Not paired — long press USER button (3s) to enter pairing mode"));
  }

  /* Init bouton USER (B1 = PC13, pull-up interne, actif LOW) */
  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
  Serial.println(F("[BTN] Bouton USER (B1) : appui long 3s = entrer mode pairing 30s"));

  Serial.println(F("Attente paquets...\r\n"));
}

/* ====================================================================== */
/* ====== Loop ====== */
/* ====================================================================== */
void loop() {
  /* ---- Bouton USER : appui long (3s) = entrer mode PAIR_LISTEN 30s ---- */
  bool btnState = digitalRead(USER_BUTTON_PIN);

  if (btnState == LOW && lastButtonState == HIGH) {
    /* Front descendant : debut appui */
    buttonPressStart = millis();
    buttonLongDetected = false;
  }

  if (btnState == LOW && !buttonLongDetected) {
    /* Bouton maintenu : verifier duree */
    if ((millis() - buttonPressStart) >= BUTTON_LONG_PRESS_MS) {
      buttonLongDetected = true;

      if (rxPairState != RX_PAIR_LISTEN) {
        Serial.println(F("[BTN] Appui long detecte — passage en PAIR_LISTEN (30s)"));
        rxPairState = RX_PAIR_LISTEN;
        pairListenStart = millis();
        isPaired = false;  /* Force re-pairing */
      } else {
        Serial.println(F("[BTN] Deja en mode PAIR_LISTEN"));
      }
    }
  }

  lastButtonState = btnState;

  /* Check pairing timeout */
  if (rxPairState == RX_PAIR_LISTEN) {
    if (millis() - pairListenStart >= PAIR_LISTEN_TIMEOUT_MS) {
      if (isPaired) {
        rxPairState = RX_PAIRED;
        Serial.println(F("[PAIR] Pairing window expired (30s). Returning to PAIRED mode."));
      } else {
        rxPairState = RX_IDLE;
        Serial.println(F("[PAIR] Pairing window expired (30s). Not paired."));
        Serial.println(F("[PAIR] Press USER button or reboot to re-enter pairing mode.\r\n"));
      }
    }
  }

  if (rxFlag) {
    rxFlag = false;

    uint8_t buffer[255];
    int len = radio.getPacketLength();
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();
    int state = radio.readData(buffer, len);

    if (state == RADIOLIB_ERR_NONE) {
      consecutiveCrcErrors = 0;
      rxSinceReset++;
      parseAndPrint(buffer, len, rssi, snr);

      if (snr < SNR_RESET_THRESHOLD) {
        doFullReset("SNR drift");
        return;
      }

      if (rxSinceReset >= PREVENTIVE_RESET_INTERVAL) {
        doFullReset("preventive");
        return;
      }

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      rxErrors++;
      consecutiveCrcErrors++;
      Serial.print(F("[CRC ERROR] len="));
      Serial.print(len);
      Serial.print(F(" RSSI="));
      Serial.print(rssi, 0);
      Serial.print(F(" SNR="));
      Serial.print(snr, 1);
      Serial.print(F(" ("));
      Serial.print(consecutiveCrcErrors);
      Serial.print(F("/"));
      Serial.print(CRC_ERROR_RESET_THRESHOLD);
      Serial.print(F(") HEX: "));
      hexDump(buffer, len);
    } else {
      rxErrors++;
      consecutiveCrcErrors++;
      Serial.print(F("[RX ERROR] code "));
      Serial.println(state);
    }

    if (consecutiveCrcErrors >= CRC_ERROR_RESET_THRESHOLD) {
      doFullReset("CRC errors");
      return;
    }

    radio.startReceive(0xFFFF);
  }
}

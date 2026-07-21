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
 * Config LoRa : 8 canaux ISM 2410-2480 MHz, SF11, BW406.25, CR4/5
 * Preamble 16sym, CRC on, IQ normal, SyncWord 0x12
 * Max 22+2 octets par paquet (payload + MIC)
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
/* SF11 LoRa demod limit ≈ -17.5 dB SNR. En portée limite, le SNR est
 * normalement bas, ce n'est PAS un signe de drift. On set le seuil bien sous le
 * demod limit pour ne pas reset inutilement. Set à -100 pour desactiver. */
#define SNR_RESET_THRESHOLD       -25.0

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
/* Tariff Label : taille variable 6..22 (header 6 + libelle 0..16) */
#define PKT_EXT_TARIF_HDR    6
#define PKT_EXT_TARIF_MAXLEN 16

/* MIC size */
#define CRYPT_MIC_SIZE  2

/* Tailles paquets pairing */
#define PAIR_REQUEST_SIZE   11
#define PAIR_RESPONSE_SIZE  21   /* + op_channel [19] + op_SF [20] */
#define PAIR_CONFIRM_SIZE   15

/* ====== LoRa channel plan (must match emitter firmware) ====== */
#define LORA_NUM_CHANNELS  8
#define LORA_PAIR_CHANNEL  3   /* 2440 MHz - canal pour pairing handshake */
const float LORA_CHANNEL_FREQS_MHZ[LORA_NUM_CHANNELS] = {
  2410.0, 2420.0, 2430.0, 2440.0, 2450.0, 2460.0, 2470.0, 2480.0
};

/* === DEFAULT_OP_CHANNEL ===
 *
 * Choix du canal operationnel quand pas de pairing en EEPROM :
 *   0..7 : forcer ce canal (skip le CCA scan)
 *   -1   : auto-detection par CCA scan (le plus calme)
 *
 * Si tu connais ton environnement RF (e.g., WiFi fixe sur ch 6),
 * c'est preferable de FORCER un canal pour avoir un comportement
 * deterministe. Mets -1 pour laisser le firmware choisir.
 *
 * Note: une fois paire, le canal est sauvegarde en EEPROM et reste
 * actif pour tous les emetteurs futurs (jusqu'a un re-pairing complet).
 */
#define DEFAULT_OP_CHANNEL  4   /* 2450 MHz par defaut */
//#define DEFAULT_OP_CHANNEL  -1  /* Uncomment pour CCA scan auto */

uint8_t opChannel = LORA_PAIR_CHANNEL;
bool isChannelInitialized = false;

/* Spreading factor operationnel du reseau (7..12). Assigne aux ZLinky lors de
 * l'appairage (PAIR_RESPONSE byte[20]). L'appairage lui-meme se fait toujours
 * a SF11 (rendez-vous fixe), les donnees a opSF. */
#define PAIR_SF     11    /* SF du rendez-vous d'appairage (fixe) */
#define SF_MIN       7
#define SF_MAX      12
uint8_t opSF = 11;

/* Sous-types extended (byte[2]) */
#define EXT_CURRENT_VOLTAGE  0x00
#define EXT_ENERGY           0x01
#define EXT_ENERGY_2         0x02
#define EXT_VOLTAGE_STATS    0x03
#define EXT_POWER_MAX_CFG    0x04
#define EXT_DAILY_ENERGY     0x05
#define EXT_COMPLEMENT       0x06
#define EXT_TARIFF_LABEL     0x07

/* AES sizes */
#define AES_KEY_SIZE    16
#define AES_BLOCK_SIZE  16
#define MAC_SIZE         8

/* ====== Multi-emitter pairing ====== */
#define MAX_PAIRED_EMITTERS  4

struct PairedEmitter {
  bool     valid;
  uint8_t  mac[MAC_SIZE];
  uint8_t  key[AES_KEY_SIZE];
  /* Per-emitter stats (in RAM only, not in EEPROM) */
  uint32_t rxCount;
  uint32_t errCount;
  uint32_t missedPackets;
  uint8_t  lastSeq;
  bool     seqInit;
  /* Contexte Linky memorise : permet aux parsers EXT (qui ne portent pas le
   * mode) d'interpreter correctement les index Tier1..Tier6.
   * Mis a jour par parseEssential (mode) et parseExtPowerMaxCfg (mode+tarif). */
  uint8_t  lastMode;    /* 0=Histo Mono, 1=Std Mono, 2=Histo Tri, 3=Std Tri... */
  uint8_t  lastTarif;   /* code LORA_TARIF_* — 0 = inconnu */
  bool     modeKnown;
  /* Libelle texte du tarif (LTARF en Standard, PTEC en Historique),
   * recu via le sous-type EXT_TARIFF_LABEL. Chaine C nul-terminee. */
  char     tarifLabel[PKT_EXT_TARIF_MAXLEN + 1];
};
PairedEmitter emitters[MAX_PAIRED_EMITTERS];
int currentPairingSlot = -1;  /* slot used during a pairing handshake */

/* ====== Lecture d'attribut a la demande (POLL) ======
 * Une requete par emetteur, en attente jusqu'a ce qu'on entende son prochain
 * uplink : c'est le seul moment ou sa fenetre de reception est ouverte. */
#define PKT_TYPE_POLL_REQUEST   0x0B
#define PKT_TYPE_POLL_RESPONSE  0x0C
#define POLL_RESP_TIMEOUT_MS    600

struct PendingPoll {
  bool     valid;
  uint16_t cluster;
  uint16_t attr;
};
PendingPoll pendingPoll[MAX_PAIRED_EMITTERS];
uint8_t pollSeq = 0;

/* ====== EEPROM layout (multi-emitter) ====== */
/* Bumped magic: 0x4C505251 ("LPRQ") to invalidate single-emitter records */
#define EEPROM_SIZE        256   /* enough for headers + 4 emitter records */
#define EEPROM_MAGIC_ADDR    0   /* uint32 magic (4B) */
#define EEPROM_OPCH_ADDR     4   /* op_channel (1B) */
#define EEPROM_COUNT_ADDR    5   /* number of valid emitter records (1B) */
#define EEPROM_OPSF_ADDR     6   /* op_SF (1B) — ex-reserved */
#define EEPROM_RESERVED_ADDR 7   /* 1B reserved */
#define EEPROM_EMITTERS_BASE 8   /* start of emitter records, 25 bytes each */
#define EEPROM_EMITTER_SIZE  25  /* 1 (valid) + 8 (mac) + 16 (key) */
#define EEPROM_MAGIC_VAL     0x4C505251  /* "LPRQ" */

/* ====== OTA Protocol (must match emitter app_lora_ota.h) ====== */
#define LORA_TIC_PKT_OTA_START   0x06
#define LORA_TIC_PKT_OTA_BLOCK   0x07
#define LORA_TIC_PKT_OTA_END     0x08
#define LORA_TIC_PKT_OTA_ACK     0x09
#define LORA_TIC_PKT_OTA_STATUS  0x0A

#define OTA_ACK_TIMEOUT_MS       5000

/* ====== Pairing state machine ====== */
typedef enum {
  RX_PAIR_LISTEN = 0,    /* 30s au boot, attend PAIR_REQUEST */
  RX_PAIRED,             /* Appaire, recoit donnees chiffrees */
  RX_IDLE,               /* Pas appaire, hors fenetre pairing */
  RX_OTA_GATEWAY         /* Mode gateway OTA : pilote par commandes serie */
} eRxPairState_t;

/* OTA gateway context */
uint8_t  otaGwTargetMAC[MAC_SIZE];
uint8_t  otaGwTargetKey[AES_KEY_SIZE];
bool     otaGwInitialized = false;

eRxPairState_t rxPairState = RX_IDLE;
uint32_t pairListenStart = 0;
#define PAIR_LISTEN_TIMEOUT_MS  30000

/* ====== Bouton USER (B1) sur STM32 Nucleo-64 = PC13 ====== */
#define USER_BUTTON_PIN  PC13
bool lastButtonState = HIGH;  /* Pull-up, actif LOW */
uint32_t buttonPressStart = 0;
bool buttonLongDetected = false;
#define BUTTON_LONG_PRESS_MS  3000  /* 3s appui long pour entrer en pairing */

/* Legacy "current pair in progress" - used during handshake (PAIR_REQUEST → PAIR_CONFIRM).
 * After PAIR_CONFIRM success, contents are copied into the emitters[] array. */
uint8_t netKey[AES_KEY_SIZE];
uint8_t emitterMAC[MAC_SIZE];
bool isPaired = false;  /* TRUE if at least one emitter in emitters[] is valid */

/* === Multi-emitter helpers === */

/* Count valid entries in emitters[] */
int countPairedEmitters() {
  int n = 0;
  for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) if (emitters[i].valid) n++;
  return n;
}

/* Find emitter index by MAC, or -1 if not found */
int findEmitterByMAC(const uint8_t *mac) {
  for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) {
    if (emitters[i].valid && memcmp(emitters[i].mac, mac, MAC_SIZE) == 0)
      return i;
  }
  return -1;
}

/* Find empty slot, or -1 if all full */
int findEmptySlot() {
  for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) {
    if (!emitters[i].valid) return i;
  }
  return -1;
}

/* Add or update a pairing in the array.
 * If MAC already known: update its key.
 * If new MAC: take an empty slot.
 * Returns index, or -1 if full. */
int addOrUpdateEmitter(const uint8_t *mac, const uint8_t *key) {
  int idx = findEmitterByMAC(mac);
  if (idx < 0) idx = findEmptySlot();
  if (idx < 0) return -1;  /* full */

  emitters[idx].valid = true;
  memcpy(emitters[idx].mac, mac, MAC_SIZE);
  memcpy(emitters[idx].key, key, AES_KEY_SIZE);
  emitters[idx].rxCount = 0;
  emitters[idx].errCount = 0;
  emitters[idx].missedPackets = 0;
  emitters[idx].seqInit = false;
  emitters[idx].lastMode = 1;      /* Std Mono par defaut */
  emitters[idx].lastTarif = 0;     /* TARIF_UNKNOWN */
  emitters[idx].modeKnown = false; /* resolu au 1er ESSENTIAL recu */
  emitters[idx].tarifLabel[0] = '\0';
  return idx;
}

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

/* Mode Standard = bit0 a 1 (1=Std Mono, 3=Std Tri, 5/7=Std Prod) */
bool isStandard(uint8_t mode) {
  return (mode & 0x01) != 0;
}

/* ====== Codes tarif (doit matcher LORA_TARIF_* dans app_lora_tic.h) ====== */
#define TARIF_UNKNOWN              0
#define TARIF_HISTO_BASE           1
#define TARIF_HISTO_HP             2
#define TARIF_HISTO_HC             3
#define TARIF_HISTO_EJP_HN         4
#define TARIF_HISTO_EJP_PM         5
#define TARIF_HISTO_TEMPO_BLEU_HC  6
#define TARIF_HISTO_TEMPO_BLEU_HP  7
#define TARIF_HISTO_TEMPO_BLANC_HC 8
#define TARIF_HISTO_TEMPO_BLANC_HP 9
#define TARIF_HISTO_TEMPO_ROUGE_HC 10
#define TARIF_HISTO_TEMPO_ROUGE_HP 11

/* Option tarifaire deduite du tarif courant (mode Historique).
 * NB: #define plutot qu'un enum — l'IDE Arduino genere les prototypes de
 * fonctions AVANT les declarations de types du sketch, un type de retour
 * custom casserait la compilation. */
#define OPT_UNKNOWN 0
#define OPT_BASE    1
#define OPT_HCHP    2
#define OPT_EJP     3
#define OPT_TEMPO   4

uint8_t histoOptarifFromTarif(uint8_t tarif) {
  switch (tarif) {
    case TARIF_HISTO_BASE:                                  return OPT_BASE;
    case TARIF_HISTO_HP:
    case TARIF_HISTO_HC:                                    return OPT_HCHP;
    case TARIF_HISTO_EJP_HN:
    case TARIF_HISTO_EJP_PM:                                return OPT_EJP;
    case TARIF_HISTO_TEMPO_BLEU_HC:
    case TARIF_HISTO_TEMPO_BLEU_HP:
    case TARIF_HISTO_TEMPO_BLANC_HC:
    case TARIF_HISTO_TEMPO_BLANC_HP:
    case TARIF_HISTO_TEMPO_ROUGE_HC:
    case TARIF_HISTO_TEMPO_ROUGE_HP:                        return OPT_TEMPO;
    default:                                                return OPT_UNKNOWN;
  }
}

/* Libelle du tarif en cours, selon mode + code tarif */
const char* tarifStr(uint8_t mode, uint8_t tarif) {
  if (isStandard(mode)) {
    return "";  /* en Standard, tarif = index NTARF, affiche numeriquement */
  }
  switch (tarif) {
    case TARIF_HISTO_BASE:           return "BASE";
    case TARIF_HISTO_HP:             return "HP";
    case TARIF_HISTO_HC:             return "HC";
    case TARIF_HISTO_EJP_HN:         return "EJP Heures Normales";
    case TARIF_HISTO_EJP_PM:         return "EJP Pointe Mobile";
    case TARIF_HISTO_TEMPO_BLEU_HC:  return "Tempo Bleu HC";
    case TARIF_HISTO_TEMPO_BLEU_HP:  return "Tempo Bleu HP";
    case TARIF_HISTO_TEMPO_BLANC_HC: return "Tempo Blanc HC";
    case TARIF_HISTO_TEMPO_BLANC_HP: return "Tempo Blanc HP";
    case TARIF_HISTO_TEMPO_ROUGE_HC: return "Tempo Rouge HC";
    case TARIF_HISTO_TEMPO_ROUGE_HP: return "Tempo Rouge HP";
    default:                         return "?";
  }
}

/* Libelle de l'index Tier N (1..6) selon le mode et l'option tarifaire.
 *
 * L'emetteur remplit les memes champs ZCL Tier1..Tier6 dans les deux modes
 * (cf. App_Linky.c) : c'est ici qu'on retrouve le nom TIC d'origine.
 *
 *   Standard    : Tier1..Tier6 = EASF01..EASF06
 *   Histo BASE  : Tier1 = BASE
 *   Histo HC/HP : Tier1 = HCHC, Tier2 = HCHP
 *   Histo EJP   : Tier1 = EJPHN, Tier2 = EJPHPM
 *   Histo Tempo : Tier1..6 = BBRHCJB, BBRHPJB, BBRHCJW, BBRHPJW, BBRHCJR, BBRHPJR
 *
 * Retourne NULL si l'index n'a pas de sens dans cette configuration
 * (ex: Tier3 en option BASE) — l'appelant peut alors masquer la ligne.
 */
const char* tierLabel(uint8_t mode, uint8_t tarif, uint8_t tierIdx) {
  if (isStandard(mode)) {
    switch (tierIdx) {
      case 1: return "EASF01";
      case 2: return "EASF02";
      case 3: return "EASF03";
      case 4: return "EASF04";
      case 5: return "EASF05";
      case 6: return "EASF06";
      default: return NULL;
    }
  }

  switch (histoOptarifFromTarif(tarif)) {
    case OPT_BASE:
      return (tierIdx == 1) ? "BASE" : NULL;

    case OPT_HCHP:
      if (tierIdx == 1) return "HCHC";
      if (tierIdx == 2) return "HCHP";
      return NULL;

    case OPT_EJP:
      if (tierIdx == 1) return "EJPHN";
      if (tierIdx == 2) return "EJPHPM";
      return NULL;

    case OPT_TEMPO:
      switch (tierIdx) {
        case 1: return "BBRHCJB";
        case 2: return "BBRHPJB";
        case 3: return "BBRHCJW";
        case 4: return "BBRHPJW";
        case 5: return "BBRHCJR";
        case 6: return "BBRHPJR";
        default: return NULL;
      }

    default:
      /* Option inconnue (pas encore recu de POWER_MAX_CFG) : on affiche
       * le nom generique plutot que de masquer la donnee. */
      switch (tierIdx) {
        case 1: return "Tier1";
        case 2: return "Tier2";
        case 3: return "Tier3";
        case 4: return "Tier4";
        case 5: return "Tier5";
        case 6: return "Tier6";
        default: return NULL;
      }
  }
}

/* Libelle de l'index total : EAST en Standard, BASE/somme en Historique */
const char* totalIndexLabel(uint8_t mode) {
  return isStandard(mode) ? "EAST" : "BASE";
}

/* Libelle du courant instantane : IRMS en Standard, IINST en Historique */
const char* currentLabel(uint8_t mode, uint8_t phase) {
  if (isStandard(mode)) {
    return (phase == 1) ? "IRMS1" : (phase == 2) ? "IRMS2" : "IRMS3";
  }
  return (phase == 1) ? "IINST1" : (phase == 2) ? "IINST2" : "IINST3";
}

/* Libelle de la puissance apparente : SINSTS en Standard, PAPP en Historique */
const char* apparentPowerLabel(uint8_t mode) {
  return isStandard(mode) ? "SINSTS" : "PAPP";
}

/* Check if a channel index is in valid range */
bool APP_isValidChannel(uint8_t ch) {
  return (ch < LORA_NUM_CHANNELS);
}

/* Set RF frequency by channel index. Falls back to PAIR_CHANNEL if invalid. */
void setChannel(uint8_t ch) {
  if (ch >= LORA_NUM_CHANNELS) {
    Serial.print(F("[CHAN] Invalid ch "));
    Serial.print(ch);
    Serial.print(F(", fallback to PAIR_CHANNEL "));
    Serial.println(LORA_PAIR_CHANNEL);
    ch = LORA_PAIR_CHANNEL;
  }
  float freq = LORA_CHANNEL_FREQS_MHZ[ch];
  radio.standby();
  int s = radio.setFrequency(freq);
  if (s != RADIOLIB_ERR_NONE) {
    Serial.print(F("[CHAN] setFrequency FAILED: "));
    Serial.println(s);
  } else {
    Serial.print(F("[CHAN] Switched to ch "));
    Serial.print(ch);
    Serial.print(F(" = "));
    Serial.print(freq, 1);
    Serial.println(F(" MHz"));
  }
}

/* Applique un spreading factor (7..12) a la radio. */
void setSF(uint8_t sf) {
  if (sf < SF_MIN || sf > SF_MAX) {
    Serial.print(F("[SF] Invalid SF "));
    Serial.print(sf);
    Serial.print(F(", fallback to "));
    Serial.println(PAIR_SF);
    sf = PAIR_SF;
  }
  radio.standby();
  int s = radio.setSpreadingFactor(sf);
  if (s != RADIOLIB_ERR_NONE) {
    Serial.print(F("[SF] setSpreadingFactor FAILED: "));
    Serial.println(s);
  } else {
    Serial.print(F("[SF] SF"));
    Serial.println(sf);
  }
}

/* Config operationnelle : canal + SF assignes au reseau. */
void setRadioOp() {
  setChannel(opChannel);
  setSF(opSF);
}

/* Rendez-vous d'appairage : canal 3 + SF11, fixes. */
void setRadioPairing() {
  setChannel(LORA_PAIR_CHANNEL);
  setSF(PAIR_SF);
}

/* CCA scan : pour chaque canal, mesurer RSSI au repos pendant 100ms.
 * Retourne l'index du canal le plus calme (RSSI le plus bas). */
uint8_t ccaScan() {
  Serial.println(F("[CCA] Scanning channels..."));
  int16_t bestRssi = 1;  /* sentinel positif (RSSI sont negatifs) */
  uint8_t bestCh = LORA_PAIR_CHANNEL;

  /* Disable IRQ pendant le scan */
  radio.clearDio1Action();

  for (uint8_t ch = 0; ch < LORA_NUM_CHANNELS; ch++) {
    radio.standby();
    radio.setFrequency(LORA_CHANNEL_FREQS_MHZ[ch]);
    radio.startReceive();
    delay(80);
    int16_t rssi = (int16_t)radio.getRSSI(false);  /* RSSI courant du canal */
    Serial.print(F("  ch "));
    Serial.print(ch);
    Serial.print(F(" ("));
    Serial.print(LORA_CHANNEL_FREQS_MHZ[ch], 0);
    Serial.print(F(" MHz) RSSI = "));
    Serial.print(rssi);
    Serial.println(F(" dBm"));
    if (bestRssi > 0 || rssi < bestRssi) {
      bestRssi = rssi;
      bestCh = ch;
    }
  }
  Serial.print(F("[CCA] Best channel = "));
  Serial.print(bestCh);
  Serial.print(F(" ("));
  Serial.print(bestRssi);
  Serial.println(F(" dBm)"));

  radio.setDio1Action(rxDone);
  return bestCh;
}

/* Full reinit du SX1280 RX (sur le canal courant) */
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

  radio.begin(LORA_CHANNEL_FREQS_MHZ[opChannel], 406.25, 11, 5, 0x12, 10, 16);  /* SF11 */
  radio.setCRC(2);
  radio.explicitHeader();
  radio.invertIQ(false);
  radio.setDio1Action(rxDone);
  radio.startReceive(0xFFFF);

  Serial.print(F("[RX] Reset complete on ch "));
  Serial.print(opChannel);
  Serial.println(F(", listening...\r\n"));
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

/* Per-emitter stats : passe l'index du slot pour avoir des stats par emetteur */
int currentSlotForStats = -1;  /* set par parseAndPrint avant d'appeler les parsers */

/* ====== Contexte Linky de l'emetteur courant ======
 * Les paquets EXTENDED (hors POWER_MAX_CFG) ne portent pas le mode Linky.
 * On memorise le dernier mode/tarif vu pour l'emetteur afin d'interpreter
 * correctement les index Tier1..Tier6 (EASF0x en Standard, HCHC/BBRxxx en
 * Historique). */

void rememberMode(uint8_t mode) {
  if (currentSlotForStats < 0 || currentSlotForStats >= MAX_PAIRED_EMITTERS) return;
  PairedEmitter &em = emitters[currentSlotForStats];
  em.lastMode  = mode;
  em.modeKnown = true;
}

void rememberTarif(uint8_t tarif) {
  if (currentSlotForStats < 0 || currentSlotForStats >= MAX_PAIRED_EMITTERS) return;
  emitters[currentSlotForStats].lastTarif = tarif;
}

void rememberTarifLabel(const char *label) {
  if (currentSlotForStats < 0 || currentSlotForStats >= MAX_PAIRED_EMITTERS) return;
  strncpy(emitters[currentSlotForStats].tarifLabel, label, PKT_EXT_TARIF_MAXLEN);
  emitters[currentSlotForStats].tarifLabel[PKT_EXT_TARIF_MAXLEN] = '\0';
}

/* Libelle texte du tarif recu via EXT_TARIFF_LABEL, "" si pas encore recu */
const char* currentTarifLabel() {
  if (currentSlotForStats >= 0 && currentSlotForStats < MAX_PAIRED_EMITTERS) {
    return emitters[currentSlotForStats].tarifLabel;
  }
  return "";
}

uint8_t currentMode() {
  if (currentSlotForStats >= 0 && currentSlotForStats < MAX_PAIRED_EMITTERS &&
      emitters[currentSlotForStats].modeKnown) {
    return emitters[currentSlotForStats].lastMode;
  }
  return 1;  /* defaut Std Mono tant qu'on n'a pas recu d'ESSENTIAL */
}

uint8_t currentTarif() {
  if (currentSlotForStats >= 0 && currentSlotForStats < MAX_PAIRED_EMITTERS) {
    return emitters[currentSlotForStats].lastTarif;
  }
  return TARIF_UNKNOWN;
}

/* Affiche une ligne d'index energie, en masquant les tiers hors option tarifaire */
void printTierIndex(uint8_t tierIdx, uint32_t value) {
  const char *label = tierLabel(currentMode(), currentTarif(), tierIdx);
  if (label == NULL) return;  /* tier sans signification pour cette option */

  Serial.print(F("  "));
  Serial.print(label);
  for (uint8_t i = strlen(label); i < 7; i++) Serial.print(' ');
  Serial.print(F(": "));
  Serial.print(value);
  Serial.print(F(" Wh ("));
  Serial.print(value / 1000.0, 3);
  Serial.println(F(" kWh)"));
}

void printStats(float rssi, float snr) {
  if (currentSlotForStats >= 0 && currentSlotForStats < MAX_PAIRED_EMITTERS &&
      emitters[currentSlotForStats].valid) {
    PairedEmitter &em = emitters[currentSlotForStats];
    Serial.print(F("  Stats slot "));
    Serial.print(currentSlotForStats);
    Serial.print(F(" : rx="));
    Serial.print(em.rxCount);
    Serial.print(F(" miss="));
    Serial.print(em.missedPackets);
    if (em.rxCount + em.missedPackets > 0) {
      Serial.print(F(" PDR="));
      Serial.print(100.0 * em.rxCount / (em.rxCount + em.missedPackets), 0);
      Serial.print(F("%"));
    }
    Serial.print(F("  RSSI="));
    Serial.print(rssi, 0);
    Serial.print(F(" SNR="));
    Serial.println(snr, 1);
    return;
  }
  /* Fallback : stats globales si pas de slot identifie (legacy) */
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
  /* Reset RAM state */
  for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) emitters[i].valid = false;

  uint32_t magic = 0;
  magic  = (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR)     << 24;
  magic |= (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR + 1) << 16;
  magic |= (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR + 2) << 8;
  magic |= (uint32_t)EEPROM.read(EEPROM_MAGIC_ADDR + 3);

  if (magic != EEPROM_MAGIC_VAL) {
    isPaired = false;
    Serial.println(F("[PAIR] No valid pairing in EEPROM"));
    return;
  }

  /* Load op_channel */
  uint8_t storedCh = EEPROM.read(EEPROM_OPCH_ADDR);
  if (storedCh < LORA_NUM_CHANNELS) {
    opChannel = storedCh;
  } else {
    Serial.print(F("[PAIR] Invalid op_channel in EEPROM ("));
    Serial.print(storedCh);
    Serial.println(F("), fallback PAIR_CHANNEL"));
    opChannel = LORA_PAIR_CHANNEL;
  }

  /* Load op_SF */
  uint8_t storedSF = EEPROM.read(EEPROM_OPSF_ADDR);
  if (storedSF >= SF_MIN && storedSF <= SF_MAX) {
    opSF = storedSF;
  } else {
    opSF = PAIR_SF;   /* EEPROM vierge (0xFF) ou invalide → SF11 */
  }

  uint8_t count = EEPROM.read(EEPROM_COUNT_ADDR);
  if (count > MAX_PAIRED_EMITTERS) count = MAX_PAIRED_EMITTERS;

  /* Load each emitter record */
  int loaded = 0;
  for (int slot = 0; slot < MAX_PAIRED_EMITTERS; slot++) {
    int base = EEPROM_EMITTERS_BASE + slot * EEPROM_EMITTER_SIZE;
    uint8_t valid = EEPROM.read(base);
    if (valid != 0x01) continue;

    emitters[slot].valid = true;
    for (int i = 0; i < MAC_SIZE; i++)
      emitters[slot].mac[i] = EEPROM.read(base + 1 + i);
    for (int i = 0; i < AES_KEY_SIZE; i++)
      emitters[slot].key[i] = EEPROM.read(base + 1 + MAC_SIZE + i);
    emitters[slot].rxCount = 0;
    emitters[slot].errCount = 0;
    emitters[slot].missedPackets = 0;
    emitters[slot].seqInit = false;
    emitters[slot].lastMode = 1;      /* Std Mono par defaut */
    emitters[slot].lastTarif = 0;     /* TARIF_UNKNOWN */
    emitters[slot].modeKnown = false; /* resolu au 1er ESSENTIAL recu */
    emitters[slot].tarifLabel[0] = '\0';

    Serial.print(F("[PAIR] Slot "));
    Serial.print(slot);
    Serial.print(F(" : MAC="));
    for (int i = 0; i < MAC_SIZE; i++) {
      if (emitters[slot].mac[i] < 0x10) Serial.print('0');
      Serial.print(emitters[slot].mac[i], HEX);
    }
    Serial.println();
    loaded++;
  }

  isPaired = (loaded > 0);
  Serial.print(F("[PAIR] Loaded "));
  Serial.print(loaded);
  Serial.print(F(" emitters from EEPROM, op_channel="));
  Serial.println(opChannel);
}

void savePairingToEEPROM() {
  uint32_t magic = EEPROM_MAGIC_VAL;
  EEPROM.update(EEPROM_MAGIC_ADDR,     (magic >> 24) & 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 1, (magic >> 16) & 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 2, (magic >> 8)  & 0xFF);
  EEPROM.update(EEPROM_MAGIC_ADDR + 3,  magic        & 0xFF);

  EEPROM.update(EEPROM_OPCH_ADDR, opChannel);
  EEPROM.update(EEPROM_COUNT_ADDR, (uint8_t)countPairedEmitters());
  EEPROM.update(EEPROM_OPSF_ADDR, opSF);

  for (int slot = 0; slot < MAX_PAIRED_EMITTERS; slot++) {
    int base = EEPROM_EMITTERS_BASE + slot * EEPROM_EMITTER_SIZE;
    EEPROM.update(base, emitters[slot].valid ? 0x01 : 0x00);
    if (emitters[slot].valid) {
      for (int i = 0; i < MAC_SIZE; i++)
        EEPROM.update(base + 1 + i, emitters[slot].mac[i]);
      for (int i = 0; i < AES_KEY_SIZE; i++)
        EEPROM.update(base + 1 + MAC_SIZE + i, emitters[slot].key[i]);
    }
  }

  Serial.print(F("[PAIR] Saved "));
  Serial.print(countPairedEmitters());
  Serial.print(F(" emitters to EEPROM (op_channel="));
  Serial.print(opChannel);
  Serial.println(F(")"));
}

void erasePairingFromEEPROM() {
  for (int i = 0; i < EEPROM_SIZE; i++)
    EEPROM.update(i, 0xFF);

  isPaired = false;
  memset(netKey, 0, AES_KEY_SIZE);
  memset(emitterMAC, 0, MAC_SIZE);
  for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) emitters[i].valid = false;

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
bool decryptPacket(uint8_t *pkt, int totalLen, const uint8_t *key, const uint8_t *mac, bool verbose = true) {
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

  if (verbose) {
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
  }

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
 * Send PAIR_RESPONSE packet (21 bytes):
 *   [0]     0x14 (v1, pair_response)
 *   [1]     seq (echo from request)
 *   [2-17]  netKey[0..15] AES-128 key
 *   [18]    status 0x00 = OK
 *   [19]    op_channel (0..7)  - emitter switches to this for data
 *   [20]    op_SF (7..12)      - spreading factor for data
 *
 * L'appairage se fait a SF11 (rendez-vous). Apres le PAIR_RESPONSE, on bascule
 * sur (op_channel, op_SF) : c'est la que l'emetteur envoie le PAIR_CONFIRM.
 */
bool sendPairResponse(uint8_t seq) {
  uint8_t pkt[PAIR_RESPONSE_SIZE];

  pkt[0] = 0x14; /* v1 | PAIR_RESPONSE */
  pkt[1] = seq;
  memcpy(&pkt[2], netKey, AES_KEY_SIZE);
  pkt[18] = 0x00;        /* status OK */
  pkt[19] = opChannel;   /* canal operationnel assigne */
  pkt[20] = opSF;        /* spreading factor operationnel assigne */

  Serial.print(F("[PAIR] TX PAIR_RESPONSE seq="));
  Serial.print(seq);
  Serial.print(F(" op_channel="));
  Serial.print(opChannel);
  Serial.print(F(" op_SF="));
  Serial.print(opSF);
  Serial.print(F(" Key="));
  for (int i = 0; i < AES_KEY_SIZE; i++) {
    if (netKey[i] < 0x10) Serial.print('0');
    Serial.print(netKey[i], HEX);
  }
  Serial.println();

  /* Switch to TX (still on PAIR_CHANNEL / SF11 where we received PAIR_REQUEST) */
  radio.clearDio1Action();
  int state = radio.transmit(pkt, PAIR_RESPONSE_SIZE);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("[PAIR] PAIR_RESPONSE TX failed: "));
    Serial.println(state);
    radio.setDio1Action(rxDone);
    radio.startReceive(0xFFFF);
    return false;
  }
  Serial.println(F("[PAIR] PAIR_RESPONSE sent OK"));

  /* Bascule sur la config operationnelle (canal + SF) : c'est la que
   * l'emetteur envoie le PAIR_CONFIRM. */
  setRadioOp();
  radio.setDio1Action(rxDone);
  radio.startReceive(0xFFFF);
  return true;
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

  /* SUCCESS — add this pair to the array */
  int idx = addOrUpdateEmitter(emitterMAC, netKey);
  if (idx < 0) {
    Serial.print(F("[PAIR] No free slot! Maximum "));
    Serial.print(MAX_PAIRED_EMITTERS);
    Serial.println(F(" emitters reached. Use 'E' to erase or 'R N' to remove a slot."));
    return;
  }

  Serial.print(F("[PAIR] *** PAIRING SUCCESSFUL *** (slot "));
  Serial.print(idx);
  Serial.println(F(")"));
  isPaired = true;
  rxPairState = RX_PAIRED;
  savePairingToEEPROM();

  Serial.print(F("[PAIR] Paired with MAC="));
  for (int i = 0; i < MAC_SIZE; i++) {
    if (emitterMAC[i] < 0x10) Serial.print('0');
    Serial.print(emitterMAC[i], HEX);
  }
  Serial.print(F(" (total emitters: "));
  Serial.print(countPairedEmitters());
  Serial.print(F("/"));
  Serial.print(MAX_PAIRED_EMITTERS);
  Serial.println(F(")"));
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

  /* Memorise le mode pour que les parsers EXTENDED (qui ne le portent pas)
   * puissent nommer correctement les index Tier1..Tier6. */
  rememberMode(mode);

  /* En Historique la puissance apparente s'appelle PAPP, en Standard SINSTS */
  const char *pwrLabel = apparentPowerLabel(mode);

  Serial.println(F("========== LINKY ESSENTIAL =========="));

  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.print(seq);
  Serial.print(F("  Mode: "));
  Serial.println(modeStr(mode));

  Serial.print(F("  "));
  Serial.print(pwrLabel);
  Serial.print(F(" : "));
  Serial.print(sinsts);
  Serial.println(F(" VA (total)"));

  if (isTri(mode)) {
    Serial.print(F("  "));
    Serial.print(pwrLabel);
    Serial.print(F("1: "));
    Serial.print(sinsts1);
    Serial.print(F("  "));
    Serial.print(pwrLabel);
    Serial.print(F("2: "));
    Serial.print(sinsts2);
    Serial.print(F("  "));
    Serial.print(pwrLabel);
    Serial.print(F("3: "));
    Serial.print(sinsts3);
    Serial.println(F(" VA"));
  } else if (isStandard(mode)) {
    /* En Standard mono, SINSTS1 existe et vaut SINSTS */
    Serial.print(F("  SINSTS1: "));
    Serial.print(sinsts1);
    Serial.println(F(" VA"));
  }
  /* En Historique mono, PAPP total suffit — pas de PAPP1 dans la TIC */

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

  uint16_t i1 = getU16(&buf[3]);
  uint16_t i2 = getU16(&buf[5]);
  uint16_t i3 = getU16(&buf[7]);
  uint16_t urms1 = getU16(&buf[9]);
  uint16_t urms2 = getU16(&buf[11]);
  uint16_t urms3 = getU16(&buf[13]);

  uint8_t mode = currentMode();
  bool tri = isTri(mode);

  Serial.println(F("---------- EXT: CURRENT_VOLTAGE ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  /* Courants : IRMSx en Standard, IINSTx en Historique (meme champ ZCL) */
  Serial.print(F("  "));
  Serial.print(currentLabel(mode, 1));
  Serial.print(F("  : "));
  Serial.print(i1);
  if (tri) {
    Serial.print(F("  "));
    Serial.print(currentLabel(mode, 2));
    Serial.print(F(": "));
    Serial.print(i2);
    Serial.print(F("  "));
    Serial.print(currentLabel(mode, 3));
    Serial.print(F(": "));
    Serial.print(i3);
  }
  Serial.println(isStandard(mode) ? F(" mA") : F(" A"));

  /* Tensions : URMS n'existe que dans la TIC Standard */
  if (isStandard(mode)) {
    Serial.print(F("  URMS1  : "));
    Serial.print(urms1);
    if (tri) {
      Serial.print(F("  URMS2: "));
      Serial.print(urms2);
      Serial.print(F("  URMS3: "));
      Serial.print(urms3);
    }
    Serial.println(F(" V"));
  }

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
  uint32_t tier1  = getU32(&buf[11]);
  uint32_t tier2  = getU32(&buf[15]);

  uint8_t mode = currentMode();

  Serial.println(F("---------- EXT: ENERGY ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  /* Index total : EAST en Standard, BASE en Historique.
   * En Historique HC/HP, EJP ou Tempo, l'emetteur ne remplit ce champ
   * que pour l'option BASE — on le masque sinon (valeur non significative). */
  if (isStandard(mode) || histoOptarifFromTarif(currentTarif()) == OPT_BASE
                       || histoOptarifFromTarif(currentTarif()) == OPT_UNKNOWN) {
    Serial.print(F("  "));
    Serial.print(totalIndexLabel(mode));
    Serial.print(F("   : "));
    Serial.print(east);
    Serial.print(F(" Wh ("));
    Serial.print(east / 1000.0, 3);
    Serial.println(F(" kWh)"));
  }

  /* EAIT (injection) : Standard uniquement — pas d'equivalent Historique */
  if (isStandard(mode)) {
    Serial.print(F("  EAIT   : "));
    Serial.print(eait);
    Serial.print(F(" Wh ("));
    Serial.print(eait / 1000.0, 3);
    Serial.println(F(" kWh) [injection]"));
  }

  /* Index tarifaires 1 et 2 : nommes selon mode + option tarifaire */
  printTierIndex(1, tier1);
  printTierIndex(2, tier2);

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

  uint32_t tier3 = getU32(&buf[3]);
  uint32_t tier4 = getU32(&buf[7]);
  uint32_t tier5 = getU32(&buf[11]);
  uint32_t tier6 = getU32(&buf[15]);

  /* En Historique BASE / HC-HP / EJP, les tiers 3 a 6 n'existent pas :
   * printTierIndex() les masque automatiquement (tierLabel() renvoie NULL).
   * Seul Tempo utilise les 6 tiers (BBRxxJB/JW/JR). */
  bool anyShown = (tierLabel(currentMode(), currentTarif(), 3) != NULL);

  Serial.println(F("---------- EXT: ENERGY_2 ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  if (!anyShown) {
    Serial.println(F("  (tiers 3-6 non utilises par cette option tarifaire)"));
  } else {
    printTierIndex(3, tier3);
    printTierIndex(4, tier4);
    printTierIndex(5, tier5);
    printTierIndex(6, tier6);
  }

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

  uint8_t mode = currentMode();
  bool tri = isTri(mode);

  Serial.println(F("---------- EXT: VOLTAGE_STATS ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  /* UMOY (tension moyenne) : etiquette Standard uniquement */
  if (isStandard(mode)) {
    Serial.print(F("  UMOY1  : "));
    Serial.print(umoy1);
    if (tri) {
      Serial.print(F("  UMOY2: "));
      Serial.print(umoy2);
      Serial.print(F("  UMOY3: "));
      Serial.print(umoy3);
    }
    Serial.println(F(" V"));
  }

  /* IMAX existe dans les deux modes (mA en Standard, A en Historique) */
  Serial.print(F("  IMAX1  : "));
  Serial.print(imax1);
  if (tri) {
    Serial.print(F("  IMAX2: "));
    Serial.print(imax2);
    Serial.print(F("  IMAX3: "));
    Serial.print(imax3);
  }
  Serial.println(isStandard(mode) ? F(" mA") : F(" A"));

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
  uint8_t  tarif  = buf[17];   /* ex-reserved : code tarif unifie */

  /* Ce paquet porte mode ET tarif : c'est lui qui permet de resoudre
   * l'option tarifaire Historique (BASE / HC-HP / EJP / Tempo) et donc
   * de nommer correctement les index Tier1..Tier6 des paquets ENERGY. */
  rememberMode(mode);
  rememberTarif(tarif);

  Serial.println(F("---------- EXT: POWER_MAX_CFG ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  Serial.print(F("  SMAXSN : "));
  Serial.print(smaxsn);
  Serial.println(F(" VA (soutire max jour)"));

  /* Injection : Standard uniquement */
  if (isStandard(mode)) {
    Serial.print(F("  SMAXIN : "));
    Serial.print(smaxin);
    Serial.println(F(" VA (injecte max jour)"));

    Serial.print(F("  SINSTI : "));
    Serial.print(sinsti);
    Serial.println(F(" VA (instantane)"));

    Serial.print(F("  CCAIN  : "));
    Serial.print(ccain);
    Serial.println(F(" VA (courbe charge)"));
  }

  Serial.print(F("  RELAIS : 0x"));
  if (relais < 0x1000) Serial.print('0');
  if (relais < 0x100)  Serial.print('0');
  if (relais < 0x10)   Serial.print('0');
  Serial.println(relais, HEX);

  /* Tarif : NTARF numerique en Standard, libelle en Historique.
   * Si on a deja recu un EXT_TARIFF_LABEL, on affiche aussi le texte brut
   * du compteur (LTARF / PTEC). */
  if (isStandard(mode)) {
    Serial.print(F("  NTARF  : "));
    Serial.print(ntarf);
    if (currentTarifLabel()[0] != '\0') {
      Serial.print(F("  ("));
      Serial.print(currentTarifLabel());
      Serial.print(F(")"));
    }
    Serial.println();
  } else {
    Serial.print(F("  PTEC   : "));
    if (currentTarifLabel()[0] != '\0') {
      Serial.print(currentTarifLabel());
      Serial.print(F(" — "));
    }
    Serial.println(tarifStr(mode, tarif));
  }

  Serial.print(F("  PCOUP  : "));
  Serial.print(pcoup);
  Serial.println(F(" kVA"));

  /* PREF en Standard = puissance de reference ; en Historique le meme champ
   * porte ISOUSC (intensite souscrite, en A). */
  if (isStandard(mode)) {
    Serial.print(F("  PREF   : "));
    Serial.print(pref);
    Serial.println(F(" kVA"));
  } else {
    Serial.print(F("  ISOUSC : "));
    Serial.print(pref);
    Serial.println(F(" A"));
  }

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

/* ====== Parse EXT 0x07 : TARIFF_LABEL (6..22B, taille variable) ======
 *   [3]    mode Linky
 *   [4]    code tarif
 *   [5]    longueur du libelle (N)
 *   [6..]  libelle ASCII (N octets) : LTARF en Standard, PTEC en Historique
 */
void parseExtTariffLabel(uint8_t *buf, int len, float rssi, float snr) {
  if (len < PKT_EXT_TARIF_HDR) {
    Serial.print(F("[EXT] TariffLabel taille: "));
    Serial.println(len);
    return;
  }

  uint8_t seq   = buf[1];
  uint8_t mode  = buf[3];
  uint8_t tarif = buf[4];
  uint8_t n     = buf[5];

  trackSequence(seq);

  /* Garde-fou : la longueur annoncee doit tenir dans le paquet recu */
  if (n > PKT_EXT_TARIF_MAXLEN || (int)(PKT_EXT_TARIF_HDR + n) > len) {
    Serial.print(F("[EXT] TariffLabel longueur invalide: n="));
    Serial.print(n);
    Serial.print(F(" len="));
    Serial.println(len);
    return;
  }

  char label[PKT_EXT_TARIF_MAXLEN + 1];
  memcpy(label, &buf[PKT_EXT_TARIF_HDR], n);
  label[n] = '\0';

  /* Ce paquet porte mode + tarif + libelle : on memorise les trois */
  rememberMode(mode);
  rememberTarif(tarif);
  rememberTarifLabel(label);

  Serial.println(F("---------- EXT: TARIFF_LABEL ----------"));
  Serial.print(F("  #"));
  Serial.print(rxCount);
  Serial.print(F(" seq="));
  Serial.println(seq);

  /* LTARF en Standard, PTEC en Historique — le mode tranche */
  Serial.print(isStandard(mode) ? F("  LTARF  : ") : F("  PTEC   : "));
  Serial.println(label);

  /* Complement : interpretation du code tarif */
  if (isStandard(mode)) {
    Serial.print(F("  NTARF  : "));
    Serial.println(tarif);
  } else {
    Serial.print(F("  Tarif  : "));
    Serial.println(tarifStr(mode, tarif));
  }

  Serial.print(F("  Mode   : "));
  Serial.println(modeStr(mode));

  printStats(rssi, snr);
  Serial.println(F("---------------------------------------\r\n"));
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

  /* Try decrypt with each known emitter (MIC validates the right pair). */
  int matched = -1;
  uint8_t backupBuf[256];
  if (len > (int)sizeof(backupBuf)) {
    Serial.println(F("[RX] Packet too large for backup buffer"));
    rxErrors++;
    return;
  }
  memcpy(backupBuf, buf, len);  /* save original ciphertext for retries */

  for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) {
    if (!emitters[i].valid) continue;
    /* Restore ciphertext for this attempt */
    memcpy(buf, backupBuf, len);
    /* Silent trial: pas de print MIC mismatch pour ce slot, on essaie le suivant */
    if (decryptPacket(buf, len, emitters[i].key, emitters[i].mac, false)) {
      matched = i;
      break;
    }
  }

  if (matched < 0) {
    Serial.print(F("[RX] Decrypt/MIC failed for all emitters! len="));
    Serial.print(len);
    Serial.print(F(" HEX: "));
    hexDump(backupBuf, len);
    rxErrors++;
    return;
  }

  /* Update per-emitter stats */
  emitters[matched].rxCount++;
  uint8_t curSeq = buf[1];
  if (emitters[matched].seqInit) {
    uint8_t expected = (emitters[matched].lastSeq + 1) & 0xFF;
    if (curSeq != expected) {
      emitters[matched].missedPackets += (uint8_t)(curSeq - expected);
    }
  }
  emitters[matched].lastSeq = curSeq;
  emitters[matched].seqInit = true;

  /* Print which emitter sent this packet (compact) */
  Serial.print(F("[RX] from slot "));
  Serial.print(matched);
  Serial.print(F(" MAC="));
  for (int i = 4; i < MAC_SIZE; i++) {  /* show last 4 bytes of MAC for compactness */
    if (emitters[matched].mac[i] < 0x10) Serial.print('0');
    Serial.print(emitters[matched].mac[i], HEX);
  }
  Serial.print(F("  rx="));
  Serial.print(emitters[matched].rxCount);
  Serial.print(F(" miss="));
  Serial.println(emitters[matched].missedPackets);

  /* Set le slot pour les printStats en aval */
  currentSlotForStats = matched;

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
      case EXT_TARIFF_LABEL:    parseExtTariffLabel(buf, dataLen, rssi, snr);   break;
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
/* ====== OTA Gateway mode ====== */
/* ====================================================================== */

/* Parse hex string into byte buffer. Returns number of bytes decoded. */
static int parseHexBuf(const char *hex, uint8_t *buf, int maxLen) {
  int len = 0;
  while (*hex && *hex != '\r' && *hex != '\n' && len < maxLen) {
    /* skip spaces */
    while (*hex == ' ') hex++;
    if (!hex[0] || !hex[1]) break;
    char hi = hex[0], lo = hex[1];
    uint8_t hiv = (hi >= '0' && hi <= '9') ? hi - '0' :
                  (hi >= 'A' && hi <= 'F') ? hi - 'A' + 10 :
                  (hi >= 'a' && hi <= 'f') ? hi - 'a' + 10 : 0xFF;
    uint8_t lov = (lo >= '0' && lo <= '9') ? lo - '0' :
                  (lo >= 'A' && lo <= 'F') ? lo - 'A' + 10 :
                  (lo >= 'a' && lo <= 'f') ? lo - 'a' + 10 : 0xFF;
    if (hiv == 0xFF || lov == 0xFF) break;
    buf[len++] = (hiv << 4) | lov;
    hex += 2;
  }
  return len;
}

static void printHex(const uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
  }
}

/* Encrypt + add MIC (same scheme as decryptPacket but reverse).
 * Inputs : pkt[0..len-1] = cleartext (type+seq+payload)
 * Output : pkt[0..len+1] = type+seq+ciphered_payload + MIC(2)
 * Returns: total length including MIC */
static int encryptOtaPacket(uint8_t *pkt, int len, const uint8_t *key, const uint8_t *mac) {
  if (len < 2) return 0;

  /* 1) Compute MIC over cleartext */
  uint8_t expectedMIC[16];
  aes128_cmac(pkt, (uint16_t)len, key, expectedMIC);

  /* 2) Encrypt payload (skip type+seq = 2 bytes header) */
  uint8_t nonce[16];
  buildNonce(nonce, mac, pkt[1]);
  int payloadLen = len - 2;
  if (payloadLen > 0) {
    aes128_ctr_crypt(&pkt[2], (uint16_t)payloadLen, key, nonce);
  }

  /* 3) Append MIC bytes [0..1] */
  pkt[len]     = expectedMIC[0];
  pkt[len + 1] = expectedMIC[1];
  return len + CRYPT_MIC_SIZE;
}

/* ====================================================================== */
/* ====== POLL : lecture d'attribut a la demande ====== */
/* ====================================================================== */

/* Decode et affiche un POLL_RESPONSE dechiffre (MIC deja retire).
 * Format : [0]type [1]seq [2-3]cluster [4-5]attr [6]status [7]zcl_type
 *          [8]len [9..]valeur (big-endian pour les numeriques). */
void decodePollResponse(const uint8_t *buf, int plainLen, int slot) {
  uint16_t cluster = ((uint16_t)buf[2] << 8) | buf[3];
  uint16_t attr    = ((uint16_t)buf[4] << 8) | buf[5];
  uint8_t  status  = buf[6];
  uint8_t  zclType = buf[7];
  uint8_t  vlen    = buf[8];

  Serial.println(F("---------- POLL RESPONSE ----------"));
  Serial.print(F("  slot="));   Serial.print(slot);
  Serial.print(F(" cluster=0x")); Serial.print(cluster, HEX);
  Serial.print(F(" attr=0x"));    Serial.println(attr, HEX);

  if (status != 0x00) {
    Serial.print(F("  status=0x")); Serial.print(status, HEX);
    Serial.println(status == 0x01 ? F(" (UNKNOWN_ATTR)") : F(" (erreur)"));
    Serial.println(F("-----------------------------------\r\n"));
    return;
  }
  if (9 + vlen > plainLen) { Serial.println(F("  (trame incoherente)")); return; }

  const uint8_t *v = &buf[9];
  Serial.print(F("  valeur : "));
  switch (zclType) {
    case 0x20: Serial.println((uint32_t)v[0]); break;                         /* uint8 */
    case 0x28: Serial.println((int8_t)v[0]); break;                           /* int8 */
    case 0x21: Serial.println((uint32_t)(((uint16_t)v[0]<<8)|v[1])); break;   /* uint16 */
    case 0x29: Serial.println((int16_t)(((uint16_t)v[0]<<8)|v[1])); break;    /* int16 */
    case 0x23:                                                                /* uint32 */
      Serial.println(((uint32_t)v[0]<<24)|((uint32_t)v[1]<<16)|((uint32_t)v[2]<<8)|v[3]); break;
    case 0x2B: {                                                              /* int32 */
      int32_t s = ((uint32_t)v[0]<<24)|((uint32_t)v[1]<<16)|((uint32_t)v[2]<<8)|v[3];
      Serial.println(s); break;
    }
    case 0x25: {                                                             /* uint48 */
      uint64_t u = 0; for (int i=0;i<6;i++) u=(u<<8)|v[i];
      Serial.println((unsigned long)u); break;
    }
    case 0x41: case 0x42: {                                                  /* string */
      for (int i=0;i<vlen;i++) Serial.write(v[i]);
      Serial.println();
      break;
    }
    default:
      Serial.print(F("(type 0x")); Serial.print(zclType, HEX); Serial.print(F(") "));
      for (int i=0;i<vlen;i++) { if(v[i]<0x10)Serial.print('0'); Serial.print(v[i],HEX); }
      Serial.println();
  }
  Serial.println(F("-----------------------------------\r\n"));
}

/* Emet un POLL_REQUEST vers l'emetteur du slot (sa fenetre RX vient de
 * s'ouvrir, juste apres l'uplink qu'on vient de recevoir), puis attend le
 * POLL_RESPONSE. A appeler immediatement apres traitement de l'uplink. */
void sendPollAndWait(int slot) {
  if (slot < 0 || slot >= MAX_PAIRED_EMITTERS || !emitters[slot].valid) return;
  PairedEmitter &em = emitters[slot];

  uint8_t pkt[6 + CRYPT_MIC_SIZE];
  pkt[0] = (0x01 << 4) | PKT_TYPE_POLL_REQUEST;   /* 0x1B */
  pkt[1] = pollSeq++;
  pkt[2] = (uint8_t)(pendingPoll[slot].cluster >> 8);
  pkt[3] = (uint8_t)(pendingPoll[slot].cluster & 0xFF);
  pkt[4] = (uint8_t)(pendingPoll[slot].attr >> 8);
  pkt[5] = (uint8_t)(pendingPoll[slot].attr & 0xFF);
  int txLen = encryptOtaPacket(pkt, 6, em.key, em.mac);

  radio.clearDio1Action();
  int s = radio.transmit(pkt, txLen);
  radio.setDio1Action(rxDone);
  rxFlag = false;
  radio.startReceive(0xFFFF);

  if (s != RADIOLIB_ERR_NONE) {
    Serial.print(F("[POLL] TX fail ")); Serial.println(s);
    pendingPoll[slot].valid = false;
    return;
  }

  unsigned long deadline = millis() + POLL_RESP_TIMEOUT_MS;
  while (millis() < deadline) {
    if (rxFlag) {
      rxFlag = false;
      uint8_t buf[64];
      int len = radio.getPacketLength();
      if (len > 0 && len <= (int)sizeof(buf) &&
          radio.readData(buf, len) == RADIOLIB_ERR_NONE) {
        if (decryptPacket(buf, len, em.key, em.mac, false)) {
          int plainLen = len - CRYPT_MIC_SIZE;
          if (plainLen >= 9 && (buf[0] & 0x0F) == PKT_TYPE_POLL_RESPONSE) {
            decodePollResponse(buf, plainLen, slot);
            pendingPoll[slot].valid = false;
            radio.startReceive(0xFFFF);
            return;
          }
        }
      }
      radio.startReceive(0xFFFF);
    }
  }
  Serial.print(F("[POLL] timeout slot ")); Serial.println(slot);
  pendingPoll[slot].valid = false;
  radio.startReceive(0xFFFF);
}

/* Send a raw OTA packet via LoRa, wait for OTA_ACK, decrypt + return payload.
 * Returns length of decrypted ACK payload (without MIC) or 0 on timeout. */
static int otaTxAndWaitAck(uint8_t *pktClear, int clearLen, uint8_t *ackOut, int ackMaxLen) {
  if (!otaGwInitialized) {
    Serial.println(F("OTA_ERROR not_initialized"));
    return 0;
  }

  /* Encrypt + MIC */
  uint8_t txBuf[256];
  if (clearLen + CRYPT_MIC_SIZE > (int)sizeof(txBuf)) {
    Serial.println(F("OTA_ERROR pkt_too_big"));
    return 0;
  }
  memcpy(txBuf, pktClear, clearLen);
  int txLen = encryptOtaPacket(txBuf, clearLen, otaGwTargetKey, otaGwTargetMAC);

  /* Switch to TX */
  radio.clearDio1Action();
  int s = radio.transmit(txBuf, txLen);
  if (s != RADIOLIB_ERR_NONE) {
    Serial.print(F("OTA_TX_FAIL "));
    Serial.println(s);
    radio.setDio1Action(rxDone);
    radio.startReceive(0xFFFF);
    return 0;
  }

  /* Switch to RX, wait for ACK with timeout */
  radio.setDio1Action(rxDone);
  rxFlag = false;
  radio.startReceive(0xFFFF);

  unsigned long deadline = millis() + OTA_ACK_TIMEOUT_MS;
  while (millis() < deadline) {
    if (rxFlag) {
      rxFlag = false;
      uint8_t buf[128];     /* assez pour OTA_BLOCK (70B) + paquets TIC voisins */
      uint8_t backupBuf[128];
      int len = radio.getPacketLength();
      if (len > 0 && len <= (int)sizeof(buf)) {
        int rs = radio.readData(buf, len);
        if (rs == RADIOLIB_ERR_NONE) {
          /* Capture RSSI/SNR avant que readData ne les invalide */
          float pktRssi = radio.getRSSI();
          float pktSnr  = radio.getSNR();

          /* Sauvegarde le ciphertext pour pouvoir reessayer (decryptPacket
           * fait du AES-CTR in-place, donc le buffer est modifie meme
           * quand le MIC echoue). */
          memcpy(backupBuf, buf, len);

          /* 1) Tente d'abord la cle de la cible OTA — silencieux */
          if (decryptPacket(buf, len, otaGwTargetKey, otaGwTargetMAC, false)) {
            int plainLen = len - CRYPT_MIC_SIZE;
            if (plainLen <= ackMaxLen) {
              memcpy(ackOut, buf, plainLen);
              return plainLen;
            }
          }

          /* 2) Pas un ACK OTA : c'est peut-etre une trame TIC d'un autre
           *    emitter appaire. On restaure le ciphertext et on passe au
           *    dispatch normal, qui essaie toutes les cles et forwarde sur
           *    Serial si match. Cela permet aux autres devices de continuer
           *    a etre servis pendant la mise a jour OTA. */
          memcpy(buf, backupBuf, len);
          parseAndPrint(buf, len, pktRssi, pktSnr);
        }
      }
      /* mauvaise donnee, paquet voisin traite, ou rien : on continue d'attendre l'ACK OTA */
      radio.startReceive(0xFFFF);
    }
  }
  return 0;  /* timeout */
}

/* Handle a Serial command starting with 'O' :
 *   "OTA_GW_INIT <mac_hex>"  → load key for mac, return OTA_GW_READY
 *   "OTA_TX <hex_packet>"    → encrypt + tx + wait ack → "OTA_RX <hex>" or "OTA_TIMEOUT"
 */
static void handleOtaGatewayCommand() {
  /* Read remainder of line "TA_..." */
  char buf[400];
  int n = 0;
  unsigned long deadline = millis() + 100;
  while (millis() < deadline && n < (int)sizeof(buf) - 1) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
      buf[n++] = c;
    }
  }
  buf[n] = 0;

  if (strncmp(buf, "TA_GW_INIT", 10) == 0) {
    const char *p = buf + 10;
    while (*p == ' ') p++;
    uint8_t mac[8];
    int ml = parseHexBuf(p, mac, 8);
    if (ml != 8) {
      Serial.println(F("OTA_GW_ERROR bad_mac"));
      return;
    }
    /* Find emitter slot for this MAC */
    int idx = findEmitterByMAC(mac);
    if (idx < 0) {
      Serial.println(F("OTA_GW_ERROR mac_not_paired"));
      return;
    }
    memcpy(otaGwTargetMAC, emitters[idx].mac, MAC_SIZE);
    memcpy(otaGwTargetKey, emitters[idx].key, AES_KEY_SIZE);
    otaGwInitialized = true;
    rxPairState = RX_OTA_GATEWAY;
    radio.setDio1Action(rxDone);
    radio.startReceive(0xFFFF);
    Serial.println(F("OTA_GW_READY"));
  }
  else if (strncmp(buf, "TA_TX", 5) == 0) {
    if (!otaGwInitialized) {
      Serial.println(F("OTA_GW_ERROR not_initialized"));
      return;
    }
    const char *p = buf + 5;
    while (*p == ' ') p++;
    uint8_t pkt[200];
    int len = parseHexBuf(p, pkt, sizeof(pkt) - CRYPT_MIC_SIZE);
    if (len < 2) {
      Serial.println(F("OTA_GW_ERROR bad_packet"));
      return;
    }

    uint8_t ack[64];
    int ackLen = otaTxAndWaitAck(pkt, len, ack, sizeof(ack));
    if (ackLen > 0) {
      Serial.print(F("OTA_RX "));
      printHex(ack, ackLen);
      Serial.println();
    } else {
      Serial.println(F("OTA_TIMEOUT"));
    }
  }
  else if (strncmp(buf, "TA_GW_EXIT", 10) == 0) {
    otaGwInitialized = false;
    rxPairState = isPaired ? RX_PAIRED : RX_IDLE;
    radio.startReceive(0xFFFF);
    Serial.println(F("OTA_GW_EXITED"));
  }
  else {
    Serial.print(F("OTA_GW_ERROR unknown_cmd: O"));
    Serial.println(buf);
  }
}

/* ====================================================================== */
/* ====== Setup ====== */
/* ====================================================================== */
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("\r\n[RX] ZLinky TIC Receiver SX1280 (multi-channel)"));
  Serial.println(F("[RX] 8 canaux ISM 2410-2480 MHz, SF11 BW406 CR4/5"));
  Serial.println(F("[RX] Essential 5s + Extended piggyback 20s"));
  Serial.println(F("[RX] AES-128-CTR + CMAC MIC enabled"));
  Serial.println(F("[RX] Max 22 octets/paquet + MIC\r\n"));

  /* Load pairing from EEPROM (sets opChannel if previously paired) */
  loadPairingFromEEPROM();

  /* Init radio with PAIR_CHANNEL frequency for the CCA scan, SF11 BW406 */
  int state = radio.begin(LORA_CHANNEL_FREQS_MHZ[LORA_PAIR_CHANNEL],
                         406.25, 11, 5, 0x12, 10, 16);  /* SF11 */
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

  /* Determine operational channel:
   *   1. If already paired (EEPROM magic OK): use saved op_channel
   *   2. Else if EEPROM_OPCH_ADDR has a valid value (set via 'C N' command): use it
   *   3. Else if DEFAULT_OP_CHANNEL is a valid index (0..7): use it
   *   4. Else: do CCA scan to pick the cleanest channel
   */
  if (isPaired) {
    Serial.print(F("[BOOT] Paired, using saved op_channel = "));
    Serial.println(opChannel);
  } else {
    /* Cas non-paired : verifier d'abord si l'utilisateur a set un canal via 'C N' */
    uint8_t storedCh = EEPROM.read(EEPROM_OPCH_ADDR);
    if (APP_isValidChannel(storedCh)) {
      opChannel = storedCh;
      Serial.print(F("[BOOT] Not paired, using user-set op_channel = "));
      Serial.print(opChannel);
      Serial.print(F(" ("));
      Serial.print(LORA_CHANNEL_FREQS_MHZ[opChannel], 0);
      Serial.println(F(" MHz)"));
    } else if (DEFAULT_OP_CHANNEL >= 0 && DEFAULT_OP_CHANNEL < LORA_NUM_CHANNELS) {
      opChannel = DEFAULT_OP_CHANNEL;
      Serial.print(F("[BOOT] Not paired, using DEFAULT_OP_CHANNEL = "));
      Serial.print(opChannel);
      Serial.print(F(" ("));
      Serial.print(LORA_CHANNEL_FREQS_MHZ[opChannel], 0);
      Serial.println(F(" MHz)"));
    } else {
      uint8_t scanned = ccaScan();
      opChannel = scanned;
      Serial.print(F("[BOOT] Not paired, CCA-selected op_channel = "));
      Serial.println(opChannel);
    }
  }

  setRadioOp();   /* canal + SF operationnels */
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

  Serial.println(F("[CMD] Serial commands :"));
  Serial.println(F("       'C N'           set op_channel = N (0..7), erase ALL"));
  Serial.println(F("       'F N'           set op_SF = N (7..12), erase ALL"));
  Serial.println(F("       'S'             show status"));
  Serial.println(F("       'P'             enter PAIR_LISTEN (30s)"));
  Serial.println(F("       'Q s cl at'     poll attribut (hex) : Q 0 FF66 300"));
  Serial.println(F("       'R N'           remove emitter at slot N"));
  Serial.println(F("       'E'             erase ALL pairings"));
  Serial.println(F("       'OTA_GW_INIT M' enter OTA gateway mode for MAC=M"));
  Serial.println(F("       'OTA_TX <hex>'  send OTA packet, get ack"));
  Serial.println(F("       'OTA_GW_EXIT'   leave OTA gateway mode"));

  Serial.println(F("Attente paquets...\r\n"));
}

/* ====================================================================== */
/* ====== Serial command handler ====== */
/* ====================================================================== */
void handleSerialCommand() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  /* skip whitespace and CR/LF */
  while (cmd == ' ' || cmd == '\r' || cmd == '\n' || cmd == '\t') {
    if (!Serial.available()) return;
    cmd = Serial.read();
  }

  if (cmd == 'C' || cmd == 'c') {
    /* "C N" : set op_channel to N */
    while (!Serial.available()) { delay(1); }
    int n = Serial.parseInt();
    if (n < 0 || n >= LORA_NUM_CHANNELS) {
      Serial.print(F("[CMD] Invalid channel "));
      Serial.print(n);
      Serial.print(F(", must be 0.."));
      Serial.println(LORA_NUM_CHANNELS - 1);
      return;
    }
    Serial.print(F("[CMD] Setting op_channel = "));
    Serial.print(n);
    Serial.print(F(" ("));
    Serial.print(LORA_CHANNEL_FREQS_MHZ[n], 0);
    Serial.println(F(" MHz)"));
    Serial.println(F("[CMD] Erasing pairing — re-pair all emitters required"));
    erasePairingFromEEPROM();
    opChannel = n;
    setRadioOp();
    /* Save the new channel even without pair (so it persists on reboot) */
    /* On l'ecrit directement, le magic reste invalide tant que pas paire */
    EEPROM.update(EEPROM_OPCH_ADDR, opChannel);
    rxPairState = RX_IDLE;
    radio.startReceive(0xFFFF);
    Serial.println(F("[CMD] Done. Long press USER button to enter pairing mode."));
  }
  else if (cmd == 'F' || cmd == 'f') {
    /* Regle le spreading factor operationnel du reseau. Comme le SF est
     * negocie a l'appairage, changer le SF impose de re-appairer. */
    while (!Serial.available()) { delay(1); }
    int sf = Serial.parseInt();
    if (sf < SF_MIN || sf > SF_MAX) {
      Serial.print(F("[CMD] Invalid SF "));
      Serial.print(sf);
      Serial.print(F(", must be "));
      Serial.print(SF_MIN);
      Serial.print(F(".."));
      Serial.println(SF_MAX);
      return;
    }
    Serial.print(F("[CMD] Setting op_SF = SF"));
    Serial.println(sf);
    Serial.println(F("[CMD] Erasing pairing — re-pair all emitters required"));
    erasePairingFromEEPROM();
    opSF = (uint8_t)sf;
    setRadioOp();
    EEPROM.update(EEPROM_OPSF_ADDR, opSF);
    rxPairState = RX_IDLE;
    radio.startReceive(0xFFFF);
    Serial.println(F("[CMD] Done. Long press USER button to enter pairing mode."));
  }
  else if (cmd == 'Q' || cmd == 'q') {
    /* "Q <slot> <cluster_hex> <attr_hex>" : arme une lecture d'attribut.
     * Envoyee au prochain uplink de l'emetteur de ce slot. */
    char line[48];
    int n = 0;
    unsigned long dl = millis() + 100;
    while (millis() < dl && n < (int)sizeof(line) - 1) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') break;
        line[n++] = c;
      }
    }
    line[n] = 0;
    int slot; unsigned int cl, at;
    if (sscanf(line, "%d %x %x", &slot, &cl, &at) == 3 &&
        slot >= 0 && slot < MAX_PAIRED_EMITTERS && emitters[slot].valid) {
      pendingPoll[slot].valid   = true;
      pendingPoll[slot].cluster = (uint16_t)cl;
      pendingPoll[slot].attr    = (uint16_t)at;
      Serial.print(F("[POLL] arme slot "));   Serial.print(slot);
      Serial.print(F(" cluster=0x"));         Serial.print(cl, HEX);
      Serial.print(F(" attr=0x"));            Serial.println(at, HEX);
      Serial.println(F("[POLL] envoi au prochain uplink de ce ZLinky"));
    } else {
      Serial.println(F("[POLL] usage: Q <slot> <cluster_hex> <attr_hex>"));
      Serial.println(F("       ex: Q 0 FF66 300  (mode Linky du slot 0)"));
    }
  }
  else if (cmd == 'S' || cmd == 's') {
    Serial.println(F("\r\n========== STATUS =========="));
    Serial.print(F("op_channel = "));
    Serial.print(opChannel);
    Serial.print(F(" ("));
    Serial.print(LORA_CHANNEL_FREQS_MHZ[opChannel], 0);
    Serial.println(F(" MHz)"));
    Serial.print(F("op_SF      = SF"));
    Serial.println(opSF);
    Serial.print(F("Paired emitters : "));
    Serial.print(countPairedEmitters());
    Serial.print(F("/"));
    Serial.println(MAX_PAIRED_EMITTERS);
    for (int i = 0; i < MAX_PAIRED_EMITTERS; i++) {
      Serial.print(F("  Slot "));
      Serial.print(i);
      Serial.print(F(": "));
      if (!emitters[i].valid) {
        Serial.println(F("(empty)"));
        continue;
      }
      Serial.print(F("MAC="));
      for (int b = 0; b < MAC_SIZE; b++) {
        if (emitters[i].mac[b] < 0x10) Serial.print('0');
        Serial.print(emitters[i].mac[b], HEX);
      }
      Serial.print(F(" rx="));
      Serial.print(emitters[i].rxCount);
      Serial.print(F(" miss="));
      Serial.println(emitters[i].missedPackets);
    }
    Serial.print(F("State : "));
    switch (rxPairState) {
      case RX_PAIR_LISTEN: Serial.println(F("PAIR_LISTEN")); break;
      case RX_PAIRED:      Serial.println(F("PAIRED")); break;
      case RX_IDLE:        Serial.println(F("IDLE")); break;
    }
    Serial.println(F("============================\r\n"));
  }
  else if (cmd == 'R' || cmd == 'r') {
    /* "R N" : remove emitter at slot N */
    while (!Serial.available()) { delay(1); }
    int n = Serial.parseInt();
    if (n < 0 || n >= MAX_PAIRED_EMITTERS) {
      Serial.print(F("[CMD] Invalid slot "));
      Serial.println(n);
      return;
    }
    if (!emitters[n].valid) {
      Serial.print(F("[CMD] Slot "));
      Serial.print(n);
      Serial.println(F(" already empty"));
      return;
    }
    Serial.print(F("[CMD] Removing slot "));
    Serial.println(n);
    emitters[n].valid = false;
    if (countPairedEmitters() == 0) isPaired = false;
    savePairingToEEPROM();
  }
  else if (cmd == 'O' || cmd == 'o') {
    /* "OTA_GW_INIT <mac_hex>"  ou  "OTA_TX <hex_packet>" */
    handleOtaGatewayCommand();
  }
  else if (cmd == 'P' || cmd == 'p') {
    Serial.print(F("[CMD] Forcing PAIR_LISTEN mode (30s) - existing emitters: "));
    Serial.print(countPairedEmitters());
    Serial.print(F("/"));
    Serial.println(MAX_PAIRED_EMITTERS);
    rxPairState = RX_PAIR_LISTEN;
    pairListenStart = millis();
    /* Note: on garde les emitters deja paired */
    setRadioPairing();   /* canal 3 + SF11 : rendez-vous d'appairage */
    radio.setDio1Action(rxDone);
    radio.startReceive(0xFFFF);
  }
  else if (cmd == 'E' || cmd == 'e') {
    Serial.println(F("[CMD] Erasing pairing data..."));
    erasePairingFromEEPROM();
    rxPairState = RX_IDLE;
    Serial.println(F("[CMD] Pairing erased."));
  }
  else {
    Serial.print(F("[CMD] Unknown command '"));
    Serial.print(cmd);
    Serial.println(F("'. Use C N / S / P / E"));
  }

  /* drain rest of line */
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
  }
}

/* ====================================================================== */
/* ====== Loop ====== */
/* ====================================================================== */
void loop() {
  /* ---- Serial commands (C/S/P/E) ---- */
  handleSerialCommand();

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
        Serial.print(F("[BTN] Existing emitters : "));
        Serial.print(countPairedEmitters());
        Serial.print(F("/"));
        Serial.println(MAX_PAIRED_EMITTERS);
        rxPairState = RX_PAIR_LISTEN;
        pairListenStart = millis();
        /* Note: on NE met PAS isPaired=false : on garde les emetteurs deja paired */

        /* Garder le opChannel courant.
         * Il a deja ete defini soit:
         *   - par l'EEPROM au boot (si paired)
         *   - par DEFAULT_OP_CHANNEL au boot (si vide + DEFAULT defini)
         *   - par CCA scan au boot (si vide + DEFAULT == -1)
         *   - par la commande Serial 'C N'
         * On NE le re-evalue PAS ici sinon on ecrase le choix utilisateur. */
        if (!APP_isValidChannel(opChannel)) {
          /* Cas defensif : opChannel invalide → fallback */
          if (DEFAULT_OP_CHANNEL >= 0 && DEFAULT_OP_CHANNEL < LORA_NUM_CHANNELS) {
            opChannel = DEFAULT_OP_CHANNEL;
          } else {
            opChannel = ccaScan();
          }
        }
        Serial.print(F("[PAIR] op_channel for next pairings = "));
        Serial.print(opChannel);
        Serial.print(F(" ("));
        Serial.print(LORA_CHANNEL_FREQS_MHZ[opChannel], 0);
        Serial.println(F(" MHz)"));

        /* Rendez-vous d'appairage : canal 3 + SF11 */
        setRadioPairing();
        radio.setDio1Action(rxDone);
        radio.startReceive(0xFFFF);
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
        /* Switch back to operational config (canal + SF) for data RX */
        setRadioOp();
        radio.setDio1Action(rxDone);
        radio.startReceive(0xFFFF);
      } else {
        rxPairState = RX_IDLE;
        Serial.println(F("[PAIR] Pairing window expired (30s). Not paired."));
        Serial.println(F("[PAIR] Press USER button or reboot to re-enter pairing mode.\r\n"));
        /* Stay on op_channel anyway (chosen by CCA scan) for future emitters */
        setChannel(opChannel);
        radio.setDio1Action(rxDone);
        radio.startReceive(0xFFFF);
      }
    }
  }

  /* En mode OTA gateway, l'ACK est consume directement par otaTxAndWaitAck().
   * On ne traite pas les data packets normaux ici. */
  if (rxPairState == RX_OTA_GATEWAY) {
    return;
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

      /* Uplink recu → la fenetre RX de cet emetteur est ouverte. Si une
       * lecture d'attribut est en attente pour lui, c'est le moment. */
      int slot = currentSlotForStats;
      if (slot >= 0 && slot < MAX_PAIRED_EMITTERS && pendingPoll[slot].valid) {
        sendPollAndWait(slot);
      }

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

#ifndef SX1280_CONFIG_H
#define SX1280_CONFIG_H

// Configuration SPI0
#define SX1280_SPI_INSTANCE         SPI0

// Pins SPI0
#define SX1280_SPI_SCK_PIN          2       // PIO2
#define SX1280_SPI_MISO_PIN         3       // PIO3
#define SX1280_SPI_MOSI_PIN         4       // PIO4

// Pins de controle SX1280 (verifiees sur schema Zlinky)
// NOTE: PIO11 = cellule I2C -> besoin de IOCON_GPIO_MODE (EGP=1) pour push-pull
// NOTE: PIO12 = cellule I2C + debug SWDIO -> besoin de EGP + desactiver DBG_MODE
#define SX1280_NSS_PIN              11      // PIO11 - Chip Select (J3 pin 7) [I2C-type]
#define SX1280_RESET_PIN            12      // PIO12 - Reset (J3 pin 5) [I2C-type + SWDIO]
#define SX1280_BUSY_PIN             15      // PIO15 - Busy (J3 pin 1) [MFIO + ADC1]
#define SX1280_DIO1_PIN             13      // PIO13 - DIO1 (J3 pin 3) [MFIO]

// TCXO Enable - pin 14 du module NiceRF LORA1280-TCXO
// CRITIQUE : cette pin doit etre a HIGH pour alimenter le TCXO 52MHz.
// Sans TCXOEN=HIGH, le SX1280 n'a pas d'horloge de reference et reste
// bloque en STDBY_RC (impossible de passer en FS/TX/RX).
// Doit etre a HIGH AVANT le reset du SX1280, avec 3ms de delai minimum.
// Choisir un GPIO libre du JN5189 (PIO14 par defaut).
#define SX1280_TCXOEN_PIN           14      // PIO14 - TCXO Enable (module pin 14) [MFIO]

// Port GPIO
#define SX1280_GPIO_PORT            0

// Configuration SPI
// Note: 1 MHz au lieu de 4 MHz pour fiabiliser la communication
// sur fils Dupont (longueur ~10-15cm, pas de plan de masse).
// Le SX1280 supporte jusqu'a 18 MHz, mais l'integrite du signal
// sur des fils volants est limitee.
#define SX1280_SPI_BAUDRATE         1000000  // 1 MHz (debug, fils Dupont)

#endif // SX1280_CONFIG_H

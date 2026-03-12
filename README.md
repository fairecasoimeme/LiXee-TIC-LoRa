# LiXee TIC LoRa

Firmware pour le **ZLinky** avec transmission des donnees Linky par **LoRa 2.4 GHz** (SX1280).

Le ZLinky TIC est un capteur autoalimente par la sortie TIC du compteur Linky. Il lit les trames de teleinformation (TIC) et les transmet sans fil vers un recepteur distant.

## Architecture

```
Compteur Linky --[UART/TIC]--> JN5189 --[SPI]--> SX1280 --[LoRa 2.4GHz]--> Recepteur
```

| Element       | Details                                          |
|---------------|--------------------------------------------------|
| MCU           | NXP JN5189 (ARM Cortex-M4, 12 MHz)              |
| Radio LoRa    | NiceRF LORA1280-TCXO (SX1280, 2.4 GHz, +10 dBm)|
| Alimentation  | Autoalimente par la sortie TIC du Linky (~130 mW)|
| Modulation    | LoRa SF10 / BW406 kHz / CR 4/5                  |
| Chiffrement   | AES-128-CTR + CMAC (cle negociee par pairing)    |
| Recepteur     | STM32 Nucleo-64 (ou tout MCU avec SX1280)        |

## Fonctionnalites

- Lecture TIC Linky mode **Standard** (9600 baud) et **Historique** (1200 baud)
- Transmission LoRa chiffree AES-128 des donnees TIC
- **Pairing securise** : echange de cle AES-128 entre emetteur et recepteur
- Paquets **ESSENTIAL** (puissance apparente, STGE, ADPS) toutes les secondes
- Paquets **EXTENDED** (energie, courants, tensions, tarifs) toutes les 10 secondes
- Stockage persistant de la cle de pairing (PDM / EEPROM)
- LED de diagnostic (clignotement rapide = pairing, lent = normal, rapide = erreur TIC)

## Protocole de pairing

L'appairage se declenche par un **appui long (3s)** sur le bouton de l'emetteur ET du recepteur :

```
Emetteur                          Recepteur
   |                                  |
   |--- PAIR_REQUEST (MAC) ---------> |
   |                                  | Genere cle AES-128
   | <------ PAIR_RESPONSE (cle) ---- |
   |                                  |
   |--- PAIR_CONFIRM (preuve AES) --> |
   |                                  | Verifie preuve
   |       PAIRED                     |       PAIRED
```

Au premier boot (PDM vide), l'emetteur entre automatiquement en mode pairing.

## Paquets LoRa

### ESSENTIAL (toutes les secondes)
| Offset | Contenu               | Taille |
|--------|-----------------------|--------|
| 0      | Type (0x11)           | 1      |
| 1      | Sequence              | 1      |
| 2-3    | SINSTS total (VA)     | 2      |
| 4-9    | SINSTS 1/2/3 (VA)     | 6      |
| 10-13  | STGE (binaire)        | 4      |
| 14-15  | ADPS                  | 2      |
| 16     | Mode TIC              | 1      |
| 17-18  | MIC (AES-CMAC)        | 2      |

### EXTENDED (toutes les 10 secondes, 7 sous-types en rotation)
Courants/tensions, energie (index tarifaires), tensions moyennes, puissances max, energie journaliere, pointes mobiles.

## Structure du projet

```
Common/Source/
  sx1280.c / sx1280.h          # Driver SX1280 haut niveau
  board/sx1280_hal.c            # HAL SPI pour JN5189
  board/sx1280_config.h         # Configuration pins SPI
  app_lora_tic.c / .h           # Protocole TX periodique LoRa TIC
  app_lora_pair.c / .h          # Machine a etats pairing + echange cle
  app_lora_crypt.c / .h         # Chiffrement AES-128-CTR + CMAC
  app_uartlinky.c / .h          # UART + parsing TIC Linky
  app_buttons.c / .h            # Gestion bouton (GINT + debounce)

ZLinky/Source/
  app_start.c                   # Init hardware, SX1280, clocks
  app_main.c                    # Boucle principale, timers ZTIMER
  app_router_node.c             # Dispatch events, init modules
  App_Linky.c / .h              # Acquisition TIC (Standard + Historique)
  app_blink_led.c / .h          # Gestion LED clignotante

Receiver/
  ZLinky_TIC_Receiver.ino       # Recepteur Arduino (STM32 Nucleo-64)

ZLinky/Build/mcux/Makefile      # Build Makefile
```

## Prerequis

- **NXP MCUXpresso IDE** 11.2.0+
- **SDK JN5189DK6** v2.6.4 (a placer dans `ZLinky/SDK_2_6_4_JN5189DK6/`)
- **GNU ARM Embedded Toolchain** (inclus dans MCUXpresso)

## Build

```bash
cd ZLinky/Build/mcux
make -j4
```

Le binaire est genere dans `Binaries/ZLinky_GpProxy/ZLinky_GpProxy.bin`.

## Flash

Utiliser **DK6Programmer** (inclus dans le SDK NXP) ou MCUXpresso pour flasher le JN5189 via la sonde de debug CMSIS-DAP.

## Pin Mapping JN5189

| Signal   | Pin JN5189 | Fonction           |
|----------|------------|--------------------|
| SPI_SCK  | PIO2       | SPI0 clock         |
| SPI_MISO | PIO3       | SPI0 data in       |
| SPI_MOSI | PIO4       | SPI0 data out      |
| NSS      | PIO11      | GPIO (chip select)  |
| RESET    | PIO12      | GPIO (SX1280 reset) |
| DIO1     | PIO13      | GPIO (IRQ TX/RX)    |
| TCXOEN   | PIO14      | GPIO (TCXO enable)  |
| BUSY     | PIO15      | GPIO (SX1280 busy)  |
| LED      | PIO10      | GPIO (diagnostic)   |
| UART RX  | PIO1       | USART1 (TIC Linky)  |
| Button   | PIO0       | GINT (pairing)      |

## LED

| Etat               | Pattern LED                    |
|---------------------|-------------------------------|
| Pairing en cours    | Clignotement rapide (~4 Hz)   |
| TIC OK              | 1 flash toutes les 3 secondes |
| TIC en erreur       | 1 flash par seconde           |
| Non appaire (boot)  | Auto-pairing immediat         |

## Licence

Firmware base sur NXP JN-AN-1243 (Zigbee 3.0 Base Device).
Voir [Doc/EULA.txt](Doc/EULA.txt) pour les conditions de licence NXP.

Modules LoRa (SX1280 driver, pairing, chiffrement) developpes par [LiXee](https://lixee.fr/).

# Zlinky TIC SX1280 - Project Guide

## Overview
Firmware for the **ZLinky** device: a Zigbee-based energy monitor interfacing with a French Linky smart meter (TIC protocol) via UART, with an **SX1280** 2.4 GHz LoRa radio module for additional wireless transmission.

**MCU:** NXP JN5189 (ARM Cortex-M4, SDK JN5189DK6)
**IDE:** MCUXpresso
**Radio LoRa:** NiceRF LORA1280-TCXO (SX1280, SPI, LDO mode)
**Protocol Zigbee:** Zigbee 3.0 (router)

## Architecture

```
Linky Meter --[UART/TIC]--> JN5189 --[Zigbee 3.0]--> Coordinator
                              |
                         [SPI] --> SX1280 --[LoRa 2.4GHz]--> Receiver
```

## Project Structure

### Source principale
- `ZLinky/Source/app_start.c` - Point d'entree, init SX1280 + test TX LoRa
- `ZLinky/Source/app_main.c` - Boucle principale (Zigbee tasks, timers, power)
- `ZLinky/Source/App_Linky.c` - Acquisition TIC Linky (Standard + Historique)
- `ZLinky/Source/app_router_node.c` - Noeud routeur Zigbee

### Driver SX1280
- `Common/Source/sx1280.c` / `sx1280.h` - Driver haut niveau SX1280
- `Common/Source/board/sx1280_hal.c` / `sx1280_hal.h` - HAL SPI
- `Common/Source/board/sx1280_config.h` - Configuration pins SPI

### Communication Linky
- `Common/Source/app_uartlinky.c` / `app_uartlinky.h` - UART + parsing TIC

### Board / Hardware
- `Common/Source/board/pin_mux.c` - Multiplexage pins
- `Common/Source/board/hardware_init.c` - Init clocks, peripheriques
- `Common/Source/board/board.c` - Debug console

### Assets
- `asset/` - Schemas (JN5189, connecteur SPI), datasheets SX1280

## Pin Mapping JN5189 <-> SX1280

| Signal    | JN5189 Pin | J3 Pin | IOCON Func | Direction | IO Cell Type     | Notes                    |
|-----------|------------|--------|------------|-----------|------------------|--------------------------|
| SPI_SCK   | PIO2       | 8      | FUNC1      | Output    | MFIO             |                          |
| SPI_MISO  | PIO3       | 10     | FUNC1      | Input     | MFIO             |                          |
| SPI_MOSI  | PIO4       | 9      | FUNC1      | Output    | MFIO             |                          |
| NSS (CS)  | PIO11      | 7      | FUNC0/GPIO | Output    | **I2C-type**     | Besoin IOCON_GPIO_MODE   |
| RESET     | PIO12      | 5      | FUNC0/GPIO | Output    | **I2C + SWDIO**  | EGP=1, DBG_MODE=0       |
| BUSY      | PIO15      | 1      | FUNC0/GPIO | Input     | MFIO + ADC1      | DIGITAL_EN obligatoire   |
| DIO1      | PIO13      | 3      | FUNC0/GPIO | Input     | MFIO             |                          |
| TCXOEN    | PIO14      | -      | FUNC0/GPIO | Output    | MFIO             | Ou direct a VCC 3.3V     |

**SPI:** SPI0, 1 MHz, MSB first, CPOL=0 CPHA=0 (Mode 0), clock source: FRO 32MHz

### Module NiceRF LORA1280-TCXO (16 pins)
- SX1280 natif + TCXO 0.5ppm, pas de PA externe, max +12.5 dBm
- Alimentation: 1.8-3.7V, mode **LDO** (pas de DC-DC sur le PCB)
- **TCXOEN (pin 14) doit etre a HIGH (3.3V)** pour alimenter le TCXO 52MHz
- Pinout module: VCC=3, GND=4/10/11/12, MISO=5, MOSI=6, SCK=7, NSS=8, NRESET=13, **TCXOEN=14**, BUSY=15, DIO1=16

### Important: types de cellules I/O JN5189
- **PIO0-9, PIO14-21**: cellules MFIO standard
- **PIO10-11**: cellules **I2C-type** → sans `IOCON_GPIO_MODE` (EGP=1), restent en open-drain I2C (ne peuvent pas driver HIGH)
- **PIO12**: cellule I2C + **debug SWDIO** → necessite EGP=1 ET `IOCON_PIO_I2C_DBG_MODE(0)` pour desactiver le debug

## UART Linky
- **USART1** (PIO1 = RX seulement)
- Mode Historique: 1200 baud, 7 bits, parite paire
- Mode Standard: 9600 baud, 7 bits, parite paire

## Build
- MCUXpresso IDE project dans `mcux/`
- Makefile dans `ZLinky/Build/mcux/`
- SDK: `ZLinky/SDK_2_6_4_JN5189DK6/`

## Etat actuel
Module NiceRF LORA1280-TCXO (remplace le shield SX1280RF1ZHP).
Mode LDO (pas de DC-DC sur le module). TCXOEN relie a VCC 3.3V.
**TX LoRa fonctionnel** : SetFs OK, CW OK, Preamble OK, TX packet OK (IRQ TX_DONE confirmee).
Tests SPI + oscillateur + TX au boot, puis init Zigbee (APP_vInitialiseRouter).

## Conventions
- Langage: C (C99)
- Commentaires/logs: francais + anglais
- Prefixes: `SX1280_` (driver haut niveau), `SX1280_HAL_` (HAL SPI)
- Types NXP: `PUBLIC`, `PRIVATE`, `uint8`, `uint32`, `bool_t`

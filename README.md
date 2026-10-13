# LiXee TIC LoRa

Firmware du **ZLinky LoRa** : il lit la télé-information (TIC) d'un compteur
Linky et la transmet par **LoRa 2,4 GHz** vers un récepteur distant.

Le ZLinky est alimenté uniquement par la sortie TIC du compteur, sans pile ni
alimentation externe. Il vise les installations où le compteur est trop loin
du logement pour le Zigbee ou le Wi-Fi.

Version courante : **1.0.0** — voir [CHANGELOG.md](CHANGELOG.md).

## Architecture

```
Compteur Linky --[UART/TIC]--> JN5189 --[SPI]--> SX1280 --[LoRa 2,4 GHz]--> Récepteur
```

| Élément | Détail |
|---|---|
| MCU | NXP JN5189 (ARM Cortex-M4) |
| Radio | Semtech SX1280 / SX1281, 2,4 GHz, +10 dBm |
| Modulation | LoRa SF11, BW 406 kHz, CR 4/5, préambule de 8 symboles |
| Alimentation | Sortie TIC du Linky (~130 mW disponibles) |
| Chiffrement | AES-128-CTR + MIC CMAC, clé négociée à l'appairage |
| Portée mesurée | ~100 m en milieu urbain ; ~160 m en milieu dégagé avec dénivelé |

## Fonctionnalités

- **TIC Standard** (9600 bauds) et **Historique** (1200 bauds), monophasé et triphasé.
- **Appairage sécurisé** avec négociation de la clé, du canal et du SF.
- **Données essentielles** à chaque cycle : puissances apparentes, registre STGE, ADPS.
- **Données étendues** en 12 sous-types : index d'énergie, courants, tensions,
  puissances max, tarif en cours, numéro de série, alertes.
- **Remontée sur changement** : un changement de tarif ou une alerte passe en
  priorité, sans attendre son tour.
- **Lecture à la demande** (POLL) de tous les attributs TIC par le récepteur.
- **Mise à jour du firmware par radio** (OTA).
- **Version du firmware** lisible par radio et sur la console.

## Protocole radio

La référence complète est **[Doc/LORA_PROTOCOL.md](Doc/LORA_PROTOCOL.md)** :
formats de trames, chiffrement, appairage, cadences, POLL, OTA et
recommandations pour le récepteur. Ce qui suit n'en est qu'un résumé.

### Appairage

Appui long sur le bouton du ZLinky, récepteur en mode appairage :

```
ZLinky                               Récepteur
   |--- PAIR_REQUEST (MAC) ------------> |
   |                                     | génère la clé AES-128
   | <--- PAIR_RESPONSE (clé, canal, SF) |
   |--- PAIR_CONFIRM (preuve AES) -----> |
   |          APPAIRÉ                    |          APPAIRÉ
```

Au premier démarrage (aucune clé en mémoire), le ZLinky entre de lui-même en
appairage. Après l'appairage, le récepteur lit `device_id`, `model` et la
version du firmware par POLL.

### Cadence des trames

| Trame | Contenu | Période réelle |
|---|---|---|
| ESSENTIAL | puissances, STGE, ADPS | ~9 s |
| EXTENDED | un sous-type à la fois | ~37 s |

L'intervalle nominal est de 5 s entre deux cycles ; la lecture de la trame TIC
s'y ajoute, d'où ~9 s mesurés en Standard triphasé.

Les sous-types EXTENDED sont tous émis une fois après un démarrage, puis
suivent une table pondérée : les index d'énergie reviennent toutes les
~2 minutes, les données statiques toutes les ~15 minutes.

### Lecture à la demande (POLL)

Le ZLinky écoute pendant 300 ms après chacune de ses émissions. Le récepteur
peut y déposer une requête de lecture d'attribut, adressée par cluster et
attribut comme en Zigbee.

### Version du firmware

Numérotée `MAJEUR.MINEUR.CORRECTIF`, définie dans
[Common/Source/app_version.h](Common/Source/app_version.h).

| Où | Forme |
|---|---|
| Console série au démarrage | `=== ZLinky LoRa - firmware 1.0.0+971cfc0 (0x0100) ===` |
| POLL, cluster `0x0000`, attribut `0xFF01` | uint16 `0x0100` |
| POLL, cluster `0x0000`, attribut `0x4000` | chaîne `1.0.0+971cfc0` |

Le suffixe après `+` est le commit git du build. `-dirty` signale un build fait
avec des sources non commitées.

### Mise à jour par radio (OTA)

Maintenir le bouton du ZLinky à la mise sous tension : il passe en mode OTA et
écoute pendant 5 minutes. Le récepteur envoie le firmware par blocs de
64 octets, acquittés par groupes pour respecter le budget d'alimentation TIC.
Compter environ 30 minutes pour 200 ko.

Voir [Doc/LORA_PROTOCOL.md](Doc/LORA_PROTOCOL.md) §10 et
[Tools/OTA_README.md](Tools/OTA_README.md).

## Structure du projet

```
Common/Source/
  sx1280.c / .h              Driver SX1280
  board/sx1280_hal.c / .h    HAL SPI pour le JN5189
  board/sx1280_config.h      Broches SPI
  app_lora_tic.c / .h        Émission périodique des trames TIC
  app_lora_pair.c / .h       Appairage et échange de clé
  app_lora_crypt.c / .h      Chiffrement AES-128-CTR + CMAC
  app_lora_channels.c / .h   Plan de canaux
  app_lora_cmd.c / .h        Lecture d'attribut à la demande (POLL)
  app_lora_ota.c / .h        Mise à jour par radio
  app_version.c / .h         Version du firmware
  app_uartlinky.c / .h       UART TIC
  app_buttons.c / .h         Bouton

ZLinky/Source/
  app_start.c                Initialisation matérielle et radio
  app_main.c                 Boucle principale, timers
  app_router_node.c          Initialisation des modules
  App_Linky.c / .h           Acquisition et décodage TIC
  app_blink_led.c / .h       LED

Doc/LORA_PROTOCOL.md         Spécification du protocole radio
Tools/                       Outils OTA (script PC, guide)
Receiver/                    Récepteur Arduino de référence
ZLinky/Build/mcux/Makefile   Build
CHANGELOG.md                 Journal des versions
```

## Prérequis

- **NXP MCUXpresso IDE** 11.2.0 ou plus récent (fournit la chaîne GNU ARM et `make`)
- **SDK JN5189DK6** v2.6.4, à placer dans `ZLinky/SDK_2_6_4_JN5189DK6/` (non inclus dans le dépôt)
- **git** dans le PATH, pour que le binaire embarque le commit du build

## Build

```bash
cd ZLinky/Build/mcux
make
```

Le binaire est généré dans `Binaries/ZLinky_GpProxy/ZLinky_GpProxy.bin`.

## Flash

Avec **DK6Programmer** (fourni avec le SDK NXP) ou MCUXpresso. Les mises à
jour suivantes peuvent se faire par radio.

## Publier une version

1. Modifier les trois constantes de `Common/Source/app_version.h`.
2. Compléter `CHANGELOG.md`.
3. Commiter, **puis** compiler : dans l'autre ordre, le binaire porte `-dirty`.
4. Poser le tag git `vX.Y.Z`.

## Brochage JN5189

| Signal | Broche | Fonction |
|---|---|---|
| SPI_SCK | PIO2 | SPI0, horloge |
| SPI_MISO | PIO3 | SPI0, entrée |
| SPI_MOSI | PIO4 | SPI0, sortie |
| NSS | PIO11 | GPIO, sélection du SX1280 |
| RESET | PIO12 | GPIO, reset du SX1280 |
| DIO1 | PIO13 | GPIO, interruption radio |
| BUSY | PIO15 | GPIO, occupation du SX1280 |
| LED | PIO10 | GPIO |
| UART RX | PIO1 | USART1, TIC du Linky |
| Bouton | PIO0 | appairage, mode OTA |

## LED

| État | Clignotement |
|---|---|
| Appairage en cours | rapide |
| TIC correcte | 1 flash toutes les 3 secondes |
| TIC en erreur | 1 flash par seconde |

## Licence

Firmware basé sur NXP JN-AN-1243 (Zigbee 3.0 Base Device).
Voir [Doc/EULA.txt](Doc/EULA.txt) pour les conditions de licence NXP.

Modules LoRa (driver SX1280, appairage, chiffrement, protocole, OTA) développés
par [LiXee](https://lixee.fr/).

# Protocole LoRa ZLinky ⇄ LiXee-Box

Spécification des trames radio entre un **ZLinky LoRa** (émetteur, JN5189 + SX1280)
et le **récepteur** (LiXee-Box / passerelle).

Version protocole : **v1** — dernière mise à jour : firmware ZLinky LoRa 2026-05.

---

## 1. Couche radio

| Paramètre | Valeur |
|---|---|
| Chip | SX1280 / SX1281 (LoRa 2.4 GHz) |
| Spreading Factor | **SF11** |
| Bandwidth | **BW 406 kHz** |
| Coding Rate | **4/5** |
| Preamble | 16 symboles (encodé `0x18`) |
| Header | Explicit |
| CRC | ON |
| IQ | Normal |
| Sync Word | registre `0x0944` = `{0x14, 0x24}` (équivalent RadioLib `setSyncWord(0x12)`) |
| TX Power | **+10 dBm** — limite ETSI EU (EN 300 440) pour la bande 2.4 GHz ISM |
| Regulator | LDO |

### Plan de fréquences (8 canaux)

| Canal | Fréquence | Note |
|---|---|---|
| 0 | 2 410 MHz | bas de bande |
| 1 | 2 420 MHz | |
| 2 | 2 430 MHz | |
| **3** | **2 440 MHz** | **PAIR_CHANNEL** (canal d'appairage, fixe et connu des deux côtés) |
| 4 | 2 450 MHz | |
| 5 | 2 460 MHz | |
| 6 | 2 470 MHz | |
| 7 | 2 480 MHz | haut de bande |

L'appairage se fait **toujours sur le canal 3**. Le récepteur assigne ensuite un
**op_channel** (0..7) que l'émetteur utilisera pour toutes les trames de données.

---

## 2. Format général d'une trame

```
┌────────┬────────┬───────────────────────────┬─────────┐
│ byte 0 │ byte 1 │      payload chiffré      │  MIC    │
│ ver|typ│  seq   │      (AES-128-CTR)        │  (2 B)  │
└────────┴────────┴───────────────────────────┴─────────┘
   clair    clair                                  clair
```

- **byte[0]** : `(version << 4) | type` — version = `0x1`
- **byte[1]** : `seq` — compteur de séquence uint8, incrémenté à chaque trame,
  wrap naturel à 255→0
- **byte[2..N-3]** : payload, **chiffré AES-128-CTR**
- **byte[N-2..N-1]** : MIC = 2 premiers octets du CMAC

> ⚠️ Les trames de **pairing** (types `0x03`, `0x04`, `0x05`) ne sont **PAS chiffrées**
> ni MIC-ées : elles circulent en clair.

### Types de paquets (byte[0] bits [3:0])

| Type | Nom | Sens | Chiffré |
|---|---|---|---|
| `0x01` | DATA_ESSENTIAL | device → box | ✅ |
| `0x02` | DATA_EXTENDED | device → box | ✅ |
| `0x03` | PAIR_REQUEST | device → box | ❌ |
| `0x04` | PAIR_RESPONSE | box → device | ❌ |
| `0x05` | PAIR_CONFIRM | device → box | ❌ |
| `0x06` | OTA_START | box → device | ✅ |
| `0x07` | OTA_BLOCK | box → device | ✅ |
| `0x08` | OTA_END | box → device | ✅ |
| `0x09` | OTA_ACK | device → box | ✅ |
| `0x0A` | OTA_STATUS | device → box | ✅ |
| `0x0B` | DOWNLINK_CMD | box → device | ✅ |
| `0x0C` | DOWNLINK_ACK | device → box | ✅ |

Donc `byte[0]` vaut `0x11` pour ESSENTIAL, `0x12` pour EXTENDED, `0x13` pour
PAIR_REQUEST, etc.

---

## 3. Chiffrement

### AES-128-CTR

- **Clé** : AES-128 partagée, échangée au pairing
- **Nonce / compteur initial (16 octets)** :

```
┌──────────────────────┬──────────────────┬──────┐
│   MAC émetteur (8B)  │  0x00 × 7        │ seq  │
│      [0..7]          │    [8..14]       │ [15] │
└──────────────────────┴──────────────────┴──────┘
```

- Le payload est padé à un multiple de 16, chiffré, puis tronqué à la longueur utile.
- **Header (`type`, `seq`) non chiffré** — nécessaire pour reconstruire le nonce.

### MIC (CMAC tronqué)

- Calculé en **AES-128-CMAC sur le message EN CLAIR**, header inclus (`type`+`seq`+payload).
- On garde les **2 premiers octets** du tag CMAC.

### Ordre des opérations

**Émission :**
1. Calculer `MIC = CMAC(clair[0..N-1], key)[0..1]`
2. Chiffrer `clair[2..N-1]` en AES-CTR
3. Appender `MIC` → longueur finale `N + 2`

**Réception :**
1. Extraire `MIC_reçu` = 2 derniers octets
2. Déchiffrer `pkt[2..N-3]` en AES-CTR (CTR est symétrique)
3. Calculer `MIC_attendu = CMAC(clair, key)[0..1]`
4. Comparer — si différent, **jeter silencieusement**

---

## 4. Appairage

Le récepteur ouvre une fenêtre de 30 s sur le **canal 3**. L'émetteur envoie un
PAIR_REQUEST toutes les 300 ms et écoute 200 ms après chaque envoi.

```
device                                box
  │                                    │
  │──── PAIR_REQUEST (ch 3) ─────────▶│
  │                                    │  génère clé AES-128
  │                                    │  choisit op_channel (CCA)
  │◀─── PAIR_RESPONSE (ch 3) ──────────│
  │                                    │
  │   bascule sur op_channel           │  bascule sur op_channel
  │                                    │
  │──── PAIR_CONFIRM (op_channel) ───▶│
  │                                    │  vérifie la preuve AES
  │                                    │  ✅ appairé
```

### 4.1 PAIR_REQUEST — device → box

**Format de base (11 octets)** — compatible récepteurs < v2.22 :

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x13` (version 1, type PAIR_REQUEST) |
| 1 | 1 | seq |
| 2 | 8 | MAC émetteur (big-endian) |
| 10 | 1 | canal (informatif, toujours `0`) |

**Format étendu (14+N octets)** — LiXee-Box ≥ v2.22, envoyé par défaut :

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x13` |
| 1 | 1 | seq |
| 2 | 8 | MAC émetteur |
| 10 | 1 | canal (informatif) |
| **11** | **2** | **device_id** (uint16 BE) — nomme le template `data/tp/<device_id>.json` |
| **13** | **1** | **longueur du model** (N) |
| **14** | **N** | **model** (ASCII, sans nul terminal) — clé dans ce fichier |

Pour le ZLinky LoRa : `device_id = 0x0051`, `model = "ZLinky_TIC"` (N = 10),
soit **24 octets** au total.

Exemple de trame :
```
13 05 00 15 8D 00 05 D2 AB E1 00 00 51 0A 5A 4C 69 6E 6B 79 5F 54 49 43
│  │  └───────── MAC ─────────┘ │  └─id─┘ │  └────── "ZLinky_TIC" ──────┘
│  seq                          ch        len=10
type
```

> Un récepteur ancien lit ses 11 premiers octets et ignore la queue — pas de casse.

### 4.2 PAIR_RESPONSE — box → device (20 octets)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x14` (type PAIR_RESPONSE) |
| 1 | 1 | seq |
| 2 | 16 | **clé AES-128** (⚠️ en clair sur l'air) |
| 18 | 1 | status (`0x00` = OK) |
| 19 | 1 | **op_channel** (0..7) — canal assigné pour les données |

Si `op_channel` est hors plage, l'émetteur retombe sur `PAIR_CHANNEL` (3).

### 4.3 PAIR_CONFIRM — device → box (15 octets)

Envoyé **sur l'op_channel** (pas sur le canal 3).

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x15` (type PAIR_CONFIRM) |
| 1 | 1 | seq |
| 2 | 8 | MAC émetteur |
| 10 | 4 | **preuve** = 4 premiers octets de `AES-128-ECB(MAC padé à 16B, netKey)` |
| 14 | 1 | status (`0x00` = OK) |

Le récepteur recalcule la preuve avec la clé qu'il a générée : si ça matche,
le device connaît bien la clé → appairage validé.

---

## 5. Cadencement des trames de données

| Élément | Période |
|---|---|
| DATA_ESSENTIAL | **toutes les 5 s** |
| DATA_EXTENDED | **1 tous les 4 ESSENTIAL** → toutes les **20 s** |
| Cycle complet des 11 sous-types EXTENDED | **~220 s** (~3 min 40) |

Les sous-types EXTENDED tournent en round-robin : 0x00 → 0x01 → … → 0x0A → 0x00 …

Un EXTENDED est envoyé **juste après** l'ESSENTIAL correspondant (piggyback),
en deux trames radio distinctes.

---

## 6. DATA_ESSENTIAL (type `0x01`)

**17 octets en clair, 19 octets sur l'air (avec MIC).**

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 0 | 1 | `0x11` | | |
| 1 | 1 | seq | | |
| 2-3 | 2 | Puissance apparente totale (VA, uint16 BE) | `SINSTS` | `PAPP` |
| 4-5 | 2 | Puissance apparente phase 1 (VA, uint16 BE) | `SINSTS1` | *(= PAPP)* |
| 6-7 | 2 | Puissance apparente phase 2 (VA, uint16 BE) | `SINSTS2` | 0 en mono |
| 8-9 | 2 | Puissance apparente phase 3 (VA, uint16 BE) | `SINSTS3` | 0 en mono |
| 10-13 | 4 | `STGE` — mot d'état (32 bits binaire) | `STGE` | 0 (absent) |
| 14-15 | 2 | `ADPS` — alerte dépassement (uint16 BE) | 0 (absent) | `ADPS` |
| 16 | 1 | **mode Linky** (voir §6.1) | | |

Les valeurs sont plafonnées à 65535 (cap uint16).

`STGE` est transmis en **binaire** : les 8 caractères hex ASCII de la TIC sont
convertis en 4 octets. Ex : `"003A1B2C"` → `0x00 0x3A 0x1B 0x2C`.

### 6.1 Encodage du mode Linky (byte 16)

Champ de bits :

| Bit | Signification |
|---|---|
| 0 | 0 = **Historique**, 1 = **Standard** |
| 1 | 0 = mono, 1 = **triphasé** |
| 2 | 1 = **production** (injection présente, étiquette `EAIT` vue) |

Valeurs observées :

| Valeur | Signification |
|---|---|
| 0 | Historique monophasé |
| 1 | Standard monophasé |
| 2 | Historique triphasé |
| 3 | Standard triphasé |
| 5 | Standard monophasé + production |
| 7 | Standard triphasé + production |

> **C'est le champ pivot du protocole.** Les paquets EXTENDED ne le portent pas
> (sauf POWER_MAX_CFG) : le récepteur doit **mémoriser le dernier mode reçu par
> émetteur** pour interpréter correctement les index Tier1..Tier6 (voir §7.2).

---

## 7. DATA_EXTENDED (type `0x02`)

**En-tête commun (3 octets) :**

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x12` |
| 1 | 1 | seq |
| 2 | 1 | **sub_type** (0x00..0x06) |

### 7.1 Sub-type `0x00` — CURRENT_VOLTAGE (15 octets)

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3-4 | 2 | Courant phase 1 (uint16 BE) | `IRMS1` (mA) | `IINST1` / `IINST` (A) |
| 5-6 | 2 | Courant phase 2 | `IRMS2` (mA) | `IINST2` (A) |
| 7-8 | 2 | Courant phase 3 | `IRMS3` (mA) | `IINST3` (A) |
| 9-10 | 2 | Tension phase 1 (V, uint16 BE) | `URMS1` | 0 (absent) |
| 11-12 | 2 | Tension phase 2 | `URMS2` | 0 (absent) |
| 13-14 | 2 | Tension phase 3 | `URMS3` | 0 (absent) |

⚠️ **Unité des courants** : mA en Standard, **A** en Historique.

### 7.2 Sub-type `0x01` — ENERGY (19 octets)

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3-6 | 4 | Index total (Wh, uint32 BE) | `EAST` | `BASE` *(uniquement si option BASE)* |
| 7-10 | 4 | Index injecté (Wh, uint32 BE) | `EAIT` | 0 (absent) |
| 11-14 | 4 | **Index tarifaire 1** (Wh, uint32 BE) | `EASF01` | voir table §7.2.1 |
| 15-18 | 4 | **Index tarifaire 2** (Wh, uint32 BE) | `EASF02` | voir table §7.2.1 |

#### 7.2.1 Interprétation des index tarifaires

L'émetteur remplit **les mêmes champs Tier1..Tier6** dans les deux modes.
Le récepteur doit résoudre le nom TIC réel à partir du **mode** (§6.1) et du
**code tarif** (§7.5) :

| Tier | Standard | Histo BASE | Histo HC/HP | Histo EJP | Histo Tempo |
|---|---|---|---|---|---|
| 1 | `EASF01` | `BASE` | `HCHC` | `EJPHN` | `BBRHCJB` |
| 2 | `EASF02` | — | `HCHP` | `EJPHPM` | `BBRHPJB` |
| 3 | `EASF03` | — | — | — | `BBRHCJW` |
| 4 | `EASF04` | — | — | — | `BBRHPJW` |
| 5 | `EASF05` | — | — | — | `BBRHCJR` |
| 6 | `EASF06` | — | — | — | `BBRHPJR` |

Les cases « — » signifient que le tier n'a **aucune signification** dans cette
configuration : la valeur transmise n'est pas pertinente, à masquer.

**L'option tarifaire Historique se déduit du code tarif** :

| Code tarif (§7.5) | Option déduite |
|---|---|
| 1 | BASE |
| 2, 3 | HC/HP |
| 4, 5 | EJP |
| 6..11 | Tempo |

### 7.3 Sub-type `0x02` — ENERGY_2 (19 octets)

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3-6 | 4 | Index tarifaire 3 (Wh, uint32 BE) | `EASF03` | `BBRHCJW` *(Tempo seul)* |
| 7-10 | 4 | Index tarifaire 4 | `EASF04` | `BBRHPJW` *(Tempo seul)* |
| 11-14 | 4 | Index tarifaire 5 | `EASF05` | `BBRHCJR` *(Tempo seul)* |
| 15-18 | 4 | Index tarifaire 6 | `EASF06` | `BBRHPJR` *(Tempo seul)* |

Hors Tempo (mode Historique), ces 4 champs sont **sans signification**.

### 7.4 Sub-type `0x03` — VOLTAGE_STATS (15 octets)

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3-4 | 2 | Tension moyenne phase 1 (V, uint16 BE) | `UMOY1` | 0 (absent) |
| 5-6 | 2 | Tension moyenne phase 2 | `UMOY2` | 0 (absent) |
| 7-8 | 2 | Tension moyenne phase 3 | `UMOY3` | 0 (absent) |
| 9-10 | 2 | Courant max phase 1 (uint16 BE) | *(SMAXSN1)* | `IMAX1` / `IMAX` (A) |
| 11-12 | 2 | Courant max phase 2 | | `IMAX2` (A) |
| 13-14 | 2 | Courant max phase 3 | | `IMAX3` (A) |

### 7.5 Sub-type `0x04` — POWER_MAX_CFG (18 octets)

**Paquet clé** : c'est le seul EXTENDED qui porte à la fois le **mode** et le **tarif**.

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3-4 | 2 | Puissance max soutirée **du jour** (VA, uint16 BE) | `SMAXSN` | *(voir note)* |
| 5-6 | 2 | Puissance max injectée jour (VA, uint16 BE) | `SMAXIN` | 0 (absent) |
| 7-8 | 2 | Puissance apparente injectée (VA, uint16 BE) | `SINSTI` | 0 (absent) |
| 9-10 | 2 | Point courbe de charge injectée (VA, uint16 BE) | `CCAIN` | 0 (absent) |
| 11-12 | 2 | `RELAIS` (uint16 BE) | `RELAIS` | 0 (absent) |
| 13 | 1 | `NTARF` — numéro tarif (uint8) | `NTARF` (1..10) | 0 (absent) |
| 14 | 1 | Seuil de coupure (kVA, uint8) | `PCOUP` | 0 (absent) |
| 15 | 1 | Puissance/intensité souscrite (uint8) | `PREF` (kVA) | `ISOUSC` (A) |
| 16 | 1 | **mode Linky** (même encodage que §6.1) | | |
| 17 | 1 | **code tarif unifié** (voir §7.5.1) | | |

> `SMAXSN` est la puissance max soutirée **du jour en cours**. La valeur de la
> veille (`SMAXSN-1`) n'est pas transmise.

#### 7.5.1 Code tarif unifié (byte 17)

**Mode Standard** — reprend `NTARF` :

| Valeur | Signification |
|---|---|
| 0 | inconnu / pas encore renseigné |
| 1..10 | index du tarif en cours (= `NTARF`) |

**Mode Historique** — code dérivé de `PTEC` :

| Valeur | `PTEC` | Signification |
|---|---|---|
| 0 | — | inconnu |
| 1 | `TH..` / `BASE` | BASE |
| 2 | `HP..` | Heures Pleines |
| 3 | `HC..` | Heures Creuses |
| 4 | `HN..` | EJP heures normales |
| 5 | `PM..` | EJP pointe mobile |
| 6 | `HCJB` | Tempo Bleu HC |
| 7 | `HPJB` | Tempo Bleu HP |
| 8 | `HCJW` | Tempo Blanc HC |
| 9 | `HPJW` | Tempo Blanc HP |
| 10 | `HCJR` | Tempo Rouge HC |
| 11 | `HPJR` | Tempo Rouge HP |

C'est **ce champ qui permet de déduire l'option tarifaire Historique** et donc de
nommer correctement les index Tier1..Tier6 (§7.2.1).

### 7.6 Sub-type `0x05` — DAILY_ENERGY (19 octets)

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3-6 | 4 | Énergie journalière J (Wh, uint32 BE) | `EASD01` | 0 (absent) |
| 7-10 | 4 | Énergie journalière J-1 | `EASD02` | 0 (absent) |
| 11-14 | 4 | Énergie journalière J-2 | `EASD03` | 0 (absent) |
| 15-18 | 4 | Énergie journalière J-3 | `EASD04` | 0 (absent) |

### 7.7 Sub-type `0x06` — COMPLEMENT (9 octets)

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3 | 1 | `DPM1` — début pointe mobile 1 | `DPM1` | 0 (absent) |
| 4 | 1 | `FPM1` — fin pointe mobile 1 | `FPM1` | 0 (absent) |
| 5 | 1 | `DPM2` | `DPM2` | 0 (absent) |
| 6 | 1 | `FPM2` | `FPM2` | 0 (absent) |
| 7 | 1 | `DPM3` | `DPM3` | 0 (absent) |
| 8 | 1 | `FPM3` | `FPM3` | 0 (absent) |

### 7.8 Sub-type `0x07` — TARIFF_LABEL (6..22 octets, **taille variable**)

Libellé **texte** du tarif en cours, tel que le compteur l'annonce.

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3 | 1 | mode Linky (même encodage que §6.1) | | |
| 4 | 1 | code tarif unifié (même encodage que §7.5.1) | | |
| 5 | 1 | **longueur du libellé** (N, 0..16) | | |
| 6..6+N-1 | N | **libellé ASCII** (sans nul terminal) | `LTARF` | `PTEC` |

Taille totale du paquet = **6 + N**.

Le libellé est transmis **espaces de fin retirés** (la TIC pade `LTARF` à 16
caractères). Les caractères non imprimables coupent la chaîne.

Exemples de libellés :

| Mode | Valeurs typiques |
|---|---|
| Standard | `"BASE"`, `"HEURE CREUSE"`, `"HEURE PLEINE"`, `"HC BLEU"`, `"HP ROUGE"` |
| Historique | `"TH.."`, `"HC.."`, `"HP.."`, `"HCJB"`, `"HPJR"` |

> Côté émetteur les deux étiquettes partagent le même champ interne
> (`au8LinkyTarifPeriod`) : `LTARF` l'écrit en Standard, `PTEC` en Historique.
> C'est le champ `mode` [3] qui indique lequel des deux est transmis.

Exemple de trame (Standard, `LTARF = "HEURE CREUSE"`) :
```
12 2A 07 01 01 0C 48 45 55 52 45 20 43 52 45 55 53 45
│  │  │  │  │  │  └──────── "HEURE CREUSE" ─────────┘
│  │  │  │  │  len=12
│  │  │  │  code tarif (NTARF=1)
│  │  │  mode=1 (Std Mono)
│  │  sub_type
│  seq
type
```

### 7.9 Sub-type `0x08` — METER_SERIAL (4..16 octets, **taille variable**)

Numéro de série du compteur.

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3 | 1 | **longueur du numéro** (N, 0..12) | | |
| 4..4+N-1 | N | **numéro de série ASCII** (sans nul terminal) | `ADSC` | `ADCO` |

Taille totale du paquet = **4 + N**.

**Pas de champ mode** : `ADSC` et `ADCO` font tous deux 12 caractères et
partagent le même champ interne côté émetteur — le numéro est identique
quel que soit le mode.

Le numéro est transmis espaces de fin retirés ; les caractères non
imprimables coupent la chaîne.

Exemple de trame (`ADSC = "021761234567"`) :
```
12 2B 08 0C 30 32 31 37 36 31 32 33 34 35 36 37
│  │  │  │  └────────── "021761234567" ───────┘
│  │  │  len=12
│  │  sub_type
│  seq
type
```

### 7.10 Sub-type `0x09` — METER_DATE (4..17 octets, **taille variable**)

Horodate interne du compteur. **Étiquette `DATE`, mode Standard uniquement.**

| Offset | Taille | Champ |
|---|---|---|
| 3 | 1 | **longueur de l'horodate** (N, 0..13) |
| 4..4+N-1 | N | **horodate ASCII** `SAAMMJJhhmmss` (sans nul terminal) |

Taille totale du paquet = **4 + N**.

En mode Historique l'étiquette n'existe pas : `N = 0` et le paquet fait
4 octets — le récepteur peut l'ignorer.

Format de l'horodate :

| Champ | Position | Signification |
|---|---|---|
| `S` | 0 | saison **et état de l'horloge** (voir ci-dessous) |
| `AA` | 1-2 | année (2 chiffres) |
| `MM` | 3-4 | mois |
| `JJ` | 5-6 | jour |
| `hh` | 7-8 | heure |
| `mm` | 9-10 | minute |
| `ss` | 11-12 | seconde |

Le premier caractère porte **deux informations** :

| Valeur | Signification |
|---|---|
| `E` / `H` | été / hiver, **horloge synchronisée** |
| `e` / `h` | été / hiver, **horloge en mode dégradé** (non synchronisée) |
| espace | saison inconnue |

> Une minuscule signale que l'horloge du compteur n'est pas synchronisée :
> l'horodate est alors à considérer comme approximative.

Exemple de trame (`DATE = "E250508143022"` → 8 mai 2026 14:30:22, heure d'été,
horloge synchronisée) :
```
12 2C 09 0D 45 32 35 30 35 30 38 31 34 33 30 32 32
│  │  │  │  └────────── "E250508143022" ─────────┘
│  │  │  len=13
│  │  sub_type
│  seq
type
```

> ⚠️ **Fraîcheur** : ce sous-type ne passe qu'une fois par cycle round-robin,
> soit toutes les ~3 min 40. L'horodate reçue peut donc avoir jusqu'à ~3 min
> de retard — utilisable pour vérifier/dériver l'horloge du compteur, pas
> comme source de temps précise.

### 7.11 Sub-type `0x0A` — TARIFF_OPTION (16 octets)

Option tarifaire souscrite, couleur Tempo du lendemain, courbe de charge
soutirée. Les deux moitiés du paquet sont **mutuellement exclusives selon
le mode**, d'où leur regroupement.

| Offset | Taille | Champ | Standard | Historique |
|---|---|---|---|---|
| 3 | 1 | mode Linky (même encodage que §6.1) | | |
| 4-7 | 4 | Option tarifaire (4 ASCII) | `0x00` × 4 | `OPTARIF` |
| 8-11 | 4 | Couleur Tempo du lendemain (4 ASCII) | `0x00` × 4 | `DEMAIN` *(Tempo seul)* |
| 12-13 | 2 | Courbe de charge soutirée, point courant (**int16 BE**, VA) | `CCASN` | 0 |
| 14-15 | 2 | Courbe de charge soutirée, point précédent (**int16 BE**, VA) | `CCASN-1` | 0 |

Les champs ASCII sont **complétés par `0x00`** dès qu'un caractère non
imprimable est rencontré : un champ absent vaut `0x00 0x00 0x00 0x00`, ce qui
le distingue sans ambiguïté d'une valeur réelle.

**Valeurs de `OPTARIF`** : `"BASE"`, `"HC.."`, `"EJP."`, `"BBR("` (Tempo).

**Valeurs de `DEMAIN`** : `"BLEU"`, `"BLAN"`, `"ROUG"`, `"----"` (inconnue).

> ⚠️ **`CCASN` / `CCASN-1` sont signés** (int16), contrairement à la plupart
> des autres puissances du protocole qui sont en uint16 — la courbe de charge
> peut être négative en cas d'injection.

> **Pourquoi `OPTARIF` n'est pas transmis en Standard** : côté émetteur, le
> champ interne porte `OPTARIF` (4 car.) en Historique mais `NGTF` (nom du
> calendrier tarifaire fournisseur, 16 car.) en Standard. Transmettre ses
> 4 premiers caractères en Standard donnerait un `NGTF` tronqué et trompeur.
> `NGTF` reste donc non transmis (cf. §8).

> **`CCASN` et `CCAIN`** : `CCAIN` (courbe de charge **injectée**) est déjà
> transmis dans POWER_MAX_CFG [9-10] (§7.5). `CCASN` est sa contrepartie
> **soutirée**.

Exemple de trame (Historique Tempo, `OPTARIF = "BBR("`, `DEMAIN = "BLEU"`) :
```
12 2D 0A 00 42 42 52 28 42 4C 45 55 00 00 00 00
│  │  │  │  └─"BBR("─┘ └─"BLEU"─┘ └CCASN┘ └CCASN-1┘
│  │  │  mode=0 (Histo Mono)
│  │  sub_type
│  seq
type
```

---

## 7bis. Canal de commande descendant

Le récepteur peut envoyer des commandes au ZLinky (changement de canal, de SF,
d'intervalle, passage en OTA). Modèle **Class A façon LoRaWAN**.

### Principe

Le ZLinky ouvre une **fenêtre de réception de 300 ms juste après chaque TX**.
Le récepteur, qui vient de recevoir l'uplink, y dépose une commande en attente.

```
ZLinky ──── ESSENTIAL / EXTENDED ────▶ Récepteur
       ◀─── DOWNLINK_CMD (optionnel) ─┘   dans les 300 ms
       ──── DOWNLINK_ACK ─────────────▶
```

Chronologie dans la fenêtre :

| Instant | Événement |
|---|---|
| T+0 | fin du TX uplink |
| T+5 ms | ZLinky en RX (restauration IRQ DIO1 + `payloadLength`) |
| T+20 ms | récepteur a déchiffré, consulté sa file, démarre son TX |
| T+60 ms | préambule détecté par le ZLinky |
| T+300 ms | fenêtre fermée, retour en standby |

> **Pourquoi une fenêtre et pas une écoute permanente** : le SX1280 tire
> ~7-10 mA en RX contre ~0,6 mA en standby. Écouter en permanence coûterait
> ~30 mW, soit près du quart du budget TIC (130 mW). La fenêtre courte revient
> à ~1,5 mW, soit ~1 %.

### DOWNLINK_CMD — box → device (7..16 octets + MIC)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x1B` (version 1, type `0x0B`) |
| 1 | 1 | seq |
| 2 | 1 | **cmd_id** |
| 3-6 | 4 | **compteur** (uint32 BE) — anti-rejeu |
| 7.. | 0-9 | paramètres, selon `cmd_id` |

**Le compteur doit être strictement croissant.** Toute commande dont le
compteur est inférieur ou égal au dernier accepté est rejetée avec
`BAD_COUNTER`. C'est la protection contre le rejeu d'une commande capturée.

| `cmd_id` | Commande | Paramètres | Rollback |
|---|---|---|---|
| `0x00` | `PING` | — | — |
| `0x01` | `SET_CHANNEL` | [7] canal 0..7 | ✅ 60 s |
| `0x02` | `SET_SF` | [7] SF 7..12 | ✅ 60 s |
| `0x03` | `SET_TX_INTERVAL` | [7-8] secondes (uint16 BE, 3..240) | ✅ 60 s |
| `0x04` | `ENTER_OTA` | — | — |
| `0x05` | `CONFIRM` | — | valide la config en attente |

### DOWNLINK_ACK — device → box (9 octets + MIC)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x1C` (version 1, type `0x0C`) |
| 1 | 1 | seq |
| 2 | 1 | `cmd_id` acquitté |
| 3-6 | 4 | compteur (écho) |
| 7 | 1 | **statut** |
| 8 | 1 | info (valeur courante, selon la commande) |

| Statut | Signification |
|---|---|
| `0x00` | OK |
| `0x01` | `BAD_COUNTER` — rejeu détecté |
| `0x02` | `BAD_PARAM` — valeur hors bornes |
| `0x03` | `UNKNOWN_CMD` |
| `0x04` | `PENDING_CONFIRM` — appliqué, attend `CONFIRM` |
| `0x05` | `NOTHING_PENDING` — `CONFIRM` sans config en attente |

### Séquence de changement de config (avec retour arrière)

Les commandes qui touchent à la liaison radio ne sont **pas persistées
immédiatement**. Sans cela, un canal mal choisi ou une commande forgée rendrait
le module injoignable jusqu'à intervention physique sur le compteur.

```
1. box → device   SET_CHANNEL(7)
2. device         applique ch7, mémorise ch4, arme 60 s
3. device → box   DOWNLINK_ACK(PENDING_CONFIRM, 7)   ← émis sur le canal 7
4. box            bascule sur ch7, entend l'ACK
5. box → device   CONFIRM
6. device         annule le timer, écrit en PDM
7. device → box   DOWNLINK_ACK(OK)
```

**Si l'étape 4 ou 5 échoue** (le récepteur n'entend rien sur le nouveau canal),
le ZLinky restaure automatiquement l'ancienne config au bout de **60 secondes**
et redevient joignable là où le récepteur écoute.

Le point de retour est toujours la **dernière config confirmée** : deux
commandes successives sans confirmation intermédiaire n'écrasent pas la
sauvegarde d'origine.

### Persistance de la config

Les champs `SF` et `intervalle TX` sont stockés dans les 3 octets anciennement
réservés de l'enregistrement PDM d'appairage. **La taille de la structure est
inchangée** : un module appairé avec une version antérieure lit ces champs à 0,
valeur invalide qui déclenche le repli sur les défauts (SF11, 5 s). Les
appairages existants survivent donc à la mise à jour.

> ⚠️ **Limite connue** : le compteur anti-rejeu n'est pas persisté. Après un
> redémarrage du ZLinky il repart de 0, ce qui rouvre brièvement la possibilité
> de rejouer une commande capturée, jusqu'au prochain envoi du récepteur.
> Persister le compteur imposerait une écriture PDM par commande.

---

## 8. Étiquettes TIC non transmises

Pour information, ces étiquettes sont parsées par l'émetteur mais **ne sont pas
encore envoyées** en LoRa :

**Standard** : `EASF07`..`EASF10`, `ERQ1`..`ERQ4`, `NJOURF`, `NJOURF+1`,
`PJOURF+1`, `PPOINTE`, `SMAXIN-1`, `SMAXSN-1`, `SMAXSN1-1`..`SMAXSN3-1`,
`CCAIN-1`, `VTIC`, `NGTF`, `MSG1`, `MSG2`, `PRM`

**Historique** : `PEJP`, `HHPHC`, `PPOT`, `ADIR1`..`ADIR3`, `PMAX`, `MOTDETAT`

---

## 9. OTA (mise à jour firmware)

Voir `Tools/OTA_README.md` pour la procédure complète.

Résumé des trames (toutes chiffrées comme les DATA) :

| Type | Nom | Sens | Taille (clair) |
|---|---|---|---|
| `0x06` | OTA_START | box → device | 15 B |
| `0x07` | OTA_BLOCK | box → device | 68 B (4 + 64 de données) |
| `0x08` | OTA_END | box → device | 7 B |
| `0x09` | OTA_ACK | device → box | 7 B |
| `0x0A` | OTA_STATUS | device → box | 8 B |

### OTA_START (15 octets)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x16` |
| 1 | 1 | seq |
| 2-3 | 2 | version du nouveau firmware (uint16 BE) |
| 4-7 | 4 | taille du firmware en octets (uint32 BE) |
| 8-11 | 4 | CRC32 du firmware complet (uint32 BE) |
| 12-13 | 2 | nombre total de blocs (uint16 BE) |
| 14 | 1 | taille d'un bloc (toujours 64) |

### OTA_BLOCK (68 octets)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x17` |
| 1 | 1 | seq |
| 2-3 | 2 | index du bloc (uint16 BE) |
| 4-67 | 64 | données (dernier bloc padé à `0xFF`) |

Les blocs doivent arriver **séquentiellement**. Si un bloc hors-séquence arrive,
le device répond `OTA_ACK` avec `status = OUT_OF_RANGE` et `block_idx` = le bloc
qu'il attend → la gateway doit reprendre à cet index.

### OTA_END (7 octets)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x18` |
| 1 | 1 | seq |
| 2-5 | 4 | CRC32 du firmware complet (uint32 BE) |
| 6 | 1 | réservé (`0x00`) |

### OTA_ACK (7 octets)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x19` |
| 1 | 1 | seq (celui du paquet acquitté) |
| 2-3 | 2 | index du bloc (uint16 BE) — `0xFFFF` pour START/END |
| 4 | 1 | status (voir table) |
| 5-6 | 2 | réservé |

**Codes status :**

| Code | Signification |
|---|---|
| `0x00` | OK |
| `0x01` | BUSY |
| `0x02` | BLOCK_BAD_CRC |
| `0x03` | FLASH_ERROR |
| `0x04` | OUT_OF_RANGE |
| `0x05` | FW_TOO_BIG |
| `0x06` | FW_BAD_CRC |

### OTA_STATUS (8 octets)

| Offset | Taille | Champ |
|---|---|---|
| 0 | 1 | `0x1A` |
| 1 | 1 | seq |
| 2 | 1 | état courant de la machine OTA |
| 3-4 | 2 | nombre de blocs reçus (uint16 BE) |
| 5-6 | 2 | index du dernier bloc reçu (uint16 BE) |
| 7 | 1 | réservé |

---

## 10. Recommandations d'implémentation côté récepteur

1. **Mémoriser `mode` et `code tarif` par émetteur.** Les paquets EXTENDED
   (hors POWER_MAX_CFG et TARIFF_LABEL) ne portent pas le mode. Sans cette
   mémorisation, impossible de nommer correctement les index (`EASF01` vs
   `HCHC` vs `BBRHCJB`).

2. **Masquer les champs sans signification** plutôt que d'afficher des zéros :
   `EAIT`/`SMAXIN`/`SINSTI`/`CCAIN`/`URMS*`/`UMOY*` en Historique, tiers 3-6 hors Tempo.

3. **Attention aux unités** : courants en mA (Standard) vs A (Historique).

4. **Multi-émetteur** : essayer chaque clé connue au déchiffrement. Le MIC sert
   de discriminant — si aucune clé ne valide, jeter silencieusement (c'est du
   bruit ou un émetteur d'un autre réseau).

5. **Suivi de séquence par émetteur** (pas de compteur global) pour calculer un
   PDR correct.

6. **Avant le premier POWER_MAX_CFG** (~2 min après le boot), l'option tarifaire
   Historique est inconnue : afficher des libellés génériques (`Tier1`, `Tier2`)
   plutôt que de masquer la donnée.

# Journal des versions — firmware ZLinky LoRa

La version du firmware est définie dans `Common/Source/app_version.h`
(`MAJEUR.MINEUR.CORRECTIF`). Elle est affichée sur la console au démarrage et
lisible par radio (POLL, cluster Basic `0x0000` : attribut `0xFF01` en uint16,
attribut `0x4000` en chaîne). Voir `Doc/LORA_PROTOCOL.md` §8.

- **MAJEUR** : rupture de compatibilité radio — le récepteur doit être mis à jour.
- **MINEUR** : ajout compatible (nouveau sous-type, nouvel attribut, cadence).
- **CORRECTIF** : correction sans effet sur le protocole.

Publier une version : modifier `app_version.h`, compléter ce fichier, commiter,
compiler (le binaire embarque alors le commit), poser le tag git `vX.Y.Z`.

## 1.1.5 — puissance de la box corrigée

### Radio
- Le calcul de la puissance d'émission adaptée utilise la puissance réelle
  de la LiXee-Box, +10 dBm, au lieu des +13 dBm supposés. Le ZLinky passe à
  0 dBm dès que la box est reçue à −80 dBm ou plus fort, et reste à
  +10 dBm en dessous de −90 dBm.
- Comme en 1.1.4, la puissance est calculée à l'appairage : un ZLinky déjà
  appairé garde sa valeur jusqu'à son prochain appairage.

## 1.1.4 — puissance d'émission adaptée au lien

### Radio
- À l'appairage, le ZLinky mesure le signal de la PAIR_RESPONSE et en
  déduit sa puissance d'émission, entre 0 et +10 dBm, pour arriver à la
  box vers −90 dBm (~30 dB de marge). Un ZLinky proche de la box émet à
  0 dBm : environ −45 % de courant d'émission (10 mA au lieu de 18 mA
  pour la puce, datasheet). Un ZLinky éloigné reste à +10 dBm.
- La PAIR_REQUEST reste à +10 dBm ; la PAIR_CONFIRM, les données, les
  réponses POLL et l'OTA utilisent la puissance adaptée.
- Valeur mémorisée avec l'appairage. Un ZLinky appairé avant la 1.1.4
  reste à +10 dBm jusqu'à son prochain appairage.

## 1.1.3 — économie d'énergie de la radio

### Radio
- Radio en sommeil (~0,4 µA) au lieu de la veille STDBY_RC (700 µA)
  entre deux cycles de données, entre deux tentatives d'appairage et
  pendant la recharge avant chaque PAIR_CONFIRM. Le courant économisé
  revient à la recharge de la réserve.
- Réveil automatique à la commande SPI suivante ; chaque émission refait
  un reset et une configuration complète, comme avant.

### Lecture à la demande (POLL)
- Fenêtre d'écoute fermée dès qu'il est sûr qu'aucune requête n'arrive :
  ~152 ms en SF11, ~118 ms en SF10, au lieu de 300 ms. Elle reste ouverte
  300 ms dès qu'un en-tête LoRa est reçu. Suppose que la box répond au
  plus 20 ms après la trame montante (cf. `Doc/LORA_PROTOCOL.md` §8).

## 1.1.2 — économie d'énergie pendant l'appairage

Suite des essais de la 1.1.1 sur le triphasé : la recharge avant la
PAIR_CONFIRM fonctionne, mais certains appairages échouent encore sans que
la box reçoive rien.

### Appairage
- LED : un flash de 20 ms toutes les 500 ms entre deux tentatives (environ
  4 % du temps, contre 40 à 67 % avant), et LED éteinte de la PAIR_REQUEST
  à la fin de la dernière PAIR_CONFIRM.
- Traces SPI coupées pendant la PAIR_CONFIRM : leur affichage la retardait
  de plusieurs centaines de millisecondes.

### Diagnostic
- Pour chaque PAIR_CONFIRM : début et fin d'émission comptés depuis la
  réception de la PAIR_RESPONSE, instant prévu, et fin d'émission confirmée
  ou non par la radio.
- Délai de recharge avant la première PAIR_REQUEST affiché.

## 1.1.1 — appairage sur les compteurs à faible énergie

Corrige des appairages qui échouaient sur certains compteurs (constaté sur un
triphasé) : la box recevait une PAIR_CONFIRM tronquée, ou rien.

### Appairage
- La PAIR_CONFIRM n'est plus émise aussitôt après la PAIR_RESPONSE : le
  ZLinky attend 1,7 s que sa réserve d'énergie se recharge, CPU en veille.
- La PAIR_CONFIRM est émise une seconde fois, identique, à 3,0 s : elle
  rattrape une première émission coupée par une chute de tension.
- L'attente de la PAIR_RESPONSE est mesurée en millisecondes et couvre la
  durée complète de la réponse : une réponse détectée en fin de fenêtre
  n'est plus abandonnée en cours de réception.

### Diagnostic
- Console : raison exacte d'une PAIR_RESPONSE manquée (rien entendu,
  préambule ou en-tête vus puis coupure, erreur CRC), numéro et instant de
  chaque PAIR_CONFIRM.
- Console : cause du dernier redémarrage au boot (POWER-ON, BROWN-OUT,
  WATCHDOG...). BROWN-OUT signale une chute de tension.

## 1.1.0 — isolation de l'appairage

Corrige des émetteurs qui perdaient leur appairage quand un autre émetteur
était appairé, y compris sur une autre box.

### Appairage
- Une PAIR_RESPONSE n'est acceptée que si son `seq` est celui de la dernière
  PAIR_REQUEST de l'émetteur ; le seq de départ est tiré au hasard.
- PAIR_RESPONSE adressée (29 octets, MAC de l'émetteur en `[21..28]`) acceptée
  et vérifiée. Les formats 20 et 21 octets restent acceptés.
- Un appui long ne supprime plus l'appairage en cours : il est restauré si la
  fenêtre expire sans succès.
- Fenêtre d'appairage bornée : 60 s après un appui long, 5 min au premier
  démarrage. Avant, l'émetteur restait en appairage indéfiniment et finissait
  par prendre la réponse de la première box qui ouvrait le sien.

### Radio
- Une trame reçue plus longue que le buffer prévu n'est plus copiée. Une trame
  voisine (EXTENDED d'un autre émetteur, bloc OTA) arrivant dans la fenêtre de
  réception de 300 ms débordait le buffer de 12 octets et écrasait la pile.

## 1.0.0 — premier lot

Première version numérotée. État du firmware pour le premier lot de ZLinky LoRa.

### Radio
- LoRa 2,4 GHz, SF11 / BW 406 kHz / CR 4/5, +10 dBm, préambule de 8 symboles.
- Appairage par PAIR_REQUEST de 11 octets ; `device_id` et `model` se lisent
  ensuite par POLL (cluster Basic `0x0000`).
- Chiffrement AES-128-CTR et MIC de 2 octets sur toutes les trames appairées.
- Mise à jour du firmware par radio (OTA), avec acquittement par fenêtre de
  blocs pour tenir dans le budget d'alimentation TIC.

### Données
- ESSENTIAL à chaque cycle : puissances apparentes, STGE, ADPS, mode.
- EXTENDED : 12 sous-types, émis une fois chacun après un démarrage, puis
  selon une table pondérée (index d'énergie toutes les ~2 min).
- Remontée sur changement : tarif en cours, PEJP, PPOT, ADIR1/2/3 et DEMAIN
  passent en priorité au prochain EXTENDED.
- Mode Historique : PAPP, PMAX, PEJP, PPOT, ADIR1/2/3 transmis ; ADIR et PEJP
  repassent à 0 à la fin de l'alerte.
- Lecture à la demande de tous les attributs TIC (POLL).

### Identification
- Version du firmware lisible par POLL : `0xFF01` (uint16) et `0x4000` (chaîne).

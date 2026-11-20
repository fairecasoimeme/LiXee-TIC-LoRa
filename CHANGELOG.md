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

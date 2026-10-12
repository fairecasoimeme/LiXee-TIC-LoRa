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

## 1.0.0 — premier lot

Première version numérotée. État du firmware pour le premier lot de ZLinky LoRa.

### Radio
- LoRa 2,4 GHz, SF11 / BW 406 kHz / CR 4/5, +10 dBm, préambule de 8 symboles.
- Appairage par PAIR_REQUEST de 11 octets ; `device_id` et `model` se lisent
  ensuite par POLL (cluster Basic `0x0000`).
- Chiffrement AES-128-CTR et MIC de 2 octets sur toutes les trames appairées.
- Mise à jour du firmware par radio (OTA).

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

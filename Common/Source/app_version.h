/*****************************************************************************
 *
 * MODULE:             app_version
 *
 * COMPONENT:          app_version.h
 *
 * DESCRIPTION:        Version du firmware ZLinky LoRa — source unique.
 *
 *                     Numerotation MAJEUR.MINEUR.CORRECTIF :
 *                       MAJEUR    : rupture de compatibilite radio (un
 *                                   recepteur anterieur ne decode plus les
 *                                   trames, ou l'appairage change)
 *                       MINEUR    : ajout compatible (nouveau sous-type,
 *                                   nouvel attribut POLL, nouvelle cadence)
 *                       CORRECTIF : correction sans effet sur le protocole
 *
 *                     A NE PAS CONFONDRE avec LORA_TIC_PROTOCOL_VERSION
 *                     (quartet haut de l'octet 0 de chaque trame), qui
 *                     versionne le format radio et non le firmware.
 *
 *                     Pour publier une version : modifier les trois
 *                     constantes ci-dessous, completer CHANGELOG.md,
 *                     compiler, puis poser le tag git v<MAJEUR.MINEUR.CORRECTIF>.
 *
 ****************************************************************************/

#ifndef APP_VERSION_H_
#define APP_VERSION_H_

#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define APP_VERSION_MAJOR   1    /* 0..255 */
#define APP_VERSION_MINOR   0    /* 0..15  */
#define APP_VERSION_PATCH   0    /* 0..15  */

/* Version sur 16 bits : 0xMMmp (MM = majeur, m = mineur, p = correctif).
 * Lisible directement en hexadecimal : 1.2.3 -> 0x0123.
 * C'est la valeur renvoyee par POLL (cluster Basic, attribut 0xFF01) et celle
 * attendue dans le champ version d'OTA_START : les deux se comparent
 * numeriquement. */
#define APP_VERSION_U16     ((uint16)(((APP_VERSION_MAJOR) << 8) | \
                                      ((APP_VERSION_MINOR) << 4) | \
                                       (APP_VERSION_PATCH)))

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Chaine de version complete, terminee par un nul :
 *        "<MAJEUR>.<MINEUR>.<CORRECTIF>+<commit git>[-dirty]", par exemple
 *        "1.0.0+5f4b612". Le suffixe "-dirty" signale un build fait avec des
 *        modifications non commitees ; "nogit" remplace le commit si git
 *        n'etait pas disponible a la compilation.
 */
PUBLIC const char *APP_pcVersionString(void);

#endif /* APP_VERSION_H_ */

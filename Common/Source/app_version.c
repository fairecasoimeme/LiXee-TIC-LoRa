/*****************************************************************************
 *
 * MODULE:             app_version
 *
 * COMPONENT:          app_version.c
 *
 * DESCRIPTION:        Chaine de version du firmware. Ce fichier est recompile
 *                     a chaque build (cf. Makefile) pour que le commit git
 *                     embarque soit toujours celui des sources compilees.
 *
 ****************************************************************************/

#include <jendefs.h>
#include "app_version.h"

/* Commit git, injecte par le Makefile (-DAPP_BUILD_GIT=...) */
#ifndef APP_BUILD_GIT
#define APP_BUILD_GIT "nogit"
#endif

#define APP_VERSION_STR_(x)  #x
#define APP_VERSION_STR(x)   APP_VERSION_STR_(x)

PRIVATE const char szVersion[] =
    APP_VERSION_STR(APP_VERSION_MAJOR) "."
    APP_VERSION_STR(APP_VERSION_MINOR) "."
    APP_VERSION_STR(APP_VERSION_PATCH) "+" APP_BUILD_GIT;

PUBLIC const char *APP_pcVersionString(void)
{
    return szVersion;
}

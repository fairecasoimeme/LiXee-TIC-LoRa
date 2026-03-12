/*****************************************************************************
 *
 * MODULE:              JN-AN-1243
 *
 * COMPONENT:          app_zcl_task.c
 *
 * DESCRIPTION:       ZCL minimal pour LoRa-only (sBaseDevice + timer TIC)
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppApi.h>
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "dbg.h"
#include "zps_gen.h"
#include "PDM_IDs.h"
#include "ZTimer.h"
#include "zcl.h"
#include "app.h"
#include "zcl_options.h"
#include "app_zcl_task.h"
#include "app_router_node.h"
#include "app_common.h"
#include "app_main.h"

#include "base_device.h"
#include "App_Linky.h"
#include "app_events.h"
#include <string.h>

extern PUBLIC tsLinkyParams sLinkyParams;

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_ZCL
#define TRACE_ZCL   TRUE
#else
#define TRACE_ZCL   FALSE
#endif

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void APP_ZCL_cbGeneralCallback(tsZCL_CallBackEvent *psEvent);
PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
tsZHA_BaseDevice sBaseDevice;

/****************************************************************************
 *
 * NAME: APP_ZCL_vInitialise
 *
 * DESCRIPTION:
 * Init ZCL minimal — enregistre sBaseDevice pour que App_Linky.c puisse
 * y ecrire les donnees TIC.
 *
 ****************************************************************************/
PUBLIC void APP_ZCL_vInitialise(void)
{
    teZCL_Status eZCL_Status;

    /* Initialise ZCL */
    eZCL_Status = eZCL_Initialise(&APP_ZCL_cbGeneralCallback, apduZCL);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        DBG_vPrintf(TRACE_ZCL, "\r\nErr: eZCL_Initialise:%d", eZCL_Status);
    }

    /* Enregistrer le endpoint pour initialiser sBaseDevice */
    eZCL_Status = eZHA_RegisterBaseDeviceEndPoint(ZLINKY_APPLICATION_ENDPOINT,
                                                   &APP_ZCL_cbEndpointCallback,
                                                   &sBaseDevice);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        DBG_vPrintf(TRACE_ZCL, "Error: eZHA_RegisterBaseDeviceEndPoint: %02x\r\n", eZCL_Status);
    }

    APP_vZCL_DeviceSpecific_Init();

    DBG_vPrintf(TRACE_ZCL, "APP_ZCL_vInitialise: minimal init OK\r\n");
}

/****************************************************************************
 *
 * NAME: APP_cbTimerLinky
 *
 * DESCRIPTION:
 * Timer callback TIC — poste APP_E_EVENT_PERIODIC_REPORT
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerLinky(void *pvParam)
{
    APP_tsEvent sAppEvent;

    sAppEvent.eType = APP_E_EVENT_PERIODIC_REPORT;
    if(ZQ_bQueueSend(&APP_msgAppEvents, &sAppEvent) == FALSE)
    {
        DBG_vPrintf(TRACE_ZCL, "\r\nEVENT PERIODIC SEND ERROR");
    }
}

/****************************************************************************
 *
 * NAME: APP_vZCL_DeviceSpecific_Init
 *
 * DESCRIPTION:
 * Initialise manufacturer strings dans sBaseDevice
 *
 ****************************************************************************/
PUBLIC void APP_vZCL_DeviceSpecific_Init(void)
{
    /* Mode LoRa-only : toujours les strings LiXee */
    memcpy(sBaseDevice.sBasicServerCluster.au8ManufacturerName, "LiXee", 5);
    sBaseDevice.sBasicServerCluster.sManufacturerName.u8Length = 5;
    sBaseDevice.sBasicServerCluster.sManufacturerName.pu8Data = sBaseDevice.sBasicServerCluster.au8ManufacturerName;

    memcpy(sBaseDevice.sBasicServerCluster.au8ModelIdentifier, "ZLinky_TIC", 10);
    sBaseDevice.sBasicServerCluster.sModelIdentifier.u8Length = 10;
    sBaseDevice.sBasicServerCluster.sModelIdentifier.pu8Data = sBaseDevice.sBasicServerCluster.au8ModelIdentifier;

    memcpy(sBaseDevice.sBasicServerCluster.au8DateCode, "20250923", CLD_BAS_DATE_SIZE);
    memcpy(sBaseDevice.sBasicServerCluster.au8SWBuildID, "4000-0017", CLD_BAS_SW_BUILD_SIZE);
    memcpy(sBaseDevice.sBasicServerCluster.au8ProductURL, "LiXee.fr", CLD_BAS_URL_SIZE);
    memcpy(sBaseDevice.sBasicServerCluster.au8ProductCode, "0017", CLD_BAS_PCODE_SIZE);
    memcpy(sBaseDevice.sLinkyServerCluster.au8LinkyOptarif, "BASE", 4);

    sBaseDevice.sBasicServerCluster.u8ApplicationVersion = 0x10;
    sBaseDevice.sSimpleMeteringServerCluster.eMeteringDeviceType = E_CLD_SM_MDT_ELECTRIC;
}

/****************************************************************************/
/***        Stub Callbacks (references par le SDK ZCL)                    ***/
/****************************************************************************/

PRIVATE void APP_ZCL_cbGeneralCallback(tsZCL_CallBackEvent *psEvent)
{
    (void)psEvent;
}

PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent)
{
    (void)psEvent;
}

#ifdef CLD_GREENPOWER
/****************************************************************************
 *
 * NAME: APP_cbTimerGPZclTick
 *
 * DESCRIPTION:
 * Stub — callback GreenPower timer (reference par app_main.c)
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerGPZclTick(void *pvParam)
{
    (void)pvParam;
}
#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

/*****************************************************************************
 *
 * MODULE:             JN-AN-1243
 *
 * COMPONENT:          app_router_node.c
 *
 * DESCRIPTION:        LoRa-only application (Zigbee runtime removed)
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "pdum_apl.h"
#include "pdum_nwk.h"
#include "pdum_gen.h"

#ifdef APP_LOW_POWER_API
#include "PWR_interface.h"
#else
#include "pwrm.h"
#endif

#include "PDM.h"
#include "zps_gen.h"
#include "zps_apl.h"
#include "zps_apl_af.h"
#include "zcl_options.h"
#include "app_common.h"
#include "app_main.h"
#include "app.h"
#include "ZTimer.h"
#include "app_events.h"
#include "app_zcl_task.h"
#include "app_router_node.h"
#include "PDM_IDs.h"
#include "zcl.h"

#if (JENNIC_CHIP_FAMILY == JN518x)
#include "fsl_reset.h"
#endif

#include "MicroSpecific.h"
#include "app_blink_led.h"
#include "App_Linky.h"
#include "app_lora_tic.h"
#include "app_lora_pair.h"
#include "app_buttons.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_APP
    #define TRACE_APP   TRUE
#else
    #define TRACE_APP   FALSE
#endif

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
PUBLIC tsDeviceDesc sDeviceDesc;
extern PUBLIC tsLinkyParams sLinkyParams;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_vInitialiseRouter
 *
 * DESCRIPTION:
 * Initialise application (LoRa-only mode: ZCL minimal + TIC Linky)
 *
 ****************************************************************************/
PUBLIC void APP_vInitialiseRouter(void)
{
    /* Stay awake */
#ifdef APP_LOW_POWER_API
    PWR_DisallowDeviceToSleep();
#else
    PWRM_eStartActivity();
#endif

    /* Initialise ZBPro stack — OBLIGATOIRE (crash sans) */
    ZPS_eAplAfInit();

    /* Initialise ZCL minimal (sBaseDevice pour App_Linky.c) */
    APP_ZCL_vInitialise();

    /* Charger les parametres Linky depuis PDM */
    LoadLinkyParams();

    /* Init manufacturer strings dans sBaseDevice */
    APP_vZCL_DeviceSpecific_Init();

    /* Init button (GINT interrupt on PIO0 + debounce timer) */
    APP_bButtonInitialise();

    /* Init pairing module (load PDM, determine state) */
    APP_vLoraPairInit();

    /* Demarrer TIC + LoRa TX seulement si appaire */
    if (APP_bLoraPairIsPaired())
    {
        /* LED blink 3s par defaut (App_Linky ajustera: 1s erreur, 3s OK) */
        vStartBlinkTimer(3000);
        vAPP_LinkySensorSample();
        APP_vLoRaTicStart();
    }
    else
    {
        DBG_vPrintf(TRACE_APP, "APP: Not paired — TIC + LoRa TX disabled, pairing in progress...\r\n");
    }

    DBG_vPrintf(TRACE_APP, "APP_vInitialiseRouter: LoRa-only init complete\r\n");
}

/****************************************************************************
 *
 * NAME: APP_taskRouter
 *
 * DESCRIPTION:
 * Main application task — handles events from queue
 *
 ****************************************************************************/
PUBLIC void APP_taskRouter(void)
{
    APP_tsEvent sAppEvent;
    sAppEvent.eType = APP_E_EVENT_NONE;

    if (ZQ_bQueueReceive(&APP_msgAppEvents, &sAppEvent) == TRUE)
    {
        switch (sAppEvent.eType)
        {
            case APP_E_EVENT_PERIODIC_REPORT:
                if (APP_bLoraPairIsPaired())
                    vAPP_LinkySensorSample();
                break;

            case APP_E_EVENT_LORA_TX:
                if (APP_bLoraPairIsPaired())
                    APP_vLoRaTicSendPacket();
                break;

            case APP_E_EVENT_BUTTON_DOWN:
                APP_vLoraPairOnButton(APP_E_EVENT_BUTTON_DOWN);
                break;

            case APP_E_EVENT_LEAVE_AND_RESET:
                APP_vLoraPairOnButton(APP_E_EVENT_LEAVE_AND_RESET);
                break;

            default:
                break;
        }
    }
}

/****************************************************************************
 *
 * NAME: APP_vBdbCallback
 *
 * DESCRIPTION:
 * Stub — peut etre reference par le SDK BDB
 *
 ****************************************************************************/
PUBLIC void APP_vBdbCallback(BDB_tsBdbEvent *psBdbEvent)
{
    (void)psBdbEvent;
}

/****************************************************************************
 *
 * NAME: APP_vFactoryResetRecords
 *
 * DESCRIPTION:
 * Stub — peut etre reference par le SDK
 *
 ****************************************************************************/
PUBLIC void APP_vFactoryResetRecords(void)
{
}

/****************************************************************************
 *
 * NAME: APP_vOobcSetRunning
 *
 * DESCRIPTION:
 * Stub — peut etre reference par le SDK
 *
 ****************************************************************************/
PUBLIC void APP_vOobcSetRunning(void)
{
}

/****************************************************************************
 *
 * NAME: app_u8GetDeviceEndpoint
 *
 * DESCRIPTION:
 * Return the application endpoint
 *
 ****************************************************************************/
PUBLIC uint8 app_u8GetDeviceEndpoint(void)
{
    return ZLINKY_APPLICATION_ENDPOINT;
}

#ifdef CLD_OTA
PUBLIC teNodeState eGetNodeState(void)
{
    return sDeviceDesc.eNodeState;
}

PUBLIC tsOTA_PersistedData sGetOTACallBackPersistdata(void)
{
    return sBaseDevice.sCLD_OTA_CustomDataStruct.sOTACallBackMessage.sPersistedData;
}
#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

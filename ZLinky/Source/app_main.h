/*****************************************************************************
 *
 * MODULE:             JN-AN-1243
 *
 * COMPONENT:          app_main.h
 *
 ***************************************************************************/

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "ZQueue.h"
#include "tsv_pub.h"
#include "bdb_api.h"

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void APP_vInitResources(void);
PUBLIC void APP_vMainLoop(void);
PUBLIC void APP_vSetUpHardware(void);
PUBLIC void APP_vRadioTempUpdate(bool_t bLoadCalibration);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/* Timers LoRa-only */
extern PUBLIC uint8 u8TimerLinky;
extern PUBLIC uint8 u8TimerBlink;
extern PUBLIC uint8 u8TimerLedON;
extern PUBLIC uint8 u8TimerLoRa;
extern PUBLIC uint8 u8TimerAwake;
extern PUBLIC uint8 u8TimerLoraPair;
extern PUBLIC uint8 u8TimerButtonScan;  /* requis par app_buttons.c */
extern PUBLIC uint8 u8TimerPowerOn;     /* requis par app_power_on_counter.c */

/* Queues Zigbee (creation preservee pour stabilite SDK) */
extern PUBLIC tszQueue zps_msgMlmeDcfmInd;
extern PUBLIC tszQueue zps_msgMcpsDcfmInd;
extern PUBLIC tszQueue zps_msgMcpsDcfm;
extern PUBLIC tszQueue zps_TimeEvents;

extern PUBLIC tszQueue APP_msgBdbEvents;
extern PUBLIC tszQueue APP_msgAppEvents;

/* Variables Linky */
extern PUBLIC uint8 u8ModeLinky;
extern PUBLIC uint8 u8OldStatusLinky;

#ifdef CLD_GREENPOWER
extern PUBLIC uint8 u8GPTimerTick;
extern PUBLIC tszQueue APP_msgGPZCLTimerEvents;
extern uint8 au8GPZCLEvent[];
extern uint8 u8GPZCLTimerEvent;
#endif

#endif /* APP_MAIN_H */

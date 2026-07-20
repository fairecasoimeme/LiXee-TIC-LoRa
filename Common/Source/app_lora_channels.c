/*****************************************************************************
 *
 * MODULE:             app_lora_channels
 *
 * COMPONENT:          app_lora_channels.c
 *
 * DESCRIPTION:        LoRa channel selection helpers for SX1280 2.4 GHz.
 *
 ****************************************************************************/

#include <jendefs.h>
#include "dbg.h"
#include "app_lora_channels.h"
#include "sx1280.h"

#define TRACE_LORA_CHAN  TRUE

/* Channel frequency table.
 * 8 channels evenly distributed in the 2.4 GHz ISM band (2400-2483.5 MHz).
 * Picked to avoid WiFi channels 1 (2412), 6 (2437), 11 (2462) when possible
 * and leave room for Zigbee.
 *
 * Channel 3 = PAIR_CHANNEL = 2440 MHz (center, accessible default).
 */
PRIVATE const uint32_t LORA_CHANNEL_FREQS[LORA_NUM_CHANNELS] = {
    2410000000UL,   /* ch 0 - bottom band, far from WiFi 6/11 */
    2420000000UL,   /* ch 1 */
    2430000000UL,   /* ch 2 */
    2440000000UL,   /* ch 3 - PAIR_CHANNEL (center band) */
    2450000000UL,   /* ch 4 */
    2460000000UL,   /* ch 5 */
    2470000000UL,   /* ch 6 */
    2480000000UL,   /* ch 7 - top band, far from WiFi 1/6 */
};

PUBLIC bool_t APP_bLoraChannelIsValid(uint8_t u8Channel)
{
    return (u8Channel < LORA_NUM_CHANNELS);
}

PUBLIC uint32_t APP_u32LoraChannelGetFreq(uint8_t u8Channel)
{
    if (!APP_bLoraChannelIsValid(u8Channel))
    {
        DBG_vPrintf(TRACE_LORA_CHAN,
            "LORA_CHAN: invalid channel %d, using PAIR_CHANNEL %d\r\n",
            u8Channel, LORA_PAIR_CHANNEL);
        u8Channel = LORA_PAIR_CHANNEL;
    }
    return LORA_CHANNEL_FREQS[u8Channel];
}

PUBLIC void APP_vLoraChannelSet(uint8_t u8Channel)
{
    uint32_t u32Freq = APP_u32LoraChannelGetFreq(u8Channel);
    DBG_vPrintf(TRACE_LORA_CHAN,
        "LORA_CHAN: switching to ch %d = %lu Hz\r\n",
        u8Channel, u32Freq);
    SX1280_SetRfFrequency(u32Freq);
}

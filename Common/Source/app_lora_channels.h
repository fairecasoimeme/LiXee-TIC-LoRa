/*****************************************************************************
 *
 * MODULE:             app_lora_channels
 *
 * COMPONENT:          app_lora_channels.h
 *
 * DESCRIPTION:        LoRa channel plan for 2.4 GHz ISM band (Europe ETSI).
 *                     Defines 8 candidate channels spread across 2400-2480 MHz,
 *                     designed to avoid common WiFi channels (1, 6, 11) and
 *                     leave room for Zigbee.
 *
 *                     The receiver picks an OP_CHANNEL based on CCA (Clear
 *                     Channel Assessment) and tells the emitter via
 *                     PAIR_RESPONSE byte[19].
 *
 *                     The PAIR_CHANNEL is fixed and known by both sides; it's
 *                     used only during the pairing handshake.
 *
 ****************************************************************************/

#ifndef APP_LORA_CHANNELS_H_
#define APP_LORA_CHANNELS_H_

#include <jendefs.h>

/* Number of supported channels */
#define LORA_NUM_CHANNELS  8

/* Channel index used for pairing handshake (well-known fixed value) */
#define LORA_PAIR_CHANNEL  3   /* 2440 MHz - centre band, between WiFi 6 & 11 */

/* Channel frequency table (Hz)
 * Each channel is ~10 MHz apart, well within 2.4 GHz ISM band (2400-2483.5 MHz).
 * Designed to avoid WiFi channels 1/6/11 centers and Zigbee 25/26.
 */
PUBLIC uint32_t APP_u32LoraChannelGetFreq(uint8_t u8Channel);

/**
 * @brief Set the SX1280 RF frequency to the specified channel.
 *        Call after SetStandby_RC and before SetTx/SetRx.
 * @param u8Channel  Channel index 0..LORA_NUM_CHANNELS-1
 *                   Falls back to PAIR_CHANNEL if out of range.
 */
PUBLIC void APP_vLoraChannelSet(uint8_t u8Channel);

/**
 * @brief Validate that a channel index is in range
 */
PUBLIC bool_t APP_bLoraChannelIsValid(uint8_t u8Channel);

#endif /* APP_LORA_CHANNELS_H_ */

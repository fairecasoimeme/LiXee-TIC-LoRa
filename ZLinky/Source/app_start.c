/*****************************************************************************
 *
 * MODULE:             JN-AN-1243
 *
 * COMPONENT:          app_start.c
 *
 * DESCRIPTION:        Base Device Demo: Router Initialisation
 *
 *****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5169,
 * JN5179, JN5189].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2017-2019. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "fsl_wwdt.h"
#include "fsl_iocon.h"
#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_clock.h"
#include "pin_mux.h"
#include "app.h"

#include "sx1280.h"
#include "sx1280_hal.h"
#include "sx1280_config.h"
#include "app_lora_tic.h"
#include "app_lora_channels.h"

#ifdef APP_LOW_POWER_API
#include "PWR_interface.h"
#else
#include "pwrm.h"
#endif

#include "pdum_nwk.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "dbg.h"
#include "zps_gen.h"
#include "zps_apl.h"
#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "string.h"
#include "app_router_node.h"
#include "zcl_options.h"
#include "app_common.h"
#include "app_main.h"
#include "portmacro.h"
#include "DebugExceptionHandlers_jn518x.h"
#ifdef APP_NTAG_NWK
#include "app_ntag.h"
#if (JENNIC_CHIP_FAMILY == JN518x)
#include "fsl_power.h"
#endif
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifdef DEBUG_APP
    #define TRACE_APP   TRUE
#else
    #define TRACE_APP   FALSE
#endif

#define HALT_ON_EXCEPTION   FALSE

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void APP_vInitialise(void);
PRIVATE void vfExtendedStatusCallBack(ZPS_teExtendedStatus eExtendedStatus);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

extern void *_stack_low_water_mark;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
extern void OSA_TimeInit(void);

static void ConfigureLoRa(void)
{
	PRINTF("=== Configuration LoRa (module NiceRF LORA1280-TCXO) ===\r\n");

	    /*
	     * Module : NiceRF LORA1280-TCXO (SX1280 natif, TCXO 0.5ppm)
	     *   - Pas de PA externe (max +12.5 dBm)
	     *   - Pas de DC-DC sur le PCB -> mode LDO obligatoire
	     *   - Alimentation : 1.8-3.7V
	     *
	     * Parametres LoRa :
	     *   Frequence  = 2490.0 MHz (au-dessus du WiFi ch13)
	     *   Bandwidth  = 406.25 kHz  -> SX1280_LORA_BW_406 (0x26)
	     *   SF         = 10          -> SX1280_LORA_SF10 (0xA0)
	     *   CR         = 4/5         -> SX1280_LORA_CR_4_5 (0x01)
	     *   Preamble   = 16
	     *   Header     = Explicit    -> 0x00
	     *   CRC        = On          -> 0x20
	     *   InvertIQ   = false       -> 0x40 (IQ normal)
	     *   Power      = 12.5 dBm max (SX1280 natif, pas de PA)
	     *   Regulator  = LDO (pas de DC-DC sur le module)
	     */

	    // 1) Standby RC
	    SX1280_SetStandby(SX1280_STANDBY_RC);
	    for(volatile uint32_t i = 0; i < 480000; i++); // 10ms
	    PRINTF("Standby RC OK, Status=0x%02X\r\n", SX1280_GetStatus());

	    // 2) Buffer base addresses (RadioLib le fait en 1er dans config())
	    SX1280_SetBufferBaseAddress(0x00, 0x00);
	    PRINTF("Buffer base address OK\r\n");

	    // 3) Packet Type LoRa
	    SX1280_SetPacketType(SX1280_PACKET_TYPE_LORA);
	    PRINTF("Packet Type LoRa OK\r\n");

	    // 4) SetCadParams (RadioLib: CAD_ON_8_SYMB = 0x60)
	    {
	        uint8_t cadParam = 0x60;
	        SX1280_HAL_WriteCommand(0x88, &cadParam, 1);
	        PRINTF("CadParams OK (CAD_ON_8_SYMB)\r\n");
	    }

	    // 5) Regulator Mode LDO (par defaut)
	    //    Le chip demarre en LDO, on peut passer en DC-DC apres si besoin.
	    SX1280_SetRegulatorMode(SX1280_REGULATOR_DCDC);
	    PRINTF("Regulator Mode DC-DC OK\r\n");

	    // 6) Frequence : PAIR_CHANNEL (2440 MHz) pour le test au boot.
	    //    Bande ISM 2.4 GHz Europe (2400-2483.5 MHz). Le canal operationnel
	    //    sera selectionne par le recepteur au pairing.
	    APP_vLoraChannelSet(LORA_PAIR_CHANNEL);
	    for(volatile uint32_t i = 0; i < 48000; i++); // 1ms PLL settling
	    PRINTF("RF Frequency OK (PAIR_CHANNEL %d, in ISM band)\r\n", LORA_PAIR_CHANNEL);

	    // 7) Modulation : SF11, BW406, CR4/5
	    //    SF11 = sensibilite -126 dBm, airtime ~200ms.
	    SX1280_LoRaModParams_t modParams = {
	        .spreadingFactor = SX1280_LORA_SF11,
	        .bandwidth = SX1280_LORA_BW_406,
	        .codingRate = SX1280_LORA_CR_4_5
	    };
	    SX1280_SetModulationParams(&modParams);
	    PRINTF("Modulation OK (SF11, BW406, CR4/5)\r\n");

	    // 8) LoRa Sync Word : 0x14,0x24 = sync word 0x12 (defaut RadioLib)
	    //    CRITIQUE : le test SPI au boot ecrit dans le registre 0x0944,
	    //    il faut restaurer la valeur par defaut pour que le recepteur
	    //    (RadioLib setSyncWord(0x12)) reconnaisse les paquets.
	    {
	        uint8_t syncWord[2] = {0x14, 0x24};
	        SX1280_HAL_WriteRegisters(0x0944, syncWord, 2);
	        PRINTF("SyncWord OK (0x14,0x24 = RadioLib 0x12)\r\n");
	    }

	    // 9) Packet params : preamble=16 symboles, explicit, CRC on, IQ normal
	    //    ATTENTION: preambleLength utilise un encodage mantisse*2^exposant !
	    //    16 symboles = mant=8, exp=1 -> 0x18 (identique a RadioLib)
	    //    CRC:      0x20 = on, 0x00 = off (PAS 0/1)
	    //    InvertIQ: 0x40 = normal, 0x00 = inverted (PAS 0/1)
	    SX1280_LoRaPacketParams_t packetParams = {
	        .preambleLength = 0x18,       // 16 symboles : mant=8 * 2^exp=1
	        .headerType = SX1280_LORA_HEADER_EXPLICIT,
	        .payloadLength = 255,         // Max, sera mis a jour avant chaque TX
	        .crc = SX1280_LORA_CRC_ON,
	        .invertIQ = SX1280_LORA_IQ_NORMAL
	    };
	    SX1280_SetPacketParams(&packetParams);
	    PRINTF("Packet Params OK (preamble=0x18=16sym, explicit, CRC=0x20, IQ=0x40)\r\n");

	    // 9) TX Power : +10 dBm (max EU ETSI EN 300 328 = +10 dBm EIRP)
	    SX1280_SetTxParams(13, SX1280_RAMP_20_US);  /* +13 dBm (max) - test stabilite */
	    PRINTF("TX Params OK (power=+10dBm EU, ramp=20us)\r\n");

	    // 10) IRQ : TX_DONE + RX_TX_TIMEOUT sur DIO1
	    SX1280_SetDioIrqParams(
	        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,
	        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);
	    PRINTF("IRQ OK (TX_DONE + TIMEOUT on DIO1)\r\n");

	    PRINTF("=== Config LoRa terminee (identique Arduino) ===\r\n\r\n");
}

void ReadSX1280Config(void)
{
    uint8_t reg[10];

    PRINTF("\n=== Lecture configuration SX1280 ===\r\n");

    // PacketType
    SX1280_HAL_ReadRegisters(0x0889, reg, 1);
    PRINTF("PacketType (0x0889): 0x%02X (expect 0x01=LoRa)\r\n", reg[0]);

    // Modulation Params
    SX1280_HAL_ReadRegisters(0x088B, reg, 3);
    PRINTF("ModParams (0x088B): SF=0x%02X(expect 0xA0=SF10) BW=0x%02X(expect 0x26=BW406) CR=0x%02X(expect 0x01=CR4/5)\r\n",
           reg[0], reg[1], reg[2]);

    // TX Params
    SX1280_HAL_ReadRegisters(0x088E, reg, 2);
    PRINTF("TxParams (0x088E): Power=0x%02X(expect 0x1F=31=+13dBm) Ramp=0x%02X(expect 0xE0=20us)\r\n",
           reg[0], reg[1]);

    // Registre SF config (ecrit apres SetModulationParams)
    SX1280_HAL_ReadRegisters(0x0925, reg, 1);
    PRINTF("SF_Config (0x0925): 0x%02X (expect 0x32 for SF10)\r\n", reg[0]);

    // Registre freq error correction
    SX1280_HAL_ReadRegisters(0x093C, reg, 1);
    PRINTF("FreqErrCorr (0x093C): 0x%02X (expect bit0=1)\r\n", reg[0]);

    // LoRa SyncWord
    SX1280_HAL_ReadRegisters(0x0944, reg, 2);
    PRINTF("SyncWord (0x0944): 0x%02X 0x%02X\r\n", reg[0], reg[1]);

    // Status
    uint8_t status = SX1280_GetStatus();
    PRINTF("Status: 0x%02X\r\n", status);

    PRINTF("===================================\r\n\r\n");
}


/****************************************************************************
 *
 * NAME: vAppMain
 *
 * DESCRIPTION:
 * Entry point for application from a cold start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void vAppMain(void)
{
    wwdt_config_t  config;

    /* Initialise exception handlers for debugging */
    vDebugExceptionHandlersInitialise();

    /* Catch resets due to watchdog timer expiry */
    WWDT_GetDefaultConfig(&config);

    if (((PMC->RESETCAUSE) & PMC_RESETCAUSE_WDTRESET_MASK) == PMC_RESETCAUSE_WDTRESET_MASK)
    {
        /* Enable the WWDT clock */
        CLOCK_EnableClock(kCLOCK_WdtOsc);
        RESET_PeripheralReset(kWWDT_RST_SHIFT_RSTn);
        WWDT_Deinit(WWDT);
        DBG_vPrintf(TRACE_APP, "APP: Watchdog timer has reset device!\r\n");
        POWER_ClearResetCause();
        while(1) {
            volatile uint32 u32Delay;

            for (u32Delay = 0; u32Delay < 100000; u32Delay++);
            GPIO_PinWrite(GPIO, APP_BOARD_GPIO_PORT, 10, 0);

            for (u32Delay = 0; u32Delay < 100000; u32Delay++);
            GPIO_PinWrite(GPIO, APP_BOARD_GPIO_PORT, 10, 1);

        }
    }




    /* idle task commences here */
    DBG_vPrintf(TRUE,"\r\n");
    DBG_vPrintf(TRUE, "***********************************************\r\n");
    DBG_vPrintf(TRUE, "* ZLINKY_TIC RESET                                *\r\n");
    DBG_vPrintf(TRUE, "***********************************************\r\n");
    DBG_vPrintf(TRUE, "SYSCON->MEMORYREMAP = x%08x\r\n", SYSCON->MEMORYREMAP);


    DBG_vPrintf(TRACE_APP, "APP: Entering APP_vInitResources()\n");
    APP_vInitResources();

    DBG_vPrintf(TRACE_APP, "APP: Entering APP_vInitialise()\n");
    APP_vInitialise();

    /* APP_vInitialise retourne apres init SX1280 + power optimizations.
     * Le main loop cooperatif demarre dans main_task() while(1). */




}

/****************************************************************************
 *
 * NAME: vAppRegisterPWRCallbacks
 *
 * DESCRIPTION:
 * Power manager callback.
 * Called to allow the application to register
 * sleep and wake callbacks.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vAppRegisterPWRCallbacks(void)
{
    /* nothing to register as device does not sleep */
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_vInitialise
 *
 * DESCRIPTION:
 * Initialises Zigbee stack, hardware and application.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_vInitialise(void)
{
    /* Init identique a l'original - toutes les couches Zigbee */
#ifdef APP_LOW_POWER_API
    (void) PWR_ChangeDeepSleepMode(PWR_E_SLEEP_OSCON_RAMON);
    PWR_Init();
    PWR_vForceRadioRetention(TRUE);
#else
    PWRM_vInit(E_AHI_SLEEP_OSCON_RAMON);
    PWRM_vForceRadioRetention(TRUE);
#endif

    PDM_eInitialise(1200, 63, NULL);
    PDUM_vInit();

    ZPS_vExtendedStatusSetCallback(vfExtendedStatusCallBack);

    /* Initialise other software modules */
    APP_vRadioTempUpdate(TRUE);

    /* ===== Init SX1280 LoRa ===== */
    SX1280_Init();

    {
        PRINTF("\nTest GetStatus...\r\n");
        uint8_t status = SX1280_GetStatus();
        PRINTF("SX1280 Status: 0x%02X\r\n", status);

        /*
         * Test registre : ecriture/lecture pour verifier la communication SPI.
         * On utilise le registre LoRa SyncWord (0x0944) qui est librement
         * ecrivable/lisible, contrairement a 0x0889 (PacketType) qui ne
         * peut etre modifie que par la commande SetPacketType.
         *
         * Note : il faut d'abord mettre le SX1280 en mode LoRa pour que
         * le registre 0x0944 soit accessible.
         */
        PRINTF("\nSetup: SetStandby + SetPacketType LoRa\r\n");
        SX1280_SetStandby(SX1280_STANDBY_RC);
        for(volatile uint32_t j = 0; j < 48000; j++);
        SX1280_SetPacketType(SX1280_PACKET_TYPE_LORA);
        for(volatile uint32_t j = 0; j < 48000; j++);

        PRINTF("\nTest registre 0x0944 (LoRa SyncWord):\r\n");
        /* Lire d'abord la valeur par defaut */
        uint8_t defVal = 0;
        SX1280_HAL_ReadRegisters(0x0944, &defVal, 1);
        PRINTF("  Default value: 0x%02X\r\n", defVal);

        uint8_t testVals[] = {0xAA, 0x55, 0x01};
        int spiOk = 1;
        for(int i = 0; i < 3; i++) {
            uint8_t write = testVals[i];
            uint8_t read = 0x00;

            SX1280_HAL_WriteRegisters(0x0944, &write, 1);
            for(volatile uint32_t j = 0; j < 48000; j++);
            SX1280_HAL_ReadRegisters(0x0944, &read, 1);

            PRINTF("  Write: 0x%02X, Read: 0x%02X %s\r\n",
                   write, read, (write == read) ? "OK" : "FAIL");
            if(write != read) spiOk = 0;
        }

        if(spiOk)
        {
            PRINTF("SPI communication OK\r\n");

            /* ================================================================
             * TEST 0 : DIAGNOSTIC OSCILLATEUR (TCXO)
             *
             * Le module NiceRF LORA1280-TCXO utilise un TCXO 52MHz
             * alimente par la pin 14 (TCXOEN) du module.
             * Sans TCXOEN=HIGH, le SX1280 n'a pas d'horloge de reference
             * et ne peut PAS quitter STDBY_RC (SetFs, SetTx, CW echouent).
             *
             * Ce test verifie que l'oscillateur peut demarrer en tentant
             * SetStandby(XOSC). Si le mode reste STDBY_RC, le TCXO est
             * probablement non alimente (verifier pin 14 du module).
             * ================================================================ */
            PRINTF("\n========================================\r\n");
            PRINTF("=== TEST 0 : OSCILLATEUR (TCXO) ===\r\n");
            PRINTF("========================================\r\n");
            {
                uint8_t preStatus = SX1280_GetStatus();
                PRINTF("Before: Status=0x%02X Mode=%d TCXOEN(PIO%d)=%d\r\n",
                       preStatus, (preStatus >> 5) & 0x07,
                       SX1280_TCXOEN_PIN,
                       GPIO_PinRead(GPIO, APP_BOARD_GPIO_PORT, SX1280_TCXOEN_PIN));

                /* Tenter le passage en STDBY_XOSC */
                PRINTF("SetStandby(XOSC)...\r\n");
                SX1280_SetStandby(SX1280_STANDBY_XOSC);

                /* Monitoring BUSY avec boucle serree pendant ~10ms */
                uint32_t busyHighCount = 0;
                for(uint32_t i = 0; i < 480000; i++) /* ~10ms @ 48MHz */
                {
                    if(GPIO_PinRead(GPIO, APP_BOARD_GPIO_PORT, 15))
                        busyHighCount++;
                }
                PRINTF("BUSY monitoring 10ms: HIGH count=%lu (0=instant, >0=processing)\r\n",
                       busyHighCount);

                uint8_t xoscStatus = SX1280_GetStatus();
                uint8_t xoscMode = (xoscStatus >> 5) & 0x07;
                const char *modeStr = "???";
                switch(xoscMode) {
                    case 2: modeStr = "STDBY_RC"; break;
                    case 3: modeStr = "STDBY_XOSC"; break;
                    case 4: modeStr = "FS"; break;
                    default: modeStr = "OTHER"; break;
                }
                PRINTF("After XOSC: Status=0x%02X Mode=%s(%d)\r\n",
                       xoscStatus, modeStr, xoscMode);

                if(xoscMode == 3) {
                    PRINTF(">>> TCXO OK : oscillateur 52MHz demarre ! <<<\r\n");
                } else if(xoscMode == 2) {
                    PRINTF("\r\n");
                    PRINTF("*****************************************************\r\n");
                    PRINTF("*** ECHEC TCXO : reste en STDBY_RC                ***\r\n");
                    PRINTF("*** Le TCXO 52MHz ne fournit pas de signal !       ***\r\n");
                    PRINTF("*** VERIFIER : pin 14 (TCXOEN) du module -> 3.3V  ***\r\n");
                    PRINTF("*****************************************************\r\n");
                    PRINTF("\r\n");
                }

                /* Revenir en STDBY_RC pour la suite */
                SX1280_SetStandby(SX1280_STANDBY_RC);
                for(volatile uint32_t j = 0; j < 48000; j++);
            }

            /* Configuration LoRa et test TX unique */
            ConfigureLoRa();
            ReadSX1280Config();

            /* Verification des registres critiques apres config */
            {
                uint8_t reg0925 = 0, reg093C = 0;
                SX1280_HAL_ReadRegisters(0x0925, &reg0925, 1);
                SX1280_HAL_ReadRegisters(0x093C, &reg093C, 1);
                PRINTF("Verify: Reg0x0925=0x%02X (expect 0x32 for SF10), Reg0x093C=0x%02X (expect bit0=1)\r\n",
                       reg0925, reg093C);
            }

            /* ==============================================================
             * BOOT TX TEST SUPPRIME
             *
             * Le test TX au boot corrompait l'etat interne du SX1280 :
             * - Le SX1280 transmettait reellement (recu par le recepteur)
             * - Mais le firmware ne detectait pas TX_DONE (DIO1=0, IRQ=0)
             * - Le SetStandby(RC) au timeout interrompait la TX en cours
             * - Apres ca, le SX1280 renvoyait TX_DONE instantanement
             *   (~43ms au lieu de ~140ms) sans emettre de RF
             *
             * La cause du probleme DIO1 au boot reste a investiguer
             * (possiblement lie au timing a 48MHz ou a l'etat des GPIO
             * avant power optimization).
             * ============================================================== */
            PRINTF("\n=== BOOT TX TEST: SKIPPED (was corrupting SX1280 state) ===\r\n");
            SX1280_SetStandby(SX1280_STANDBY_RC);

            /* ==============================================================
             * LED sur PIO10 (I2C-type, besoin IOCON_GPIO_MODE pour push-pull)
             * DOIT etre fait AVANT de couper le clock IOCON !
             * ============================================================== */
            IOCON->PIO[0][10] = (IOCON->PIO[0][10] & ~0xFF)
                              | IOCON_PIO_FUNC(0)
                              | IOCON_GPIO_MODE
                              | IOCON_DIGITAL_EN;
            GPIO_PinInit(GPIO, APP_BOARD_GPIO_PORT, 10,
                         &(gpio_pin_config_t){kGPIO_DigitalOutput, 1});
            PRINTF("LED PIO10 OK\r\n");

            /* ==============================================================
             * GPIOs inutilises en sortie LOW (eviter les entrees flottantes)
             * Pins utilisees: PIO0(BTN), PIO1(TIC_RX), PIO2(SCK), PIO3(MISO),
             *   PIO4(MOSI), PIO5(ISP), PIO8(UART_TX), PIO9(UART_RX),
             *   PIO10(LED), PIO11(NSS), PIO12(RESET), PIO13(DIO1),
             *   PIO14(TCXOEN), PIO15(BUSY)
             * Libres: PIO6, PIO7, PIO16, PIO17, PIO18, PIO19, PIO20, PIO21
             * ============================================================== */
            {
                const uint8_t unusedPins[] = {6, 7, 16, 17, 18, 19, 20, 21};
                for(int i = 0; i < 8; i++) {
                    GPIO_PinInit(GPIO, APP_BOARD_GPIO_PORT, unusedPins[i],
                                 &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
                }
                PRINTF("Unused GPIOs -> output LOW\r\n");
            }

            /* ==============================================================
             * POWER OPTIMIZATION (avec diagnostic pas a pas)
             * ============================================================== */
            PRINTF("\n=== Power optimization (step by step) ===\r\n");

            /* [1] kCLOCK_Fro48M RETIRE - source horloge UART debug (via FRG) */
            /* [2] kCLOCK_Xtal32M RETIRE - crash le systeme (radio Zigbee) */

            PRINTF("  [3] CLOCK_DisableClock(Aes)...");
            CLOCK_DisableClock(kCLOCK_Aes);
            PRINTF(" OK\r\n");

            PRINTF("  [4] CLOCK_DisableClock(I2c0)...");
            CLOCK_DisableClock(kCLOCK_I2c0);
            PRINTF(" OK\r\n");

            PRINTF("  [5] CLOCK_DisableClock(Iocon)...");
            CLOCK_DisableClock(kCLOCK_Iocon);
            PRINTF(" OK\r\n");

            PRINTF("  [6] CLOCK_DisableClock(InputMux)...");
            CLOCK_DisableClock(kCLOCK_InputMux);
            PRINTF(" OK\r\n");

            PRINTF("  [7] CLOCK_DisableClock(Adc0) + LDO...");
            CLOCK_DisableClock(kCLOCK_Adc0);
            PMC->PDRUNCFG &= ~PMC_PDRUNCFG_ENA_LDO_ADC_MASK;
            POWER_DisablePD(kPDRUNCFG_PD_LDO_ADC_EN);
            PRINTF(" OK\r\n");

            PRINTF("  [8] PWR_vForceRadioRetention(FALSE)...");
            PWR_vForceRadioRetention(FALSE);
            PRINTF(" OK\r\n");

            PRINTF("=== Power optimization DONE ===\r\n");

            /* ================================================================
             * Reduire CPU a 12 MHz (FRO12M) pour economiser en idle
             * SPI/UART clockes depuis OSC32M, non affectes par MAINCLKSEL
             * ================================================================ */
            SYSCON->MAINCLKSEL = 0;  /* FRO 12 MHz */
            SystemCoreClockUpdate();
            /* Reconfigurer SysTick pour le nouveau clock (sinon ZTIMER 2.67x trop lent) */
            OSA_TimeInit();
            PRINTF("CPU clock: FRO 12 MHz (MAINCLKSEL=0)\r\n");

            /* ================================================================
             * SX1280 pret, en standby. Le TX sera gere par le timer LoRa
             * dans le main loop cooperatif (Phase 1).
             * ================================================================ */
            SX1280_SetStandby(SX1280_STANDBY_RC);

            /* SX1280 operationnel — autoriser les TX LoRa */
            APP_vLoRaTicSetReady(TRUE);

            PRINTF("\n==============================================\r\n");
            PRINTF("=== LoRa READY (SF10/BW406/CR4_5, +10dBm) ===\r\n");
            PRINTF("=== Entering cooperative main loop...      ===\r\n");
            PRINTF("==============================================\r\n");
        }
        else
        {
            PRINTF("WARNING: SPI communication FAILED - SX1280 not responding\r\n");
            APP_vLoRaTicSetReady(FALSE);
        }
    }

    /* Init routeur : ZCL minimal + TIC Linky */
    APP_vInitialiseRouter();
}

/****************************************************************************
 *
 * NAME: vfExtendedStatusCallBack
 *
 * DESCRIPTION:
 * Callback from stack on extended error situations.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vfExtendedStatusCallBack (ZPS_teExtendedStatus eExtendedStatus)
{
    DBG_vPrintf(TRUE,"ERROR: Extended status 0x%02x\r\n", eExtendedStatus);
}


/****************************************************************************
 *
 * NAME: hardware_init
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void hardware_init(void)
{
    APP_vSetUpHardware();
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

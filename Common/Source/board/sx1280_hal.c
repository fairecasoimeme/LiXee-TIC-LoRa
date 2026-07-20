#include "sx1280_hal.h"
#include "sx1280_config.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_debug_console.h"

#include <string.h>

/*
 * Architecture SPI pour le SX1280 :
 *
 * Le JN5189 ne supporte pas SPI0_SSEL0 sur PIO11, donc on gere le NSS
 * manuellement via GPIO. Le hardware SPI pilote SSEL0 internement (sur
 * une pin non connectee au SX1280).
 *
 * Pour que le transfert SPI fonctionne correctement, il faut:
 * - configFlags = kSPI_FrameAssert pour que le hardware SPI mette le bit
 *   EOT (End Of Transfer) sur le dernier octet. Sans ce bit, le FIFO SPI
 *   ne termine jamais et les clocks peuvent etre bloques.
 * - NSS gere par GPIO avant/apres chaque transfert.
 * - Un seul appel SPI_MasterTransferBlocking par commande SX1280
 *   (pas de transferts multiples au sein d'une meme transaction).
 */

/* Flag pour couper les PRINTF du HAL pendant le TX RF
 * (les PRINTF UART generent de l'activite electrique qui degrade le SNR) */
volatile uint8_t g_u8SX1280_HalSilent = 0;

/* Buffer pour les transferts SPI combines (commande + donnees en une seule transaction) */
#define SX1280_SPI_BUF_SIZE 260
static uint8_t sSpiTxBuf[SX1280_SPI_BUF_SIZE];
static uint8_t sSpiRxBuf[SX1280_SPI_BUF_SIZE];

static void SX1280_HAL_InitPins(void);
static void SX1280_HAL_InitSPI(void);

/* ====== DIAGNOSTIC: pin-by-pin GPIO toggle test for hw debug ====== */
/* Active ce define pour tester individuellement chaque ligne SPI au multimetre */
#define SX1280_DIAG_PIN_TEST  0

#if SX1280_DIAG_PIN_TEST
/* Delai bloquant ~1 sec @ 12MHz CPU */
static void SX1280_HAL_DiagDelay1s(void) {
    for(volatile uint32_t i = 0; i < 2400000; i++);
}

static void SX1280_HAL_DiagPinTest(void)
{
    PRINTF("\r\n========================================\r\n");
    PRINTF(" DIAGNOSTIC PIN TEST - mesure au chip\r\n");
    PRINTF("========================================\r\n");

    /* Activer clocks + IOCON */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Iocon);
    GPIO_PortInit(GPIO, SX1280_GPIO_PORT);

    gpio_pin_config_t out_low  = { kGPIO_DigitalOutput, 0 };
    gpio_pin_config_t out_high = { kGPIO_DigitalOutput, 1 };

    /* Configure NSS, SCK, MOSI, RESET en GPIO output */
    /* NSS = PIO11 (I2C-type) */
    IOCON_PinMuxSet(IOCON, 0, SX1280_NSS_PIN,
                    IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN);
    GPIO_PinInit(GPIO, 0, SX1280_NSS_PIN, &out_high);

    /* RESET = PIO12 (I2C-type + dbg) */
    IOCON_PinMuxSet(IOCON, 0, SX1280_RESET_PIN,
                    IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN |
                    IOCON_INPFILT_OFF | IOCON_PIO_I2C_DBG_FUNC(0) | IOCON_PIO_I2C_DBG_MODE(0));
    GPIO_PinInit(GPIO, 0, SX1280_RESET_PIN, &out_high);

    /* SCK = PIO2 (MFIO) en GPIO */
    IOCON_PinMuxSet(IOCON, 0, SX1280_SPI_SCK_PIN,
                    IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
    GPIO_PinInit(GPIO, 0, SX1280_SPI_SCK_PIN, &out_high);

    /* MOSI = PIO4 (MFIO) en GPIO */
    IOCON_PinMuxSet(IOCON, 0, SX1280_SPI_MOSI_PIN,
                    IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
    GPIO_PinInit(GPIO, 0, SX1280_SPI_MOSI_PIN, &out_high);

    /* === Test chaque pin: HIGH 3s, LOW 3s, HIGH 3s === */
    struct { uint32_t pin; const char* name; uint32_t chip_pin; } pins[] = {
        { SX1280_NSS_PIN,      "NSS",   19 },
        { SX1280_RESET_PIN,    "RESET",  3 },
        { SX1280_SPI_SCK_PIN,  "SCK",   18 },
        { SX1280_SPI_MOSI_PIN, "MOSI",  17 },
    };

    for(int i = 0; i < 4; i++) {
        PRINTF("\r\n>>> [%s/PIO%d] -> chip pin %d <<<\r\n",
               pins[i].name, pins[i].pin, pins[i].chip_pin);

        PRINTF("  >> POSITIONNE LA POINTE SUR LE CHIP PIN %d MAINTENANT <<\r\n", pins[i].chip_pin);
        PRINTF("     (5s pour se preparer...)\r\n");
        GPIO_PinWrite(GPIO, 0, pins[i].pin, 1);
        for(int d=0; d<5; d++) SX1280_HAL_DiagDelay1s();

        PRINTF("  ETAT 1: HIGH (8s) - tu DOIS lire ~3.3V\r\n");
        GPIO_PinWrite(GPIO, 0, pins[i].pin, 1);
        for(int d=0; d<8; d++) SX1280_HAL_DiagDelay1s();

        PRINTF("  ETAT 2: LOW  (8s) - tu DOIS lire ~0V\r\n");
        GPIO_PinWrite(GPIO, 0, pins[i].pin, 0);
        for(int d=0; d<8; d++) SX1280_HAL_DiagDelay1s();

        PRINTF("  ETAT 3: HIGH (8s) - tu DOIS lire ~3.3V\r\n");
        GPIO_PinWrite(GPIO, 0, pins[i].pin, 1);
        for(int d=0; d<8; d++) SX1280_HAL_DiagDelay1s();
    }

    /* Test MISO en input: drive depuis JN5189 impossible, juste lire */
    PRINTF("\r\n>>> [MISO/PIO%d] -> chip pin 16 <<<\r\n", SX1280_SPI_MISO_PIN);
    IOCON_PinMuxSet(IOCON, 0, SX1280_SPI_MISO_PIN,
                    IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
    gpio_pin_config_t in_cfg = { kGPIO_DigitalInput, 0 };
    GPIO_PinInit(GPIO, 0, SX1280_SPI_MISO_PIN, &in_cfg);
    PRINTF("  PIO3 input + pull-up: mesure 3.3V au chip pin 16 (sauf si chip drive LOW)\r\n");
    PRINTF("  GPIO_PinRead(MISO) = %d (1=line HIGH = pull-up OK ou chip drive HIGH)\r\n",
           GPIO_PinRead(GPIO, 0, SX1280_SPI_MISO_PIN));
    SX1280_HAL_DiagDelay1s(); SX1280_HAL_DiagDelay1s(); SX1280_HAL_DiagDelay1s();

    PRINTF("\r\n========================================\r\n");
    PRINTF(" PIN TEST DONE - normal init dans 5s\r\n");
    PRINTF("========================================\r\n\r\n");
    SX1280_HAL_DiagDelay1s();
    SX1280_HAL_DiagDelay1s();
    SX1280_HAL_DiagDelay1s();
    SX1280_HAL_DiagDelay1s();
    SX1280_HAL_DiagDelay1s();
}
#endif /* SX1280_DIAG_PIN_TEST */

void SX1280_HAL_Init(void)
{
#if SX1280_DIAG_PIN_TEST
    SX1280_HAL_DiagPinTest();
#endif
    SX1280_HAL_InitPins();
    SX1280_HAL_InitSPI();
    SX1280_HAL_Reset();
}

static void SX1280_HAL_InitPins(void)
{
    PRINTF("=== Init SX1280 Pins ===\r\n");

    // Enable clocks
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Iocon);

    /*
     * IMPORTANT : GPIO_PortInit DOIT etre appele AVANT la configuration
     * des pins SPI, sinon il reinitialise le port et ecrase les IOCON.
     * On l'appelle ici en premier, puis on configure toutes les pins.
     */
    GPIO_PortInit(GPIO, SX1280_GPIO_PORT);

    // ===== Configuration GPIO de controle (AVANT SPI pour eviter conflit) =====

    /*
     * NSS = PIO11 : cellule I/O de type I2C (comme PIO10).
     * Il FAUT activer IOCON_GPIO_MODE (bit EGP=1) sinon la pin reste
     * en mode I2C open-drain et ne peut pas piloter un niveau HIGH.
     * C'etait la cause du bug "NSS LOW=0 HIGH=0 FAIL".
     *
     * ATTENTION: Pour les pins I2C (PIO10-12), le layout IOCON est different:
     *   bit 3 = EGP (1=GPIO push-pull, 0=I2C open-drain)
     *   bit 4 = ECS (pull-up current enable)
     * Ne PAS utiliser IOCON_MODE_INACT ici car bits 4:3 = MODE seulement
     * pour les pins MFIO. Pour I2C, on met EGP=1 (GPIO_MODE) et ECS=0.
     */
    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_NSS_PIN,
                    IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN);
    gpio_pin_config_t output_high = { kGPIO_DigitalOutput, 1 };
    GPIO_PinInit(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, &output_high);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);
    PRINTF("PIO%d = NSS (GPIO+EGP, HIGH)\r\n", SX1280_NSS_PIN);

    /*
     * RESET = PIO12 : cellule I/O de type I2C + pin debug (SWDIO).
     * Deux problemes potentiels :
     * 1) Sans IOCON_GPIO_MODE (EGP), reste en mode I2C open-drain
     * 2) Les bits DBG_MODE/DBG_FUNC (specifiques aux pins I2C/debug)
     *    peuvent maintenir la fonction debug active.
     *
     * On utilise IOCON_I2C_CFG(0) comme base (EGP=1, digital, filter off,
     * DBG_FUNC=0, DBG_MODE=1 pour signaler qu'on a explicitement choisi)
     * MAIS on met DBG_MODE=0 pour desactiver la fonction debug SWDIO.
     *
     * Layout registre PIO12 (type I2C) :
     *   bits [2:0]  = FUNC (0=GPIO)
     *   bit  [3]    = EGP (1=GPIO mode, pas I2C open-drain)
     *   bit  [4]    = ECS (pull-up current)
     *   bit  [5]    = EHS (speed)
     *   bit  [7]    = DIGIMODE (1=digital)
     *   bit  [8]    = FILTEROFF
     *   bits [16:13] = DBG_FUNC (0=pas de debug)
     *   bit  [17]    = DBG_MODE (0=desactive le debug)
     */
    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_RESET_PIN,
                    IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN |
                    IOCON_INPFILT_OFF | IOCON_PIO_I2C_DBG_FUNC(0) | IOCON_PIO_I2C_DBG_MODE(0));
    GPIO_PinInit(GPIO, SX1280_GPIO_PORT, SX1280_RESET_PIN, &output_high);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_RESET_PIN, 1);
    PRINTF("PIO%d = RESET (GPIO+EGP, dbg off, HIGH)\r\n", SX1280_RESET_PIN);

    /*
     * BUSY = PIO15 (cellule MFIO standard, mais double fonction ADC1)
     * DIGITAL_EN (DIGIMODE=1) obligatoire pour activer le buffer d'entree digital.
     * Sans DIGIMODE=1, le GPIO lit un etat indetermine (souvent HIGH).
     *
     * IMPORTANT: PAS de pull-down ! Le pull-down masquait le signal BUSY du SX1280,
     * empechant la detection correcte de l'etat BUSY=HIGH. Le SX1280 pilote
     * activement ce signal (push-pull), donc pas besoin de pull-up/down.
     * Avec le pull-down, on observait BUSY=0 en permanence alors que le SX1280
     * essayait de le mettre a HIGH (conflit electrique).
     *
     * On utilise IOCON_MODE_INACT (pas de pull) pour lire le vrai etat.
     */
    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_BUSY_PIN,
                    IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
    /* Delai pour laisser le buffer d'entree digital se stabiliser */
    for(volatile uint32_t d = 0; d < 1000; d++);
    gpio_pin_config_t input_config = { kGPIO_DigitalInput, 0 };
    GPIO_PinInit(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN, &input_config);
    PRINTF("PIO%d = BUSY (GPIO input, NO PULL)\r\n", SX1280_BUSY_PIN);
    /* Lecture immediate pour debug */
    PRINTF("  BUSY immediate read: %d (IOCON=0x%08X)\r\n",
           GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN),
           IOCON->PIO[0][SX1280_BUSY_PIN]);

    /*
     * DIO1 = PIO13 (cellule MFIO standard)
     * IMPORTANT: PAS de pull-down ! Le pull-down masquait le signal DIO1 du SX1280
     * (meme probleme que BUSY/PIO15). Le SX1280 pilote DIO1 en push-pull,
     * mais le pull-down peut empecher la detection du front montant TX_DONE.
     * Utiliser IOCON_MODE_INACT (pas de pull) comme pour BUSY.
     */
    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_DIO1_PIN,
                    IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
    GPIO_PinInit(GPIO, SX1280_GPIO_PORT, SX1280_DIO1_PIN, &input_config);
    PRINTF("PIO%d = DIO1 (GPIO input, NO PULL)\r\n", SX1280_DIO1_PIN);

    /*
     * TCXOEN = PIO14 (cellule MFIO standard)
     * CRITIQUE : doit etre HIGH pour alimenter le TCXO 52MHz du module
     * NiceRF LORA1280-TCXO. Sans cette pin a HIGH, le SX1280 n'a pas
     * d'horloge de reference et ne peut pas quitter STDBY_RC.
     * Doit etre active AVANT le reset du SX1280 (dans InitPins, pas dans Reset).
     */
    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_TCXOEN_PIN,
                    IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
    GPIO_PinInit(GPIO, SX1280_GPIO_PORT, SX1280_TCXOEN_PIN, &output_high);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_TCXOEN_PIN, 1);
    PRINTF("PIO%d = TCXOEN (GPIO output, HIGH) - TCXO 52MHz alimenté\r\n", SX1280_TCXOEN_PIN);

    // ===== Configuration SPI0 EN DERNIER (apres GPIO_PortInit + GPIO init) =====
    // Note JN5189 : MODE encoding inverse - 0=PULL_UP, 1=REPEATER, 2=INACTIVE, 3=PULL_DOWN
    // MISO doit avoir pull-up actif sinon la ligne flotte quand le chip est Hi-Z
    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_SPI_SCK_PIN,
                    IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
    PRINTF("PIO%d = SPI0_SCK (FUNC1, pull-up)\r\n", SX1280_SPI_SCK_PIN);

    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_SPI_MISO_PIN,
                    IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
    PRINTF("PIO%d = SPI0_MISO (FUNC1, pull-up)\r\n", SX1280_SPI_MISO_PIN);

    IOCON_PinMuxSet(IOCON, SX1280_GPIO_PORT, SX1280_SPI_MOSI_PIN,
                    IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
    PRINTF("PIO%d = SPI0_MOSI (FUNC1, pull-up)\r\n", SX1280_SPI_MOSI_PIN);

    // === Diagnostic : lire l'etat de toutes les pins ===
    PRINTF("GPIO states: NSS(PIO%d)=%d RESET(PIO%d)=%d BUSY(PIO%d)=%d DIO1(PIO%d)=%d TCXOEN(PIO%d)=%d\r\n",
           SX1280_NSS_PIN, GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN),
           SX1280_RESET_PIN, GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_RESET_PIN),
           SX1280_BUSY_PIN, GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN),
           SX1280_DIO1_PIN, GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_DIO1_PIN),
           SX1280_TCXOEN_PIN, GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_TCXOEN_PIN));
    // Verif IOCON de chaque pin (PIO[port][pin] = 2D array!)
    PRINTF("IOCON: PIO%d=0x%08X PIO%d=0x%08X PIO%d=0x%08X PIO%d=0x%08X\r\n",
           SX1280_NSS_PIN, IOCON->PIO[0][SX1280_NSS_PIN],
           SX1280_RESET_PIN, IOCON->PIO[0][SX1280_RESET_PIN],
           SX1280_BUSY_PIN, IOCON->PIO[0][SX1280_BUSY_PIN],
           SX1280_DIO1_PIN, IOCON->PIO[0][SX1280_DIO1_PIN]);
    PRINTF("IOCON SPI: SCK(PIO%d)=0x%08X MISO(PIO%d)=0x%08X MOSI(PIO%d)=0x%08X\r\n",
           SX1280_SPI_SCK_PIN, IOCON->PIO[0][SX1280_SPI_SCK_PIN],
           SX1280_SPI_MISO_PIN, IOCON->PIO[0][SX1280_SPI_MISO_PIN],
           SX1280_SPI_MOSI_PIN, IOCON->PIO[0][SX1280_SPI_MOSI_PIN]);
    // Verifier que le MASK register ne bloque pas les pins
    PRINTF("GPIO->MASK[0] = 0x%08X (should be 0)\r\n", GPIO->MASK[SX1280_GPIO_PORT]);
    PRINTF("Pins initialized\r\n");
}

static void SX1280_HAL_InitSPI(void)
{
    spi_master_config_t masterConfig;

    PRINTF("=== Init SPI0 ===\r\n");

    // Attach clock to SPI0
    CLOCK_AttachClk(kOSC32M_to_SPI_CLK);
    CLOCK_EnableClock(kCLOCK_Spi0);

    // Reset SPI
    RESET_PeripheralReset(kSPI0_RST_SHIFT_RSTn);

    // Configuration
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = SX1280_SPI_BAUDRATE;
    /*
     * sselNum = kSPI_Ssel0 : le hardware SPI pilote le signal SSEL0 dans
     * les registres FIFOWR (bits TXSSEL0_N). Meme si PIO11 est en mode GPIO,
     * cela est necessaire pour que le driver SPI construise correctement
     * les mots FIFOWR avec les bits de controle SSEL.
     */
    masterConfig.sselNum = kSPI_Ssel0;
    masterConfig.sselPol = kSPI_SpolActiveAllLow;
    masterConfig.direction = kSPI_MsbFirst;
    masterConfig.phase = kSPI_ClockPhaseFirstEdge;    /* CPHA=0 */
    masterConfig.polarity = kSPI_ClockPolarityActiveHigh; /* CPOL=0 */

    uint32_t srcClock_Hz = CLOCK_GetFreq(kCLOCK_Fro32M);
    PRINTF("SPI Clock Source: %lu Hz\r\n", srcClock_Hz);

    SPI_MasterInit(SX1280_SPI_INSTANCE, &masterConfig, srcClock_Hz);

    PRINTF("SPI initialized at %lu bps\r\n", masterConfig.baudRate_Bps);

    /* Debug : verifier les registres SPI */
    PRINTF("SPI0->CFG    = 0x%08X\r\n", SX1280_SPI_INSTANCE->CFG);
    PRINTF("SPI0->STAT   = 0x%08X\r\n", SX1280_SPI_INSTANCE->STAT);
    PRINTF("SPI0->FIFOCFG= 0x%08X\r\n", SX1280_SPI_INSTANCE->FIFOCFG);
}

void SX1280_HAL_Reset(void)
{
    PRINTF("=== SX1280 Reset ===\r\n");

    /*
     * Verifier que TCXOEN est bien HIGH avant le reset.
     * Le TCXO doit etre alimente AVANT que le SX1280 ne sorte de reset,
     * sinon il n'a pas d'horloge de reference et reste bloque en STDBY_RC.
     * Delai minimum : 3ms pour le demarrage du TCXO.
     */
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_TCXOEN_PIN, 1);
    PRINTF("TCXOEN(PIO%d) = %d (must be 1)\r\n",
           SX1280_TCXOEN_PIN,
           GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_TCXOEN_PIN));
    /* 5ms de delai pour le demarrage du TCXO (spec = 3ms min) */
    for(volatile uint32_t i = 0; i < 240000; i++); /* ~5ms @ 48MHz */
    PRINTF("TCXO startup delay done (5ms)\r\n");

    /* Debug : registres GPIO direction et valeur */
    PRINTF("GPIO->DIR[0] = 0x%08X, GPIO->PIN[0] = 0x%08X\r\n",
           GPIO->DIR[SX1280_GPIO_PORT], GPIO->PIN[SX1280_GPIO_PORT]);

    /* Test GPIO NSS (PIO11) */
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t d = 0; d < 100; d++);
    uint8_t nss_low_b = GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);
    for(volatile uint32_t d = 0; d < 100; d++);
    uint8_t nss_high_b = GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN);
    PRINTF("NSS(PIO%d): LOW=%d HIGH=%d %s\r\n", SX1280_NSS_PIN,
           nss_low_b, nss_high_b,
           (nss_low_b == 0 && nss_high_b == 1) ? "OK" : "FAIL");

    /* BUSY avant reset */
    PRINTF("BUSY(PIO%d) before reset: %d\r\n",
           SX1280_BUSY_PIN,
           GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN));

    /*
     * Sequence de reset SX1280 (datasheet section 9.1) :
     *
     * Le signal BUSY peut ne pas etre correctement lu (fil Dupont, pull-down).
     * On utilise un delai fixe de 10ms apres le release de RESET.
     * Le boot du SX1280 prend ~2ms (datasheet), 10ms est tres confortable.
     */

    /* S'assurer que NSS est HIGH pendant le reset */
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    /* Pulse RESET LOW - 1ms (spec = 100us min) */
    PRINTF("Reset pulse LOW...\r\n");
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_RESET_PIN, 0);
    for(volatile uint32_t i = 0; i < 48000; i++); /* ~1ms @ 48MHz */

    /* Verifier que RESET est bien LOW */
    uint8_t rst_low = GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_RESET_PIN);
    PRINTF("RESET during pulse: %d (should be 0), BUSY: %d\r\n",
           rst_low,
           GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN));

    /* Release RESET -> SX1280 boot commence */
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_RESET_PIN, 1);
    PRINTF("Reset released, waiting for BUSY to go LOW...\r\n");

    /*
     * Apres le release de RESET, le SX1280 met BUSY=HIGH pendant le boot
     * (~2ms typique). Maintenant que BUSY est sans pull-down, on peut
     * le poller directement. On garde un delai min de 2ms en backup.
     */
    for(volatile uint32_t i = 0; i < 96000; i++); /* ~2ms min @ 48MHz */

    /* Attendre BUSY LOW (max 20ms supplementaires) */
    {
        uint32_t count = 0;
        uint32_t max_wait = 960000; /* ~20ms @ 48MHz */
        while(GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN))
        {
            count++;
            if(count >= max_wait)
            {
                PRINTF("WARN: BUSY still HIGH after reset (20ms timeout)\r\n");
                break;
            }
        }
        if(count > 0 && count < max_wait)
            PRINTF("BUSY went LOW after ~%lu us\r\n", count / 48);
    }

    PRINTF("Post-reset: BUSY=%d\r\n",
           GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN));

    /*
     * ===== TEST SPI BAS NIVEAU =====
     * Envoyer 0xC0 (GetStatus) directement via les registres SPI
     * pour verifier que le hardware SPI fonctionne et que MISO renvoie des donnees.
     */
    PRINTF("\n=== RAW SPI TEST ===\r\n");

    /* Vider les FIFOs */
    SX1280_SPI_INSTANCE->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    SX1280_SPI_INSTANCE->FIFOSTAT |= SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

    PRINTF("Before: FIFOSTAT=0x%08X STAT=0x%08X\r\n",
           SX1280_SPI_INSTANCE->FIFOSTAT, SX1280_SPI_INSTANCE->STAT);

    /* NSS LOW */
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t d = 0; d < 100; d++);

    /*
     * Ecrire directement dans FIFOWR :
     * bits [7:0]  = txdata (0xC0 = GetStatus)
     * bit  [16]   = TXSSEL0_N (0=assert SSEL0)
     * bit  [20]   = EOT (1=end of transfer)
     * bits [27:24] = LEN (7 = 8 bits, car LEN = nbits-1)
     */
    uint32_t fifowr_val = 0xC0                        /* data */
                        | (0u << 16)                   /* TXSSEL0_N = 0 (assert) */
                        | SPI_FIFOWR_EOT_MASK          /* EOT */
                        | SPI_FIFOWR_LEN(7);           /* 8 bits */
    PRINTF("Writing FIFOWR=0x%08X\r\n", fifowr_val);
    SX1280_SPI_INSTANCE->FIFOWR = fifowr_val;

    /* Attendre que le transfert soit termine */
    uint32_t wait = 0;
    while(!(SX1280_SPI_INSTANCE->FIFOSTAT & SPI_FIFOSTAT_TXEMPTY_MASK))
    {
        wait++;
        if(wait > 100000)
        {
            PRINTF("TX FIFO never emptied! FIFOSTAT=0x%08X\r\n",
                   SX1280_SPI_INSTANCE->FIFOSTAT);
            break;
        }
    }
    PRINTF("TX done (wait=%lu)\r\n", wait);

    /* Attendre un peu que le RX arrive */
    for(volatile uint32_t d = 0; d < 1000; d++);

    PRINTF("After TX: FIFOSTAT=0x%08X STAT=0x%08X\r\n",
           SX1280_SPI_INSTANCE->FIFOSTAT, SX1280_SPI_INSTANCE->STAT);

    /* Lire le RX FIFO */
    if(SX1280_SPI_INSTANCE->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
    {
        uint32_t rxdata = SX1280_SPI_INSTANCE->FIFORD;
        PRINTF("FIFORD = 0x%08X (data=0x%02X, SSEL0_N=%d, SOT=%d)\r\n",
               rxdata,
               rxdata & 0xFF,
               (rxdata >> 16) & 1,
               (rxdata >> 20) & 1);
    }
    else
    {
        PRINTF("RX FIFO empty - no data received!\r\n");
        PRINTF("FIFOSTAT=0x%08X\r\n", SX1280_SPI_INSTANCE->FIFOSTAT);
    }

    /* NSS HIGH */
    for(volatile uint32_t d = 0; d < 100; d++);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    /* Essayer aussi avec 2 octets (GetStatus + NOP) pour voir le status byte */
    PRINTF("\n--- RAW SPI: 2 bytes (0xC0 + 0x00) ---\r\n");
    SX1280_SPI_INSTANCE->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    SX1280_SPI_INSTANCE->FIFOSTAT |= SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t d = 0; d < 100; d++);

    /* Byte 1: 0xC0 sans EOT */
    SX1280_SPI_INSTANCE->FIFOWR = 0xC0 | (0u << 16) | SPI_FIFOWR_LEN(7);
    /* Byte 2: 0x00 avec EOT */
    SX1280_SPI_INSTANCE->FIFOWR = 0x00 | (0u << 16) | SPI_FIFOWR_EOT_MASK | SPI_FIFOWR_LEN(7);

    /* Attendre fin TX */
    wait = 0;
    while(!(SX1280_SPI_INSTANCE->FIFOSTAT & SPI_FIFOSTAT_TXEMPTY_MASK))
    {
        wait++;
        if(wait > 100000) break;
    }
    for(volatile uint32_t d = 0; d < 1000; d++);

    PRINTF("FIFOSTAT=0x%08X\r\n", SX1280_SPI_INSTANCE->FIFOSTAT);

    /* Lire les 2 octets RX */
    for(int i = 0; i < 2; i++)
    {
        if(SX1280_SPI_INSTANCE->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
        {
            uint32_t rxdata = SX1280_SPI_INSTANCE->FIFORD;
            PRINTF("RX[%d] = 0x%02X (raw=0x%08X)\r\n", i, rxdata & 0xFF, rxdata);
        }
        else
        {
            PRINTF("RX[%d] = EMPTY\r\n", i);
        }
    }

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);
    PRINTF("=== END RAW SPI TEST ===\r\n\r\n");
}

uint8_t SX1280_HAL_WaitOnBusy(void)
{
    /*
     * Attend que le SX1280 finisse de traiter la commande precedente.
     *
     * Maintenant que BUSY (PIO15) est sans pull-down, on peut lire
     * le vrai signal BUSY du SX1280. Le protocole SX1280 est :
     * - Le MCU doit attendre BUSY=LOW avant d'envoyer une commande SPI.
     * - Apres chaque commande, BUSY passe a HIGH puis revient a LOW
     *   quand le SX1280 est pret pour la commande suivante.
     *
     * Strategie :
     * 1) Attendre que BUSY passe a LOW (timeout 10ms)
     * 2) Si BUSY reste HIGH, on log un warning et on continue
     *    (pour ne pas bloquer indefiniment si le fil est deconnecte)
     *
     * Temps de traitement SX1280 typiques (datasheet Table 11-2) :
     * - SetStandby : <1us
     * - SetPacketType : <1us
     * - SetRfFrequency : <50us (PLL calibration)
     * - SetTx/SetRx : <50us
     * - Calibrate : ~3.5ms
     * - Reset boot : ~2ms
     */

    uint32_t count = 0;
    uint32_t max_wait = 480000; /* ~10ms @ 48MHz */

    while(GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN))
    {
        count++;
        if(count >= max_wait)
        {
            PRINTF("WARN: WaitOnBusy timeout after ~10ms (BUSY still HIGH)\r\n");
            return 0;
        }
    }

    return 1;
}

/**
 * Transfert SPI unique combine.
 *
 * kSPI_FrameAssert = EOT (End Of Transfer) : OBLIGATOIRE pour que le hardware
 * SPI termine correctement le transfert. Sans ce flag, le FIFO SPI reste en
 * etat "transfert en cours" et les clocks peuvent etre bloques pour les
 * transferts suivants.
 *
 * Le NSS est gere manuellement par GPIO AVANT et APRES cet appel.
 */
static void SX1280_HAL_SpiTransfer(uint8_t *txData, uint8_t *rxData, uint16_t totalSize)
{
    spi_transfer_t xfer;

    /*
     * Flush des FIFOs SPI avant chaque transfert.
     * Il peut rester des donnees parasites dans le FIFO RX suite a
     * un transfert precedent interrompu ou a du bruit sur MISO.
     * Cela evite de lire des donnees corrompues.
     */
    SX1280_SPI_INSTANCE->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    SX1280_SPI_INSTANCE->FIFOSTAT |= SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

    xfer.txData = txData;
    xfer.rxData = rxData;
    xfer.dataSize = totalSize;
    xfer.configFlags = kSPI_FrameAssert; /* EOT sur le dernier octet */

    SPI_MasterTransferBlocking(SX1280_SPI_INSTANCE, &xfer);
}

void SX1280_HAL_WriteCommand(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
    uint16_t totalSize = 1 + size;

    SX1280_HAL_WaitOnBusy();

    /* Construire le buffer TX : [cmd][data...] */
    sSpiTxBuf[0] = cmd;
    if(size > 0 && buffer != NULL)
    {
        memcpy(&sSpiTxBuf[1], buffer, size);
    }
    memset(sSpiRxBuf, 0, totalSize);

    // NSS = LOW
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    // Transfert unique avec EOT
    SX1280_HAL_SpiTransfer(sSpiTxBuf, sSpiRxBuf, totalSize);

    for(volatile uint32_t i = 0; i < 100; i++);
    // NSS = HIGH
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    /* Debug pour les commandes importantes (desactive en mode silent) */
    if(!g_u8SX1280_HalSilent &&
       (cmd == 0x83 || cmd == 0x82 || cmd == 0x86 || cmd == 0x8B ||
        cmd == 0x80 || cmd == 0x8A || cmd == 0x8C || cmd == 0x8E ||
        cmd == 0x8F || cmd == 0x8D || cmd == 0x88 || cmd == 0x96 ||
        cmd == 0x97 || cmd == 0xC1 || cmd == 0xD1 || cmd == 0xD2 ||
        cmd == 0x89)) {
        PRINTF("WriteCmd(0x%02X): TX=", cmd);
        for(int i = 0; i < totalSize && i < 8; i++)
            PRINTF(" %02X", sSpiTxBuf[i]);
        PRINTF(" | RX=");
        for(int i = 0; i < totalSize && i < 8; i++)
            PRINTF(" %02X", sSpiRxBuf[i]);
        PRINTF(" | BUSY=%d\r\n",
               GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN));
    }

    /*
     * Post-transfert : 1us delai puis attente BUSY LOW.
     * C'est exactement ce que fait RadioLib dans SPItransferStream :
     *   delayMicroseconds(1);
     *   while(digitalRead(gpioPin));  // wait BUSY LOW
     * Sans cette attente, la commande suivante peut arriver trop tot
     * et etre ignoree par le SX1280.
     */
    for(volatile uint32_t d = 0; d < 50; d++); /* ~1us */
    SX1280_HAL_WaitOnBusy();
}

void SX1280_HAL_ReadCommand(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
    /* Protocole SX1280 ReadCommand : [cmd][NOP][data...] */
    uint16_t totalSize = 1 + 1 + size;

    SX1280_HAL_WaitOnBusy();

    /* Construire le buffer TX : [cmd][0x00 NOP][0x00...] */
    memset(sSpiTxBuf, 0x00, totalSize);
    sSpiTxBuf[0] = cmd;
    memset(sSpiRxBuf, 0, totalSize);

    // NSS = LOW
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    // Transfert unique avec EOT
    SX1280_HAL_SpiTransfer(sSpiTxBuf, sSpiRxBuf, totalSize);

    for(volatile uint32_t i = 0; i < 100; i++);
    // NSS = HIGH
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    /* Debug : afficher tous les bytes RX */
    if(!g_u8SX1280_HalSilent) {
        PRINTF("ReadCmd(0x%02X): RX[%d]=", cmd, totalSize);
        for(int i = 0; i < totalSize && i < 8; i++)
            PRINTF(" %02X", sSpiRxBuf[i]);
        PRINTF("\r\n");
    }

    /* Les donnees utiles commencent a l'offset 2 (apres cmd + NOP) */
    memcpy(buffer, &sSpiRxBuf[2], size);
}

void SX1280_HAL_WriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
    /* Protocole : [0x18][addr_hi][addr_lo][data...] */
    uint16_t totalSize = 3 + size;

    SX1280_HAL_WaitOnBusy();

    sSpiTxBuf[0] = 0x18; // WRITE_REGISTER
    sSpiTxBuf[1] = (address >> 8) & 0xFF;
    sSpiTxBuf[2] = address & 0xFF;
    memcpy(&sSpiTxBuf[3], buffer, size);
    memset(sSpiRxBuf, 0, totalSize);

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    SX1280_HAL_SpiTransfer(sSpiTxBuf, sSpiRxBuf, totalSize);

    for(volatile uint32_t i = 0; i < 100; i++);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    if(!g_u8SX1280_HalSilent) {
        PRINTF("WriteReg(0x%04X): TX=", address);
        for(int i = 0; i < totalSize && i < 8; i++)
            PRINTF(" %02X", sSpiTxBuf[i]);
        PRINTF(" | RX=");
        for(int i = 0; i < totalSize && i < 8; i++)
            PRINTF(" %02X", sSpiRxBuf[i]);
        PRINTF("\r\n");
    }

    /* Post-transfert : 1us + WaitOnBusy (comme RadioLib) */
    for(volatile uint32_t d = 0; d < 50; d++); /* ~1us */
    SX1280_HAL_WaitOnBusy();
}

void SX1280_HAL_ReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
    /* Protocole : [0x19][addr_hi][addr_lo][NOP][data...] */
    uint16_t totalSize = 4 + size;

    SX1280_HAL_WaitOnBusy();

    memset(sSpiTxBuf, 0x00, totalSize);
    sSpiTxBuf[0] = 0x19; // READ_REGISTER
    sSpiTxBuf[1] = (address >> 8) & 0xFF;
    sSpiTxBuf[2] = address & 0xFF;
    memset(sSpiRxBuf, 0, totalSize);

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    SX1280_HAL_SpiTransfer(sSpiTxBuf, sSpiRxBuf, totalSize);

    for(volatile uint32_t i = 0; i < 100; i++);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    if(!g_u8SX1280_HalSilent) {
        PRINTF("ReadReg(0x%04X): RX[%d]=", address, totalSize);
        for(int i = 0; i < totalSize && i < 8; i++)
            PRINTF(" %02X", sSpiRxBuf[i]);
        PRINTF("\r\n");
    }

    /* Les donnees utiles commencent a l'offset 4 */
    memcpy(buffer, &sSpiRxBuf[4], size);
}

void SX1280_HAL_WriteBuffer(uint8_t offset, uint8_t *buffer, uint16_t size)
{
    /* Protocole : [0x1A][offset][data...] */
    uint16_t totalSize = 2 + size;

    SX1280_HAL_WaitOnBusy();

    sSpiTxBuf[0] = 0x1A; // WRITE_BUFFER
    sSpiTxBuf[1] = offset;
    memcpy(&sSpiTxBuf[2], buffer, size);
    memset(sSpiRxBuf, 0, totalSize);

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    SX1280_HAL_SpiTransfer(sSpiTxBuf, sSpiRxBuf, totalSize);

    for(volatile uint32_t i = 0; i < 100; i++);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    if(!g_u8SX1280_HalSilent) {
        PRINTF("WriteBuffer(offset=%d, size=%d): RX[0]=0x%02X RX[1]=0x%02X BUSY=%d\r\n",
               offset, size, sSpiRxBuf[0], sSpiRxBuf[1],
               GPIO_PinRead(GPIO, SX1280_GPIO_PORT, SX1280_BUSY_PIN));
    }

    /* Post-transfert : 1us + WaitOnBusy (comme RadioLib) */
    for(volatile uint32_t d = 0; d < 50; d++); /* ~1us */
    SX1280_HAL_WaitOnBusy();
}

void SX1280_HAL_ReadBuffer(uint8_t offset, uint8_t *buffer, uint16_t size)
{
    /* Protocole : [0x1B][offset][NOP][data...] */
    uint16_t totalSize = 3 + size;

    SX1280_HAL_WaitOnBusy();

    memset(sSpiTxBuf, 0x00, totalSize);
    sSpiTxBuf[0] = 0x1B; // READ_BUFFER
    sSpiTxBuf[1] = offset;
    memset(sSpiRxBuf, 0, totalSize);

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    SX1280_HAL_SpiTransfer(sSpiTxBuf, sSpiRxBuf, totalSize);

    for(volatile uint32_t i = 0; i < 100; i++);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    /* Les donnees utiles commencent a l'offset 3 */
    memcpy(buffer, &sSpiRxBuf[3], size);
}

uint8_t SX1280_HAL_GetStatus(void)
{
    /*
     * GetStatus (0xC0) sur le SX1280 :
     * On envoie [0xC0][0x00] (commande + NOP).
     * Le status est retourne sur MISO pendant le 2eme octet (NOP).
     * Le 1er octet MISO (pendant la commande) retourne 0xFF.
     *
     * Confirme par le raw SPI test : RX[0]=0xFF, RX[1]=0x61 (status valide).
     */
    uint8_t txBuf[2] = { SX1280_CMD_GET_STATUS, 0x00 };
    uint8_t rxBuf[2] = { 0x00, 0x00 };
    spi_transfer_t xfer;

    SX1280_HAL_WaitOnBusy();

    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 0);
    for(volatile uint32_t i = 0; i < 100; i++);

    xfer.txData = txBuf;
    xfer.rxData = rxBuf;
    xfer.dataSize = 2;
    xfer.configFlags = kSPI_FrameAssert; /* EOT obligatoire */

    SPI_MasterTransferBlocking(SX1280_SPI_INSTANCE, &xfer);

    for(volatile uint32_t i = 0; i < 100; i++);
    GPIO_PinWrite(GPIO, SX1280_GPIO_PORT, SX1280_NSS_PIN, 1);

    return rxBuf[1]; /* Status sur le 2eme octet */
}

#include "sx1280.h"
#include "sx1280_hal.h"
#include "fsl_debug_console.h"

/* Flag defini dans sx1280_hal.c — coupe les PRINTF pendant le TX RF */
extern volatile uint8_t g_u8SX1280_HalSilent;

void SX1280_Init(void)
{
    PRINTF("=== Init SX1280 ===\r\n");

    SX1280_HAL_Init();
    SX1280_SetStandby(SX1280_STANDBY_RC);
    SX1280_SetPacketType(SX1280_PACKET_TYPE_LORA);

    /*
     * SetRegulatorMode : choix entre LDO (0x00) et DC-DC (0x01).
     * - Module NiceRF LORA1280-TCXO : pas de DC-DC sur le PCB -> mode LDO
     * - Shield SX1280RF1ZHP (Semtech) : DC-DC sur le PCB -> mode DC-DC
     * - Proto custom (SX1281IMLTRT + L3 15uH) : DC-DC sur le PCB -> mode DC-DC
     */
    SX1280_SetRegulatorMode(SX1280_REGULATOR_LDO);

    PRINTF("SX1280 initialized\r\n");
}

void SX1280_SetStandby(uint8_t mode)
{
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_STANDBY, &mode, 1);
}

void SX1280_SetPacketType(uint8_t packetType)
{
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_PACKET_TYPE, &packetType, 1);
}

void SX1280_SetRfFrequency(uint32_t frequency)
{
    uint8_t buf[3];

    // Calcul du registre de frequence en arithmetique entiere
    // FREQ_STEP = Fxtal / 2^18 = 52000000 / 262144 = 198.364... Hz
    // freqReg = frequency / FREQ_STEP = frequency * 2^18 / Fxtal
    uint64_t freqCalc = (uint64_t)frequency * 262144ULL;
    uint32_t freqReg = (uint32_t)(freqCalc / 52000000ULL);

    buf[0] = (freqReg >> 16) & 0xFF;
    buf[1] = (freqReg >> 8) & 0xFF;
    buf[2] = freqReg & 0xFF;

    if(!g_u8SX1280_HalSilent)
        PRINTF("SetRfFrequency: freq=%lu, freqReg=0x%06X (%lu), buf=[%02X %02X %02X]\r\n",
               frequency, freqReg, freqReg, buf[0], buf[1], buf[2]);

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_RF_FREQUENCY, buf, 3);
}

void SX1280_SetModulationParams(SX1280_LoRaModParams_t *params)
{
    uint8_t buf[3];

    buf[0] = params->spreadingFactor;
    buf[1] = params->bandwidth;
    buf[2] = params->codingRate;

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_MODULATION_PARAMS, buf, 3);

    /*
     * Ecritures obligatoires apres SetModulationParams (datasheet rev 3.2, section 14.4.1).
     * Confirmees par RadioLib (reg 0x0925 = LORA_SF_CONFIG) et sx1280OverSpi.
     * Sans ces ecritures, le modem LoRa ne fonctionne pas correctement.
     */

    /* Registre 0x0925 : valeur dependante du Spreading Factor */
    uint8_t reg0925;
    if(params->spreadingFactor == SX1280_LORA_SF5 ||
       params->spreadingFactor == SX1280_LORA_SF6)
    {
        reg0925 = 0x1E;
    }
    else if(params->spreadingFactor == SX1280_LORA_SF7 ||
            params->spreadingFactor == SX1280_LORA_SF8)
    {
        reg0925 = 0x37;
    }
    else  /* SF9 a SF12 */
    {
        reg0925 = 0x32;
    }
    SX1280_HAL_WriteRegisters(0x0925, &reg0925, 1);
    if(!g_u8SX1280_HalSilent)
        PRINTF("  Reg 0x0925 = 0x%02X (SF=0x%02X)\r\n", reg0925, params->spreadingFactor);

    /* Registre 0x093C : mettre le bit 0 a 1 (freq error correction).
     * IMPORTANT : read-modify-write (comme RadioLib) pour ne pas
     * ecraser les autres bits du registre. RadioLib fait :
     *   readRegister(0x093C, &data, 1);
     *   data |= 0x01;
     *   writeRegister(0x093C, &data, 1);
     */
    uint8_t reg093C;
    SX1280_HAL_ReadRegisters(0x093C, &reg093C, 1);
    if(!g_u8SX1280_HalSilent)
        PRINTF("  Reg 0x093C before = 0x%02X\r\n", reg093C);
    reg093C |= 0x01;
    SX1280_HAL_WriteRegisters(0x093C, &reg093C, 1);
    if(!g_u8SX1280_HalSilent)
        PRINTF("  Reg 0x093C after  = 0x%02X (bit0 set)\r\n", reg093C);
}

void SX1280_SetPacketParams(SX1280_LoRaPacketParams_t *params)
{
    uint8_t buf[7];

    buf[0] = params->preambleLength;
    buf[1] = params->headerType;
    buf[2] = params->payloadLength;
    buf[3] = params->crc;
    buf[4] = params->invertIQ;
    buf[5] = 0x00;
    buf[6] = 0x00;

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_PACKET_PARAMS, buf, 7);
}

void SX1280_SetTxParams(int8_t power, uint8_t rampTime)
{
    uint8_t buf[2];

    // Limiter la puissance entre -18 et +13 dBm
    if(power < -18) power = -18;
    if(power > 13) power = 13;

    buf[0] = power + 18;  // Offset pour le registre
    buf[1] = rampTime;

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_TX_PARAMS, buf, 2);
}

void SX1280_SetBufferBaseAddress(uint8_t txBase, uint8_t rxBase)
{
    uint8_t buf[2];

    buf[0] = txBase;
    buf[1] = rxBase;

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_BUFFER_BASE_ADDR, buf, 2);
}

void SX1280_SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask)
{
    uint8_t buf[8];

    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF;
    buf[2] = (dio1Mask >> 8) & 0xFF;
    buf[3] = dio1Mask & 0xFF;
    buf[4] = 0x00;  // DIO2 mask
    buf[5] = 0x00;
    buf[6] = 0x00;  // DIO3 mask
    buf[7] = 0x00;

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

void SX1280_SendPayload(uint8_t *payload, uint8_t size)
{
    uint8_t buf[7];

    /*
     * Sequence TX calquee sur RadioLib stageMode(TX) :
     * 1) SetPacketParams avec le payloadLength reel
     * 2) SetBufferBaseAddress
     * 3) WriteBuffer (payload)
     * 4) ClearIrqStatus
     * 5) SetTx (lancement)
     */

    /* 1) Mettre a jour SetPacketParams avec la taille reelle du payload.
     *    C'est OBLIGATOIRE : le SX1280 doit connaitre la taille exacte
     *    pour generer le header LoRa et calculer le temps d'emission.
     *    RadioLib le fait systematiquement avant chaque TX.
     */
    buf[0] = 0x18;                      /* preambleLength = 16 sym (mant=8 * 2^1) */
    buf[1] = SX1280_LORA_HEADER_EXPLICIT;  /* headerType = explicit */
    buf[2] = size;                      /* payloadLength = taille reelle */
    buf[3] = SX1280_LORA_CRC_ON;       /* CRC on */
    buf[4] = SX1280_LORA_IQ_NORMAL;    /* IQ normal */
    buf[5] = 0x00;
    buf[6] = 0x00;
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_PACKET_PARAMS, buf, 7);

    /* 2) SetBufferBaseAddress */
    SX1280_SetBufferBaseAddress(0x00, 0x00);

    /* 3) Ecriture du payload dans le buffer TX (offset 0x00) */
    SX1280_HAL_WriteBuffer(0x00, payload, size);

    /* 3b) Readback verification (uniquement en mode debug, pas pendant TX rapide) */
    if(!g_u8SX1280_HalSilent) {
        uint8_t readback[26];
        uint8_t rbSize = (size <= 26) ? size : 26;
        SX1280_HAL_ReadBuffer(0x00, readback, rbSize);

        uint8_t mismatch = 0;
        for (uint8_t i = 0; i < rbSize; i++) {
            if (readback[i] != payload[i]) {
                mismatch++;
                if (mismatch <= 3)
                    PRINTF("  MISMATCH [%d]: wrote=0x%02X read=0x%02X\r\n",
                           i, payload[i], readback[i]);
            }
        }
        if (mismatch == 0) PRINTF("ReadBuffer OK (%d bytes verified)\r\n", rbSize);
        else PRINTF("*** ReadBuffer MISMATCH: %d/%d bytes differ! ***\r\n", mismatch, rbSize);
    }

    /* 4) Clear toutes les IRQ avant TX */
    SX1280_ClearIrqStatus(SX1280_IRQ_ALL);

    /* 5) SetTx - demarrage de la transmission.
     *    periodBase=0x02 (1ms), count=10000 (0x2710) = timeout 10 secondes.
     *    Avec un timeout, le SX1280 revient en STDBY apres 10s si TX echoue.
     *    Cela genere une IRQ RX_TX_TIMEOUT plutot qu'un blocage indefini.
     */
    buf[0] = 0x02;      /* Periodbase = 1ms */
    buf[1] = 0x27;      /* Timeout MSB = 0x2710 = 10000 */
    buf[2] = 0x10;      /* Timeout LSB */

    if(!g_u8SX1280_HalSilent)
        PRINTF("SetTx: launching transmission...\r\n");
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_TX, buf, 3);
}

void SX1280_SetRx(uint16_t timeout)
{
    uint8_t buf[3];

    buf[0] = 0x02;      // Periodbase = 1ms
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;

    SX1280_HAL_WriteCommand(SX1280_CMD_SET_RX, buf, 3);
}

uint8_t SX1280_GetRxPayload(uint8_t *buffer, uint8_t *size, int8_t *rssi, int8_t *snr)
{
    uint8_t status[2];
    uint8_t packetStatus[5];

    // Lecture du status du buffer RX
    SX1280_HAL_ReadCommand(SX1280_CMD_GET_RX_BUFFER_STATUS, status, 2);
    *size = status[0];
    uint8_t offset = status[1];

    // Lecture du payload
    if(*size > 0 && *size <= 255)
    {
        SX1280_HAL_ReadBuffer(offset, buffer, *size);
    }
    else
    {
        *size = 0;
        return 0;
    }

    // Lecture du packet status (RSSI, SNR)
    SX1280_HAL_ReadCommand(SX1280_CMD_GET_PACKET_STATUS, packetStatus, 5);

    // RSSI = -packetStatus[0] / 2 (dBm)
    *rssi = -(int8_t)(packetStatus[0] / 2);

    // SNR = packetStatus[1] / 4 (dB)
    *snr = ((int8_t)packetStatus[1]) / 4;

    return (*size > 0) ? 1 : 0;
}

uint16_t SX1280_GetIrqStatus(void)
{
    uint8_t irqStatus[2];

    SX1280_HAL_ReadCommand(SX1280_CMD_GET_IRQ_STATUS, irqStatus, 2);

    return ((uint16_t)irqStatus[0] << 8) | irqStatus[1];
}

void SX1280_ClearIrqStatus(uint16_t irq)
{
    uint8_t buf[2];

    buf[0] = (irq >> 8) & 0xFF;
    buf[1] = irq & 0xFF;

    SX1280_HAL_WriteCommand(SX1280_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

uint8_t SX1280_GetStatus(void)
{
    /* GetStatus utilise la fonction HAL dediee car le protocole est special :
     * le status est retourne sur MISO pendant l'envoi de la commande 0xC0 */
    return SX1280_HAL_GetStatus();
}

void SX1280_SetRegulatorMode(uint8_t mode)
{
    /*
     * SetRegulatorMode (0x96) : 1 parametre (0x00=LDO, 0x01=DC-DC).
     *
     * Le choix depend du hardware :
     * - Module NiceRF LORA1280-TCXO : PAS de DC-DC sur le PCB -> LDO (0x00)
     * - Shield SX1280RF1ZHP (Semtech) : DC-DC sur le PCB -> DC-DC (0x01)
     *
     * ATTENTION : si le mode DC-DC est choisi mais qu'il n'y a pas
     * d'inductance externe, le SX1280 crash au PLL lock (BUSY=HIGH).
     * Au power-up le SX1280 est deja en LDO (defaut).
     */
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_REGULATOR_MODE, &mode, 1);

    if(!g_u8SX1280_HalSilent)
        PRINTF("SetRegulatorMode: mode=%s (0x%02X)\r\n",
               (mode == SX1280_REGULATOR_DCDC) ? "DC-DC" : "LDO", mode);
}

void SX1280_SetFs(void)
{
    /* SetFs (0xC1) : pas de parametres.
     * Le SX1280 passe en mode FS (Frequency Synthesis) :
     * le PLL se verrouille sur la frequence configuree.
     * Si le status passe a 0x81 (mode FS), cela confirme que
     * le SX1280 peut quitter STDBY et que le PLL fonctionne. */
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_FS, NULL, 0);
}

void SX1280_SetTxContinuousWave(void)
{
    /* SetTxContinuousWave (0xD1) : pas de parametres.
     * Le SX1280 emet une porteuse continue (onde sinusoidale pure)
     * sans modulation. C'est le test TX le plus simple possible.
     * Le SX1280 reste en mode TX CW jusqu'a ce qu'on envoie SetStandby.
     * Si le status passe a 0xA1 ou 0xE1 (mode TX), cela confirme
     * que le chemin RF TX fonctionne. */
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_TX_CONTINUOUS_WAVE, NULL, 0);
}

void SX1280_SetTxContinuousPreamble(void)
{
    /* SetTxContinuousPreamble (0xD2) : pas de parametres.
     * Le SX1280 emet un preambule LoRa continu.
     * C'est un test intermediaire entre CW et TX normal.
     * Necessite que les modulation params soient configures.
     * Le SX1280 reste en TX jusqu'a SetStandby. */
    SX1280_HAL_WriteCommand(SX1280_CMD_SET_TX_CONTINUOUS_PREAMBLE, NULL, 0);
}

void SX1280_Calibrate(uint8_t calibParam)
{
    /* Calibrate (0x89) : force la recalibration interne du SX1280.
     * Doit etre appele en mode STDBY_RC.
     * calibParam bitmask :
     *   bit6 = RC 64kHz oscillator
     *   bit5 = RC 13MHz oscillator
     *   bit4 = PLL
     *   bit3 = ADC pulse
     *   bit2 = ADC bulk N
     *   bit1 = ADC bulk P
     * 0x7E = recalibrer tout.
     * Duree ~3.5ms (BUSY HIGH pendant la calibration). */
    SX1280_HAL_WriteCommand(SX1280_CMD_CALIBRATE, &calibParam, 1);

    if(!g_u8SX1280_HalSilent)
        PRINTF("SX1280_Calibrate: param=0x%02X\r\n", calibParam);
}


#ifndef SX1280_HAL_H
#define SX1280_HAL_H

#include <stdint.h>

/**
 * @brief Initialise le HAL complet (pins + SPI + reset)
 */
void SX1280_HAL_Init(void);

/**
 * @brief Reset matériel du SX1280
 */
void SX1280_HAL_Reset(void);

/**
 * @brief Attend que le signal BUSY passe à LOW
 * @return 1 si OK, 0 si timeout
 */
uint8_t SX1280_HAL_WaitOnBusy(void);

/**
 * @brief Envoie une commande au SX1280
 * @param cmd Commande à envoyer
 * @param buffer Données à envoyer (peut être NULL)
 * @param size Taille des données
 */
void SX1280_HAL_WriteCommand(uint8_t cmd, uint8_t *buffer, uint16_t size);

/**
 * @brief Lit une réponse du SX1280
 * @param cmd Commande de lecture
 * @param buffer Buffer pour recevoir les données
 * @param size Taille à lire
 */
void SX1280_HAL_ReadCommand(uint8_t cmd, uint8_t *buffer, uint16_t size);

/**
 * @brief Écrit dans les registres du SX1280
 * @param address Adresse du registre
 * @param buffer Données à écrire
 * @param size Taille des données
 */
void SX1280_HAL_WriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief Lit les registres du SX1280
 * @param address Adresse du registre
 * @param buffer Buffer pour recevoir les données
 * @param size Taille à lire
 */
void SX1280_HAL_ReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief Écrit dans le buffer TX/RX du SX1280
 * @param offset Offset dans le buffer
 * @param buffer Données à écrire
 * @param size Taille des données
 */
void SX1280_HAL_WriteBuffer(uint8_t offset, uint8_t *buffer, uint16_t size);

/**
 * @brief Lit le buffer TX/RX du SX1280
 * @param offset Offset dans le buffer
 * @param buffer Buffer pour recevoir les données
 * @param size Taille à lire
 */
void SX1280_HAL_ReadBuffer(uint8_t offset, uint8_t *buffer, uint16_t size);

/**
 * @brief Lit le status du SX1280 (commande speciale 0xC0)
 *        Le status est retourne sur MISO pendant l'envoi de la commande.
 * @return Status (8 bits)
 */
uint8_t SX1280_HAL_GetStatus(void);

/* Commande GetStatus utilisee par le HAL */
#define SX1280_CMD_GET_STATUS  0xC0

#endif // SX1280_HAL_H

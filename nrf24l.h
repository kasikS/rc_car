/*
 * nrf24l.h
 *
 *  Created on: Oct 13, 2019
 *      Author: kasik
 */

#ifndef NRF24L_H_
#define NRF24L_H_

#include <stdint.h>

// Connections;
// PA5 SCK
// PA6 MISO
// PA7 MOSI
// PA4 CS
// PC12 IRQ					??
// PC9 CE (enable RX/TX)	??


//unknown
#define NRF24L_IRQ_PORT     GPIOB
#define NRF24L_IRQ_PIN      GPIO0

#define NRF24L_CE_PORT      GPIOB
#define NRF24L_CE_PIN       GPIO1

/**
 * @brief Initialises the NRF24L module. After initialisation, the module
 * stays in RX mode.
 * @return false in case of error.
 */
int nrf24l_init(void);

/**
 * @brief Sends a byte through the radio link.
 * @param data is the byte to be sent.
 */
int nrf24l_putc(char data);

/**
 * @brief Sends an array of bytes through the radio link.
 * @param buf is the array to be sent.
 */
int nrf24l_puts(const char *buf);

/**
 * Sends a specified number of characters through the radio link (non-blocking).
 * @param string is the data to be sent.
 * @param len is number of characters to be sent.
 * @return false in case of error.
 */
int nrf24l_write(const char *string, int len);

/**
 * @brief Reads a character from the RX buffer (non-blocking).
 * @param c is a pointer to the variable where the read character will be saved.
 * If there is nothing in the buffer, it will not be modified.
 * @return true if a character was received.
 */
int nrf24l_getc(char *c);

/**
 * @brief Checks the module status. See nrf24l_STATUS_xxx defines for details.
 * @return Status word.
 */
uint8_t nrf24l_get_status(void);

uint8_t nrf24_is_sending(void);

/**
 * @brief Sets channel (i.e. radio frequency) used for communication.
 * @param channel is the new channel number. Valid values are in range 0-125.
 */
void nrf24l_set_channel(uint8_t channel);

//void nrf24l_set_power(void);

/**
 * @brief Requests interrupt handler execution. It should be called in
 * nIRQ falling edge interrupt handler.
 */
int nrf24l_irq(void);


#endif /* NRF24L_H_ */

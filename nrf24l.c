/*
 * nrf24l.c
 *
 *  Created on: Oct 13, 2019
 *      Author: kasik
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "nrf24l.h"
#include "nrf24l_reg.h"

#include "link.h"
#include "delay_timer.h"

#define pdTRUE		0
#define pdFALSE		1

#define FALLING 0
#define RISING 1

#ifdef  USE_FULL_ASSERT


  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))


  void assert_failed(uint8_t* file, uint32_t line);


#else


  #define assert_param(expr) ((void)0)


#endif



// Chip select control
static inline void nrf24l_cs_enable(void) { gpio_clear(GPIOA, GPIO_SPI1_NSS); }
static inline void nrf24l_cs_disable(void) { gpio_set(GPIOA, GPIO_SPI1_NSS); }

// RF enable
static inline void nrf24l_ce_enable(void) { gpio_set(NRF24L_CE_PORT, NRF24L_CE_PIN); }
static inline void nrf24l_ce_disable(void) { gpio_clear(NRF24L_CE_PORT, NRF24L_CE_PIN); }

static void nrf24l_transmit(uint8_t data);

static volatile enum MODE nrf24l_mode;

inline static void spi_wait_until_finished(void)
{
    while (SPI_SR(SPI1) & SPI_SR_BSY);
}

// Raw communication functions
inline static void spi_tx(uint8_t data)
{
    // NOTE: you have to handle chipselect!
    spi_send(SPI1, data);
}

inline static uint8_t spi_tx_rx(uint8_t data)
{
    // NOTE: you have to handle chipselect!
    spi_tx(data);
    return spi_read(SPI1);
}

static void nrf24l_raw_multi(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    int i;
    nrf24l_cs_enable();

    if(rx) {
        for(i = 0; i < len; ++i) *rx++ = spi_tx_rx(*tx++);
    } else {
        for(i = 0; i < len; ++i) spi_tx(*tx++);
    }

    spi_wait_until_finished();
    nrf24l_cs_disable();
}

static uint16_t nrf24l_raw_16b(uint16_t data)
{
    uint16_t read;

    nrf24l_cs_enable();
    read = spi_tx_rx((uint8_t)(data >> 8));
    read <<= 8;
    read |= spi_tx_rx((uint8_t)(data & 0xff));
    spi_wait_until_finished();
    nrf24l_cs_disable();

    return read;
}

static uint8_t nrf24l_raw_8b(uint8_t data)
{
    uint8_t read;

    nrf24l_cs_enable();
    read = spi_tx_rx(data);
    nrf24l_cs_disable();

    spi_wait_until_finished();
    return read;
}

static uint8_t nrf24l_read_reg(uint8_t reg)
{
    assert_param(reg <= NRF24L_REG_MAX);
    return (uint8_t) nrf24l_raw_16b((NRF24L_R_REGISTER | reg) << 8);
}

static void nrf24l_write_reg(uint8_t reg, uint8_t val)
{
    assert_param(reg <= NRF24L_REG_MAX);
    nrf24l_raw_16b(((NRF24L_W_REGISTER | reg) << 8) | val);
}

// Higher-level functions
static void nrf24l_flush_rx_fifo(void) { nrf24l_raw_8b(NRF24L_FLUSH_RX); }
static void nrf24l_flush_tx_fifo(void) { nrf24l_raw_8b(NRF24L_FLUSH_TX); }

static void nrf24l_clear_irq(void)
{
    nrf24l_write_reg(NRF24L_STATUS, NRF24L_STATUS_RX_DR |
                                    NRF24L_STATUS_TX_DS |
                                    NRF24L_STATUS_MAX_RT);
}

static void nrf24l_set_mode(enum MODE mode)
{
    const int DEFAULT_CONF = NRF24L_CONF_EN_CRC | NRF24L_CONF_CRCO;

    if(nrf24l_mode == mode)
        return;

    switch(mode)
    {
        case PWR_DOWN:
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, DEFAULT_CONF);
            break;

        case STANDBY:
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, DEFAULT_CONF | NRF24L_CONF_PWR_UP);

            if(nrf24l_mode == PWR_DOWN)
                delay_us(1500);
            break;

        case TX:
            // should not go directly from PWR_DOWN/RX to TX mode (fig.3/datasheet)
            if(nrf24l_mode != STANDBY)
                nrf24l_set_mode(STANDBY);

            nrf24l_flush_tx_fifo();
            nrf24l_clear_irq();
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, DEFAULT_CONF | NRF24L_CONF_PWR_UP);
            // no nrf24l_ce_enable() - it is done in the transmitter task

            if(nrf24l_mode == STANDBY)
                delay_us(130);
            break;

        case RX:
            // should not go directly from PWR_DOWN/TX to RX mode (fig.3/datasheet)
            if(nrf24l_mode != STANDBY)
                nrf24l_set_mode(STANDBY);

            nrf24l_flush_rx_fifo();
            nrf24l_clear_irq();
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, DEFAULT_CONF | NRF24L_CONF_PWR_UP |
                             NRF24L_CONF_PRIM_RX);
            nrf24l_ce_enable();

            if(nrf24l_mode == STANDBY)
                delay_us(130);
            break;

        default:
            assert_param(0);
            break;
    }

#if 0
    switch(mode) {
        case PWR_DOWN:
            serial_puts("pwrdwn\r\n");
            break;

        case STANDBY:
            serial_puts("stdby\r\n");
            break;

        case RX:
            serial_puts("rx\r\n");
            break;

        case TX:
            serial_puts("tx\r\n");
            break;
    }
#endif

    nrf24l_mode = mode;
}

static void nrf24l_read_fifo(uint8_t *data)
{
    const uint8_t cmd[PACKET_TOTAL_SIZE + 1] = { NRF24L_R_RX_PAYLOAD, 0, };
    nrf24l_raw_multi(cmd, data, PACKET_TOTAL_SIZE + 1);
}

static uint8_t nrf24l_get_fifo_status(void)
{
    return nrf24l_read_reg(NRF24L_FIFO_STATUS);
}

int nrf24l_init(void)
{
    // Clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RST_SPI1);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);

    /* Enable AFIO clock. */
    rcc_periph_clock_enable(RCC_AFIO); //what is this for? any other needed?


    // GPIO configuration
    // MISO, MOSI, SCK
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK); //sck

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT, GPIO_SPI1_MISO); //miso

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_MOSI); //mosi

    // CS
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI1_NSS); //ss; correct modes?
    gpio_set(GPIOA, GPIO_SPI1_NSS);

    // CE
    gpio_set_mode(NRF24L_CE_PORT, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, NRF24L_CE_PIN);

    // INT
    gpio_set_mode(NRF24L_IRQ_PORT, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, NRF24L_IRQ_PIN);
    gpio_set(NRF24L_IRQ_PORT, NRF24L_IRQ_PIN);

    // Initially disable CS & RF part
    nrf24l_cs_disable();
    nrf24l_ce_disable();

    // Wait for module to initialize after power-on
    delay_ms(11);

    // Configure IRQ pin interrupt
    nvic_enable_irq(NVIC_EXTI0_IRQ);

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);
    gpio_set_mode(NRF24L_IRQ_PORT, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, NRF24L_IRQ_PIN);
    gpio_clear(NRF24L_IRQ_PORT,NRF24L_IRQ_PIN);

    exti_select_source(EXTI0, NRF24L_IRQ_PORT);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI0);

    // SPI configuration
    //TODO check if correct settings
    spi_reset(SPI1);
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
        //what is the value of fpclk? 72 MHz?

    spi_set_full_duplex_mode(SPI1);
    spi_enable(SPI1);

    // Be sure that we are in the initial mode
    nrf24l_mode = UNKNOWN;
    nrf24l_set_mode(PWR_DOWN);

    nrf24l_set_channel(120);

    // Set pipelines size
    nrf24l_write_reg(NRF24L_RX_PW_P0, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P1, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P2, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P3, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P4, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P5, PACKET_TOTAL_SIZE);

    // Set data rate and output power
    nrf24l_write_reg(NRF24L_RF_SETUP, NRF24L_RF_SET_PWR_0DBM |
                                      NRF24L_RF_SET_RF_250KBPS);

    // Enable CRC, 2-bytes encoding
    nrf24l_write_reg(NRF24L_CONFIG, NRF24L_CONF_EN_CRC | NRF24L_CONF_CRCO);

    // Enable auto-acknowledgment for all pipes
    nrf24l_write_reg(NRF24L_EN_AA, NRF24L_EN_AA_P5 | NRF24L_EN_AA_P4 |
                                   NRF24L_EN_AA_P3 | NRF24L_EN_AA_P2 |
                                   NRF24L_EN_AA_P1 | NRF24L_EN_AA_P0);

    // Enable RX addresses
    nrf24l_write_reg(NRF24L_EN_RXADDR, NRF24L_RXADDR_P5 | NRF24L_RXADDR_P4 |
                                       NRF24L_RXADDR_P3 | NRF24L_RXADDR_P2 |
                                       NRF24L_RXADDR_P1 | NRF24L_RXADDR_P0);

    // Auto retransmit delay: 500 us and up to 15 retransmit trials
    nrf24l_write_reg(NRF24L_SETUP_RETR, NRF24L_SET_RETR_ARD(NRF24L_ARD_500_US) |
                                        NRF24L_SET_RETR_ARC(15));

    // Dynamic length configurations: No dynamic length
    nrf24l_write_reg(NRF24L_DYNPD, NRF24L_DPL_P0 | NRF24L_DPL_P1 |
                                   NRF24L_DPL_P2 | NRF24L_DPL_P3 |
                                   NRF24L_DPL_P4 | NRF24L_DPL_P5);

    nrf24l_flush_tx_fifo();
    nrf24l_flush_rx_fifo();

    nrf24l_clear_irq();
    nrf24l_set_mode(STANDBY);
    nrf24l_set_mode(RX);

    return 0;
}

#if 0
static uint8_t nrf24l_rx_data_ready(void)
{
    // slightly modified get_status(), so we could get both fifo & general
    // status in the same request
    const uint8_t cmd[2] = { NRF24L_R_REGISTER | NRF24L_FIFO_STATUS, 0 };
    uint8_t read[2];

    nrf24l_raw_multi(cmd, read, 2);

    if(read[0] & NRF24L_STATUS_RX_DR)               // general status
        return 1;

    if(!(read[1] & NRF24L_FIFO_STAT_RX_EMPTY))      // fifo status
        return 1;

    return 0;
}
#endif

int nrf24l_putc(char data)
{
    nrf24l_transmit(data);

    return pdTRUE;
}

int nrf24l_puts(const char *buf)
{
    while(*buf)
    {
        if(nrf24l_putc(*buf++) != pdTRUE)
            return pdFALSE;
    }

    return pdTRUE;
}

int nrf24l_write(const char *string, int len)
{
    for(int i = 0; i < len; ++i)
    {
        if(nrf24l_putc(*string++) != pdTRUE)
            return pdFALSE;  // Error, stop filling the queue
    }

    // No errors
    return pdTRUE;
}

static uint8_t rx_buffer[PACKET_TOTAL_SIZE + 1];
static int buffer_index;

int nrf24l_getc(char *c)
{
    if (buffer_index > PACKET_TOTAL_SIZE)
    {
        return 1;
    }
    *c = rx_buffer[buffer_index];
    buffer_index++;
    return 0;
}

uint8_t nrf24l_get_status(void)
{
    // TODO use NOP - it is only one byte
    return nrf24l_read_reg(NRF24L_STATUS);
}

////TODO check this!
//uint8_t nrf24_is_sending(void)
//{
//    uint8_t status;
//
//    /* read the current status */
//    status = nrf24l_get_status();
//
//    /* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
//    if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
//    {
//        return 0; /* false */
//    }
//
//    return 1; /* true */
//}

void nrf24l_set_channel(uint8_t channel)
{
    assert_param(channel <= 125);

    nrf24l_write_reg(NRF24L_RF_CH, channel);
}

#if 0
uint8_t nrf24l_get_lost_packets(void)
{
    return (nrf24l_read_reg(NRF24L_OBSERVE_TX) >> 4);
}

uint8_t nrf24l_get_retransmitted_packets(void)
{
    return (nrf24l_read_reg(NRF24L_OBSERVE_TX) & 0x0f);
}

uint8_t nrf24l_get_rx_power(void)
{
    return nrf24l_read_reg(NRF24L_RPD);
}

void nrf24l_set_power(void)
{
}
#endif


void exti0_isr(void)
{
    exti_reset_request(EXTI0);
    uint8_t status, irq_src;

    status = nrf24l_get_status();
    irq_src = status & (NRF24L_STATUS_RX_DR | NRF24L_STATUS_TX_DS |
                        NRF24L_STATUS_MAX_RT);

    if(status & NRF24L_STATUS_RX_DR) {
#ifdef SHOW_IRQ
        serial_putc('R');
#endif

        nrf24l_read_fifo(rx_buffer);
        // rx_buffer[0] contains shit, start with rx_buffer[1]
        buffer_index = 1;
    }

    if(status & NRF24L_STATUS_TX_DS) {
        nrf24l_ce_disable();
#ifdef SHOW_IRQ
        serial_putc('T');
#endif

        nrf24l_set_mode(RX);
    }

    if(status & NRF24L_STATUS_MAX_RT) {
        nrf24l_ce_disable();
#ifdef SHOW_IRQ
        serial_putc('E');
#endif
    }

    // Clear IRQ
    nrf24l_write_reg(NRF24L_STATUS, irq_src);
}


static void nrf24l_transmit(uint8_t data)
{
    // Full transmit command
   static uint8_t tx_cmd[PACKET_TOTAL_SIZE + 1] = { NRF24L_W_TX_PAYLOAD, 0, };
    // Point to the data buffer in the command above
   static uint8_t * const tx_buf = &tx_cmd[1];
   static uint8_t cnt = 0;

   tx_buf[cnt] = data;

   // TODO check if fifo is not full
    if(++cnt == PACKET_TOTAL_SIZE) {
        cnt = 0;
        nrf24l_set_mode(TX);
        nrf24l_raw_multi(tx_cmd, NULL, PACKET_TOTAL_SIZE + 1);
        nrf24l_ce_enable(); /* ce is disabled in the IRQ handler */
    }
}

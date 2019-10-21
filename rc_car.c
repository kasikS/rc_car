/*
 * Copyright (C) 2019 Maciej Suminski <orson@orson.net.pl>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include "delay_timer.h"
#include "servo.h"
#include "serial.h"
#include "adc.h"
#include "buttons.h"
#include "nrf24l.h"

int main(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOC);

    delay_init();

//    servo_init();

    /* LED pin */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set(GPIOC, GPIO13);

    int pos = MIN_POSITION + (MAX_POSITION - MIN_POSITION) / 2;
    int step = 100;


    serial_init(9600);
    adc_init();
    buttons_init();

	//buffer for characters after itoa
	uint8_t chars[12] = {0,};

	//number of characters to send via serial
	uint32_t length=0;

	//array to store ADC counts
	uint32_t adcBuf[ADC_NUMBER];

	//state of the buttons
	uint16_t buttonsState = 0;


	nrf24l_init();

	uint8_t status;

	/////////////spi
/*	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RST_SPI1);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
    		GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI1_NSS); //ss; correct modes? ok that we control it manually?
    gpio_set(GPIOA, GPIO_SPI1_NSS);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
    		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK); //sck

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_FLOAT, GPIO_SPI1_MISO); //miso

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
    		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_MOSI); //mosi

    spi_reset(SPI1);

    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST); //what is the value of fpclk? 72 MHz? do we send 8 or 16 bits?

    spi_set_full_duplex_mode(SPI1); //needed?
    spi_enable(SPI1);

    uint16_t spi_val = 0;
  */


//ext interrupts
/*	  nvic_enable_irq(NVIC_EXTI0_IRQ);

	  rcc_periph_clock_enable(RCC_GPIOB);
	  rcc_periph_clock_enable(RCC_AFIO);
	  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
	  gpio_clear(GPIOB,GPIO0);

	  exti_select_source(EXTI0, GPIOB);
	  exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	  exti_enable_request(EXTI0);
*/

    while (1)
    {
//        if(pos >= MAX_POSITION)
//            step = -10;
//        else if(pos <= MIN_POSITION)
//            step = 10;
//
//        pos += step;
//        servo_set(pos);


        /*for (int i = 0; i < 0x800000; i++) __asm__("nop");*/


        gpio_toggle(GPIOC, GPIO13);

//		adc
 /*       adc_read(adcBuf);

        for (int chans=0; chans<ADC_NUMBER; chans++)
        {
			length = itoa(adcBuf[chans], chars);
			serial_write(chars, length);

			serial_putc('\n');
			serial_putc('\r');
        }
*/


  //	buttons
  /*      buttonsState = buttons_read();

		length = itoa(buttonsState, chars);
		serial_write(chars, length);
*/

//		spi
/*        gpio_clear(GPIOA, GPIO_SPI1_NSS);

        spi_write(SPI1, 'a');
        spi_val = spi_read(SPI1);

        gpio_set(GPIOA, GPIO_SPI1_NSS);

        serial_putc(spi_val);

        serial_putc('\n');
		serial_putc('\r');
*/

//    	serial_putc('b');
//    	serial_putc('\n');
        status = nrf24l_get_status();

		length = itoa(status, chars);
		serial_write(chars, length);
		serial_putc('\r');
		serial_putc('\n');
		  delay_ms(50);
    }
}


/*void exti0_isr(void)
{
	exti_reset_request(EXTI0);

	serial_putc('a');
	serial_putc('\r');
	serial_putc('\n');
}
*/

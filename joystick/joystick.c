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
#include "link.h"
#include "indicators.h"

int main(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOC);


    delay_init();

    /* LED pin board*/
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set(GPIOC, GPIO13);

    led_init();
    buzzer_init();


    serial_init(9600);
    adc_init();
    buttons_init();

	//buffer for characters after toa
	uint8_t chars[12] = {0,};

	//number of characters to send via serial
	uint32_t length=0;

	//array to store ADC counts
	uint32_t adcBuf[ADC_NUMBER];

	//state of the buttons
	uint16_t buttonsState = 0;

	nrf24l_init();

	uint8_t status;
	struct packet dataPacket;
	uint16_t * dataPointer;

    while (1)
    {
    	//LED board
        gpio_toggle(GPIOC, GPIO13);

//        buzzer_toggle();
//        led_toggle();

        adc_read(adcBuf);
        buttonsState = buttons_read();


        dataPointer = &dataPacket.adc0;
        for (int chans=0; chans<ADC_NUMBER; chans++)
        {
        	*dataPointer =  adcBuf[chans];
        	dataPointer++;
        }
        *dataPointer = buttonsState;

        nrf24l_write((const char*)&dataPacket, PACKET_TOTAL_SIZE);

		delay_ms(50);
    }
}



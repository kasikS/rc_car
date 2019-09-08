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
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>

#include "delay_timer.h"
#include "servo.h"

uint32_t itoa(uint32_t number, uint8_t stringArray[])
{
	uint32_t temp = 0;
	uint32_t digits = 0;
	uint32_t i = 0;

	while(number!=0)
	{
		stringArray[digits] = number % 10;
		number = number/10;
		digits++;
	}

	for(i=0; i< (digits/2); i++)
	{
		temp = stringArray[i];
		stringArray[i] = stringArray[digits-1-i];
		stringArray[digits-1-i] = temp;
	}

	return digits;
}

int main(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOC);

    delay_init();
    servo_init();

    /* LED pin */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set(GPIOC, GPIO13);

    int pos = MIN_POSITION + (MAX_POSITION - MIN_POSITION) / 2;
    int step = 100;

/////////////////////////////////uart

    // todo: send_string; uart_init

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
    		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
        		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_RX);

    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);


/////////////////////////////

/////////////////////////////ADC
    //todo: adc_init; correct set_sample time;
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO0);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO1);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO2);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO3);

    uint8_t adcChannels[4]={ADC_CHANNEL0, ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3};
    uint8_t sequence[1]={ADC_CHANNEL0};

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_power_off(ADC1);
	rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);
	adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_1DOT5CYC); //make it in a loop?
	adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_1DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_1DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL3, ADC_SMPR_SMP_1DOT5CYC);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);
	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);

	uint32_t adcValue = 0;
	uint8_t chars[12] = {0,};
	uint32_t index=0;

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

        for (int chans=0; chans<4; chans++)
        {
			sequence[0]=adcChannels[chans];
			adc_set_regular_sequence(ADC1, 1, sequence);
			adc_start_conversion_regular(ADC1);

			while (! adc_eoc(ADC1));
			adcValue = adc_read_regular(ADC1);

			index = itoa(adcValue, chars);

			for (int i=0; i<index; i++)
			{
				usart_send_blocking(USART1, '0'+ chars[i]);
			}
			usart_send_blocking(USART1, '\n');
			usart_send_blocking(USART1, '\r');
        }

		  delay_ms(50);
    }
}

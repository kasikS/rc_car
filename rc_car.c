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

uint32_t itoa(uint32_t number, char stringArray[])
{
	uint32_t temp = 0;
	uint32_t digits = 0;
	uint32_t i = 0;

	if (number == 0)
	{
		stringArray[0] = '0';
		digits++;
	}

	while(number!=0)
	{
		stringArray[digits] = '0' + number % 10;
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

void serial_init(uint32_t baud)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
    		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
        		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_RX);

    usart_set_baudrate(USART1, baud);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

void serial_putc(const char c)
{
		/* sends a single character*/
		usart_send_blocking(USART1, c);
}

void serial_write(const char *string, int len)
{
	int i;
	for(i=0; i<len; i++)
	{
		/* Puts data into buffer, sends the data*/
		serial_putc(*string);
		++string;
	}
}

#define ADC_NUMBER	4
static uint8_t sequence[1];
static uint8_t adcChannels[ADC_NUMBER];

void adc_init(void)
{
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO0);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO1);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO2);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
    		GPIO_CNF_INPUT_ANALOG, GPIO3);


    int i=0;
    for (i=0; i<ADC_NUMBER; i++)
    {
    	adcChannels[i] = ADC_CHANNEL0 + i;
    }

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_power_off(ADC1);
	rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);
	adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);

    for (i=0; i<ADC_NUMBER; i++)
    {
    	adc_set_sample_time(ADC1, adcChannels[i], ADC_SMPR_SMP_1DOT5CYC);
    }

	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);
	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

void adc_read(uint32_t * adcReadout)
{
	int i = 0;
    for (i = 0; i < ADC_NUMBER; i++)
    {
		sequence[0] = adcChannels[i];
		adc_set_regular_sequence(ADC1, 1, sequence);
		adc_start_conversion_regular(ADC1);

		while (! adc_eoc(ADC1));
		*adcReadout = adc_read_regular(ADC1);

		adcReadout++;
    }
}

#define BUTTONS_NUMBER 9

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


    serial_init(9600);
    adc_init();

	uint32_t adcValue = 0;
	uint8_t chars[12] = {0,};
	uint32_t length=0;
	uint32_t adcBuf[ADC_NUMBER];

	//buttons
   static const uint16_t buttons[BUTTONS_NUMBER] = {GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9, GPIO12, GPIO13};
   uint16_t buttonsState=0;

   rcc_periph_clock_enable(RCC_GPIOB);

   for(int i = 0; i < BUTTONS_NUMBER; i++)
   {
	    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	    		GPIO_CNF_INPUT_PULL_UPDOWN, buttons[i]);
   }

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

//        adc_read(adcBuf);
//
//        for (int chans=0; chans<ADC_NUMBER; chans++)
//        {
//			length = itoa(adcBuf[chans], chars);
//			serial_write(chars, length);
//
//			serial_putc('\n');
//			serial_putc('\r');
//        }

//        if(gpio_get(GPIOB, GPIO3))
//        {
//        	buttonsState |= 1<< 1;
//        } else
//        {
//        	buttonsState &= !(1<<1);
//        }

//array of pins and in for loop; can we do better? any more fancy way?
        buttonsState = 0;
        for(int i = 0; i < BUTTONS_NUMBER; i++)
        {
        	buttonsState |= (gpio_get(GPIOB, buttons[i]) && buttons[i]) << i;
        }

		length = itoa(buttonsState, chars);
		serial_write(chars, length);
		serial_putc('\n');
		serial_putc('\r');

		  delay_ms(50);
    }
}

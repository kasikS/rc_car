/*
 * indicators.c
 *
 *  Created on: Jan 12, 2020
 *      Author: kasik
 */

#include "indicators.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>


#define LED_PORT		GPIOB
#define LED_PIN			GPIO11
#define BUZZE_PORT		GPIOB
#define BUZZER_PIN		GPIO10

void led_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			  GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
}

void led_on(void)
{
	gpio_set(LED_PORT, LED_PIN);
}


void led_off(void)
{
	gpio_clear(LED_PORT, LED_PIN);
}

void led_toggle(void)
{
	 gpio_toggle(LED_PORT, LED_PIN);
}


void buzzer_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(BUZZE_PORT, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, BUZZER_PIN);
}

void buzzer_on(void)
{
	gpio_set(BUZZE_PORT, BUZZER_PIN);
}


void buzzer_off(void)
{
	gpio_clear(BUZZE_PORT, BUZZER_PIN);
}

void buzzer_toggle(void)
{
	 gpio_toggle(BUZZE_PORT, BUZZER_PIN);
}

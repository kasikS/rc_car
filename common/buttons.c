/*
 * buttons.c
 *
 *  Created on: Sep 14, 2019
 *      Author: kasik
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "buttons.h"

#define BUTTONS_NUMBER 9
static const uint16_t buttons[] = {GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9, GPIO12, GPIO13, GPIO14};

/**
 * \brief Initializes GPIO as inputs
 */
void buttons_init(void)
{
	   rcc_periph_clock_enable(RCC_GPIOB);

	   for(int i = 0; i < BUTTONS_NUMBER; i++)
	   {
		    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		    		GPIO_CNF_INPUT_PULL_UPDOWN, buttons[i]);
		    gpio_set(GPIOB, buttons[i]);
	   }
}

/**
 * \brief Reads the state of the buttons
 *
 * \return uint16_t The bit position of the pin value returned corresponds to the pin index in buttons[]
 */
uint16_t buttons_read(void)
{
	uint16_t buttonsState = 0;

    for(int i = 0; i < BUTTONS_NUMBER; i++)
    {
    	if(gpio_get(GPIOB, buttons[i]))
    		buttonsState |= (1 << i);
    }

    return buttonsState;
}

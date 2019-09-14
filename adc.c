/*
 * adc.c
 *
 *  Created on: Sep 14, 2019
 *      Author: kasik
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "adc.h"

//Number of ADC channels used
const int ADC_NUMBER = 4;

//Sequence of channels to be read
static uint8_t sequence[1];

//Array of GPIOs used as ADC
static const uint16_t adcChannels[] = {GPIO0, GPIO1, GPIO2, GPIO3};


/**
 * \brief Initializes ADC1
 */
void adc_init(void)
{
    int i = 0;

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

    for (i = 0; i<ADC_NUMBER; i++)
    {
        gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
        		GPIO_CNF_INPUT_ANALOG, adcChannels[i]);
    }

	adc_power_off(ADC1);
	rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);
	adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);

    for (i = 0; i<ADC_NUMBER; i++)
    {
    	adc_set_sample_time(ADC1, adcChannels[i], ADC_SMPR_SMP_1DOT5CYC);
    }

	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);
	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

/**
 * \brief Reads ADC channels
 *
 * \param adcReadout is a pointer where the counts read from ADC channels are placed
 */
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

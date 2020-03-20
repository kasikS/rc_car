/*
 * Copyright (C) 2020 Maciej Suminski <orson@orson.net.pl>
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

#include "motor.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <inttypes.h>

// Timer frequency [kHz]
#define TIMER_CLK   3200
// Control signal period [ms]
#define PERIOD      3
// Duty cycle responding to the period (100%) [counter units]
#define MAX_DUTY    4096
//((uint32_t)(PERIOD * TIMER_CLK))

/* h-bridge inputs connected to PB6 & PB7 */
#define MOTOR_TIM       TIM4
#define MOTOR_RCC_GPIO  RCC_GPIOB
#define MOTOR_RCC_TIM   RCC_TIM4
#define MOTOR_GPIO      GPIOB
#define MOTOR_OC_A      TIM_OC1
#define MOTOR_OC_B      TIM_OC2
#define MOTOR_TIM_CH_A  GPIO_TIM4_CH1
#define MOTOR_TIM_CH_B  GPIO_TIM4_CH2

#define MOTOR_FAULT_PORT GPIOB
#define MOTOR_FAULT_PIN  GPIO2

#define MOTOR_SLEEP_PORT GPIOB
#define MOTOR_SLEEP_PIN  GPIO3

void motor_init()
{
    rcc_periph_clock_enable(MOTOR_RCC_TIM);
    rcc_periph_clock_enable(MOTOR_RCC_GPIO);
    rcc_periph_clock_enable(RCC_AFIO);

    // I/O
    // disable JTAG pins (PB3)
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);

    gpio_set_mode(MOTOR_GPIO, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MOTOR_TIM_CH_A);
    gpio_set_mode(MOTOR_GPIO, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MOTOR_TIM_CH_B);
    // TODO FAULT
    gpio_set_mode(MOTOR_SLEEP_PORT, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, MOTOR_SLEEP_PIN);
    gpio_set(MOTOR_SLEEP_PORT, MOTOR_SLEEP_PIN);

    // Timer
    timer_set_mode(MOTOR_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(MOTOR_TIM, 2 * rcc_apb1_frequency / (TIMER_CLK * 1000) - 1);
    timer_set_repetition_counter(MOTOR_TIM, 0);
    timer_enable_preload(MOTOR_TIM);
    timer_continuous_mode(MOTOR_TIM);
    timer_set_period(MOTOR_TIM, MAX_DUTY);

    timer_disable_oc_output(MOTOR_TIM, MOTOR_OC_A);
    timer_set_oc_mode(MOTOR_TIM, MOTOR_OC_A, TIM_OCM_PWM1);
    timer_set_oc_value(MOTOR_TIM, MOTOR_OC_A, 0);

    timer_disable_oc_output(MOTOR_TIM, MOTOR_OC_B);
    timer_set_oc_mode(MOTOR_TIM, MOTOR_OC_B, TIM_OCM_PWM1);
    timer_set_oc_value(MOTOR_TIM, MOTOR_OC_B, 0);

    timer_enable_oc_output(MOTOR_TIM, MOTOR_OC_A);
    timer_enable_oc_output(MOTOR_TIM, MOTOR_OC_B);

    timer_enable_counter(MOTOR_TIM);
}

void motor_run(direction_t dir, unsigned int speed)
{
    if (speed >= MAX_DUTY)
        speed = MAX_DUTY - 1;

    switch (dir)
    {
        case FORWARD:
            timer_set_oc_value(MOTOR_TIM, MOTOR_OC_A, speed);
            timer_set_oc_value(MOTOR_TIM, MOTOR_OC_B, 0);
            break;

        case REVERSE:
            timer_set_oc_value(MOTOR_TIM, MOTOR_OC_A, 0);
            timer_set_oc_value(MOTOR_TIM, MOTOR_OC_B, speed);
            break;
    }
}

void motor_stop()
{
    timer_set_oc_value(MOTOR_TIM, MOTOR_OC_A, 0);
    timer_set_oc_value(MOTOR_TIM, MOTOR_OC_B, 0);
}

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

#include "servo.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <inttypes.h>

const int MIN_POSITION = 400;
const int MAX_POSITION = 2700;

// Timer frequency [kHz]
#define TIMER_CLK   3200
// Control signal period [ms]
#define PERIOD      3
// Duty cycle responding to the period (100%) [counter units]
#define MAX_DUTY    ((uint32_t)(PERIOD * TIMER_CLK))
// One millisecond [counter units]
#define MILLISECOND ((uint32_t)(MAX_DUTY / PERIOD))

 /* servo connected to PA8 */
#define SERVO_TIM       TIM1
#define SERVO_RCC_TIM   RCC_TIM1
#define SERVO_OC        TIM_OC1
#define SERVO_GPIO      GPIOA
#define SERVO_TIM_CH    GPIO_TIM1_CH1

void servo_init(void)
{
    rcc_periph_clock_enable(SERVO_RCC_TIM);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_AFIO);

    // I/O
    gpio_set_mode(SERVO_GPIO, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, SERVO_TIM_CH);

    // Timer
    timer_set_mode(SERVO_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(SERVO_TIM, 2 * rcc_apb1_frequency / (TIMER_CLK * 1000) - 1);
    timer_set_repetition_counter(SERVO_TIM, 0);
    timer_enable_preload(SERVO_TIM);
    timer_continuous_mode(SERVO_TIM);
    timer_set_period(SERVO_TIM, MAX_DUTY);

    timer_disable_oc_output(SERVO_TIM, SERVO_OC);
    timer_set_oc_mode(SERVO_TIM, SERVO_OC, TIM_OCM_PWM1);
    timer_set_oc_value(SERVO_TIM, SERVO_OC, 0);

    timer_enable_oc_output(SERVO_TIM, SERVO_OC);

    timer_enable_break_main_output(SERVO_TIM);
    timer_enable_counter(SERVO_TIM);
}

void servo_set(int position)
{
    if(position < MIN_POSITION)
	position = MIN_POSITION;
    else if(position > MAX_POSITION)
	position = MAX_POSITION;

    // PPM output:
    // |------|____________|----..
    // <-tHI-> <---tLOW--->
    // tHI + tLOW = 20 ms (50 Hz)
    // min position for tHI = 1 ms
    // max position for tHI = 2 ms
    int val = (position * MILLISECOND) / 1000;

    timer_set_oc_value(SERVO_TIM, SERVO_OC, val);
}

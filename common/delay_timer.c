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

#include "delay_timer.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

// Multiplier adjusted to reduce rounding error for prescaler
#define TIMER_FREQ_MPLIER   16
#define TIMER_FREQ          (1000000 * TIMER_FREQ_MPLIER)
#define TIMER_USECS_LIMIT   (65535 / TIMER_FREQ_MPLIER - 1)

void delay_init(void)
{
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, ((rcc_apb1_frequency * 2) / TIMER_FREQ));
    timer_set_period(TIM3, 65535);
    timer_enable_preload(TIM3); // TODO check
    timer_continuous_mode(TIM3);
    timer_enable_counter(TIM3);
}

void delay_us(int usecs)
{
    // Split the delay value to smaller ones if necessary
    while(usecs > TIMER_USECS_LIMIT)
    {
        delay_us(TIMER_USECS_LIMIT);
        usecs -= TIMER_USECS_LIMIT;
    }

    uint32_t finish = TIMER_FREQ_MPLIER * usecs;
    timer_set_counter(TIM3, 0);
    while(timer_get_counter(TIM3) < finish);
}

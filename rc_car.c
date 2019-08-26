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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "delay_timer.h"
#include "servo.h"

int main(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOC);

    delay_init();
    servo_init();

    /* LED pin */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
    gpio_set(GPIOC, GPIO11);

    int pos = MIN_POSITION + (MAX_POSITION - MIN_POSITION) / 2;
    int step = 100;

    while (1)
    {
        if(pos >= MAX_POSITION)
            step = -10;
        else if(pos <= MIN_POSITION)
            step = 10;

        pos += step;
        servo_set(pos);


        /*for (int i = 0; i < 0x800000; i++) __asm__("nop");*/
        delay_ms(50);
        gpio_toggle(GPIOC, GPIO11);
    }
}

/*
 * Copyright (C) 2015 Maciej Suminski <orson@orson.net.pl>
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

#ifndef LINK_H
#define LINK_H

#include <stdint.h>

#define PACKET_TOTAL_SIZE   ((int)sizeof(struct packet))

struct packet
{
    // be sure to have all fields with PACKET_DATA_SIZE length
	uint16_t adc0;
	uint16_t adc1;
	uint16_t adc2;
	uint16_t adc3;
	uint16_t buttons;

} __attribute__((packed));

#endif /* LINK_H */

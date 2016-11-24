/*
 * Copyright (c) 2014 Manuel Braga <mul.braga@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "vepoc.h"

#define SQUARESIZE 16
#define SQUARE_BORDERLUMA   0xf0

void picture_generate(uint32_t width, uint32_t height, uint8_t *Y, uint8_t *C)
{
	int x;
	int y;
	int pixelNum;
	int m;
	int w2 = width / 2;
	int w4 = width / 4;

    /* generate Luma buffer */
	for(y = 0; y < height; y++) {
		for(x = 0; x < width; x++) {
			pixelNum = width * y + x;
			m = y / SQUARESIZE;
			if(x % SQUARESIZE && y % SQUARESIZE) {
				if(m < 12) {
					Y[pixelNum] = 0x10 + 0x30 * (x / w4);
				} else {
					Y[pixelNum] = 0x20 + 0xc0 * x / width;
                }
			} else {
				Y[pixelNum] = SQUARE_BORDERLUMA;
            }
		}
    }
	
    /* generate Chroma buffer */
	for(y = 0; y < height; y++) {
		for(x = 0; x < width; x++) {
			pixelNum = width * y + x;
			m = y / SQUARESIZE;
			if(m < 4) {
				C[pixelNum] = 0x80;
			} else if(m == 4) {
				if(x / w2) {
					C[pixelNum] = (x % 2) ? 0x30 : 0xc0;
				} else {
					C[pixelNum] = (x % 2) ? 0xc0 : 0x30;
                }
			} else if(m == 5) {
				if(x / w2) {
					C[pixelNum] = (x % 2) ? 0x80 : 0x10;
				} else {
					C[pixelNum] = (x % 2) ? 0x80 : 0xf0;
                }
			 } else {
				C[pixelNum] = 0x20 + 0xc0 * x / width;
            }
		}
    }
}


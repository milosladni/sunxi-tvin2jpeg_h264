/*
 * Copyright (c) 2014 Manuel Braga <mul.braga@gmail.com>
 * Copyright (c) 2016 Milos Ladicorbic <milos dot ladicorbic at gmail dot com>
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

static veavc_encoder_mode_t encoder_mode = VEAVC_ENCODER_MODE_H264;

void veavc_select_subengine(uint8_t engine) {
    
	uint32_t ctrl = L(VE_CTRL);
	ctrl = (ctrl & 0xfffffff0) | (engine & 0xf);
	S(ctrl, VE_CTRL);
}

void veavc_release_subengine (void) {
    
	uint32_t ctrl = L(VE_CTRL);
    
	ctrl = (ctrl & 0xfffffff0) | 0x7;
	S(ctrl, VE_CTRL);
}

void veavc_subengine_reset (void) {
    
	uint32_t flags = 0x01;
    
	S(flags, VE_SUBENGINE_RESET);
}

void veavc_init_vle(uint8_t *J, uint32_t size)
{
	uint32_t pJ = ve_virt2phys(J);
	uint32_t end = pJ + size - 1;
	uint32_t maxbits = (size * 8 + 0xffff) & ~0xffff;
	uint32_t max = maxbits > 0x0fff0000 ? 0x0fff0000 : maxbits;
    S(0	        , VE_AVC_VLE_OFFSET);                                    /* IT MUST BE SET FIRST! */
	S(pJ	    , VE_AVC_VLE_ADDR);
	S(end	    , VE_AVC_VLE_END);
    //S(0	        , VE_AVC_VLE_OFFSET);                                    /* IT MUST BE SET FIRST! */
	S(max	    , VE_AVC_VLE_MAX);
	//printf("[VEAVC] outbuf of size %d, write only max %d bytes\n", size, max / 8);
}

void veavc_init_ctrl(veavc_encoder_mode_t mode)
{
	uint32_t trigger = (mode & 1) << 16;
	uint32_t status;
	encoder_mode = mode;

	S(0x0000000f  , VE_AVC_CTRL);                                          /* enable IRQ from AVC */
	S(trigger     , VE_AVC_TRIGGER);                                        /* select jpeg triger */

	/* clear status bits */
	status = L(VE_AVC_STATUS);
	S(status | 0xf, VE_AVC_STATUS);
}

void veavc_jpeg_parameters(uint8_t fill1, uint8_t stuff, uint32_t biasY, uint32_t biasC)
{
	uint32_t valfill1 = fill1 > 0 ? 1 : 0;
	uint32_t valstuff = stuff > 0 ? 1 : 0;
	uint32_t value = 0;
	value |= (valfill1 & 1) << 31;
	value |= (valstuff & 1) << 30;
	value |= (biasC & 0x7ff) << 16;
	value |= (biasY & 0x7ff) << 0;
	S(value, VE_AVC_PARAM);
}

static const char* status_to_print(uint32_t status)
{
	uint32_t value = status & 0xf;
	if(value == 0) return "none";
	if(value == 1) return "success";
	if(value == 2) return "failed";
	return "unknown";
}

void veavc_put_bits(uint8_t nbits, uint32_t data)
{
	uint32_t trigger = (encoder_mode & 1) << 16;
	uint32_t status;
	trigger |= (nbits & 0x3f) << 8;
	trigger |= 1;
	S(data   , VE_AVC_BASIC_BITS);
	S(trigger, VE_AVC_TRIGGER);

	status = L(VE_AVC_STATUS) & 0xf;
	if(status)
		printf("[VEAVC] put bits status %d (%s)\n", status, status_to_print(status));
}

void veavc_sdram_index(uint32_t index)
{
	S(index, VE_AVC_SDRAM_INDEX);
}

void veavc_jpeg_quantization(uint16_t *tableY, uint16_t *tableC, uint32_t length)
{
	uint32_t data;
	uint32_t i;

	veavc_sdram_index(0x0);

/*
	When compared to libjpeg, there are still rounding errors in the
	coefficients values (around 1 unit of difference).
*/
	for(i = 0; i < length; i++)
	{
		data  = 0x0000ffff & (0xffff / tableY[i]);
		data |= 0x00ff0000 & (((tableY[i] + 1) / 2) << 16);
		S(data, VE_AVC_SDRAM_DATA);
	}
	for(i = 0; i < length; i++)
	{
		data  = 0x0000ffff & (0xffff / tableC[i]);
		data |= 0x00ff0000 & (((tableC[i] + 1) / 2) << 16);
		S(data, VE_AVC_SDRAM_DATA);
	}
}

void veavc_launch_encoding(void)
{
	uint32_t trigger = (encoder_mode & 1) << 16;
	trigger |= 8;
	S(trigger, VE_AVC_TRIGGER);
}

void veavc_check_status(void)
{
	uint32_t status = L(VE_AVC_STATUS) & 0xf;
	if(status) {
		//printf("[VEAVC] finish status %d (%s)\n", status, status_to_print(status));
    }
    S(status | 0x07, VE_AVC_STATUS);
}

uint32_t veavc_get_written(void)
{
	uint32_t bits = L(VE_AVC_VLE_LENGTH);
	return bits / 8;
}

void veavc_set_mbInfo(uint8_t *pBuff) {

	uint32_t pMbInfoBuff = ve_virt2phys(pBuff);
	S(pMbInfoBuff, VE_AVC_MB_INFO);
}


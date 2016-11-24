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
#include "debug.h"

void veisp_set_buffers(uint8_t *Y, uint8_t *C)
{
	uint32_t pY = ve_virt2phys(Y);
	uint32_t pC = ve_virt2phys(C);
	S(pY, VE_ISP_PIC_LUMA);
	S(pC, VE_ISP_PIC_CHROMA);
}

void veisp_set_outputBuffers (uint8_t *pY, uint8_t *pC) {
    
	uint32_t physY = ve_virt2phys(pY);
	uint32_t physC = ve_virt2phys(pC);
	S(physY, VE_ISP_OUTPUT_LUMA);
	S(physC, VE_ISP_OUTPUT_CHROMA);
}

inline void veisp_set_picture_size(uint32_t w, uint32_t h) {
    
    uint32_t width_mb  = DIV_ROUND_UP(w, 16);
	uint32_t height_mb = DIV_ROUND_UP(h, 16);
	uint32_t size  = ((width_mb & 0x3ff) << 16) | (height_mb & 0x3ff);
	uint32_t stride = (width_mb & 0x3ff) << 16;
    
    DBGF("Width: %d, Height: %d => %dx%d macroblocks", w, h, width_mb, height_mb);

    S(size	, VE_ISP_PIC_SIZE);
	S(stride, VE_ISP_PIC_STRIDE);
    
}

void veisp_init_picture(uint32_t w, uint32_t h, veisp_color_format_t f) {
    
	uint32_t format = (f & 0xf) << 29;
	veisp_set_picture_size(w, h);

	S(format, VE_ISP_CTRL);
    
    	/* enable interrupt and clear status flags */
	//writel(readl(regs + VE_AVC_CTRL) | 0xf, regs + VE_AVC_CTRL);
	//writel(readl(regs + VE_AVC_STATUS) | 0x7, regs + VE_AVC_STATUS);
}

void veisp_init_pictureMb(uint32_t width_mb, uint32_t height_mb, uint32_t stride_mb, veisp_color_format_t f) {
    
	uint32_t format = (f & 0xf) << 29;
	uint32_t size  = ((width_mb & 0x3ff) << 16) | (height_mb & 0x3ff);
	uint32_t stride = (stride_mb & 0x3ff) << 16;
    
    S(size	, VE_ISP_PIC_SIZE);
	S(stride, VE_ISP_PIC_STRIDE);
	S(format, VE_ISP_CTRL);
}

void veisp_init_pictureMbWithDivision (uint32_t width_mb, uint32_t height_mb, veisp_color_format_t f) {
    
	uint32_t format;
    uint32_t outpicStride_mb;
    uint32_t size;
	uint32_t stride;
    
    /* set macroblocks size */
    outpicStride_mb  = width_mb>>1;
    size  = ((width_mb & 0x3ff) << 16) | (height_mb & 0x3ff);
    stride = ((width_mb & 0x3ff) << 16) | (outpicStride_mb & 0x3ff);
    S(size	, VE_ISP_PIC_SIZE);
	S(stride, VE_ISP_PIC_STRIDE);

    /* enable division */
    format = (f & 0xf) << 29;
    format |= 0x02 << 25;                                                    /* set division to 2 */
    format |= 0x01 << 19;                                                        /* enable output */
    //format |= 0x01 << 16;                                     /* Enable scaler/Clear IRQ status */
    format |= 0x01;                                                        /* Enable IRQ from ISP */
	S(format, VE_ISP_CTRL);
}

void veisp_triger (void) {
    
	uint32_t trigerFlags = 0x01;
    
	S(trigerFlags, VE_ISP_TRIGGER);
}

/*
 Quick and very hurrily done calculation of the polyphase filter coeficients
 for bilinear scaling. Table A is for horizontal direction and table B for
 vertical.
*/
void veisp_write_table_A(void) {
    
	uint16_t h0;
	uint16_t h1;
	uint16_t h2;
	uint16_t h3;
	uint32_t h1h0;
	uint32_t h3h2;
	int i;
    
	for(i = 0; i < 16; i++) {
		h0 = 0;
		h1 = 0x100 - (i * (0x100/16));
		h2 = 0x100 - h1;
		h3 = 0;

		h1h0 = (h1 << 16) | h0;
		h3h2 = (h3 << 16) | h2;
		S(h1h0, VE_ISP_SRAM_DATA);
		S(h3h2, VE_ISP_SRAM_DATA);
	}
}

void veisp_write_table_B(void) {
    
	uint16_t h0;
	uint16_t h1;
	uint32_t h1h0;
	int i;
	for(i = 0; i < 32; i++)
	{
		h0 = 0x100 - (i * (0x100/32));
		h1 = 0x100 - h0;

		h1h0 = (h1 << 16) | h0;
		S(h1h0, VE_ISP_SRAM_DATA);
	}
}

void veisp_set_scaler (uint32_t newwidth, uint32_t newheight,
                       float offsetx, float offsety, float scalex, float scaley) {
                           
	uint32_t width_mb  = DIV_ROUND_UP(newwidth, 16);
	uint32_t height_mb = DIV_ROUND_UP(newheight, 16);
	uint32_t newsize = ((height_mb & 0xff) << 8) | (width_mb & 0xff);
	uint32_t offx = (0x100 * offsetx);
	uint32_t offy = (0x100 * offsety);
	uint32_t offset = ((offy & 0xffff) << 16) | (offx & 0xffff);
	uint32_t sx = (0x100 / scalex);
	uint32_t sy = (0x100 / scaley);
	uint32_t factor = ((sy & 0xfff) << 12) | (sx & 0xfff);


	S(newsize, VE_ISP_SCALER_SIZE);
	S(offset , VE_ISP_SCALER_OFFSET_Y);
	S(offset , VE_ISP_SCALER_OFFSET_C);
	S(factor , VE_ISP_SCALER_FACTOR);

	S(0x400, VE_ISP_SRAM_INDEX);
	veisp_write_table_A();
	veisp_write_table_B();

	uint32_t ctrl = L(VE_ISP_CTRL) | (1 << 16);
	S(ctrl, VE_ISP_CTRL);
}



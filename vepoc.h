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

#ifndef _VEPOC_H_
#define _VEPOC_H_

#include <stdint.h>
#include "ve.h"

#define ALIGN(x, a) (((x) + ((typeof(x))(a) - 1)) & ~((typeof(x))(a) - 1)) /* zaokruzuje da bude deljivo sa a */
#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)            /* deljivo sa 'a' vraca 0 */
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))                     /* vraca broj makroblokova */

typedef enum {
	VEISP_COLOR_FORMAT_NV12 = 0,
	VEISP_COLOR_FORMAT_NV16 = 1,
	VEISP_COLOR_FORMAT_TILE = 2,
} veisp_color_format_t;

typedef enum {
	VEAVC_ENCODER_MODE_H264 = 0,
	VEAVC_ENCODER_MODE_JPEG = 1,
} veavc_encoder_mode_t;

void picture_generate(uint32_t width, uint32_t height, uint8_t *Y, uint8_t *C);

void vejpeg_header_create(int w, int h, int quality);
void vejpeg_header_destroy(void);
void vejpeg_write_SOF0(void);
void vejpeg_write_SOS(void);
void vejpeg_write_quantization(void);
void vejpeg_write_file(const char *filename, uint8_t *buffer, uint32_t length);

void veisp_set_buffers(uint8_t *Y, uint8_t *C);                          /* set isp input buffers */
void veisp_set_outputBuffers (uint8_t *pY, uint8_t *pC); /* set isp output buffers for divide-scaled raw frame */
void veisp_set_picture_size(uint32_t w, uint32_t h); /* set isp subengine source picture size in macroblocks (16x16) */
void veisp_init_picture(uint32_t w, uint32_t h, veisp_color_format_t f);
void veisp_init_pictureMb(uint32_t width_mb, uint32_t height_mb, uint32_t stride_mb, veisp_color_format_t f);
void veisp_init_pictureMbWithDivision(uint32_t width_mb, uint32_t height_mb, veisp_color_format_t f);
void veisp_triger (void);
void veisp_set_scaler (uint32_t newwidth, uint32_t newheight, float offsetx, float offsety, float scalex, float scaley);

void veavc_select_subengine(void);                                 /* select h264-jpeg encoder VE */
void veavc_select_ISPsubengine(void);                                            /* select ISP VE */
void veavc_release_subengine(void);                                          /* release subengine */ 
void veavc_subengine_reset(void);                                              /* reset subengine */
void veavc_init_vle(uint8_t *J, uint32_t size);
void veavc_init_ctrl(veavc_encoder_mode_t mode);
void veavc_jpeg_parameters(uint8_t fill1, uint8_t stuff, uint32_t biasY, uint32_t biasC);
void veavc_put_bits(uint8_t nbits, uint32_t data);
void veavc_sdram_index(uint32_t index);
void veavc_jpeg_quantization(uint16_t *tableY, uint16_t *tableC, uint32_t length);
void veavc_launch_encoding(void);
void veavc_check_status(void);
uint32_t veavc_get_written(void);
void veavc_set_mbInfo(uint8_t *pBuff);

#endif


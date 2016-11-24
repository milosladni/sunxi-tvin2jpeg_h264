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
#include <jpeglib.h>
#include "vepoc.h"

static struct jpeg_error_mgr err;
static struct jpeg_compress_struct jcs;
unsigned char *outbuf = NULL;
unsigned long  outlen = 0;

/*
	Use libjpeg to create the quantization tables and the huffman tables.
*/
void vejpeg_header_create(int w, int h, int quality)
{
	jcs.err = jpeg_std_error(&err);
	jpeg_create_compress(&jcs);
	jpeg_mem_dest(&jcs, &outbuf, &outlen);

	jcs.image_width = w;
	jcs.image_height = h;
	jcs.input_components = 3;
	jcs.in_color_space = JCS_YCbCr;
	jpeg_set_defaults(&jcs);
	jpeg_set_quality(&jcs, quality, TRUE);

/*
	SOI,
	DQT, DQT,
	DHT, DHT, DHT, DHT,
	EOI
*/
	jpeg_write_tables(&jcs);
	/* EOI is also written. */
}

void vejpeg_header_destroy(void)
{
	jpeg_destroy_compress(&jcs);
	free(outbuf);
}

void vejpeg_write_SOF0(void)
{
	int i;
	jpeg_component_info *c;
	unsigned short length;
	veavc_put_bits(16, 0xffc0);

	length = 2 + 1 + 2 + 2 + 1 + 3 * jcs.num_components;
	veavc_put_bits(16, length);
	veavc_put_bits( 8, jcs.data_precision);
	veavc_put_bits(16, jcs.image_height);
	veavc_put_bits(16, jcs.image_width);
	veavc_put_bits( 8, jcs.num_components);
	for(i = 0; i < jcs.num_components; i++)
	{
		c = &jcs.comp_info[i];
		veavc_put_bits(8, c->component_id);
		veavc_put_bits(4, c->h_samp_factor);
		veavc_put_bits(4, c->v_samp_factor);
		veavc_put_bits(8, c->quant_tbl_no);
	}
}

void vejpeg_write_SOS(void)
{
	int i;
	jpeg_component_info *c;
	unsigned short length;
	veavc_put_bits(16, 0xffda);

/*
	Here should be cur_num_components and cur_comp_info, but libjpeg doesn't
	initialize them when jpeg_write_tables is used.
*/
	length = 2 + 1 + 2 * jcs.num_components + 1 + 1 + 1;
	veavc_put_bits(16, length);

	veavc_put_bits( 8, jcs.num_components);
	for(i = 0; i < jcs.num_components; i++)
	{
		c = &jcs.comp_info[i];
		veavc_put_bits(8, c->component_id);
		veavc_put_bits(4, c->dc_tbl_no);
		veavc_put_bits(4, c->ac_tbl_no);
	}
	
	/* again not initialized, and are zero */
	veavc_put_bits(8, jcs.Ss);
	veavc_put_bits(8, jcs.lim_Se);
	veavc_put_bits(4, jcs.Al);
	veavc_put_bits(4, jcs.Ah);
}

void vejpeg_write_quantization(void)
{
	int qtnoY = jcs.comp_info[0].quant_tbl_no;
	int qtnoC = jcs.comp_info[1].quant_tbl_no;
	uint16_t *qtableY = jcs.quant_tbl_ptrs[qtnoY]->quantval;
	uint16_t *qtableC = jcs.quant_tbl_ptrs[qtnoC]->quantval;
	uint32_t biasY = 0x400 / qtableY[0];
	uint32_t biasC = 0x400 / qtableC[0];

	veavc_jpeg_parameters(1, 1, biasY, biasC);
	veavc_jpeg_quantization(qtableY, qtableC, DCTSIZE2);
}

void vejpeg_write_file(const char *filename, uint8_t *buffer, uint32_t length)
{
	FILE *fp;
	if(outlen > 2)
	{
		fp = fopen(filename, "w");
		/* don't write EOI */
		fwrite(outbuf, outlen-2, 1, fp);
		fwrite(buffer, length, 1, fp);
		/* write EOI */
		fwrite(outbuf+outlen-2, 2, 1, fp);
		fclose(fp);
	}
}


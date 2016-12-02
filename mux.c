/*
 * Copyright (c) 2016 Manuel Braga <mul.braga@gmail.com>
 * Copyright (c) 2016 Milos Ladicorbic <milos dot ladicorbic at gmail dot com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <libavformat/avformat.h>

#define ERET(...) do { fprintf(stderr, "mux: " __VA_ARGS__); exit(1); } while(0)

static AVFormatContext *formatContext = NULL;

void Mux_init(const char *outfile) {
                  /* Initializes libavformat and registers all the muxers, demuxers and protocols */
	av_register_all(); 

	formatContext = avformat_alloc_context();                        /* allocate avformat context */
	if (!formatContext) {
        ERET("alloc context, failed.\n");
    }
                                                                   /* set output container format */
	formatContext->oformat = av_guess_format("matroska", NULL, NULL);
	if (!formatContext->oformat) {
        ERET("guess format, failed.\n");
    }
                                                                           /* set output filename */
    snprintf(formatContext->filename, sizeof(formatContext->filename), "%s", outfile);

                                          /* Create and initialize a AVIOContext for writing only */
	if (avio_open(&formatContext->pb, formatContext->filename, AVIO_FLAG_WRITE) < 0) {
		ERET("io open, failed.\n");
    }


	AVStream *stream = NULL;
	stream = avformat_new_stream(formatContext, NULL);        /* Add a new stream to a media file */
	if (!stream) {
        ERET("new stream, failed.\n");
    }

    AVCodecContext *codecContext = stream->codec;
	codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
	codecContext->codec_id = CODEC_ID_H264;
	codecContext->width = 16;
	codecContext->height = 16;
	codecContext->bit_rate = 1000000;
	codecContext->time_base.num = 1000 / 25;
	codecContext->time_base.den = 1000;
	codecContext->pix_fmt = PIX_FMT_NV12;
	if(formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
		codecContext->flags |= CODEC_FLAG_GLOBAL_HEADER;
    }
	codecContext->flags |= CODEC_FLAG_BITEXACT;
}

void Mux_writeHeader(uint8_t *extradata, int extrasize) {
    
    int ret;
    
	if (!formatContext) {
        ERET("formatContext is NULL.\n");
    }

    AVCodecContext *codecContext = formatContext->streams[0]->codec;

	if (extradata) {
		codecContext->extradata = av_malloc(extrasize);
		memcpy(codecContext->extradata, extradata, extrasize);
		codecContext->extradata_size = extrasize;
	}

    /* Write the stream header, if any. */
	/*av_dump_format(formatContext, 0, formatContext->filename, 1);*/
	ret = avformat_write_header(formatContext, NULL);
    if (ret < 0) {
        //fprintf(stderr, "Error occurred when opening output file: %s\n",
                //av_err2str(ret));
        return;
    }
}

void Mux_writeData(uint8_t *data, int size, int keyframe) {
    
	if(!formatContext) ERET("formatContext is NULL.\n");

	AVPacket packet;
	av_init_packet(&packet);
	packet.data = data;
	packet.size = size;
    
	if (keyframe) {
        packet.flags |= AV_PKT_FLAG_KEY;
    }

	if (av_interleaved_write_frame(formatContext, &packet) < 0) {
		ERET("write frame, failed.\n");
    }
	
    av_free_packet(&packet);
}

void Mux_close(void) {
    
	if(!formatContext) ERET("formatContext is NULL.\n");

	av_write_trailer(formatContext);

	avio_close(formatContext->pb);
	avformat_free_context(formatContext);                                     /* free the context */
}


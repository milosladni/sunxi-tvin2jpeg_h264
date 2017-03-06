/*
 * Copyright (c) 2016 Milos Ladicorbic <milos dot ladicorbic at gmail dot com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
 * associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

#ifndef TVIN2JPEG_CFG_H_
#define TVIN2JPEG_CFG_H_

/**
* \file     appl_cfg.h
* \ingroup  g_applspec
* \brief    The configuration file for {{{tvin2jpeg}}.
* \author   Milos Ladicorbic
**/

/* Define software version */
#define FW_VERSION          "0.4.0"
/* Hardware version of the device. */
#define HW_VERSION          0x1

/* Display type */
//#define USE_HDMI
#define USE_LCD
/* Output picture resolution (this is not hw display resolution) */
/* defile display layers (DL) */
#define DISP_DL_RAW_SCN_WIDTH	542
#define DISP_DL_RAW_SCN_HEIGHT	359
#define DISP_DL_JPEG_SCN_WIDTH	200
#define DISP_DL_JPEG_SCN_HEIGHT	150
//#define DISP_DL_H264_SCN_WIDTH	200
//#define DISP_DL_H264_SCN_HEIGHT	150
#define DISP_DL_H264_SCN_WIDTH	542
#define DISP_DL_H264_SCN_HEIGHT	359
/* Picture position */
#define DISP_DL_RAW_SCN_POS_X  30
#define DISP_DL_RAW_SCN_POS_Y  121
#define DISP_DL_JPEG_SCN_POS_X  565
#define DISP_DL_JPEG_SCN_POS_Y  122
//#define DISP_DL_H264_SCN_POS_X  565
//#define DISP_DL_H264_SCN_POS_Y  300
#define DISP_DL_H264_SCN_POS_X  30
#define DISP_DL_H264_SCN_POS_Y  121

#endif                                                                        /* TVIN2JPEG_CFG_H_ */


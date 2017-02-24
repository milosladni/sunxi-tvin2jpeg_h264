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


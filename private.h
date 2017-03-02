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
 
#ifndef __PRIVATE_H__
#define __PRIVATE_H__
 
/* This is private for VC - VideoCodec */
#include <stdlib.h>
#include <stdint.h>

/**
 * \brief The set of all known YCbCr surface formats.
 */
typedef uint32_t VcYCbCrFormat;
#define INTERNAL_YCBCR_FORMAT (VcYCbCrFormat)0xffff

/* type definitions */
typedef enum {
    FALSE = 0,
    TRUE  = 1
} bool_t;

typedef enum {
  NAL_UNKNOWN = 0,
  NAL_SLICE = 1,
  NAL_SLICE_DPA = 2,
  NAL_SLICE_DPB = 3,
  NAL_SLICE_DPC = 4,
  NAL_SLICE_IDR = 5,
  NAL_SEI = 6,
  NAL_SPS = 7,
  NAL_PPS = 8,
  NAL_AU_DELIMITER = 9,
  NAL_SEQ_END = 10,
  NAL_STREAM_END = 11,
  NAL_FILTER_DATA = 12
} NalUnitType;

#define H264_IS_P_SLICE(type)  ((type % 5) == H264_P_SLICE)
#define H264_IS_B_SLICE(type)  ((type % 5) == H264_B_SLICE)
#define H264_IS_I_SLICE(type)  ((type % 5) == H264_I_SLICE)
#define H264_IS_SP_SLICE(type) ((type % 5) == H264_SP_SLICE)
#define H264_IS_SI_SLICE(type) ((type % 5) == H264_SI_SLICE)

#define PIC_TOP_FIELD		0x1
#define PIC_BOTTOM_FIELD	0x2
#define PIC_FRAME		    0x3

#define PIC_TYPE_FRAME	    0x0
#define PIC_TYPE_FIELD	    0x1
#define PIC_TYPE_MBAFF	    0x2

#define NUM_REF_FRAMES      16                     /* 16 is the max allowed by the H.264 standard */

 /**
 * \brief The set of all known compressed video formats, and
 *        associated profiles, that may be decoded.
 */
typedef uint32_t decoderProfile_t;
/* profile IDC */
/** \brief MPEG 4 part 10 == H.264 == AVC */
#define ENCODER_DECODER_PROFILE_H264_BASELINE           ((decoderProfile_t)66)
#define ENCODER_DECODER_PROFILE_H264_MAIN               ((decoderProfile_t)77)
#define ENCODER_DECODER_PROFILE_H264_HIGH               ((decoderProfile_t)100)

/* level IDC */
#define ENCODER_DECODER_LEVEL_H264_1     10
#define ENCODER_DECODER_LEVEL_H264_1b    9
#define ENCODER_DECODER_LEVEL_H264_1_1   11
#define ENCODER_DECODER_LEVEL_H264_1_2   12
#define ENCODER_DECODER_LEVEL_H264_1_3   13
#define ENCODER_DECODER_LEVEL_H264_2     20
#define ENCODER_DECODER_LEVEL_H264_2_1   21
#define ENCODER_DECODER_LEVEL_H264_2_2   22
#define ENCODER_DECODER_LEVEL_H264_3     30
#define ENCODER_DECODER_LEVEL_H264_3_1   31
#define ENCODER_DECODER_LEVEL_H264_3_2   32
#define ENCODER_DECODER_LEVEL_H264_4     40
#define ENCODER_DECODER_LEVEL_H264_4_1   41
#define ENCODER_DECODER_LEVEL_H264_4_2   42
#define ENCODER_DECODER_LEVEL_H264_5     50
#define ENCODER_DECODER_LEVEL_H264_5_1   51

/* VE max resolution */
//#define VE_MAX_WIDTH                3840                                             /* max width */
//#define VE_MAX_HEIGHT               2160                                            /* max height */
//if width is bigger then 2048 h264 engine needs 2 extra buffers
#define VE_MAX_WIDTH                UL1920                                           /* max width */
#define VE_MAX_HEIGHT               UL1080                                          /* max height */
#define VE_MAX_MACROBLOCKS UL8100 //((ENCODER_DECODER_MAX_WIDTH * ENCODER_DECODER_MAX_HEIGHT) / (16 * 16)) /* Max macroblocks */

/**
 * \brief An invalid object handle value.
 *
 * This value may be used to represent an invalid, or
 * non-existent, object (\ref VcDevice "VcDevice",
 * \ref VcVideoSurface "VcVideoSurface", etc.)
 *
 * Note that most APIs require valid object handles in all
 * cases, and will fail when presented with this value.
 */
#define VC_INVALID_HANDLE 0xffffffffU

/**
 * \brief The set of all chroma formats for \ref VideoSurface
 */
typedef enum ChromaTypeTag {
    CHROMA_TYPE_420 = 0,                                                  /* 4:2:0 chroma format. */
    CHROMA_TYPE_422,                                                      /* 4:2:2 chroma format. */
    CHROMA_TYPE_444                                                       /* 4:4:4 chroma format. */
} ChromaType_t;

/**
 * \brief The set of all known YCbCr surface formats.
 */
typedef enum YCbCrFormatTag {
    YCBCR_FORMAT_NV12 = 0,
    YCBCR_FORMAT_YV12,
    YCBCR_FORMAT_UYVY,
    YCBCR_FORMAT_YUYV,
    YCBCR_FORMAT_Y8U8V8A8,
    YCBCR_FORMAT_V8U8Y8A8
} YCbCrFormat_t;

/**
 * \brief  The set of all known RGB surface formats.
 */
typedef enum YRGBAFormatTag {
    RGBA_FORMAT_B8G8R8A8 = 0,
    RGBA_FORMAT_R8G8B8A8,
    RGBA_FORMAT_R10G10B10A2,
    RGBA_FORMAT_B10G10R10A2,
    RGBA_FORMAT_A8
} RGBAFormat_t;

typedef struct VideoSurfaceTag {
	uint32_t        width, height;
	ChromaType_t    chromaType;                                                 /* 422 420 444... */
	YCbCrFormat_t   sourceFormat;                                            /* NV12 YV12 UYVY... */
	void           *data;                                                  /* output decoded data */
    uint32_t        size;                                                    /* decoded data size */
	int             luma_size;                                 /* luma plane size of decoded data */
	void           *extra_data;
    //int             extra_data_len;
	uint8_t         pos;
    uint8_t         pic_type;
} VideoSurface_t;

typedef uint32_t VideoSurface;
typedef uint32_t VcHandle;

 /**
 * \brief Information about an H.264 reference frame
 *
 * Note: References to bitstream fields below may refer to data literally parsed
 * from the bitstream, or derived from the bitstream using a mechanism described
 * in the specification.
 */
typedef struct ReferenceFrameH264Tag {
    /**
     * The surface that contains the reference image.
     * Set to VDP_INVALID_HANDLE for unused entries.
     */
    VideoSurface    surface;                                                                      //TODO videti da li i kako ovo implementirati.. mozda pomocu liste ako je neki dinamicki bafer
    /** Is this a long term reference (else short term). */
    bool_t          is_long_term;
    /**
     * Is the top field used as a reference.
     * Set to VDP_FALSE for unused entries.
     */
    bool_t          top_is_reference;
    /**
     * Is the bottom field used as a reference.
     * Set to VDP_FALSE for unused entries.
     */
    bool_t          bottom_is_reference;
    /** [0]: top, [1]: bottom */
    int32_t         field_order_cnt[2];
    /**
     * Copy of the H.264 bitstream field:
     * frame_num from slice_header for short-term references,
     * LongTermPicNum from decoding algorithm for long-term references.
     */
    uint16_t        frame_idx;
} ReferenceFrameH264_t;

typedef struct PictureInfoH264Tag {
    /** Number of slices in the bitstream provided. */
    uint32_t slice_count;
    /** [0]: top, [1]: bottom */
    int32_t  field_order_cnt[2];
    /** Will the decoded frame be used as a reference later. */
    bool_t   is_reference;
    
    /** \name H.264 bitstream
     *
     * Copies of the H.264 bitstream fields.
     * @{ */
    uint16_t frame_num;                         //from slice
    uint8_t  field_pic_flag;                    //slice
    uint8_t  bottom_field_flag;                 //slice
    uint8_t  num_ref_frames;                //
    uint8_t  mb_adaptive_frame_field_flag;  //
    uint8_t  constrained_intra_pred_flag;   //
    uint8_t  weighted_pred_flag;            //
    uint8_t  weighted_bipred_idc;           //
    uint8_t  frame_mbs_only_flag;           //
    uint8_t  transform_8x8_mode_flag;       //
    int8_t   chroma_qp_index_offset;        //                            /* chroma qp index offset */
    int8_t   second_chroma_qp_index_offset; //                     /* second chroma qp index offset */
    int8_t   pic_init_qp_minus26;           //
    uint8_t  num_ref_idx_l0_active_minus1;              //from slice
    uint8_t  num_ref_idx_l1_active_minus1;              //slice
    uint8_t  log2_max_frame_num_minus4;     //
    uint8_t  pic_order_cnt_type;            //
    uint8_t  log2_max_pic_order_cnt_lsb_minus4;         //
    uint8_t  delta_pic_order_always_zero_flag;          //
    uint8_t  direct_8x8_inference_flag;                 //
    uint8_t  entropy_coding_mode_flag;                  //
    uint8_t  pic_order_present_flag;                    //
    uint8_t  deblocking_filter_control_present_flag;    //
    uint8_t  redundant_pic_cnt_present_flag;            //
    /** Convert to raster order. */                     //
    uint8_t scaling_lists_4x4[6][16];                   //
    /** Convert to raster order. */                     //
    uint8_t scaling_lists_8x8[2][64];                   //
    /** @} */

    /** See \ref VdpPictureInfoH264 for instructions regarding this field. */
    ReferenceFrameH264_t referenceFrames[NUM_REF_FRAMES];
} PictureInfoH264_t;

typedef struct h264PictureTag {
	VideoSurface_t *surface;
	uint16_t top_pic_order_cnt;
	uint16_t bottom_pic_order_cnt;
	uint16_t frame_idx;
    uint8_t  field;
} h264Picture_t;


typedef enum SliceTypeTag {
    SLICE_TYPE_P = 0,   //0
    SLICE_TYPE_B,       //1
    SLICE_TYPE_I,       //2
    SLICE_TYPE_SP,      //3
    SLICE_TYPE_SI       //4
} SliceType_t;

typedef struct h264HeaderTag {
	uint8_t nal_unit_type;
	uint16_t first_mb_in_slice;
	uint8_t slice_type;
	uint8_t pic_parameter_set_id;
	uint16_t frame_num;
	uint8_t field_pic_flag;
	uint8_t bottom_field_flag;
	uint16_t idr_pic_id;
	uint32_t pic_order_cnt_lsb;
	int32_t delta_pic_order_cnt_bottom;
	int32_t delta_pic_order_cnt[2];
	uint8_t redundant_pic_cnt;
	uint8_t direct_spatial_mv_pred_flag;
	uint8_t num_ref_idx_active_override_flag;
	uint8_t num_ref_idx_l0_active_minus1;
	uint8_t num_ref_idx_l1_active_minus1;
	uint8_t cabac_init_idc;
	int8_t slice_qp_delta;
	uint8_t sp_for_switch_flag;
	int8_t slice_qs_delta;
	uint8_t disable_deblocking_filter_idc;
	int8_t slice_alpha_c0_offset_div2;
	int8_t slice_beta_offset_div2;

	uint8_t luma_log2_weight_denom;
	uint8_t chroma_log2_weight_denom;
	int8_t luma_weight_l0[32];
	int8_t luma_offset_l0[32];
	int8_t chroma_weight_l0[32][2];
	int8_t chroma_offset_l0[32][2];
	int8_t luma_weight_l1[32];
	int8_t luma_offset_l1[32];
	int8_t chroma_weight_l1[32][2];
	int8_t chroma_offset_l1[32][2];

	h264Picture_t RefPicList0[32];
	h264Picture_t RefPicList1[32];
} h264Header_t;

typedef struct h264ContextTag {
    void           *regs;
	h264Header_t    header;
	PictureInfoH264_t const *info;
	VideoSurface_t *output;
	uint8_t picture_width_in_mbs_minus1;
	uint8_t picture_height_in_mbs_minus1;
    
    uint8_t default_scaling_lists;
    int     video_extra_data_len;
    
	int ref_count;
	h264Picture_t           ref_pic[NUM_REF_FRAMES];
} H264Context_t;

typedef struct outputSurfaceTag {
	//device_ctx_t *device;
	RGBAFormat_t    rgba_format;
	uint32_t        width, height;
	VideoSurface_t *vs;
	uint32_t        video_x, video_y, video_width, video_height;
} outputSurface_t;
 
 
/**
 * \brief The set of all possible error codes.
 */
typedef enum {
    /** The operation completed successfully; no error. */
    VC_STATUS_OK = 0,
    /**
     * An invalid handle value was provided.
     *
     * Either the handle does not exist at all, or refers to an object of an
     * incorrect type.
     */
    VC_STATUS_INVALID_HANDLE,
    /**
     * An invalid pointer was provided.
     *
     * Typically, this means that a NULL pointer was provided for an "output"
     * parameter.
     */
    VC_STATUS_INVALID_POINTER,
    /**
     * An invalid/unsupported \ref ChromaType value was supplied.
     */
    VC_STATUS_INVALID_CHROMA_TYPE,
    /**
     * An invalid/unsupported \ref VcYCbCrFormat value was supplied.
     */
    VC_STATUS_INVALID_Y_CB_CR_FORMAT,
    /**
     * An invalid/unsupported \ref ColorStandard value was supplied.
     */
    VC_STATUS_INVALID_COLOR_STANDARD,
    /**
     * An invalid/unsupported \ref ColorTableFormat value was supplied.
     */
    VC_STATUS_INVALID_COLOR_TABLE_FORMAT,
    /**
     * An invalid/unsupported flag value/combination was supplied.
     */
    VC_STATUS_INVALID_FLAG,
    /**
     * An invalid/unsupported \ref FuncId value was supplied.
     */
    VC_STATUS_INVALID_FUNC_ID,
    /**
     * The size of a supplied object does not match the object it is being
     * used with.
     */
    VC_STATUS_INVALID_SIZE,
    /**
     * An invalid/unsupported value was supplied.
     *
     * This is a catch-all error code for values of type other than those
     * with a specific error code.
     */
    VC_STATUS_INVALID_VALUE,
    /**
     * The system does not have enough resources to complete the requested
     * operation at this time.
     */
    VC_STATUS_RESOURCES,
    /**
     * A catch-all error, used when no other error code applies.
     */
    VC_STATUS_ERROR,
} VcStatus;

typedef struct {
    /**
     * This field must be filled with VC_BITSTREAM_BUFFER_VERSION
     */
    uint32_t     struct_version;
    /** A pointer to the bitstream data bytes */
    void const * bitstream;
    /** The number of data bytes */
    uint32_t     bitstream_bytes;
} BitstreamBuffer;

 
//handles
typedef uint32_t VcHandle;
void *Handle_create(size_t size, VcHandle *handle);
void *Handle_get(VcHandle handle);
void Handle_destroy(VcHandle handle);

//video surface
VcStatus VideoSurface_create(ChromaType_t chromaType, uint32_t width, uint32_t height, VideoSurface *pSurface);
VcStatus VideoSurface_destroy(VideoSurface surface);
VcStatus VideoSurface_getParameters(VideoSurface surface, ChromaType_t *chromaType, uint32_t *width, uint32_t *height);
VcStatus VideoSurface_getBits_yCbCr(VideoSurface surface, YCbCrFormat_t destinationYcbcrFormat, void *const *destinationData, uint32_t const *destinationPitches);
VcStatus VideoSurface_putBits_yCbCr(VideoSurface surface, YCbCrFormat_t sourceYcbcrFormat, void const *const *sourceData, uint32_t const *sourcePitches);
VcStatus VideoSurface_queryCaps(ChromaType_t surfaceChromaType, bool_t *isSupported, uint32_t *maxWidth, uint32_t *maxHeight);
VcStatus VideoSurface_queryGetPutBits_yCbCrCaps(ChromaType_t surfaceChromaType, YCbCrFormat_t bits_yCbCrFormat, bool_t *isSupported);

#endif                                                                           /* __PRIVATE_H__ */
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

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
 
/**
* \file     tvin2jpeg.c
* \ingroup  g_applspec
* \brief    Implementation of the tvin2jpeg class.
* \author   Milos Ladicorbic
*/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include <getopt.h>            
#include <fcntl.h>             
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>         
#include <linux/videodev2.h>
#include <time.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <stdint.h>
#include <pthread.h>                                                           /* pthread_mutex_t */
#include <sys/epoll.h>
#include <sys/prctl.h>
//
#include "sunxi_disp_ioctl.h"
#include <sys/time.h>
#include <turbojpeg.h>
#include "debug.h"
#include "tvin2jpeg_cfg.h"
#include "mux.h"
//
#include "vepoc.h"
#include "jpeg.h"
#include "private.h"
#include "gsth264parser.h"

#define DRAM_OFFSET (0x40000000)
#define VLE_SIZE (1 * 1024 * 1024)
#define VBV_SIZE (4 * 1024 * 1024)
#define NALU_BUFFER_LENGTH  4194304                                                           //2`22 //1024*1024*4

#define MAX_FRAMES          25
#define MAX_REFERENCES      NUM_REF_FRAMES
#define OUTPUT_SURFACES     2
#define NUM_EXTRA_DPB_BUFF  6
#define MAX_DPB_SIZE        MAX_REFERENCES + NUM_EXTRA_DPB_BUFF + 1

#define H264DEC_MAX_WIDTH   3840UL
#define H264DEC_MAX_HEIGHT  2160UL
#define H264DEC_MX_MACROBLOCKS ((H264DEC_MAX_WIDTH * H264DEC_MAX_HEIGHT) / (16 * 16))

#define QUEUED_FOR_DISPLAY 2
#define QUEUED_FOR_REFERENCE 1
#define NOT_QUEUED 0

#define CAMERA_DEVICE       "/dev/video1"
#define DISPLAY_DEVICE      "/dev/disp"
#define MAX_RESOLUTION      4096
#define JPEG_ENC_QUALITY    100
#define N_BUFFERS           5
#define MAX_BUFFERS         10
#define PADDING             2

#define ARSIZE(x) (sizeof(x) / sizeof((x)[0]))

#if (N_BUFFERS < 2) || (N_BUFFERS > MAX_BUFFERS)
#error N_BUFFERS must be in range 2<->MAX_BUFFERS!
#endif

/* Display type */
#ifdef USE_HDMI
#define DISPLAY_ON  DISP_CMD_HDMI_ON
#define DISPLAY_OFF DISP_CMD_HDMI_OFF
#elif defined(USE_LCD)
#define DISPLAY_ON  DISP_CMD_LCD_ON
#define DISPLAY_OFF DISP_CMD_LCD_OFF
#else
#error Display type must be defined!
#endif

#define MAX_JPEG_SIZE       0x800000                            /* max buffer size for jpeg image */

#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define PAD(v, p) ((v+(p)-1)&(~((p)-1)))
#define UNUSED_ARGUMENT(x_) (void)(x_)

#define NUM_REFERENT_PIC    2                                      /* number of referent pictures */

#define THREAD_FUNCTION_RETURN_TYPE  void*
#define THREAD_FUNCTION_ATTRIBUTE
#define THREAD_FUNCTION_RETURN_VALUE  (void*)0

const char *subNameLong[TJ_NUMSAMP] = { "4:4:4", "4:2:2", "4:2:0", "GRAY", "4:4:0", "4:1:1" };
const char *subName[TJ_NUMSAMP] = {"444", "422", "420", "GRAY", "440", "411"};
const char *pixFormatStr[TJ_NUMPF]= {
	"RGB", "BGR", "RGBX", "BGRX", "XBGR", "XRGB", "Grayscale",
	"RGBA", "BGRA", "ABGR", "ARGB", "CMYK"
};

typedef enum TypeTag {                                         /* do task by hardware or software */
    SW = 1,                                             /* software croper, scaler, encoder etc.. */
    HW                                                  /* hardware croper, scaler, encoder etc.. */
} Type_t;

typedef THREAD_FUNCTION_RETURN_TYPE (THREAD_FUNCTION_ATTRIBUTE *threadFunction_t)(void*);

typedef struct ThreadTag {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    pthread_t       id;
    bool_t          running;
    bool_t          process;
    bool_t          finished;
} Thread_t;


typedef enum dpb_reference_valueTag {
    DPB_UNUSED_FOR_REFERENCE            = 0,
    DPB_USED_FOR_SHORT_TERM_REFERENCE   = 1,
    DPB_USED_FOR_LONG_TERM_REFERENCE    = 2,
} dpb_reference_value;

/* video buffers */
typedef struct BufferTag {
        void   *start;
        size_t  length;
        bool_t  queued;
} Buffer_t;

typedef struct SizeTag {
	int width;
	int height;
} Size_t;

typedef enum VeSubengTypeTag {
    SUBENG_JPEG_ENC,
    SUBENG_JPEG_DEC,
    SUBENG_H264_ENC,
    SUBENG_H264_DEC
} VeSubengType_t;

static char *subengNames[4] = {"JPEG_ENC", "JPEG_DEC", "H264_ENC", "H264_DEC"};

typedef enum scalerTypeTag {
    ARBITRARY_VGA = 1,
    ARBITRARY_QVGA,
} scalerType_t;

typedef struct frameTag {
    const void *pFrame;
    int         frameSize;
    uint32_t    index;
} frame_t;

typedef struct CameraTag {
    char           *deviceName;                                              /* video device name */
    int             fd;                                   /* video driver source fd (/dev/videoX) */
    Size_t          size;                                                          /* source size */
    int             format;
    uint32_t        YplaneSize;
    uint32_t        CplaneSize;
    unsigned int    bytesperline;                                               /* bytes per line */
    long unsigned int   rawSize;                                                /* raw image size */
    uint32_t    mb_width;                                           /* width in 16x16 macroblocks */
    uint32_t    mb_height;                                         /* height in 16x16 macroblocks */
    uint32_t    mb_stride;                                         /* stride in 16x16 macroblocks */
    uint32_t    crop_right;
    uint32_t    crop_bottom;
    /* buffers */
    Buffer_t            buffers[MAX_BUFFERS];                                   /* camera buffers */
    unsigned int        n_buffers;                                 /* number of allocated buffers */
} Camera_t;

enum DysplayLayerTypeTag {
    DL_RAW = 0,                                        /* display layer for raw frame from camera */
    DL_JPEG,                                            /* display layer for jpeg decoded picture */
    DL_H264,                                             /* display layer for decoded h264 stream */
    DL_MAX
};

typedef struct DisplayLayerTag {
    bool_t              initialized;                              /* display layer is initialized */
    unsigned int        layerId;                                               /* diplay layer id */
    __disp_layer_info_t layerPara;                                    /* display layer parameters */
    uint32_t            lastId;                                                  /* last frame id */
} DisplayLayer_t;

typedef struct DisplayTag {
    int                 fd;                                      /* display driver fd (/dev/disp) */
    int                 fb_fd;
    int                 sel;                                                  /* which screen 0/1 */
    DisplayLayer_t      layer[DL_MAX];      /* diaplay layers, raw camera, jpeg dec, h264 dec ... */
    //__disp_video_fb_t   videoFb;                                   /* display video framebuffer */
    //__disp_pixel_fmt_t  format;                                         /* display pixel format *///DISP_FORMAT_YUV420
    //__disp_pixel_mod_t  mode;                                             /* display pixel mode *///DISP_MOD_NON_MB_UV_COMBINED
    //__disp_pixel_seq_t	seq;                                             /* display pixel seq *///DISP_SEQ_UVUV yuv420
} Display_t;

typedef struct TurbojpegTag {   /* sw jpeg enc param */
    tjhandle            jpegCompressor;
    long unsigned int   jpegSize;
    unsigned char      *pCompressedImage; /*! Memory is allocated by tjCompress2 if jpegSize == 0 */
} Turbojpeg_t;

typedef struct HwJpegTag {  /* hw jpeg enc param */
    /* buffers */
    uint8_t     *JpegBuff;                                        /* compressed jpeg image buffer */
    uint32_t     Jwritten;                                                  /* size of jpeg image */
} HwJpeg_t;

typedef struct JpegEncTag { /* jpeg enc param */
    bool_t              encode;
    Type_t              type;                                         /* encode frame by sw or hw */
    int                 jpegEncQuality;                                /* encoder picture quality */
    uint32_t            id;
    Turbojpeg_t         tj;                                                    /* turbojpeg param */
    HwJpeg_t            hwj;
} JpegEnc_t;

typedef struct ScalerTag {                                                        /* scaler param */
    scalerType_t    type;                           /* arbitrary-scaler to VGA or QVGA resolution */
    Size_t          size;
    uint64_t        rawSize;                                                        /* raw image size */
    uint32_t        YplaneSize;
    uint32_t        CplaneSize;
    float           xScaleFactor;
    float           yScaleFactor;
    
    uint32_t    mb_width;                                           /* width in 16x16 macroblocks */
    uint32_t    mb_height;                                         /* height in 16x16 macroblocks */
    uint32_t    mb_stride;                                         /* stride in 16x16 macroblocks */
    uint32_t    crop_right;
    uint32_t    crop_bottom;
} Scaler_t;

typedef enum entropyCodingTag {
    H264_EC_CAVLC = 0,
    H264_EC_CABAC = 1
} entropyCoding_t;

typedef enum sliceTypeTag {
    SLICE_P = 0,
    SLICE_I = 2
} sliceType_t;

typedef struct h264encRefPicTag {
    void *luma_buffer;
    void *chroma_buffer;
    void *extra_buffer; /* unknown purpose, looks like smaller luma */
} h264encRefPicture_t;

typedef struct H264encParamTag {
    bool_t          encode;
    int             file;
    uint32_t        id;
    
	unsigned int    profileIdc;
    unsigned int    levelIdc;
    unsigned int    constraints;
	entropyCoding_t ecMode;                                                      /* entropyCoding */
	unsigned int    qp;
	unsigned int    keyframeInterval;
    //bytestreams
    uint8_t        *pBytestreamBuffer;
	unsigned int    bytestreamBufferSize;
	unsigned int    bytestreamLength;
    
    h264encRefPicture_t refPic[NUM_REFERENT_PIC];                            /* referent pictures */
    void *extra_buffer_line, *extra_buffer_frame, *extra_motion_est;           /* unknown purpose */
    
    bool_t          spsPps;                      /* sequence parameter set, picture parameter set */
    unsigned int    frameNum;                                             /* current frame number */
	sliceType_t     sliceType;                                     /* current slice type (I or P) */
} H264enc_t;

typedef struct VeispTag {
    veisp_color_format_t   colorFormat;                                           /* color format */
    uint8_t    *mb_info_buf;
    Scaler_t    scaler;                                                              /* ve scaler */
} Veisp_t;

typedef struct JpegDecTag {
    bool_t          decode;
    int             file;
    uint32_t        id;
    /* hw buffers */
    uint8_t        *input_buffer;
    uint8_t        *luma_buffer;
	uint8_t        *chroma_buffer;
    /* debug */
    double          maxTimeForOneFrame;
    double          minTimeForOneFrame;
} JpegDec_t;

typedef struct GstNalParserTag {
    GstH264NalParser   *parser;
    GstH264NalUnit     *nalu;
    GstH264SliceHdr    *slice;
    GstH264SPS         *sps;
    GstH264PPS         *pps;
    GstH264SEIMessage  *sei;
} GstNalParser_t;

typedef struct H264DpbTag H264Dpb_t;

typedef struct H264FrameTag {
    VideoSurface    surface;
    GstH264SliceHdr slice_hdr;

    uint32_t  poc;
    uint32_t  frame_idx;
    bool_t    is_reference;
    bool_t    is_long_term;
    bool_t    output_needed;
    uint32_t  id;
} H264Frame_t;

typedef GstFlowReturn (*H264DPBOutputFunc) (H264Dpb_t *dpb, H264Frame_t *h264_frame, void *user_data);

struct H264DpbTag {
    H264Frame_t *frames[MAX_REFERENCES];
    uint32_t    n_frames;
    uint32_t    max_frames;
    int32_t     max_longterm_frame_idx;
    H264DPBOutputFunc output;
    void       *user_data;
    //
    H264Frame_t     scratch_frames[MAX_DPB_SIZE];
    uint32_t        lastUsedDpbId;
};

typedef struct H264DecTag {
    bool_t          decode;
    uint32_t        id;
    char           *fileName;
    int             fileId;
    uint8_t        *pFileData;
    uint32_t        fileOffset;
    struct stat     s;
    //source parameters
    uint32_t        width, height;                                            /* video dimensions */
    uint32_t        framerate;                                                       /* framerate */
    //
    H264Dpb_t       dpb;                                                /* decoded picture buffer */
    GstNalParser_t  gst;
    bool_t          got_idr;
    uint32_t        poc_msb;
    uint32_t        prev_poc_lsb;
    int32_t         fps_n, fps_d, par_n, par_d;                                 /* vui parameters */
    /* hw buffers */
    void           *data;  /* VBV data (Video Buffering Verifier). Input buffer which contains input bitestream (one instance of NAL) */
    uint32_t        dataLen;                    /* Size of input bitestream (one instance of NAL) */
    void           *extra_data;                                           /* extra working buffer */
    int             extra_data_size;
    /* output surfaces */
    VideoSurface    output[OUTPUT_SURFACES];
    int             video_extra_data_len;
    /* debug */
    double          maxTimeForOneFrame;
    double          minTimeForOneFrame;
} H264dec_t;

typedef struct VeTag {                                                            /* video engine */
    pthread_mutex_t mutex;                                   /* critical section for hw resources */
    void           *pRegs;                           /* pointer to virtual mapped cedar registers */
    /* input buffers */
	uint8_t        *pLumaSrc;                                                /* input luma buffer */
	uint8_t        *pChromaSrc;                                            /* input chroma buffer */
    /* veisp */
    Veisp_t         isp;
    /* encoders */
    H264enc_t       h264enc;                                                      /* h264 encoder */
    /* decoders */
    JpegDec_t       jpegdec;                                                      /* jpeg decoder */
    H264dec_t       h264dec;                                                      /* h264 decoder */
} Ve_t;

typedef struct ApplTag {
    pthread_mutex_t     mutex;
    long signed int     frameCount;                                             /* frames to read */
    long unsigned int   compressCount;                                      /* frames to compress */
    long unsigned int   rawCount;                                           /* raw frames to save */
    bool_t              output_marks;
    bool_t              preview;
    bool_t              readFile;
    char               *pFileName;
    bool_t              yuvToRgb;
    //
    frame_t             frame;
    Camera_t            camera;                                                   /* camera param */
    Display_t           disp;                                               /* display parameters */
    JpegEnc_t           jpegEnc;                                           /* jpeg encoder params */
    //HW VE - Video Engine
    Ve_t                ve;                                                       /* video engine */
    //thread - critical section
    Thread_t            threadEncoder;                               /* thread for encoding frame */
    Thread_t            threadJpegDecoder;                      /* thread for decoding JPEG image */
    Thread_t            threadH264Decoder;                      /* thread for decoding H264 image */
    //other
    bool_t              run;
    double              maxTimeForOneFrame;
    double              minTimeForOneFrame;
    
} Appl_t;

static Appl_t       l_appl;
static Appl_t      *pThis = &l_appl;

/* forward declarations */
//pthread
static void CriticalSectionCreate__(pthread_mutex_t *mutex);
static void CriticalSectionEnter__(pthread_mutex_t *mutex);
static void CriticalSectionExit__(pthread_mutex_t *mutex);
static void CriticalSectionDestroy__(pthread_mutex_t *mutex);
static void ConditionVariable_create__(pthread_cond_t  *cond);
static void ConditionVariable_wait__(pthread_cond_t  *cond, pthread_mutex_t *mutex);
static void ConditionVariable_signal__(pthread_cond_t  *cond);
static void ConditionVariable_destroy__(pthread_cond_t  *cond);
static void Thread_create__(Thread_t *pThread, threadFunction_t pThreadFunction, void *pThreadParameter);
static void Thread_destroy__(void);
THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadEncodeFrame__(void *pArgument);
THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadDecodeJpegFrame__(void *pArgument);
THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadDecodeH264Frame__(void *pArgument);
//Tvin2jpeg
static int Tvin2jpeg_readFrame__(void);
static void Tvin2jpeg_processImage__(const void *p, int picSize);
static char* Tvin2Jpeg_readFile__(char *pName, uint32_t *readSize);
static void Tvin2Jpeg_handleInt__(int n);
static void Tvin2jpeg_initVars__(void);
//camera
static void Camera_init__(void);
static void Camera_close__(void);
static void Camera_mapBuffers__(void);
static void Camera_unmapBuffers__(void);
static void Camera_streamOn__(void);
static void Camera_streamOff__(void);
static void Camera_queryFrame__(uint32_t bufIdx);
//display
static int Disp_init__(DisplayLayer_t *pLayer, __disp_pixel_fmt_t format, __disp_pixel_mod_t  mode, __disp_pixel_seq_t	seq, uint32_t srcWidth, uint32_t srcHeight, uint32_t outX, uint32_t outY, uint32_t outWidth, const int outHeight);
static void Disp_start__(DisplayLayer_t *pLayer);
static void Disp_stop__(DisplayLayer_t *pLayer);
static int Disp_on__(void);
static int Disp_layerRelease__(DisplayLayer_t *pLayer);
static int Disp_exit__(void);
static int Disp_set_addr__(DisplayLayer_t *pLayer, int w, int h, int *addr);
static int Disp_newDecoder_frame__(DisplayLayer_t *pLayer, int w, int h, const uint32_t luma_buffer, 
                                    const uint32_t chroma_buffer, const int id);
//libturbojpeg
static void Tj_init__(void);
static void Tj_close__(void);
//jpeg enc
static void JpegEnc_encodePicture__(const void *pFrame, int frameSize);
//hw VE (VideoEngine)
static void Ve_init__(void);
static void Ve_allocInputBuffers__(void);
static void Ve_allocOutputBuffers__(void);
static void Ve_initScalerBuffers__(void);
static void Ve_fillInputBuffers__(const void *pFrame, int frameSize);
static void Ve_selectSubengine__(Appl_t *pThis, VeSubengType_t subengine);
static void Ve_releaseSubengine__(Appl_t *pThis, VeSubengType_t subengine);
static void Ve_trigerSubebgine__(VeSubengType_t subengine);
static void Ve_veisp_setInputBuffers__(uint8_t *Y, uint8_t *C);
static void Ve_veisp_initPicture__(uint32_t width_mb, uint32_t height_mb, uint32_t stride_mb, veisp_color_format_t format);
static void Ve_freeInputBuffers__(void);
static void Ve_freeOutputBuffers__(void);
static void Ve_freeMbInfoBuffer__(void);
static void Ve_free__(void);
//hw h264enc
static void H264enc_init__(void);
static void H264enc_new__(void);
static void H264enc_free__(void);
static int H264enc_encodePicture__(void);
static void put_bits__(uint32_t x, int num);
static void put_ue__(uint32_t x);
static void put_se__(int x);
static void put_start_code__(unsigned int nal_ref_idc, unsigned int nal_unit_type);
static void put_rbsp_trailing_bits__(void);
static void put_seq_parameter_set__(void);
static void put_pic_parameter_set__(void);
static void put_slice_header__(void);
//hw jpeg decoder
static void JpegDec_init__(void);
static void JpegDec_decode__(struct jpeg_t *jpeg);
static void JpegDec_setFormat__(struct jpeg_t *jpeg);
static void JpegDec_setSize__(struct jpeg_t *jpeg);
static void JpegDec_setQuantizationTables__(struct jpeg_t *jpeg);
static void JpegDec_setHuffmanTables__(struct jpeg_t *jpeg);
//hw h264 decoder
static void H264dec_init__(void);
static void H264dec_new__(H264dec_t *pH264dec);
static void H264dec_free__(void);
static VcStatus H264dec_decode__(Appl_t *pThis, PictureInfoH264_t const *info, const int len, VideoSurface_t *output);
static int H264dec_findStartcode__(const uint8_t *data, int len, int start);
static int H264dec_fillFrameLists__(Appl_t *pThis, H264Context_t *context);
static int H264dec_picOrderCnt__(const h264Picture_t *pic);
static int H264dec_sortRefPicsByPOC__(const void *p1, const void *p2);
static int H264dec_sortRefPicsByFrameNum__(const void *p1, const void *p2);
static void H264dec_splitRefFields__(h264Picture_t *out, h264Picture_t **in, int len, int cur_field);
static void H264dec_fillDefaultRefPicList__(H264Context_t *context);
static void H264dec_decodeSliceHeader__(H264Context_t *context);
static void* H264dec_getSurfacePriv__(H264Context_t *context, VideoSurface_t *surface);
static void H264dec_refPicListModification__(H264Context_t *context);
static void H264dec_predWeightTable__(H264Context_t *context);
static void H264dec_decRefPicMarking__(H264Context_t *context);
static uint32_t get_u__(void *regs, int num);
static uint32_t get_ue__(void *regs);
static int32_t get_se__(void *regs);
static int H264dec_handleSps__(Appl_t *pThis);
static int H264dec_handlePps__(Appl_t *pThis);
static int H264dec_handleSei__(Appl_t *pThis);
static int H264dec_idr__(H264dec_t *h264Dec, H264Frame_t *h264Frame);
static uint32_t H264dec_calculatePoc__(H264dec_t *h264dec, GstH264SliceHdr *slice);
static int H264dec_checkScalingLists__(H264Context_t *context);
//h264dec DBP
static void H264dec_dpb_fillReferenceFrames__(H264Dpb_t *dpb, ReferenceFrameH264_t reference_frames[16]);
static void H264dec_dpb_remove__(H264Dpb_t *dpb, uint32_t idx);
static GstFlowReturn H264dec_dpb_output__(H264Dpb_t *dpb, uint32_t idx);
static bool_t H264dec_dpb_bump__(H264Dpb_t *dpb, uint32_t poc, GstFlowReturn *ret);
static GstFlowReturn H264dec_dpb_add__(H264Dpb_t *dpb, H264Frame_t *h264_frame);
static void H264dec_dpb_flush__(H264Dpb_t *dpb, bool_t output);
static void H264dec_dpb_markSliding__(H264Dpb_t *dpb);
static void H264dec_dpb_markLongTerm__(H264Dpb_t *dpb, uint32_t pic_num, uint32_t long_term_frame_idx);
static void H264dec_dpb_markShortTermUnused__(H264Dpb_t *dpb, uint32_t pic_num);
static void H264dec_dpb_markLongTermUnused__(H264Dpb_t *dpb, uint32_t long_term_pic_num);
static void H264dec_dpb_markAllUnused__(H264Dpb_t *dpb);
static void H264dec_dpb_setOutputFunc__(H264Dpb_t *dpb, H264DPBOutputFunc func, void *user_data);
static void H264dec_dpb_init__(H264Dpb_t *dpb);
static H264Frame_t* H264dec_dpb_alloc__(H264dec_t *h264dec);
static bool_t H264dec_dpb_free__(H264Dpb_t *dpb, H264Frame_t *h264_frame);
static void H264dec_dpb_finalize__(H264Dpb_t *dpb);
static void H264dec_dpb_set_NumRefFrames__(H264Dpb_t *dpb, uint32_t numRefFrames);
static void H264dec_dpb_set_MaxLongTermIdx__(H264Dpb_t *dpb, int32_t maxLongTermIdx);
//h264 decoder continue..
static GstFlowReturn H264dec_output__(H264Dpb_t *dpb, H264Frame_t *h264_frame, void *user_data);
static bool_t H264dec_flush__(Appl_t *pThis);
static bool_t H264dec_calculatePar__(GstH264VUIParams *vui, uint16_t *par_n, uint16_t *par_d);
static void H264dec_initFrameInfo__(H264dec_t *h264_dec, GstH264NalUnit *nalu, H264Frame_t *h264_frame);
static PictureInfoH264_t H264dec_fillInfo__(H264dec_t *h264_dec, H264Frame_t *h264_frame);
static void H264dec_dumpInfo__(PictureInfoH264_t *info);

//gst
static int Gst_allocateObjects__(GstH264NalUnit** nalu, GstH264SliceHdr** slice, GstH264SPS** sps, GstH264PPS** pps, GstH264SEIMessage** sei);
static int Gst_freeObjects__(GstH264NalUnit** nalu, GstH264SliceHdr** slice, GstH264SPS** sps, GstH264PPS** pps, GstH264SEIMessage** sei);
static int Gst_checkNaluResult__(GstH264ParserResult result);
static int Nal_peekNextUnit__(Appl_t *pThis);
static int Nal_getNextUnit__(Appl_t *pThis, const void *pBuf, int* nalLength);
//other
static void yuv422YUYV_YUY2_2rgb__(char *pIn, char *pOut, unsigned long len);
static void crop_nv12__(char *pSrc, char *pDst, uint32_t srcWidth, uint32_t srcHeight, uint32_t dstWidth, uint32_t dstHeight);
static void printBuffer__(uint8_t *pBuffer, uint32_t length, char *pBufferName);
static void errno_exit__(const char *s);
static uint32_t min__(uint32_t a, uint32_t b);

int testopt2_flag = 0;
enum privateLongOptionsTag {
    OPT_TEST__ = 1000,
    OPT_YUVTORGB__,
    OPT_JPEG_ENC__,
    OPT_SCALE__,
    OPT_H264_ENC__,
    OPT_JPEG_DEC__,
    OPT_H264_DEC__
};
static const char short_options[] = "h:c:C:d:o:q:r:p:f";

static const struct option
long_options[] = {
        { "help",           no_argument,        NULL, 'h' },
        { "count",          required_argument,  NULL, 'c' },
        { "compress",       required_argument,  NULL, 'C' },
        { "device",         required_argument,  NULL, 'd' },
        { "outputMarks",    no_argument,        NULL, 'o' },
        { "quality",        required_argument,  NULL, 'q' },
        { "raw",            required_argument,  NULL, 'r' },
        { "preview",        no_argument,        NULL, 'p' },
        { "readFile",       required_argument,  NULL, 'f' },
        { "testOption",     required_argument,  NULL,  OPT_TEST__ },
        { "yuvToRgb",       no_argument,        NULL,  OPT_YUVTORGB__ },
        { "jpegEnc",        required_argument,  NULL,  OPT_JPEG_ENC__ },
        { "scale",          required_argument,  NULL,  OPT_SCALE__},
        { "testOption2",    required_argument,  &testopt2_flag, 1 },
        { "h264Enc",        no_argument,        NULL,  OPT_H264_ENC__ },
        { "jpegDec",        no_argument,        NULL,  OPT_JPEG_DEC__ },
        { "h264Dec",        required_argument,  NULL,  OPT_H264_DEC__ },
        { NULL, 0, NULL, 0 }
};

static void usage__(FILE *fp, int argc, char **argv) {
    
    fprintf(fp,
     "\nUsage: %s [options]\n"
     "Version %s\n"
     "Options:\n"
     "-h | --help          Print this message\n"
     "-c | --count         Number of frames to grab\n"
     "-C | --compress      Number of frames to compress\n"
     "-d | --device name   Video device name [%s]\n"
     "-o | --outputMarks   Print outputs marks\n"
     "-q | --quality       Jpeg enc quality\n"
     "-r | --raw           Number of raw frames to save\n"
     "-p | --preview       Preview frames to display\n"
     "-f | --reafFile      Get frame from file\n"
     "     --yuvToRgb      Convert yuv4:2:2 -> rgb -> jpeg\n"
     "     --jpegEnc       Enable jpeg encoder.\n"
     "                     Select software or hardware jpeg encoder:\n"
     "                         | 1 -> SW       |\n"
     "                         | 2 -> HW       |\n"
     "     --scale         Scale source image to (Default: none):\n"
     "                         NOTE: It can be only used with HW encoder.\n"
     "                         | 0 -> none                  |\n"
     "                         | 1 -> ARBITRARY-SCALER-VGA  |\n"
     "                         | 2 -> ARBITRARY-SCALER_QVGA |\n"
     "     --h264Enc       Enable H264 Encoder (Default: Disabled)\n"
     "     --jpegDec       Enable jpeg Decoder (Default: Disabled)\n"
     "                         Note: It can be only used with jpeg encoder.\n"
     "     --h264Dec       Enable H264 Decoder (Default: Disabled)\n"
     "\n",
     argv[0]+2, FW_VERSION, pThis->camera.deviceName);
}

static void get_options__ (int argc, char **argv)
{
    for (;;) {
        int idx = 0;
        int c;

        c = getopt_long(argc, argv, short_options, long_options, &idx);

        if (c == -1)
            break;

        switch (c) {
            case 0: {/* getopt_long() flag */
                if (testopt2_flag == 1) {
                    printf("DO something! %s\n", optarg);
                }
                break;
            }
            case OPT_TEST__: {
                printf("Test Option detected! %s\n", optarg);
                break;
            }
            case OPT_YUVTORGB__: {
                pThis->yuvToRgb = TRUE;
                break;
            }
            case OPT_JPEG_ENC__: {
                uint32_t type = strtol(optarg, NULL, 0);
                if (type > 0 && type <= HW) {
                    pThis->jpegEnc.encode = TRUE;
                    pThis->jpegEnc.type = type;
                }
                break;
            }
            case OPT_SCALE__: {
                uint32_t type = strtol(optarg, NULL, 0);
                if (type <= ARBITRARY_QVGA) {
                    pThis->ve.isp.scaler.type = type;
                }
                break;
            }
            case OPT_H264_ENC__: {
                pThis->ve.h264enc.encode = TRUE;
                break;
            }
            case OPT_JPEG_DEC__: {
                pThis->ve.jpegdec.decode = TRUE;
                break;
            }
            case OPT_H264_DEC__: {
                pThis->ve.h264dec.fileName = optarg;
                pThis->ve.h264dec.decode = TRUE;
                break;
            }
            case 'h': {
                usage__(stdout, argc, argv);
                exit(EXIT_SUCCESS);
            }
            case 'c': {
                errno = 0;
                pThis->frameCount = strtol(optarg, NULL, 0);
                pThis->compressCount = pThis->frameCount;
                if (errno)
                    errno_exit__(optarg);
                break;
            }
            case 'C': {
                errno = 0;
                pThis->compressCount = strtol(optarg, NULL, 0);
                pThis->frameCount = pThis->compressCount;
                if (errno)
                    errno_exit__(optarg);
                break;
            }
            case 'd':
                pThis->camera.deviceName = optarg;
                break;

            case 'o': {
                pThis->output_marks = TRUE;
                break;
            }
            case 'q': {
                errno = 0;
                pThis->jpegEnc.jpegEncQuality = strtol(optarg, NULL, 0);
                if (errno)
                    errno_exit__(optarg);
                if (pThis->jpegEnc.jpegEncQuality > 100) {
                    pThis->jpegEnc.jpegEncQuality = 100;
                } else if (pThis->jpegEnc.jpegEncQuality < 5) {
                    pThis->jpegEnc.jpegEncQuality = 5;
                }
                break;
            }
            case 'r': {
                errno = 0;
                pThis->rawCount = strtol(optarg, NULL, 0);
                if (errno)
                    errno_exit__(optarg);
                break;
            }
            case 'p': {
                pThis->preview = TRUE;
                break;
            }
            case 'f': {
                pThis->readFile = TRUE;
                pThis->pFileName = optarg;
                break;
            }
            default: {
                usage__(stderr, argc, argv);
                exit(EXIT_FAILURE);
            }
        }
    }
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (r == -1 && errno == EINTR);

    return r;
}

int main(int argc, char* argv[]) {
    
	struct sigaction sigact;
    long signed int nFrames = 0;
    struct      timeval tvStart, tvEnd;
    double      timediff;
    
    printf("*********************\n");
    printf("TVD demo start!\n");
    printf("*********************\n");

    /* set signal action handler */
	memset(&sigact, 0, sizeof(sigact)); 
	sigact.sa_handler = Tvin2Jpeg_handleInt__;
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    
    /* init vars */
    Tvin2jpeg_initVars__();
    get_options__(argc, argv);
    
	/* initialize camera input - video source */
    Camera_init__();
    Camera_mapBuffers__();
    
    /* initialize thread */
    CriticalSectionCreate__(&pThis->mutex);                            /* global critical section */
    CriticalSectionCreate__(&pThis->ve.mutex);             /* hardware resources critical section */
    CriticalSectionCreate__(&pThis->threadEncoder.mutex);             /* encoder critical section */
    ConditionVariable_create__(&pThis->threadEncoder.cond);              /* encoder cond variable */
    CriticalSectionCreate__(&pThis->threadJpegDecoder.mutex);    /* JPEG decoder critical section */
    ConditionVariable_create__(&pThis->threadJpegDecoder.cond);     /* JPEG decoder cond variable */
    Thread_create__(&pThis->threadEncoder, ThreadEncodeFrame__, pThis);   /* start encoder thread */
                                                                     /* start JPEG decoder thread */
    Thread_create__(&pThis->threadJpegDecoder, ThreadDecodeJpegFrame__, pThis);
    DBG("Create: %p, %p, %p, %p\n", &pThis->mutex, &pThis->ve.mutex, &pThis->threadEncoder.mutex, 
                                  &pThis->threadJpegDecoder.mutex);

    /* initialize jpegturbo compressor */
    Tj_init__();
    
    /* initialize jpeg decoder */
    JpegDec_init__();
    
    /* initialize VE - Video Engine */
    Ve_init__();
    Ve_allocInputBuffers__();
    Ve_initScalerBuffers__();
    Ve_allocOutputBuffers__();
    usleep(10000);                                                                                  //todo delete
    
    /* initialize h264 encoder */
    if (pThis->ve.h264enc.encode) {
        H264enc_init__(); 
        H264enc_new__();
        /* initialize muxer */
        Mux_init("/tmp/tvin.mkv");
        Mux_writeHeader((uint8_t*)0, 0);
    }
    
    /* initialize h264 decoder */
    if (pThis->ve.h264dec.decode == TRUE) {
        H264dec_init__();
    }
    
    usleep(300000);
    
    /* start stream */
    Camera_streamOn__();
    
    gettimeofday(&tvStart, 0);
	while((pThis->run == TRUE) && ((nFrames < pThis->frameCount) || (pThis->frameCount == -1))) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;
			
			FD_ZERO(&fds);
			FD_SET(pThis->camera.fd, &fds);
			
			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;
			r = select(pThis->camera.fd + 1, &fds, NULL, NULL, &tv);
			if (r == -1) {
				if (EINTR == errno)
					continue;
				printf("select err\n");
			} else if (r == 0) {
				fprintf(stderr, "select timeout\n");
				exit(EXIT_FAILURE);
				goto close;
			}
            if (FD_ISSET(pThis->camera.fd, &fds)) {
                if (Tvin2jpeg_readFrame__()) {
                    nFrames++;
                    break;
                }
            }
		} 
	}
    
close:
    gettimeofday(&tvEnd, 0);
    timediff = (((tvEnd.tv_sec - tvStart.tv_sec) * 1000000.0) + (tvEnd.tv_usec - tvStart.tv_usec))
                    / 1000000.0;
    printf("\nTime for %d frames: %f\n", (int)nFrames, timediff);
    
    /* destroy all threads */
    {
    /* destroy encoder thread */
    while(1) {                                    /* wait until encode process image is completed */
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadEncoder.process == FALSE) {
            CriticalSectionExit__(&pThis->mutex);
            break;
        }
        CriticalSectionExit__(&pThis->mutex);
        usleep(5000);
    }
    /* destroy decoder thread */
    while(1) {                                    /* wait until decode process image is completed */
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadJpegDecoder.process == FALSE) {
            CriticalSectionExit__(&pThis->mutex);
            break;
        }
        CriticalSectionExit__(&pThis->mutex);
        usleep(5000);
    }
    Thread_destroy__();
    while(1) {                                            /* wait until encode thread is finished */
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadEncoder.finished == TRUE) {
            CriticalSectionExit__(&pThis->mutex);
            break;
        }
        ConditionVariable_signal__(&pThis->threadEncoder.cond);
        CriticalSectionExit__(&pThis->mutex);
        usleep(5000);
    }
    ConditionVariable_destroy__(&pThis->threadEncoder.cond);     /* destroy encoder condition var */
    CriticalSectionDestroy__(&pThis->threadEncoder.mutex);               /* destroy encoder mutex */
    while(1) {                                            /* wait until decode thread is finished */
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadJpegDecoder.finished == TRUE) {
            CriticalSectionExit__(&pThis->mutex);
            break;
        }
        ConditionVariable_signal__(&pThis->threadJpegDecoder.cond);
        CriticalSectionExit__(&pThis->mutex);
        usleep(5000);
    }
    }

    /* uninitialize jpeg turbo compressor */
    Tj_close__();
    
    /* close camera */
    Camera_streamOff__();
    Camera_unmapBuffers__();
    Camera_close__();
    
    /* free h264 encoder */
    if (pThis->ve.h264enc.encode) {
        H264enc_free__();
        /* free muxer */
        Mux_close();
    }
    
    /* close h264 decoder */
    if (pThis->ve.h264dec.decode == TRUE) {
        Disp_stop__(&pThis->disp.layer[DL_H264]);              /* stop h264 decoder video display */
        Disp_layerRelease__(&pThis->disp.layer[DL_H264]);     /* close jpeg decoder display layer */
        H264dec_free__();
    }
    
    /* close display */
    {
    if (pThis->preview == TRUE && 
       (pThis->ve.jpegdec.decode == FALSE || pThis->ve.h264dec.decode == FALSE)) {
        Disp_stop__(&pThis->disp.layer[DL_RAW]);                     /* stop camera video display */
        Disp_layerRelease__(&pThis->disp.layer[DL_RAW]);	        /* close camera display layer */
    }
    if (pThis->ve.jpegdec.decode == TRUE) {
        Disp_stop__(&pThis->disp.layer[DL_JPEG]);              /* stop jpeg decoder video display */
        Disp_layerRelease__(&pThis->disp.layer[DL_JPEG]);     /* close jpeg decoder display layer */
    }
    Disp_exit__();                                                  /* close and turn off display */
    }
    
    /* destroy critical sections */
    ConditionVariable_destroy__(&pThis->threadJpegDecoder.cond); /* destroy decoder condition var */
    CriticalSectionDestroy__(&pThis->threadJpegDecoder.mutex);      /* destroy JPEG decoder mutex */
    CriticalSectionDestroy__(&pThis->ve.mutex);                               /* destroy hw mutex */
    CriticalSectionDestroy__(&pThis->mutex);                              /* destroy global mutex */
    
    /* uninitialize VE */
    Ve_freeOutputBuffers__();
    Ve_freeInputBuffers__();
    Ve_free__();
    
    DBG("Encoder MaxTime: %f, MinTime: %f", pThis->maxTimeForOneFrame, pThis->minTimeForOneFrame);
    
    DBG("Jpeg Decoder MaxTime: %f, MinTime: %f", pThis->ve.jpegdec.maxTimeForOneFrame, 
                                                 pThis->ve.jpegdec.minTimeForOneFrame);
	printf("TVD demo bye!\n");
	return 0;						
}

/* static functions definitions */
//pthread
static void CriticalSectionCreate__ (pthread_mutex_t *mutex) {
    
    int retVal;

    retVal = pthread_mutex_init(mutex, NULL);                      /* dynamically mutexinitmethod */

    assert(retVal == 0);                         /* pthread_mutex_init() must return with success */
}

static void CriticalSectionEnter__ (pthread_mutex_t *mutex) {

    int retVal;

    retVal = pthread_mutex_lock(mutex);

    //if (retVal != 0) {
        //DBGF("RetVal: %p, %d", mutex, retVal);
    //} else {
        //printf("Enter: %p\n", mutex);
    //}
    assert(retVal == 0);                         /* pthread_mutex_lock() must return with success */
}

static void CriticalSectionExit__ (pthread_mutex_t *mutex) {

    int retVal;

    retVal = pthread_mutex_unlock(mutex);

    //if (retVal != 0) {
        //DBGF("RetVal: %p, %d", mutex, retVal);
    //} else {
        //printf("Exit: %p\n", mutex);
    //}
    assert(retVal == 0);                       /* pthread_mutex_unlock() must return with success */
}

static void CriticalSectionDestroy__ (pthread_mutex_t *mutex) {

    int retVal;

    retVal = pthread_mutex_destroy(mutex);

    //if (retVal != 0) {
        //DBGF("RetVal: %p, %d", mutex, retVal);//Why is this happening?? Return 16 EBUSY //TODO!!!
    //} else {
        //DBGF("%p", mutex);
    //}
    assert(retVal == 0);                    /* pthread_mutex_destroy() must return with success */
}

static void ConditionVariable_create__ (pthread_cond_t  *cond) {

    int retVal;
    
    retVal = pthread_cond_init(cond, NULL);          /* dynamically condinitmethod */

    assert(retVal == 0);                          /* pthread_cond_init() must return with success */
}

static void ConditionVariable_wait__ (pthread_cond_t  *cond, pthread_mutex_t *mutex) {

    int retVal;

    retVal = pthread_cond_wait(cond, mutex);

    assert(retVal == 0);                          /* pthread_cond_wait() must return with success */
}

static void ConditionVariable_signal__ (pthread_cond_t  *cond) {

    int retVal;
    
    retVal = pthread_cond_signal(cond);

    assert(retVal == 0);                        /* pthread_cond_signal() must return with success */
}

static void ConditionVariable_destroy__ (pthread_cond_t  *cond) {

    int retVal;

    retVal = pthread_cond_destroy(cond);                 /* free the specified condition variable */

    assert(retVal == 0);                       /* pthread_cond_destroy() must return with success */
}

static void Thread_create__ (Thread_t *pThread, threadFunction_t pThreadFunction, void *pThreadParameter) {

    int retVal;
    
    pThread->running = TRUE;
    pThread->process = FALSE;
    pThread->finished = FALSE;
    retVal = pthread_create(&pThread->id, NULL, pThreadFunction, pThreadParameter);

    assert(retVal == 0);                             /* pthread_create() must return with success */
}

static void Thread_destroy__ (void) {

    //int retVal;
    
    /* stop encoder thread */
    CriticalSectionEnter__(&pThis->mutex);
    pThis->threadEncoder.running = FALSE;
    CriticalSectionExit__(&pThis->mutex);
    ConditionVariable_signal__(&pThis->threadEncoder.cond);
    //retVal = pthread_join(pThis->threadEncoder.id, NULL);
    //assert(retVal == 0);                             /* pthread_join() must return with success */
        
    /* stop decoder thread */
    CriticalSectionEnter__(&pThis->mutex);
    pThis->threadJpegDecoder.running = FALSE;
    CriticalSectionExit__(&pThis->mutex);
    ConditionVariable_signal__(&pThis->threadJpegDecoder.cond);
    //retVal = pthread_join(pThis->threadJpegDecoder.id, NULL);
    //assert(retVal == 0);                             /* pthread_join() must return with success */
}

THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadEncodeFrame__ (void *pArgument) {

    Appl_t     *pThis;
    const void *pFrame;
    int         frameSize;
    struct      timeval tvStart, tvEnd;
    double      timediff;
        /* global and loval variable pThis have same name but local variable will take preference */
    pThis = (Appl_t*)pArgument;
    
    /* set thread name */
    //prctl(PR_SET_NAME, "EncodeThread", 0, 0, 0);
    
    while (1) {
        CriticalSectionEnter__(&pThis->threadEncoder.mutex);
        ConditionVariable_wait__(&pThis->threadEncoder.cond, &pThis->threadEncoder.mutex);
        CriticalSectionExit__(&pThis->threadEncoder.mutex);
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadEncoder.running == FALSE) {
            CriticalSectionExit__(&pThis->mutex);                             /* stop this thread */
            break;
        }
        pFrame = pThis->frame.pFrame;                                   /* take user data pointer */
        frameSize = pThis->frame.frameSize;
        pThis->threadEncoder.process = TRUE;
        CriticalSectionExit__(&pThis->mutex);
        
        gettimeofday(&tvStart, 0);
        Tvin2jpeg_processImage__(pFrame, frameSize);
        gettimeofday(&tvEnd, 0);
        timediff = (((tvEnd.tv_sec - tvStart.tv_sec) * 1000000.0) +
                     (tvEnd.tv_usec - tvStart.tv_usec)) / 1000000.0;
        //printf("\nTime for 1 frame: %f\n", timediff);
        if (timediff > pThis->maxTimeForOneFrame) {
            pThis->maxTimeForOneFrame = timediff;
        }
        if (timediff < pThis->minTimeForOneFrame) {
            pThis->minTimeForOneFrame = timediff;
        }
        
        CriticalSectionEnter__(&pThis->mutex);
        pThis->threadEncoder.process = FALSE;                                   /* thread is idle */
        CriticalSectionExit__(&pThis->mutex);
    }
    
    CriticalSectionEnter__(&pThis->mutex);
    pThis->threadEncoder.finished = TRUE;                                          /* thread died */
    CriticalSectionExit__(&pThis->mutex);
    DBGF("Finished...");
    return THREAD_FUNCTION_RETURN_VALUE;
}

THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadDecodeJpegFrame__ (void *pArgument) {

    Appl_t     *pThis;
    uint32_t    id;
    int         in;
    struct stat s;
    uint8_t    *data;
    struct      timeval tvStart, tvEnd;
    double      timediff;
    char        fname[2048];
        /* global and loval variable pThis have same name but local variable will take preference */
    pThis = (Appl_t*)pArgument;
    
    /* set thread name */
    prctl(PR_SET_NAME, "DecodeJPEG_Thread", 0, 0, 0);
    
    /* wait for start decoding signal */
    CriticalSectionEnter__(&pThis->threadJpegDecoder.mutex);
        ConditionVariable_wait__(&pThis->threadJpegDecoder.cond, &pThis->threadJpegDecoder.mutex);
    CriticalSectionExit__(&pThis->threadJpegDecoder.mutex);
    
    while (1) {
        CriticalSectionEnter__(&pThis->mutex);
        id = pThis->jpegEnc.id;
        CriticalSectionExit__(&pThis->mutex);
        
        if (pThis->ve.jpegdec.id >= id) {                      /* wait for new frame from encoder */
            CriticalSectionEnter__(&pThis->threadJpegDecoder.mutex);
            ConditionVariable_wait__(&pThis->threadJpegDecoder.cond, &pThis->threadJpegDecoder.mutex);
            CriticalSectionExit__(&pThis->threadJpegDecoder.mutex);
        }
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadJpegDecoder.running == FALSE) {
            CriticalSectionExit__(&pThis->mutex);                             /* stop this thread */
            break;
        }
        pThis->threadJpegDecoder.process = TRUE;
        CriticalSectionExit__(&pThis->mutex);
        
        /* prepare file name for decode */
        /* open file */
        snprintf(fname, sizeof(fname), "/tmp/testImage_%03d.jpg", pThis->ve.jpegdec.id);
        if ((in = open(fname, O_RDONLY)) == -1) {
            DBG("Error open: %s", fname);
            exit(EXIT_FAILURE);
        }
        if (fstat(in, &s) < 0) {
            DBG("Error stat %s", fname);
            exit(EXIT_FAILURE);
        }
        if (s.st_size == 0) {
            DBG("Error %s empty", fname);
            exit(EXIT_FAILURE);
        }
        /* map file */
        if ((data = mmap(NULL, s.st_size, PROT_READ, MAP_SHARED, in, 0)) == MAP_FAILED) {
            DBG("Error mmap %s", fname);
            exit(EXIT_FAILURE);
        }
        gettimeofday(&tvStart, 0);
        /* decode jpeg */
        {
            struct jpeg_t jpeg ;
            
            memset(&jpeg, 0, sizeof(jpeg));
            /* parse jpeg picture */
            if (!parse_jpeg(&jpeg, data, s.st_size)) {
                DBG("Can't parse JPEG");
                munmap(data, s.st_size);
                close(in);
                continue;
            }
            /* dump jpeg */
            //dump_jpeg(&jpeg);
            //fflush(stdout);
            /* decode picture */
            JpegDec_decode__(&jpeg);
            
            /* show frame on display */
            if (pThis->disp.layer[DL_JPEG].initialized == FALSE) {
                                                        /*initialize display and show first frame */
                uint8_t             fmt;
                __disp_pixel_fmt_t  format;
                
                fmt = (jpeg.comp[0].samp_h << 4) | jpeg.comp[0].samp_v;
                switch (fmt) {
                    case 0x11:
                    case 0x21: {
                        format = DISP_FORMAT_YUV422;
                        break;
                    }
                    case 0x12:
                    case 0x22:
                    default: {
                        format = DISP_FORMAT_YUV420;
                        break;
                    }
                }
                
                if (Disp_init__(&pThis->disp.layer[DL_JPEG], format, 
                    DISP_MOD_MB_UV_COMBINED, DISP_SEQ_UVUV, jpeg.width, 
                    jpeg.height, DISP_DL_JPEG_SCN_POS_X, DISP_DL_JPEG_SCN_POS_Y, 
                    DISP_DL_JPEG_SCN_WIDTH, DISP_DL_JPEG_SCN_HEIGHT) == 0) {
                    Disp_start__(&pThis->disp.layer[DL_JPEG]);
                    Disp_on__();
                    pThis->disp.layer[DL_JPEG].initialized = TRUE;
                    /* show first frame */
                    Disp_newDecoder_frame__(&pThis->disp.layer[DL_JPEG], jpeg.width, jpeg.height,
                    ve_virt2phys(pThis->ve.jpegdec.luma_buffer), 
                    ve_virt2phys(pThis->ve.jpegdec.chroma_buffer),
                    pThis->ve.jpegdec.id);
                    
                }

            } else {                                                           /* show next frame */
                    Disp_newDecoder_frame__(&pThis->disp.layer[DL_JPEG], jpeg.width, jpeg.height,
                    ve_virt2phys(pThis->ve.jpegdec.luma_buffer), 
                    ve_virt2phys(pThis->ve.jpegdec.chroma_buffer),
                    pThis->ve.jpegdec.id);
            }
            
            /* close input file */
            munmap(data, s.st_size);
            close(in);
        }
        gettimeofday(&tvEnd, 0);
        timediff = (((tvEnd.tv_sec - tvStart.tv_sec) * 1000000.0) +
                     (tvEnd.tv_usec - tvStart.tv_usec)) / 1000000.0;
        //printf("\nTime for 1 frame: %f\n", timediff);
        if (timediff > pThis->ve.jpegdec.maxTimeForOneFrame) {
            pThis->ve.jpegdec.maxTimeForOneFrame = timediff;
        }
        if (timediff < pThis->ve.jpegdec.minTimeForOneFrame) {
            pThis->ve.jpegdec.minTimeForOneFrame = timediff;
        }
        
        /* decode h264 */
        //TODO
        
        
        DBG("DecodeJpeg.. Id: %d", pThis->ve.jpegdec.id);
        pThis->ve.jpegdec.id++;                                          /* increment frame index */
        CriticalSectionEnter__(&pThis->mutex);
        pThis->threadJpegDecoder.process = FALSE;                                   /* thread is idle */
        CriticalSectionExit__(&pThis->mutex);
    }
    
    CriticalSectionEnter__(&pThis->mutex);
    pThis->threadJpegDecoder.finished = TRUE;                                          /* thread died */
    CriticalSectionExit__(&pThis->mutex);
    DBGF("Finished...");
    return THREAD_FUNCTION_RETURN_VALUE;
}

THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadDecodeH264Frame__ (void *pArgument) {

    Appl_t     *pThis;
    uint32_t    id;
    //int         target_index = -1;
    uint32_t    i;
    int         nals = 0;
    int         nalLength;
    VideoSurface_t     *dpbSurface;
    BitstreamBuffer     bitstreamBuffer;
    GstH264ParserResult h264ParserResult;
    bool_t      spsSet = FALSE;
    bool_t      ppsSet = FALSE;
    struct      timeval tvStart, tvEnd;
    double      timediff;
    GstFlowReturn gstRet;
    //
    H264Frame_t     *h264Frame;
    GstH264NalUnit  *nalu;
    GstH264SliceHdr *slice;
    PictureInfoH264_t   info;
        /* global and loval variable pThis have same name but local variable will take preference */
    pThis = (Appl_t*)pArgument;
    
    /* set thread name */
    prctl(PR_SET_NAME, "DecodeH264_Thread", 0, 0, 0);
    
    /* wait for start decoding signal */
    CriticalSectionEnter__(&pThis->threadH264Decoder.mutex);
    ConditionVariable_wait__(&pThis->threadH264Decoder.cond, &pThis->threadH264Decoder.mutex);
    CriticalSectionExit__(&pThis->threadH264Decoder.mutex);
    
    /* allocate bitstream buffer */
    bitstreamBuffer.bitstream = calloc(NALU_BUFFER_LENGTH, sizeof(uint8_t));
    if(bitstreamBuffer.bitstream == NULL) {
        DBG(KRED"Error: H264 Decoder => MALLOC: bitstreamBuffer!"KNRM);
        gst_h264_nal_parser_free(pThis->ve.h264dec.gst.parser);
        Gst_freeObjects__(&pThis->ve.h264dec.gst.nalu, &pThis->ve.h264dec.gst.slice, 
                &pThis->ve.h264dec.gst.sps, &pThis->ve.h264dec.gst.pps, &pThis->ve.h264dec.gst.sei);
        exit(EXIT_FAILURE);
    }

//WaitForPPSSPS:
    /* find stream properties (most important: SPS, PPS) */
    spsSet = FALSE;
    ppsSet = FALSE;
    while(Nal_getNextUnit__(pThis, bitstreamBuffer.bitstream, &nalLength) == 0) {
        /* Got a NAL unit. Now parse it. */
        nals++;
        h264ParserResult = gst_h264_parser_identify_nalu(
                     pThis->ve.h264dec.gst.parser,
                     (const guint8 *)bitstreamBuffer.bitstream,
                     0,
                     (gsize)nalLength,
                     pThis->ve.h264dec.gst.nalu);
                     
        if (h264ParserResult != 0) {
            Gst_checkNaluResult__(h264ParserResult);                       /* print error message */
            /* print NAL block */
            //uint8_t *bitstream;
            //bitstream = (uint8_t*)(bitstreamBuffer.bitstream);
            //DBG(KRED"ERROR: NAL decode %d:"KNRM, nalLength);
            //for (i = 0; i < nalLength; i++) {
                //if ((i+1)%16 == 0) {
                    //printf("0x%X\n", *bitstream++);
                //} else {
                    //printf("0x%X, ", *bitstream++);
                //}
            //}
            //printf("\n");
            //fflush(stdout);
            goto out;
        } else {
            nalu = pThis->ve.h264dec.gst.nalu;
            DBG(KMAG"NAL found! Nal_ref_idc: %d, Type: %d"KNRM, nalu->ref_idc, nalu->type);
            switch(nalu->type) {
                /* Sequence Parameter Set */
                case GST_H264_NAL_SPS: {
                    H264dec_handleSps__(pThis);
                    DBG("Sequence Parameter Set: Id: %d", pThis->ve.h264dec.gst.sps->id);
                    spsSet = TRUE;
                    if (ppsSet == TRUE) {
                        goto SPSPPS;
                    }
                    break;
                }
                /* Picture Parameter Set */
                case GST_H264_NAL_PPS: {
                    H264dec_handlePps__(pThis);
                    DBG("Picture Parameter Set: Id: %d", pThis->ve.h264dec.gst.pps->id);
                    ppsSet = TRUE;
                    if (spsSet == TRUE) {
                        goto SPSPPS;
                    }
                    break;
                }
                case GST_H264_NAL_SEI: {
                    DBG("Supplemental Enhancement Information");
                    H264dec_handleSei__(pThis);
                    break;
                }
                default: {
                    printf("Uknown NAL Unit type...\n");
                    gst_h264_parser_parse_nal(pThis->ve.h264dec.gst.parser, nalu);
                    //goto EndOfWhile;
                }
            }
        }
        //todo break kada nadjem sps pps
    }
    
SPSPPS:
    /* create reference and output surfaces */
    if (pThis->ve.h264dec.width == 0 || pThis->ve.h264dec.height == 0) {
        DBGF(KRED"ERROR: H264 Decoder => get picture parameters!"KNRM);
        goto out;
    } else {
        DBG("PictureInfo: %dx%d", pThis->ve.h264dec.width, pThis->ve.h264dec.height);
        if ((pThis->ve.h264dec.width > H264DEC_MAX_WIDTH) ||
            (pThis->ve.h264dec.height > H264DEC_MAX_HEIGHT)) {
                goto out;                                          /* resolution is not supported */
        }
        if ((pThis->ve.h264dec.gst.sps->profile_idc != ENCODER_DECODER_PROFILE_H264_BASELINE) &&
            (pThis->ve.h264dec.gst.sps->profile_idc != ENCODER_DECODER_PROFILE_H264_MAIN) &&
            (pThis->ve.h264dec.gst.sps->profile_idc != ENCODER_DECODER_PROFILE_H264_HIGH)) {
            goto out;                                           /* profile level is not supported */
        }
        if (pThis->ve.h264dec.gst.sps->level_idc > ENCODER_DECODER_LEVEL_H264_5_1) {
            goto out;                                                /* level id is not supported */
        }
        H264dec_new__(&pThis->ve.h264dec);
    }
    for (i = 0; i < OUTPUT_SURFACES; i++) {
        if (VideoSurface_create(CHROMA_TYPE_420, pThis->ve.h264dec.width, pThis->ve.h264dec.height,
                                &pThis->ve.h264dec.output[i]) != VC_STATUS_OK) {
            DBGF(KRED"ERROR: H264 Decoder => create output surface!"KNRM);
            goto out;
        } else {
            DBGF("H264 Decoder => create output surface: %d", pThis->ve.h264dec.output[i]);
        }
    }
    
    while (1) {
        CriticalSectionEnter__(&pThis->mutex);
        id = pThis->ve.h264enc.id;
        CriticalSectionExit__(&pThis->mutex);
        
        if (pThis->ve.h264dec.id >= id) {                      /* wait for new frame from encoder */
            CriticalSectionEnter__(&pThis->threadH264Decoder.mutex);
            ConditionVariable_wait__(&pThis->threadH264Decoder.cond, &pThis->threadH264Decoder.mutex);
            CriticalSectionExit__(&pThis->threadH264Decoder.mutex);
        }
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadH264Decoder.running == FALSE) {
            CriticalSectionExit__(&pThis->mutex);                             /* stop this thread */
            break;
        }
        pThis->threadH264Decoder.process = TRUE;
        CriticalSectionExit__(&pThis->mutex);
        
        /* fill input data */
        if(Nal_getNextUnit__(pThis, bitstreamBuffer.bitstream, &nalLength) == 0) {
            /* Got a NAL unit. Now parse it... */
            nals++;
            h264ParserResult = gst_h264_parser_identify_nalu(
                         pThis->ve.h264dec.gst.parser,
                         (const guint8 *)bitstreamBuffer.bitstream,
                         0,
                         (gsize)nalLength,
                         pThis->ve.h264dec.gst.nalu);
            
            if (h264ParserResult != 0) {
                Gst_checkNaluResult__(h264ParserResult);                   /* print error message */
                /* print NAL block */
                //uint8_t *bitstream;
                //bitstream = (uint8_t*)(bitstreamBuffer.bitstream);
                //DBG(KRED"ERROR: NAL decode %d:"KNRM, nalLength);
                //for (i = 0; i < nalLength; i++) {
                    //if ((i+1)%16 == 0) {
                        //printf("0x%X\n", *bitstream++);
                    //} else {
                        //printf("0x%X, ", *bitstream++);
                    //}
                //}
                //printf("\n");
                //fflush(stdout);
                goto out;
            } else {
                nalu = pThis->ve.h264dec.gst.nalu;
                DBG(KMAG"NAL found! Nal_ref_idc: %d, Type: %d, %sIdr: %s"KNRM, 
                    nalu->ref_idc, nalu->type,
                    nalu->idr_pic_flag == 1 ? KCYN : KYEL,
                    nalu->idr_pic_flag == 1 ? "TRUE" : "FALSE");
                
                switch(nalu->type) {
                    /* Sequence Parameter Set */
                    case GST_H264_NAL_SPS: {
                        H264dec_handleSps__(pThis);
                        DBG("Sequence Parameter Set: Id: %d", pThis->ve.h264dec.gst.sps->id);
                        CriticalSectionEnter__(&pThis->mutex);
                        pThis->threadH264Decoder.process = FALSE;               /* thread is idle */
                        CriticalSectionExit__(&pThis->mutex);
                        break;
                    }
                    /* Picture Parameter Set */
                    case GST_H264_NAL_PPS: {
                        H264dec_handlePps__(pThis);
                        DBG("Picture Parameter Set: Id: %d", pThis->ve.h264dec.gst.pps->id);
                        CriticalSectionEnter__(&pThis->mutex);
                        pThis->threadH264Decoder.process = FALSE;               /* thread is idle */
                        CriticalSectionExit__(&pThis->mutex);
                        break;
                    }
                    case GST_H264_NAL_SEI: {
                        DBG("Supplemental Enhancement Information");
                        H264dec_handleSei__(pThis);
                        CriticalSectionEnter__(&pThis->mutex);
                        pThis->threadH264Decoder.process = FALSE;               /* thread is idle */
                        CriticalSectionExit__(&pThis->mutex);
                        break;
                    }
                    case GST_H264_NAL_SLICE:
                    case GST_H264_NAL_SLICE_IDR: {
                        //DBG("%s: RefIdc: %d", pThis->ve.h264dec.gst.nalu->type == GST_H264_NAL_SLICE ? 
                                //"GST_H264_NAL_SLICE" : "GST_H264_NAL_SLICE_IDR", 
                                    //pThis->ve.h264dec.gst.nalu->ref_idc);
                        /* NAL unit decoding process. */
                        /* Populate GstH264SliceHdr... */
                        if (gst_h264_parser_parse_slice_hdr(pThis->ve.h264dec.gst.parser, nalu, 
                                   pThis->ve.h264dec.gst.slice, TRUE, TRUE) != GST_H264_PARSER_OK) {
                            /* invalid packet */
                            CriticalSectionEnter__(&pThis->mutex);
                            pThis->threadH264Decoder.process = FALSE;           /* thread is idle */
                            CriticalSectionExit__(&pThis->mutex);
                            break;                                                  /* skip frame */
                        }
                        
                        slice = pThis->ve.h264dec.gst.slice;
                        
                        /* print info */
                        DBG("FrameNum: %d, dimensions: %dx%d MB (Macroblocks)",
                                slice->frame_num, slice->pps->sequence->pic_width_in_mbs_minus1+1,
                                slice->pps->sequence->pic_height_in_map_units_minus1+1
                        );
                        
                        /* get new h264Frame DPB */
                        if ((h264Frame = H264dec_dpb_alloc__(&pThis->ve.h264dec)) == NULL) {
                            CriticalSectionEnter__(&pThis->mutex);
                            pThis->threadH264Decoder.process = FALSE;
                            CriticalSectionExit__(&pThis->mutex);
                            DBG(KRED"ERROR: Can not allocate DPB!"KNRM);
                            goto out;
                        }
                        h264Frame->slice_hdr = *slice;
                        
                        /* propagate IDR */
                        if (nalu->idr_pic_flag) {
                            gstRet = H264dec_idr__ (&pThis->ve.h264dec, h264Frame);
                            if (gstRet == GST_FLOW_OK)
                              pThis->ve.h264dec.got_idr = TRUE;
                            else {
                              DBG(KRED"Error: Skip Frame!"KNRM);
                                CriticalSectionEnter__(&pThis->mutex);
                                pThis->threadH264Decoder.process = FALSE;       /* thread is idle */
                                CriticalSectionExit__(&pThis->mutex);
                              break;
                            }
                        }
                        
                        /* check if we've got a IDR frame yet */
                        if (!pThis->ve.h264dec.got_idr) {
                            /* skip frame.. wait for idr.. */
                            CriticalSectionEnter__(&pThis->mutex);
                            pThis->threadH264Decoder.process = FALSE;           /* thread is idle */
                            CriticalSectionExit__(&pThis->mutex);
                            break;
                        }
                        
                        /* init h264_frame info */
                        H264dec_initFrameInfo__(&pThis->ve.h264dec, nalu, h264Frame);

                        /* ...and propagate information to PictureInfo. */
                        info = H264dec_fillInfo__(&pThis->ve.h264dec, h264Frame);
                        //H264dec_dumpInfo__(&info);
                        
                        /* create bitstream buffer */
                        while(Nal_peekNextUnit__(pThis) == 0) {
                            int nal_extra_length;
                            //printf("Another NAL unit for this picture found!\n");
                            /* Truncate by 4 - don't repeat start codes */
                            Nal_getNextUnit__(pThis, bitstreamBuffer.bitstream+nalLength-8, 
                                                &nal_extra_length);
                            nalLength += nal_extra_length - 8;
                            info.slice_count++;
                        }
                        
                        /* copy all slices to VE buffer */
                        pThis->ve.h264dec.dataLen = nalLength - 8;
                        memcpy(pThis->ve.h264dec.data, bitstreamBuffer.bitstream, 
                               pThis->ve.h264dec.dataLen);
                        memset(pThis->ve.h264dec.data + pThis->ve.h264dec.dataLen, 0, 
                               VBV_SIZE - pThis->ve.h264dec.dataLen);
                        /* flush cache */
                        ve_flush_cache(pThis->ve.h264dec.data, VBV_SIZE);
                        ve_flush_cache(pThis->ve.h264dec.extra_data, 
                                       pThis->ve.h264dec.extra_data_size);
                        /* start decoding */
                        dpbSurface = Handle_get(h264Frame->surface);
                        gettimeofday(&tvStart, 0);
                        DBG("DecodeH264.. Id: %d, DPB_SurfaceId: %d", pThis->ve.h264dec.id, 
                                h264Frame->surface);
                        H264dec_decode__(pThis, &info, pThis->ve.h264dec.dataLen, 
                                         dpbSurface);
                        //ve_flush_cache(pThis->ve.h264dec.data, VBV_SIZE);
                        //ve_flush_cache(pThis->ve.h264dec.extra_data, pThis->ve.h264dec.extra_data_size);
                                                                             /* flush output data */
                        ve_flush_cache(dpbSurface->data, dpbSurface->size);
                        if (dpbSurface->extra_data != NULL) {
                            ve_flush_cache(dpbSurface->extra_data, pThis->ve.h264dec.video_extra_data_len*2);
                        }
                        
                        { /* DPB handling -> Start */
                        if (nalu->ref_idc != 0 && !nalu->idr_pic_flag) {
                            if (slice->dec_ref_pic_marking.adaptive_ref_pic_marking_mode_flag) {
                                GstH264RefPicMarking *marking;
                                guint i;
                                
                                DBG("DPB handling: adaptive_ref_pic_marking_mode_flag: %d",
                                        slice->dec_ref_pic_marking.n_ref_pic_marking);

                                marking = slice->dec_ref_pic_marking.ref_pic_marking;
                                for (i = 0; i < slice->dec_ref_pic_marking.n_ref_pic_marking; i++) {
                                    
                                    DBG("Operation: %d", 
                                            marking[i].memory_management_control_operation);

                                    switch (marking[i].memory_management_control_operation) {
                                        case 1: {
                                            guint16 pic_num;

                                            pic_num = slice->frame_num -
                                            (marking[i].difference_of_pic_nums_minus1 + 1);
                                            H264dec_dpb_markShortTermUnused__(&pThis->ve.h264dec.dpb, 
                                                                               pic_num);
                                            break;
                                        }
                                        case 2: {
                                            H264dec_dpb_markLongTermUnused__(&pThis->ve.h264dec.dpb,
                                                                      marking[i].long_term_pic_num);
                                            break;
                                        }
                                        case 3: {
                                            uint32_t pic_num;

                                            pic_num = slice->frame_num - 
                                                     (marking[i].difference_of_pic_nums_minus1 + 1);
                                            H264dec_dpb_markLongTerm__(&pThis->ve.h264dec.dpb, 
                                                           pic_num, marking[i].long_term_frame_idx);
                                            break;
                                        }
                                        case 4: {
                                            //g_object_set (h264_dec->dpb, "max-longterm-frame-idx",
                                            //marking[i].max_long_term_frame_idx_plus1 - 1, NULL);
                                            H264dec_dpb_set_MaxLongTermIdx__(&pThis->ve.h264dec.dpb, 
                                                      marking[i].max_long_term_frame_idx_plus1 - 1);
                                            break;
                                        }
                                        case 5: {
                                            H264dec_dpb_markAllUnused__ (&pThis->ve.h264dec.dpb);
                                            //g_object_set (h264_dec->dpb, "max-longterm-frame-idx", -1, NULL);
                                            H264dec_dpb_set_MaxLongTermIdx__(&pThis->ve.h264dec.dpb, 
                                                                             - 1);
                                            break;
                                        }
                                        default: {
                                            break;
                                        }
                                        break;
                                    }
                                }
                            } else {
                                //DBG("DPB handling: H264dec_dpb_markSliding__");
                                H264dec_dpb_markSliding__(&pThis->ve.h264dec.dpb);
                            }
                        }

                        H264dec_dpb_add__(&pThis->ve.h264dec.dpb, h264Frame);
                        } /* DPB handling -> End */
                        
                        gettimeofday(&tvEnd, 0);
                        CriticalSectionEnter__(&pThis->mutex);
                        pThis->threadH264Decoder.process = FALSE;               /* thread is idle */
                        CriticalSectionExit__(&pThis->mutex);
                        timediff = (((tvEnd.tv_sec - tvStart.tv_sec) * 1000000.0) +
                                     (tvEnd.tv_usec - tvStart.tv_usec)) / 1000000.0;
                        //printf("\nTime for 1 frame: %f\n", timediff);
                        if (timediff > pThis->ve.jpegdec.maxTimeForOneFrame) {
                            pThis->ve.jpegdec.maxTimeForOneFrame = timediff;
                        }
                        if (timediff < pThis->ve.jpegdec.minTimeForOneFrame) {
                            pThis->ve.jpegdec.minTimeForOneFrame = timediff;
                        }
                        pThis->ve.h264dec.id++;                          /* increment frame index */
                        break;
                    }
                    default: {
                        printf("Uknown NAL Unit type...\n");
                        gst_h264_parser_parse_nal(pThis->ve.h264dec.gst.parser, nalu);
                        CriticalSectionEnter__(&pThis->mutex);
                        pThis->threadH264Decoder.process = FALSE;               /* thread is idle */
                        CriticalSectionExit__(&pThis->mutex);
                        break;
                    }
                }
            }
        } else {
            /* there are no more NAL-s */
            goto out;
        }
    }
    
out:
    if (bitstreamBuffer.bitstream != NULL) {
        free((void *)bitstreamBuffer.bitstream);                          /* free bitstream bufer */
    }
    CriticalSectionEnter__(&pThis->mutex);
    pThis->threadH264Decoder.process = FALSE;                                   /* thread is idle */
    pThis->threadH264Decoder.finished = TRUE;                                  /* thread finished */
    CriticalSectionExit__(&pThis->mutex);
    DBGF("H264 Decoder Thread Finished...");
    DBG("Found %d NAL units!", nals);
    return THREAD_FUNCTION_RETURN_VALUE;
}


//Tvin2jpeg
static int Tvin2jpeg_readFrame__ (void) {
    
	struct v4l2_buffer buf;
    
    /* print timestamp */
    //time_t t = time(NULL);
    //struct tm tm = *localtime(&t);
    //printf("Tvin2jpeg_readFrame__: %d:%d:%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
	
    /* get data */
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (xioctl(pThis->camera.fd, VIDIOC_DQBUF, &buf) == -1) {
        switch (errno) {
            case EAGAIN:
                DBG("Tvin2jpeg_readFrame__ EAGAIN!");
                return 0;
            case EIO:
                /* Could ignore EIO, see spec. */
                /* fall through */
                DBG("EIO!");
            default:
                errno_exit__("VIDIOC_DQBUF");
        }
    }
	assert(buf.index < pThis->camera.n_buffers);
    assert(pThis->camera.buffers[buf.index].queued == TRUE);
    pThis->camera.buffers[buf.index].queued = FALSE;
    
    //printf("Get frame: bufIndex: %d, buffStart: %p, buffOffset: %p, Length: %d, bytesused: %d, "
           //"Sequence: %d, TimeStamp: %ld\n", buf.index, pThis->camera.buffers[buf.index].start, 
           //(void *)buf.m.offset, buf.length, buf.bytesused, buf.sequence, buf.timestamp.tv_sec);
    
    /* process image */
    //Tvin2jpeg_processImage__(pThis->camera.buffers[buf.index].start, buf.bytesused, buf.index);
    CriticalSectionEnter__(&pThis->mutex);
    pThis->frame.pFrame = pThis->camera.buffers[buf.index].start;
    pThis->frame.frameSize = buf.bytesused;
    pThis->frame.index++;
    CriticalSectionExit__(&pThis->mutex);
    ConditionVariable_signal__(&pThis->threadEncoder.cond); /* triger encoding in encoding thread */
    
    printf("\n");
    /* show frame to display */
    if (pThis->preview == TRUE &&
       (pThis->ve.jpegdec.decode == FALSE || pThis->ve.h264dec.decode == FALSE)) {
        /* initialize display */
        if (pThis->disp.layer[DL_RAW].initialized == FALSE) {
            //pThis->disp.format = DISP_FORMAT_YUV420; //DISP_FORMAT_YUV420 //DISP_FORMAT_YUV422
            //pThis->disp.seq=DISP_SEQ_UVUV;
            if (Disp_init__(&pThis->disp.layer[DL_RAW], DISP_FORMAT_YUV420, 
                DISP_MOD_NON_MB_UV_COMBINED, DISP_SEQ_UVUV, pThis->camera.size.width, 
                pThis->camera.size.height, DISP_DL_RAW_SCN_POS_X, DISP_DL_RAW_SCN_POS_Y, 
                DISP_DL_RAW_SCN_WIDTH, DISP_DL_RAW_SCN_HEIGHT) == 0) {
                Disp_start__(&pThis->disp.layer[DL_RAW]);
                Disp_on__();
                pThis->disp.layer[DL_RAW].initialized = TRUE;
                Disp_set_addr__(&pThis->disp.layer[DL_RAW], pThis->camera.size.width, 
                                 pThis->camera.size.height, (int *)&buf.m.offset);
            }
        } else {
            Disp_set_addr__(&pThis->disp.layer[DL_RAW], pThis->camera.size.width, 
                             pThis->camera.size.height, (int *)&buf.m.offset);
        }
        DBG("DispRawCamera: buffIndex_%d", buf.index);
    }
	
    /* query new frame for this buffer */
    Camera_queryFrame__(buf.index);
	return 1;
}

static void Tvin2jpeg_processImage__ (const void *p, int picSize) {
    
    static int      id = 0;
    FILE           *pFile;
    char            fname[100];
    char           *pFrame = (char*)p;
    char           *pFrameFromFile = NULL;
    uint32_t        frameSize = (uint32_t)picSize;
    
    if (pThis->output_marks) {
        fprintf(stderr, "#");
    }

    /* compress frame */
    if (pThis->compressCount > 0) {
        pThis->compressCount--;
        /* get frame from file */
        if (pThis->readFile == TRUE) {
            if ((pFrameFromFile = Tvin2Jpeg_readFile__(pThis->pFileName, &frameSize)) != NULL) {
                pFrame = pFrameFromFile;
            }
        }
        
        /* encode frame to jpeg */
        if (pThis->jpegEnc.encode == TRUE) {
            JpegEnc_encodePicture__(pFrame, frameSize);
        }
        
        /* encode to h264 */
        if (pThis->ve.h264enc.encode == TRUE) {
                                    /* copy frame to input buffers if hw jpeg encoder is not used */
            if (pThis->jpegEnc.encode == FALSE || pThis->jpegEnc.type != HW) {
                memcpy(pThis->ve.pLumaSrc, pFrame, frameSize);
            }
            
            if (H264enc_encodePicture__()) {
                write(pThis->ve.h264enc.file, pThis->ve.h264enc.pBytestreamBuffer, 
                      pThis->ve.h264enc.bytestreamLength);
                Mux_writeData(pThis->ve.h264enc.pBytestreamBuffer, 
                    pThis->ve.h264enc.bytestreamLength, pThis->ve.h264enc.frameNum ? 0 : 1);
                DBG("EncodeH264.. Id: %d", pThis->ve.h264enc.id++);
            } else {
                printf("ERROR: h264 encoding!\n");
            }
        }
        
        /* trigger decoding JPEG decoder thread if JPEG encoding is also enabled */
        if (pThis->jpegEnc.encode == TRUE && pThis->ve.jpegdec.decode == TRUE) {
            ConditionVariable_signal__(&pThis->threadJpegDecoder.cond);
        }
        
        /* trigger decoding h264 decoder thread */
        if (pThis->ve.h264dec.decode == TRUE) {
            ConditionVariable_signal__(&pThis->threadH264Decoder.cond);
        }
        
        if (pThis->readFile == TRUE) {
            if (pFrameFromFile != NULL) {
                free(pFrameFromFile);
            }
        }
    }
    
    /* save raw frame */
    if (pThis->rawCount > 0) {
        pThis->rawCount--;
        snprintf(fname, sizeof(fname), "/tmp/frame_%03d.raw", id);
        pFile = fopen(fname, "w");
        if (pFile != NULL) {
            fwrite(p, picSize, 1, pFile);
            fclose(pFile);
        }
        
    }
    
    id++;
    if (pThis->output_marks)
        fprintf(stderr, ".");
 }

static char* Tvin2Jpeg_readFile__ (char *pName, uint32_t *readSize) {
    
    FILE   *pFile = NULL;
    long    lSize;
    char   *pBuffer = NULL;
    size_t  nRead;
        
    pFile = fopen(pName, "rb");
    if (pFile != NULL) {
        /* obtain file size */
        fseek (pFile , 0 , SEEK_END);
        if ((lSize = ftell(pFile)) <= 0) {
            goto out;
        }
        rewind (pFile);
        /* allocate memory to contain the whole file */
        pBuffer = (char*) malloc ((sizeof(char))*lSize);
        if (pBuffer == NULL) {
            goto out;
        }
        /* copy the file into the buffer */
        nRead = fread(pBuffer, 1, lSize, pFile);
        if (nRead != lSize) {
            goto out;
        }
        *readSize = nRead;
        /* the whole file is now loaded in the memory buffer. */
        fclose(pFile);
        return pBuffer;
    } else {
        DBG("Open file ERROR!");
    }

out:
    if (pFile != NULL) {
        fclose(pFile);
    }
    return NULL;
}

static void Tvin2Jpeg_handleInt__ (int n) {

    pThis->run = FALSE;
}

static void Tvin2jpeg_initVars__ (void) {
    //camera
    pThis->camera.deviceName = CAMERA_DEVICE;
    pThis->camera.fd = -1;
    pThis->camera.n_buffers = 0;
    CLEAR(pThis->camera.buffers);
    //display
    pThis->disp.layer[DL_RAW].initialized = FALSE;
    pThis->disp.layer[DL_JPEG].initialized = FALSE;
    pThis->disp.layer[DL_H264].initialized = FALSE;
    pThis->disp.fd = -1;
    pThis->disp.fb_fd = -1;
    pThis->frameCount = -1;
    pThis->compressCount = 0;
    pThis->rawCount = 0;
    pThis->output_marks = FALSE;
    pThis->preview = FALSE;
    pThis->readFile = FALSE;
    pThis->pFileName = NULL;
    pThis->yuvToRgb = FALSE;
    //jpeg encoder
    pThis->jpegEnc.encode = FALSE;
    pThis->jpegEnc.jpegEncQuality = JPEG_ENC_QUALITY;
    pThis->jpegEnc.tj.jpegSize = 0;
    pThis->jpegEnc.tj.pCompressedImage = NULL;
    pThis->jpegEnc.hwj.JpegBuff = NULL;
    pThis->jpegEnc.hwj.Jwritten = 0;
    //VE - Video Engine
    pThis->ve.pRegs = NULL;
    pThis->ve.pLumaSrc = NULL;
    pThis->ve.pChromaSrc = NULL;
    //
    pThis->run = TRUE;
    pThis->maxTimeForOneFrame = 0;
    pThis->minTimeForOneFrame = 100.00;
    pThis->ve.jpegdec.maxTimeForOneFrame = 0;
    pThis->ve.jpegdec.minTimeForOneFrame = 100.00;
}

//camera
static void Camera_init__ (void) {
    
    struct stat st;
    
	pThis->camera.format = V4L2_PIX_FMT_NV12;//format=V4L2_PIX_FMT_NV12;//V4L2_PIX_FMT_NV16
	struct v4l2_format fmt;
	struct v4l2_format fmt_priv;
	
    /* open camera device */
    if (stat(pThis->camera.deviceName, &st) == -1) {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                pThis->camera.deviceName, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
    if (!S_ISCHR(st.st_mode)) {
        fprintf(stderr, "%s is no device\n", pThis->camera.deviceName);
        exit(EXIT_FAILURE);
    }
	pThis->camera.fd = open (pThis->camera.deviceName, O_RDWR /* required */ | O_NONBLOCK, 0);
    if (pThis->camera.fd == -1) {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                pThis->camera.deviceName, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
	
    /* set camera format */
	CLEAR (fmt_priv);
	fmt_priv.type                = V4L2_BUF_TYPE_PRIVATE;
	fmt_priv.fmt.raw_data[0] = 0;//0 interface composite CVBS;//1 TVD_YPBPR_I;//2 TVD_YPBPR_P
	fmt_priv.fmt.raw_data[1] = 1;//0 system NTSC 480i;//1 system PAL 576i
	fmt_priv.fmt.raw_data[2] = 0;//format 1=mb (macroblocks), for test only (0->720 non MB, 1->704 MB)
	fmt_priv.fmt.raw_data[8] = 1;//row, num rows
	fmt_priv.fmt.raw_data[9] = 1;//column, num columns
	fmt_priv.fmt.raw_data[10] = 1;//channel_index
	fmt_priv.fmt.raw_data[11] = 0;//2;//channel_index
	fmt_priv.fmt.raw_data[12] = 0;//3;//channel_index
	fmt_priv.fmt.raw_data[13] = 0;//4;//channel_index
	if (ioctl(pThis->camera.fd, VIDIOC_S_FMT, &fmt_priv) == -1) {
		DBG("VIDIOC_S_FMT error!");
		exit(EXIT_FAILURE); 
	}
	
	//CLEAR (fmt);
	//fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//fmt.fmt.pix.width       = 720*fmt_priv.fmt.raw_data[8];//720; 
	//fmt.fmt.pix.height      = 576*fmt_priv.fmt.raw_data[9];//576;//480;
	//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;//V4L2_PIX_FMT_YUV422P;//V4L2_PIX_FMT_NV12;//V4L2_PIX_FMT_YUYV;
	//fmt.fmt.pix.field       = V4L2_FIELD_ANY;//V4L2_FIELD_NONE //V4L2_FIELD_ANY
	//if (ioctl(pThis->camera.fd, VIDIOC_S_FMT, &fmt) == -1)
	//{
		//printf("VIDIOC_S_FMT error! b\n");
		//exit(EXIT_FAILURE);
	//}
	
    /* get camera format */
	usleep(100000);//delay 100ms if you want to check the status after set fmt
    CLEAR (fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(pThis->camera.fd, VIDIOC_G_FMT, &fmt) == -1) {
		printf("VIDIOC_G_FMT error!  a V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		exit(EXIT_FAILURE);
	}
    pThis->camera.size.width = fmt.fmt.pix.width;
    pThis->camera.size.height = fmt.fmt.pix.height;
    pThis->camera.bytesperline = fmt.fmt.pix.bytesperline;
    pThis->camera.rawSize = fmt.fmt.pix.sizeimage;
    
    //pThis->disp.mode=fmt_priv.fmt.raw_data[2]?DISP_MOD_MB_UV_COMBINED:DISP_MOD_NON_MB_UV_COMBINED;//DISP_MOD_NON_MB_UV_COMBINED DISP_MOD_MB_UV_COMBINED
	//pThis->camera.size.width = fmt_priv.fmt.raw_data[8]*(fmt_priv.fmt.raw_data[2]?704:720);//width
	//pThis->camera.size.height = fmt_priv.fmt.raw_data[9]*(fmt_priv.fmt.raw_data[1]?576:480);//height
	printf("pThis->camera.size.width=%d\n", pThis->camera.size.width);
	printf("pThis->camera.size.height=%d\n", pThis->camera.size.height);
    
	CLEAR (fmt_priv);
    fmt_priv.type = V4L2_BUF_TYPE_PRIVATE;
	if (ioctl(pThis->camera.fd, VIDIOC_G_FMT, &fmt_priv) == -1) {
		printf("VIDIOC_G_FMT error!  a TYPE_VIDEO_CAPTURE\n");
		exit(EXIT_FAILURE);
	}
	printf("interface=%d\n", fmt_priv.fmt.raw_data[0]);
	printf("system=%d\n", fmt_priv.fmt.raw_data[1]);
	printf("format=%d\n", fmt_priv.fmt.raw_data[2]);
	printf("row=%d\n", fmt_priv.fmt.raw_data[8]);
	printf("column=%d\n", fmt_priv.fmt.raw_data[9]);
	printf("channel_index[0]=%d\n", fmt_priv.fmt.raw_data[10]);
	printf("channel_index[1]=%d\n", fmt_priv.fmt.raw_data[11]);
	printf("channel_index[2]=%d\n", fmt_priv.fmt.raw_data[12]);
	printf("channel_index[3]=%d\n", fmt_priv.fmt.raw_data[13]);
	printf("status[0]=%d\n", fmt_priv.fmt.raw_data[16]);
	printf("status[1]=%d\n", fmt_priv.fmt.raw_data[17]);
	printf("status[2]=%d\n", fmt_priv.fmt.raw_data[18]);
	printf("status[3]=%d\n", fmt_priv.fmt.raw_data[19]);
    
    /* calculate source planes size */
    pThis->camera.YplaneSize = tjPlaneSizeYUV(0, pThis->camera.size.width, 0, 
                                                 pThis->camera.size.height, TJSAMP_420);
    pThis->camera.CplaneSize = (tjPlaneSizeYUV(1, pThis->camera.size.width, 0, 
                                                  pThis->camera.size.height, TJSAMP_420)) << 1;
    /* calculate macroblocks size */
    pThis->camera.mb_width = DIV_ROUND_UP(pThis->camera.size.width, 16);
    pThis->camera.mb_height = DIV_ROUND_UP(pThis->camera.size.height, 16);
    pThis->camera.mb_stride = pThis->camera.size.width / 16;
    pThis->camera.crop_right = (pThis->camera.mb_width * 16 - pThis->camera.size.width) / 2;
    pThis->camera.crop_bottom = (pThis->camera.mb_height * 16 - pThis->camera.size.height) / 2;

    printf("Picture Info from VIDIOC_G_FMT: %dx%d, bytesperline: %d, rawSize: %ld\n", 
                pThis->camera.size.width, pThis->camera.size.height, 
                pThis->camera.bytesperline, pThis->camera.rawSize);
}

static void Camera_close__ (void) {

    if (pThis->camera.fd >= 0) {
        close (pThis->camera.fd);     /* close camera source driver /dev/video1 */
    }
}

static void Camera_mapBuffers__ (void) {

    struct v4l2_requestbuffers req;
    
    /* request buffers */
	CLEAR (req);
	req.count               = N_BUFFERS;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;
	//ioctl(pThis->camera.fd, VIDIOC_REQBUFS, &req);
    if (xioctl(pThis->camera.fd, VIDIOC_REQBUFS, &req) == -1) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support "
                    "memory mapping\n", pThis->camera.deviceName);
            exit(EXIT_FAILURE);
        } else {
            errno_exit__("VIDIOC_REQBUFS");
        }
    }
	//buffers = calloc (req.count, sizeof (*buffers));
	for (pThis->camera.n_buffers = 0; pThis->camera.n_buffers < req.count; ++pThis->camera.n_buffers) {
        struct v4l2_buffer buf;
        
        CLEAR (buf);
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = pThis->camera.n_buffers;

        if (ioctl(pThis->camera.fd, VIDIOC_QUERYBUF, &buf) == -1) {
            printf("VIDIOC_QUERYBUF error\n");
            exit(EXIT_FAILURE);
        }
        pThis->camera.buffers[pThis->camera.n_buffers].length = buf.length;
        pThis->camera.buffers[pThis->camera.n_buffers].start  = mmap (NULL /* start anywhere */,
                                         buf.length,
                                         PROT_READ | PROT_WRITE /* required */,
                                         MAP_SHARED /* recommended */,
                                         pThis->camera.fd, buf.m.offset);
        printf("MMAP %d: %p OFF: %p\n", pThis->camera.n_buffers, 
                    pThis->camera.buffers[pThis->camera.n_buffers].start, (void *)buf.m.offset);

        if (MAP_FAILED == pThis->camera.buffers[pThis->camera.n_buffers].start) {
            printf("mmap failed\n");
            errno_exit__("mmap");
        }
	}
}

static void Camera_unmapBuffers__ (void) {

    uint32_t    i;
    
	for (i = 0; i < pThis->camera.n_buffers; ++i) {
		if (munmap (pThis->camera.buffers[i].start, pThis->camera.buffers[i].length) == -1) {
			printf("munmap error");
		}
	}
}

static void Camera_streamOn__ (void) {
    
    uint32_t    i;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_buffer buf;
    
    /* query all buffers */
	for (i = 0; i < pThis->camera.n_buffers; ++i) {
		CLEAR (buf);		
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = i;

		if (xioctl(pThis->camera.fd, VIDIOC_QBUF, &buf) == -1) {
			printf("VIDIOC_QBUF failed\n");
            exit(EXIT_FAILURE);
        } else {
            pThis->camera.buffers[i].queued = TRUE;
        }
	}
    
    /* start stream */
	if (ioctl(pThis->camera.fd, VIDIOC_STREAMON, &type) == -1) {
		printf("VIDIOC_STREAMON failed\n");
        exit(EXIT_FAILURE);
    }
    
}

static void Camera_streamOff__ (void) {

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
	if (ioctl(pThis->camera.fd, VIDIOC_STREAMOFF, &type) == -1) {
		printf("VIDIOC_STREAMOFF failed\n");
    }
}

static void Camera_queryFrame__ (uint32_t bufIdx) {

    struct v4l2_buffer buf;
    
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = bufIdx;

    /* query new frame for this buffer */
    if (xioctl(pThis->camera.fd, VIDIOC_QBUF, &buf) == -1) {
        errno_exit__("VIDIOC_QBUF failed!");
    } else {
        pThis->camera.buffers[bufIdx].queued = TRUE;
    }
}

//Display
static int Disp_init__(DisplayLayer_t *pLayer, __disp_pixel_fmt_t format, __disp_pixel_mod_t mode, 
                       __disp_pixel_seq_t seq, uint32_t srcWidth, uint32_t srcHeight, 
                       uint32_t outX, uint32_t outY, uint32_t outWidth, const int outHeight) {
    
    uint32_t args[4] = { 0, DISP_LAYER_WORK_MODE_SCALER, 0, 0 };
    
    pThis->disp.sel = 0;                                                      /* which screen 0/1 */
    /* open display if it is not already opened */
    if (pThis->disp.fd < 0) {
    	if((pThis->disp.fd = open(DISPLAY_DEVICE, O_RDWR)) == -1) {        /* open display device */
            DBG("Display %s open fail.", DISPLAY_DEVICE);
            return -1;
        }
        
        int tmp = SUNXI_DISP_VERSION;
        if (ioctl(pThis->disp.fd, DISP_CMD_VERSION, &tmp) < 0) {
            close(pThis->disp.fd);
            pThis->disp.fd = -1;
            return -1;
        }
        
        __disp_colorkey_t ck;
        ck.ck_max.red = ck.ck_min.red = 0;
        ck.ck_max.green = ck.ck_min.green = 1;
        ck.ck_max.blue = ck.ck_min.blue = 2;
        ck.red_match_rule = 2;
        ck.green_match_rule = 2;
        ck.blue_match_rule = 2;
        args[1] = (unsigned long)(&ck);
        ioctl(pThis->disp.fd, DISP_CMD_SET_COLORKEY, args);
    }
    
    /* request layer *///layer0
    args[0] = 0; //sel
    args[1] = DISP_LAYER_WORK_MODE_SCALER;   //mode
    pLayer->layerId = ioctl(pThis->disp.fd, DISP_CMD_LAYER_REQUEST, (void*)args);
    if(pLayer->layerId == 0) {
        DBG("request layer0 fail");
        close(pThis->disp.fd);
        pThis->disp.fd = -1;
        return -1;
    } else {
        DBG("Display: get layer id: %d", pLayer->layerId);
    }

    memset(&pLayer->layerPara, 0, sizeof(pLayer->layerPara));
    pLayer->layerPara.pipe = 1;
    pLayer->layerPara.mode = DISP_LAYER_WORK_MODE_SCALER;
    //pLayer->layerPara.pipe = 0; 
    pLayer->layerPara.fb.addr[0]       = 0;//your Y address,modify this 
    pLayer->layerPara.fb.addr[1]       = 0; //your C address,modify this 
    pLayer->layerPara.fb.addr[2]       = 0; 
    pLayer->layerPara.fb.size.width    = srcWidth;
    pLayer->layerPara.fb.size.height   = srcHeight;
    pLayer->layerPara.fb.mode          = mode;///DISP_MOD_INTERLEAVED;//DISP_MOD_NON_MB_PLANAR;//DISP_MOD_NON_MB_UV_COMBINED;
    pLayer->layerPara.fb.format        = format;//DISP_FORMAT_YUV420;//DISP_FORMAT_YUV422;//DISP_FORMAT_YUV420;
    pLayer->layerPara.fb.br_swap       = 0;
    pLayer->layerPara.fb.seq           = seq;//DISP_SEQ_UVUV;//DISP_SEQ_YUYV;//DISP_SEQ_YVYU;//DISP_SEQ_UYVY;//DISP_SEQ_VYUY//DISP_SEQ_UVUV
    pLayer->layerPara.fb.cs_mode       = DISP_BT601; //DISP_YCC //DISP_BT601
    pLayer->layerPara.ck_enable        = 1;                                                         //todo PROVERITI STA JE OVO.. KOD VDPAU JE setovano na 1
    pLayer->layerPara.alpha_en         = 1; 
    pLayer->layerPara.alpha_val        = 0xff;
    pLayer->layerPara.src_win.x        = 0;
    pLayer->layerPara.src_win.y        = 0;
    pLayer->layerPara.src_win.width    = srcWidth;
    pLayer->layerPara.src_win.height   = srcHeight;
    pLayer->layerPara.scn_win.x        = outX;
    pLayer->layerPara.scn_win.y        = outY;
    pLayer->layerPara.scn_win.width    = outWidth;
    pLayer->layerPara.scn_win.height   = outHeight;
    
    if (pLayer->layerPara.scn_win.y < 0) {
        int scn_clip = -(pLayer->layerPara.scn_win.y);
        int src_clip = scn_clip * pLayer->layerPara.src_win.height / pLayer->layerPara.scn_win.height;
		pLayer->layerPara.src_win.y += src_clip;
		pLayer->layerPara.src_win.height -= src_clip;
		pLayer->layerPara.scn_win.y = 0;
		pLayer->layerPara.scn_win.height -= scn_clip;
	}
    
	args[0] = pThis->disp.sel;
    args[1] = pLayer->layerId;
    args[2] = (__u32)&pLayer->layerPara;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_SET_PARA, (void*)args);
#if 0
    args[0] = pThis->disp.sel;
    args[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_TOP, (void*)args);
#endif
#if 0
    args[0] = pThis->disp.sel;
    args[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_BOTTOM, args);
#endif
    args[0] = pThis->disp.sel;
    args[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_OPEN, (void*)args);

#if 1
    if (pThis->disp.fb_fd < 0) {
        unsigned long fb_layer;
        //void *addr = NULL;
        pThis->disp.fb_fd = open("/dev/fb0", O_RDWR);
        if (ioctl(pThis->disp.fb_fd, FBIOGET_LAYER_HDL_0, &fb_layer) == -1) {
            DBG("get fb layer handel");	
        }
        close(pThis->disp.fb_fd);
        args[0] = 0;
        args[1] = fb_layer;
        ioctl(pThis->disp.fd, DISP_CMD_LAYER_BOTTOM, (void *)args);
    }
#endif
	return 0;
}

static void Disp_start__ (DisplayLayer_t *pLayer) {
    
    __u32 arg[4];
    
    arg[0] = pThis->disp.sel;
    arg[1] = (__u32)6291462;
    arg[2] = (__u32)&(pLayer->layerPara.scn_win);
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_SET_SCN_WINDOW, (void*)arg);
    
	arg[0] = pThis->disp.sel;
    arg[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_VIDEO_START,  (void*)arg);
}

static void Disp_stop__ (DisplayLayer_t *pLayer) {
    
    __u32 arg[4];
    
	arg[0] = pThis->disp.sel;
    arg[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_VIDEO_STOP,  (void*)arg);
}

static int Disp_on__ (void) {
    
    __u32 arg[4];
    
	arg[0] = 0;
    ioctl(pThis->disp.fd, DISPLAY_ON, (void*)arg);
    
    return 0;
}

static int Disp_layerRelease__ (DisplayLayer_t *pLayer) {

	__u32 arg[4];
	arg[0] = 0;

    arg[0] = pThis->disp.sel;
    arg[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_CLOSE,  (void*)arg);

    arg[0] = pThis->disp.sel;
    arg[1] = pLayer->layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_RELEASE,  (void*)arg);
    
    return 0;
}

static int Disp_exit__ (void) {
    
    //ioctl(pThis->disp.fd, DISPLAY_OFF, (void*)arg);
    close (pThis->disp.fd);
    
    return 0;
}

static int Disp_set_addr__ (DisplayLayer_t *pLayer, int w, int h, int *addr) {
    
    __u32 arg[4];
    
#if 0
	pLayer->layerPara.fb.addr[0]       = *addr;//your Y address,modify this 
    pLayer->layerPara.fb.addr[1]       = *addr+w*h; //your C address,modify this 
    pLayer->layerPara.fb.addr[2]       = *addr+w*h*3/2; 
    
    arg[0] = pThis->disp.sel;
    arg[1] = pLayer->layerId;
    arg[2] = (__u32)&pLayer->layerPara;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_SET_PARA,(void*)arg);
#endif

	__disp_video_fb_t  fb_addr;	
	memset(&fb_addr, 0, sizeof(__disp_video_fb_t));

	fb_addr.interlace       = 1;///////////////////////////////////////////////////////
	fb_addr.top_field_first = 0;
	fb_addr.frame_rate      = 25;
	fb_addr.addr[0] = *addr;
//	fb_addr.addr[1] = *addr + w * h;
//	fb_addr.addr[2] = *addr + w*h*3/2;
	
	
	switch(pThis->camera.format){
		case V4L2_PIX_FMT_YUV422P:
	    case V4L2_PIX_FMT_YUYV:
	    case V4L2_PIX_FMT_YVYU:
	    case V4L2_PIX_FMT_UYVY:
	    case V4L2_PIX_FMT_VYUY:		
	    	fb_addr.addr[1]       = *addr+w*h; //your C address,modify this 
	    	fb_addr.addr[2]       = *addr+w*h*3/2; 
	    	break;
	    case V4L2_PIX_FMT_YUV420:
	    	fb_addr.addr[1]       = *addr+w*h; //your C address,modify this 
	    	//fb_addr.addr[2]       = *addr+w*h*5/4;//todo vratiti ako treba.. mada za uv kombinaciju ovo je nepotrebno
	    	break;
	    case V4L2_PIX_FMT_NV16:
	    case V4L2_PIX_FMT_NV12:	
	    case V4L2_PIX_FMT_HM12:	
	    	fb_addr.addr[1]       = *addr+w*h; //your C address,modify this 
	    	//fb_addr.addr[2]       = pLayer->layerPara.fb.addr[1];
	    	break;
	    
	    default:
	    	DBG("format is not found!");
	    	break;
    
  	}
  	
  	fb_addr.id = 0;  //TODO
    arg[0] = pThis->disp.sel;
    arg[1] = pLayer->layerId;
    arg[2] = (__u32)&fb_addr;
    if (ioctl(pThis->disp.fd, DISP_CMD_VIDEO_SET_FB, (void*)arg) == -1) {
        DBG(KRED"ERROR set: DISP_CMD_VIDEO_SET_FB"KNRM);
    }
    
    return 0;
}

static int Disp_newDecoder_frame__(DisplayLayer_t *pLayer, int w, int h, const uint32_t luma_buffer, 
                                    const uint32_t chroma_buffer, const int id) {
    uint32_t            args[4];
	__disp_video_fb_t   fb_addr;
    
	memset(&fb_addr, 0, sizeof(__disp_video_fb_t));
	fb_addr.id = id;
	fb_addr.frame_rate = 25;
    fb_addr.addr[0] = luma_buffer + DRAM_OFFSET;
	fb_addr.addr[1] = chroma_buffer + DRAM_OFFSET;
	//fb_addr.addr[0] = *pLuma_buffer + DRAM_OFFSET;
	//fb_addr.addr[1] = *pChroma_buffer+w*h + DRAM_OFFSET;
    //fb_addr.addr[0] = *pLuma_buffer;
	//fb_addr.addr[1] = *pChroma_buffer+w*h;
    //fb_addr.addr[2] = *pChroma_buffer + ((w*h)/4);
    fb_addr.id = 0;
	args[0] = 0;
	args[1] = pLayer->layerId;
	args[2] = (unsigned long)(&fb_addr);
	args[3] = 0;

    ioctl(pThis->disp.fd, DISP_CMD_VIDEO_SET_FB, (void*)args);
	pLayer->lastId = id;

	return 1;
}

//libturbojpeg
static void Tj_init__ (void) {

    if ((pThis->jpegEnc.tj.jpegCompressor = tjInitCompress()) == NULL) {
        DBG("tjInitCompress error '%s'\n",  tjGetErrorStr());
        exit(EXIT_FAILURE);
    }
}

static void Tj_close__ (void) {

    if (pThis->jpegEnc.tj.jpegCompressor != NULL) {
        tjDestroy(pThis->jpegEnc.tj.jpegCompressor);
    }
    if (pThis->jpegEnc.tj.pCompressedImage != NULL) {
        tjFree(pThis->jpegEnc.tj.pCompressedImage);
    }
}

//jpeg enc
static void JpegEnc_encodePicture__ (const void *pFrame, int frameSize) {
    
    int                 retval;
    int                 flags = TJFLAG_FASTDCT;
    unsigned long       yuvBuffSize;
    long unsigned int   _jpegSize;
    unsigned char      *pJpegImage = NULL;
    int                 pf = TJPF_RGB;
    FILE               *pFile;
    char                fname[100];
    
    if (pThis->jpegEnc.type == SW) {
        /* compress image to jpeg by software */
        _jpegSize = pThis->jpegEnc.tj.jpegSize;
        
        if (pThis->yuvToRgb == TRUE) {
            /* convert YUV_4:2:2_YUYV_YUY2 interleaved -> RGB and than compress image from 
             * rgb to jpeg */
            char           *pRgbBuf=NULL;
            unsigned long   rgbSize;
            
            /* allocate buffer for rgb image */
            rgbSize = pThis->camera.size.width * pThis->camera.size.height * tjPixelSize[pf];
            if ((pRgbBuf=(char*)malloc(rgbSize)) == NULL) {
                DBG("ERROR: Memory allocation failure: %d", __LINE__);
                goto bailout;
            }
            memset(pRgbBuf, 0, rgbSize);
            /* convert */
            yuvBuffSize = tjBufSizeYUV2(pThis->camera.size.width, PADDING, 
                                        pThis->camera.size.height, TJSAMP_422);
            yuv422YUYV_YUY2_2rgb__((char*)pFrame, pRgbBuf, yuvBuffSize);
            /* compress */
            retval = tjCompress2(pThis->jpegEnc.tj.jpegCompressor, (unsigned char*)pRgbBuf, 
                        pThis->camera.size.width, 0, pThis->camera.size.height, TJPF_RGB, 
                        &pThis->jpegEnc.tj.pCompressedImage, &_jpegSize, TJSAMP_444,
                        pThis->jpegEnc.jpegEncQuality, TJFLAG_FASTDCT);
            printf("Convert YUV 4:2:2 YUYV_YUY2 => RGB 4:4:4 => jpeg\n");
            pJpegImage = pThis->jpegEnc.tj.pCompressedImage;
        } else {
            /* compress direct from YUV_4:2:0_NV12 to jpeg */
            const unsigned char *srcPlanes[3];
            int pw0, ph0, pw1, ph1, strides[3];
            unsigned char *pPicture = NULL;
            unsigned char *pChromaSrc = NULL;
            unsigned char *pChromaBdst = NULL;
            unsigned char *pChromaRdst = NULL;
            long unsigned int   i;
            
            yuvBuffSize = tjBufSizeYUV2(pThis->camera.size.width, PADDING, 
                                        pThis->camera.size.height, TJSAMP_420);
            printf("YUV Size: %dx%d: %ld:%d -> YUV: %s , Y:%d, C:%d*2\n", 
                    pThis->camera.size.width, pThis->camera.size.height, yuvBuffSize, frameSize, 
                   subNameLong[TJSAMP_420], pThis->camera.YplaneSize, pThis->camera.CplaneSize);
                
            pw0=tjPlaneWidth(0, pThis->camera.size.width, TJSAMP_420);
            ph0=tjPlaneHeight(0, pThis->camera.size.height, TJSAMP_420);
            strides[0]=PAD(pw0, PADDING);
            pw1=tjPlaneWidth(2, pThis->camera.size.width, TJSAMP_420);
            ph1=tjPlaneHeight(2, pThis->camera.size.height, TJSAMP_420);
            strides[1]=strides[2]=PAD(pw1, PADDING);
            
            //NOTE: Because libjpegturbo currently does not support YUV_4:2:0_NV12 repack it to
            //YUV_4:2:0_I420 format.
            
            //4:2:0 NV12 format example picture block 8x4 pixels:
            //               "yyyy YYYY"    //1 line luma
            // |xxxxxxxx|    "uvuv UVUV"    //1-2 line chroma
            // |xxxxxxxx| => "yyyy YYYY"    //2 line luma
            // |xxxxxxxx|    "YYYY yyyy"    //3 line luma
            // |xxxxxxxx|    "UVUV uvuv"    //3-4 line chroma
            //               "YYYY yyyy     //4 line luma
            // It has 32 Luma components and 16 chroma components (8 U and 8 V)
            // NV12 buffer non mb UV combined nas 2 address and looks like: 
            //"YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYUVUVUVUVUVUVUVUV"
            // I420 buffer non mb planar has 3 address and looks like: 
            //"YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYUUUUUUUUVVVVVVVV"
            
            /* convert NV12 to I420 */
            if ((pPicture = (unsigned char *)malloc(frameSize)) == NULL) {
                DBG("ERROR: Memory allocation failure: %d", __LINE__);
                goto bailout;
            }
            memset(pPicture, 0, frameSize);
            DBG("test");
            memcpy(pPicture, pFrame, pThis->camera.YplaneSize);
            pChromaSrc = (unsigned char *)(pFrame + pThis->camera.YplaneSize);
            pChromaBdst = pPicture + pThis->camera.YplaneSize;
            pChromaRdst = pChromaBdst + (pThis->camera.CplaneSize>>1);
            for (i = 0; i < (pThis->camera.CplaneSize>>1); i++) {
                *pChromaBdst++ = *pChromaSrc++;
                *pChromaRdst++ = *pChromaSrc++;
            }
            DBG("test1");
            srcPlanes[0] = (const unsigned char*)pPicture;
            srcPlanes[1]=srcPlanes[0]+strides[0]*ph0;
            srcPlanes[2]=srcPlanes[1]+strides[1]*ph1;
            printf("Info: strides[0]:%d, strides[1]:%d, strides[2]:%d, pw0:%d, ph0:%d, pw1:%d, "
                   "ph1:%d, SrcPlanes[0]:%p, SrcPlanes[1]:%p, SrcPlanes[2]:%p ",
                    strides[0], strides[1], strides[2], pw0, ph0, pw1, ph1, srcPlanes[0], 
                    srcPlanes[1], srcPlanes[2]);
            
            retval = tjCompressFromYUVPlanes(pThis->jpegEnc.tj.jpegCompressor, srcPlanes, 
                      pThis->camera.size.width, strides, pThis->camera.size.height, 
                      TJSAMP_420, &pThis->jpegEnc.tj.pCompressedImage, &_jpegSize,
                      pThis->jpegEnc.jpegEncQuality, flags);
            if (pPicture) {
                free(pPicture);
            }
            pJpegImage = pThis->jpegEnc.tj.pCompressedImage;
        }
        if (retval != 0) {
            printf("tjCompress [ERROR] '%s'\n",  tjGetErrorStr());
        } else {
            printf("tjCompress [OK]!");
        }
        pThis->jpegEnc.tj.jpegSize = pThis->jpegEnc.tj.jpegSize >= _jpegSize ? 
                                                         pThis->jpegEnc.tj.jpegSize : _jpegSize;
        
        /* save image to file */
        if (pThis->output_marks) {
            fprintf(stderr, "#");
        }
        /* write to file */
        {
        CriticalSectionEnter__(&pThis->threadEncoder.mutex);
        uint32_t    id = pThis->jpegEnc.id;
        CriticalSectionExit__(&pThis->threadEncoder.mutex);
        snprintf(fname, sizeof(fname), "/tmp/testImage_%03d.jpg", id);
        printf("Save Image: %s -> size: %ld, width: %d, height: %d Bytesperline: %d\n", fname, 
                            _jpegSize, pThis->camera.size.width, pThis->camera.size.height, 
                            pThis->camera.bytesperline);
        if (pJpegImage != NULL) {
            pFile = fopen(fname, "w");
            if (pFile != NULL) {
                fwrite(pJpegImage, _jpegSize, 1, pFile);
                fclose(pFile);
            }
        }
        }
    } else if (pThis->jpegEnc.type == HW) {  /* ***** ***** HARDWARE VIDEO ENGINE ***** ***** */
        Ve_selectSubengine__(pThis, SUBENG_JPEG_ENC);         /* select AVC-JPEG Encoder, lock HW */
        /* compress image to jpeg by hardware */
        // flush output buffer, otherwise we might read old cached data
        ve_flush_cache(pThis->jpegEnc.hwj.JpegBuff, MAX_JPEG_SIZE);
        Ve_fillInputBuffers__(pFrame, frameSize);
        Ve_veisp_initPicture__(pThis->camera.mb_width, pThis->camera.mb_height,
                               pThis->camera.mb_stride, pThis->ve.isp.colorFormat);
        Ve_veisp_setInputBuffers__(pThis->ve.pLumaSrc, pThis->ve.pChromaSrc);

        if (pThis->ve.isp.scaler.type > 0) {                       /* use arbitrary scaler engine */
            //DBG("Use arbitrary scaler");
            veisp_set_scaler(pThis->ve.isp.scaler.size.width, pThis->ve.isp.scaler.size.height, 
                  0.0, 0.0, pThis->ve.isp.scaler.xScaleFactor, pThis->ve.isp.scaler.yScaleFactor);
        }
        
        /* set output buffer */
                                                      /* set buffer for encoded picture frame */
        veavc_init_vle(pThis->jpegEnc.hwj.JpegBuff, MAX_JPEG_SIZE); 
        veavc_init_ctrl(VEAVC_ENCODER_MODE_JPEG);  /* enable IRQ from AVC, enable jpeg triger */
        veavc_jpeg_parameters(1, 0, 0, 0);                               /* set avc jpeg ctrl */
        /* create jpeg header */
        vejpeg_header_create(pThis->ve.isp.scaler.size.width, 
                            pThis->ve.isp.scaler.size.height, pThis->jpegEnc.jpegEncQuality);
        vejpeg_write_SOF0();
        vejpeg_write_SOS();
        vejpeg_write_quantization();
        /* launch encoding */
        //printf("[JEPOC] launch encoding.\n");
        Ve_trigerSubebgine__(SUBENG_JPEG_ENC);
        pThis->jpegEnc.hwj.Jwritten = veavc_get_written();
        ve_flush_cache(pThis->jpegEnc.hwj.JpegBuff, MAX_JPEG_SIZE);          /* flush for A20 */
        
        /* save image to file */
        if (pThis->output_marks) {
            fprintf(stderr, "#");
        }
        {
        CriticalSectionEnter__(&pThis->threadEncoder.mutex);
        uint32_t    id = pThis->jpegEnc.id;
        CriticalSectionExit__(&pThis->threadEncoder.mutex);
        snprintf(fname, sizeof(fname), "/tmp/testImage_%03d.jpg", id);
        vejpeg_write_file(fname, pThis->jpegEnc.hwj.JpegBuff, pThis->jpegEnc.hwj.Jwritten);
        DBG("[JEPOC] EncodeJpeg.. to %s", fname);
        }
        //printBuffer__((uint8_t*)pFrame+100, 100, "pFrame");
        //printBuffer__(pThis->ve.pLumaSrc+100, 100, "Ysrc");
        //printBuffer__(pThis->ve.pChromaSrc+100, 100, "Csrc");
        //printBuffer__(pThis->jpegEnc.hwj.JpegBuff+100, 100, "JpegBuff");
        Ve_releaseSubengine__(pThis, SUBENG_JPEG_ENC);     /* release AVC-JPEG Encoder, unlock HW */
    }

bailout:
        CriticalSectionEnter__(&pThis->threadEncoder.mutex);
        pThis->jpegEnc.id++;
        CriticalSectionExit__(&pThis->threadEncoder.mutex);
}

//hw VE (VideoEngine)
static void Ve_init__ (void) {
    
    ve_init();                                                              /* init video encoder */
    if ((pThis->ve.pRegs = ve_open()) == NULL) {                            /* open video encoder */
        DBG("Ve_init_error!");
        exit(EXIT_FAILURE);
    }
}

static void Ve_allocInputBuffers__ (void) {

    /* input frame must match 16x16 macroblocks */
    if (pThis->camera.size.width % 16 != 0) {
        DBG("ERROR: input width frame must match 16x16 macroblocks");
        exit(EXIT_FAILURE);
    }
    
    if (pThis->camera.size.height % 16 != 0) {
        DBG("ERROR: input height frame must match 16x16 macroblocks");
        exit(EXIT_FAILURE);
    }
    
    /* allocate memory from VE */
    pThis->ve.pLumaSrc = ve_malloc(pThis->camera.YplaneSize + pThis->camera.CplaneSize);
	//pThis->ve.pChromaSrc = ve_malloc(pThis->camera.CplaneSize);
    if (pThis->ve.pLumaSrc == NULL) {
        DBG("ERROR: Cannot allocate input buffers!");
        exit(EXIT_FAILURE);
	}
    pThis->ve.pChromaSrc = pThis->ve.pLumaSrc + pThis->camera.YplaneSize;
	memset(pThis->ve.pLumaSrc, 0x80, pThis->camera.rawSize);
    
    DBGF("Ysize: %d, Csize: %d", pThis->camera.YplaneSize, pThis->camera.CplaneSize);
}

static void Ve_initScalerBuffers__ (void) {

    switch (pThis->ve.isp.scaler.type) {
        /* ARBITRARY-SCALER */
        case ARBITRARY_VGA: {
            pThis->ve.isp.scaler.size.width = 640;
            pThis->ve.isp.scaler.size.height = 480;
            pThis->ve.isp.scaler.xScaleFactor = 640.0f / pThis->camera.size.width;
            pThis->ve.isp.scaler.yScaleFactor = 480.0f / pThis->camera.size.height;
            break;
        }
        case ARBITRARY_QVGA: {
            pThis->ve.isp.scaler.size.width = 320;
            pThis->ve.isp.scaler.size.height = 240;
            pThis->ve.isp.scaler.xScaleFactor = 320.0f / pThis->camera.size.width;
            pThis->ve.isp.scaler.yScaleFactor = 240.0f / pThis->camera.size.height;
            break;
        }
        default: {                                                          /* scaler is not used */
            pThis->ve.isp.scaler.size.width = pThis->camera.size.width;
            pThis->ve.isp.scaler.size.height = pThis->camera.size.height;
            break;
        }
    }
    
    /* calculate macroblocks size */
    pThis->ve.isp.scaler.mb_width = DIV_ROUND_UP(pThis->ve.isp.scaler.size.width, 16);
    pThis->ve.isp.scaler.mb_height = DIV_ROUND_UP(pThis->ve.isp.scaler.size.height, 16);
    pThis->ve.isp.scaler.mb_stride = pThis->ve.isp.scaler.size.width / 16;
    pThis->ve.isp.scaler.crop_right = (pThis->ve.isp.scaler.mb_width * 16 - 
                                        pThis->ve.isp.scaler.size.width) / 2;
    pThis->ve.isp.scaler.crop_bottom = (pThis->ve.isp.scaler.mb_height * 16 - 
                                        pThis->ve.isp.scaler.size.height) / 2;
    
    pThis->ve.isp.scaler.YplaneSize = tjPlaneSizeYUV(0, pThis->ve.isp.scaler.size.width, 0, 
                                                       pThis->ve.isp.scaler.size.height, TJSAMP_420);
    pThis->ve.isp.scaler.CplaneSize = (tjPlaneSizeYUV(1, pThis->ve.isp.scaler.size.width, 0, 
                                                 pThis->ve.isp.scaler.size.height, TJSAMP_420)) << 1;
    pThis->ve.isp.scaler.rawSize = pThis->ve.isp.scaler.YplaneSize + pThis->ve.isp.scaler.CplaneSize;

    DBGF("SCALER: %dx%d, YscaledSize: %d, CscaledSize: %d, xFactor: %.3f, yFactor:%.3f",
            pThis->ve.isp.scaler.size.width, pThis->ve.isp.scaler.size.height, 
            pThis->ve.isp.scaler.YplaneSize, pThis->ve.isp.scaler.CplaneSize, 
            pThis->ve.isp.scaler.xScaleFactor, pThis->ve.isp.scaler.yScaleFactor);
}

static void Ve_allocOutputBuffers__ (void) {

    pThis->jpegEnc.hwj.JpegBuff = ve_malloc(MAX_JPEG_SIZE);
    if (!pThis->jpegEnc.hwj.JpegBuff) {
        DBG("ERROR: Cannot allocate jpeg buffer!");
        exit(EXIT_FAILURE);
    }
}

static void Ve_fillInputBuffers__ (const void *pFrame, int frameSize) {
    
    //DBGF("YplaneSize: %d, CplaneSize: %d => %p, %p", pThis->camera.YplaneSize, 
            //pThis->camera.CplaneSize, pFrame, pFrame + pThis->camera.YplaneSize);
    memcpy(pThis->ve.pLumaSrc, pFrame, frameSize);
    //memcpy(pThis->ve.pChromaSrc, pFrame + pThis->camera.YplaneSize, pThis->camera.CplaneSize);

	/* flush for A20 */
	ve_flush_cache(pThis->ve.pLumaSrc, frameSize);
	//ve_flush_cache(pThis->ve.pChromaSrc, pThis->camera.CplaneSize);
}

static void Ve_selectSubengine__ (Appl_t *pThis, VeSubengType_t subengine) {
    
    CriticalSectionEnter__(&pThis->ve.mutex);                                          /* lock hw */
    
    DBG(KYEL"%s -> ENTER -> Type: %s"KNRM, __FUNCTION__, subengNames[subengine]);

    switch (subengine) {
        case SUBENG_JPEG_ENC: {                                                   /* jpeg encoder */
            veavc_select_subengine(VE_ENGINE_AVC);
            break;
        }
        case SUBENG_JPEG_DEC: {                                                   /* jpeg decoder */
            veavc_select_subengine(VE_ENGINE_MPEG);
            break;
        }
        case SUBENG_H264_ENC: {                                                   /* h264 encoder */
            veavc_select_subengine(VE_ENGINE_AVC);
            break;
        }
        case SUBENG_H264_DEC: {                                                   /* h264 decoder */
            veavc_select_subengine(VE_ENGINE_H264);
            writel((readl(pThis->ve.pRegs + VE_CTRL) & ~0xf) | VE_ENGINE_H264
                | (pThis->ve.h264dec.width >= 2048 ? (0x1 << 21) : 0x0), pThis->ve.pRegs + VE_CTRL);
            break;
        }
        default: {
            assert(FALSE);
            break;
        }
    }
}

static void Ve_releaseSubengine__ (Appl_t *pThis, VeSubengType_t subengine) {

    DBG(KWHT"%s -> EXIT -> Type: %s"KNRM, __FUNCTION__, subengNames[subengine]);
    
    veavc_release_subengine();                                               /* release subengine */
    CriticalSectionExit__(&pThis->ve.mutex);                                         /* unlock hw */
}

static void Ve_trigerSubebgine__ (VeSubengType_t subengine) {

    switch (subengine) {
        case SUBENG_JPEG_ENC: {
            veavc_launch_encoding();
            ve_wait(1);
            veavc_check_status();
            break;
        }
        default: {
            break;
        }
    }
}

static void Ve_veisp_setInputBuffers__ (uint8_t *Y, uint8_t *C) {
    
    veisp_set_buffers(Y, C);
}

static void Ve_veisp_initPicture__ (uint32_t width_mb, uint32_t height_mb, uint32_t stride_mb, veisp_color_format_t format) {

    veisp_init_pictureMb(width_mb, height_mb, stride_mb, format);
}

static void Ve_freeInputBuffers__ (void) {

    ve_free(pThis->ve.pLumaSrc);
}

static void Ve_freeOutputBuffers__ (void) {
    
	ve_free(pThis->jpegEnc.hwj.JpegBuff);
}

__attribute__((unused)) static void Ve_freeMbInfoBuffer__ (void) {

    if (pThis->ve.isp.mb_info_buf) {
        ve_free(pThis->ve.isp.mb_info_buf);
        pThis->ve.isp.mb_info_buf = NULL;
    }
}

static void Ve_free__ (void) {
    
    ve_close();
}

//hw h264enc
static void H264enc_init__ (void) {
    /* open output file */
    if ((pThis->ve.h264enc.file = open("/tmp/tvin.h264", O_CREAT | O_RDWR | O_TRUNC,
			S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)) == -1) {
		printf("ERROR: could not open output file\n");
		exit(EXIT_FAILURE);
	}
    
    pThis->ve.h264enc.profileIdc = ENCODER_DECODER_PROFILE_H264_MAIN;     /* Main Profile (MP,77) */
    pThis->ve.h264enc.levelIdc = ENCODER_DECODER_LEVEL_H264_4_1;                     /* level 4.1 */
    pThis->ve.h264enc.ecMode = H264_EC_CABAC;      /* entropyCoding: H264_EC_CAVLC, H264_EC_CABAC */
	pThis->ve.h264enc.qp = 24;                                                      /* range 1-47 */
	pThis->ve.h264enc.keyframeInterval = 25;
    
    pThis->ve.h264enc.pBytestreamBuffer = NULL;
    pThis->ve.h264enc.extra_buffer_line = NULL;
    pThis->ve.h264enc.extra_buffer_frame = NULL;
    pThis->ve.h264enc.extra_motion_est = NULL;
}

static void H264enc_new__ (void) {
    
    uint32_t i;
    uint32_t luma_size;
    uint32_t chroma_size;
    
    
    /* copy parameters */
    pThis->ve.h264enc.spsPps = TRUE;
    pThis->ve.h264enc.frameNum = 0;
    
    /* allocate bytestream output buffer */
	pThis->ve.h264enc.bytestreamBufferSize = VLE_SIZE;
	pThis->ve.h264enc.pBytestreamBuffer = ve_malloc(pThis->ve.h264enc.bytestreamBufferSize);
	if (pThis->ve.h264enc.pBytestreamBuffer == NULL) {
		goto nomem;
    }
    
    /* allocate reference picture memory */
    luma_size = ALIGN(pThis->ve.isp.scaler.mb_width * 16, 32) * ALIGN(pThis->ve.isp.scaler.mb_height * 16, 32);
	chroma_size = ALIGN(pThis->ve.isp.scaler.mb_width * 16, 32) * ALIGN(pThis->ve.isp.scaler.mb_height * 8, 32);
	for (i = 0; i < NUM_REFERENT_PIC; i++) {
		pThis->ve.h264enc.refPic[i].luma_buffer = ve_malloc(luma_size + chroma_size);
		pThis->ve.h264enc.refPic[i].chroma_buffer = pThis->ve.h264enc.refPic[i].luma_buffer + 
                                                            luma_size;
		pThis->ve.h264enc.refPic[i].extra_buffer = ve_malloc(luma_size / 4);
		if (pThis->ve.h264enc.refPic[i].luma_buffer == NULL || 
            pThis->ve.h264enc.refPic[i].extra_buffer == NULL) {
			goto nomem;
        }
	}
    
    /* allocate unknown purpose buffers */
	pThis->ve.h264enc.extra_buffer_frame = ve_malloc(ALIGN(pThis->ve.isp.scaler.mb_width, 4) * 
                                                              pThis->ve.isp.scaler.mb_height * 8);
	pThis->ve.h264enc.extra_buffer_line = ve_malloc(pThis->ve.isp.scaler.mb_width * 32);
    pThis->ve.h264enc.extra_motion_est = ve_malloc(pThis->ve.isp.scaler.mb_width * pThis->ve.isp.scaler.mb_height * 32);//guessing size - TODO: find out proper size
	if (pThis->ve.h264enc.extra_buffer_frame == NULL || 
        pThis->ve.h264enc.extra_buffer_line == NULL || pThis->ve.h264enc.extra_motion_est == NULL) {
		goto nomem;
    }
    
    return;
        
nomem:
	DBG("can't allocate VE memory");
	H264enc_free__();
}

static void H264enc_free__ (void) {

	uint32_t i;

    /* free extra buffers */
    if (pThis->ve.h264enc.extra_buffer_line != NULL) {
    	ve_free(pThis->ve.h264enc.extra_buffer_line);
    }
    if (pThis->ve.h264enc.extra_buffer_frame != NULL) {
        ve_free(pThis->ve.h264enc.extra_buffer_frame);
    }
    if (pThis->ve.h264enc.extra_motion_est != NULL) {
        ve_free(pThis->ve.h264enc.extra_motion_est);
    }
	/* free referent pictures */
	for (i = 0; i < NUM_REFERENT_PIC; i++) {
        if (pThis->ve.h264enc.refPic[i].luma_buffer) {
            ve_free(pThis->ve.h264enc.refPic[i].luma_buffer);
        }
	    if (pThis->ve.h264enc.refPic[i].extra_buffer) {
            ve_free(pThis->ve.h264enc.refPic[i].extra_buffer);
        }
	}
    /* free bytestream buffers */
    if (pThis->ve.h264enc.pBytestreamBuffer) {
        ve_free(pThis->ve.h264enc.pBytestreamBuffer);
    }
    /* close output file */
    if (pThis->ve.h264enc.file != -1) {
        close(pThis->ve.h264enc.file);
    }
}

static int H264enc_encodePicture__ (void) {
	pThis->ve.h264enc.sliceType = pThis->ve.h264enc.frameNum ? SLICE_P : SLICE_I;

    Ve_selectSubengine__(pThis, SUBENG_H264_ENC);             /* select AVC-H264 Encoder, lock HW */

	/* flush buffers (output because otherwise we might read old data later) */
	ve_flush_cache(pThis->ve.h264enc.pBytestreamBuffer, pThis->ve.h264enc.bytestreamBufferSize);
	ve_flush_cache(pThis->ve.pLumaSrc, pThis->camera.rawSize);

	/* set output buffer */
	writel(0x0, pThis->ve.pRegs + VE_AVC_VLE_OFFSET);
	writel(ve_virt2phys(pThis->ve.h264enc.pBytestreamBuffer), pThis->ve.pRegs + VE_AVC_VLE_ADDR);
	writel(ve_virt2phys(pThis->ve.h264enc.pBytestreamBuffer) + 
            pThis->ve.h264enc.bytestreamBufferSize - 1, pThis->ve.pRegs + VE_AVC_VLE_END);
                                                                        /* size in bits not bytes */
	writel(pThis->ve.h264enc.bytestreamBufferSize * 8, pThis->ve.pRegs + VE_AVC_VLE_MAX);

	/* write headers */
	if (pThis->ve.h264enc.spsPps == TRUE) {
		put_seq_parameter_set__();
		put_pic_parameter_set__();
		pThis->ve.h264enc.spsPps = FALSE;
	}
	put_slice_header__();

	/* set input size */ /* set input format */
    Ve_veisp_initPicture__(pThis->camera.mb_width, pThis->camera.mb_height, pThis->camera.mb_stride,
                           pThis->ve.isp.colorFormat);
                           
	/* set input buffer */
    Ve_veisp_setInputBuffers__(pThis->ve.pLumaSrc, pThis->ve.pChromaSrc);
    
    /* add scaler */
    if (pThis->ve.isp.scaler.type > 0) {
        /* use arbitrary scaler engine */
        //DBG("Use arbitrary scaler for h264");
        veisp_set_scaler(pThis->ve.isp.scaler.size.width, pThis->ve.isp.scaler.size.height,
              0.0, 0.0, pThis->ve.isp.scaler.xScaleFactor, pThis->ve.isp.scaler.yScaleFactor);
    }

	/* set reconstruction buffers */
	h264encRefPicture_t *ref_pic = &pThis->ve.h264enc.refPic[pThis->ve.h264enc.frameNum % 2];
	writel(ve_virt2phys(ref_pic->luma_buffer), pThis->ve.pRegs + VE_AVC_REC_LUMA);
	writel(ve_virt2phys(ref_pic->chroma_buffer), pThis->ve.pRegs + VE_AVC_REC_CHROMA);
	writel(ve_virt2phys(ref_pic->extra_buffer), pThis->ve.pRegs + VE_AVC_REC_SLUMA);

	/* set reference buffers */
	if (pThis->ve.h264enc.sliceType != SLICE_I) {
		ref_pic = &pThis->ve.h264enc.refPic[(pThis->ve.h264enc.frameNum + 1) % 2];
		writel(ve_virt2phys(ref_pic->luma_buffer), pThis->ve.pRegs + VE_AVC_REF_LUMA);
		writel(ve_virt2phys(ref_pic->chroma_buffer), pThis->ve.pRegs + VE_AVC_REF_CHROMA);
		writel(ve_virt2phys(ref_pic->extra_buffer), pThis->ve.pRegs + VE_AVC_REF_SLUMA);
	}

	/* set unknown purpose buffers */
	writel(ve_virt2phys(pThis->ve.h264enc.extra_buffer_line), pThis->ve.pRegs + VE_AVC_MB_INFO);
	writel(ve_virt2phys(pThis->ve.h264enc.extra_buffer_frame), pThis->ve.pRegs + VE_AVC_UNK_BUF);
    writel(ve_virt2phys(pThis->ve.h264enc.extra_motion_est), pThis->ve.pRegs + 0xbc4);

	/* enable interrupt and clear status flags */
	writel(readl(pThis->ve.pRegs + VE_AVC_CTRL) | 0xf, pThis->ve.pRegs + VE_AVC_CTRL);
	writel(readl(pThis->ve.pRegs + VE_AVC_STATUS) | 0x7, pThis->ve.pRegs + VE_AVC_STATUS);

	/* set encoding parameters */
	uint32_t params = 0x0;
	if (pThis->ve.h264enc.ecMode) {
		params |= 0x100;
    }
	if (pThis->ve.h264enc.sliceType == SLICE_P) {
		params |= 0x10;
    }
	writel(params, pThis->ve.pRegs + VE_AVC_PARAM);
	writel((4 << 16) | (pThis->ve.h264enc.qp << 8) | 
            pThis->ve.h264enc.qp, pThis->ve.pRegs + VE_AVC_QP);
	writel(0x00040104, pThis->ve.pRegs + VE_AVC_MOTION_EST);

	/* trigger encoding */
	writel(0x8, pThis->ve.pRegs + VE_AVC_TRIGGER);
	ve_wait(1);

	/* check result */
	uint32_t status = readl(pThis->ve.pRegs + VE_AVC_STATUS);
	writel(status, pThis->ve.pRegs + VE_AVC_STATUS);

	/* save bytestream length */
	pThis->ve.h264enc.bytestreamLength = readl(pThis->ve.pRegs + VE_AVC_VLE_LENGTH) / 8;

	/* next frame */
	pThis->ve.h264enc.frameNum++;
	if (pThis->ve.h264enc.frameNum >= pThis->ve.h264enc.keyframeInterval)
		pThis->ve.h264enc.frameNum = 0;

    Ve_releaseSubengine__(pThis, SUBENG_H264_ENC);         /* release AVC-H264 Encoder, unlock HW */
    
	return (status & 0x3) == 0x1;
}

static void put_bits__ (uint32_t x, int num) {
    
	writel(x, pThis->ve.pRegs + VE_AVC_BASIC_BITS);
	writel(0x1 | ((num & 0x1f) << 8), pThis->ve.pRegs + VE_AVC_TRIGGER);
	/* again the problem, how to check for finish? */
}

static void put_ue__ (uint32_t x) {

	x++;
	put_bits__(x, (32 - __builtin_clz(x)) * 2 - 1);
}

static void put_se__ (int x) {

	x = 2 * x - 1;
	x ^= (x >> 31);
	put_ue__(x);
}

static void put_start_code__ (unsigned int nal_ref_idc, unsigned int nal_unit_type) {

	uint32_t tmp = readl(pThis->ve.pRegs + VE_AVC_PARAM);

	/* disable emulation_prevention_three_byte */
	writel(tmp | (0x1 << 31), pThis->ve.pRegs + VE_AVC_PARAM);

	put_bits__(0, 24);
	put_bits__(0x100 | (nal_ref_idc << 5) | (nal_unit_type << 0), 16);

	writel(tmp, pThis->ve.pRegs + VE_AVC_PARAM);
}

static void put_rbsp_trailing_bits__ (void) {

	unsigned int cur_bs_len = readl(pThis->ve.pRegs + VE_AVC_VLE_LENGTH);

	int num_zero_bits = 8 - ((cur_bs_len + 1) & 0x7);
	put_bits__(1 << num_zero_bits, num_zero_bits + 1);
}

static void put_seq_parameter_set__ (void) {

    unsigned int frameCroppingFlag;
	
    put_start_code__(3, 7);

	put_bits__(pThis->ve.h264enc.profileIdc, 8);
	put_bits__(pThis->ve.h264enc.constraints, 8);
	put_bits__(pThis->ve.h264enc.levelIdc, 8);

	put_ue__(/* seq_parameter_set_id = */ 0);

	put_ue__(/* log2_max_frame_num_minus4 = */ 0);
	put_ue__(/* pic_order_cnt_type = */ 2);

	put_ue__(/* max_num_ref_frames = */ 1);
	put_bits__(/* gaps_in_frame_num_value_allowed_flag = */ 0, 1);

	put_ue__(pThis->ve.isp.scaler.mb_width - 1);
	put_ue__(pThis->ve.isp.scaler.mb_height - 1);

	put_bits__(/* frame_mbs_only_flag = */ 1, 1);

	put_bits__(/* direct_8x8_inference_flag = */ 0, 1);

	frameCroppingFlag = pThis->camera.crop_right || pThis->camera.crop_bottom;
	put_bits__(frameCroppingFlag, 1);
	if (frameCroppingFlag) {
		put_ue__(0);
		put_ue__(pThis->camera.crop_right);
		put_ue__(0);
		put_ue__(pThis->camera.crop_bottom);
	}

	put_bits__(/* vui_parameters_present_flag = */ 0, 1);

	put_rbsp_trailing_bits__();
}

static void put_pic_parameter_set__ (void) {

	put_start_code__(3, 8);

	put_ue__(/* pic_parameter_set_id = */ 0);
	put_ue__(/* seq_parameter_set_id = */ 0);

	put_bits__(pThis->ve.h264enc.ecMode, 1);

	put_bits__(/* bottom_field_pic_order_in_frame_present_flag = */ 0, 1);
	put_ue__(/* num_slice_groups_minus1 = */ 0);

	put_ue__(/* num_ref_idx_l0_default_active_minus1 = */ 0);
	put_ue__(/* num_ref_idx_l1_default_active_minus1 = */ 0);

	put_bits__(/* weighted_pred_flag = */ 0, 1);
	put_bits__(/* weighted_bipred_idc = */ 0, 2);

	put_se__((int)pThis->ve.h264enc.qp - 26);
	put_se__((int)pThis->ve.h264enc.qp - 26);
	put_se__(/* chroma_qp_index_offset = */ 4);

	put_bits__(/* deblocking_filter_control_present_flag = */ 1, 1);
	put_bits__(/* constrained_intra_pred_flag = */ 0, 1);
	put_bits__(/* redundant_pic_cnt_present_flag = */ 0, 1);

	put_rbsp_trailing_bits__();
}

static void put_slice_header__ (void) {

	if (pThis->ve.h264enc.sliceType == SLICE_I) {
		put_start_code__(3, 5);
    } else {
		put_start_code__(2, 1);
    }
    
	put_ue__(/* first_mb_in_slice = */ 0);
	put_ue__(pThis->ve.h264enc.sliceType);
	put_ue__(/* pic_parameter_set_id = */ 0);

	put_bits__(pThis->ve.h264enc.frameNum & 0xf, 4);

	if (pThis->ve.h264enc.sliceType == SLICE_I) {
		put_ue__(/* idr_pic_id = */ 0);
    }

	if (pThis->ve.h264enc.sliceType == SLICE_P) {
		put_bits__(/* num_ref_idx_active_override_flag = */ 0, 1);
		put_bits__(/* ref_pic_list_modification_flag_l0 = */ 0, 1);
		put_bits__(/* adaptive_ref_pic_marking_mode_flag = */ 0, 1);
		if (pThis->ve.h264enc.ecMode)
			put_ue__(/* cabac_init_idc = */ 0);
	}

	if (pThis->ve.h264enc.sliceType == SLICE_I)
	{
		put_bits__(/* no_output_of_prior_pics_flag = */ 0, 1);
		put_bits__(/* long_term_reference_flag = */ 0, 1);
	}

	put_se__(/* slice_qp_delta = */ 0);

	put_ue__(/* disable_deblocking_filter_idc = */ 0);
	put_se__(/* slice_alpha_c0_offset_div2 = */ 0);
	put_se__(/* slice_beta_offset_div2 = */ 0);
}

//hw jpeg decoder
static void JpegDec_init__ (void) {

    pThis->ve.jpegdec.input_buffer = NULL;
    pThis->ve.jpegdec.luma_buffer = NULL;
    pThis->ve.jpegdec.chroma_buffer = NULL;
}

static void JpegDec_decode__ (struct jpeg_t *jpeg) {

    int input_size;
    int output_size;
    
    Ve_selectSubengine__(pThis, SUBENG_JPEG_DEC);            /* select MPEG-JPEG Decoder, lock HW */
    
    /* free previous allocated buffers */
    if (pThis->ve.jpegdec.input_buffer != NULL) {
        ve_free(pThis->ve.jpegdec.input_buffer);
        ve_free(pThis->ve.jpegdec.luma_buffer);
        ve_free(pThis->ve.jpegdec.chroma_buffer);
        pThis->ve.jpegdec.input_buffer = NULL;
        pThis->ve.jpegdec.luma_buffer = NULL;
        pThis->ve.jpegdec.chroma_buffer = NULL;
    }
    /* allocate new input and output buffers */
    input_size = (jpeg->data_len + 65535) & ~65535;
    pThis->ve.jpegdec.input_buffer = ve_malloc(input_size);
    output_size = ((jpeg->width + 31) & ~31) * ((jpeg->height + 31) & ~31);
    pThis->ve.jpegdec.luma_buffer = ve_malloc(output_size);
    pThis->ve.jpegdec.chroma_buffer = ve_malloc(output_size);
    
    /* copy source frame to ve buffer */
    memcpy(pThis->ve.jpegdec.input_buffer, jpeg->data, jpeg->data_len);
    ve_flush_cache(pThis->ve.jpegdec.input_buffer, jpeg->data_len);

    /* set restart interval (DRI marker) */
    writel(jpeg->restart_interval, pThis->ve.pRegs + VE_MPEG_JPEG_RES_INT); 

    /* set JPEG format */
    JpegDec_setFormat__(jpeg);

    /* set raw output buffers (Luma / Croma) */
    /* Luma Rotate/Scale Output Buffer Address (must be 1KB aligned)  */
    writel(ve_virt2phys(pThis->ve.jpegdec.luma_buffer), pThis->ve.pRegs + VE_MPEG_ROT_LUMA);
    /* Chroma Rotate/Scale Output Buffer Address (must be 1KB aligned)  */
    writel(ve_virt2phys(pThis->ve.jpegdec.chroma_buffer), pThis->ve.pRegs + VE_MPEG_ROT_CHROMA);

    /* set JPEG size */
    JpegDec_setSize__(jpeg);

    /* ?? - Used for control Rotate/Scale buffer */
    writel(0x00000000, pThis->ve.pRegs + VE_MPEG_SDROT_CTRL);

    /* set input end (video source buffer last address) */
    writel(ve_virt2phys(pThis->ve.jpegdec.input_buffer) + input_size - 1, 
           pThis->ve.pRegs + VE_MPEG_VLD_END);

    /* ?? - IRQ enable flag */
    writel(0x0000007c, pThis->ve.pRegs + VE_MPEG_CTRL);

    /* set input offset in bits -VLD Offset in bits - current frame offset from VLD start address */
    writel(0 * 8, pThis->ve.pRegs + VE_MPEG_VLD_OFFSET);

    /* set input length in bits -  	VLD Length in bits - source video size */
    writel(jpeg->data_len * 8, pThis->ve.pRegs + VE_MPEG_VLD_LEN);

    /* set input buffer - VLD Address LOW bits (for first bits dropped from address),
     * the 0x7 flag is used for both MPEG and JPEG decoding  */
    writel(ve_virt2phys(pThis->ve.jpegdec.input_buffer) | 0x70000000, 
            pThis->ve.pRegs + VE_MPEG_VLD_ADDR);

    /* set Quantisation Table */
    JpegDec_setQuantizationTables__(jpeg);

    /* set Huffman Table */ /* Used for reload huffman table(JPEG decoding) */
    writel(0x00000000, pThis->ve.pRegs + VE_MPEG_RAM_WRITE_PTR);
    JpegDec_setHuffmanTables__(jpeg);

    /* trigger decoding */
    writeb(0x0e, pThis->ve.pRegs + VE_MPEG_TRIGGER);                                                //TODO probati sa 0xd
    ve_wait(1); /* wait for interrupt */

    /* clean interrupt flag (??) */
    writel(0x0000c00f, pThis->ve.pRegs + VE_MPEG_STATUS);

	//output_ppm(stdout, jpeg, output, output + (output_buf_size / 2));
    
    /* stop MPEG engine */
    Ve_releaseSubengine__(pThis, SUBENG_JPEG_DEC);        /* release MPEG-JPEG Decoder, unlock HW */
}

static void JpegDec_setFormat__ (struct jpeg_t *jpeg) {

	uint8_t fmt = (jpeg->comp[0].samp_h << 4) | jpeg->comp[0].samp_v;

	switch (fmt) {
        case 0x11: {
            writeb(0x1b, pThis->ve.pRegs + VE_MPEG_TRIGGER + 0x3);
            break;
        }
        case 0x21: {
            writeb(0x13, pThis->ve.pRegs + VE_MPEG_TRIGGER + 0x3);
            break;
        }
        case 0x12: {
            writeb(0x23, pThis->ve.pRegs + VE_MPEG_TRIGGER + 0x3);
            break;
        }
        case 0x22: {
            writeb(0x03, pThis->ve.pRegs + VE_MPEG_TRIGGER + 0x3);
            break;
        }
        default: {
            break;
        }
	}
}

static void JpegDec_setSize__ (struct jpeg_t *jpeg) {
    
    uint16_t height;
    uint16_t width;
    
    height = (jpeg->height - 1) / (8 * jpeg->comp[0].samp_v);
	width = (jpeg->width - 1) / (8 * jpeg->comp[0].samp_h);
	writel((uint32_t)height << 16 | width, pThis->ve.pRegs + VE_MPEG_JPEG_SIZE);
}

static void JpegDec_setQuantizationTables__ (struct jpeg_t *jpeg) {

	int i;
    
	for (i = 0; i < 64; i++) {
		writel((uint32_t)(64 + i) << 8 | jpeg->quant[0]->coeff[i], 
                pThis->ve.pRegs + VE_MPEG_IQ_MIN_INPUT);
    }
	for (i = 0; i < 64; i++) {
		writel((uint32_t)(i) << 8 | jpeg->quant[1]->coeff[i], 
                pThis->ve.pRegs + VE_MPEG_IQ_MIN_INPUT);
    }
}

static void JpegDec_setHuffmanTables__ (struct jpeg_t *jpeg) {

    uint32_t buffer[512];
	int i;
    
    memset(buffer, 0, 4*512);
    
	for (i = 0; i < 4; i++) {
		if (jpeg->huffman[i]) {
			int j, sum, last;

			last = 0;
			sum = 0;
			for (j = 0; j < 16; j++) {
				((uint8_t *)buffer)[i * 64 + 32 + j] = sum;
				sum += jpeg->huffman[i]->num[j];
				if (jpeg->huffman[i]->num[j] != 0) {
					last = j;
                }
			}
			memcpy(&(buffer[256 + 64 * i]), jpeg->huffman[i]->codes, sum);
			sum = 0;
			for (j = 0; j <= last; j++) {
				((uint16_t *)buffer)[i * 32 + j] = sum;
				sum += jpeg->huffman[i]->num[j];
				sum *= 2;
			}
			for (j = last + 1; j < 16; j++) {
				((uint16_t *)buffer)[i * 32 + j] = 0xffff;
			}
		}
	}

	for (i = 0; i < 512; i++) {
		writel(buffer[i], pThis->ve.pRegs + VE_MPEG_RAM_WRITE_DATA);
	}
}

//hw h264 decoder
static void H264dec_init__ (void) {
    
    uint32_t    i;
    
    /* open input file */
    if ((pThis->ve.h264dec.fileId = open(pThis->ve.h264dec.fileName, O_RDONLY)) == -1) {
        DBG("Error open: %s", pThis->ve.h264dec.fileName);
        exit(EXIT_FAILURE);
    }
    pThis->ve.h264dec.fileOffset = 0;
    if (fstat(pThis->ve.h264dec.fileId, &pThis->ve.h264dec.s) < 0) {
        DBG("Error stat %s", pThis->ve.h264dec.fileName);
        exit(EXIT_FAILURE);
    }
    if (pThis->ve.h264dec.s.st_size == 0) {
        DBG("Error %s empty", pThis->ve.h264dec.fileName);
        exit(EXIT_FAILURE);
    }
    /* map file */
    if ((pThis->ve.h264dec.pFileData = mmap(NULL, pThis->ve.h264dec.s.st_size, PROT_READ, MAP_SHARED, 
                                            pThis->ve.h264dec.fileId, 0)) == MAP_FAILED) {
        DBG("Error mmap %s", pThis->ve.h264dec.fileName);
        exit(EXIT_FAILURE);
    }
    
    DBG("H264Dec Open And MMAP Input FIle: %s => %ld bytes", pThis->ve.h264dec.fileName, 
                                                             pThis->ve.h264dec.s.st_size);
    
    //TODO
    //TODO unpack mkv container "DEMUX".. read all video parameters..
    //set framerate
    
    memset(&pThis->ve.h264dec.dpb, 0, sizeof(pThis->ve.h264dec.dpb));
    memset(&pThis->ve.h264dec.gst, 0, sizeof(pThis->ve.h264dec.gst));
    
    H264dec_dpb_init__(&pThis->ve.h264dec.dpb);
    H264dec_dpb_setOutputFunc__(&pThis->ve.h264dec.dpb, H264dec_output__, pThis);
    
    /* init output surfaces */
    for (i = 0; i < OUTPUT_SURFACES; i++) {
        pThis->ve.h264dec.output[i] = VC_INVALID_HANDLE;
    }
    
    /* Initialize GStreamer library for AVC NAL Unit parsing. */
    pThis->ve.h264dec.gst.parser = gst_h264_nal_parser_new();
    if(!pThis->ve.h264dec.gst.parser) {
        DBG(KRED"Error: unable to call gst_h264_nal_parser_new."KNRM);
        exit(EXIT_FAILURE);
    }
    if(Gst_allocateObjects__(&pThis->ve.h264dec.gst.nalu, &pThis->ve.h264dec.gst.slice, 
          &pThis->ve.h264dec.gst.sps, &pThis->ve.h264dec.gst.pps, &pThis->ve.h264dec.gst.sei) < 0) {
        DBG(KRED"Failed to allocate Gst objects."KNRM);
        gst_h264_nal_parser_free(pThis->ve.h264dec.gst.parser);
        exit(EXIT_FAILURE);
    }
        
    /* We don't have the width or height here, we need to parse those from
       the SPS as pic_width_in_luma_samples/pic_height_in_luma_samples/
    */
    /* START h264 decoder thread */
    CriticalSectionCreate__(&pThis->threadH264Decoder.mutex);     /* 264 decoder critical section */
    ConditionVariable_create__(&pThis->threadH264Decoder.cond);     /* h264 decoder cond variable */
    Thread_create__(&pThis->threadH264Decoder, ThreadDecodeH264Frame__, pThis);
}

static void H264dec_new__ (H264dec_t *pH264dec) {
    
    int extra_data_size = 320 * 1024;
    
    pThis->ve.h264dec.data = ve_malloc(VBV_SIZE);                                     /* VBV 1 MB */
	if (!(pThis->ve.h264dec.data)) {
        DBG("Error alloc h264dec VBV data");
        exit(EXIT_FAILURE);
	}
    
    if ((ve_get_version() == 0x1625) || (pH264dec->width >= 2048)) {
		/* Engine version 0x1625 needs two extra buffers */
		extra_data_size += ((pH264dec->width - 1) / 16 + 32) * 192;
		extra_data_size = (extra_data_size + 4095) & ~4095;
		extra_data_size += ((pH264dec->width - 1) / 16 + 64) * 80;
	}

	pThis->ve.h264dec.extra_data = ve_malloc(extra_data_size);
	if (!pThis->ve.h264dec.extra_data) {
        DBG("Error alloc h264dec extraData");
        exit(EXIT_FAILURE);
    }

	memset(pThis->ve.h264dec.extra_data, 0, extra_data_size);
	ve_flush_cache(pThis->ve.h264dec.extra_data, extra_data_size);
    pThis->ve.h264dec.extra_data_size = extra_data_size;
}

static void H264dec_free__ (void) {
    
    //int retVal;
    
    /* close decoder thread */
    while(1) {                                    /* wait until decode process image is completed */
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadH264Decoder.process == FALSE) {
            pThis->threadH264Decoder.running = FALSE;
            CriticalSectionExit__(&pThis->mutex);
            break;
        }
        CriticalSectionExit__(&pThis->mutex);
        usleep(5000);
    }
    ConditionVariable_signal__(&pThis->threadH264Decoder.cond);
    //retVal = pthread_join(pThis->threadH264Decoder.id, NULL);
    //assert(retVal == 0);                             /* pthread_join() must return with success */
    
    while(1) {                                            /* wait until decode thread is finished */
        CriticalSectionEnter__(&pThis->mutex);
        if (pThis->threadH264Decoder.finished == TRUE) {
            CriticalSectionExit__(&pThis->mutex);
            break;
        }
        ConditionVariable_signal__(&pThis->threadH264Decoder.cond);
        CriticalSectionExit__(&pThis->mutex);
        usleep(5000);
    }
    
    ConditionVariable_destroy__(&pThis->threadH264Decoder.cond); /* destroy decoder condition var */
    CriticalSectionDestroy__(&pThis->threadH264Decoder.mutex);           /* destroy decoder mutex */
    
    /* free VE memory */
    if (pThis->ve.h264dec.extra_data != NULL) {
        ve_free(pThis->ve.h264dec.extra_data);
        pThis->ve.h264dec.extra_data = NULL;
    }
    if (pThis->ve.h264dec.data != NULL) {
        ve_free(pThis->ve.h264dec.data);
        pThis->ve.h264dec.data = NULL;
    }
    
    /* free gstreamer objects */
    gst_h264_nal_parser_free(pThis->ve.h264dec.gst.parser);
    Gst_freeObjects__(&pThis->ve.h264dec.gst.nalu, &pThis->ve.h264dec.gst.slice, 
                &pThis->ve.h264dec.gst.sps, &pThis->ve.h264dec.gst.pps, &pThis->ve.h264dec.gst.sei);
    
    /* close input file */
    munmap(pThis->ve.h264dec.pFileData, pThis->ve.h264dec.s.st_size);         /* unmap input file */
    close(pThis->ve.h264dec.fileId);                                          /* close input file */
    
    /* release DPB reference surfaces */
    H264dec_dpb_finalize__(&pThis->ve.h264dec.dpb);
    
    /* release output surfaces */
    uint32_t    i;
    for (i = 0; i < OUTPUT_SURFACES; i++) {
        if (pThis->ve.h264dec.output[i] != VC_INVALID_HANDLE) {
            DBG("Destroy output surface: %d->%d", i, pThis->ve.h264dec.output[i]);
            if (VideoSurface_destroy(pThis->ve.h264dec.output[i]) != VC_STATUS_OK) {
                DBGF(KRED"ERROR: H264 Decoder => destroy output surface!"KNRM);
            } else {
                pThis->ve.h264dec.output[i] = 0;
            }
        }
    }
}

/* NOTE: pre pozivanja smemstiti u data buffer sve nal-ove (slice-ve) koji pripadaju jednom frejmu.
 * Popuniti slice_count vrednost pre pozivanja
*/
static VcStatus H264dec_decode__ (Appl_t *pThis, PictureInfoH264_t const *info, const int len, 
                             VideoSurface_t *output) {
    
    VcStatus        ret;
	H264Context_t  *context = calloc(1, sizeof(H264Context_t));
    context->regs = pThis->ve.pRegs;
	context->picture_width_in_mbs_minus1 = (pThis->ve.h264dec.width - 1) / 16;
    if (!info->frame_mbs_only_flag) {
		context->picture_height_in_mbs_minus1 = ((pThis->ve.h264dec.height / 2) - 1) / 16;
	} else {
		context->picture_height_in_mbs_minus1 = (pThis->ve.h264dec.height - 1) / 16;
    }
	context->info = info;
	context->output = output;
    context->video_extra_data_len = ((pThis->ve.h264dec.width + 15) / 16) * 
                                    ((pThis->ve.h264dec.height + 15) / 16) * 32;
    
    pThis->ve.h264dec.video_extra_data_len = context->video_extra_data_len;//todo delete
    
    if (!info->frame_mbs_only_flag) {
		DBG(KWHT"We decode interlaced Frames!"KNRM);
	}
    
    /* create extra buffer if it does not exist.. */
    H264dec_getSurfacePriv__(context, output);
    if (!output->extra_data) {
        DBG(KRED"H264dec_getSurfacePriv__! Sorry"KNRM);
        ret = VC_STATUS_RESOURCES;
        goto err_free;
    }
    
    if (info->field_pic_flag) {
		output->pic_type = PIC_TYPE_FIELD;
	} else if (info->mb_adaptive_frame_field_flag) {
		output->pic_type = PIC_TYPE_MBAFF;
	} else {
		output->pic_type = PIC_TYPE_FRAME;
    }
    
    //DBGF("SliceCount: %d", info->slice_count);

	/* lock the VE hardware and select h264 decoder subengine */
    Ve_selectSubengine__(pThis, SUBENG_H264_DEC);                 /* select H264 Decoder, lock HW */
    
    /* some buffers */
	uint32_t extra_buffers = ve_virt2phys(pThis->ve.h264dec.extra_data);
	writel(extra_buffers, context->regs + VE_H264_EXTRA_BUFFER1);
	writel(extra_buffers + 0x48000, context->regs + VE_H264_EXTRA_BUFFER2);
	if ((ve_get_version() == 0x1625) || pThis->ve.h264dec.width >= 2048) {
		int size = (context->picture_width_in_mbs_minus1 + 32) * 192;
		size = (size + 4095) & ~4095;
        writel(pThis->ve.h264dec.width >= 2048 ? 0x5 : 0xa, context->regs + 0x50);
		writel(extra_buffers + 0x50000, context->regs + 0x54);
		writel(extra_buffers + 0x50000 + size, context->regs + 0x58);
	}
    
    /* write custom scaling lists */
	if (!(context->default_scaling_lists = H264dec_checkScalingLists__(context))) {
		const uint32_t *sl4 = (uint32_t *)&context->info->scaling_lists_4x4[0][0];
		const uint32_t *sl8 = (uint32_t *)&context->info->scaling_lists_8x8[0][0];
        
        //DBG("write custom scaling lists");

		writel(VE_SRAM_H264_SCALING_LISTS, context->regs + VE_H264_RAM_WRITE_PTR);

		int i;
		for (i = 0; i < 2 * 64 / 4; i++) {
			writel(sl8[i], context->regs + VE_H264_RAM_WRITE_DATA);
        }

		for (i = 0; i < 6 * 16 / 4; i++) {
			writel(sl4[i], context->regs + VE_H264_RAM_WRITE_DATA);
        }
	}
    
    /* sdctrl */
	writel(0x00000000, context->regs + VE_H264_SDROT_CTRL);
    if (!H264dec_fillFrameLists__(pThis, context)) {
        DBG(KRED"NOTE: H264dec_fillFrameLists__ skip this nal!"KNRM);
        ret = VC_STATUS_ERROR;
        goto err_ve_put;
    } else {
        DBG("FillFrameLists: %d", context->ref_count);
    }

	unsigned int slice, pos = 0;
	for (slice = 0; slice < info->slice_count; slice++) {
		h264Header_t *h = &context->header;
		memset(h, 0, sizeof(h264Header_t));
        
        //DBGF("SliceCountSliceId: %d", slice);

        /* find sequence 0x00, 0, 0, 1 and set pos to start of header */
		pos = H264dec_findStartcode__(pThis->ve.h264dec.data, len, pos) + 3;

        /* get nal unit type */
		h->nal_unit_type = ((uint8_t *)(pThis->ve.h264dec.data))[pos++] & 0x1f;

		if (h->nal_unit_type != NAL_SLICE_IDR && h->nal_unit_type != NAL_SLICE) {
            DBG(KRED"NOTE %d: h->nal_unit_type != NAL_SLICE_IDR && h->nal_unit_type != NAL_SLICE skip this nal!"KNRM, h->nal_unit_type);
            ret = VC_STATUS_ERROR;
			goto err_ve_put;
            //continue;   // skip this nal
		}

		/* ?? Enable start code detection */
		writel((0x1 << 25) | (0x1 << 10), pThis->ve.pRegs + VE_H264_CTRL);

		/* set input buffer */
		writel((len - pos) * 8, pThis->ve.pRegs + VE_H264_VLD_LEN);  /* set buffer length in bits */
		writel(pos * 8, pThis->ve.pRegs + VE_H264_VLD_OFFSET);    /* set start position in bits.. */
		uint32_t input_addr = ve_virt2phys(pThis->ve.h264dec.data);
		writel(input_addr + VBV_SIZE - 1, pThis->ve.pRegs + VE_H264_VLD_END);/* end of bitestream buffer */
		writel((input_addr & 0x0ffffff0) | (input_addr >> 28) | (0x7 << 28), 
                pThis->ve.pRegs + VE_H264_VLD_ADDR);                    /* set start data address */

		/* ?? some sort of reset maybe */
		writel(0x7, pThis->ve.pRegs + VE_H264_TRIGGER);

        H264dec_decodeSliceHeader__(context);
        
        //DBG("Decode SliceHeader..: %d", slice);
        
        int i;
		/* write RefPicLists */
		if (h->slice_type != SLICE_TYPE_I && h->slice_type != SLICE_TYPE_SI) {
			writel(VE_SRAM_H264_REF_LIST0, pThis->ve.pRegs + VE_H264_RAM_WRITE_PTR);
            //DBG("write RefPicLists");
			for (i = 0; i < h->num_ref_idx_l0_active_minus1 + 1; i += 4) {
				int j;
				uint32_t list = 0;
                //DBG("write RefPicLists:: %d, num_ref_idx_l0_active_minus1: %d", i, h->num_ref_idx_l0_active_minus1+1);
				for (j = 0; j < 4; j++) {
					if (h->RefPicList0[i + j].surface) {
                        //printf("\nTRUE surface: %p, Flags: %d, TopPicOrderCnt: %d, BottomPicOrderCnt: %d, FrameIdx: %d, ", 
                        //h->RefPicList0[i + j]->surface, h->RefPicList0[i + j]->flags,
                        //h->RefPicList0[i + j]->top_pic_order_cnt, h->RefPicList0[i + j]->bottom_pic_order_cnt,
                        //h->RefPicList0[i + j]->frame_idx);
                        if (h->RefPicList0[i + j].surface != NULL) {
                            list |= ((h->RefPicList0[i + j].surface->pos * 2 + (h->RefPicList0[i + j].field == PIC_BOTTOM_FIELD)) << (j * 8));
                        }
                    } else {
                        //printf("\nFALSE, ");
                    }
                }
				writel(list, pThis->ve.pRegs + VE_H264_RAM_WRITE_DATA);
                
			}
		}
        //DBG("write RefPicListsExit");
		if (h->slice_type == SLICE_TYPE_B) {
            DBG("SLICE_TYPE_B");
			writel(VE_SRAM_H264_REF_LIST1, pThis->ve.pRegs + VE_H264_RAM_WRITE_PTR);
			for (i = 0; i < h->num_ref_idx_l1_active_minus1 + 1; i += 4) {
				int j;
				uint32_t list = 0;
				for (j = 0; j < 4; j++) {
					if (h->RefPicList1[i + j].surface) {
                        if (h->RefPicList1[i + j].surface != NULL) {
                            list |= ((h->RefPicList1[i + j].surface->pos * 2 + (h->RefPicList1[i + j].field  == PIC_BOTTOM_FIELD)) << (j * 8));
                        }
                    }
                }
				writel(list, pThis->ve.pRegs + VE_H264_RAM_WRITE_DATA);
			}
		}

		/* picture parameters */
        writel(((info->entropy_coding_mode_flag & 0x1) << 15)
			| ((info->num_ref_idx_l0_active_minus1 & 0x1f) << 10)
			| ((info->num_ref_idx_l1_active_minus1 & 0x1f) << 5)
			| ((info->weighted_pred_flag & 0x1) << 4)
			| ((info->weighted_bipred_idc & 0x3) << 2)
			| ((info->constrained_intra_pred_flag & 0x1) << 1)
			| ((info->transform_8x8_mode_flag & 0x1) << 0)
			, pThis->ve.pRegs + VE_H264_PIC_HDR);

        /* sequence parameters */
		writel((0x1 << 19)
			| ((context->info->frame_mbs_only_flag & 0x1) << 18)
			| ((context->info->mb_adaptive_frame_field_flag & 0x1) << 17)
			| ((context->info->direct_8x8_inference_flag & 0x1) << 16)
			| ((context->picture_width_in_mbs_minus1 & 0xff) << 8)
			| ((context->picture_height_in_mbs_minus1 & 0xff) << 0)
			, pThis->ve.pRegs + VE_H264_FRAME_SIZE);

		/* slice parameters */
		writel((((h->first_mb_in_slice % (context->picture_width_in_mbs_minus1 + 1)) & 0xff) << 24)
            | (((h->first_mb_in_slice / (context->picture_width_in_mbs_minus1 + 1)) & 0xff) *
				(output->pic_type == PIC_TYPE_MBAFF ? 2 : 1) << 16)
			| ((info->is_reference & 0x1) << 12)
			| ((h->slice_type & 0xf) << 8)
			| ((slice == 0 ? 0x1 : 0x0) << 5)
			| ((info->field_pic_flag & 0x1) << 4)
			| ((info->bottom_field_flag & 0x1) << 3)
			| ((h->direct_spatial_mv_pred_flag & 0x1) << 2)
			| ((h->cabac_init_idc & 0x3) << 0)
			, pThis->ve.pRegs + VE_H264_SLICE_HDR);

		writel(((h->num_ref_idx_l0_active_minus1 & 0x1f) << 24)
			| ((h->num_ref_idx_l1_active_minus1 & 0x1f) << 16)
			| ((h->num_ref_idx_active_override_flag & 0x1) << 12)
			| ((h->disable_deblocking_filter_idc & 0x3) << 8)
			| ((h->slice_alpha_c0_offset_div2 & 0xf) << 4)
			| ((h->slice_beta_offset_div2 & 0xf) << 0)
			, pThis->ve.pRegs + VE_H264_SLICE_HDR2);

		/* qp offsets */
        writel(((context->default_scaling_lists & 0x1) << 24)
			| ((info->second_chroma_qp_index_offset & 0x3f) << 16)
			| ((info->chroma_qp_index_offset & 0x3f) << 8)
			| (((info->pic_init_qp_minus26 + 26 + h->slice_qp_delta) & 0x3f) << 0)
			, pThis->ve.pRegs + VE_H264_QP_PARAM);

        /* run decoder */
		/* clear status flags */
        //DBG("start: 0x%x", readl(pThis->ve.pRegs + VE_H264_STATUS));
		writel(readl(pThis->ve.pRegs + VE_H264_STATUS), pThis->ve.pRegs + VE_H264_STATUS);
		/* enable int */
		writel(readl(pThis->ve.pRegs + VE_H264_CTRL) | 0x7, pThis->ve.pRegs + VE_H264_CTRL);
		/* SHOWTIME - TRIGGER DECODER */
        //DBG("Start Decode: 0x%x, 0x%x", readl(pThis->ve.pRegs + VE_STATUS), readl(pThis->ve.pRegs + VE_H264_STATUS));
		writel(0x8, pThis->ve.pRegs + VE_H264_TRIGGER);
		ve_wait(1);
        //DBG("Stop Decode");
		/* clear status flags */
        uint32_t status;
        status = readl(pThis->ve.pRegs + VE_H264_STATUS);
        //printf("Status: 0x%x\n", status);
        writel(status, pThis->ve.pRegs + VE_H264_STATUS);
        //writel(readl(pThis->ve.pRegs + VE_H264_STATUS), pThis->ve.pRegs + VE_H264_STATUS);

		pos = (readl(pThis->ve.pRegs + VE_H264_VLD_OFFSET) / 8) - 3;
	}
    
    ret = VC_STATUS_OK;

err_ve_put:
	/* stop H264 decoder subengine */
    Ve_releaseSubengine__(pThis, SUBENG_H264_DEC);             /* release H264 Decoder, unlock HW */
err_free:
	free(context);                                                                /* free context */
    
	return ret;
}

static int H264dec_findStartcode__ (const uint8_t *data, int len, int start) {

	int pos, zeros = 0;
    
	for (pos = start; pos < len; pos++) {
		if (data[pos] == 0x00) {
			zeros++;
		} else if (data[pos] == 0x01 && zeros >= 2) {//0 0 1 or 0 0 0 1
			return pos - 2;
		} else {
			zeros = 0;
        }
	}

	return -1;
}

static int H264dec_fillFrameLists__ (Appl_t *pThis, H264Context_t *context) {

	int i;

	/* collect reference frames */
	h264Picture_t *frame_list[18];
	memset(frame_list, 0, sizeof(frame_list));
    
    int output_placed = 0;

	for (i = 0; i < 16; i++) {
		const ReferenceFrameH264_t *rf = &(context->info->referenceFrames[i]);
		if (rf->surface != VC_INVALID_HANDLE) {
			if (rf->is_long_term) {
				DBG(KRED"NOT IMPLEMENTED: We got a longterm reference!"KNRM);
            }

			VideoSurface_t *surface = Handle_get(rf->surface);
            H264dec_getSurfacePriv__(context, surface);
            if (!surface->extra_data) {
                DBG(KRED"!surface->extra_data"KNRM);
                assert(FALSE);
                return 0;
            }
            
            if (surface == context->output) {
				output_placed = 1;
            }
            
			context->ref_pic[context->ref_count].surface = surface;
			context->ref_pic[context->ref_count].top_pic_order_cnt = rf->field_order_cnt[0];
			context->ref_pic[context->ref_count].bottom_pic_order_cnt = rf->field_order_cnt[1];
			context->ref_pic[context->ref_count].frame_idx = rf->frame_idx;
			context->ref_pic[context->ref_count].field =
                             (rf->top_is_reference ? PIC_TOP_FIELD : 0) | 
                             (rf->bottom_is_reference ? PIC_BOTTOM_FIELD : 0);

			frame_list[surface->pos] = &context->ref_pic[context->ref_count];
			context->ref_count++;
		}
	}

	/* write picture buffer list */
	writel(VE_SRAM_H264_FRAMEBUFFER_LIST, context->regs + VE_H264_RAM_WRITE_PTR);

	for (i = 0; i < 18; i++) {
		if (!output_placed && !frame_list[i]) {
			writel((uint16_t)context->info->field_order_cnt[0], context->regs + VE_H264_RAM_WRITE_DATA);
			writel((uint16_t)context->info->field_order_cnt[1], context->regs + VE_H264_RAM_WRITE_DATA);
            writel(context->output->pic_type << 8, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(context->output->data), context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(context->output->data) + context->output->luma_size, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(context->output->extra_data), context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(context->output->extra_data) + context->video_extra_data_len, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(0, context->regs + VE_H264_RAM_WRITE_DATA);

			context->output->pos = i;
			output_placed = 1;
		} else if (!frame_list[i]) {
			int j;
			for (j = 0; j < 8; j++)
				writel(0x0, context->regs + VE_H264_RAM_WRITE_DATA);
		} else {
			VideoSurface_t *surface = frame_list[i]->surface;

			writel(frame_list[i]->top_pic_order_cnt, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(frame_list[i]->bottom_pic_order_cnt, context->regs + VE_H264_RAM_WRITE_DATA);
            writel(surface->pic_type << 8, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(surface->data), context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(surface->data) + surface->luma_size, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(surface->extra_data), context->regs + VE_H264_RAM_WRITE_DATA);
			writel(ve_virt2phys(surface->extra_data) + context->video_extra_data_len, context->regs + VE_H264_RAM_WRITE_DATA);
			writel(0, context->regs + VE_H264_RAM_WRITE_DATA);
		}
	}
    
    /* output index */
	writel(context->output->pos, context->regs + VE_H264_OUTPUT_FRAME_IDX);
    
    return 1;
}

static int H264dec_picOrderCnt__ (const h264Picture_t *pic) {

    if (pic->field == PIC_FRAME) {
		return min__(pic->top_pic_order_cnt, pic->bottom_pic_order_cnt);
	} else if (pic->field == PIC_TOP_FIELD) {
		return pic->top_pic_order_cnt;
	} else {
		return pic->bottom_pic_order_cnt;
    }
}

static int H264dec_sortRefPicsByPOC__ (const void *p1, const void *p2) {
    
	const h264Picture_t *r1 = p1;
	const h264Picture_t *r2 = p2;

	return H264dec_picOrderCnt__(r1) - H264dec_picOrderCnt__(r2);
}

static int H264dec_sortRefPicsByFrameNum__ (const void *p1, const void *p2) {
    
	const h264Picture_t *r1 = p1;
	const h264Picture_t *r2 = p2;

	return r1->frame_idx - r2->frame_idx;
}

static void H264dec_splitRefFields__ (h264Picture_t *out, h264Picture_t **in, int len, int cur_field) {

	int even = 0, odd = 0;
	int index = 0;

	while (even < len || odd < len) {
		while (even < len && !(in[even]->field & cur_field)) {
			even++;
        }
		if (even < len) {
			out[index] = *in[even++];
			out[index].field = cur_field;
			index++;
		}

		while (odd < len && !(in[odd]->field & (cur_field ^ PIC_FRAME))) {
			odd++;
        }
		if (odd < len) {
			out[index] = *in[odd++];
			out[index].field = cur_field ^ PIC_FRAME;
			index++;
		}
	}
}

static void H264dec_fillDefaultRefPicList__ (H264Context_t *context) {

	h264Header_t *h = &context->header;
	PictureInfoH264_t const *info = context->info;
    int cur_field = h->field_pic_flag ? (h->bottom_field_flag ? PIC_BOTTOM_FIELD : PIC_TOP_FIELD) : PIC_FRAME;

	if (h->slice_type == SLICE_TYPE_P) {
		qsort(context->ref_pic, context->ref_count, sizeof(context->ref_pic[0]), &H264dec_sortRefPicsByFrameNum__);

		int i;
		int ptr0 = 0;
        h264Picture_t *sorted[16];
		for (i = 0; i < context->ref_count; i++) {
			if (context->ref_pic[context->ref_count - 1 - i].frame_idx <= info->frame_num) {
				sorted[ptr0++] = &context->ref_pic[context->ref_count - 1 - i];
            }
		}
		for (i = 0; i < context->ref_count; i++) {
			if (context->ref_pic[context->ref_count - 1 - i].frame_idx > info->frame_num) {
                sorted[ptr0++] = &context->ref_pic[context->ref_count - 1 - i];
            }
		}
        
        H264dec_splitRefFields__(h->RefPicList0, sorted, context->ref_count, cur_field);
        
	} else if (h->slice_type == SLICE_TYPE_B) {
        qsort(context->ref_pic, context->ref_count, sizeof(context->ref_pic[0]), &H264dec_sortRefPicsByPOC__);
        
        int cur_poc;
		if (h->field_pic_flag) {
			cur_poc = (uint16_t)info->field_order_cnt[cur_field == PIC_BOTTOM_FIELD];
		} else {
			cur_poc = min__((uint16_t)info->field_order_cnt[0], (uint16_t)info->field_order_cnt[1]);
        }
            
		int i;
		int ptr0 = 0, ptr1 = 0;
        h264Picture_t *sorted[2][16];
		for (i = 0; i < context->ref_count; i++) {
            if (H264dec_picOrderCnt__(&context->ref_pic[context->ref_count - 1 - i]) <= cur_poc) {
				sorted[0][ptr0++] = &context->ref_pic[context->ref_count - 1  - i];
            }
            
			if (H264dec_picOrderCnt__(&context->ref_pic[i]) > cur_poc) {
				sorted[1][ptr1++] = &context->ref_pic[i];
            }
		}
		for (i = 0; i < context->ref_count; i++) {
            if (H264dec_picOrderCnt__(&context->ref_pic[i]) > cur_poc) {
				sorted[0][ptr0++] = &context->ref_pic[i];
            }

			if (H264dec_picOrderCnt__(&context->ref_pic[context->ref_count - 1 - i]) <= cur_poc) {
				sorted[1][ptr1++] = &context->ref_pic[context->ref_count - 1 - i];
            }
		}
        
        H264dec_splitRefFields__(h->RefPicList0, sorted[0], context->ref_count, cur_field);
        H264dec_splitRefFields__(h->RefPicList1, sorted[1], context->ref_count, cur_field);
	}
}

static void H264dec_decodeSliceHeader__ (H264Context_t *context) {
    
    h264Header_t *h = &context->header;
	PictureInfoH264_t const *info = context->info;
	h->num_ref_idx_l0_active_minus1 = info->num_ref_idx_l0_active_minus1;
	h->num_ref_idx_l1_active_minus1 = info->num_ref_idx_l1_active_minus1;

	h->first_mb_in_slice = get_ue__(context->regs);
	h->slice_type = get_ue__(context->regs);
	if (h->slice_type >= 5)
		h->slice_type -= 5;
	h->pic_parameter_set_id = get_ue__(context->regs);
    
    //DBGF("h->first_mb_in_slice: %d: SliceType: %d", h->first_mb_in_slice, h->slice_type);

	// separate_colour_plane_flag isn't available in VDPAU
	/*if (separate_colour_plane_flag == 1)
		colour_plane_id u(2)*/

	h->frame_num = get_u__(context->regs, info->log2_max_frame_num_minus4 + 4);

	if (!info->frame_mbs_only_flag) {
		h->field_pic_flag = get_u__(context->regs, 1);
		if (h->field_pic_flag) {
			h->bottom_field_flag = get_u__(context->regs, 1);
        }
	}

	if (h->nal_unit_type == 5) {
		h->idr_pic_id = get_ue__(context->regs);
    }

	if (info->pic_order_cnt_type == 0) {
		h->pic_order_cnt_lsb = get_u__(context->regs, info->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (info->pic_order_present_flag && !info->field_pic_flag) {
			h->delta_pic_order_cnt_bottom = get_se__(context->regs);
        }
	}

	if (info->pic_order_cnt_type == 1 && !info->delta_pic_order_always_zero_flag) {
		h->delta_pic_order_cnt[0] = get_se__(context->regs);
		if (info->pic_order_present_flag && !info->field_pic_flag) {
			h->delta_pic_order_cnt[1] = get_se__(context->regs);
        }
	}

	if (info->redundant_pic_cnt_present_flag) {
		h->redundant_pic_cnt = get_ue__(context->regs);
    }

	if (h->slice_type == SLICE_TYPE_B) {
        DBG("SLICE_TYPE_BB");
		h->direct_spatial_mv_pred_flag = get_u__(context->regs, 1);
    }

	if (h->slice_type == SLICE_TYPE_P || h->slice_type == SLICE_TYPE_SP ||
        h->slice_type == SLICE_TYPE_B) {
		h->num_ref_idx_active_override_flag = get_u__(context->regs, 1);
		if (h->num_ref_idx_active_override_flag) {
			h->num_ref_idx_l0_active_minus1 = get_ue__(context->regs);
			if (h->slice_type == SLICE_TYPE_B) {
				h->num_ref_idx_l1_active_minus1 = get_ue__(context->regs);
            }
		}
	}
    
    H264dec_fillDefaultRefPicList__(context);

	if (h->nal_unit_type == 20) {
		{}//ref_pic_list_mvc_modification(); // specified in Annex H
	} else {
		H264dec_refPicListModification__(context);
    }

	if ((info->weighted_pred_flag && (h->slice_type == SLICE_TYPE_P || h->slice_type == SLICE_TYPE_SP)) || 
        (info->weighted_bipred_idc == 1 && h->slice_type == SLICE_TYPE_B)) {
		H264dec_predWeightTable__(context);
    }

	if (info->is_reference) {
		H264dec_decRefPicMarking__(context);
    }

	if (info->entropy_coding_mode_flag && h->slice_type != SLICE_TYPE_I && 
        h->slice_type != SLICE_TYPE_SI) {
		h->cabac_init_idc = get_ue__(context->regs);
    }

	h->slice_qp_delta = get_se__(context->regs);

	if (h->slice_type == SLICE_TYPE_SP || h->slice_type == SLICE_TYPE_SI) {
		if (h->slice_type == SLICE_TYPE_SP) {
			h->sp_for_switch_flag = get_u__(context->regs, 1);
        }
		h->slice_qs_delta = get_se__(context->regs);
	}

	if (info->deblocking_filter_control_present_flag) {
		h->disable_deblocking_filter_idc = get_ue__(context->regs);
		if (h->disable_deblocking_filter_idc != 1) {
			h->slice_alpha_c0_offset_div2 = get_se__(context->regs);
			h->slice_beta_offset_div2 = get_se__(context->regs);
		}
	}

	// num_slice_groups_minus1, slice_group_map_type, slice_group_map_type aren't available in VDPAU
	/*if (num_slice_groups_minus1 > 0 && slice_group_map_type >= 3 && slice_group_map_type <= 5)
		slice_group_change_cycle u(v)*/
}

static void* H264dec_getSurfacePriv__ (H264Context_t *context, VideoSurface_t *surface) {

	if (surface->extra_data == NULL) {
		surface->extra_data = ve_malloc(context->video_extra_data_len * 2);
		if (!surface->extra_data) {
			return NULL;
		}
	}

	return surface->extra_data;
}

static void H264dec_refPicListModification__ (H264Context_t *context) {

	h264Header_t *h = &context->header;
	PictureInfoH264_t const *info = context->info;
	const int MaxFrameNum = 1 << (info->log2_max_frame_num_minus4 + 4);
	const int MaxPicNum = (info->field_pic_flag) ? 2 * MaxFrameNum : MaxFrameNum;

	if (h->slice_type != SLICE_TYPE_I && h->slice_type != SLICE_TYPE_SI) {
		int ref_pic_list_modification_flag_l0 = get_u__(context->regs, 1);
		if (ref_pic_list_modification_flag_l0) {
			unsigned int modification_of_pic_nums_idc;
			int refIdxL0 = 0;
			unsigned int picNumL0 = info->frame_num;
            if (h->field_pic_flag) {
				picNumL0 = picNumL0 * 2 + 1;
            }
            
			do {
				modification_of_pic_nums_idc = get_ue__(context->regs);
				if (modification_of_pic_nums_idc == 0 || modification_of_pic_nums_idc == 1) {
					unsigned int abs_diff_pic_num_minus1 = get_ue__(context->regs);

					if (modification_of_pic_nums_idc == 0) {
						picNumL0 -= (abs_diff_pic_num_minus1 + 1);
					} else {
						picNumL0 += (abs_diff_pic_num_minus1 + 1);
                    }

					picNumL0 &= (MaxPicNum - 1);
                    
                    int frame_num = picNumL0;
					int field = PIC_FRAME;

					if (h->field_pic_flag) {
						field = h->bottom_field_flag ? PIC_BOTTOM_FIELD : PIC_TOP_FIELD;
						if (!(frame_num & 1)) {
							field ^= PIC_FRAME;
                        }

						frame_num /= 2;
					}

					int i, j;
					for (i = 0; i < context->ref_count; i++) {
						if (context->ref_pic[i].frame_idx == frame_num) {
							break;
                        }
					}

					for (j = h->num_ref_idx_l0_active_minus1 + 1; j > refIdxL0; j--) {
						h->RefPicList0[j] = h->RefPicList0[j - 1];
                    }
                    h->RefPicList0[refIdxL0] = context->ref_pic[i];
                    if (h->field_pic_flag) {
						h->RefPicList0[refIdxL0].field = field;
                    }
					i = ++refIdxL0;
					for (j = refIdxL0; j <= h->num_ref_idx_l0_active_minus1 + 1; j++) {
						if (h->RefPicList0[j].frame_idx != frame_num || h->RefPicList0[j].field != field) {
							h->RefPicList0[i++] = h->RefPicList0[j];
                        }
                    }
				} else if (modification_of_pic_nums_idc == 2) {
					DBG("NOT IMPLEMENTED: modification_of_pic_nums_idc == 2");
					unsigned int long_term_pic_num = get_ue__(context->regs);
                    UNUSED_ARGUMENT(long_term_pic_num);
				}
			} while (modification_of_pic_nums_idc != 3);
		}
	}

	if (h->slice_type == SLICE_TYPE_B) {
        DBG("SLICE_TYPE_BBB");
		int ref_pic_list_modification_flag_l1 = get_u__(context->regs, 1);
		if (ref_pic_list_modification_flag_l1) {
			DBG("NOT IMPLEMENTED: ref_pic_list_modification_flag_l1 == 1");
			unsigned int modification_of_pic_nums_idc;
			do {
				modification_of_pic_nums_idc = get_ue__(context->regs);
				if (modification_of_pic_nums_idc == 0 || modification_of_pic_nums_idc == 1) {
					unsigned int abs_diff_pic_num_minus1 = get_ue__(context->regs);
                    UNUSED_ARGUMENT(abs_diff_pic_num_minus1);
				} else if (modification_of_pic_nums_idc == 2) {
					unsigned int long_term_pic_num = get_ue__(context->regs);
                    UNUSED_ARGUMENT(long_term_pic_num);
				}
			} while (modification_of_pic_nums_idc != 3);
		}
	}
}

static void H264dec_predWeightTable__ (H264Context_t *context) {

	h264Header_t *h = &context->header;
	int i, j, ChromaArrayType = 1;

	h->luma_log2_weight_denom = get_ue__(context->regs);
	if (ChromaArrayType != 0) {
		h->chroma_log2_weight_denom = get_ue__(context->regs);
    }

	for (i = 0; i < 32; i++) {
		h->luma_weight_l0[i] = (1 << h->luma_log2_weight_denom);
		h->luma_weight_l1[i] = (1 << h->luma_log2_weight_denom);
		h->chroma_weight_l0[i][0] = (1 << h->chroma_log2_weight_denom);
		h->chroma_weight_l1[i][0] = (1 << h->chroma_log2_weight_denom);
		h->chroma_weight_l0[i][1] = (1 << h->chroma_log2_weight_denom);
		h->chroma_weight_l1[i][1] = (1 << h->chroma_log2_weight_denom);
	}

	for (i = 0; i <= h->num_ref_idx_l0_active_minus1; i++) {
		int luma_weight_l0_flag = get_u__(context->regs, 1);
		if (luma_weight_l0_flag) {
			h->luma_weight_l0[i] = get_se__(context->regs);
			h->luma_offset_l0[i] = get_se__(context->regs);
		}
		if (ChromaArrayType != 0) {
			int chroma_weight_l0_flag = get_u__(context->regs, 1);
			if (chroma_weight_l0_flag)
				for (j = 0; j < 2; j++) {
					h->chroma_weight_l0[i][j] = get_se__(context->regs);
					h->chroma_offset_l0[i][j] = get_se__(context->regs);
				}
		}
	}

	if (h->slice_type == SLICE_TYPE_B) {
        DBG("SLICE_TYPE_BBBB");
		for (i = 0; i <= h->num_ref_idx_l1_active_minus1; i++) {
			int luma_weight_l1_flag = get_u__(context->regs, 1);
			if (luma_weight_l1_flag) {
				h->luma_weight_l1[i] = get_se__(context->regs);
				h->luma_offset_l1[i] = get_se__(context->regs);
			}
			if (ChromaArrayType != 0) {
				int chroma_weight_l1_flag = get_u__(context->regs, 1);
				if (chroma_weight_l1_flag) {
					for (j = 0; j < 2; j++) {
						h->chroma_weight_l1[i][j] = get_se__(context->regs);
						h->chroma_offset_l1[i][j] = get_se__(context->regs);
					}
                }
			}
		}
    }

	writel(((h->chroma_log2_weight_denom & 0xf) << 4)
		| ((h->luma_log2_weight_denom & 0xf) << 0)
		, context->regs + VE_H264_PRED_WEIGHT);

	writel(VE_SRAM_H264_PRED_WEIGHT_TABLE, context->regs + VE_H264_RAM_WRITE_PTR);
	for (i = 0; i < 32; i++) {
		writel(((h->luma_offset_l0[i] & 0x1ff) << 16)
			| (h->luma_weight_l0[i] & 0xff), context->regs + VE_H264_RAM_WRITE_DATA);
    }
	for (i = 0; i < 32; i++) {
		for (j = 0; j < 2; j++) {
			writel(((h->chroma_offset_l0[i][j] & 0x1ff) << 16)
				| (h->chroma_weight_l0[i][j] & 0xff), context->regs + VE_H264_RAM_WRITE_DATA);
        }
    }
	for (i = 0; i < 32; i++) {
		writel(((h->luma_offset_l1[i] & 0x1ff) << 16)
			| (h->luma_weight_l1[i] & 0xff), context->regs + VE_H264_RAM_WRITE_DATA);
    }
	for (i = 0; i < 32; i++) {
		for (j = 0; j < 2; j++) {
			writel(((h->chroma_offset_l1[i][j] & 0x1ff) << 16)
				| (h->chroma_weight_l1[i][j] & 0xff), context->regs + VE_H264_RAM_WRITE_DATA);
        }
    }
}

static void H264dec_decRefPicMarking__ (H264Context_t *context) {

	h264Header_t *h = &context->header;
    
	// only reads bits to allow decoding, doesn't mark anything
	if (h->nal_unit_type == NAL_SLICE_IDR) {
		get_u__(context->regs, 1);
		get_u__(context->regs, 1);
	} else {
		int adaptive_ref_pic_marking_mode_flag = get_u__(context->regs, 1);
		if (adaptive_ref_pic_marking_mode_flag) {
			unsigned int memory_management_control_operation;
			do {
				memory_management_control_operation = get_ue__(context->regs);
				if (memory_management_control_operation == 1 || memory_management_control_operation == 3) {
					get_ue__(context->regs);
				}
				if (memory_management_control_operation == 2) {
					get_ue__(context->regs);
				}
				if (memory_management_control_operation == 3 || memory_management_control_operation == 6) {
					get_ue__(context->regs);
				}
				if (memory_management_control_operation == 4) {
					get_ue__(context->regs);
				}
			} while (memory_management_control_operation != 0);
		}
	}
}

static uint32_t get_u__ (void *regs, int num) {
    
	writel(0x00000002 | (num << 8), regs + VE_H264_TRIGGER);

    while (readl(regs + VE_H264_STATUS) & (1 << 8));

	return readl(regs + VE_H264_BASIC_BITS);
}

static uint32_t get_ue__ (void *regs) {
    
	writel(0x00000005, regs + VE_H264_TRIGGER);

	while (readl(regs + VE_H264_STATUS) & (1 << 8));

	return readl(regs + VE_H264_BASIC_BITS);
}

static int32_t get_se__ (void *regs) {

	writel(0x00000004, regs + VE_H264_TRIGGER);

	while (readl(regs + VE_H264_STATUS) & (1 << 8));

	return readl(regs + VE_H264_BASIC_BITS);
}

static int H264dec_handleSps__ (Appl_t *pThis) {
    
    /* Populate GstH264SPS */
    gst_h264_parser_parse_sps(pThis->ve.h264dec.gst.parser,
             pThis->ve.h264dec.gst.nalu, pThis->ve.h264dec.gst.sps, (gboolean)TRUE);
    /* A.4.1 General tier and level limits. Calculate MaxDpbSize.*/
    /* TODO - Make this more general. This is written against the
       NVIDIA VDPAU implementation which supports Tier 5.1. */
    /* TODO - Move this into update_picture_info_sps ? */
    pThis->ve.h264dec.width = pThis->ve.h264dec.gst.sps->width;
    pThis->ve.h264dec.height = pThis->ve.h264dec.gst.sps->height;

    return 0;
}

static int H264dec_handlePps__ (Appl_t *pThis) {

    /* Populate GstH264PPS */
    gst_h264_parser_parse_pps(pThis->ve.h264dec.gst.parser,
            pThis->ve.h264dec.gst.nalu, pThis->ve.h264dec.gst.pps);
    return 0;
}

static int H264dec_handleSei__ (Appl_t *pThis) {

    /* Populate GstH264SEIMessage */
    //TODO
    //gst_h264_parser_parse_sei(pThis->ve.h264dec.gst.parser,
            //pThis->ve.h264dec.gst.nalu, &pThis->ve.h264dec.gst.sei);
            
    return 0;
}

static int H264dec_idr__ (H264dec_t *h264Dec, H264Frame_t *h264Frame) {

    GstH264SliceHdr *slice;
    GstH264SPS      *seq;
    
    h264Dec->poc_msb = 0;
    h264Dec->prev_poc_lsb = 0;
    
    /* slice */
    slice = &h264Frame->slice_hdr;
    if (slice->dec_ref_pic_marking.no_output_of_prior_pics_flag) {
        H264dec_dpb_flush__(&h264Dec->dpb, FALSE);
    } else {
        H264dec_dpb_flush__(&h264Dec->dpb, TRUE);
    }
    
    if (slice->dec_ref_pic_marking.long_term_reference_flag) {
        //g_object_set (h264_dec->dpb, "max-longterm-frame-idx", 0, NULL);
        H264dec_dpb_set_MaxLongTermIdx__(&pThis->ve.h264dec.dpb, 0);
    } else {
        //g_object_set (h264_dec->dpb, "max-longterm-frame-idx", -1, NULL);
        H264dec_dpb_set_MaxLongTermIdx__(&pThis->ve.h264dec.dpb, -1);
    }
    
    /* sequence */
    seq = slice->pps->sequence;
    if (seq != h264Dec->gst.sps) {
        
        /* calculate framerate if we haven't got one */
        if (h264Dec->fps_n == 0 && seq->vui_parameters_present_flag) {
            GstH264VUIParams *vui;
            uint16_t par_n = 0;
            uint16_t par_d = 0;
            
            DBG("calculate framerate");

            vui = &seq->vui_parameters;

            if (H264dec_calculatePar__(vui, &par_n, &par_d)) {
                h264Dec->par_n = par_n;
                h264Dec->par_d = par_d;
            }

            if (vui->timing_info_present_flag && vui->fixed_frame_rate_flag) {
                h264Dec->fps_n = vui->time_scale;
                h264Dec->fps_d = vui->num_units_in_tick;

                if (seq->frame_mbs_only_flag) {
                    h264Dec->fps_d *= 2;
                }
            }
            DBG("par_n: %d, par_d: %d, fps_n: %d, fps_f: %d", par_n, par_d, h264Dec->fps_n, 
                    h264Dec->fps_d);
        }
        
        //g_object_set (h264_dec->dpb, "num-ref-frames", seq->num_ref_frames, NULL);
        H264dec_dpb_set_NumRefFrames__(&h264Dec->dpb, seq->num_ref_frames);
        
        h264Dec->gst.sps = seq;
    }
    
    return GST_FLOW_OK;
}

static uint32_t H264dec_calculatePoc__ (H264dec_t *h264dec, GstH264SliceHdr *slice) {
    
    GstH264PPS *pps;
    GstH264SPS *seq;
    uint32_t    poc = 0;

    pps = slice->pps;
    seq = pps->sequence;

    if (seq->pic_order_cnt_type == 0) {
        guint32 max_poc_cnt_lsb = 1 << (seq->log2_max_pic_order_cnt_lsb_minus4 + 4);

        if ((slice->pic_order_cnt_lsb < h264dec->prev_poc_lsb) &&
            ((h264dec->prev_poc_lsb - slice->pic_order_cnt_lsb) >= (max_poc_cnt_lsb / 2))) {
            h264dec->poc_msb = h264dec->poc_msb + max_poc_cnt_lsb;

        } else if ((slice->pic_order_cnt_lsb > h264dec->prev_poc_lsb) &&
            ((slice->pic_order_cnt_lsb - h264dec->prev_poc_lsb) > (max_poc_cnt_lsb / 2))) {
            h264dec->poc_msb = h264dec->poc_msb - max_poc_cnt_lsb;
        }

        poc = h264dec->poc_msb + slice->pic_order_cnt_lsb;

        h264dec->prev_poc_lsb = slice->pic_order_cnt_lsb;
    }

    return poc;
}

static int H264dec_checkScalingLists__ (H264Context_t *context) {

	const uint32_t *sl4 = (uint32_t *)&context->info->scaling_lists_4x4[0][0];
	const uint32_t *sl8 = (uint32_t *)&context->info->scaling_lists_8x8[0][0];

	int i;
	for (i = 0; i < 6 * 16 / 4; i++) {
		if (sl4[i] != 0x10101010) {
			return 0;
        }
    }

	for (i = 0; i < 2 * 64 / 4; i++) {
		if (sl8[i] != 0x10101010) {
			return 0;
        }
    }

	return 1;
}

//h264dec DBP
static void H264dec_dpb_fillReferenceFrames__ (H264Dpb_t *dpb, 
                                               ReferenceFrameH264_t reference_frames[16]) {
    H264Frame_t   **frames;
    uint32_t        i;

    /* fill used refenences */
    frames = dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        H264Frame_t *frame = frames[i];

        reference_frames[i].surface = frame-> surface;
        reference_frames[i].is_long_term = frame->is_long_term;
        reference_frames[i].top_is_reference = frame->is_reference;
        reference_frames[i].bottom_is_reference = frame->is_reference;
        reference_frames[i].field_order_cnt[0] = frame->poc;
        reference_frames[i].field_order_cnt[1] = frame->poc;
        reference_frames[i].frame_idx = frame->frame_idx;
    }

    /* set other references as unused */
    for (i = dpb->n_frames; i < MAX_REFERENCES; i++) {
        reference_frames[i].surface = VC_INVALID_HANDLE;
        reference_frames[i].top_is_reference = FALSE;
        reference_frames[i].bottom_is_reference = FALSE;
    }
}

static void H264dec_dpb_remove__ (H264Dpb_t *dpb, uint32_t idx) {

    H264Frame_t   **frames;
    uint32_t        i;

    /* remove specific frame "idx" from frame list */
    /* it is called with is_reference == FALSE and output_needed == FALSE */
    frames = dpb->frames;
    dpb->n_frames--;
    for (i = idx; i < dpb->n_frames; i++) {
        frames[i] = frames[i + 1];
    }
}

static GstFlowReturn H264dec_dpb_output__ (H264Dpb_t *dpb, uint32_t idx) {

    GstFlowReturn ret;
    H264Frame_t *frame = dpb->frames[idx];

    ret = dpb->output(dpb, frame, dpb->user_data);
    frame->output_needed = FALSE;

    if (!frame->is_reference) {
        H264dec_dpb_remove__(dpb, idx);
    }

  return ret;
}

static bool_t H264dec_dpb_bump__ (H264Dpb_t *dpb, uint32_t poc, GstFlowReturn *ret) {

    H264Frame_t   **frames;
    uint32_t        i;
    int32_t         bump_idx;

    frames = dpb->frames;
    bump_idx = -1;
    
    for (i = 0; i < dpb->n_frames; i++) {
        if (frames[i]->output_needed) {
          bump_idx = i;
          break;
        }
    }

    if (bump_idx != -1) {
        for (i = bump_idx + 1; i < dpb->n_frames; i++) {
            /* show frame with lowest poc first */
          if (frames[i]->output_needed && (frames[i]->poc < frames[bump_idx]->poc)) {
            bump_idx = i;
          }
        }

        if (frames[bump_idx]->poc < poc) {              /* show frame only if poc is lower then.. */
          *ret = H264dec_dpb_output__(dpb, bump_idx);
          return TRUE;
        }
    }
    
    return FALSE;
}

static GstFlowReturn H264dec_dpb_add__ (H264Dpb_t *dpb, H264Frame_t *h264_frame) {

    GstFlowReturn ret;
    
    assert(dpb != NULL);
    assert(h264_frame != NULL);

    //DBG("ADD frame %d with poc: %d, is_reference: %s", h264_frame->id, h264_frame->poc, 
    //     h264_frame->is_reference == TRUE ? "TRUE" : "FALSE");

    if ((h264_frame->is_reference && h264_frame->is_long_term) &&
        ((int32_t)h264_frame->frame_idx > dpb->max_longterm_frame_idx)) {
        h264_frame->is_reference = FALSE;
    }
    

    if (h264_frame->is_reference) {
        ret = GST_FLOW_OK;
        while (dpb->n_frames >= dpb->max_frames) {
            //ako nema mesta za novi frejm prikazati frejm koji je spreman za output i osloboditi to parce za novi frejm
            //DBG("Try to dump: dpb->n_frames >= dpb->max_frames -> %d >= %d", dpb->n_frames, dpb->max_frames);
            if (!H264dec_dpb_bump__(dpb, G_MAXUINT, &ret)) {
                DBGF(KRED"ERROR: Couldn't make room in DPB"KNRM);
                /* what now? Try to remove the oldest frame or..?? */
                //H264dec_dpb_remove__(dpb, 0);
                return GST_FLOW_OK;
            }
        }
        //DBG("%d Added to: %d idx", h264_frame->id, dpb->n_frames);
        /* this frame is refecence, save it to frame list */
        dpb->frames[dpb->n_frames++] = h264_frame;
        
    } else {
        while (H264dec_dpb_bump__(dpb, h264_frame->poc, &ret)) {
            /* show all frames less than h264_frame->poc */
            //DBG("Try to bump %d", h264_frame->id);
            if (ret != GST_FLOW_OK) {
                DBG(KRED"ERROR: H264dec_dpb_bump__!"KNRM);
                return ret;
            }
        }
        //DBG("Frame %d is not reference put to output", h264_frame->id);
        ret = dpb->output(dpb, h264_frame, dpb->user_data);
    }
    
    return ret;
}

static void H264dec_dpb_flush__ (H264Dpb_t *dpb, bool_t output) {

    GstFlowReturn   ret;
    H264Frame_t   **frames;
    uint32_t        i;

    DBGF("flush H264dec_dpb_bump__: %s", output == TRUE ? "TRUE" : "FALSE");

    if (output) {                                                        /* show all ready frames */
        while (H264dec_dpb_bump__(dpb, G_MAXUINT, &ret));
    }

    frames = (H264Frame_t **) dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        //gst_video_frame_unref (frames[i]);
        //frames[i]->surface = VC_INVALID_HANDLE;//ovo nikako.. ako ovo setujem smatrace da treba da kreira novi surface..
        memset(&frames[i]->slice_hdr, 0, sizeof(GstH264SliceHdr));
        frames[i]->poc = 0;
        frames[i]->frame_idx = 0;
        frames[i]->is_reference = FALSE;
        frames[i]->is_long_term = FALSE;
        frames[i]->output_needed = FALSE;
        frames[i]->id = 0;
    }

    dpb->n_frames = 0;
}

static void H264dec_dpb_markSliding__ (H264Dpb_t *dpb) {

    H264Frame_t   **frames;
    uint32_t        i;
    int32_t         mark_idx = -1;

    if (dpb->n_frames != dpb->max_frames) {
        return;
    }

    frames = dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        if (frames[i]->is_reference && !frames[i]->is_long_term) {
          mark_idx = i;
          break;
        }
    }

    if (mark_idx != -1) {
        for (i = mark_idx; i < dpb->n_frames; i++) {
            if (frames[i]->is_reference && !frames[i]->is_long_term &&
                frames[i]->frame_idx < frames[mark_idx]->frame_idx) {
                mark_idx = i;
            }
        }

        frames[mark_idx]->is_reference = FALSE;
        if (!frames[mark_idx]->output_needed) {
            H264dec_dpb_remove__(dpb, mark_idx);
        }
    }
}

static void H264dec_dpb_markLongTerm__ (H264Dpb_t *dpb, uint32_t pic_num, uint32_t long_term_frame_idx) {

    H264Frame_t   **frames;
    guint           i;
    gint            mark_idx = -1;

    frames = dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        if (frames[i]->is_reference && !frames[i]->is_long_term &&
            frames[i]->frame_idx == pic_num) {
            mark_idx = i;
            break;
        }
    }

    if (mark_idx != -1) {
        frames[mark_idx]->is_long_term = TRUE;
        frames[mark_idx]->frame_idx = long_term_frame_idx;
    }
}

static void H264dec_dpb_markShortTermUnused__ (H264Dpb_t *dpb, uint32_t pic_num) {
    
    H264Frame_t  **frames;
    uint32_t        i;
    int32_t         mark_idx = -1;
    
    DBGF("pic_num: %d", pic_num);

    frames = dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        if (frames[i]->is_reference && !frames[i]->is_long_term &&
            frames[i]->frame_idx == pic_num) {
            mark_idx = i;
            break;
        }
    }

    if (mark_idx != -1) {
        frames[mark_idx]->is_reference = FALSE;
        DBG("Found: mark_idx: %d, output_needed: %d", mark_idx, frames[mark_idx]->output_needed);
        if (!frames[mark_idx]->output_needed) {
            DBG("Remove them..");
            H264dec_dpb_remove__(dpb, mark_idx);
        }
    }
}

static void H264dec_dpb_markLongTermUnused__ (H264Dpb_t *dpb, uint32_t long_term_pic_num) {

    H264Frame_t   **frames;
    uint32_t        i;
    int32_t         mark_idx = -1;

    frames = dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        if (frames[i]->is_reference && frames[i]->is_long_term &&
            frames[i]->frame_idx == long_term_pic_num) {
            mark_idx = i;
            break;
        }
    }

    if (mark_idx != -1) {
        frames[mark_idx]->is_reference = FALSE;
        if (!frames[mark_idx]->output_needed) {
            H264dec_dpb_remove__(dpb, mark_idx);
        }
    }
}

static void H264dec_dpb_markAllUnused__ (H264Dpb_t *dpb) {

    H264Frame_t   **frames;
    uint32_t        i;

    frames = dpb->frames;
    for (i = 0; i < dpb->n_frames; i++) {
        frames[i]->is_reference = FALSE;
        if (!frames[i]->output_needed) {
            H264dec_dpb_remove__(dpb, i);
            i--;
        }
    }
}

static void H264dec_dpb_setOutputFunc__ (H264Dpb_t *dpb, H264DPBOutputFunc func, void *user_data) {

    if (dpb == NULL) {
        return;
    }

    dpb->output = func;
    dpb->user_data = user_data;
}

static void H264dec_dpb_init__ (H264Dpb_t *dpb) {

    assert(dpb != NULL);
    uint32_t        i;

    
    dpb->n_frames = 0;
    dpb->max_longterm_frame_idx = -1;
    dpb->max_frames = MAX_REFERENCES;
    dpb->lastUsedDpbId = 0;
    
    /* init reference frames */
    for (i = 0; i < MAX_DPB_SIZE; i++) {
        dpb->scratch_frames[i].surface = VC_INVALID_HANDLE;
        dpb->scratch_frames[i].is_reference = FALSE;
        dpb->scratch_frames[i].output_needed = FALSE;
    }
}

static H264Frame_t* H264dec_dpb_alloc__ (H264dec_t *h264dec) {
    
    assert(h264dec != NULL);
    H264Dpb_t *dpb = &h264dec->dpb;
    uint32_t    i;

    /* get reference frame */
    for (i = h264dec->dpb.lastUsedDpbId; i <= h264dec->dpb.max_frames + NUM_EXTRA_DPB_BUFF; i++) {
        /* try to find already allocated surface */
        if ((dpb->scratch_frames[i].is_reference == FALSE) &&
            (dpb->scratch_frames[i].output_needed == FALSE)) {
            
            if (dpb->scratch_frames[i].surface != VC_INVALID_HANDLE) {
                DBGF("H264 Decoder => use old DPB surface: %d",  dpb->scratch_frames[i].surface);
                h264dec->dpb.lastUsedDpbId = i;
                return &dpb->scratch_frames[i];
            } else {
                /* try to allocate new surface */
                if (VideoSurface_create(CHROMA_TYPE_420, h264dec->width, h264dec->height,
                                        &dpb->scratch_frames[i].surface) != VC_STATUS_OK) {
                    DBGF(KRED"ERROR: H264 Decoder => create DPB reference surface!"KNRM);
                    h264dec->dpb.lastUsedDpbId = 0;
                    return NULL;
                } else {
                    DBGF("H264 Decoder => create DPB reference surface: %d", 
                            dpb->scratch_frames[i].surface);
                    h264dec->dpb.lastUsedDpbId = i;
                    return &dpb->scratch_frames[i];
                }
            }
        }
    }
    
    /* get reference frame */
    for (i = 0; i <= h264dec->dpb.max_frames + NUM_EXTRA_DPB_BUFF; i++) {
        /* try to find already allocated surface */
        if ((dpb->scratch_frames[i].is_reference == FALSE) &&
            (dpb->scratch_frames[i].output_needed == FALSE)) {
            
            if (dpb->scratch_frames[i].surface != VC_INVALID_HANDLE) {
                DBGF("H264 Decoder => use old DPB surface: %d",  dpb->scratch_frames[i].surface);
                h264dec->dpb.lastUsedDpbId = i;
                return &dpb->scratch_frames[i];
            } else {
                /* try to allocate new surface */
                if (VideoSurface_create(CHROMA_TYPE_420, h264dec->width, h264dec->height,
                                        &dpb->scratch_frames[i].surface) != VC_STATUS_OK) {
                    h264dec->dpb.lastUsedDpbId = 0;
                    DBGF(KRED"ERROR: H264 Decoder => create DPB reference surface!"KNRM);
                    return NULL;
                } else {
                    DBGF("H264 Decoder => create DPB reference surface: %d", 
                            dpb->scratch_frames[i].surface);
                    h264dec->dpb.lastUsedDpbId = i;
                    return &dpb->scratch_frames[i];
                }
            }
        }
    }
    
    return NULL;
}

__attribute__((unused)) static bool_t H264dec_dpb_free__ (H264Dpb_t *dpb, H264Frame_t *h264_frame) {

    assert(dpb);
    assert(h264_frame);
    uint32_t i;
    
    for (i = 0; i < MAX_DPB_SIZE; i++) {
        /* try to find this DPB surface */
        if ((&dpb->scratch_frames[i] == h264_frame)) {
            //dpb->dpb_reference_values[i] = DPB_UNUSED_FOR_REFERENCE;
            //todo..
            return TRUE;
        }
    }
    
    DBGF(KRED"ERROR"KNRM);
    return FALSE;
}

static void H264dec_dpb_finalize__ (H264Dpb_t *dpb) {

    assert(dpb != NULL);
    uint32_t        i;

    for (i = 0; i < MAX_DPB_SIZE; i++) {
        if (dpb->scratch_frames[i].surface != VC_INVALID_HANDLE) {
            DBG("Destroy DPB reference surface: %d->%d", i, dpb->scratch_frames[i].surface);
            if (VideoSurface_destroy(dpb->scratch_frames[i].surface) !=  VC_STATUS_OK) {
                DBGF(KRED"ERROR: H264 Decoder => destroy DPB reference surface!"KNRM);
            } else {
                dpb->scratch_frames[i].surface = VC_INVALID_HANDLE;
                memset(&dpb->scratch_frames[i].slice_hdr, 0, sizeof(GstH264SliceHdr));
                dpb->scratch_frames[i].poc = 0;
                dpb->scratch_frames[i].frame_idx = 0;
                dpb->scratch_frames[i].is_reference = FALSE;
                dpb->scratch_frames[i].is_long_term = FALSE;
                dpb->scratch_frames[i].output_needed = FALSE;
                dpb->scratch_frames[i].id = 0;
            }
        }
    }
}

static void H264dec_dpb_set_NumRefFrames__ (H264Dpb_t *dpb, uint32_t numRefFrames) {
    
    uint32_t        i;
    GstFlowReturn   ret;

    dpb->max_frames = numRefFrames;
    for (i = dpb->n_frames; i > dpb->max_frames; i--) {
        H264dec_dpb_bump__(dpb, G_MAXUINT, &ret);
    }
}

static void H264dec_dpb_set_MaxLongTermIdx__ (H264Dpb_t *dpb, int32_t maxLongTermIdx) {

      H264Frame_t **frames;
      uint32_t      i;

      dpb->max_longterm_frame_idx = maxLongTermIdx;

      frames = dpb->frames;
      for (i = dpb->n_frames; i < dpb->n_frames; i++) {
          if (frames[i]->is_reference && frames[i]->is_long_term &&
              (int32_t)frames[i]->frame_idx > dpb->max_longterm_frame_idx) {
              frames[i]->is_reference = FALSE;
              if (!frames[i]->output_needed) {
                  H264dec_dpb_remove__(dpb, i);
                  i--;
              }
          }
      }
}

static GstFlowReturn H264dec_output__ (H264Dpb_t *dpb, H264Frame_t *h264_frame, void *user_data) {

    Appl_t *pThis;
    VideoSurface_t *dpbSurface;
    VideoSurface_t *outputSurface;
    uint32_t        outHandleId;
        /* global and loval variable pThis have same name but local variable will take preference */
    pThis = (Appl_t*)user_data;
    
    if ((dpbSurface = Handle_get(h264_frame->surface)) == NULL) {
        DBGF(KRED"ERROR: dpbSurface"KNRM);
    }
    outHandleId = (pThis->ve.h264dec.id % 2);
    outputSurface = Handle_get(pThis->ve.h264dec.output[outHandleId]);
    
    /* copy data to output surface */
    memcpy(outputSurface->data, dpbSurface->data, dpbSurface->size);
    ve_flush_cache(outputSurface->data, outputSurface->size);
    
    DBG(KBLU"H264DEC: Show frame %d.. poc: %d, surfaceId: %d"KNRM, h264_frame->id, h264_frame->poc, h264_frame->surface);
    
    /* show frame on display */
    if (pThis->disp.layer[DL_H264].initialized == FALSE) {
                                   /* initialize display and show first frame */
        if (Disp_init__(&pThis->disp.layer[DL_H264], DISP_FORMAT_YUV420, 
            DISP_MOD_MB_UV_COMBINED, DISP_SEQ_UVUV, pThis->ve.h264dec.width, 
            pThis->ve.h264dec.height, DISP_DL_H264_SCN_POS_X, DISP_DL_H264_SCN_POS_Y, 
            DISP_DL_H264_SCN_WIDTH, DISP_DL_H264_SCN_HEIGHT) == 0) {
            Disp_start__(&pThis->disp.layer[DL_H264]);
            Disp_on__();
            pThis->disp.layer[DL_H264].initialized = TRUE;
            /* show first frame */
            Disp_newDecoder_frame__(&pThis->disp.layer[DL_H264], 
            pThis->ve.h264dec.width, 
            pThis->ve.h264dec.height, ve_virt2phys(outputSurface->data),
            (ve_virt2phys(outputSurface->data) + outputSurface->luma_size), 
            pThis->ve.h264dec.id);
            DBG("Init H264Display: Source: %dx%d", pThis->ve.h264dec.width,
                                                   pThis->ve.h264dec.height);
        }

    } else {                                               /* show next frame */
        Disp_newDecoder_frame__(&pThis->disp.layer[DL_H264], 
        pThis->ve.h264dec.width, 
        pThis->ve.h264dec.height, ve_virt2phys(outputSurface->data),
        (ve_virt2phys(outputSurface->data) + outputSurface->luma_size), 
        pThis->ve.h264dec.id);
        //usleep(500000);
    }
    
    return GST_FLOW_OK;
}

__attribute__((unused)) static bool_t H264dec_flush__ (Appl_t *pThis) {
    
  H264dec_t *h264_dec = &pThis->ve.h264dec;

  h264_dec->got_idr = FALSE;
  H264dec_dpb_flush__(&h264_dec->dpb, FALSE);

  return TRUE;
}

static bool_t H264dec_calculatePar__ (GstH264VUIParams *vui, uint16_t *par_n, uint16_t *par_d) {

    uint16_t aspect[16][2] = { {1, 1}, {12, 11}, {10, 11}, {16, 11}, {40, 33},
                            {24, 11}, {20, 11}, {32, 11}, {80, 33}, {18, 11}, {15, 11}, {64, 33},
                            {160, 99}, {4, 3}, {3, 2}, {2, 1}
    };

    if (vui->aspect_ratio_idc >= 1 && vui->aspect_ratio_idc <= 16) {
        *par_n = aspect[vui->aspect_ratio_idc - 1][0];
        *par_d = aspect[vui->aspect_ratio_idc - 1][1];
        return TRUE;
    } else if (vui->aspect_ratio_idc == 255) {
        *par_n = vui->sar_height;
        *par_d = vui->sar_width;
        return TRUE;
    }

    return FALSE;
}

static void H264dec_initFrameInfo__ (H264dec_t *h264_dec, GstH264NalUnit *nalu, H264Frame_t *h264_frame) {

    GstH264SliceHdr *slice;

    slice = &h264_frame->slice_hdr;

    h264_frame->poc = H264dec_calculatePoc__(h264_dec, slice);

    h264_frame->output_needed = TRUE;
    h264_frame->is_long_term  = FALSE;
    h264_frame->frame_idx = slice->frame_num;

    /* is reference */
    if (nalu->ref_idc == 0) {
        h264_frame->is_reference = FALSE;
    } else if (nalu->idr_pic_flag) {
        h264_frame->is_reference = TRUE;
        if (slice->dec_ref_pic_marking.long_term_reference_flag) {
            h264_frame->is_long_term = TRUE;
            h264_frame->frame_idx = 0;
        }
    } else {
        h264_frame->is_reference = TRUE;

        if (slice->dec_ref_pic_marking.adaptive_ref_pic_marking_mode_flag) {
            GstH264RefPicMarking *marking;
            guint i;

            marking = slice->dec_ref_pic_marking.ref_pic_marking;
            for (i = 0; i < slice->dec_ref_pic_marking.n_ref_pic_marking; i++) {
                if (marking[i].memory_management_control_operation == 6) {
                    h264_frame->is_long_term = TRUE;
                    h264_frame->frame_idx = marking[i].long_term_frame_idx;
                    break;
                }
            }
        }
    }
    
    h264_frame->id = h264_dec->id;
}

static PictureInfoH264_t H264dec_fillInfo__ (H264dec_t *h264_dec, H264Frame_t *h264_frame) {

    GstH264SliceHdr *slice;
    GstH264PPS      *pps;
    GstH264SPS      *seq;
    PictureInfoH264_t info;

    memset(&info, 0, sizeof(PictureInfoH264_t));
    
    slice = &h264_frame->slice_hdr;
    pps = slice->pps;
    seq = pps->sequence;

    //info->slice_count = h264_frame->slices->len;
    /*set initial number of slices to 1 for this frame and later increment it if needed in nal pick*/
    info.slice_count = 1;

    /* FIXME: we only handle frames for now */
    info.field_order_cnt[0] = h264_frame->poc;
    info.field_order_cnt[1] = h264_frame->poc;

    info.is_reference = h264_frame->is_reference;
    info.frame_num = slice->frame_num;

    info.field_pic_flag = slice->field_pic_flag;
    info.bottom_field_flag = slice->bottom_field_flag;
    info.num_ref_idx_l0_active_minus1 = slice->num_ref_idx_l0_active_minus1;
    info.num_ref_idx_l1_active_minus1 = slice->num_ref_idx_l1_active_minus1;

    /* sps */
    info.num_ref_frames = seq->num_ref_frames;
    info.frame_mbs_only_flag = seq->frame_mbs_only_flag;
    info.mb_adaptive_frame_field_flag = seq->mb_adaptive_frame_field_flag;
    info.log2_max_frame_num_minus4 = seq->log2_max_frame_num_minus4;
    info.pic_order_cnt_type = seq->pic_order_cnt_type;
    info.log2_max_pic_order_cnt_lsb_minus4 = seq->log2_max_pic_order_cnt_lsb_minus4;
    info.delta_pic_order_always_zero_flag = seq->delta_pic_order_always_zero_flag;
    info.direct_8x8_inference_flag = seq->direct_8x8_inference_flag;

    /* pps */
    info.constrained_intra_pred_flag = pps->constrained_intra_pred_flag;
    info.weighted_pred_flag = pps->weighted_pred_flag;
    info.weighted_bipred_idc = pps->weighted_bipred_idc;
    info.transform_8x8_mode_flag = pps->transform_8x8_mode_flag;
    info.chroma_qp_index_offset = pps->chroma_qp_index_offset;
    info.second_chroma_qp_index_offset = pps->second_chroma_qp_index_offset;
    info.pic_init_qp_minus26 = pps->pic_init_qp_minus26;
    info.entropy_coding_mode_flag = pps->entropy_coding_mode_flag;
    info.pic_order_present_flag = pps->pic_order_present_flag;
    info.deblocking_filter_control_present_flag = pps->deblocking_filter_control_present_flag;
    info.redundant_pic_cnt_present_flag = pps->redundant_pic_cnt_present_flag;

    memcpy (&info.scaling_lists_4x4, &pps->scaling_lists_4x4, 96);
    memcpy (&info.scaling_lists_8x8, &pps->scaling_lists_8x8, 128);

    H264dec_dpb_fillReferenceFrames__(&h264_dec->dpb, info.referenceFrames);
    
    return info;
}

__attribute__((unused)) static void H264dec_dumpInfo__ (PictureInfoH264_t *info) {
    
    uint32_t    i;
    
    printf("Dump Info Reference Surfaces:\n");
    /* dump references */
    for (i = 0; i < MAX_REFERENCES; i++) {
        printf("reference_frames[%d].surface: %d\n", i, info->referenceFrames[i].surface);
        printf("reference_frames[%d].is_long_term: %d\n", i, info->referenceFrames[i].is_long_term);
        printf("reference_frames[%d].top_is_reference: %d\n", i, info->referenceFrames[i].top_is_reference);
        printf("reference_frames[%d].bottom_is_reference: %d\n", i, info->referenceFrames[i].bottom_is_reference);
        printf("reference_frames[%d].field_order_cnt[0]: %d\n", i, info->referenceFrames[i].field_order_cnt[0]);
        printf("reference_frames[%d].field_order_cnt[1]: %d\n", i, info->referenceFrames[i].field_order_cnt[1]);
        printf("reference_frames[%d].frame_idx: %d\n", i, info->referenceFrames[i].frame_idx);
    }
}

//gst
static int Gst_allocateObjects__ (GstH264NalUnit** nalu, GstH264SliceHdr** slice, GstH264SPS** sps, 
                                   GstH264PPS** pps, GstH264SEIMessage** sei) {
                                       
    *slice = calloc(1, sizeof(GstH264SliceHdr));
    if(*slice == NULL) { 
        goto failure;
    }
    *sps = calloc(1, sizeof(GstH264SPS));
    if(*sps == NULL) {
        goto failure;
    }
    *pps = calloc(1, sizeof(GstH264PPS));
    if(*pps == NULL) {
        goto failure;
    }
    *sei = calloc(1, sizeof(GstH264SEIMessage));
    if(*sei == NULL) {
        goto failure;
    }
    *nalu = calloc(1, sizeof(GstH264NalUnit));
    if(*nalu == NULL) {
        goto failure;
    }

    return 0;
failure:
    Gst_freeObjects__(nalu, slice, sps, pps, sei);
    return -1;
}

static int Gst_freeObjects__ (GstH264NalUnit** nalu, GstH264SliceHdr** slice, GstH264SPS** sps, 
                              GstH264PPS** pps, GstH264SEIMessage** sei) {

    if(*nalu == NULL) {
        free(*nalu);
    }
    if(*slice == NULL) { 
        free(*slice);
    }
    if(*sps == NULL) {
        free(*sps);
    }
    if(*pps == NULL) {
        free(*pps);
    }
    if(*sei == NULL) {
        free(*sei);
    }

    return 0;
}

static int Gst_checkNaluResult__ (GstH264ParserResult result) {
    
    if(result) {
        printf("ERROR: gst_h264_parser_identify_nalu: %x ", result);
        switch(result) {
            case 0:
                printf("GST_H264_PARSER_OK\n");
                break;
            case 1:
                printf("GST_H264_PARSER_BROKEN_DATA\n");
                break;
            case 2:
                printf("GST_H264_PARSER_BROKEN_LINK\n");
                break;
            case 3:
                printf("GST_H264_PARSER_ERROR\n");
                break;
            case 4:
                printf("GST_H264_PARSER_NO_NAL\n");
                break;
            case 5:
                printf("GST_H264_PARSER_NO_NAL_END\n");
                break;
            default:
                printf("GST_H264_PARSER_UNKNOWN_ERROR\n");
                break;
        }
        return -1;
    } else {
        //printf("Got NAL.\n");
    }
    return 0;
}

/* The following functions implement a rudimentary H.264/AVC
   elementary stream parser:
   Nal_peekNextUnit__
   Nal_getNextUnit__
*/
/* Report the type of the next NAL unit in the stream.
   Assumes that stream is already positioned at start of a NAL unit.
Returns:
0, if this is a VCL NAL unit with first_slice_segment_in_pic_flag set.
-1, if end of file is found.
1, otherwise.
 */
static int Nal_peekNextUnit__ (Appl_t *pThis) {

    uint8_t    *pData = pThis->ve.h264dec.pFileData;
    uint32_t    offset = pThis->ve.h264dec.fileOffset;
    uint8_t     a, b, c, type;
    //uint8_t     temporal_id;
    //uint8_t     layer_id;
    //long        start_pos;
    uint16_t    nal_unit_header;
    int         retval;
    uint8_t     type2;
    
    /* Report on the type of this NAL Unit. */
    /* Skip the first three bytes, should be 0x0 0x0 0x1. */
    offset += 3;
    
    if (offset+3 >= pThis->ve.h264dec.s.st_size) {
        return -1;
    }
    /* nal_unit_header begins after start code prefix. */
    //start_pos = offset;
    /* Implement nal_unit_header(), 7.3.1.2, here. */
    a = (uint8_t)pData[offset++];
    b = (uint8_t)pData[offset++];
    c = (uint8_t)pData[offset++];
    if(offset >= pThis->ve.h264dec.s.st_size) {
        return -1;
    }
    nal_unit_header = a<<8 | b;
    type = (nal_unit_header & 0x7e00) >> 9;
    //layer_id = (nal_unit_header & 0x1f8 ) >> 3;
    //temporal_id = (nal_unit_header & 0x7) - 1;
    //printf("NALU at 0x%08lx, type %d, layer id %d, temporal id %d\n",
           //start_pos, type, layer_id, temporal_id);
    /* Go back to where we started. */
    retval = (type >= 0 && type < 32) ? c>>7 : 1;
    type2 = b & 0x1F;
    //DBG("a: 0x%x, b: 0x%x, c: 0x%x\nType: %d:%d => RetVal: %d", a, b, c, type, type2, retval);
    if (type2 == 0x07 || type2==0x08) {
        return 1;                                                              /* this is sps pps */
    }
    return retval;
}

/**
 Find the beginning and end of a NAL (Network Abstraction Layer) unit in a byte buffer containing H264 bitstream data.
 Return it in buf.
 @param[in]   buf        the buffer
 @param[in]   size       the size of the buffer
 @param[out]  nal_start  the beginning offset of the nal
 @param[out]  nal_end    the end offset of the nal
 @return                 the 0 if found start and end of nal, or -1 if did not find start and end of nal
 */
static int Nal_getNextUnit__ (Appl_t *pThis, const void *pBuf, int *nalLength) {

    uint32_t nalStart = 0;
    uint32_t nalEnd = 0;
    uint8_t *pData = pThis->ve.h264dec.pFileData;
    uint32_t offset = pThis->ve.h264dec.fileOffset;
    bool_t   eof = FALSE;
    
                              /* ( next_bits( 24 ) != 0x000001 && next_bits( 32 ) != 0x00000001 ) */
    while ((pData[offset] != 0 || pData[offset+1] != 0 || pData[offset+2] != 0x01) && 
           (pData[offset] != 0 || pData[offset+1] != 0 || pData[offset+2] != 0 || 
            pData[offset+3] != 0x01)) {
        offset++;                                                            /* skip leading zero */
        if (offset+4 >= pThis->ve.h264dec.s.st_size) {
            /* did not find nal start */
            return -1; 
        }
    }
    if (pData[offset] != 0 || pData[offset+1] != 0 || pData[offset+2] != 0x01) {
                                                               /* ( next_bits( 24 ) != 0x000001 ) */
        offset++;
    }
    if  (pData[offset] != 0 || pData[offset+1] != 0 || pData[offset+2] != 0x01) {
        /* error, should never happen */
        assert(FALSE);
        return -1;
    }
    nalStart = offset;
    offset += 3;
                                /* ( next_bits( 24 ) != 0x000000 && next_bits( 24 ) != 0x000001 ) */
    while ((pData[offset] != 0 || pData[offset+1] != 0 || pData[offset+2] != 0) && 
           (pData[offset] != 0 || pData[offset+1] != 0 || pData[offset+2] != 0x01)) {
        offset++;
        // FIXME the next line fails when reading a nal that ends exactly at the end of the data
        if (offset+3 >= pThis->ve.h264dec.s.st_size) {
            nalEnd = pThis->ve.h264dec.s.st_size;
            eof = TRUE;
            //return -1; 
            break;
        } // did not find nal end, stream ended first
    }
    
    pThis->ve.h264dec.fileOffset = offset;/* update file offset to the beginning of next nal unit */
    if (eof == FALSE) {
        nalEnd = offset;
    }
    *nalLength = nalEnd - nalStart + 4 + 4;
    if(*nalLength > NALU_BUFFER_LENGTH) {
        DBG(KRED"Skipping jumbo sized NALU of size %x"KNRM, *nalLength);
        return -1;
    }
    
    /* copy data to bitestream buffer */
    memcpy((void *)(pBuf), (const void *)(pData + nalStart), *nalLength);
    if (eof == TRUE) {
        ///* Need to add a start code after the NAL unit. */
        uint8_t eos[3] = { 0x0, 0x0, 0x1 };
        memcpy((void *)(pBuf + *nalLength), (const void *)&eos, 3);
        *nalLength = *nalLength + 3;
    }
    //DBG("NalLength is %d", *nalLength);
    return 0;
}


//other
static void yuv422YUYV_YUY2_2rgb__ (char *pIn, char *pOut, unsigned long len) {//todo use neon

    signed int color;
    int Y0;
    int Y1;
    int U;
    int V;
    int C;
    int D;
    int E;
    uint32_t i;
    
    for (i=0; i < len>>2; i++) {
        Y0 = *pIn++;
        U = *pIn++;
        Y1 = *pIn++;
        V = *pIn++;
        C = Y0 - 16;
        D = U - 128;
        E = V - 128;
        color = (signed int)(( 298 * C + 409 * E + 128) >> 8); // red
        if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }
        *pOut++ = (unsigned char)(color); // red
        color = (signed int)(( 298 * C - 100 * D - 208 * E + 128) >> 8); // green
        if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }
        *pOut++ = (unsigned char)(color); // green
        color = (signed int)(( 298 * C + 516 * D + 128) >> 8); // blue
        if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }
        *pOut++ = (unsigned char)(color); // blue
        //////
        C = Y1 - 16;
        color = (signed int)(( 298 * C + 409 * E + 128) >> 8); // red
        if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }
        *pOut++ = (unsigned char)(color); // red
        color = (signed int)(( 298 * C - 100 * D - 208 * E + 128) >> 8); // green
        if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }
        *pOut++ = (unsigned char)(color); // green
        color = (signed int)(( 298 * C + 516 * D + 128) >> 8); // blue
        if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }
        *pOut++ = (unsigned char)(color); // blue
    }
}

__attribute__((unused)) static void crop_nv12__ (char *pSrc, char *pDst, uint32_t srcWidth, 
                                        uint32_t srcHeight, uint32_t dstWidth, uint32_t dstHeight) {
                            
    /* crop the middle of 4:2:0 NV12 source image */
    uint32_t    i;
    uint32_t    restW;
    uint32_t    restH;
    char       *pSource;
    char       *pDestination;
    
    restW = (srcWidth - dstWidth) >> 1;                   /* NOTE: the rest must be multiple of 4 */
    restH = (srcHeight - dstHeight) >> 1;                           /* NOTE: the rest must be odd */
    
    /* copy luma */
    pSource = pSrc + (restH * srcWidth) + restW;
    pDestination = pDst;
    for (i = 0; i < dstHeight; i++) {
        memcpy(pDestination, pSource, dstWidth);
        pSource = pSource + srcWidth;
        pDestination = pDestination + dstWidth;
    }
    
    /* copy chroma */
    pSource = pSrc + (srcWidth * srcHeight) + ((restH>>1) * (srcWidth>>1)) + (restW>>2);
    pDestination = pDst + (dstWidth * dstHeight);
    for (i = 0; i < (dstHeight>>1); i++) {
        memcpy(pDestination, pSource, (dstWidth>>1));
        pSource = pSource + (srcWidth>>1);
        pDestination = pDestination + (dstWidth>>1);
    }
}

__attribute__((unused)) static void printBuffer__ (uint8_t *pBuffer, uint32_t length, 
                                                   char *pBufferName) {
    
    uint32_t    i;
    
    printf("%s: ", pBufferName);
    for (i = 0; i < length; i++) {
        if (pBuffer != NULL) {
            printf("ox%X, ", *pBuffer++);
        } else {
            break;
        }
    }
    printf("\n");
    fflush(stdout);
}

static void errno_exit__ (const char *s) {
    
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static uint32_t min__ (uint32_t a, uint32_t b) {
    
    return a<b?a:b;
}


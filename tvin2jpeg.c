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
//
#include "sunxi_disp_ioctl.h"
#include <sys/time.h>
#include <turbojpeg.h>
#include "debug.h"
#include "tvin2jpeg_cfg.h"
//
#include "vepoc.h"

#define CAMERA_DEVICE       "/dev/video1"
#define DISPLAY_DEVICE      "/dev/disp"
#define MAX_RESOLUTION      4096
#define JPEG_ENC_QUALITY    100
#define N_BUFFERS           5
#define MAX_BUFFERS         10
#define PADDING             2

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

/* type definition. */
typedef enum {
    FALSE = 0,
    TRUE  = 1
} bool_t;

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
} Thread_t;

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
    SUBENG_ISP,
    SUBENG_AVCENC,
} VeSubengType_t;

typedef enum scalerTypeTag {
    ARBITRARY_VGA = 1,
    ARBITRARY_QVGA,
    DIVIDE_PAL
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
    /* buffers */
    Buffer_t            buffers[MAX_BUFFERS];                                   /* camera buffers */
    unsigned int        n_buffers;                                 /* number of allocated buffers */
} Camera_t;

typedef struct sourceTag {
    Size_t      size;                                                              /* source size */
    uint32_t    YplaneSize;                                                          /* luma size */
    uint32_t    CplaneSize;                                                        /* chroma size */
    uint64_t    rawSize;                                                        /* raw image size */
    uint32_t    mb_width;                                           /* width in 16x16 macroblocks */
    uint32_t    mb_height;                                         /* height in 16x16 macroblocks */
    uint32_t    mb_stride;                                         /* stride in 16x16 macroblocks */
    uint32_t    crop_right;
    uint32_t    crop_bottom;
} source_t;

typedef struct DisplayTag {
    int                 fd;                                      /* display driver fd (/dev/disp) */
    unsigned int        layerId;                                               /* diplay layer id */
    __disp_layer_info_t layerPara;                                    /* display layer parameters */
    __disp_video_fb_t   videoFb;                                     /* display video framebuffer */
    int                 sel;                                                  /* which screen 0/1 */
    __disp_pixel_fmt_t  format;                                           /* display pixel format *///DISP_FORMAT_YUV420
    __disp_pixel_mod_t  mode;                                               /* display pixel mode *///DISP_MOD_NON_MB_UV_COMBINED
    __disp_pixel_seq_t	seq;                                                 /* display pixel seq *///DISP_SEQ_UVUV yuv420
    bool_t              initialized;                                    /* display is initialized */
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
    Type_t              type;                                         /* encode frame by sw or hw */
    int                 jpegEncQuality;                                /* encoder picture quality */
    Turbojpeg_t         tj;                                                    /* turbojpeg param */
    HwJpeg_t            hwj;
} JpegEnc_t;

typedef struct ScalerTag {                                                        /* scaler param */
    scalerType_t    type;          /* arbitrary-scaler or divide-scaler to VGA or QVGA resolution */
    Size_t          size;
    uint64_t        rawSize;                                                        /* raw image size */
    uint32_t        YplaneSize;
    uint32_t        CplaneSize;
    float           xScaleFactor;
    float           yScaleFactor;
    /* scaled source buffers */
	uint8_t *YscaledSrc;                                                    /* scaled luma buffer */
	uint8_t *CscaledSrc;                                                  /* scaled chroma buffer */
    
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
    void *extra_buffer_line, *extra_buffer_frame;                              /* unknown purpose */
    
    bool_t          spsPps;                      /* sequence parameter set, picture parameter set */
    unsigned int    frameNum;                                             /* current frame number */
	sliceType_t     sliceType;                                     /* current slice type (I or P) */
} H264enc_t;

typedef struct VeispTag {
    veisp_color_format_t   colorFormat;                                           /* color format */
    uint8_t    *mb_info_buf;
    Scaler_t    scaler;                                                              /* ve scaler */
} Veisp_t;

typedef struct VeTag {                                                            /* video engine */
    void        *pRegs;                              /* pointer to virtual mapped cedar registers */
    /* input buffers */
	uint8_t    *pLumaSrc;                                                    /* input luma buffer */
	uint8_t    *pChromaSrc;                                                /* input chroma buffer */
    /* veisp */
    Veisp_t     isp;
    /* h264 encoder */
    H264enc_t    h264enc;                                                         /* h264 encoder */
} Ve_t;


typedef struct ApplTag {
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
    source_t            source;                                             /* source frame param */
    Camera_t            camera;                                                   /* camera param */
    Display_t           disp;                                               /* display parameters */
    JpegEnc_t           jpegEnc;                                           /* jpeg encoder params */
    //HW VE - Video Engine
    Ve_t                ve;                                                       /* video engine */
    //thread - critical section
    Thread_t            thread;                                      /* thread for encoding frame */
    //other
    bool_t              run;
    double              maxTimeForOneFrame;
    double              minTimeForOneFrame;
} Appl_t;

static Appl_t       l_appl;
static Appl_t      *pThis = &l_appl;

/* forward declarations */
//pthread
static void CriticalSectionCreate__(void);
static void CriticalSectionEnter__(void);
static void CriticalSectionExit__(void);
static void CriticalSectionDestroy__(void);
static void ConditionVariable_create__(void);
static void ConditionVariable_wait__(void);
static void ConditionVariable_signal__(void);
static void ConditionVariable_destroy__(void);
static void Thread_create__(threadFunction_t pThreadFunction, void *pThreadParameter);
static void Thread_destroy__(void);
THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadProcessFrame__(void *pArgument);
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
static int Disp_init__(int width, int height);
static void Disp_start__(void);
static void Disp_stop__(void);
static int Disp_on__(void);
static int Disp_exit__(void);
static int Disp_set_addr__(int w,int h,int *addr);
//libturbojpeg
static void Tj_init__(void);
static void Tj_close__(void);
//jpeg enc
static void JpegEnc_encodePicture__(const void *pFrame, int frameSize);
//hw VE (VideoEngine)
static void Ve_init__(void);
static void Ve_allocInputBuffers__(void);
static void Ve_allocScalerBuffers__(void);
static void Ve_allocOutputBuffers__(void);
static void Ve_fillInputBuffers__(const void *pFrame, int frameSize);
static void Ve_selectSubengine__(VeSubengType_t subengine);
static void Ve_trigerSubebgine__(VeSubengType_t subengine);
static void Ve_veisp_setInputBuffers__(uint8_t *Y, uint8_t *C);
static void Ve_veisp_setDivideScalerOutputBuffers__(uint8_t *Y, uint8_t *C);
static void Ve_veisp_initPicture__(uint32_t width_mb, uint32_t height_mb, uint32_t stride_mb, veisp_color_format_t format);
static void Ve_veisp_initPictureWithDivision__(uint32_t width_mb, uint32_t height_mb, veisp_color_format_t format);
static void Ve_freeInputBuffers__(void);
static void Ve_freeScalerBuffers__(void);
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
//other
static void yuv422YUYV_YUY2_2rgb__(char *pIn, char *pOut, unsigned long len);
static void crop_nv12__(char *pSrc, char *pDst, uint32_t srcWidth, uint32_t srcHeight, uint32_t dstWidth, uint32_t dstHeight);
static void printBuffer__(uint8_t *pBuffer, uint32_t length, char *pBufferName);
static void errno_exit__(const char *s);

int testopt2_flag = 0;
enum privateLongOptionsTag {
    OPT_TEST__ = 1000,
    OPT_YUVTORGB__,
    OPT_JPEGENCTYPE__,
    OPT_SCALE__,
    OPT_H264ENC__
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
        { "jpegEncType",    required_argument,  NULL,  OPT_JPEGENCTYPE__ },
        { "scale",          required_argument,  NULL,  OPT_SCALE__},
        { "testOption2",    required_argument,  &testopt2_flag, 1 },
        { "h264enc",        no_argument,        NULL,  OPT_H264ENC__ },
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
     "     --jpegEnc       Select software or hardware jpeg encoder (Default: 1)\n"
     "                      | 0 -> disable  |\n"
     "                      | 1 -> SW       |\n"
     "                      | 2 -> HW       |\n"
     "     --scale         Scale source image to (Default: none):\n"
     "                     NOTE: it must be uset with HW encoder.\n"
     "                      | 0 -> none                  |\n"
     "                      | 1 -> ARBITRARY-SCALER-VGA  |\n"
     "                      | 2 -> ARBITRARY-SCALER_QVGA |\n"
     "                      | 3 -> DIVIDE-SCALER-PAL/2   |\n"
     "     --h264Enc       Enable h264Encoder (Default: Disabled)\n"
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
            case OPT_JPEGENCTYPE__: {
                uint32_t type = strtol(optarg, NULL, 0);
                if (type <= HW) {
                    pThis->jpegEnc.type = type;
                }
                break;
            }
            case OPT_SCALE__: {
                uint32_t type = strtol(optarg, NULL, 0);
                if (type <= DIVIDE_PAL) {
                    pThis->ve.isp.scaler.type = type;
                }
                break;
            }
            case OPT_H264ENC__: {
                pThis->ve.h264enc.encode = TRUE;
                break;
            }
            case 'h': {
                usage__(stdout, argc, argv);
                exit(EXIT_SUCCESS);
            }
            case 'c': {
                errno = 0;
                pThis->frameCount = strtol(optarg, NULL, 0);
                if (errno)
                    errno_exit__(optarg);
                break;
            }
            case 'C': {
                errno = 0;
                pThis->compressCount = strtol(optarg, NULL, 0);
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

void Source_init__ (void) {
    
    if (pThis->ve.isp.scaler.type == DIVIDE_PAL) {
        pThis->source.YplaneSize = tjPlaneSizeYUV(0, 640, 0, 480, TJSAMP_420);
        pThis->source.CplaneSize = tjPlaneSizeYUV(1, 640, 0, 480, TJSAMP_420) << 1;
        pThis->source.rawSize = tjBufSizeYUV2(640, PADDING, 480, TJSAMP_420);
        pThis->source.size.width = 640;
        pThis->source.size.height = 480;
    } else {
        pThis->source.YplaneSize = pThis->camera.YplaneSize;
        pThis->source.CplaneSize = pThis->camera.CplaneSize;
        pThis->source.rawSize = pThis->camera.rawSize;
        pThis->source.size.width = pThis->camera.size.width;
        pThis->source.size.height = pThis->camera.size.height;
    }
    
    /* calculate macroblocks size */
    pThis->source.mb_width = DIV_ROUND_UP(pThis->source.size.width, 16);
    pThis->source.mb_height = DIV_ROUND_UP(pThis->source.size.height, 16);
    pThis->source.mb_stride = pThis->source.size.width / 16;
    pThis->source.crop_right = (pThis->source.mb_width * 16 - pThis->source.size.width) / 2;
    pThis->source.crop_bottom = (pThis->source.mb_height * 16 - pThis->source.size.height) / 2;
    
    /* set color format */
    pThis->ve.isp.colorFormat = VEISP_COLOR_FORMAT_NV12;
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
    Source_init__();
    
    /* initialize thread */
    CriticalSectionCreate__();
    ConditionVariable_create__();
    Thread_create__(ThreadProcessFrame__, pThis);

    /* initialize jpegturbo compressor */
    Tj_init__();
    
    /* initialize VE - Video Engine */
    Ve_init__();
    Ve_allocInputBuffers__();
    Ve_allocScalerBuffers__();
    Ve_allocOutputBuffers__();
    usleep(10000);                                                                                  //todo delete
    
    /* initialize h264 encoder */
    H264enc_init__(); 
    H264enc_new__();
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
    gettimeofday(&tvEnd, 0);
    timediff = (((tvEnd.tv_sec - tvStart.tv_sec) * 1000000.0) + (tvEnd.tv_usec - tvStart.tv_usec))
                    / 1000000.0;
    printf("\nTime for %d frames: %f\n", (int)nFrames, timediff);
    
    /* destroy thread */
    while(1) {                                           /* wait until process image is completed */
        CriticalSectionEnter__();
        if (pThis->thread.process == FALSE) {
            CriticalSectionExit__();
            break;
        }
        CriticalSectionExit__();
        usleep(5000);
    }
    Thread_destroy__();
    ConditionVariable_destroy__();
    CriticalSectionDestroy__();
    
    /* uninitialize jpeg turbo compressor */
    Tj_close__();
    
    /* uninitialize hwve */
    Ve_freeOutputBuffers__();
    Ve_freeScalerBuffers__();
    Ve_freeInputBuffers__();
    Ve_free__();
close:
    /* close camera */
    Camera_streamOff__();
    Camera_unmapBuffers__();
    Camera_close__();
	/* close display */
    Disp_stop__();                                                          /* stop video display */
	Disp_exit__();	                                                       /* close display layer */
    
    /* free h264 encoder */
    H264enc_free__();
    
    DBG("MaxTime: %f, MinTime: %f", pThis->maxTimeForOneFrame, pThis->minTimeForOneFrame);
	printf("TVD demo bye!\n");
	return 0;						
}

/* static functions definitions */
//pthread
static void CriticalSectionCreate__ (void) {
    
    int retVal;

    retVal = pthread_mutex_init(&pThis->thread.mutex, NULL);       /* dynamically mutexinitmethod */

    assert(retVal == 0);                         /* pthread_mutex_init() must return with success */
}

static void CriticalSectionEnter__ (void) {

    int retVal;

    retVal = pthread_mutex_lock(&pThis->thread.mutex);

    assert(retVal == 0);                         /* pthread_mutex_lock() must return with success */
}

static void CriticalSectionExit__ (void) {

    int retVal;

    retVal = pthread_mutex_unlock(&pThis->thread.mutex);

    assert(retVal == 0);                       /* pthread_mutex_unlock() must return with success */
}

static void CriticalSectionDestroy__ (void) {

    int retVal;

    retVal = pthread_mutex_destroy(&pThis->thread.mutex);

    if (retVal != 0) {
        DBGF("RetVal: %d", retVal);//Why is this happening?? Return 16 EBUSY //TODO!!!
    }
    //assert(retVal == 0);                    /* pthread_mutex_destroy() must return with success */
}

static void ConditionVariable_create__ (void) {

    int retVal;
    
    retVal = pthread_cond_init(&pThis->thread.cond, NULL);          /* dynamically condinitmethod */

    assert(retVal == 0);                          /* pthread_cond_init() must return with success */
}

static void ConditionVariable_wait__ (void) {

    int retVal;

    retVal = pthread_cond_wait(&pThis->thread.cond, &pThis->thread.mutex);

    assert(retVal == 0);                          /* pthread_cond_wait() must return with success */
}

static void ConditionVariable_signal__ (void) {

    int retVal;
    
    retVal = pthread_cond_signal(&pThis->thread.cond);

    assert(retVal == 0);                        /* pthread_cond_signal() must return with success */
}

static void ConditionVariable_destroy__ (void) {

    int retVal;

    retVal = pthread_cond_destroy(&pThis->thread.cond);  /* free the specified condition variable */

    assert(retVal == 0);                       /* pthread_cond_destroy() must return with success */
}

static void Thread_create__ (threadFunction_t pThreadFunction, void *pThreadParameter) {

    int retVal;
    
    pThis->thread.running = TRUE;
    pThis->thread.process = FALSE;
    retVal = pthread_create(&pThis->thread.id, NULL, pThreadFunction, pThreadParameter);

    assert(retVal == 0);                             /* pthread_create() must return with success */
}

static void Thread_destroy__ (void) {

    //int retVal;
    
    CriticalSectionEnter__();
    pThis->thread.running = FALSE;
    CriticalSectionExit__();
    ConditionVariable_signal__();
    //retVal = pthread_join(pThis->thread.id, NULL);
    
    //assert(retVal == 0);                               /* pthread_join() must return with success */
}

THREAD_FUNCTION_RETURN_TYPE THREAD_FUNCTION_ATTRIBUTE ThreadProcessFrame__ (void *pArgument) {

    const void *pFrame;
    int         frameSize;
    struct      timeval tvStart, tvEnd;
    double      timediff;
    UNUSED_ARGUMENT(pArgument);

    while (1) {
        CriticalSectionEnter__();
        ConditionVariable_wait__();
        if (pThis->thread.running == FALSE) {
            CriticalSectionExit__();
            break;
        }
        pFrame = pThis->frame.pFrame;
        frameSize = pThis->frame.frameSize;
        pThis->thread.process = TRUE;
        CriticalSectionExit__();
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
        CriticalSectionEnter__();
        pThis->thread.process = FALSE;
        CriticalSectionExit__();
    }
    
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
    CriticalSectionEnter__();
    pThis->frame.pFrame = pThis->camera.buffers[buf.index].start;
    pThis->frame.frameSize = buf.bytesused;
    CriticalSectionExit__();
    ConditionVariable_signal__();                           /* triger encoding in encoding thread */
    
    /* show frame to display */
    if (pThis->preview == TRUE) {
        /* initialize display */
        if (pThis->disp.initialized == FALSE) {
            pThis->disp.format = DISP_FORMAT_YUV420; //DISP_FORMAT_YUV420 //DISP_FORMAT_YUV422
            pThis->disp.seq=DISP_SEQ_UVUV;
            if (Disp_init__(pThis->camera.size.width, pThis->camera.size.height) == 0) {
                Disp_start__();
                Disp_on__();
                pThis->disp.initialized = TRUE;
            }
        }
        Disp_set_addr__(pThis->camera.size.width, pThis->camera.size.height, (int *)&buf.m.offset);
    }
	
    /* query new frame for this buffer */
    Camera_queryFrame__(buf.index);
	return 1;
}

static void Tvin2jpeg_processImage__ (const void *p, int picSize) {
    
    FILE           *pFile;
    char            fname[100];
    static int      id = 0;
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
        if (pThis->jpegEnc.type > 0) {
            JpegEnc_encodePicture__(pFrame, frameSize);
        }
        
        /* encode to h264 */
        if (pThis->ve.h264enc.encode == TRUE) {
            if (pThis->jpegEnc.type != HW) {                       /* copy frame to input buffers */
                memcpy(pThis->ve.pLumaSrc, pFrame, frameSize);
            }
            
            if (H264enc_encodePicture__()) {
                write(pThis->ve.h264enc.file, pThis->ve.h264enc.pBytestreamBuffer, 
                      pThis->ve.h264enc.bytestreamLength);
            } else {
                printf("ERROR: h264 encoding!\n");
            }
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
    pThis->disp.initialized = FALSE;
    pThis->frameCount = -1;
    pThis->compressCount = 0;
    pThis->rawCount = 0;
    pThis->output_marks = FALSE;
    pThis->preview = FALSE;
    pThis->readFile = FALSE;
    pThis->pFileName = NULL;
    pThis->yuvToRgb = FALSE;
    //jpeg encoder
    pThis->jpegEnc.type = SW;                            /* use software encoder by default */
    pThis->jpegEnc.jpegEncQuality = JPEG_ENC_QUALITY;
    pThis->jpegEnc.tj.jpegSize = 0;
    pThis->jpegEnc.tj.pCompressedImage = NULL;
    pThis->jpegEnc.hwj.JpegBuff = NULL;
    pThis->jpegEnc.hwj.Jwritten = 0;
    //VE - Video Engine
    pThis->ve.pRegs = NULL;
    pThis->ve.pLumaSrc = NULL;
    pThis->ve.pChromaSrc = NULL;
    pThis->ve.isp.scaler.YscaledSrc = NULL;
    pThis->ve.isp.scaler.CscaledSrc = NULL;
    //
    pThis->run = TRUE;
    pThis->maxTimeForOneFrame = 0;
    pThis->minTimeForOneFrame = 100.00;
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
    
    pThis->disp.mode=fmt_priv.fmt.raw_data[2]?DISP_MOD_MB_UV_COMBINED:DISP_MOD_NON_MB_UV_COMBINED;//DISP_MOD_NON_MB_UV_COMBINED DISP_MOD_MB_UV_COMBINED
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
    
    pThis->camera.YplaneSize = tjPlaneSizeYUV(0, pThis->camera.size.width, 0, 
                                                 pThis->camera.size.height, TJSAMP_420);
    pThis->camera.CplaneSize = (tjPlaneSizeYUV(1, pThis->camera.size.width, 0, 
                                                  pThis->camera.size.height, TJSAMP_420)) << 1;
                                 
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
        printf("MMAP: %p OFF: %p\n", pThis->camera.buffers[pThis->camera.n_buffers].start, 
                                     (void *)buf.m.offset);

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
static int Disp_init__ (int width, int height) {
    
    __u32 arg[4];
    
    pThis->disp.sel = 0;                                                      /* which screen 0/1 */
	if((pThis->disp.fd = open(DISPLAY_DEVICE, O_RDWR)) == -1) {            /* open display device */
		DBG("Display %s open fail.", DISPLAY_DEVICE);
		return -1;
	}

    /* request layer *///layer0
    arg[0] = 0;
    arg[1] = DISP_LAYER_WORK_MODE_SCALER;
    pThis->disp.layerId = ioctl(pThis->disp.fd, DISP_CMD_LAYER_REQUEST, (void*)arg);
    if(pThis->disp.layerId == 0) {
        DBG("request layer0 fail");
        close(pThis->disp.fd);
        return -1;
    }

    pThis->disp.layerPara.mode = DISP_LAYER_WORK_MODE_SCALER;
    pThis->disp.layerPara.pipe = 0; 
    pThis->disp.layerPara.fb.addr[0]       = 0;//your Y address,modify this 
    pThis->disp.layerPara.fb.addr[1]       = 0; //your C address,modify this 
    pThis->disp.layerPara.fb.addr[2]       = 0; 
    pThis->disp.layerPara.fb.size.width    = width;
    pThis->disp.layerPara.fb.size.height   = height;
    pThis->disp.layerPara.fb.mode          = pThis->disp.mode;///DISP_MOD_INTERLEAVED;//DISP_MOD_NON_MB_PLANAR;//DISP_MOD_NON_MB_UV_COMBINED;
    pThis->disp.layerPara.fb.format        = pThis->disp.format;//DISP_FORMAT_YUV420;//DISP_FORMAT_YUV422;//DISP_FORMAT_YUV420;
    pThis->disp.layerPara.fb.br_swap       = 0;
    pThis->disp.layerPara.fb.seq           = pThis->disp.seq;//DISP_SEQ_UVUV;//DISP_SEQ_YUYV;//DISP_SEQ_YVYU;//DISP_SEQ_UYVY;//DISP_SEQ_VYUY//DISP_SEQ_UVUV
    pThis->disp.layerPara.ck_enable        = 0;
    pThis->disp.layerPara.alpha_en         = 1; 
    pThis->disp.layerPara.alpha_val        = 0xff;
    pThis->disp.layerPara.src_win.x        = 0+2;
    pThis->disp.layerPara.src_win.y        = 0+5;
    pThis->disp.layerPara.src_win.width    = width-2;
    pThis->disp.layerPara.src_win.height   = height-5;
    pThis->disp.layerPara.scn_win.x        = DISP_SCN_POS_X;
    pThis->disp.layerPara.scn_win.y        = DISP_SCN_POS_Y;
    pThis->disp.layerPara.scn_win.width    = DISP_SCN_WIDTH;
    pThis->disp.layerPara.scn_win.height   = DISP_SCN_HEIGHT;
    
	arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    arg[2] = (__u32)&pThis->disp.layerPara;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_SET_PARA, (void*)arg);
#if 0
    arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_TOP, (void*)arg);
#endif
    arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_OPEN, (void*)arg);

#if 1
	int fb_fd;
	unsigned long fb_layer;
	//void *addr = NULL;
	fb_fd = open("/dev/fb0", O_RDWR);
	if (ioctl(fb_fd, FBIOGET_LAYER_HDL_0, &fb_layer) == -1) {
		DBG("get fb layer handel");	
	}
	close(fb_fd);
	arg[0] = 0;
	arg[1] = fb_layer;
	ioctl(pThis->disp.fd, DISP_CMD_LAYER_BOTTOM, (void *)arg);
#endif
	return 0;
}

static void Disp_start__ (void) {
    
    __u32 arg[4];
    
    arg[0] = pThis->disp.sel;
    arg[1] = (__u32)6291462;
    arg[2] = (__u32)&(pThis->disp.layerPara.scn_win);
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_SET_SCN_WINDOW, (void*)arg);
    
	arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    ioctl(pThis->disp.fd, DISP_CMD_VIDEO_START,  (void*)arg);
}

static void Disp_stop__ (void) {
    
    __u32 arg[4];
    
	arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    ioctl(pThis->disp.fd, DISP_CMD_VIDEO_STOP,  (void*)arg);
}

static int Disp_on__ (void) {
    
    __u32 arg[4];
    
	arg[0] = 0;
    ioctl(pThis->disp.fd, DISPLAY_ON, (void*)arg);
    
    return 0;
}

static int Disp_exit__ (void) {
    
	__u32 arg[4];
	arg[0] = 0;
    //ioctl(pThis->disp.fd, DISPLAY_OFF, (void*)arg);

    arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_CLOSE,  (void*)arg);

    arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    ioctl(pThis->disp.fd, DISP_CMD_LAYER_RELEASE,  (void*)arg);
    close (pThis->disp.fd);
    
    return 0;
}

static int Disp_set_addr__ (int w,int h,int *addr) {
    
    __u32 arg[4];
    
#if 0
	pThis->disp.layerPara.fb.addr[0]       = *addr;//your Y address,modify this 
    pThis->disp.layerPara.fb.addr[1]       = *addr+w*h; //your C address,modify this 
    pThis->disp.layerPara.fb.addr[2]       = *addr+w*h*3/2; 
    
    arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    arg[2] = (__u32)&pThis->disp.layerPara;
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
	    	fb_addr.addr[2]       = *addr+w*h*5/4;
	    	break;
	    case V4L2_PIX_FMT_NV16:
	    case V4L2_PIX_FMT_NV12:	
	    case V4L2_PIX_FMT_HM12:	
	    	fb_addr.addr[1]       = *addr+w*h; //your C address,modify this 
	    	fb_addr.addr[2]       = pThis->disp.layerPara.fb.addr[1];
	    	break;
	    
	    default:
	    	DBG("format is not found!");
	    	break;
    
  	}
  	
  	fb_addr.id = 0;  //TODO
    arg[0] = pThis->disp.sel;
    arg[1] = pThis->disp.layerId;
    arg[2] = (__u32)&fb_addr;
    ioctl(pThis->disp.fd, DISP_CMD_VIDEO_SET_FB, (void*)arg);
    
    return 0;
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
    
    static int          id = 0;
    int                 retval;
    char               *pCropFrame = NULL;
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
    } else if (pThis->jpegEnc.type == HW) {  /* ***** ***** HARDWARE VIDEO ENGINE ***** ***** */
        /* compress image to jpeg by hardware */
        // flush output buffer, otherwise we might read old cached data
        ve_flush_cache(pThis->jpegEnc.hwj.JpegBuff, MAX_JPEG_SIZE);
        if (pThis->ve.isp.scaler.type == DIVIDE_PAL) {                  /* use divivde-scaler */
            /* first crop source to VGA resolution */
            if ((pCropFrame = (char*)malloc(pThis->source.rawSize)) == NULL) {
                DBG("ERROR: Memory allocation for cropping failure: %d", __LINE__);
                goto bailout;
            }
            memset(pCropFrame, 0x80, pThis->source.rawSize);
            /* crop source frame to VGA 640x480 */
            crop_nv12__((char*)pFrame, pCropFrame, pThis->camera.size.width, pThis->camera.size.height,
                        pThis->source.size.width, pThis->source.size.height);
            /* fill the buffers */
            Ve_fillInputBuffers__(pCropFrame, pThis->source.rawSize);
            /* scale VGA to QVGA */
            Ve_selectSubengine__(SUBENG_ISP);                       /* select VEISP subengine */
            Ve_veisp_setDivideScalerOutputBuffers__(pThis->ve.isp.scaler.YscaledSrc, 
                                                      pThis->ve.isp.scaler.CscaledSrc);
            Ve_veisp_initPictureWithDivision__(pThis->source.mb_width, pThis->source.mb_height, 
                                               pThis->ve.isp.colorFormat);
        } else {
            Ve_fillInputBuffers__(pFrame, frameSize);
            Ve_selectSubengine__(SUBENG_AVCENC);
            Ve_veisp_initPicture__(pThis->source.mb_width, pThis->source.mb_height,
                                   pThis->source.mb_stride, pThis->ve.isp.colorFormat);
        }
        Ve_veisp_setInputBuffers__(pThis->ve.pLumaSrc, pThis->ve.pChromaSrc);

        if (pThis->ve.isp.scaler.type == DIVIDE_PAL) {                   /* use divide scaler */
            DBG("Use divide scaler");
            Ve_trigerSubebgine__(SUBENG_ISP);                       /* triger divide-scaler */
            /* flush output buffers */
            ve_flush_cache(pThis->ve.isp.scaler.YscaledSrc, pThis->ve.isp.scaler.rawSize);
            veavc_release_subengine();
            Ve_selectSubengine__(SUBENG_AVCENC);
            Ve_veisp_setInputBuffers__(pThis->ve.isp.scaler.YscaledSrc, 
                                         pThis->ve.isp.scaler.CscaledSrc);
            Ve_veisp_initPicture__(pThis->ve.isp.scaler.mb_width, 
                                pThis->ve.isp.scaler.mb_height, pThis->ve.isp.scaler.mb_stride, 
                                pThis->ve.isp.colorFormat);
        } else if (pThis->ve.isp.scaler.type > 0) {             /* use arbitrary scaler engine */
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
        Ve_trigerSubebgine__(SUBENG_AVCENC);
        pThis->jpegEnc.hwj.Jwritten = veavc_get_written();
        ve_flush_cache(pThis->jpegEnc.hwj.JpegBuff, MAX_JPEG_SIZE);          /* flush for A20 */
        
        /* save image to file */
        if (pThis->output_marks) {
            fprintf(stderr, "#");
        }
        snprintf(fname, sizeof(fname), "/tmp/testImage_%03d.jpg", id);
        vejpeg_write_file(fname, pThis->jpegEnc.hwj.JpegBuff, pThis->jpegEnc.hwj.Jwritten);
        //printf("[JEPOC] written %d bytes to %s\n", pThis->jpegEnc.hwj.Jwritten, fname);
        
        //printBuffer__((uint8_t*)pFrame+100, 100, "pFrame");
        //printBuffer__(pThis->ve.pLumaSrc+100, 100, "Ysrc");
        //printBuffer__(pThis->ve.pChromaSrc+100, 100, "Csrc");
        //printBuffer__(pThis->jpegEnc.hwj.JpegBuff+100, 100, "JpegBuff");
        veavc_release_subengine();
    }

bailout:
    if (pThis->ve.isp.scaler.type == DIVIDE_PAL) {
        if (pCropFrame != NULL) {
            free(pCropFrame);
            DBG("Free crop buffer!");
        }
    }
    id++;
}

//hw VE (VideoEngine)
static void Ve_init__ (void) {
    
    ve_init();                                                              /* init video encoder */
    if ((pThis->ve.pRegs = ve_open()) == NULL) {                               /* open video encoder */
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

static void Ve_allocScalerBuffers__ (void) {

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
        /* DIVIDE_SCALER: for vga-qvga resolutions you can crop picture to VGA by software and 
         * than scale it with divide-sclaer to QVGA */
        case DIVIDE_PAL: {
            pThis->ve.isp.scaler.size.width = 320;
            pThis->ve.isp.scaler.size.height = 240;
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

    if (pThis->ve.isp.scaler.type == DIVIDE_PAL) {
        /* allocate memory from VE */
        pThis->ve.isp.scaler.YscaledSrc = ve_malloc(pThis->ve.isp.scaler.rawSize);
        if (!pThis->ve.isp.scaler.YscaledSrc) {
            DBG("ERROR: Cannot allocate scaler buffers!");
            exit(EXIT_FAILURE);
        }
        pThis->ve.isp.scaler.CscaledSrc = pThis->ve.isp.scaler.YscaledSrc + 
                                          pThis->ve.isp.scaler.YplaneSize;
        memset(pThis->ve.isp.scaler.YscaledSrc, 0x80, pThis->ve.isp.scaler.rawSize);
        ve_flush_cache(pThis->ve.isp.scaler.YscaledSrc, pThis->ve.isp.scaler.rawSize);
    }
    
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
    
    //DBGF("YplaneSize: %d, CplaneSize: %d => %p, %p", pThis->source.YplaneSize, 
            //pThis->source.CplaneSize, pFrame, pFrame + pThis->source.YplaneSize);
    memcpy(pThis->ve.pLumaSrc, pFrame, frameSize);
    //memcpy(pThis->ve.pChromaSrc, pFrame + pThis->source.YplaneSize, pThis->source.CplaneSize);

	/* flush for A20 */
	ve_flush_cache(pThis->ve.pLumaSrc, frameSize);
	//ve_flush_cache(pThis->ve.pChromaSrc, pThis->camera.CplaneSize);
}

static void Ve_selectSubengine__ (VeSubengType_t subengine) {

    switch (subengine) {
        case SUBENG_ISP: {
            veavc_select_ISPsubengine();
            break;
        }
        case SUBENG_AVCENC: {
            veavc_select_subengine();
            break;
        }
        default: {
            break;
        }
    }
}

static void Ve_trigerSubebgine__ (VeSubengType_t subengine) {

    switch (subengine) {
        case SUBENG_ISP: {
            veisp_triger();
            ve_wait(1);
            break;
        }
        case SUBENG_AVCENC: {
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

static void Ve_veisp_setDivideScalerOutputBuffers__ (uint8_t *Y, uint8_t *C) {

    veisp_set_outputBuffers(Y, C);
}

static void Ve_veisp_initPicture__ (uint32_t width_mb, uint32_t height_mb, uint32_t stride_mb, veisp_color_format_t format) {

    veisp_init_pictureMb(width_mb, height_mb, stride_mb, format);
}

static void Ve_veisp_initPictureWithDivision__ (uint32_t width_mb, uint32_t height_mb, 
                                              veisp_color_format_t format) {

    veisp_init_pictureMbWithDivision(width_mb, height_mb, format);
}

static void Ve_freeInputBuffers__ (void) {

    ve_free(pThis->ve.pLumaSrc);
}

static void Ve_freeScalerBuffers__ (void) {

    if (pThis->ve.isp.scaler.type == DIVIDE_PAL) {
        ve_free(pThis->ve.isp.scaler.YscaledSrc);
    }
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
    
    pThis->ve.h264enc.profileIdc = 77;
    pThis->ve.h264enc.levelIdc = 41;
    pThis->ve.h264enc.ecMode = H264_EC_CABAC;                                    /* entropyCoding */
	pThis->ve.h264enc.qp = 24;                                                      /* range 1-47 */
	pThis->ve.h264enc.keyframeInterval = 25;
    
    pThis->ve.h264enc.pBytestreamBuffer = NULL;
    pThis->ve.h264enc.extra_buffer_line = NULL;
    pThis->ve.h264enc.extra_buffer_frame = NULL;
}

static void H264enc_new__ (void) {
    
    uint32_t i;
    uint32_t luma_size;
    uint32_t chroma_size;
    
    
    /* copy parameters */
    pThis->ve.h264enc.spsPps = TRUE;
    pThis->ve.h264enc.frameNum = 0;
    
    /* allocate bytestream output buffer */
	pThis->ve.h264enc.bytestreamBufferSize = 1 * 1024 * 1024;
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
	if (pThis->ve.h264enc.extra_buffer_frame == NULL || 
        pThis->ve.h264enc.extra_buffer_line == NULL) {
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

    Ve_selectSubengine__(SUBENG_AVCENC);

	/* flush buffers (output because otherwise we might read old data later) */
	ve_flush_cache(pThis->ve.h264enc.pBytestreamBuffer, pThis->ve.h264enc.bytestreamBufferSize);
	ve_flush_cache(pThis->ve.pLumaSrc, pThis->camera.rawSize);

	/* set output buffer */
	writel(0x0, pThis->ve.pRegs + VE_AVC_VLE_OFFSET);
	writel(ve_virt2phys(pThis->ve.h264enc.pBytestreamBuffer), pThis->ve.pRegs + VE_AVC_VLE_ADDR);
	writel(ve_virt2phys(pThis->ve.h264enc.pBytestreamBuffer) + 
            pThis->ve.h264enc.bytestreamBufferSize - 1, pThis->ve.pRegs + VE_AVC_VLE_END);
	writel(pThis->ve.h264enc.bytestreamBufferSize * 8, pThis->ve.pRegs + VE_AVC_VLE_MAX);

	/* write headers */
	if (pThis->ve.h264enc.spsPps == TRUE) {
		put_seq_parameter_set__();
		put_pic_parameter_set__();
		pThis->ve.h264enc.spsPps = FALSE;
	}
	put_slice_header__();

	/* set input size */ /* set input format */
    Ve_veisp_initPicture__(pThis->source.mb_width, pThis->source.mb_height, pThis->source.mb_stride,
                           pThis->ve.isp.colorFormat);
                           
	/* set input buffer */
    Ve_veisp_setInputBuffers__(pThis->ve.pLumaSrc, pThis->ve.pChromaSrc);
    
    /* add scaler */
    if (pThis->ve.isp.scaler.type > 0 && pThis->ve.isp.scaler.type != DIVIDE_PAL) {
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
	writel((4 << 16) | (pThis->ve.h264enc.qp << 8) | pThis->ve.h264enc.qp, pThis->ve.pRegs + VE_AVC_QP);
	writel(0x00000104, pThis->ve.pRegs + VE_AVC_MOTION_EST);

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

    veavc_release_subengine();                                           /* release avc subengine */

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

	frameCroppingFlag = pThis->source.crop_right || pThis->source.crop_bottom;
	put_bits__(frameCroppingFlag, 1);
	if (frameCroppingFlag) {
		put_ue__(0);
		put_ue__(pThis->source.crop_right);
		put_ue__(0);
		put_ue__(pThis->source.crop_bottom);
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

static void crop_nv12__ (char *pSrc, char *pDst, uint32_t srcWidth, uint32_t srcHeight, 
                         uint32_t dstWidth, uint32_t dstHeight) {
                            
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


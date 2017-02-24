/*
 * Copyright (c) 2013 Jens Kuske <jenskuske@gmail.com>
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

#include <string.h>
#include "private.h"
#include "ve.h"
#include "vepoc.h"
#include "tiled_yuv.h"

VcStatus VideoSurface_create (ChromaType_t chromaType, uint32_t width, uint32_t height, VideoSurface *surface) {

	if (!surface) {
		return VC_STATUS_INVALID_POINTER;
    }

	if (!width || !height) {
		return VC_STATUS_INVALID_SIZE;
    }
    
    if (width < 1 || width > 8192 || height < 1 || height > 8192) {
		return VC_STATUS_INVALID_SIZE;
    }

	VideoSurface_t *vs = Handle_create(sizeof(*vs), surface);
	if (!vs) {
		return VC_STATUS_RESOURCES;
    }

	vs->width = width;
	vs->height = height;
	vs->chromaType = chromaType;

    vs->luma_size = ALIGN(width, 32) * ALIGN(height, 32);
    

	switch (chromaType) {
        case CHROMA_TYPE_444: {
            vs->size = vs->luma_size * 3;
            vs->data = ve_malloc(vs->luma_size * 3);
            ve_flush_cache(vs->data, vs->luma_size * 3);
            break;
        }
        case CHROMA_TYPE_422: {
            vs->size = vs->luma_size * 2;
            vs->data = ve_malloc(vs->luma_size * 2);
            ve_flush_cache(vs->data, vs->luma_size * 2);
            break;
        }
        case CHROMA_TYPE_420: {
            vs->size = vs->luma_size + ALIGN(vs->width, 32) * ALIGN(vs->height / 2, 32);
            vs->data = ve_malloc(vs->size);
            ve_flush_cache(vs->data, vs->size);
            break;
        }
        default: {
            Handle_destroy(*surface);
            return VC_STATUS_INVALID_CHROMA_TYPE;
        }
	}

	if (!vs->data) {
        Handle_destroy(*surface);
		return VC_STATUS_RESOURCES;
	}

	return VC_STATUS_OK;
}

VcStatus VideoSurface_destroy(VideoSurface surface) {
    
	VideoSurface_t *vs = Handle_get(surface);
    
	if (!vs) {
		return VC_STATUS_INVALID_HANDLE;
    }

	if (vs->extra_data) {
		ve_free(vs->extra_data);
    }
    if (vs->data) {
		ve_free(vs->data);
    }

	Handle_destroy(surface);

	return VC_STATUS_OK;
}

VcStatus VideoSurface_getParameters(VideoSurface surface, ChromaType_t *chromaType, uint32_t *width, uint32_t *height) {
    
	VideoSurface_t *vs = Handle_get(surface);
    
	if (!vs) {
		return VC_STATUS_INVALID_HANDLE;
    }

	if (chromaType) {
		*chromaType = vs->chromaType;
    }

	if (width) {
		*width = vs->width;
    }

	if (height) {
		*height = vs->height;
    }

	return VC_STATUS_OK;
}

VcStatus VideoSurface_getBits_yCbCr(VideoSurface surface, YCbCrFormat_t destinationYcbcrFormat, void *const *destinationData, uint32_t const *destinationPitches) {

	VideoSurface_t *vs = Handle_get(surface);
    
	if (!vs) {
		return VC_STATUS_INVALID_HANDLE;
    }
    
    if (vs->chromaType != CHROMA_TYPE_420 || vs->sourceFormat != INTERNAL_YCBCR_FORMAT) {
		return VC_STATUS_INVALID_Y_CB_CR_FORMAT;
    }
    
    if (destinationPitches[0] < vs->width || destinationPitches[1] < vs->width / 2) {
		return VC_STATUS_ERROR;
    }

	switch (destinationYcbcrFormat) {
        case YCBCR_FORMAT_NV12: {
            tiled_to_planar(vs->data, destinationData[0], destinationPitches[0], vs->width, vs->height);
            tiled_to_planar(vs->data + vs->luma_size, destinationData[1], destinationPitches[1], vs->width, vs->height / 2);
            return VC_STATUS_OK;
        }
        case YCBCR_FORMAT_YV12: {
            if (destinationPitches[2] != destinationPitches[1]) {
                return VC_STATUS_ERROR;
            }
            tiled_to_planar(vs->data, destinationData[0], destinationPitches[0], vs->width, vs->height);
            tiled_deinterleave_to_planar(vs->data + vs->luma_size, destinationData[2], destinationData[1], destinationPitches[1], vs->width, vs->height / 2);
            return VC_STATUS_OK;
        }
        default: {
            break;
        }
	}

	return VC_STATUS_ERROR;
}

VcStatus VideoSurface_putBits_yCbCr(VideoSurface surface, YCbCrFormat_t sourceYcbcrFormat, void const *const *sourceData, uint32_t const *sourcePitches) {

	VideoSurface_t *vs = Handle_get(surface);
    
	if (!vs) {
		return VC_STATUS_INVALID_HANDLE;
    }

	vs->sourceFormat = sourceYcbcrFormat;

	switch (sourceYcbcrFormat) {
        case YCBCR_FORMAT_YUYV:
        case YCBCR_FORMAT_UYVY:
        case YCBCR_FORMAT_Y8U8V8A8:
        case YCBCR_FORMAT_V8U8Y8A8: {

            break;
        }
        case YCBCR_FORMAT_NV12: {

            break;
        }
        case YCBCR_FORMAT_YV12: {
            memcpy(vs->data, sourceData[0], sourcePitches[0] * vs->height);
            memcpy(vs->data + vs->luma_size, sourceData[1], sourcePitches[1] * vs->height / 2);
            memcpy(vs->data + vs->luma_size + (vs->luma_size / 4), sourceData[2], sourcePitches[2] * vs->height / 2);
            break;
        }
        default: {
            break;
        }
	}

	return VC_STATUS_OK;
}

VcStatus VideoSurface_queryCaps(ChromaType_t surfaceChromaType, bool_t *isSupported, uint32_t *maxWidth, uint32_t *maxHeight) {

	if (!isSupported || !maxWidth || !maxHeight) {
		return VC_STATUS_INVALID_POINTER;
    }

	*isSupported = surfaceChromaType == CHROMA_TYPE_420;
	*maxWidth = 8192;
	*maxHeight = 8192;

	return VC_STATUS_OK;
}

VcStatus VideoSurface_queryGetPutBits_yCbCrCaps(ChromaType_t surfaceChromaType, YCbCrFormat_t bits_yCbCrFormat, bool_t *isSupported) {

	if (!isSupported) {
		return VC_STATUS_INVALID_POINTER;
    }

    if (surfaceChromaType == CHROMA_TYPE_420) {
		*isSupported = (bits_yCbCrFormat == YCBCR_FORMAT_NV12) ||
				(bits_yCbCrFormat == YCBCR_FORMAT_YV12);
    } else {
		*isSupported = FALSE;
    }

	return VC_STATUS_OK;
}

/*
 * Copyright (c) 2013 Jens Kuske <jenskuske@gmail.com>
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

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <assert.h>
#include <stropts.h>
#include <sys/mman.h>
#include <string.h>
#include "ve.h"
#include "debug.h"

#define DEVICE "/dev/cedar_dev"
#define PAGE_OFFSET (0xc0000000) // from kernel
#define PAGE_SIZE (4096)

/* type definition. */
typedef enum {
    FALSE = 0,
    TRUE  = 1
} bool_t;

enum IOCTL_CMD {
    
    /* CORE IOCTL */
	IOCTL_UNKOWN = 0x100,
	IOCTL_GET_ENV_INFO, /* 0x101 */ /* Get VE info. Memory offset, memory size and register ofset */
	IOCTL_WAIT_VE,      /* 0x102 */            /* wait fow encoding/decoding IRQ (operation ends) */
	IOCTL_RESET_VE,     /* 0x103 */                               /* 0x103 do reset cedarx engine */
	IOCTL_ENABLE_VE,    /* 0x104 */                       /* 0x104, start base clocks for cedarx  */
	IOCTL_DISABLE_VE,   /* 0x105 */                             /* disable base clocks for cedarx */
	IOCTL_SET_VE_FREQ,  /* 0x106 */ /* 0x106, config cedarx frequency, get in argument freqency in Mhz */
    /* AVS2 IOCTL */
	IOCTL_CONFIG_AVS2 = 0x200,
	IOCTL_GETVALUE_AVS2 ,   /* 0x201 */
	IOCTL_PAUSE_AVS2 ,      /* 0x202 */
	IOCTL_START_AVS2 ,      /* 0x203 */
	IOCTL_RESET_AVS2 ,      /* 0x204 */
	IOCTL_ADJUST_AVS2,      /* 0x205 */
    /* ENGINE IOCTL */
	IOCTL_ENGINE_REQ,       /* 0x206 */ /* 0x206, count references to cedar hardware and more important start some clocks that required for cedar init */
	IOCTL_ENGINE_REL,       /* 0x207 */ /* decrement reference count  */
	IOCTL_ENGINE_CHECK_DELAY,   /* 0x208 */
	IOCTL_GET_IC_VER,       /* 0x209 */
	IOCTL_ADJUST_AVS2_ABS,  /* 0x20a */
	IOCTL_FLUSH_CACHE       /* 0x20b */ /* do invalidate CPU cache for internal cedar dma  */
};

typedef struct ve_infoTag {
	uint32_t    mem_offset;                             /* constain cedarx reserved memory offset */
	int         mem_size;                                 /* constain cedarx reserved memory size */
	uint32_t    registers_offset;             /* constain io remapped  cedarx hw registers offset */
} ve_info_t;

typedef struct cedarvCacheRangeTag {
	long start;
	long end;
} cedarvCacheRange_t;

void      *regs;

typedef struct MemchunkTag {
	uint32_t    phys_addr;                                             /* physical memory address */
	int         size;                                                           /* size of memory */
	void       *virt_addr;                                              /* virtual memory address */
	struct MemchunkTag *next;                                     /* pointer to the next memchunk */
} Memchunk_t;

/* the VE object's data structure */
typedef struct VeTag {
    ve_info_t   veInfo;                                                      /* video engine info */
    int         fd;                                                  /* cedar dev file descriptor */
    int         version;
    void       *pRegs;                               /* pointer to virtual mapped cedar registers */
    Memchunk_t  FirstChunk;                                                           /* memchunk */
    pthread_rwlock_t memoryLock;
} Ve_t;

static Ve_t ve;                                     /* single instance of the video engine object */

static void MemLockCreate__(void);
static bool_t MemLockWREnter__(void);
static bool_t MemLockRDEnter__(void);
static bool_t MemLockExit__(void);
static void MemLockDestroy__(void);

void ve_init(void) {
    
    /* init ve */
    memset(&ve, 0, sizeof(ve));
    ve.fd = -1;
    ve.version = 0;
    ve.pRegs = NULL;
    ve.FirstChunk.phys_addr = 0x0;
    ve.FirstChunk.size = 0;
    ve.FirstChunk.virt_addr = NULL;
    ve.FirstChunk.next = NULL;

    MemLockCreate__();
}

void* ve_open(void) { /* open and init VE */
    
    uint32_t    ctrl;
	if (ve.fd != -1) {                                                    /* VE is already opened */
		return NULL;
    }
    
    ve_init();
    
	ve.fd = open(DEVICE, O_RDWR);                                            /* open cedar device */
	if (ve.fd == -1) {                                                  /* can not open cedar_dev */
		return NULL;
    }

    /* copy memory configuration info to userspace struct */
	if (ioctl(ve.fd, IOCTL_GET_ENV_INFO, (void *)(&ve.veInfo)) == -1) {
		goto err;
	}

	ve.pRegs = mmap(NULL, 0x800, PROT_READ | PROT_WRITE, MAP_SHARED, ve.fd, ve.veInfo.registers_offset);
    if (ve.pRegs == MAP_FAILED) {
		goto err;
    }
    regs = ve.pRegs;
    
	ve.FirstChunk.phys_addr = ve.veInfo.mem_offset - PAGE_OFFSET;
	ve.FirstChunk.size = ve.veInfo.mem_size;
    DBG("Meminfo: RegisterOffset: %d->%p, MemOffset: %d, MemSize: %d", ve.veInfo.registers_offset,
            ve.pRegs, ve.veInfo.mem_offset, ve.veInfo.mem_size);

	ioctl(ve.fd, IOCTL_ENGINE_REQ, 0);  /* 0x206, count references to cedar hardware and more important start some clocks that required for cedar init */
	ioctl(ve.fd, IOCTL_ENABLE_VE, 0);                     /* 0x104, start base clocks for cedarx  */
	ioctl(ve.fd, IOCTL_SET_VE_FREQ, 320);   /* 0x106, config cedarx frequency, get in argument freqency in Mhz */
	ioctl(ve.fd, IOCTL_RESET_VE, 0);                              /* 0x103 do reset cedarx engine */

    /* set general control register */
    //ctrl = 0x00130007;
    ctrl = 0x00000007;                        /* Note: selecting 0x7 mean 'place in reset state'  */
    ctrl |= 0x13 << 16;                                                /* select DDR3 memory type */
    //ctrl = 0x01 << 5;
    //ctrl = 0x01 << 6;
    //ctrl = 0x01 << 7;
	writel(ctrl, ve.pRegs + VE_CTRL);
    //writel(0x80000000, ve.pRegs + 0x8);   //just play game..:)
    //writel(0x80000260, ve.pRegs + VE_ANAGLYPH_CTRL);

	ve.version = readl(ve.pRegs + VE_VERSION) >> 16;
	printf("[SUNXI] VE version 0x%04x opened.\n", ve.version);

	return ve.pRegs;
    
err:
	close(ve.fd);
	ve.fd = -1;
	return NULL;
}

void ve_close(void) { /* deinit and close VE */
    
	if (ve.fd == -1) {
		return;
    }

	ioctl(ve.fd, IOCTL_DISABLE_VE, 0);              /* 0x105 */ /* disable base clocks for cedarx */
	ioctl(ve.fd, IOCTL_ENGINE_REL, 0);                  /* 0x207 */ /* decrement reference count  */

	munmap(ve.pRegs, 0x800);

    MemLockDestroy__();
	
    close(ve.fd);
	ve.fd = -1;
}

void ve_flush_cache(void *start, int len) {                /* flush cache (neded for A20) - Milos */

	if (ve.fd == -1)
		return;

    cedarvCacheRange_t cacheRange;
    
    cacheRange.start = (int)start;
    cacheRange.end = (int)(start + len);

	ioctl(ve.fd, IOCTL_FLUSH_CACHE, (void*)(&cacheRange)); /* 0x20b */ /* do invalidate CPU cache for internal cedar dma  */
}

void *ve_get_regs(void) {
    
	if (ve.fd == -1)
		return NULL;

	return ve.pRegs;
}

int ve_get_version(void) {
    
	return ve.version;
}

int ve_wait(int timeout) {
    
	if (ve.fd == -1)
		return 0;

	return ioctl(ve.fd, IOCTL_WAIT_VE, timeout);
}

void *ve_malloc(int size) {
    
	if (ve.fd == -1)
		return NULL;
        
	if (MemLockWREnter__() == FALSE) {
		return NULL;
    }
    
    void *addr = NULL;

    //printf("Malloc: %d ", size);
	size = (size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1); /* round it to page size blocks */
    //printf("NewSize: %d\n", size);
	Memchunk_t *currentChunk, *best_chunk = NULL;
	for (currentChunk = &ve.FirstChunk; currentChunk != NULL; currentChunk = currentChunk->next) {
		if(currentChunk->virt_addr == NULL && currentChunk->size >= size) {
			if (best_chunk == NULL || currentChunk->size < best_chunk->size) {
				best_chunk = currentChunk;
            }
			if (currentChunk->size == size) {
				break;
            }
		}
    }

	if (!best_chunk) {
        goto out;
    }

	int left_size = best_chunk->size - size;

	addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, ve.fd, best_chunk->phys_addr + PAGE_OFFSET);
    if (addr == MAP_FAILED) {
        addr = NULL;
		goto out;
    }
    best_chunk->virt_addr = addr;
	best_chunk->size = size;

	if (left_size > 0)
	{   /* alocate one more memory chunk */
        Memchunk_t *nextChunk;
		nextChunk = malloc(sizeof(Memchunk_t));
		nextChunk->phys_addr = best_chunk->phys_addr + size;
		nextChunk->size = left_size;
		nextChunk->virt_addr = NULL;
		nextChunk->next = best_chunk->next;
		best_chunk->next = nextChunk;
	}

out:
	MemLockExit__();
	return addr;
}

void ve_free(void *ptr) {
    
    Memchunk_t *currentChunk;
    
	if (ve.fd == -1)
		return;
	if (ptr == NULL)
		return;
	if (MemLockWREnter__() == FALSE) {
		return;
    }

	/* unmap phisical memory */
	for (currentChunk = &ve.FirstChunk; currentChunk != NULL; currentChunk = currentChunk->next) {
		if (currentChunk->virt_addr == ptr) {
			munmap(ptr, currentChunk->size);
			currentChunk->virt_addr = NULL;
			break;
		}
    }
    /* connect first chunk to next chunk and free current chunk */
	for (currentChunk = &ve.FirstChunk; currentChunk != NULL; currentChunk = currentChunk->next) {
		if (currentChunk->virt_addr == NULL) {
			while (currentChunk->next != NULL && currentChunk->next->virt_addr == NULL) {
				Memchunk_t *nextChunk = currentChunk->next;
				currentChunk->size += nextChunk->size;
				currentChunk->next = nextChunk->next;
				free(nextChunk);
			}
        }
    }
    
    MemLockExit__();
}

uint32_t ve_virt2phys(void *ptr) {
    
	if (ve.fd == -1)
		return 0;
        
	if (MemLockRDEnter__() == FALSE) {
		return 0;
    }
    
    uint32_t addr = 0;

	Memchunk_t *currentChunk;
	for (currentChunk = &ve.FirstChunk; currentChunk != NULL; currentChunk = currentChunk->next) {
		if (currentChunk->virt_addr == NULL) {
			continue;
        }
		if (currentChunk->virt_addr == ptr) {
            addr = currentChunk->phys_addr;
            break;
        } else if (ptr > currentChunk->virt_addr && 
                   ptr < (currentChunk->virt_addr + currentChunk->size)) {
            addr = currentChunk->phys_addr + (ptr - currentChunk->virt_addr);
            break;
        }
	}

	MemLockExit__();
	return addr;
}


static void MemLockCreate__ (void) {
    
    int retVal;

    retVal = pthread_rwlock_init(&ve.memoryLock, NULL);            /* dynamically mutexinitmethod */
    assert(retVal == 0);                        /* pthread_rwlock_init() must return with success */
}

static bool_t MemLockWREnter__ (void) {
    
	if (pthread_rwlock_wrlock(&ve.memoryLock) != 0) {
		return FALSE;
    }
    
    return TRUE;
}

static bool_t MemLockRDEnter__ (void) {
    
	if (pthread_rwlock_rdlock(&ve.memoryLock) != 0) {
		return FALSE;
    }
    
    return TRUE;
}

static bool_t MemLockExit__ (void) {

	if (pthread_rwlock_unlock(&ve.memoryLock) != 0) {
		return FALSE;
    }
    
    return TRUE;
}

static void MemLockDestroy__ (void) {

    pthread_rwlock_destroy(&ve.memoryLock);
}

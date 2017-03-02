#For compiling without SB2
#make ARCH=arm CROSS_COMPILE=/home/milos/A20/toolchain/gcc-linaro-5.3.1-2016.05-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
#make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

CC = arm-linux-gnueabihf-gcc

#CFLAGS= -Wall -march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -ffast-math -mfloat-abi=hard -Ofast
CFLAGS = -Wall -march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -ffast-math -mfloat-abi=hard -O0
#CFLAGS = -Wall -march=armv7-a -O0
CFLAGS += $(shell pkg-config --cflags gstreamer-0.10)
CFLAGS += $(shell pkg-config --cflags --libs gthread-2.0)
#LDFLAGS = -static
#LDFLAGS = -static -L/home/milos/A20/a10_iptRootfs/usr/lib
LDFLAGS = -L/home/milos/A20/a10_iptRootfs/usr/lib
LDFLAGS += -L/home/milos/A20/a10_iptRootfs/usr/lib/arm-linux-gnueabihf/neon/vfp
LDFLAGS	+= -lavformat -lavcodec -lavutil
LDFLAGS += -lstdc++ -lasound
LDLIBS = -ljpeg

LIBS += $(shell pkg-config --libs gtk+-3.0)
LIBS += -lrt
LIBS += -lgstreamer-0.10

EXE = $(shell basename `pwd`)
SRC = $(wildcard *.c)
SRC += $(wildcard *.S)
OBJ = $(SRC:.c=.o)

.PHONY : all
all: $(SRC) $(EXE)
	
$(EXE): $(OBJ) 
	$(CC) $(LDFLAGS) $(OBJ) $(LDLIBS) $(LIBS) -o $@ -lrt -pthread .libs/libturbojpeg.a
	
.c.o:
	$(CC) -c $(CFLAGS) $(LDLIBS) $< -o $@
	
.S.o:
	$(CC) -c $< -o $@
	
.PHONY : all
clean : 
	-rm -f *.o	$(EXE)

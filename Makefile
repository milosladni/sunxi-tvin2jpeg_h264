#For compiling without SB2
#make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

CC = arm-linux-gnueabihf-gcc

CFLAGS= -Wall -march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -ffast-math -mfloat-abi=hard -Ofast
#CFLAGS = -Wall -march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -ffast-math -mfloat-abi=hard -O0

LDFLAGS += -L/home/milos/A20/a10_iptRootfs/usr/lib/arm-linux-gnueabihf
LDFLAGS	+= -lavformat -lavcodec -lavutil

EXE = $(shell basename `pwd`)
SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)

.PHONY : all
all: $(SRC) $(EXE)
	
$(EXE): $(OBJ) 
	$(CC) $(LDFLAGS) $(OBJ) $(LDLIBS) -o $@ -lrt -pthread .libs/libturbojpeg.a
	
.c.o:
	$(CC) -c $(CFLAGS) $(LDLIBS) $< -o $@
	
.PHONY : all
clean : 
	-rm -f *.o	$(EXE)

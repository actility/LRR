################################################################################
include $(ROOTACT)/rtbase/base/makefile.def
include $(ROOTACT)/lrr/base/makefile.def

################################################################################

# not an option but mandatory
 CFLAGS += -g -DLP_MODULE_LRR -DUSELIBLGW3	# for libloragw >= 3.2.0

#ARCH_RADIO=SX1301_X1
#ARCH_RADIO=SX1301_X8

#ARCH_COMM=usb
#ARCH_COMM=spi

ifeq ($(ARCH_COMM),spi)
	CFLAGS += -DWITH_SPI
endif

ifeq ($(ARCH_RADIO),SX1301_X1)
# default options for SPI + SX1301 X1 + GPS on kerlink/wirma
ifeq ($(SYSTEM),fcmlb)
	CFLAGS += -DWITH_SX1301_X1
else
	CFLAGS += -DWITH_SX1301_X1 -DWITH_GPS
endif
	SEMTECH=lora_gateway/libloragw
	LIBSEMTECH=$(ROOTACT)/$(SEMTECH)/libloragw.a
	INCSEMTECH=$(ROOTACT)/$(SEMTECH)/inc
endif

ifeq ($(ARCH_RADIO),SX1301_X8)
# default options for SX1301 X8 - GPS on raspberry
	CFLAGS += -DWITH_SX1301_X8
	SEMTECH=SX1301array_driver
	LIBSEMTECH=$(ROOTACT)/$(SEMTECH)/libsx1301ar/libsx1301ar.a
	INCSEMTECH=$(ROOTACT)/$(SEMTECH)/libsx1301ar/inc
endif


LIB=\
	$(LIBSEMTECH)\
	$(LIBBASE)

INC=-I../include -I../../include -I$(ROOTACT) -I$(ROOTACT)/include -I$(INCSEMTECH)


#MYLDFLAGS=-lpcap -L/usr/local/lib -lmpsse -lrt
#MYLDFLAGS=-lmpsse -lusb -lcrypto -lm -lrt
#MYLDFLAGS=-lpcap -lmpsse -lcrypto -lm -lrt
#MYLDFLAGS=-lmpsse -lcrypto -lm -lrt

MYLDFLAGS=-lm -lrt

ifeq ($(SYSTEM),wirmams)
	MYLDFLAGS=-lm -lrt -lcrypto
endif

ifeq ($(SYSTEM),ir910)
LIB += $(SDK)/usr/lib/libmpsse.a $(SDK)/usr/lib/libftdi1.a $(SDK)/usr/lib/libusb-1.0.a
endif

ifeq ($(SYSTEM),natrbpi)
	ifeq ($(ARCH_COMM),usb)
		LIB += /usr/lib/libmpsse.a
		MYLDFLAGS += -lftdi -lusb
	endif
endif


################################################################################
EXELRR=lrr

SRC=\
	../networkfilter.c\
	../headerloramac.c\
	../crc-ccitt.c\
	../inivar.c\
	../gps.c\
	../cellular.c\
	../lgw_gen.c\
	../avdv.c\
	../netitf.c\
	../rttping.c\
	../shellcmd.c\
	../semtech.c\
	../keycrypt.c\
	../keybuild.c\
	../dutycycle.c\
	../system_api.c
	../main.c\

OBJ=\
	networkfilter.o\
	headerloramac.o\
	crc-ccitt.o\
	inivar.o\
	gps.o\
	cellular.o\
	lgw_gen.o\
	avdv.o\
	netitf.o\
	rttping.o\
	shellcmd.o\
	semtech.o\
	keycrypt.o\
	keybuild.o\
	dutycycle.o\
	system_api.o\
	main.o\


ifeq ($(SYSTEM),wirmams)
SRC += ../lgw_ms.c
OBJ += lgw_ms.o
else
ifeq ($(ARCH_RADIO),SX1301_X1)
SRC += ../lgw_x1.c
OBJ += lgw_x1.o
endif
endif

ifeq ($(ARCH_RADIO),SX1301_X8)
SRC += ../lgw_x8.c
SRC += ../spi_linuxdev.c
OBJ += lgw_x8.o
OBJ += spi_linuxdev.o
endif

#OBJ=$(SRC:.c=.o)

HDEPS=\
	../../include/_whatstr.h\
	../spi_linuxdev.h\
	../xlap.h\
	../define.h\
	../struct.h\
	../infrastruct.h\
	../system_api.h


################################################################################
all	: $(EXELRR).x

$(EXELRR).x : $(OBJ) $(LIB)
	$(CC) $(STATIC) $(LDFLAGS) -o $@ $^ $(MYLDFLAGS) $(THREADLD)

%.o	: ../%.c $(HDEPS)
	$(CC) $(CCFLAGS) $(INC) $(DEFINC) -c $<

clean	:
	@(rm _cproto.h *.x *.o *.a core *.log 2>/dev/null || exit 0)

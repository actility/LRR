################################################################################
include $(ROOTACT)/rtbase/base/makefile.def
include $(ROOTACT)/lrr/base/makefile.def

################################################################################

# migrating to external configuration of gateway capabilities
# remove setting present into this makefile when configuration file is ready
# done
# - Multitech


# Makefile for lrr with
#	ARCH_RADIO=SX1301_X8
#	ARCH_COMM=spi

ifneq ($(SYSTEM_NAME),)
    # used to add a dependency on
    TARGETDEF = $(ROOTACT)/lrr/com/makefile.target
    include $(TARGETDEF)

else
    CFLAGS += -DWITH_SPI -DWITH_SX1301_X8
endif

# not an option but mandatory
CFLAGS += -DLP_MODULE_LRR

SEMTECH=lora_gateway/libsx1301ar
# define LIBSEMTECH and INCSEMTECH if not already defined by target toolchain definition
ifeq ($(LIBSEMTECH),)
    LIBSEMTECH=$(ROOTACT)/$(SEMTECH)/libsx1301ar.a
endif
ifeq ($(INCSEMTECH),)
    INCSEMTECH=$(ROOTACT)/$(SEMTECH)/inc
endif

LIB=\
	$(LIBSEMTECH)\
	$(LIBBASE)

INC=-I../include -I../../include -I$(ROOTACT) -I$(ROOTACT)/include -I$(INCSEMTECH)

MYLDFLAGS=-lm -lrt -lcrypto

ifeq ($(SYSTEM),ciscoms)
	CFLAGS += -DWITH_GPS
endif

ifeq ($(SYSTEM),fcloc)
	CFLAGS += -DWITH_GPS
endif

# HALVERSION_MAJOR and HALVERSION_MINOR defined in lrr/MAKE
ifeq ($(HALVERSION_MAJOR),5)
	CFLAGS += -DHAL_VERSION_5
ifeq ($(HALVERSION_MINOR),1)
	CFLAGS += -DHAL_VERSION_5_1
else
	CFLAGS += -DHAL_VERSION_5_0
endif
endif

### Build mode can be either 'debug' or 'release'. Default is 'release'
BUILD_MODE ?=release
ifeq ($(BUILD_MODE),debug)
	CFLAGS += -g -O0
endif

### SPI headers and code depend on the system 
ifneq ($(SYSTEM_NAME),)
ifneq ($(SPI_DIR),)
	SPIDIR=$(SPI_DIR)
	INC += -I$(SPIDIR)
endif
else
ifeq ($(SYSTEM),$(filter $(SYSTEM),ciscoms))

	SPIDIR=../spi/$(SYSTEM)
else
	SPIDIR=../spi/generic_x8
endif
INC += -I$(SPIDIR)
endif

################################################################################
EXELRR=lrr
EXE_GET_HAL_VERSION=get_hal_version
EXE_GET_FPGA_VERSION=get_fpga_version
CHIP_ID=chip_id

SRC=\
	../networkfilter.c\
	../headerloramac.c\
	../crc-ccitt.c\
	../inivar.c\
	../lgw_gen.c\
	../gps.c\
	../cellular.c\
	../avdv.c\
	../netitf.c\
	../rttping.c\
	../shellcmd.c\
	../keycrypt.c\
	../keybuild.c\
	../dutycycle.c\
	../main.c\
	../system_api.o\
	../get_hal_version.c \
	../get_fpga_version.c\
	$(SPIDIR)/spi_linuxdev.c

ifneq ($(RADIO_FILE),)
	SRC += $(RADIO_FILE)
else
	SRC += ../lgw_x8.c
endif

OBJ=\
	networkfilter.o\
	headerloramac.o\
	crc-ccitt.o\
	inivar.o\
	lgw_gen.o\
	gps.o\
	cellular.o\
	avdv.o\
	netitf.o\
	rttping.o\
	shellcmd.o\
	keycrypt.o\
	keybuild.o\
	dutycycle.o\
	main.o\
	system_api.o\
	spi_linuxdev.o

OBJ_HAL=\
	spi_linuxdev.o \
	get_hal_version.o
OBJ_FPGA=\
	spi_linuxdev.o \
	get_fpga_version.o

ifneq ($(GET_CUSTOM_VERSION),)
OBJ_CUSTOM_VERSION=\
	$(GET_CUSTOM_VERSION).o
endif

ifneq ($(RADIO_FILE),)
	RADIO_OBJ = $(subst .c,.o,$(notdir $(RADIO_FILE)))
	OBJ += $(RADIO_OBJ)
else
	OBJ += lgw_x8.o
endif

# look for chip_id at different possible places

chipid = $(ROOTACT)/$(SEMTECH)/../util/chip_id
ifeq ("$(wildcard $(chipid))","")
	chipid = $(ROOTACT)/$(SEMTECH)/../utils/chip_id
	ifeq ("$(wildcard $(chipid))","")
		chipid = $(ROOTACT)/$(SEMTECH)/../util_chip_id/chip_id
	endif
endif

HDEPS=\
	../../include/_whatstr.h\
	../xlap.h\
	../define.h\
	../struct.h\
	../infrastruct.h\
	../system_api.h\
	$(SPIDIR)/spi_linuxdev.h

################################################################################
ifneq ($(GET_CUSTOM_VERSION),)
all	: $(EXELRR).x $(EXE_GET_HAL_VERSION).x  $(GET_CUSTOM_VERSION).x 
else
all	: $(EXELRR).x $(EXE_GET_HAL_VERSION).x $(EXE_GET_FPGA_VERSION).x $(CHIP_ID)
endif

$(EXELRR).x : $(OBJ) $(LIB)
	$(CC) $(STATIC) $(LDFLAGS) -o $@ $^ $(MYLDFLAGS) $(THREADLD)

$(EXE_GET_HAL_VERSION).x : $(OBJ_HAL) $(LIB)
	$(CC) $(LDFLAGS)  -o $@ $^ $(MYLDFLAGS) $(THREADLD)
	-cp $(EXE_GET_HAL_VERSION).x ../$(EXE_GET_HAL_VERSION).x

$(EXE_GET_FPGA_VERSION).x : $(OBJ_FPGA) $(LIB)
	$(CC) $(LDFLAGS)  -o $@ $^ $(MYLDFLAGS) $(THREADLD)
	-cp $(EXE_GET_FPGA_VERSION).x ../$(EXE_GET_FPGA_VERSION).x

$(GET_CUSTOM_VERSION).x : $(OBJ_CUSTOM_VERSION) $(LIB)
	$(CC) $(LDFLAGS)  -o $@ $^ $(MYLDFLAGS) $(THREADLD)
	-cp $(GET_CUSTOM_VERSION).x ../$(GET_CUSTOM_VERSION).x

$(CHIP_ID) :
	-cp $(chipid) ../$(CHIP_ID)

%.o	: ../%.c $(HDEPS)
	$(CC) $(CCFLAGS) $(INC) $(DEFINC) -c $<

%.o	: $(SPIDIR)/%.c $(HDEPS)
	$(CC) $(CCFLAGS) $(INC) $(DEFINC) -c $<

ifneq ($(RADIO_FILE),)
$(RADIO_OBJ) : $(RADIO_FILE) $(HDEPS)
	$(CC) $(CCFLAGS) $(INC) $(DEFINC) -c $<
endif

clean	:
	@(rm _cproto.h *.x *.o *.a ../chip_id core *.log 2>/dev/null || exit 0)

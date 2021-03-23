################################################################################
include $(ROOTACT)/rtbase/base/makefile.def
include $(ROOTACT)/lrr/base/makefile.def
################################################################################

# migrating to external configuration of gateway capabilities
# remove setting present into this makefile when configuration file is ready
# done
# - Multitech

# Makefile for lrr with
#	ARCH_RADIO=SX1301_X1
#	ARCH_COMM=spi

ifneq ($(SYSTEM_NAME),)
	# used to add a dependency on
	TARGETDEF = $(ROOTACT)/lrr/com/makefile.target
	include $(TARGETDEF)
	CFLAGS += -DLP_MODULE_LRR -DUSELIBLGW3
else
	CFLAGS += -DWITH_SPI -DWITH_SX1301_X1 -DLP_MODULE_LRR -DUSELIBLGW3
endif

# not an option but mandatory
# SEMTECH libloragw >= 3.2.0

SEMTECH=lora_gateway/libloragw
LIBSEMTECH=$(ROOTACT)/$(SEMTECH)/libloragw.a
INCSEMTECH=$(ROOTACT)/$(SEMTECH)/inc
OBJSEMTECH=$(ROOTACT)/$(SEMTECH)/obj


LIB=\
    $(LIBSEMTECH)\
    $(LIBBASE)

ifeq ($(CONFIG_MULTI_BOARD),y)
    LIBLORACOMMON=$(ROOTACT)/lrr/com/exe_spi_x1/libloracommon.a
    LIBLORABOARD0=$(ROOTACT)/lrr/com/exe_spi_x1/libloraboard0.so
    LIBLORABOARD1=$(ROOTACT)/lrr/com/exe_spi_x1/libloraboard1.so
    LIBFORLRR=\
        $(LIBLORACOMMON)\
		$(LIBBASE)
else
    LIBLORABOARD0=
    LIBLORABOARD1=
    LIBLORACOMMON=
    LIBFORLRR=$(LIB)
endif

INC=-I../include -I../../include -I$(ROOTACT) -I$(ROOTACT)/include -I$(INCSEMTECH)

### SPI headers and code depend on the system 
ifneq ($(SYSTEM_NAME),)
ifneq ($(SPI_DIR),)
	SPIDIR=$(SPI_DIR)
	INC += -I$(SPIDIR)
endif
else
SPIDIR=../spi/generic_x1
INC += -I$(SPIDIR)
endif

MYLDFLAGS=-lm -lrt -lcrypto

#----------------------------------------------------
ifneq ($(SYSTEM_NAME),)
#----------------------------------------------------

# note: autoconfig.h contains information about HAL version

ifneq ($(HAL_LIBRARY_NAME),)
    # HAL already built
    MYLDFLAGS += $(ROOTACT)/$(SEMTECH)/$(HAL_LIBRARY_NAME)
	# only depends on rtbase
    LIB=$(LIBBASE)
endif

#----------------------------------------------------
else
#----------------------------------------------------

ifeq ($(SYSTEM),fcmlb)
	CFLAGS += -DWITH_GPS
endif

ifeq ($(SYSTEM),oielec)
	CFLAGS += -DWITH_GPS
endif

ifeq ($(SYSTEM),tracknet)
    # HAL already built (.so)
    MYLDFLAGS += $(ROOTACT)/$(SEMTECH)/libloragw.so
    LIB=$(LIBBASE)
endif

#----------------------------------------------------
endif
#----------------------------------------------------

# HALVERSION_MAJOR and HALVERSION_MINOR defined in lrr/MAKE
ifeq ($(HALVERSION_MAJOR),5)
	CFLAGS += -DHAL_VERSION_5
endif

################################################################################
EXELRR=lrr
EXE_GET_HAL_VERSION=get_hal_version
EXE_GET_FPGA_VERSION=get_fpga_version

SRC=\
	../networkfilter.c\
	../headerloramac.c\
	../crc-ccitt.c\
	../inivar.c\
	../leds.c\
	../gps.c\
	../cellular.c\
	../lgw_gen.c\
	../lgw_synchro.c\
	../avdv.c\
	../netitf.c\
	../rttping.c\
	../shellcmd.c\
	../semtech.c\
	../keycrypt.c\
	../keybuild.c\
	../dutycycle.c\
	../main.c\
	../lgw_x1.c\
	../system_api.c\
	../get_hal_version.c \
	../get_fpga_version.c

OBJ=\
	networkfilter.o\
	headerloramac.o\
	crc-ccitt.o\
	inivar.o\
	leds.o\
	lgw_gen.o\
	lgw_synchro.o\
	gps.o\
	cellular.o\
	avdv.o\
	netitf.o\
	rttping.o\
	shellcmd.o\
	semtech.o\
	keycrypt.o\
	keybuild.o\
	dutycycle.o\
	main.o\
	system_api.o\
	lgw_x1.o

OBJ_HAL=\
	get_hal_version.o

OBJ_FPGA=\
	get_fpga_version.o

HDEPS=\
	../../include/_whatstr.h\
	$(SPIDIR)/spi_linuxdev.h\
	../xlap.h\
	../define.h\
	../struct.h\
	../infrastruct.h\
	../system_api.h

# loragw_gps.o appears twice as it contains board and non-board functions
# (decoding of NMEA packets for instance)

OBJSEMTECHCOMMON=\
    $(OBJSEMTECH)/loragw_gps.o

OBJSEMTECHBOARD=\
    $(OBJSEMTECH)/loragw_hal.o\
    $(OBJSEMTECH)/loragw_reg.o\
    $(OBJSEMTECH)/loragw_spi.o\
    $(OBJSEMTECH)/loragw_radio.o\
    $(OBJSEMTECH)/loragw_fpga.o\
    $(OBJSEMTECH)/loragw_gps.o\
    $(OBJSEMTECH)/loragw_lbt.o\
    $(OBJSEMTECH)/loragw_aux.o


################################################################################
all	: $(EXELRR).x $(EXE_GET_HAL_VERSION).x $(EXE_GET_FPGA_VERSION).x \
      $(LIBLORABOARD0) $(LIBLORABOARD1) $(LIBLORACOMMON)

$(EXELRR).x : $(OBJ) $(LIBFORLRR)
	$(CC) $(STATIC) $(LDFLAGS) -s -o $@ $^ $(MYLDFLAGS) $(THREADLD) $(LIBLORACOMMON)

ifeq ($(CONFIG_MULTI_BOARD),y)
# build static library with obj not related to board
$(LIBLORACOMMON): $(OBJSEMTECHCOMMON)
	$(AR) rcs $@ $^ 

# build shared library with obj related to board
$(LIBLORABOARD0): $(OBJSEMTECHBOARD)
	$(CC) -shared -fPIC $^ -o $@

$(LIBLORABOARD1): $(OBJSEMTECHBOARD)
	$(CC) -shared -fPIC $^ -o $@

endif

$(EXE_GET_HAL_VERSION).x : $(OBJ_HAL) $(LIB)
	$(CC) $(LDFLAGS) -s -o $@ $^ $(MYLDFLAGS) $(THREADLD)
	-cp $(EXE_GET_HAL_VERSION).x ../$(EXE_GET_HAL_VERSION).x

$(EXE_GET_FPGA_VERSION).x : $(OBJ_FPGA) $(LIB)
	$(CC) $(LDFLAGS) -s -o $@ $^ $(MYLDFLAGS) $(THREADLD)
	-cp $(EXE_GET_FPGA_VERSION).x ../$(EXE_GET_FPGA_VERSION).x

%.o	: ../%.c $(HDEPS)
	$(CC) $(CCFLAGS) $(INC) $(DEFINC) -c $<

clean	:
	@(rm _cproto.h *.x *.o *.a core *.log 2>/dev/null || exit 0)

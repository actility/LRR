################################################################################
include $(ROOTACT)/rtbase/base/makefile.def
include $(ROOTACT)/lrr/base/makefile.def

################################################################################

# migrating to external configuration of gateway capabilities
# remove setting present into this makefile when configuration file is ready
# done
# - Multitech

ifneq ($(SYSTEM_NAME),)
    # used to add a dependency on
    TARGETDEF = $(ROOTACT)/lrr/com/makefile.target
    include $(TARGETDEF)
    CFLAGS += -Wno-unused-result -DLP_MODULE_LRR -DUSELIBLGW3
else
    CFLAGS += -Wno-unused-result -DWITH_TTY -DWITH_SX1301_X1 -DLP_MODULE_LRR -DUSELIBLGW3
endif

# Makefile for lrr with
#	ARCH_RADIO=SX1301_X1
#	ARCH_COMM=tty

# not an option but mandatory
# SEMTECH libloragw >= 3.2.0

SEMTECH=lora_gateway/libloragw
LIBSEMTECH=$(ROOTACT)/$(SEMTECH)/libloragw.a
INCSEMTECH=$(ROOTACT)/$(SEMTECH)/inc

LIB=\
	$(LIBSEMTECH)\
	$(LIBBASE)

INC=-I../include -I../../include -I$(ROOTACT) -I$(ROOTACT)/include -I$(INCSEMTECH)

MYLDFLAGS=-lm -lrt -lcrypto

ifeq ($(SYSTEM),linux-x86)
#	STATIC=-static
endif

################################################################################
EXELRR=lrr
EXE_GET_HAL_VERSION=get_hal_version

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
	../get_hal_version.c

OBJ=\
	networkfilter.o\
	headerloramac.o\
	crc-ccitt.o\
	inivar.o\
	leds.o\
	gps.o\
	cellular.o\
	lgw_gen.o\
	lgw_synchro.o\
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

HDEPS=\
	../../include/_whatstr.h\
	../xlap.h\
	../define.h\
	../struct.h\
	../infrastruct.h\
	../system_api.h


################################################################################
all	: $(EXELRR).x $(EXE_GET_HAL_VERSION).x

$(EXELRR).x : $(OBJ) $(LIB)
	$(CC) $(STATIC) $(LDFLAGS) -o $@ $^ $(MYLDFLAGS) $(THREADLD)

$(EXE_GET_HAL_VERSION).x : $(OBJ_HAL) $(LIB)
	$(CC) $(LDFLAGS) -s -o $@ $^ $(MYLDFLAGS) $(THREADLD)
	-cp $(EXE_GET_HAL_VERSION).x ../$(EXE_GET_HAL_VERSION).x

%.o	: ../%.c $(HDEPS)
	$(CC) $(CCFLAGS) $(INC) $(DEFINC) -c $<

clean	:
	@(rm _cproto.h *.x *.o *.a core *.log 2>/dev/null || exit 0)

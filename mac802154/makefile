################################################################################
include $(ROOTACT)/rtbase/base/makefile.def
include $(ROOTACT)/lrr/base/makefile.def
################################################################################

ifneq ($(SYSTEM_NAME),)
	# used to add a dependency on
	TARGETDEF = $(ROOTACT)/lrr/com/makefile.target
	include $(TARGETDEF)
endif

SRC=\
	mac802154.c\
	macip6.c
OBJ=$(SRC:.c=.o)
################################################################################


all	: $(OBJ)


%.o	: %.c
	$(CC) $(CCFLAGS) -c $<

clean	:
	@(rm *.x *.o *.a core 2>/dev/null || exit 0)


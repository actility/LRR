include $(ROOTACT)/rtbase/base/makefile.def
include $(ROOTACT)/lrr/base/makefile.def

all: _whatstr.h _halversion.h

_whatstr.h: ../Version
	$(MKWHAT) -p lrr -vf ../Version > _whatstr.h

_halversion.h: _whatstr.h
	$(MKHALV) > _halversion.h
	cat _halversion.h >> _whatstr.h

clean:
	@(rm *.x *.o *.a core _whatstr.h _halversion.h 2>/dev/null || exit 0)


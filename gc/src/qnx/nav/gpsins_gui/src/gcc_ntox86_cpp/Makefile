#
# Application Makefile   
#

DBG_DEBUG = -g
OPT_DEBUG = -O0
DBG_RELEASE = -fomit-frame-pointer
OPT_RELEASE = -O3
ifdef DBG
DEBUG=$(DBG_$(DBG))
OPTIM=$(OPT_$(DBG))
endif

ifndef DEBUG
DEBUG = $(DBG_DEBUG)
endif

ifndef OPTIM
OPTIM = $(OPT_DEBUG)
endif

include	../abOfiles
include ../indOfiles

include	../abSfiles
include ../indSfiles

include	../abHfiles
include ../indHfiles

include	../abWfiles
include	../abLfiles

PLATFORM=gcc_ntox86_cpp
PHABOPTS= 

CC  = qcc
CXX = qcc
LD  = qcc

CFLAGS  = -V$(PLATFORM) -w8 $(OPTIM) $(DEBUG) $(PHABOPTS) -I.
CXXFLAGS = $(CFLAGS)
LDFLAGS = -V$(PLATFORM) -Bstatic $(DEBUG) -l phexlib $(ABLIB) $(PHABOPTS)
SDFLAGS = -V$(PLATFORM)          $(DEBUG) -l phexlib $(ABLIB) $(PHABOPTS)

VPATH = ..

#
# Application Program
#

shr:	$(ABOBJ) $(MYOBJ)
		$(LD) $(ABOBJ) $(MYOBJ) $(SDFLAGS) -M -o gpsins_gui
		usemsg gpsins_gui ../Usemsg
		phabbind gpsins_gui $(ABMOD)

app:	$(ABOBJ) $(MYOBJ)
		$(LD) $(ABOBJ) $(MYOBJ) $(LDFLAGS) -M -o gpsins_gui
		usemsg gpsins_gui ../Usemsg
		phabbind gpsins_gui $(ABMOD)

clean:
		rm -f *.o *.err *.map gpsins_gui

install:
		-cp -vnf gpsins_gui /usr/photon/bin

#
# Headers
#

$(ABOBJ) $(MYOBJ):	$(MYHDR)

#
# Prototype Generation
#

proto:
		-rm -f proto.h
		approto -p $(ABSRC) $(MYSRC) > proto.h

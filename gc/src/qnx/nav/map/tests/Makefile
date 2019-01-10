LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)


#===== Macros and Targets added to file
ifndef SANDBOX
SANDBOX=$(HOME)/sandbox
endif
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
LIBS+= newsteer gccontrol gcui gccomm gcmath socket
LIBS_g+= newsteer_g gccontrol_g gcui_g gccomm_g gcmath_g socket
EXTRA_INCVPATH+=../../.. ../../../newsteer
EXTRA_LIBVPATH+=../../../newsteer/x86/a
EXTRA_LIBVPATH_g+=../../../newsteer/x86/a-g
EXTRA_SRCVPATH+=../../..

EXCLUDE_OBJS=main.o

NAME=tiltCorrect

LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)

SANDBOX+=$(HOME)/sandbox
LIBS+=gccontrol gccomm gcui gcmath getoptions socket
LIBS_g+=gccontrol_g gccomm_g gcui_g gcmath_g getoptions_g
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk


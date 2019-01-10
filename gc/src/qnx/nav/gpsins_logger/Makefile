LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)

SANDBOX+=$(HOME)/sandbox
LIBS+=gcmath gcui
LIBS_g+=gcmath_g gcui_g
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
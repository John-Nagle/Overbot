LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)

SANDBOX+=$(HOME)/sandbox
LIBS+= gcui
LIBS_g+=gcui
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
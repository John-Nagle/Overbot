LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)

SANDBOX+=$(HOME)/sandbox
LIBS+=gcmath getoptions
LIBS_g+=gcmath_g getoptions_g
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
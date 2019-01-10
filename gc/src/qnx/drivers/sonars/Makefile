LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)

SANDBOX+=$(HOME)/sandbox
LIBS+=getoptions gccomm
LIBS_g+=getoptions_g gccomm_g
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)

#===== Macros and Targets added to file
SANDBOX+=$(HOME)/sandbox
LIBS+=getoptions dscud5
#dscud5 does not come with a debug version
LIBS_g+=getoptions_g dscud5
PUBLIC_STARTFILES = 
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
PUBLIC_HEADERS = 
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk	# for install_headers
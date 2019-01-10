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
LIBS+=gccontrol gccomm gcui socket
PUBLIC_STARTFILES = start_check_controller.txt
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk

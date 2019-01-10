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
LIBS+=gcactserver gccontrol gcui
PUBLIC_STARTFILES = start_check_actserver.txt
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
PUBLIC_HEADERS = check_actserver.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk	# for install_headers

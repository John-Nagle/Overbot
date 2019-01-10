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
LIBS+=gcui gccomm socket
PUBLIC_STARTFILES = start_moveserver.txt
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
PUBLIC_HEADERS = moveservermsg.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk	# for install_headers

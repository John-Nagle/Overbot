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
PUBLIC_HEADERS = actserver.h actserver_.h actservermenu.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk
LIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)


#===== Macros and Targets added to file
SANDBOX=$(HOME)/sandbox
LIBS+=
#	Look in OpenSteer internal directories.
#	Should be unnecessary, but exported include files include too much.
EXTRA_INCVPATH+=$(SANDBOX)/gc/src/qnx/common/lib/opensteer/include
EXTRA_INCVPATH+=.. ../.. ../../..
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk 
# PUBLIC_HEADERS = mapservermsg.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk	# for install_headers

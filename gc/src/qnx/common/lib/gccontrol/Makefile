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
PUBLIC_HEADERS = timedloop.h timedloopmenu.h controller.h controllermenu.h steeringcontroller.h throttlecontroller.h brakecontroller.h transmissioncontroller.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk
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
PUBLIC_HEADERS = Constants.h Conversions.h Frames.h Kalman.h Matrix_Invert.h Matrix.h Nav.h Quat.h rk4.h SixDOF.h Vector_Rotate.h Vector.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk
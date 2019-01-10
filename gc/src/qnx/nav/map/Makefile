XXLIST=CPU
ifndef QRECURSE
QRECURSE=recurse.mk
ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif
endif
include $(QRDIR)$(QRECURSE)


#===== Macros and Targets added to file
SANDBOX+=$(HOME)/sandbox
LIBS+= newsteer gccontrol gcui gccomm gcmath socket
LIBS_g+= newsteer_g gccontrol_g gcui_g gccomm_g gcmath_g socket
#	Look in OpenSteer internal directories.
#	Should be unnecessary, but exported include files include too much.
# EXTRA_INCVPATH+=$(SANDBOX)/gc/src/qnx/common/lib/opensteer/include
#	These should be done using the correct QNX macros.
EARLY_DIRS+=newsteer
EXTRA_INCVPATH+=../../newsteer
EXTRA_LIBVPATH+=../../newsteer/x86/a
EXTRA_LIBVPATH_g+=../../newsteer/x86/a-g
PUBLIC_STARTFILES = start_mapserver.txt
include $(SANDBOX)/gc/src/qnx/common/make/bin.mk 
PUBLIC_HEADERS = mapservermsg.h
include $(SANDBOX)/gc/src/qnx/common/make/lib.mk	# for install_headers
#	***STILL DOESNT RECOGNIZE DEPENDENCY ON ARCHIVE***
######################################################################
#
#    File: bin.mk
#
#    Usage: 
#        SANDBOX = $(HOME)/cvsfiles
#        LIBS += gcxxx
#        include $(SANDBOX)/gc/src/qnx/common/make/bin.mk
#
#    Description:
#        A generic set of make macros and targets for making an
#        executable.
#
#    Written By:
#        Celia Oakley
#        Team Overbot
#        September, 2003
#
######################################################################

#===== directories to search for library files.
EXTRA_LIBVPATH += $(SANDBOX)/gc/src/qnx/common/lib

#===== location for public executables
INSTALLDIR += ../$(SANDBOX)/gc/src/qnx/common/bin

#===== location for watchdog startfiles
INSTALL_ROOT_STARTFILES = $(SANDBOX)/gc/src/qnx/common/startfiles

#===== target to install watchdog startfiles
install_startfiles:
	/bin/cp -vnfpc $(PUBLIC_STARTFILES) $(INSTALL_ROOT_STARTFILES)

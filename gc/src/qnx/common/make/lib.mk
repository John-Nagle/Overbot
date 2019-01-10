######################################################################
#
#    File: lib.mk
#
#    Usage: 
#        SANDBOX = $(HOME)/cvsfiles
#        PUBLIC_HEADERS = myheader.h
#        include $(SANDBOX)/gc/src/qnx/common/make/lib.mk
#
#    Description:
#        A generic set of make macros and targets for making a
#        library.
#
#    Written By:
#        Celia Oakley
#        Team Overbot
#        September, 2003
#
######################################################################

#===== location for public libraries
INSTALLDIR += ../$(SANDBOX)/gc/src/qnx/common/lib

#===== location for public header files
INSTALL_ROOT_HEADERS = $(SANDBOX)/gc/src/qnx/common/include

#===== target to install public headers
install_headers:
	/bin/cp -vnfpc $(PUBLIC_HEADERS) $(INSTALL_ROOT_HEADERS)

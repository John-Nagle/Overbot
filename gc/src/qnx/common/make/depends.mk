######################################################################
#
#    File: depends.mk
#
#    Usage: 
#        SANDBOX = $(HOME)/cvsfiles
#        include $(SANDBOX)/gc/src/qnx/common/make/depends.mk
#
#    Description:
#        This Makefile will create a dependencies file in the project
#        directory that consists of a set of targets relating the
#        object files to the source and header files.
#
#        The Makefile will complain the first time it is run when
#        the $(DEPENDENCIES) file is not created yet.  This is okay,
#        because then an up-to-date version of the dependencies file
#        is created.
#
#        This capability should be built into QNX 6.3?
#
#    Written By:
#        Celia Oakley
#        Team Overbot
#        September, 2003
#
######################################################################

#===== system include directories beyond the defaults
EXTRA_INCVPATH += $(SANDBOX)/gc/src/qnx/common/include

#===== the file containing the dependencies
DEPENDENCIES = $(PROJECT_ROOT)/dependencies.mk

#===== need to have each include directory appended with -I
DEPENDS_INCPATH = $(INCVPATH:%=-I%)

#===== the target for the dependencies file
$(DEPENDENCIES): $(SRCS)
	@-rm $(DEPENDENCIES)
	-gcc -MM $(DEPENDS_INCPATH) $(SRCS) > $(DEPENDENCIES)
	@echo "Dependencies updated."

#===== include the targets created in the dependency file
include $(DEPENDENCIES)

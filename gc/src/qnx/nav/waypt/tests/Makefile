SANDBOX=$(HOME)/sandbox

ifndef QCONFIG
QRDIR=$(dir $(QCONFIG))
endif


include /usr/include/qconfig.mk

INC=-I.. -I$(SANDBOX)/gc/src/qnx/common/include

all: inpoly pt

inpoly: ../inpoly.cc
	qcc -DSTANDALONE $(INC) -o $@ $?

pt: pt.cc
	qcc $(INC) -o $@ $?



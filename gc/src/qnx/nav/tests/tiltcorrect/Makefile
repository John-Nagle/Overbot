#  Makefile for Rottest
#
#	With automatic dependency update.
#	J. Nagle
#	Team Overbot
#	January, 2003

SRC = main.cpp
OBJS = main.o ../../map/x86/o-g/LMStiltcorrect.o
LIBS = 
TARGET = tiltcorrect_test
TARGETDIR = 
INCLUDE_PATH = -I. -I.. -I../../../common/include -I../../map
OUTPUTTYPE = -o
LIB_PATH = -L../../../common/lib
LIBS = -lgccontrol_g -lgccomm_g -lgcui_g -lgcmath_g

#	Everything from this point on is generic.

#	Workaround for inability of QCC to make dependencies
DEPENDLIBPATHS = 

all: $(TARGET)
DEPENDENCIES = dependencies.make
TEMPDEPENDENCIES = dependencies.tmp
include $(DEPENDENCIES)

#	Compile options
CC = QCC  -Vgcc_ntox86
CPPFLAGS = -Wall -Werror -g $(INCLUDE_PATH) 
LINKER = QCC -Vgcc_ntox86 -lang-c++
LINKERFLAGS = -g $(LIB_PATH)

#	Make the actual target file
$(TARGET): $(OBJS) $(DEPENDENCIES)
	$(LINKER)  $(LINKERFLAGS)  $(OBJS)  $(LIBS) $(OUTPUTTYPE) $(TARGET)

#	General rules for compiles
.cpp.o: 
	$(CC) $(CPPFLAGS) -c $<
.SUFFIXES: .cpp .c .o

#	Rebuild dependency list. This happens every time any source file
#	changes, which is inefficient, but not overly so.
$(DEPENDENCIES): $(SRC)
	-rm $(DEPENDENCIES)
	gcc -MM $(INCLUDE_PATH) $(DEPENDLIBPATHS) $(SRC) > $(TEMPDEPENDENCIES)
	mv $(TEMPDEPENDENCIES) $(DEPENDENCIES)
	echo "Dependencies updated."

clean:
	rm *.o
	rm $(DEPENDENCIES) $(TEMPDEPENDENCIES)

install:	$(TARGET)
	cp $(TARGET) ../../common/bin/

#  Makefile for devfw-ohci/examples
#
#	With automatic dependency update.
#	J. Nagle
#	Animats
#	January, 2002

SRC = cameratest.cpp 
OBJS = cameratest.o
TARGET = cameratest
## LIB_PATH = -L ../mindready/app -L ../dc1394 
##LIB_PATH =  -L ../dc1394 
##LIBS = -ldc1394
##INCLUDE_PATH = -I.. -I../dc1394
INCLUDE_PATH = -I../../../common/include
## -I../mindready/inc -I../mindready/app 
OUTPUTTYPE = -o

#	Everything from this point on is generic.

#	Workaround for inability of QCC to make dependencies
DEPENDLIBPATHS = 
## -I/usr/include/g++-3 -I/usr/ntox86/include -I/usr/lib/gcc-lib/ntox86/2.95.3/include 

all: $(TARGET)
DEPENDENCIES = dependencies.make
TEMPDEPENDENCIES = dependencies.tmp
include $(DEPENDENCIES)

#Compile options
##CC = qcc -Vgcc_ntox86_gpp
CC = QCC  -Vgcc_ntox86
CPPFLAGS = -Wall -Werror -g $(INCLUDE_PATH) 
LINKER = QCC -Vgcc_ntox86
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


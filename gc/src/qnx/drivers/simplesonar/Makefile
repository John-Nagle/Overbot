#  Makefile for voradserver
#
#	With automatic dependency update.
#	J. Nagle
#	Animats
#	January, 2002

SRC = simplesonar.cpp
OBJS = simplesonar.o  
TARGET = simplesonar
INCLUDE_PATH = -I.. -I../../common/include
LIB_PATH = -L ../../common/lib
LIBS = -lgcui -lgccontrol -lgccomm -lsocket
OUTPUTTYPE = -o

#	Everything from this point on is generic.

#	Workaround for inability of QCC to make dependencies
DEPENDLIBPATHS = 

all: $(TARGET)
DEPENDENCIES = dependencies.make
TEMPDEPENDENCIES = dependencies.tmp
include $(DEPENDENCIES)

#Compile options
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
	
install:
	cp voradserver ../../common/bin/voradserver


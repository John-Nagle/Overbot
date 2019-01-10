#  Makefile for road follower
#
#	With automatic dependency update.
#	J. Nagle
#	Animats
#	January, 2003

SRC = roadfollower.cpp offroadfollower.cpp ralphfollower.cpp imagesampler.cpp \
	sampleiterator.cpp kmeans.cpp iplimagecameraread.cpp ffmpegwrite.cpp avconvertimage.cpp roadserver.cpp
OBJS = roadfollower.o offroadfollower.o ralphfollower.o imagesampler.o \
	sampleiterator.o kmeans.o iplimagecameraread.o ffmpegwrite.o avconvertimage.o roadserver.o
# TARGET = libroadfollower.a
TARGET = roadfollowerserver
INSTALLDIR = $(HOME)/sandbox/gc/src/qnx/common/bin
INCLUDE_PATH = -I.. -I../../common/include/cv -I../../common/include/cvaux -I../../common/include -I../../common/include/ffmpeg
# OUTPUTTYPE = -A
OUTPUTTYPE = -o
LIB_PATH =  -L../../common/lib  
LIBS = -l cv  -l avformat -l avcodec -l gcui

#	Everything from this point on is generic.

#	Workaround for inability of QCC to make dependencies
DEPENDLIBPATHS = 

all: $(TARGET)
DEPENDENCIES = dependencies.make
TEMPDEPENDENCIES = dependencies.tmp
include $(DEPENDENCIES)

#Compile options
CC = QCC  -Vgcc_ntox86 
CPPFLAGS = -Wall -Werror -g $(INCLUDE_PATH) -O
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
	cp $(TARGET) $(INSTALLDIR)/$(TARGET)


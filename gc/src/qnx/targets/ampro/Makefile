#  Makefile for QNX boot images
#
#	J. Nagle
#	Team Overbot
#	August, 2003

TARGETDIR=/common/qnx/ampro/build
TARGET1=cm400qnxbase.ifs
SRC1=cm400qnxbase.build
MKIFSFLAGS= -r /common/qnx/build
TARGET=$(TARGETDIR)/$(TARGET1)


all: $(TARGET)

$(TARGETDIR)/$(TARGET1):	$(SRC1)
	mkifs $(MKIFSFLAGS) $< $@

#	Everything from this point on is generic.

.cpp.o: 
	$(CC) $(CPPFLAGS) -c $<
.SUFFIXES: .cpp .c .o

.build.ifs: 
	mkifs $(MKIFSFLAGS)  $< $@
.SUFFIXES: .build .ifs

clean:
	rm *.ifs



#  Makefile for devfw-ohci

##CC = qcc -Vgcc_ntox86_gpp
CC = QCC  -Vgcc_ntox86
LINK = QCC -Vgcc_ntox86
LINKFLAGS = -g 
LIBS =  -l 1394llaMulti 

INCLUDE_PATH =   -I../../common/include  -I. -I/usr/include/mindready/1394
## PROFILEFLAGS = -p
CPPFLAGS = -Wall  -gstabs+ $(PROFILEFLAGS) $(INCLUDE_PATH) 
# CPPFLAGS = -Wall  -fexceptions -O2 $(INCLUDE_PATH) 
# CPPFLAGS = -Wall  -g -fexceptions -O2 -I/usr/include/g++-3 -I/usr/ntox86/include -I/usr/lib/gcc-lib/ntox86/2.95.3/include
## CPPFLAGS = -Wall  -g -O2  -I. -I/usr/include/g++-3 -I/usr/ntox86/include -I/usr/lib/gcc-lib/ntox86/2.95.3/include

OBJS = devfw-ohci.o devindex.o businfo.o  fwadapter.o utilities.o devfactory.o  devdccam.o devdccamtables.o
OBJS += logprint.o
OBJS += drvadapter.o drvbus.o drvasync.o drviso.o

devfw-ohci: $(OBJS)
	$(LINK)  $(LINKFLAGS) $(LIB_PATH)  $(OBJS) $(LIBS) -o devfw-ohci

.cpp.o:
	$(CC) $(CPPFLAGS) -c $<
.SUFFIXES: .cpp .c .o

clean:
	rm *.o

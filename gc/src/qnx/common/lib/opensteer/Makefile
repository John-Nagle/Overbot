
#  Makefile for OpenSteer as part of the Overbot system
#	John Nagle
#	Team Overbot
#	January, 2005

#	Where things go
LIBDIR=..
INCLUDEDIR=../../include
#	What we need to export
## INCLUDES=ReactiveDriver.h Homunculus.h

all:
	cd linux; make CC=QCC

clean:
	cd linux; make clean
	
install: all
	cp linux/objs_optimized/libopensteer.a ${LIBDIR}
	
install_headers:
	cp plugins/Overbot/ReactiveDriver.h ${INCLUDEDIR}
	cp plugins/Overbot/Homunculus.h ${INCLUDEDIR}
	cp plugins/Overbot/oou.h ${INCLUDEDIR}
	cp include/OpenSteer/SimpleVehicle.h ${INCLUDEDIR}

	
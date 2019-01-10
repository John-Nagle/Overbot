#
#	Makefile for "ffmpeg"
#
#	This makes the codecs from a downloaded archive.
#	This build process does not support making changes to the codec files.
#	It's intended to allow building them unmodified only.
#
#	J. Nagle
#	Team Overbot
#	December, 2003
#
# make=zzzzzz
#	Version of gzipped file. Change if source changes.
#
#	Configuration
VERSION=0.4.8
INSTALLLIBDIR=..
INSTALLINCLUDEDIR=../../include/ffmpeg


#	Make all libraries
all: libavformat.a libavcodec.a

#	Make tests
libavtest: ffmpeg-$(VERSION)/config.mak
	cd ffmpeg-$(VERSION);/usr/bin/make libavtest


#	Install into project directories
install: libavformat.a libavcodec.a
	cp libavformat.a $(INSTALLLIBDIR)
	cp libavcodec.a $(INSTALLLIBDIR)
	-mkdir $(INSTALLINCLUDEDIR)
	cp  ffmpeg-$(VERSION)/libavcodec/*.h $(INSTALLINCLUDEDIR)
	cp  ffmpeg-$(VERSION)/libavformat/*.h $(INSTALLINCLUDEDIR)
	
#	Unpack gzipped file
ffmpeg.tar: ffmpeg-$(VERSION).tar.gz
	-rm ffmpeg.tar.gz
	cp -p ffmpeg-$(VERSION).tar.gz ffmpeg.tar.gz
	gunzip ffmpeg.tar.gz

#	Untar tar file to get configure file
ffmpeg-$(VERSION)/configure: ffmpeg.tar
	-rm -r ffmpeg-$(VERSION)
	tar xf ffmpeg.tar
	touch ffmpeg-$(VERSION)/configure
	
#	Run configure to get makefile
ffmpeg-$(VERSION)/config.mak: ffmpeg-$(VERSION)/configure
	cd ffmpeg-$(VERSION); ./configure --disable-v4l --disable-dv1394    --disable-audio-oss --disable-network --disable-ffserver --cc=qcc
	echo Configure complete.
	
#	Make libavcodec.a
ffmpeg-$(VERSION)/libavcodec/libavcodec.a: ffmpeg-$(VERSION)/config.mak
	cd ffmpeg-$(VERSION);/usr/bin/make 
	
#	Make libavformat.a
ffmpeg-$(VERSION)/libavformat/libavformat.a: ffmpeg-$(VERSION)/config.mak
	cd ffmpeg-$(VERSION);/usr/bin/make 
	
libavcodec.a: ffmpeg-$(VERSION)/libavcodec/libavcodec.a
	cp ffmpeg-$(VERSION)/libavcodec/libavcodec.a .
	
libavformat.a: ffmpeg-$(VERSION)/libavformat/libavformat.a
	cp ffmpeg-$(VERSION)/libavformat/libavformat.a .
	
#	Clean 
clean:
	-rm *.tar
	-rm *.a
	cd ffmpeg-$(VERSION); /usr/bin/make clean		
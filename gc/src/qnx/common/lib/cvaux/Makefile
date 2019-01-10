


LIB = ../libcvaux.so

libcvaux_INCLUDES = -I. -I include/cvaux

libcvaux_SOURCES = \
cv3dtracker.cpp    cvface.cpp             cvhmm1d.cpp          \
cvaux.cpp          cvfacedetection.cpp    cvlcm.cpp            cvsubdiv2.cpp \
cvauxutils.cpp     cvfacetemplate.cpp     cvlee.cpp            cvtexture.cpp \
cvcalibfilter.cpp  cvfindface.cpp         cvmat.cpp            cvvecfacetracking.cpp \
cvdpstereo.cpp     cvhaar.cpp             cvmorphcontours.cpp  cvvideo.cpp \
cvepilines.cpp     cvhaarfacecascade.cpp  cvsegment.cpp        precomp.cpp

libcvaux_OBJECTS = $(libcvaux_SOURCES:.cpp=.o)

INCLUDE = -I. -I../../include/cvaux -I../../include/cv -I../cv
CXX = QCC
CXXFLAGS += -shared $(INCLUDE)

$(LIB): $(libcvaux_OBJECTS)
	$(LD) -shared -o $(LIB) $(libcvaux_OBJECTS)


clean:
	-@rm -f *.o *~ $(LIB)
	
install:
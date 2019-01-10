


LIB = ../libcv.so

libopencv_INCLUDES = -I cv -I include/cv

libopencv_SOURCES = \
camshift.cpp                 cveigendecomposite.cpp  cvmorphing.cpp        \
cvabsdiff.cpp                cveigenobjectswrap.cpp  cvmotempl.cpp         \
cvaccum.cpp                  cveigenprojection.cpp   cvnorm.cpp            \
cvadapthresh.cpp             cvellipsefit.cpp        cvoptflowbm.cpp       \
cvaffine.cpp                 cvemd.cpp               cvoptflowhs.cpp       \
cvalloc.cpp                  cverror.cpp             cvoptflowlk.cpp       \
cvapprox.cpp                 cvfeatureselect.cpp     cvouttext.cpp         \
cvarithm.cpp                 cvfindhandregion.cpp    cvpersistence.cpp     \
cvarray.cpp                  cvfloodfill.cpp         cvpgh.cpp             \
cvcalccontrasthistogram.cpp  cvfundam.cpp            cvposit.cpp           \
cvcalccovarmatrix.cpp        cvgeometry.cpp          cvprecomp.cpp         \
cvcalceigenobjects.cpp       cvhistogram.cpp         cvprecorner.cpp       \
cvcalcimagehomography.cpp    cvhmm.cpp               cvprewarp.cpp         \
cvcalibinit.cpp              cvhmmobs.cpp            cvprint.cpp           \
cvcalibration.cpp            cvhmmwrap.cpp           cvproject.cpp         \
cvcamshift.cpp               cvhough.cpp             cvpyramids.cpp        \
cvcanny.cpp                  cvimage.cpp             cvpyrsegmentation.cpp \
cvcmp.cpp                    cvjacobieigens.cpp      cvrand.cpp            \
cvcolor.cpp                  cvkalman.cpp            cvrotcalipers.cpp     \
cvcondens.cpp                cvlinefit.cpp           cvsamplers.cpp        \
cvcontours.cpp               cvlines.cpp             cvscanlines.cpp       \
cvcontourtree.cpp            cvlkpyramid.cpp         cvshapedescr.cpp      \
cvconvert.cpp                cvlmeds.cpp             cvsmooth.cpp          \
cvconvhull.cpp               cvlogic.cpp             cvsnakes.cpp          \
cvcopy.cpp                   cvmatchcontours.cpp     cvsubdivision2d.cpp   \
cvcorner.cpp                 cvmathfuncs.cpp         cvsumpixels.cpp       \
cvcornersubpix.cpp           cvmatmul.cpp            cvsvd.cpp             \
cvcorrespond.cpp             cvmatrix64d.cpp         cvswitcher.cpp        \
cvcreatehandmask.cpp         cvmatrix.cpp            cvtables.cpp          \
cvdatastructs.cpp            cvmean.cpp              cvtemplmatch.cpp      \
cvderiv.cpp                  cvmeansdv.cpp           cvthresh.cpp          \
cvdetectwr.cpp               cvmineval.cpp           cvundistort.cpp       \
cvdistransform.cpp           cvminmaxloc.cpp         cvutils.cpp           \
cvdominants.cpp              cvmoments.cpp           cvdxt.cpp             \
cvdrawing.cpp                cvmorph.cpp

libopencv_OBJECTS = $(libopencv_SOURCES:.cpp=.o)

INCLUDE = -I. -I../../include/cv
CXX = QCC
CXXFLAGS += -shared $(INCLUDE)

$(LIB): $(libopencv_OBJECTS)
	$(LD) -shared -o $(LIB) $(libopencv_OBJECTS)


clean:
	-@rm -f *.o *~ $(LIB)
	
install:
	cp $(LIB) ..
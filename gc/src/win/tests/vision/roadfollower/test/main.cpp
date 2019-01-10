#include "cv.h"
#include "cvcam.h"
#include "highgui.h"
#include "roadfollower.h"
#include <roadfollowerport.h>
#include <iostream.h>

#define HEIGHT 240
#define WIDTH 320

static RoadFollower *roadFollower=0;
static CvAVIWriter *writer=0;

void callback(IplImage* image) {
    assert(image);
    roadFollower->processFrame(image);
    double angle = roadFollower->getAngle();
    double score = roadFollower->getScore();
    double offset = roadFollower->getOffset();
    cout << "\tframe=" << roadFollower->getCounter()
         << "\tangle=" << angle 
         << "\tscore=" << score 
         << "\toffset=" << offset << endl;
    if (writer) {
      // write the image to a file
      cvWriteToAVI(writer, image);
    }
}
static char *usage = 
"Usage: roadfollower -ll <x> <y> -lr <x> <y> -ul <x> <y> -ur <x> <y>\n"
"                   [-w <pixels>] [-h <pixels>] [-threshold <number>]\n"
"                   [-i <file>] [-o <file>] [-debug] [-display] [-help] [-?]\n"
" -w <pixels>  The width of output video frames in pixels\n"
" -h <pixels>  The height of output video frames in pixels\n"
" -i <file>  Specify an input AVI file to get image data from\n"
" -o <file>  Specify and output AVI file to write (processed) image data to\n"
" -ll x y    Specify pixel coordinate of lower left corner of sample region\n"
" -lr x y    Specify pixel coordinate of lower right corner of sample region\n"
" -ul x y    Specify pixel coordinate of upper left corner of sample region\n"
" -ur x y    Specify pixel coordinate of upper right corner of sample region\n"
" -threshold <number>  Establish a confidence threshold based on road follower score\n"
" -debug     Set debug mode\n"
" -display   Display video in a window with heads up display\n"
" -help      Print this message\n"
" -?         Print this message\n";

#define CVEXIT(msg, code)  {cerr << msg << endl; cvcamExit(); exit(code);}

/**
 * This test program grabs frames from a Camera and processes each frame.
 *
 */
void main(int argc, char *argv[])
{
    int debug = false;
    int display = false;
    int height = 240;
    int width = 320;
    int camera = 0;
    double threshold = 0.0;
    const char *infile = 0;
    const char *outfile = 0;
    MYWINDOW MyWin = 0;

    // create an instance of a road follower
    roadFollower = new RoadFollower();
    roadFollower->setLowerLeft(30, 40);
    roadFollower->setLowerRight(80, 40);
    roadFollower->setUpperLeft(50, 70);
    roadFollower->setUpperRight(60, 70);

    // process command line args
    for (int i=1; i<argc; i++) {
        char *arg = argv[i];
        if (!strcmp(arg, "-debug")) {
            debug = true;
        }
        else if (!strcmp(arg, "-display")) {
            display = true;
        }
        else if (!strcmp(arg, "-help") || !strcmp(arg, "-?")) {
            cout << usage << endl;
            exit(0);
        }
        else if (!strcmp(arg, "-threshold")) {
            if (i+1 >= argc) {
                cerr << "Paramter -threshold requires 1 argument" << endl;
                cerr << usage << endl;
                exit(1);
            }
            threshold = atof(argv[++i]);
        }
        else if (!strcmp(arg, "-w")) {
            if (i+1 >= argc) {
                cerr << "Paramter -w requires 1 argument" << endl;
                cerr << usage << endl;
                exit(1);
            }
            width = atoi(argv[++i]);
        }
        else if (!strcmp(arg, "-h")) {
            if (i+1 >= argc) {
                cerr << "Paramter -h requires 1 argument" << endl;
                cerr << usage << endl;
                exit(1);
            }
            height = atoi(argv[++i]);
        }
        else if (!strcmp(arg, "-i")) {
            if (i+1 >= argc) {
                cerr << "Paramter -i requires 1 argument" << endl;
                cerr << usage << endl;
                exit(1);
            }
            infile = argv[++i];
        }
        else if (!strcmp(arg, "-o")) {
            if (i+1 >= argc) {
                cerr << "Paramter -o requires 1 argument" << endl;
                cerr << usage << endl;
                exit(1);
            }
            outfile = argv[++i];
        }
        else if (!strcmp(arg, "-ll")) {
            if (i+2 >= argc) {
                cerr << "Paramter -ll requires 2 arguments" << endl;
                cerr << usage << endl;
                exit(1);
            }
            int x = atoi(argv[++i]);
            int y = atoi(argv[++i]);
            roadFollower->setLowerLeft(x, y);
        }
        else if (!strcmp(arg, "-lr")) {
            if (i+2 >= argc) {
                cerr << "Paramter -lr requires 2 arguments" << endl;
                cerr << usage << endl;
                exit(1);
            }
            int x = atoi(argv[++i]);
            int y = atoi(argv[++i]);
            roadFollower->setLowerRight(x, y);
        }
        else if (!strcmp(arg, "-ul")) {
            if (i+2 >= argc) {
                cerr << "Paramter -ul requires 2 arguments" << endl;
                cerr << usage << endl;
                exit(1);
            }
            int x = atoi(argv[++i]);
            int y = atoi(argv[++i]);
            roadFollower->setUpperLeft(x, y);
        }
        else if (!strcmp(arg, "-ur")) {
            if (i+2 >= argc) {
                cerr << "Paramter -ur requires 2 arguments" << endl;
                cerr << usage << endl;
                exit(1);
            }
            int x = atoi(argv[++i]);
            int y = atoi(argv[++i]);
            roadFollower->setUpperRight(x, y);
        }
        else {
            cerr << "Invalid argument: " << arg << endl;
            cerr << usage << endl;
            exit(1);
        }
    }
    roadFollower->setDebug(debug);
    roadFollower->setDisplayResults(display);
    roadFollower->setThreshold(threshold);

    if (display) {

        // create a window and store it's id in MyWin variable.
	    // MyWin is of type HWND on Windows and Window on linux
        cvNamedWindow("original", CV_WINDOW_AUTOSIZE);
        //cvNamedWindow("processed", 0);
        //cvResizeWindow("processed", 320, 300);

        MyWin = (MYWINDOW)cvGetWindowHandle("original");

        if (!MyWin) {
            CVEXIT("Can't open display window", 1);
        }

    }

    //int width = WIDTH;
    //int height = HEIGHT;
    //cvcamSetProperty(0, CVCAM_RNDWIDTH, &width);
    //cvcamSetProperty(0, CVCAM_RNDHEIGHT, &height);

    // we're writing video to an output file
    if (outfile) {

        CvSize size;
        size.width = width;
        size.height = height;

        // create a CvAVIWriter object to write a new AVI
        writer = cvCreateAVIWriter(
            outfile,
//           CV_FOURCC('M','J','P','G'), 
           -1, // pic codec from dialog
           15, // frame rate
           size // size of AVI frames
           );
        if (!writer) {
            CVEXIT("Could not create output file", 1);
        }
    }

    if (infile) {
        // we're getting video from a video file  
        
        // create a capture object to read the AVI
        CvCapture* cvc = cvCaptureFromAVI(infile);
        if (!cvc) {
            CVEXIT("Could not capture video from input file", 1);
        }

        // loop over each frame of the AVI
        while (cvc && cvGrabFrame(cvc)) {

            // get the image for the current frame
            IplImage* img = cvRetrieveFrame(cvc);
            if (img == 0) {
                // assume this is the end of AVI
                break;
            }

            // process the image
            callback(img);
    
            // display the image
            if (display) {
                cvShowImage("original", img);
            }

            // add a delay so this program does not suck up all the CPU
            cvWaitKey(50);
        }

        // clean up
        cvReleaseCapture(&cvc);
    }
    else {
        // we're getting video from the camera

        // get the number of cameras available in the system
        //int *out;
    	//int ncams = cvcamSelectCamera(&out);
        int ncams = cvcamGetCamerasCount();
        if (!ncams && !infile) {
            CVEXIT("No cameras detected.", 1);
        }
        // grab 1st camera on the list
        //camera = out[0];

        if (display) {
            // set up display window showing output of camera
        	cvcamSetProperty(camera, CVCAM_PROP_ENABLE, CVCAMTRUE); 

            // sets the window for video rendering
        	cvcamSetProperty(camera, CVCAM_PROP_WINDOW, &MyWin);   
        }

        //this callback will process every frame
        cvcamSetProperty(camera, CVCAM_PROP_CALLBACK, callback);

        if (!cvcamInit()) {
            CVEXIT("Could not initialize CVCAM", 1);
        }


        if (cvcamStart()) {
            CVEXIT("Could not start start CVCAM", 1);
        }

    	// We're in a loop grabing frames here, this leads other threads have some CPU
        cvWaitKey(0);

        // We're done capturing video
	    cvcamStop();
    }

    //delete roadFollower;
    if (display) {
        cvDestroyWindow("original");
        //cvDestroyWindow("processed");
    }
    if (writer) {
        cvReleaseAVIWriter(&writer);
    }
	cvcamExit();
	exit(0);
}


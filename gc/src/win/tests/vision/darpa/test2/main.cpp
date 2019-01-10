#include "cv.h"
#include "cvcam.h"
#include "highgui.h"
#include <ImageSampler.h>
#include <RALPHFollower.h>
#include <KMeans.h>
#include <iostream.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#define HEIGHT 240
#define WIDTH 320
#define PI 3.141597

static ImageSampler *imgSampler;
static RALPHFollower *ralph;
static KMeans *kmeans;
static char *usage = 
"Road follower\n"
" -i <file>  Specify an input AVI file to get image data from\n"
" -o <file>  Specify and output AVI file to write (processed) image data to\n"
" -ll x y    Specify pixel coordinate of lower left corner of sample region\n"
" -lr x y    Specify pixel coordinate of lower right corner of sample region\n"
" -ul x y    Specify pixel coordinate of upper left corner of sample region\n"
" -ul x y    Specify pixel coordinate of upper right corner of sample region\n"
" -debug     Set debug mode\n"
" -draw      Set draw mode\n"
" -help      Print this message\n"
" -?         Print this message\n";

void callback(IplImage* image);

/**
 * This test program reads an AVI file, grabs each frame, and processes it with
 * the road follower.  The processing done by the road follower is shown graphically
 * superimposed on each frame and written to an output AVI.
 *
 */
int main(int argc, char *argv[])
{
    // process command line arguments
    for (int i=0; i<argc; i++) {
        cout << argv[i] << endl;
    }
    // instantiate road follower classes
    imgSampler = new ImageSampler();
    ralph = new RALPHFollower();
    ralph->setDrawMode(4);
    kmeans = new KMeans(6);
    kmeans->setDebug(1);

    // create a window to display the original view
    cvNamedWindow("original", CV_WINDOW_AUTOSIZE);

    // create a window to display the processed view
    cvNamedWindow("processed", 0);

    // blow up the processed view so it's easier to see
    cvResizeWindow("processed", 320, 300);

    // create a handle to access the original window
    HWND MyWin = (HWND)cvGetWindowHandle("original");

    /* road.avi  */
    imgSampler->setLowerLeft(0, 80);
    imgSampler->setLowerRight(319, 80);
    imgSampler->setUpperLeft(125, 115);
    imgSampler->setUpperRight(205, 115);
    /* road2.avi */
    //imgSampler->setLowerLeft(5, 90);
    //imgSampler->setLowerRight(310, 90);
    //imgSampler->setUpperLeft(100, 130);
    //imgSampler->setUpperRight(200, 130);

    /*
    imgSampler->setLowerLeft(20, 130);
    imgSampler->setLowerRight(310, 130);
    imgSampler->setUpperLeft(100, 150);
    imgSampler->setUpperRight(250, 150);
    */
    //cvcamPlayAVI(0, 0, 0, 0, 0);

    // create a capture object to read the AVI
    CvCapture* cvc = cvCaptureFromAVI("c:\\temp\\dirt_curves1.avi");
    CvSize size;
    size.width = 320;
    size.height = 240;

    // create a CvAVIWriter object to write a new AVI
    CvAVIWriter *writer = cvCreateAVIWriter(
            "c:\\temp\\out.avi",
//           CV_FOURCC('M','J','P','G'), 
           -1, 
           15, size);

    // loop over each frame of the AVI
    while (cvGrabFrame(cvc) && cvc) {

      // get the image for the current frame
      IplImage* img = cvRetrieveFrame(cvc);
      if (img == 0) {
        // end of AVI
        break;
      }

      // process the image
      callback(img);

      // display the image
      cvShowImage("original", img);

      // write the image to a file
      cvWriteToAVI(writer, img);

      // add a delay so this program does not suck up all the CPU
      cvWaitKey(50);
    }

    // clean up
    cvReleaseCapture(&cvc);
    cvReleaseAVIWriter(&writer);
    cvDestroyWindow("original");
    cvDestroyWindow("processed");
	return 0;
}
void callback(IplImage* image)
{
    // create a pointer to the input image
    IplImage* image1 = image;

    // make sure we've got something
    assert (image);

    // sample the image, perform inverse perspective mapping, and reduce image to 32x30
    const IplImage *image2 = imgSampler->sample(image1);

    // analyze the sampled image with the RALPH road follower
    IplImage *image3 = (IplImage *)image2;
    double score;
    double angle = ralph->analyze(image3, &score);

    //kmeans->cluster(image3);

    // show analytical info, like the angle, on the original image
    char msg[100];
    sprintf(msg, "Curve: %2.1f", angle);
    CvPoint pt;
    pt.x = 5;
    pt.y = 20;
    CvFont font;
    cvInitFont(&font, CV_FONT_VECTOR0, 0.5, 0.5, 0.0, 1);
    cvPutText(image, msg, pt, &font, CV_RGB(255, 255, 255));
    sprintf(msg, "Score: %3.3f", score);
    pt.x = 5;
    pt.y = 40;
    cvPutText(image, msg, pt, &font, CV_RGB(255, 255, 255));

    // draw a line showing the direction in which to steer

    // this is the starting point of the line
    CvPoint pt1;
    pt1.x = 160;
    pt1.y = 0;

    // this is the ending point
    CvPoint pt2;
    pt2.x = 160 + (int)(200.0*sin(angle*PI/180));
    pt2.y = (int)(200.0*cos(angle*PI/180));

    // draw the line
    cvLine(image, pt1, pt2, CV_RGB(255, 255, 255), 2, 8);

    //kmeans->getColorAssignments();

    // this embeds the sampled image inside the original image in the lower right corner
    for (int j=0; j<image2->height; j++) {
        uchar *ptr1 = (uchar*)((image1)->imageData + (image1)->widthStep*(j+5));
        uchar *ptr2 = (uchar*)((image2)->imageData + 
                (image2)->widthStep*(image2->height - j - 1));
        for (int i=0; i<image2->width; i++) {
            int index2 = 3*i;
            int index1 = 3*(i+250);
            if (j==0 || j==image2->height - 1 || i==0 || i==image2->width - 1) {
                // draw a blue bounding box around the sampled image
                (ptr1)[index1] = (char)255;
                (ptr1)[index1+1] = 0;
                (ptr1)[index1+2] = 0;
            }
            else {
                // replace pixels in the original image with the sampled image
                (ptr1)[index1] = (ptr2)[index2];
                (ptr1)[index1+1] = (ptr2)[index2+1];
                (ptr1)[index1+2] = (ptr2)[index2+2];
            }

        }
    }

    // show the processed image in it's own window
    cvShowImage("processed", image2);    

}

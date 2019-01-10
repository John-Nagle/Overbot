#include "roadfollower.h"
#include "roadfollowerdata.h"
#include "roadfollowerport.h"
#include <iostream.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
/**
 * High level interface to the road follower.
 *
 * Processes image frames, returns results.
 * 
 */

#include "ralphfollower.h"
#include "offroadfollower.h"
#include "imagesampler.h"


const int k_textcolor = CV_RGB(0,0,255);										// text in blue, for visibility

/**
 * Constructor.
 *
 */
RoadFollower::RoadFollower() :
    m_display_image(0), m_angle(0.0), m_score(0.0), m_offset(0.0),
    m_threshold(0.0)
,m_debug(0), m_display(0),m_ralph(0), m_offroad(0), m_sampler(0), m_counter(0)
    
{
    m_ralph = new RALPHFollower();
    m_offroad = new OffRoadFollower();
    m_sampler = new ImageSampler();
}

/**
 * Destructor.
 */
RoadFollower::~RoadFollower() {
    delete m_ralph;
    delete m_offroad;
    delete m_sampler;
    if (m_display_image) {
        cvReleaseImage(&m_display_image);
    }
}

/**
 * Set debug level.
 *
 * 0 is no debug output (default).
 *
 * @param  d  Set the debug level.
 */
void RoadFollower::setDebug(int d) {
    m_debug = d;
    m_ralph->setDebug(m_debug);
    m_offroad->setDebug(m_debug);
}

/**
 * Set display results mode.  Results from the road follower are graphically
 * displayed by superimposing graphics and information over the input image.
 *
 * 0 is no display node (default).
 *
 * @param  d  Set the display mode.
 */
void RoadFollower::setDisplayResults(int d) {
    m_display = d;
    m_sampler->setDisplayMode(d);
    if (d) {
        // select the middle curvature hypothesis under the assumption that
        // this is the one that has no shifts in the input image
        // for example if there are 7 hypotheses we want the 4th one (index=dmode=3)
        double half = NUM_CURVATURE_HYPOTHESES / 2.0;
        double fraction = half - (int)half;
        int dmode = (int)half;
        if (fraction >= 0.5) {
            dmode += 1;
        }
        // pass in the index of the curvature hypothesis image we want to display
        // you can set this value to be the index of any of the curvature hypothesis
        // which will cause the corresponding shifted image to appear in the overlay
        // display image
        m_ralph->setDrawMode(dmode);
        m_offroad->setDrawMode(dmode);
    }
}

/**
 * Get the angle of the road curvature from the current frame.
 *
 * @return  An angle (in degrees) with 0 being dead ahead.
 */
double RoadFollower::getAngle() {
    return m_angle;
}

/**
 * Get a score that measures the confidence that we're looking
 * at a road in the current frame.  Negative angle is to the left,
 * positive angle to the right.  This return value should give some
 * indication of how much the vehicle needs to steer to remain on
 * the road.
 *
 * @return  score  Use the value to establish a threshold.
 */
double RoadFollower::getScore() {
    return m_score;
}

/**
 * The relative lateral offset of the road with respect to the camera's
 * line of sight.
 * A negative offset means the center of the road is to the left of 
 * the camera and a positive offset means the center of the road is to
 * the right.  Zero means we're in the center of the road.
 * TODO: provide mechanism to calibrate this value to some useful units.
 *
 * @return  offset  Offset value.
 */
double RoadFollower::getOffset() {
    return m_offset;
}

/**
 * Define the upper right corner of the trapezoid defining the region
 * of the original image to sample for analysis of road features.
 *
 * @param  x  The horizontal axis coordinate.
 * @param  y  The vertical axis coordinate.
 */
void RoadFollower::setUpperRight(int x, int y) {
    m_sampler->setUpperRight(x, y);
}

/**
 * Define the upper left corner of the trapezoid defining the region
 * of the original image to sample for analysis of road features.
 *
 * @param  x  The horizontal axis coordinate.
 * @param  y  The vertical axis coordinate.
 */
void RoadFollower::setUpperLeft(int x, int y) {
    m_sampler->setUpperLeft(x, y);
}

/**
 * Define the lower right corner of the trapezoid defining the region
 * of the original image to sample for analysis of road features.
 *
 * @param  x  The horizontal axis coordinate.
 * @param  y  The vertical axis coordinate.
 */
void RoadFollower::setLowerRight(int x, int y) {
    m_sampler->setLowerRight(x, y);
}

/**
 * Define the lower left corner of the trapezoid defining the region
 * of the original image to sample for analysis of road features.
 *
 * @param  x  The horizontal axis coordinate.
 * @param  y  The vertical axis coordinate.
 */
void RoadFollower::setLowerLeft(int x, int y) {
    m_sampler->setLowerLeft(x, y);
}

/**
 * Set a confidence threshold based on output score of the road follower
 * (as reported by getScore()).  It is assumed we have a very low confidence
 * of a result if the score is below this threshold.
 *
 * 0.0 is (default).
 *
 * @param  t  The confidence threshold for road follower score.
 */
void RoadFollower::setThreshold(double t) {
    m_threshold = t;
}

/**
 * Process a frame in the image.
 *
 * @param  image  The input image to be analyzed for road features.
 */
void RoadFollower::processFrame(IplImage* image) {
    int i;

    // increment the counter of frames processed
    m_counter++;
    if (m_debug) {
        cout << m_counter;
    }
    if (!image) {
        return;
    }
    assert (image);

    // create a pointer to the input image
    IplImage* image1 = image;
    int height = image1->height;
    int width = image1->width;

    // make sure we've got something

    // sample the image, perform inverse perspective mapping, and reduce image to 32x30
    m_sampler->sample(image1);

    // analyze the sampled image with the road follower algorithm
    //IplImage *image3 = (IplImage *)image2;
    double score;
    double offset;
    double angle;
    
    //angle = m_ralph->analyze(image3, &score, &offset);
    angle = m_offroad->analyze(m_sampler, &score, &offset);
    IplImage *debug_image = m_offroad->getDisplayImage();

    // set member variables
    m_angle = angle;
    m_score = score - m_threshold;                        // re-zero score so that >0 means road and <0 means no road
    m_offset = offset;

    //kmeans->cluster(image3);

    if (m_display) {
        // show analytical info, like the angle, on the original image
        //	All this font stuff is being done for EVERY FRAME - ineffiicient.  JN
        CvFont font;
        // adjust height of text font
        double xscale = 1.0;
        double yscale = 1.0;
        int textheight = 20;
        for (i=0; i<10; i++) {
            yscale -= 0.1;
            CvSize textsize;
            int ymin;
            // find a text scale that fits in the lower quadrant
            cvInitFont(&font, CV_FONT_VECTOR0, xscale, yscale, 0.0, 1);

            // test the font
            cvGetTextSize("Score: 123.123", &font, &textsize, &ymin);
            if (2*textsize.height < height/10) {
                // keep height of all text to less than 1/10 height of image
                textheight = textsize.height;
                break;
            }

        }
        // adjust width of text font
        for (i=0; i<10; i++) {
            xscale -= 0.1;
            CvSize textsize;
            int ymin;
            // find a text scale that fits in the lower quadrant
            cvInitFont(&font, CV_FONT_VECTOR0, xscale, yscale, 0.0, 1);

            // test the font
            cvGetTextSize("Score: 123.123", &font, &textsize, &ymin);
            // keep width of text to less than 1/2 width of image
            if (textsize.width < width/2) {
                break;
            }

        }
#ifdef OBSOLETE
        // show the curve value
        sprintf(msg, "Curve: %2.1f", angle);
        CvPoint pt;
        pt.x = 5;
        pt.y = textheight;
        cvPutText(image, msg, pt, &font, k_textcolor);

        // show the score value
        sprintf(msg, "Score: %3.3f", score);
        pt.x = 5;
        pt.y = 2*textheight;
        cvPutText(image, msg, pt, &font, k_textcolor);

        // show the offset value
        sprintf(msg,"Offset: %3.3f", offset);
        pt.x = 5;
        pt.y = 3*textheight;
        cvPutText(image, msg, pt, &font, k_textcolor);
#endif // OBSOLETE

		//	Show the road follower outputs
		{	
        	CvPoint pt;
        	char msg[100];
			snprintf(msg,sizeof(msg), "%1.3f 1/m  %4.2f",angle, offset);
       		pt.x = 5;
        	pt.y = int((3.9)*textheight);
		// allow some leading
        	cvPutText(image, msg, pt, &font, k_textcolor);
        }
		{	
        	CvPoint pt;
        	char msg[100];
			snprintf(msg,sizeof(msg), "Score: %6.3f", score);
       		pt.x = 5;
        	pt.y = int((2.6)*textheight);
		// allow some leading
        	cvPutText(image, msg, pt, &font, k_textcolor);
        }
        // show the frame counter and time
        {	time_t nowt;
			time(&nowt);       	
			struct tm* now = localtime(&nowt);	// get time now
       		char msg[100];
	       snprintf(msg, sizeof(msg),"%02d:%02d:%02d (%d)", now->tm_hour, now->tm_min, now->tm_sec,m_counter);
        	CvPoint pt;
 	       pt.x = 5;
        	pt.y = int((1.3)*textheight);
		// allow some leading
   		   cvPutText(image, msg, pt, &font, k_textcolor);

		}
        // draw a line showing the direction in which to steer
        if (m_score > 0.0) {
            // this is the starting point of the line
            CvPoint pt1;
            pt1.x = width/2;
            pt1.y = 0;

            // this is the ending point
            CvPoint pt2;

            // approx steering angle, 5.0m from vehicle center to image
            // and 'angle' is really curvature here
            double sinangle = 5.0*angle;
            double cosangle = sqrt(1 - sinangle*sinangle);
            pt2.x = width/2 + (int)(height*sinangle);
            pt2.y = (int)(height*cosangle);
    
            // draw a white line 2 pixels thick
            cvLine(image, pt1, pt2, CV_RGB(255, 255, 255), 2);
        }
        else {
            // draw a red X meaning not a road
            CvPoint pt1;
            pt1.x = (int)(width*0.25);
            pt1.y = (int)(height*0.25);
            CvPoint pt2;
            pt2.x = (int)(width*0.75);
            pt2.y = (int)(height*0.75);
    
            // draw a white line 2 pixels thick
            cvLine(image, pt1, pt2, CV_RGB(255, 0, 0), 2);
            CvPoint pt3;
            pt3.x = (int)(width*0.75);
            pt3.y = (int)(height*0.25);
            CvPoint pt4;
            pt4.x = (int)(width*0.25);
            pt4.y = (int)(height*0.75);
    
            // draw a white line 2 pixels thick
            cvLine(image, pt3, pt4, CV_RGB(255, 0, 0), 2);

        }
        //kmeans->getColorAssignments();

        // this embeds the sampled image inside the original image in the lower right corner

        // rescale sampled image based on dimensions of original image to insert into
        double scale_factor = 1;
        for (i=1; i<5; i++) {
            if (SAMPLE_HEIGHT * i > height/2) {
                // make sure we fit in a quadrant
                break;
            }
            if (SAMPLE_WIDTH * i > width/2) {
                // make sure we fit in a quadrant
                break;
            }
            scale_factor = i;
        }
        
        // allocate space for the display of the sampled image if dimensions have changed
        int new_height = (int)(SAMPLE_HEIGHT * scale_factor);
        int new_width = (int)(SAMPLE_WIDTH * scale_factor);
        if (!m_display_image ||
            new_height != m_display_image->height ||
            new_width != m_display_image->width) {
            CvSize csize;
            csize.width = new_width;
            csize.height = new_height;
            if (m_display_image) {
                cvReleaseImage(&m_display_image);                
            }
            m_display_image = cvCreateImage(csize, IPL_DEPTH_8U, 3 );
        }
        // resize the sampled image into the display image
        cvResize(debug_image, m_display_image);

        // i.e. it copies pixels from image2 to image1
        for (int j=0; j<m_display_image->height; j++) {
            // get a pointer to the pixel row in the sampled image to be copied
            uchar *ptr2 = (uchar*)((m_display_image)->imageData + 
                (m_display_image)->widthStep*(m_display_image->height - j - 1));

            // get a pointer to pixel row in the original image to copy pixels to
            // we start 5 pixles above the bottom
            uchar *ptr1 = (uchar*)((image1)->imageData + (image1)->widthStep*(j+5));

            // loop over columns in the sampled image
            for (int i=0; i<m_display_image->width; i++) {

                // get the column in the sampled image
                int index2 = 3*i;

                // get the row in the original image from the starting point
                // center it in the lower right quadrant
                int colstart1 = (int)(width*0.75 - m_display_image->width/2.0);
                int index1 = 3*(i+colstart1);
                if (j==0 || 
                    j==m_display_image->height - 1 || 
                    i==0 || 
                    i==m_display_image->width - 1) {
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
    }

}


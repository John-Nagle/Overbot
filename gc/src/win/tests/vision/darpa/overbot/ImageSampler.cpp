#include "ImageSampler.h"
#include "SampleIterator.h"
#include <iostream.h>
#include "Data.h"

/**
 * This class take in input image (presumably larger) and produces a sub-sampled image
 * (presumabely smaller) and stores it in an image buffer.  The sub-sampled image is
 * created by drawing pixels based on a trapezoid that defines the perspective geometry and
 * regions of interest in the input image.  Therefore the sub-sampled image should produce
 * an "over-head" view with perspective removed from the image (i.e. performs an
 * inverse perspective transformation).  The individual pixel values in the sub-sampled image
 * are determined by block averaging of 2x2 blocks in the input image to reduce the effect
 * of noisy pixels in the input video.  (This is based on the results report in the paper:
 * Crisman, J.D., Thorpe, C.E., "SCARF: A Color Vision System That Tracks Roads and Intersections"
 *
 * The region within the trapezoid is mapped to the region in the sub-sampled image
 * with dimensions SAMPLE_HEIGHT x SAMPLE_WIDTH, and the coordinate system has its origin
 * at the lower left corner of this rectangle.  In addition we map pixels from the left and
 * right of the trapezoid into cooresponding buffer-zones to the left and right of the
 * sub-sampled image.  This is to accomodate shifts needed to test our curvature hypotheses.
 * If the shifts need data beyond what is available in the original input image, we use the
 * pixel value at the extreme edges of the original input image.
 */
// define block of pixels to average to determine sampled pixel
#define X_AVERAGE 2
#define Y_AVERAGE 2

/**
 * Constructor.
 */
ImageSampler::ImageSampler() :
    m_x1(0), m_x2(0), m_x3(0), m_x4(0), m_y1(0), m_y2(0), m_y3(0), m_y4(0),
    m_height(SAMPLE_HEIGHT), m_width(SAMPLE_WIDTH),
    m_image(0), m_display(0)
{
    // find the extreme shifts from the curvature hypotheses
    m_min_shift = 0; // a negative number, represents how many extra pixels we need on the left
                     // side of the sub-sampled image to accomodate shifts to the right
    m_max_shift = 0; // a positive number, represents how many extra pixels we need on the right
                     // side of the sub-sampled image to accomodate shifts to the left.

    // go through all curvature hypotheses and determine the extreme amount of left, right shifts
    for (int i=0; i<NUM_CURVATURE_HYPOTHESES; i++) {
        for (int j=0; j<SAMPLE_HEIGHT; j++) {
            int shift = shift_matrix[i][j];
            if (shift < m_min_shift) {
                m_min_shift = shift;
            }
            if (shift > m_max_shift) {
                m_max_shift = shift;
            }
            
        }
    }

    // allocate enough space in the image buffer to store the sampled image, plus
    // room to accomodate the curvature hypotheses shifts
    CvSize csize;
    // image buffer width includes room at either side to store pixels need to compute shifts
    csize.width = m_width + m_max_shift - m_min_shift;
    csize.height = m_height;
    m_image = cvCreateImage(csize, IPL_DEPTH_8U, 3 );
}

/**
 * Destructor.
 */
ImageSampler::~ImageSampler() {
    if (m_image) {
        cvReleaseImage(&m_image);
    }
}

/**
 * Get the height of the sub-sampled image
 */
int ImageSampler::getSampleHeight() {
    return m_height; 
}

/**
 * Get the width of the sub-sampled image (not including buffer-zones to left and right)
 */
int ImageSampler::getSampleWidth() {
    return m_width;
}

/**
 * Set the lower right coordinates of the trapezoid
 * @param  x  The x coord.
 * @param  y  The y coord.
 */
void ImageSampler::setLowerRight(int x, int y) {
   m_x2 = x;
   m_y2 = m_y1 = y;
}

/**
 * Set the lower left coordinates of the trapezoid
 * @param  x  The x coord.
 * @param  y  The y coord.
 */
void ImageSampler::setLowerLeft(int x, int y) {
    m_x1 = x;
    m_y1 = m_y2 = y;
}

/**
 * Set the upper right coordinates of the trapezoid
 * @param  x  The x coord.
 * @param  y  The y coord.
 */
void ImageSampler::setUpperRight(int x, int y) {
    m_x3 = x;
    m_y3 = m_y4 = y;
}

/**
 * Set the upper left coordinates of the trapezoid
 * @param  x  The x coord.
 * @param  y  The y coord.
 */
void ImageSampler::setUpperLeft(int x, int y) {
    m_x4 = x;
    m_y4 = m_y3 = y;
}

/**
 * Sample an input image and derive a sub-sampled image according to the
 * current dimensions of the trapezoid.
 * @param  img  A pointer to the input image (i.e. a frame of video)
 * @return  A pointer to the sub-sampled image (you shouldn't use this)
 */  
const IplImage *ImageSampler::sample(IplImage *img) {
    int x, y;

    // slope of the left boundary of trapezoid
    double m1 = (double)(m_x1 - m_x4)/(m_y1 - m_y4);

    // slope of the right boundary of trapezoid
    double m2 = (double)(m_x2 - m_x3)/(m_y2 - m_y3);

    // iterator to sample the y axis (evenly maps pixel rows in the trapezoid to sub-sampled) 
    SampleIterator y_iterator(m_y4 - m_y1, m_height);

    // sample the y-axis
    int yy;
    for (yy=0; yy<m_height; yy++) {
        // get y coordinate of row to sample in the original image
        y = m_y4 - y_iterator.next();

        // pointer to row of pixels in original image
        uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*y);
        // pointer to row of pixels in the sub-sampled image buffer
        uchar *ptr2 = (uchar *)(m_image->imageData + m_image->widthStep*yy);

        // compute x boundaries of trapezoid
        int x_left = (int)((y - m_y1)*m1) + m_x1; // x-coord of left side of trapezoid
        int x_right= (int)((y - m_y2)*m2) + m_x2; // x-coord of right side of trapezoid


        // iterator to sample pixels in the current row (evenly maps pixel columns
        // in the trapezoid to the sub-sampled image)
        SampleIterator x_iterator(x_right - x_left, m_width);

        // extreme leftmost x-coord to sample from in original image
        // this is where we get data to shift pixels to the right to test hypostheses
        double scale_factor = (double)(x_right - x_left)/m_width;
        int x_min = (int)(m_min_shift * scale_factor) + x_left;

        // sample the x-axis
        int xx;
        for (xx=0; xx<m_image->width; xx++) {
            // get the offset from the left boundary of the pixel to sample
            int xsample = x_iterator.next() + x_min;
            if (xsample < 0) {
                // take pixel from edge if we've gone out of bounds
                xsample = 0;
            }
            if (xsample >= img->width) {
                // take pixel from edge if we've gone out of bounds
                xsample = img->width - 1;                
            }

            // at this point the coordintes of the pixel we want to sample
            // in the original image is (xsample, yy)


            // make sure we're within the bounds of the images
            if (xx < m_image->width && yy < m_image->height && xsample < img->width) {
                unsigned index = 3*xsample;
                unsigned index2 = 3 * xx;

                // get the pixel value by calling the function that does block average
                // based on nearby pixels
                unsigned char red;
                unsigned char green;
                unsigned char blue;
                getAveragePixel(img, xsample, y, &red, &green, &blue);
                // copy the pixel from the original to the new image
                /*
                (ptr2)[index2] = (ptr)[index];
                (ptr2)[index2+1] = (ptr)[index+1];
                (ptr2)[index2+2] = (ptr)[index+2];
                */
                (ptr2)[index2] = blue;
                (ptr2)[index2+1] = green;
                (ptr2)[index2+2] = red;
                   
    
                // draw a white spot at the sample pixel in original image
                /* 
                (ptr)[index] = (unsigned char)255;
                (ptr)[index+1] = (unsigned char)255;
                (ptr)[index+2] = (unsigned char)255;
                */
            }
        }
    }

    // display boundaries of the trapezoid in the original image
    if (m_display) {
        for (y=m_y1; y<m_y4 && y<(img)->height; y++) {
            // pointer to row of pixels in original image
            uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*y);

            // compute x boundaries of sample window
            int x_left = (int)((y - m_y1)*m1) + m_x1;
            int x_right = (int)((y - m_y2)*m2) + m_x2;
            /*
            double scale_factor = (double)(x_left - x_right)/m_width;
            int x_min = (int)(m_min_shift * scale_factor) + x_left;
            int x_max = (int)(m_max_shift * scale_factor) + x_right;
            if (x_min < 0) {
                x_min = 0;
            }
            if (x_max >= img->width) {
                x_max = img->width - 1;
            }

            // show extremes of sampled pixels
            if (x == x_min || x == x_max) {
                
            }
            */

            // draw boundary box of the trapezoid in the original image
            for (x = x_left; x < x_right && x<(img)->width; x++) {
                //cout << "sample x=" << x << " y=" << y << endl;
                unsigned index = 3*x;

                // check if we're on the boundary
                if (y==m_y1 || y==m_y4-1 || x == x_left || x == x_right - 1) {
    
                    // make the pixel blue
                    (ptr)[index] = (char)255;
                    (ptr)[index+1] = 0;
                    (ptr)[index+2] = 0;
                }
                else {
                    // leave interior points alone
                    continue;
                }    
            }

        }
    }
    
    return m_image;
}

/**
 * Toggle display mode which draws trapezoid back into the original image
 * 
 * @param  d  Flag is non-zero to turn on display mode
 */
void ImageSampler::setDisplayMode(int d) {
    m_display = d;
}

/**
 * Get the pixel values for a given coordinate in the sub-sampled image.
 *
 * @param  x  The x-ccord (can be negative which means get a pixel from the left buffer zone)
 * @param  y  The y-coord
 * @param  *r  Return the red value
 * @param  *g  Return the green value
 * @param  *b  Return the blue value
 */
void ImageSampler::getSamplePixel(int x, int y, 
                                 unsigned char *r, 
                                 unsigned char *g, 
                                 unsigned char *b) {

    // transform the user coordinate (with original at left edge of sub-sampled region
    // but not including the left buffer zone...which is represented by negative values)
    // into the real image coordinates with origin at extreme left edge.
    int xvalue = x - m_min_shift;

    if (xvalue < 0) {
        // report value from the left edge if we're going out of bounds
        xvalue = 0;
    }
    if (x >= m_image->width) {
        // report the value from the right edge if we're out of bounds
        xvalue = m_image->width - 1;
    }

    // get a pixel from open CV image
    uchar *ptr = (uchar*)((m_image)->imageData + (m_image)->widthStep*y);
    unsigned index = 3*xvalue;

    // return the pixel values
    *b = (ptr)[index];
    *g = (ptr)[index+1];
    *r = (ptr)[index+2];

}

/**
 * Private method to derive a pixel value for a coordinate in the original image
 * based on block averaging nearby pixels
 * 
 * @param  img  The pointer to the original image
 * @param  x  The x-coord in the original image
 * @param  y  The y-coord in the original image
 * @param  *r Return the red value
 * @param  *g Return the green value
 * @param  *b Return the blue value
 */
void ImageSampler::getAveragePixel(IplImage *img, int x, int y, 
                                 unsigned char *r, 
                                 unsigned char *g, 
                                 unsigned char *b) {
    double red=0;
    double green=0;
    double blue=0;
    int counter = 0;

    // loop over the y-coords
    for (int j=y; j<y+Y_AVERAGE; j++) {

        if (j<0 || j>=img->height) {
            continue;
        }

        uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*j);

        // loop over the x-coords
        for (int i=x; i<x+X_AVERAGE; i++) {
            if (i<0 || i>=img->width) {
                continue;
            }
            unsigned index = 3*i;

            // acculate sum of each pixel in the block
            blue += (ptr)[index];
            green += (ptr)[index+1];
            red += (ptr)[index+2];
            counter++;
        }
    }

    // average the pixels
    if (counter) {
        blue /= counter;
        red /= counter;
        green /= counter;
    }

    // return the average pixel value
    *b = (unsigned char)blue;
    *g = (unsigned char)green;
    *r = (unsigned char)red;

}
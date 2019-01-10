#include "RALPHFollower.h"
#include "Data.h"
#include "Port.h"
#include <iostream.h>
#include <math.h>

/**
 * Road following algorithm as described in the paper:
 *
 *
 * 
 * Measures the amount of vertical linear features in an image.  The first step is
 * summing the pixel intensities of each column [call it x]
 * in the image to compute a "scanline intensity" function [call it v(x)]. 
 * We then compute the differences in scanline intensity between adjacent columns.  
 * To quantify the amount of strong vertical features in the image we compute a histogram of
 * the differences, and sum the largest differences to arrive at the final value.  This
 * procedure measures the amount of discontinuity between adjacent columns.  More discontinuity
 * is an indicator of more vertical linear features.  
 * 
 * The road follower applies a fixed set of curvature hypothesis which attempt to
 * corrent and "straighten" out the road curvature.  We simply apply each curvature
 * hypothesis and use the above procedure to measure the amount of vertical features
 * the straighened image.  The hypothesis that does the best job of straightening the
 * image is assumed to best characterize the type of curvature in the road.
 *
 *
 *
 */


// the maximum value that a column can have in the scanline intensity profile
#define MAX_SCANLINE_VALUE 255 * SAMPLE_HEIGHT


// stores values of relative differences between adjacent columns in scanline
// intensity profile
struct intensityRecord {
    double value;
    int position;
};
// use to pixels
struct pixelValue {
    unsigned char r;    // the Red value
    unsigned char g;    // the Green value
    unsigned char b;    // the Blue value 
};

/*
 * a static function passed into qsort to sort 
 * positive floating point numbers in descending order
 */
static int compare_values(const void *a, const void *b) {
    const intensityRecord *ia = (const intensityRecord *)a;
    const intensityRecord *ib = (const intensityRecord *)b;
    return (int)(ib->value - ia->value);
}




/**
 * Constructor.
 *
 */
RALPHFollower::RALPHFollower() :
    m_height(SAMPLE_HEIGHT), m_width(SAMPLE_WIDTH),
    m_rowBuffer(0), m_intensities(0), m_colBuffer(0),
    m_debug(0), m_draw(0)
{
    m_rowBuffer = new double[m_width];
    m_intensities = new intensityRecord[m_width];
    m_colBuffer = new pixelValue[m_height];
}

/**
 * Destructor.
 */
RALPHFollower::~RALPHFollower() {
    delete m_rowBuffer;
    delete m_intensities;
    delete m_colBuffer;
}

/**
 * Set debug level.
 *
 * 0 is no debug output (default).
 *
 * @param  d  Set the debug level.
 */
void RALPHFollower::setDebug(int d) {
    m_debug = d;
}

/**
 * Set draw mode which draws an graph of the scanline intensity profile
 * on the input image as a blue line.  Also draws the lateral offset of the road
 * as a vertical red line.
 *
 * @param  d  Set non-zero to turn on draw mode.
 */
void RALPHFollower::setDrawMode(int d) {
    m_draw = d;
}
/**
 * Analyze an image to determine road curvature.
 * @param  img  An image...size must be WIDTH x HEIGHT
 * @param  score  Return value for overall confidence that we're looking at a road
 * @param  offset  Return value for the lateral offset of the center of the road
 * @return  The amount of road curvature (in degrees).
 */
double RALPHFollower::analyze(IplImage *img, double *score, double *offset) {
    int i;
    double max = 0.0;
    double normalize = 1.0;
    int center = 0;
    int imax = 0;

    // if image size has changed we need to resize our storage
    if (img->width != m_width || 
        img->height != m_height) {
        MYERROR("Error in RALPHFollower::analyze: bad image size");
    }

    // compute a scanline intensity for each hypothesis
    for (i=0; i<NUM_CURVATURE_HYPOTHESES; i++) {
        double n;
        int c;
        if (m_debug) {
            cout << "\t" << i;
        }
        double value = scanlineIntensity(img, i, &n, &c);
        if (m_debug) {
            cout << "\t" << value;
        }
        // keep track of which curvature hypothesis is best
        if (value > max) {
            max = value;
            normalize = n;
            center = c;
            imax = i;
        }
    }

    if (m_draw && center)
    {
        // draw a red line up the image at the center of the road
        unsigned index = 3*center;
        double test = m_height - 
                    (double)m_rowBuffer[i]*m_height/(double)(MAX_SCANLINE_VALUE);
        for (int j=0; j<m_height; j++) {
            uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*j);
            (ptr)[index] = 0;
            (ptr)[index+1] = 0;
            (ptr)[index+2] = 255;
        }

    }
    // base a confidence value on the max scanline intensity
    if (score) {
        // normalize the max return value 
        *score = max / (NUM_SCANLINE_DIFFERENCES * MAX_SCANLINE_VALUE);
    }
    if (offset) {
        // TODO: add a constant factor to convert this to meaningful units (i.e. meters)
        *offset = (double)(center - m_width/2.0);
    }
    // report angle of curvature for best hypothesis
    return road_curvature[imax];
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Apply a curvature hypothesis to an image and measure the scanline intensity of the
 * resulting "straightened" image.
 * @param  img  The input image...size must be WIDTH x HEIGHT
 * @param  s  The curvature hyposthesis to apply.
 * @param  area  A pointer to return the area under the scanline intensity graph.
 * @param  icenter  A pointer to return the position of the center of the road
 * @return  The resulting amount of verticle features in the straightened image.  This number
 *          can be used to compare which curvature hypothesis did the best job of straightening
 *          out the image and hence is the best fit.
 * TODO: Add some error checking if we give a bogus value of s.
 * TODO: Add some error checking if we have an image that is the wrong size.
 */
double RALPHFollower::scanlineIntensity(IplImage *img, int s, double *area, int *icenter) {
    int i;
    int x, y;

    if (s<0 || s>=NUM_CURVATURE_HYPOTHESES) {
        MYERROR("Error in RALPHFollower::scanlineIntensity: bad s value");
    }
    if (m_height<2) {
        MYERROR("Error in RALPHFollower::scanlineIntensity: image height less than 2");
    }
    // loop over pixel columns in the image
    for (i=0; i<m_width; i++) {

        // initialize the sum of pixel intensities to zero
        m_rowBuffer[i] = 0;
       
        // loop over the pixels (rows) in this column
        for (y=0; y<m_height; y++) {

            // get a pointer to pixel column of pixels
            uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*y);

            // get the amount to shift (number of pixels) 
            // for this row in the image for the curvature hypothesis
            int delta = shift_matrix[s][m_height - y - 1];

            // instead of actually shifting pixels in the whole row,
            // we just grab the pixel from the
            // column that contains the value we want for this row.
            x = i + delta;

            // check for out of bounds, borrow from edges
            if (x < 0) {
                // grab pixel from leftmost column in the image
                x = 0;
            }
            if (x > img->width - 1) {
                // grab pixel from the rightmost column of the image
                x = img->width - 1;
            }

            // compute the index that gives the pixel at the desired row, column
            // note the 3 is assuming we're using the RBG color model and not grayscale
            // TODO: determine the color value from image object attributes
            unsigned index = 3*x;

            // get RGB values from the image
            unsigned char b = (ptr)[index];
            unsigned char g = (ptr)[index+1];
            unsigned char r = (ptr)[index+2];

            // store RGB values in the column buffer
            m_colBuffer[y].r = r;
            m_colBuffer[y].g = g;
            m_colBuffer[y].b = b;

        }

            
        // compute an intensity value for this column
        double value = 0.0;
        for (y=0; y<m_height; y++) {
            // grab values for this pixel
            unsigned char r = m_colBuffer[y].r;
            unsigned char g = m_colBuffer[y].g;
            unsigned char b = m_colBuffer[y].b;

            // use standard albeit imperfect transformation from RGB to greyscale
            //value += 0.3*r + 0.59*g + 0.11*b;

            // use a transformation that cuts the level of G which assumes that
            // road pixels usually are not very Green
            value += 0.4*r + 0.2*g + 0.4*b;

        }

        // base value on sum of squares of variances
        // value = MAX_SCANLINE_VALUE - (sigma_r + sigma_g + sigma_b);
        // save the intensity value for this column
        m_rowBuffer[i] = value;
        if (m_debug) {
            cout << "\t" << value;
        }
        /*
        if (m_draw && m_draw == s+1)
        {
            // draw the selected scanline intensity profile on the input image
            unsigned index = 3*i;
            double test = m_height - 
                    (double)m_rowBuffer[i]*m_height/(double)(MAX_SCANLINE_VALUE);
            uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*(int)test);
            (ptr)[index] = (char)255;
            (ptr)[index+1] = 0;
            (ptr)[index+2] = 0;

        }
        */
    }

    // compute the absolute difference between scanline intensity of
    // adjacent columns
    for (i=0; i<m_width-1; i++) {
        m_intensities[i].value = fabs(m_rowBuffer[i] - m_rowBuffer[i+1]);
        m_intensities[i].position = i;
    }

    // return the area under the scanline intesity profile
    double integrate = 0.0;
    for (i=0; i<m_width; i++) {
        integrate += m_rowBuffer[i];
    }
    if (area) {
        *area = integrate;
    }
    // sort the intensity values to create histogram
    qsort(m_intensities, m_width-1, sizeof(intensityRecord), compare_values);

    // sum absolute differences of top values & compute center of mass
    double sum = 0.0;
    double center = 0.0;
    for (i=0; i<NUM_SCANLINE_DIFFERENCES; i++) {
        //cout << s << "," << i << "=" << m_intensities[i] << endl;
        double value = m_intensities[i].value;
        int pos = m_intensities[i].position;
        //double value = m_rowBuffer[i];
        //int pos = i;
        sum += value;
        center += pos * value;
    }
    if (sum > 0.0 && icenter) {
        *icenter = (int)(center/sum);
    }
    //cout << endl;
    return sum;
}

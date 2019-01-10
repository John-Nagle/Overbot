#include "OffRoadFollower.h"
#include "ImageSampler.h"
#include "Data.h"
#include "Port.h"
#include <iostream.h>
#include <math.h>

/**
 * This is a variant on the RALPH Road following algorithm that is somewhat better
 * able to analyze dirt and desert road conditions.  The RALPH algorithm measures the
 * amount of contrast between adjacent vertical columns under the assumption that
 * the view of a straight road will have a large amount of contrast.  This assumption
 * is good for paved roads which have distinctive vertical features due to lane
 * markers, double yellow stripes, shoulders, curbs, and tire tread marks.  However
 * on dirt roads and desert roads the amount of sharp constrast is less.  Therefore this
 * new algorithm operates under the assumption that for non-paved roads the major feature
 * that distinguishes road from non-road regions is the amount of homogeneity in color
 * and brightness.  Road areas are generally homogenious in color and brightness, at
 * least compared to non-road areas which have more differences due to variations in color
 * of foliage, rocks, and natural terrain.  Like RALPH this algorithm assumes that an
 * image of a straight road will have more pronounced vertical features compared to
 * curved roads where features spill over into adjacent columns of the image because
 * they have more of a horizontal component.
 *
 * 
 * Measures the amount of vertical linear features in an image based on homogenity.  
 * The first step is
 * determining the mean R,G,B values for each vertical column of pixels in the image.
 * We then compute the variances of the R,G,B intensities in each vertical column.  
 * To quantify the amount of strong vertical features in the image we compute a histogram of
 * the variances, and sum the variances in each column to arrive at the final value.  This
 * procedure measures the amount of vertical color homogeneity.  We then transform
 * this aggregrate measure of variance into a homogeneity score.  More homogeneity
 * is an indicator of looking at a straighter road.  
 * 
 * The road follower applies a fixed set of curvature hypothesis which attempt to
 * corrent and "straighten" out the road curvature.  We simply apply each curvature
 * hypothesis and use the above procedure to measure the amount of vertical features
 * the straighened image.  The hypothesis that does the best job of straightening the
 * image is assumed to best characterize the type of curvature in the road.
 *
 * This algorithm has several advantages over the RALPH algorithm:
 * - It takes into account actual color values not just greyscale pixel intensity
 * - It attempts to identify regions of road vs non-road and therefore we can more
 *   easily determine the center of the road as well as a confidence measure of the
 *   amount of "roadness" in the image.
 * - It is more tolerant of rough dirt and desert roads which may not have clearly
 *   defined boundaries between the road and the surrounding area.
 */


// the maximum value that a column can have in the scanline intensity profile
#define MAX_SCANLINE_VALUE 1.0


// stores values of relative differences between adjacent columns in scanline
// intensity profile
struct intensityRecord {
    double value;
    int position;
};

// use to store pixels
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
OffRoadFollower::OffRoadFollower() :
    m_height(SAMPLE_HEIGHT), m_width(SAMPLE_WIDTH),
    m_rowBuffer(0), m_intensities(0), m_colBuffer(0),
    m_debug(0), m_draw(0)
{
    m_rowBuffer = new double[m_width];
    m_intensities = new intensityRecord[m_width];
    m_colBuffer = new pixelValue[m_height];
    CvSize csize;
    csize.width = m_width;
    csize.height = m_height;
    m_display_image = cvCreateImage(csize, IPL_DEPTH_8U, 3 );
}

/**
 * Destructor.
 */
OffRoadFollower::~OffRoadFollower() {
    delete m_rowBuffer;
    delete m_intensities;
    delete m_colBuffer;
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
void OffRoadFollower::setDebug(int d) {
    m_debug = d;
}

/**
 * Set draw mode which draws an graph of the scanline intensity profile
 * on the input image as a blue line.  Also draws the lateral offset of the road
 * as a vertical red line.
 *
 * @param  d  Set non-zero to turn on draw mode.
 */
void OffRoadFollower::setDrawMode(int d) {
    m_draw = d;
}
/**
 * Analyze an image to determine road curvature.
 * @param  sampler  An ImageSampler object to return pixels in the sub-sampled image
 * @param  score  Return value for overall confidence that we're looking at a road
 * @param  offset  Return value for the lateral offset of the center of the road
 * @return  The amount of road curvature (in degrees).
 */
double OffRoadFollower::analyze(ImageSampler *sampler, double *score, double *offset) {
    int i;
    double max = 0.0;
    double normalize = 1.0;
    int center = 0;
    int imax = 0;

    // if image size has changed we need to resize our storage
    if (sampler->getSampleWidth() != m_width || 
        sampler->getSampleHeight() != m_height) {
        MYERROR("Error in OffRoadFollower::analyze: bad image size");
    }


    // compute a scanline homogeneity intensity for each curvature hypothesis
    for (i=0; i<NUM_CURVATURE_HYPOTHESES; i++) {
        double n;
        int c;
        if (m_debug) {
            cout << "\t" << i;
        }

        // test a curvature hypothesis
        double value = scanlineIntensity(sampler, i, &n, &c);

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

    // draw some results into the internal display image
    if (m_draw) {
        // draw curves that represent the curvature hypothesis overlayed on the road image
        for (i=0; i<NUM_CURVATURE_HYPOTHESES; i++) {
            for (int y=0; y<m_height; y++) {
                uchar *ptr = (uchar*)((m_display_image)->imageData + 
                        (m_display_image)->widthStep*y);
                int x = m_width/2 + shift_matrix[i][m_height - y - 1];
                if (x >= 0 && x<m_width) {
                    unsigned index = 3*x;
                    if (i == imax) {
                        // color the winning hypothesis white
                        (ptr)[index] = 255;
                        (ptr)[index+1] = 255;
                        (ptr)[index+2] = 255;
                    }
                    else {
                        // color the other hypotheses blue
                        (ptr)[index] = 255;
                        (ptr)[index+1] = 0;
                        (ptr)[index+2] = 0;
                    }
                }
            }
        }
        if (center) {
            // draw a red line up the image at the center of the road
            unsigned index = 3*center;
            double test = m_height - 
                    (double)m_rowBuffer[i]*m_height/(double)(MAX_SCANLINE_VALUE);
            for (int j=0; j<m_height; j++) {
                uchar *ptr = (uchar*)((m_display_image)->imageData + 
                        (m_display_image)->widthStep*j);
                (ptr)[index] = 0;
                (ptr)[index+1] = 0;
                (ptr)[index+2] = 255;
            }
        }
    }
    // base a confidence value on the max scanline intensity
    if (score) {
        // normalize the max return value 
        *score = normalize;
    }
    if (offset) {
        // TODO: add a constant factor to convert this to meaningful units (i.e. meters)
        *offset = (double)(center - m_width/2.0);
    }
    // report angle of curvature for best hypothesis
    return road_curvature[imax];
}


/**
 * Get the current display image.  Used for debugging purposes.
 *
 * @return  A display image.
 */
IplImage *OffRoadFollower::getDisplayImage() {
    return m_display_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Apply a curvature hypothesis to an image and measure the scanline homogeneity of the
 * resulting "straightened" image.  NOTE: in the current implementation the value returned
 * by this method and value returned in the "area" parameter are the same.
 *
 * @param  img  An ImageSampler object to get pixels in the sub-sampled image
 * @param  s  The curvature hyposthesis to apply.
 * @param  area  A pointer to return the area under the scanline intensity graph.
 * @param  icenter  A pointer to return the position of the center of the road
 * @return  The resulting amount of verticle features in the straightened image.  This number
 *          can be used to compare which curvature hypothesis did the best job of straightening
 *          out the image and hence is the best fit.
 * TODO: Add some error checking if we give a bogus value of s.
 * TODO: Add some error checking if we have an image that is the wrong size.
 */
double OffRoadFollower::scanlineIntensity(ImageSampler *sampler, int s, double *area, int *icenter) {
    int i;
    int x, y;

    if (s<0 || s>=NUM_CURVATURE_HYPOTHESES) {
        MYERROR("Error in OffRoadFollower::scanlineIntensity: bad s value");
    }
    if (m_height<2) {
        MYERROR("Error in OffRoadFollower::scanlineIntensity: image height less than 2");
    }
    // loop over pixel columns in the image
    for (i=0; i<m_width; i++) {

        // initialize the sum of pixel intensities to zero
        m_rowBuffer[i] = 0;

        // initialize mean values for color value
        double mean_r = 0.0;
        double mean_g = 0.0;
        double mean_b = 0.0;
       
        // loop over the pixels (rows) in this column
        for (y=0; y<m_height; y++) {

            // get a pointer to pixel column of pixels
            //uchar *ptr = (uchar*)((img)->imageData + (img)->widthStep*y); // input image
            uchar *ptr2 = (uchar*)(m_display_image->imageData
                    + m_display_image->widthStep*y); // display image

            // get the amount to shift (number of pixels) 
            // for this row in the image for the curvature hypothesis
            int delta = shift_matrix[s][m_height - y - 1];

            // instead of actually shifting pixels in the whole row,
            // we just grab the pixel from the
            // column that contains the value we want for this row.
            x = i + delta;

            // compute the index that gives the pixel at the desired row, column
            // note the 3 is assuming we're using the RBG color model and not grayscale
            // TODO: determine the color value from image object attributes
            unsigned index = 3*x;
            unsigned index2 = 3*i;

            // get RGB values from the image
            unsigned char b;
            unsigned char g;
            unsigned char r;
            sampler->getSamplePixel(x, y, &r, &g, &b);

            if (m_draw && m_draw - 1 == s) {
                // copy the pixel into the display image if we're in display mode
                (ptr2)[index2] = b;
                (ptr2)[index2+1] = g;
                (ptr2)[index2+2] = r;
            }

            // accumulate the sum of each color value in the means
            mean_r += r;
            mean_g += g;
            mean_b += b;

            // store RGB values in the column buffer
            m_colBuffer[y].r = r;
            m_colBuffer[y].g = g;
            m_colBuffer[y].b = b;

        }

        // compute averages
        mean_r /= m_height;
        mean_g /= m_height;
        mean_b /= m_height;
            
        // compute an color variances for this column
        double value = 0.0;
        double sigma_r = 0.0;
        double sigma_g = 0.0;
        double sigma_b = 0.0;
        for (y=0; y<m_height; y++) {
            // grab values for this pixel
            unsigned char r = m_colBuffer[y].r;
            unsigned char g = m_colBuffer[y].g;
            unsigned char b = m_colBuffer[y].b;

            // compute variances from mean color values
            sigma_r += (r - mean_r)*(r - mean_r);
            sigma_g += (g - mean_g)*(g - mean_g);
            sigma_b += (b - mean_b)*(b - mean_b);

        }

        if (m_height > 1) {
            sigma_r /= m_height - 1;
            sigma_g /= m_height - 1;
            sigma_b /= m_height - 1;
        }

        // save the intensity value for this column
        // sum of variances is a good approx. measure of overall homogeneity
        // but neglects correlation between colors which would give cross terms
        value = (sigma_r + sigma_g + sigma_b);

        // convert intensity value based on variances to value between 1 and 0
        // where 1 means more homogeneity
        m_rowBuffer[i] = 1.0 / (1.0 + value);
        if (m_debug) {
            cout << "\t" << m_rowBuffer[i];
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


    // return the area under the scanline homogeneity profile
    double integrate = 0.0;
    for (i=0; i<m_width; i++) {
        integrate += m_rowBuffer[i];
    }
    if (area) {
        *area = integrate;
    }

    // compute center of mass of scanline homogeneity
    double sum = 0.0;
    double center = 0.0;
    for (i=0; i<m_width; i++) {
        //cout << s << "," << i << "=" << m_intensities[i] << endl;
        //double value = m_intensities[i].value;
        //int pos = m_intensities[i].position;
        double value = m_rowBuffer[i];
        int pos = i;
        sum += value;
        center += pos * value;
    }
    if (sum > 0.0 && icenter) {
        // return pixel column with the center of mass
        *icenter = (int)(center/sum);
    }

    return sum;
}

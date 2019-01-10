/**
 * This header contains constants and calibration data for the road
 * road follower algorithms.  The goal of this file is to collect all important
 * data values that will have to be tweaked to optimize performance of
 * the road follower.
 *
 */

const double PI=3.1415926535897932384626433832795;
/**
 * The road follower doesn't process the entire input image from the camera,
 * but rather it analyzes a reduced size image obtained by sub-sampling a 
 * suitable region of the originial input image.  The following constants
 * define the size of the sampled image.
 */

// fix the width (number of pixel columns) in the sub-sampled image
#define SAMPLE_WIDTH 32
// fix the height (number of pixel rows) in the sub-sampled image
#define SAMPLE_HEIGHT 30

/**
 * The Ralph road follower determines road curvature by matching the sub-sampled
 * image to a pre-defined set of curvature hypotheses.  The following constants
 * define the number of curvature hypotheses, the curvatures, and the output value
 * corresponding to the best curvature hypothesis.
 */
#define NUM_CURVATURE_HYPOTHESES 7 

/**
 * matrix of curvature hypothesis.
 * each row contains a curvature hypothesis.
 * each column contains the amount by which we want to shift each row of pixels in the image.
 * these values could be deduced by analyzing and graphing curves in the road coordinate system,
 * but the following set of values was created "by eye" in an ad hoc manner.
 * TODO: create a better set of curvature hypotheses
 */
const int shift_matrix[NUM_CURVATURE_HYPOTHESES][SAMPLE_HEIGHT] = {
    {-1, -2, -3, -4, -5, -6, -7, -8, -9, -10,
     -11, -12, -13, -14, -15, -16, -17, -18, -19, -20,
     -21, -22, -23, -24, -25, -26, -27, -28, -29, -30},
    {0, 0, 0, 0, 0, -1, -1, -1, -2, -2,
     -3, -3, -4, -5, -5, -6, -6, -7, -8, -9,
     -10, -11, -12, -13, -14, -16, -17, -19, -20, -21},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, -1, -1, -1, -1, -1, -1, -2, -2, -3,
     -3, -4, -4, -5, -5, -6, -6, -7, -8, -9},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 1, 1, 1, 1, 1, 1, 2, 2, 3,
     3, 4, 4, 5, 5, 6, 6, 7, 8, 9},
    {0, 0, 0, 0, 0, 1, 1, 1, 2, 2,
     3, 3, 4, 5, 5, 6, 6, 7, 8, 9,
     10, 11, 12, 13, 14, 16, 17, 19, 20, 21},
    {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
     11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
     21, 22, 23, 24, 25, 26, 27, 28, 29, 30}
};

/*
 * Static array containing values that quantify amount of curvature for each
 * hypothesis.  These values must correspond to the curvature hypotheses
 * defined above in shift_matrix.
 * The "angle" to steer the vehicle (in degrees) based on road curvature.  Zero is
 * straight ahead, negative is a left hand turn and positive is a right hand turn.
 * These values are more or less made up right now.  
 * TODO: Calibrate these values to give meaningful input to the steering actuator
 * based on road curvature.
 */
const double road_curvature[NUM_CURVATURE_HYPOTHESES] = {
    -60.0, -45.0, -20.0, 0.0, 20.0, 45.0, 60.0
};

/**
 * The Ralph road follower determines the best curvature hypothesis by 
 * computing a scan-line intensity profile and then computing the absolute
 * relative differences between adjacent scan-line intensities.  An aggregrate
 * value for the amount of variation in the scan-line intensity profile (which
 * provides a measure of the amount of vertical linear features in the image)
 * is obtained by summing a fixed number of the maximum differences.  The
 * following constant NUM_SCANLINE_DIFFERENCES defines the number of differences
 * to use for the sum.  This value must be less than SAMPLE_WIDTH-1
 */
#define NUM_SCANLINE_DIFFERENCES 6


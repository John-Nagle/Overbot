#include "sampleiterator.h"
/**
 * Simple utility class to sample points from one interval into another at
 * an even spacing.  
 * For example if you wanted to take 4 samples from the points between 1 and 10 this would
 * give you 1, 3, 6, 8 since that is the best way (according to this algorithm)
 * of evenly sampling 4 points taken from the interval [1,10].
 *
 * This is basically similar to the Bresenham Algorithm in one
 * dimension (you could use 2 instances of SampleIterator to draw 2-d
 * lines).
 */

/**
 * Constructor.
 *
 * @param  population  The interval to take samples from
 * @param  sample  The number of samples to draw from the population
 */
SampleIterator::SampleIterator(int population, int sample) :
    m_x1(0), m_x2(0), m_slope(0.0), m_increment(0), m_counter(0.0), m_laststepsize(0)
{
    m_x1 = population;
    m_x2 = sample;
    m_slope = (double)m_x1/m_x2;
    reset();
}

/**
 * Destructor.
 */
SampleIterator::~SampleIterator() {

}

/**
 * Reset the iterator
 */
void SampleIterator::reset() {
    m_increment = (int)(m_slope / 2); 
    m_counter = 0.0;
}

/**
 * Grab the next value in the sequence.  Gives results subsequent results with increasing values.
 */
int SampleIterator::next() {
    // check if we've increased by a whole amount
    if (m_counter >= 1.0) {
        // set the step to integer amount
        int step = (int)m_counter;
        // set counter to be just the fractional amount
        m_counter -= step;

        // increase our sample position by the integer amount
        m_increment += step;
        m_laststepsize = step;
    }

    // increase the counter by the next chunk for the next sample
    m_counter += m_slope;

    return m_increment;
}

int SampleIterator::getLastStepSize() {
    return m_laststepsize;
}

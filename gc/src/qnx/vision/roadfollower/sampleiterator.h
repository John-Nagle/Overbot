#ifndef _SAMPLEITERATOR
#define _SAMPLEITERATOR


class SampleIterator {
    int m_x1; // the size of the interval to sample from
    int m_x2; // the number of samples to draw from the interval
    double m_slope; // the number of units (pixels) in interval between samples
    int m_increment; // the current position to take a sample from
    double m_counter; // a counter to keep track of fractional sampling rate
    int m_laststepsize; // how much did we move to get from previous to current position

public:
    SampleIterator(int population, int sample);
    ~SampleIterator();
    
    void reset();
    int next();
    int getLastStepSize();
};

#endif

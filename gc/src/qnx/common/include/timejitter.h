//
//	timejitter.h  -- time jitter correction
//
//	John Nagle
//	Team Overbot
//	March, 2005
//
#ifndef TIMEJITTER_H
#define TIMEJITTER_H
#include <inttypes.h>
//
//	class SimplePLL  -- simple phase locked loop
//
//	Fixed time period
//
class SimplePLL {
	int m_period;												// period
	uint64_t m_time;										// last time
	uint64_t m_max_error;								// max error before resync							
	float m_slewrate;										// max slew rate as fraction of cycle
public:
	SimplePLL(uint64_t period, uint64_t maxerror, float slewrate)
	: m_period(period), m_time(0), m_max_error(maxerror), m_slewrate(slewrate)
	{}
	void setslewrate(float slew) {	m_slewrate = slew; }	// allowed range 0..1, typically 0.05
	bool correcttimebase(uint64_t& time);				// does the work
	uint64_t gettimebase() const { return(m_time);	}	// get timebase, mostly for debug
	int getperiod() const { return(m_period);	}		// get period
private:
	void resync(uint64_t time)								// resync on error
	{	m_time = time;	}
};
//
//	Implementation
//
//	correcttimebase  --  correct a sequence of time values
//
//	Time values are supposed to be m_period apart, but may contain
//	substantial time jitter.  This corrects the time jitter.
//
inline bool SimplePLL::correcttimebase(uint64_t& time)
{	int diff = 0;														// time difference, signed
	if (time < m_time)												// if time difference negative
	{	uint64_t tdiff = m_time - time;						// compute unsigned 64-bit diff
		if (tdiff > m_max_error)									// if too big
		{		resync(time); return(false);	}				// reset
		diff = -int(tdiff);												// now small enough to reduce to 32 bits
	} else {															// time difference positive
		uint64_t tdiff = time - m_time;						// compute nonnegative time difference
		if (tdiff > m_max_error)									// if too much time has elapsed
		{	resync(time); return(false); }					// resync
		diff = int(tdiff);												// now small enough to reduce to 32 bits
	}
	//	"diff" is now the time difference as a 32-bit signed number.
	int err = diff-m_period;										// <0 means fast
	int correction = int(err*double(m_slewrate));	// compute corrected time diff
	m_time += uint64_t(m_period + correction);		// compute corrected time for next event
	time = m_time - m_period;									// return corrected time for this event
	return(true);														// success
}
#endif // TIMEJITTER_H
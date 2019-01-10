#ifdef OBSOLETE
//
//	curvatureest.cc  -- curvature estimator
//
//	John Nagle
//	Team Overbot
//	February, 2005
//
//	class CurvatureEstimator  -- estimates curvature given a set of points
//
//	***NOT READY FOR USE***
//
#include "algebra3.h"
//
//	Constants
//
const double k_big_move = 5;								// more than 5m, bogus
const double k_small_move = 0.20;							// too small, skip
//	
class CurvatureEstimator {
private:
	float	m_filterconst;									// filter constant
	int		m_minpts;										// min points in filter
	float	m_curvature;									// filtered curvature
	vec2	m_prevpt;										// last point seen
	int		m_filteredpoints;								// count of points in filter
public:
	CurvatureEstimator(float filterconstant, int minpts = 5);
	void addPoint(const vec2& pt, const vec2& forwarddir);	// add a new point
	bool getCurvature(float& curv);							// get current curvature estimate
	void reset();								// clear
};
//
//	Constructor
//
CurvatureEstimator::CurvatureEstimator(float filterconstant, int minpts)
	: m_filterconst(filterconstant), m_minpts(minpts)
{	reset();	}
//
//	reset  -- reset to empty state
//
void CurvatureEstimator::reset()
{	m_curvature = 0;
	m_filteredpoints = 0;
}

//
//	addPoint  --  add a point to the curvature estimator.
//
//	The current and previous points form two points of a triangle,
//	with sides of the radius of the turning circle.
//
//	***TOO SENSITIVE TO YAW ERROR*** rethink. 
//
void CurvatureEstimator::addPoint(const vec2& pt, const vec2& forwarddir)
{	
	if (m_filteredpoints++ == 0)							// if no points yet
	{	m_prevpt = pt;										// save first point
		return;												// nothing more yet
	}
	vec2 delta(m_prevpt - pt);								// vector since last point
	double lensq = delta.length2();							// square of length
	if (lensq > (k_big_move*k_big_move))					// if too big a move
	{	reset();
		m_prevpt = pt;										// start from here
		return;
	}
	if (lensq < (k_small_move*k_small_move))				// not enough move
	{	return;	}											// ignore
	//	Length of delta is reasonable.  Use it.
	const vec2 rightdir(forwarddir[1], -forwarddir[0]);		// unit vector to right
	////double len = sqrt(lensq);								// forward length
	////double h = delta*forwarddir;							// distance moved ahead
	double d = delta*rightdir;								// distance moved to right
	//	***CHECK GEOMETRY BELOW***
	double curvature = 2*d/(lensq);							// curvature, this move
	if (m_filteredpoints > 1)								// at least one delta
	{	//	Apply unweighted low pass filter to curvature.
		m_curvature = curvature*m_filterconst + m_curvature*(1.0-m_filterconst);		
	} else {
		m_curvature = curvature;							// initialize filter
	}
}
//
//	getCurvature  -- get estimated curvature
//
bool CurvatureEstimator::getCurvature(float& curv)
{
	curv = m_curvature;
	return(m_filteredpoints >= m_minpts);					// valid if at least this many
}
#endif // OBSOLETE

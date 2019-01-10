//
//	photonwidget.h  --  useful encapsulations for Photon widgets
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef PHOTONWIDGET_H
#define PHOTONWIDGET_H

#include <math.h>
#include <Pt.h>

//
//	class PhotonWidget --  encapsulation of a Photon widget
//
class PhotonWidget {
	PtWidget_t* m_widget;								// the widget
public:
	PhotonWidget(PtWidget_t* widget)
	: m_widget(widget) {}
	PtWidget_t* GetWidget() const { return(m_widget); }
	template <class T>
	void GetResource(long resourceid, T& val, size_t len=0)
	{	T* valp = 0;
		PtGetResource(m_widget, resourceid, &valp, len );	// get pointer to widget data
		assert(valp);
		val = *valp;
	}
	template <class T>
	void SetResource(long resourceid, const T& val, size_t len=0)
	{	PtSetResource(m_widget, resourceid, val, len );	}				
};
//
//	class PhotonMeter -- encapsulation of Photon PtMeter
//
class PhotonMeter: public PhotonWidget {
	PgColor_t	m_origcolor[3];							// original dial colors
	short m_metermin;
	short m_metermax;
	float m_valmin;
	float m_valmax;
	bool m_valid;											// meter is valid, not red
public:
	PhotonMeter(PtWidget_t* widget, float valmin, float valmax);
	void SetValue(float meterval);
	void SetUnscaledValue(float meterval);
	float GetValue();
	void SetColors(const PgColor_t[]);				// set meter colors
	void ResetColors() { SetColors(m_origcolor);	}	// set back to original
	float GetMinValue() const { return(m_valmin); }
	float GetMaxValue() const { return(m_valmax); }
};
//
//	class PhotonTrend -- encapsulation of Photon PtTrend
//
class PhotonTrend: public PhotonWidget {
	short m_metermin;
	short m_metermax;
	float m_valmin;
	float m_valmax;
	short m_trends;
public:
	PhotonTrend(PtWidget_t* widget, float valmin, float valmax);
	void SetValue(const float meterva[], size_t len);
	void SetColors(const PgColor_t colors[], size_t len);	// set colors for each trend
};
//
//	class PhotonMeterGroup  -- a group consisting of a goal meter, an actual value meter, and a trend.
//
class PhotonMeterGroup {
public:																	// for now
	PhotonMeter m_goalmeter;
	PhotonMeter m_actualmeter;
	PhotonTrend m_trend;
public:
	PhotonMeterGroup(PtWidget_t* goalwidget, PtWidget_t* actualwidget, PtWidget_t* trendwidget, float valmin, float valmax);
};
//
//	class PhotonButton --  a non-momentary button, which can be set or cleared
//
class PhotonButton: public PhotonWidget
{
	PgColor_t m_origcolor;										// color at startup
public:
	PhotonButton(PtWidget_t* widget);
	void SetColor(const PgColor_t& color);				// set button color
	void ResetColor() { SetColor(m_origcolor);	}	// set back to original
	void SetPressed(bool pressed, bool blocked=false);		// set as pressed/unpressed
};
//
//	Implementation
//
//
//	Class PhotonMeter
//
const int k_metercolorids[3] = {				// table of meter color Photon resource IDs
		Pt_ARG_METER_LEVEL1_COLOR,
		Pt_ARG_METER_LEVEL2_COLOR,
		Pt_ARG_METER_LEVEL3_COLOR };

//
//	Constructor
//
inline PhotonMeter::PhotonMeter(PtWidget_t* widget,  float valmin, float valmax)
	: PhotonWidget(widget), m_valmin(valmin), m_valmax(valmax), m_valid(true)
{
	for (int i=0; i<3; i++)					// save original meter colors
	{	GetResource(k_metercolorids[i], m_origcolor[i]); }
	//	Save meter bounds
	GetResource(Pt_ARG_METER_MIN_NEEDLE_POSITION, m_metermin);	
	GetResource(Pt_ARG_METER_MAX_NEEDLE_POSITION, m_metermax);
};	
//
//	SetUnscaledValue -- set to indicated value
//
//	Unscaled value - same values as declared in PhAB
//
//	Setting the value to NAN will turn the meter red
//
inline void PhotonMeter::SetUnscaledValue(float val)
{	const PgColor_t k_invalidmetercolors[3] = { Pg_RED, Pg_RED, Pg_RED };
	if (isnan(val))																			// if invalid value
	{	if (!m_valid) return;																// if already invalid, done
		SetColors(k_invalidmetercolors);											// set to red
		m_valid = false;	
		return;								
	}
	//	Valid number, not a NAN
	if (!m_valid)																				// if was invalid, but now valid
	{	ResetColors(); 																		// reset to usual colors
		m_valid = true;
	}
	float meterfract = (val-m_metermin) / (m_metermax - m_metermin);		// into same range as meter
	short meterpos = short(0.5+m_metermin + meterfract *  (m_metermax - m_metermin)); // to pointer value
	short oldmeterpos;
	GetResource(Pt_ARG_METER_NEEDLE_POSITION, oldmeterpos);
	if (oldmeterpos == meterpos) return;										// avoid unnecessary update
	SetResource(Pt_ARG_METER_NEEDLE_POSITION, meterpos);		// change on-screen meter
}
//
//	SetValue -- set to indicated value
//
//	Rescales from floating point space so we can handle big meter ranges
//
//	Setting the value to NAN will turn the meter red
//
inline void PhotonMeter::SetValue(float val)
{	const PgColor_t k_invalidmetercolors[3] = { Pg_RED, Pg_RED, Pg_RED };
	if (isnan(val))																			// if invalid value
	{	if (!m_valid) return;																// if already invalid, done
		SetColors(k_invalidmetercolors);											// set to red
		m_valid = false;	
		return;								
	}
	//	Valid number, not a NAN
	if (!m_valid)																				// if was invalid, but now valid
	{	ResetColors(); 																		// reset to usual colors
		m_valid = true;
	}
	float meterfract = (val-m_valmin) / (m_valmax-m_valmin);		// into range 0..1
	short meterpos = short(0.5+m_metermin + meterfract *  (m_metermax - m_metermin)); // to pointer value
	short oldmeterpos;
	GetResource(Pt_ARG_METER_NEEDLE_POSITION, oldmeterpos);
	if (oldmeterpos == meterpos) return;										// avoid unnecessary update
	SetResource(Pt_ARG_METER_NEEDLE_POSITION, meterpos);		// change on-screen meter
}
//
//	GetValue
//
inline float PhotonMeter::GetValue() 
{	short meterpos = 0;
	GetResource(Pt_ARG_METER_NEEDLE_POSITION, meterpos);
	float meterfract = (meterpos - m_metermin) / float(m_metermax - m_metermin);
	return(m_valmin + meterfract*(m_valmax - m_valmin));
}

//
//	SetColors -- set meter colors
//
inline void PhotonMeter::SetColors(const PgColor_t colors[])
{	
	for (int i=0; i<3; i++)
	{	SetResource(k_metercolorids[i],colors[i]); }	// set widget data
}
//
//	Constructor
//
inline PhotonTrend::PhotonTrend(PtWidget_t* widget,  float valmin, float valmax)
	: PhotonWidget(widget), m_valmin(valmin), m_valmax(valmax)
{
	GetResource(Pt_ARG_TREND_MIN, m_metermin);	
	GetResource(Pt_ARG_TREND_MAX, m_metermax);
	GetResource(Pt_ARG_TREND_COUNT, m_trends);
};	
//
//	SetValue -- add data to trend graphs
//
//	Rescales from floating point space so we can handle big meter ranges
//
inline void PhotonTrend::SetValue(const float val[], size_t len)
{
	assert(m_trends == short(len));				// size of values must match
	const size_t k_maxtrendvals = 10;			// max number of trends
	short trendpos[k_maxtrendvals];				// trend value area
	assert(len <= k_maxtrendvals);				// must fit
	//	Scale all items into trend graph
	for (size_t i=0; i<len; i++)
	{	double fval = val[i];
		if (isnan(fval)) fval = 0.0;						// treat NAN as 0
		float meterfract = (fval-m_valmin) / (m_valmax-m_valmin);		// into range 0..1
		trendpos[i] = short(m_metermin + meterfract *  (m_metermax - m_metermin)); // to pointer value
	}
	//	Send values to trend display
	PtSetResource(GetWidget(), Pt_ARG_TREND_DATA , trendpos, len);
}
//
//	SetColors -- set colors for each trend
//
//	PhAB doesn't offer this option.
//
inline void PhotonTrend::SetColors(const PgColor_t colors[], size_t len)
{	const size_t k_max_trends = 10;
	assert(m_trends == short(len));				// size of values must match
	assert(m_trends <= short(k_max_trends));		// must fit
	// 	Code modelled after help for PtTrend.
	PtTrendAttr_t several_attr[k_max_trends];	// attributes of each trend.
	//	Send color list to widget
	PtSetResource(GetWidget(), Pt_ARG_TREND_COLOR_LIST, colors, len);
	//	Map the trends to colors.	1:1 mapping.
	//	Note assumption that PtTrendAttr has only one field. Currently valid.
	for (size_t i=0; i<len; i++)							// for all trend lines
	{	several_attr[i].map = i+1;	}					// this means use color I from list
	//	Set the colors 
	PtSetResource(GetWidget(), Pt_ARG_TREND_ATTRIBUTES, several_attr,0);
}
//
//	PhotonMeterGroup
//
inline PhotonMeterGroup::PhotonMeterGroup(PtWidget_t* goalwidget, PtWidget_t* actualwidget, PtWidget_t* trendwidget,
	float valmin, float valmax)
: 	m_goalmeter(goalwidget, valmin, valmax),
	m_actualmeter(actualwidget,valmin, valmax),
	m_trend(trendwidget, valmin, valmax)
{
}
//
//	PhotonButton
//
inline PhotonButton::PhotonButton(PtWidget_t* widget)
: PhotonWidget(widget)
{	GetResource(Pt_ARG_FILL_COLOR,m_origcolor); }
//
//	SetColor -- set button color
//
//	Used for buttons which can change state due to application actions.
//
inline void PhotonButton::SetColor(const PgColor_t& color)
{	PgColor_t oldcolor;
	GetResource(Pt_ARG_FILL_COLOR, oldcolor);
	if (oldcolor == color) return;						// avoid unnecessary redraws
	SetResource(Pt_ARG_FILL_COLOR, color);
}
//
//	SetPressed -- set "Pressed" status of button
//
inline void PhotonButton::SetPressed(bool pressed, bool blocked)
{	const long k_pressedbits = Pt_SET | Pt_BLOCKED | Pt_GHOST;
	bool ghost = blocked;
	if (pressed) blocked = true;							// pressed implies blocked, but not ghost appearance
	long flags = (pressed ? Pt_SET : 0) | (blocked ? Pt_BLOCKED : 0) | (ghost ? Pt_GHOST : 0);
	//	Set relevant bits. Note that this requires a mask.
	PtSetResource(GetWidget(), Pt_ARG_FLAGS, flags, k_pressedbits);
}
//
//	PhotonLock -- declare a lock at the entrance to any code that calls
//					Photon from another thread.
//
class PhotonLock {
	int m_lock;
public:
	PhotonLock(int flags = 0)								// lock in constructor
	{	m_lock = PtEnter(flags); 
		if (m_lock < 0) throw("PhotonLock: PtEnter lock error");
	}
	~PhotonLock()												// auto unlock at destructor
	{	PtLeave(m_lock); }
};
#endif // PHOTONWIDGET_H
//
//	simplecontroller.h  --  simple controller interface for Galil controllers.
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef SIMPLECONTROLLER_H
#define SIMPLECONTROLLER_H

#include <math.h>
#include "controller.h"

//
//	Class SimpleLowPassFilter  -- a simple low-pass filter 
//
class SimpleLowPassFilter {
	bool m_prevvalid;													// true if stored value is meaningful
	float m_prev;															// stored history
	float m_filterconstant;											// range 0..1
public:
	SimpleLowPassFilter(float filterconstant)
	: m_prevvalid(false), m_prev(0), m_filterconstant(filterconstant)
	{}
	float GetOutput() const { return(m_prev); }
	void PutInput(float next)
	{	if (!m_prevvalid)												// first time, initialize filter
		{	m_prev = next; m_prevvalid = true; }
		//	Usual case, filter
		m_prev = (m_prev * (1.0-m_filterconstant)) + (next *m_filterconstant);
	}
	void SetFilterConstant(float filterconstant)			// change filter constant
	{	m_filterconstant = filterconstant; m_prevvalid = false; }
};
//
//	class SimpleErrorMonitor  -- monitor amount of error
//
//	This computes how well a controller is tracking. 
//
//	Each update (provided at some constant rate) updates the goal,
//	which is low-pass filtered to compensate for control lag.
//	The error is then computed, and low-pass filtered to reduce
//	the effects of noise. The result is the controller error.
//
class SimpleErrorMonitor {
	SimpleLowPassFilter m_goalfilter;							// low-pass filter of goal values
	SimpleLowPassFilter m_errfilter;							// low-pass filter of error values
public:
	SimpleErrorMonitor(float goalfilterval, float errfilterval)
	:	m_goalfilter(goalfilterval), m_errfilter(errfilterval)
	{}
	SimpleErrorMonitor()												// default constructor, no filtering
	:	m_goalfilter(1), m_errfilter(1)
	{}
	float Update(float newgoal, float newactual)		// updates and returns current filtered error
	{	m_goalfilter.PutInput(newgoal);							// update low-pass filtered goal
		float err = fabs(newactual - m_goalfilter.GetOutput());	// get error
		m_errfilter.PutInput(err);									// update low-pass filtered error
		return(m_errfilter.GetOutput());							// return filtered error
	}
	void SetFilterConstants(float goalfilterval, float errfilterval)	// set filter constants
	{	m_goalfilter.SetFilterConstant(goalfilterval);
		m_errfilter.SetFilterConstant(errfilterval);
	}
	//	Access
	float GetFilteredGoal() const { return (m_goalfilter.GetOutput()); }
	float GetFilteredError() const { return (m_errfilter.GetOutput()); }
};
//
//	Class SimpleController  -- simple control of a Galil controller
//
//	This requires our software in the controller.  That software
//	uses the variables GOAL, READY, AUTO, and FAULT
//	to communicate with this object.
//
//	By default, this assumes a positional controller. If the controller
//	is controlling something else, this class must be subclassed and
//	a GetActual function provided which obtainst the actual value
//	being servoed from the controller.
//
class SimpleController: public Controller
{
private:
	float m_goal;
	float m_actual;
	bool m_auto;																	// true if in auto mode
	float m_goalmin;															// minimum allowed goal value
	float m_goalmax;															// maximum allowed goal value
	SimpleErrorMonitor m_errormonitor;
public:
	SimpleController(const char* name, bool readonly=false);
	virtual ~SimpleController() {}
	//	Low-level interface
	Controller::Err SetSpeed(float speed);
	Controller::Err SetGoal(float goal);
	Controller::Err GetGoal(float& goal);
	Controller::Err GetGoalMin(float& goal);
	Controller::Err GetGoalMax(float& goal);
	Controller::Err GetFault(int& faultcode);
	Controller::Err GetReady(bool& isready);
	Controller::Err GetAuto(bool& isauto);
	Controller::Err SetAuto(bool isauto);
	Controller::Err Stop();
	Controller::Err Start();
	Controller::Err Reset();
	virtual Controller::Err GetActual(float& goalactual)		// must be subclassed if goal is not a position
	{	return(ActualPosnGet(&goalactual));}						// by default, gets position
	//	High-level interface
	virtual Controller::Err UpdateState();								// update local state
	float GetLocalGoal() const { return(m_goal); }				// returns NAN if invalid
	float GetLocalActual() const {return(m_actual); }			// returns NAN if invalid
	float GetLocalGoalMin() const { return(m_goalmin); }	// returns NAN if invalid
	float GetLocalGoalMax() const { return(m_goalmax); }// returns NAN if invalid
	bool GetLocalAuto() const { return(m_auto); } 			// returns TRUE If ready for remote control
	float GetFilteredError() const { return(m_errormonitor.GetFilteredError()); }	// returns filtered error value
	float GetFilteredGoal() const { return(m_errormonitor.GetFilteredGoal()); }	// returns filtered goal value
	void SetFilterConstants(float goalfilterval, float errfilterval)	// set error estimator filter constants
	{	m_errormonitor.SetFilterConstants(goalfilterval, errfilterval); }
private:
	Controller::Err UpdateGoalMinMax();								// gets limits at startup
};
#endif // SIMPLECONTROLLER_H
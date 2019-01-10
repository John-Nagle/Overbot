//
//	simplecontroller.cc  --  simple controller interface for Galil controllers.
//
//	John Nagle
//	Team Overbot
//	June, 2004
//
#include "simplecontroller.h"
#include <assert.h>

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
const float k_NaN = _FNan._Float;									// quiet NaN value for floats					
//
//	class SimpleController
//
//	Constructor
//
SimpleController::SimpleController(const char* name, bool readonly=false)
	: Controller(name, readonly),
	m_goal(k_NaN), m_actual(k_NaN), m_auto(false),
	m_goalmin(k_NaN), m_goalmax(k_NaN)
	{	assert(isnan(k_NaN));											// some QNX libraries are broken
		if (!isnan(k_NaN)) abort();										// make really sure
	}

	

Controller::Err SimpleController::SetSpeed(float speed)
{
	return(VariableSet("SPEED",speed));
}
//
//	SetGoal -- set the controller's goal.
//
//	If enabled, the controller will follow
//
Controller::Err SimpleController:: SetGoal(float goal)
{	if (!finite(goal)) return(ERR_PARAMETER_OUT_OF_RANGE);	 // bogus value
	if (!m_auto) return(ERR_PROGRAM_NOT_READY);						// fails if program not ready
	Controller::Err stat = VariableSet("GOAL",goal);
	return(stat);
}
//
//	GetGoal -- get the controller's goal
//
Controller::Err SimpleController::GetGoal(float& goal)
{	Controller::Err stat = VariableGet("GOAL",&goal);
	if (stat != ERR_OK)										// if fails
	{	goal = k_NaN;	}										// invalid goal
	return(stat);
}
//
//	GetGoalMin -- get the controller's minimum goal
//
Controller::Err SimpleController::GetGoalMin(float& goal)
{	Controller::Err stat = VariableGet("GOALMIN",&goal);
	if (stat != ERR_OK)										// if fails
	{	goal = k_NaN;	}										// invalid goal
	return(stat);
}
//
//	GetGoalMax -- get the controller's maximum goal
//
Controller::Err SimpleController::GetGoalMax(float& goal)
{	Controller::Err stat = VariableGet("GOALMAX",&goal);
	if (stat != ERR_OK)										// if fails
	{	goal = k_NaN;	}										// invalid goal
	return(stat);
}
//
//	GetFault -- get fault code from controller program
//
Controller::Err SimpleController::GetFault(int& faultcode)
{	float val;
	Controller::Err stat = VariableGet("FAULT", &val);
	faultcode = int(val);										// fault code as integer
	return(stat);
}
//
//	GetAuto -- get auto status from controller program
//
Controller::Err SimpleController::GetAuto(bool& isauto)
{	float val;
	Controller::Err stat = VariableGet("AUTO", &val);
	isauto = val > 0;											// true if nonzero
	return(stat);
}
//
//	SetAuto -- set AUTO flag in controller
//
Controller::Err SimpleController::SetAuto(bool ison)
{	return(VariableSet("AUTO", ison ? 1.0 : 0.0)); }
//	
//	GetReady  -- get READY status from controller program
//
Controller::Err SimpleController::GetReady(bool& isready)
{	float val;
	Controller::Err stat = VariableGet("READY", &val);
	isready = val == 1.0;										// true if value is one
	return(stat);
}
Controller::Err SimpleController::Stop()
{
	return(MotorOff());											// emergency stop
}
//	
//	Start -- take over controller program
//
Controller::Err SimpleController::Start()
{
	return(VariableSet("AUTO",1.0));					// enter AUTO mode
}
//	
//	Reset -- force a controller reset
//
Controller::Err SimpleController::Reset()
{
	return(ResetToPowerUp());
}
//
//	UpdateGoalMinMax  -- update goal min/max if necessary
//
//	Only needed once, at startup, to get config parameters from controller.
//
Controller::Err SimpleController::UpdateGoalMinMax()
{	if (finitef(m_goalmin) && finitef(m_goalmax)) return(ERR_OK);	// have good values, done
	Controller::Err stat = GetGoalMin(m_goalmin);							// try to get goal min
	if (stat != ERR_OK) return(stat);													// fails
	stat = GetGoalMax(m_goalmax);												// try to get goal max
	if (stat != ERR_OK) return(stat);													// fails
	return(ERR_OK);
}

//
//	High level functions
//
//
//	UpdateState -- get the current actual value, check controller status.
//
//	This issues commands to the controller and checks for error conditions.
//	OK for use in read only mode.
//
//	If this returns other than ERR_OK, or m_auto is false on return, we do NOT have control.
//	Updates m_goal and m_actual from the controller.
//
Controller::Err SimpleController::UpdateState()
{	m_goal = m_actual = k_NaN;													// invalidate goal and actual values
	m_auto = false;																			// not ready for automatic control
	//	First, get actual value of controlled variable. Meaningful even if controller program not running
	float actual;
	Controller::Err stat = GetActual(actual);										// get actual value
	if (stat != ERR_OK) return(stat);													// if fail
	m_actual = actual;																	// good actual value
	//	Nextt, make sure program is still running	
	bool isprogramrunning = false;												// check that program is running
	stat = ProgramRunning(&isprogramrunning);							// make sure program is running
	if (stat != ERR_OK) return(stat);													// if fail
	if (!isprogramrunning) return(ERR_PROGRAM_NOT_RUNNING);	// fails if program not running
	//	Get allowed goal limits from controller, available only if program running.
	stat = UpdateGoalMinMax();														// get limits from controller
	if (stat != ERR_OK) return(stat);													// if fail
	//	Are we in AUTO mode?
	stat = GetAuto(m_auto);															// are we in AUTO mode (not onboard manual)?
	if (stat != ERR_OK) return(stat);													// if fail
	stat = GetGoal(m_goal);															// read manually set goal
	if (stat != ERR_OK) return(stat);													// fails
	//	Next, make sure program is in ready status
	bool isready	= false;																	// check that program is running
	stat = GetReady(isready);														// make sure program is running
	if (stat != ERR_OK) return(stat);													// if fail
	if (!isready) return(ERR_PROGRAM_NOT_READY);						// fails if program not ready
	//	Update error estimator
	m_errormonitor.Update(m_goal, m_actual);								// update error estimator
	return(ERR_OK);																		// success
}
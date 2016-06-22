#include "SimPID.h"
//#include "WPILib.h"
#include <stdio.h>

#define PI 3.141592653589793f

/**
 * Initializes the SimPID object. All parameters default to 0.
 */
SimPID::SimPID(float p, float i, float d, float epsilon)
{
	m_p = p;
	m_i = i;
	m_d = d;

	m_errorEpsilon = epsilon;
	m_desiredValue = 0; // Default to 0, set later by the user
	m_firstCycle = true;
	m_maxOutput = 1.0; // Default to full range
	m_minOutput = 0.0;
	m_errorIncrement = 1;

	m_cycleCount = 0;
	m_minCycleCount = 10; // Default
	m_errorSum = 0;
	m_previousValue = 0;

	IsContinuousAngle = false;
}

/**
 * Sets the PID constants to new values.
 */
void SimPID::setConstants(float p, float i, float d)
{
	m_p = p;
	m_i = i;
	m_d = d;
	
}

void SimPID::setContinuousAngle(bool set)
{
	IsContinuousAngle = set;
}
inline float SimPID::normal(float x)
{
	if(x > PI)
		x -= 2*PI;
	else if(x < -PI)
		x += 2*PI;
	return x;
}
float SimPID::getP()
{
	return m_p;
}

int SimPID::getErrorSum()
{
	return m_errorSum;
}

/**
 * Sets the allowable error range away from the desired value.
 */
void SimPID::setErrorEpsilon(float epsilon)
{
	m_errorEpsilon = epsilon;
}

/**
 * Sets the maximum increment to the error sum used in the I component
 * calculation.
 * This defaults to 1 in the constructor, which has worked well for 1114 the
 * past few years.
 */
void SimPID::setErrorIncrement(int inc)
{
	m_errorIncrement = inc;
}

/**
 * Sets the desired value.
 */
void SimPID::setDesiredValue(float val)
{
	m_desiredValue = val;
	if(IsContinuousAngle == true)
		m_desiredValue = normal(m_desiredValue);
}
	
/**
 * Sets the ceiling for the output of the calculation.
 * This defaults to 1.0 (full output). Values should be between 0.0 and 1.0.
 */
void SimPID::setMaxOutput(float max)
{	if(max >= 0.0 && max <= 1.0)
	{
		m_maxOutput = max;
	}
}

void SimPID::setMinOutput(float min)
{
	if (min >= 0.0 && min <= 1.0){
		m_minOutput = min;
	}
}

/**
 * Resets the error sum back to zero.
 */
void SimPID::resetErrorSum(void)
{
	m_errorSum = 0;
}

/**
 * Calculates the PID output based on the current value.
 * PID constants and desired value should be set before calling this
 * function.
 */
float SimPID::calcPID(float currentValue)
{	
	// Initialize all components to 0.0 to start.
	float pVal = 0.0;
	float iVal = 0.0;
	float dVal = 0.0;
		
	// Don't apply D the first time through.
	if(m_firstCycle)
	{	
		m_previousValue = currentValue;  // Effective velocity of 0
		m_firstCycle = false;
	}
	
	// Calculate P Component.
	error = m_desiredValue - currentValue;
	if(IsContinuousAngle == true)
		error = normal(m_desiredValue - currentValue);
	pVal = m_p * (float)error;
	printf("SIMPID SAYS: Target: %f, current: %f, error: %f\n", m_desiredValue, currentValue, error);

	
	// Calculate I Component.
	// Error is positive and outside the epsilon band.
	if(error >= m_errorEpsilon)
	{	
		// Check if epsilon was pushing in the wrong direction.
		if(m_errorSum < 0)
		{
			// If we are fighting away from the point, reset the error.
			m_errorSum = 0;
		}
		if(error < m_errorIncrement)
		{
			// If the error is smaller than the max increment amount, add it.
			m_errorSum += error;
		}
		else 
		{
			// Otherwise, add the maximum increment per cycle.
			m_errorSum += m_errorIncrement;      
		}
	}
	// Error is negative and outside the epsilon band.
	else if(error <= -m_errorEpsilon)
	{	
		if(m_errorSum > 0)
		{
			// If we are fighting away from the point, reset the error.
			m_errorSum = 0;
		}
		// error is small than max contribution -> just subtract error amount
		if(error > -m_errorIncrement)
		{
			// If the error is smaller than the max increment amount, add it.
			m_errorSum += error; // Error is negative
		}
		else
		{
			// Otherwise, subtract the maximum increment per cycle.
			m_errorSum -= m_errorIncrement;
		}
	}
	// Error is inside the epsilon band. 
	else
	{
		m_errorSum = 0;
	}
	iVal = m_i * (float)m_errorSum;
	
	// Calculate D Component.
	int velocity = currentValue - m_previousValue;
	dVal = m_d * (float)velocity;

	// Calculate and limit the ouput: Output = P + I - D
	float output = pVal + iVal - dVal;
	if(output > m_maxOutput)
	{
		output = m_maxOutput;
	}
	else if(output < -m_maxOutput)
	{
		output = -m_maxOutput;
	}
	
	if (output < m_minOutput && output > -m_minOutput){
		if (output > 0)
			output = m_minOutput;
		else
			output = -m_minOutput;
	}

	if (m_previousValue <= m_desiredValue + m_errorEpsilon
				&& m_previousValue >= m_desiredValue - m_errorEpsilon
				&& !m_firstCycle && m_minOutput != 0){
			output = 0.0;
	}



	// Save the current value for next cycle's D calculation.
	m_previousValue = currentValue;
	
	return output;
}

/**
 * Sets the minimum number of cycles the value must be in the epsilon range
 * before the system is considered stable.
 */
void SimPID::setMinDoneCycles(int n)
{
	m_minCycleCount = n;
}

float SimPID::getError(void)
{
	return error;
}

/**
 * Returns true if the last input was within the epsilon range of the
 * destination value, and the system is stable.
 */
bool SimPID::isDone(void)
{	
	if (m_previousValue <= m_desiredValue + m_errorEpsilon
			&& m_previousValue >= m_desiredValue - m_errorEpsilon
			&& !m_firstCycle)
	{
		if(m_cycleCount >= m_minCycleCount)
		{
			return true;
		}
		else 
		{	
			m_cycleCount++;
			return false;
		}
	}
	else
	{
		m_cycleCount = 0;
		return false;
	}
}

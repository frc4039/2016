#ifndef SIMPID_H_
#define SIMPID_H_

class SimPID
{
public:
	SimPID(float p = 0.0, float i = 0.0, float d = 0.0, float epsilon = 0.0);
	
	void setConstants(float p, float i, float d);
	float getP(void);
	int getErrorSum(void);
	int getVelocity(void);
	void setErrorEpsilon(float epsilon);
	void setErrorIncrement(int inc);
	void setDesiredValue(float val);
	void setMaxOutput(float max);
	void setMinOutput(float min);
	void resetErrorSum(void);
	float getError(void);

	void setContinuousAngle(bool set);

		
	float calcPID(float current);
	
	bool isDone(void);
	void setMinDoneCycles(int n);
	
private:
	float m_p;   // P coefficient
	float m_i;   // I coefficient
	float m_d;   // D coefficient

	float m_desiredValue; // Desired value
	float m_previousValue; // Value at last call
	int m_errorSum; // Sum of previous errors (for I calculation)
	int m_errorIncrement; // Max increment to error sum each call
	float m_errorEpsilon; // Allowable error in determining when done
	float error;
	
	bool IsContinuousAngle;

	bool m_firstCycle; // Flag for first cycle
	float m_maxOutput; // Ceiling on calculation output
	float m_minOutput; // floor on calculation output

	int m_minCycleCount; // Minimum number of cycles in epsilon range to be done
	int m_cycleCount; // Current number of cycles in epsilon range
	inline float normal(float x);

};

#endif // SIMPID_H_

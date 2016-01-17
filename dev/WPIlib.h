#ifndef __WPI
#define __WPI

//this is a dummy gyro class to allow for test compilation
class Gyro
{
	private:
	int m_port;
	float m_val;
	
	protected:
	Gyro(){};


	public:
	Gyro(int port) { m_port = port; m_val = 42.0f; };
	float getValue(void) { return m_val; };
	void setSensitivity(float x) {};
};

class Encoder
{
	private:
	Encoder(){};
	int m_port;
	int m_val;

	public: 
	Encoder(int port) { m_port = port; m_val = 99; };
	int getVal(void) { return m_val; };
};

#endif

#ifndef __AUTODRIVE__
#define __AUTODRIVE__

struct Vector {
	float x, y, a;
}

class AutoDrive
{
	private:
		AutoDrive();
		void m_drive(float speed, float rotation);
		Vector m_closestPoint(void);
		Vector m_position;
		bool m_enabled;
		#define KP_D 1 // P value for distance
 		#define KP_H 1 // P value for heading
	public:
		AutoDrive(Gyro* gyro, Encoder* left, Encoder* right);
		void setPath(char* file);
		void startPath(void);
		void stopPath(void);
		int tick(void)
};
#endif

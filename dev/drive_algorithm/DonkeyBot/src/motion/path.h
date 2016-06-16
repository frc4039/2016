#ifndef __PATH__
#define __PATH__

class Path
{
public:
	Path();
	virtual ~Path();
	int **path;
	float *pathDistance;
	int size;
	virtual void calculate() = 0;
	void add(Path* partOfPath);
	void show();
	int* getPoint(int i);
	int* getEndPoint(void);
	float getPathDistance(int i);
	void calcPathDistance(void);
};

#endif

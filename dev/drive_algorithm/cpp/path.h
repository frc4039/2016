#ifndef __PATH__
#define __PATH__

class Path
{
public:
	Path();
	int **path;
	int size;
	virtual void calculate() = 0;
	void add(Path* partOfPath);
	void show();
	int* getEndPoint();
	int* getPoint(int i);


};

#endif

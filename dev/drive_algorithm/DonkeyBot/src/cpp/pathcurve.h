#ifndef __CURVE_PATH__
#define __CURVE_PATH__

#include "path.h"

class PathCurve : public Path
{
private:
	int *start;
	int *end;
	int *cp1;
	int *cp2;
	float f(int i, float u);
	int fact(int x);
public: 
	PathCurve(int start[2], int ncp1[2], int ncp2[2], int end[2], int nresolution);
	void calculate();
};

#endif

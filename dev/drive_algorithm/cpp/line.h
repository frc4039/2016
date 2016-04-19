#ifndef __LINE_PATH__
#define __LINE_PATH__

#include "path.h"

class Line : public Path
{
private:
	int *start;
	int *end;
public: 
	Line(int start[2], int end[2], int nresolution);
	void calculate();
};

#endif

#ifndef __LINE_PATH__
#define __LINE_PATH__

#include "path.h"

class PathLine : public Path
{
private:
	PathLine();
	int *start;
	int *end;
public: 
	PathLine(int start[2], int end[2], int nresolution);
	void calculate();
};

#endif

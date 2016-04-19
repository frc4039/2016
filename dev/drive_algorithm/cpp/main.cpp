#include "line.h"
#include "path.h"
#include "curve.h"

int main(){
	Path *line;
	Path *curve;

	int start[2] = {0,0};
	int end[2] = {5000,3000};

	int c1[2] = {0,0};
	int c2[2] = {0, 3000};
	int c3[2] = {2000, 1000};
	int c4[2] = {5000,3000};

	line = new Line(start, end, 100);
	line->calculate();
	
	curve = new Curve(c1,c2,c3,c4,200);
	curve->calculate();

	line->add(curve);
	line->show();

	return 0;
}

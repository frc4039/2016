#include "line.h"
#include "path.h"
#include "curve.h"
#include "pathfollower.h"

int main(){
	Path *auto1;
	Path *curve;
	PathFollower *robot;

	int start[2] = {0,0};
	int end[2] = {16000,0};
	int c2[2] = {8000, 0};
	int c3[2] = {12000, 5000};
	int c4[2] = {0, 5000};

	auto1 = new Line(start,end, 100);
	auto1->calculate();
	
	curve = new Curve(end,c2,c3,c4,200);
	curve->calculate();

	robot = new PathFollower();

	auto1->add(curve);
	auto1->show();

	float left, right;
	int x = 0, y = 0;
	robot->initPath(auto1);
	robot->followPath(x, y, &left, &right);

	return 0;
}

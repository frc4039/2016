#include "motion/path.h"
#include "motion/pathcurve.h"
#include "motion/pathfollower.h"
#include "motion/pathline.h"

int main(){
	Path *auto1;
	Path *curve;
	PathFollower *robot;

	int start[2] = {0,0};
	int end[2] = {16000,0};
	int c2[2] = {8000, 0};
	int c3[2] = {16000, -5000};
	int c4[2] = {0, -5000};

	auto1 = new PathLine(start,end, 100);
	//auto1->calculate();
	
	curve = new PathCurve(end,c2,c3,c4,200);
	//curve->calculate();

	robot = new PathFollower();

	auto1->add(curve);
	auto1->show();

	float left, right;
	int x = 0, y = 0;
	robot->initPath(auto1, PathForward);

	/*while (robot->followPath(nav->GetYaw(), &left, &right) == 0){
		leftDrive->set(left)*/

	robot->updatePos();
	//}

	return 0;
}
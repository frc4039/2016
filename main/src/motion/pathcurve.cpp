#include "pathcurve.h"

#include <stdio.h>
#include <math.h>

PathCurve::PathCurve(int nstart[2], int ncp1[2], int ncp2[2], int nend[2], int nresolution){
	size = nresolution;
	start = nstart;
	end = nend;
	cp1 = ncp1;
	cp2 = ncp2;
	calculate();
	calcPathDistance();
}

int PathCurve::fact(int x){
	if ( x == 0 )
		return 1;
	int y = 1;
	for(int i = 2; i <= x; i++)
		y *= i;
	return y;
}

float PathCurve::f(int i, float u){
	return (fact(3) / fact(i) / fact(3-i))*pow(u,i)*pow(1-u, 3-i);
}

void PathCurve::calculate(){
	//get ready for parameterization
	float U = 0;
	int I = 0;
	//set up the path variable with how many points we
	//are going to put into it
	path = new int*[size];
	for(int i = 0; i < size; i++)
		path[i] = new int[2];

	//fill in the points of the path
	//this is done using the parameterization
	//of a bezier curve  
	while(I < size){
		//fill in the point
		path[I][0] = start[0]*f(0,U) + cp1[0]*f(1,U) + cp2[0]*f(2,U) + end[0]*f(3,U);
		path[I][1] = start[1]*f(0,U) + cp1[1]*f(1,U) + cp2[1]*f(2,U) + end[1]*f(3,U);
		
		//for debug
		//printf("%d,%d\n",path[I][0], path[I][1]);
		//printf("\tU: %f, I: %d\n", U, I);

		//go to next point
		U += 1.f/(size-1);
		I++;
		
	}
}

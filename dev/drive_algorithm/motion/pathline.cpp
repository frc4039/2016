#include "pathline.h"

#include <stdio.h>

PathLine::PathLine(){}

PathLine::PathLine(int nstart[2], int nend[2], int nresolution){
	size = nresolution;
	start = nstart;
	end = nend;
	calculate();
	calcPathDistance();
}

void PathLine::calculate(){
	//get ready for parameterization
	float U = 0;
	int I = 0;
	//set up the path variable with how many points we
	//are going to put into it
	path = new int*[size];
	for(int i = 0; i < size; i++)
		path[i] = new int[2];

	//fill in the points of the path  
	while(I < size){
		//fill in the point
		path[I][0] = start[0] + ((end[0] - start[0]) * U);
		path[I][1] = start[1] + ((end[1] - start[1]) * U);
	
		//for debug
		//printf("%d,%d\n",path[I][0], path[I][1]);
		//printf("\tU: %f, I: %d\n", U, I);

		//go to next point
		U += 1.f/(size-1);
		I++;
		
	}
}

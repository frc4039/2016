#include "path.h"
#include <stdio.h>
#include <math.h>
#define SQ(X) ((X)*(X))
	
Path::Path(){}

Path::~Path(){
	for(int i = 0; i < size; i++)
			delete[] path[i];
		delete[] path;
		delete[] pathDistance;
}

void Path::add(Path* partOfPath){
	int newsize = size+partOfPath->size;

	//make a tempPath variable to hold values
	int** tempPath = new int*[newsize];
	float* tempDistance = new float[newsize];
	for(int i = 0; i < newsize; i++)
		tempPath[i] = new int[2];

	//put values from each path into tempPath
	for(int i = 0; i < newsize; i++){
		if (i < size){
			tempPath[i][0] = path[i][0];
			tempPath[i][1] = path[i][1];
			tempDistance[i] = pathDistance[i] + partOfPath->pathDistance[0];
		} else {
			tempPath[i][0] = partOfPath->path[i-size][0];
			tempPath[i][1] = partOfPath->path[i-size][1];
			tempDistance[i] = partOfPath->pathDistance[i-size];
		}
	}

	//copy values into redefined path variable
	for(int i = 0; i < size; i++)
		delete[] path[i];
	delete[] path;
	delete[] pathDistance;

	path = new int*[newsize];
	pathDistance = new float[newsize];

	for(int i = 0; i < newsize; i++)
		path[i] = new int[2];

	path = tempPath;
	pathDistance = tempDistance;

	size = newsize;
}

void Path::show(void){
	for(int i = 0; i < size; i++){
		printf("X: %d,Y: %d,\tDistance: %f\n",path[i][0],path[i][1], pathDistance[i]);
	}
}

int* Path::getPoint(int i){
	return path[i];
}

int* Path::getEndPoint(void){
	return path[size-1];
}

float Path::getPathDistance(int i){
	return pathDistance[i];

}

void Path::calcPathDistance(void){
	pathDistance = new float[size];
	float accum = 0;
	pathDistance[size-1] = accum;
	for (int i = size-2; i >= 0; i--){
		accum += sqrt((double)SQ(path[i + 1][0] - path[i][0]) + SQ(path[i + 1][1] - path[i][1]));
		pathDistance[i] = accum;
	}
}

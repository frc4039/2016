#include "path.h"
#include <stdio.h>
#include <math.h>
#define SQ(X) ((X)*(X))
	
Path::Path(){};

void Path::add(Path* partOfPath){
	int newsize = size+partOfPath->size;

	//make a temp variable to hold values
	int** temp = new int*[newsize];
	for(int i = 0; i < newsize; i++)
		temp[i] = new int[2];

	//put values from each path into temp
	for(int i = 0; i < newsize; i++){
		if (i < size){
			temp[i][0] = path[i][0];
			temp[i][1] = path[i][1];
		} else {
			temp[i][0] = partOfPath->path[i-size][0];
			temp[i][1] = partOfPath->path[i-size][1];
		}
	}

	//copy values into redefined path variable
	delete[]path;
	path = new int*[newsize];
	for(int i = 0; i < newsize; i++)
		path[i] = new int[2];
	path = temp;

	size = newsize;
}

void Path::show(void){
	for(int i = 0; i < size; i++)
		printf("%d,%d\n",path[i][0],path[i][1]);
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
	pathDistance = new float[size-1];
	float accum = 0;
	pathDistance[size-1] = accum;
	for (int i = size-2; i >= 0; i--){
		accum += sqrt(SQ(path[i + 1][0] - path[i][0]) + SQ(path[i + 1][1] - path[i][1]));
		pathDistance[i] = accum;
	}
}

#include <stdio.h>
#define RES_X 640
#define RES_Y 480

int main(){
	char array[RES_Y][RES_X];
	for(int i = 0; i < RES_Y; i++)
		for (int j = 0; j < RES_X; j++)
			array[i][j] = 0;
	printf("i made it\n");	
	return 0;
}

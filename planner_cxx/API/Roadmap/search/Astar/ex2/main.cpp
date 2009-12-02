/*
 *
 * Main function of the example code-1: 8-puzzle
 *
 * */

#include<iostream>
#include<fstream>
#include "State.h"
#include "A_Star.h"
using namespace std;

/* reading the input from file with the name specified in filename */
int read_inp(const char *filename, int*** array, int& sx, int& sy, int& start_x, int& start_y, int& target_x, int& target_y){
	ifstream fin;
	fin.open(filename);
	fin >> sx >> sy >> start_x >> start_y >> target_x >> target_y;
	*array = new int* [sx];
	for(int i=0; i<sx; i++)
		(*array)[i] = new int[sy];
	
	for(int i=0; i<sx; i++)
		for(int j=0; j<sy; j++)
			fin >> (*array)[i][j];

	fin.close();
	return 0;
}

void usage(){ cerr << "Usage: <exe> <input file>\n"; }

int main(int argc, char *argv[]){
	int** array;
	int sx,sy,start_x,start_y,target_x,target_y;

	if(argc!=2){
		usage();
		exit(1);
	}
	
	read_inp(argv[1],&array,sx,sy,start_x,start_y,target_x,target_y);
	
	/* the initial state (the root of the a-star tree) */
	State *x = new State(array, sx, sy, start_x, start_y, target_x, target_y);
	x->get_env()->print();

	A_Star as;
	int number_of_states;
	State *sol = as.solve(x,number_of_states);

	cout << "\nThe path followed:\n";
    for(int i=0;i<number_of_states;i++)
    	sol[i].print();

	cout << "\nThe map locations that have been explored during A-star search are shown with X\n";
	sol[0].get_env()->print();
	
	return 0;
}


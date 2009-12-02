/*
 *
 * Main function of the example code-1: 8-puzzle
 *
 * */

#include<iostream>
#include "State.h"
#include "A_Star.h"
using namespace std;

void message(){
	cout << "Enter the initial configuration of the 8-puzzle\n";
	cout << "(Integer numbers in the range 0-8. 0 denotes the empty place)\n\n";
    cout << "Example:\n";
    cout << "5 6 4\n3 1 0\n8 7 2\n\n";
}

void take_input(int a[3][3]){
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			cin >> a[i][j];
	cout << "\nInput was entered successfully\n\n";
}

int main(){
	int a[3][3]; 

	message();
	take_input(a);

	State *x = new State(a);
	A_Star as;
	int number_of_states;
	State *sol = as.solve(x,number_of_states);
	for(int i=0;i<number_of_states;i++)
		sol[i].print();

	return 0;
}


/*
 *
 * State.cpp
 *
 * This is an example State class. The purpose of this
 * class is to show how to use derivation to solve 
 * the 8-puzzle problem.
 *
 * */

#include<iostream>
#include "State_Base.h"
#include "State.h"

using namespace std;

State::State(){
	calculate_f(NULL);
	last_operation = START;
}

State::~State(){}

/* The first state is initialized with this function. The content of "config"
 * is set by "array".*/
State::State(int array[3][3]){
	for(int i=0; i<3; i++)
		for(int j=0;j<3;j++){
			config[i][j] = array[i][j];
			if(config[i][j]<=0 || config[i][j]>=9){
				emp_place.x = i;
				emp_place.y = j;
			}
		}
	calculate_f(NULL);
	last_operation = START;
}

/* This function provides the next state from the state "parent" 
 * with the move of empty place in the direction "operation".
 * The function is used when branching from a node is done. */
State::State(State & parent,Direction operation){
	*this = parent;
	if(operation==UP){
		config[emp_place.x][emp_place.y] = config[emp_place.x-1][emp_place.y];
		config[emp_place.x-1][emp_place.y] = 0;
		emp_place.x = emp_place.x-1;
	}else if(operation==LEFT){
		config[emp_place.x][emp_place.y] = config[emp_place.x][emp_place.y-1];
		config[emp_place.x][emp_place.y-1] = 0;
		emp_place.y = emp_place.y-1;
	}else if(operation==DOWN){
		config[emp_place.x][emp_place.y] = config[emp_place.x+1][emp_place.y];
		config[emp_place.x+1][emp_place.y] = 0;
		emp_place.x = emp_place.x+1;
	}else if(operation==RIGHT){
		config[emp_place.x][emp_place.y] = 
		config[emp_place.x][emp_place.y+1];
		config[emp_place.x][emp_place.y+1] = 0;
		emp_place.y = emp_place.y+1;
	}else{
		cerr << "Error in State::State(State & parent,Direction operation):"
			 << "\tinappropriate operation\n";
		exit(1);
	}
	calculate_f(&parent);
	last_operation = operation;
}

/* For each branch branched_nodes are initialized with a four element array. The
 * members of the array with NULL are inappropriate nodes. These are not
 * taken into account in A_Star::solve()  */
State_Base** State::Branch(int &nodes_n){
	State_Base **branched_nodes=NULL, *state_up=NULL, *state_left=NULL, *state_down=NULL, *state_right=NULL;
	nodes_n = 4;
	if(emp_place.x>0)
		state_up = new State(*this,UP);
	if(emp_place.x<2)
		state_down = new State(*this,DOWN);
	if(emp_place.y>0)
		state_left= new State(*this,LEFT);
	if(emp_place.y<2)
		state_right = new State(*this,RIGHT);
	branched_nodes = new State_Base* [4];
	branched_nodes[0] = state_up;
	branched_nodes[1] = state_left;
	branched_nodes[2] = state_down;
	branched_nodes[3] = state_right;
	
	return branched_nodes;
}

/* Prints the state information and the last operation is 
 * shown as the move of the appropriate neighboring block 
 * of the empty space */
void State::print(){
	cout << "------------------------------\n";
	for(int i=0; i<3; i++, cout<<endl)
		for(int j=0; j<3; j++)
			cout << config[i][j] << " ";
	cout << endl << "f: " << f << "\tg: " << g << "\th: " << h << endl;
	cout << "last operation: ";
	switch(last_operation){
		case START:
			cout << "START\n"; break;
		case UP:
			cout << "DOWN\n"; break;
		case LEFT:
			cout << "RIGHT\n"; break;
		case DOWN:
			cout << "UP\n"; break;
		case RIGHT:
			cout << "LEFT\n"; break;
	}
	cout << "empty place: (" << emp_place.x << ',' << emp_place.y << ")\n\n";
} 

/* Number of steps gone so far */
int State::calculate_g(State_Base *parent){
	if(!parent)
		return g=0;
	return (g = parent->get_g() + 1);
}

/* Approximate number of displacements that will be taken to reach the goal.
 * This is the total of Mahalonobis distance of each block to its exact place
 * in the final state */
int State::calculate_h(State_Base *parent){
	int score = 0;
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			if(config[i][j]==0)
				continue;
			else {
				int x, y;
				x = static_cast<int>((config[i][j]-1)/3);
				y = (config[i][j]-1)%3;
				score += abs(x-i) + abs(y-j);
			}
		}
	h = score;
	return score;
}


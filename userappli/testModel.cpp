/*
 * testModel.cpp
 *
 *  Created on: Jul 8, 2009
 *      Author: jmainpri
 */
#include "testModel.hpp"

using namespace std;
using namespace tr1;

TestModel::TestModel() :
	nbColisionTest(10000000),
	nbLocalPathTest(1000000){
	modelRobot = new Robot(XYZ_ROBOT);
	cout << modelRobot->getName() << endl;
}

int TestModel::nbOfColisionsPerSeconds(){

	double tu,ts;
	int nbCol(0);
	ChronoOn();

	for(int i=0;;i++){
		modelRobot->shoot()->IsInCollision();
		if(i%4==1){
			ChronoTimes(&tu,&ts);
			if(tu>5){
				nbCol=i+1;
				break;
			}
		}
	}

	ChronoPrint("");
	ChronoTimes(&tu,&ts);
	ChronoOff();
	double val = (double)nbCol/tu;
	return (int)val;
}

void TestModel::distEnv(){

	shared_ptr<Configuration> q;

	for(int i=0;i<100;i++){
			q = modelRobot->shoot(true);
//			q->print();
//			cout << "Conf is In Colision = " << (int)q->IsInCollision() << endl;
			cout << "Distance from obst = " << (double)q->distEnv() << endl;
	}

	return ;
}

int TestModel::nbOfLocalPathsPerSeconds(){

	shared_ptr<Configuration> q1;
	shared_ptr<Configuration> q2;


	double tu,ts;
	ChronoOn();
	int nbLP=0;
	int nbColTest=0;
	int nbLPValid=0;

	for(int i=0;;i++){

			q1 = modelRobot->shoot();
			q2 = modelRobot->shoot();

			LocalPath LP1(q1,q2);
			q2 = LP1.configAtParam(5*p3d_get_env_dmax());

			LocalPath LP2(q1,q2);
			if(LP2.getValid()){
				nbLPValid++;
			}

			nbColTest += LP2.getNbColTest();

//			if(i%10==1){
				ChronoTimes(&tu,&ts);
				if(tu>10){
					nbLP =i+1;
					break;
//				}
			}
		}

	cout << "Nb Tested = " << nbLP << endl;
	cout << "Nb Valid = " << nbLPValid << endl;
	cout << "----------------------------------" << endl;
	cout << "nbColTest/sec = " << (double)nbColTest/10 << endl;
	cout << "nbColTest/LP = " << (double)nbColTest/(double)nbLP << endl;
	cout << "Ratio of Valid/Total = " << (double)nbLPValid/(double)nbLP << endl;

	ChronoPrint("");
	ChronoTimes(&tu,&ts);
	ChronoOff();
	double val = (double)nbLP/tu;
	return (int)val;
}

void TestModel::runAllTests(){

	cout << "StarTingTests -----------------------------" << endl;
//	cout << "nbColisionTest = " << nbColisionTest << endl;
//	cout << "nbLocalPathTest = " << nbLocalPathTest << endl;

	int colPerSec =  nbOfColisionsPerSeconds();
	int lpPerSec = nbOfLocalPathsPerSeconds();

	cout << colPerSec << " Collisions per second and " ;
	cout << lpPerSec << " localpaths per second" << endl;
	cout << " -- End tests--" << endl;

}

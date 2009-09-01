/*
 * ExpExpansionDirection.cpp
 *
 *  Created on: Jul 24, 2009
 *      Author: jmainpri
 */

#include "ExpExpansionMethod.hpp"
#include "P3d-pkg.h"
#include "../../proto/p3d_sample_proto.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <vector>

using namespace std;


shared_ptr<Configuration> ExpansiveExpansionMethod::getExpansionDirection(Node* fromComp,bool& succed){

	return selectExpansionDirection(fromComp,succed);

}

shared_ptr<Configuration> ExpansiveExpansionMethod::selectExpansionDirection(Node* expansionNode,bool& succed){

	Direction* Dir = expansionNode->getDirectionVector();

	if(Dir->getNextDirection())

	if( Dir->areDirectionsLeft() )
	{
		succed = true;
		return Dir->getNextDirection();
	}
	else
	{
		succed = false;
		shared_ptr<Configuration> ptrConf;
		return ptrConf;
	}

//	for(int i=0;i<NbDirections;i++){
//
//		if( !Dir->getTested(i) ){
//
//			if(Dir->isDirectionValid(i)){
//
//				return Dir->getLastConfTested();
//			}
//		}
//	}



	shared_ptr<Configuration> ptrNull;
	return ptrNull;
}

void ExpansiveExpansionMethod::sortDirections( vector< vector<bool> >& nCube){

	const uint Vn = static_cast<uint>(pow(2,NbDof));

	vector<uint> vertices(Vn);

	int i=0;

	for (uint x = 0;x<(Vn/2-1);x++)
	{
		vertices.at(i) = x;
		vertices.at(i+1) = Vn-x;
		i=i+2;
	}

	vector< vector<bool> > 	newNCube;

	for (uint i = 0;i<Vn;i++)
	{
		newNCube.push_back(nCube[vertices.at(i)]);
	}

	for (uint i = 0;i<Vn;i++)
	{
		cout << "vertex["<<i<<"] =  (";

		for (int j = 0;j<NbDof;j++){
			cout << " " << newNCube[i].at(j);
			if(j!=NbDof-1) {
				cout << " , ";
			}
			else{
				cout << " )" << endl;
			}
		}
	}

}

vector< vector<bool> >  ExpansiveExpansionMethod::generateAllDiagonals(){

	vector< vector<bool> > 	nCube;

	uint nbVertices = NbDirections;

	for(uint j=0;j<nbVertices;j++)
	{
		vector<bool> vertex;

		uint bit=1;

		cout << "VERTEX number " << j <<endl;
		for(uint i=0;i<NbDof;i++)
		{
			uint dimention = j&bit;
			dimention = dimention>>i;

			vertex.push_back(static_cast<bool>(dimention));

//			cout << "vertex["<<i<<"] =  "<< vertex[i] << endl;

			bit = bit<<1;
		}
		nCube.push_back(vertex);
		cout << "---------------------" << endl;
	}

	sortDirections(nCube);

	return nCube;
}

void ExpansiveExpansionMethod::generateDirectionMap(const vector< vector<bool> >& nCube){

	Robot* ptrRob = mGraph->getRobot();

	shared_ptr<Configuration> ptrRobPos = ptrRob->copyRobotPosSP();

	int idUserDof;

	if(NbDirections!=nCube.size()){
		cout << "NbDirections and HyperCube are not of the same size" << endl;
	}

	mExpansionDirection.resize(nCube.size());

	for(uint i=0;i<nCube.size();i++){

		double *newConfig = new double[ ptrRob->getNbOfDof() ];

		idUserDof=0;

		for(int k=0;k<ptrRob->getNbOfDof();k++){

			if( ptrRob->IsUserDof(k) ){

				bool Max = nCube[i].at(idUserDof);

				newConfig[k] = ptrRob->getDofBound(k,Max);

				idUserDof++;
			}
			else {
				newConfig[k] = 0;
				//ewConfig[k] = (ptrRobPos->getP3dConfigPt())[k];
			}
		}

		mExpansionDirection[i] = shared_ptr<Configuration>(new Configuration(ptrRob,newConfig));
		mExpansionDirection[i]->print();
	}
}

void ExpansiveExpansionMethod::generateDirectionMap(){

	Robot* ptrRob = mGraph->getRobot();

	bool Max1,Max2;

	int id=0;


	configPt newConfig;

	for(int i=0;i<ptrRob->getNbOfUserDof();i++){
		for(int j=i;j<ptrRob->getNbOfUserDof();j++){

			int idDof1 =  ptrRob->userDofToDof(i);
			int idDof2 =  ptrRob->userDofToDof(j);

			/*for(int l=0;l<4;l++){

				newConfig = new double(ptrRob->getNbOfDof());

				Max1 = sequence[l]->mFirst;
				Max2 = sequence[l]->mSecond;

				for(int k=0;k<ptrRob->getNbOfDof();k++){

					if( idDof1==k ){
						newConfig[k] = ptrRob->getDofBound(idDof1,Max1);
					}
					else if( idDof2==k && idDof2!=idDof1 ){
						newConfig[k] = ptrRob->getDofBound(idDof2,Max2);
					}
					else{
						newConfig[k] = 0;
					}

					cout << "newConfig["<<k<<"] = " << newConfig[k] << endl;
				}

				//				Configuration*ptrConf = new Configuration(ptrRob,newConfig);
				//				shared_ptr<Configuration> ptrConf(new Configuration(ptrRob,newConfig()));
				//				ptrConf->print();

				//				Direction* ptrDirect(new Direction(ptrConf));
				//
				//				Direct[id] = ptrDirect;
				id++;

				cout << "----------------------------------" << endl;
			}*/
		}
	}

	NbDirections = id;
}



shared_ptr<Configuration> Direction::operator+(shared_ptr<Configuration> ptrConf){

	Robot* ptrRob = ptrConf->getRobot();

	double* newConf = new double[ptrRob->getNbOfDof()];

	for(int i=0;i<ptrRob->getNbOfDof();i++){

		double dof = ptrConf->getP3dConfigPt()[i];
		double dir = mConfig->getP3dConfigPt()[i];

		if(dir==0){
			newConf[i] = dof;
		}
		else{
			newConf[i] = dof+dir;
		}
	}

	shared_ptr<Configuration> ptrNewConf(new Configuration(ptrRob,newConf));
	return ptrNewConf;
}


bool Direction::testValid(int i){

	if( staticEnv && mTested.at(i) ){

		if( mStates.at(i)==Valid )
			return true;
		else
			return false;
	}
	else{
		lastConfTested = (*this) + mDirections->at(i);

		if(lastConfTested->inCFree())
		{
			mStates.at(i) = Valid;
		}
		else
		{
			mStates.at(i) = NotValid;
		}

		mTested.at(i) = true;
	}
}

bool Direction::testNextDirection(shared_ptr<Configuration> ptrConf){

	mRobot->setAndUpdate(*ptrConf);

	testValid(mNextDirection);
}


shared_ptr<Configuration> Direction::getNextDirection(){

	shared_ptr<Configuration> newDir = mDirections->at(mNextDirection);

	mNextDirection++;

	return newDir;
}

bool Direction::areDirectionsLeft(){

	if( mNextDirection < mDirections->size() )
	{
		return true;
	}
	else
	{
		return false;
	}
}


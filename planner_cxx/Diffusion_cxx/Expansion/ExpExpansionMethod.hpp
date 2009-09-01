/*
 * TreeExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_EXP_EXPANSION_HPP
#define P3D_EXP_EXPANSION_HPP

#include "../../PlanningAPI/planningAPI.hpp"
#include "../../qtWindow/qtBase/env.hpp"
#include "BaseExpansionMethod.hpp"

#include <list>
#include <vector>

class Direction {

	enum State {
		NotValid,
		Valid,
		Tested
	};

	std::tr1::shared_ptr<Configuration> operator+(std::tr1::shared_ptr<Configuration> ptrC);

public:
	Direction(){}
	Direction(
			Robot* ptrRob,
			std::vector<State> states,
			std::vector< std::tr1::shared_ptr<Configuration> >* ptrDirect,
			std::tr1::shared_ptr<Configuration> ptrConf ):
		mStates(states),
		mDirections(ptrDirect),
		mConfig(ptrConf),
		mNextDirection(0),
		mRobot(ptrRob){
				mTested.resize(mDirections->size());
				}

	bool isDirectionValid(int i){

		if(mStates.at(i)==Valid)
			return true;
		else
			return testValid(i);
	}

	bool getTested(int i){return mTested.at(i);}
	bool testValid(int i);
	bool testNextDirection(std::tr1::shared_ptr<Configuration> ptrConf);
	std::tr1::shared_ptr<Configuration> getNextDirection();
	int getNextDirectionId() { return mNextDirection; }


	bool areDirectionsLeft();

	std::tr1::shared_ptr<Configuration> getLastConfTested(){return lastConfTested;}

private:

	bool staticEnv;

	int mNextDirection;

	Robot* mRobot;

	std::tr1::shared_ptr<Configuration> lastConfTested;
	std::tr1::shared_ptr<Configuration> mConfig;

	std::vector<bool> mTested;
	std::vector<State> mStates;

	std::vector< std::tr1::shared_ptr<Configuration> > *mDirections;

};

class ExpansiveExpansionMethod : public BaseExpansionMethod {

private:

	enum Biasing {
		squared,
		cubic,
		exponential
	};

	enum State {
		NotTested,
		NotValid,
		Valid
	};

	class BoolPair {

	public:
		BoolPair() {}
		BoolPair(bool F,bool S) : mFirst(F), mSecond(S){}

		bool mFirst;
		bool mSecond;
	};

public:

	ExpansiveExpansionMethod();
	ExpansiveExpansionMethod(Graph* prtGraph);

	~ExpansiveExpansionMethod();

	std::tr1::shared_ptr<Configuration> getExpansionDirection(Node* fromComp,bool& succed);
	std::tr1::shared_ptr<Configuration> selectExpansionDirection(Node* fromComp,bool& succed);

	Node* getExpansionNode(Node* compNode);
	Node* selectExpansionNode(Node* compNode);

private:

	double getBiasedParam(Biasing type);
	void addNewNode(Node* nodeToAdd);
	void generateDirectionMap();
	void generateDirectionMap(const std::vector< std::vector<bool> >& nCube);
	std::vector< std::vector<bool> >  generateAllDiagonals();
	void sortDirections(std::vector< std::vector<bool> >& nCube);


	Biasing typeOfBiasing;

	int NbDirections;
	int NbDof;


	Robot* mRobot;

	// EST Fields -------------------------------------------------------------
	std::vector<Node*> 		sortedNodes;
	std::vector<std::tr1::shared_ptr<Configuration> >	mExpansionDirection;

};

#endif

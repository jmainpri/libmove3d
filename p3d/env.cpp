//
// C++ Implementation: env
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "env.hpp"

using namespace std;

Env::Env() 
{
	mBoolMap.insert(boolMap_t(Env::use_p3d_structures, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isPRMvsDiffusion, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::treePlannerIsEST, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawDisabled, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawFrame, new boolContainer(true)));
  mBoolMap.insert(boolMap_t(Env::drawExploration, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::drawGraph, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawTraj, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawOTPTraj, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawTrajVector, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawAll, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawGrid, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawEntireGrid, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawLightSource, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawDistance, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawPoints, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawGaze, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawBox, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawOnlyOneLine, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawVectorField, new boolContainer(false)));
  mBoolMap.insert(boolMap_t(Env::drawMultiColorLocalpath, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::biDir, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::minimize, new boolContainer(false)));
  mBoolMap.insert(boolMap_t(Env::setActiveJointsGroup, new boolContainer(false)));
  mBoolMap.insert(boolMap_t(Env::setStompPlanner, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isCostSpace, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isPasExtWhenAct, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useDist, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::useRefiRadius, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::expandToGoal, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::expandBalanced, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::addCycles, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::expandControl, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::refinementControl, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::discardNodes, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::findLowCostConf, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isManhattan, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isMultiRRT, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::enableHri, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIPlannerTS, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIPlannerWS, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIPlannerCS, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIPlannerTRRT, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIPathDistance, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIleftArmVsRightArm, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIcameraBehindHuman, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRINoRobot, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIComputeOTP, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::HRIAutoLoadGrid, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::useHriDis, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useHriPen, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useHriNat, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::computeGrid, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::ligandExitTrajectory, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printTemp, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printRadius, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printNbQRand, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printCollFail, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printCostFail, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::debugCostOptim, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::costBeforeColl, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::costExpandToGoal, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::costThresholdRRT, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::costThresholdPlanner, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::costStarRRT, new boolContainer(false)));

	//OTP
	mBoolMap.insert(boolMap_t(Env::FastComputingRobotBase, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::DrawRobotBaseGridCosts, new boolContainer(false)));

	
	// Smoothing stage
	mBoolMap.insert(boolMap_t(Env::trajCostRecompute, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::withMaxIteration, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::withGainLimit, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::withSmoothing, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::withShortCut, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::withDeformation, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::saveTrajCost, new boolContainer(false)));
	
	mBoolMap.insert(boolMap_t(Env::withCleaning, new boolContainer(true)));
	
	mBoolMap.insert(boolMap_t(Env::useTRRT, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::isRunning, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::initPlot, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::tRrtComputeGradient, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useBoxDist, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useBallDist, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::isGoalBiased, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isInverseKinematics, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isWeightedRotation, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::randomConnectionToGoal, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::tryClosest, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::StopMultiRun, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::FKShoot, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::FKDistance, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::RecomputeCellCost, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::UseDPGGrids, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::showOneCell, new boolContainer(false)));
#ifdef MULTILOCALPATH
	mBoolMap.insert(boolMap_t(Env::plotSoftMotionCurve, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::writeSoftMotionFiles, new boolContainer(false)));
  mBoolMap.insert(boolMap_t(Env::smoothSoftMotionTraj, new boolContainer(true)));
#endif
	mBoolMap.insert(boolMap_t(Env::startWithFKCntrt, new boolContainer(false)));
	
	mIntMap.insert(intMap_t(Env::PRMType, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::nbOfSeeds, new intContainer(3)));
  mIntMap.insert(intMap_t(Env::jntToDraw, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::maxNodeCompco, new intContainer(10000)));
	mIntMap.insert(intMap_t(Env::maxNode, new intContainer(10000)));
	mIntMap.insert(intMap_t(Env::maxConnect, new intContainer(100)));
	mIntMap.insert(intMap_t(Env::NbTry, new intContainer(10000)));
	mIntMap.insert(intMap_t(Env::MaxExpandNodeFail, new intContainer(10)));
	mIntMap.insert(intMap_t(Env::MaxPassiveExpand, new intContainer(10)));
	mIntMap.insert(intMap_t(Env::DistConfigChoice, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::ExpansionNodeMethod, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::costMethodChoice, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::maxCostOptimFailures, new intContainer(1000)));
	mIntMap.insert(intMap_t(Env::nbQRand, new intContainer(0.0)));
	mIntMap.insert(intMap_t(Env::nbCostTransFailed, new intContainer(0.0)));
	mIntMap.insert(intMap_t(Env::nbCollExpanFailed, new intContainer(0.0)));
	mIntMap.insert(intMap_t(Env::nbRound, new intContainer(100)));
	mIntMap.insert(intMap_t(Env::nbMultiRun, new intContainer(10)));
	mIntMap.insert(intMap_t(Env::nbMultiSmooth, new intContainer(10)));
	mIntMap.insert(intMap_t(Env::nbCostOptimize, new intContainer(100)));
	mIntMap.insert(intMap_t(Env::nbGreedyTraj, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::test, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::akinJntId, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::heightFactor, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::progress, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::hriCostType, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::hriActiveGrid, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::hriShownGridLine, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::costDeltaMethod, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::tRrtNbtry, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::cellToShow, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::lineToShow, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::nbCells, new intContainer(30)));
	mIntMap.insert(intMap_t(Env::typeRobotBaseGrid, new intContainer(0)));
	mIntMap.insert(intMap_t(Env::setOfActiveJoints, new intContainer(-1)));
  mIntMap.insert(intMap_t(Env::currentArmId, new intContainer(-1)));
  
	mDoubleMap.insert(doubleMap_t(Env::dmax, new doubleContainer(30.)));
	mDoubleMap.insert(doubleMap_t(Env::FPS, new doubleContainer(30.)));
	mDoubleMap.insert(doubleMap_t(Env::extensionStep, new doubleContainer(6.0)));
	mDoubleMap.insert(doubleMap_t(Env::manhatRatio, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::dist, new doubleContainer(0.0)));
	mDoubleMap.insert(doubleMap_t(Env::temperatureStart, new doubleContainer(0.000001)));
	mDoubleMap.insert(doubleMap_t(Env::temperatureGoal, new doubleContainer(0.000001)));
	mDoubleMap.insert(doubleMap_t(Env::initialTemperature, new doubleContainer(0.000001)));
	mDoubleMap.insert(doubleMap_t(Env::temperatureRate, new doubleContainer(0.1)));
	mDoubleMap.insert(doubleMap_t(Env::alpha, new doubleContainer(0.5)));
	mDoubleMap.insert(doubleMap_t(Env::costStep, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::costThreshold, new doubleContainer(10.0)));
	mDoubleMap.insert(doubleMap_t(Env::zone_size, new doubleContainer(0.5)));
	mDoubleMap.insert(doubleMap_t(Env::coeffPen, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffDis, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffNat, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffJoint, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffEnerg, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffConfo, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffArmPr, new doubleContainer(0.0)));
	mDoubleMap.insert(doubleMap_t(Env::multCost, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::refiRadius, new doubleContainer(2.0)));
	
	// Optimization Variables
	mDoubleMap.insert(doubleMap_t(Env::MaxFactor, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::MinStep, new doubleContainer(2.0)));
	
	// HRI costs
	mDoubleMap.insert(doubleMap_t(Env::Kdistance, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::Kvisibility, new doubleContainer(10.0)));
	mDoubleMap.insert(doubleMap_t(Env::Kreachable, new doubleContainer(10.0)));
	mDoubleMap.insert(doubleMap_t(Env::Knatural, new doubleContainer(10.0)));
	
	mDoubleMap.insert(doubleMap_t(Env::KlengthWeight, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::visThresh, new doubleContainer(10.0)));
	mDoubleMap.insert(doubleMap_t(Env::CellSize, new doubleContainer(0.40)));
	mDoubleMap.insert(doubleMap_t(Env::PlanCellSize, new doubleContainer(0.50)));
	mDoubleMap.insert(doubleMap_t(Env::Bias, new doubleContainer(0.10)));
	mDoubleMap.insert(doubleMap_t(Env::RotationWeight, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::colorThreshold1, new doubleContainer(0.80)));
	mDoubleMap.insert(doubleMap_t(Env::colorThreshold2, new doubleContainer(1.80)));
	mDoubleMap.insert(doubleMap_t(Env::findLowCostThreshold, new doubleContainer(0.07)));
	mDoubleMap.insert(doubleMap_t(Env::bestCost, new doubleContainer(0.07)));
	mDoubleMap.insert(doubleMap_t(Env::costMax, new doubleContainer(0.0)));
	mDoubleMap.insert(doubleMap_t(Env::minimalFinalExpansionGap, new doubleContainer(10.0)));
	mDoubleMap.insert(doubleMap_t(Env::showTrajFPS, new doubleContainer(1.0)));
    mDoubleMap.insert(doubleMap_t(Env::timeOptimize, new doubleContainer(4.0)));

    mDoubleMap.insert(doubleMap_t(Env::optimalDist, new doubleContainer(1.4)));
    mDoubleMap.insert(doubleMap_t(Env::robotMaximalDist, new doubleContainer(3.0)));
    mDoubleMap.insert(doubleMap_t(Env::gazeAngle, new doubleContainer(45.0)));


    mDoubleMap.insert(doubleMap_t(Env::optimalDistFactor, new doubleContainer(0.33)));
    mDoubleMap.insert(doubleMap_t(Env::robotMaximalDistFactor, new doubleContainer(0.33)));
    mDoubleMap.insert(doubleMap_t(Env::gazeAngleFactor, new doubleContainer(0.33)));

	
#ifdef QT_LIBRARY
	mStringMap.insert(stringMap_t(Env::nameOfFile, new stringContainer("CostStat")));
	mStringMap.insert(stringMap_t(Env::numberOfCollisionPerSec, new stringContainer("0 Collision per second")));
	mStringMap.insert(stringMap_t(Env::numberOfLocalPathPerSec, new stringContainer("0 LocalPaths per second")));
	mStringMap.insert(stringMap_t(Env::numberOfCostPerSec, new stringContainer("0 Cost per second")));
	mStringMap.insert(stringMap_t(Env::ObjectToCarry, new stringContainer("Nothing")));
	mStringMap.insert(stringMap_t(Env::ActiveGrid, new stringContainer("Nothing")));
#endif
  
	mVectorMap.insert(vectorMap_t(Env::costAlongTraj, new vectorContainer()));
	
	mExpansionMethod = Connect;
}

Env::~Env() {
  for (map<boolParameter,boolContainer*>::iterator it = mBoolMap.begin();
      it !=  mBoolMap.end(); ++it ) {
    delete it->second;
  }
  for (map<intParameter,intContainer*>::iterator it = mIntMap.begin();
       it !=  mIntMap.end(); ++it ) {
    delete it->second;
  }
  for (map<doubleParameter, doubleContainer*>::iterator it = mDoubleMap.begin();
       it !=  mDoubleMap.end(); ++it ) {
    delete it->second;
  }
#ifdef QT_LIBRARY
  for (map<stringParameter, stringContainer*>::iterator it = mStringMap.begin();
       it !=  mStringMap.end(); ++it ) {
    delete it->second;
  }
#endif
  for (map<vectorParameter, vectorContainer*>::iterator it = mVectorMap.begin();
       it !=  mVectorMap.end(); ++it ) {
    delete it->second;
  }
}

int Env::getInt(intParameter p) {
	return (mIntMap[p]->get());
}

void Env::setInt(intParameter p, int v) {
	mIntMap[p]->set(v);
}

#ifdef QT_LIBRARY
QString Env::getString(stringParameter p) {
	return (mStringMap[p]->get());
}

void Env::setString(stringParameter p, QString v) {
	mStringMap[p]->set(v);
}
#endif

std::vector<double> Env::getVector(vectorParameter p) {
	return (mVectorMap[p]->get());
}

void Env::setVector(vectorParameter p, std::vector<double> v) {
	mVectorMap[p]->set(v);
}

double Env::getDouble(doubleParameter p) {
	return (mDoubleMap[p]->get());
}

void Env::setDouble(doubleParameter p, double v) {
	mDoubleMap[p]->set(v);
}

bool Env::getBool(boolParameter p) {
	return (mBoolMap[p]->get());
}

void Env::setBool(boolParameter p, bool v) {
	mBoolMap[p]->set(v);
}
#ifdef QT_LIBRARY
QObject* Env::getObject(intParameter p) {
	return (mIntMap[p]);
}

QObject* Env::getObject(boolParameter p) {
	return (mBoolMap[p]);
}

QObject* Env::getObject(doubleParameter p) {
	return (mDoubleMap[p]);
}

QObject* Env::getObject(vectorParameter p) {
	return (mVectorMap[p]);
}

QObject* Env::getObject(stringParameter p) {
	return (mStringMap[p]);
}
#endif

void Env::setExpansionMethod(expansionMethod method) {
	if (mExpansionMethod != method) {
		mExpansionMethod = method;
#ifdef QT_LIBRARY
		emit expansionMethodChanged((int) method);
#endif
	}
}

Env::expansionMethod Env::getExpansionMethod() {
	return (mExpansionMethod);
}

void Env::setExpansionMethodSlot(int method) {
	this->setExpansionMethod((expansionMethod) method);
}

Env ENV;

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

intContainer::intContainer(int v) :
	_Value(v) {
}

int intContainer::get() {
	return (_Value);
}

void intContainer::set(int v) {
	if (_Value != v) {
		_Value = v;
#ifdef QT_LIBRARY
		emit valueChanged(v);
#endif
	}
}

stringContainer::stringContainer(string v) :
	_Value(v) {
}

string stringContainer::get() {
	return (_Value);
}

void stringContainer::set(string v) {
	if (_Value != v) {
		_Value = v;
#ifdef QT_LIBRARY
		emit valueChanged(v);
#endif
	}
}

doubleContainer::doubleContainer(double v) :
	_Value(v) {
}

double doubleContainer::get() {
	return (_Value);
}

void doubleContainer::set(double v) {
	if (_Value != v) {
		_Value = v;
#ifdef QT_LIBRARY
		emit valueChanged(v);
#endif
	}
}

boolContainer::boolContainer(bool v) :
	_Value(v) {
}

bool boolContainer::get() {
	return (_Value);
}

void boolContainer::set(bool v) {
	if (_Value != v) {
		_Value = v;
#ifdef QT_LIBRARY
		emit valueChanged(v);
#endif
	}
}

Env::Env() {

	mIntMap.insert(intMap_t(Env::maxNodeCompco, new intContainer(10000)));
	mIntMap.insert(intMap_t(Env::maxNode, new intContainer(10000)));
	mIntMap.insert(intMap_t(Env::NbTry, new intContainer(10000)));
	mIntMap.insert(intMap_t(Env::MaxExpandNodeFail, new intContainer(10)));
	mIntMap.insert(intMap_t(Env::MaxPassiveExpand, new intContainer(10)));
	mIntMap.insert(intMap_t(Env::DistConfigChoice, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::ExpansionNodeMethod, new intContainer(1)));
	mIntMap.insert(intMap_t(Env::CostMethodChoice, new intContainer(1)));

	mIntMap.insert(intMap_t(Env::maxCostOptimFailures, new intContainer(1000)));
	mIntMap.insert(intMap_t(Env::nb_rounds, new intContainer(50)));
	mIntMap.insert(intMap_t(Env::nbQRand, new intContainer(0.0)));
	mIntMap.insert(intMap_t(Env::nbCostTransFailed, new intContainer(0.0)));
	mIntMap.insert(intMap_t(Env::nbCollExpanFailed, new intContainer(0.0)));
	mIntMap.insert(intMap_t(Env::nbCostOptimize, new intContainer(20)));
	mIntMap.insert(intMap_t(Env::nbGreedyTraj, new intContainer(1)));

	mDoubleMap.insert(doubleMap_t(Env::extensionStep, new doubleContainer(10.)));
	mDoubleMap.insert(doubleMap_t(Env::temperatureRate, new doubleContainer(2.)));
	mDoubleMap.insert(doubleMap_t(Env::alpha, new doubleContainer(0.5)));
	mDoubleMap.insert(doubleMap_t(Env::manhatRatio, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::dist, new doubleContainer(0.0)));

	mDoubleMap.insert(doubleMap_t(Env::temperature, new doubleContainer(0.000001)));
	mDoubleMap.insert(doubleMap_t(Env::initialTemperature, new doubleContainer(0.000001)));
	mDoubleMap.insert(doubleMap_t(Env::zone_size, new doubleContainer(300.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffPen, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffDis, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffNat, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffHei, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffLim, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::coeffTas, new doubleContainer(50.0)));
	mDoubleMap.insert(doubleMap_t(Env::multCost, new doubleContainer(1.0)));
	mDoubleMap.insert(doubleMap_t(Env::refiRadius, new doubleContainer(2.0)));
	mDoubleMap.insert(doubleMap_t(Env::MaxFactor, new doubleContainer(100.0)));
	mDoubleMap.insert(doubleMap_t(Env::MinStep, new doubleContainer(20.0)));

	mBoolMap.insert(boolMap_t(Env::drawGraph, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawTraj, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::drawAll, new boolContainer(false)));

	mBoolMap.insert(boolMap_t(Env::biDir, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::minimize, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isCostSpace, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isPasExtWhenAct, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useDist, new boolContainer(true)));

	mBoolMap.insert(boolMap_t(Env::useRefiRadius, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::expandToGoal, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::expandBalanced, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::minimize, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::addCycles, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::expandControl, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::discardNodes, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::isManhattan, new boolContainer(false)));
	
	mBoolMap.insert(boolMap_t(Env::useHriDis, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useHriPen, new boolContainer(true)));
	mBoolMap.insert(boolMap_t(Env::useHriNat, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::enableHri, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::computeGrid, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::useRefiRadius, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::ligandExitTrajectory, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printTemp, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printRadius, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printNbQRand, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printCollFail, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::printCostFail, new boolContainer(false)));
	mBoolMap.insert(boolMap_t(Env::debugCostOptim, new boolContainer(false)));

	mStringMap.insert(stringMap_t(Env::nameOfFile, new stringContainer(
			"CostStat")));

	mExpansionMethod = Extend;
}

Env::~Env() {
}

int Env::getInt(intParameter p) {
	return (mIntMap[p]->get());
}

void Env::setInt(intParameter p, int v) {
	mIntMap[p]->set(v);
}

std::string Env::getString(stringParameter p) {
	return (mStringMap[p]->get());
}

void Env::setString(stringParameter p, std::string v) {
	mStringMap[p]->set(v);
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

#ifdef QT_LIBRARY
#include "moc_env.cpp"
#endif
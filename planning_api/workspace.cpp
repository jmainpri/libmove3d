//
// C++ Implementation: workspace
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "workspace.hpp"

using namespace std;

WorkSpace::WorkSpace()
{
}

WorkSpace::WorkSpace(string nameMainEnv)
{
	if(XYZ_ROBOT == NULL){
		cout << "Warning: XYZ_ROBOT doesn't exist" << endl;
	}
	Robot* xyz_Robot = new Robot(XYZ_ROBOT);
	Environnement* BasicEnv = new Environnement("MainEnv",xyz_Robot);
	insertEnvironnement(BasicEnv);
	setActivEnvironnement("MainEnv");
}


WorkSpace::~WorkSpace()
{
}

Environnement* WorkSpace::getActivEnvironnement()
{
	for(uint i = 0; i < _Environnements.size(); i = i + 1)
	{
		if(_Environnements[i]->getName() == _activEnvironnement)
		{
			return _Environnements[i];
		}
	}
	Environnement* env = new Environnement(_activEnvironnement);
	this->insertEnvironnement(env);
	return env;
}

void WorkSpace::setActivEnvironnement(string name)
{
	_activEnvironnement = name;
	for(uint i = 0; i < _Environnements.size(); i = i + 1)
	{
		if(_Environnements[i]->getName() == _activEnvironnement)
		{
			return;
		}
	}
	Environnement* env = new Environnement(_activEnvironnement);
	this->insertEnvironnement(env);
}

void WorkSpace::insertEnvironnement(Environnement* E)
{
	_Environnements.push_back(E);
}

WorkSpace WS;


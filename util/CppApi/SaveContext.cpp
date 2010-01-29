/*
 * SaveContext.cpp
 *
 *  Created on: Sep 11, 2009
 *      Author: jmainpri
 */

#include "SaveContext.hpp"
#include <iostream>

#ifdef QT_LIBRARY
#include "../qtWindow/qtLibrary.h"
#endif

using namespace std;

SaveContext storedContext;

SaveContext::SaveContext()
{
//	clear();
//	saveCurrentEnvToStack();
	clear();
	_Time.clear();
}

SaveContext::~SaveContext()
{
	clear();
}

void SaveContext::clear()
{
	cout << "Deleting " << _MapBool.size() << " context" << endl;

	_MapBool.clear();
	_MapInt.clear();
	_MapDouble.clear();
	_MapString.clear();

	for(unsigned int i=0;i<_Time.size();i++)
	{
		_Time.at(i).clear();
	}

	_Time.clear();
}

unsigned int SaveContext::getNumberStored()
{
	return _MapBool.size();
}

unsigned int SaveContext::saveCurrentEnvToStack()
{

	_MapBool.push_back( copyMap(ENV.getBoolMap()) );
	_MapInt.push_back( copyMap(ENV.getIntMap()) );
	_MapDouble.push_back( copyMap(ENV.getDoubleMap()) );
	_MapString.push_back( copyMap(ENV.getStringMap()) );

	return getNumberStored();
}

void SaveContext::printData(unsigned int i)
{
	printVariables(i);

	cout << "_Map["<<i<<"]"<<endl;
}




void SaveContext::printVariables(unsigned int id)
{
	cout << "Bool -------------------------------" << endl;

	map<Env::boolParameter, boolContainer*>::iterator iter1 = _MapBool[id].begin();

	for(; iter1 != _MapBool[id].end(); iter1++)
	{
		cout << iter1->second->get() << endl;
	}


	cout << "Integers ---------------------------" << endl;
	map<Env::intParameter, intContainer*>::iterator iter2 = _MapInt[id].begin();
//	cout << "size " << d->getIntMap().size() << endl;

	for(; iter2 != _MapInt[id].end(); iter2++)
	{
		cout << iter2->second->get() << endl;
	}

	cout << "Doubles ----------------------------" << endl;
	map<Env::doubleParameter, doubleContainer*>::iterator iter3 = _MapDouble[id].begin();

	for(; iter3 != _MapDouble[id].end(); iter3++)
	{
		cout << iter3->second->get() << endl;
	}

#ifdef QT_LIBRARY
	cout << "String -----------------------------" << endl;
	map<Env::stringParameter, stringContainer*>::iterator iter4 = _MapString[id].begin();

	for(; iter4 != _MapString[id].end(); iter4++)
	{
                cout << iter4->second->get().toStdString() << endl;
	}
#endif
	cout << "------------ end --------------" << endl;
}


void SaveContext::switchCurrentEnvTo(unsigned int i)
{
	cout << "Switching to " <<  i << " context" << endl;
	cout << "       " <<  getNumberStored() << " are stored" << endl;

	for(map<Env::boolParameter, boolContainer*>::iterator iter = _MapBool[i].begin();
			iter != _MapBool[i].end();
			iter++)
	{
		ENV.setBool(iter->first,iter->second->get());
	}

	for(map<Env::intParameter, intContainer*>::iterator iter = _MapInt[i].begin();
			iter != _MapInt[i].end();
			iter++)
	{
		ENV.setInt(iter->first,iter->second->get());
	}

	for(map<Env::doubleParameter, doubleContainer*>::iterator iter = _MapDouble[i].begin();
			iter != _MapDouble[i].end();
			iter++)
	{
		ENV.setDouble(iter->first,iter->second->get());
	}

#ifdef QT_LIBRARY
	for(map<Env::stringParameter, stringContainer*>::iterator iter = _MapString[i].begin();
			iter != _MapString[i].end();
			iter++)
	{
		ENV.setString(iter->first,iter->second->get());
	}
#endif
}

map<Env::boolParameter, boolContainer*> SaveContext::copyMap( map<Env::boolParameter, boolContainer*> myMap )
{
	map<Env::boolParameter, boolContainer*>::iterator iter = myMap.begin();

	for(; iter != myMap.end(); iter++)
	{
//		cout << iter->second->get() << endl;
		iter->second = new boolContainer(iter->second->get());
	}

	return myMap;
}

map<Env::intParameter, intContainer*> SaveContext::copyMap( map<Env::intParameter, intContainer*> myMap )
{
	map<Env::intParameter, intContainer*>::iterator iter = myMap.begin();

	for(; iter != myMap.end(); iter++)
	{
//		cout << iter->second->get() << endl;
		iter->second = new intContainer(iter->second->get());
	}

	return myMap;
}

map<Env::doubleParameter, doubleContainer*> SaveContext::copyMap( map<Env::doubleParameter, doubleContainer*> myMap )
{
	map<Env::doubleParameter, doubleContainer*>::iterator iter = myMap.begin();

	for(; iter != myMap.end(); iter++)
	{
//		cout << iter->second->get() << endl;
		iter->second = new doubleContainer(iter->second->get());
	}

	return myMap;
}

map<Env::stringParameter, stringContainer*> SaveContext::copyMap( map<Env::stringParameter, stringContainer*> myMap )
{
	map<Env::stringParameter, stringContainer*>::iterator iter = myMap.begin();

	for(; iter != myMap.end(); iter++)
	{
//		cout << iter->second->get() << endl;
#ifdef QT_LIBRARY
		iter->second = new stringContainer(iter->second->get());
#endif
	}

	return myMap;

}


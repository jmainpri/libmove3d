/*
 * SaveContext.hpp
 *
 *  Created on: Sep 11, 2009
 *      Author: jmainpri
 */

#ifndef SAVECONTEXT_HPP_
#define SAVECONTEXT_HPP_

#include "p3d/env.hpp"
#include <vector>

class SaveContext {

public:
	SaveContext();
	~SaveContext();

	void clear();

	Env* copyVariables(Env& d);
	unsigned int getNumberStored();
	unsigned int saveCurrentEnvToStack();
	void switchCurrentEnvTo(unsigned int i);

	void printVariables(unsigned int id);
	void printData(unsigned int i);

	std::map<Env::boolParameter, boolContainer*> copyMap( std::map<Env::boolParameter, boolContainer*> map );
	std::map<Env::intParameter, intContainer*> copyMap( std::map<Env::intParameter, intContainer*> map );
	std::map<Env::doubleParameter, doubleContainer*> copyMap( std::map<Env::doubleParameter, doubleContainer*> map );
	std::map<Env::stringParameter, stringContainer*> copyMap( std::map<Env::stringParameter, stringContainer*> map );

private:
	std::vector< std::map<Env::boolParameter, boolContainer*> > _MapBool;
	std::vector< std::map<Env::intParameter, intContainer*> > _MapInt;
	std::vector< std::map<Env::doubleParameter, doubleContainer*> > _MapDouble;
	std::vector< std::map<Env::stringParameter, stringContainer*> > _MapString;
};

#endif /* SAVECONTEXT_HPP_ */

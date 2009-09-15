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

/**
 * Saves the context of run and
 * Associate a Statistic Data structure
 */
class SaveContext {

public:
	/**
	 * Constructor
	 */
	SaveContext();

	/**
	 * Destructor
	 */
	~SaveContext();

	/**
	 * Erases all contexts
	 */
	void clear();

	/**
	 * returns the number of environments
	 * Stored in the data structures
	 */
	unsigned int getNumberStored();

	/**
	 * Saves Current Environment
	 * to Stack
	 */
	unsigned int saveCurrentEnvToStack();

	/**
	 * Changes the Current Environment
	 * to a stored one
	 */
	void switchCurrentEnvTo(unsigned int i);

	/**
	 * Prints the variables of
	 * The environment
	 */
	void printVariables(unsigned int id);

	/**
	 * Print all variables of an environment
	 */
	void printData(unsigned int i);

	/**
	 * Copy constructors of the 4 type of maps
	 */
	std::map<Env::boolParameter, boolContainer*> copyMap( std::map<Env::boolParameter, boolContainer*> map );
	std::map<Env::intParameter, intContainer*> copyMap( std::map<Env::intParameter, intContainer*> map );
	std::map<Env::doubleParameter, doubleContainer*> copyMap( std::map<Env::doubleParameter, doubleContainer*> map );
	std::map<Env::stringParameter, stringContainer*> copyMap( std::map<Env::stringParameter, stringContainer*> map );

private:
	std::vector< std::map<Env::boolParameter, boolContainer*> > _MapBool;
	std::vector< std::map<Env::intParameter, intContainer*> > _MapInt;
	std::vector< std::map<Env::doubleParameter, doubleContainer*> > _MapDouble;
	std::vector< std::map<Env::stringParameter, stringContainer*> > _MapString;

//	std::vector< std::vector<Statistics*> > _Statistics;
};

extern SaveContext storedContext;

#endif /* SAVECONTEXT_HPP_ */

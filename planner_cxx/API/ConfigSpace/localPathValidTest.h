/*
 *  localPathValidTest.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef LOCALPATH_VALID_TEST_HPP
#define LOCALPATH_VALID_TEST_HPP

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

/**
 * @ingroup CONFIG_SPACE
 * \brief Tests the localPath using a colision checker
 */
class LocalPathValidTest : public LocalPath {

public:
	/**
	 * Costructors
	 */
//	LocalPathValidTest();
	LocalPathValidTest(const LocalPath& LP);
	
	/**
	 * Destructor
	 */
	~LocalPathValidTest();
	
	/**
	 * Tests the
	 */
	bool testIsValid();
	
	/**
	 *
	 */
	void setClassicTest(bool isClassic);
	
	/**
	 *
	 */
	unsigned int getNbCollisionTest();
	
	/**
	 *
	 */
	std::tr1::shared_ptr<Configuration> getLastValidConfiguration();
	
private:
	
	bool testClassic();
	bool testDichotomic();
	
	bool changePositionRobot(double l) ;
	bool changePositionRobotWithoutCntrt(double l);
	bool invalidJointLimits();
	
	/**
	 * Variables
	 */
	bool mIsClassicVsDicho;
	
	bool mDoSelf;
	bool mDoStatics;
	bool mDoOthers;
	
	bool mMicrocollisionAvoidance;
	
	std::tr1::shared_ptr<Configuration> mLastValidConfiguration;
	
	unsigned int mNbTest;

};

#endif
 
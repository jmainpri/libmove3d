/*
 *  hri_costspace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "planner_cxx/cost_space.hpp"
#include "API/ConfigSpace/configuration.hpp"

// Main cost function
extern double HRICS_getConfigCost(Configuration& Conf);

// Human kinematics
const int HRICS_HUMANj_BODY=      2;
const int HRICS_HUMANj_NECK_PAN=  5;
const int HRICS_HUMANj_NECK_TILT= 6;
const int HRICS_HUMANj_RHAND=     29; /* or 30 or 31 */
const int HRICS_HUMANj_LHAND=     26; /* or 27 or 28 */

#include "HRICS_Distance.h"
#include "HRICS_Visibility.h"
#include "HRICS_Natural.h"
#include "HRICS_Workspace.h"
#include "HRICS_ConfigSpace.h"

#ifdef HRI_PLANNER
#include "HRICS_HAMP.h"
extern HRICS::HriSpaceCost* hriSpace;
#endif

// Elementary cost maps
const int HRICS_Distance = 0;
const int HRICS_Visibility = 1;
const int HRICS_Naturality = 2;
const int HRICS_Reachability = 3;
const int HRICS_Combine = 4;

/**
 * Active Elementary Cost Spaces 
 * Object
 */
extern HRICS::Distance*		HRICS_activeDist;
extern HRICS::Visibility*	HRICS_activeVisi;
extern HRICS::Natural*		HRICS_activeNatu;
extern HRICS::Natural*		HRICS_activeReac;

/**
 * Active Motion planner framework
 */
extern HRICS::HumanAwareMotionPlanner*	HRICS_MotionPL;

/**
 * Cells to be drawn
 */ 
extern API::ThreeDCell*		BiasedCell3D;
extern API::TwoDCell*		BiasedCell2D;

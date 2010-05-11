/*
 *  HRI_CostSpace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "HRICS_Distance.h"
#include "HRICS_Visibility.h"
#include "HRICS_Natural.h"
#include "HRICS_Workspace.h"
#include "HRICS_ConfigSpace.h"


#ifdef HRI_PLANNER
#include "HRICS_HAMP.h"
extern HRICS::HriSpaceCost* hriSpace;
#endif

/**
 * Active Elementary Cost Function
 * Classes
 */
extern HRICS::Distance*		HRICS_activeDist;
extern HRICS::Visibility*	HRICS_activeVisi;
extern HRICS::Natural*		HRICS_Natural;

/**
 * Motion planner frameworks
 */
extern HRICS::Workspace*	HRICS_WorkspaceMPL;
extern HRICS::ConfigSpace*	HRICS_CSpaceMPL;

/**
 * Cells to be drawn
 */ 
extern API::ThreeDCell*		BiasedCell3D;
extern API::TwoDCell*		BiasedCell2D;


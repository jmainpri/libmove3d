/*
 *  HRI_CostSpace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "HRICS_Planner.h"
#include "HRICS_Distance.h"
#include "HRICS_CSpace.h"
#include "HRICS_Natural.h"

#ifdef HRI_PLANNER
HRICS::HriSpaceCost* hriSpace = NULL;
#endif

extern HRICS::MainPlanner* HRICS_MOPL;
extern HRICS::Distance* HRICS_activeDist;
extern HRICS::CSpace* HRICS_CSpaceMPL;
extern HRICS::Natural* HRICS_Natural;

extern API::ThreeDCell* BiasedCell3D;
extern API::TwoDCell* BiasedCell2D;
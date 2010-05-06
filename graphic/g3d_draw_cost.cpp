/*
 *  g3d_draw_cost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#ifdef HRI_COSTSPACE
#include "../planner_cxx/HRI_CostSpace/HRICS_costspace.h"
#endif

#include <iostream>
#include <tr1/memory>

using namespace std;
using namespace tr1;

/**
 * @ingroup graphics
 * Draws the things related to cost spaces
 */
std::vector<double> vect_jim;

void g3d_draw_costspace()
{

	if((ENV.getBool(Env::drawDistance)||ENV.getBool(Env::HRIPlannerWS)) && ENV.getBool(Env::drawDistance))
	{
#ifdef HRI_COSTSPACE
		vect_jim = HRICS_activeDist->getVectorJim();
#endif

		for (unsigned int i = 0; i < vect_jim.size() / 6; i++)
		{
			g3d_drawOneLine(vect_jim[0 + 6 * i], vect_jim[1 + 6 * i],
							vect_jim[2 + 6 * i], vect_jim[3 + 6 * i],
							vect_jim[4 + 6 * i], vect_jim[5 + 6 * i], Red, NULL);
		}
	}
#ifdef HRI_COSTSPACE	
	if ( ENV.getBool(Env::drawGaze) && ENV.getBool(Env::HRIPlannerWS) )
	{
		vector<double> Gaze;
		Gaze.clear();
		
		cout << "Draw Gaze" << endl;
		
		Gaze = HRICS_WorkspaceMPL->getVisibility()->getGaze();
		
		if( (Gaze.size() == 6))
		{		
			g3d_drawOneLine(Gaze[0], Gaze[1],
							Gaze[2], Gaze[3],
							Gaze[4], Gaze[5], Red, NULL);
		}
	}
#endif
}

void g3d_draw_grids()
{
#ifdef HRI_COSTSPACE
	if( ENV.getBool(Env::drawGrid) && API_activeGrid )
	{
		API_activeGrid->draw();
	}
	
	if( ENV.getBool(Env::drawPoints) )
	{
		if(PointsToDraw)
		{
			PointsToDraw->drawAllPoints();
		}
	}
#endif
}

/**
 * @ingroup graphics
 * Draws the thing related to HRI_COSTSPACE
 */
#ifdef HRI_COSTSPACE
void g3d_draw_hrics()
{
	if( ENV.getBool(Env::enableHri) )
	{
		if( ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::drawTraj) )
		{
			//          printf("Draw 2d path\n");
			HRICS_CSpaceMPL->draw2dPath();
		}
	}
	
	if( ENV.getBool(Env::isCostSpace) )
	{
		if( ENV.getBool(Env::enableHri) )
		{
			if( ENV.getBool(Env::HRIPlannerWS) && ENV.getBool(Env::drawTraj) )
			{
				//              printf("Draw 3d path\n");
				HRICS_WorkspaceMPL->draw3dPath();
			}
		}
		else
		{
			if( ENV.getBool(Env::isCostSpace) )
			{
				for (int num = 0; num < 2; num++)
				{
					for (int it = 0; it < 3; it++)
					{
#ifdef P3D_COLLISION_CHECKING
						if (vectMinDist[num][it] != 0)
						{
							g3d_drawOneLine(vectMinDist[0][0],
											vectMinDist[0][1], vectMinDist[0][2],
											vectMinDist[1][0], vectMinDist[1][1],
											vectMinDist[1][2], Red, NULL);
							break;
						}
#endif
					}
				}
			}
		}
	}
}
#endif
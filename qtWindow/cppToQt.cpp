/*
 * cppToQt.cpp
 *
 *  Created on: Aug 19, 2009
 *      Author: jmainpri
 *
 *      This file implements a pipe to read out commands from qt and
 *      pass it on to the XForms thread, the main loop runs in a
 *      distinct thread using X11 and XForms doesn't permit qt's thread to act upon X11
 *      at the same time (causing a segmentation fault).
 *
 *      This can be solved by passing the 3D display to Ogre or having the OpenGl in a
 *      Qt window.
 */

#include <string>
#include <iostream>

#include "../qtWindow/cppToQt.hpp"

using namespace std;

void read_pipe(int fd, void* data)
{
	char buffer[256];
	while (read(fd, buffer, sizeof(buffer)) > 0)
		;
	printf("Qt to XForms => %s\n", buffer);

	string bufferStr(buffer);

	cout << bufferStr << endl;

	if (bufferStr.compare("p3d_RunDiffusion") == 0)
	{
		p3d_SetStopValue(FALSE);
		int res = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
		if (res)
		{
			if(ENV.getBool(Env::isCostSpace))
			{
					p3d_ExtractBestTraj(XYZ_GRAPH);
			}
			else
			{
				if (p3d_graph_to_traj(XYZ_ROBOT))
				{
					g3d_add_traj((char*) "Globalsearch", p3d_get_desc_number(
							P3D_TRAJ));
				}
				else
				{
					printf("Problem during trajectory extraction\n");


				}
			}
			g3d_draw_allwin_active();
			return;
		}
	}
	if (bufferStr.compare("p3d_RunGreedy") == 0)
	{
		p3d_SetStopValue(FALSE);
		p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
		return;
	}
	if (bufferStr.compare("p3d_RunEST") == 0)
	{
		p3d_SetStopValue(FALSE);
		//  	  p3d_RunEST(XYZ_GRAPH, fct_stop, fct_draw);
		return;
	}
	if (bufferStr.compare("ResetGraph") == 0)
	{
		p3d_del_graph(XYZ_GRAPH);
		MY_ALLOC_INFO("After the graph destruction");
		g3d_draw_allwin_active();
		return;
	}
	if (bufferStr.compare("g3d_draw_allwin_active") == 0)
	{
		g3d_draw_allwin_active();
		return;
	}
	if (bufferStr.compare("optimize") == 0)
	{

		p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
		p3d_traj* CurrentTrajPt = robotPt->tcur;

		if (robotPt->tcur == NULL)
		{
			return;
		}

		//	p3d_SetIsCostFuncSpace(TRUE);

		CostOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);

		for (int i = 0; i < ENV.getInt(Env::nbCostOptimize); i++)
		{
			optimTrj.oneLoopDeform(ENV.getDouble(Env::MinStep));
//			optimTrj.removeRedundantNodes();
			optimTrj.replaceP3dTraj(CurrentTrajPt);
			g3d_draw_allwin_active();
		}

		if (CurrentTrajPt == NULL)
		{
			PrintInfo(("Warning: no current trajectory to optimize\n"));
		}
		return;
	}

	  if(bufferStr.compare("oneStepOptim") == 0 )
	  {

	  	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	  	p3d_traj* CurrentTrajPt = robotPt->tcur;

//	  	p3d_SetIsCostFuncSpace(TRUE);

	  	CostOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);

	  	optimTrj.oneLoopDeform(20);
	  //		optimTrj.removeRedundantNodes();
	  	optimTrj.replaceP3dTraj(CurrentTrajPt);
	  	g3d_draw_allwin_active();


	  	if(CurrentTrajPt == NULL){
	  		PrintInfo(("Warning: no current trajectory to optimize\n"));
	  		}
	  	return;
	  	}

	if (bufferStr.compare("shortCut") == 0)
	{

		p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
		p3d_traj* CurrentTrajPt = robotPt->tcur;

		BaseOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);

			for(int i=0;i<ENV.getInt(Env::nbCostOptimize);i++){
				optimTrj.oneLoopShortCut();
				optimTrj.replaceP3dTraj(CurrentTrajPt);
				g3d_draw_allwin_active();
				}

		if (CurrentTrajPt == NULL)
		{
			PrintInfo(("Warning: no current trajectory to optimize\n"));
		}
		return;
	}
	else
	{
		printf("Error, pipe not implemented\n");
	}

}

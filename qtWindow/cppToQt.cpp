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

/**
 * C++ basic headers (they all have name spaces)
 */
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <string>
//#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <tr1/memory>

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"

#include "../qtWindow/cppToQt.hpp"

#ifdef CXX_PLANNER
#include "../util/CppApi/SaveContext.hpp"
#endif

using namespace std;
using namespace tr1;

// -------------------------------------------------------------
// ------------------   Pipe  ----------------------------------
// -------------------------------------------------------------

void read_pipe(int fd, void* data)
{
    char buffer[256];
    while (read(fd, buffer, sizeof(buffer)) > 0);
    //	printf("Qt to XForms => %s\n", buffer);

    string bufferStr(buffer);

    //	cout << bufferStr << endl;
	//cout << "Reading Pipe!!!!!!!" << endl;

    if (bufferStr.compare("ResetGraph") == 0)
    {
		qt_resetGraph();
		return;
    }

    if (bufferStr.compare("g3d_draw_allwin_active") == 0)
    {
		qt_drawAllWinActive();
		return;
    }

    if (bufferStr.compare("RunDiffusion") == 0)
    {
		qt_runDiffusion();
		return;
    }

    if (bufferStr.compare("RunPRM") == 0)
    {
		qt_runPRM();
		return;
    }
	
	if (bufferStr.compare("shortCut") == 0)
    {
		qt_shortCut();
		return;
    }
	
	if(bufferStr.compare("readP3DScenarion") == 0 )
    {
		qt_readScenario();
		return;
    }


    if (bufferStr.compare("p3d_RunGreedy") == 0)
    {
#ifdef P3D_PLANNER
        p3d_SetStopValue(FALSE);
#ifdef CXX_PLANNER
        p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
#endif
#endif
        return;
    }

    if (bufferStr.compare("MultiSmooth") == 0)
    {
#ifdef CXX_PLANNER
        MultiRun multiSmooths;
        multiSmooths.runMutliSmooth();
#endif
        return;
    }

    if (bufferStr.compare("MultiRRT") == 0)
    {
#ifdef CXX_PLANNER
        MultiRun multiRRTs;
        multiRRTs.runMutliRRT();
#endif
        return;
    }

    if (bufferStr.compare("optimize") == 0)
    {
//        if(ENV.getBool(Env::isRunning))
//        {
//            cout << "Warning : Planner is Running "  << endl;
//            return;
//        }

        ENV.setBool(Env::isRunning,true);
        p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
        p3d_traj* CurrentTrajPt = robotPt->tcur;

        if (robotPt->tcur == NULL)
        {
            cout << "No robotPt->tcur to Smooth"  << endl;
            return;
        }
#ifdef CXX_PLANNER
        Robot trajRobot(robotPt);
        CostOptimization optimTrj(&trajRobot,CurrentTrajPt);
        optimTrj.runDeformation(ENV.getInt(Env::nbCostOptimize));
        optimTrj.replaceP3dTraj();
#endif
        g3d_draw_allwin_active();
        ENV.setBool(Env::isRunning,false);
        return;
    }



    if (bufferStr.compare("oneStepOptim") == 0)
    {
        p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
        p3d_traj* CurrentTrajPt = robotPt->tcur;

        //	  	p3d_SetIsCostFuncSpace(TRUE);
#ifdef CXX_PLANNER
        Robot* trajRobot = new Robot(robotPt);
        CostOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);

        optimTrj.oneLoopDeform(20);
        //		optimTrj.removeRedundantNodes();
        optimTrj.replaceP3dTraj(CurrentTrajPt);
#endif
        g3d_draw_allwin_active();

        if (CurrentTrajPt == NULL)
        {
            PrintInfo(("Warning: no current trajectory to optimize\n"));
        }
        return;
    }


    if (bufferStr.compare("removeRedunantNodes") == 0)
    {
        p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
        p3d_traj* CurrentTrajPt = robotPt->tcur;

        if (CurrentTrajPt == NULL)
        {
            PrintInfo(("Warning: no current trajectory to optimize\n"));
        }
#ifdef CXX_PLANNER
        Robot* trajRobot = new Robot(robotPt);
        CostOptimization optimTrj(trajRobot,CurrentTrajPt);
        delete trajRobot;

        optimTrj.removeRedundantNodes();
        optimTrj.replaceP3dTraj(CurrentTrajPt);
        g3d_draw_allwin_active();

        if (optimTrj.getValid())
        {
            cout << "Trajectory valid" << endl;
        }
        else
        {
            cout << "Trajectory not valid" << endl;
        }
#endif

        return;
    }

//    if (bufferStr.compare("graphSearchTest") == 0)
//    {
//        //		Dijkstra graphS;
//        //		graphS.example();
//
//        Graph* ptrGraph = new Graph(XYZ_GRAPH);
//
//        Dijkstra graphS(ptrGraph);
//
//        //		int start = 1;
//        //		int goal = (int)ptrGraph->getNbNode()/2;
//
//        shared_ptr<Configuration> Init =
//                ptrGraph->getRobot()->getInitialPosition();
//        shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();
//
//        Trajectory* traj = graphS.extractTrajectory(Init, Goal);
//        traj->replaceP3dTraj();
//
//        g3d_draw_allwin_active();
//        return;
//    }

#ifdef HRI_COSTSPACE

    if (bufferStr.compare("computeWorkspacePath") == 0)
    {
#ifdef HRI_PLANNER
        hriSpace->computeWorkspacePath();
#else
		cout << "HRI_PLANNER not compiled" << endl;
#endif
        g3d_draw_allwin_active();
        return;
    }

    if (bufferStr.compare("computeHoleManipulationPath") == 0)
    {
#ifdef HRI_PLANNER
        hriSpace->computeHoleManipulationPath();
#else
		cout << "HRI_PLANNER not compiled" << endl;
#endif
        g3d_draw_allwin_active();
        return;
    }

    if (bufferStr.compare("runHRICSRRT") == 0)
    {
//        if( HRICS_MOPL->runHriRRT() )
//        {
//            Trajectory optimTrj(new Robot(XYZ_ROBOT),XYZ_ROBOT->tcur);
//            if( !optimTrj.getValid() )
//            {
//                cout << "Trajector NOT VALID!!!"  << endl;
//            }
//            cout << "Trajectory mean coll test : "  << optimTrj.meanCollTest() << endl;
//        }
//        ENV.setBool(Env::drawTraj,true);
//        g3d_draw_allwin_active();
//        return;
    }

#endif

    //    if( )
    //    {
    //
    //    }

    else
    {
        printf("Error, pipe not implemented\n");
//#ifdef CXX_PLANNER
//        Graph* ptrGraph = new Graph(XYZ_GRAPH);
//#endif
    }

}

// -------------------------------------------------------------
// ------------------   Callback functions  --------------------
// -------------------------------------------------------------
/**
 * Reset Graph
 */
void qt_resetGraph()
{
#ifdef P3D_PLANNER
	if(XYZ_GRAPH)
	{
		p3d_del_graph(XYZ_GRAPH);
	}
#endif
}

/**
 * Draw All Win Active
 */
void qt_drawAllWinActive()
{
	g3d_draw_allwin_active();
}

/**
 * Run Diffusion
 */
void qt_runDiffusion()
{
	cout << "Diffusion" << endl;
	
#ifdef P3D_PLANNER
	p3d_SetStopValue(FALSE);
#endif
	ChronoOn();
	
	int res;
	cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
	if (ENV.getBool(Env::treePlannerIsEST))
	{
#ifdef CXX_PLANNER
		res = p3d_run_est(XYZ_GRAPH, fct_stop, fct_draw);
	}
	else
	{
		res = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
#endif
	}
	
	ChronoPrint("");
	ChronoOff();
	
	g3d_draw_allwin_active();
}

/**
 * Run PRM
 */
void qt_runPRM()
{
#ifdef P3D_PLANNER
	p3d_SetStopValue(FALSE);
#endif
	
	int res;
	int fail;
	
	ChronoOn();
	
	//        cout << "ENV.getInt(Env::PRMType)  = "  << ENV.getInt(Env::PRMType) << endl;
	
	switch(ENV.getInt(Env::PRMType))
	{
#ifdef CXX_PLANNER
        case 0:
            res = p3d_run_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);
            break;
        case 1:
            res = p3d_run_vis_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);
            break;
        case 2:
            res = p3d_run_acr(XYZ_GRAPH, &fail, fct_stop, fct_draw);
            break;
#endif
        default:
            cout << "Error No Other PRM"  << endl;
            ChronoPrint("");
            ChronoOff();
            return;
	}
	
	
	ChronoPrint("");
	ChronoOff();
	
	if (ENV.getBool(Env::expandToGoal))
	{
		if (res)
		{
			if (ENV.getBool(Env::isCostSpace))
			{
#ifdef P3D_PLANNER
				p3d_ExtractBestTraj(XYZ_GRAPH);
#endif
			}
			else
			{
				if (p3d_graph_to_traj(XYZ_ROBOT))
				{
					g3d_add_traj((char*) "Globalsearch",
								 p3d_get_desc_number(P3D_TRAJ));
				}
				else
				{
					printf("Problem during trajectory extraction\n");
				}
			}
			g3d_draw_allwin_active();
		}
	}
	
	ENV.setBool(Env::isRunning,false);
}

/**
 * Short Cut
 */
void qt_shortCut()
{
	//        if(ENV.getBool(Env::isRunning))
	//        {
	//            cout << "Warning : Planner is Running "  << endl;
	//            return;
	//        }
	ENV.setBool(Env::isRunning,true);
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	if (robotPt->tcur == NULL)
	{
		cout << "No robotPt->tcur to Smooth"  << endl;
		return;
	}
#ifdef CXX_PLANNER
	Robot trajRobot(robotPt,false);
	
	Smoothing optimTrj(&trajRobot,
					   trajRobot.getTrajStruct());
	
	optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
	optimTrj.replaceP3dTraj();
#endif
	g3d_draw_allwin_active();
	ENV.setBool(Env::isRunning,false);
}

/**
 * Read Scenario
 */
void qt_readScenario()
{
	std::string fileToOpen(qt_fileName);
	cout <<" Should Open scenarion " << fileToOpen << endl;
	
	p3d_rw_scenario_init_name();
	p3d_read_scenario(qt_fileName);
}

/**
 * Save Scenario
 */
void qt_saveScenario()
{
	std::string fileToOpen(qt_fileName);
	cout <<" Should Open scenarion " << fileToOpen << endl;
	
	p3d_rw_scenario_init_name();
	p3d_save_scenario(qt_fileName);
}

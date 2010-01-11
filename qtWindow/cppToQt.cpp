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

#ifdef CXX_PLANNER
#include "../userappli/CppApi/SaveContext.hpp"
#endif

using namespace std;
using namespace tr1;

void save_vector_to_file(vector<vector<double> >& vectDoubles,
                         vector<string> name)
{
    std::ostringstream oss;
    oss << "statFiles/"<< ENV.getString(Env::nameOfFile).toStdString() << ".csv";

    const char *res = oss.str().c_str();

    std::ofstream s;
    s.open(res);

    cout << "Opening save file" << endl;

    for (unsigned int i = 0; i < name.size(); i++)
    {
        s << name.at(i) << ",";
    }

    s << endl;

    for (unsigned int j = 0; j < vectDoubles[0].size(); j++)
    {
        for (unsigned int i = 0; i < vectDoubles.size(); i++)
        {
            s << vectDoubles[i][j] << ",";
        }
        s << endl;
    }

    cout << "Closing save file" << endl;

    s.close();
}

void read_pipe(int fd, void* data)
{
    char buffer[256];
    while (read(fd, buffer, sizeof(buffer)) > 0)
        ;
    //	printf("Qt to XForms => %s\n", buffer);

    string bufferStr(buffer);

    //	cout << bufferStr << endl;

    if (bufferStr.compare("ResetGraph") == 0)
    {
        if(XYZ_GRAPH)
        {
            p3d_del_graph(XYZ_GRAPH);
        }
//        MY_ALLOC_INFO("After the graph destruction");
        g3d_draw_allwin_active();
        return;
    }

    if (bufferStr.compare("g3d_draw_allwin_active") == 0)
    {
        g3d_draw_allwin_active();
        return;
    }

    if (bufferStr.compare("RunDiffusion") == 0)
    {
        p3d_SetStopValue(FALSE);

        int res;
        cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
        if (ENV.getBool(Env::treePlannerIsEST))
        {
            res = p3d_run_est(XYZ_GRAPH, fct_stop, fct_draw);
        }
        else
        {
            res = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
        }

        if (res)
        {
            if (ENV.getBool(Env::isCostSpace))
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
            if(ENV.getBool(Env::withShortCut))
            {
                ENV.setBool(Env::isRunning,true);
                p3d_traj* CurrentTrajPt = XYZ_ROBOT->tcur;
                BaseOptimization optimTrj(new Robot(XYZ_ROBOT),XYZ_ROBOT->tcur);
                optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
                optimTrj.replaceP3dTraj();
                ENV.setBool(Env::isRunning,false);
            }
            g3d_draw_allwin_active();
            return;
        }
    }

    if (bufferStr.compare("RunPRM") == 0)
    {
        p3d_SetStopValue(FALSE);

        int res;
        int fail;

        res = p3d_run_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);

        if (ENV.getBool(Env::expandToGoal))
        {
            if (res)
            {
                if (ENV.getBool(Env::isCostSpace))
                {
                    p3d_ExtractBestTraj(XYZ_GRAPH);
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
        return;
    }


    if (bufferStr.compare("p3d_RunGreedy") == 0)
    {
        p3d_SetStopValue(FALSE);
        p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
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

        cout << "Traj cost : " << optimTrj.cost() << endl;

        if (CurrentTrajPt == NULL)
        {
            PrintInfo(("Warning: no current trajectory to optimize\n"));
        }
        return;
    }

    if (bufferStr.compare("oneStepOptim") == 0)
    {

        p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
        p3d_traj* CurrentTrajPt = robotPt->tcur;

        //	  	p3d_SetIsCostFuncSpace(TRUE);

        CostOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);

        optimTrj.oneLoopDeform(20);
        //		optimTrj.removeRedundantNodes();
        optimTrj.replaceP3dTraj(CurrentTrajPt);
        g3d_draw_allwin_active();

        if (CurrentTrajPt == NULL)
        {
            PrintInfo(("Warning: no current trajectory to optimize\n"));
        }
        return;
    }

    if (bufferStr.compare("shortCut") == 0)
    {
        p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
        p3d_traj* CurrentTrajPt = robotPt->tcur;

        BaseOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);

        optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
        optimTrj.replaceP3dTraj();
        g3d_draw_allwin_active();
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

        CostOptimization optimTrj(new Robot(robotPt),CurrentTrajPt);
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

        return;
    }

    if (bufferStr.compare("MultiRRT") == 0)
    {
        cout << "Running Multi RRT" << endl;
        if (storedContext.getNumberStored() == 0)
        {
            cout << "WARNING: No context on the context stack" << endl;
            return;
        }

        vector<double> time;

        vector<vector<double> > vectDoubles;
        vector<string> names;

        names.push_back("Time");
        names.push_back("Cost");

        for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
        {
            vectDoubles.resize(2);
            storedContext.switchCurrentEnvTo(j);
            //			storedContext.getTime(j).clear();
            //			vector<double> time = storedContext.getTime(j);
            for (int i = 0; i < ENV.getInt(Env::nbRound); i++)
            {
                double tu(0.0);
                double ts(0.0);
                ChronoOn();

                p3d_SetStopValue(FALSE);
                int res = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
                if (res)
                {
                    if (ENV.getBool(Env::isCostSpace))
                    {

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
                }

                ChronoPrint("");
                ChronoTimes(&tu, &ts);
                ChronoOff();
                time.push_back(tu);

                bool isRRT;
                if (ENV.getBool(Env::isCostSpace))
                {
                    isRRT = false;
                }
                else
                {
                    isRRT = true;
                }

                ENV.setBool(Env::isCostSpace,true);

                p3d_rob *robotPt =
                        (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

                p3d_graph_to_traj(robotPt);
                p3d_traj* CurrentTrajPt = robotPt->tcur;

                Trajectory optimTrj(
                        new Robot(robotPt),
                        CurrentTrajPt);

                vectDoubles[0].push_back(tu);
                vectDoubles[1].push_back(optimTrj.cost());

                if(isRRT)
                {
                    ENV.setBool(Env::isCostSpace,false);
                }
                p3d_del_graph(XYZ_GRAPH);
            }
            storedContext.addTime(time);
            cout << "Save to file" << endl;
            save_vector_to_file(vectDoubles, names);
        }

        cout << " End of Tests ----------------------" << endl;

        return;
    }

    if (bufferStr.compare("MultiGreedy") == 0)
    {

        if (storedContext.getNumberStored() == 0)
        {
            cout << "WARNING: No context on the context stack" << endl;
            return;
        }

        vector<double> time;
        vector<double> cost;

        vector<vector<double> > vectDoubles;
        vector<string> names;

        names.push_back("Time");
        names.push_back("Cost");

        for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
        {
            storedContext.switchCurrentEnvTo(j);
            //			storedContext.getTime(j).clear();
            //			vector<double> time = storedContext.getTime(j);
            for (int i = 0; i < ENV.getInt(Env::nbRound); i++)
            {
                vectDoubles.resize(2);
                double tu(0.0);
                double ts(0.0);
                ChronoOn();

                p3d_SetStopValue(FALSE);
                int res = p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
                ChronoPrint("");
                ChronoTimes(&tu, &ts);
                ChronoOff();
                p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
                p3d_traj* CurrentTrajPt = robotPt->tcur;
                if (CurrentTrajPt == NULL)
                {
                    PrintInfo(("Warning: no current trajectory to optimize\n"));
                }
                Trajectory optimTrj(new Robot(robotPt),CurrentTrajPt);
                vectDoubles[0].push_back(tu);
                vectDoubles[1].push_back(optimTrj.cost());
                time.push_back(tu);
            }
            storedContext.addTime(time);
            save_vector_to_file(vectDoubles, names);
        }

        cout << " End of Tests ----------------------" << endl;

        return;
    }

    if (bufferStr.compare("graphSearchTest") == 0)
    {
        //		Dijkstra graphS;
        //		graphS.example();

        Graph* ptrGraph = new Graph(XYZ_GRAPH);

        Dijkstra graphS(ptrGraph);

        //		int start = 1;
        //		int goal = (int)ptrGraph->getNbNode()/2;

        shared_ptr<Configuration> Init =
                ptrGraph->getRobot()->getInitialPosition();
        shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();

        Trajectory* traj = graphS.extractTrajectory(Init, Goal);
        traj->replaceP3dTraj();

        g3d_draw_allwin_active();
        return;
    }

#ifdef HRI_COSTSPACE

    if (bufferStr.compare("computeWorkspacePath") == 0)
    {
        hriSpace->computeWorkspacePath();
        g3d_draw_allwin_active();
        return;
    }

    if (bufferStr.compare("computeHoleManipulationPath") == 0)
    {
        hriSpace->computeHoleManipulationPath();
        g3d_draw_allwin_active();
        return;
    }

    if (bufferStr.compare("runHRICSRRT") == 0)
    {
        HRICS_MOPL->runHriRRT();
        ENV.setBool(Env::drawTraj,true);
        g3d_draw_allwin_active();
        return;
    }

#endif

//    if( )
//    {
//
//    }


    else
    {
        printf("Error, pipe not implemented\n");
        Graph* ptrGraph = new Graph(XYZ_GRAPH);
    }

}

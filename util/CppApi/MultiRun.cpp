#include "MultiRun.hpp"
#include "SaveContext.hpp"
#include "../../planner_cxx/planners_cxx.hpp"
#include "../../planner_cxx/plannerFunctions.hpp"
#include "../../planner_cxx/Greedy/GreedyCost.hpp"

using namespace std;
using namespace tr1;

MultiRun::MultiRun()
{
}

/**
  * Saves Cost and Time in a file under the names
  */
void MultiRun::saveVectorToFile()
{
    std::ostringstream oss;
    oss << "statFiles/"<< ENV.getString(Env::nameOfFile).toStdString() << ".csv";

    const char *res = oss.str().c_str();

    std::ofstream s;
    s.open(res);

    cout << "Opening save file : " << res << endl;

    for (unsigned int i = 0; i < mNames.size(); i++)
    {
        s << mNames.at(i) << ";";
    }

    s << endl;

    for (unsigned int j = 0; j < mVectDoubles[0].size(); j++)
    {
        for (unsigned int i = 0; i < mVectDoubles.size(); i++)
        {
            s << mVectDoubles[i][j] << ";";
        }
        s << endl;
    }

    cout << "Closing save file" << endl;

    s.close();
}

/**
  * Run multiple RRT runs
  */
void MultiRun::runMutliRRT()
{
    cout << "Running Multi RRT" << endl;
    if (storedContext.getNumberStored() == 0)
    {
        cout << "WARNING: No context on the context stack" << endl;
        return;
    }
    else
    {
        cout << " Running " << storedContext.getNumberStored() << " RRTs from stored context" << endl;
    }

    mNames.clear();
    mVectDoubles.clear();
    mTime.clear();

    mNames.push_back("Time");
    mNames.push_back("NbQRand");
    mNames.push_back("NbNodes");
    mNames.push_back("Integral");
    mNames.push_back("Meca-Work");

    for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
    {
        mVectDoubles.resize(5);
        storedContext.switchCurrentEnvTo(j);
        // storedContext.getTime(j).clear();
        // vector<double> time = storedContext.getTime(j);
        for (int i = 0; i < ENV.getInt(Env::nbMultiRun); i++)
        {
            double tu(0.0);
            double ts(0.0);
            ChronoOn();

            p3d_SetStopValue(FALSE);

            int nbNodes = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);

            if( nbNodes > 0)
            {
                if (ENV.getBool(Env::isCostSpace))
                {
                    p3d_ExtractBestTraj(XYZ_GRAPH);
                }
                else
                {
                    if (p3d_graph_to_traj(XYZ_ROBOT))
                    {
                        // g3d_add_traj((char*) "Globalsearch",
                        // p3d_get_desc_number(P3D_TRAJ));
                    }
                    else
                    {
                        cout << "Warning : Problem during trajectory extraction"  << endl;

                    }
                }
//                bool isRRT(!ENV.getBool(Env::isCostSpace));
//                ENV.setBool(Env::isCostSpace,true);

                p3d_rob *robotPt =
                        (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

                BaseOptimization optimTrj(
                        new Robot(robotPt),
                        robotPt->tcur);

                if(ENV.getBool(Env::withShortCut))
                {
                    optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
                    optimTrj.replaceP3dTraj();
                }

                ChronoPrint("");
                ChronoTimes(&tu, &ts);
                ChronoOff();
                mTime.push_back(tu);
                mVectDoubles[0].push_back(tu);

                mVectDoubles[1].push_back( ENV.getInt(Env::nbQRand) );
                mVectDoubles[2].push_back( nbNodes );


                ENV.setInt(Env::costDeltaMethod,INTEGRAL);
                mVectDoubles[3].push_back( optimTrj.cost() );

                ENV.setInt(Env::costDeltaMethod,MECHANICAL_WORK);
                mVectDoubles[4].push_back( optimTrj.cost() );

                cout << " Mean Collision test : "  << optimTrj.meanCollTest() << endl;
                g3d_draw_allwin_active();
                if(ENV.getBool(Env::StopMultiRun))
                    break;

            }
            else
            {
                cout << "Error : No traj Found " << endl;
                return;
            }

            saveGraph(i);

//            if(isRRT)
//            {
//                ENV.setBool(Env::isCostSpace,false);
//            }
            p3d_del_graph(XYZ_GRAPH);
        }
        storedContext.addTime(mTime);
        saveVectorToFile();
        ENV.setBool(Env::StopMultiRun,false);
        cout << "Save to file" << endl;
    }

    cout << " End of Tests ----------------------" << endl;

    return;
}

void MultiRun::runMutliGreedy()
{

    if (storedContext.getNumberStored() == 0)
    {
        cout << "WARNING: No context on the context stack" << endl;
        return;
    }
    else
    {
        cout << " Running " << storedContext.getNumberStored() << " RRTs from stored context" << endl;
    }

    mNames.clear();
    mVectDoubles.clear();
    mTime.clear();

    mNames.push_back("Time");
    mNames.push_back("NbQRand");
    mNames.push_back("NbNodes");
    mNames.push_back("Integral");
    mNames.push_back("Meca-Work");

    mVectDoubles.resize(5);

    for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
    {
        storedContext.switchCurrentEnvTo(j);
        //			storedContext.getTime(j).clear();
        //			vector<double> time = storedContext.getTime(j);
        for (int i = 0; i < ENV.getInt(Env::nbMultiRun); i++)
        {

            double tu(0.0);
            double ts(0.0);
            ChronoOn();

            p3d_SetStopValue(FALSE);
            /*int res = */p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
            ChronoPrint("");
            ChronoTimes(&tu, &ts);
            ChronoOff();

            p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
            p3d_traj* CurrentTrajPt = robotPt->tcur;
            if (CurrentTrajPt == NULL)
            {
                PrintInfo(("Warning: no current trajectory to optimize\n"));
                continue;
            }

            BaseOptimization optimTrj(
                    new Robot(robotPt),
                    robotPt->tcur);

            mTime.push_back(tu);
            mVectDoubles[0].push_back(tu);

            mVectDoubles[1].push_back( ENV.getInt(Env::nbQRand) );
            mVectDoubles[2].push_back( XYZ_GRAPH->nnode );


            ENV.setInt(Env::costDeltaMethod,INTEGRAL);
            mVectDoubles[3].push_back( optimTrj.cost() );

            ENV.setInt(Env::costDeltaMethod,MECHANICAL_WORK);
            mVectDoubles[4].push_back( optimTrj.cost() );

            cout << " Mean Collision test : "  << optimTrj.meanCollTest() << endl;
        }
        storedContext.addTime(mTime);
        saveVectorToFile();
    }

    cout << " End of Tests ----------------------" << endl;

    return;
}

void MultiRun::saveGraph(int i)
{
    std::ostringstream oss;
    oss << "statFiles/Graph_"<< i << ".graph";
    const char *file = oss.str().c_str();
    cout << "Saving graph to : " << file << endl;
//	    if () {
	//       p3d_write_graph(XYZ_GRAPH, (char *)file);
	p3d_writeGraph(XYZ_GRAPH, file, DEFAULTGRAPH);//Mokhtar Using XML Format
//	    }
//	  }
}

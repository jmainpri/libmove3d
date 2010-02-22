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
void MultiRun::saveVectorToFile(int Context)
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


    for (unsigned int i = 0; i < ENV.getInt(Env::nbMultiRun); i++)
    {
        for (unsigned int j = 0; j < mVectDoubles.size(); j++)
        {
            s << mVectDoubles[j][Context*ENV.getInt(Env::nbMultiRun)+i] << ";";
        }
        s << endl;
    }

    s << endl;

    cout << "Closing save file" << endl;

    s.close();
}

void MultiRun::saveGraph(int i)
{
    char file[256];
    sprintf(file,"statFiles/Graph_%d.graph",i);
    cout << "Saving graph to : " << file << endl;
    p3d_writeGraph(XYZ_GRAPH, file, DEFAULTGRAPH);//Mokhtar Using XML Format

}

void MultiRun::loadGraph()
{
    char file[256];
    // /Users/jmainpri/workspace/BioMove3DDemos/CostHriFunction/SCENARIOS
    sprintf(file,"../BioMove3DDemos/CostHriFunction/SCENARIOS/JidoEasy.graph");
    cout << "Loading graph to : " << file << endl;
    p3d_readGraph(file, DEFAULTGRAPH);
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

    mNames.push_back("Time (RRT)");
    mNames.push_back("Time (Tot)");
    mNames.push_back("NbQRand");
    mNames.push_back("NbNodes");
    mNames.push_back("Cost1");
    mNames.push_back("Cost2");
    mNames.push_back("Integral");
    mNames.push_back("Meca-Work");

    mVectDoubles.resize(8);

    for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
    {
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

            if( nbNodes > 0 )
            {
                ChronoPrint("");
                ChronoTimes(&tu, &ts);
                ChronoOff();
                mTime.push_back(tu);

                //                ENV.setBool(Env::isCostSpace,true);

                Trajectory Traj(
                        new Robot((p3d_rob *) p3d_get_desc_curid(P3D_ROBOT)),
                        ((p3d_rob *)p3d_get_desc_curid(P3D_ROBOT))->tcur);

                mVectDoubles[0].push_back(XYZ_GRAPH->rrtTime);
                mVectDoubles[1].push_back(tu);

                mVectDoubles[2].push_back( ENV.getInt(Env::nbQRand) );
                mVectDoubles[3].push_back( nbNodes );

                bool tmpUnSetCostSpace =false;
                bool tmpUnSetDistancePath =false;

                if(ENV.getBool(Env::HRIPlannerWS) || ENV.getBool(Env::HRIPlannerCS) )
                {
                    if(!ENV.getBool(Env::isCostSpace))
                    {
                        tmpUnSetCostSpace = true;
                        ENV.setBool(Env::isCostSpace,true);
                    }
                    if(ENV.getBool(Env::HRIPathDistance))
                    {
                        tmpUnSetDistancePath = true;
                        ENV.setBool(Env::HRIPathDistance,false);
                    }
                }

                int tmp = ENV.getInt(Env::costDeltaMethod);

                // Before and after Smoothing
                mVectDoubles[4].push_back( XYZ_GRAPH->rrtCost1 );
                mVectDoubles[5].push_back( XYZ_GRAPH->rrtCost2 );

                ENV.setInt(Env::costDeltaMethod,INTEGRAL);
                mVectDoubles[6].push_back( Traj.cost() );

                ENV.setInt(Env::costDeltaMethod,MECHANICAL_WORK);
                mVectDoubles[7].push_back( Traj.costDeltaAlongTraj() );

                ENV.setInt(Env::costDeltaMethod,tmp);

                if( ENV.getBool(Env::HRIPlannerWS)  || ENV.getBool(Env::HRIPlannerCS)  )
                {
                    if(tmpUnSetCostSpace)
                    {
                        tmpUnSetCostSpace = false;
                        ENV.setBool(Env::isCostSpace,false);
                    }
                    if(tmpUnSetDistancePath)
                    {
                        tmpUnSetDistancePath = false;
                        ENV.setBool(Env::HRIPathDistance,true);
                    }
                }

                //                ENV.setBool(Env::isCostSpace,false);

                cout << " Mean Collision test : "  << Traj.meanCollTest() << endl;
                g3d_draw_allwin_active();
                if(ENV.getBool(Env::StopMultiRun))
                    break;
            }
            else
            {
                cout << "--------------------------------------------------------------"  << endl;
                cout << "Warning : No traj Found : Problem during trajectory extraction"  << endl;
                cout << "--------------------------------------------------------------"  << endl;
                p3d_del_graph(XYZ_GRAPH);
            }

            saveGraph(i);
            p3d_del_graph(XYZ_GRAPH);
        }

        storedContext.addTime(mTime);
        saveVectorToFile(j);
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
                          mVectDoubles[4].push_back( optimTrj.costDeltaAlongTraj() );

                          cout << " Mean Collision test : "  << optimTrj.meanCollTest() << endl;
                      }
        storedContext.addTime(mTime);
        saveVectorToFile(j);
    }

    cout << " End of Tests ----------------------" << endl;

    return;
}

/**
  * Run multiple Smooth runs
  */
void MultiRun::runMutliSmooth()
{
//    cout << "Running Multi RRT" << endl;
//    if (storedContext.getNumberStored() == 0)
//    {
//        cout << "WARNING: No context on the context stack" << endl;
//        return;
//    }
//    else
//    {
//        cout << " Running " << storedContext.getNumberStored() << " RRTs from stored context" << endl;
//    }
//
//    mNames.clear();
//    mVectDoubles.clear();
//    mTime.clear();
//
//    mNames.push_back("Time");
//    mNames.push_back("Integral");
//    mNames.push_back("Meca-Work");
//
//    mVectDoubles.resize(5);

    mNames.clear();
    mVectDoubles.clear();
    mTime.clear();

    mNames.push_back("Time");
    mNames.push_back("Cost");

    mVectDoubles.resize(2);

    for(int i =0;i<ENV.getInt(Env::nbMultiSmooth);i++)
    {

        loadGraph();
        p3d_rob* robotPt = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
        p3d_ExtractBestTraj(XYZ_GRAPH);

        CostOptimization optimTrj(new Robot(robotPt),robotPt->tcur);

        if (robotPt->tcur == NULL)
        {
            cout << "No robotPt->tcur to Smooth"  << endl;
            break;
        }

        cout << "Run Nb"  << i << " = " << endl;

        ENV.setBool(Env::isRunning,true);

        double tu(0.0);
        double ts(0.0);
        ChronoOn();

        if(ENV.getBool(Env::withDeformation))
        {
            ENV.setBool(Env::FKShoot,true);
            optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , i );
            ENV.setBool(Env::FKShoot,false);
        }

        if(ENV.getBool(Env::withShortCut))
        {
            optimTrj.runShortCut( ENV.getInt(Env::nbCostOptimize) , i );
        }



        ChronoPrint("");
        ChronoTimes(&tu, &ts);
        ChronoOff();
        ENV.setBool(Env::isRunning,false);

        mTime.push_back(tu);
        mVectDoubles[0].push_back(tu);
        mVectDoubles[1].push_back( optimTrj.cost() );

        optimTrj.replaceP3dTraj();
        ENV.setBool(Env::drawTraj,true);
        g3d_draw_allwin_active();

        if(ENV.getBool(Env::StopMultiRun))
        {
            break;
        }
    }

    saveVectorToFile(0);

    cout << " End of Tests ----------------------" << endl;
    return;
}

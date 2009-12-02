/*
 * testModel.cpp
 *
 *  Created on: Jul 8, 2009
 *      Author: jmainpri
 */
#include "p3d_sys.h"
#include "testModel.hpp"

using namespace std;
using namespace tr1;

TestModel::TestModel() :
        nbColisionTest(10000000), nbLocalPathTest(1000000)
{
    modelRobot = new Robot(XYZ_ROBOT);
    cout << modelRobot->getName() << endl;
}

int TestModel::nbOfColisionsPerSeconds()
{

    double tu, ts;
    int nbTested(0);
    int nbInCol(0);
    ChronoOn();

    for (int i = 0;; i++)
    {
        if (modelRobot->shoot()->IsInCollision())
        {
            nbInCol++;
        }

        ChronoTimes(&tu, &ts);
        if (tu > 5)
        {
            nbTested = i + 1;
            cout << "Percenatge in collision = " << ((double) nbInCol
                                                     / (double) nbTested) << endl;
            break;
        }
    }

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbTested / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 Collision per second").arg(val);
    ENV.setString(Env::numberOfCollisionPerSec,str);
#endif
    return (int) val;
}

int TestModel::nbOfCostPerSeconds()
{

    double tu, ts;
    int nbTested(0);
    int nbCost(0);
    ChronoOn();

    for (int i = 0;; i++)
    {
        if (modelRobot->shoot()->cost())
        {
            nbCost++;
        }

        ChronoTimes(&tu, &ts);
        if (tu > 5)
        {
            break;
        }
    }

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbCost / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 Cost per second").arg(val);
    ENV.setString(Env::numberOfCostPerSec,str);
#endif QT_LIBRARY
    return (int) val;
}

void TestModel::distEnv()
{

    shared_ptr<Configuration> q;

    for (int i = 0; i < 100; i++)
    {
        q = modelRobot->shoot(true);
        //			q->print();
        //			cout << "Conf is In Colision = " << (int)q->IsInCollision() << endl;
        cout << "Distance from obst = " << (double) q->distEnv() << endl;
    }

    return;
}

int TestModel::nbOfLocalPathsPerSeconds()
{

    shared_ptr<Configuration> q1;
    shared_ptr<Configuration> q2;

    double tu, ts;
    ChronoOn();
    int nbLP = 0;
    int nbColTest = 0;
    int nbLPValid = 0;
    int nbMaxTest = 0;
    int nbTest = 0;

    vector<double> dist;

    double x = ENV.getDouble(Env::extensionStep) * p3d_get_env_dmax();

    for (int i = 0;/*i<100*/; i++)
    {

        ChronoTimes(&tu, &ts);
        if (tu > 10)
        {
            nbLP = i;
            break;
        }

        q1 = modelRobot->shoot();
        q1->setConstraints();

        if (q1->IsInCollision())
        {
            continue;
        }

        q2 = modelRobot->shoot();
        LocalPath LP1(q1, q2);



        q2 = LP1.configAtParam(x);
        q2->setConstraints();

        LocalPath LP2(q1, q2);

        if (LP2.getValid())
        {
            nbLPValid++;
        }

        nbTest = LP2.getNbColTest();
        nbColTest += nbTest;

        dist.resize(aveBBDist.size());
        for (int i = 0; i < dist.size(); i++)
        {
            dist[i] += aveBBDist[i];
        }

        if (nbMaxTest < nbTest)
        {
            nbMaxTest = nbTest;
        }
    }

    cout << "Nb Tested = " << nbLP << endl;
    cout << "Nb Valid = " << nbLPValid << endl;
    cout << "Ratio of Valid/Total = " << (double) nbLPValid / (double) nbLP
            << endl;
    cout << "----------------------------------" << endl;
    cout << "nbColTest/sec = " << (double) nbColTest / 10 << endl;
    cout << "nbColTest/LP = " << (double) nbColTest / (double) nbLP << endl;
    cout << "nbColTestMax/LP = " << nbMaxTest << endl;

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbLP / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 LocalPaths per second").arg(val);
    ENV.setString(Env::numberOfLocalPathPerSec,str);
#endif
    return (int) val;
}

void TestModel::runAllTests()
{

    cout << "StarTingTests -----------------------------" << endl;
    //	cout << "nbColisionTest = " << nbColisionTest << endl;
    //	cout << "nbLocalPathTest = " << nbLocalPathTest << endl;
    int costPerSec(0);

    int colPerSec = nbOfColisionsPerSeconds();
    int lpPerSec = nbOfLocalPathsPerSeconds();

    if(ENV.getBool(Env::isCostSpace))
    {
        costPerSec = nbOfCostPerSeconds();
    }

    cout << colPerSec << " Collisions per second and ";
    cout << lpPerSec << " Local Paths per second" << endl;

#ifdef QT_LIBRARY
    QString str1 = QString("%1 Collisions per second").arg(colPerSec);
    ENV.setString(Env::numberOfCollisionPerSec,str1);
    QString str2 = QString("%1 LocalPaths per second").arg(lpPerSec);
    ENV.setString(Env::numberOfLocalPathPerSec,str2);
#endif

    if(ENV.getBool(Env::isCostSpace))
    {
        cout << costPerSec << " Cost computation per second " << endl;
#ifdef QT_LIBRARY
        QString str3 = QString("%1 Cost per second").arg(costPerSec);
        ENV.setString(Env::numberOfCostPerSec,str3);
#endif
    }

    cout << " -- End tests--" << endl;

}

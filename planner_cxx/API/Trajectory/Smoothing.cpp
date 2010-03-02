/*
 * Smoothing.cpp
 *
 *  Created on: Jun 26, 2009
 *      Author: jmainpri
 */
#include "Smoothing.hpp"

using namespace std;
using namespace tr1;

Smoothing::Smoothing() :
        nbBiased(0), nbReallyBiased(0), ShortCutBiased(true)
{
    mSelected.resize(0);
}

Smoothing::Smoothing(const Trajectory& T) :
        Trajectory(T),
        nbBiased(0), nbReallyBiased(0), ShortCutBiased(true)
{
    setSortedIndex();
    mSelected.resize(0);
}

Smoothing::Smoothing(Robot* R, p3d_traj* t) :
        Trajectory(R, t),
        nbBiased(0), nbReallyBiased(0), ShortCutBiased(true)
{
    setSortedIndex();
    mSelected.resize(0);
}

Smoothing::~Smoothing()
{

    //	cout << "Delete Smoothing" << endl;

}

bool Smoothing::oneLoopShortCut()
{

    bool isOptimSuccess(false);

    double lFirst(0.0);
    double lSecond(0.0);

    if (!(getNbPaths() > 1))
        return false;

    vector<shared_ptr<Configuration> > vectConf;

    vectConf = get2RandomConf(lFirst, lSecond);

    shared_ptr<Configuration> qFirstPt = vectConf.at(0);
    shared_ptr<Configuration> qSecondPt = vectConf.at(1);

    // Take a configuration in a certain direction
    LocalPath* newPath = new LocalPath(qFirstPt, qSecondPt);

    bool supposedValid = true;
    // If the path is valid
    // Check for cost
    if ( !ENV.getBool(Env::costBeforeColl) )
    {
        supposedValid = newPath->getValid();
    }
    if ( supposedValid )
    {
        if (ENV.getBool(Env::debugCostOptim))
        {
            //			if( getIdOfPathAt(lFirst)==getHighestCostId() || getIdOfPathAt(lSecond)==getHighestCostId() ){
            //				debugShowTraj(lFirst,lSecond);
            //				nbReallyBiased++;
            //			}
        }


        vector<LocalPath*> paths;

        /**
          * Computes the sub portion on the sides
          */
        unsigned int first,last;

        vector< shared_ptr<Configuration> >  confs =
                getTowConfigurationAtParam(lFirst,lSecond,first,last);

        for(unsigned int i=first;i<=last;i++)
        {
            paths.push_back(getLocalPathPtrAt(i));
        }

        //			cout << "costOfPortion =>"<< endl;
        double costOfPortion = computeSubPortionCost(paths);

        if( !(*vectConf[0] == *qFirstPt) || !(*vectConf[1] == *qSecondPt) )
        {
            cout << "Error in oneLoopShortCut" << endl;
        }

        paths.clear();

        paths.push_back(newPath);

        LocalPath* LP1 = new LocalPath(getLocalPathPtrAt(first)->getBegin(),qFirstPt);
        if(LP1->getParamMax()>0)
        {
            paths.insert(paths.begin(),LP1);
        }


        LocalPath* LP2 = new LocalPath(qSecondPt,getLocalPathPtrAt(last)->getEnd());
        if(LP2->getParamMax()>0)
        {
            paths.push_back(LP2);
        }


        /**
          * Compute the cost of the portion to be replaced
          */
        double sumOfCost = computeSubPortionCost(paths);

        delete LP1;
        delete LP2;

        // If the new path is of lower cost
        // Replace in trajectory
        //		if (newPath->cost() < costOfPortion(lFirst, lSecond))

        bool lowerCost = ( sumOfCost < costOfPortion );

        if(ENV.getBool(Env::costBeforeColl))
        {
            supposedValid = newPath->getValid();
        }
        if ( lowerCost && supposedValid )
        {
            /**
              * Replace
              */
            vector<LocalPath*> vect_path;
            vect_path.push_back(newPath);

            replacePortion(lFirst, lSecond, vect_path);
            setSortedIndex();

            if (!getBegin()->equal(*configAtParam(0)))
            {
                cout << "ERROR" << endl;
            }

            if (!getEnd()->equal(*configAtParam(getRangeMax())))
            {
                cout << "ERROR" << endl;
            }

            isOptimSuccess = true;
        }

    }

    if (isOptimSuccess == false)
    {
        delete newPath;
    }

    return isOptimSuccess;
}

bool Smoothing::oneLoopShortCutRecompute()
{

    bool isOptimSuccess(false);

    double lFirst(0.0);
    double lSecond(0.0);

    if (!(getNbPaths() > 1))
        return false;

    vector<shared_ptr<Configuration> > vectConf;

    vectConf = get2RandomConf(lFirst, lSecond);

    shared_ptr<Configuration> qFirstPt = vectConf.at(0);
    shared_ptr<Configuration> qSecondPt = vectConf.at(1);

    // Take a configuration in a certain direction
    LocalPath* newPath = new LocalPath(qFirstPt, qSecondPt);

    // If the new path is free in CFree
    if (newPath->getValid())
    {
        vector<LocalPath*> paths;
        paths.push_back(newPath);

        Trajectory newTraj(*this);
        newTraj.replacePortion(lFirst, lSecond, paths);

        // If the new path is of lower cost
        // Replace in trajectory
        if (newTraj.cost() < this->cost() )
        {
            vector<LocalPath*> vect_path;
            vect_path.push_back(new LocalPath(*newPath));

            replacePortion(lFirst, lSecond, vect_path);

            setSortedIndex();

            if (!getBegin()->equal(*configAtParam(0)))
            {
                cout << "ERROR" << endl;
            }

            if (!getEnd()->equal(*configAtParam(getRangeMax())))
            {
                cout << "ERROR" << endl;
            }
            isOptimSuccess = true;
        }
    }

    return isOptimSuccess;
}


void Smoothing::removeRedundantNodes()
{
    if(!getValid())
    {
        cout << "Trajectory Not Valid" << endl;
        return;
    }
    else
    {
        cout << "Trajectory Valid" << endl;
    }

    //	this->cutTrajInSmallLP(20);
    //	this->cost();

    uint NbNodes = getNbPaths();
    uint initNbNodes = NbNodes;
    uint nbRemoved(0);

    for (uint i = 0; i < NbNodes - 2; i++)
    {
        for (uint j = i + 2; j < NbNodes; j++)
        {
            shared_ptr<Configuration> start = getLocalPathPtrAt(i)->getBegin();
            shared_ptr<Configuration> end = getLocalPathPtrAt(j)->getBegin();

            LocalPath* pathPtr = new LocalPath(start, end);

            if(start->equal(*end))
            {
                cout << "Error start equal goal" << endl;
            }

            if (pathPtr->getValid())
            {
                double costOfPortion = 0;

                for (uint k = i; k < j; k++)
                {
                    costOfPortion += getLocalPathPtrAt(k)->cost();
                }

                if (pathPtr->cost() < costOfPortion)
                {
                    vector<LocalPath*> path;
                    path.push_back(pathPtr);
                    this->replacePortion(i, j, path);
                    NbNodes = getNbPaths();
                    nbRemoved++;
                    if(!getValid())
                    {
                        cout << "Trajectory Not Valid" << endl;
                    }
                }
                else
                {
                    delete pathPtr;
                }
            }
            else
            {
                delete pathPtr;
            }
        }
    }

    setSortedIndex();

    cout << nbRemoved << " nodes were removed out of " << initNbNodes << endl;
}

void Smoothing::debugShowTraj(double lPrev, double lNext)
{

    vector<shared_ptr<Configuration> > vectConf(2);
    vectConf.at(0) = configAtParam(lPrev);
    vectConf.at(1) = configAtParam(lNext);

    /*	trajToDraw.resize(4);

         trajToDraw.at(0) = extractSubTrajectory(0,lPrev);
         trajToDraw.at(1) = extractSubTrajectory(lPrev,lNext);
         trajToDraw.at(2) = *new Trajectory(vectConf);
         trajToDraw.at(3) = extractSubTrajectory(lNext,getRangeMax());

         trajToDraw.at(0).setColor(0);
         trajToDraw.at(1).setColor(2);
         trajToDraw.at(2).setColor(1);
         trajToDraw.at(3).setColor(0);*/

    //			basicTraj.print();
    //			triangleTraj.print();
    g3d_draw_allwin_active();

    vector<LocalPath*> pathsTmp(1);
    pathsTmp.at(0) = (new LocalPath(vectConf.at(0), vectConf.at(1)));

    Trajectory tmpT(*this);
    tmpT.replacePortion(lPrev, lNext, pathsTmp);
    double newCost = tmpT.cost();

    //	if( mincost > newCost ){
    //		mincost = newCost;
    //
    //	}

    double oldCost = cost();
    double sumOfCost = pathsTmp.at(0)->cost();
    double costOfPortion = this->costOfPortion(lPrev, lNext);

    cout << "Difference on the portion : " << (costOfPortion - sumOfCost)
            << endl;
    cout << "Difference on the trajectory: " << (oldCost - newCost) << endl;
    cout << "---------------------------------------------------------" << endl;

    // double diff = fabs((costOfPortion - sumOfCost) - (oldCost - newCost));

    //	if(diff > 0.001 ){
    //		Errors.push_back(diff);
    //		nbErrors++;
    //	}

}

vector<shared_ptr<Configuration> > Smoothing::get2RandomConf(
        double& firstDist, double& secondDist)
{

    uint id1 = 0;
    uint id2 = 0;

    if (getNbPaths() == 1)
    {
        vector<shared_ptr<Configuration> > vect(0);
        return vect;
    }

    vector<shared_ptr<Configuration> > vectConf(2);

    while (id1 == id2)
    {

        double param;

        if (ShortCutBiased)
        {

            param = getBiasedParamOnTraj();

            if (fabs(param - getRangeMax()) < fabs(param))
            {
                secondDist = param;
                firstDist = p3d_random(0, secondDist);
            }
            else
            {
                firstDist = param;
                secondDist = p3d_random(firstDist, getRangeMax());
            }
        }
        else
        {
            firstDist = p3d_random(0, getRangeMax());
            secondDist = p3d_random(firstDist, getRangeMax());
        }

        id1 = getIdOfPathAt(firstDist);
        id2 = getIdOfPathAt(secondDist);

        if (id1 != id2)
        {
            vectConf.at(0) = configAtParam(firstDist);
            vectConf.at(1) = configAtParam(secondDist);
        }
    }

    return vectConf;
}

class myComparator
{

public:
    Smoothing* ptrOptim;

    bool operator()(uint i, uint j)
    {

        return (ptrOptim->getLocalPathPtrAt(i)->cost()
                > ptrOptim->getLocalPathPtrAt(j)->cost());
    }

} myCompObject;

void Smoothing::setSortedIndex()
{

    myCompObject.ptrOptim = this;

    mIdSorted.resize(getNbPaths());

    for (int i = 0; i < getNbPaths(); i++)
    {
        mIdSorted.at(i) = i;
    }

    sort(mIdSorted.begin(), mIdSorted.end(), myCompObject);

    isHighestCostIdSet = true;
    HighestCostId = mIdSorted[0];

    if (ENV.getBool(Env::debugCostOptim))
    {
        /*for(uint i=0;i<mIdSorted.size();i++){
                 cout<<"mIdSorted["<<i<<"]"<<getLocalPathPtrAt(mIdSorted[i])->cost()<<endl;
                 }*/
    }
}

double Smoothing::getBiasedParamOnTraj()
{

    double x = (double) (pow(p3d_random(0, 1), 3));

    uint id = (uint) (x * (getNbPaths() - 1));

    double randDist = 0;

    //	cout <<"mIdSorted[id] = "<<  mIdSorted[id] << endl;
    //	cout <<"getNbPaths()"<< getNbPaths() << endl;

    for (uint i = 0; i < mIdSorted[id]; i++)
    {
        randDist += getLocalPathPtrAt(i)->getParamMax();
    }

    randDist += p3d_random(0, getLocalPathPtrAt(mIdSorted[id])->getParamMax());

    if (getIdOfPathAt(randDist) == HighestCostId)
    {
        nbBiased++;
    }

    //	double percent = (double)id/(double)(mIdSorted.size());
    if (ENV.getBool(Env::debugCostOptim))
    {
        mSelected.push_back(id);
    }

    return randDist;
}

void Smoothing::saveOptimToFile(string fileName)
{
    std::ostringstream oss;
    oss << "statFiles/"<< fileName << ".csv";

    const char *res = oss.str().c_str();

    std::ofstream s;
    s.open(res);

    cout << "Opening save file : " << res << endl;

    s << " Cost" << ";";
    s << endl;

    for (unsigned int i = 0; i < mOptimCost.size(); i++)
    {
        s << mOptimCost[i] << ";";
        s << endl;
    }

    cout << "Closing save file" << endl;

    s.close();
}

void Smoothing::runShortCut(int nbIteration, int idRun )
{
    cout << "Before Short Cut : Traj cost = " << this->costNoRecompute() << endl;
    mOptimCost.clear();

    double CurrentCost = this->cost();

    for (int i = 0; i < nbIteration; i++)
    {
        oneLoopShortCut();

        if(ENV.getBool(Env::saveTrajCost))
        {
            double NewCost = this->cost();
            if ( NewCost > CurrentCost )
            {
                cout << "Smoothing::runDeformation : NewCost > CurrentCost"  << endl;
            }
            mOptimCost.push_back( NewCost );
        }
    }

    if (getValid())
    { cout << "Trajectory valid" << endl; }
    else
    { cout << "Trajectory not valid" << endl;}

    if(ENV.getBool(Env::saveTrajCost))
    {
        ostringstream oss;
        oss << "ShortCutOptim_"<< idRun << "_" ;
        this->saveOptimToFile(oss.str());
    }
    cout << "After Short Cut : cost = " << this->costNoRecompute() << endl;
}

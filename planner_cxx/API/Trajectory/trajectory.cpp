/*
 * trajectory.cpp
 *
 *  Created on: Jun 17, 2009
 *      Author: jmainpri
 */

#include "trajectory.hpp"
//#include "../planner/Diffusion/proto/p3d_SpaceCost_proto.h"

#ifdef HRI_COSTSPACE
#include "../planner_cxx/HRI_CostSpace/HRICS_HAMP.h"
#endif

using namespace std;
using namespace tr1;

Trajectory::Trajectory() :
	HighestCostId(0),
	isHighestCostIdSet(false),
	name(""),
	file(""),
	mRobot(NULL),
	nloc(0),
	color(0),
	range_param(0),
	mBegin(new Configuration(mRobot,NULL))
{
}

Trajectory::Trajectory(Robot* R) :
	HighestCostId(0),
	isHighestCostIdSet(false),
	name(""),
	file(""),
	mRobot(R),
	nloc(0),
	color(0),
	range_param(0),
	mBegin(new Configuration(mRobot,NULL))
{
}

Trajectory::Trajectory(const Trajectory& T) :
  HighestCostId(T.HighestCostId), isHighestCostIdSet(T.isHighestCostIdSet),
  name(T.name), file(T.file), mRobot(T.mRobot), nloc(T.nloc), color(T.color),
  range_param(T.range_param), mBegin(T.mBegin), mEnd(T.mEnd)
{

	for (uint i = 0; i < nloc; i++)
	{
		mCourbe.push_back(new LocalPath(*(T.mCourbe.at(i))));
	}
	//	cout << "Copy Trajectory" << endl;
}

Trajectory& Trajectory::operator=(const Trajectory& t)
{

	if (nloc > 0)
	{
		for (uint i = 0; i < nloc; i++)
		{
			delete mCourbe.at(i);
		}
	}

	mCourbe.clear();

	// TODO Name of file and robot
	name = t.name;
	file = t.file;

	mRobot = t.mRobot;

	/* Number of localpath */
	nloc = t.nloc;
	range_param = t.range_param;

	for (uint i = 0; i < nloc; i++)
	{
		mCourbe.push_back(new LocalPath(*(t.mCourbe.at(i))));
	}

	//	cout << "Copy in operator= Trajtecory" << endl;

	mBegin = t.mBegin;
	mEnd = t.mEnd;
	color = t.color;
	HighestCostId = t.HighestCostId;
	isHighestCostIdSet = t.isHighestCostIdSet;

	return *this;
}

Trajectory::Trajectory(std::vector<shared_ptr<Configuration> > C) :
	HighestCostId(0), isHighestCostIdSet(false)
{

	if (C.size() > 1)
	{
		name = "";
		file = "";

		nloc = C.size() - 1; /* Number of localpath */
		mRobot = C.at(0)->getRobot();
		range_param = 0;

		mBegin = C.at(0);
		mEnd = C.at(nloc);

		for (uint i = 0; i < nloc; i++)
		{
			LocalPath* path = new LocalPath(C.at(i), C.at(i + 1));
			range_param += path->getParamMax();
			mCourbe.push_back(path);
		}
	}
	else
	{
		cout
				<< "Warning: Constructing a class out of empty vector of configuration"
				<< endl;
	}
	color = 1;
}

Trajectory::Trajectory(Robot* R, p3d_traj* t)
{

	// TODO Name and file (string based)

	mRobot = R;

	nloc = t->nlp;

	p3d_localpath *localpathPt;

	if (t != NULL)
	{

		localpathPt = t->courbePt;

		while (localpathPt != NULL)
		{
			LocalPath* path = new LocalPath(mRobot, localpathPt);
			//			path->getBegin()->print();
			//			path->getEnd()->print();
			mCourbe.push_back(path);
			localpathPt = localpathPt->next_lp;
		}
		range_param = p3d_compute_traj_rangeparam(t);
	}

	mBegin = shared_ptr<Configuration> (new Configuration(mRobot,
			p3d_config_at_param_along_traj(t, 0)));

	mBegin->setConstraints();

	//	cout << "mBegin:" << endl;
	//	mBegin->print();

	mEnd = shared_ptr<Configuration> (new Configuration(mRobot,
			p3d_config_at_param_along_traj(t, range_param)));

	mEnd->setConstraints();
	//	cout << "mEnd:" << endl;
	//	mEnd->print();

	updateRange();
	//	cout << "range_param = " << range_param << endl;
	color = 0;

	if (!getBegin()->equal(*configAtParam(0)))
	{
		cout << "Error in constructor : !getBegin()->equal(*configAtParam(0))" << endl;
	}

	if (!getEnd()->equal(*configAtParam(getRangeMax())))
	{
		cout << "------------------------------------------" << endl;
		cout << "Error in constructor : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
		getEnd()->print();
		configAtParam(getRangeMax())->print();
	}
}

Trajectory::~Trajectory()
{

	//	cout << "Delete Trajectory ("<<nloc<<")" << endl;

	for (uint i = 0; i < nloc; i++)
	{
		//		cout<<"delete "<<i<<endl;
		delete mCourbe.at(i);
	}
}

void Trajectory::replaceP3dTraj()
{

	replaceP3dTraj(mRobot->getTrajStruct());

}

void Trajectory::replaceP3dTraj(p3d_traj* trajPt)
{

	//	print();

	if(trajPt!=NULL)
	{
		destroy_list_localpath(mRobot->getRobotStruct(), trajPt->courbePt);
	}
	else
	{
		trajPt = p3d_create_empty_trajectory(mRobot->getRobotStruct());
	}

	//	trajPt->name       = strdup(name);
	//	trajPt->file       = NULL;  // Modification Fabien
	trajPt->num = 0; //mRobot->getRobotStruct()->nt;
	trajPt->rob = mRobot->getRobotStruct();

	//	cout << mRobot->getRobotStruct() << endl;

	nloc = (int) mCourbe.size();
	//	cout << "Number of local paths : " << nloc << endl;

	p3d_localpath *localpathPt = mCourbe.at(0)->getLocalpathStruct()->copy(
			mRobot->getRobotStruct(), mCourbe.at(0)->getLocalpathStruct());
	p3d_localpath *localprevPt = NULL;

	localpathPt->prev_lp = localprevPt;
	trajPt->courbePt = localpathPt;

	for (uint i = 1; i < nloc; i++)
	{

		localprevPt = localpathPt;

		localpathPt = mCourbe.at(i)->getLocalpathStruct()->copy(
				mRobot->getRobotStruct(), mCourbe.at(i)->getLocalpathStruct());
		//		cout << "localpathPt = " << localpathPt << endl;

		localprevPt->next_lp = localpathPt;
		//		cout << "localprevPt->next_lp = " << localpathPt << endl;

		localpathPt->prev_lp = localprevPt;
		//		cout << "localpathPt->prev_lp = " << localpathPt << endl;

		if (i != nloc - 1)
		{
			localpathPt->next_lp = mCourbe.at(i + 1)->getLocalpathStruct();
			//			cout << "localpathPt->next_lp = " << mCourbe.at(i+1)->getLocalpathStruct() << endl;
		}
	}

	localpathPt->next_lp = NULL;
	trajPt->nlp = nloc;
	trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);

	//	print()

}

shared_ptr<Configuration> Trajectory::configAtParam(double param)
{

	double soFar(0.0);
	double prevSoFar(0.0);

	for (uint i = 0; i < nloc; i++)
	{

		soFar = soFar + mCourbe.at(i)->getParamMax();

		// Parameter lies in the previous local path
		// return configuration inside previous LP
		if (param < soFar)
		{

			if (param < prevSoFar)
			{
				cout
						<< "Error: getting Configuration at parameter in trajectory"
						<< endl;
			}

			return mCourbe.at(i)->configAtParam(param - prevSoFar);
		}
		prevSoFar = soFar;
	}
	return mCourbe.back()->configAtParam(param);
}

vector<shared_ptr<Configuration> > Trajectory::getNConfAtParam(double delta)
{
	vector<shared_ptr<Configuration> > tmpVector(0);

	double param = 0;

	double soFar(0.0);
	double prevSoFar(0.0);

	for (uint i = 0; i < nloc; i++)
	{

		soFar = soFar + mCourbe.at(i)->getParamMax();

		// Parameter lies in the previous local path
		// return configuration inside previous LP
		while (param < soFar)
		{

			if (param < prevSoFar)
			{
				cout
						<< "Error: getting Configuration at parameter in trajectory"
						<< endl;
			}

			tmpVector.push_back(mCourbe.at(i)->configAtParam(param - prevSoFar));
			param += delta;
		}
		// TODO watch out
		if (i < nloc - 1)
		{
			prevSoFar = soFar;
		}
	}

	tmpVector.push_back(mCourbe.at(nloc - 1)->configAtParam(param - prevSoFar));

	return tmpVector;

}

LocalPath* Trajectory::getLocalPathPtrAt(uint id)
{
	return mCourbe.at(id);
}

int Trajectory::getNbPaths()
{
	return mCourbe.size();
}

double Trajectory::computeSubPortionRange(vector<LocalPath*> portion)
{

	double range(0.0);

        for (unsigned int i = 0; i < portion.size(); i++)
	{
		range += portion[i]->getParamMax();
	}

	return range;
}

void Trajectory::updateRange()
{

	nloc = mCourbe.size();
	range_param = computeSubPortionRange(mCourbe);

}

bool Trajectory::getValid()
{
        for (unsigned int i = 0; i < mCourbe.size(); i++)
	{
		if (!mCourbe[i]->getValid())
		{
			return false;
		}
                cout <<"LocalPath["<<i<<"] = "<< mCourbe[i]->getNbColTest() << endl;
	}

	return true;
}

double Trajectory::computeSubPortionCost(vector<LocalPath*> portion)
{
	double sumCost(0.0);
	double cost(0.0);

        for (unsigned int i = 0; i < portion.size(); i++)
	{
		cost = portion[i]->cost();
//		cout << "Cost["<<i<<"] = "<< cost << endl;
		sumCost += cost;
	}

	return sumCost;
}

double Trajectory::ReComputeSubPortionCost(vector<LocalPath*> portion)
{
        double sumCost(0.0);
        double cost(0.0);

        for (unsigned int i = 0; i < portion.size(); i++)
        {
                portion[i]->resetCostComputed();
                cost = portion[i]->cost();
//		cout << "Cost["<<i<<"] = "<< cost << endl;
                sumCost += cost;
        }

        return sumCost;
}

double Trajectory::computeSubPortionIntergralCost(vector<LocalPath*> portion)
{
	double cost(0.0);
//	double dmax = p3d_get_env_dmax()/2;
	double dmax = 0;

//	cout << "Computing resolution" << endl;

	for( unsigned int i=0; i<mCourbe.size();i++)
	{
		double resol = mCourbe[i]->getResolution();
//		cout << "resol["<< i <<"] = "<< resol << endl;
		dmax += resol;
	}

	dmax /= (double)mCourbe.size();

	double currentParam(0.0);
	double prevCost;
	double currentCost = portion[0]->getBegin()->cost();
	double range = computeSubPortionRange(portion);

	while (currentParam <= range)
	{
		currentParam += dmax;
		prevCost = currentCost;
		currentCost = configAtParam(currentParam)->cost();
		//the cost associated to a small portion of curve
		double step_cost =
				p3d_ComputeDeltaStepCost(prevCost, currentCost, dmax);
		cost += step_cost;
	}

	return cost;
}

double Trajectory::computeSubPortionCostVisib( vector<LocalPath*> portion )
{
	double epsilon = 0.002;
	double cost(0.0);
	double dmax = 0;

	bool inVisibilty(false);

	for( unsigned int i=0; i<mCourbe.size();i++)
	{
		double resol = mCourbe[i]->getResolution();
		dmax += resol;
	}

	dmax /= (double)mCourbe.size();

	double currentParam(0.0);
	double prevCost;
	double currentCost = portion[0]->getBegin()->cost();
	double range = computeSubPortionRange(portion);

	int jnt_id=0;

#ifdef HRI_COSTSPACE
			jnt_id = hriSpace->getTask();
#else
			cout << "Error : HRI Planner not compiled nor linked" << endl;
			return 0;
#endif

	mRobot->setAndUpdate(*mBegin);
        Vector3d prevPos;
        Vector3d currentPos = mRobot->getJointPos(jnt_id);

	while (currentParam <= range)
	{
		currentParam += dmax;

		shared_ptr<Configuration> currentConf = configAtParam(currentParam);

		prevCost = currentCost;
		prevPos = currentPos;

		mRobot->setAndUpdate(*currentConf);
		currentPos = mRobot->getJointPos(jnt_id);
		double distStep=0;
		for(int k=0;k<currentPos.size();k++)
		{
			distStep += pow((currentPos[k]-prevPos[k]),2);
		}
		distStep = sqrt(distStep);

		double step_cost;

		if(!inVisibilty)
		{
			currentCost = currentConf->cost();
			//the cost associated to a small portion of curve


			step_cost =
                                        p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep);

                        cout << " Step Cost = " << step_cost << endl;

			if( currentCost < ENV.getDouble(Env::visThresh) )
			{
				inVisibilty = true;
			}
		}
		else
		{
			step_cost =  epsilon * distStep;
		}

		cost += step_cost;
	}

	return cost;
}

double Trajectory::cost()
{
	double cost(0.0);

	if( p3d_GetDeltaCostChoice() != VISIBILITY )
	{
//		cost = computeSubPortionCost(mCourbe);
            cost = ReComputeSubPortionCost(mCourbe);
	}
	else
	{
		cost = computeSubPortionCostVisib(mCourbe);
	}
//   cost = computeSubPortionIntergralCost(mCourbe);

   return cost;
}

vector<shared_ptr<Configuration> > Trajectory::getTowConfigurationAtParam(
		double param1, double param2, uint& lp1, uint& lp2)
{
	vector<shared_ptr<Configuration> > conf;

	if (param1 < 0)
	{
		cout << "Error: the parameter is out of band" << endl;
	}
	if (param2 > this->range_param)
	{
		cout << "Error: the parameter is out of band" << endl;
	}

	if (param1 > param2)
	{
		cout
				<< "Error: not possible to replace trajectory because of the parameters"
				<< endl;
		return conf;
	}

	// Looks for the local paths to be changed linear in NLOC
	//-------------------------------------------------------------------
	double soFar(0.0);
	double prevSoFar(0.0);

	shared_ptr<Configuration> q1;
	shared_ptr<Configuration> q2;

	for (uint i = 0; i < mCourbe.size(); i++)
	{
		soFar = soFar + mCourbe.at(i)->getParamMax();

		if (param1 < soFar)
		{
			q1 = mCourbe.at(i)->configAtParam(param1 - prevSoFar);
			//			q1->print();
			//			param_start = param1-prevSoFar;
			lp1 = i;
			break;
		}
		prevSoFar = soFar;
		if (i == (nloc - 1))
		{
			q1 = mCourbe.at(i)->getEnd();
			lp1 = i;
			cout << "Error : conf ends local path" << endl;
		}
	}

	// id_LP_1 is the id of the path of which param1 lies in
	// we start searching for parm2 from here
	soFar = prevSoFar;

	for (uint i = lp1; i < mCourbe.size(); i++)
	{
		soFar = soFar + mCourbe.at(i)->getParamMax();

		if (param2 < soFar)
		{
			q2 = mCourbe.at(i)->configAtParam(param2 - prevSoFar);
			//			q2->print();
			//			param_end = param2-soFar;
			lp2 = i;
			break;
		}
		prevSoFar = soFar;

		if (i == (nloc - 1))
		{
			q2 = mCourbe.at(nloc - 1)->getEnd();
			lp2 = i;
		}
	}

//        q1->setConstraints();
//        q2->setConstraints();

	conf.push_back(q1);
	conf.push_back(q2);
	return conf;
}

double Trajectory::extractCostPortion(double param1, double param2)
{
	double totalCost(0.0);

	vector<shared_ptr<Configuration> > conf;

	uint first;
	uint last;

	conf = getTowConfigurationAtParam(param1, param2, first, last);

	shared_ptr<Configuration> q1 = conf.at(0);
	shared_ptr<Configuration> q2 = conf.at(1);

	if (first > last)
	{
		cout
				<< "Error: extractCostPortion: inconsistent query for subpath extraction"
				<< endl;
		return 0;
	}

	// Case where they lie in the same path
	//----------------------------------------------------------------------------
	if (first == last)
	{
		if (q1->equal(*q2))
		{
			cout
					<< "Error: extractCostPortion: q1 and q2 are the same in subportion extraction"
					<< endl;
			return 0;
		}
		LocalPath LP(q1, q2);
		return LP.cost();
	}

	shared_ptr<Configuration> start = mCourbe.at(first)->getBegin();
	shared_ptr<Configuration> end = mCourbe.at(last)->getEnd();

	// Adds the modified first local path to subpaths
	// Verifies that the configuration is not starting the local path
	if (!start->equal(*q1))
	{
		if (!mCourbe.at(first)->getEnd()->equal(*q1))
		{
			LocalPath LP(q1, mCourbe.at(first)->getEnd());

			if (LP.getValid())
			{
				totalCost += LP.cost();
			}
			else
			{
				cout << "Error: extractCostPortion: portion of path not valid"
						<< endl;
				return -1;
			}
		}
	}
	else
	{

		if (mCourbe.at(first)->getValid())
		{
			totalCost += mCourbe.at(first)->cost();
		}
		else
		{
			cout << "Error: extractCostPortion: portion of path not valid"
					<< endl;
			return -1;
		}
	}

	// Adds all the paths between First and Last
	for (uint i = first + 1; i < last; i++)
	{
		totalCost += mCourbe.at(i)->cost();
	}

	// Verifies that the configuration is not ending the local path
	// Adds the modified Last local path to subpaths
	if (!end->equal(*q2))
	{
		if (!mCourbe.at(last)->getBegin()->equal(*q2))
		{
			LocalPath LP(mCourbe.at(last)->getBegin(), q2);

			if (LP.getValid())
			{
				totalCost += LP.cost();
			}
			else
			{
				cout << "Error: extractCostPortion: portion of path not valid"
						<< endl;
				return -1;
			}
		}
	}
	else
	{
		if (mCourbe.at(last)->getValid())
		{
			totalCost += mCourbe.at(last)->cost();
		}
		else
		{
			cout << "Error: extractCostPortion: portion of path not valid"
					<< endl;
			return -1;
		}
	}

	// Verifies the integrity of the sub paths
	/*if ((!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(
	 *paths.back()->getEnd())))
	 {
	 paths.at(0)->getBegin()->print();
	 q1->print();
	 paths.back()->getEnd()->print();
	 q2->print();
	 cout << "Error: extractCostPortion: in extract sub portion integrity" << endl;
	 }*/

	return totalCost;
}

vector<LocalPath*> Trajectory::extractSubPortion(double param1, double param2,
		uint& first, uint& last)
{

	vector<LocalPath*> paths;
	vector<shared_ptr<Configuration> > conf;

	conf = getTowConfigurationAtParam(param1, param2, first, last);

	shared_ptr<Configuration> q1 = conf.at(0);
	shared_ptr<Configuration> q2 = conf.at(1);

	if (first > last)
	{
		cout << "Error: inconsistent query for subpath extraction" << endl;
		return paths;
	}

	// Case where they lie in the same path
	//----------------------------------------------------------------------------
	if (first == last)
	{
		if (q1->equal(*q2))
		{
			paths.resize(0);
			cout << "Error: q1 and q2 are the same in subportion extraction"
					<< endl;
			return paths;
		}
		paths.push_back(new LocalPath(q1, q2));
		return paths;
	}

	shared_ptr<Configuration> start = mCourbe.at(first)->getBegin();
	shared_ptr<Configuration> end = mCourbe.at(last)->getEnd();

	// Adds the modified first local path to subpaths
	// Verifies that the configuration is not starting the local path
	if (!start->equal(*q1))
	{
		if (!mCourbe.at(first)->getEnd()->equal(*q1))
		{
			LocalPath* startNew =
					new LocalPath(q1, mCourbe.at(first)->getEnd());

			if (startNew->getValid())
			{
				paths.push_back(startNew);
			}
			else
			{
				cout << "Error: portion of path not valid" << endl;
				return paths;
			}
		}
	}
	else
	{
		LocalPath* startNew = new LocalPath(*mCourbe.at(first));

		if (startNew->getValid())
		{
			paths.push_back(startNew);
		}
		else
		{
			cout << "Error: portion of path not valid" << endl;
			return paths;
		}
	}

	// Adds all the paths between First and Last
	for (uint i = first + 1; i < last; i++)
	{
		paths.push_back(new LocalPath(*mCourbe.at(i)));
	}

	// Verifies that the configuration is not ending the local path
	// Adds the modified Last local path to subpaths
	if (!end->equal(*q2))
	{
		if (!mCourbe.at(last)->getBegin()->equal(*q2))
		{
			LocalPath* endNew = new LocalPath(mCourbe.at(last)->getBegin(), q2);

			if (endNew->getValid())
			{
				paths.push_back(endNew);
			}
			else
			{
				cout << "Error: portion of path not valid" << endl;
				return paths;
			}
		}
	}
	else
	{
		LocalPath* endNew = new LocalPath(*mCourbe.at(last));

		if (endNew->getValid())
		{
			paths.push_back(endNew);
		}
		else
		{
			cout << "Error: portion of path not valid" << endl;
			return paths;
		}
	}

	// Verifies the integrity of the sub paths
	if ((!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(
			*paths.back()->getEnd())))
	{
		paths.at(0)->getBegin()->print();
		q1->print();
		paths.back()->getEnd()->print();
		q2->print();
		cout << "Error: in extract sub portion integrity" << endl;
	}

	return paths;
}

Trajectory Trajectory::extractSubTrajectory(double param1, double param2)
{

	uint first(0);
	uint last(0);

	vector<LocalPath*> path;

	if (param1 > param2)
	{
		cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
	}
	else
	{
		path = extractSubPortion(param1, param2, first, last);
	}

	Trajectory newTraj(mRobot);

	newTraj.mCourbe = path;
	newTraj.nloc = path.size();

	if (path.size() == 0)
	{
		newTraj.mBegin = configAtParam(param1);
		newTraj.mEnd = newTraj.mBegin;
		newTraj.range_param = 0;
	}
	else
	{
		newTraj.mBegin = path.at(0)->getBegin();
		newTraj.mEnd = path.back()->getEnd();
		newTraj.range_param = computeSubPortionRange(path);
	}

	return newTraj;
}

extern double ZminEnv;
extern double ZmaxEnv;

extern void* GroundCostObj;

void Trajectory::drawGL(int nbKeyFrame)
{

	double du = range_param / nbKeyFrame;
	double u = du;

	int val1, val2;
	double Cost1, Cost2;

	p3d_obj *o;

	int NumBody = mRobot->getRobotStruct()->no - 1;

	if ((NumBody >= mRobot->getRobotStruct()->no) || (NumBody < 0))
		return;

	if (!(o = mRobot->getRobotStruct()->o[NumBody]))
		return;

	shared_ptr<Configuration> qSave = mRobot->getCurrentPos();
	shared_ptr<Configuration> q = mBegin;
	mRobot->setAndUpdate(*q);

	p3d_vector3 pi, pf;
	p3d_jnt_get_cur_vect_point(o->jnt, pi);

	int saveColor;
	bool red = false;

	while (u < range_param)
	{
		/* position of the robot corresponding to parameter u */
		q = configAtParam(u);
		mRobot->setAndUpdate(*q);
		p3d_jnt_get_cur_vect_point(o->jnt, pf);

		if (isHighestCostIdSet)
		{
			if (getIdOfPathAt(u) == HighestCostId && !red)
			{
				red = true;
				saveColor = color;
				color = 3;
			}
			if ((color == 3) && (getIdOfPathAt(u) != HighestCostId) && red)
			{
				color = saveColor;
				red = false;
			}
		}

		if ((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL))
		{
			glLineWidth(3.);
			g3d_drawOneLine(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2], color,
					NULL);
			glLineWidth(1.);
		}
		else
		{
			val1 = GHintersectionVerticalLineWithGround(GroundCostObj, pi[0],
					pi[1], &Cost1);
			val2 = GHintersectionVerticalLineWithGround(GroundCostObj, pf[0],
					pf[1], &Cost2);
			glLineWidth(3.);
			g3d_drawOneLine(pi[0], pi[1], Cost1 + (ZmaxEnv - ZminEnv) * 0.02,
					pf[0], pf[1], Cost2 + (ZmaxEnv - ZminEnv) * 0.02, color,
					NULL);
			glLineWidth(1.);
		}
		p3d_vectCopy(pf, pi);
		u += du;
	}

	mRobot->setAndUpdate(*qSave);
}

double Trajectory::costOfPortion(double param1, double param2)
{

	uint first(0);
	uint last(0);

	vector<LocalPath*> path = extractSubPortion(param1, param2, first, last);

	double Cost = computeSubPortionCost(path);

	for (unsigned int i = 0; i < path.size(); i++)
	{
		delete path.at(i);
	}
	return Cost;

}
//double Trajectory::costOfPortion(double param1,double param2){
//
//	double soFar(0.0);
//	double prevSoFar(0.0);
//	uint id_start;
//	uint id_end;
//	shared_ptr<Configuration> confPtrStart;
//	shared_ptr<Configuration> confPtrEnd;
//
//	// TODO change that function
//
//	if( param1 > param2 ){
//		cout << "Error: in Trajectory::costofPortion() " << endl;
//		return 0;
//	}
//
//	for(uint i=0;i<nloc;i++){
//
//		soFar = soFar + mCourbe.at(i)->getParamMax();
//
//		// Parameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param1 < soFar){
//
//			if(param1 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrStart = mCourbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrStart = mCourbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	soFar = prevSoFar;
//
//	for(uint i=id_start;i<nloc;i++){
//
//		soFar = soFar + mCourbe.at(i)->getParamMax();
//
//		// Paramameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param2 < soFar){
//
//			if(param2 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrEnd = mCourbe.at(i)->configAtParam(param2-prevSoFar);
//			id_end = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrEnd = mCourbe.at(i)->getEnd();
//			id_end = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	LocalPath pathStart(confPtrStart, mCourbe.at(id_start)->getEnd());
//	LocalPath pathEnd(mCourbe.at(id_end)->getBegin(),confPtrEnd);
//
//	double cost =0.0;
//
//	cost += pathStart.cost() + pathEnd.cost();
//
//	for(uint i=id_start+1;i<id_end;i++){
//		cost+=mCourbe.at(i)->cost();
//	}
//
//	return cost;
//}

uint Trajectory::cutPortionInSmallLP(vector<LocalPath*>& portion, uint nLP)
{
	double range = computeSubPortionRange(portion);
	double dMax = range / 40;

	uint j = 0;
	double soFar = 0.0;
	double prevSoFar = 0.0;
	double param = 0.0;

	if (nLP == 0)
	{
		cout << "Error: cutPortionInSmallLP" << endl;
	}

	if (nLP == 1)
	{
		return nLP;
	}

	vector<LocalPath*> portionTmp(nLP);
	shared_ptr<Configuration> confPtr;
	shared_ptr<Configuration> confPtrTmp;

	prevSoFar = 0.0;
	soFar = portion.at(0)->getParamMax();
	confPtrTmp = portion.at(0)->getBegin();

	// Loop for every Small LP
	for (uint i = 0; i < nLP - 1; i++)
	{

		param = param + dMax;

		// Check to go seek the conf on next LP
		while ((param > soFar) && (j < portion.size()))
		{
			j = j + 1;
			prevSoFar = soFar;
			soFar = soFar + portion.at(j)->getParamMax();
		}

		confPtr = portion.at(j)->configAtParam(param - prevSoFar);
		portionTmp.at(i) = new LocalPath(confPtrTmp, confPtr);
		confPtrTmp = confPtr;
	}

	// The last LP is made of the end configuration
	confPtr = portion.back()->getEnd();
	confPtrTmp = portionTmp.at(nLP - 2)->getEnd();

	portionTmp.back() = new LocalPath(confPtrTmp, confPtr);

	// Delete and replace every thing
	for (uint i = 0; i < portion.size(); i++)
	{
		delete portion.at(i);
	}

	portion.clear();
	portion = portionTmp;

	if (nLP != portion.size())
	{
		cout << "Error: cutPortionInSmallLP" << endl;
	}

	return nLP;
}

void Trajectory::push_back(shared_ptr<Configuration> q)
{
	if(mCourbe.size()==0)
	{
		Configuration q_null(NULL);
		if(*mBegin==q_null)
		{
			mBegin = q;
		}
		else
		{
			mCourbe.push_back(new LocalPath(mBegin,q));
			mEnd = q;
			updateRange();
		}
	}
	else
	{
		mCourbe.push_back(new LocalPath(mEnd,q));
		mEnd = q;
		updateRange();
	}
}

void Trajectory::cutTrajInSmallLP(unsigned int nLP)
{

	cutPortionInSmallLP(mCourbe, nLP);
	updateRange();

	cout << "Cutting into " << nLP << " local paths" << endl;
	cout << "Traj Size = " << nloc << endl;
	cout << "Cost Of trajectory :" << this->cost() << endl;

	if (!mBegin->equal(*configAtParam(0)))
	{
		cout << "Error" << endl;
	}

	if (!mEnd->equal(*configAtParam(range_param)))
	{
		mEnd->print();
		configAtParam(range_param)->print();
		cout << "Error" << endl;
	}

}

void Trajectory::replacePortion(uint id1, uint id2, vector<LocalPath*> paths)
{
	// WARNING: the ids are ids of nodes and not
	// LocalPaths

	if (id1 < id2)
	{
		for (uint i = id1; i < id2; i++)
		{
			delete mCourbe.at(i);
		}

		mCourbe.erase(mCourbe.begin() + id1, mCourbe.begin() + id2);
	}
	if (id1 == id2)
	{
		cout << "Error: in replace local (id1 and id2 are the same)" << endl;
	}

	mCourbe.insert(mCourbe.begin() + id1, paths.begin(), paths.end());

	updateRange();

}

void Trajectory::replacePortion(double param1, double param2,
		vector<LocalPath*> paths)
{
	shared_ptr<Configuration> q11 = paths.at(0)->getBegin();
	shared_ptr<Configuration> q12 = paths.back()->getEnd();

	shared_ptr<Configuration> q21;
	shared_ptr<Configuration> q22;

	if (param1 > param2)
	{
		cout
				<< "Warning: Error not possible to replace trajectory because of the parameters"
				<< endl;
		return;
	}

	// TODO replace with extratSubPortion

	// Looks for the local paths to be changed linear in NLOC
	//-------------------------------------------------------------------
	double soFar(0.0);
	double prevSoFar(0.0);

	double param_start(0.0);
	double param_end(0.0);

	uint id_LP_1(0);
	uint id_LP_2(0);

	for (uint i = 0; i < nloc; i++)
	{
		soFar = soFar + mCourbe[i]->getParamMax();

		// param1 is in local path i
		if (param1 < soFar)
		{
			// get configuration in local path i
			q21 = mCourbe.at(i)->configAtParam(param1 - prevSoFar);
			param_start = param1 - prevSoFar;
			id_LP_1 = i;
			break;
		}
		prevSoFar = soFar;
		if (i == (nloc - 1))
		{
			cout << "Warning: first parameter not found on trajectory" << endl;
			//			return;
		}
	}

	soFar = prevSoFar;

	for (uint i = id_LP_1; i < nloc; i++)
	{
		soFar = soFar + mCourbe[i]->getParamMax();

		// param2 is in local path i
		if (param2 < soFar)
		{
			// get configuration in local path i
			q22 = mCourbe.at(i)->configAtParam(param2 - prevSoFar);
			param_end = param2 - soFar;
			id_LP_2 = i;
			break;
		}
		prevSoFar = soFar;

		if (i == (nloc - 1))
		{
			q22 = mCourbe.at(nloc - 1)->getEnd();
			id_LP_2 = i;
			param_end = soFar;
		}
	}

	// Condition has to be false if the trajectory is consistent
	if ((!q11->equal(*q21)) || (!q12->equal(*q22)))
	{
		if (!q11->equal(*q21))
		{
			q11->print();
			q21->print();
		}
		if (!q12->equal(*q22))
		{
			q12->print();
			q22->print();
		}
		cout
				<< "Warning: Error not possible to replace trajectory because configuration are not the same"
				<< endl;
		return;
	}

	// Adds to paths the portion of local path (At begin and End of the portion)
	//----------------------------------------------------------------------------
	shared_ptr<Configuration> start = mCourbe.at(id_LP_1)->getBegin();
	shared_ptr<Configuration> end = mCourbe.at(id_LP_2)->getEnd();

	/*cout << "Start and End" << endl;
	start->print(); end->print();
	cout << "q21 and q22" << endl;
	q21->print(); q22->print();*/

	// Verifies that the configuration is not starting the local path
	if (!start->equal(*q21))
	{
		LocalPath* startNew = new LocalPath(start, q21);

		if (startNew->getValid())
		{
			paths.insert(paths.begin(), new LocalPath(start, q21));
		}
		else
		{
			cout << "Error: portion of path not valid" << endl;
			return;
		}
	}

	// Verifies that the configuration is not ending the local path
	if (!end->equal(*q22))
	{
		LocalPath* endNew = new LocalPath(q22, end);

		if (endNew->getValid())
		{
			paths.push_back(endNew);
		}
		else
		{
			cout << "Error: portion of path not valid" << endl;
			return;
		}
	}

	/*for(int i=0;i<paths.size();i++)
	{
		paths[i]->print();
	}*/
	//	cout << "PathSizeOf() = " << paths.size() << endl;
	// TODO optional
	//cutPortionInSmallLP(paths);


	// Free, Erases the old ones and the Inserts the new ones
	//---------------------------------------------------------------------------
	uint id1_erase = id_LP_1;
	uint id2_erase = id_LP_2 + 1;

	replacePortion(id1_erase, id2_erase, paths);
}

unsigned int Trajectory::getIdOfPathAt(double param)
{
	double soFar(0.0);

        for (unsigned int i = 0; i < nloc; i++)
        {
		soFar = soFar + mCourbe.at(i)->getParamMax();

		if (param < soFar)
		{
			return i;
		}
	}
	return nloc - 1;
}

int Trajectory::meanCollTest()
{
    int CollTest = 0.0;
    for(unsigned int i=0;i<mCourbe.size();i++)
    {
       if(!(mCourbe[i]->getValid()))
       {
           cout << "Trajectory::Warning => LocalPath is not valid in trajectory" << endl;
       }
       CollTest += mCourbe[i]->getNbColTest();
    }
    return (int)(CollTest/mCourbe.size());
}

vector<double> Trajectory::getCostAlongTrajectory(int nbSample)
{

    double step = this->getRangeMax() / (double) nbSample;

    vector<double> cost;

    for( double param=0; param<this->getRangeMax(); param = param + step)
    {
        cost.push_back(this->configAtParam(param)->cost());
//        cout << this->configAtParam(param)->cost() << endl;
    }

    cout << "Compute Cost Along Traj of " << cost.size() << " samples" << endl;

    cost.resize(nbSample);

    return cost;
}

void Trajectory::print()
{

	cout << "-------------- Trajectory --------------" << endl;
	cout << " Number of LP " << nloc << endl;
	cout << " Range Parameter " << this->range_param << endl;

	for (uint i = 0; i < nloc; i++)
	{
		cout << "Number " << i << endl;
		mCourbe.at(i)->print();
	}

	cout << "-----------------------------------" << endl;
}

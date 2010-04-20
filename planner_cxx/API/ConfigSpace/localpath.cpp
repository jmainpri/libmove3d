//
// C++ Implementation: localpath
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "../planningAPI.hpp"
//#include "../../HRI_CostSpace/HRICS_HAMP.h"

using namespace std;
using namespace tr1;

#include "Localpath-pkg.h"
#include "Planner-pkg.h"

LocalPath::LocalPath(shared_ptr<Configuration> B, shared_ptr<Configuration> E) :
  _LocalPath(NULL),
  _Begin(B), _End(E),
  _Robot(B->getRobot()),
  //_Graph(_Robot->getActivGraph()),
  _Valid(false),
  _Evaluated(false),
  _lastValidParam(0.0),
  _lastValidEvaluated(false),
  _NbColTest(0),
  _costEvaluated(false), _Cost(0.0),
  _ResolEvaluated(false), _Resolution(0.0),
  _Type(LINEAR)
{
	/*if(*_Begin == *_End)
	 {
	 cout << "Error: LocalPath _Begin == _End" << endl;
	 }*/
}

// Creates a new localpath from path, according to the connect extension method.
LocalPath::LocalPath(LocalPath& path, double& p) :
  _LocalPath(NULL),
  _Begin(path._Begin),
  _End(path.getLastValidConfig(p)),
  //	_Graph(path.getGraph()),
  // The new path represents the valid part
  // of the path given to the constructor.
  // If the parameter p is > 0,
  // the given path is at least partially valid
  // and consequently the new path is valid.
  _Robot(path.getRobot()), _Valid(p > 0),
  _Evaluated(path._Evaluated), _lastValidParam(p > 0 ? 1.0 : 0.0),
  _lastValidEvaluated(true),
  _NbColTest(path._NbColTest),
  _costEvaluated(false),  _Cost(path._Cost),
  _ResolEvaluated(false), _Resolution(0.0),
  _Type(path._Type)
{
}

LocalPath::LocalPath(const LocalPath& path) :
  _LocalPath(path._LocalPath ?
	     path._LocalPath->copy(path._Robot->getRobotStruct(), path._LocalPath) :
	     NULL),
  _Begin(path._Begin),
  _End(path._End),
  //	_Graph(path._Graph),
  _Robot(path._Robot),
  _Valid(path._Valid),
  _Evaluated(path._Evaluated),
  _lastValidParam(0.0),
  _lastValidConfig(path._lastValidConfig),  
  _lastValidEvaluated(false),
  _NbColTest(path._NbColTest),
  _costEvaluated(path._costEvaluated), _Cost(path._Cost),
  _ResolEvaluated(path._costEvaluated), _Resolution(path._Resolution),
  _Type(path._Type)
{
}

LocalPath::LocalPath(Robot* R, p3d_localpath* lpPtr) :
  _LocalPath(NULL),
  _Robot(R),
  _Valid(false),
  _Evaluated(false),
  _lastValidParam(0.0),
  _lastValidEvaluated(false),
  _NbColTest(0),
  _costEvaluated(false), _Cost(0.0),
  _ResolEvaluated(false), _Resolution(0.0),  
  _Type(lpPtr->type_lp)
{
  // TODO : check which copy are/are not necessary.
  if (lpPtr)
  {
    _LocalPath = lpPtr->copy(_Robot->getRobotStruct(), lpPtr);
    
    _Begin = shared_ptr<Configuration> (
      new Configuration(_Robot,
			p3d_copy_config(_Robot->getRobotStruct(),
					getLocalpathStruct()->config_at_param(
					  _Robot->getRobotStruct(), getLocalpathStruct(),
					  0))));
    
    _Begin->setConstraints();
    
    _End = shared_ptr<Configuration> (
      new Configuration(_Robot,
			p3d_copy_config(_Robot->getRobotStruct(),
					getLocalpathStruct()->config_at_param(
					  _Robot->getRobotStruct(), getLocalpathStruct(),
					  getLocalpathStruct()->range_param))));
    
    _End->setConstraints();
  }
  else
  {
    cout << "Warning : creating Localpath from uninitialized p3d_localpath"
	 << endl;
  }
}

LocalPath::~LocalPath()
{
  if (_LocalPath)
  {
    _LocalPath->destroy(_Robot->getRobotStruct(), _LocalPath);
  }
}

//Accessors
p3d_localpath* LocalPath::getLocalpathStruct()
{
	if (!_LocalPath)
	{
		_LocalPath = p3d_local_planner(_Robot->getRobotStruct(),
				_Begin->getConfigStruct(), _End->getConfigStruct());

		if (_LocalPath)
		{
			_Type = _LocalPath->type_lp;
		}
	}
	return _LocalPath;
}

shared_ptr<Configuration> LocalPath::getBegin()
{
	return _Begin;
}

shared_ptr<Configuration> LocalPath::getEnd()
{
	return _End;
}

//Graph* LocalPath::getGraph()
//{
//	return _Graph;
//}

Robot* LocalPath::getRobot()
{
	return _Robot;
}

bool LocalPath::getEvaluated()
{
	return _Evaluated;
}

p3d_localpath_type LocalPath::getType()
{
	if (getLocalpathStruct())
	{
		return _Type;
	}
	else
	{
		return (p3d_localpath_type) (NULL);
	}
}

shared_ptr<Configuration> LocalPath::getLastValidConfig(double& p)
{
	_lastValidConfig = shared_ptr<Configuration> (new Configuration(_Robot));
	this->classicTest();
	p = _lastValidParam;
	return (_lastValidConfig);
}

bool LocalPath::classicTest()
{
	if (!_lastValidEvaluated)
	{
		if (_lastValidConfig == NULL)
		{
			_lastValidConfig = shared_ptr<Configuration> (new Configuration(_Robot));
		}
		configPt q = _lastValidConfig->getConfigStruct();
		configPt *q_atKpath = &q;
		
		_Valid = !p3d_unvalid_localpath_classic_test(_Robot->getRobotStruct(),
							     this->getLocalpathStruct(),
							     /*&(_Graph->getGraphStruct()->nb_test_coll)*/
							     &_NbColTest,
							     &_lastValidParam, q_atKpath);
		//		LocalPathValidTest testLP(*this);
		//		_NbColTest = static_cast<unsigned int>(testLP.getNbCollisionTest());
		
		_Evaluated = true;
		_lastValidEvaluated = true;
	}
	return(_Valid);
}

bool LocalPath::getValid()
{
	if (!_Evaluated)
	{
		if (_End->IsInCollision())
		{
			_Valid = false;
//                        cout << "End in collision" << endl;
		}
		else
		{
			if (*_Begin != *_End)
			{
			  //LocalPathValidTest testLP(*this);
			  //_Valid = testLP.testIsValid();
				
			  _Valid = !p3d_unvalid_localpath_test(_Robot->getRobotStruct(),
							       this->getLocalpathStruct(), &_NbColTest);
			}
		}
		_NbColTest++;
		_Evaluated = true;
	}
	return _Valid;
}

int LocalPath::getNbColTest()
{
	if (_Evaluated)
	{
		return _NbColTest;
	}
        else
        {
            cout << "LocalPath::Warning => is not evaluated in getNbColTest" << endl;
        }
	return 0;
}

double LocalPath::length()
{
	if (this->getLocalpathStruct())
	{
		return (this->getLocalpathStruct()->length_lp);
	}
	else
	{
		if (_Begin->equal(*_End))
			return 0;
		else
		{
		  std::cout << "ERROR : in Localpath::length() : this->getLocalpathStruct() is NULL" << std::endl;
		  return(-1);
		}
	}
}

double LocalPath::getParamMax()
{
	if (*_Begin == *_End)
	{
		return 0;
	}

	return this->getLocalpathStruct()->range_param;
}

shared_ptr<Configuration> LocalPath::configAtDist(double dist)
{
	//fonction variable en fonction du type de local path
	configPt q;
	switch (getType())
	{
	case HILFLAT://hilare
		q = p3d_hilflat_config_at_distance(_Robot->getRobotStruct(),
				getLocalpathStruct(), dist);
		break;
	case LINEAR://linear
		q = p3d_lin_config_at_distance(_Robot->getRobotStruct(),
				getLocalpathStruct(), dist);
		break;
	case MANHATTAN://manhatan
		q = p3d_manh_config_at_distance(_Robot->getRobotStruct(),
				getLocalpathStruct(), dist);
		break;
	case REEDS_SHEPP://R&S
		q = p3d_rs_config_at_distance(_Robot->getRobotStruct(),
				getLocalpathStruct(), dist);
		break;
	case TRAILER:
		q = p3d_trailer_config_at_distance(_Robot->getRobotStruct(),
				getLocalpathStruct(), dist);
		break;
	default:
	  // TODO : implement those methods !
	  std::cout << "ERROR : LocalPath::configAtDist : the TRAILER_FORWARD, HILFLAT_FORWARD, and DUBINS localpath types are not implemented." << std::endl;
	}
	return shared_ptr<Configuration> (new Configuration(_Robot, q));
}

shared_ptr<Configuration> LocalPath::configAtParam(double param)
{
	//fonction variable en fonction du type de local path
	configPt q;

	if (param > getParamMax())
	{
		return _End;
	}
	if (param < 0)
	{
		return _Begin;
	}

	/*switch (getType())
	 {
	 case HILFLAT://hilare
	 q = p3d_hilflat_config_at_param(_Robot->getRobotStruct(),
	 getLocalpathStruct(), param);
	 break;
	 case LINEAR://linear
	 q = p3d_lin_config_at_distance(_Robot->getRobotStruct(),
	 getLocalpathStruct(), param);
	 break;
	 case MANHATTAN://manhatan
	 q = p3d_manh_config_at_distance(_Robot->getRobotStruct(),
	 getLocalpathStruct(), param);
	 break;
	 case REEDS_SHEPP://R&S
	 q = p3d_rs_config_at_param(_Robot->getRobotStruct(),
	 getLocalpathStruct(), param);
	 break;
	 case TRAILER:
	 q = p3d_trailer_config_at_param(_Robot->getRobotStruct(),
	 getLocalpathStruct(), param);
	 break;
	 }*/

	q = getLocalpathStruct()->config_at_param(_Robot->getRobotStruct(),
			getLocalpathStruct(), param);

	shared_ptr<Configuration> ptrQ(new Configuration(_Robot, q));
	ptrQ->setConstraints();
	return ptrQ;
}

double LocalPath::stayWithInDistance(double u, bool goForward, double* distance)
{
	int way;
	
	if (goForward) {
		way = 1;
	}
	else {
		way = -1;
	}

	double du = getLocalpathStruct()->stay_within_dist(_Robot->getRobotStruct() , getLocalpathStruct(), 
											  u,
											  way, 
											  distance);
	
	return du;
}

bool LocalPath::unvalidLocalpathTest(Robot* R, int* ntest)
{
	return p3d_unvalid_localpath_test(R->getRobotStruct(),
			getLocalpathStruct(), ntest);
}

double LocalPath::getResolution()
{
	if (!_ResolEvaluated)
	{
		if (p3d_get_env_dmax() > getParamMax())
		{
			_Resolution = getParamMax();
			_ResolEvaluated = true;
			return _Resolution;
		}

		_Resolution = getParamMax() / (double) (int) ((getParamMax()
                                / (p3d_get_env_dmax() / ENV.getDouble(Env::costStep))) + 0.5);
		_ResolEvaluated = true;
		//		cout << "_Resolution = " << _Resolution << endl;
		return _Resolution;
	}
	else
	{
		//		cout << "_Resolution = " << _Resolution << endl;
		return _Resolution;
	}
}

double LocalPath::cost()
{
	if (!ENV.getBool(Env::isCostSpace))
	{
		return getParamMax();
	}

	if (!_costEvaluated)
	{
		_Cost = 0;

		double currentCost, prevCost;
                Vector3d taskPos, prevTaskPos;

		double currentParam = 0;

                double DeltaStep = getResolution();
                double CostDistStep = DeltaStep;

//                cout << "DeltaStep  = "  << DeltaStep << endl;
                unsigned int nStep = getParamMax() / DeltaStep;

		shared_ptr<Configuration> confPtr;
		prevCost = _Begin->cost();
		
		if(ENV.getBool(Env::HRIPlannerWS))
		{
			prevTaskPos = _Begin->getTaskPos();
		}
		// Case of task space
		vector<double> Pos;

//                cout << "nStep =" << nStep << endl;
		for (unsigned int i = 0; i < nStep; i++)
		{
                        currentParam += DeltaStep;

			confPtr = configAtParam(currentParam);
			currentCost = confPtr->cost();

			// Case of task space
                        if(ENV.getBool(Env::HRIPlannerWS) )
                        {
                                taskPos = confPtr->getTaskPos();
                                CostDistStep = ( taskPos - prevTaskPos ).norm();
                                prevTaskPos = taskPos;
                        }
                        _Cost += p3d_ComputeDeltaStepCost(prevCost, currentCost, CostDistStep);

			prevCost = currentCost;
		}
		_costEvaluated = true;
	}

	return _Cost;
}

void LocalPath::print()
{
	cout << "------------ Localpath Description----------------" << endl;
	cout << "mBegin =>" << endl;
	_Begin->print();
	cout << "mEnd   =>" << endl;
	_End->print();
	cout << "range_param = " << this->getParamMax() << endl;
	cout << "length = " << length() << endl;
	cout << "--------------- End  Description ------------------" << endl;
}

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

using namespace std;
using namespace tr1;

LocalPath::LocalPath(shared_ptr<Configuration> B, shared_ptr<Configuration> E) :
	_Begin(B),
	_End(E),
	_LocalPath(NULL),
	_Robot(B->getRobot()),
			//	_Graph(_Robot->getActivGraph()),
	_Valid(false),
	_Evaluated(false),
	_lastValidParam(0.0),
	_lastValidEvaluated(false),
	_Type(LINEAR),
	_costEvaluated(false),
	_ResolEvaluated(false),
	_Cost(0.0),
	_NbColTest(0)
{
}

LocalPath::LocalPath(LocalPath& path, double& p) :
	_LocalPath(0x00),
	_Begin(path._Begin),
	_End(path.getLastValidConfig(p)),
			//	_Graph(path.getGraph()),
	_Robot(path.getRobot()),
	_Valid(false),
	_Evaluated(path._Evaluated),
	_lastValidParam(0.0),
	_lastValidEvaluated(false),
	_Type(path._Type),
	_costEvaluated(false),
	_ResolEvaluated(false),
	_Cost(path._Cost),
	_NbColTest(path._NbColTest)
{
	if(_Evaluated)
	{
		_Valid = path.getValid();
	}
}

LocalPath::LocalPath(const LocalPath& path) :
	_LocalPath(path._LocalPath),
	_Begin(path._Begin),
	_End(path._End),
	_lastValidConfig(path._lastValidConfig),
			//	_Graph(path._Graph),
	_Robot(path._Robot),
	_Valid(false),
	_Evaluated(path._Evaluated),
	_lastValidParam(0.0),
	_lastValidEvaluated(false),
	_Type(path._Type),
	_costEvaluated(path._costEvaluated),
	_ResolEvaluated(path._costEvaluated),
	_Cost(path._Cost),
	_NbColTest(path._NbColTest)
{
	if (path._LocalPath)
	{
		_LocalPath = path._LocalPath->copy(_Robot->getRobotStruct(),
				path._LocalPath);
	}
	else
	{
		_LocalPath = NULL;
	}
}

LocalPath::LocalPath(Robot* R, p3d_localpath* lpPtr) :
	_LocalPath(NULL),
	_Robot(R),
			//	_Graph(_Robot->getActivGraph()),
	_Valid(false),
	_Evaluated(false),
	_lastValidParam(0.0),
	_lastValidEvaluated(false),
	_Type(lpPtr->type_lp),
	_costEvaluated(false),
	_ResolEvaluated(false),
	_Cost(0.0),
	_NbColTest(0)
{
	if (lpPtr)
	{
		_LocalPath = lpPtr->copy(_Robot->getRobotStruct(), lpPtr);

		_Begin = shared_ptr<Configuration> (new Configuration(_Robot,
				p3d_copy_config(_Robot->getRobotStruct(),
						getLocalpathStruct()->specific.lin_data->q_init)));

		_End = shared_ptr<Configuration> (new Configuration(_Robot,
				p3d_copy_config(_Robot->getRobotStruct(),
						getLocalpathStruct()->specific.lin_data->q_end)));
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
		_LocalPath = p3d_local_planner(
				_Robot->getRobotStruct(),
				_Begin->getConfigStruct(),
				_End->getConfigStruct());

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
		return (p3d_localpath_type)(NULL);
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
			_lastValidConfig = shared_ptr<Configuration> (new Configuration(
					_Robot));
		}
		configPt q = _lastValidConfig->getConfigStruct();
		configPt *q_atKpath = &q;

		_Valid = !p3d_unvalid_localpath_classic_test(_Robot->getRobotStruct(),
				this->getLocalpathStruct(),
				/*&(_Graph->getGraphStruct()->nb_test_coll)*/&_NbColTest,
				&_lastValidParam, q_atKpath);

//		Configuration* end = _End.get();
//		cout << "dist = " << _Begin->dist(*end) << endl;
//		cout << "_lastValidParam = " << _lastValidParam << endl;

		_Evaluated = true;
		_lastValidEvaluated = true;
	}

	if(_Valid){
		return true;
	}
}

bool LocalPath::getValid()
{
	if (!_Evaluated)
	{
//		this->classicTest();
		if( _End->IsInCollision() )
		{
			_Valid =  false;
		}
		else
		{
				_Valid = !p3d_unvalid_localpath_test(
						_Robot->getRobotStruct(),
						this->getLocalpathStruct(),
						&_NbColTest);
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
	}
}

double LocalPath::getParamMax()
{
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
	}
	return shared_ptr<Configuration> (new Configuration(_Robot, q));
}

shared_ptr<Configuration> LocalPath::configAtParam(double param)
{
	//fonction variable en fonction du type de local path
	configPt q;
	switch (_Type)
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
	}
	return shared_ptr<Configuration> (new Configuration(_Robot, q));
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
		_Resolution = getParamMax() / (double) (int) ((getParamMax()
				/ p3d_get_env_dmax()) + 0.5);
		return _Resolution;
	}
	else
	{
		return _Resolution;
	}
}

double LocalPath::cost()
{
	if (!_costEvaluated)
	{

		_Cost = 0;

		double currentCost, prevCost;
		double currentParam = 0;

		double dist = getResolution();

		shared_ptr<Configuration> confPtr;
		prevCost = _Begin->cost();

		for (int i = 0; i < (int) ((getParamMax() / p3d_get_env_dmax()) + 0.5); i++)
		{

			currentParam += dist;

			confPtr = configAtParam(currentParam);
			currentCost = confPtr->cost();

			//			cout << "prevCost = " << prevCost << " currentCost = " << currentCost << endl;
			//			cout << "subPath(" << i << ") = " << p3d_ComputeDeltaStepCost(prevCost,currentCost,dist) << endl;

			_Cost += p3d_ComputeDeltaStepCost(prevCost, currentCost, dist);

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
	cout << "range_param = " << this->getLocalpathStruct()->range_param << endl;
	cout << "length = " << length() << endl;
	cout << "--------------- End  Description ------------------" << endl;
}

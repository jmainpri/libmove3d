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

LocalPath::LocalPath(shared_ptr<Configuration> B, shared_ptr<Configuration> E)
{
  _Begin = B;
  _End = E;
  _LocalPath = NULL;
  _Robot = B->getRobot();
  _Graph = _Robot->getActivGraph();
  _Valid = FALSE;
  _Evaluated = FALSE;
  _lastValidParam = 0.0;
  _lastValidEvaluated = FALSE;
}

LocalPath::LocalPath(LocalPath& path, double& p) :
  _LocalPath(0x00),
  _Begin(path.getBegin()),
  _End(path.getLastValidConfig(p)),
  _Graph(path.getGraph()),
  _Robot(path.getRobot()),
  _Valid(false),
  _Evaluated(path.getEvaluated()),
  _lastValidParam(0.0),
  _lastValidEvaluated(false),
  _Type(path.getType()){

	if(_Evaluated)
	{
	_Valid = path.getValid();
	}
}

LocalPath::LocalPath(const LocalPath& path) :
  _LocalPath(path._LocalPath),
  _Begin(path._Begin),
  _End(path._lastValidConfig),
  _Graph(path._Graph),
  _Robot(path._Robot),
  _Valid(false),
  _Evaluated(path._Evaluated),
  _lastValidParam(0.0),
  _lastValidEvaluated(false),
  _Type(path._Type)
/*	cost_evaluated(path.cost_evaluated),
	resol_evaluated(false)*/
{
	if(path._LocalPath)
		_LocalPath = path._LocalPath->copy(_Robot->getRobotStruct(),path._LocalPath);
	else
		_LocalPath = NULL;
}

LocalPath::~LocalPath()
{
  if(_LocalPath)
    _LocalPath->destroy(_Robot->getRobotStruct(), _LocalPath);
}

//Accessors
p3d_localpath* LocalPath::getLocalpathStruct()
{
  if(!_LocalPath)
  {
    _LocalPath = p3d_local_planner(
    		_Robot->getRobotStruct(),
    		_Begin->getConfigStruct(),
    		_End->getConfigStruct());

    if(_LocalPath)
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

Graph* LocalPath::getGraph()
{
  return _Graph;
}

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
  return _Type;
}

shared_ptr<Configuration> LocalPath::getLastValidConfig(double& p)
{
  _lastValidConfig = shared_ptr<Configuration>(new Configuration(_Robot));
  this->classicTest();
  p = _lastValidParam;
  return(_lastValidConfig);
}

void LocalPath::classicTest()
{
  if(!_lastValidEvaluated)
  {
    if (_lastValidConfig ==NULL)
    {
      _lastValidConfig = shared_ptr<Configuration>(new Configuration(_Robot));
    }
    configPt q = _lastValidConfig->getConfigStruct();
    configPt *q_atKpath = &q;

    _Valid = !p3d_unvalid_localpath_classic_test(
    		_Robot->getRobotStruct(),
    		this->getLocalpathStruct(),
    		&(_Graph->getGraphStruct()->nb_test_coll),
    		&_lastValidParam,
    		q_atKpath);

    _Evaluated = true;
    _lastValidEvaluated = true;
  }
}

bool LocalPath::getValid()
{
  this->classicTest();
  return _Valid;
}

double LocalPath::length()
{
  if(this->getLocalpathStruct())
  {
    return(this->getLocalpathStruct()->length_lp);
  }
  else
  {
    if (_Begin->equal(*_End))
      return 0;
  }
}

shared_ptr<Configuration> LocalPath::configAtDist(double dist)
{
  //fonction variable en fonction du type de local path
  configPt q;
  switch (_Type){
  case HILFLAT://hilare
    q = p3d_hilflat_config_at_distance(_Robot->getRobotStruct(), _LocalPath, dist);
    break;
  case LINEAR://linear
    q = p3d_lin_config_at_distance(_Robot->getRobotStruct(), _LocalPath, dist);
    break;
  case MANHATTAN://manhatan
    q = p3d_manh_config_at_distance(_Robot->getRobotStruct(), _LocalPath, dist);
    break;
  case REEDS_SHEPP://R&S
    q = p3d_rs_config_at_distance(_Robot->getRobotStruct(), _LocalPath, dist);
    break;
  case TRAILER:
    q = p3d_trailer_config_at_distance(_Robot->getRobotStruct(), _LocalPath, dist);
    break;
  }
  return shared_ptr<Configuration>(new Configuration(_Robot,q));
}

shared_ptr<Configuration> LocalPath::configAtParam(double param)
{
  //fonction variable en fonction du type de local path
  configPt q;
  switch (_Type){
  case HILFLAT://hilare
    q = p3d_hilflat_config_at_param(_Robot->getRobotStruct(), _LocalPath, param);
    break;
  case LINEAR://linear
    q = p3d_lin_config_at_distance(_Robot->getRobotStruct(), _LocalPath, param);
    break;
  case MANHATTAN://manhatan
    q = p3d_manh_config_at_distance(_Robot->getRobotStruct(), _LocalPath, param);
    break;
  case REEDS_SHEPP://R&S
    q = p3d_rs_config_at_param(_Robot->getRobotStruct(), _LocalPath, param);
    break;
  case TRAILER:
    q = p3d_trailer_config_at_param(_Robot->getRobotStruct(), _LocalPath, param);
    break;
  }
  return shared_ptr<Configuration>(new Configuration(_Robot,q));
}

bool LocalPath::unvalidLocalpathTest(Robot* R,int* ntest)
{
  if(!_LocalPath)
    _LocalPath = p3d_local_planner(
    		_Robot->getRobotStruct(),
    		_Begin->getConfigStruct(),
    		_End->getConfigStruct());

  return p3d_unvalid_localpath_test(R->getRobotStruct(),_LocalPath, ntest);
}

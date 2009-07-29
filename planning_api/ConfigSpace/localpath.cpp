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

LocalPath::LocalPath(LocalPath& path, double& p)
{
  _LocalPath = NULL;
  _Begin = path.getBegin();
  _End = path.getLastValidConfig(p);
  _Graph = path.getGraph();
  _Robot = path.getRobot();
  _Valid = TRUE;
  _Evaluated = TRUE;
  _lastValidParam = 0.0;
  _lastValidEvaluated = FALSE;
  _Type = path.getType();
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
    _LocalPath = p3d_local_planner(_Robot->getRobotStruct(), _Begin->getConfigurationStruct(), _End->getConfigurationStruct());
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
    _Valid = !p3d_unvalid_localpath_classic_test(_Robot->getRobotStruct(), this->getLocalpathStruct(), &(_Graph->getGraphStruct()->nb_test_coll), &_lastValidParam, _lastValidConfig->getConfigPtStruct());
    _Evaluated = true;
    _lastValidEvaluated = true;
  }
}

bool LocalPath::getValid()
{
  this->classicTest();
  return _Valid;
}

double LocalPath::Length()
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

shared_ptr<Configuration> LocalPath::configAtDist(Robot* R, double dist)
{
  //fonction variable en fonction du type de local path
  configPt q;
  switch (_Type){
  case HILFLAT://hilare
    q = p3d_hilflat_config_at_distance(R->getRobotStruct(), _LocalPath, dist);
    break;
  case LINEAR://linear
    q = p3d_lin_config_at_distance(R->getRobotStruct(), _LocalPath, dist);
    break;
  case MANHATTAN://manhatan
    q = p3d_manh_config_at_distance(R->getRobotStruct(), _LocalPath, dist);
    break;
  case REEDS_SHEPP://R&S
    q = p3d_rs_config_at_distance(R->getRobotStruct(), _LocalPath, dist);
    break;
  case TRAILER:
    q = p3d_trailer_config_at_distance(R->getRobotStruct(), _LocalPath, dist);
    break;
  }
  return shared_ptr<Configuration>(new Configuration(R,q));
}

shared_ptr<Configuration> LocalPath::configAtParam(Robot* R, double param)
{
  //fonction variable en fonction du type de local path
  configPt q;
  switch (_Type){
  case HILFLAT://hilare
    q = p3d_hilflat_config_at_param(R->getRobotStruct(), _LocalPath, param);
    break;
  case LINEAR://linear
    q = p3d_lin_config_at_distance(R->getRobotStruct(), _LocalPath, param);
    break;
  case MANHATTAN://manhatan
    q = p3d_manh_config_at_distance(R->getRobotStruct(), _LocalPath, param);
    break;
  case REEDS_SHEPP://R&S
    q = p3d_rs_config_at_param(R->getRobotStruct(), _LocalPath, param);
    break;
  case TRAILER:
    q = p3d_trailer_config_at_param(R->getRobotStruct(), _LocalPath, param);
    break;
  }
  return shared_ptr<Configuration>(new Configuration(R,q));
}

bool LocalPath::unvalidLocalpathTest(Robot* R,int* ntest)
{
  if(!_LocalPath)
    _LocalPath = p3d_local_planner(_Robot->getRobotStruct(), _Begin->getConfigurationStruct(), _End->getConfigurationStruct());
  return p3d_unvalid_localpath_test(R->getRobotStruct(),_LocalPath, ntest);
}

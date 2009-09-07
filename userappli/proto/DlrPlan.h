#ifndef __DLRPLAN_H__
#define __DLRPLAN_H__
#include <iostream>
#include <vector>
#include "../userappli/proto/DlrObject.h"
#include "p3d_matrix.h"

class DlrPlan {
public:
	enum planType{APPROACH, GRASP, CARRY};
  //Constructors and destructors
  DlrPlan(planType type);
  virtual ~DlrPlan();
	//functions
	void setObstaclesAtRightPos();
  //setters and getters
	void setType(planType type);
	void setObject(DlrObject* object);
	void setStartPos(DlrObject* object, int posId);
	void setTargetPos(DlrObject* object, int posId);
	void addObstacle(DlrObject* object, int posId);
	void setExecute(bool execute);
	planType getType();
	DlrObject* getObject();
	void getStartPos(p3d_matrix4 startPos);
	void getTargetPos(p3d_matrix4 targetPos);
	bool getExecute();
	
protected:

private:
//static members
	planType _type;
	DlrObject* _object;
	p3d_matrix4 _startPos;
	p3d_matrix4 _targetPos;
	std::map<DlrObject*, int> _obstacles;
	bool _execute;
public:
};

#endif

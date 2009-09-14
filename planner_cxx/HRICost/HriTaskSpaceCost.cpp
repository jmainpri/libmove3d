/*
 * HRITaskSpaceCost.cpp
 *
 *  Created on: Sep 8, 2009
 *      Author: jmainpri
 */

#include "HriTaskSpaceCost.hpp"

using namespace std;

HriSpaceCost* hriSpace;

HriSpaceCost::HriSpaceCost(p3d_rob* rob,int jnt) :
	_JntId(jnt),
	_test(0)
{
	_Robot = new Robot(rob);
	_Bitmap = hri_exp_init();
}

HriSpaceCost::~HriSpaceCost()
{
 // WARNING Implement
}

vector<int> HriSpaceCost::getTaskPosition()
{

	vector<int> pos1(3);

	vector<double> pos2 = _Robot->getJointPos(_JntId);


	pos1[0] = (int)((pos2[0]-_Bitmap->realx)/_Bitmap->pace);
	pos1[1] = (int)((pos2[1]-_Bitmap->realy)/_Bitmap->pace);
	pos1[2] = (int)((pos2[2]-_Bitmap->realz)/_Bitmap->pace);

//	cout << "Task is at :"
//				<< " ( " << pos2[0]
//				<< " , " << pos2[1]
//				<< " , " << pos2[2] << " ) " << endl;
//
//	cout << "Task is at :"
//			<< " ( " << pos1[0]
//			<< " , " << pos1[1]
//			<< " , " << pos1[2] << " ) " << endl;

	return pos1;
}

void HriSpaceCost::changeTask(int idJnt)
{
	_JntId = idJnt;
}

int HriSpaceCost::test()
{
	return _test;
}

void HriSpaceCost::changeTest(int i)
{
	_test = i;
}

double HriSpaceCost::distanceCost()
{
	vector<int> pos = getTaskPosition();

	double cost = hri_exp_distance_val(_Bitmap,
			pos.at(0),
			pos.at(1),
			pos.at(2));

//	cout << "Cost is : " << cost << endl;

	return cost;
}

double HriSpaceCost::visibilityCost()
{
	vector<int> pos = getTaskPosition();

	return hri_exp_vision_val(_Bitmap,
			pos.at(0),
			pos.at(1),
			pos.at(2));

//	return 0;
//	hri_exp_combined_val(hri_bitmapset* btset, int x, int y, int z)
}

double HriSpaceCost::comfortCost()
{
	vector<int> pos = getTaskPosition();

	return hri_exp_hcomfort_val(_Bitmap,
				pos.at(0),
				pos.at(1),
				pos.at(2));

}

double HriSpaceCost::combinedCost()
{
	vector<int> pos = getTaskPosition();

	return hri_exp_combined_val(_Bitmap,
			pos.at(0),
			pos.at(1),
			pos.at(2));
}

double HriSpaceCost::switchCost()
{
	vector<int> pos = getTaskPosition();

	switch(_test)
	{
		case 1 :	distanceCost(); break;
		case 2 :	comfortCost() ; break;
		case 3 : 	visibilityCost(); break;
		case 4 : 	combinedCost(); break;
		default : 	cout << "No Cost" << endl;
	}
}

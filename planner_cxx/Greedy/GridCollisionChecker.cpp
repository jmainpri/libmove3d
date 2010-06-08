/*
 *  GridCollisionChecker.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "GridCollisionChecker.h"
#include "CellCollisionChecker.h"

#include "P3d-pkg.h"

GridCollisionChecker* global_GridCollisionChecker = NULL;

using namespace std;
using namespace tr1;

void p3d_compute_object_point_cloud(p3d_obj* obj, double step)
{
	for(int i = 0; i < obj->np; i++)
	{
		cout << "Object : " << obj->name << endl;
		
		if(obj->pol[i]->TYPE != P3D_GRAPHIC)
		{
			p3d_polyhedre* poly = obj->pol[i]->poly;
			p3d_vector3 the_points[poly->nb_points];
			for(unsigned int j = 0; j < poly->nb_points; j++)
			{
				the_points[j][0] = poly->the_points[j][0];
				the_points[j][1] = poly->the_points[j][1];
				the_points[j][2] = poly->the_points[j][2];
				if (obj->type == P3D_OBSTACLE)
				{//real point position
					p3d_xformPoint(obj->pol[i]->pos0, poly->the_points[j], the_points[j]);
				}
				else
				{
					p3d_matrix4 inv_pos, mat;
					p3d_matInvertXform( obj->jnt->pos0, inv_pos );
					p3d_matMultXform(inv_pos, obj->pol[i]->pos0, mat);
					p3d_xformPoint(mat, poly->the_points[j], the_points[j]);
				}
			}
			
			for(unsigned int j = 0; j < poly->nb_faces; j++)
			{
				unsigned int nbPoints = 0;
				
				p3d_vector3* tmp = sample_triangle_surface(
														   the_points[poly->the_faces[j].the_indexs_points[0] - 1], 
														   the_points[poly->the_faces[j].the_indexs_points[1] - 1], 
														   the_points[poly->the_faces[j].the_indexs_points[2] - 1], step, &nbPoints);
				
				obj->pointCloud = MY_REALLOC(obj->pointCloud, p3d_vector3, obj->nbPointCloud, obj->nbPointCloud + nbPoints);
				
				for(unsigned int k = 0; k < nbPoints; k++)
				{
					obj->pointCloud[obj->nbPointCloud + k][0] = tmp[k][0];
					obj->pointCloud[obj->nbPointCloud + k][1] = tmp[k][1];
					obj->pointCloud[obj->nbPointCloud + k][2] = tmp[k][2];
				}
				obj->nbPointCloud += nbPoints;
				free(tmp);
			}
		}
	}
}

void p3d_compute_static_objects_point_cloud(p3d_env* env, double step)
{
	for(int i = 0; i < env->no; i++)
	{
		p3d_compute_object_point_cloud(env->o[i], step);
	}
}

void p3d_compute_robot_bodies_point_cloud(p3d_rob* robot, double step)
{
	for(int i = 0; i <= robot->njoints; i++)
	{
		if(robot->joints[i]->o)
		{
			p3d_compute_object_point_cloud(robot->joints[i]->o, step);
		}
	}
}

void p3d_compute_all_robots_bodies_point_cloud(p3d_env* env, double step)
{
	for(int i = 0; i < env->nr; i++)
	{
		p3d_compute_robot_bodies_point_cloud(env->robot[i], step);
	}
}


GridCollisionChecker::GridCollisionChecker()
{
	m_nbMaxCells = 30;
	
	double cellSize = (XYZ_ENV->box.x2 - XYZ_ENV->box.x1);
	
	cellSize = MAX(XYZ_ENV->box.y2 - XYZ_ENV->box.y1, cellSize);
	cellSize = MAX(XYZ_ENV->box.z2 - XYZ_ENV->box.z1, cellSize);
	cellSize /= m_nbMaxCells;
	
	_originCorner[0] = XYZ_ENV->box.x1;
	_originCorner[1] = XYZ_ENV->box.y1;
	_originCorner[2] = XYZ_ENV->box.z1;
	
	_nbCellsX = (XYZ_ENV->box.x2 - XYZ_ENV->box.x1)/cellSize;
	_nbCellsY = (XYZ_ENV->box.y2 - XYZ_ENV->box.y1)/cellSize;
	_nbCellsZ = (XYZ_ENV->box.z2 - XYZ_ENV->box.z1)/cellSize;
	
	//_nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;
	_cellSize[0] = _cellSize[1] = _cellSize[2] = cellSize;
	
	this->createAllCells();
	
	cout << "Cell size(0) = " << _cellSize[0] << endl;
	cout << "Cell size(1) = " << _cellSize[1] << endl;
	cout << "Cell size(2) = " << _cellSize[2] << endl;
	
	cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
	
	//Build the meshes env edges
	if(XYZ_ENV->o[0]->pol[0]->poly->the_edges == NULL)
	{
		for(int i = 0; i < XYZ_ENV->no; i++)
		{
			p3d_obj * obj = XYZ_ENV->o[i];
			for(int j = 0; j < obj->np; j++)
			{
				poly_build_edges(obj->pol[j]->poly);
			}
		}
	}
	
	init();
	
	m_Robot = global_Project->getActiveScene()->getActiveRobot();
}

GridCollisionChecker::~GridCollisionChecker()
{
	
}

void GridCollisionChecker::init(void)
{
	p3d_compute_static_objects_point_cloud(XYZ_ENV, _cellSize[0]*0.25);
	p3d_compute_all_robots_bodies_point_cloud(XYZ_ENV, _cellSize[0]*0.25);
	
	//unvalid static object Cells
	for(int i = 0; i < XYZ_ENV->no; i++)
	{
		unvalidObjectCells(XYZ_ENV->o[i]);
	}
	//unvalid cells for each robot except current one ?
}

void GridCollisionChecker::updateRobotOccupationCells(Robot* myRobot)
{
	vector<CellCollisionChecker*> robotCell;
	
	for(unsigned int i = 0; i < getNumberOfCells(); i++)
	{
		dynamic_cast<CellCollisionChecker*>(_cells[i])->setOccupied(false);
	}
	
	for(unsigned int i = 0; i < myRobot->getNumberOfJoints(); i++)
	{
		p3d_obj* obj = myRobot->getJoint(i)->getJointStruct()->o;
		
		if(obj)
		{
			vector<CellCollisionChecker*> objectCell = getCellListForObject( 
																			obj, 
																			myRobot->getJoint(i)->getMatrixPos() );
			
			robotCell.insert(robotCell.end(), objectCell.begin(), objectCell.end());
		}
	}
	
	m_OccupationCells = robotCell;
	
//	MY_FREE(robot->dpgCells, DpgCell*,  robot->nbDpgCells);
//	robot->nbDpgCells = robotCell.size();
//	robot->dpgCells = MY_ALLOC(DpgCell* , robot->nbDpgCells);
	
	for(unsigned int i = 0; i < robotCell.size(); i++)
	{
		m_OccupationCells[i]->setOccupied(true);
	}
}

void GridCollisionChecker::unvalidObjectCells(p3d_obj* obj)
{
	Eigen::Transform3d Trans(Eigen::Matrix4d::Identity());
	
	vector<CellCollisionChecker*> cellTab = getCellListForObject(obj,Trans);
	
	for(unsigned int i = 0; i < cellTab.size(); i++)
	{
		cellTab[i]->setValid(false);
	}
}

vector<CellCollisionChecker*> GridCollisionChecker::getCellListForObject(p3d_obj* obj, const Eigen::Transform3d& Trans)
{
	vector<CellCollisionChecker*> objectCells;
	
	for(unsigned int i = 0; i < obj->nbPointCloud; i++)
	{
		Eigen::Vector3d WSPoint;
		
		p3d_vector3 vect;
		
		p3d_vectCopy(obj->pointCloud[i],vect);
		
		WSPoint[0] = vect[0];
		WSPoint[1] = vect[1];
		WSPoint[2] = vect[2];
		
		WSPoint = Trans * WSPoint;

		CellCollisionChecker* cell = dynamic_cast<CellCollisionChecker*>(getCell(WSPoint));
		
		if( cell == NULL )
		{
			continue;
		}
		
		if(!cell->isVisited())
		{
			cell->setVisited(true);
			objectCells.push_back(cell);
		}
	}
	
	for(unsigned int i = 0; i < objectCells.size(); i++)
	{
		objectCells[i]->setVisited(false);
	}
	return objectCells;
}

vector<CellCollisionChecker*> GridCollisionChecker::computeOccupiedCells(LocalPath& path)
{
	double ParamMax = path.getParamMax();
	double Step = path.getResolution();
	double u = 0.0;
	
	set<CellCollisionChecker*> setCells;
	
	while( u < ParamMax ) 
	{
		shared_ptr<Configuration> q = path.configAtParam(u);
		
		if( m_Robot->setAndUpdate(*q) )
		{
			updateRobotOccupationCells(m_Robot);
			setCells.insert( m_OccupationCells.begin(), m_OccupationCells.end() );
		}
		u += Step;
	}
	
	
	vector<CellCollisionChecker*> vectCells;
	set<CellCollisionChecker*>::iterator it;
	
	for (it=setCells.begin(); it!=setCells.end(); it++)
	{
		vectCells.push_back(*it);
	}
	
	return vectCells;
}

bool GridCollisionChecker::areCellsValid(vector<CellCollisionChecker*> cells)
{
	for (unsigned int i=0; i<cells.size(); i++) 
	{
		if (!cells[i]->isValid()) 
		{
			return false;
		}
	}
	
	return true;
}

void GridCollisionChecker::draw()
{
	CellCollisionChecker* cell;
	
	updateRobotOccupationCells(m_Robot);
	
	for(unsigned int i=0; i < getNumberOfCells(); i++)
	{
		cell = static_cast<CellCollisionChecker*>(_cells[i]);
		cell->draw();
	}
}

//protected functions
/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param index the cell index
 * \param x The position of the cell over x
 * \param y The position of the cell over y
 * \param z The position of the cell over z
 */
CellCollisionChecker* GridCollisionChecker::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
{
    CellCollisionChecker* newCell = new CellCollisionChecker( index, computeCellCorner(x,y,z) , this );
    return newCell;
}

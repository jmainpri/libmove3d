/*
 *  hri_distance.c
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 05/11/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Hri_planner-pkg.h"

#include <limits>

double								hri_cost_to_display=0;
bool									hri_draw_distance=true;
std::vector<double>		hri_disp_dist; // to draw the distance between the human and the robot
std::vector<double>		hri_histo_dist; // to save all distances that have been computed
static const double		hri_safe_radius = 1.0; // in meters

//! Enables colision detection
//! between the robot and the human
void hri_set_human_robot_dist( p3d_rob* rob, HRI_AGENTS* agents )
{
	for(int i=0; i<agents->humans_no; i++)
	{
//		std::cout << "Human is : " << GLOBAL_AGENTS->humans[i]->robotPt->name << std::endl;
		p3d_col_activate_rob_rob(rob,
														 agents->humans[i]->robotPt);
	}
}

//! Sets the collision checker in a normal
//! detection mode
void hri_set_normal_dist(p3d_rob* rob)
{

}

//! Comptue the distance between the
//! robot and the closest human agent
double hri_robot_min_distance( HRI_AGENTS* agents )
{
	p3d_rob* rob = agents->robots[agents->source_agent_idx]->robotPt;

//	printf("hri_robot_min_distance\n");
	hri_set_human_robot_dist(rob,agents);

	int nof_bodies = rob->no;
	double* distances = new double[nof_bodies];
	p3d_vector3 *body = new p3d_vector3[nof_bodies];
	p3d_vector3 *other = new p3d_vector3[nof_bodies];

	int k=0;
	double minDist = std::numeric_limits<double>::max();

	// Checkout the collision checker version
	switch (p3d_col_get_mode())
	{
		case p3d_col_mode_kcd:
		{
			int settings = get_kcd_which_test();
			set_kcd_which_test((p3d_type_col_choice)(20+3));
			// 40 = KCD_ROB_ENV
			// 3 = DISTANCE_EXACT

			p3d_col_test_choice();
			// Collision detection with other robots only

			p3d_kcd_closest_points_between_bodies(rob,body,other,distances);

			// Get Min indice of distance
			for(int i=0;i<nof_bodies;i++)
			{
				std::cout << "distances[" << i << "] = " << distances[i] << std::endl;
				if( minDist > distances[i] )
				{
					minDist = distances[i];
					k = i;
				}
			}

			set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL

			break;
		}
		case p3d_col_mode_pqp:
		{
			minDist =  pqp_robot_robot_distance(rob,
																					GLOBAL_AGENTS->humans[0]->robotPt,
																					body[k],
																					other[k]);
			break;
		}
	}

	if(hri_draw_distance)
	{
		hri_disp_dist.clear();
		hri_disp_dist.push_back(body[k][0]);
		hri_disp_dist.push_back(body[k][1]);
		hri_disp_dist.push_back(body[k][2]);
		hri_disp_dist.push_back(other[k][0]);
		hri_disp_dist.push_back(other[k][1]);
		hri_disp_dist.push_back(other[k][2]);

//		std::cout	<< "vect_jim[0] = " << hri_disp_dist[0]
//							<< " vect_jim[1] = " << hri_disp_dist[1]
//					    << " vect_jim[2] = " << hri_disp_dist[2] << std::endl;
//
//		std::cout	<< "vect_jim[3] = " << hri_disp_dist[3]
//							<< " vect_jim[4] = " << hri_disp_dist[4]
//					    << " vect_jim[5] = " << hri_disp_dist[5] << std::endl;
	}

//	hri_set_normal_dist(rob);

	delete[] distances;
	delete[] other;
	delete[] body;

	return minDist;
}

//! Cost between 0 and 1 for the distance to the robot
//! the minimal distance is computed and a squaling is done to have
//! a cost in [0 1]
double hri_distance_cost(HRI_AGENTS* agents, double& distance)
{
	distance = hri_robot_min_distance(agents);

	double penetrationDist = (hri_safe_radius - distance)/hri_safe_radius;

	double Cost = 0.00001;
	// Compute of the hri cost function
	if ( penetrationDist > 0 )
	{
		Cost += (exp(penetrationDist-1) - exp(-1) ) / ( 1 - exp(-1) );
		//            Cost += _PenetrationDist[k];
	}


	// Set the Hri Cost To Display
	hri_cost_to_display = distance;

	return Cost;
}


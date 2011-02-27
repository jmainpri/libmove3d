//
// C++ Interface: HRI_tasks
//
// Description: 
//
//
// Author: Amit Kumar Pandey <akpandey@verdier.laas.fr>, (C) 2010
//
// Copyright: See COPYING file that comes with this distribution
//
//
//High Level action IDs
#ifndef _HRI_tasks_H
#define _HRI_tasks_H

////#include "Mightability_Maps.h"
////#define SECOND_HUMAN_EXISTS


typedef enum HRI_TASK_TYPE_ENUM
{
MAKE_OBJECT_ACCESSIBLE=0,
SHOW_OBJECT,
GIVE_OBJECT,
HIDE_OBJECT,
PUT_AWAY_OBJECT,
HIDE_AWAY_OBJECT,
MAKE_SPACE_FREE_OF_OBJECT_OBJ,
PUT_INTO_OBJECT,
//Add new tasks here before the last line

MAXI_NUM_OF_HRI_TASKS
}HRI_TASK_TYPE;

/*
typedef struct world_state_configs
{
 //int no_robots;
 std::vector <configPt> robot_config;// robot_config[50]; //To store configurations of all the robots, it should be synchronized with the index of the robots
   
 
}world_state_configs;

typedef struct atomic_HRI_task
{
 HRI_TASK_TYPE task_type;
 world_state_configs before_task;
 world_state_configs after_task;
 
}atomic_HRI_task;*/

#endif
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

////#include "Mightability_Analysis.h"
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
//NOTE: Don't forget to add any new task in init_HRI_task_name_ID_map() also
//Add new tasks here before the last line

MAXI_NUM_OF_HRI_TASKS
}HRI_TASK_TYPE;


typedef enum HRI_SUB_TASK_TYPE
{
REACH_TO_TAKE=0,
REACH_TO_GRASP,
GRASP,
LIFT_OBJECT,
CARRY_OBJECT,
PUT_DOWN_OBJECT,
//NOTE: Don't forget to add any new task in init_HRI_task_name_ID_pair() also
//Add new sub tasks here before the last line

MAXI_NUM_SUB_TASKS
}HRI_SUB_TASK_TYPE;




#endif
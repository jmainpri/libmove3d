#ifndef _HRI_PLANNER_H
#define _HRI_PLANNER_H

#include "hri_bitmap.h"
#include "hri_manip.h"

#define GIK_MAX_JOINT_NO 50
#define GIK_MAX_TASK_NO 5
#define GIK_CONSTRAINTS 3

typedef enum HRI_AGENT_TYPE_ENUM{
	HRI_JIDO1,
	HRI_HRP214,
	HRI_B21,
	HRI_JUSTIN,
	HRI_SUPERMAN,
	HRI_ACHILE,
	HRI_TINMAN,
  HRI_BH
} HRI_AGENT_TYPE;

typedef enum HRI_GIK_TASK_TYPE_ENUM{
	GIK_LOOK,
	GIK_RAREACH,
	GIK_LAREACH,
  GIK_RATREACH,
	GIK_LATREACH,
	GIK_RAPOINT,
	GIK_LAPOINT,
  GIK_RATPOINT,
	GIK_LATPOINT,
	GIK_NOTASK
} HRI_GIK_TASK_TYPE;

typedef struct STRUCT_GIK_TASK{
	HRI_GIK_TASK_TYPE type;
	int default_joints[GIK_MAX_JOINT_NO];
	int default_joints_no;
	int actual_joints[GIK_MAX_JOINT_NO];
	int actual_joints_no;
	int active_joint;
	double target[GIK_CONSTRAINTS];
} GIK_TASK;

typedef struct STRUCT_HRI_MANIP{
	hri_gik * gik;

	signed int gik_max_step; //TODO: add these two to hri_gik structure
	double reach_tolerance;

	GIK_TASK * tasklist;
	int tasklist_no;

	int activetasks[GIK_MAX_TASK_NO];
  int activetasks_no;
} HRI_MANIP;

typedef struct STRUCT_HRI_AGENT{
	HRI_AGENT_TYPE type;
	p3d_rob * robotPt;
	HRI_MANIP * manip;
	hri_bitmapset * btset;
	int btset_initialized;
	int exists;
	/* number of possible states for this human (e.g. handicaped humans have different states) */
  int states_no;
  int actual_state;
  /* possible states */
	/* TODO: change the name human_state -> agent state */
  hri_human_state * state;
} HRI_AGENT;


typedef struct STRUCT_HRI_AGENTS{
	HRI_AGENT ** robots;
	int robots_no;
	HRI_AGENT ** humans;
	int humans_no;
}HRI_AGENTS;

typedef struct struct_hri_shared_zone{
  double x;
  double y;
  double z;
  int value;
} hri_shared_zone;

#endif
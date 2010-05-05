#ifndef _HRIMANIP_H
#define _HRIMANIP_H

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#ifdef HRI_JIDO

#define ROBOT_ARM_JOINT_NO 6
#define ROBOT_HEAD_JOINT_NO 2

#define ROBOT_CONSTRAINT_NO 6 /* 3 if you reach x,y,z , 6 if you reach also dx,dy,dz */

/****** Dof numbers ******/
#define ROBOTq_X 6
#define ROBOTq_Y 7
#define ROBOTq_Z 8
#define ROBOTq_RX 9
#define ROBOTq_RY 10
#define ROBOTq_RZ 11
#define ROBOTq_PAN 12
#define ROBOTq_TILT 13

/****** Same for joint numbers ******/

#define ROBOTj_BASE 1
#define ROBOTj_GRIP 16
#define ROBOTj_LOOK 14 // 18 for hrp, 32 for BH
#define ROBOTj_PAN 2
#define ROBOTj_TILT 3
#define ROBOTj_OBJECT 13
#define ROBOTj_POINT 15
#define ROBOTj_RHAND 16
#define ROBOTj_LHAND 16
#define ROBOTj_RSHOULDER 4
#define ROBOTj_LSHOULDER 4
  
#elif defined(HRI_BHWO)

#define ROBOT_ARM_JOINT_NO 8 /* 6 for jido, 8 for hrp2 */
#define ROBOT_HEAD_JOINT_NO 4 /* 2 for jido, 4 for hrp2 */

#define ROBOT_CONSTRAINT_NO 6 /* 3 if you reach x,y,z , 6 if you reach also dx,dy,dz */

/****** Dof numbers ******/
#define ROBOTq_X 6
#define ROBOTq_Y 7
#define ROBOTq_Z 8
#define ROBOTq_RZ 11
#define ROBOTq_PAN 15

/****** Same for joint numbers ******/

#define ROBOTj_BASE 1
#define ROBOTj_RHAND 33
#define ROBOTj_LHAND 34
#define ROBOTj_LOOK 32
#define ROBOTj_PAN 5
#define ROBOTj_TILT 6
#define ROBOTj_OBJECT 33

#elif defined HRI_TUM_BH

#define ROBOT_ARM_JOINT_NO 8 /* 6 for jido, 8 for hrp2 */
#define ROBOT_HEAD_JOINT_NO 4 /* 2 for jido, 4 for hrp2 */

#define ROBOT_CONSTRAINT_NO 6 /* 3 if you reach x,y,z , 6 if you reach also dx,dy,dz */

/****** Dof numbers ******/
#define ROBOTq_X 6
#define ROBOTq_Y 7
#define ROBOTq_Z 8
#define ROBOTq_RX 9
#define ROBOTq_RY 10
#define ROBOTq_RZ 11
#define ROBOTq_PAN 15
#define ROBOTq_TILT 16

/****** Same for joint numbers ******/

#define ROBOTj_BASE 1
#define ROBOTj_RHAND 11
#define ROBOTj_LHAND 12
#define ROBOTj_LOOK 32
#define ROBOTj_PAN 5
#define ROBOTj_TILT 6
#define ROBOTj_OBJECT 33
#define ROBOTj_GRIP 11
#define ROBOTj_POINT 12

#elif defined HRI_HRP2

#define ROBOT_ARM_JOINT_NO 8
#define ROBOT_HEAD_JOINT_NO 4

#define ROBOT_CONSTRAINT_NO 3 /* 3 if you reach x,y,z , 6 if you reach also dx,dy,dz */

/****** Dof numbers ******/
#define ROBOTq_X 6
#define ROBOTq_Y 7
#define ROBOTq_Z 8
#define ROBOTq_RZ 11
#define ROBOTq_PAN 26
#define ROBOTq_TILT 27

/****** Same for joint numbers ******/

#define ROBOTj_BASE 1
#define ROBOTj_RHAND 27
#define ROBOTj_LHAND 40
#define ROBOTj_LOOK 18
#define ROBOTj_PAN 16
#define ROBOTj_TILT 17
#define ROBOTj_OBJECT 49
#define ROBOTj_GRIP 48
#define ROBOTj_POINT 48
#define ROBOTj_RSHOULDER 19
#define ROBOTj_LSHOULDER 32

#elif defined HRI_BERT1

#define ROBOT_ARM_JOINT_NO 8
#define ROBOT_HEAD_JOINT_NO 4

#define ROBOT_CONSTRAINT_NO 3 /* 3 if you reach x,y,z , 6 if you reach also dx,dy,dz */

/****** Dof numbers ******/
#define ROBOTq_X 6
#define ROBOTq_Y 7
#define ROBOTq_Z 8
#define ROBOTq_RZ 11
#define ROBOTq_PAN 14
#define ROBOTq_TILT 15

/****** Same for joint numbers ******/

#define ROBOTj_BASE 1
#define ROBOTj_RHAND 9
#define ROBOTj_LHAND 14
#define ROBOTj_LOOK 15
#define ROBOTj_PAN 4
#define ROBOTj_TILT 5
#define ROBOTj_OBJECT 14
#define ROBOTj_GRIP 14
#define ROBOTj_POINT 14


#else
#error "no robot has been defined in include/Hri-planner-pkg"
#endif


#define MOBJECTq_X 6
#define MOBJECTq_Y 7
#define MOBJECTq_Z 8
#define MOBJECTq_RX 9
#define MOBJECTq_RY 10
#define MOBJECTq_RZ 11


/* The coordinates change from one human model to the other */
/* Change these numbers when you change the human model */
/* The rest will follow these changes */
#define HUMANq_X 6
#define HUMANq_Y 7
#define HUMANq_Z 8
#define HUMANq_RZ 11
#define HUMANq_NECKZ 63

#ifdef HRI_HUMAN_ACHILE
#define HUMANq_PAN 17
#define HUMANq_TILT 15
#define HUMANq_TORSO_PAN 11
#define HUMANq_TORSO_TILT 10
#endif
#ifdef HRI_HUMAN_SUPERMAN
#define HUMANq_PAN 64 
#define HUMANq_TILT 65 
#define HUMANq_TORSO_PAN 11
#define HUMANq_TORSO_TILT 14
#endif


/****** Same for joint numbers ******/

#define HUMANj_BODY 1

#ifdef HRI_HUMAN_ACHILE
#define HUMANj_NECK_PAN 5 // for achile ->5, for superman ->54 
#define HUMANj_NECK_TILT 6 // for achile ->6, for superman ->55
#define HUMANj_RHAND 36 // for achile ->36, for superman ->29 /* or 30 or 31 */
#define HUMANj_LHAND 37 // for achile ->37, for superman ->26 /* or 27 or 28 */
#define HUMANj_RSHOULDER 8
#define HUMANj_LSHOULDER 15
#endif

//AKP: Joint indices superman
#ifdef HRI_HUMAN_SUPERMAN

#define HUMANj_NECK_PAN 54
#define HUMANj_NECK_TILT 55
#define HUMANj_RHAND 29 /* or 30 or 31 */
#define HUMANj_LHAND 26 /* or 27 or 28 */
#define HUMANj_RSHOULDER 11
#define HUMANj_LSHOULDER 14

#endif

#define CMB_HRI_SUM 1
#define HRI_EYE_TOLERANCE_TILT 0.3
#define HRI_EYE_TOLERANCE_PAN 0.3

#define GIK_FORCEBALL_NO 5
#ifndef DISTANCE2D
#define DISTANCE2D(x1,y1,x2,y2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))))
#endif
#ifndef DISTANCE3D
#define DISTANCE3D(x1,y1,z1,x2,y2,z2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))+((z2)-(z1))*((z2)-(z1))))
#endif

typedef struct GIK_joint_info_struct {
  p3d_jnt * joint;
  int no_dof;
  int index;
} hri_gik_joint_info;


typedef struct GIK_tasks_struct{
  int m;    /* target dof's for ex. for position of eef there are 3 dof's-> x.y.z  */
  int n;    /* dof's we control  */
  gsl_matrix * Jacobian;
  gsl_matrix * PsInvJacobianWS;
  gsl_matrix * PsInvJacobianWoS;
  gsl_matrix * PJ;
  gsl_vector * deltaTheta;
  gsl_vector * deltaX;
  gsl_vector * goal;
  hri_gik_joint_info ** jnt;
  int eef_no;
  int jnt_no;
  int initialized;
  int priority;

} hri_gik_task;



typedef struct GIK_struct{

  hri_gik_task ** task;
  int task_no;
  p3d_rob ** forceballs;
  int direct;
  p3d_rob * robot;
  p3d_jnt ** joints;
  int * free_joints;
  int joint_no;
  int GIKInitialized;
	int human_no;
  hri_human ** human;

} hri_gik;







#endif

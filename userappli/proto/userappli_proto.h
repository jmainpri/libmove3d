/* 
 *    File generated automatically. Do not edit by hand.
 */
#ifndef __CEXTRACT__
#include "Planner-pkg.h"

extern void openChainPlannerOptions(void);
extern void closedChainPlannerOptions(void);

extern void viewTraj(void);
#ifdef DPG
extern int checkForCollidingLpAlongPath(void);
#endif
extern void showConfig(configPt conf);
extern void computeOfflineOpenChain(p3d_rob* robot, p3d_matrix4 objectInitPos);
extern void computeOfflineClosedChain(p3d_rob* robot, p3d_matrix4 objectInitPos);

/** ////////// MISC /////////////*/
extern void globalPlanner(void);
extern void findPath(void);
extern void p3d_specificSuperGraphLearn(void);
extern void p3d_computeTests(void);

#ifdef DPG
extern int checkForColPath(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, configPt current, p3d_localpath* currentLp);
#endif
extern void p3dAddTrajToGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj);
extern p3d_node* p3d_addConfToGraph(p3d_rob* robot, p3d_graph* graph, configPt q, int* ikSol);
extern p3d_node* p3d_findInsertConnectTrajConfigInGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj, configPt q, p3d_localpath* currentLp);
extern p3d_localpath* p3d_findConfigLocalPathInTraj(p3d_rob* robot, p3d_traj* traj, configPt q);
/** ////////// MISC /////////////*/

extern void fixJoint(p3d_rob * robot, p3d_jnt * joint,  p3d_matrix4 initPos);
extern void unFixJoint(p3d_rob * robot, p3d_jnt * joint);

extern void nbLocalPathPerSecond(void);
extern void nbCollisionPerSecond(void);

#endif /* __CEXTRACT__ */

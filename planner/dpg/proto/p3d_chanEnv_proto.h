#ifndef __P3DCHANENV_H__
#define __P3DCHANENV_H__
//Enable DPG Flag in compilation to use

extern int checkCollisionsOnPathAndReplan(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, int optimized);
extern int replanForCollidingPath(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, configPt current, p3d_localpath* currentLp, int optimized);
extern int checkForCollidingPath(p3d_rob* robot, p3d_traj* traj, p3d_localpath* currentLp);

#endif
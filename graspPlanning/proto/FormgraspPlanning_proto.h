
#ifndef FORM_GRASP_PLANNING_PROTO_H
#define FORM_GRASP_PLANNING_PROTO_H

extern void g3d_create_grasp_planning_form ( void );
extern void g3d_show_grasp_planning_form ( void );
extern void g3d_hide_grasp_planning_form ( void );
extern void g3d_delete_grasp_planning_form ( void );

extern int GP_Init(char *objectName);
extern p3d_cntrt* GP_GetArmCntrt(p3d_rob *robotPt);
extern int GP_ComputeGraspList(char *objectName);
extern configPt GP_FindGraspConfig(bool &needs_to_move);
extern int GP_FindPath();
extern int GP_FindPathForArmOnly();
extern configPt* GP_GetTrajectory(p3d_rob *robotPt, p3d_traj *traj, int &nb_configs);
extern configPt* GP_GetAllTrajectoriesAsOne(p3d_rob *robotPt, int &nb_configs);
extern int GP_ConcateneAllTrajectories(p3d_rob *robotPt);
extern void GP_Reset();
extern void Gp_ResetGraph();

#endif


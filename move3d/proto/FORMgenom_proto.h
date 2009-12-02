
#if defined(GRASP_PLANNING) && defined(MULTILOCALPATH)
extern void g3d_create_genom_form ( void );
extern void g3d_show_genom_form ( void );
extern void g3d_hide_genom_form ( void );
extern void g3d_delete_genom_form ( void );
extern void genomCleanRoadmap(p3d_rob* robotPt);
extern int genomArmGotoQ(p3d_rob* robotPt, int cartesian, int lp[], Gb_q6 positions[], int *nbPositions);
extern int genomSetArmQ(p3d_rob *robot, double q1, double q2, double q3, double q4, double q5, double q6);
extern int genomGetArmQ(p3d_rob *robot, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
extern int genomSetArmX(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
extern int genomGetArmX(p3d_rob *robotPt, double *x, double *y, double *z, double *rx, double *ry, double *rz);
extern int genomSetInterfaceQuality();
extern int genomFindSimpleGraspConfiguration(p3d_rob *robotPt, char *object_name, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
extern int genomArmComputePRM(p3d_rob* robotPt, int cartesian);
extern int genomCheckCollisionOnTraj(p3d_rob* robotPt, int cartesian, double* armConfig, int currentLpId, int lp[], Gb_q6 positions[],  int *nbPositions);
extern int genomGetCollideStatus(int status);
extern int genomSetFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
extern int genomSetFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz)
;

extern int genomComputeGraspList(p3d_rob *hand_robotPt, char *object_name);
extern int genomFindGraspConfiguration(p3d_rob *robotPt, p3d_rob *hand_robotPt, char *object_name, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
extern int genomFindPregraspAndGraspConfiguration(p3d_rob *robotPt, p3d_rob *hand_robotPt, char *object_name, double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
extern int genomPrintConstraintInfo(p3d_rob *robotPt);

#endif

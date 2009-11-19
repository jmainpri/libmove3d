extern void g3d_create_genom_form ( void );
extern void g3d_show_genom_form ( void );
extern void g3d_hide_genom_form ( void );
extern void g3d_delete_genom_form ( void );
#ifdef GRASP_PLANNING
extern int genomArmGotoQ(p3d_rob* robotPt, int cartesian, int lp[], Gb_q6 positions[], int *nbPositions);
extern int genomSetArmQ(p3d_rob *robot, double q1, double q2, double q3, double q4, double q5, double q6);
extern int genomGetArmQ(p3d_rob *robot, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);

extern int genomFindSimpleGraspConfiguration(p3d_rob *robotPt, char *object_name, int nbTries, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);

#endif

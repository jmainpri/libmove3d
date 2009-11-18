
#ifdef MULTILOCALPATH
#ifndef P3D_MULTILOCALPATH_PROTO
#define  P3D_MULTILOCALPATH_PROTO

extern void p3d_destroy_multiLocalPath_data(p3d_rob* robotPt);
extern void p3d_multiLocalPath_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt);
extern configPt p3d_multiLocalPath_config_at_param(p3d_rob *robotPt, p3d_localpath *localpathPt, double param);
extern double p3d_multiLocalPath_length(p3d_rob* robotPt, p3d_localpath* localpathPt);
extern double p3d_multiLocalPath_stay_within_dist(p3d_rob* robotPt, p3d_localpath* localpathPt, double parameter, whichway dir, double *distances);
extern p3d_localpath *p3d_copy_multiLocalPath_localpath(p3d_rob* robotPt, p3d_localpath* localpathPt);
extern p3d_localpath *p3d_extract_multiLocalPath(p3d_rob *robotPt, p3d_localpath *localpathPt, double l1, double l2);
extern double p3d_multiLocalPath_cost(p3d_rob *robotPt, p3d_localpath *localpathPt);
extern p3d_localpath *p3d_simplify_multiLocalPath(p3d_rob *robotPt, p3d_localpath *localpathPt, int *need_colcheck);
extern p3d_localpath *p3d_multiLocalPath_localplanner(p3d_rob *robotPt, p3d_softMotion_data** softMotion_data, configPt qi, configPt qf, configPt qfp1, int* ikSol);
extern void lm_destroy_multiLocalPath_params(p3d_rob *robotPt, void *paramPt);
extern void p3d_write_multiLocalPath(FILE* filePtr, p3d_rob* robotPt, p3d_localpath* localpathPt);
extern int p3d_multiLocalPath_get_value_groupToPlan(p3d_rob* robotPt, const int mgID);
extern void p3d_multiLocalPath_set_groupToPlan(p3d_rob* robotPt, int mgID, int value) ;
extern configPt p3d_separateMultiLocalPathConfig(p3d_rob *r, configPt refConfig, configPt config, int mlpID, p3d_multiLocalPathJoint ** mlpJoints);
extern void p3d_multiLocalPath_set_groupToPlan_by_name(p3d_rob* robotPt, char* name, int flag) ;
extern void p3d_multiLocalPath_disable_all_groupToPlan(p3d_rob* robotPt);
extern int p3d_multiLocalPath_get_group_by_name(p3d_rob* robotPt, char* name);
#endif
#endif

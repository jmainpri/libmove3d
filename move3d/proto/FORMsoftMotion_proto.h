#include "../lightPlanner/proto/ManipulationStruct.h"

extern void g3d_create_soft_motion_form ( void );
extern void g3d_show_soft_motion_form ( void );
extern void g3d_hide_soft_motion_form ( void );
extern void g3d_delete_soft_motion_form ( void );

extern int p3d_optim_traj_softMotion(p3d_traj *trajPt, bool param_write_file, double *gain, int *ntest, std::vector <int> &lp, std::vector < std::vector <double> > &positions, MANPIPULATION_TRAJECTORY_STR &segments);
extern int p3d_convert_traj_to_softMotion(p3d_traj *trajPt, bool param_write_file, std::vector <int> &lp, std::vector < std::vector <double> > &positions, MANPIPULATION_TRAJECTORY_STR &segments);

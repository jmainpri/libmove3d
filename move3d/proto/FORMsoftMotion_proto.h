extern void g3d_create_soft_motion_form ( void );
extern void g3d_show_soft_motion_form ( void );
extern void g3d_hide_soft_motion_form ( void );
extern void g3d_delete_soft_motion_form ( void );

extern int p3d_optim_traj_softMotion(p3d_traj *trajPt, int param_write_file, double *gain, int *ntest);
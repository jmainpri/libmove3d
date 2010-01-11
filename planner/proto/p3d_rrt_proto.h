
#ifndef __CEXTRACT__

extern int p3d_get_rrt_maxn_fails(void);
extern p3d_node *hrm_nearest_neighbor(p3d_rob *rob, configPt q, p3d_compco *comp, double *distPt, int bio_ligand_mode);
extern p3d_node *hrm_extend_on_surface(p3d_rob *rob, p3d_graph *G, p3d_node *Nnear, configPt qnew);
extern p3d_node *p3d_get_current_rrt_Nnear ( void );
extern void p3d_set_current_rrt_Nnear(p3d_node *Nnear);
extern void p3d_set_save_intermediate_rrt_N(int val);
extern int p3d_get_save_intermediate_rrt_N ( void );
extern p3d_list_node* is_config_inside_tree_shape ( p3d_rob* robotPt, p3d_compco* comp, configPt q );
extern double hrm_dist_config(p3d_rob *robotPt, configPt q_i, configPt q_f);
extern int hrm_expand_one_rrt ( p3d_graph *G, p3d_compco **CompPt, configPt q, int sample_passive);
extern int p3d_expand_start_rrt ( p3d_graph *G, int (*fct_stop)(void) );
extern int p3d_expand_bidir_rrt ( p3d_graph *G, int (*fct_stop)(void) );
extern int p3d_resize_rrt_box(p3d_rob* robotPt, configPt box_env[], p3d_node* NewNode, int koef);
#endif /* __CEXTRACT__ */

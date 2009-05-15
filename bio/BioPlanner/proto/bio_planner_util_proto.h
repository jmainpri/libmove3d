


extern int bio_restrict_sc_shoot(p3d_rob* robotPt, p3d_node* Nnear, configPt q);
extern int bio_restrict_sc_shoot_with_NnearInfo(p3d_rob* p3d_robotPt, p3d_node* Nnear, configPt q);
extern int bio_side_chain_shoot(p3d_rob *robotPt, configPt q);
extern int bio_side_chain_inbox_shoot(p3d_rob *robotPt, configPt box_env[], configPt q);
extern int bio_ligand_shoot(p3d_rob* robotPt, configPt q);
extern int bio_ligand_inbox_shoot(p3d_rob* robotPt, configPt box_env[], configPt q);
extern double bio_compute_ligand_dist(p3d_rob *robotPt, configPt q_i, configPt q_f);
extern p3d_node *bio_ligand_nearest_neighbor(p3d_rob *robotPt, configPt q, p3d_compco *comp, double* dist);
extern int bio_get_max_nnode_fail(void);
extern void bio_set_max_nnode_fail(int nfail);

extern void bio_init_array_sch(int *array_sch, int *ordered_array_sch, int num_sch);
extern int bio_generate_one_sch_conf_and_checkcoll(p3d_rob *robotPt, int index_res,   
						   Joint_tablespt *jnt_table, configPt q); 
extern int bio_generate_one_bkb_conf_and_checkcoll(p3d_rob *robotPt, int index_res,   
						   Joint_tablespt *jnt_table, configPt q); 
extern int bio_generate_one_bkb_conf_ramachandran_and_checkcoll(p3d_rob *robotPt, int index_res,   
								Joint_tablespt *jnt_table, configPt q); 
extern int bio_perturb_one_sch_conf_and_checkcoll(p3d_rob *robotPt, int index_res,   
						  Joint_tablespt *jnt_table, configPt q, double sigma);
extern int bio_deform_schs_avoiding_collision(p3d_rob *robPt, configPt q, int print_messages);
extern int bio_perturb_and_check_passive_params_involved_in_collision(p3d_rob *robotPt, configPt q_inv);

extern int bio_get_list_of_passive_joints_involved_in_collision(p3d_rob *robotPt, configPt qinv, 
								int *npassJPt, p3d_jnt ***passJlistPt);

extern void bio_free_multimol_collision_report(int_tab** sc_autocol_list, int_tab** scsc_col_list, 
					int_tab** sc_bb_col_list, int_tab** sc_ligand_col_list);
extern int bio_multimol_collision_report(p3d_rob *robotPt, configPt q, int_tab** sc_autocol_list,
				  int_tab** scsc_col_list , int_tab** sc_bb_col_list, int_tab** sc_ligand_col_list,
					 int* is_ligand_bb_col, int* is_ligand_autocol, int* is_bb_autocol);
extern void bio_free_monomol_collision_report(int_tab** sc_autocol_list, int_tab** scsc_col_list, 
				       int_tab** sc_bb_col_list);
extern int bio_monomol_collision_report(p3d_rob *robotPt, configPt q, int_tab** sc_autocol_list,
					int_tab ** scsc_col_list , int_tab** sc_bb_col_list, int* is_bb_autocol) ;	    

extern int bio_set_all_sch_dofs_as_passive_parameters_for_planner(p3d_rob *robotPt);
extern int bio_set_all_sch_dofs_as_active_parameters_for_planner(p3d_rob *robotPt);

extern int bio_set_triade(int resSeq1, int resSeq2, int resSeq3, int conftype);
extern int bio_get_triade(int triade_num, p3d_jnt **triade_jntPt, int *triade_conftype);
extern double bio_measure_triade_surface_area(p3d_jnt **triade_jntPt, configPt q);

extern int bio_set_pairs_for_dist(int nump, int *pairslist, int conftype);
extern int bio_get_pairs_for_dist(int *nump, p3d_jnt ***pairs_jntPt, int *conftype);
extern double bio_measure_distance_between_atom_pairs(int nump, p3d_jnt **pairs_jntPt, configPt q);

extern void bio_search_max_weight_in_curr_rrt(void);

extern int bio_set_goal_jnt_coordinates(void);
extern int bio_get_goal_jnt_coordinates(p3d_vector3 **goal_jntcoordsPt);
extern void bio_free_goal_jnt_coords(int n_alloc_elems);
//extern int bio_get_goal_jnt_coordinates(double **goal_jntvPt);
extern double bio_rmsd_to_goal_jnt_coords(p3d_rob *robPt, p3d_vector3 *goal_jnt_coords, configPt q);
//extern double bio_rmsd_to_goal_jnt_coords(p3d_rob *robPt, double *goal_jntvPt, configPt q);

extern int bio_set_init_jnt_coordinates(void);
extern int bio_get_init_jnt_coordinates(p3d_vector3 **init_jntcoordsPt);
extern void bio_free_init_jnt_coords(int n_alloc_elems);
extern double bio_rmsd_to_init_jnt_coords(p3d_rob *robPt, p3d_vector3 *init_jnt_coords, configPt q);

extern double bio_rmsd_between_confs(p3d_rob *robPt, configPt q1, configPt q2);

extern int bio_compute_molecule_center_of_mass(p3d_rob* robPt, p3d_vector3 CoM);
extern double bio_compute_molecule_radius_of_giration(p3d_rob* robPt);

extern p3d_node* bio_shoot_free_conf(p3d_graph *graphPt);

/**
 * p3d_ComputeVdWMaxConf
 * Compute the maximal value of the Van 
 * der waals Radius for which the configuration is
 * valid
 * @param[In]: the given configuration
 * @return: the maximal VdW value valid for the conf 
 * (between 0 and 1)
 */
extern double p3d_ComputeVdWMaxConf(configPt q);

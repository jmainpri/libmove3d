extern int bio_expand_side_chain(p3d_graph*G, p3d_node* last_node, int_tab* sc_lignand_col_list);
extern p3d_node*bio_expandnode_ligand(p3d_graph* G,p3d_node* Nnear, configPt qexp_direction, 
				int_tab** sc_ligand_col_list, int* is_ligand_autocolPt, 
				      int* is_ligand_bb_colPt);
extern int bio_expand_side_chain(p3d_graph*G, p3d_node* last_node, int_tab* sc_lignand_col_list); 
extern int bio_expand_ligand(p3d_graph*G, configPt q_lig, int_tab** sc_ligand_col_list, int*  is_ligand_autocolPt, 
			     int* is_ligand_bb_colPt);
extern int bio_start_lig_sc_rrt(p3d_graph *G,int (*fct_stop)(void));

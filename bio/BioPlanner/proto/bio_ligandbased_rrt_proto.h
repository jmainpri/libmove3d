extern void check_if_node_connects_connected_components(p3d_graph *G, p3d_node *NewNode );
extern int bio_start_ligandbased_rrt(p3d_graph *G,int (*fct_stop)(void));
extern int bio_expandcomp_ligandbased_rrt(p3d_graph *G, p3d_compco *CompPt, configPt q);
extern p3d_node* bio_expandnode_ligandbased_rrt(p3d_graph* G,p3d_node* Nnear,
						configPt qexp_direction) ;

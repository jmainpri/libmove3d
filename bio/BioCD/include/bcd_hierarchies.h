/****************************************************************************/
/*!			bcd_hierarchies.h
 *
 *  
 *
 ****************************************************************************/
 
#if !defined (BCD_HIERARCHIES)
#define BCD_HIERARCHIES

/* user-level functions : BEGIN +++++++++++++++++++ */
void bio_set_surface_d(double distance);
double min_dist_report(p3d_poly **poly1, p3d_poly **poly2);
void biocol_report(int *col_number, p3d_poly ***list1, p3d_poly ***list2);
void bio_set_col_mode(int mode);
void set_required_collisions(int rc);
void set_required_collisions_to_max(void);
/*user-level functions : END+++++++++++++++++++++++++++++++++++++++++++++*/


int bio_get_col_mode(void);
void set_minimum_Sdistance_to_max(void);
void set_n_collisions_to_zero(void);
int get_n_collisions(void);
int too_much_collisions(void);
boxnodept create_rigid_root(Rigid_structure *rigido);
void bio_create_molecule_root(Robot_structure *robot);
void sup_collision(supnodept node1, supnodept node2);
void box_coll_withinProt(Bboxpt b1, Bboxpt b2);
void box_coll_general(Bboxpt b1, Bboxpt b2);
void freeboxhierarchy(Robot_structure *robot, int i);
void freeboxhierarchy2(Rigid_structure *r);
void freesuphierarchy(Robot_structure *robot);
void my_robot_autocollision(Robot_structure *robot);


#endif

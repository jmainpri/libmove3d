
extern double bio_get_atom_mass_from_type(int a_type);
extern int get_AAtotal_number(p3d_rob* robotPt );
extern p3d_jnt** get_list_firstjnts_flexible_sc(p3d_rob* robotPt );
extern int get_nb_flexible_sc(p3d_rob* robotPt);
extern p3d_jnt *get_AAfirstjnt( int AAnumber );
extern int get_AAnumber_from_jnt (p3d_jnt* jnt);
extern int  get_AAnumber_from_name( char *name );
extern void  get_AAtype_from_name( char *name, char AAtype[4] );
extern int get_AAtype_from_AAnumber( int AAnumber, char AAtype[4] );
extern int get_AAfirstjnt_number ( int AAnumber );
extern int next_joint( p3d_jnt **Outjnt, p3d_jnt **Injnt, int in_branch);
extern int nb_dof_ligand (p3d_rob* robotPt);
extern int num_subrobot_ligand (p3d_rob* robotPt);
extern int num_subrobot_AA_from_AAnumber(p3d_rob* robotPt,int AAnumber);
extern int is_ligand_in_robot(p3d_rob* robotPt);
extern int bio_copy_sch_conf(p3d_rob *robPt, configPt q_src, configPt q_dst);
extern int bio_copy_ligand_conf(p3d_rob *robPt, configPt q_src, configPt q_dst);
extern int make_rigid_AA_jnt(p3d_jnt* firstAAjnt); 
extern int make_rigid_AA_num(int num); 
extern int make_flexible_AA_jnt(p3d_jnt* firstAAjnt);
extern int make_flexible_AA_num(int num);
extern int bio_get_position_meaning_frame(p3d_rob* robotPt, p3d_vector3 pos);
extern int update_ligand_bounding_box(p3d_rob* robotPt);
extern int update_sidechain_BBox_from_firstjnt(p3d_rob* robotPt, p3d_jnt* firstAAjnt);
/*extern int update_sidechain_BBox_from_AAnumber(p3d_rob* robotPt, int  AAnumber);*/
extern int update_all_sidechain_BBoxes(p3d_rob* robotPt);
extern int dist_ligand_sidechainBBoxes(p3d_rob* robotPt, int  AAnumber, double * dist_vector);
extern void PrintInfo_BBoxesdist(p3d_rob* robotPt, int  AAnumber);
extern int bio_get_pdb_atom_number(p3d_poly* poly); // Added by Ron
extern void afficher_lescollisions ( void );

/*extern void test_perf ( void );*/
/*extern void changer_rigidite ( int numero, int nature ); */
/*extern int is_AA_rigid ( int numero, int nature ); */

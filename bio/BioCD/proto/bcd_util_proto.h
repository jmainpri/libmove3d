/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Wed Jun 23 14:30:19 2004
 */
#ifndef __CEXTRACT__

extern Robot_structure **get_bcd_robots(void);
extern void bio_set_autocol ( int nrobot, int nforbidden, int *nrigid1, int *natom1, int *nrigid2, int *natom2 );
extern void rebuild ( Robot_structure *robot );
extern void preprogr_collisions ( Robot_structure *robot );
extern void free_robot ( Robot_structure *robot );
extern void bio_init_molecules ( double scale );
extern void bio_create_molecule ( int i, double scale );
extern void bio_create_molecules ( double scale );
extern void p3d_start_bio_col ( void );
extern int bio_all_molecules_col ( void );
extern int bio_molecule_col ( int nrobot );
extern int bio_all_molecules_col_with_report ( void );
extern void set_affichage_en_rouge ( int affich );
extern int get_affichage_en_rouge ( void );
extern int biocol_robot_report ( int nrobot );
extern int bio_molecule_autocol ( int i );
extern int bio_two_molecule_col ( int i, int j );
extern int supress_sc_rigid_sequence ( int nrobot, int first, int last );
extern int activate_sc_rigid_sequence ( int nrobot, int first, int last );
extern int activate_sc_rigid ( int nrobot, int namino );
extern int supress_sc_rigid ( int nrobot, int namino );
extern int supress_bb_rigid_sequence ( int nrobot, int first, int last );
extern int activate_bb_rigid_sequence ( int nrobot, int first, int last );
extern int activate_bb_rigid ( int nrobot, int namino );
extern int supress_bb_rigid ( int nrobot, int namino );
extern Joint_tablespt *give_joint_tables ( int nrobot );
extern int bio_sc_col ( int nrobot, int namino );
extern int bio_bb_col ( int nrobot, int namino );
extern int get_number_of_bcd_robots( void );
extern int bio_molecule_col_no_autocol( int nrobot );
extern double bioGetInvMinLigProtDist(void);
double bioGetLigRotaMotionFromInit(p3d_rob* robotPt,  
				   configPt currentConf);
#endif /* __CEXTRACT__ */

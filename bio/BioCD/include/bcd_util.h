/****************************************************************************/
/*!		bcd_util.h
 *
 *		
 *
 ****************************************************************************/
 
#if !defined (BCD_UTIL)
#define BCD_UTIL


void bio_set_autocol(int nrobot, int nforbidden, int  *nrigid1, 
    int *natom1, int  *nrigid2, int *natom2);


void bio_init_molecules(double scale);
void bio_create_molecule(int i, double scale);
void bio_create_molecules(double scale);
int bio_all_molecules_col(void);
int bio_molecule_col(int nrobot);
int bio_all_molecules_col_with_report(void);
int biocol_robot_report(int nrobot);
int bio_molecule_autocol(int i);
int bio_two_molecule_col(int i, int j);
int supress_sc_rigid_sequence(int nrobot,int first,int last);
int activate_sc_rigid_sequence(int nrobot,int first,int last);
int activate_sc_rigid(int nrobot,int namino);
int supress_sc_rigid(int nrobot,int namino);
int supress_bb_rigid_sequence(int nrobot,int first,int last);
int activate_bb_rigid_sequence(int nrobot,int first,int last);
int activate_bb_rigid(int nrobot,int namino);
int supress_bb_rigid(int nrobot,int namino);
Joint_tablespt *give_joint_tables(int nrobot);
int bio_sc_col(int nrobot, int namino);
int bio_bb_col(int nrobot, int namino);

void rebuild(Robot_structure *robot);
void preprogr_collisions(Robot_structure *robot);

//void set_charlatan(int i);



#endif
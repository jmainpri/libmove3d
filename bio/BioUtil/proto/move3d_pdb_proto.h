/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

typedef enum {
 trajExpMinNONE = 0, trajExpMinAMBERSaveFromM3D = 1, trajExpMinAMBERSaveFromAmber = 2, trajExpMinRRT = 3
} trajExportMinimizationType;


//extern void readamino ( char *s );
//extern void intable ( p3d_poly *polygordo );
extern void common_infile ( FILE *fp );
extern void write_atom ( FILE *fp, int i );
extern void ILE_infile ( FILE *fp );
extern void infile ( FILE *fp );
//extern void move3d_to_pdb ( pp3d_rob protein, char *filename );
extern int traj_to_pdb ( p3d_rob *robotPt, double dmax, char *radix );
extern int graph_to_pdb ( void );
extern int traj_counter ( p3d_rob *robotPt, double dmax );
extern void serchDmaxAndWrite ( pp3d_rob protein, char *radix, int desired );
extern void translate_conf_to_pdb(pp3d_rob protein, FILE *fp);
extern int bio_write_pdbs_unifom_dep(pp3d_rob robotPt, char *radix, double dep_dist, trajExportMinimizationType minimizationType);
extern int bio_write_pdbs_of_N_in_path(pp3d_rob robotPt, char *radix, trajExportMinimizationType minimizationType);
extern void bio_write_baryplot(void);

#endif /* __CEXTRACT__ */

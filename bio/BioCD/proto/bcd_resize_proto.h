/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Wed Jun 23 14:30:18 2004
 */
#ifndef __CEXTRACT__

extern double GetPrevVdw(void);
extern void SetPrevVdw(double prevVdwRadius);
extern void set_SUM_VDW_to_EBI ( void );
extern void set_SUM_VDW_to_usual ( void );
extern void set_atom_type ( p3d_poly *atom, enum atom_type atype );
extern void write_wdw_radii ( void );
extern void write_SUM_VDW ( void );
extern void reduce_don_acep_mat ( double scale );
extern void bio_resize_molecule ( int nrobot, double shrink );
extern void bio_resize_molecules ( double shrink );
extern void bio_resize_rigid ( int nrobot, int nrigid, double shrink );
extern void bio_true_resize_molecule ( int nrobot );
extern void bio_true_resize_molecules ( void );

#endif /* __CEXTRACT__ */

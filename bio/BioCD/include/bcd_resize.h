/****************************************************************************/
/*!			bcd_resize.h
 *
 *		f 
 *
 ****************************************************************************/
 
#if !defined (BCD_RESIZE)
#define BCD_RESIZE

#include "P3d-pkg.h"

void set_atom_type(p3d_poly *atom, enum atom_type atype);
void bio_resize_molecules(double shrink);
void bio_resize_molecule(int nrobot, double  shrink);
void bio_true_resize_molecule(int nrobot);
void bio_true_resize_molecules();
void set_SUM_VDW_to_EBI();
void set_SUM_VDW_to_usual();

#ifndef HYDROGEN_BOND
void bio_resize_rigid(int nrobot,int nrigid, double  shrink);
void reduce_don_acep_mat(double scale);
#endif



/*developer */
void write_wdw_radii(void);
void write_SUM_VDW(void);
#endif
/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
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
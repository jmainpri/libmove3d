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
#ifndef PDBFORMAT_H
#define PDBFORMAT_H

#include "atoms.h"
#include "protein.h"

typedef enum {
  INSIGHT, AMBER
} formatTypes;

extern void translate_pdb_res_name(char* pdb_res_name, char* psf_res_name, 
				   formatTypes pdb_format);
extern void translate_pdb_atom_name(char* pdb_atom_name, char* psf_atom_name,
				    residueTypes resType, formatTypes pdb_format);
extern int scanFile(FILE* pdbfile, formatTypes* format);
extern int update_amber_serial(residue* resPt, int firstSer, int* lastSer);
extern void write_residue_with_amber_order(FILE* pdboutfile, residue* resPt);
extern int get_amber_serial(atom* aPt, int* serial);

#endif

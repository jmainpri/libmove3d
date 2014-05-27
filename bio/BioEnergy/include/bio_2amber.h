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
#ifndef BIO_2AMBER_H
#define BIO_2AMBER_H


/* amber */
#include <basics.h>
#include <stringExtra.h>
#include <vector.h>
#include <matrix.h>
#include <classes.h>
#include <dictionary.h>
#include <database.h>
#include <library.h>
#include <parmLib.h>
#include <pdbFile.h>
#include <help.h>
#include <parser.h>
#include <tools.h>
#include <amber.h>
#include <commands.h>
#include <defaults.h>
#include <leap.h>
#include <octree.h>
#include <tripos.h>
#include <block.h>
#include <leap.h>
#include <getline.h> 




void AMBER_addPath( STRING dir_name );
OBJEKT AMBER_add_source(char *filename);
OBJEKT AMBER_loadPdb(  FILE *fPdb);
OBJEKT AMBER_loadPrep(char*file_prep);
OBJEKT AMBER_loadMol2( char *mol2FileName );
OBJEKT AMBER_loadParams(char *frcmod);
void AMBER_saveParm(UNIT    uUnit,char *name_first );
void AMBER_savePdbInGivenFile( OBJEKT objProt, FILE *fOut);
void AMBER_savePdb( OBJEKT objProt, char *name_first);
OBJEKT AMBER_Combine(OBJEKT  objProt, OBJEKT objLig);

#endif /* #ifndef BIO_2AMBER_H */

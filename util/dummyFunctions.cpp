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
#include "P3d-pkg.h"
#include "Bio-pkg.h"


#include <iostream>

using namespace std;

//-----------------------------------------------------------------
//-----------------------------------------------------------------
const string BioError("Error: Bio module is not compiled");

// BIO
void insert_pointer_in_list ( void *thePt, void ***listPt, int *nelems )
{
    cout << BioError << endl;
}

int biocol_robot_report(int nrobot)

{
    cout << BioError << endl;
    return 0;
}

int bio_get_PRINT_LOOP_GEOM ( void )
{
    cout << BioError << endl;
    return 0;
}

int bio_all_molecules_col(void)
{
    cout << BioError << endl;
    return 0;
}

int bio_all_molecules_col_with_report ( void )
{
    cout << BioError << endl;
    return 0;
}

int bio_set_ik ( p3d_cntrt *ct )
{
    cout << BioError << endl;
    return 0;
}

void bio_print_loop_geometry ( p3d_cntrt *ct )
{
    cout << BioError << endl;
}

int bio_generate_ct_conf( p3d_rob *robotPt, p3d_cntrt *ct, configPt q )
{
    cout << BioError << endl;
    return 0;
}

int bio_compute_ik_nopep_new ( p3d_cntrt *ct, double **sol_configs )
{
    cout << BioError << endl;
    return 0;
}

int bio_set_ik_nopep ( p3d_cntrt *ct )
{
    cout << BioError << endl;
    return 0;
}

void p3d_start_bio_col ( void )
{
    cout << BioError << endl;
}

int bio_compute_ik ( p3d_cntrt *ct, double **sol_configs )
{
    cout << BioError << endl;
	return 0;
}

int bio_compute_ik_nopep ( p3d_cntrt *ct, double **sol_configs )
{
    cout << BioError << endl;
	return 0;
}

#ifdef WITH_XFORMS
#include <forms.h>
FL_FORM  *BIO_COLLISION_FORM = NULL;
#endif

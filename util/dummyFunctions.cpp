#include "P3d-pkg.h"
#include "Move3d-pkg.h"
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
FL_FORM  *BIO_COLLISION_FORM = NULL;
#endif

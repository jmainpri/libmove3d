#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Bio-pkg.h"

// XFORMS
// int fct_stop(void);
// void fct_draw(void);
// int g3d_get_KCD_CHOICE_IS_ACTIVE();
// int p3d_get_user_drawnjnt(void);
// p3d_traj *p3d_graph_to_traj ( p3d_rob *robotPt );
// void g3d_add_traj ( char *name, int i );

// Undefined XFORMS
/*
g3d_draw_frame
 _G3D_MODIF_VIEW
 g3d_add_traj
qt_get_cur_g3d_win
g3d_get_KCD_CHOICE_IS_ACTIVE
p3d_get_user_drawnjnt
qt_canvas_viewing
_G3D_ACTIVE_CC
g3d_set_picking
g3d_get_cur_win
g3d_draw_allwin_active
qt_calc_cam_param
fct_draw
fct_stop
_G3D_WIN
g3d_build_shadow_matrices
g3d_restore_win_camera
g3d_set_light
_G3D_SAVE_MULT
*/

// BIO
void insert_pointer_in_list ( void *thePt, void ***listPt, int *nelems )
{
    printf("ERROR: NO BIO Module\n");
}

int biocol_robot_report(int nrobot)

{
    printf("ERROR: NO BIO Module\n");
    return 0;
}

int bio_get_PRINT_LOOP_GEOM ( void )
{
    printf("ERROR: NO BIO Module\n");
    return 0;
}

int bio_all_molecules_col(void)
{
    printf("ERROR: NO BIO Module\n");
    return 0;
}

int bio_all_molecules_col_with_report ( void )
{
    printf("ERROR: NO BIO Module\n");
    return 0;
}

int bio_set_ik ( p3d_cntrt *ct )
{
    printf("ERROR: NO BIO Module\n");
    return 0;
}

void bio_print_loop_geometry ( p3d_cntrt *ct )
{
    printf("ERROR: NO BIO Module\n");
}

int bio_generate_ct_conf( p3d_rob *robotPt, p3d_cntrt *ct, configPt q )
{
    printf("ERROR: NO BIO Module\n");
     return 0;
}

int bio_compute_ik_nopep_new ( p3d_cntrt *ct, double **sol_configs )
{
    printf("ERROR: NO BIO Module\n");
     return 0;
}

int bio_set_ik_nopep ( p3d_cntrt *ct )
{
    printf("ERROR: NO BIO Module\n");
     return 0;
}

void p3d_start_bio_col ( void )
{
    printf("ERROR: NO BIO Module\n");
}

int bio_compute_ik ( p3d_cntrt *ct, double **sol_configs )
{
    printf("ERROR: NO BIO Module\n");
}

int bio_compute_ik_nopep ( p3d_cntrt *ct, double **sol_configs )
{
    printf("ERROR: NO BIO Module\n");
}

#ifdef WITH_XFORMS
FL_FORM  *BIO_COLLISION_FORM = NULL;
#endif













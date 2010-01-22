/* 
 *  This file includes the prototypes of the functions defined in the
 *  directory bio and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef BIO_PKG_H
#define BIO_PKG_H

/* struct */
#ifdef BIO
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "bio.h"
#include "cntrt.h"
#include "p3d.h"

#include "device.h"

#include "../bio/BioStructures/include/BioSTR_pkg.h"

/* protos */
#include "../bio/BioCD/proto/bcd_proto.h"
#include "../bio/BioStructures/proto/bst_proto.h"
#include "../bio/BioUtil/proto/butl_proto.h"
#include "../bio/BioLoop/proto/bloop_proto.h"
#include "../bio/BioPlanner/proto/bplan_proto.h"
#endif

#ifndef BIO
#include "bio.h"
#include "cntrt.h"

extern void insert_pointer_in_list ( void *thePt, void ***listPt, int *nelems );
extern int biocol_robot_report(int nrobot);
extern int bio_get_PRINT_LOOP_GEOM ( void );
extern int bio_all_molecules_col(void);
extern int bio_all_molecules_col_with_report ( void );
extern int bio_set_ik ( p3d_cntrt *ct );
extern void bio_print_loop_geometry ( p3d_cntrt *ct );
extern int bio_generate_ct_conf( p3d_rob *robotPt, p3d_cntrt *ct, configPt q );
extern int bio_compute_ik_nopep_new ( p3d_cntrt *ct, double **sol_configs ) ;
extern int bio_set_ik_nopep ( p3d_cntrt *ct ) ;
extern int bio_compute_ik ( p3d_cntrt *ct, double **sol_configs ) ;
extern int bio_compute_ik_nopep ( p3d_cntrt *ct, double **sol_configs );


//extern void p3d_set_matrix4_values ( p3d_matrix4 M, double m11, double m12, double m13,
//                              double m14, double m21, double m22, double m23, double m24,
//                              double m31, double m32, double m33, double m34, double m41, double m42, double m43, double m44 );
//
//extern  void p3d_rotate_vect_around_axis ( p3d_vector3 axis, double theta, p3d_vector3 v_in, p3d_vector3 v_out );
//
//extern void bio_set_PRINT_LOOP_GEOM ( int val );

#endif

#endif /* #ifndef BIO_PKG_H */

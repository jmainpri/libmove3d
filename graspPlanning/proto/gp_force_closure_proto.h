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
#ifndef GP_FORCE_CLOSURE_PROTO_H
#define GP_FORCE_CLOSURE_PROTO_H

extern int gpFriction_cone_contact_plane_intersection( p3d_vector3 vertex, p3d_vector3 normal, double mu, p3d_plane contactPlane, p3d_vector3 new_normal, double *new_mu );

extern double gpForce_closure_2D_grasp( double (*position)[2], double (*normal)[2], double mu[], unsigned int nbContacts );

extern double gpForce_closure_2D_grasp2(double (*position)[2], double (*normal)[2], double mu[], unsigned int nbContacts);


extern double gpForce_closure_3D_grasp( double (*position)[3], double (*normal)[3], double mu[], unsigned int nbContacts, unsigned int nbSlices );

extern int gpForce_closure_3D_grasp2(double (*position)[3], double (*normal)[3], double mu[], unsigned int nbContacts, unsigned int nbSlices);

#endif

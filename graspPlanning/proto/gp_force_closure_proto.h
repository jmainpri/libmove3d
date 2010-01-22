#ifndef GP_FORCE_CLOSURE_PROTO_H
#define GP_FORCE_CLOSURE_PROTO_H

extern int gpFriction_cone_contact_plane_intersection( p3d_vector3 vertex, p3d_vector3 normal, double mu, p3d_plane contactPlane, p3d_vector3 new_normal, double *new_mu );

extern double gpForce_closure_2D_grasp( double (*position)[2], double (*normal)[2], double mu[], unsigned int nbContacts );

extern double gpForce_closure_2D_grasp2(double (*position)[2], double (*normal)[2], double mu[], unsigned int nbContacts);


extern double gpForce_closure_3D_grasp( double (*position)[3], double (*normal)[3], double mu[], unsigned int nbContacts, unsigned int nbSlices );

extern int gpForce_closure_3D_grasp2(double (*position)[3], double (*normal)[3], double mu[], unsigned int nbContacts, unsigned int nbSlices);

#endif


#ifndef P3D_PQP_H
#define P3D_PQP_H


typedef unsigned int pqp_triangle[3];


//! This structure is used to know which object pairs must pass
//! the collision test.
//! Each p3d_obj variable has a pqpID field (see p3d.h).
//! This field is an index in the obj_obj array.
//! For instance, obj_obj[body1->pqpID][body2->pqpID]= 1
//! means that the collision between body1 and body2 will be tested
//! while obj_obj[body1->pqpID][body2->pqpID]= 0 means that it will not be tested.
//! There is one static pqp_collision_grid declared in p3d_pqp.c.
//! It is initialized with the pqp_create_collision_pairs() function.
//! If objects are deleted (robot bodies or environment obstacles),
//! it must be cleaned and reinitialized.
//! NB: obj_obj is symetric and should be kept symetric.
//! colliding_body1 and colliding_body2 are the two bodies 
//! that were reported as colliding in the last collision test.
typedef struct pqp_collision_grid
{
  unsigned int nb_robots; /*!< number of robots in the XYZ_ENV struct */
  unsigned int nb_obstacles; /*!< number of obstacles in the XYZ_ENV struct */
  unsigned int nb_objs;  /*!< total number of bodies (environment obstacles plus robot bodies) */
  unsigned int **obj_obj; /*!< see pqp_collision_grid struct description */
  p3d_obj **obj_from_pqpID; /*!< see pqp_collision_grid struct description */
  p3d_obj *colliding_body1, *colliding_body2;  /*!< pointer to the last two bodies that were found to be colliding */
} pqp_collision_grid;




#endif


#ifndef _POLYHEDRE_H
#define _POLYHEDRE_H

/******************************************************************************************
   
  FICHIER: polyhedre.h

  USAGE: module gerant des objets a facettes, creation par sommet 
         puis definition des faces
         le calcul des plans des faces est effectuee et reste memorisee uniquement 
         a la demande. De meme pour les arretes

******************************************************************************************/


/*#include "p3d_matrix.h" */
#ifdef _p3d_MATRIX_H

#define  poly_type_de_base p3d_matrix_type
#define poly_vector3 p3d_vector3
#define poly_matrix4 p3d_matrix4

#else
typedef double  poly_type_de_base;

typedef poly_type_de_base  poly_vector3[3];

typedef poly_type_de_base poly_matrix4[4][4];
#endif


#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef UNKNOWN
#define UNKNOWN 2
#endif


#define poly_error_null_pointer             1
#define poly_error_malloc_failled           2
#define poly_error_unknown_point            3
#define poly_error_impossible_edge          4
#define poly_error_impossible_index         5
#define poly_error_edge_needs_2_points      6
#define poly_error_normal_nulle             7
#define poly_error_plane_needs_3_points     8
#define poly_error_impossible_list_edges    9
#define poly_error_impossible_polyhedre     10


/*************************************************
 cet valeur est nulle quand il n y a pas d erreur
  et les valeurs M3d_error????? sinon
*************************************************/
extern unsigned int poly_error_value;


typedef unsigned int poly_index;
typedef poly_index *poly_index_p ;

typedef struct  poly_plane
  { poly_vector3      normale;
    poly_type_de_base  d;
  } poly_plane;

typedef struct poly_edge
  { poly_index     point1,point2;
    poly_index     face1,face2;
    poly_vector3  u;
  } poly_edge;

typedef struct poly_face
  { poly_plane     *plane;
    int  nb_points;
    poly_index     *the_indexs_points;
    unsigned int face_is_convex;

#ifdef GRASP_PLANNING
    //! the ID of the part the face belongs to (used by the polyhedron segmentation algorithm):
    unsigned int part;

    //! array of the indices (starting from zero) of the neighbours faces (all faces must be triangles;  
    //! consequently a face has at most 3 adjacent faces) in the p3d_polyhedre's face array:
    int neighbours[3];
#endif

  } poly_face;  

typedef struct poly_polyhedre
  { char *name;
    int  nb_points;
    int  nb_faces;
    int  nb_edges;
    poly_vector3  *the_points;
    poly_face      *the_faces;
    poly_edge      *the_edges;
    poly_matrix4   pos;

    #ifdef GRASP_PLANNING
     //! polyhedron's center of mass:
     p3d_vector3 cmass;

     //! polyhedron's main inertia axes:
     p3d_matrix3 inertia_axes;
 
     //! dimensions of the bounding box (aligned on its main inertia axes) of the polyhedron:
     double iaabb[6];

     //! surface normals on each vertex:
     p3d_vector3  *normals;
    #endif

  } poly_polyhedre;

/*
void poly_f_2_v3(float x, float y, float z, poly_vector3 *vector);
void poly_d_2_v3(double x, double y, double z, poly_vector3 *vector);
void poly_v3_2_f( poly_vector3 *vector, float *x, float *y, float *z);
void poly_v3_2_d(poly_vector3 *vector, double *x, double *y, double *z);
void poly_plane_2_d(poly_plane *plane, double *a, double *b, double *c, double *d);
void poly_show_error_on_shell(int value);
int poly_get_error_value();
int poly_error();
void poly_init_poly(poly_polyhedre *polyhedre,char *name);
poly_polyhedre *poly_create_poly(char *name);
void poly_destroy_poly(poly_polyhedre *polyhedre);
void poly_destroy_edges(poly_polyhedre *polyhedre);
void poly_destroy_planes(poly_polyhedre *polyhedre);
char *poly_get_name(poly_polyhedre *polyhedre);
unsigned int poly_get_nb_points(poly_polyhedre *polyhedre);
unsigned int poly_get_nb_points_in_face(poly_polyhedre *polyhedre, poly_index face);
unsigned int poly_get_nb_faces(poly_polyhedre *polyhedre);
unsigned int poly_get_nb_edges_in_face(poly_polyhedre *polyhedre, poly_index face);
unsigned int poly_get_nb_edges(poly_polyhedre *polyhedre);
int poly_get_index_point_in_face(poly_polyhedre *polyhedre,poly_index face, poly_index index);
int poly_get_edge_points(poly_polyhedre *polyhedre, poly_index edge, poly_index *p1, poly_index *p2);
int poly_get_edge_faces(poly_polyhedre *polyhedre, poly_index edge, poly_index *f1, poly_index *f2);
int poly_add_point(poly_vector3 point, poly_polyhedre *polyhedre);
int poly_find_point(poly_vector3 point, poly_polyhedre *polyhedre);
int poly_get_point_2_v3(poly_polyhedre *polyhedre, poly_index index, poly_vector3 *vector);
int poly_get_point_2_d(poly_polyhedre *polyhedre, poly_index index, double *x,double *y,double *z);
void poly_get_poly_pos(poly_polyhedre *polyhedre, poly_matrix4 mat);
poly_matrix4 *poly_get_poly_mat(poly_polyhedre *polyhedre);
void poly_set_poly_pos(poly_polyhedre *polyhedre, poly_matrix4 mat);
int poly_find_pos_in_face(poly_polyhedre *polyhedre, poly_index index, poly_index point);
int poly_get_point_in_pos_in_face(poly_polyhedre *polyhedre, poly_index face, poly_index point, double *x, double *y, double *z);
int poly_get_point_in_edge(poly_polyhedre *polyhedre, poly_index edge, poly_index point, double *x, double *y, double *z);
int poly_find_edge(poly_polyhedre *polyhedre, poly_index p1, poly_index p2);
int poly_build_edges(poly_polyhedre *polyhedre);
int poly_build_plane_face(poly_polyhedre *polyhedre, poly_index numero);
int poly_get_plane(poly_polyhedre *polyhedre, poly_index index, poly_plane *plane);
int poly_get_plane_2_d(poly_polyhedre *polyhedre, poly_index index, double *a,double *b,double *c, double *d);
int poly_build_planes(poly_polyhedre *polyhedre);
int poly_build_face(poly_index *the_indexs, unsigned int nombre, poly_polyhedre *polyhedre);
*/

#endif

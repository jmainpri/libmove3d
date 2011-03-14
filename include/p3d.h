
/***************************************/
/* structure de polyhedre utilisee     */
/***************************************/     

#ifndef _P3D_H
#define _P3D_H

/*
  #include "p3d_sys.h"
  #include "p3d_type.h"
  #include "p3d_matrix.h"
  #include "p3d_poly.h"
*/
 #include "polyhedre.h"
/* the entity shapes: 0=polyh, 1=sphere, 2=cube, 3=box, 4=cylinder, 5=cone */
#define POLYHEDRON_ENTITY 0
#define CONVEX_POLYHEDRON 1
#define CONCAVE_POLYHEDRON 2
#define SPHERE_ENTITY 3
#define CUBE_ENTITY 4
#define BOX_ENTITY 5
#define CYLINDER_ENTITY 6
#define CONE_ENTITY 7

#include <../collision/PQP/include/PQP.h>

typedef double * configPt;

typedef struct {
  configPt q;
  char * name;
  int * ikSol;
} config_name, * config_namePt;

/* Point 3D */
typedef struct p3d_point {
  double x, y, z;
} p3d_point;

/* Boite 3D (utile pour les graphiques, la marge de manoeuvre des robots...) */
typedef struct box {
  double x1,x2,y1,y2,z1,z2;
} p3d_box, *pp3d_box;


/*---------------------------------------------------------------------------
 * BB structures.
 */

/*!
 * \brief List element for the pair selection of bounding boxes 
 *        distance conputation.
 */
typedef struct elem_list_BB {
  /*! \brief Fisrt objet 
   *
   * The distance is computed between \a obj1 and \a obj2.
   * \note This is always a body of the robot that owns the list.
   */
  struct obj * obj1;
  /*! \brief Second object.
   *
   * The distance is computed between \a obj1 and \a obj2.
   */
  struct obj * obj2;
  /*! \brief The next element of the list (NULL is the end of list) */
  struct elem_list_BB * next;
} p3d_elem_list_BB;


/*!
 * \brief Bounding boxes handle.
 *
 * Store the pair of object activate.
 * \warning All operationon a ::p3d_BB_handle are valid only in the 
 *          current environment.
 */
typedef struct BB_handle {
  /*! \brief The environment index. 
   * \note Always check if the current environment is the same.
   */
  int env_id;
  /*! \brief The robot number
   * \note Allow the memory release if the current environment has changed.
   */
  int nb_robot;
  /*! \brief Array of list of collision with the environment for each robot. */
  p3d_elem_list_BB ** lists_links_env;
  /*! \brief Array of list of collision with other robot for each robot. */
  p3d_elem_list_BB ** lists_links_rob;
  /*! \brief Array of list of autocollision for each robot. */
  p3d_elem_list_BB ** lists_links_autocol;
} p3d_BB_handle;

typedef struct BB {  
    double xmin,ymin,zmin;  
    double xmax,ymax,zmax;  
  } p3d_BB, *pp3d_BB; 

typedef struct p3d_primitive { 
  double radius;  /* sphere, cylinder, cone */
  double other_radius;  /* cone */ /* Carl 10 oct. 2000 */
  double height;  /* cylinder, cone */
  double sin_slope; /* sinus of slope of cone, angle at "top" between symmetry axis and side */
  double x_length; /* cube, box */
  double y_length; /* cube, box */
  double z_length; /* cube, box */
}p3d_primitive;


/* Modif Bio */
#ifdef BIO
typedef struct s_autocol {
  int size;
  struct p3d_poly **list; /*list of atoms with forbidden interaction. All must belong to rigids of higher index than the following field, nrigid */
  int nrigid; /* index of the rigid to which the atom owner of this structure belongs */
} autocol_struct;


enum atom_type {SULPHUR, SULPHUR_H, OXYGEN, OXYGEN_H, NITROGEN, NITROGEN_H, NITROGEN_FULL, CARBON, HYDROGEN,BROMINE, IODINE, FLUORINE, PHOSPHORUS, CHLORINE}; 
#endif



//! This enum is used to modify how the poly will be displayed.
//! This is meant to be used to determine if the object is visible or hides something.
typedef enum {
  POLY_DEFAULT_DISPLAY, /*!< default display mode */
  POLY_NO_DISPLAY, /*!< will not be displayed at all*/
  POLY_UNLIT_BLACK_DISPLAY, /*!< will be displayed in black with no light */
  POLY_UNLIT_RED_DISPLAY, /*!< will be displayed in red with no light */
  POLY_RED_DISPLAY, /*!< will be displayed in red */
  POLY_UNLIT_GREEN_DISPLAY, /*!< will be displayed in green with no light */
  POLY_GREEN_DISPLAY, /*!< will be displayed in green */
  POLY_UNLIT_BLUE_DISPLAY, /*!< will be displayed in blue with no light */
  POLY_BLUE_DISPLAY, /*!< will be displayed in blue */
  POLY_CUSTOM_COLOR_DISPLAY, /*!< will be displayed in the custom color */
  POLY_UNLIT_CUSTOM_COLOR_DISPLAY /*!< will be displayed in custom color with no light */
} poly_display_mode;

/* Structure de polyhedre permettant d'utiliser I_COLLIDE */
typedef struct p3d_poly
{
  struct obj *p3d_objPt; // pointer to the object (modif Juan)
  int entity_type;      /* the shape : 0,1,2=polyh, 3=sphere, 4=cube, 5=box, 6=cylinder, 7=cone */
  p3d_primitive *primitive_data; /* the data in case entity_type != polyhedron */
  poly_polyhedre *poly;  /*tsiano inclusion structure propre */
  void *end_user_data;
  int id;        /* numero d'identification */
  int geo_id;   /* identifier of poly from which this one was copied, or in case 
		   of direct construction the same as the one in field  id  */ /* UI 19042001 */
  int is_used_in_object_flag;  /* TRUE if poly is used in an object, FALSE otherwise */
  int p_in_o_id;  /* identifier of poly in the object, independent of instance of the object */   /* UI 30042001 */
  p3d_box box; /* boite pour les positions */
  p3d_matrix4 pos0 ; /* position initiale */
  /*! \brief Relative position from the joint of the object */
  p3d_matrix4 pos_rel_jnt ;
  int TYPE ; /* P3D_REAL : detecte et dessine ; P3D_GRAPHIC : dessine mais pas detecte ; P3D_GHOST : detecte mais pas dessine */
  int forceBBComputation;
  int color ; 
  double *color_vect;
  int list;       /* liste opengl du polyhedre */
  int listfil;    /* liste opengl du polyhedre filaire*/
  int listgour;   /* liste opengl du polyedre gouraud ? */
  int MODIF ;

  poly_display_mode display_mode; /*!< used to modify how the poly will be displayed (default display, red display,etc.)*/
  double custom_color[4];  /*!< used to temporarily change the poly's color while keeping the original color */


  /* Modif BIO */
#ifdef BIO
  enum atom_type type;
  autocol_struct *autocol_data;
#ifdef HYDROGEN_BOND
  double *r;      /* collision radius pointer*/
#else
  double r;       /* collision radius */
#endif
  /* vector of collision radii for hydrogen bridges and the like mod */
  double *radii; 
  double pos[3]; /* atom position */
  double (*matpos)[4]; /* transformation matrix of the atom */
#endif
 } p3d_poly;

//! enum to mark a body as belonging to a special part of a robot
typedef enum p3d_robot_part
{
  P3D_NOT_A_PART,
  P3D_HAND_PART,
  P3D_RIGHT_HAND_PART,
  P3D_LEFT_HAND_PART,
  P3D_ARM_PART,
  P3D_RIGHT_ARM_PART,
  P3D_LEFT_ARM_PART,
  P3D_BASE_PART,
  P3D_FINGER_PART,
  P3D_FINGERTIP_PART
} p3d_robot_part;

//! This enum is used to modify how the object will be displayed.
//! This is meant to be used to determine if the object is visible or hides something.
typedef enum {
  P3D_OBJ_DEFAULT_DISPLAY, /*!< default display mode */
  P3D_OBJ_NO_DISPLAY, /*!< will not be displayed at all */
  P3D_OBJ_UNLIT_BLACK_DISPLAY, /*!< will be displayed in black with no light */
  P3D_OBJ_UNLIT_RED_DISPLAY, /*!< will be displayed in red with no light */
  P3D_OBJ_RED_DISPLAY, /*!< will be displayed in red */
  P3D_OBJ_UNLIT_GREEN_DISPLAY, /*!< will be displayed in green with no light */
  P3D_OBJ_GREEN_DISPLAY, /*!< will be displayed in green */
  P3D_OBJ_UNLIT_BLUE_DISPLAY, /*!< will be displayed in blue with no light */
  P3D_OBJ_BLUE_DISPLAY, /*!< will be displayed in blue */
  P3D_OBJ_CUSTOM_COLOR_DISPLAY,  /*!< will be displayed in custom color */
  P3D_OBJ_UNLIT_CUSTOM_COLOR_DISPLAY,  /*!< will be displayed in custom color with no light */
} p3d_obj_display_mode;

/* Structure d'objet (obstacle ou corps d'un robot) */
typedef struct obj {
  char       *name;
  int         num;  /* non-unique identifier: starts at 0 for static objects '
                       as well as for each robot */
  int         o_id; /* unique identifier for static objects, bodies and mov objs. */
  int  o_id_in_env; /* unique identifier for static objects, bodies and mov objs. in this environment (used in p3d_kcd.c to make the link) */
  int         geo_id;   /* identifier of object from which this one was copied, or in case 
		   of direct construction the same as the one in field  id  */ /* UI 20042001 */
  int  GRAPHIC_TYPE; /* P3D_DEACTIVATED_OBSTACLE, P3D_ACTIVATED_OBSTACLE, 
			P3D_ADDABLE_OBSTACLE ou P3D_ADDED_OBSTACLE , 
			ou encore P3D_REAL_OBJECT, P3D_GRAPHIC_OBJECT, P3D_GHOST_OBJECT */
  void *end_user_data;
  int is_used_in_device_flag;  /* TRUE if object is used in a device, FALSE otherwise */
  int is_used_in_env_flag;  /* TRUE if object is used in an environment (as obstacle or as body), FALSE otherwise */
  int         type;
  p3d_matrix4  opos; /* for a body: position of the local frame (just information) */
  p3d_matrix4  BodyWrtPilotingJoint; /* for a body: placement matrix expressing
					the pos/orie of a body w.r.t. the frame of its piloting joint. */
  struct env *env;
  int np;             /* nombre de polyhedres dans l'objet */
  p3d_poly **pol, *polcur;
  p3d_box box;
  struct jnt *jnt;  /* joint that pilots this object */
  p3d_BB      BB;     /* boite englobante */
  p3d_BB      BB0;
  int concat; //if the object is concat flag to don't draw it
  int contact_surface; /*!< flag to indicates that (in some circumstances) contacts will be allowed with this object */


  PQP_Model *pqpModel; /*!< pointer to the collision model used by the PQP library */
  struct obj *pqpPreviousBody; /*!< possible previous body in the kinematics chain */
  unsigned int pqpID; /*!< identifiant used as an index in the collision pair array (see p3d_pqp.h) */
  p3d_matrix4 pqpPose; 
  struct obj *pqpUnconcatObj; /*!<if the object flag "concat" is 1, this will point to the object that is associated 
             to the same joint (that has the same field "jnt") but has flag concat=0 */



  p3d_obj_display_mode display_mode;  /*!< used to modify how the object will be displayed (default display, red display,etc.)*/

#ifdef HRI_PLANNER_GUI
  int caption_selected; 
  int show_pos_area;     /* boolean show/hide disc of position area */
  double max_pos_range;  /* Max Range distance for the position area */
  double min_pos_range;  /* Min Range distance for the position area */
  int trans;  /* TRUE if the object is transparent  */  
#endif

#ifdef DPG
  unsigned int nbPointCloud;
  p3d_vector3* pointCloud;
#endif
  enum p3d_robot_part robot_part; //! used to mark the body as belonging to a specific part of the robot
  double distance_weight; //! weight used in robot distance computation (to another robot or body)
  bool isDeformable; //! for human morphology adaptation
} p3d_obj, *pp3d_obj;


/* structure de pre-jacobienne */
typedef struct prejac {
  int numder;                 /*!< dof id from which the prejac derivates */
  double alpha;               /*!< dotproduct coefficient for this dof*/
  p3d_matrix4 J;              /*!< prejacobian matrix */
  struct prejac *prev_prejac; /*!< previous prejac */
} p3d_prejac;


/************************************************************/

extern double p3d_version;

//start path deform
typedef struct config_on_edge{ 
/*To test the visibility 
  of an edge from a given configuration*/
  configPt q;
  double p;  /*param along the edge*/
} p3d_config_on_edge;
//end path deform

#endif


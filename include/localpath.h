#ifndef _LOCALPATH_DEF_H
#define _LOCALPATH_DEF_H

#include "../localpath/flat/include/struct_flat.h"
#include "../other_libraries/softMotion/src/softMotionStruct.h"
#include "../localpath/include/p3d_localPathGroup.h"

#define NB_COORD_TRAILER  5
#define NB_COORD_REEDS_SHEPP 4
#define NB_COORD_HILFLAT 4

#define NB_JNT_TRAILER      3
#define NB_JNT_HILFLAT      2
#define NB_JNT_REEDS_SHEPP  1

/* Index of each coordinate in array numdof of struct flat_trailer */
#define TRAILER_DOF_X     0
#define TRAILER_DOF_Y     1
#define TRAILER_DOF_THETA 2
#define TRAILER_DOF_PHI   3
#define TRAILER_DOF_DC_DS 4

/* Index of each coordinate in array numdof of struct lm_hilflat */
#define HILFLAT_DOF_X     0
#define HILFLAT_DOF_Y     1
#define HILFLAT_DOF_THETA 2
#define HILFLAT_DOF_CURV   3


//#define SOFTMOTION_DOF    7
#define NB_PARAMS_SOFTMOTION 6
/* Number of params from .p3d that include Number of kinematic constraints  \
initialized with p3d files (Jmax, Amax and Vmax) \
for linear and angular motion*/
#ifdef MULTILOCALPATH
/* Maximun number of multigraph, you can change it if you want */
#define MAX_MULTILOCALPATH_NB 10
#endif

typedef enum {
 REEDS_SHEPP,
 LINEAR,
 MANHATTAN,
 TRAILER,
 TRAILER_FORWARD,
 HILFLAT,
 HILFLAT_FORWARD,
 DUBINS
#ifdef MULTILOCALPATH
 ,SOFT_MOTION  /* XB */
 ,MULTI_LOCALPATH
#endif
} p3d_localpath_type;

/* local planner type */
typedef enum {
  P3D_RSARM_PLANNER,
  P3D_LINEAR_PLANNER,
  P3D_MANH_PLANNER,
  P3D_TRAILER_PLANNER,
  P3D_TRAILER_FW_PLANNER,
  P3D_HILFLAT_PLANNER,
  P3D_HILFLAT_FW_PLANNER,
  P3D_DUBINS_PLANNER,
#ifdef MULTILOCALPATH
  P3D_SOFT_MOTION_PLANNER,    /* XB */
  P3D_MULTILOCALPATH_PLANNER,  // Call softMotion planner and softmotion call the specific planner ...
#endif
  P3D_NBLP_TYPE
} p3d_localplanner_type;

extern int P3D_NB_LOCAL_PLANNER;

#define DEFAULT_LOCAL_PLANNER P3D_LINEAR_PLANNER

/* types of Reeds and Shepp curve parts */
#define rs_type  int
#define RIGHT    1
#define LEFT     2
#define STRAIGHT 3

#define whichway int
#define BACKWARD  -1
#define FORWARD   1


/*! \brief structure to report distances done by joints */
typedef struct s_stay_within_dist_data {
  double vmax;   /* maximum linear speed for the joint */
  double wmax;   /* maximum angular speed for the joint */
  p3d_point p;   /* position of the joint */
  struct s_stay_within_dist_data * prev; /* previous joint */
  double vmax0;   /* maximum linear speed before the joint */
  double wmax0;   /* maximum angular speed before the joint */

} p3d_stay_within_dist_data;

/* structure de chemin de Reeds et Shepp sur les trois premieres
   variables de configuration et lineaires sur les autres */
typedef struct rs_data{
  configPt q_init;    /* config init sur la courbe   */
  configPt q_end;     /* config fin  sur la courbe   */
  double centre_x;        /* centre du cercle correspondant */
  double centre_y;        /* en x et y si c'est le cas (sinon 0) */
  double radius;          /* rayon du disque */
  whichway dir_rs;            /* sens = 1/-1 pour avant/arriere  */
  double val_rs;          /* valeur de la portion de courbe */
  rs_type type_rs;            /* type = 1/2/3 pour droite/gauche/segment */
  struct rs_data *next_rs; /* pointeur sur portion suivante  */
  struct rs_data *prev_rs; /* pointeur sur portion precedente */
} p3d_rs_data, *pp3d_rs_data;

/* structure de chemin lineaire */
typedef struct lin_data{
  configPt q_init;    /* config init sur la courbe   */
  configPt q_end;     /* config fin  sur la courbe   */
} p3d_lin_data, *pp3d_lin_data;

/* structure de chemin Manhattan */
typedef struct manh_data{
  configPt q_init;    /* config init sur la courbe   */
  configPt q_end;     /* config fin  sur la courbe   */
  configPt crit_q_init;    /* critere sens depend de cette  config init sur la courbe originale   */
  configPt crit_q_end;     /* critere sens depend de cette   config fin  sur la courbeo riginale   */
  double val_manh;         /* valeur de la portion de courbe */
} p3d_manh_data, *pp3d_manh_data;

typedef struct sub_trailer_data{
  configPt q_init; /* init config on sub local path   */
  configPt q_end; /* end config on sub local path   */
  double  v; /* projection of q_end on Gamma_init */
  double length;    /*length of the sub local path between q_init and q_end*/
  double alpha_0;    /* third derivative of alpha at beginning */
  double alpha_1;    /* third derivative of alpha at end */
  double  u_start;   /* parameter beginning of localpath on combination */
  double  u_end;     /* parameter end of localpath on combination */
  double gamma_1_min; /*the min of the first derivation of the curve*/
  double v_1_rob_max;/*maximal linear acceleration of robot on all the sub path*/
  double v_1_rem_max;/*maximal linear acceleration of trailer on all the sub path*/
  double w_1_rob_max;/* max angular acceleration of robot along sub path */
  double w_1_rem_max;/* max angular acceleration of trailer along sub path */
  double phi_max;/*the max of phi on all the way*/
  double phi_min;/*the min of phi on all the way*/
  double phi_1_tot;/*this is the kind of integral of phi_2 on the path*/
}p3d_sub_trailer_data, *pp3d_sub_trailer_data;


typedef struct trailer_data{
  p3d_sub_trailer_data *init_cusp;
  p3d_sub_trailer_data *cusp_end;
  int symmetric;
} p3d_trailer_data, *pp3d_trailer_data;

typedef struct sub_hilflat_data{
  configPt q_init; /* init config on sub local path   */
  configPt q_end; /* end config on sub local path   */
  double  v; /* projection of q_end on Gamma_init */
  double length;    /*length of the sub local path between q_init and q_end*/
  double  u_start;   /* parameter beginning of localpath on combination */
  double  u_end;     /* parameter end of localpath on combination */
  double gamma_1_min; /*the min of the first derivation of the curve*/
  double v_1_rob_max; /*maximal linear acceleration of robot on
			all the sub path*/
  double w_1_rob_max; /*maximal angular acceleration of robot
			on all the sub path*/
  double theta_1_tot;/*this is the kind of integral of theta' along the path*/
} p3d_sub_hilflat_data, *pp3d_sub_hilflat_data;


typedef struct hilflat_data{
  p3d_sub_hilflat_data *init_cusp;
  p3d_sub_hilflat_data *cusp_end;
  int symmetric;
} p3d_hilflat_data, *pp3d_hilflat_data;


/* Soft Motion interpolation */
// typedef struct softMotion_data_FREEFLYER{
//   double   J_max_lin;   /* Initialized with p3d files */
// 	double   A_max_lin;   /* Initialized with p3d files */
// 	double   V_max_lin;   /* Initialized with p3d files */
// 	double   J_max_ang;   /* Initialized with p3d files */
// 	double   A_max_ang;   /* Initialized with p3d files */
// 	double   V_max_ang;   /* Initialized with p3d files */
// 	Gb_v3    poseLinInit;
// 	Gb_v3    poseLinEnd;
// 	Gb_v3    velLinInit;
// 	Gb_v3    velLinEnd;
// 	Gb_v3    poseAngInit;
// 	Gb_v3    poseAngEnd;
// 	Gb_v3    velAngInit;
// 	Gb_v3    velAngEnd;
// 	SM_MOTION motion;
// 	double   motionTime; //Duration of motion = range_param
// } p3d_softMotion_data_FREEFLYER, *pp3d_softMotion_data_FREEFLYER;
//
// typedef struct softMotion_data_joint{
// 	double*   J_max;   /* Initialized with p3d files */
// 	double*   A_max;   /* Initialized with p3d files */
// 	double*   V_max;   /* Initialized with p3d files */
// 	double*   velInit;
// 	double*   velEnd;
// 	SM_MOTION_MONO* motion;
// 	double   motionTime; //Duration of motion = range_param
// } p3d_softMotion_data_joint, *pp3d_softMotion_data_joint;

typedef struct softMotion_data_specific{
	double*   J_max;   /* Initialized with p3d files */
	double*   A_max;   /* Initialized with p3d files */
	double*   V_max;   /* Initialized with p3d files */
	double*   velInit;
	double*   velEnd;
	double*   accInit;
	double*   accEnd;
	SM_MOTION_MONO* motion;
	double   motionTime; //Duration of motion = range_param
} p3d_softMotion_data_specific, *pp3d_softMotion_data_specific;

/* Soft Motion interpolation */
typedef struct softMotion_data{
	int isPlanned;        /* TRUE motion is planned else FALSE */
	int isPTP;            /* TRUE is point to point motion else FALSE */
	int nbJoints;
	int nbDofs;
	p3d_group_type gpType;
	configPt q_init;      /* config init     */
	configPt q_end;       /* config end      */
	configPt q_endp1;     /* config end+1    */
	p3d_softMotion_data_specific* specific;
} p3d_softMotion_data, *pp3d_softMotion_data;

/* data relative to softMotion local method and stored in p3d_rob */
typedef struct lm_softMotion{
	int nbJoints;
	int nbDofs;
	pgp_specific_str specific;
} softMotion_str, *psoftMotion_str;

/* pointeur sur les structues specifiques a chaque methode locale */
typedef union lm_specific {
  pp3d_rs_data rs_data;
  pp3d_lin_data lin_data;
  pp3d_manh_data manh_data;
  pp3d_trailer_data trailer_data;
  pp3d_hilflat_data hilflat_data;
  pp3d_softMotion_data softMotion_data; /* XB */
} p3d_lm_specific, *pp3d_lm_specific;

/* data structure that encodes a local path */
typedef struct localpath{
  /*
   *  fields
   */
  p3d_localpath_type     type_lp;  /* type de methode locale */
  char *localplanner_name;         /* name of local planner that
				      built this local path */
  p3d_lm_specific specific; /* pointeur vers les donnees specifiques
			       a chaque type de chemin local */

#ifdef MULTILOCALPATH
	//p3d_lm_specific mgSpecific[MAX_MULTIGRAPH_NB]; /* pointeur vers les donnees specifiques
	//a chaque type de chemin local pour chaque multigraph*/
	struct localpath* mlpLocalpath[MAX_MULTILOCALPATH_NB];
	int mlpID; /* ID of graph to which the localpath is associated */
	configPt q_init;
#endif

  struct localpath* prev_lp; /* the local paths can be put in a list */
  struct localpath* next_lp;
  int  valid;             /* TRUE if no collision */
  int  lp_id;             /* index of loc path in p3d_traj */
  int  sdk_id;         /* id of localpath in SDK */
  double length_lp;       /* store length of local path */
  double range_param;     /* parameter range: [0,range_param] */
  int* ikSol;             /* witch solutions of constraint are used*/
  int nbActiveCntrts;     /*The number of active constraints in the localpath*/
  int* activeCntrts;      /*The ids of active constraints in the localpath*/

#if defined(PQP) && defined(LIGHT_PLANNER)
	int isCarryingObject;
	struct rob *carriedObject; /*!< if the carried object is a freeflying robot */
	p3d_matrix4 Tgrasp;
#endif

  /*
   *  methods associated to the local path *
   */
  /*computes length of local path */
  double (*length)(struct rob *, struct localpath*);
  /*copy the local path */
  struct localpath* (*copy)(struct rob*, struct localpath*);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  struct localpath* (*extract_sub_localpath)(struct rob *,
					     struct localpath *localpathPt,
					     double l1, double l2);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  struct localpath* (*extract_by_param)(struct rob *,
					struct localpath *localpathPt,
					double l1, double l2);
  /* destroy the localpath */
  void (*destroy)(struct rob*, struct localpath*);
  /* computes the configuration at given distance along the path */
  configPt (*config_at_distance)(struct rob*,
				 struct localpath*,
				 double);
  /* computes the configuration for a given parameter along the path */
  configPt (*config_at_param)(struct rob*,
			      struct localpath*,
			      double);
  /* from a configuration on a local path, this function computes an
     interval of parameter on the local path on which all the points
     of the robot move by less than the distance given as input.
     The interval is centered on the configuration given as input. The
     function returns the half length of the interval */
  double (*stay_within_dist)(struct rob*,
			     struct localpath*,
			     double, whichway, double*);
  /* compute the cost of a local path */
  double (*cost)(struct rob*, struct localpath*);
  /* function that simplifies the sequence of two local paths: valid
     only for RS curves */
  struct localpath* (*simplify)(struct rob*,
				struct localpath*,
				int*);
  int (*write)(FILE *, struct rob*, struct localpath*);
} p3d_localpath , *pp3d_localpath;

/* data relative to RS local method and stored in p3d_rob */
typedef struct lm_reeds_shepp{
  double     radius;    /* rayon de braquage */
  int        numdof[NB_COORD_REEDS_SHEPP]; /* degrees of freedom on which
			   RS method applies (x, y, theta, z) (z is optional
			   if its value is -1, we don't use it) */
  int numjnt      ;     /* the number of the main joint */
  int nb_other_jnt;     /* number of other joints which are free */
  int *other_jnt;       /* other joints that are free */
} lm_reeds_shepp_str, *plm_reeds_shepp_str;

/* data relative to the trailer and stored in p3d_rob */
typedef struct flat_trailer{
  int numdof[NB_COORD_TRAILER];  /* respective coord of x, y, theta,
				    phi, dc_ds */
  int numjnt[NB_JNT_TRAILER];    /* respective the number of the main
				    joint of the trailer joint and of
				    the dc_ds joint*/
  int nb_other_jnt;              /* number of other joints which are free */
  int *other_jnt;                /* other joints that are free */
  FLAT_STR *flat_str;
} flat_trailer_str, *pflat_trailer_str;

/* data relative to RS local method and stored in p3d_rob */
typedef struct lm_hilflat{
  int        numdof[NB_COORD_HILFLAT]; /* degrees of freedom on
			   which hilare flat method applies (x, y,
			   theta, curv) (z is optional if its value is
			   -1, we don't use it) */
  int numjnt[NB_JNT_HILFLAT];     /* joint ids */
  int nb_other_jnt;     /* number of other joints which are free */
  int *other_jnt;       /* other joints that are free */
} hilflat_str, *philflat_str;


/* pointer to a local planner function */
typedef pp3d_localpath (*ptr_to_localplanner)(struct rob*, configPt, configPt, int*);
typedef pp3d_localpath (*ptr_to_softMotion_localplanner)(struct rob*, configPt, configPt, configPt, int*);

/* Array of pointers to localplanner functions. The indices of the array
 are the elements of the p3d_localplanner_type enumeration. */

extern ptr_to_localplanner array_localplanner[];

extern char * array_localplanner_name[];

typedef int (*ptr_to_softMotion_groupplanner)(struct rob*, int , p3d_group_type , p3d_softMotion_data*, int*);

extern ptr_to_softMotion_groupplanner  array_softMotion_groupplanner[];
extern char * array_group_name[];

#endif

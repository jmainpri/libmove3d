#ifndef UNIT_P3D_TYPE 
#define UNIT_P3D_TYPE 

#define P3D_ENV        0
#define P3D_OBSTACLE   1
#define P3D_ROBOT      2
#define P3D_BODY       3
#define P3D_TRAJ       4
#define TRJ_TRAJ       5
#define P3D_BODIES     6

/*---------------------------------------------------------------------------*/
/*! \brief Type of joints reconized by Move3D. */
typedef enum {
  /*! Joint rotate (1 dof). Its default axis is z. */
  P3D_ROTATE = 7,
  /*! Joint translate (1 dof). Its default axis is z. */
  P3D_TRANSLATE = 8,
  /*! Joint base. Only use as the first joint. It gives compatibility with
    previous version of Move3D. Furthermore, it could be use as placement 
    joint. It owns 6 dof (3 translations x, y, z, 3 rotations rx, ry, rz) */
  P3D_BASE = 9,
  /*! Joint plan (3 dof). Its default use is 2 translation x, y and
    then a rotation rz. */
  P3D_PLAN = 10,
  /*! This joint looks like the base joint but it could be place at any 
    level in the kinematic. */
  P3D_FREEFLYER = 11,
  /*! This joint is fixed, there is no degree of freedom. */
  P3D_FIXED = 13,
  /*! Knee joint (3 dof). It owns 3 dof (3 rotations rx, ry, rz) */
  P3D_KNEE = 15
} p3d_type_joint;

#define NB_JNT_TYPE 7

/*! Type of joint use by kineo. This is a P3D_ROTATE. */
#define P3D_PPIVOT     12

#define P3D_MAX_ENV    10
#define P3D_MAX_ROB    10

#define P3D_NULL_OBJ   -1

/* the entity shapes: 0=polyh, 1=sphere, 2=cube, 3=box, 4=cylinder, 5=cone */
#define POLYHEDRON_ENTITY 0
#define CONVEX_POLYHEDRON 1
#define CONCAVE_POLYHEDRON 2
#define SPHERE_ENTITY 3
#define CUBE_ENTITY 4
#define BOX_ENTITY 5
#define CYLINDER_ENTITY 6
#define CONE_ENTITY 7

/* Predefined colors */
#define Black 0
#define Blue 1
#define Yellow 2
#define Red 3
#define Green 4
#define White 5
#define Grey 6
#define Brown 7
#define Filaire 8
#define Skin 9
#define Blue2 10
#define DGrey 11
#define DSkin 12
#define DBrown 13
#define DGreen 14
#define Any 15
#define Violet 16
#define Orange 33

/* Tansparency colors */
#define tBlack 17
#define tBlue 18
#define tYellow 19
#define tRed 20
#define tGreen 21
#define tWhite 22
#define tGrey 23
#define tBrown 24
#define tFilaire 25
#define tSkin 26
#define tBlue2 27
#define tDGrey 28
#define tDSkin 29
#define tDBrown 30
#define tDGreen 31
#define tViolet 32
#define tOrange 34

/*  Les collisions checkers */

#define p3d_col_mode_none 0
#define p3d_col_mode_i_collide 1
#define p3d_col_mode_v_collide 2
#ifdef SOLID_ACT
#define p3d_col_mode_solid 3
#endif
#define p3d_col_mode_kcd 4
#define p3d_col_mode_gjk 5
#define p3d_col_mode_bio 6   /* modif Vicente */
#define p3d_col_mode_pqp 7   

/* D�ut modification Fabien */

/* Choice of the trajectory collision detector */

/*---------------------------------------------------------------------------*/
/*! \brief Type of test of collision for the local path. */
typedef enum { 
  /*! Test collision from the beginning to the end with the static environment
   *  and the autocollision (but with no garanty for autocollision). */
  TEST_TRAJ_CLASSIC,
  /*! Divide the localpath to test the collisions with the static environment
   *  and the autocollision (but with no garanty for autocollision).   */
  TEST_TRAJ_DICHOTOMIE,
  /*! Test collision from the beginning to the end with the static environment.
   *  Then test auto-collision from the beginning to the end 
   *  (with garanty for non-autocollision).   */
  TEST_TRAJ_CLASSIC_ALL, 
  /*! Divide the localpath to test the collisions with the static environment.
   *  Then test auto-collision from the beginning to the end 
   *  (with garanty for non-autocollision).   */
  TEST_TRAJ_DICHOTOMIE_ALL,
  /*! Test collision from the beginning to the end with the environment, other
   *  robots and the autocollision (but with no garanty for autocollision). */
  TEST_TRAJ_OTHER_ROBOTS_CLASSIC,
  /*! Divide the localpath to test the collisions with the environment, other
   *  robots and the autocollision (but with no garanty for autocollision). */
  TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE, 
  /*! Test collision from the beginning to the end with the environment and
   *  other robots. Then test auto-collision from the beginning to the end 
   *  (with garanty for non-autocollision).   */
  TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL,
  /*! Divide the localpath to test the collisions with the environment and
   *  other robots. Then test auto-collision from the beginning to the end 
   *  (with garanty for non-autocollision).   */
  TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE_ALL,
  /*! Test collision from the beginning to the end with the current
   *  collision context. */
  TEST_TRAJ_CONTEXT_CLASSIC,
  /*! Divide the localpath to test the collisions with the current
   *  collision context. */
  TEST_TRAJ_CONTEXT_DICHOTOMIE
} p3d_traj_test_type;


/* Parameter of bounding box selection */

typedef enum { 
//   ACTIVATE_BB_CLASSIC,     /* Activate BB but not for autocollision     */
  ACTIVATE_BB_ALL,         /* Activate BB with the collision activation */
  DEACTIVATE_BB            /* Deactivate BB                             */
} p3d_BB_selection_type;

/* Parameter of bounding box computation */

typedef enum { 
  COMPUTE_BB_CLOSE,        /* Compute BB with close method              */
  COMPUTE_BB_LARGE,        /* Compute BB with large method              */
  COMPUTE_BB_COL           /* Compute BB with collision checker data    */
} p3d_BB_compute_type;


/*---------------------------------------------------------------------------*/
/*! \brief Type of context of collision. */
typedef enum {
  /*! Select all the collision between one robot and all the environment. */
  P3D_COL_ROBOT_ALL,
  /*! Select all the collision between one robot and the static environment. */
  P3D_COL_ROBOT_ENV,
  /*! Select all the collision between one robot and all the environment,
   *  but not the autocollision. */
  P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL,
  /*! Select all the collision between one robot and the static environment,
   *  and the autocollision, but not with other robots. */
  P3D_COL_ROBOT_ALL_WITHOUT_OTHER,
  /*! Select the auto-collision for one robot. */
  P3D_COL_ROBOT_AUTOCOL,
  /*! Select all the collisions in the environment. */
  P3D_COL_ENV_ALL,
  /*! Select no collisions in the environment. */
  P3D_COL_ENV_NONE
} p3d_type_col_env_context;


/*---------------------------------------------------------------------------*/
/*! \brief Type of distance computation (for the collision checker). 
 *
 *  This parameter could be acceded by set_kcd_which_test() or
 *  get_kcd_which_test().
 *
 *  \note These types are added to the report parameter.   */
typedef enum {
  /*! Classic selection: all the distances with the static environment. */
  P3D_KCD_COL_TEST = 0,
  /*! Select all the distances with the static environment, 
   *  the other robot and the autocollision. */
  P3D_KCD_ROB_ALL = 10,
  /*! Select all the distances with the other robot. */
  P3D_KCD_ROB_ROB = 20,
  /*! Select all the distances with the autocollsision. */
  P3D_KCD_ROB_AUTO = 30,
  /*! Select all the distances with the environment. */
  P3D_KCD_ROB_ENV = 40,
  /*! Select all the distances with the static environment, 
   *  the other robot and the autocollision. For p3d_col_test_choice(),
   *  test all the robots. */
  P3D_KCD_EVERYTHING = 50,
  /*! Select all the distances with the environment. */
  P3D_KCD_MO_ENV = 60,
} p3d_type_col_choice;

/* Fin modification Fabien */

/* Status pour la planification */

#define P3D_SUCCESS    0
#define P3D_FAILURE    1
#define P3D_WRONG_CALL 2
#define P3D_ILLEGAL_START 3
#define P3D_ILLEGAL_GOAL  4
#define P3D_CONFIG_EQUAL  5

#define NDISCR 10 
#define NTEST 50


#define ISOLATED 0
#define LINKING 1



/************************************************
  Enumeration of the methods which 
   can be used to choose a direction of expansion
   during a tree expansion 
**************************************************/
#define GLOBAL_CS_EXP 1
#define SUBREGION_CS_EXP 2

/************************************************
  Enumeration of the methods which 
  can be used to choose the node to select
  during a tree expansion 
**************************************************/
#define NEAREST_EXP_NODE_METH 1
#define K_NEAREST_EXP_NODE_METH 2
#define BEST_SCORE_EXP_METH 3
#define K_BEST_SCORE_EXP_METH 4
#define RANDOM_IN_SHELL_METH 5
#define RANDOM_NODE_METH 6
#define NAVIGATION_BEFORE_MANIPULATION 7

/************************************************
  Enumeration of the available distances which 
  can be used
**************************************************/
#define GENERAL_CSPACE_DIST 1
#define ACTIVE_CONFIG_DIST 2
#define LIGAND_PROTEIN_DIST 3
#define MOBILE_FRAME_DIST 4
#define ONLY_ROBOT_BASE 5

/************************************************
  Enumeration of the methods which 
  can be used to choose to expand a node during
  a tree expansion
**************************************************/
#define ONE_NODE_CONNECT_EXP_CHOICE 1
#define N_NODES_EXTEND_EXP_CHOICE 2
#define EXTEND_EXP_CHOICE 3

/************************************************
  Enumeration of the methods which 
  can be used to choose to connect a node to
  a given connected componant
**************************************************/
#define NEAREST_NODE_COMP 1
#define K_NEAREST_NODE_COMP 2

/************************************************
  Enumeration of the methods which 
  can be used to weight a given node
**************************************************/
#define RMSD_DISANCE_WEIGHT 1
#define ENERGY_WEIGHT 2

/************************************************
  Enumeration of the choices to compute the cost
 of an elementary path in function of the
 cost of the extremites configurations
**************************************************/
#define INTEGRAL 0
#define MECHANICAL_WORK 1
#define AVERAGE_CONFIG_COST 2
#define CONFIG_COST_AND_DIST 3
#define BOLTZMANN_COST 4
#define VISIBILITY 5

/**
 * Enumeration of  the methods used for 
 * exploring cost spaces (note that monte carlo
 * can also be used in classical spaces as a 
 * random walk method).
 */
#define MAXIMAL_THRESHOLD 1 //Ettlin / Bleuer variant
#define URMSON_TRANSITION 2 //Urmson / Simmons variant
#define TRANSITION_RRT 3
#define TRANSITION_RRT_CYCLE 4
#define MONTE_CARLO_SEARCH 5
#define DOWN_SEARCH 6
#define ADAPTIVE_THRESHOLD 7 //the threshold value depends of successes and failures

/**
 *  Enumeration of sampling methods used for 
 *  PRM-like algorithms
 */
#define P3D_UNIFORM_SAMPLING 1
#define P3D_GAUSSIAN_SAMPLING 2
//#define P3D_DYNAMIC_DOMAIN_SAMPLING 3
#define P3D_BRIDGE_SAMPLING 4
#define P3D_OBPRM_SAMPLING 5

#define P3D_HALTON_SAMPLING 1
#define P3D_RANDOM_SAMPLING 2

#define P3D_NO_EXPAND 0
#define P3D_EXPAND_BOX 1
#define P3D_EXPAND_DISCR_CONT 2
#define P3D_EXPAND_RAND_WALK 3


#define P3D_BASIC 1
#define P3D_ISOLATE_LINKING 2
#define P3D_ALL_PRM 3
#define P3D_PDR 4
#define P3D_DIFFUSION 5
#define BIO_COLDEG_RRT 6


#define P3D_NB_CONNECT 1
#define P3D_DIST_NODE 2

#define P3D_REAL 0
#define P3D_GRAPHIC 1
#define P3D_GHOST 2
#define P3D_LINES 3      // PROV. MODIF JUAN

#define P3D_REAL_OBJECT          100
#define P3D_GRAPHIC_OBJECT       101
#define P3D_GHOST_OBJECT         102
#define P3D_DEACTIVATED_OBSTACLE 103
#define P3D_ACTIVATED_OBSTACLE   104
#define P3D_ADDABLE_OBSTACLE     105
#define P3D_ADDED_OBSTACLE       106

//Inverse Kinematic
#define IK_NORMAL 0
#define IK_UNIQUE 1
#define IK_MULTISOL 2

//PLANNING TYPE
#define P3D_NONE 0
#define P3D_GLOBAL 1
#define P3D_SPECIFIC 2

//Astar Graph Type
#define DEFAULTGRAPH 0
#ifdef MULTIGRAPH
#define MGGRAPH 1
#endif

//start path deform
#define  ALL_RETRACT_SEARCH 0
#define  FORWARD_RETRACT_SEARCH 1
#define  LINEAR_RETRACT_SEARCH 2
//end path deform
#endif

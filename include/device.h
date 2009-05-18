/****************************************************************************/
/*!
 * \file device.h
 *
 * \brief Device and joint structures.
 *
 *     Description of the joint (::p3d_jnt) and device (::p3d_rob) structures.
 */
/****************************************************************************/

#ifndef _DEVICE_H
#define _DEVICE_H

/*! \brief Maximum number of degree of freedom for complex joints */
#define JNT_NB_DOF_MAX     6

/*! \brief Maximum size for the name of a joint */
#define JNT_MAX_SIZE_NAME 80

#define NDOF_BASE           6
#define NDOF_BASE_TRANSLATE 3
#define NDOF_BASE_ROTATE    3

/*! \brief Number of position parameters (x, y, z, Rx, Ry, Rz). */
#define NB_POS_PARAM 6
/*! \brief Number max of check point positions */
#define MAX_TRANSITION 10

/*--------------------------------------------------------------------------*/
/*! \brief Structure to hold the informations on links between joints.
 *  \note  Use only for the interface of joint creation.
 *  \internal
 */
typedef struct s_read_jnt_link_data {
  /*! \brief Link to an other joint. */
  struct jnt * jnt_link;
  /*! \brief Other joint num. */
  int jnt_num;
  /*! \brief Other joint multi-robot num (-1 is the environment). */
  int robot_num;
  /*! \brief TRUE if \a jnt_link is a previous joint. */
  int flag_prev;
  /*! \brief Relative position between the two joints (x, y, z, Rx, Ry, Rz) */
  double pos_param[NB_POS_PARAM];
} p3d_read_jnt_link_data;

/*--------------------------------------------------------------------------*/
/*! \brief Structure to store the parameters use for the joint definition.
 *  \note  Use only for the interface of joint creation.
 *  \internal
 */
typedef struct p3d_read_jnt_data {
  p3d_type_joint type;
  double scale;
  char name[JNT_MAX_SIZE_NAME];
  int flag_name;
  int prev_jnt;
  int flag_prev_jnt;
  int nb_dof;
  double * v;
  int flag_v;
  double * v_pos0;
  int flag_v_pos0;
  double * vmin;
  int flag_vmin;
  double * vmax;
  int flag_vmax;
  double * vmin_rand;
  int flag_vmin_rand;
  double * vmax_rand;
  int flag_vmax_rand;
  int * is_user;
  int flag_is_user;
  int * is_active_for_planner;
  int flag_is_active_for_planner;
  int nb_param;
  double * param;
  int flag_param;
  p3d_matrix4 pos;
  int flag_pos;
  int flag_relative_pos;

  int nb_links;
  struct s_read_jnt_link_data ** link_array;

} p3d_read_jnt_data;


/*!
 * \brief Structure used to store the data of each degree of freedom.
 */
typedef struct
{
  /*! \brief Axis of the degree of freedom.  */
  p3d_vector3 axis;

  /*! \brief Value of the degree of freedom. */
  double v;

  /*! \brief Old value of the degree of freedom.
  *   \note  Use to see if there is a change in this degree of freedom. */
  double old_v;

  /*! \brief Reference value of the degree of freedom.
   *
   * Note: Use only in the sdk. The real value is v+v0.
   */
  double v0;

  /*! \brief Minimum bounds value of the degree of freedom. */
  double vmin;

  /*! \brief Maximum bounds value of the degree of freedom. */
  double vmax;

  /*! \brief Minimum bounds of random value of the degree of freedom.
   *
   * Note: Those bounds are used in p3d_shoot.
   */
  double vmin_r;

  /*! \brief Maximum bounds of random value of the degree of freedom.
   *
   * Note: Those bounds are used in p3d_shoot.
   */
  double vmax_r;

  /*! \brief Flag to show if the degree of freedom is controled by user.
   *
   * \note If it isn't controled by user, then it isn't controled by p3d_shoot.
   *       This flag could be used for placement joint, passif joint or
   *       degree of freedom computed by local method.
   */
  int   is_user;

  /*! \brief Flag to show if the degree of freedom is active/passive for the planner (used by ML-RRT).
   *
   * \note This flag maybe redundant with the flag "is_user".
   */
  int   is_active_for_planner;

  /*! \brief Flag to show if the degree of freedom has been modified.
   *
   * \note This flag is not set to FALSE by
   *       p3d_update_this_robot_pos_without_obj(), but
   *       p3d_update_this_robot_pos_without_cntrt() and
   *       p3d_update_this_robot_pos() put it to false after its modification.
   */
  int   is_modified;

  /*! \warning unused... at this time. */
  int lock;
} p3d_dof_data;


/*!
 * \brief Structure used to store the data of each degree of freedom.
 */
typedef struct link_between_joint
{
  /*! \brief Relative position between two joints.  */
  p3d_matrix4 rel_pos;

  /*! \brief Previous joint */
  struct jnt * prev_jnt;

  /*! \brief Next joint */
  struct jnt * next_jnt;

} p3d_link_between_joint;


/*!
 * \brief Joint structure
 */
typedef struct jnt
{
  /*!
   * \brief The joint type.
   *
   * Note: This is one of the ::p3d_type_joint joint type. There are presently:
   *   - ::P3D_ROTATE    : Joint rotate (1 dof). See p3d_jnt_rotate.c
   *   - ::P3D_TRANSLATE : Joint translate (1 dof). See p3d_jnt_translate.c
   *   - ::P3D_BASE      : Joint base. Only use as the first joint. It gives
   *                       compatibility with previous version of Move3D.
   *                       Furthermore, it could be use as placement joint.
   *                       It owns 6 dof (3 translations x, y, z,
   *                       3 rotations rx, ry, rz). See p3d_jnt_base.c
   *   - ::P3D_PLAN      : Joint plan (3 dof). Its default use is 2
   *                       translation x, y and then a rotation rz.
   *                       See p3d_jnt_plan.c
   *   - ::P3D_FREEFLYER : This joint looks like the base joint but it could
   *                       be place at any level in the kinematic and can be
   *                       placed with any rotation. See p3d_jnt_freeflyer.c
   */
  p3d_type_joint   type;

  /*! \brief Kineo joint type.
   *
   * Note: KD_JNT_TYPE_... PRISMATIC, REVOLUTE, FREEFLYER, PLAN, PPIVOT,
   *       TRAILER_FALSE
   *
   * (API add-in) KINEO DEV:18/09/2001
   */
  int kinematic_type;
#ifdef BIO
  /*! \brief Joint type in bio-molecules. */
  int bio_jnt_type;  // modif Juan
   /*! \brief AAnumber associated to the joint  in bio-molecules. */
  int bio_AAnumber;  // modif ljaillet
   /*! \brief num of the subrobot for the joint  in biostructure. */
  int num_subrobot;
#endif
  /*! \brief Joint rank in device kinematic chain. */
  int num;

  /*! \brief Joint name. */
  char  name[JNT_MAX_SIZE_NAME];

  /*! \brief joint in ::XYZ_JOINT (UI KIN)
   *
   * modif. Carl 16052001
   */
  int ui_kin_num;

  /*! \brief Identifier of kin_joint data structure
   *         defined through UI KINEMATICS
   *
   * modif. Carl 03052001
   * \warning not used
   */
  int kin_joint_id;

  /*! \brief Counts number of times the joint was used in a device */
  int is_used_in_device_flag;

  /*! \brief Robot to which the joint belongs */
  struct rob  *rob;

  /*! \brief Object connected by the joint */
  struct obj  *o;

  /*----------------------------------------------------------------------
   * Old parameters of the joints (one dof)
   */

  /*! \brief Axis of the degree of freedom.
   * \warning Only used for compatibility.
   */
  p3d_vector3 axe;

  /*! \brief Value of the degree of freedom.
   * \warning Only used for compatibility.
   */
  double v;

  /*! \brief Minimum bounds value of the degree of freedom.
   * \warning Only used for compatibility.
   */
  double vmin;

  /*! \brief Maximum bounds value of the degree of freedom.
   * \warning Only used for compatibility.
   */
  double vmax;

  /*! \brief Minimum bounds of random value of the degree of freedom.
   *
   * Note: Those bounds are used in p3d_shoot.
   * \warning Only used for compatibility.
   */
  double vmin_rand;

  /*! \brief Maximum bounds of random value of the degree of freedom.
   *
   * Note: Those bounds are used in p3d_shoot.
   * \warning Only used for compatibility.
   */
  double vmax_rand;


  /*----------------------------------------------------------------------
   * Placement parameters
   */

  /*! \brief Point where the joint is attached in the environment in
   *         initial position.
   *  \note  Used to compute the relative position of the obstacle */
  p3d_matrix4 pos0_obs;

  /*! \brief Point where the joint is attached in the environment in
   *         initial position.
   *  \note Can be modify by p3d_set_dof_pos0 */
  p3d_matrix4 pos0;

  /*! \brief Point where the joint is attached in the environment in
   *         initial position.
   * \warning DO NOT USE (instead use p3d_jnt_get_point(),
   * p3d_jnt_get_vect_point() or jnt::pos0 */
  p3d_point p0;

  /*! \brief Point where the joint is attached
   *        (relative to the previous joint)
   * \warning Only used by the sdk.
   */
  p3d_point relative_p0;

  /*! \brief Flag to set the position relative to the previous joint.
   *
   * TRUE if relative_p0 given by initialization (p0 must be computed),
   * FALSE if p0 given by initialization (relative_p0 contains trash)
   * \warning Only used by the sdk.
   */
  int   is_defined_relative;

  /*! \brief Matrix of the current absolute position of the joint.
   *
   *  \note It is compute at each update of the robot by
   *  p3d_jnt_calc_mat_pos(), p3d_jnt_init_mat_pos() or
   *  p3d_jnt_calc_mat_pos_and_jac(), p3d_jnt_init_mat_pos_and_jac()
   */
  p3d_matrix4 abs_pos;

  /*! \brief Flag to show if the absolute position has changed. */
  int abs_pos_modified;

  /*! \brief Matrix of the current absolute position of the joint before
   *         the apply of jnt::jnt_mat.
   *
   *  \note It is compute at each update of the robot by
   *  p3d_jnt_calc_mat_pos(), p3d_jnt_init_mat_pos() or
   *  p3d_jnt_calc_mat_pos_and_jac(), p3d_jnt_init_mat_pos_and_jac()
   */
  p3d_matrix4 abs_pos_before_jnt;

  /*! \brief Flag to show if jnt::abs_pos_before_jnt has changed. */
  int abs_pos_before_jnt_modified;

  /*! \brief Flag to show if the joint position has been updated. */
  int pos_obj_modified;

  /*! \brief Flag to show if the joint position has been updated. */
  int pos_updated;

  /*! \brief Joint matrice.
   *
   * \note This matrice represent the modification due to the degree
   *       of freedom.
   */
  p3d_matrix4   jnt_mat;

  /*! \brief Flag to show if the joint matrice jnt::jnt_mat has changed. */
  int mat_modified;


  /*----------------------------------------------------------------------
   * The link to other joints.
   */

  /*! \brief Previous joint. */
  struct jnt  *prev_jnt;

  /*! \brief Array of the next joints. */
  struct jnt  **next_jnt;

  /*! \brief Number of next joints in jnt::next_jnt. */
  int   n_next_jnt;

  /*! \brief Array of the links between joints. */
  struct link_between_joint **link_jnt_arr;

  /*! \brief Number of links between joints in jnt::link_jnt_arr. */
  int        n_link_jnt;

  /*! \brief Array of the links between joints owned by this joint.
   *  \note  Use to destroy them at the end. */
  struct link_between_joint **link_jnt_owned_arr;

  /*! \brief Number of links between joints in jnt::link_jnt_owned_arr. */
  int        n_link_jnt_owned;

  /*! \brief Maximal distance between the point the joint
   * is attached to and the points of the body.
   */
  double  dist;

  /*! \brief pre-jacobian matrix list */
  p3d_prejac  *prejac;

  /*! \brief Pointer for usre data.
   * \warning Used by the sdk.
   */
  void  *end_user_data;

  /*! \brief index of the joint's dof in a ::configPt */
  int   index_dof;

  /*! \brief index of the joint's dof in a user config
   *
   * Note: The user config, is a config where there aren't the degree of
   * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
   */
  int   index_user_dof;

  /*! \brief The number of degree of freedom */
  int dof_equiv_nbr;

  /*! \brief The number of user degree of freedom that appear in a
   *         user configuration.
   *
   * Note: The user config, is a config where there aren't the degree of
   * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
   */
  int user_dof_equiv_nbr;

  /*! \brief The degree of freedom data. */
  p3d_dof_data  dof_data[JNT_NB_DOF_MAX];

} p3d_jnt, *pp3d_jnt;

/*!
 * \brief List of local methods
 */
typedef struct lm_list_param
{
  p3d_localplanner_type lpl_type;
  void  *lm_param;
  struct lm_list_param  *next;

} lm_list_param_str, *plm_list_param_str;

#ifdef MULTIGRAPH
/** \brief List of joint. each list is independant (we can found a joint only once in all the multiGraphJoint struct)*/
typedef struct multiGraphJoint{
  /** \brief Number of joint implicated in this struct*/
  int nbJoints;
  /** \brief Implicated joint. Only this joints and the joint in no list are sampled at each time*/
  int * joints;
  /** \brief Implicated constraints*/
  int * cntrts;
}p3d_multiGraphJoint;

typedef struct multiGraph{
  /** \brief Number of multiGraphJoints lists = number of created graphs*/
  int nbGraphs;
  /** \brief The graphs list for the multiGraph computations*/
  struct graph ** graphs;
  /** \brief 1 if the mg is active 0 otherwise*/
  int * active;
  /** \brief the list of multiGraphJoints*/
  struct multiGraphJoint ** mgJoints;
  /** \brief A joint list to save if the joint is in a multiGraphJoints list or not (1 if used 0 otherwise) this list have the robot nbjoints size*/
  int * usedJoint;
  /** \brief The flat super graph: a coordination graph between the generated graphs*/
  struct flatSuperGraph * fsg;
} p3d_multiGraph;
#endif

#ifdef MULTILOCALPATH
/** \brief List of joint. each list is independant (we can found a joint only once in all the multiGraphJoint struct)*/
typedef struct multiLocalPathJoint{
	/** \brief Number of joint implicated in this struct*/
	int nbJoints;
	/** \brief Implicated joint. Only this joints and the joint in no list are sampled at each time*/
	int * joints;
	/** \brief Group name see array_group_name in group.h*/
	char gpName[30];
	/*! \brief Group type see p3d_group_type in group.h */
	p3d_group_type	gpType;
	/*! \brief Current local planner associated to the sub-graph */
	p3d_localplanner_type	lplType;
	/*! \brief Parameters of local planner associated to the sub-graph */
	plm_list_param_str local_method_params;

}p3d_multiLocalPathJoint;

typedef struct multiLocalPath{
	/** \brief Number of multiGraphJoints lists = number of created localPath group*/
	int nblpGp;
	/** \brief 1 if the mlp is active 0 otherwise*/
	int * active;
	/** \brief the list of multiLocalpathJoints*/
	struct multiLocalPathJoint ** mlpJoints;
	/*! \brief Array of pointers to trajectories.*/
	p3d_traj	**t;
} p3d_multiLocalPath;
#endif
/*!
 * \brief Structure de robot
 *
 * Note: For compatibility reasons, tx, ty, tz translation values dof
 *       are also stored in joints[0].p0.x, joints[0].p0.y, joints[0].p0.z
 */
typedef struct rob
{
  char  *name;

  /*! \brief Robot number in the environment (env::robots). */
  int num;

  /*! \brief Identifier of device data structure in ::XYZ_DEVICE
   * defined through UI KINEMATICS
   *
   * modif. Carl 03052001
   */
  int ui_kin_num;

  /*! \brief Counts number of times the device was used in an environment  */
  int is_used_in_env_flag;

  /*! KD_KIN_TYPE_ARTICULATED, _FREEFLYER, _CART, _FIXED  */
  int kinematic_type;

  struct env  *env;

  /*----------------------------------------------------------------------
   * corps du robots
   */
  int no;
  p3d_obj **o,*ocur;

  /*! \brief Number of degre of freedom in the robot */
  int  nb_dof;

  /*! \brief Number of degre of freedom in the robot user config
   *
   * Note: The user config, is a config where there aren't the degree of
   * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
   */
  int  nb_user_dof;

  /*! \brief Number of joints in rob::joints */
  int njoints;
  /*! \brief Array of joints in the robot */
  p3d_jnt **joints;

  /*! \brief First joint modified.
   *
   * \note Use to reduce the computation of the update of joint
   *       position matrix (jnt::abs_pos).
   *
   * \warning Do not use anymore, now the function p3d_jnt_set_dof() select
   *          the joints that need update.
   */
  p3d_jnt *j_modif;

  /*! \brief First of the kinematic chain
   *
   *  It could be use to inverse the cinematic.
   *  If it is NULL, use the first joint of rob::joints.
   */
  p3d_jnt *first_joint;

  /*! \brief Absolute position of the first joint
   *
   *  This gives the absolute position of the first joint (rob::first_joint).
   *  If the pointer is NULL, then the absolute position (jnt::abs_pos)
   *  is given by the joint matrix (jnt::jnt_mat).
   */
  p3d_matrix4 * first_abs_pos;

  /*! \brief Number of trajectories stored in array rob::t */
  int   nt;

  /*! \brief Array of pointers to trajectories. */
  p3d_traj  **t;
  /*! \brief Pointer to current trajectory */
  p3d_traj  *tcur;

  /*! \brief Bounds for the base joint.
   * \warning Doesn't use anymore (stored in the ::P3D_BASE joint).
   */
  p3d_box box;
  /*! \brief Rotation angles of main body
   * \warning Doesn't use anymore (stored in the ::P3D_BASE joint).
   */
  double rx;
  /*! \brief Rotation angles of main body
   * \warning Doesn't use anymore (stored in the ::P3D_BASE joint).
   */
  double ry;
  /*! \brief Rotation angles of main body
   * \warning Doesn't use anymore (stored in the ::P3D_BASE joint).
   */
  double rz;
  /*! \brief Minimal bounds for the orientations of main body
   * \warning Doesn't use anymore (stored in the ::P3D_BASE joint).
   */
  double vmin_rot[NDOF_BASE_ROTATE];
  /*! \brief Maximal bounds for the orientations of main body
   * \warning Doesn't use anymore (stored in the ::P3D_BASE joint).
   */
  double vmax_rot[NDOF_BASE_ROTATE];

  /*! \brief Flag to show the robot collision. */
  int coll;

  /*! \brief Robot bounding box. */
  p3d_BB BB;

  /*! \brief Number of configuration */
  int nconf;
  /*! \brief array of configuration */
  config_namePt *conf;
  /*! \brief current configuration */
  config_namePt confcur;
  /*! \brief Current configuration */
  configPt  ROBOT_POS;
  /*! \brief Goal configuration */
  configPt  ROBOT_GOTO;
#ifdef MULTILOCALPATH
  /*! \brief Intermediate configuration : configuration displayed in 3D windows */
  configPt	ROBOT_INTPOS;
#endif
  /*! \brief Transition configuration */
  configPt transitionConfigs[MAX_TRANSITION];
  /*! \brief Number of transition configurations */
  int nTransition;
  // modif Juan ( not used yet ! )
  /*! \brief Start iksol vector */
  int *ikSolPos;
  /*! \brief Goal iksol vector */
  int *ikSolGoto;
  /*! \brief Transition iksol vector */
  int *ikSolTransition[MAX_TRANSITION];
  /*!  \brief the robot IKSol */
  int *ikSol;
  // fmodif Juan
  /*! \brief sdk identifier of ROBOT_POS  (for sdk only !) */
  int sdk_id_qinit;
  /*! \brief sdk identifier of ROBOT_GOTO (for sdk only !) */
  int sdk_id_qgoal;

  /*! \brief array of length coefficients weighting values
   * of angular joints in distance computations.
   * \warning Not used anymore
   */
  double *length_array;

  /*! \brief Current local planner associated to the robot */
  p3d_localplanner_type lpl_type;
  /*! \brief Parameters of local planner associated to the robot */
  plm_list_param_str local_method_params;

  /*! \brief robot free positions graph */
  struct graph  *GRAPH;

  /*! \brief Management of robot's constraints */
  struct cntrt_management *cntrt_manager;  // modif. Juan

  /*! \brief pointer for user applications */
  void    *user_appli;

  configPt currect_q_inv;  // temporary modif

#ifdef BIO
  int nbAA;
  int nb_flexible_sc;
  p3d_jnt** list_firstjnts_flexible_sc;
  int  nb_ligand_dof;
  int num_subrobot_ligand;
  p3d_jnt** list_AA_firstjnt;
#endif

#ifdef MULTIGRAPH
  struct multiGraph *mg;
#endif

#ifdef MULTILOCALPATH
  struct multiLocalPath *mlp;
#endif

#ifdef LIGHT_MODE
  int graspNbJoints;
  p3d_jnt** graspJoints;
  p3d_jnt* baseJnt;
  p3d_jnt* objectJnt;
  int nbCcCntrts; //Closed Chain constraints
  struct cntrt ** ccCntrts;
#endif
} p3d_rob, *pp3d_rob;

#endif /* #ifndef _DEVICE_H */

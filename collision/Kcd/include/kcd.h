#ifndef KCD_H
#define KCD_H

typedef int kcd_index;
typedef kcd_index *kcd_index_p;
typedef double kcd_vector3[3];
typedef kcd_vector3 *pkcd_vector3;
typedef double kcd_matrix3[3][3];
typedef double kcd_matrix4[4][4];


/*--------------------------------------------------------------------------
 * Activation / deactivation of collision test.
 *
 * Modif: Fabien Gravot
 */

/*! \brief Type use to define the activation of movable object.
 *  \note The possible values are:
 *        - ::KCD_TABLE_COL_ACT
 *        - ::KCD_TABLE_DIST_ACT
 *        - ::KCD_TABLE_COL_DIST_ACT
 *        - ::KCD_TABLE_COL_DIST_DEACT
 */
typedef unsigned char mo_act_type;

/*! \brief Type use to define the activation of movable object.
 *  \note The absolute value is the number of activation of movable
 *        object in the group.
 *        If this value is negative, then the group is deactivate.
 */
typedef short grp_act_type;

/*! \brief Activate only a collision pair.
 *  \note  This allow to specified the test needed between two objects
 *         kcd_col_handle.act_mo_pair_arr or kcd_col_handle.act_mo_env_arr */
#define KCD_TABLE_COL_ACT        ((mo_act_type)0x01)
/*! \brief Activate only a distance computation pair.
 *  \note  This allow to specified the test needed between two objects
 *         kcd_col_handle.act_mo_pair_arr or kcd_col_handle.act_mo_env_arr */
#define KCD_TABLE_DIST_ACT       ((mo_act_type)0x02)
/*! \brief Activate both collision and distance computation pair.
 *  \note  This allow to specified the test needed between two objects
 *         kcd_col_handle.act_mo_pair_arr or kcd_col_handle.act_mo_env_arr */
#define KCD_TABLE_COL_DIST_ACT   ((mo_act_type)0x03)
/*! \brief Deactivate both collision and distance computation pair.
 *  \note  This allow to specified the test needed between two objects
 *         kcd_col_handle.act_mo_pair_arr or kcd_col_handle.act_mo_env_arr */
#define KCD_TABLE_COL_DIST_DEACT ((mo_act_type)0x00)

/*!
 * \brief Collision handle for Kcd.
 *
 * Store the pair of collision that must be tested.
 */
typedef struct kcd_col_handle
{
  /*! \brief Number of movable object */
  int nof_mos;

  /*! \brief Selection of pair of collision between two movable objects.
   *
   * Note: Triangular matrix, only use act_mo_pair_arr[i][k-1-j] for i < j
   */
  mo_act_type **act_mo_pair_arr;
  
  /*! \brief Selection of pair of collision between movable object and 
   *         the environment.
   */
  mo_act_type *act_mo_env_arr;  

  /*! \brief Number of group of movable object */
  int nof_grps;

  /*! \brief Selection of pair of collision between two group 
   *         of movable objects.
   *
   * Note: Triangular matrix, only use act_mo_pair_arr[i][k-1-j] for i <= j
   */
  grp_act_type **act_grp_pair_arr;
  
  /*! \brief Selection of pair of collision between group of movable object
   *         and the environment.
   */
  grp_act_type *act_grp_env_arr;  
} kcd_col_handle, *kcd_col_handle_p;


typedef struct kcd_bb
{
  int bb_id_in_all_bbs;         /* unique id, place in array all_bbs[] */
  /* what is the best box represented here */
  int best_box_type;                 /* UNSET: 0, AABB_BOXTYPE: 1, OBB_BOXTYPE: 2 */

  /* AABB: */
  double x1,x2,y1,y2,z1,z2;    /* x1 <= x2, y1 <= y2, z1 <= z2, values of box vertices */
  int is_aabb_tree_leaf;       /* TRUE or FALSE */
  int current_status_aabb;     /* UNSET, UNCHANGED, MOVED_FORWARD, MOVED_BACKWARD, MOVED */
  int hier_level;              /* in case of non-leaf, the indices */
  int place_at_level;          /*  that allow access to position in AABB-tree */

  /* B Kineo Carl 27.02.2002 */
  int ext_obj_id;   /* external identifier of object to which belong the polyhedrons and solids
		       inside this box, if this object is the same for all; -1 otherwise */
  /* E Kineo Carl 27.02.2002 */

  /* OBB: */
  int is_obb_root;       /* true if OBB is root of OBB-tree or if OBB around solid */
  int type_of_obb;       /* NEW: OBB_FLAT,OBB_SPHERICAL or OBB_TALL */
  double d[3];                 /* half box size in three "eigen" directions */
  double volume;               /* volume of box = 8.0 * d[0] * d[1] * d[2]    */
  double center[3];            /* the center of the box w.r.t. frame specified below */
  kcd_vector3 eigenv[3];       /* the eigen vectors (unit vectors!) ordered in decending importance, 
				   w.r.t. frame specified below */
  int reference_frame;         /* WORK, LOCAL */
  int current_status_obb;      /* UNSET, UNCHANGED, MOVED */
  struct kcd_bb *parent_bb;    /* parent box in an OBB-TREE, for complex things */
  struct kcd_bb **children_bb;    /* child boxes in an OBB-TREE, for complex things */
  int nof_children_bb;            /* number of child boxes in array above */
  /* TO BE ADDED: pointer to low level object for which the box is a BV */
  int what_is_inside;           /* POLYHEDRON, SOLID, BBs,... */
  int entity_type;        /* SPHERE, CONVEX_POLYHEDRON,...,KCD_SMALL_VOLUME_BOX */ 
  void *pol;          /* in case what_is_inside is POLYHEDRON or SOLID */
  int ext_pol_id;   /* external identifier of pol, whenever it is a POLYHEDRON or SOLID (25.02.2002) */
  int nof_facet_vertices; /* in case what_is_inside is POLYHEDRON, we have a facet... */
  int *facet_vertices;    /* ... of the *pol with these vertices */
  int interesting_seed;   /* KCD GJK: interation starts here in list facet_vertices */
  int is_robot_prim; /* only TRUE if *pol belongs to a moving object */
  int nof_bbs;                    /* in case what_is_inside is BBs */
  struct kcd_bb **array_of_bbs;    /* in case what_is_inside is BBs 
			             pointer to nof_bbs BBs */

  /* attributes for min_max filtering Pepijn */
  double min_estimated_distance;
  double max_estimated_distance;

  /* connecting them */
  struct kcd_bb *prev;
  struct kcd_bb *next;
}kcd_bb, *kcd_bb_p;


typedef struct id_bb
{
  int       p3d_poly_id;
  int       obb_type;
  int       obb_ref_num_by_type;
}id_bb;

typedef struct prim_obj_table
{
  int is_movable;
  int ext_obj_id;
  int ext_prim_id;
  int kcd_obj_id;
  int kcd_prim_bb_id;
}prim_obj_table, *prim_obj_table_p;

typedef struct obj_prim_interval_table
{
  int ext_obj_id;
  int m_or_s_id;
  int is_movable;
  int is_active;    /* TRUE if (movable) object must be taken into account for collision testing */
  int is_moved;     /* if FALSE, collision detector can re-use previous result for this movable object (flag only useful for movable objects) */
  int sml_index;
  int big_index;
}obj_prim_interval_table, *obj_prim_interval_table_p;

typedef struct poly_bb_table
{
  int p3d_poly_id;
  int bb_id;
}poly_bb_table, *poly_bb_table_p;

typedef struct link_bb_table
{
  int mo_id;
  int kcd_mo_id;
  int bb_id;
}link_bb_table, *link_bb_table_p;

typedef struct mov_obj_bb_table
{
  int ext_o_id;  /* the external obj_id of mo */
  int obj_id;  /* the obj_id of mo */
  int bb_id;   /* for each mo, the bb around it */
  int grp_id;  /* if mo belongs to group (robot), or -1 if not */
  int ith_mo;  /* mo is the ith_mo-th group element (the ith_mo-th body) */
}mov_obj_bb_table, *mov_obj_bb_table_p;

typedef struct group_bb_table
{
  int group_id;
  int hierheight;
  int bb_id;
}group_bb_table;




#endif



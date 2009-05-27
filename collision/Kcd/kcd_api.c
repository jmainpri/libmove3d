/*! \file kcd_api.c
 \brief This files contains API functions of KCD
*/
#include "Util-pkg.h"
#include "Collision-pkg.h"


/* type */

typedef struct kcd_scene_data
{
  int nof_bbs;  /* number of filled entries in array all_bbs */
  int nof_bbs_extra;  /* number of unfilled entries in array all_bbs */
  int nof_mos;  /* number of movable objects */
  int nof_mo_grps;  /* number of groups of movable objects */
  int nof_prims;    /* number of primitives in the scene */
  int nof_prims_user;    /* number of primitives in the scene communicated by user*/
  /* AABB_TREE */
  kcd_aabb_list           *aabb_tree;
  int                     *nr_aabb_hier_el;
  int                      nr_kcd_aabb_list;
  /* all_bbs */
  kcd_bb_p                *all_bbs;
  obj_prim_interval_table *kcd_o_prm_table;
  prim_obj_table          *kcd_obj_prim_table;
  poly_bb_table           *kcd_poly_bb_table;
  mov_obj_bb_table        *kcd_mo_bb_table; 
  int                     *nof_grp_elts; 
  int                     *ith_static_obj;
  link_bb_table_p         *kcd_body_bb_table;

  id_bb                   *flat_obbs;
  id_bb                   *spherical_obbs;
  id_bb                   *tall_obbs;
  int                      nof_flat_obbs;
  int                      nof_spherical_obbs;
  int                      nof_tall_obbs;

  kcd_col_handle          *handlePt;
}kcd_scene_data, *kcd_scene_data_p;

/* global variable */
kcd_scene_data_p kcd_scene_data_arr = NULL;

static int kcd_nof_prims_user; /* this is the number given by the user, size of certain tables */

static int current_scene = -1;
static int kcd_nof_envs = 0;
static int kcd_nof_mo_grps = 0;
static int kcd_nof_mos = 0;
static int kcd_nof_sos = 0;
static int ext_obj_id = -1;

static int new_env = FALSE;
static int new_so  = FALSE;
static int new_mo  = FALSE;
static unsigned int kcd_nof_prims = 0;

static int prim_sml_i;
static int prim_big_i;

static int kcd_initialized = FALSE;


/* announcement of functions defined below: 
int kcd_robot_collides_itself(int ,int );
int kcd_robot_collides_robot(int ,int , int );
int kcd_robot_collides(int , int , double *, int *);
int kcd_collision_exists(int , double *);
int kcd_mo_collides(int , int , double *, int *);
int kcd_robot_collides_mo(int , int );
int kcd_mo_collides_mo(int , int );
int kcd_robot_vs_robot(int ,int , int , double *, int *);
int kcd_robot_vs_mo(int , int , int , double *, int *);
int kcd_mo_vs_mo(int , int , int , double *, int *);
*/

/*! \fn void kcd_mo_moved(int kcd_obj_id,int is_moved)
 \brief keeps track of status of a movable object since the 
        latest call to KCD collision test
 \param kcd_obj_id     kcd_obj_id-th object (static or movable)
                       given by user in the current scene
 \param is_moved       TRUE if object moved since last call
                       to a collision test ( FALSE if not )
 \note
 sets flag to  is_moved  allowing the collision test to avoid testing unmoved movable objects 
 does nothing when kcd_obj_id-th object is a static one
*/
void kcd_mo_moved(int kcd_obj_id,int is_moved)
{
  if(kcd_o_prm_table[kcd_obj_id].is_movable)
    kcd_o_prm_table[kcd_obj_id].is_moved = is_moved;
  else
    PrintInfo(("KCD API warning: tried to move static object\n"));
}

/*! \fn int kcd_get_is_moved(int mo_id)
 \brief returns whether or not the movable object must be tested
        for collision again (whether or not it moved more 
        recently than the latest collision test)
 \param  mo_id      mo_id-th movable (!) object
 \return TRUE if mo_id-th movable object moved recently, 
         FALSE otherwise
*/
int kcd_get_is_moved(int mo_id)
{
  int val,kcd_obj_id;

  kcd_obj_id = kcd_mo_bb_table[mo_id].ext_o_id;
  val = kcd_o_prm_table[kcd_obj_id].is_moved;
  return val;
}

/*  *********************************************************** 
    *********************************************************** 
       Functions for definition of an environment
    ***********************************************************
    ***********************************************************  */

/*! \fn int kcd_is_initialized()
  \brief returns the value of kcd_initialized
*/
int kcd_is_initialized()
{
  return kcd_initialized;
}
/*! \fn int kcd_get_nof_mos()
 \brief Get the number of movable objects known by KCD
 \note  used to test all different movable objects for collision
*/
int kcd_get_nof_mos()
{
  return kcd_nof_mos;
}
/*! \fn int kcd_get_nof_grps()
 \brief Get the number of groups
*/
int kcd_get_nof_grps()
{
  return kcd_nof_mo_grps;
}
/*! \fn int kcd_get_nof_sos()
 \brief Get the number of static objects
*/
int kcd_get_nof_sos()
{
  return kcd_nof_sos;
}

static void modif_kcd_data_arr()
{
  int cur_scene = current_scene;

  if(kcd_nof_envs != 0)
    {
      kcd_scene_data_arr[cur_scene].nof_grp_elts           = nof_grp_elts;
      kcd_scene_data_arr[cur_scene].kcd_mo_bb_table        = kcd_mo_bb_table;
      kcd_scene_data_arr[cur_scene].kcd_body_bb_table      = kcd_body_bb_table;
      kcd_scene_data_arr[cur_scene].nof_grp_elts = nof_grp_elts;
    }
}

static void modif_so_kcd_data_arr()
{
  int cur_scene = current_scene;

  if(kcd_nof_envs != 0)
    {
      kcd_scene_data_arr[cur_scene].nof_prims = kcd_nof_prims_user;
      kcd_scene_data_arr[cur_scene].nof_prims_user = kcd_nof_prims_user;
      kcd_scene_data_arr[cur_scene].nof_bbs = nof_bbs;
      kcd_scene_data_arr[cur_scene].all_bbs = all_bbs;
      kcd_scene_data_arr[cur_scene].ith_static_obj = ith_static_obj;
      kcd_scene_data_arr[cur_scene].kcd_o_prm_table = kcd_o_prm_table;
      kcd_scene_data_arr[cur_scene].kcd_obj_prim_table = kcd_obj_prim_table;
      kcd_scene_data_arr[cur_scene].kcd_poly_bb_table = kcd_poly_bb_table;
    }
}

static void add_kcd_data_arr()
{
  if(kcd_nof_envs == 1)
    {
      kcd_scene_data_arr = NULL;
      kcd_scene_data_arr = MY_ALLOC(kcd_scene_data,1);
    }
  else
    {
      kcd_scene_data_arr = MY_REALLOC(kcd_scene_data_arr,kcd_scene_data,kcd_nof_envs-1,kcd_nof_envs);
    }

  kcd_scene_data_arr[kcd_nof_envs-1].nof_bbs                     = nof_bbs;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_bbs_extra               = nof_bbs_extra;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_mo_grps                 = kcd_nof_mo_grps;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_mos                     = kcd_nof_mos;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_prims                   = kcd_nof_prims;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_prims_user              = kcd_nof_prims_user;
  kcd_scene_data_arr[kcd_nof_envs-1].all_bbs                     = all_bbs;
  kcd_scene_data_arr[kcd_nof_envs-1].aabb_tree                   = AABB_TREE;
  kcd_scene_data_arr[kcd_nof_envs-1].nr_aabb_hier_el             = nr_aabb_hier_el;
  kcd_scene_data_arr[kcd_nof_envs-1].nr_kcd_aabb_list            = nr_kcd_aabb_list;
  kcd_scene_data_arr[kcd_nof_envs-1].kcd_o_prm_table             = kcd_o_prm_table;
  kcd_scene_data_arr[kcd_nof_envs-1].kcd_obj_prim_table          = kcd_obj_prim_table;
  kcd_scene_data_arr[kcd_nof_envs-1].kcd_poly_bb_table           = kcd_poly_bb_table;
  kcd_scene_data_arr[kcd_nof_envs-1].kcd_mo_bb_table             = kcd_mo_bb_table;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_grp_elts                = nof_grp_elts;
  kcd_scene_data_arr[kcd_nof_envs-1].ith_static_obj              = ith_static_obj;
  kcd_scene_data_arr[kcd_nof_envs-1].kcd_body_bb_table           = kcd_body_bb_table;
  kcd_scene_data_arr[kcd_nof_envs-1].flat_obbs                   = flat_obbs;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_flat_obbs               = nof_flat_obbs;
  kcd_scene_data_arr[kcd_nof_envs-1].spherical_obbs              = spherical_obbs;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_spherical_obbs          = nof_spherical_obbs;
  kcd_scene_data_arr[kcd_nof_envs-1].tall_obbs                   = tall_obbs;
  kcd_scene_data_arr[kcd_nof_envs-1].nof_tall_obbs               = nof_tall_obbs;
  kcd_scene_data_arr[kcd_nof_envs-1].handlePt                    = 
    kcd_copy_col_handle(kcd_get_cur_col_handle());
}
/*! \fn static void clean_kcd_col_data()
 \brief Cleans up the data of KCD, (deallocation of memory)
*/
static void clean_kcd_col_data()
{
  int n_of_bbs;

  /* clean_up what's left of old scene and shouldn't be stored: */
  /* GJK initialization values: */
  n_of_bbs = kcd_get_number_of_bbs();
  kcd_gjk_support_cleanup_interesting_seed(n_of_bbs);
  /* B Kineo Carl 22.02.2002 */
  kcd_clean_up_pairInCollision();
  /* E Kineo Carl 22.02.2002 */
  /* distance report */
  kcd_clean_up_dist_mo_report_table(kcd_nof_mos);
  kcd_clean_up_distance_report_table();
  /* collision report by robots/groups */
  kcd_clean_up_report();
  /* data on results of collision between movable objects and AABB hierarchy */
  kcd_clean_up_mov_stuff();
}

/*! \fn static void set_kcd_data_null()
 \brief Set all the kcd data null
*/
static void set_kcd_data_null()
{
  /* set global variables to initial values NULL and 0: */
  all_bbs                   = NULL;
  kcd_nof_mos               = 0;
  kcd_nof_prims             = 0;
  kcd_nof_prims_user        = 0;
  kcd_nof_mo_grps           = 0;
  nof_bbs                   = 0;
  nof_bbs_extra             = 0;
  AABB_TREE                 = NULL;
  nr_aabb_hier_el           = NULL;
  nr_kcd_aabb_list          = 0;
  kcd_o_prm_table           = NULL;
  kcd_obj_prim_table        = NULL;
  kcd_poly_bb_table         = NULL;
  kcd_mo_bb_table           = NULL;
  nof_grp_elts              = NULL;
  ith_static_obj            = NULL;
  kcd_body_bb_table         = NULL;
  flat_obbs                 = NULL;
  nof_flat_obbs             = 0;
  spherical_obbs            = NULL;
  nof_spherical_obbs        = 0;
  tall_obbs                 = NULL;
  nof_tall_obbs             = 0;

  /* re-init global vars */
  current_scene = -1;
  kcd_nof_envs = 0;
  kcd_scene_data_arr = NULL;
  kcd_nof_sos = 0;
  ext_obj_id = -1;
  
  new_env = FALSE;
  new_so  = FALSE;
  new_mo  = FALSE;
}

/*! \fn void deconnect_kcd_data()
 \brief Cleans up KCD and resets the data
 \note calls clean_kcd_col_data() and set_kcd_data_null()
*/
void deconnect_kcd_data()
{
  clean_kcd_col_data(); 
  set_kcd_data_null();
}

/*! \fn static void connect_kcd_data(int kcd_scene_id)
 \brief Global variables of KCD are set to stored values for scene with identifier kcd_scene_id 
*/
static void connect_kcd_data(int kcd_scene_id)
{
  current_scene = kcd_scene_id;

  all_bbs                   = kcd_scene_data_arr[kcd_scene_id].all_bbs;
  nof_bbs                   = kcd_scene_data_arr[kcd_scene_id].nof_bbs;
  nof_bbs_extra             = kcd_scene_data_arr[kcd_scene_id].nof_bbs_extra;
  kcd_nof_mo_grps           = kcd_scene_data_arr[kcd_scene_id].nof_mo_grps;
  kcd_nof_mos               = kcd_scene_data_arr[kcd_scene_id].nof_mos;
  kcd_nof_prims             = kcd_scene_data_arr[kcd_scene_id].nof_prims;
  kcd_nof_prims_user        = kcd_scene_data_arr[kcd_scene_id].nof_prims_user;
  AABB_TREE                 = kcd_scene_data_arr[kcd_scene_id].aabb_tree;
  nr_kcd_aabb_list          = kcd_scene_data_arr[kcd_scene_id].nr_kcd_aabb_list;
  nr_aabb_hier_el           = kcd_scene_data_arr[kcd_scene_id].nr_aabb_hier_el;
  kcd_o_prm_table           = kcd_scene_data_arr[kcd_scene_id].kcd_o_prm_table;
  kcd_obj_prim_table        = kcd_scene_data_arr[kcd_scene_id].kcd_obj_prim_table;
  kcd_poly_bb_table         = kcd_scene_data_arr[kcd_scene_id].kcd_poly_bb_table;
  kcd_mo_bb_table           = kcd_scene_data_arr[kcd_scene_id].kcd_mo_bb_table;
  nof_grp_elts              = kcd_scene_data_arr[kcd_scene_id].nof_grp_elts;
  ith_static_obj            = kcd_scene_data_arr[kcd_scene_id].ith_static_obj;
  kcd_body_bb_table         = kcd_scene_data_arr[kcd_scene_id].kcd_body_bb_table;
  flat_obbs                 = kcd_scene_data_arr[kcd_scene_id].flat_obbs;
  nof_flat_obbs             = kcd_scene_data_arr[kcd_scene_id].nof_flat_obbs;
  spherical_obbs            = kcd_scene_data_arr[kcd_scene_id].spherical_obbs;
  nof_spherical_obbs        = kcd_scene_data_arr[kcd_scene_id].nof_spherical_obbs;
  tall_obbs                 = kcd_scene_data_arr[kcd_scene_id].tall_obbs;
  nof_tall_obbs             = kcd_scene_data_arr[kcd_scene_id].nof_tall_obbs;

  /* Set the collision table handle */
  if (kcd_scene_data_arr[kcd_scene_id].handlePt != NULL) {
    kcd_init_col_table(kcd_nof_mos, kcd_nof_mo_grps);
    kcd_copy_col_handle_into(kcd_scene_data_arr[kcd_scene_id].handlePt,
			     kcd_get_cur_col_handle());
  }
}


/*! \fn static void kcd_clean_up_current_scene()
 \brief cleans data pointed to by the global variables, the current scene
        (all information that usually gets stored in kcd_scene_arr entry) 
*/
static void kcd_clean_up_current_scene()
{
  int nof_ext_objs = kcd_nof_mos + kcd_nof_sos;

  kcd_clean_up_dist(); 
  kcd_clean_up_col_table();
  kcd_clean_up_ith_static_obj(kcd_nof_sos);
  kcd_clean_up_mo_bb_table(kcd_nof_mos);
  kcd_clean_up_body_bb_table(kcd_nof_mo_grps);
  kcd_clean_up_grps(kcd_nof_mo_grps);
  kcd_clean_up_kcd_o_prm_table(nof_ext_objs);
  kcd_clean_up_prim_obj_table(kcd_nof_prims_user);
  kcd_clean_up_poly_bb_table(kcd_nof_prims_user);
  /* clean up flat_obbs, spherical_obbs, tall_obbs: */
  kcd_clean_up_kcd_type_obbs();

  /* clean up the AABB hierarchy in AABB_TREE */
  kcd_clean_up_AABB_TREE();

  /* clean up the array for boxes all_bbs */
  kcd_clean_all_bb();
}
/*! \fn static void clean_up_kcd_data_arr()
 \brief cleans up the information of all scenes
 \note deallocates the memory of kcd_scene_data_arr
*/
static void clean_up_kcd_data_arr()
{
  int i;

  /* clean up current internal collision data */
  clean_kcd_col_data(); 
  /* clean up each scene */
  for(i=0;i<kcd_nof_envs;i++)
    {
      connect_kcd_data(i);
      kcd_clean_up_col_handle(kcd_scene_data_arr[i].handlePt);
      kcd_scene_data_arr[i].handlePt = NULL;
      kcd_clean_up_current_scene();      
    }
  /* clean up kcd_scene_data_arr itself */
  MY_FREE(kcd_scene_data_arr,kcd_scene_data,kcd_nof_envs);
  /* set global variables to initial NULL and 0 values */
  set_kcd_data_null();
}

static void kcd_begin_static_object()
{
  new_so = TRUE;
}

static void kcd_begin_movable_object()
{
  new_mo = TRUE;
}

static int kcd_end_static_object()
{
  int kcd_so_id = kcd_nof_sos;
  kcd_nof_sos++;
  new_so = FALSE;
  /* keep track of at which moment the kcd_so_id-th static object was given */
  kcd_fill_ith_static_obj(kcd_so_id,ext_obj_id);

  return kcd_so_id;
}

static int kcd_end_movable_object()
{
  int kcd_mo_id = kcd_nof_mos;
  int kcd_mo_all_bb_id;

  kcd_nof_mos++;
  /* make space for bb around this movable object 
     and keep track of it in a look-up table */
  kcd_mo_all_bb_id = put_new_aabb_away(ext_obj_id,TRUE,0.0,0.0,0.0,0.0,0.0,0.0,NULL,0,-1);
  kcd_fill_kcd_mo_bb_table(ext_obj_id,kcd_mo_id,kcd_mo_all_bb_id-1,-1);

  new_mo = FALSE;
  return kcd_mo_id;
}

/* API */
/*! \fn void kcd_remember_scene(int kcd_scene_id)
\brief Select a scene with the kcd_scene_id
\param kcd_scene_id when the user defined several scenes, this function selects the
  data created by KCD for the scene for which kcd_end_scene() has returned
  the kcd_env_id identifier
\note this function does not recompute the data structures but re-uses
  the earlier initialization
*/
void kcd_remember_scene(int kcd_scene_id)
{
  int n_of_bbs, nof_mov_obj;

  if(kcd_scene_id < kcd_nof_envs)
    {
      /* Store the collision table data */
      if ((current_scene>=0) &&
	  (kcd_scene_data_arr[current_scene].handlePt != NULL)) {
	kcd_copy_col_handle_into(kcd_get_cur_col_handle(),
				 kcd_scene_data_arr[current_scene].handlePt);
      }
      /* get stored data back */
      connect_kcd_data(kcd_scene_id);

      /* re-initialize some un-stored data */
      /* prepare GJK data structure for this size of the environment */
      n_of_bbs = kcd_get_number_of_bbs();
      kcd_gjk_support_initialize_interesting_seed(n_of_bbs);
      /* prepare KCD data structure to store distance report */
      nof_mov_obj = kcd_scene_data_arr[kcd_scene_id].nof_mos;
      kcd_init_dist_mo_report_table(nof_mov_obj);

      /* initialize table to store distances between bodies and their nearest static obstacle */
      /* initialize the overlapping AABBs */
      kcd_init_movable_stuff();
      kcd_init_distance_report_table();
      kcd_init_distance_report_table_mos();
    }
}

/* API */
/*! \fn void kcd_beg_scene(int tot_nof_prims,int nof_st_obj,int nof_mo_grps,int nof_mov_obj)
 \brief Initializes some internal KCD data structures
 \param tot_nof_prims the number of primitives in the scene (polyhedrons + Solids)
 \param nof_st_obj the number of static objects in data structure of the user
 \param nof_mo_grps the number of groups of movable objects the user will define
 \param nof_mov_obj the number of movable objects in data structure of the user
 \note will be followed by a non-zero number of sequences kcd_beg_obj(), kcd_add_prim(), kcd_end_obj()
*/
void kcd_beg_scene(int tot_nof_prims,int nof_st_obj,int nof_mo_grps,int nof_mov_obj)
{
  /* int kcd_scene_id = kcd_nof_envs; */
  int tot_nof_obj;

  /* tell the user */
  PrintInfo(("started initializing KCD\n"));
  PrintInfo(("KCD: reading objects...\n"));

  if (current_scene>=0) {
    /* Store the collision table data */
    if (kcd_scene_data_arr[current_scene].handlePt != NULL) {
      kcd_copy_col_handle_into(kcd_get_cur_col_handle(),
			       kcd_scene_data_arr[current_scene].handlePt);
    }
  }
  current_scene = kcd_nof_envs;
  new_env = TRUE;
  tot_nof_obj = nof_st_obj + nof_mov_obj;
  kcd_nof_prims_user = tot_nof_prims;

  /* data related to collision detection and report */
  kcd_init_dist_mo_report_table(nof_mov_obj);

  /* B Kineo Carl 22.02.2002 */
  /* initialize table to store polyh. and solid found to be in collision for each mo */
  kcd_init_pairInCollision();
  /* E Kineo Carl 22.02.2002 */

  /* initialize data for new scene */  
  kcd_init_kcd_o_prm_table(tot_nof_obj);
  kcd_init_prim_obj_table(tot_nof_prims);
  kcd_init_kcd_poly_bb_table(tot_nof_prims);
  kcd_init_mo_bb_table(nof_mov_obj);
  kcd_init_body_bb_table(nof_mo_grps);
  kcd_init_grps(nof_mo_grps);
  kcd_init_col_table(nof_mov_obj, nof_mo_grps);
  kcd_init_ith_static_obj(nof_st_obj);
}

/* API */
/*! \fn int kcd_end_scene()
 \brief builds remaining internal hierarchies necessary for KCD
 \note Builds AABB hierarchy, and constructs OBB-trees
 \return kcd_scene_id an identifier of the scene
*/
int kcd_end_scene()
{
  int n_of_bbs,kcd_scene_id = kcd_nof_envs;

  /* make access to kcd_bb pile */
  make_hash_table_on_bbs(); 
  
  /* tell the user */
  PrintInfo(("KCD: reading objects done \n"));
  PrintInfo(("KCD: building OBB-tree on movable objects...\n"));
  /* put OBB-tree on top of OBB-roots in each of the robot bodies */
  kcd_obb_construct_on_links();
  /* tell the user */
  PrintInfo(("KCD: building OBB-tree on movable objects done\n"));

  /* tell the user */
  PrintInfo(("KCD: building internal hierarchies... \n"));
  /* put aabb around the primitives, construct the aabb_hierarchy,
     put obb-tree on static objects determined by the aabb_hierarchy */
  kcd_aabb_construct();
  /* tell the user */
  PrintInfo(("KCD: building internal hierarchies done\n"));

  /* prepare GJK data structure for this size of the environment */
  n_of_bbs = kcd_get_number_of_bbs();
  kcd_gjk_support_initialize_interesting_seed(n_of_bbs);

  /* initialize table to store distances between bodies and their nearest static obstacle */
  /* put a box around devices, freights, movable, and 
     deformable objects and initialize the overlapping AABBs */
  kcd_init_movable_stuff();
  kcd_init_distance_report_table();
  /* pepijn july 2001
   * initialisation of the report table
   */
  kcd_init_distance_report_table_mos();
  /* initialisation of internal structures to calculate exact distances */
  kcd_init_distance_exact();
  /* environment constructed, hence count */
  kcd_nof_envs++;
  new_env = FALSE;

  /* store internal KCD data of new scene in the array of data of all scenes */
  add_kcd_data_arr();

  kcd_initialized = TRUE;
  /* return kcd environment identifier */
  return kcd_scene_id;
}

/* API */
/*! \fn void kcd_clean_up()
\brief free-s all memory of the internal data structures used by KCD
*/
void kcd_clean_up()
{
  int n_of_bbs;

  PrintInfo(("cleaning up KCD\n"));

  n_of_bbs = kcd_get_number_of_bbs();

  /* cleans all global variables stored for each of the scenes */
  clean_up_kcd_data_arr();
  PrintInfo(("cleaned\n"));
}



/* API */
/*! \fn void kcd_beg_obj(int can_move)
\brief adding an object
\param can_move if can_move is FALSE, the object will be considered
   as a static obstacle.
\note will be followed by a number of calls to kcd_add_prim()
*/
void kcd_beg_obj(int can_move)
{
  prim_sml_i = kcd_nof_prims;
  ext_obj_id++;
  if(can_move)
    {
      kcd_begin_movable_object();
    }
  else
    {
      kcd_begin_static_object();
    }
}

/* API */
/*! \fn int kcd_end_obj()
\brief creates OBB-tree on all single OBBs and on all roots 
   of OBB-trees of the primitives and polyhedrons in the object 
\return returns object identifier: unique number for each object (static or movable)
*/
int kcd_end_obj()
{
  int obj_id, is_movable;

  prim_big_i = kcd_nof_prims - 1;

  /* if object can move, return the id of the root obb */
  if(new_so) 
    {
      is_movable = FALSE;
      obj_id = kcd_end_static_object();
    }
  else if(new_mo) 
    {
      is_movable = TRUE;
      obj_id = kcd_end_movable_object();
    }
  else 
    {
      is_movable = -1;
      obj_id = -1;
      PrintInfo(("KCD error: wrong type of object closed\n"));
    }

  kcd_fill_kcd_o_prm_table(ext_obj_id,obj_id,is_movable,prim_sml_i,prim_big_i);

  return ext_obj_id; /* the ext_obj_id-th object (movable and static) given by the user */
}

/* API */
/*! \fn int kcd_add_prim(void *primPt)
\brief creates OBB's on the primitives

- creates OBB on sphere, cylinder, box, cube, cone, 
   convex_facet, convex_polyhedron;
- creates OBB-tree on all OBBs of the convex facets defined
   in a polyhedron;
- places the OBB at pos0
- stores which solids and/or polyhedrons belong to currently defined object

\return primitive_id
*/
int kcd_add_prim(void *primPt)
{
  int is_movable,obj_id,ext_p_id,kcd_p_id=0,kcd_obj_id=-1;

  if(new_mo)
    {
      obj_id = kcd_nof_mos;
      is_movable = TRUE;
    }
  else
    {
      obj_id = kcd_nof_sos;
      is_movable = FALSE;
    }
  kcd_p_id = kcd_put_obb_around_polyh(primPt,kcd_nof_prims,is_movable,ext_obj_id);
  ext_p_id = kcd_nof_prims;
  kcd_fill_prim_obj_table(ext_obj_id,ext_p_id,kcd_obj_id,kcd_p_id,is_movable);
  kcd_fill_kcd_poly_bb_table(ext_p_id,kcd_p_id);
  kcd_nof_prims++;
  return kcd_p_id;
}


/* API */
/*! \fn int kcd_def_mo_grp(int *arr_kcd_ids, int nof_mo_ids)
 \brief  makes group of several given movable objects
 \param arr_mo_ids     an array of kcd-ids of objects
 \param  nof_mo_ids     number of entries in array arr_mo_ids
 \return a group-id (e.g. can be seen as a robot-id)
 \warning movable objects previously defined and not in other group
*/
int kcd_def_mo_grp(int *arr_kcd_ids, int nof_mo_ids)
{
  int grp_id = kcd_nof_mo_grps;

  kcd_init_body_bb_table_entry(grp_id,nof_mo_ids);
  kcd_fill_body_bb_table_entry(grp_id,arr_kcd_ids,nof_mo_ids);
  kcd_fill_grps(grp_id, nof_mo_ids);

  kcd_nof_mo_grps++;
  modif_kcd_data_arr();
  return grp_id;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief  makes a group (-1) with movable object without groups
 */
void kcd_def_mo_no_grp(void)
{
  int nof_mo_ids;

  nof_mo_ids = kcd_fill_no_body_bb_table_entry();
  kcd_fill_grps(-1, nof_mo_ids);

  modif_kcd_data_arr();
}


/*  **************************************************************** 
    **************************************************************** 
    Functions: collision pairs
    **************************************************************** 
    **************************************************************** */


/* API */
/*! \fn void kcd_act_mo(int mo_id)
\brief the given movable object is considered by the collision test 
*/
void kcd_act_mo(int mo_id)
/* - the given movable object is considered by the collision test */
{
  kcd_o_prm_table[kcd_mo_bb_table[mo_id].ext_o_id].is_active = TRUE;
}

/* API */
/*! \fn void kcd_deact_mo(int mo_id)
\brief the given movable object is ignored by the collision test 
*/
void kcd_deact_mo(int mo_id)
/* - the given movable object is ignored by the collision test */
{
  kcd_o_prm_table[kcd_mo_bb_table[mo_id].ext_o_id].is_active = FALSE;
}

/* API */
/*! \fn void kcd_act_grp(int grp_id)
\brief the group of links of the given robot is considered by the collision test 
\warning  grp_id < total number of groups 
*/
void kcd_act_grp(int grp_id)
/* - the group of links of the given robot is considered  */
/*   by the collision test */
/* PRECOND:  grp_id < total number of groups */
{
  grp_is_active[grp_id] = TRUE;
}

/*! \fn void kcd_deact_grp(int grp_id)
\brief the group of links of the given robot is ignored by the collision test 
\warning  grp_id < total number of groups 
*/
/* API */
void kcd_deact_grp(int grp_id)
/* - the group of links of the given robot is ignored by the collision test */
/* PRECOND:  grp_id < total number of groups */
{
  grp_is_active[grp_id] = FALSE;
}


/* B Kineo Carl 27.02.2002 */
/* API */
void kcd_act_obst(int ext_obj_id)
{
  /* (re)activate given static object
     (by default object is activated) */
  kcd_set_active_kcd_o_prm_table(ext_obj_id, TRUE); 
}

/* API */
void kcd_deact_obst(int ext_obj_id)
{
  /* deactivate given static object
     (by default object is activated) */
  kcd_set_active_kcd_o_prm_table(ext_obj_id, FALSE); 
}

int kcd_obst_is_act(int ext_obj_id)
{
  /* - returns TRUE if given static object is activated,  */
  /*   returns FALSE otherwise */
  return kcd_get_active_kcd_o_prm_table(ext_obj_id);
}
/* E Kineo Carl 27.02.2002 */


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the test of collision and the distance computation
 *        between two movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id1:    The index of the first movable object.
 * \param  kcd_id2:    The index of the second movable object.
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it activate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_act_mo_pair(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1, mo_id2;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  kcd_fill_act_mo_pair_arr(handlePt, mo_id1, mo_id2, KCD_TABLE_COL_DIST_ACT);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the test of collision and the distance computation
 *        between two movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id1:    The index of the first movable object.
 * \param  kcd_id2:    The index of the second movable object.
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it deactivate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_mo_pair(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1, mo_id2;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  kcd_fill_act_mo_pair_arr(handlePt, mo_id1, mo_id2, KCD_TABLE_COL_DIST_DEACT);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the test of collision between two movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id1:    The index of the first movable object.
 * \param  kcd_id2:    The index of the second movable object.
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it activate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_act_col_mo_pair(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1, mo_id2;
  mo_act_type act_val;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  act_val = kcd_get_mo_col_pair_is_act(handlePt, mo_id1, mo_id2);
  act_val |= KCD_TABLE_COL_ACT;
  kcd_fill_act_mo_pair_arr(handlePt, mo_id1, mo_id2, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the test of collision between two movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id1:    The index of the first movable object.
 * \param  kcd_id2:    The index of the second movable object.
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it deactivate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_col_mo_pair(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1, mo_id2;
  mo_act_type act_val;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  act_val = kcd_get_mo_col_pair_is_act(handlePt, mo_id1, mo_id2);
  act_val &= ~KCD_TABLE_COL_ACT;
  kcd_fill_act_mo_pair_arr(handlePt, mo_id1, mo_id2, act_val);
}



/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the distance computation between two movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id1:    The index of the first movable object.
 * \param  kcd_id2:    The index of the second movable object.
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it activate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_act_dist_mo_pair(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1, mo_id2;
  mo_act_type act_val;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  act_val = kcd_get_mo_col_pair_is_act(handlePt, mo_id1, mo_id2);
  act_val |= KCD_TABLE_DIST_ACT;
  kcd_fill_act_mo_pair_arr(handlePt, mo_id1, mo_id2, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the distance computation between two movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id1:    The index of the first movable object.
 * \param  kcd_id2:    The index of the second movable object.
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it deactivate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_dist_mo_pair(kcd_col_handle * handlePt, int kcd_id1,int kcd_id2)
{
  int mo_id1, mo_id2;
  mo_act_type act_val;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  act_val = kcd_get_mo_col_pair_is_act(handlePt, mo_id1, mo_id2);
  act_val &= ~KCD_TABLE_DIST_ACT;
  kcd_fill_act_mo_pair_arr(handlePt, mo_id1, mo_id2, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the test of collision and the distance computation
 *        between a movable object and the environment..
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  kcd_id:    The index of the movable object.
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it activate the collision between this group and the environment.
 * \note In other case it doesn't change the activation of groups.
 */
void kcd_act_mo_env(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  kcd_fill_act_mo_env_arr(handlePt, mo_id, KCD_TABLE_COL_DIST_ACT);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the test of collision and the distance computation
 *        between a movable object and the environment..
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  kcd_id:    The index of the movable object.
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it deactivate the collision between this group
 *       and the environment.
 * \note In other case it doesn't change the activation of groups.
 */
void kcd_deact_mo_env(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  kcd_fill_act_mo_env_arr(handlePt, mo_id, KCD_TABLE_COL_DIST_DEACT);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the test of collision between a movable object
 *        and the environment..
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  kcd_id:    The index of the movable object.
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it activate the collision between this group and the environment.
 * \note In other case it doesn't change the activation of groups.
 */
void kcd_act_col_mo_env(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;
  mo_act_type act_val;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  act_val = kcd_get_mo_col_env_is_act(handlePt, mo_id);
  act_val |= KCD_TABLE_COL_ACT;
  kcd_fill_act_mo_env_arr(handlePt, mo_id, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the test of collision between a movable object
 *        and the environment..
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  kcd_id:    The index of the movable object.
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it deactivate the collision between this group
 *       and the environment.
 * \note In other case it doesn't change the activation of groups.
 */
void kcd_deact_col_mo_env(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;
  mo_act_type act_val;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  act_val = kcd_get_mo_col_env_is_act(handlePt, mo_id);
  act_val &= ~(KCD_TABLE_COL_ACT);
  kcd_fill_act_mo_env_arr(handlePt, mo_id, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the distance computation between a movable object
 *        and the environment..
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  kcd_id:    The index of the movable object.
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it activate the collision between this group and the environment.
 * \note In other case it doesn't change the activation of groups.
 */
void kcd_act_dist_mo_env(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;
  mo_act_type act_val;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  act_val = kcd_get_mo_col_env_is_act(handlePt, mo_id);
  act_val |= KCD_TABLE_DIST_ACT;
  kcd_fill_act_mo_env_arr(handlePt, mo_id, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the distance computation between a movable object
 *        and the environment..
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  kcd_id:    The index of the movable object.
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it deactivate the collision between this group
 *       and the environment.
 * \note In other case it doesn't change the activation of groups.
 */
void kcd_deact_dist_mo_env(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;
  mo_act_type act_val;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  act_val = kcd_get_mo_col_env_is_act(handlePt, mo_id);
  act_val &= ~(KCD_TABLE_DIST_ACT);
  kcd_fill_act_mo_env_arr(handlePt, mo_id, act_val);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the test of collision between two groups of movable object.
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  grp_id1:   The index of the first movable objects group.
 * \param  grp_id2:   The index of the second movable objects group.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by kcd_act_mo_pair() or
 *       kcd_deact_mo_pair().
 * \note If (\a grp_id1 == \a grp_id2) then this active the autocollision.
 * \note grp_id1 == -1 activate the set of movable objects without group.
 */
void kcd_act_grp_pair(kcd_col_handle * handlePt, int grp_id1, int grp_id2)
{
  kcd_fill_act_grp_pair_arr(handlePt, grp_id1, grp_id2, TRUE);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the test of collision between two groups
 *        of movable object.
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  grp_id1:   The index of the first movable objects group.
 * \param  grp_id2:   The index of the second movable objects group.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by kcd_act_mo_pair() or
 *       kcd_deact_mo_pair(). Then a call to kcd_act_grp_pair()
 *       restore all those links.
 * \note if (\a grp_id1 == \a grp_id2) then this deactive the autocollision.
 * \note grp_id1 == -1 deactivate the set of movable objects without group.
 */
void kcd_deact_grp_pair(kcd_col_handle * handlePt, int grp_id1, int grp_id2)
{
  kcd_fill_act_grp_pair_arr(handlePt, grp_id1, grp_id2, FALSE);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Activate the test of collision between a groups of movable object
 *        and the environment.
 *
 * \param handlePt:   The kcd collision handle.
 * \param grp_id: The index of the movable objects group.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by kcd_act_mo_env() or
 *       kcd_deact_mo_env().
 * \note grp_id == -1 activate the set of movable objects without group.
 */
void kcd_act_grp_env(kcd_col_handle * handlePt, int grp_id)
{
  kcd_fill_act_grp_env_arr(handlePt, grp_id, TRUE);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate the test of collision between a group
 *        of movable object and the environment.
 *
 * \param handlePt:   The kcd collision handle.
 * \param grp_id:     The index of the movable objects group.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by kcd_act_mo_env() or
 *       kcd_deact_mo_env(). Then a call to kcd_act_grp_env()
 *       restore all those links.
 * \note grp_id == -1 deactivate the set of movable objects without group.
 */
void kcd_deact_grp_env(kcd_col_handle * handlePt, int grp_id)
{
  kcd_fill_act_grp_env_arr(handlePt, grp_id, FALSE);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactivate all the test of collision.
 *
 * \param handlePt:   The kcd collision handle.
 */
void kcd_deactivate_all_mo(kcd_col_handle * handlePt)
{
  kcd_deactivate_all_col_handle(handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if given movable object is activated. 
 *
 * \return Returns TRUE if given movable object is activated,  
 *         FALSE otherwise 
 */
int kcd_mo_is_act(int mo_id)
{
  return kcd_o_prm_table[kcd_mo_bb_table[mo_id].ext_o_id].is_active;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if given group of movable object is activated. 
 *
 * \return Returns TRUE if given group is activated,  
 *         FALSE otherwise 
 *
 * \note grp_id == -1 represents all the movable objects that are not
 *       in any group.
 *
 * \warning  grp_id < total number of groups 
 */
int kcd_grp_is_act(int grp_id)
{
  return grp_is_active[grp_id];
}

/*! \fn int kcd_mo_belongs_to_group(int mobj_id)
\brief see id a mo belongs to a group
 \param mobj_id id of the movable object
 \return TRUE if movable object belongs to a group 
         FALSE otherwise
*/
int kcd_mo_belongs_to_group(int mobj_id)
{
  if(kcd_mo_bb_table[mobj_id].grp_id == -1)
    return FALSE;
  else
    return TRUE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a test of collision or a distance computation
 *        between two movable objects.
 *
 * \param handlePt:   The kcd collision handle.
 * \param kcd_id1:    The index of the first movable object.
 * \param kcd_id2:    The index of the second movable object.
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between their groups are
 *          deactivate and so even if no collision test is done.
 */
int kcd_mo_pair_is_act(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1,mo_id2;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  if (kcd_get_mo_col_pair_is_act(handlePt, mo_id1, mo_id2))
    { return TRUE; }
  return FALSE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a test of collision between two movable objects.
 *
 * \param handlePt:   The kcd collision handle.
 * \param kcd_id1:    The index of the first movable object.
 * \param kcd_id2:    The index of the second movable object.
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between their groups are
 *          deactivate and so even if no collision test is done.
 */
int kcd_col_mo_pair_is_act(kcd_col_handle * handlePt, int kcd_id1, int kcd_id2)
{
  int mo_id1,mo_id2;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  if (kcd_get_mo_col_pair_is_act(handlePt, mo_id1, mo_id2) & KCD_TABLE_COL_ACT)
    { return TRUE; }
  return FALSE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a distance computation 
 *        between two movable objects.
 *
 * \param handlePt:   The kcd collision handle.
 * \param kcd_id1:    The index of the first movable object.
 * \param kcd_id2:    The index of the second movable object.
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between their groups are
 *          deactivate and so even if no collision test is done.
 */
int kcd_dist_mo_pair_is_act(kcd_col_handle * handlePt, int kcd_id1,int kcd_id2)
{
  int mo_id1,mo_id2;

  mo_id1 = kcd_o_prm_table[kcd_id1].m_or_s_id;
  mo_id2 = kcd_o_prm_table[kcd_id2].m_or_s_id;
  if (kcd_get_mo_col_pair_is_act(handlePt,mo_id1,mo_id2) & KCD_TABLE_DIST_ACT)
    { return TRUE; }
  return FALSE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a test of collision or a distance computation
 *        between a movable object and the environment..
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id: The index of the movable object.
 *
 * \return TRUE if given pair of movable object and environment exists and
 *         is activated, FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between its group and the
 *          environment is deactivate and so even if no collision test is done.
 */
int kcd_mo_env_is_act(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  if (kcd_get_mo_col_env_is_act(handlePt, mo_id))
    { return TRUE; }
  return FALSE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a test of collision between a movable object
 *        and the environment..
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id: The index of the movable object.
 *
 * \return TRUE if given pair of movable object and environment exists and
 *         is activated, FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between its group and the
 *          environment is deactivate and so even if no collision test is done.
 */
int kcd_col_mo_env_is_act(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  if (kcd_get_mo_col_env_is_act(handlePt, mo_id) & KCD_TABLE_COL_ACT)
    { return TRUE; }
  return FALSE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a distance computation
 *        between a movable object and the environment..
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  kcd_id: The index of the movable object.
 *
 * \return TRUE if given pair of movable object and environment exists and
 *         is activated, FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between its group and the
 *          environment is deactivate and so even if no collision test is done.
 */
int kcd_dist_mo_env_is_act(kcd_col_handle * handlePt, int kcd_id)
{
  int mo_id;

  mo_id = kcd_o_prm_table[kcd_id].m_or_s_id;
  if (kcd_get_mo_col_env_is_act(handlePt, mo_id) & KCD_TABLE_DIST_ACT)
    { return TRUE; }
  return FALSE;
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a test of collision between two groups
 *        of movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param grp_id1: The index of the first movable objects group.
 * \param grp_id2: The index of the second movable objects group.
 *
 * \return TRUE if given pair of group of movable objects exists and is
 *         activated, FALSE otherwise 
 *
 * \note grp_id1 == -1 return the activation of the set of movable
 *       objects without group.
 */
int kcd_grp_pair_is_act(kcd_col_handle * handlePt, int grp_id1,int grp_id2)
{
  return kcd_get_grp_col_pair_is_act(handlePt, grp_id1, grp_id2);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Returns if there is a test of collision between a group
 *        of movable objects and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 * \param  grp_id:   The index of the movable objects group.
 *
 * \return TRUE if a group of movable objects is
 *         activated with the environment, FALSE otherwise 
 *
 * \note grp_id1 == -1 return the activation of the set of movable
 *       objects without group.
 */
int kcd_grp_env_is_act(kcd_col_handle * handlePt, int grp_id)
{
  return kcd_get_grp_col_env_is_act(handlePt, grp_id);
}


/* *********************************************************************** */
/* Functions that manage the Kcd collision handle                          */
/* *********************************************************************** */


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Create and initialize a kcd collision handle.
 *
 * Note: this function deactivate all tesst of collisions.
 *
 * \param  nof_mos: The number of movable object.
 *
 * \return The kcd collision handle. NULL if there isn't enough memory.
 */
kcd_col_handle * kcd_create_handle(void)
{
  int nof_mos  = kcd_get_nof_mos();
  int nof_grps = kcd_get_nof_grps();

  return kcd_create_col_handle(nof_mos, nof_grps);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Copy a kcd collision handle to a new structure.
 *
 * \param  handlePt: The kcd collision handle that must be copied.
 *
 * \return The kcd collision handle. NULL if there isn't enough memory.
 */
kcd_col_handle * kcd_copy_handle(kcd_col_handle * handlePt)
{
  return kcd_copy_col_handle(handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Copy a kcd collision handle into an other structure.
 *
 * \warning The number of movable object must be the same in both handle.
 *
 * \param src_handlePt:   The kcd collision handle that must be copied.
 * \param dest_handlePt:  The kcd collision handle that must store the copy.
 */
void kcd_copy_handle_into(kcd_col_handle * src_handlePt,
			  kcd_col_handle * dest_handlePt)
{
  kcd_copy_col_handle_into(src_handlePt, dest_handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Merge two kcd collision handle (sum of collision tests)
 *
 * Note: It is an "or" opperation between all flags.
 *
 * \warning The number of movable object must be the same in both handle.
 *
 * \param src_handlePt:   The source kcd collision handle.
 * \param dest_handlePt:  The kcd collision handle that must store the sum.
 */
void kcd_add_handle_into(kcd_col_handle * src_handlePt,
			 kcd_col_handle * dest_handlePt)
{
  kcd_add_col_handle_into(src_handlePt, dest_handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Merge two kcd collision handle (subtraction of collision tests)
 *
 * Note: It is an "dest and not src" opperation between all flags.
 *
 * \warning The number of movable object must be the same in both handle.
 *
 * \param src_handlePt:   The source kcd collision handle.
 * \param dest_handlePt:  The kcd collision handle that must store the sum.
 */
void kcd_sub_handle_into(kcd_col_handle * src_handlePt,
			 kcd_col_handle * dest_handlePt)
{
  kcd_sub_col_handle_into(src_handlePt, dest_handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Deactive all collision test between movable object 
 *        and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 */
void kcd_deactivate_all_handle(kcd_col_handle * handlePt)
{
  kcd_deactivate_all_col_handle(handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Release the memory of the kcd collisions handle.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \note This function cannot destroy the default collision handle.
 */
void kcd_destroy_handle(kcd_col_handle * handlePt)
{
  kcd_clean_up_col_handle(handlePt);
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Get the current collision handle.
 *
 * \return  The current kcd collision handle.
 */
kcd_col_handle * kcd_get_cur_handle(void)
{
  return kcd_get_cur_col_handle();
}


/*--------------------------------------------------------------------------
 * API */
/*!
 * \brief Put a new collision handle as the current collision handle.
 *
 * \param  handlePt: The kcd collision handle.
 */
void kcd_set_cur_handle(kcd_col_handle * handlePt)
{
  kcd_set_cur_col_handle(handlePt);
}



/* ******************************************************************************* */
/*! \fn static void update_kcd_bb_aabb(int kcd_ext_o,int bb_id)
 \brief looks for new placement matrix for given movable object, 
        computes new AABB on the movable object
	stores the resulting AABB in the entry of the mov.obj. in the array all_bbs[]
 \param   kcd_ext_o        id of mov.obj.
 \param   bb_id            index in all_bbs of the mov.obj.'s BV
 \warning   input is a movable (!) object
 \note  update_kcd_bb_aabb() isn't called from outside, only by 
        function kcd_set_aabb_on_mo() below
*/
/* ******************************************************************************* */
static void update_kcd_bb_aabb(int kcd_ext_o,int bb_id)
{
  void *obj_ptr;
  int ext_p_id,sml_i,big_i,kid_bb_id;
  double mo_x1=0.0,mo_x2=0.0,mo_y1=0.0,mo_y2=0.0,mo_z1=0.0,mo_z2=0.0;
  int initialization = FALSE;

  /* visit all primitives of given mov.obj. */  
  sml_i = kcd_o_prm_table[kcd_ext_o].sml_index;      /* ext_p_id ! */
  big_i = kcd_o_prm_table[kcd_ext_o].big_index;      /* ext_p_id ! */
  for(ext_p_id=sml_i;ext_p_id<=big_i;ext_p_id++)
    {
      kid_bb_id = kcd_poly_bb_table[ext_p_id].bb_id;
      obj_ptr = all_bbs[kid_bb_id]->pol;
      /* re-compute the AABB of each primitive */ 
      kcd_put_aabb_around_polyh(obj_ptr,kid_bb_id,TRUE,initialization);
      /* keep track of extreme values */
      if(ext_p_id==sml_i)
	{
	  mo_x1 = all_bbs[kid_bb_id]->x1;
	  mo_x2 = all_bbs[kid_bb_id]->x2;
	  mo_y1 = all_bbs[kid_bb_id]->y1;
	  mo_y2 = all_bbs[kid_bb_id]->y2;
	  mo_z1 = all_bbs[kid_bb_id]->z1;
	  mo_z2 = all_bbs[kid_bb_id]->z2;
	}
      else
	{
	  if(mo_x1 > all_bbs[kid_bb_id]->x1)
	    mo_x1 = all_bbs[kid_bb_id]->x1;
	  if(mo_x2 < all_bbs[kid_bb_id]->x2)
	    mo_x2 = all_bbs[kid_bb_id]->x2;
	  if(mo_y1 > all_bbs[kid_bb_id]->y1)
	    mo_y1 = all_bbs[kid_bb_id]->y1;
	  if(mo_y2 < all_bbs[kid_bb_id]->y2)
	    mo_y2 = all_bbs[kid_bb_id]->y2;
	  if(mo_z1 > all_bbs[kid_bb_id]->z1)
	    mo_z1 = all_bbs[kid_bb_id]->z1;
	  if(mo_z2 < all_bbs[kid_bb_id]->z2)
	    mo_z2 = all_bbs[kid_bb_id]->z2;
	}
    }
  /* set AABB of mov.obj. with extreme values */
  all_bbs[bb_id]->x1 = mo_x1;
  all_bbs[bb_id]->x2 = mo_x2;
  all_bbs[bb_id]->y1 = mo_y1;
  all_bbs[bb_id]->y2 = mo_y2;
  all_bbs[bb_id]->z1 = mo_z1;
  all_bbs[bb_id]->z2 = mo_z2;

}

/* API */
/*! \fn void kcd_set_aabb_on_mo(int kcd_ext_o)
\brief user calls this function when he/she wants KCD to compute the 
   AABB around (moved) movable object given by kcd_ext_o 
*/
void kcd_set_aabb_on_mo(int kcd_ext_o)
{
  int bb_id,mobj_id;

  if(kcd_o_prm_table[kcd_ext_o].is_movable)
    {
      mobj_id = kcd_o_prm_table[kcd_ext_o].m_or_s_id;
      bb_id = kcd_mo_bb_table[mobj_id].bb_id;
      update_kcd_bb_aabb(kcd_ext_o,bb_id);
    }
}

/*  *********************************************************** 
    *********************************************************** 
      Functions calling for collision detection:
        kcd_robot_collides_itself()
        kcd_robot_collides_robot()
        kcd_robot_collides()
        kcd_collision_exists()
        kcd_mo_collides()
        kcd_robot_collides_mo()
        kcd_mo_collides_mo()
    ***********************************************************
    *********************************************************** */


/* API */
/* ******************************************************************************* */
/*! \fn int kcd_robot_collides_itself(int robot_number,int with_report,
			      double *min_dist, int *near_obs)
 \brief Group of movable objects tests for autocollision

 - returns TRUE if given robot is self-colliding   

 - the with_report parameter determines the report made
 - verifies only those pairs of group elements for which a collision pair was defined
*/
/* ******************************************************************************* */
int kcd_robot_collides_itself(int robot_number,int with_report,
			      double *min_dist, int *near_obs)
{
  int collides = FALSE;
  int nof_links, link_it1, link_it2;
  int bodc,bod;
  
  if(!(kcd_grp_is_active(robot_number)) || 
     !(kcd_get_grp_col_pair_cur_is_act(robot_number, robot_number)))
    {
      /* group of movable objects is deactivated, must not be
         considered for collision testing => function returns FALSE */
      return FALSE;
    }
  
  /* LIST: begin */
  nof_links = kcd_get_nof_grp_elts(robot_number);
  
  for(link_it1=0; (link_it1 < nof_links-1) &&
	((!collides) || ((robot_number<0) && (with_report))); link_it1++) {
    bodc = kcd_get_body_bb_table(robot_number, link_it1).kcd_mo_id;
    
    for(link_it2=link_it1+1;(link_it2 < nof_links)&&
	  ((!collides) || ((robot_number<0) && (with_report))); link_it2++) {
      bod = kcd_get_body_bb_table(robot_number, link_it2).kcd_mo_id;
      if (kcd_get_mo_col_pair_cur_is_act(bodc, bod)) {
	switch(with_report) {
	case DISTANCE_EXACT:
	case DISTANCE_ESTIMATE:
	  collides = kcd_mo_vs_mo(bodc,bod,with_report,min_dist,near_obs); 
	  break;
	default:
	  collides = kcd_mo_collides_mo(bodc,bod);
	}
      }
    }
  }
	
  if((with_report != NO_REPORT) && (robot_number>=0)) {
    if(collides) {
      /* we report self collision */
      kcd_add_report(robot_number,-1);
      return TRUE;
    }
  }

  return collides;

}

/* API */
/* ******************************************************************************* */
/*! \fn int kcd_robot_collides_robot(int robot_number1,int robot_number2, int with_report)
\brief returns TRUE if given groups of movable objects collide,
\note does not test self-collision, 
      verifies only those pairs of group elements for which a collision pair was defined 
*/
 /* ******************************************************************************* */
int kcd_robot_collides_robot(int robot_number1,int robot_number2, int with_report)
{
  /* kcd_robot_vs_robot(robot_number1,robot_number2,with_report,NULL,NULL); */
  int collides = FALSE;
  int nof_links1,link_it1,nof_links2,link_it2;
  int bodc,bod;
  
  if( (!(kcd_grp_is_active(robot_number1)) )  || 
      (!(kcd_grp_is_active(robot_number2)) )    )
    {
      /* at least one of the group of movable objects is deactivated, must
	 not be considered for collision testing => function returns FALSE */
      return FALSE;
    }

  if( !(kcd_get_grp_col_pair_cur_is_act(robot_number1,robot_number2) ) )
    {
      /* pair of grps of mo's is deactivated, must
	 not be considered for collision testing => function returns FALSE */
      return FALSE;
    }

  /* LIST: begin */
  nof_links1 = kcd_get_nof_grp_elts(robot_number1);
  nof_links2 = kcd_get_nof_grp_elts(robot_number2);

  for(link_it1=nof_links1-1; (link_it1 >= 0) &&
	((!collides) || ((robot_number1<0) && (with_report))); link_it1--) {
    bodc = kcd_get_body_bb_table(robot_number1, link_it1).kcd_mo_id ;

    for(link_it2=nof_links2-1; (link_it2 >= 0) &&
	  ((!collides) || ((robot_number2<0) && (with_report))); link_it2--) {
      bod  = kcd_get_body_bb_table(robot_number2, link_it2).kcd_mo_id ;

      if(kcd_get_mo_col_pair_cur_is_act(bodc,bod)) {
	/* this corresponds to an activated collision pair */
	collides = kcd_mo_collides_mo(bodc,bod);
      }
    } 
  }
  /* LIST: end */

  if(with_report) {
    if(collides) {
      /* we report collision */
      if (robot_number1>=0)
	{ kcd_add_report(robot_number1,-1); }
      if (robot_number2>=0)
	{ kcd_add_report(robot_number2,-1); }
      return TRUE;
    }
  }  
  return collides;
}

int kcd_robot_vs_robot(int robot_number1, int robot_number2, int with_report,
		       double *min_dist_estimate, int *nearest_obstacle)
{
  int collides = FALSE;
  int nof_links1,link_it1,nof_links2,link_it2;
  int bodc, bod;
  double a_dist_estimate;
  int a_near_mo;

  if( (!(kcd_grp_is_active(robot_number1)) )  || 
      (!(kcd_grp_is_active(robot_number2)) )    )
    {
      /* at least one of the group of movable objects is deactivated, must
	 not be considered for collision testing => function returns FALSE */
      return FALSE;
    }

  if( !(kcd_get_grp_col_pair_cur_is_act(robot_number1,robot_number2) ) )
    {
      /* pair of grps of mo's is deactivated, must
	 not be considered for collision testing => function returns FALSE */
      return FALSE;
    }

  /* LIST: begin */
  nof_links1 = kcd_get_nof_grp_elts(robot_number1);
  nof_links2 = kcd_get_nof_grp_elts(robot_number2);


  for(link_it1=nof_links1-1; (link_it1 >= 0) &&
	((!collides) || ((robot_number1<0) && (with_report))); link_it1--) {
    bodc = kcd_get_body_bb_table(robot_number1, link_it1).kcd_mo_id;
    
    for(link_it2=nof_links2-1; (link_it2 >= 0) &&
	  ((!collides) || ((robot_number2<0) && (with_report))); link_it2--) {
      bod  = kcd_get_body_bb_table(robot_number2, link_it2).kcd_mo_id;

      if(kcd_get_mo_col_pair_cur_is_act(bodc,bod)) {
	/* this corresponds to an activated collision pair */
	collides = kcd_mo_vs_mo(bodc, bod, with_report, &a_dist_estimate,
				&a_near_mo);
	switch(with_report) {
	case DISTANCE_ESTIMATE:
	case DISTANCE_EXACT:
	  if (!collides) {
	    if(a_dist_estimate < *min_dist_estimate) {
	      *min_dist_estimate = a_dist_estimate;
	      *nearest_obstacle  = a_near_mo;
	    }
	  } else {
	    *min_dist_estimate = 0.0;
	    *nearest_obstacle  = a_near_mo;
	  }
	  break;
	}
      }
    }
  }
  
  /* LIST: end */

  if(with_report) {
    if(collides) {
      /* we report collision */
      if (robot_number1>=0)
	{ kcd_add_report(robot_number1,-1); }
      if (robot_number2>=0)
	{ kcd_add_report(robot_number2,-1); }
      return TRUE;
    }
  }

  return collides;  
}

/* API */
/* ************************************************************************/
/*! \fn int kcd_robot_collides(int robot_id, int with_report, double *min_dist_estimate, int *nearest_obstacle)
\param with_report see documentation
\return *min_dist_estimate shortest distance found
\brief looks only for collision between given robot and static objects
\note does not look for self-collision, for collision with other robots
*/
/* ************************************************************************* */
int kcd_robot_collides(int robot_id, int with_report, double *min_dist_estimate, int *nearest_obstacle)
{
  int nearest_obst_for_link,link_id, place_counter, nof_bodies, collides = FALSE;
  double min_dist_for_link;
  int kcd_ext_mo, grp_id;
  kcd_vector3 pointA, pointB;


  if(!(kcd_grp_is_active(robot_id)) || 
     !(kcd_get_grp_col_env_cur_is_act(robot_id)))
    {
      /* group of movable objects is deactivated, must not be
         considered for collision testing => function returns FALSE */
      return FALSE;
    }

  nof_bodies = kcd_get_nof_grp_elts(robot_id);
  for(link_id=0; (link_id<nof_bodies) &&
	((!collides) || (robot_id<0)); link_id++)
    {
      grp_id = robot_id;
      kcd_ext_mo = kcd_get_body_bb_table(grp_id, link_id).kcd_mo_id;
      if (kcd_get_mo_col_env_cur_is_act(kcd_ext_mo)) {

	place_counter = kcd_ext_mo;
	collides = kcd_mo_in_collision(place_counter, with_report,
				       &min_dist_for_link,
				       &nearest_obst_for_link);
	if(!collides) {
	  switch (with_report) {
	  case DISTANCE_ESTIMATE:
	    if(*min_dist_estimate > min_dist_for_link) {
	      *min_dist_estimate = min_dist_for_link;
	      *nearest_obstacle = nearest_obst_for_link;
	    } 
	    if(grp_id != -1) {
	      /* movable object belongs to a group: storage by group
		 and index of mo in group */		    
	      kcd_set_distance_body_obst(robot_id,link_id,min_dist_for_link);
	    }		    
	    kcd_set_dist_mo_so(kcd_ext_mo,min_dist_for_link);
	    break;
	  case DISTANCE_EXACT:
	    /* take into account the relative error */
	    min_dist_for_link = min_dist_for_link -
	      (min_dist_for_link*kcd_get_kcd_exact_epsilon());
	 	     	      	  		 
	    if(*min_dist_estimate > min_dist_for_link) {
	      *min_dist_estimate = min_dist_for_link;
	      *nearest_obstacle = nearest_obst_for_link;
	    }
		    
	    /* get the closest points to set them in the report */
	    kcd_dist_get_closest_points(pointA, pointB); 		
	    /* storage of distance estimate for each movable object: */
	    if(grp_id != -1) {
	      /* movable object belongs to a group: storage by
		 group and index of mo in group */		    
	      kcd_set_distance_body_obst(robot_id,link_id,min_dist_for_link);
	      kcd_set_points_closest_pair(robot_id,link_id,pointA, pointB);
	    }		 		   
	    kcd_set_dist_mo_so(kcd_ext_mo,min_dist_for_link);
	    kcd_set_points_closest_pair_mo_so(kcd_ext_mo, pointA, pointB);
	    /* STILL FILL IN AND DO TEST Of NOt collides*/
	    break;
	  }     
	}
      }
    }
  return collides;
}

/* API */
/* ******************************************************************************** */
/*! int kcd_collision_exists(int with_report, double *min_dist_estimate)
 \brief looks for the existance of collision for any robot or movable object. 
 This is a general function that tests all possible collisions.

 \param with_report This parameter determines how the collision checker tests
   
 \note for collision tests between movable objects (self-collision, 
       collision between two groups or between a group and a mo or   
       between two mo's) only those pairs of group elements are  
       verified for which a collision pair was defined 

 \todo documentation of the with_report parameter
*/ 
/* ******************************************************************************** */
int kcd_collision_exists(int with_report, double *min_dist_estimate)
{
  int i,j;
  int i_collides = FALSE;
  int collides   = FALSE;
  int nr_robots, nearest_obstacle = -1;
  
  if (with_report == NO_REPORT)
    { min_dist_estimate = NULL; }

  nr_robots = kcd_get_nof_grps();


  /*initialisation of the report */
  kcd_set_report_to_zero();

  /* do the testing */
  
  /* test robots (groups of movable objects) */
  for(i=-1;i<nr_robots;i++) {
    i_collides = kcd_robot_collides(i, with_report, min_dist_estimate,
				    &nearest_obstacle);
    collides = collides || i_collides;

    /* add nearest obstacle */	
    if(( with_report==DISTANCE_ESTIMATE) && (i>=0)) {
      kcd_modify_nearest_obstacle(i,nearest_obstacle);
    }
    if ((!i_collides) || (with_report) || (i<0)) {
      i_collides = kcd_robot_collides_itself(i,with_report,min_dist_estimate,
					     &nearest_obstacle);
      collides = collides || i_collides;
      /* add nearest obstacle */	
      if(( with_report==DISTANCE_ESTIMATE) && (i>=0)) {
	kcd_modify_nearest_obstacle(i,nearest_obstacle);
      }
    }
    for(j=i+1; (j<nr_robots) && ((!i_collides)||(with_report)||(i<0)) ; j++) {
      i_collides = kcd_robot_vs_robot(i, j, with_report, min_dist_estimate,
				      &nearest_obstacle);	
      collides = collides || i_collides;
      if((with_report==DISTANCE_ESTIMATE) && (i >= 0)) {
	kcd_modify_nearest_obstacle(i,nearest_obstacle);
      }
    }
  }
  
  return collides;
}

/* API */
/* ************************************************************************/
/*! \fn int kcd_mo_collides(int mobj_id, int with_report, double *min_dist_estimate, int *nearest_obstacle)
 \brief looks only for collision between given mo and static objects

 \note with_report parameter determines the report
 \note does not look for collision with other robots and mo-s

 \todo verify utility of if(!(kcd_get_is_moved(kcd_ext_o)) )
*/

/* ************************************************************************* */
int kcd_mo_collides(int mobj_id, int with_report, double *min_dist_estimate, int *nearest_obstacle)
{
  int nearest_obst_for_link,robot_id,ith_body, place_counter,  collides = FALSE;
  double min_dist_for_link;
  double cur_dist_for_link;
  int kcd_ext_o;
  kcd_vector3 pointA,pointB;

  kcd_ext_o = kcd_mo_bb_table[mobj_id].ext_o_id;
  if(!(kcd_o_prm_table[kcd_ext_o].is_active) ||
     !(kcd_get_mo_col_env_cur_is_act(mobj_id)))
    {
      /* mo is not active, must not be tested for collision 
         => this function returns FALSE */
      return FALSE;
    }

  if(!(kcd_get_is_moved(kcd_ext_o)) )
    {
      /* mo didn't move recently: */
      /* CASE 1: */
      /* if mo didn't collide recently, it doesn't either now with static stuff,
         so we return FALSE (no collision) */

      /* CASE 2: */
      /* if mo collided recently, check with which obstacle, and 
         if this obstacle didn't move recently (i.e. it was a static obstacle), 
         return TRUE (collision), otherwise go on testing */

    }

  if(kcd_get_nof_prm_of_ext_o(kcd_ext_o) > 0)
    { /* movable object not empty */
      place_counter = mobj_id;
      robot_id = kcd_mo_bb_table[mobj_id].grp_id;
      ith_body = kcd_mo_bb_table[mobj_id].ith_mo;
      collides = kcd_mo_in_collision(place_counter,with_report,
				     &min_dist_for_link,
				     &nearest_obst_for_link);

      if (kcd_get_mo_col_env_cur_is_act(mobj_id) & KCD_TABLE_DIST_ACT) {
	switch (with_report) {
	case DISTANCE_ESTIMATE:

	  if(*min_dist_estimate > min_dist_for_link) {
	    *min_dist_estimate = min_dist_for_link;
	    *nearest_obstacle = nearest_obst_for_link;
	  } 
	  if(robot_id != -1) {
	    /* movable object belongs to a group: storage by group and
	       index of mo in group */	
	    kcd_get_dist_grp_mo_so(robot_id,ith_body,&cur_dist_for_link);
	    if (min_dist_for_link<cur_dist_for_link) {
	      kcd_set_distance_body_obst(robot_id,ith_body,min_dist_for_link);
	    }		    
	  }
	  kcd_get_dist_mo_so(mobj_id, &cur_dist_for_link);
	  if (min_dist_for_link<cur_dist_for_link)
	    { kcd_set_dist_mo_so(mobj_id,min_dist_for_link); }
	  break;

	case DISTANCE_EXACT:
	    /* take into account the relative error */
	  min_dist_for_link = min_dist_for_link -
	    (min_dist_for_link*kcd_get_kcd_exact_epsilon());

	  if(*min_dist_estimate > min_dist_for_link) {
	    *min_dist_estimate = min_dist_for_link;
	    *nearest_obstacle = nearest_obst_for_link;
	  }
		    
	  /* get the closest points to set them in the report */
	  kcd_dist_get_closest_points(pointA, pointB); 		
	  /* storage of distance estimate for each movable object: */

	  if(robot_id != -1) {
	    /* movable object belongs to a group: storage by group and
	       index of mo in group */		    
	    kcd_get_dist_grp_mo_so(robot_id,ith_body,&cur_dist_for_link);
	    if (min_dist_for_link<cur_dist_for_link) {
	      kcd_set_distance_body_obst(robot_id,ith_body,min_dist_for_link);
	      kcd_set_points_closest_pair(robot_id,ith_body,pointA, pointB);
	    }
	  }
	  /* storage of distance estimate by movable object */
	  kcd_get_dist_mo_so(mobj_id, &cur_dist_for_link);
	  if (min_dist_for_link<cur_dist_for_link) {
	    /* min_dist_estimate -> min_dist_for_link */
	    kcd_set_dist_mo_so(mobj_id,min_dist_for_link);
	    /* DO TEST  Not collides to fill the report*/
	    kcd_set_points_closest_pair_mo_so(mobj_id, pointA, pointB);
	  }
	  break;
	}
      }
    }
  if (!(kcd_get_mo_col_env_cur_is_act(mobj_id) & KCD_TABLE_COL_ACT))
    { return FALSE; }
  return collides;
}

/* API */
/* ******************************************************************************* */
/*! \fn int kcd_robot_collides_mo(int robot_id, int mo_id)
  \brief returns TRUE if given robot collides with given movable object
 
 - does not verify for definition of collision pair between given movable object 
   and an element of the given group.
 
 - does not test self-collision of robot
*/
/* ******************************************************************************* */
int kcd_robot_collides_mo(int robot_id, int mo_id)
{
  /* kcd_robot_vs_mo(robot_id,mo_id,NO_REPORT,NULL,NULL) */
  int in_collision = FALSE;
  int i,nof_r_mo,r_mo;

  if(!(kcd_grp_is_active(robot_id)))
    {
      /* group of movable objects is deactivated, must not be
         considered for collision testing => function returns FALSE */
      return FALSE;
    }

  if((!(kcd_o_prm_table[kcd_mo_bb_table[mo_id].ext_o_id].is_active)))
    {
      /* the movable object with mo_id is not active, must thus
         not be tested for collision => this function returns FALSE   */
      return FALSE; 
      /* in this way we do not even bother to enter the loop below */ 
    }

  if(!kcd_get_grp_col_pair_cur_is_act(-1, robot_id))
    {
      /* the group of movable object is not activate with the robot, must thus
         not be tested for collision => this function returns FALSE   */
      return FALSE; 
      /* in this way we do not even bother to enter the loop below */ 
    }

  nof_r_mo = kcd_get_nof_grp_elts(robot_id);
  /* visit each mov. obj. in group robot_id */
  /* call collision test between two movable objects 
     (stop testing when a collision is found) */
  for(i=0;(i<nof_r_mo)&&(!in_collision);i++)
    {
      r_mo = kcd_get_body_bb_table(robot_id, i).kcd_mo_id;
      in_collision = kcd_mo_collides_mo(r_mo,mo_id);
    }
  return in_collision;
}



/* ******************************************************************************* */
/*! \fn int kcd_robot_vs_mo(int robot_id, int mo_id, int with_report, double *min_dist_estimate, int *nearest_obstacle)
  \brief tests a group of movable objects against a movable object

  This function has a different functionality depending on the with_report parameter
  (see also kcd_api.h)

  \param robot_id the group_id of the movable objects
  \param mo_id id of the other movable object
  \param with_report  will decide how the mo's are tested
  \param *min_dist_estimate depending on with_report this returns nothing, an exact or 
  estimated distance
  \param  *nearest_obstacle  
*/       
/* ******************************************************************************* */

int kcd_robot_vs_mo(int robot_id, int mo_id, int with_report, double *min_dist_estimate, int *nearest_obstacle)
{
  int in_collision = FALSE;
  int i,nof_r_mo,r_mo;
  double a_dist_estimate;
  int a_near_mo;

  if(!(kcd_grp_is_active(robot_id)))
    {
      /* group of movable objects is deactivated, must not be
         considered for collision testing => function returns FALSE */
      return FALSE;
    }

  if((!(kcd_o_prm_table[kcd_mo_bb_table[mo_id].ext_o_id].is_active)))
    {
      /* the movable object with mo_id is not active, must thus
         not be tested for collision => this function returns FALSE   */
      return FALSE; 
      /* in this way we do not even bother to enter the loop below */ 
    }

  if(!kcd_get_grp_col_pair_cur_is_act(-1, robot_id))
    {
      /* the group of movable object is not activate with the robot, must thus
         not be tested for collision => this function returns FALSE   */
      return FALSE; 
      /* in this way we do not even bother to enter the loop below */ 
    }

  nof_r_mo = kcd_get_nof_grp_elts(robot_id);
  /* visit each mov. obj. in group robot_id */
  /* call collision test between two movable objects 
     (stop testing when a collision is found) */

  switch( with_report)
    {
    case NO_REPORT:
      {
	for(i=0;(i<nof_r_mo)&&(!in_collision);i++)
	  {
	    r_mo = kcd_get_body_bb_table(robot_id, i).kcd_mo_id;
	    if (kcd_get_mo_col_pair_cur_is_act(r_mo, mo_id)) {
	      in_collision = kcd_mo_vs_mo(r_mo,mo_id,with_report,
					  &a_dist_estimate,&a_near_mo);
	    }
	  }
	break;
      }
    case JUST_BOOL:
      {
	for(i=0;(i<nof_r_mo)&&(!in_collision);i++)
	  {
	    r_mo = kcd_get_body_bb_table(robot_id, i).kcd_mo_id;
	    if (kcd_get_mo_col_pair_cur_is_act(r_mo, mo_id)) {
	      in_collision = kcd_mo_vs_mo(r_mo,mo_id,with_report,
					  &a_dist_estimate,&a_near_mo);
	    }
	  }
	break;
      }
    default: 
      {
	for(i=0;(i<nof_r_mo)&&(!in_collision);i++)
	  {
	    r_mo = kcd_get_body_bb_table(robot_id, i).kcd_mo_id;
	    if (kcd_get_mo_col_pair_cur_is_act(r_mo, mo_id)) {
	      in_collision = kcd_mo_vs_mo(r_mo,mo_id,with_report,
					  &a_dist_estimate,&a_near_mo);	    
	      if((!in_collision)&&(a_dist_estimate < *min_dist_estimate))
		{
		  *min_dist_estimate = a_dist_estimate;
		  *nearest_obstacle  = a_near_mo;
		}	   
	    }
	  }	
      }
    }
  return in_collision;
}


/* API */
/* ******************************************************************************* */
/*! \fn int kcd_mo_collides_mo(int mov_obj_id1, int mov_obj_id2)
  \brief tests two movable objects for collision (NO_REPORT), returns TRUE if two movable objects collide, FALSE otherwise

  Does not verify for definition of collision pair between given movable objects,
  it just calls kcd_mo_vs_mo with with_report set to NO_REPORT

 \note     WAS: int kcd_body_hits_body(int mo1, int mo2),          
*/       
/* ******************************************************************************* */
int kcd_mo_collides_mo(int mo1, int mo2)
{
  return kcd_mo_vs_mo(mo1,mo2,NO_REPORT,NULL,NULL);
}

/* ******************************************************************************* */
/*! \fn int kcd_mo_vs_mo(int mo1, int mo2, int with_report, double *min_dist_estimate, int *nearest_obstacle)
  \brief tests two movable objects for collision

  This function has a different functionality depending on the with_report parameter
  (see also kcd_api.h)

  \param mo1 id of the first movable object
  \param mo1 id of the second movable object
  \param with_report  will decide how the mo's are tested
  \param *min_dist_estimate depending on with_report this returns nothing, an exact or 
  estimated distance
  \param  *nearest_obstacle 
  \note writtes the found result in the report
  \bug nearest_obstacle contains bugs
*/       
/* ******************************************************************************* */

int kcd_mo_vs_mo(int mo1, int mo2, int with_report, double *min_dist_estimate, int *nearest_obstacle)
{
  int guess_of_poly_id1, guess_of_poly_id2;
  int collision_exists=FALSE;
  kcd_matrix4 M1,*mat1,*M2,invM2,mat2;
  kcd_vector3 *a_center = NULL;
  kcd_vector3 pointA,pointB;
  double current_min_dist, current_min_dist2, gjk_dist;

  if( (!(kcd_o_prm_table[kcd_mo_bb_table[mo1].ext_o_id].is_active)) ||
      (!(kcd_o_prm_table[kcd_mo_bb_table[mo2].ext_o_id].is_active)) ||
      (!kcd_get_mo_col_pair_cur_is_act(mo1, mo2)) ||
      ((with_report != DISTANCE_EXACT) && (with_report != DISTANCE_ESTIMATE)
       && !(kcd_get_mo_col_pair_cur_is_act(mo1, mo2) & KCD_TABLE_COL_ACT))) {
    /* at least one of the movable objects is not active, must thus
       not be tested for collision => this function returns FALSE   */
    return FALSE;
  }

  if (!(kcd_get_mo_col_pair_cur_is_act(mo1, mo2) & KCD_TABLE_DIST_ACT))
    { with_report = JUST_BOOL; }

  guess_of_poly_id1 = kcd_mo_bb_table[mo1].bb_id;
  kcd_get_obj_abs_pos(kcd_mo_bb_table[mo1].ext_o_id,&mat1);
  M1[0][0] = all_bbs[guess_of_poly_id1]->eigenv[0][0];
  M1[1][0] = all_bbs[guess_of_poly_id1]->eigenv[0][1];
  M1[2][0] = all_bbs[guess_of_poly_id1]->eigenv[0][2];
  M1[0][3] = all_bbs[guess_of_poly_id1]->center[0];
  M1[0][1] = all_bbs[guess_of_poly_id1]->eigenv[1][0];
  M1[1][1] = all_bbs[guess_of_poly_id1]->eigenv[1][1];
  M1[2][1] = all_bbs[guess_of_poly_id1]->eigenv[1][2];
  M1[1][3] = all_bbs[guess_of_poly_id1]->center[1];
  M1[0][2] = all_bbs[guess_of_poly_id1]->eigenv[2][0];
  M1[1][2] = all_bbs[guess_of_poly_id1]->eigenv[2][1];
  M1[2][2] = all_bbs[guess_of_poly_id1]->eigenv[2][2];
  M1[2][3] = all_bbs[guess_of_poly_id1]->center[2];
  M1[3][0] = 0.0;
  M1[3][1] = 0.0;
  M1[3][2] = 0.0;
  M1[3][3] = 1.0;

  guess_of_poly_id2 = kcd_mo_bb_table[mo2].bb_id;
  kcd_get_obj_abs_pos(kcd_mo_bb_table[mo2].ext_o_id,&M2);
  kcd_matInvertTransfo(*M2,mat2);
  invM2[0][0] = all_bbs[guess_of_poly_id2]->eigenv[0][0];
  invM2[0][1] = all_bbs[guess_of_poly_id2]->eigenv[0][1];
  invM2[0][2] = all_bbs[guess_of_poly_id2]->eigenv[0][2];
  invM2[1][0] = all_bbs[guess_of_poly_id2]->eigenv[1][0];
  invM2[1][1] = all_bbs[guess_of_poly_id2]->eigenv[1][1];
  invM2[1][2] = all_bbs[guess_of_poly_id2]->eigenv[1][2];
  invM2[2][0] = all_bbs[guess_of_poly_id2]->eigenv[2][0];
  invM2[2][1] = all_bbs[guess_of_poly_id2]->eigenv[2][1];
  invM2[2][2] = all_bbs[guess_of_poly_id2]->eigenv[2][2];

  a_center = &(all_bbs[guess_of_poly_id2]->center);
  invM2[0][3] = -invM2[0][0]*(*a_center)[0] -invM2[0][1]*(*a_center)[1] -invM2[0][2]*(*a_center)[2];
  invM2[1][3] = -invM2[1][0]*(*a_center)[0] -invM2[1][1]*(*a_center)[1] -invM2[1][2]*(*a_center)[2];
  invM2[2][3] = -invM2[2][0]*(*a_center)[0] -invM2[2][1]*(*a_center)[1] -invM2[2][2]*(*a_center)[2];

  invM2[3][0] = 0.0;
  invM2[3][1] = 0.0;
  invM2[3][2] = 0.0;
  invM2[3][3] = 1.0;
  


  /* modified 13 Feb. 2001: in order to get distance estimate for 2 movable objects: */
  /* collision_exists = collision_exists_between_2_robot_obbtrees(all_bbs[guess_of_poly_id2],invM2,mat2,*mat1,M1,all_bbs[guess_of_poly_id1],NULL,NULL); */

  switch( with_report )
    { 
    case DISTANCE_EXACT:       
      { 
	kcd_dist_init_closest_points();	

	kcd_get_dist_mo_mo(mo1,&current_min_dist);
	kcd_get_dist_mo_mo(mo2,&current_min_dist2);

	/* it is necessary to take into consideration the two distances 
	 * if for mo2 the current_min_distance is bigger then it must be
	 * taken into consideration
	 */
	gjk_dist = MAX(current_min_dist, current_min_dist2);

	collision_exists =  kcd_dist_exact_between_2_obbtrees(all_bbs[guess_of_poly_id2],invM2,mat2,*mat1,
							      all_bbs[guess_of_poly_id1],&M1,NULL,&gjk_dist,nearest_obstacle,GO_DOWN_2_ROBOT_TREES); 


	/* take into account the relative error */
	gjk_dist = gjk_dist -(gjk_dist*kcd_get_kcd_exact_epsilon()) ;
     
	if ((gjk_dist < current_min_dist) || (gjk_dist < current_min_dist2)) {
	  kcd_dist_get_closest_points(pointA,pointB);   

	  if (gjk_dist < current_min_dist) {
	    kcd_set_points_closest_pair_mos(mo1, mo2, pointA,  pointB);
	    kcd_set_dist_mo_mo(mo1,gjk_dist);
	  }
	  if (gjk_dist < current_min_dist2) {
	    kcd_set_points_closest_pair_mos(mo2, mo1, pointB,  pointA);
	    kcd_set_dist_mo_mo(mo2,gjk_dist);	
	  }
	}
	if(min_dist_estimate)
	  { *min_dist_estimate = gjk_dist; }
	break;
	
      } 
    case DISTANCE_ESTIMATE:
      {
	 collision_exists = collision_exists_between_2_robot_obbtrees(all_bbs[guess_of_poly_id2],invM2,mat2,*mat1,M1,all_bbs[guess_of_poly_id1],
								     NULL,NULL,with_report,min_dist_estimate,nearest_obstacle);

	 kcd_get_dist_mo_mo(mo1,&current_min_dist);
	 if (*min_dist_estimate  <= current_min_dist)
	   { kcd_set_dist_mo_mo(mo1,*min_dist_estimate); }
	 else
	   { *min_dist_estimate= current_min_dist; }
	 kcd_get_dist_mo_mo(mo2,&current_min_dist);
	 if (*min_dist_estimate  <= current_min_dist)
	   { kcd_set_dist_mo_mo(mo2,*min_dist_estimate); }
	 else
	   { *min_dist_estimate= current_min_dist; }
	 break;  
      }	
    default:
      {
        collision_exists = collision_exists_between_2_robot_obbtrees(all_bbs[guess_of_poly_id2],invM2,mat2,*mat1,M1,all_bbs[guess_of_poly_id1],
								     NULL,NULL,with_report,min_dist_estimate,nearest_obstacle);
      }
      

    }

  if (!(kcd_get_mo_col_pair_cur_is_act(mo1, mo2) & KCD_TABLE_COL_ACT))
    { return FALSE; }
     
  return collision_exists;
}

/*  *************************************************************************  
    ************************************************************************* 
     Functions for parameter setting
    ************************************************************************* 
    ************************************************************************* */

/* API */
/* ********************************************************************** */
/*! \fn void  kcd_set_tolerance(double epsilon)
 \brief    sets tolerance value to epsilon
 \param   epsilon            the size of the security distance 
*/
/* ********************************************************************** */
void kcd_set_tolerance(double epsilon)
{
  kcd_assign_tolerance(epsilon);
}

/* API */
/* ********************************************************************** */
/*! \fn void kcd_ignore_tolerance() 
 \brief    resets tolerance value to zero 
           (does the same as kcd_set_tolerance(0.0))

*/
/* ********************************************************************** */
void kcd_ignore_tolerance()
{
  kcd_assign_tolerance(0.0);
}

/* ********************************************************************** */
/*! \fnvoid kcd_set_relative_error(double epsilon)
 \brief sets the value of kcd_epsilon in kcd, only usefull for exact distances
 \param   epsilon a value that has to be an percentage (0 <= epsilon < 100)
 \note the given value will be divided by 100, kcd uses the parameter as a value
       between 0 and 1
 \note works only with the DISTANCE EXACT option
*/
void kcd_set_relative_error(double value)
{
 if( (value < 0.0) || (value >= 100) )
    {
      PrintInfo(("KCD WARNING kcd_exact_epsilon is a %% a must be between 0 <= value < 100"));
      kcd_set_kcd_exact_epsilon(0.0);
      PrintInfo(("KCD: we set kcd_exact_epsilon to 0.0\n"));      
    }
  else
    {      
      PrintInfo(("KCD: we set kcd_exact_epsilon to %f\n", value));
      /* kcd_exact_epsilon is between 0 and 1 */
      kcd_set_kcd_exact_epsilon(value/100);
    }
}

/*!
 * \brief gets the value of kcd_epsilon in kcd 
 */
double kcd_get_relative_error()
{
  return (kcd_get_kcd_exact_epsilon()*100);
}
/* ********************************************************************** */


/* API */
/* ********************************************************************** */
/*! \fn kcd_init_min_vol_of_obj_detail(double one_dim)
 \brief sets smallest undividable volume for a bounding box
        before initialization of OBB(-tree)s and AABB hierarchy
       (to be called before definition of an environment)
 \param  one_dim     the smalllest undividable volume is a cube or
                          a square with sizes of length one_dim
*/
/* ********************************************************************** */
void kcd_init_min_vol_of_obj_detail(double one_dim)
{
  kcd_init_user_defined_small_volume(one_dim);
}

/* API */
/* ********************************************************************** */
/*! \fn void kcd_set_min_vol_of_obj_detail(double one_dim)
 \brief    sets smallest undividable volume for a bounding box (must be
     larger than the initial value), stores original value used 
     at initalization, uses the smallest undividable volume in 
     collision test functions
 \param  one_dim     the smalllest undividable volume is a cube or
                     a square with sizes of length one_dim
*/
/* ********************************************************************** */
void kcd_set_min_vol_of_obj_detail(double one_dim)
{
  kcd_set_user_defined_small_volume(one_dim);
}

/* API */
/* ********************************************************************** */
/*! \fn void kcd_ignore_min_vol_of_obj_detail()
 \brief  resets smallest undividable volume for a bounding box to
     original value used at initialization by the user (or to 0.0 
     by default, when the user didn't specify at initialization)
*/
/* ********************************************************************** */
void kcd_ignore_min_vol_of_obj_detail()
{
  kcd_reset_user_defined_small_volume();
}

/*! \fn int kcd_report_get_cor_mo(int kcd_obj_id)
 \brief get for a given kcd_obj_id the corresponding mo_id or so_id 
 \param kcd_obj_id is the id of the object in the list of all the KCD
        objects
 \return if it is a movable object a mo_id is return, this mo_id can be used
         to retrieve the information written in the reports of KCD
*/
int kcd_report_get_cor_mo(int kcd_obj_id)
{
  return (kcd_o_prm_table[kcd_obj_id].m_or_s_id);
}

/* API */

/* API */
/* Checks collision for device number 'i', i.e. tests for
   self-collision, collision with other devices and with static obstacles.
   Note: Computes only distance in case of test against static objects. */
int kcd_robot_collides_something(int i, int with_report, double *min_dist_estimate)
{
  int i_collides = FALSE;
  int j;
  int nr_robots, nearest_obstacle;

  
  if (with_report == NO_REPORT)
    { min_dist_estimate = NULL; }

  nr_robots = kcd_get_nof_grps();

  /*initialisation of the report */
  kcd_set_report_to_zero();

  /* do the testing */

  i_collides = kcd_robot_collides(i, with_report, min_dist_estimate,
				  &nearest_obstacle);

  /* add nearest obstacle */	
  if(( with_report==DISTANCE_ESTIMATE) && (i>=0)) {
    kcd_modify_nearest_obstacle(i,nearest_obstacle);
  }
  for(j=0; (j<nr_robots) && ((!i_collides) || (with_report) || (i<0)) ; j++) {
    if (i == j) { /* Autocollision */
      i_collides = kcd_robot_collides_itself(i,with_report,min_dist_estimate,
					     &nearest_obstacle);
      /* add nearest obstacle */	
      if(( with_report==DISTANCE_ESTIMATE) && (i>=0)) {
	kcd_modify_nearest_obstacle(i,nearest_obstacle);
      }
    } else {
      /* Carl: mo's now return dist estimate: */
      i_collides = kcd_robot_vs_robot(i, j, with_report, min_dist_estimate,
				      &nearest_obstacle);	
      if((with_report==DISTANCE_ESTIMATE) && (i >= 0)) {
	kcd_modify_nearest_obstacle(i,nearest_obstacle);
      }
    }
  }

  return i_collides;
}


/* ************************************************************ *
 * ************************************************************ *
 * 
 * new functions allowing to add an obstacle dynamically to KCD
 * 
 * ************************************************************ *
 * ************************************************************ */

static int obst_to_add = 0;

int kcd_nof_obst_to_add()
{
  return obst_to_add;
}

/* API */
/* function kcd_addObst_beg_scene
 * IN :    tot_nof_prims   total number of primitives to add 
 *                         (those contained in obstacles to add)
 *         nof_st_obj      total number of obstacles to add
 * prepares the internal data structures in order to add the given
 * number of obstacles and primitives without complete reinitialization
 * of the internal data structures
 */
void kcd_addObst_beg_scene(int current_scene, int tot_nof_prims, int nof_st_obj)
{
  int prev_tot_nof_obj; /* number of objects in current scene before adding any */
  int prev_tot_nof_prims; /* number of prims in current scene before adding any */

  // keep track of number of objects to add 
  obst_to_add = nof_st_obj;

  /* is this a new scene?  I have no clue where this flag (a static global 
     variable) is used */
  //  new_env = FALSE;  

  // count
  // the only place where the number of objects is stored, is in the 
  // static global variables kcd_nof_sos and kcd_nof_mos
  prev_tot_nof_obj = kcd_nof_mos + kcd_nof_sos;
  // the only place where the number of primitives is stored, is in the 
  // static global variable kcd_nof_prims_user
  prev_tot_nof_prims = kcd_nof_prims_user;

  // static global var: number of primitives user thought to give 
  // kcd_nof_prims_user = tot_nof_prims;

  /* resize data for current scene */  

  //  kcd_init_kcd_o_prm_table(tot_nof_obj);
  kcd_resize_kcd_o_prm_table(prev_tot_nof_obj,nof_st_obj);

  // kcd_init_prim_obj_table(tot_nof_prims);
  kcd_resize_prim_obj_table(prev_tot_nof_prims,tot_nof_prims);

  // kcd_init_kcd_poly_bb_table(tot_nof_prims);
  kcd_resize_kcd_poly_bb_table(prev_tot_nof_prims,tot_nof_prims);

  // kcd_init_ith_static_obj(nof_st_obj);
  kcd_resize_ith_static_obj(prev_tot_nof_obj,nof_st_obj);

  // modify the number of primitives the user thinks to have given:
  kcd_nof_prims_user += tot_nof_prims;  
}

/* API */
/*! \fn int kcd_end_scene()
 \brief modifies remaining internal hierarchies necessary for KCD
 \note Modifies AABB hierarchy, and connects OBB-trees to new AABBs
*/
void kcd_addObst_end_scene()
{
  int n_of_bbs;
  int nof_bbs_to_add;

  /* keep track of number of bbs to be added */
  // nof_bbs_to_add = kcd_get_number_of_bbs_to_add();
  nof_bbs_to_add = get_nr_on_pile();
  /* make access to kcd_bb pile */
  make_hash_table_on_bbs(); 
  /* collect number of BBs now defined */
  n_of_bbs = kcd_get_number_of_bbs();
  
  /* tell the user */
  PrintInfo(("KCD: kcd_addObst_end_scene...\n"));
  /* tell the user */
  PrintInfo(("KCD: modifying internal hierarchies A... \n"));

 /* put aabb around the primitive and 
     modify the aabb_hierarchy (insert new AABBs at each of the levels) */
  kcd_aabb_add(nof_bbs_to_add,n_of_bbs);
  
  /* tell the user */
  PrintInfo(("KCD: building internal hierarchies A done\n"));

  /* prepare GJK data structure for this size of the environment */
  kcd_gjk_support_resize_interesting_seed(n_of_bbs-nof_bbs_to_add,nof_bbs_to_add);
  kcd_initialized = TRUE;

  /* keep track of scene data */
  modif_so_kcd_data_arr();

  /* tell the user */
  PrintInfo(("KCD: kcd_addObst_end_scene done\n"));

}

/* API */
void kcd_addObst_beg_obj()
{
  /* static global variables */
  prim_sml_i = kcd_nof_prims;
  /* count total number of objects */
  ext_obj_id++;
  // printf("\nWe add object number %i\n",ext_obj_id);
  /* we begin a static object */
  new_so = TRUE;  /* was done in static kcd_begin_static_object() */
}

/* API */
int kcd_addObst_add_prim(void *primPt)
{
  int is_movable;      /* status flag */
  int ext_p_id;        /* external identifier for given primitive primPt */
  int kcd_p_id = 0;    /* return value */
  int kcd_obj_id;      /* root obb around moving object, or -1 */

  /* static global variable counting the number of objects in total */
  // ext_obj_id
  /* static global variable counting the number of primitives in total */
  // kcd_nof_prims

  /* this function is only used for dynamically adding a static object */
  is_movable = FALSE;
  kcd_obj_id = -1;

  /* put OBB(-tree) around the primitive */
  kcd_p_id = kcd_put_obb_around_polyh(primPt,kcd_nof_prims,is_movable,ext_obj_id);

  /* update tables : */
  ext_p_id = kcd_nof_prims;
  kcd_fill_prim_obj_table(ext_obj_id,ext_p_id,kcd_obj_id,kcd_p_id,is_movable);
  kcd_fill_kcd_poly_bb_table(ext_p_id,kcd_p_id);
  
  /* we added a primitive, count for it: */
  kcd_nof_prims++;

  return kcd_p_id;
}

/* API */
int kcd_addObst_end_obj()
{
  int obj_id;
  int is_movable = FALSE;

  /* static global variable */
  prim_big_i = kcd_nof_prims - 1;

  // corresponds to the call to function static kcd_end_static_object() */
  obj_id = kcd_nof_sos;
  kcd_nof_sos++;
  new_so = FALSE;
  /* tables: keep track of at which moment the kcd_so_id-th static object was given */
  kcd_fill_ith_static_obj(obj_id,ext_obj_id);

  /* table to be filled: */
  kcd_fill_kcd_o_prm_table(ext_obj_id,obj_id,is_movable,prim_sml_i,prim_big_i);

  return ext_obj_id; /* the ext_obj_id-th object (movable and static) given by the user */  
}


int kcd_get_nof_objects()
{
  int tot_nof;

  tot_nof = kcd_nof_sos + kcd_nof_mos;

  return tot_nof;
}


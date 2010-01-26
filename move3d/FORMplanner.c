#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Bio-pkg.h"
#include "sys/stat.h"

#ifdef CXX_PLANNER
#include "../planner_cxx/plannerFunctions.hpp"
#endif

#define MAX_NB_TRY_OPTIM  20

int G3D_SAVE_MULT = FALSE;
//extern int test_brunet; // KINEO-DEV :doit etre declare dans un .h !!
extern FL_OBJECT *planner_obj;  // KINEO-DEV: doit etre declare dans un .h !!
extern FL_FORM *OPTIM_FORM;
extern MENU_ROBOT *ROBOTS_FORM;
extern FL_OBJECT* DRAW_GRAPH_OBJ;
FL_FORM  *PLANNER_FORM = NULL;
static FL_OBJECT *PLANNER_CHOICE = NULL;
static FL_OBJECT *OPTIMIZE_FRAME = NULL;
static FL_OBJECT *GRAPH_NODE_FRAME = NULL;
static FL_OBJECT *INPUT_FRAME = NULL;
static FL_OBJECT *TRAJECTORY_FRAME = NULL;
static FL_OBJECT *SPACE3 = NULL;
static FL_OBJECT *SPACE4 = NULL;
static FL_OBJECT *SPACE5 = NULL;
static FL_OBJECT *SPACE6 = NULL;



static void g3d_create_smr_confs_obj(void);   // modif Ron
static void g3d_create_local_search_obj(void);
static void g3d_create_random_confs_obj(void);   // modif Juan
#ifdef MULTILOCALPATH
static void g3d_create_multiLocalPath_obj(void);
#endif
static void g3d_create_global_search_obj(void);
static void g3d_create_specific_search_obj(void);
static void g3d_create_N_specific_obj(void);
static void g3d_create_transSpePlanner_obj(void);
static void g3d_create_optim_obj(void);
static void g3d_create_stat_obj(void);
static void g3d_create_drawnjnt_obj(void);    // modif Juan
static void g3d_create_ik_button(void);
static void g3d_create_ik_form(void);
static void g3d_create_global_input_obj(void);
static void g3d_create_global_param_obj(void);
static void g3d_create_compco_input_obj(void);
static void g3d_create_compco_param_obj(void);
static void g3d_create_del_param_obj(void);
static void g3d_create_DMAX_param_obj(void);
static void g3d_create_print_obj(void);
static void g3d_create_saveInfo_obj(void);
#ifdef MULTIGRAPH
static void g3d_create_multiGraph_obj(void);
#endif
static void g3d_create_draw_obj(void);
static void g3d_create_draw_optim_obj(void);
static void g3d_create_load_graph_obj(void);
static void g3d_create_save_graph_obj(void);
static void g3d_create_draw_light_trace_obj(void);
static void g3d_create_draw_trace_obj(void);
static void g3d_create_graph_in_grid_obj(void);
static void g3d_create_stop_obj(void);
static void g3d_create_pinpoint_node_obj(void);
static void g3d_create_expand_obj(void);
static void g3d_create_isolate_obj(void);
static void g3d_create_linking_obj(void);
static void g3d_create_test_obj(void);
static void g3d_create_del_node_obj(void);
static void g3d_create_plan_strategy_obj(void);
static void g3d_create_sorting_obj(void);
static void g3d_create_sampling_obj(void);
static void g3d_create_nbtry_input_obj(void);
static void g3d_create_nbtry_param_obj(void);
static void g3d_create_add_obj(void);
static void g3d_create_frame_obj(void);
static void g3d_create_oriented_obj(void);
static void g3d_create_path_defom_obj(void);
static void g3d_create_tmax_obj(void);
static void g3d_create_max_connect_obj(void);
static void g3d_create_save_mult_obj(void);
static void p3d_desactivate_ik_draw(int desactivate);
static void p3d_desactivate_planning(int desactivate);
static void g3d_create_stat_obj(void); // Stats Button; Commit Jim; date: 01/10/2008


static void g3d_delete_local_search_obj(void);
static void g3d_delete_random_confs_obj(void);   // modif Juan
#ifdef MULTILOCALPATH
static void g3d_delete_multiLocalPath_obj(void);
#endif
static void g3d_delete_smr_confs_obj(void);   // modif Ron
static void g3d_delete_drawnjnt_obj(void);   // modif Juan
static void g3d_delete_global_search_obj(void);
static void g3d_delete_specific_search_obj(void);
static void g3d_delete_transSpePlanner_obj(void);
static void g3d_delete_N_specific_obj(void);
static void g3d_delete_optim_obj(void);
static void g3d_delete_stat_obj(void);
static void g3d_delete_global_input_obj(void);
static void g3d_delete_global_param_obj(void);
static void g3d_delete_compco_input_obj(void);
static void g3d_delete_compco_param_obj(void);
static void g3d_delete_del_param_obj(void);
static void g3d_delete_DMAX_param_obj(void);
static void g3d_delete_print_obj(void);
static void g3d_delete_saveInfo_obj(void);
#ifdef MULTIGRAPH
static void g3d_delete_multiGraph_obj(void);
#endif
static void g3d_delete_draw_obj(void);
static void g3d_delete_draw_optim_search_obj(void);
static void g3d_delete_draw_light_trace_obj(void);
static void g3d_delete_draw_trace_obj(void);
static void g3d_delete_load_graph_obj(void);
static void g3d_delete_save_graph_obj(void);
static void g3d_delete_graph_in_grid_obj(void);
static void g3d_delete_stop_obj(void);
static void g3d_delete_pinpoint_node_obj(void);
static void g3d_delete_expand_obj(void);
static void g3d_delete_isolate_obj(void);
static void g3d_delete_linking_obj(void);
static void g3d_delete_test_obj(void);
static void g3d_delete_del_node_obj(void);
static void g3d_delete_plan_strategy_obj(void);
static void g3d_delete_sorting_obj(void);
static void g3d_delete_sampling_obj(void);
static void g3d_delete_nbtry_input_obj(void);
static void g3d_delete_nbtry_param_obj(void);
static void g3d_delete_add_obj(void);
static void g3d_delete_frame_obj(void);
static void g3d_delete_oriented_obj(void);
static void g3d_delete_path_deform_obj(void);
static void g3d_delete_tmax_obj(void);
static void g3d_delete_max_connect_obj(void);
static void g3d_delete_save_mult_obj(void);
static void g3d_delete_ik_button(void);
static void g3d_delete_ik_form(void);
static void g3d_delete_stat_obj(void);	/* Stats button destructor;
								Commit Jim; date: 01/10/2008 */

/* booleen d'arret */
//static int STOP = FALSE;

/* variable pour guarder la derniere conf. valide d'un robot */
static configPt last_p_deg;

/************************/
/* Option form creation */
/************************/
void g3d_create_planner_form(void) {

  g3d_create_form(&PLANNER_FORM, 500, 435, FL_UP_BOX);
  g3d_create_frame(&PLANNER_CHOICE, FL_NO_FRAME, 150, 123, "", (void**)&PLANNER_FORM, 1);
#if defined(MULTILOCALPATH)
	g3d_create_multiLocalPath_obj();
#endif
  g3d_create_local_search_obj();
  g3d_create_random_confs_obj();   // modif Juan
  g3d_create_global_search_obj();
  g3d_create_smr_confs_obj();   // modif Ron
  g3d_create_specific_search_obj();
  g3d_create_N_specific_obj();
  g3d_create_transSpePlanner_obj();
  g3d_create_frame_obj();
  g3d_create_draw_obj();
  g3d_create_drawnjnt_obj();      // modif Juan
  g3d_create_print_obj();
  g3d_create_saveInfo_obj();
  #ifdef MULTIGRAPH
  g3d_create_multiGraph_obj();
  #endif
  g3d_create_ik_button();
  g3d_create_labelframe(&TRAJECTORY_FRAME, FL_ENGRAVED_FRAME, 80, 113, "Trajectory", (void**)&PLANNER_FORM, 1);
  g3d_create_draw_optim_obj();
  g3d_create_save_mult_obj();
  g3d_create_draw_light_trace_obj();
  g3d_create_draw_trace_obj();
  g3d_create_frame(&OPTIMIZE_FRAME, FL_NO_FRAME, 111, 113, "", (void**)&PLANNER_FORM, 1);
  g3d_create_path_defom_obj();     //add Mokhtar
  g3d_create_optim_obj();
  g3d_create_stat_obj();
  g3d_create_plan_strategy_obj();
  g3d_create_oriented_obj();
  g3d_create_sorting_obj();
  g3d_create_frame(&INPUT_FRAME, FL_NO_FRAME, -1, -1, "", (void**)&PLANNER_FORM, 1);
  g3d_create_tmax_obj();
  g3d_create_max_connect_obj();
  g3d_create_sampling_obj();
  g3d_create_frame(&GRAPH_NODE_FRAME, FL_NO_FRAME, -1, -1, "", (void**)&PLANNER_FORM, 1);
  g3d_create_load_graph_obj();
  g3d_create_DMAX_param_obj();
  g3d_create_save_graph_obj();
  g3d_create_global_input_obj();
  g3d_create_global_param_obj();
  g3d_create_del_param_obj();
  g3d_create_compco_input_obj();
  g3d_create_compco_param_obj();
  g3d_create_test_obj();
  g3d_create_nbtry_input_obj();
  g3d_create_nbtry_param_obj();
  g3d_create_pinpoint_node_obj();
  g3d_create_frame(&SPACE3, FL_NO_FRAME, -1, -1, "", (void**)&GRAPH_NODE_FRAME, 0);
  g3d_create_expand_obj();
  g3d_create_del_node_obj();
  g3d_create_add_obj();
  g3d_create_frame(&SPACE4, FL_NO_FRAME, -1, -1, "", (void**)&GRAPH_NODE_FRAME, 0);
  g3d_create_isolate_obj();
  g3d_create_linking_obj();
  g3d_create_frame(&SPACE5, FL_NO_FRAME, -1, -1, "", (void**)&GRAPH_NODE_FRAME, 0);
  g3d_create_graph_in_grid_obj();
  g3d_create_frame(&SPACE6, FL_NO_FRAME, -1, -1, "", (void**)&GRAPH_NODE_FRAME, 0);
  g3d_create_stop_obj();

  fl_end_form();

  g3d_create_optim_form();
}

/**********************************************************************/
static FL_OBJECT  *SEARCH_OBJ;
static FL_OBJECT  *RANDCONFS_OBJ;   // modif Juan
static FL_OBJECT  *SMRCONFS_OBJ;   // modif Ron
static FL_OBJECT  *SEARCH_GLOBAL_OBJ;
static FL_OBJECT  *SEARCH_SPECIFIC_OBJ;
static FL_OBJECT  *SEARCH_SPECIFIC_TRANS_OBJ;
static FL_OBJECT  *N_SPECIFIC_OBJ;

FL_OBJECT  *SEARCH_LINEAR_OBJ;
FL_OBJECT  *SEARCH_PLATEFORM_OBJ;
FL_OBJECT  *SEARCH_ARM_OBJ;
FL_OBJECT  *SEARCH_LOCAL_OBJ;
FL_OBJECT  *SEARCH_MANHAT_OBJ;

FL_OBJECT  *SEARCH_GLOBAL_PARAM_OBJ;
static FL_OBJECT  *SEARCH_GLOBAL_INPUT_OBJ;
static FL_OBJECT  *SEARCH_DEL_PARAM_OBJ;

FL_OBJECT  *SEARCH_DMAX_PARAM_OBJ;

FL_OBJECT  *DRAWNJNT_OBJ;   // modif Juan

FL_OBJECT  *SEARCH_DRAW_OBJ;
FL_OBJECT  *DRAW_LIGHT_TRACE_OBJ;
FL_OBJECT  *DRAW_TRACE_OBJ;
FL_OBJECT  *SEARCH_PRINT_OBJ;
FL_OBJECT  *SAVE_INFO_OBJ;
#ifdef MULTIGRAPH
static FL_OBJECT  *MULTIGRAPH_OBJ;
#endif

static FL_OBJECT  *IK_BUTTON;
static FL_FORM    *IK_FORM = NULL;
static FL_OBJECT  *IK_FRAME;
static FL_OBJECT  *IK_GROUP;
static FL_OBJECT  *NORMAL_CB;
static FL_OBJECT  *MULTISOL_CB;
static FL_OBJECT  *UNIQUE_CB;
static FL_OBJECT  *IK_DRAW_FRAME;
static FL_OBJECT  **IK_DRAW;

FL_OBJECT  *SEARCH_COMPCO_PARAM_OBJ;
static FL_OBJECT  *SEARCH_COMPCO_INPUT_OBJ;

FL_OBJECT *OPTIM_OBJ;
static FL_OBJECT *STAT_FRAME_OBJ;
static FL_OBJECT *STAT_OBJ;
static FL_OBJECT *STAT_PRINT_OBJ;

FL_OBJECT  *SEARCH_DRAW_OPTIM_OBJ;

static FL_OBJECT  *SAVE_GRAPH_OBJ;
static FL_OBJECT  *LOAD_GRAPH_OBJ;

static FL_OBJECT  *GRAPH_IN_GRID_OBJ;

static FL_OBJECT  *STOP_OBJ;

static FL_OBJECT  *EXPAND_OBJ;
static FL_OBJECT  *PINPOINT_NODE_OBJ;
static FL_OBJECT  *ISOLATE_OBJ;
static FL_OBJECT  *LINKING_OBJ;

static FL_OBJECT  *DEL_NODE_OBJ;

static FL_OBJECT  *TEST_OBJ;

FL_OBJECT  *STRAT0_OBJ;
FL_OBJECT  *STRAT1_OBJ;
FL_OBJECT  *STRAT2_OBJ;
FL_OBJECT  *STRAT3_OBJ;
FL_OBJECT  *STRAT4_OBJ;

FL_OBJECT  *SORT1_OBJ;
FL_OBJECT  *SORT2_OBJ;
FL_OBJECT  *COST_OBJ;

FL_OBJECT  *SAMPL1_OBJ; //random
FL_OBJECT  *SAMPL2_OBJ; //halton
FL_OBJECT  *SAMPL3_OBJ; //gaussian
FL_OBJECT  *SAMPL4_OBJ; //bridge
FL_OBJECT  *SAMPL5_OBJ; //obprm

FL_OBJECT  *ORIENTED_OBJ;

static FL_OBJECT  *GROUP2;
static FL_OBJECT  *GROUP3;
static FL_OBJECT  *GROUP4;

FL_OBJECT  *SEARCH_NBTRY_PARAM_OBJ;
static FL_OBJECT  *SEARCH_NBTRY_INPUT_OBJ;

static FL_OBJECT  *ADD_OBJ;

static FL_OBJECT  *FRAME_OBJ;

static FL_OBJECT *PLANNING_STATEGY = NULL;
static FL_OBJECT *ORIENTED_FRAME = NULL;
static FL_OBJECT *CONNEXION_STRATEGY = NULL;
static FL_OBJECT *SAMPLING_STRATEGY = NULL;
static FL_OBJECT *DISTANCE_MAX = NULL;

static FL_OBJECT *TMAX_OBJ;
static FL_OBJECT *MAX_CONNECT_OBJ;
static FL_OBJECT *SEARCH_DRAW_MULT_OBJ;
FL_OBJECT *PATH_DEFORMATION = NULL;
FL_FORM *PATH_DEFORM_FORM = NULL;

#ifdef MULTILOCALPATH
FL_OBJECT *MULTILOCALPATH_OBJ = NULL;
FL_FORM *MULTILOCALPATH_FORM = NULL;
#endif

/**********************************************************************/


static int  s_default_drawtraj_fct(p3d_rob* robot, p3d_localpath* curLp) {
  g3d_draw_allwin_active();
  fl_check_forms();
  return(!p3d_GetStopValue());
}

/* planification local + test de validite */
static void CB_local_search_obj(FL_OBJECT *ob, long arg) {
  p3d_rob *robotPt = NULL;
  char      str[255], sti[255];
  p3d_localpath *localpathPt = NULL;
  int ntest, ntrj, /*ir,*/ *ikSol = NULL;
  p3d_localplanner_type lpl_type;

  p3d_SetStopValue(FALSE);

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  strcpy(str, "loctrj.");
  sprintf(sti, "%s", robotPt->name);
  strcat(str, sti);
  sprintf(sti, "%s", ".");
  strcat(str, sti);
  ntrj = p3d_get_desc_number(P3D_TRAJ) + 1;
  sprintf(sti, "%d", ntrj);
  strcat(str, sti);

  lpl_type = robotPt->lpl_type;

  p3d_get_non_sing_iksol(robotPt->cntrt_manager, robotPt->ikSolPos, robotPt->ikSolGoto, &ikSol);
  localpathPt = p3d_local_planner_multisol(robotPt,
                robotPt->ROBOT_POS, robotPt->ROBOT_GOTO, ikSol);
  if (localpathPt != NULL) {
    p3d_beg_desc(P3D_TRAJ, str);
    p3d_add_desc_courbe(localpathPt);
    p3d_end_desc();
    g3d_add_traj((char*)"Localsearch", p3d_get_desc_number(P3D_TRAJ));
  }

  PrintInfo(("MP : robot %d : p3d_localplanner : ", robotPt->num));
  switch (p3d_search_status()) {
    case P3D_SUCCESS:
      PrintInfo(("SUCCESS"));
      break;
    case P3D_FAILURE:
      PrintInfo(("FAILURE"));
      break;
    case P3D_CONFIG_EQUAL:
      PrintInfo(("CONFIG_EQUAL"));
      break;
    case P3D_ILLEGAL_START:
      PrintInfo(("ILLEGAL_START"));
      break;
    case P3D_ILLEGAL_GOAL:
      PrintInfo(("ILLEGAL_GOAL"));
      break;
    case P3D_WRONG_CALL:
      PrintInfo(("WRONG_CALL"));
      break;
    default:
      PrintInfo(("???"));
  }
  PrintInfo(("\n"));

  if (localpathPt != NULL) {
    ntest = 0;
    if (p3d_unvalid_localpath_test(robotPt, robotPt->tcur->courbePt, &ntest)) {  // <- modif Juan
      PrintInfo(("p3d_col_test_localpath: PATH NOT VALID ! nbtest : %d\n", ntest));
    } else {
      PrintInfo(("p3d_col_test_localpath: PATH VALID; nbtest : %d\n", ntest));
    }
  }

  g3d_show_tcur_rob(robotPt, s_default_drawtraj_fct);
  fl_set_button(SEARCH_OBJ, 0);
}



static void g3d_create_local_search_obj(void) {
  g3d_create_button(&SEARCH_OBJ, FL_PUSH_BUTTON, 50.0, 30.0, "Local\nPlanner", (void**)&PLANNER_CHOICE, 0);//modif Juan
  fl_set_call_back(SEARCH_OBJ, CB_local_search_obj, 0);
}


/***********************************************************/
/****************** planificateur global *******************/
/***********************************************************/



int fct_stop(void) {
  static double ti = 0;
  double ts, tu;
  double tmax = p3d_get_tmax();

  ChronoTimes(&tu, &ts);
  if (tu > ti) {
    fl_check_forms();
    ti = tu + 0.5;
  }
  if ((p3d_GetStopValue()) || ((tu > tmax) && (tmax != 0))) {
    // STOP = FALSE;
    //  p3d_SetStopValue(FALSE);
    ti = 0;
    return(FALSE);
  }
  return(TRUE);
}

void fct_draw(void) {
  if (ENV.getBool(Env::drawGraph))
    g3d_draw_allwin_active();
}

/*static*/
void CB_global_search_obj(FL_OBJECT *ob, long arg) {

  MY_ALLOC_INFO("Avant la creation du graphe");
  p3d_set_planning_type(P3D_GLOBAL);
  p3d_desactivate_ik_draw(TRUE);
  p3d_desactivate_planning(TRUE);

#ifndef USE_CXX_PLANNER
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
#else
  p3d_learn_cxx(p3d_get_NB_NODES(), fct_stop, fct_draw);
#endif

  p3d_desactivate_ik_draw(FALSE);
  p3d_desactivate_planning(FALSE);
  p3d_set_planning_type(P3D_NONE);
  fl_ringbell(0);
  fl_set_button(SEARCH_GLOBAL_OBJ, 0);
}


static void g3d_create_global_search_obj(void) {
  g3d_create_button(&SEARCH_GLOBAL_OBJ, FL_PUSH_BUTTON, 79.0, 30.0, "Global Planner", (void**)&PLANNER_CHOICE, 0);
  fl_set_call_back(SEARCH_GLOBAL_OBJ, CB_global_search_obj, 0);
}


// modif Juan
/***********************************************************/
/****************** random configurations ******************/
/***********************************************************/

// as CB_global_search_obj but noder are not connected
static void CB_random_confs_obj(FL_OBJECT *ob, long arg) {
  //  STOP = FALSE;
  p3d_SetStopValue(FALSE);
  MY_ALLOC_INFO("Avant la creation du graphe");

  p3d_randconfs(p3d_get_NB_NODES(), fct_stop, fct_draw);

  fl_ringbell(0);
  fl_set_button(RANDCONFS_OBJ, 0);
}


// modif Ron
// Generate collision-free random configrations and save to a file for use with the SMR.
static void CB_smr_confs_obj(FL_OBJECT *ob, long arg) {

  p3d_rob *robotPt;
  configPt q;
  int collision_free;
  int num_configs_found = 0;
  p3d_jnt *jntPt;
  p3d_obj *objPt;
  p3d_poly *polPt;
  p3d_vector3 a_pos;
  int i, j, k;
  const char* read_filename;
  char movable_atoms_list_filename[1024];
  FILE* movable_atoms_list_file;
  char movable_atoms_output_filename[1024];
  FILE* movable_atoms_output_file;
  int read_num_movable_atoms;
  int num_movable_atoms = 0;
  int read_movable_atom_pdb_num;
  p3d_poly* movable_atom[20000]; // TODO: dynamically allocate this array so number of atoms is not limited to a fixed constant.
  int atom_num;
  int success;

#ifdef BIO
  // STOP = FALSE;
  p3d_SetStopValue(FALSE);
  // Get the system robot
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  // Specify input and output files
  read_filename = fl_show_fselector("Open file with list of movable atoms", "~/", "*.*", "");
  if (read_filename == NULL) {
    printf("No filename to open provided.");
    return;
  }
  strcpy(movable_atoms_list_filename, read_filename);
  read_filename = fl_show_fselector("Save movable atoms file", "~/", "*.*", "");
  if (read_filename == NULL) {
    printf("No filename to save provided.");
    return;
  }
  strcpy(movable_atoms_output_filename, read_filename);

  // Load input file
  printf("Loading input file %s...\n", movable_atoms_list_filename);
  movable_atoms_list_file = fopen(movable_atoms_list_filename, "r");
  if (movable_atoms_list_file == NULL) {
    printf("Error: Could not open input file %s. Exiting function.", movable_atoms_list_filename);
    return;
  }
  // Read the file
  fscanf(movable_atoms_list_file, "%d", &read_num_movable_atoms);
  printf("Reading %d movable atoms from file...\n", read_num_movable_atoms);
  for (k = 0; k < read_num_movable_atoms; k++) {
    fscanf(movable_atoms_list_file, "%d", &read_movable_atom_pdb_num);
    printf("Read movable atom %d.\n", read_movable_atom_pdb_num);

    success = FALSE;

    // Get the polygon associated with the PDB atom number.
    // Loop over each joint and get the body asociated with the joint (which encodes an atom).
    // If multiple atoms in the robot have the same PDB atom number, then multiple polygons will be added to the list.
    for (i = 0; i <= robotPt->njoints; i++) {
      jntPt = robotPt->joints[i];
      objPt = jntPt->o;
      if (objPt != NULL) {
        for (j = 0;j < objPt->np;j++) {
          polPt = objPt->pol[j];
          atom_num = bio_get_pdb_atom_number(polPt);

          if (atom_num == read_movable_atom_pdb_num && !success) {
            movable_atom[num_movable_atoms] = polPt;
            num_movable_atoms++;
            success = TRUE;
            printf("Found movable atom %d.\n", read_movable_atom_pdb_num);
          }
        }
      }
    }

  }
  fclose(movable_atoms_list_file);

  printf("Found %d movable atoms.\n", num_movable_atoms);

  if (num_movable_atoms != read_num_movable_atoms) {
    printf("Error: Number of movable atoms specified %d does not match number read %d.", read_num_movable_atoms, num_movable_atoms);
    return;
  }

  // Open output file for writing
  movable_atoms_output_file = fopen(movable_atoms_output_filename, "w");

  // Write SMRoadmap header
  fprintf(movable_atoms_output_file, "SMRoadmap\n1\n");

  // Write the number of DOF, which equals the number of movable atoms * 3 translation DOF/atom
  fprintf(movable_atoms_output_file, "%d\n", 3*num_movable_atoms);

  // Write the number of random configurations to generate
  fprintf(movable_atoms_output_file, "%d\n", p3d_get_NB_NODES());

  // Write the number of movable atoms to the file
// fprintf(movable_atoms_output_file, "%d\n", num_movable_atoms);


  printf("Generating and writing random configurations...\n");

  // Loop until the desired number of collision-free configurations are found
  while (num_configs_found < p3d_get_NB_NODES()) {

    collision_free = FALSE;

    // Sample configurations until a collision-free configuration is found
    while (!collision_free) {

//   printf("|");

      // Allocate and generate a random robot configuration q
      q = p3d_alloc_config(robotPt);
      p3d_shoot(robotPt, q, 1);

      // TODO: delete the memory allocated for this config when it is no longer in use

      // Set current robot configuration to q.
      // This function may modify values of q to satisfy constraints (if possible).
      p3d_set_and_update_this_robot_conf(robotPt, q);

      // Check validity of current config (set by q)
      collision_free = !p3d_col_test();

      // If the config is not collision-free, then delete it
      if (!collision_free)
        p3d_destroy_config(robotPt, q);
    }

    num_configs_found++;

    printf(".");
    fflush(stdout);

    fprintf(movable_atoms_output_file, "%d\t%d\t", (num_configs_found - 1), 3*num_movable_atoms);
    // Write the moving atoms in the collision-free configuration to a file.
    for (i = 0; i < num_movable_atoms; i++) {

      polPt = movable_atom[i];
      atom_num = bio_get_pdb_atom_number(polPt);

      for (k = 0; k < 3; k++) {
        a_pos[k] = polPt->poly->pos[k][3];
        fprintf(movable_atoms_output_file, "%f\t", (float)(a_pos[k]));
      }

    }
    /*
      ljl: Computation of the energy with Amber
    */


    // Print 'X' for node weight and collision-free status since they are unknown by SMR requirements
    fprintf(movable_atoms_output_file, "X\tX\n");

    /*  fprintf(movable_atoms_output_file, "%d\n", num_configs_found);
      // Write the moving atoms in the collision-free configuration to a file.
      for( i = 0; i < num_movable_atoms; i++ ) {

       polPt = movable_atom[i];
       atom_num = bio_get_pdb_atom_number(polPt);

       fprintf(movable_atoms_output_file, "%d\t%d", (i+1), atom_num);
       for( k = 0; k < 3; k++ ) {
        a_pos[k] = polPt->poly->pos[k][3];
        fprintf(movable_atoms_output_file, "\t%f", (float)(a_pos[k]));
       }
       fprintf(movable_atoms_output_file, "\n");
      }
    */
    // Delete the configuration since it is not longer needed
    p3d_destroy_config(robotPt, q);

  }

  fclose(movable_atoms_output_file);

  printf("\n");
// printf("Input file: %s\n", movable_atoms_list_filename);
  printf("Output file: %s\n", movable_atoms_output_filename);

#else
  printf("This feature not implemented for non-biology applications.\n");
#endif

  // Notify UI that this procedure is complete
  fl_ringbell(0);
  fl_set_button(SMRCONFS_OBJ, 0);

}


static void g3d_create_random_confs_obj(void) { // modif Juan
  g3d_create_button(&RANDCONFS_OBJ, FL_PUSH_BUTTON, 50.0, 30.0, "Random\nConfs", (void**)&PLANNER_CHOICE, 0);
  fl_set_call_back(RANDCONFS_OBJ, CB_random_confs_obj, 0);
}

static void g3d_create_smr_confs_obj(void) { // modif Ron
  g3d_create_button(&SMRCONFS_OBJ, FL_PUSH_BUTTON, 28.0, 30.0, "Write\nSMR", (void**)&PLANNER_CHOICE, 0);
  fl_set_call_back(SMRCONFS_OBJ, CB_smr_confs_obj, 0);
}


/***********************************************************/
/****************** multigraph form ******************/
/***********************************************************/
#ifdef MULTILOCALPATH
static void CB_multiLocalPath_obj(FL_OBJECT *ob, long arg) {
	if (fl_get_button(ob)) {
		g3d_create_multiLocalPath_form();
		fl_show_form(MULTILOCALPATH_FORM,FL_PLACE_SIZE,TRUE,"MultiLocPath");
	} else {
		g3d_destroy_multiLocalPath_Form();
	}
}

static void g3d_create_multiLocalPath_obj(void) {
	g3d_create_button(&MULTILOCALPATH_OBJ,FL_PUSH_BUTTON,75.0,30.0,"MultiLocPath",(void**)&PLANNER_CHOICE,0);
	fl_set_call_back(MULTILOCALPATH_OBJ,CB_multiLocalPath_obj,0);
}

#endif

/***********************************************************/
/***********************************************************/

static void CB_N_specific_input_obj(FL_OBJECT *ob, long arg) {
  p3d_set_NB_specific(atoi(fl_get_input(N_SPECIFIC_OBJ)));
}

static void g3d_create_N_specific_obj(void) {
  char buffer[10];
  g3d_create_input(&N_SPECIFIC_OBJ, FL_NORMAL_INPUT, 28.0, 20, "", (void**)&PLANNER_CHOICE, 0);
  sprintf(buffer, "%d", p3d_get_NB_specific());
  fl_set_input(N_SPECIFIC_OBJ, buffer);
  fl_set_call_back(N_SPECIFIC_OBJ, CB_N_specific_input_obj, 0);
}

/***********************************************************/

void CB_specific_search_obj(FL_OBJECT *ob, long arg){
  char *filePrefix = NULL;
  char c_dir_name[300];

  if(ob){fl_deactivate_object(ob);}
  fl_deactivate_object(SEARCH_DEL_PARAM_OBJ);

  if (G3D_SAVE_MULT) {
    p3d_get_directory(c_dir_name);
    filePrefix = (char *)fl_show_simple_input("Directory to save files", (const char*)c_dir_name);
    if (filePrefix == NULL) {
      printf("Error: Directory to save trajectory files not specified.");
      filePrefix = (char*)"";
    }
  }
  p3d_specific_search(filePrefix);
  g3d_draw_allwin_active();
  fl_ringbell(0);
  fl_activate_object(SEARCH_DEL_PARAM_OBJ);
  if(ob){fl_activate_object(ob);}
  fl_set_button(SEARCH_SPECIFIC_OBJ, 0);
}


void p3d_printTrajGraphContactPdbFiles(char* filePrefix, int index, p3d_rob *robotPt){
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  char fileGraph[300], fileTraj[300], fileContacts[300];
  FILE *contacts_file;

  printf("\nSMOOTHING PATH ...\n");
  CB_start_optim_obj(NULL, 0);
  fl_set_button(ROBOTS_FORM[ir].SHOWTRAJ_OBJ, 1);
  CB_showtraj_obj(ROBOTS_FORM[ir].SHOWTRAJ_OBJ, 0);
  // identify the exhausted nodes
  p3d_identify_exhausted_nodes();

  sprintf(fileTraj, "%s/%d.trj", filePrefix, index);
  printf("Saving .trj file <%s>.\n", fileTraj);
  p3d_traj *traj = (p3d_traj *)p3d_get_desc_curid(P3D_TRAJ);
  p3d_save_traj(fileTraj, traj);

  sprintf(fileGraph, "%s/%d.graph", filePrefix, index);
  printf("Saving .graph file <%s>...\n", fileGraph);
  p3d_write_graph(XYZ_GRAPH, fileGraph);

#ifdef BIO
  // Write list of atom "contacts" along path
  if (bio_get_flag_evaluate_traj() == 1) {
    sprintf(fileContacts, "%s/%d.contacts", filePrefix, index);
    printf("Saving .contacts file <%s>...\n", fileContacts);
    if (!(contacts_file = fopen(fileContacts, "w"))) {
      printf("ERROR: can't open %s\n", fileContacts);
      return;
    }
    bio_evaluate_traj(contacts_file);
    fclose(contacts_file);
  }

  // Write PDB's
  double dep_dist = 0.2;  // MAKE IT VARIABLE !!!
  sprintf(fileGraph, "%s/%d/", filePrefix, index);
  printf("Saving .pdb files in directory <%s>...\n", fileGraph);
  mkdir(fileGraph, S_IRUSR | S_IWUSR | S_IXUSR);
  bio_write_pdbs_unifom_dep(robotPt, fileGraph, dep_dist, trajExpMinNONE);
#endif
}

/* relie deux configuration par une recherche aleatoire */
void CB_specific_search_writing_path_obj(FL_OBJECT *ob, long arg) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt qs, qg;
  int res = 0;
  int *iksols = NULL, *iksolg = NULL;
  int i, j, imin = 0, imax = 0;
  double *arraytimes;
  double mintime, maxtime, ttime, avtime;
  double std_dev, add_diff;
  int nfail = 0;
  FILE *paths_file;

  /* test_brunet=-2; */

  //  STOP = FALSE;
  p3d_SetStopValue(FALSE);
  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);


  arraytimes = MY_ALLOC(double, p3d_get_NB_specific());

  if (!(paths_file = fopen("./test_paths.trjs", "w"))) {
    printf("ERROR: can't open %s\n", "./test_paths.trjs");
    return;
  }

  /* on construit un graph ou qs et qg seront dans la meme composante connexe */
  MY_ALLOC_INFO("Avant la creation du graphe");
  for (i = 0; i < p3d_get_NB_specific(); i++) {
    if (i > 0) {
      p3d_del_graph(robotPt->GRAPH);
      if (p3d_get_RANDOM_CHOICE() == P3D_HALTON_SAMPLING) {
        p3d_init_random_seed(i);
      }
    }

#ifndef USE_CXX_PLANNER
    res = p3d_specific_learn(qs, qg, iksols, iksolg, fct_stop, fct_draw);
#else
    res = p3d_specific_learn_cxx(qs, qg, iksols, iksolg, fct_stop, fct_draw);
#endif

    if (!res) {
      nfail++;
      arraytimes[i] = -1;
      printf("p3d_specific_planner : ECHEC : il n'existe pas de chemin\n");
      //break;
    } else {
      // keep time
      arraytimes[i] = robotPt->GRAPH->time;
      // resets
      if (p3d_graph_to_traj(robotPt)) {
        g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
        g3d_draw_allwin_active();
        //save path
        fprintf(paths_file, "\n\nPATH NUM.%d\n", i);
        save_trajectory_without_header(paths_file, (traj*)p3d_get_desc_curid(P3D_TRAJ));
        ////////// BIO /////////
#ifdef BIO
        if (bio_get_flag_evaluate_traj() == 1)
          bio_evaluate_traj(NULL);
#endif
        //////////
      } else {
        printf("Problem during trajectory extraction\n");
        g3d_draw_allwin_active();
        MY_FREE(arraytimes, double, p3d_get_NB_specific());
        fclose(paths_file);
        return;
      }
    }
  }

  // print average
  printf("\n## Computing time of %d tests ##\n", i);
  //if(i<30) {
  //  printf("Too small number of tests for making average\n");
  //}
  //else {
  mintime = P3D_HUGE;
  maxtime = -P3D_HUGE;
  ttime = 0;
  for (j = 0; j < i; j++) {
    if (arraytimes[j] > 0) {
      if (arraytimes[j] < mintime) {
        imin = j;
        mintime = arraytimes[j];
      }
      if (arraytimes[j] > maxtime) {
        imax = j;
        maxtime = arraytimes[j];
      }
    }
  }
  for (j = 0; j < i; j++) {
    printf("Time %d = %f s.\n", j + 1, arraytimes[j]);
    if (arraytimes[j] > 0) {
      ttime += arraytimes[j];
    }
  }
  //avtime = ttime/((double) i - 2.0);
  avtime = ttime / ((double)(i - nfail));
  add_diff = 0.0;
  for (j = 0; j < i; j++) {
    if (arraytimes[j] > 0)
      add_diff += SQR(arraytimes[j] - avtime);
  }
  std_dev = sqrt((1.0 / (((double)(i - nfail)) - 1.0)) * add_diff);

  printf("\nAverage time   = %f s.\n", avtime);
  printf("Minimum time   = %f s.\n", mintime);
  printf("Maximum time   = %f s.\n", maxtime);
  printf("Std. deviation = %f\n", std_dev);
  printf("Num. fails     = %d\n", nfail);


  p3d_destroy_config(robotPt, qs);
  p3d_destroy_config(robotPt, qg);



  fclose(paths_file);
  MY_FREE(arraytimes, double, p3d_get_NB_specific());
  fl_ringbell(0);
  fl_set_button(SEARCH_SPECIFIC_OBJ, 0);
}



static void g3d_create_specific_search_obj(void) {
  g3d_create_button(&SEARCH_SPECIFIC_OBJ, FL_PUSH_BUTTON, 79.0, 20, "Specific Planner", (void**)&PLANNER_CHOICE, 0);
  fl_set_call_back(SEARCH_SPECIFIC_OBJ, CB_specific_search_obj, 0);

}

static void CB_optim_obj(FL_OBJECT *ob, long arg) {
  int val = fl_get_button(ob);
  if (val)
    fl_show_form(OPTIM_FORM, FL_PLACE_SIZE, TRUE, "Optimization");
  else
    fl_hide_form(OPTIM_FORM);
}

static void g3d_create_optim_obj(void) {
  g3d_create_button(&OPTIM_OBJ, FL_PUSH_BUTTON, 49.0, 30.0, "Optimize", (void**)&OPTIMIZE_FRAME, 0);
  fl_set_call_back(OPTIM_OBJ, CB_optim_obj, 0);
}

static FL_OBJECT  *INPUT_OBJ;
static FL_FORM *INPUT_FORM;
static void CB_global_input_obj(FL_OBJECT *ob, long arg) {
  p3d_set_NB_NODES(atoi(fl_get_input(INPUT_OBJ)));
  fl_hide_form(INPUT_FORM);
  fl_freeze_form(PLANNER_FORM);
  fl_set_slider_value(SEARCH_GLOBAL_PARAM_OBJ, p3d_get_NB_NODES());
  fl_unfreeze_form(PLANNER_FORM);
  fl_free_object(INPUT_OBJ);
  fl_free_form(INPUT_FORM);
}


static void CB_cancel(FL_OBJECT *ob, long arg) {
  fl_hide_form(INPUT_FORM);
  fl_free_object(INPUT_OBJ);
  fl_free_form(INPUT_FORM);
}

static void CB_create_global_input_obj(FL_OBJECT *ob, long arg) {
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX, 90.0, 75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX, 0.0, 0.0, 90.0, 75.0, "");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT, 40.0, 10.0, 40.0, 30.0,
                            "Nodes");
  fl_set_input_color(INPUT_OBJ, 0, 0);
  cancel = fl_add_button(FL_PUSH_BUTTON, 40, 45, 40, 20, "Annuler");
  fl_set_object_callback(cancel, CB_cancel, 0);
  fl_end_form();

  fl_set_object_callback(INPUT_OBJ, CB_global_input_obj, 0);

  fl_set_button(SEARCH_GLOBAL_INPUT_OBJ, 0);
  fl_show_form(INPUT_FORM, FL_PLACE_SIZE, TRUE, "");
}


static void g3d_create_global_input_obj(void) {
  g3d_create_button(&SEARCH_GLOBAL_INPUT_OBJ, FL_PUSH_BUTTON, 50, 30, "Nb\nnode", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(SEARCH_GLOBAL_INPUT_OBJ, CB_create_global_input_obj, 0);
}

static void CB_global_param_obj(FL_OBJECT *ob, long arg) {
  p3d_set_NB_NODES((int)floor(fl_get_slider_value(SEARCH_GLOBAL_PARAM_OBJ)));
}


static void g3d_create_global_param_obj(void) {
  g3d_create_valslider(&SEARCH_GLOBAL_PARAM_OBJ, FL_HOR_SLIDER, 324, 30.0, "", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_slider_step(SEARCH_GLOBAL_PARAM_OBJ, 1.0);
  fl_set_slider_bounds(SEARCH_GLOBAL_PARAM_OBJ, 1, 100000); // modif Juan
  fl_set_slider_value(SEARCH_GLOBAL_PARAM_OBJ, p3d_get_NB_NODES());
  fl_set_object_callback(SEARCH_GLOBAL_PARAM_OBJ, CB_global_param_obj, 0);
}

/*******************************************************************/

static void CB_tranbsSpePlanner_obj(FL_OBJECT *ob, long arg){
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt q[robotPt->nTransition + 2];

  for(int i = 1, j = 0; i < robotPt->nTransition + 1; i++, j++){
    q[i] = robotPt->transitionConfigs[j];
  }
  q[0] = robotPt->ROBOT_POS;
  q[robotPt->nTransition + 1] = robotPt->ROBOT_GOTO;

  for(int i = 0; i < robotPt->nTransition + 1; i++){
    robotPt->ROBOT_POS = q[i];
    robotPt->ROBOT_GOTO = q[i+1];
    CB_specific_search_obj(ob, 0);
  }
  robotPt->ROBOT_POS = q[0];
  robotPt->ROBOT_GOTO = q[robotPt->nTransition + 1];
  //concatenation des trajectoires
  for(int i = robotPt->nt - robotPt->nTransition; i < robotPt->nt; i++){
    p3d_concat_traj(robotPt->t[robotPt->nt-(robotPt->nTransition+1)], robotPt->t[i]);
  }
  robotPt->tcur = robotPt->t[robotPt->nt-(robotPt->nTransition+1)];
  g3d_refresh_allwin_active();
}

static void g3d_create_transSpePlanner_obj(void){
  g3d_create_button(&SEARCH_SPECIFIC_TRANS_OBJ,FL_NORMAL_BUTTON , 79.0, 20, "Trans Specific", (void**)&PLANNER_CHOICE, 0);
  fl_set_call_back(SEARCH_SPECIFIC_TRANS_OBJ, CB_tranbsSpePlanner_obj, 0);
}

/*******************************************************************/

/* detruit le graphe courant */
void CB_del_param_obj(FL_OBJECT *ob, long arg) {
#ifdef MULTIGRAPH
  if(p3d_get_multiGraph()){
    p3d_resetMultiGraph(XYZ_ROBOT);
  }
#endif
  p3d_del_graph(XYZ_GRAPH);
  p3d_reinit_array_exhausted_nodes();
  p3d_reinit_array_farther_nodes();
  //  XYZ_GRAPH = NULL; Modif Fabien
  MY_ALLOC_INFO("Apres destruction du graphe");
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DEL_PARAM_OBJ, 0);
}


static void g3d_create_del_param_obj(void) {
  g3d_create_button(&SEARCH_DEL_PARAM_OBJ, FL_PUSH_BUTTON, 50, 30, "Reset\ngraph", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(SEARCH_DEL_PARAM_OBJ, CB_del_param_obj, 0);
}

/*************************************************************/

/* permet de modifier le parametre DMAX */
void  CB_DMAX_param_obj(FL_OBJECT *ob, long arg) {
  double val = fl_get_slider_value(ob);

  p3d_set_DMAX(val);
}

static void g3d_create_DMAX_param_obj(void) {
  void  CB_DMAX_param_obj(FL_OBJECT *ob, long arg);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  double vmin, vmax;


  p3d_set_DMAX(p3d_calc_DMAX(r));
  vmin = p3d_get_DMAX() / 1000.;
  vmax = p3d_get_DMAX() * 5.;
  g3d_create_box(&DISTANCE_MAX, FL_UP_BOX, 50, 30, "Distance\nmax", (void**)&GRAPH_NODE_FRAME, 0);
  g3d_create_valslider(&SEARCH_DMAX_PARAM_OBJ, FL_HOR_SLIDER, 324, 30.0, "", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_slider_bounds(SEARCH_DMAX_PARAM_OBJ, vmin, vmax);
  fl_set_slider_value(SEARCH_DMAX_PARAM_OBJ, p3d_get_DMAX());
  fl_set_call_back(SEARCH_DMAX_PARAM_OBJ, CB_DMAX_param_obj, 0);
}

/***********************************************************/

/* permet de light_tracer ou non le graphe courant */
static void CB_draw_obj(FL_OBJECT *ob, long arg) {

  ENV.setBool(Env::drawGraph, !ENV.getBool(Env::drawGraph));
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DRAW_OBJ,ENV.getBool(Env::drawGraph));
  if (DRAW_GRAPH_OBJ != NULL) {
  	fl_set_button(DRAW_GRAPH_OBJ,ENV.getBool(Env::drawGraph));
  }
}

// modif Juan
static void g3d_create_draw_obj(void) {
  g3d_create_checkbutton(&SEARCH_DRAW_OBJ, FL_PUSH_BUTTON, -1, 30, "", (void**)&FRAME_OBJ, 0);
  fl_set_object_color(SEARCH_DRAW_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SEARCH_DRAW_OBJ, CB_draw_obj, 0);
  fl_set_button(SEARCH_DRAW_OBJ, 0);
}

/***********************************************************/
// allows to choice the jnt which frame is drawn in the graph

static int user_drawnjnt = -1;

int p3d_get_user_drawnjnt(void) {
  return(user_drawnjnt);
}
void p3d_set_user_drawnjnt(int jnt) {
  user_drawnjnt = jnt;
}

static void CB_drawnjnt_obj(FL_OBJECT *ob, long arg) {
  p3d_set_user_drawnjnt(atoi(fl_get_input(INPUT_OBJ)));

  fl_hide_form(INPUT_FORM);

  fl_free_object(INPUT_OBJ);
  fl_free_form(INPUT_FORM);
}

void CB_create_drawnjnt_obj(FL_OBJECT *ob, long arg) {
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX, 90.0, 75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX, 0.0, 0.0, 90.0, 75.0, "");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT, 40.0, 10.0, 40.0, 30.0,
                            "Index\nJNT");
  fl_set_input_color(INPUT_OBJ, 0, 0);
  cancel = fl_add_button(FL_PUSH_BUTTON, 40, 45, 40, 20, "Annuler");
  fl_set_object_callback(cancel, CB_cancel, 0);
  fl_end_form();

  fl_set_object_callback(INPUT_OBJ, CB_drawnjnt_obj, 0);

  fl_set_button(DRAWNJNT_OBJ, 0);
  fl_show_form(INPUT_FORM, FL_PLACE_SIZE, TRUE, "");
}


static void g3d_create_drawnjnt_obj(void) {
  g3d_create_button(&DRAWNJNT_OBJ, FL_PUSH_BUTTON, 55, 20, "Draw", (void**)&FRAME_OBJ, 0);
  fl_set_object_callback(DRAWNJNT_OBJ, CB_create_drawnjnt_obj, 0);
}

/***********************************************************/
// fmodif Juan


/* permet d'imprimer ou non des infos sur le graphe courant */
/*modif Mokhtar dessin du graph en utilisant "dot"*/
/*On commence par ecrire de .dot et on genere un .ps par la suite.*/
void CB_print_obj(FL_OBJECT *ob, long arg) {
  p3d_graph *G = XYZ_GRAPH;
  p3d_compco *comp;
  p3d_node *Ncomp = NULL;
  p3d_list_edge *edge;
  p3d_list_node *list_node;

  FILE *dotFile = NULL;
  char dotNameDir[255];
  char dotNameFile[255];
  char dot2ps[255];
  int i = 0, found = 0, j = 0, nbEdges;
  int *graphNodeList;

  sprintf(dotNameDir, "%s/video/", getenv("HOME_MOVE3D"));
  sprintf(dotNameFile, "%sgraph.dot", dotNameDir);
  dotFile = fopen(dotNameFile, "w");
  if (dotFile != NULL) {
    fprintf(dotFile, "digraph G {\n");
    if (XYZ_GRAPH) {
      nbEdges = XYZ_GRAPH->nedge;
      graphNodeList = (int *)malloc(nbEdges * 2 * sizeof(int));
      if (!XYZ_GRAPH->oriented) {
        fprintf(dotFile, "edge [arrowhead=normal,arrowtail=normal];\n");// edges <--->
      }//sinon edges --->
      printf("Composantes connexes :\n");
      comp = G->comp;
      while (comp != NULL) {
        list_node = comp->dist_nodes;
        printf("%d ; nombre de noeuds  %d : ", comp->num, comp->nnode);
        while (list_node != NULL) {
          Ncomp = list_node->N;
          printf(" %d(%d", Ncomp->num, Ncomp->nedge);
          if (Ncomp->nedge > 0) {
            printf(" :");
            edge = Ncomp->edges;
            while (edge != NULL) {
              for (j = 0, found = 0; j < nbEdges; j++) {
                if ((edge->E->Ni->num == graphNodeList[j] && edge->E->Nf->num == graphNodeList[j+nbEdges]) || (edge->E->Ni->num == graphNodeList[j+nbEdges] && edge->E->Nf->num == graphNodeList[j])) {
                  found = 1;
                }
              }
              if (!found) {
                graphNodeList[i] = edge->E->Ni->num;
                graphNodeList[i+nbEdges] = edge->E->Nf->num;
                if (edge->E->Ni->iksol != NULL) {
                  fprintf(dotFile, "\t%d [label=\"%d(%d)\\n ", edge->E->Nf->num, edge->E->Nf->num, edge->E->Nf->numcomp);
                  if(p3d_get_ik_choice() != IK_NORMAL){
                    for (j = 0; j < G->rob->cntrt_manager->ncntrts; j++) {
                      fprintf(dotFile, "%d", edge->E->Ni->iksol[j]);
                      if (j < G->rob->cntrt_manager->ncntrts - 1) {
                        fprintf(dotFile, " ");
                      } else {
                        fprintf(dotFile, "\"");
                      }
                    }
                    if (edge->E->Ni->isSingularity == TRUE) {
                      fprintf(dotFile, ", color=red, fontcolor=red");
                    }

                  }
                  fprintf(dotFile, "];\n");

                  fprintf(dotFile, "\t%d [label=\"%d(%d)\\n ", edge->E->Nf->num, edge->E->Nf->num, edge->E->Nf->numcomp);
                  if(p3d_get_ik_choice() != IK_NORMAL){
                    for (j = 0; j < G->rob->cntrt_manager->ncntrts; j++) {
                      fprintf(dotFile, "%d", edge->E->Nf->iksol[j]);
                      if (j < G->rob->cntrt_manager->ncntrts - 1) {
                        fprintf(dotFile, " ");
                      } else {
                        fprintf(dotFile, "\"");
                      }
                    }
                    if (edge->E->Nf->isSingularity == TRUE) {
                      fprintf(dotFile, ", color=red, fontcolor=red");
                    }
                  }
                  fprintf(dotFile, "];\n");
                }else{
                  fprintf(dotFile, "\t%d [label=\"%d(%d)\"", edge->E->Ni->num, edge->E->Ni->num, edge->E->Ni->numcomp);
#ifdef MULTIGRAPH
                  if (edge->E->Ni->needMgCycle == TRUE) {
                    fprintf(dotFile, ", color=red, fontcolor=red");
                  }
#endif
                  fprintf(dotFile, "];\n");
                  fprintf(dotFile, "\t%d [label=\"%d(%d)\" ", edge->E->Nf->num, edge->E->Nf->num, edge->E->Nf->numcomp);
#ifdef MULTIGRAPH
                  if (edge->E->Nf->needMgCycle == TRUE) {
                    fprintf(dotFile, ", color=red, fontcolor=red");
                  }
#endif
                  fprintf(dotFile, "];\n");
                }
                fprintf(dotFile, "\t%d -> %d [label=\"", edge->E->Ni->num, edge->E->Nf->num);
                if(p3d_get_ik_choice() != IK_NORMAL){
                  int *ikSol = NULL;
                  p3d_get_non_sing_iksol(G->rob->cntrt_manager, edge->E->Ni->iksol, edge->E->Nf->iksol, &ikSol);
                  for (j = 0; j < G->rob->cntrt_manager->ncntrts; j++) {
                    fprintf(dotFile, "%d", ikSol[j]);
                    if (j < G->rob->cntrt_manager->ncntrts - 1) {
                      fprintf(dotFile, " ");
                    } else {
                      fprintf(dotFile, "\"");
                    }
                  }
                }else{
                  fprintf(dotFile, "\"");
                }
                if(edge->E->unvalid == TRUE){
                  fprintf(dotFile, ", color=green");
                }
                // #ifdef MULTIGRAPH
                //                 if (edge->E->for_cycle == TRUE) {
                //                   fprintf(dotFile, ", color=red");
                //                 }
                // #endif
                fprintf(dotFile, "];\n");
                i++;
              }
              printf(" [%d %d]", edge->E->Ni->num, edge->E->Nf->num);
              edge = edge->next;
            }
          }else{
            fprintf(dotFile, "\t%d [label=\"%d(%d)\"]\n", Ncomp->num, Ncomp->num, Ncomp->numcomp);
          }
          printf(")");
          list_node = list_node->next;
        }
        printf("\n");
        comp = comp->suiv;
      }
      free(graphNodeList);
    }
    fprintf(dotFile, "}\n");
    fclose(dotFile);
    sprintf(dot2ps, "dot -Tps %sgraph.dot -o %sgraph.ps", dotNameDir, dotNameDir);
    system(dot2ps);
    sprintf(dot2ps, "rm %sgraph.dot", dotNameDir);
  }
//   system(dot2ps);
  //printf("on met le bouton a 0\n");
  fl_set_button(SEARCH_PRINT_OBJ, 0);
}


static void g3d_create_print_obj(void) {
  g3d_create_checkbutton(&SEARCH_PRINT_OBJ, FL_PUSH_BUTTON, -1, 30, "Print", (void**)&FRAME_OBJ, 0);
  fl_set_object_color(SEARCH_PRINT_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SEARCH_PRINT_OBJ, CB_print_obj, 0);
}
/************************************************************/

static void CB_saveInfoInFile_obj(FL_OBJECT *ob, long arg) {
  p3d_set_saveInfoInFile(!p3d_get_saveInfoInFile());
}

static void g3d_create_saveInfo_obj(void) {
  g3d_create_checkbutton(&SAVE_INFO_OBJ, FL_PUSH_BUTTON, -1, 30, "Save Info", (void**)&FRAME_OBJ, 0);
  fl_set_object_color(SAVE_INFO_OBJ, FL_MCOL, FL_GREEN);
  fl_set_button(SAVE_INFO_OBJ, p3d_get_saveInfoInFile());
  fl_set_call_back(SAVE_INFO_OBJ, CB_saveInfoInFile_obj, 0);
}
/************************************************************/
#ifdef MULTIGRAPH
static void CB_multiGraph_obj(FL_OBJECT *ob, long arg) {
  p3d_set_multiGraph(!p3d_get_multiGraph());
}

static void g3d_create_multiGraph_obj(void) {
  g3d_create_checkbutton(&MULTIGRAPH_OBJ, FL_PUSH_BUTTON, -1, 30, "Multi Graph", (void**)&FRAME_OBJ, 0);
  fl_set_object_color(MULTIGRAPH_OBJ, FL_MCOL, FL_GREEN);
  fl_set_button(MULTIGRAPH_OBJ, p3d_get_multiGraph());
  fl_set_call_back(MULTIGRAPH_OBJ, CB_multiGraph_obj, 0);
}
#endif
/************************************************************/
static void CB_ik_choice(FL_OBJECT *ob, long arg) {
  p3d_set_ik_choice((int)arg);
}

static int CB_ikForm_OnClose(FL_FORM *form, void *arg) {
  //Call the fonction closing the form.
  g3d_delete_ik_form();
  fl_set_button(IK_BUTTON, 0);//release the button path_Deformation on planner FORM
  //If we return FL_OK, the application will continue to try to shut down itself
  //   if however we return FL_IGNORE, the application will not continue this event
  return FL_IGNORE;
}

static void CB_ik_draw_obj(FL_OBJECT *ob, long arg) {
  int i = 0, counter = 0, value = 0;

  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  for (i = 0; i < robot->cntrt_manager->ncntrts;i++) {
    if ((robot->cntrt_manager->cntrts[i])->nbSol > 1) {
      if (counter++ == arg) {
        counter--;
        break;
      }
    }
  }
  value = atoi(fl_get_input(ob));
  if (value <= (robot->cntrt_manager->cntrts[i])->nbSol) { //if 0 show all
    p3d_set_ik_draw(i, value);
  }
  if (ENV.getBool(Env::drawGraph)) {
    g3d_draw_allwin_active();
  }
}

static void g3d_create_ik_form(void) {
  p3d_rob *robot = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
  int nbMulti = 0, multiOffset = 0, i = 0, counter = 0, *ikSol = NULL;
  char buffer [10];

  if ((nbMulti = p3d_is_multisol(robot->cntrt_manager))) {
    multiOffset = 30 + nbMulti * 40;
  }
  g3d_create_form(&IK_FORM, 225, 45 + multiOffset, FL_UP_BOX);
  g3d_create_labelframe(&IK_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Ik", (void**)&IK_FORM, 1);
  IK_GROUP = fl_bgn_group();
  g3d_create_checkbutton(&NORMAL_CB, FL_RADIO_BUTTON, -1, 20, "Normal", (void**)&IK_FRAME, 0);
  fl_set_object_color(NORMAL_CB, FL_MCOL, FL_GREEN);
  fl_set_call_back(NORMAL_CB, CB_ik_choice, (long)IK_NORMAL);

  g3d_create_checkbutton(&UNIQUE_CB, FL_RADIO_BUTTON, -1, 20, "Unique", (void**)&IK_FRAME, 0);
  fl_set_object_color(UNIQUE_CB, FL_MCOL, FL_GREEN);
  fl_set_call_back(UNIQUE_CB, CB_ik_choice, (long)IK_UNIQUE);

  g3d_create_checkbutton(&MULTISOL_CB, FL_RADIO_BUTTON, -1, 20, "Multisol", (void**)&IK_FRAME, 0);
  fl_set_object_color(MULTISOL_CB, FL_MCOL, FL_GREEN);
  fl_set_call_back(MULTISOL_CB, CB_ik_choice, (long)IK_MULTISOL);
  //IK_GROUP = 
  fl_end_group();
  switch (p3d_get_ik_choice()) {
    case(IK_NORMAL): {
      fl_set_button(NORMAL_CB, 1);
      break;
    }
    case(IK_UNIQUE): {
      fl_set_button(UNIQUE_CB, 1);
      break;
    }
    case(IK_MULTISOL): {
      fl_set_button(MULTISOL_CB, 1);
      break;
    }
  }

  if (nbMulti != 0) {//if there is constraint with multisolutions
    g3d_create_labelframe(&IK_DRAW_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Draw", (void**)&IK_FORM, 1);
    IK_DRAW = (FL_OBJECT**) malloc(nbMulti * sizeof(FL_OBJECT*));
    for (i = 0; i < robot->cntrt_manager->ncntrts; i++) {
      if ((robot->cntrt_manager->cntrts[i])->nbSol > 1) {
        sprintf(buffer, "IkSol %d", counter);
        g3d_create_input(&IK_DRAW[counter], FL_NORMAL_INPUT, -1, 27, buffer, (void**)&IK_DRAW_FRAME, 0);
        ikSol = p3d_get_ik_draw();
        sprintf(buffer, "%d", ikSol[counter]);
        fl_set_input(IK_DRAW[counter], buffer);
        fl_set_call_back(IK_DRAW[counter], CB_ik_draw_obj, counter);
        counter++;
      }
    }
  }

  fl_end_form();
  fl_set_form_icon(IK_FORM, GetApplicationIcon(), 0);
  fl_set_form_atclose(IK_FORM, CB_ikForm_OnClose, NULL);
}

static void CB_ik_button(FL_OBJECT *ob, long arg) {
  if (fl_get_button(ob)) {
    g3d_create_ik_form();
    fl_show_form(IK_FORM, FL_PLACE_SIZE, TRUE, "Inverse Kinematics");
  } else {
    g3d_delete_ik_form();
  }
}

static void g3d_create_ik_button(void) {
  g3d_create_button(&IK_BUTTON, FL_PUSH_BUTTON, 50, 20, "Inv. Kin.", (void**)&FRAME_OBJ, 0);
  fl_set_object_callback(IK_BUTTON, CB_ik_button, 0);
  p3d_init_ik_draw_current();
}

/************************************************************/

/* permet de modifier le parametre COMP_NODES */
static void CB_compco_input_obj(FL_OBJECT *ob, long arg) {
	ENV.setInt(Env::maxNodeCompco,atoi(fl_get_input(INPUT_OBJ)));
  fl_hide_form(INPUT_FORM);

  fl_freeze_form(PLANNER_FORM);
  fl_set_slider_value(SEARCH_COMPCO_PARAM_OBJ, p3d_get_COMP_NODES());
  fl_unfreeze_form(PLANNER_FORM);

  fl_free_object(INPUT_OBJ);
  fl_free_form(INPUT_FORM);
}

static void CB_create_compco_input_obj(FL_OBJECT *ob, long arg) {
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX, 90.0, 75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX, 0.0, 0.0, 90.0, 75.0, "");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT, 40.0, 10.0, 40.0, 30.0,
                            "Nodes");
  fl_set_input_color(INPUT_OBJ, 0, 0);
  cancel = fl_add_button(FL_PUSH_BUTTON, 40, 45, 40, 20, "Annuler");
  fl_set_object_callback(cancel, CB_cancel, 0);
  fl_end_form();

  fl_set_object_callback(INPUT_OBJ, CB_compco_input_obj, 0);

  fl_set_button(SEARCH_COMPCO_INPUT_OBJ, 0);
  fl_show_form(INPUT_FORM, FL_PLACE_SIZE, TRUE, "");
}

static void g3d_create_compco_input_obj(void) {
  g3d_create_button(&SEARCH_COMPCO_INPUT_OBJ, FL_PUSH_BUTTON, 50, 30, "Nb node\n/Comp.co.", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(SEARCH_COMPCO_INPUT_OBJ, CB_create_compco_input_obj, 0);
}

static void CB_compco_param_obj(FL_OBJECT *ob, long arg) {
	ENV.setInt(Env::maxNodeCompco,floor(fl_get_slider_value(SEARCH_COMPCO_PARAM_OBJ)));
}


static void g3d_create_compco_param_obj(void) {
  g3d_create_valslider(&SEARCH_COMPCO_PARAM_OBJ, FL_HOR_SLIDER, 324, 30.0, "", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_slider_step(SEARCH_COMPCO_PARAM_OBJ, 1.0);
  fl_set_slider_bounds(SEARCH_COMPCO_PARAM_OBJ, 1, 100000); // modif Juan
  fl_set_slider_value(SEARCH_COMPCO_PARAM_OBJ, p3d_get_COMP_NODES());
  fl_set_object_callback(SEARCH_COMPCO_PARAM_OBJ, CB_compco_param_obj, 0);
}

/************************************************************/
/* permet de light_tracer la courbe a optimiser */
static void CB_draw_optim_obj(FL_OBJECT *ob, long arg) {
  ENV.setBool(Env::drawTraj, !ENV.getBool(Env::drawTraj));
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DRAW_OPTIM_OBJ, ENV.getBool(Env::drawTraj));
}

static void g3d_create_draw_optim_obj(void) {
  g3d_create_checkbutton(&SEARCH_DRAW_OPTIM_OBJ, FL_PUSH_BUTTON, -1, 30, "Draw", (void**)&TRAJECTORY_FRAME, 0);
  fl_set_object_color(SEARCH_DRAW_OPTIM_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SEARCH_DRAW_OPTIM_OBJ, CB_draw_optim_obj, 0);
}

/************************************************************/
/* permet d'afficher la light_trace de la courbe courante de tous les robots de la scene*/
static void CB_draw_light_trace_obj(FL_OBJECT *ob, long arg) {
  G3D_DRAW_TRACE = !G3D_DRAW_TRACE;
  g3d_draw_allwin_active();
  fl_set_button(DRAW_LIGHT_TRACE_OBJ, G3D_DRAW_TRACE);
}

static void g3d_create_draw_light_trace_obj(void) {
  g3d_create_checkbutton(&DRAW_LIGHT_TRACE_OBJ, FL_PUSH_BUTTON, -1, 30, "Light\ntrace", (void**)&TRAJECTORY_FRAME, 0);
  fl_set_object_color(DRAW_LIGHT_TRACE_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(DRAW_LIGHT_TRACE_OBJ, CB_draw_light_trace_obj, 0);
}

static void CB_draw_trace_obj(FL_OBJECT *ob, long arg) {
  g3d_set_win_drawer(G3D_WIN, g3d_draw_trace);
  g3d_draw_allwin_active();
  g3d_set_win_drawer(G3D_WIN, g3d_draw);
}

static void g3d_create_draw_trace_obj(void) {
  g3d_create_button(&DRAW_TRACE_OBJ, FL_NORMAL_BUTTON, 50, 20, "Trace", (void**)&TRAJECTORY_FRAME, 0);
  fl_set_call_back(DRAW_TRACE_OBJ, CB_draw_trace_obj, 0);
}

/***************************************************************/

static void CB_stat_obj(FL_OBJECT *ob, long arg){
  if (getStatStatus()){
    disableStats();
  }else{
    if(XYZ_GRAPH){
      if(!XYZ_GRAPH->stat){
        XYZ_GRAPH->stat = createStat();
      }
    }
    if(!XYZ_ENV->stat){
      XYZ_ENV->stat = createStat();
    }
    enableStats();
  }
  fl_set_button(STAT_OBJ,getStatStatus());
}

static void CB_stat_print_obj(FL_OBJECT *ob, long arg){
  if(XYZ_GRAPH->stat){
    PrintInfo(("############# Graph Stat #############\n"));
    setTotalCountVar(XYZ_GRAPH);
    printStatsGraph(XYZ_GRAPH->stat, 1);
  }
  if(XYZ_ENV->stat){
    PrintInfo(("############# Env Stat #############\n"));
    printStatsEnv(XYZ_ENV->stat , 1);
  }
}

static void g3d_create_stat_obj(void){
  g3d_create_labelframe(&STAT_FRAME_OBJ, FL_ENGRAVED_FRAME, -1, -1, "Stats", (void**)&OPTIMIZE_FRAME, 0);
  g3d_create_checkbutton(&STAT_OBJ, FL_PUSH_BUTTON, -1, -1, "Enabled", (void**)&STAT_FRAME_OBJ, 0);
  fl_set_call_back(STAT_OBJ, CB_stat_obj, 0);
  fl_set_button(STAT_OBJ,getStatStatus());
  fl_set_object_color(STAT_OBJ, FL_MCOL, FL_GREEN);
  g3d_create_button(&STAT_PRINT_OBJ, FL_NORMAL_BUTTON, 55, 15, "Print stats", (void**)&STAT_FRAME_OBJ, 0);
  fl_set_call_back(STAT_PRINT_OBJ, CB_stat_print_obj, 0);
}
/***************************************************************/

static void CB_save_mult_obj(FL_OBJECT *ob, long arg) {
  G3D_SAVE_MULT = !G3D_SAVE_MULT;
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DRAW_MULT_OBJ, G3D_SAVE_MULT);
}

static void g3d_create_save_mult_obj(void) {
  g3d_create_checkbutton(&SEARCH_DRAW_MULT_OBJ, FL_PUSH_BUTTON, -1, 30, "Save", (void**)&TRAJECTORY_FRAME, 0);
  fl_set_object_color(SEARCH_DRAW_MULT_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SEARCH_DRAW_MULT_OBJ, CB_save_mult_obj, 0);
  fl_set_button(SEARCH_DRAW_MULT_OBJ, ENV.getBool(Env::drawTraj));
}
/***************************************************************/

/* permet de sauvegarder le graphe courant */
static void CB_save_graph_obj(FL_OBJECT *ob, long arg) {
  char TERM[20], DEF_FILE[200], c_dir_name[300];
  char *env_name;
  const char *file;
  int c_sz = 0, i;
  pp3d_rob robotPt;

  if (!XYZ_GRAPH) {
    printf("save_graph : ERREUR : no current graph...\n");
  } else { /* Dut modification Fabien */
    if ((XYZ_GRAPH == NULL) || (XYZ_GRAPH->file == NULL)) {
      env_name = p3d_get_desc_curname(P3D_ENV);
      robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
      strcpy(DEF_FILE, env_name);
      strcat(DEF_FILE, ".");
      strcat(DEF_FILE, robotPt->name);
      strcpy(TERM, ".graph");
      strcat(DEF_FILE, TERM);

      p3d_get_directory(c_dir_name);
      c_sz = (int) strlen(c_dir_name);
      if (c_dir_name[c_sz-1] != '/') {
        strcat(c_dir_name, "/");
      }
      strcat(c_dir_name, "GRAPH");
    } else {
      strcpy(c_dir_name, XYZ_GRAPH->file);
      c_sz = (int) strlen(c_dir_name);
      /* On s�are le nom du fichier de celui du r�ertoire  */
      for (i = c_sz - 1; (i >= 0) && (c_dir_name[i] != '/'); i--);
      strcpy(DEF_FILE, c_dir_name + i + 1);
      c_dir_name[i+1] = '\0';
    }
    /* Fin modification Fabien */
    file = fl_show_fselector("P3D_GRAPH filename", c_dir_name, "*.graph", DEF_FILE);

    if (file) {
//       p3d_write_graph(XYZ_GRAPH, (char *)file);
#ifdef MULTIGRAPH
      if(p3d_get_multiGraph()){
        if(XYZ_GRAPH->rob->mg->fsg != NULL){
          p3d_writeGraph(XYZ_ROBOT->mg, (char *)file, MGGRAPH);//Mokhtar Using XML Format
        }else{
          p3d_writeGraph(XYZ_GRAPH, (char *)file, DEFAULTGRAPH);//Mokhtar Using XML Format
        }
      }else{
#endif
        p3d_writeGraph(XYZ_GRAPH, (char *)file, DEFAULTGRAPH);//Mokhtar Using XML Format
#ifdef MULTIGRAPH
      }
#endif
    }
  }
  fl_set_button(SAVE_GRAPH_OBJ, 0);
}

static void g3d_create_save_graph_obj(void) {
  g3d_create_button(&SAVE_GRAPH_OBJ, FL_PUSH_BUTTON, 50, 30, "Save\nGraph", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(SAVE_GRAPH_OBJ, CB_save_graph_obj, 0);
}

/* permet de charger un grave sauve dans le graphe courant */
static void CB_load_graph_obj(FL_OBJECT *ob, long arg) {
  char TERM[20], DEF_FILE[200], c_dir_name[200];
  char *env_name;
  const char *file;
  int  c_sz = 0, i;
  pp3d_rob robotPt;

  if (XYZ_GRAPH) {
    printf("load_graph : ATTENTION : le graphe courant sera ecrase...\n");
  }

  /* Debut modification Fabien */
  // p3d_del_graph(XYZ_GRAPH);
  // XYZ_GRAPH = NULL;

  if ((XYZ_GRAPH == NULL) || (XYZ_GRAPH->file == NULL)) {
    env_name = p3d_get_desc_curname(P3D_ENV);
    robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
    strcpy(DEF_FILE, env_name);
    strcat(DEF_FILE, ".");
    strcat(DEF_FILE, robotPt->name);
    strcpy(TERM, ".graph");
    strcat(DEF_FILE, TERM);

    p3d_get_directory(c_dir_name);
    c_sz = (int) strlen(c_dir_name);
    if (c_dir_name[c_sz-1] != '/') {
      strcat(c_dir_name, "/");
    }
    strcat(c_dir_name, "GRAPH");
  } else {
    strcpy(c_dir_name, XYZ_GRAPH->file);
    c_sz = (int) strlen(c_dir_name);
    /* On separe le nom du fichier de celui du repertoire  */
    for (i = c_sz - 1; (i >= 0) && (c_dir_name[i] != '/'); i--);
    strcpy(DEF_FILE, c_dir_name + i + 1);
    c_dir_name[i+1] = '\0';
  }
  /* Fin modification Fabien */

  file = fl_show_fselector("P3D_GRAPH filename", c_dir_name, "*.graph", DEF_FILE);

  if (file) {
    if (!p3d_readGraph((char *) file, DEFAULTGRAPH)){
      printf("Trying the old Format\n");
      p3d_read_graph((char *) file);  // Modif Fabien
    }
  }

  g3d_draw_allwin_active();

  MY_ALLOC_INFO("Apres lecture du graphe");

  fl_set_button(LOAD_GRAPH_OBJ, 0);
}

static void g3d_create_load_graph_obj(void) {
  g3d_create_button(&LOAD_GRAPH_OBJ, FL_NORMAL_BUTTON, 50, 30, "Load\nGraph", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(LOAD_GRAPH_OBJ, CB_load_graph_obj, 0);
}

/******************************************************************/

void CB_graph_in_grid_obj(FL_OBJECT *ob, long arg) {
  p3d_put_graph_in_grid_3d(XYZ_GRAPH);
  //test_grid();
  p3d_write_curr_grid3d_in_geomview_file();
  fl_set_button(GRAPH_IN_GRID_OBJ, 0);
}

static void g3d_create_graph_in_grid_obj(void) {
  g3d_create_button(&GRAPH_IN_GRID_OBJ, FL_PUSH_BUTTON, 55, 30, "Put Graph\n in Grid", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(GRAPH_IN_GRID_OBJ, CB_graph_in_grid_obj, 0);
}

/******************************************************************/

/* permet de stoper la creation de noeuds dans le noeud courant */
void CB_stop_obj(FL_OBJECT *ob, long arg) {
  //STOP = TRUE;
  p3d_SetStopValue(TRUE);
  fl_set_button(STOP_OBJ, 0);
}

static void g3d_create_stop_obj(void) {
  g3d_create_button(&STOP_OBJ, FL_PUSH_BUTTON, 45, 30, "Stop", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(STOP_OBJ, CB_stop_obj, 0);
}

/*********************************************************/

/* cree le menu d'expansion des noeuds */
static void CB_expand_obj(FL_OBJECT *ob, long arg) {
  FL_FORM *EXPAND_MENU;
  FL_OBJECT *obj, *ok, *cancel, *choice1, *choice2, *node,
  *nbnode, *glob, *nbwalk;
  int NNODE = 0, i;
  p3d_list_node *list_node;
  p3d_node *Nex = NULL;

  //  STOP = FALSE;
  p3d_SetStopValue(FALSE);

  EXPAND_MENU = fl_bgn_form(FL_FLAT_BOX, 230.0, 145.0);

  obj = fl_add_box(FL_UP_BOX, 0.0, 0.0, 230.0, 145.0, "");

  obj = fl_add_frame(FL_ENGRAVED_FRAME, 110, 20, 110, 60, "");
  obj = fl_add_box(FL_FLAT_BOX, 125, 15, 80, 10, "Expand type");

  choice1 = fl_add_checkbutton(FL_RADIO_BUTTON, 120, 30, 55, 30, "Centered box");
  fl_set_object_color(choice1, FL_MCOL, FL_GREEN);
  fl_set_button(choice1, 1);
  choice2 = fl_add_checkbutton(FL_RADIO_BUTTON, 120, 50, 55, 30, "Random walk");
  fl_set_object_color(choice2, FL_MCOL, FL_GREEN);

  node = fl_add_input(FL_NORMAL_INPUT, 50.0, 10.0, 40.0, 20.0,
                      "No node");
  fl_set_input_color(node, 0, 0);

  nbnode = fl_add_input(FL_NORMAL_INPUT, 50.0, 40.0, 40.0, 20.0,
                        "Nb node");
  fl_set_input_color(nbnode, 0, 0);

  nbwalk = fl_add_input(FL_NORMAL_INPUT, 50.0, 70.0, 40.0, 20.0,
                        "Nb walk");
  fl_set_input_color(nbnode, 0, 0);


  ok = fl_add_button(FL_PUSH_BUTTON, 10, 110, 40, 20, "OK");
  cancel = fl_add_button(FL_PUSH_BUTTON, 60, 110, 40, 20, "Annuler");

  glob = fl_add_button(FL_PUSH_BUTTON, 140, 110, 80, 20, "Global expand");

  fl_end_form();

  fl_show_form(EXPAND_MENU, FL_PLACE_SIZE, TRUE, "Expand");
  while (1) {
    obj = fl_do_forms();
    if (obj == ok) {

      fl_hide_form(EXPAND_MENU);

      NNODE = atoi(fl_get_input(node));
      p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));

      if (NNODE > XYZ_GRAPH->nnode) {
        printf("PAS ASSEZ DE NOEUDS !\n");
        return;
      }
      list_node = XYZ_GRAPH->nodes;
      for (i = 1;i <= XYZ_GRAPH->nnode;i++) {
        if (list_node->N->num == NNODE) {
          Nex = list_node->N;
          break;
        } else {
          list_node = list_node->next;
        }
      }

      p3d_APInode_expand(XYZ_GRAPH, Nex, fct_stop, fct_draw);

      fl_set_button(EXPAND_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(node);
      fl_free_object(nbnode);
      fl_free_object(nbwalk);
      fl_free_object(glob);
      fl_free_form(EXPAND_MENU);
      return;
    }
    if (obj == glob) {

      fl_hide_form(EXPAND_MENU);

      p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));

      p3d_expand_graph(XYZ_GRAPH, 0.1, fct_stop, fct_draw);

      fl_set_button(EXPAND_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(node);
      fl_free_object(nbnode);
      fl_free_object(nbwalk);
      fl_free_object(glob);
      fl_free_form(EXPAND_MENU);
      return;
    }

    if (obj == cancel) {
      fl_hide_form(EXPAND_MENU);
      fl_set_button(EXPAND_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(nbnode);
      fl_free_object(nbwalk);
      fl_free_object(node);
      fl_free_object(glob);
      fl_free_form(EXPAND_MENU);
      return;
    }
    if (obj == choice1) {
      p3d_set_EXPAND_CHOICE(P3D_EXPAND_BOX);
    }
    if (obj == choice2) {
      p3d_set_EXPAND_CHOICE(P3D_EXPAND_RAND_WALK);
    }
  }
}

static void g3d_create_expand_obj(void) {
  g3d_create_button(&EXPAND_OBJ, FL_PUSH_BUTTON, 45, 30, "Expand\nnode", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(EXPAND_OBJ, CB_expand_obj, 0);
}

/*****************************************************************/

/* lance la creation de NB_NODES noeuds isoles */
static void CB_isolate_obj(FL_OBJECT *ob, long arg) {
  //  STOP = FALSE;
  p3d_SetStopValue(FALSE);
  p3d_create_orphans(p3d_get_NB_NODES(), fct_stop, fct_draw);

  fl_set_button(ISOLATE_OBJ, 0);
}


static void g3d_create_isolate_obj(void) {
  g3d_create_button(&ISOLATE_OBJ, FL_PUSH_BUTTON, 45, 30, "Create\nisolated", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(ISOLATE_OBJ, CB_isolate_obj, 0);

}

/* lance la creation de NB_NODES noeuds "liants" */
static void CB_linking_obj(FL_OBJECT *ob, long arg) {
  //  STOP = FALSE;
  p3d_SetStopValue(FALSE);
  p3d_create_linking(p3d_get_NB_NODES(), fct_stop, fct_draw);

  fl_set_button(LINKING_OBJ, 0);
}

static void g3d_create_linking_obj(void) {
  g3d_create_button(&LINKING_OBJ, FL_PUSH_BUTTON, 45, 30, "Create\nlinking", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(LINKING_OBJ, CB_linking_obj, 0);

}



/* lance le test d'un graphe */
static void CB_test_obj(FL_OBJECT *ob, long arg) {
  FL_FORM *TEST_MENU;
  FL_OBJECT *obj, *ok, *cancel, *choice1, *choice2, *nbtest;
  int NBTEST, TEST_CHOICE = 1;
  double success = 0.;

  //  STOP = FALSE;
  p3d_SetStopValue(FALSE);
  TEST_MENU = fl_bgn_form(FL_FLAT_BOX, 230.0, 110.0);

  obj = fl_add_box(FL_UP_BOX, 0.0, 0.0, 230.0, 110.0, "");

  obj = fl_add_frame(FL_ENGRAVED_FRAME, 110, 20, 110, 80, "");
  obj = fl_add_box(FL_FLAT_BOX, 125, 15, 80, 10, "Test type");

  choice1 = fl_add_checkbutton(FL_RADIO_BUTTON, 120, 30, 55, 30, "Test connexity");
  fl_set_object_color(choice1, FL_MCOL, FL_GREEN);
  fl_set_button(choice1, 1);
  choice2 = fl_add_checkbutton(FL_RADIO_BUTTON, 120, 50, 55, 30, "Test covering");
  fl_set_object_color(choice2, FL_MCOL, FL_GREEN);

  nbtest = fl_add_input(FL_NORMAL_INPUT, 50.0, 20.0, 40.0, 20.0,
                        "Nb test");
  fl_set_input_color(nbtest, 0, 0);

  ok = fl_add_button(FL_PUSH_BUTTON, 10, 80, 40, 20, "OK");
  cancel = fl_add_button(FL_PUSH_BUTTON, 60, 80, 40, 20, "Annuler");

  fl_end_form();

  fl_show_form(TEST_MENU, FL_PLACE_SIZE, TRUE, "Test");
  while (1) {
    obj = fl_do_forms();
    if (obj == ok) {

      if (!XYZ_GRAPH) {
        printf("test : ERREUR : pas de graphe courant\n");
        fl_hide_form(TEST_MENU);
        fl_set_button(TEST_OBJ, 0);
        fl_free_object(cancel);
        fl_free_object(ok);
        fl_free_object(choice1);
        fl_free_object(choice2);
        fl_free_object(nbtest);
        fl_free_form(TEST_MENU);
        return;
      }

      fl_hide_form(TEST_MENU);

      NBTEST = atoi(fl_get_input(nbtest));

      switch (TEST_CHOICE) {
        case 1:
          success = p3d_test_quality_graph(XYZ_GRAPH, NBTEST);
          break;

        case 2:
          success = p3d_test_good_graph(XYZ_GRAPH, NBTEST);
          break;

        default:
          printf("WRONG TYPE OF TEST\n");
          break;
      }

      printf("Taux de reussite : %f\n", 100*success);

      fl_set_button(TEST_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(nbtest);
      fl_free_form(TEST_MENU);
      return;
    }

    if (obj == cancel) {
      fl_hide_form(TEST_MENU);
      fl_set_button(TEST_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(nbtest);
      fl_free_form(TEST_MENU);
      return;
    }
    if (obj == choice1) {
      TEST_CHOICE = 1;
    }
    if (obj == choice2) {
      TEST_CHOICE = 2;
    }

  }
}

static void g3d_create_test_obj(void) {
  g3d_create_button(&TEST_OBJ, FL_PUSH_BUTTON, 50, 30, "Test\ngraph", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(TEST_OBJ, CB_test_obj, 0);
}


/***************************************************************************************/

static p3d_node *lastpinpointedN = NULL;

static void p3d_pinpoint_node(p3d_node *N, p3d_graph *G) {
  if (lastpinpointedN != NULL) {
    lastpinpointedN->pinpointed = 0;
  }
  N->pinpointed = 1;
  lastpinpointedN = N;

  if (!ENV.getBool(Env::drawGraph)) {
    p3d_set_and_update_this_robot_conf_without_cntrt(G->rob, N->q);
    p3d_copy_config_into(G->rob, N->q, &(G->rob->ROBOT_GOTO));
    printf("\nROBOT_GOTO has been updated with the configuration of the pinpointed node\n");
  }
}


/* cree le menu de selection d'un noeuds */
static void CB_pinpoint_node_obj(FL_OBJECT *ob, long arg) {
  FL_FORM *PINPOINT_NODE_MENU;
  FL_OBJECT *obj, *ok, *cancel, *node;
  int NNODE = 0, i;
  p3d_list_node *list_node;
  p3d_node *Nex = NULL;


  PINPOINT_NODE_MENU = fl_bgn_form(FL_FLAT_BOX, 110.0, 70.0);

  obj = fl_add_box(FL_UP_BOX, 0.0, 0.0, 230.0, 145.0, "");

  node = fl_add_input(FL_NORMAL_INPUT, 50.0, 10.0, 40.0, 20.0,
                      "No node");
  fl_set_input_color(node, 0, 0);

  ok = fl_add_button(FL_PUSH_BUTTON, 10, 40, 40, 20, "OK");
  cancel = fl_add_button(FL_PUSH_BUTTON, 60, 40, 40, 20, "Annuler");

  fl_end_form();

  fl_show_form(PINPOINT_NODE_MENU, FL_PLACE_SIZE, TRUE, "Pinpoint Node");
  while (1) {
    obj = fl_do_forms();
    if (obj == ok) {

      fl_hide_form(PINPOINT_NODE_MENU);

      NNODE = atoi(fl_get_input(node));

      if (NNODE > XYZ_GRAPH->nnode) {
        printf("Wrong node number !\n");
        return;
      }
      Nex = NULL;
      list_node = XYZ_GRAPH->nodes;
      for (i = 1;i <= XYZ_GRAPH->nnode;i++) {
        if (list_node->N->num == NNODE) {
          Nex = list_node->N;
          break;
        } else {
          list_node = list_node->next;
        }
      }
      if (Nex == NULL) {
        printf("ERROR : no node corresponding to number\n");
      } else {
        p3d_pinpoint_node(Nex, XYZ_GRAPH);
      }

      fl_set_button(PINPOINT_NODE_OBJ, 0);

      g3d_draw_allwin_active();

      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(node);

      fl_free_form(PINPOINT_NODE_MENU);
      return;
    }
    if (obj == cancel) {
      fl_hide_form(PINPOINT_NODE_MENU);
      fl_set_button(PINPOINT_NODE_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(node);
      fl_free_form(PINPOINT_NODE_MENU);
      return;
    }
  }
}

static void g3d_create_pinpoint_node_obj(void) {
  g3d_create_button(&PINPOINT_NODE_OBJ, FL_PUSH_BUTTON, 45, 30, "Pinpoint\nnode", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(PINPOINT_NODE_OBJ, CB_pinpoint_node_obj, 0);
}

/***************************************************************************************/

/* cree le menu de destruction d'un noeuds */
static void CB_del_node_obj(FL_OBJECT *ob, long arg) {
  FL_FORM *DEL_NODE_MENU;
  FL_OBJECT *obj, *ok, *cancel, *node;
  int NNODE = 0, i;
  p3d_list_node *list_node;
  p3d_node *Nex = NULL;


  DEL_NODE_MENU = fl_bgn_form(FL_FLAT_BOX, 110.0, 70.0);

  obj = fl_add_box(FL_UP_BOX, 0.0, 0.0, 230.0, 145.0, "");

  node = fl_add_input(FL_NORMAL_INPUT, 50.0, 10.0, 40.0, 20.0,
                      "No node");
  fl_set_input_color(node, 0, 0);

  ok = fl_add_button(FL_PUSH_BUTTON, 10, 40, 40, 20, "OK");
  cancel = fl_add_button(FL_PUSH_BUTTON, 60, 40, 40, 20, "Annuler");

  fl_end_form();

  fl_show_form(DEL_NODE_MENU, FL_PLACE_SIZE, TRUE, "Delete Node");
  while (1) {
    obj = fl_do_forms();
    if (obj == ok) {

      fl_hide_form(DEL_NODE_MENU);

      NNODE = atoi(fl_get_input(node));

      if (NNODE > XYZ_GRAPH->nnode) {
        printf("PAS ASSEZ DE NOEUDS !\n");
        return;
      }
      Nex = NULL;
      list_node = XYZ_GRAPH->nodes;
      for (i = 1;i <= XYZ_GRAPH->nnode;i++) {
        if (list_node->N->num == NNODE) {
          Nex = list_node->N;
          break;
        } else {
          list_node = list_node->next;
        }
      }
      if (Nex == NULL) {
        printf("ERREUR : pas de noeud\n");
      } else {
        p3d_del_node(Nex, XYZ_GRAPH);
      }

      fl_set_button(DEL_NODE_OBJ, 0);

      g3d_draw_allwin_active();

      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(node);

      fl_free_form(DEL_NODE_MENU);
      return;
    }
    if (obj == cancel) {
      fl_hide_form(DEL_NODE_MENU);
      fl_set_button(DEL_NODE_OBJ, 0);
      fl_free_object(cancel);
      fl_free_object(ok);
      fl_free_object(node);
      fl_free_form(DEL_NODE_MENU);
      return;
    }
  }
}

static void g3d_create_del_node_obj(void) {
  g3d_create_button(&DEL_NODE_OBJ, FL_PUSH_BUTTON, 45, 30, "Delete\nnode", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(DEL_NODE_OBJ, CB_del_node_obj, 0);
}



/* choix de la strategie globale */
static void CB_plan_strategy_obj(FL_OBJECT *ob, long arg) {
  switch (arg) {
    case 0 :
      p3d_set_MOTION_PLANNER(P3D_BASIC);
      break;
    case 1 :
      p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
      break;
    case 2 :
      p3d_set_MOTION_PLANNER(P3D_ALL_PRM);
      break;
    case 3 :
      p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
      break;
    case 4 :
      p3d_set_MOTION_PLANNER(P3D_PDR);
      break;

  }
}

static void g3d_create_plan_strategy_obj(void) {
  g3d_create_labelframe(&PLANNING_STATEGY, FL_ENGRAVED_FRAME, -1, -1, "Planning Strategy", (void**)&PLANNER_FORM, 1);

  GROUP2 = fl_bgn_group();
  g3d_create_checkbutton(&STRAT0_OBJ, FL_RADIO_BUTTON, -1, -1, "PRM", (void**)&PLANNING_STATEGY, 0);
  fl_set_object_color(STRAT0_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(STRAT0_OBJ, CB_plan_strategy_obj, 0);
  g3d_create_checkbutton(&STRAT1_OBJ, FL_RADIO_BUTTON, -1, -1, "VisPRM", (void**)&PLANNING_STATEGY, 0);
  fl_set_object_color(STRAT1_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(STRAT1_OBJ, CB_plan_strategy_obj, 1);
  g3d_create_checkbutton(&STRAT2_OBJ, FL_RADIO_BUTTON, -1, -1, "ACR", (void**)&PLANNING_STATEGY, 0);
  fl_set_object_color(STRAT2_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(STRAT2_OBJ, CB_plan_strategy_obj, 2);
  g3d_create_checkbutton(&STRAT3_OBJ, FL_RADIO_BUTTON, -1, -1, "Diffu", (void**)&PLANNING_STATEGY, 0);
  fl_set_object_color(STRAT3_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(STRAT3_OBJ, CB_plan_strategy_obj, 3);

  g3d_create_checkbutton(&STRAT4_OBJ, FL_RADIO_BUTTON, -1, -1, "PDR", (void**)&PLANNING_STATEGY, 0);
  fl_set_object_color(STRAT4_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(STRAT4_OBJ, CB_plan_strategy_obj, 4);

  //GROUP2 = 
  fl_end_group();
  fl_set_button(STRAT3_OBJ, 1);
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
}


/* choix de la strategie globale */
static void CB_sorting_obj(FL_OBJECT *ob, long arg) {
  switch(arg){
    case 0 :{
      p3d_set_SORTING(P3D_NB_CONNECT);
      p3d_order_list_node_nofconnex();
      break;
    }
    case 1 :{
      p3d_set_SORTING(P3D_DIST_NODE);
      break;
    }
    case 2 :{
      p3d_set_costComputation(!p3d_get_costComputation());
      break;
    }
  }
}

static void g3d_create_sorting_obj(void) {
  g3d_create_labelframe(&CONNEXION_STRATEGY, FL_ENGRAVED_FRAME, -1, -1, "Connexion Strategy", (void**)&PLANNER_FORM, 1);

  GROUP3 = fl_bgn_group();
  g3d_create_checkbutton(&SORT1_OBJ, FL_RADIO_BUTTON, -1, -1, "Nb edges", (void**)&CONNEXION_STRATEGY, 0);
  fl_set_object_color(SORT1_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SORT1_OBJ, CB_sorting_obj, 0);
  g3d_create_checkbutton(&SORT2_OBJ, FL_RADIO_BUTTON, -1, -1, "Distance", (void**)&CONNEXION_STRATEGY, 0);
  fl_set_object_color(SORT2_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SORT2_OBJ, CB_sorting_obj, 1);
  //GROUP3 = 
  fl_end_group();
  g3d_create_checkbutton(&COST_OBJ, FL_RADIO_BUTTON, -1, -1, "Cost", (void**)&CONNEXION_STRATEGY, 0);
  fl_set_object_color(COST_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(COST_OBJ, CB_sorting_obj, 2);
  fl_set_button(SORT2_OBJ, 1);
  fl_set_button(COST_OBJ, p3d_get_costComputation());
}
static void  CB_sampling_obj(FL_OBJECT *ob, long arg) {
  switch (arg) {
    case 0:
      p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
      break;
    case 1:
      p3d_set_RANDOM_CHOICE(P3D_HALTON_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
      break;
    case 3:
      p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_GAUSSIAN_SAMPLING);
      break;
    case 4:
      p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_BRIDGE_SAMPLING);
      break;
    case 5:
      p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_OBPRM_SAMPLING);
      break;
    default:
      PrintInfo(("Error: Sampling strategy not defined \n"));
  }

}

static void g3d_create_sampling_obj(void) {
  g3d_create_labelframe(&SAMPLING_STRATEGY, FL_ENGRAVED_FRAME, -1, -1, "Sampling Strategy", (void**)&PLANNER_FORM, 1);

  GROUP4 = fl_bgn_group();
  g3d_create_checkbutton(&SAMPL1_OBJ, FL_RADIO_BUTTON, -1, -1, "Random", (void**)&SAMPLING_STRATEGY, 0);
  fl_set_object_color(SAMPL1_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SAMPL1_OBJ, CB_sampling_obj, 0);
  g3d_create_checkbutton(&SAMPL2_OBJ, FL_RADIO_BUTTON, -1, -1, "Halton", (void**)&SAMPLING_STRATEGY, 0);
  fl_set_object_color(SAMPL2_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SAMPL2_OBJ, CB_sampling_obj, 1);
  g3d_create_checkbutton(&SAMPL3_OBJ, FL_RADIO_BUTTON, -1, -1, "Gaussian", (void**)&SAMPLING_STRATEGY, 0);
  fl_set_object_color(SAMPL3_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SAMPL3_OBJ, CB_sampling_obj, 3);
  g3d_create_checkbutton(&SAMPL4_OBJ, FL_RADIO_BUTTON, -1, -1, "Bridge", (void**)&SAMPLING_STRATEGY, 0);
  fl_set_object_color(SAMPL4_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SAMPL4_OBJ, CB_sampling_obj, 4);
  g3d_create_checkbutton(&SAMPL5_OBJ, FL_RADIO_BUTTON, -1, -1, "OBPRM", (void**)&SAMPLING_STRATEGY, 0);
  fl_set_object_color(SAMPL5_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(SAMPL5_OBJ, CB_sampling_obj, 5);
  //GROUP4 = 
  fl_end_group();
  fl_set_button(SAMPL1_OBJ, 1);
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
}

/* choix de l'orientation du graph */
static void CB_oriented_obj(FL_OBJECT *ob, long arg) {
  int val;

  val = fl_get_button(ob);

  p3d_set_ORIENTED(val);
  if (XYZ_GRAPH != NULL) {
    XYZ_GRAPH->oriented = val;
  }
}

static void g3d_create_oriented_obj(void) {
  g3d_create_frame(&ORIENTED_FRAME, FL_ENGRAVED_FRAME, -1, -1, "", (void**)&PLANNER_FORM, 1);
  g3d_create_checkbutton(&ORIENTED_OBJ, FL_PUSH_BUTTON, -1, -1, "Oriented", (void**)&ORIENTED_FRAME, 0);
  fl_set_object_color(ORIENTED_OBJ, FL_MCOL, FL_GREEN);
  fl_set_call_back(ORIENTED_OBJ, CB_oriented_obj, 0);
  fl_set_button(ORIENTED_OBJ, 0);
}

/* permet de modifier le parametre NB_TRY */
static void CB_nbtry_input_obj(FL_OBJECT *ob, long arg) {
	ENV.setInt(Env::NbTry,atoi(fl_get_input(INPUT_OBJ)));

  fl_hide_form(INPUT_FORM);

  fl_freeze_form(PLANNER_FORM);
  fl_set_slider_value(SEARCH_NBTRY_PARAM_OBJ, ENV.getInt(Env::NbTry));
  fl_unfreeze_form(PLANNER_FORM);

  fl_free_object(INPUT_OBJ);
  fl_free_form(INPUT_FORM);
}

static void CB_create_nbtry_input_obj(FL_OBJECT *ob, long arg) {
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX, 90.0, 75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX, 0.0, 0.0, 90.0, 75.0, "");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT, 40.0, 10.0, 40.0, 30.0,
                            "Nodes");
  fl_set_input_color(INPUT_OBJ, 0, 0);
  cancel = fl_add_button(FL_PUSH_BUTTON, 40, 45, 40, 20, "Annuler");
  fl_set_object_callback(cancel, CB_cancel, 0);
  fl_end_form();

  fl_set_object_callback(INPUT_OBJ, CB_nbtry_input_obj, 0);

  fl_set_button(SEARCH_NBTRY_INPUT_OBJ, 0);
  fl_show_form(INPUT_FORM, FL_PLACE_SIZE, TRUE, "");
}

static void g3d_create_nbtry_input_obj(void) {
  g3d_create_button(&SEARCH_NBTRY_INPUT_OBJ, FL_PUSH_BUTTON, 50, 30, "Nb try", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_object_callback(SEARCH_NBTRY_INPUT_OBJ, CB_create_nbtry_input_obj, 0);
}

static void CB_nbtry_param_obj(FL_OBJECT *ob, long arg) {
  ENV.setInt(Env::NbTry,floor(fl_get_slider_value(SEARCH_NBTRY_PARAM_OBJ)));
}

static void g3d_create_nbtry_param_obj(void) {
  g3d_create_valslider(&SEARCH_NBTRY_PARAM_OBJ, FL_HOR_SLIDER, 324, 30.0, "", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_slider_step(SEARCH_NBTRY_PARAM_OBJ, 1.0);
  fl_set_slider_bounds(SEARCH_NBTRY_PARAM_OBJ, 10, 10000);
  fl_set_slider_value(SEARCH_NBTRY_PARAM_OBJ, ENV.getInt(Env::NbTry));
  fl_set_object_callback(SEARCH_NBTRY_PARAM_OBJ, CB_nbtry_param_obj, 0);
}


static FL_FORM *ADD_FORM;
static FL_OBJECT **POSITION_OBJ, **POSITION_BOX_OBJ, **POSITION_WALK_OBJ;
static configPt qadd, qadd_deg; /* configuration of the robot expressed
           in radian and in degree */
double *dq_exp,  /* angles expressed in radian */
*delta_exp;           /* angles expressed in radian */
static FL_FORM *EXP_CHOICE_FORM, *EXP_BOX_FORM, *EXP_WALK_FORM;
static int EXPAND = FALSE;
static FL_OBJECT *ok, *cancel, *expand;

static void CB_exp_walk_form(FL_OBJECT *ob, long arg) {
  double val = fl_get_slider_value(ob), f_min, f_max;
  p3d_rob * robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);

  p3d_get_robot_dof_bounds(robotPt, arg, &f_min, &f_max);
  delta_exp[arg] = val * (f_max - f_min) / 100.;
}


static void g3d_create_exp_walk_form(void) {
  int njnt, nb_dof;
  double s;
  double f_min, f_max;  // angles expressed in radian
  int ord, i, j, k;
  char  str[30];
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;
  FL_OBJECT *obj, *ok, *nbnode, *nbwalk;


  nb_dof = calc_real_dof();
  s = nb_dof * 20.0 + 60.0;

  nb_dof = robotPt->nb_dof;
  njnt = robotPt->njoints;

  POSITION_WALK_OBJ = MY_ALLOC(FL_OBJECT *, nb_dof);
  delta_exp = MY_ALLOC(double, nb_dof);
  p3d_set_EXPAND_DELTA(delta_exp);

  EXP_WALK_FORM = fl_bgn_form(FL_UP_BOX, 500.0, s);
  ord = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
        p3d_jnt_get_dof_bounds(jntPt, j, &f_min, &f_max);
        POSITION_WALK_OBJ[k] =
          fl_add_valslider(FL_HOR_SLIDER, 110.0, 10. + 20.0 * ord, 280.0, 20.0, "");
        fl_set_slider_bounds(POSITION_WALK_OBJ[k], 0., 100.);
        strcpy(str, "Delta ");
        strcat(str, p3d_jnt_get_dof_name(jntPt, j));
        fl_add_box(FL_UP_BOX, 10.0, 10. + 20.0*ord, 100.0, 20.0, str);

        fl_set_slider_value(POSITION_WALK_OBJ[k], 10.);
        fl_set_call_back(POSITION_WALK_OBJ[k], CB_exp_walk_form, k);
        delta_exp[k] = 0.1 * (f_max - f_min);
        ord = ord + 1;
      } else {
        POSITION_WALK_OBJ[k] = NULL;
      }
    }
  }

  ok = fl_add_button(FL_PUSH_BUTTON, 10, 20. + 20.0 * ord, 60, 30, "OK");

  nbnode = fl_add_input(FL_NORMAL_INPUT, 450.0, 140.0, 40.0, 20.0,
                        "Nb node");
  fl_set_input_color(nbnode, 0, 0);

  nbwalk = fl_add_input(FL_NORMAL_INPUT, 450.0, 170.0, 40.0, 20.0,
                        "Nb walk");
  fl_set_input_color(nbnode, 0, 0);

  fl_end_form();
  fl_show_form(EXP_WALK_FORM, FL_PLACE_SIZE, TRUE, "Expand Walk Param.");

  while (1) {
    obj = fl_do_forms();
    if (obj == ok) {
      p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));
      p3d_set_EXPAND_NB_WALK(atoi(fl_get_input(nbwalk)));
      fl_hide_form(EXP_WALK_FORM);
      fl_free_object(ok);
      fl_free_object(nbnode);
      fl_free_object(nbwalk);

      for (i = 0;i < nb_dof;i++) {
        if (POSITION_WALK_OBJ[i] != NULL) {
          fl_free_object(POSITION_WALK_OBJ[i]);
        }
      }
      MY_FREE(POSITION_WALK_OBJ, FL_OBJECT *, nb_dof);
      fl_free_form(EXP_WALK_FORM);
      return;
    }
  }
}


static void CB_exp_box_form(FL_OBJECT *ob, long arg) {
  double val = fl_get_slider_value(ob), f_min, f_max;
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  p3d_get_robot_dof_bounds(robotPt, arg, &f_min, &f_max);
  dq_exp[arg] = val * (f_max - f_min) / 100.;
}


static void g3d_create_exp_box_form(void) {
  int njnt, nb_dof;
  double s;
  double f_min, f_max;
  int ord, i, j, k;
  char      str[30];
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;
  FL_OBJECT *obj, *ok, *nbnode;


  nb_dof = calc_real_dof();
  s = nb_dof * 20.0 + 60.0;

  nb_dof = robotPt->nb_dof;
  njnt = robotPt->njoints;

  POSITION_BOX_OBJ = MY_ALLOC(FL_OBJECT *, nb_dof);
  dq_exp = MY_ALLOC(double, nb_dof);
  p3d_set_EXPAND_DQ(dq_exp);

  EXP_BOX_FORM = fl_bgn_form(FL_UP_BOX, 500.0, s);
  ord = 0;

  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
        p3d_jnt_get_dof_bounds(jntPt, j, &f_min, &f_max);
        POSITION_BOX_OBJ[k] =
          fl_add_valslider(FL_HOR_SLIDER, 110.0, 10. + 20.0 * ord, 280.0, 20.0, "");
        fl_set_slider_bounds(POSITION_BOX_OBJ[k], 0., 100.);
        strcpy(str, "Delta ");
        strcat(str, p3d_jnt_get_dof_name(jntPt, j));
        fl_add_box(FL_UP_BOX, 10.0, 10. + 20.0*ord, 100.0, 20.0, str);

        fl_set_slider_value(POSITION_BOX_OBJ[k], 10.);
        fl_set_call_back(POSITION_BOX_OBJ[k], CB_exp_box_form, k);
        dq_exp[k] = 0.1 * (f_max - f_min);
        ord = ord + 1;
      } else {
        POSITION_BOX_OBJ[k] = NULL;
      }
    }
  }

  ok = fl_add_button(FL_PUSH_BUTTON, 10, 20. + 20.0 * ord, 60, 30, "OK");
  nbnode = fl_add_input(FL_NORMAL_INPUT, 450.0, 140.0, 40.0, 20.0,
                        "Nb node");
  fl_set_input_color(nbnode, 0, 0);

  fl_end_form();
  fl_show_form(EXP_BOX_FORM, FL_PLACE_SIZE, TRUE, "Expand Box Param.");

  while (1) {
    obj = fl_do_forms();
    if (obj == ok) {
      p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));
      fl_hide_form(EXP_BOX_FORM);
      fl_free_object(ok);
      fl_free_object(nbnode);

      for (i = 0;i < nb_dof;i++) {
        if (POSITION_BOX_OBJ[i] != NULL) {
          fl_free_object(POSITION_BOX_OBJ[i]);
        }
      }
      MY_FREE(POSITION_BOX_OBJ, FL_OBJECT *, nb_dof);
      fl_free_form(EXP_BOX_FORM);
      return;
    }
  }
}

static void g3d_create_expand_choice_form(void) {
  FL_OBJECT *obj, *group, *choice1, *choice2;

  EXP_CHOICE_FORM = fl_bgn_form(FL_UP_BOX, 115.0, 105);
  obj = fl_add_frame(FL_ENGRAVED_FRAME, 10, 20, 95, 70, "");
  obj = fl_add_box(FL_FLAT_BOX, 25, 15, 60, 10, "Expand type");

  group = fl_bgn_group();
  choice1 = fl_add_checkbutton(FL_RADIO_BUTTON, 10, 25, 60, 30, "Centered box");
  fl_set_object_color(choice1, FL_MCOL, FL_GREEN);
  p3d_set_EXPAND_CHOICE(P3D_EXPAND_BOX);
  choice2 = fl_add_checkbutton(FL_RADIO_BUTTON, 10, 55, 60, 30, "Random walk");
  fl_set_object_color(choice2, FL_MCOL, FL_GREEN);
  //group = 
  fl_end_group();

  fl_end_form();
  fl_show_form(EXP_CHOICE_FORM, FL_PLACE_SIZE, TRUE, "Expansion Mode");

  while (1) {
    obj = fl_do_forms();
    if (obj == choice1) {
      fl_hide_form(EXP_CHOICE_FORM);
      p3d_set_EXPAND_CHOICE(P3D_EXPAND_BOX);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(group);
      fl_free_form(EXP_CHOICE_FORM);
      g3d_create_exp_box_form();

    }
    if (obj == choice2) {
      fl_hide_form(EXP_CHOICE_FORM);
      p3d_set_EXPAND_CHOICE(P3D_EXPAND_RAND_WALK);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(group);
      fl_free_form(EXP_CHOICE_FORM);
      g3d_create_exp_walk_form();
    }
  }
}


static void CB_add_form(FL_OBJECT *ob, long arg) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;
  double val = fl_get_slider_value(ob);
  int i, i_dof;
  int nb_dof;
  int I_can;

  qadd_deg[arg] = val;
  I_can = p3d_set_and_update_robot_conf_deg(qadd_deg);

  nb_dof = p3d_get_robot_ndof();

  if (robotPt->cntrt_manager->cntrts != NULL) {
    if (I_can) {
      p3d_get_robot_config_deg_into(robotPt, &qadd_deg);
      for (i = 0; i < nb_dof; i++) {
        fl_set_slider_value(POSITION_OBJ[i], qadd_deg[i]);
      }
      p3d_copy_config_into(robotPt, qadd_deg, &last_p_deg);
    } else {
      p3d_copy_config_into(robotPt, last_p_deg, &qadd_deg);
      for (i = 0; i < nb_dof; i++) {
        fl_set_slider_value(POSITION_OBJ[i], qadd_deg[i]);
      }
      jntPt = p3d_robot_dof_to_jnt(robotPt, arg, &i_dof);
      p3d_jnt_set_dof_deg(jntPt, i_dof, qadd_deg[arg]);
      p3d_update_this_robot_pos_without_cntrt(robotPt);
    }
  }

  p3d_convert_config_deg_to_rad(robotPt, qadd_deg, &qadd);

  if (G3D_ACTIVE_CC) {
    p3d_col_test_all();
  }

  g3d_draw_allwin_active();
}


static void CB_add_exp_button_obj(FL_OBJECT *ob, long arg) {
  EXPAND = TRUE;
  g3d_create_expand_choice_form();
}

static void CB_add_ok_button_obj(FL_OBJECT *ob, long arg) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  int i, nb_dof = robotPt->nb_dof;
  p3d_node *N, *Nc;
  int nb_node_sorted;
  p3d_compco *comp;
  int LINK_LINK, LINK_ORPH, NB_LINK_ORPH = 0, NB_LINK = 0, GRAPH_LINKED = FALSE;
  double dist;
  p3d_list_node *node, *linked_nodes = NULL, *last_node = NULL,
                                       *destr_node, *list_node;
  int **iksol = NULL; // <- modif Juan



  fl_hide_form(ADD_FORM);

  p3d_set_and_update_robot_conf(qadd);

  // modif Juan
  p3d_get_iksol_vector(robotPt->cntrt_manager, &iksol);
  N = p3d_APInode_make_multisol(XYZ_GRAPH, qadd, *iksol);
  // fmodif Juan

  switch (p3d_get_MOTION_PLANNER()) {
    case 1:
      p3d_insert_node(XYZ_GRAPH, N);
      if (XYZ_GRAPH->nnode > 1) {
        comp = XYZ_GRAPH->comp;
        while (comp != NULL) {
          if (N->numcomp != comp->num) {
            if (p3d_link_node_comp(XYZ_GRAPH, N, &comp)) {
              GRAPH_LINKED = TRUE;
            }
          }
          comp = comp->suiv;
        }
      }
      break;
    case 2:

      if (XYZ_GRAPH->nnode >= 1) {
        comp = XYZ_GRAPH->comp;
        /* on parcourt toutes les composantes connexes */
        while (comp != NULL) {
          LINK_LINK = FALSE;
          LINK_ORPH = FALSE;
          PrintInfo(("On essaye de relier le noeud %d a la comp %d\n",
                     N->num, comp->num));
          list_node = comp->dist_nodes;
          nb_node_sorted = 0;
          while (list_node != NULL) {
            Nc = list_node->N;
            /* si le noeud est reliable, on arrete */
            if (p3d_APInode_linked(XYZ_GRAPH, N, Nc, &dist)) {
              PrintInfo(("elles sont reliees\n"));
              if (Nc->type == ISOLATED) {
                if (!LINK_ORPH) {
                  NB_LINK_ORPH = NB_LINK_ORPH + 1;
                }
                LINK_ORPH = TRUE;
              }
              if (!LINK_LINK) {
                NB_LINK = NB_LINK + 1;
              }
              LINK_LINK = TRUE;
              p3d_add_node_compco(Nc, comp, TRUE);
              node = MY_ALLOC(p3d_list_node, 1);
              node->N = Nc;
              node->next = NULL;
              if (NB_LINK == 1) {
                linked_nodes = node;
                last_node = node;
              } else if (NB_LINK > 1) {
                last_node->next = node;
                last_node = last_node->next;
              }
            }
            list_node = list_node->next;
          }
          comp = comp->suiv;
        }
      }

      /* quand toutes les composantes ont ete parcourues */
      if (NB_LINK_ORPH == 0 && NB_LINK == 0) {
        N->type = ISOLATED;
        p3d_insert_node(XYZ_GRAPH, N);

        GRAPH_LINKED = FALSE;
      } else {
        if (NB_LINK_ORPH >= 0 && NB_LINK > 1) {
          N->type = LINKING;
        } else {
          N->type = ISOLATED;
        }
        p3d_insert_node(XYZ_GRAPH, N);
        node = linked_nodes;
        while (node != NULL) {
          if (N->numcomp < node->N->numcomp) {
            p3d_merge_comp(XYZ_GRAPH, N->comp, &(node->N->comp));
          } else {
            p3d_merge_comp(XYZ_GRAPH, node->N->comp, &(N->comp));
          }
          p3d_create_edges(XYZ_GRAPH, N, node->N, dist);
          destr_node = node;
          node = node->next;
          MY_FREE(destr_node, p3d_list_node, 1);
        }
        GRAPH_LINKED = TRUE;
      }
      break;
    default:
      p3d_insert_node(XYZ_GRAPH, N);
      if (XYZ_GRAPH->nnode > 1) {
        comp = XYZ_GRAPH->comp;
        while (comp != NULL) {
          if (N->numcomp != comp->num) {
            if (p3d_link_node_comp(XYZ_GRAPH, N, &comp)) {
              GRAPH_LINKED = TRUE;
            }
          }
          comp = comp->suiv;
        }
      }
  }

  PrintInfo(("GRAPH_LINKED : %d\n", GRAPH_LINKED));

  if (!(GRAPH_LINKED) && EXPAND)
    p3d_APInode_expand(XYZ_GRAPH, N, fct_stop, fct_draw);

  fl_set_button(ADD_OBJ, 0);
  fl_free_object(cancel);
  fl_free_object(ok);
  fl_free_object(expand);

  for (i = 0;i < nb_dof;i++) {
    if (POSITION_OBJ[i] != NULL) {
      fl_free_object(POSITION_OBJ[i]);
    }
  }
  MY_FREE(POSITION_OBJ, FL_OBJECT *, nb_dof);
  fl_free_form(ADD_FORM);
  p3d_destroy_config(robotPt, last_p_deg);
  return;
}

static void CB_add_cancel_button_obj(FL_OBJECT *ob, long arg) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  int nb_dof = robotPt->nb_dof;
  int i;

  fl_hide_form(ADD_FORM);

  p3d_destroy_config(robotPt, qadd_deg);
  p3d_destroy_config(robotPt, qadd);

  fl_set_button(ADD_OBJ, 0);
  fl_free_object(cancel);
  fl_free_object(ok);
  fl_free_object(expand);

  for (i = 0;i < nb_dof;i++) {
    if (POSITION_OBJ[i] != NULL) {
      fl_free_object(POSITION_OBJ[i]);
    }
  }
  MY_FREE(POSITION_OBJ, FL_OBJECT *, nb_dof);
  fl_free_form(ADD_FORM);
  return;
}


static void CB_add_obj(FL_OBJECT *ob, long arg) {
  int njnt, nb_dof;
  double s;
  double f_min , f_max;
  int ord, i, j, k;
  char      str[30];
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;

  /* copy the position of the current robot into qadd */
  qadd_deg = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_POS);
  qadd = p3d_copy_config(robotPt, robotPt->ROBOT_POS);

  last_p_deg = p3d_alloc_config(robotPt);
  p3d_copy_config_into(robotPt, qadd_deg, &last_p_deg);

  if (!XYZ_GRAPH) {
    printf("ERREUR : pas de graphe courant\n");
    return;
  }

  nb_dof = calc_real_dof();
  s = nb_dof * 20.0 + 60.0;

  nb_dof = robotPt->nb_dof;
  njnt = robotPt->njoints;

  POSITION_OBJ = MY_ALLOC(FL_OBJECT *, nb_dof);

  ADD_FORM = fl_bgn_form(FL_UP_BOX, 500.0, s);
  ord = 0;

  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
        p3d_jnt_get_dof_bounds_deg(jntPt, j, &f_min, &f_max);
        POSITION_OBJ[k] =
          fl_add_valslider(FL_HOR_SLIDER, 110.0, 10. + 20.0 * ord, 280.0, 20.0, "");
        fl_set_slider_bounds(POSITION_OBJ[k], f_max, f_min);
        strcpy(str, "Pos ");
        strcat(str, p3d_jnt_get_dof_name(jntPt, j));
        fl_add_box(FL_UP_BOX, 10.0, 10. + 20.0*ord, 100.0, 20.0, str);

        fl_set_slider_value(POSITION_OBJ[k], qadd_deg[k]);
        fl_set_call_back(POSITION_OBJ[k], CB_add_form, k);
        if (robotPt->cntrt_manager->in_cntrt[k] == 2) {
          fl_set_slider_size(POSITION_OBJ[k], 1.0);
        }
        ord = ord + 1;
      } else {
        POSITION_OBJ[k] = NULL;
      }
    }
  }

  ok = fl_add_button(FL_PUSH_BUTTON, 10, 20. + 20.0 * ord, 60, 30, "OK");
  fl_set_call_back(ok, CB_add_ok_button_obj, arg);
  cancel = fl_add_button(FL_PUSH_BUTTON, 330, 20. + 20.0 * ord, 60, 30, "cancel");
  fl_set_call_back(cancel, CB_add_cancel_button_obj, arg);
  expand = fl_add_checkbutton(FL_RADIO_BUTTON, 410, 20, 55, 30, "Expand \nif linking \nfails");
  fl_set_call_back(expand, CB_add_exp_button_obj, arg);
  fl_set_object_color(expand, FL_MCOL, FL_GREEN);

  fl_end_form();
  fl_show_form(ADD_FORM, FL_PLACE_SIZE, TRUE, "Add Node");
}


static void g3d_create_add_obj(void) {
  g3d_create_button(&ADD_OBJ, FL_PUSH_BUTTON, 45, 30, "Add\nnode", (void**)&GRAPH_NODE_FRAME, 0);
  fl_set_call_back(ADD_OBJ, CB_add_obj, 0);
}

static void g3d_create_frame_obj(void) {
  g3d_create_labelframe(&FRAME_OBJ, FL_ENGRAVED_FRAME, 100, 113, "Graph", (void**)&PLANNER_FORM, 1);
}

static void CB_path_deformation_obj(FL_OBJECT *ob, long arg) {
  if (fl_get_button(ob)) {
    g3d_create_path_deformation_form();
    fl_show_form(PATH_DEFORM_FORM, FL_PLACE_SIZE, TRUE, "Path Deformation");
  } else {
    g3d_destroy_pathDeformForm();
  }
}

static void g3d_create_path_defom_obj(void) {
  g3d_create_button(&PATH_DEFORMATION, FL_PUSH_BUTTON, 49.0, 30.0, "Path\nDeform.", (void**)&OPTIMIZE_FRAME, 0);
  fl_set_call_back(PATH_DEFORMATION, CB_path_deformation_obj, 0);
}

static void CB_tmax_obj(FL_OBJECT *ob, long arg) {
//if tmax == 0.0 don't use timer to stop.
  p3d_set_tmax((double)atoi(fl_get_input(TMAX_OBJ)));
}

static void g3d_create_tmax_obj(void) {
  char buffer [10];
  g3d_create_input(&TMAX_OBJ, FL_NORMAL_INPUT, -1, 27, "Tmax", (void**)&INPUT_FRAME, 0);
  sprintf(buffer, "%d", (int)p3d_get_tmax());
  fl_set_input(TMAX_OBJ, buffer);
  fl_set_call_back(TMAX_OBJ, CB_tmax_obj, 0);
}

static void CB_max_connect_obj(FL_OBJECT *ob, long arg) {
  p3d_set_max_connect(atoi(fl_get_input(MAX_CONNECT_OBJ)) == 0 ? p3d_get_max_connect() : atoi(fl_get_input(MAX_CONNECT_OBJ)));
}

static void g3d_create_max_connect_obj(void) {
  char buffer[10];
  g3d_create_input(&MAX_CONNECT_OBJ, FL_NORMAL_INPUT, -1, 27, "nlink", (void**)&INPUT_FRAME, 0);
  sprintf(buffer, "%d", p3d_get_max_connect());
  fl_set_input(MAX_CONNECT_OBJ, buffer);
  fl_set_call_back(MAX_CONNECT_OBJ, CB_max_connect_obj, 0);
}
/*****************************************************************/
//desactivations
static void p3d_desactivate_ik_draw(int desactivate) {
  if (IK_FORM != NULL) {
    if (desactivate) {
      fl_deactivate_object(NORMAL_CB);
      fl_deactivate_object(MULTISOL_CB);
      fl_deactivate_object(UNIQUE_CB);
    } else {
      fl_activate_object(NORMAL_CB);
      fl_activate_object(MULTISOL_CB);
      fl_activate_object(UNIQUE_CB);
    }
  }
}

static void p3d_desactivate_planning(int desactivate) {
  if (PLANNER_FORM != NULL) {
    if (desactivate) {
      fl_deactivate_object(SEARCH_GLOBAL_OBJ);
      fl_deactivate_object(SEARCH_SPECIFIC_OBJ);
    } else {
      fl_activate_object(SEARCH_GLOBAL_OBJ);
      fl_activate_object(SEARCH_SPECIFIC_OBJ);
    }
  }
}

/*****************************************************************/

/* fonctions de destruction des objets xforms */
void g3d_delete_planner_form(void) {
  if (fl_get_button(planner_obj)) {
    fl_hide_form(PLANNER_FORM);
  }

  g3d_delete_local_search_obj();
  g3d_delete_random_confs_obj();   // modif Juan
  g3d_delete_smr_confs_obj();   // modif Ron
  g3d_delete_drawnjnt_obj();       // modif Juan
  g3d_delete_global_search_obj();
  g3d_delete_specific_search_obj();
  g3d_delete_transSpePlanner_obj();
#ifdef MULTIGRAPH
  g3d_delete_multiGraph_obj();
#endif
  g3d_delete_saveInfo_obj();
  g3d_delete_N_specific_obj();
  g3d_delete_optim_form();
  g3d_delete_optim_obj();
  g3d_delete_stat_obj();
  g3d_delete_draw_light_trace_obj();
  g3d_delete_draw_trace_obj();

  g3d_delete_global_input_obj();
  g3d_delete_global_param_obj();

  g3d_delete_compco_input_obj();
  g3d_delete_compco_param_obj();

  g3d_delete_del_param_obj();

  g3d_delete_DMAX_param_obj();

  g3d_delete_draw_optim_search_obj();

  g3d_delete_print_obj();
  g3d_delete_draw_obj();
  g3d_delete_ik_button();
  g3d_delete_save_mult_obj();

  g3d_delete_load_graph_obj();
  g3d_delete_save_graph_obj();

  g3d_delete_graph_in_grid_obj();

  g3d_delete_stop_obj();

  g3d_delete_pinpoint_node_obj();
  g3d_delete_expand_obj();
  g3d_delete_isolate_obj();
  g3d_delete_linking_obj();
  g3d_delete_test_obj();

  g3d_delete_del_node_obj();
  g3d_delete_plan_strategy_obj();
  g3d_delete_sorting_obj();
  g3d_delete_sampling_obj();
  g3d_delete_oriented_obj();
  g3d_delete_nbtry_input_obj();
  g3d_delete_nbtry_param_obj();
  g3d_delete_add_obj();

  g3d_delete_frame_obj();
  g3d_delete_path_deform_obj();
  if (PATH_DEFORM_FORM && PATH_DEFORM_FORM->label != NULL) {
    g3d_destroy_pathDeformForm();
  }
  g3d_fl_free_object(TRAJECTORY_FRAME);
  g3d_fl_free_object(OPTIMIZE_FRAME);
  g3d_fl_free_object(GRAPH_NODE_FRAME);
  g3d_fl_free_object(PLANNER_CHOICE);
  g3d_delete_tmax_obj();
  g3d_delete_max_connect_obj();

  g3d_fl_free_object(INPUT_FRAME);
  g3d_fl_free_form(PLANNER_FORM);
}


static void g3d_delete_local_search_obj(void) {
  g3d_fl_free_object(SEARCH_OBJ);
}

// modif Juan
static void g3d_delete_random_confs_obj(void) {
  g3d_fl_free_object(RANDCONFS_OBJ);
}

// modif Ron
static void g3d_delete_smr_confs_obj(void) {
  g3d_fl_free_object(SMRCONFS_OBJ);
}

static void g3d_delete_drawnjnt_obj(void) {
  g3d_fl_free_object(DRAWNJNT_OBJ);
}
// fmodif Juan

static void g3d_delete_global_search_obj(void) {
  g3d_fl_free_object(SEARCH_GLOBAL_OBJ);
}


static void g3d_delete_specific_search_obj(void) {
  g3d_fl_free_object(SEARCH_SPECIFIC_OBJ);
}

static void g3d_delete_N_specific_obj(void) {
  g3d_fl_free_object(N_SPECIFIC_OBJ);
}

static void g3d_delete_optim_obj(void) {
  g3d_fl_free_object(OPTIM_OBJ);
}

static void g3d_delete_global_input_obj(void) {
  g3d_fl_free_object(SEARCH_GLOBAL_INPUT_OBJ);
}

static void g3d_delete_global_param_obj(void) {
  g3d_fl_free_object(SEARCH_GLOBAL_PARAM_OBJ);
}

static void g3d_delete_del_param_obj(void) {
  g3d_fl_free_object(SEARCH_DEL_PARAM_OBJ);
}

static void g3d_delete_DMAX_param_obj(void) {
  g3d_fl_free_object(SEARCH_DMAX_PARAM_OBJ);
}

static void g3d_delete_draw_obj(void) {
  g3d_fl_free_object(SEARCH_DRAW_OBJ);
}

static void g3d_delete_ik_form(void) {
  fl_hide_form(IK_FORM);
  g3d_fl_free_object(NORMAL_CB);
  g3d_fl_free_object(MULTISOL_CB);
  g3d_fl_free_object(UNIQUE_CB);
  fl_free_object(IK_GROUP);
  g3d_fl_free_object(IK_FRAME);
  g3d_fl_free_form(IK_FORM);
  IK_FORM = NULL;
}
static void g3d_delete_ik_button(void) {
  g3d_fl_free_object(IK_BUTTON);
}

static void g3d_delete_print_obj(void) {
  g3d_fl_free_object(SEARCH_PRINT_OBJ);
}

static void g3d_delete_save_mult_obj(void) {
  g3d_fl_free_object(SEARCH_DRAW_MULT_OBJ);
}

static void g3d_delete_compco_param_obj(void) {
  g3d_fl_free_object(SEARCH_COMPCO_PARAM_OBJ);
}

static void g3d_delete_compco_input_obj(void) {
  g3d_fl_free_object(SEARCH_COMPCO_INPUT_OBJ);
}

static void g3d_delete_draw_optim_search_obj(void) {
  g3d_fl_free_object(SEARCH_DRAW_OPTIM_OBJ);
}

static void g3d_delete_load_graph_obj(void) {
  g3d_fl_free_object(LOAD_GRAPH_OBJ);
}

static void g3d_delete_save_graph_obj(void) {
  g3d_fl_free_object(SAVE_GRAPH_OBJ);
}

static void g3d_delete_graph_in_grid_obj(void) {
  g3d_fl_free_object(GRAPH_IN_GRID_OBJ);
}

static void g3d_delete_stop_obj(void) {
  g3d_fl_free_object(STOP_OBJ);
}

static void g3d_delete_pinpoint_node_obj(void) {
  g3d_fl_free_object(PINPOINT_NODE_OBJ);
}

static void g3d_delete_expand_obj(void) {
  g3d_fl_free_object(EXPAND_OBJ);
}

static void g3d_delete_isolate_obj(void) {
  g3d_fl_free_object(ISOLATE_OBJ);
}

static void g3d_delete_linking_obj(void) {
  g3d_fl_free_object(LINKING_OBJ);
}

static void g3d_delete_test_obj(void) {
  g3d_fl_free_object(TEST_OBJ);
}

static void g3d_delete_del_node_obj(void) {
  g3d_fl_free_object(DEL_NODE_OBJ);
}

static void g3d_delete_tmax_obj(void) {
  g3d_fl_free_object(TMAX_OBJ);
}

static void g3d_delete_max_connect_obj(void) {
  g3d_fl_free_object(MAX_CONNECT_OBJ);
}

static void g3d_delete_plan_strategy_obj(void) {
  g3d_fl_free_object(STRAT0_OBJ);
  g3d_fl_free_object(STRAT1_OBJ);
  g3d_fl_free_object(STRAT2_OBJ);
  g3d_fl_free_object(STRAT3_OBJ);
  g3d_fl_free_object(STRAT4_OBJ);
}

static void g3d_delete_sorting_obj(void) {
  g3d_fl_free_object(SORT1_OBJ);
  g3d_fl_free_object(SORT2_OBJ);
}

static void g3d_delete_sampling_obj(void) {
  g3d_fl_free_object(SAMPL1_OBJ);
  g3d_fl_free_object(SAMPL2_OBJ);
  g3d_fl_free_object(SAMPL4_OBJ);
  g3d_fl_free_object(SAMPL5_OBJ);
}

static void g3d_delete_oriented_obj(void) {
  g3d_fl_free_object(ORIENTED_OBJ);
}

static void g3d_delete_nbtry_param_obj(void) {
  g3d_fl_free_object(SEARCH_NBTRY_PARAM_OBJ);
}

static void g3d_delete_nbtry_input_obj(void) {
  g3d_fl_free_object(SEARCH_NBTRY_INPUT_OBJ);
}

static void g3d_delete_add_obj(void) {
  g3d_fl_free_object(ADD_OBJ);
}

static void g3d_delete_frame_obj(void) {
  g3d_fl_free_object(FRAME_OBJ);
}

static void g3d_delete_path_deform_obj(void) {
  g3d_fl_free_object(PATH_DEFORMATION);
}
static void g3d_delete_saveInfo_obj(void){
  g3d_fl_free_object(SAVE_INFO_OBJ);
}

#ifdef MULTIGRAPH
static void g3d_delete_multiGraph_obj(void){
  g3d_fl_free_object(MULTIGRAPH_OBJ);
}
#endif

static void g3d_delete_stat_obj(void){
  g3d_fl_free_object(STAT_OBJ);
  g3d_fl_free_object(STAT_FRAME_OBJ);
}

static void g3d_delete_transSpePlanner_obj(void){
  g3d_fl_free_object(SEARCH_SPECIFIC_TRANS_OBJ);
}

static void g3d_delete_draw_light_trace_obj(void){
  g3d_fl_free_object(DRAW_LIGHT_TRACE_OBJ);
}

static void g3d_delete_draw_trace_obj(void){
  g3d_fl_free_object(DRAW_TRACE_OBJ);
}

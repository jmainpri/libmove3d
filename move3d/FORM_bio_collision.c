#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif

/****************************************************************/
#define NB_MAX_DOF_AA 8
#define NB_MAX_LOOP 10
#define NB_MAX_DOF_LIGAND 100
#define NB_MAX_AA 2000
#define NB_MAX_COL_AFFICH 800        //soit 400 paires en collisions

/****************************************************************/
static int REQUIRED_COLLISIONS_INIT = 50;
/****************************************************************/

extern MENU_ROBOT *ROBOTS_FORM;
extern FL_OBJECT* WITH_GOAL_DIR_OBJ;
extern FL_OBJECT* ML_RRT_METHOD;
extern FL_OBJECT* MY_RRT_METHOD;
extern FL_OBJECT* MANHATTAN_CHECK;

/*********************/
static int AA_courant;
static int AA_list[NB_MAX_AA];     // on se limite donc a 2000 AA dans la proteine
static double PrevVdwRadius;
static p3d_poly *poly_en_col[NB_MAX_COL_AFFICH];
static int nb_poly_en_col = 0;
/*********************/

FL_FORM  *BIO_COLLISION_FORM = NULL;

extern FL_OBJECT  *bio_collision_obj;

FL_OBJECT *GROUP_MODES;
FL_OBJECT *MODES1_OBJ;
FL_OBJECT *MODES2_OBJ;
FL_OBJECT *MODES3_OBJ;
FL_OBJECT *NBPAIRES_MODES_SLIDER_OBJ;
FL_OBJECT *DISTANCE_MODES_SLIDER_OBJ;
//FL_OBJECT *TEST_CC_BTN_OBJ;

FL_OBJECT *PAS_CC_SLIDER_OBJ;
//FL_OBJECT *PAS_CC_BTN_OBJ;

FL_OBJECT *POURCENT_VDW_SLIDER_OBJ;
FL_OBJECT *PRINTCOLL_OBJ;
FL_OBJECT *REMOVECOLL_OBJ;

FL_OBJECT *MAX_VDW_PATH_OBJ;
FL_OBJECT *VDW_PATH_PROFILE_OBJ;

FL_OBJECT *RIGID_FLEXIBLE_BTN_OBJ;

FL_OBJECT *CHOIX_AA;
FL_OBJECT *SELECT_AA_BTN_OBJ;
FL_OBJECT *AA_DOF_SLIDER_OBJ[NB_MAX_DOF_AA]; /* car les AA ont au plus 8 dof */
FL_OBJECT *AA_DOF_BUTTON_OBJ[NB_MAX_DOF_AA];
FL_OBJECT *RIGIDITE_SC;

FL_OBJECT *LOOP_REF_JNT_OBJ;
FL_OBJECT *LOOP_BTN_OBJ;  // in main bio FORM
FL_FORM   *LOOP_FORM;
FL_OBJECT *LOOP_OK_OBJ;
FL_OBJECT *LOOP_FST_RES_OBJ;
FL_OBJECT *LOOP_LST_RES_OBJ;

FL_FORM   *LIGAND_FORM;
FL_OBJECT *LIGAND_BTN_OBJ;
FL_OBJECT *LIGAND_DOF_BOUTON_OBJ[NB_MAX_DOF_LIGAND];
FL_OBJECT *LIGAND_DOF_SLIDER_OBJ[NB_MAX_DOF_LIGAND];
FL_OBJECT *POS_CHOICE_OBJ;

FL_OBJECT *TEST_LIG_BTN_OBJ;
FL_OBJECT *TEST_LOOP_BTN_OBJ;
FL_OBJECT *STOP_COMPUT_PATH_OBJ;
FL_OBJECT *PATH_LENGTH_OBJ;

FL_OBJECT *AFFICH_BIO_BTN_OBJ;

FL_OBJECT *LIGAND_TYPE_OBJ;

FL_FORM   *FLEXIBLE_TO_RIGID_FORM;
FL_OBJECT *BB_AA_OBJ[1000];
FL_OBJECT *SC_AA_OBJ[1000];
FL_OBJECT *SELECT_AA_OBJ[1000];
FL_FORM   *SELECT_AA_FORM;

FL_OBJECT *LOOP_STAT_OBJ;
FL_OBJECT *PASSIVE_SCH_OBJ;
FL_OBJECT *EVAL_TRAJ_OBJ;
FL_OBJECT *PDB_NODES_OBJ;

FL_OBJECT *BIO_PRINT_JNTS_OBJ;

FL_OBJECT *BIO_GOAL_P3D_OBJ;

FL_OBJECT *PDB_TRAJ_OBJ;
FL_OBJECT *CURRENT_CONF_TO_PDB_OBJ;

FL_FORM    *PDB_TRAJ_FORM = NULL;
FL_OBJECT  *pdb_OKbutton_obj;
FL_OBJECT  *pdb_dir_obj;
FL_OBJECT  *pdb_numberframes_obj;

#ifdef ENERGY
	FL_OBJECT  *RADIO_AMBER = NULL, *RADIO_NONE = NULL, *RADIO_MOVE3D = NULL, *RADIO_RRT = NULL;
#endif
	FL_OBJECT  *pdb_CANCELbutton = NULL;




/*******************************************************************/

static void CB_modes_obj(FL_OBJECT *ob, long arg);
static void CB_nbpaires_modes_slider_obj(FL_OBJECT *ob, long arg);
static void CB_distance_modes_slider_obj(FL_OBJECT *ob, long arg);
#ifdef ENERGY
static void CB_RadioUseEnergyMinimization_OnChange(FL_OBJECT *ob, long ar);
#endif
static void g3d_create_modes(void);
static void g3d_delete_modes(void);

static void CB_pas_CC_slider_obj(FL_OBJECT *ob, long arg);
//static void CB_pas_CC_btn_obj(FL_OBJECT *ob, long arg);
static void g3d_create_pas_CC(void);
static void g3d_delete_pas_CC(void);

static void CB_pourcent_vdw_slider_obj(FL_OBJECT *ob, long arg);
static void CB_printcoll_obj(FL_OBJECT *ob, long arg);
static void g3d_create_resize(void);
static void g3d_delete_resize(void);
static void g3d_create_printcoll(void);
static void g3d_delete_printcoll(void);

static void CB_removecoll_obj(FL_OBJECT *ob, long arg);
static void g3d_create_removecoll_obj(void);
static void g3d_delete_removecoll_obj(void);
static void CB_ComputeBestVdW_obj(FL_OBJECT *ob, long arg);
static void CB_ComputeVdWProfile_obj(FL_OBJECT *ob, long arg);
static void g3d_create_TestMaxVdWPath_obj(void);
static void g3d_delete_TestMaxVdWPath_obj(void);
static void g3d_create_VdWPathProfile_obj(void);
static void g3d_delete_VdWPathProfile_obj(void);

static void g3d_create_move_AA_obj(void);
static void g3d_delete_move_AA_obj(void);

static void CB_loop_ref_jnt_obj(FL_OBJECT *ob, long arg);
static void g3d_create_loop_ref_jnt_obj(void);
static void g3d_delete_loop_ref_jnt_obj(void);

static void CB_loop_btn_obj(FL_OBJECT *ob, long arg);
static void g3d_create_loop_btn_obj(void);
static void g3d_delete_loop_btn_obj(void);
static void g3d_create_loop_form(void);
static void g3d_delete_loop_form(void);
static void CB_loop_ok_obj(FL_OBJECT *ob, long arg);
static void g3d_create_loop_ok_obj(void);
static void g3d_delete_loop_ok_obj(void);
static void g3d_create_loop_fst_res_obj(void);
static void g3d_delete_loop_fst_res_obj(void);
static void g3d_create_loop_lst_res_obj(void);
static void g3d_delete_loop_lst_res_obj(void);

static void g3d_create_ligand_form(void);
static void g3d_delete_ligand_form(void);
static void CB_ligand_btn_obj(FL_OBJECT *ob, long arg);
static void g3d_create_ligand_btn_obj(void);
static void g3d_delete_ligand_btn_obj(void);

static void CB_test_lig_btn_obj(FL_OBJECT *ob, long arg);
static void g3d_create_test_lig_btn_obj(void);
static void g3d_delete_test_lig_btn_obj(void);

static void CB_test_loop_btn_obj(FL_OBJECT *ob, long arg);
static void g3d_create_test_loop_btn_obj(void);
static void g3d_delete_test_loop_btn_obj(void);

static void CB_stop_comput_path_obj(FL_OBJECT *ob, long arg);
static void g3d_create_stop_comput_path_obj(void);
static void g3d_delete_stop_comput_path_obj(void);

static void CB_path_length_obj(FL_OBJECT *ob, long arg);
static void g3d_create_path_length_obj(void);
static void g3d_delete_path_length_obj(void);

static void CB_affich_bio_obj(FL_OBJECT *ob, long arg);
static void g3d_create_affich_bio_obj(void);
static void g3d_delete_affich_bio_obj(void);

static void CB_ligand_type_obj(FL_OBJECT *ob, long arg);
static void g3d_create_ligand_type_obj(void);
static void g3d_delete_ligand_type_obj(void);

static void CB_loop_stat_obj(FL_OBJECT *ob, long arg);
static void g3d_create_loop_stat_obj(void);
static void g3d_delete_loop_stat_obj(void);

static void CB_passive_sch_obj(FL_OBJECT *ob, long arg);
static void g3d_create_passive_sch_obj(void);
static void g3d_delete_passive_sch_obj(void);

static void CB_eval_traj_obj(FL_OBJECT *ob, long arg);
static void g3d_create_eval_traj_obj(void);
static void g3d_delete_eval_traj_obj(void);

static void CB_pdb_nodes_obj(FL_OBJECT *ob, long arg);
static void g3d_create_pdb_nodes_obj(void);
static void g3d_delete_pdb_nodes_obj(void);

static void CB_bio_print_jnts_obj(FL_OBJECT *ob, long arg);
static void g3d_create_bio_print_jnts_obj(void);
static void g3d_delete_bio_print_jnts_obj(void);

static void CB_bio_goal_p3d_obj(FL_OBJECT *ob, long arg);
static void g3d_create_bio_goal_p3d_obj(void);
static void g3d_delete_bio_goal_p3d_obj(void);

static void CB_pdb_traj_obj(FL_OBJECT *ob, long arg);
static void g3d_create_pdb_traj_obj(void);
static void g3d_delete_pdb_traj_obj(void);

static void g3d_create_pdb_traj_form(void);
static void g3d_delete_pdb_traj_form(void);  // <- TO DO
static void CB_pdbOK_obj(FL_OBJECT *ob, long arg);
static void CB_pdbCANCEL_OnClick(FL_OBJECT *ob, long arg);

void WriteCurrentConfigurationToPDBFile( char *pdbFileName );
static void CB_current_conf_to_pdb_obj(FL_OBJECT *ob, long arg);

static int CB_WritePDBAlongPathWindow_OnClose( FL_FORM *form, void *argument );


/****************************************************/

static void test_affichage(void);
static void init_ligand_autocol(int index_ligtype);
static void init_ligand_autocol_from_file(void);
static void bio_draw_allwin(int ncol);
void afficher_lescollisions(void);
//double get_VdW_pourcent(void);


/****************************************************************/
/* fonctions du collision checker bio utilisees ici             */
/****************************************************************/
/*void bio_set_col_mode(int mode);
void set_required_collisions(int rc); */
/* void bio_set_surface_d(double distance); */
/* void bio_resize_molecule(int nrobot, double  shrink); */
/* void bio_resize_molecules(double shrink); */
/* void bio_true_resize_molecule(int nrobot); */
/* void bio_true_resize_molecules(); */
/* void biocol_report(int *col_number, p3d_poly ***list1, p3d_poly ***list2); */
/* void bio_create_molecules(double scale); */
/* int biocol_robot_report(int nrobot); */
/* int bio_all_molecules_col_with_report(void); */
/* /\* void set_charlatan(int i); *\/ */
/* void bio_set_autocol(int nrobot, int nforbidden, int  *nrigid1,  */
/* 		     int *natom1, int  *nrigid2, int *natom2); */
/* void set_affichage_en_rouge(int affich); */
/****************************************************************/
/****************************************************************/




/****************************************************************/
/****************************************************************/


void g3d_create_bio_collision_form(void)
{
  if(p3d_col_get_mode()!=p3d_col_mode_bio)  return; // modif Juan

  BIO_COLLISION_FORM = fl_bgn_form(FL_UP_BOX,400.0,630.0);
  g3d_create_modes();
  g3d_create_pas_CC();
  g3d_create_resize();
  g3d_create_printcoll();
  g3d_create_removecoll_obj();
  g3d_create_TestMaxVdWPath_obj();
  g3d_create_VdWPathProfile_obj();
  g3d_create_move_AA_obj();
  g3d_create_ligand_type_obj();
  g3d_create_loop_stat_obj();
  g3d_create_bio_print_jnts_obj();
  g3d_create_bio_goal_p3d_obj();
  g3d_create_pdb_traj_obj();
  g3d_create_passive_sch_obj();
  g3d_create_eval_traj_obj();
  g3d_create_pdb_nodes_obj();
  g3d_create_loop_ref_jnt_obj();
  g3d_create_loop_btn_obj();
/*   g3d_create_rigid_flexible(); */
/*   g3d_create_select_AA(); */
  if(num_subrobot_ligand(XYZ_ROBOT)>=0) g3d_create_ligand_btn_obj();
  g3d_create_affich_bio_obj();
  g3d_create_test_lig_btn_obj();
  g3d_create_test_loop_btn_obj();
  g3d_create_stop_comput_path_obj();
  g3d_create_path_length_obj();
  fl_end_form();

  g3d_create_pdb_traj_form();   /* Cree la fenetre PDB */

/*   g3d_create_pane_AA();  */
/*   g3d_create_select_form();  */
  if(num_subrobot_ligand(XYZ_ROBOT)>=0) g3d_create_ligand_form();
  // ININT BioCD
  //if(p3d_col_get_mode()==p3d_col_mode_bio)  {
    bio_set_col_mode(NORMAL_BIOCOL_MODE);
    set_required_collisions(REQUIRED_COLLISIONS_INIT);
    SetPrevVdw(1.0);
    /*   bio_create_molecules(1.0);   */
    bio_true_resize_molecules();
    puts("avt le set_autocol");
    init_ligand_autocol(0);
    puts("apres le set_autocol");
    /*   set_charlatan(0); */
    puts("Initialisation du bio collision checker");
    /*   test_affichage(); */
    //}
  AA_courant = 0;
}

void g3d_delete_bio_collision_form(void)
{
  if(BIO_COLLISION_FORM == NULL) return;  // modif Juan

  if(fl_get_button(bio_collision_obj))
    fl_hide_form(BIO_COLLISION_FORM);

  if(fl_get_button(PDB_TRAJ_OBJ))
    fl_hide_form(PDB_TRAJ_FORM);
  g3d_delete_pdb_traj_form();

  g3d_delete_modes();
  g3d_delete_pas_CC();
  g3d_delete_resize();
  g3d_delete_printcoll();
  g3d_delete_removecoll_obj();
  g3d_delete_TestMaxVdWPath_obj();
  g3d_delete_VdWPathProfile_obj();
  g3d_delete_move_AA_obj();
  g3d_delete_ligand_type_obj();
  g3d_delete_loop_stat_obj();
  g3d_delete_bio_print_jnts_obj();
  g3d_delete_bio_goal_p3d_obj();
  g3d_delete_pdb_traj_obj();
  g3d_delete_passive_sch_obj();
  g3d_delete_eval_traj_obj();
  g3d_delete_pdb_nodes_obj();
/*   g3d_delete_rigid_flexible(); */
/*   g3d_delete_select_AA(); */
  g3d_delete_test_lig_btn_obj();
  g3d_delete_test_loop_btn_obj();
  g3d_delete_stop_comput_path_obj();
  g3d_delete_path_length_obj();
  g3d_delete_affich_bio_obj();
  g3d_delete_loop_ref_jnt_obj();
  g3d_delete_loop_btn_obj();
  if(num_subrobot_ligand(XYZ_ROBOT)>=0) g3d_delete_ligand_btn_obj();
/*   g3d_delete_select_form(); */
/*   g3d_delete_pane_AA(); */
  if(num_subrobot_ligand(XYZ_ROBOT)>=0) g3d_delete_ligand_form();

  fl_free_form(BIO_COLLISION_FORM);
}


/************************************************/
/* fenetre "choix du mode" du collision checker */
/************************************************/

static void CB_modes_obj(FL_OBJECT *ob, long arg)
{
/* Suivant la valeur du parametre arg rentrer dans un certain mode : */
  if(arg == 0){bio_set_col_mode(NORMAL_BIOCOL_MODE);}
  if(arg == 1){bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);}
  if(arg == 2){bio_set_col_mode(MINIMUM_DISTANCE_BIOCOL_MODE);}
}

static void CB_nbpaires_modes_slider_obj(FL_OBJECT *ob, long arg)
{
  set_required_collisions((int)fl_get_slider_value(NBPAIRES_MODES_SLIDER_OBJ));
}

static void CB_distance_modes_slider_obj(FL_OBJECT *ob, long arg)
{
  bio_set_surface_d(fl_get_slider_value(DISTANCE_MODES_SLIDER_OBJ));
}

/* static void CB_test_cc_btn_obj(FL_OBJECT *ob, long arg) */
/* {  */
/*   test_perf(); */
/* } */

static void g3d_create_modes(void)
{
  FL_OBJECT *obj;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,20,360,100,"");
  obj = fl_add_box(FL_FLAT_BOX,160,15,80,10,"BioCD Mode");

  /* le groupe de boutons radio */
  GROUP_MODES = fl_bgn_group();
  MODES1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,40,25,50,30,"Normal");
  fl_set_object_color(MODES1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(MODES1_OBJ,CB_modes_obj,0);
  MODES2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,40,55,50,30,"Surface Distance");
  fl_set_object_color(MODES2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(MODES2_OBJ,CB_modes_obj,1);
  MODES3_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,40,85,50,30,"Minimum Distance");
  fl_set_object_color(MODES3_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(MODES3_OBJ,CB_modes_obj,2);
  //GROUP_MODES = 
  fl_end_group();
  fl_set_button(MODES1_OBJ,1);

  /* les deux sliders de parametres */
  NBPAIRES_MODES_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,200,30,170.0,25.0,"Number of Pairs");
  fl_set_slider_step(NBPAIRES_MODES_SLIDER_OBJ,.1);
  fl_set_slider_bounds(NBPAIRES_MODES_SLIDER_OBJ,0,100);
  fl_set_slider_value(NBPAIRES_MODES_SLIDER_OBJ,REQUIRED_COLLISIONS_INIT);
  fl_set_object_callback(NBPAIRES_MODES_SLIDER_OBJ,CB_nbpaires_modes_slider_obj,0);

  DISTANCE_MODES_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,200,75,170.0,25.0,"Distance");
  fl_set_slider_step(DISTANCE_MODES_SLIDER_OBJ,0.1);
  fl_set_slider_bounds(DISTANCE_MODES_SLIDER_OBJ,-5,10);
  fl_set_slider_value(DISTANCE_MODES_SLIDER_OBJ,0);
  fl_set_object_callback(DISTANCE_MODES_SLIDER_OBJ,CB_distance_modes_slider_obj,0);

  /* le bouton de test */
/*   TEST_CC_BTN_OBJ = fl_add_button(FL_TOUCH_BUTTON, 30,120,290,25,"Test de collisions"); */
/*   fl_set_call_back(TEST_CC_BTN_OBJ,CB_test_cc_btn_obj,0); */
}

static void g3d_delete_modes(void)
{
  fl_free_object(GROUP_MODES);
  fl_free_object(MODES1_OBJ);
  fl_free_object(MODES2_OBJ);
  fl_free_object(MODES3_OBJ);
  fl_free_object(NBPAIRES_MODES_SLIDER_OBJ);
}


/*****************************************************************************/
/* fenetre pour modifier le pas statique lors du parcours d'une trajectoire  */
/*****************************************************************************/

static void CB_pas_CC_slider_obj(FL_OBJECT *ob, long arg)
{
  bio_col_set_step_deplacement(fl_get_slider_value(PAS_CC_SLIDER_OBJ) );
}

/* static void CB_pas_CC_btn_obj(FL_OBJECT *ob, long arg) */
/* { */
/*   bio_col_set_step_deplacement(fl_get_slider_value(PAS_CC_SLIDER_OBJ) ); */
/* } */

static void g3d_create_pas_CC(void)
{
  FL_OBJECT *obj;
  double step_CC;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,130,360,40,"");
  obj = fl_add_box(FL_FLAT_BOX,60,125,280,10,"Collision Checking Step on Local Paths (Angstroms)");

  bio_col_get_step_deplacement(&step_CC);
  PAS_CC_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,140,340,25,"");
  fl_set_slider_step(PAS_CC_SLIDER_OBJ,0.01);
  fl_set_slider_bounds(PAS_CC_SLIDER_OBJ,0.01,0.6);
  fl_set_slider_value(PAS_CC_SLIDER_OBJ,step_CC);
  fl_set_object_callback(PAS_CC_SLIDER_OBJ,CB_pas_CC_slider_obj,0);

/*   PAS_CC_BTN_OBJ = fl_add_button(FL_TOUCH_BUTTON, 325,130,45,25, "Modify"); */
/*   fl_set_call_back(PAS_CC_BTN_OBJ,CB_pas_CC_btn_obj,0); */
}

static void g3d_delete_pas_CC(void)
{
  fl_free_object(PAS_CC_SLIDER_OBJ);
  //fl_free_object(PAS_CC_BTN_OBJ);
}


/**************************************************************/
/* fenetre "taille des atomes" pour redimensionner les atomes */
/**************************************************************/

static void CB_pourcent_vdw_slider_obj(FL_OBJECT *ob, long arg)
{
  int ncol = 0;
  double val = fl_get_slider_value(POURCENT_VDW_SLIDER_OBJ);

  bio_resize_molecules(val/GetPrevVdw());
  //  PrintInfo(("val: %f\n",val ));
  bio_true_resize_molecules();

  SetPrevVdw(val);

  /* collision checking */
  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);
}


double GetPrevVdw(void) {
  return PrevVdwRadius;
}

void SetPrevVdw(double prevVdwRadius) {
  PrevVdwRadius = prevVdwRadius;
}


static void g3d_create_resize(void)
{
  FL_OBJECT *obj;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,180,360,40,"");
  obj = fl_add_box(FL_FLAT_BOX,130,175,140,10,"Atom Size (% of VdW radii)");

  POURCENT_VDW_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,190,340,25,"");
  fl_set_slider_step(POURCENT_VDW_SLIDER_OBJ,0.01);
  fl_set_slider_bounds(POURCENT_VDW_SLIDER_OBJ,0.0001,1);
  fl_set_slider_value(POURCENT_VDW_SLIDER_OBJ,1.0);
  fl_set_object_callback(POURCENT_VDW_SLIDER_OBJ,CB_pourcent_vdw_slider_obj,0);
}

static void g3d_delete_resize(void)
{
  fl_free_object(POURCENT_VDW_SLIDER_OBJ);
}

/***************************************************************************/

static void CB_printcoll_obj(FL_OBJECT *ob, long arg)
{
/*   int nb_col = 0; */

/*   nb_col = bio_all_molecules_col_with_report();  */
  puts("test de collisions fait");

  afficher_lescollisions();
}

static void g3d_create_printcoll(void)
{
  PRINTCOLL_OBJ = fl_add_button(FL_TOUCH_BUTTON,20,230,60,25, "Print Col.");
  fl_set_call_back(PRINTCOLL_OBJ,CB_printcoll_obj,0);
}



static void g3d_delete_printcoll(void)
{
  fl_free_object(PRINTCOLL_OBJ);
}

/***************************************************************************/

static void CB_removecoll_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robPt;
  configPt q;

  robPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  q = p3d_alloc_config(robPt);
  p3d_get_robot_config_into(robPt,&q);

  if (fl_get_button(ob)) {
    bio_deform_schs_avoiding_collision(robPt,q,1);
  }
  p3d_set_and_update_this_robot_conf_without_cntrt(robPt,q);

  printf("Updated INIT and GOTO positions\n");
  p3d_copy_config_into(robPt, q, &(robPt->ROBOT_POS));
  p3d_copy_config_into(robPt, q, &(robPt->ROBOT_GOTO));

  bio_draw_allwin(0);

  p3d_destroy_config(robPt, q);
}

static void g3d_create_removecoll_obj(void)
{
  REMOVECOLL_OBJ = fl_add_button(FL_TOUCH_BUTTON,80,230,70,25, "Remove Col.");
  fl_set_call_back(REMOVECOLL_OBJ,CB_removecoll_obj,0);
}

static void g3d_create_TestMaxVdWPath_obj(void) {
  MAX_VDW_PATH_OBJ = fl_add_button(FL_TOUCH_BUTTON,150,230,60,25, "VdW Path");
  fl_set_call_back(MAX_VDW_PATH_OBJ,CB_ComputeBestVdW_obj,0);
}


static void g3d_create_VdWPathProfile_obj(void) {
  VDW_PATH_PROFILE_OBJ = fl_add_button(FL_TOUCH_BUTTON,210,230,70,25, "VdW Profile");
  fl_set_call_back(VDW_PATH_PROFILE_OBJ,CB_ComputeVdWProfile_obj,0);
}



static void g3d_delete_TestMaxVdWPath_obj(void)
{
  fl_free_object(MAX_VDW_PATH_OBJ);
}

static void g3d_delete_VdWPathProfile_obj(void)
{
  fl_free_object(MAX_VDW_PATH_OBJ);
}


static void g3d_delete_removecoll_obj(void)
{
  fl_free_object(REMOVECOLL_OBJ);
}

/***************************************************************************/

//double get_VdW_pourcent(void)
//{
//  return PrevVdwRadius;
//}

/***************************************************************************/
/* Bouton pour activer/descativer l'affichage des col en mode bio          */
/***************************************************************************/

static void CB_affich_bio_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);
  int ncol = 0;

  if(val) set_affichage_en_rouge(FALSE);
  else    set_affichage_en_rouge(TRUE);

  /* collision checking */
  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);
}

static void g3d_create_affich_bio_obj(void)
{
  AFFICH_BIO_BTN_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,280,255,65,20,"Loc.Col.Display");

  fl_set_object_color(AFFICH_BIO_BTN_OBJ,FL_MCOL,FL_RED);
  fl_set_button(AFFICH_BIO_BTN_OBJ,0);
  fl_set_call_back(AFFICH_BIO_BTN_OBJ,CB_affich_bio_obj,0);
}

static void g3d_delete_affich_bio_obj(void)
{
  fl_free_object(AFFICH_BIO_BTN_OBJ);
}


/*****************************************************************/
/* Tout ce qu'il faut pour deplacer les sidechain de la proteine */
/*****************************************************************/

// CONCERNE LE BOUTON POUR RENDRE RIGIDE/FLEXIBLE
static void CB_rigidite_sc_obj(FL_OBJECT *ob, long arg)
{
  // changer la rigidite de la chaine lat //
  int val = fl_get_button(ob);
  int num_jnt_rob = 0;
  p3d_jnt *jnt_AA;
  char AAtype[4];
  int numero_slider = 2;

  num_jnt_rob = get_AAfirstjnt_number(AA_courant);
  num_jnt_rob++;    // on se place sur le second joint de l'AA car on s'interesse aux sc
  jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
  if(jnt_AA->name[0] == 'p')
    {
      num_jnt_rob++;
      jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
    }
  get_AAtype_from_name(jnt_AA->o->name, AAtype);

  while( (jnt_AA->name[0]=='g')
	 && ( !(strcmp(AAtype,"LIG")==0) ) )
    {
      if(val == 0) //le rendre rigide
	{
	  p3d_jnt_set_dof_bounds(jnt_AA,0,jnt_AA->v,jnt_AA->v);
	  p3d_jnt_set_dof_rand_bounds(jnt_AA,0,jnt_AA->v,jnt_AA->v);
	}
      else // le rendre flexible
	{
	  p3d_jnt_set_dof_bounds(jnt_AA,0,-3.1415926535897931,3.1415926535897931);
	  p3d_jnt_set_dof_rand_bounds(jnt_AA,0,-3.1415926535897931,3.1415926535897931);
	}

      fl_set_slider_bounds(AA_DOF_SLIDER_OBJ[numero_slider],RTOD(jnt_AA->vmin),RTOD(jnt_AA->vmax));
      fl_set_slider_value(AA_DOF_SLIDER_OBJ[numero_slider],RTOD(jnt_AA->v));

      if(jnt_AA->vmin != jnt_AA->vmax)
	{
	  fl_set_slider_size(AA_DOF_SLIDER_OBJ[numero_slider],0.1);
	}
      else fl_set_slider_size(AA_DOF_SLIDER_OBJ[numero_slider],1.0);

      numero_slider++;
      num_jnt_rob++;
      jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
      get_AAtype_from_name(jnt_AA->o->name, AAtype);
    }
}

// POUR LE BOUTON DE CHOIX DE L'AA ARTICULE
static void CB_move_AA_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_choice(ob);
  int i = 0;
  int num_jnt_rob = get_AAfirstjnt_number(AA_courant);
  p3d_jnt *jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
  p3d_poly *poly;
  int encoreUnJoint = 1;
  int numero_joint = 0;
  char AAtype[4];
  int flexibilite = 0;
  int ncol = 0;
  double color_vect[4];

  // CHANGER LES COULEURS de l'acide amine courant : lui remettre ses bonnes couleurs
  if(AA_courant != 0)
    {
      while(encoreUnJoint)
	{
	  /// faire des trucs communs a tous
	  if(jnt_AA->o)
	    {
	      for(i=0;i<jnt_AA->o->np;i++)
		{
		  poly = jnt_AA->o->pol[i];
		  switch(poly->type)
		    {
		    case CARBON: p3d_poly_set_color(poly,Green,NULL);
		      break;
		    case OXYGEN: p3d_poly_set_color(poly,Red,NULL);
		      break;
		    case OXYGEN_H: p3d_poly_set_color(poly,Red,NULL);
		      break;
		    case NITROGEN : p3d_poly_set_color(poly,Blue,NULL);
		      break;
		    case NITROGEN_H : p3d_poly_set_color(poly,Blue,NULL);
		      break;
		    case HYDROGEN : p3d_poly_set_color(poly,White,NULL);
		      break;
		    case SULPHUR : p3d_poly_set_color(poly,Yellow,NULL);
		      break;
		    case SULPHUR_H : p3d_poly_set_color(poly,Yellow,NULL);
		      break;
		    case BROMINE :
		      color_vect[0] = 153; color_vect[0] = 83; color_vect[2] = 4; color_vect[3] = 1;
		      p3d_poly_set_color(poly,Any,color_vect);
		      break;
		    default : p3d_poly_set_color(poly,Black,NULL);
		    }
		}
	      // pas encore traites //
	      /* ,BROMINE, IODINE, FLUORINE, PHOSPHORUS, CHLORINE, NITROGEN_FULL */
	    }
	  // fin commun
	  if(num_jnt_rob < XYZ_ROBOT->njoints)
	    {
	      num_jnt_rob++;
	      jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
	      if(jnt_AA->o)
		{
		  get_AAtype_from_name(jnt_AA->o->name, AAtype);
		  // on a encore un joint si jnt_AA->name est different de omega
		  // et que le joint n'appartient pas au ligand
		  encoreUnJoint = ( (jnt_AA->name[0]!='o') && ( !(strcmp(AAtype,"LIG")==0) ) );
		}
	    }
	  else encoreUnJoint = 0;
	}
    }
  encoreUnJoint = 1;
  // FIN CHANGER LES COULEURS

  AA_courant = AA_list[val-2];
  num_jnt_rob = get_AAfirstjnt_number(AA_courant);
  jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];

  while(encoreUnJoint)
    {
      if((jnt_AA->name[0]=='p')&&(jnt_AA->name[1]=='s'))
	{ /* cas ou on arrive au dernier joint */
	  for(i=numero_joint;i<7;i++)
	    {
	      fl_set_slider_bounds(AA_DOF_SLIDER_OBJ[i],0,0);
	      fl_set_slider_value(AA_DOF_SLIDER_OBJ[i],0);
	      fl_set_slider_size(AA_DOF_SLIDER_OBJ[i],1.0);
	      numero_joint = 7;
	      encoreUnJoint = 0;
	    }
	}
      if((jnt_AA->name[0]=='g')&&(numero_joint<3))
      { /* cas ou on n'a pas de omega et/ou de phi */
	for(i=numero_joint;i<2;i++)
	  {
	    fl_set_slider_bounds(AA_DOF_SLIDER_OBJ[i],0,0);
	    fl_set_slider_value(AA_DOF_SLIDER_OBJ[i],0);
	    fl_set_slider_size(AA_DOF_SLIDER_OBJ[i],1.0);
	    numero_joint = 2;
	  }
      }

      fl_set_slider_bounds(AA_DOF_SLIDER_OBJ[numero_joint],RTOD(jnt_AA->vmin),RTOD(jnt_AA->vmax));
      fl_set_slider_value(AA_DOF_SLIDER_OBJ[numero_joint],RTOD(jnt_AA->v));
      for(i=0;i<jnt_AA->o->np;i++)
	{
	  poly = jnt_AA->o->pol[i];
	  p3d_poly_set_color(poly,Yellow,NULL);
	}

      if(jnt_AA->vmin != jnt_AA->vmax)
	{
	  fl_set_slider_size(AA_DOF_SLIDER_OBJ[numero_joint],0.1);
	  flexibilite = 1;
	}
      else fl_set_slider_size(AA_DOF_SLIDER_OBJ[numero_joint],1.0);

      if((num_jnt_rob < XYZ_ROBOT->njoints)&&(numero_joint <7 ))
	{
	  num_jnt_rob++;
	  jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
	  get_AAtype_from_name(jnt_AA->o->name, AAtype);
	  // on a encore un joint si jnt_AA->name est different de omega
	  // et que le joint n'appartient pas au ligand
	  encoreUnJoint = ( (jnt_AA->name[0]!='o') && ( !(strcmp(AAtype,"LIG")==0) ) );
	}
      else encoreUnJoint = 0;

      numero_joint++;

      if(!encoreUnJoint)
	{
	  for(i=numero_joint;i<NB_MAX_DOF_AA;i++)
	    {
	      fl_set_slider_bounds(AA_DOF_SLIDER_OBJ[i],0,0);
	      fl_set_slider_value(AA_DOF_SLIDER_OBJ[i],0);
	      fl_set_slider_size(AA_DOF_SLIDER_OBJ[i],1.0);
	    }
	}
    }
  if(flexibilite) fl_set_button(RIGIDITE_SC,1);
  else fl_set_button(RIGIDITE_SC,0);

  /* collision checking */
  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);
}


// POUR LES DIFFERENTS SLIDERS
static void CB_dof_slider_obj(FL_OBJECT *ob, long arg)
{
  int num_jnt_rob = get_AAfirstjnt_number(AA_courant);
  p3d_jnt *jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
  int encoreUnJoint = 1;
  int numero_joint = 0;
  int ncol = 0;
  char AAtype[4];

  if(AA_courant == 1)
    {
      /* on n'a pas de omega on est donc au phi */

      p3d_set_robot_jnt(jnt_AA->num,DTOR(fl_get_slider_value(AA_DOF_SLIDER_OBJ[1])));
      jnt_AA->mat_modified = TRUE;

      /* pour les joints suivants */
      encoreUnJoint = next_joint(&jnt_AA, &jnt_AA, encoreUnJoint);
      p3d_set_robot_jnt(jnt_AA->num,DTOR(fl_get_slider_value(AA_DOF_SLIDER_OBJ[7])));
      jnt_AA->mat_modified = TRUE;

      /* on place ds jnt_AA un pointeur vers le joint gamma1 */
      jnt_AA = jnt_AA->prev_jnt->prev_jnt->next_jnt[1];
      numero_joint = 2;
      encoreUnJoint = 1;
      while(encoreUnJoint != 0)
	{
	  p3d_set_robot_jnt(jnt_AA->num,DTOR(fl_get_slider_value(AA_DOF_SLIDER_OBJ[numero_joint])));
	  jnt_AA->mat_modified = TRUE;
	  encoreUnJoint = next_joint(&jnt_AA, &jnt_AA, encoreUnJoint);
	  numero_joint++;
	}
    }
  else
    {
      while(encoreUnJoint)
	{
	  if((jnt_AA->name[0]=='p')&&(jnt_AA->name[1]=='s'))
	    {
	      /* cas ou on arrive au dernier joint */
	      numero_joint = 7;
	    }
	  if((jnt_AA->name[0]=='g')&&(numero_joint<3))
	    { /* cas ou on n'a pas de omega et/ou de phi */
	      numero_joint = 2;
	    }
	  if( DTOR(fl_get_slider_value(AA_DOF_SLIDER_OBJ[numero_joint])) != jnt_AA->v )
	    jnt_AA->mat_modified = TRUE;
	  p3d_set_robot_jnt(num_jnt_rob,DTOR(fl_get_slider_value(AA_DOF_SLIDER_OBJ[numero_joint])));

	  if( (num_jnt_rob < XYZ_ROBOT->njoints)&&(numero_joint <7 ) )
	    {
	      num_jnt_rob++;
	      jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
	      get_AAtype_from_name(jnt_AA->o->name, AAtype);
	      encoreUnJoint = ( (jnt_AA->name[0]!='o') && ( !(strcmp(AAtype,"LIG")==0) ) );
	    }
	  else encoreUnJoint = 0;

	  numero_joint++;

	}
    }

  p3d_update_robot_pos();

  /* collision checking */
  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);
}


static void g3d_create_move_AA_obj(void)
{
  int i = 0;
  char angle[8][7] = {"omega","phi","gamma1","gamma2","gamma3","gamma4","gamma5","psi"};
  int j = 0;
  int k = 0;
  int numAA = 0;
  int ancienNumAA = 0;
  char AAtype[4];
  char s_numAA[6];
  char tiret[2] = {'-'};
  char nom[15] = {};

  for(i=0;i<NB_MAX_DOF_AA;i++)
    {
      AA_DOF_BUTTON_OBJ[i] = fl_add_box(FL_UP_BOX,20,350+20*i,60,20,angle[i]);
      AA_DOF_SLIDER_OBJ[i] = fl_add_valslider(FL_HOR_SLIDER,80,350+20*i,300,20,"");
      fl_set_slider_step(NBPAIRES_MODES_SLIDER_OBJ,10);
      fl_set_slider_bounds(AA_DOF_SLIDER_OBJ[i],0,0);
      fl_set_slider_value(AA_DOF_SLIDER_OBJ[i],0);
      fl_set_object_callback(AA_DOF_SLIDER_OBJ[i],CB_dof_slider_obj,i);
    }

  CHOIX_AA = fl_add_choice(FL_NORMAL_CHOICE,20,300,100.0,40.0,"");
  fl_set_call_back(CHOIX_AA,CB_move_AA_obj,0);
  fl_addto_choice(CHOIX_AA,"MOV.RESIDUES");
  /* parcours de la liste des objets pour trouver ceux qui sont articules */
  while(j<XYZ_ROBOT->no)
    {
      if(!((XYZ_ROBOT->o[j]->name[0] == 'a') &&
	   (XYZ_ROBOT->o[j]->name[1] == 'l') &&
	   (XYZ_ROBOT->o[j]->name[2] == 'l') &&
	   (XYZ_ROBOT->o[j]->name[3] == '-') ) )
	{
	  get_AAtype_from_name(XYZ_ROBOT->o[j]->name,AAtype);
	  if( !(strcmp(AAtype,"LIG")==0) )  numAA = get_AAnumber_from_name(XYZ_ROBOT->o[j]->name);

	  if((ancienNumAA != numAA )  && !(strcmp(AAtype,"LIG")==0))
	    {
	      nom[0] = '\0';
	      strcat(nom,AAtype);
	      strcat(nom,tiret);
	      sprintf(s_numAA,"%d",numAA);
     	      strcat(nom,s_numAA);
	      //strcat(nom,itoa(numAA));   // modif Juan
	      fl_addto_choice(CHOIX_AA,nom);
	      ancienNumAA = numAA;
	      AA_list[k] = numAA; /* XYZ_ROBOT->o[j]->name; */
	      k++;
	    }
	}
      j++;
    }

  RIGIDITE_SC = fl_add_checkbutton(FL_PUSH_BUTTON,20,510,65,20,"Flexible Side-Chain");
  fl_set_object_color(RIGIDITE_SC,FL_MCOL,FL_RED);
  fl_set_button(RIGIDITE_SC,1);
  fl_set_call_back(RIGIDITE_SC,CB_rigidite_sc_obj,0);

/*   CB_move_AA_obj(CHOIX_AA, 1); */

}

static void g3d_delete_move_AA_obj(void)
{
  int i = 0;
  for(i = 0;i<NB_MAX_DOF_AA;i++)
    {
      fl_free_object(AA_DOF_SLIDER_OBJ[i]);
      fl_free_object(AA_DOF_BUTTON_OBJ[i]);
    }
  fl_free_object(RIGIDITE_SC);
  fl_free_object(CHOIX_AA);
}


/*****************************************************************/
/* Bouton et form gerant le "setting" des loops                  */
/*****************************************************************/

static void g3d_create_loop_form(void)
{
  int i,nloops;
  int cntrt_ind;
  p3d_cntrt *ctPt;
  p3d_rob *robotPt;
  char loopname[10],s_resnum[5];

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  // WARNING : GLOBAL COUNTER FOR POSSIBLY MULTIPLE PROTEINS
  nloops = bio_get_num_loops();

  LOOP_FORM = fl_bgn_form(FL_UP_BOX,220.0,30.0*(nloops+1)+40.0);
  fl_add_text(FL_NORMAL_TEXT,60.0,5.0,55.0,20.0,"First Res.");
  fl_add_text(FL_NORMAL_TEXT,115.0,5.0,55.0,20.0,"Last Res.");
  cntrt_ind = 0;
  for(i=0; i<nloops; i++) {
    do {
      ctPt = robotPt->cntrt_manager->cntrts[cntrt_ind];
      cntrt_ind++;
    } while(strcmp(ctPt->namecntrt,"p3d_6R_bio_ik_nopep_new")!=0);
    sprintf(loopname,"LOOP %d",i+1);
    fl_add_text(FL_NORMAL_TEXT,10.0,27.0+(30.0*(double)i),60.0,20.0,loopname);
    sprintf(s_resnum,"%d",get_AAnumber_from_jnt(ctPt->rlgPt->rlgchPt->rlg_data[0]->jnt));
    fl_add_box(FL_FRAME_BOX,60.0,27.0+(30.0*(double)i),50.0,20.0,s_resnum);
    sprintf(s_resnum,"%d",get_AAnumber_from_jnt(ctPt->pasjnts[7]));
    fl_add_box(FL_FRAME_BOX,115.0,27.0+(30.0*(double)i),50.0,20.0,s_resnum);
  }
  g3d_create_loop_ok_obj();
  g3d_create_loop_fst_res_obj();
  g3d_create_loop_lst_res_obj();
  fl_end_form();
}


static void g3d_delete_loop_form(void)
{
  g3d_delete_loop_ok_obj();
  g3d_delete_loop_fst_res_obj();
  g3d_delete_loop_lst_res_obj();
  fl_free_form(LOOP_FORM);
}


static void CB_loop_ok_obj(FL_OBJECT *ob, long arg)
{
  int fres=0,lres=0;
  int ifj,ilj;
  Joint_tablespt *jnt_table;
  p3d_rob *robotPt=NULL;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  fres = atoi(fl_get_input(LOOP_FST_RES_OBJ));
  lres = atoi(fl_get_input(LOOP_LST_RES_OBJ));

  jnt_table =  give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,fres));

  ifj = jnt_table[fres]->bb_joints[1]->num;
  ilj = jnt_table[lres+1]->bb_joints[0]->num;


  if(!bio_set_loop(ifj,ilj)) {
    printf("ERROR : can't set loop between residues %d - %d\n",fres,lres);
  }

  fl_hide_form(LOOP_FORM);
  g3d_delete_loop_form();
  fl_set_button(LOOP_BTN_OBJ,0);
}


static void g3d_create_loop_ok_obj(void)
{
  int nloops;

  nloops = bio_get_num_loops();
  LOOP_OK_OBJ = fl_add_button(FL_PUSH_BUTTON,170.0,27.0+(30.0*(double)nloops),40.0,20.0,"OK");
  fl_set_call_back(LOOP_OK_OBJ,CB_loop_ok_obj,0);
}


static void g3d_delete_loop_ok_obj(void)
{
  fl_free_object(LOOP_OK_OBJ);
}


static void g3d_create_loop_fst_res_obj(void)
{
  int nloops;

  nloops = bio_get_num_loops();
  LOOP_FST_RES_OBJ = fl_add_input(FL_NORMAL_INPUT,60.0,27.0+(30.0*(double)nloops),50.0,20.0,"");
}


static void g3d_delete_loop_fst_res_obj(void)
{
  fl_free_object(LOOP_FST_RES_OBJ);
}


static void g3d_create_loop_lst_res_obj(void)
{
  int nloops;

  nloops = bio_get_num_loops();
  LOOP_LST_RES_OBJ = fl_add_input(FL_NORMAL_INPUT,115.0,27.0+(30.0*(double)nloops),50.0,20.0,"");
}


static void g3d_delete_loop_lst_res_obj(void)
{
  fl_free_object(LOOP_LST_RES_OBJ);
}


static void CB_loop_ref_jnt_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);
  if(val) {
    CB_create_drawnjnt_obj(NULL,0);
  }
}

static void g3d_create_loop_ref_jnt_obj(void)
{
  LOOP_REF_JNT_OBJ = fl_add_button(FL_TOUCH_BUTTON, 110,265,90,25, "Set Loop\nRef Joint");
  fl_set_call_back(LOOP_REF_JNT_OBJ,CB_loop_ref_jnt_obj,0);
}

static void g3d_delete_loop_ref_jnt_obj(void)
{
  fl_free_object(LOOP_REF_JNT_OBJ);
}


static void CB_loop_btn_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);
  if(val) {
    g3d_create_loop_form();
    fl_show_form(LOOP_FORM,FL_PLACE_SIZE,TRUE, "Loop Setting");
  }
  else {
    fl_hide_form(LOOP_FORM);
    g3d_delete_loop_form();
  }
}

static void g3d_create_loop_btn_obj(void)
{
  LOOP_BTN_OBJ = fl_add_button(FL_PUSH_BUTTON, 20,265,90,25, "Set New Loop");
  fl_set_call_back(LOOP_BTN_OBJ,CB_loop_btn_obj,0);
}

static void g3d_delete_loop_btn_obj(void)
{
  fl_free_object(LOOP_BTN_OBJ);
}

/*****************************************************************/
/* Bouton et form gerant le deplacement du ligand                */
/*****************************************************************/

static void CB_ligand_btn_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);
  if(val) fl_show_form(LIGAND_FORM,FL_PLACE_SIZE,TRUE, "Ligand Location");
  else    fl_hide_form(LIGAND_FORM);
}

static void CB_pos_choice_obj(FL_OBJECT *ob, long arg)
{
  int i=0;
  int val = fl_get_choice(ob);
  configPt p=NULL, p_deg=NULL; /* the same vector expressed in radian and in degree */
  p3d_rob *robotPt=NULL;
  int ncol = 0;
  int nb_dof = nb_dof_ligand(XYZ_ROBOT) ;

  int num_jnt = XYZ_ROBOT->joints[0]->next_jnt[1]->num;   // numero du FreeFlyer du ligand
  p3d_jnt *jnt_lig = XYZ_ROBOT->joints[num_jnt];

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if(val == 1)
    p = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  else
    p = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);

  /* vector p with angles expressed in degree */
  p_deg = p3d_copy_config_rad_to_deg(robotPt, p);

  fl_freeze_form(LIGAND_FORM);

  for(i=0; i<nb_dof; i++)
    {
      fl_set_slider_value(LIGAND_DOF_SLIDER_OBJ[i], p_deg[jnt_lig->index_dof+i]);
    }

  p3d_set_robot_config(robotPt, p);
  p3d_update_robot_pos();

  fl_unfreeze_form(LIGAND_FORM);

  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);

  /* desallocate vectors p and p_deg */
  p3d_destroy_config(robotPt, p);
  p3d_destroy_config(robotPt, p_deg);
}

static void CB_ligand_dof_slider_obj(FL_OBJECT *ob, long arg)
{
/*   int i = 0; */
  int num_jnt = XYZ_ROBOT->joints[0]->next_jnt[1]->num;   // numero du FreeFlyer du ligand
/*   int numero_joint = 0; */
  p3d_jnt *jnt_lig = XYZ_ROBOT->joints[num_jnt];
/*   int index_dof_lig = 0; */

  int ncol = 0;
/*   char AAtype[4]; */

  ////////// modif
  double val = fl_get_slider_value(ob);
  configPt p=NULL, p_deg=NULL;
  p3d_rob *robotPt;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p = p3d_alloc_config(robotPt);
  p_deg = p3d_alloc_config(robotPt);
/*   p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg); */
  if(fl_get_choice(POS_CHOICE_OBJ) == 1){
    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg);
  }
  else{
    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO, &p_deg);
  }
  p_deg[jnt_lig->index_dof+arg]=val;
  p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);

  p3d_set_robot_config(robotPt, p);
/*   p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_POS)); */
  if(fl_get_choice(POS_CHOICE_OBJ) == 1){
    p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_POS));
  }
  else{
    p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_GOTO));
  }

  p3d_update_robot_pos();

  /* collision checking */
  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);

  p3d_destroy_config(robotPt, p_deg);
  p3d_destroy_config(robotPt, p);
}

static void g3d_create_ligand_btn_obj(void)
{
  LIGAND_BTN_OBJ = fl_add_button(FL_PUSH_BUTTON, 150,300,130,40, "Ligand Location");
  fl_set_call_back(LIGAND_BTN_OBJ,CB_ligand_btn_obj,0);
}

static void g3d_delete_ligand_btn_obj(void)
{
  fl_free_object(LIGAND_BTN_OBJ);
}

static void g3d_create_ligand_form(void)
{
  int i = 0;
  int j = 0;
  int nb_dof = nb_dof_ligand(XYZ_ROBOT);
  int num_jnt = XYZ_ROBOT->joints[0]->next_jnt[1]->num;   // numero du FreeFlyer du ligand
  p3d_jnt *jnt_lig = XYZ_ROBOT->joints[num_jnt];


  LIGAND_FORM = fl_bgn_form(FL_UP_BOX,400,90+20*nb_dof);

  POS_CHOICE_OBJ = fl_add_choice(FL_NORMAL_CHOICE,20,20,150.0,40.0,"");
  fl_addto_choice(POS_CHOICE_OBJ,"Current Position");
  fl_addto_choice(POS_CHOICE_OBJ,"Goto    Position");
  fl_set_choice(POS_CHOICE_OBJ,1);
  fl_set_call_back(POS_CHOICE_OBJ,CB_pos_choice_obj,0);

  while(i<nb_dof)
    {
      jnt_lig = XYZ_ROBOT->joints[num_jnt];
      if(jnt_lig->type == P3D_ROTATE)
	{
	  LIGAND_DOF_BOUTON_OBJ[i] = fl_add_box(FL_UP_BOX,20,70+20*i,60,20,jnt_lig->name);
	  LIGAND_DOF_SLIDER_OBJ[i] = fl_add_valslider(FL_HOR_SLIDER,80,70+20*i,300,20,"");
	  fl_set_slider_bounds(LIGAND_DOF_SLIDER_OBJ[i],RTOD(jnt_lig->vmin),RTOD(jnt_lig ->vmax));
	  fl_set_slider_value(LIGAND_DOF_SLIDER_OBJ[i],RTOD(jnt_lig->v));
	  fl_set_object_callback(LIGAND_DOF_SLIDER_OBJ[i],CB_ligand_dof_slider_obj,i);
	  i++;
	}
      if(jnt_lig->type == P3D_FREEFLYER)
	{
	  for(j=0 ; j<3 ; j++)
	    {
	      LIGAND_DOF_BOUTON_OBJ[i] = fl_add_box(FL_UP_BOX,20,70+20*i,60,20,jnt_lig->name);
	      LIGAND_DOF_SLIDER_OBJ[i] = fl_add_valslider(FL_HOR_SLIDER,80,70+20*i,300,20,"");
	      fl_set_slider_bounds(LIGAND_DOF_SLIDER_OBJ[i],jnt_lig->dof_data[j].vmin,jnt_lig->dof_data[j].vmax);
	      fl_set_slider_value(LIGAND_DOF_SLIDER_OBJ[i],jnt_lig->dof_data[j].v0);
	      fl_set_object_callback(LIGAND_DOF_SLIDER_OBJ[i],CB_ligand_dof_slider_obj,i);
	      i++;
	    }
	  for(j=3 ; j<6 ; j++)
	    {
	      LIGAND_DOF_BOUTON_OBJ[i] = fl_add_box(FL_UP_BOX,20,70+20*i,60,20,jnt_lig->name);
	      LIGAND_DOF_SLIDER_OBJ[i] = fl_add_valslider(FL_HOR_SLIDER,80,70+20*i,300,20,"");
	      fl_set_slider_bounds(LIGAND_DOF_SLIDER_OBJ[i],RTOD(jnt_lig->dof_data[j].vmin),RTOD(jnt_lig->dof_data[j].vmax));
	      fl_set_slider_value(LIGAND_DOF_SLIDER_OBJ[i],RTOD(jnt_lig->dof_data[j].v0));
	      fl_set_object_callback(LIGAND_DOF_SLIDER_OBJ[i],CB_ligand_dof_slider_obj,i);
	      i++;
	    }
	}
      num_jnt++;
    }
  fl_end_form();
}

static void g3d_delete_ligand_form(void)
{
  int i = 0;
  int nb_dof = nb_dof_ligand(XYZ_ROBOT);

  if(fl_get_button(bio_collision_obj))
    fl_hide_form(LIGAND_FORM);

  fl_free_object( POS_CHOICE_OBJ );

  for(i=0;i<nb_dof;i++)
    {
      fl_free_object( LIGAND_DOF_BOUTON_OBJ[i] );
      fl_free_object( LIGAND_DOF_SLIDER_OBJ[i] );
    }
  fl_free_form(LIGAND_FORM);
}

/***************************************************************************/
/* Call function bio_loop_graph_statistics                                 */
/***************************************************************************/

static void CB_loop_stat_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(ob)) {
    //#ifdef ENERGY
    fl_set_button(WITH_GOAL_DIR_OBJ,TRUE);
    ENV.setBool(Env::expandToGoal,true);
    bio_search_max_weight_in_curr_rrt();
    //#endif
    //bio_loop_graph_statistics();
    fl_set_button(ob,0);
  }
}

static void g3d_create_loop_stat_obj(void)
{
  LOOP_STAT_OBJ = fl_add_button(FL_PUSH_BUTTON,20,535,120,30,"Goto = Best Node");
  fl_set_call_back(LOOP_STAT_OBJ,CB_loop_stat_obj,0);
}

static void g3d_delete_loop_stat_obj(void)
{
  fl_free_object(LOOP_STAT_OBJ);
}



/***************************************************************************/
/* PRINT JOINTS OF RESIDUES                                                */
/***************************************************************************/

static void CB_bio_print_jnts_obj(FL_OBJECT *ob, long arg)
{
  Joint_tablespt *jnt_table;
  //p3d_rob *robotPt=NULL;
  Robot_structure **bcd_robsPt;
  Robot_structure *bcd_rob_iPt;
  int nres;
  int nfres;
  int nbcdrobs;
  int nr,i,j;
  p3d_jnt *jntPt;
  FILE *fp = NULL;

  if(fl_get_button(ob)) {
    // WARNING :ONLY ONE ROBOT !!!
    fp = fopen("bio_joint_list.txt","w");
    //robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    fprintf(fp,"\n## JOINT NUMBERS ##\n");
    bcd_robsPt = get_bcd_robots();
    nbcdrobs = get_number_of_bcd_robots();
    for (nr=0;nr < nbcdrobs; nr++){
      bcd_rob_iPt = bcd_robsPt[nr];
      nres = bcd_rob_iPt->n_aminos;
      if(nres != 0) {
	jnt_table = bcd_rob_iPt->joint_tables;
	nfres = 0;
	while(*jnt_table == NULL) {
	  jnt_table++;
	  nfres++;
	}
	for (i=nfres;i<nres;i++) {
	  fprintf(fp,"-- residue %d --\n",(*jnt_table)->namino);
	  jntPt = (*jnt_table)->bb_joints[0];
	  if(jntPt != NULL)
	    fprintf(fp,"omega  -> J%d\n",jntPt->num);
	  jntPt = (*jnt_table)->bb_joints[1];
	  if(jntPt != NULL)
	    fprintf(fp,"phi    -> J%d\n",jntPt->num);
	  jntPt = (*jnt_table)->bb_joints[2];
	  if(jntPt != NULL)
	    fprintf(fp,"psi    -> J%d\n",jntPt->num);
	  for(j=0;j<(*jnt_table)->n_sc_joints;j++) {
	    jntPt = (*jnt_table)->sc_joints[j];
	    fprintf(fp,"gamma%d -> J%d\n",j+1,jntPt->num);
	  }
	  if(i != (nres-1)) jnt_table++;
	}
      }
    }
    fl_set_button(ob,0);
    fclose(fp);
    printf("Printing file ./bio_joint_list.txt : DONE\n");
  }
}

static void g3d_create_bio_print_jnts_obj(void)
{
  BIO_PRINT_JNTS_OBJ = fl_add_button(FL_PUSH_BUTTON,260,510,120,20,"Print Joints");
  fl_set_call_back(BIO_PRINT_JNTS_OBJ,CB_bio_print_jnts_obj,0);
}

static void g3d_delete_bio_print_jnts_obj(void)
{
  fl_free_object(BIO_PRINT_JNTS_OBJ);
}



/***************************************************************************/
/* LOAD GOAL P3D MODEL                                                     */
/***************************************************************************/

static void CB_bio_goal_p3d_obj(FL_OBJECT *object, long arg)
{
  if (fl_get_button(object)) {
    if(bio_set_goal_jnt_coordinates())
      printf("DONE : goal jnt coordinates have been set\n");
    else
      printf("ERROR : goal jnt coordinates cannot be set\n");
    fl_set_button(object,0);
  }
}

static void g3d_create_bio_goal_p3d_obj(void)
{
  BIO_GOAL_P3D_OBJ = fl_add_button(FL_PUSH_BUTTON, 150,510,90,20, "Select goal p3d");
  fl_set_call_back(BIO_GOAL_P3D_OBJ, CB_bio_goal_p3d_obj,0);
}

static void g3d_delete_bio_goal_p3d_obj(void)
{
  fl_free_object(BIO_GOAL_P3D_OBJ);
}



/***************************************************************************/
/* save CURRENT_CONF_TO_PDB_OBJ                                 */
/***************************************************************************/

static void CB_current_conf_to_pdb_obj(FL_OBJECT *ob, long arg)
{
	FILE *fPDB;
	char  *pdbFileName = NULL;
	const char *tempPtr = NULL;
	p3d_rob *robotPt = NULL;

	fl_set_button(CURRENT_CONF_TO_PDB_OBJ,0);

	//We now need a file name into which the PDB content will be deposited
	tempPtr = fl_show_fselector("Enter a name for the PDB file to be generated:","../../Energy/","*.pdb", "");

	//Check whether the file can be opened or not
	if ( tempPtr == NULL )
		return;	//The user has pressed CANCEL in the file selector window

	pdbFileName = strdup( tempPtr );
	if (! (fPDB = fopen(pdbFileName, "w") ) )
	{
		fl_show_alert("The file could not be opened !","File does not exist or you do not have the right to open it.",pdbFileName,1);
		return;
	}

	//Now that we have a valid PDB file name, just call the method
	// that writes the current configuration into a PDB file
	robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	translate_conf_to_pdb(robotPt, fPDB);

	fclose(fPDB);

	PrintInfo(("Current configuration saved to file: %s\n", pdbFileName));
}


/***************************************************************************/
/* show PDB_TRAJ_FORM                                                      */
/***************************************************************************/

static void CB_pdb_traj_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(ob)) {
    fl_show_form(PDB_TRAJ_FORM,FL_PLACE_SIZE,TRUE,"PDBs along Path");
    //fl_set_button(ob,0);
  }
}

static void g3d_create_pdb_traj_obj(void)
{
  CURRENT_CONF_TO_PDB_OBJ =  fl_add_button(FL_PUSH_BUTTON,150,535,90,30,"Write a PDB now");
  fl_set_call_back(CURRENT_CONF_TO_PDB_OBJ,CB_current_conf_to_pdb_obj,0);

  PDB_TRAJ_OBJ = fl_add_button(FL_PUSH_BUTTON,260,535,120,30,"Write PDBs along Path");
  fl_set_call_back(PDB_TRAJ_OBJ,CB_pdb_traj_obj,0);
}

static void g3d_delete_pdb_traj_obj(void)
{
  fl_free_object(PDB_TRAJ_OBJ);
}


/***************************************************************************/
/* Activation/deactivation of the side-chains as passive planner params    */
/***************************************************************************/

static void CB_passive_sch_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  if(val) {
    p3d_SetManhattanRrtParam();
    fl_set_button(ML_RRT_METHOD,TRUE);
    //  fl_set_button(MANHATTAN_CHECK, TRUE);

    bio_set_all_sch_dofs_as_passive_parameters_for_planner(robotPt);
  }
  else {
    fl_set_button(MANHATTAN_CHECK, FALSE);
    ENV.setBool(Env::isManhattan,false);
    fl_set_button(MY_RRT_METHOD,TRUE);
    bio_set_all_sch_dofs_as_active_parameters_for_planner(robotPt);
  }
}

static void g3d_create_passive_sch_obj(void)
{
  PASSIVE_SCH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,15,560,65,30,"Passive Side-Chains");
  fl_set_object_color(PASSIVE_SCH_OBJ,FL_MCOL,FL_GREEN);
  fl_set_button(PASSIVE_SCH_OBJ,0);
  fl_set_call_back(PASSIVE_SCH_OBJ,CB_passive_sch_obj,0);
}

static void g3d_delete_passive_sch_obj(void)
{
  fl_free_object(PASSIVE_SCH_OBJ);
}

/***************************************************************************/
/* Activation/deactivation of bio_evaluate_traj function                   */
/***************************************************************************/

static int flag_eval_traj = 0;

int bio_get_flag_evaluate_traj(void)
{
  return(flag_eval_traj);
}


static void CB_eval_traj_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

  flag_eval_traj = val;
}

static void g3d_create_eval_traj_obj(void)
{
  EVAL_TRAJ_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,140,560,65,30,"Evaluate Path");
  fl_set_object_color(EVAL_TRAJ_OBJ,FL_MCOL,FL_GREEN);
  fl_set_button(EVAL_TRAJ_OBJ,0);
  fl_set_call_back(EVAL_TRAJ_OBJ,CB_eval_traj_obj,0);
}

static void g3d_delete_eval_traj_obj(void)
{
  fl_free_object(EVAL_TRAJ_OBJ);
}


/***************************************************************************/
/* Activate/deactivate option for writing each sampled node in pdb format  */
/* NOTE : works with function "p3d_randomconfs"                            */
/***************************************************************************/

static int flag_pdb_noded = 0;

int bio_get_flag_pdb_nodes(void)
{
  return(flag_pdb_noded);
}


static void CB_pdb_nodes_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

  flag_pdb_noded = val;
}

static void g3d_create_pdb_nodes_obj(void)
{
  PDB_NODES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,230,560,65,30,"Write Samples in PDB File");
  fl_set_object_color(PDB_NODES_OBJ,FL_MCOL,FL_GREEN);
  fl_set_button(PDB_NODES_OBJ,0);
  fl_set_call_back(PDB_NODES_OBJ,CB_pdb_nodes_obj,0);
}

static void g3d_delete_pdb_nodes_obj(void)
{
  fl_free_object(PDB_NODES_OBJ);
}


/***********************************/
/* Write PDBs for points in traj   */
/***********************************/
extern int TRACK_LOOP_FLAG;

static void CB_pdbOK_obj(FL_OBJECT *ob, long arg)
{
	int val = fl_get_button(ob);
	//char dir[50];
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	//int num_frames;
	double dep_dist;
	trajExportMinimizationType whatMinimizationToUse;
	whatMinimizationToUse = trajExpMinNONE;
#ifdef ENERGY
	char systemCommand[500];
#endif

#ifdef ENERGY
	if ( fl_get_button( RADIO_AMBER ) )
		whatMinimizationToUse = trajExpMinAMBERSaveFromAmber;
	else 	if ( fl_get_button( RADIO_MOVE3D) )
		whatMinimizationToUse = trajExpMinAMBERSaveFromM3D;
	else 	if ( fl_get_button( RADIO_RRT) )
		whatMinimizationToUse = trajExpMinRRT;
#endif

	if ( (robotPt->tcur) && (robotPt->tcur->courbePt != NULL) && (val) )
	{
		//num_frames = (int) fl_get_slider_value(pdb_numberframes_obj);
		dep_dist = (double) fl_get_slider_value(pdb_numberframes_obj);

		//printf("number of PDBs : %d -- %d\n",(int) fl_get_slider_value(pdb_numberframes_obj), num_frames);

		//serchDmaxAndWrite(robotPt, (char *)fl_get_input(pdb_dir_obj),num_frames);
		if(TRACK_LOOP_FLAG)
			bio_write_pdbs_of_N_in_path(robotPt, (char *)fl_get_input(pdb_dir_obj), whatMinimizationToUse);
		else
			bio_write_pdbs_unifom_dep(robotPt, (char *)fl_get_input(pdb_dir_obj),dep_dist, whatMinimizationToUse);

#ifdef ENERGY
		sprintf(systemCommand, "cd %s;gnuplot -bg white ~/BioMove3D/plotTraj", (char *)fl_get_input(pdb_dir_obj) );
		PrintInfo(("Executing system:\n%s", systemCommand));
		system(systemCommand);
#endif
	}
	else
	{
		fl_show_alert("There is no trajectory to be saved.", "No PDB file was created !","",1);
	}

	fl_set_button(pdb_OKbutton_obj, 0);
	fl_set_button(PDB_TRAJ_OBJ, 0);
	fl_hide_form(PDB_TRAJ_FORM);
}

/**
 * @brief This method is called when the "SavePDBs along path" window is closed from the X button
	If we do not use this call back, XForms tries to close the entire application
	  and we do not want that. Instead we will just click on the Cancel button
 */
static int CB_WritePDBAlongPathWindow_OnClose( FL_FORM *form, void *argument )
{
	//Press the Cancel button
	fl_trigger_object( pdb_CANCELbutton );

	//If we return FL_OK, the application will continue to try to shut down itself
	//   if however we return FL_IGNORE, the application will not continue this event
	return FL_IGNORE;
}


/**
 * @brief Called when the user presses the Cancel button in the "Save PDBs along trajectory" form
 */
static void CB_pdbCANCEL_OnClick(FL_OBJECT *ob, long arg)
{
    fl_set_button(pdb_CANCELbutton, 0);
    fl_set_button(PDB_TRAJ_OBJ, 0);
    fl_hide_form(PDB_TRAJ_FORM);
}


static void g3d_create_pdb_traj_form(void)
{
  char dir[250];
  FL_OBJECT *obj;
  unsigned int currentY = 0;


	if (PDB_TRAJ_FORM != NULL)
	{	//The form has already been created
		return;
	}

/* Create one for all the robots */
    PDB_TRAJ_FORM = fl_bgn_form(FL_UP_BOX,400.0,210.0);

	currentY = 10;
    fl_add_box(FL_FLAT_BOX, 10.0, currentY, 175.0, 20.0, "Dep. between frames (Angstroms):");
    //"Approximate number of frames");
    pdb_numberframes_obj = fl_add_valslider(FL_HOR_SLIDER,185.0,currentY,205.0,20.0,"");

    fl_set_slider_bounds(pdb_numberframes_obj,0.0,10.0);
    fl_set_slider_value(pdb_numberframes_obj,1.0);
    //fl_set_slider_precision(pdb_numberframes_obj,0.1);
    fl_set_slider_step(pdb_numberframes_obj,0.1);
    //fl_set_call_back(pdb_numberframes_obj,CB_pdb_numberframes,0);


	currentY += 30;
    pdb_dir_obj = fl_add_input(FL_NORMAL_INPUT, 30.0,currentY,360.0,20.0, "Dir");
    p3d_get_directory(dir);
    fl_set_input(pdb_dir_obj,dir);
 // fl_set_call_back(pdb_dir_obj,CB_pdb_dir,0);

	currentY += 40;
	obj = fl_add_frame(FL_ENGRAVED_FRAME,10, currentY, 380, 100,"");
	obj = fl_add_box(FL_FLAT_BOX, 30, currentY-5, 130, 10, "Use minimization ?");


#ifdef ENERGY
	fl_bgn_group();

	currentY += 10;
	RADIO_NONE = fl_add_checkbutton(FL_RADIO_BUTTON,20, currentY+25, 50, 30, "No");
	fl_set_call_back(RADIO_NONE, CB_RadioUseEnergyMinimization_OnChange, 0);

	obj = fl_add_box(FL_FLAT_BOX, 120, currentY, 70, 20, "Yes with:");

	RADIO_RRT = fl_add_checkbutton(FL_RADIO_BUTTON, 130, currentY + 25, 50, 30, "RRT");
	//fl_set_button(RADIO_AMBER, 0); //0=not checked
	fl_set_call_back(RADIO_RRT,CB_RadioUseEnergyMinimization_OnChange,3);

	obj = fl_add_box(FL_FLAT_BOX, 240, currentY, 130, 10, "Yes with energy minimization");

	RADIO_AMBER = fl_add_checkbutton(FL_RADIO_BUTTON, 250, currentY +15, 50, 30, "and Amber save");
	//fl_set_button(RADIO_AMBER, 0); //0=not checked
	fl_set_call_back(RADIO_AMBER,CB_RadioUseEnergyMinimization_OnChange,1);

	RADIO_MOVE3D = fl_add_checkbutton(FL_RADIO_BUTTON, 250, currentY + 50, 50, 30, "and BioMove3D save");
	fl_set_call_back(RADIO_MOVE3D,CB_RadioUseEnergyMinimization_OnChange,2);

  	fl_end_group();
#else
	currentY += 10;
	//If the energy module was not compiled,. then instead of the radio buttons,
	// display a message in the window so that the user is informed of this
	obj = fl_add_box(FL_FLAT_BOX, 20, currentY, 200, 20, "- Energy module was not compiled -");
#endif

	currentY += 95;
    pdb_OKbutton_obj = fl_add_button(FL_PUSH_BUTTON, 120.0, currentY, 80.0, 20.0,  "OK");
    fl_set_call_back(pdb_OKbutton_obj,CB_pdbOK_obj,0);
    pdb_CANCELbutton = fl_add_button(FL_PUSH_BUTTON, 210.0, currentY, 80.0, 20.0, "Cancel");
    fl_set_call_back(pdb_CANCELbutton,CB_pdbCANCEL_OnClick,0);

    fl_end_form();
    fl_set_form_icon(PDB_TRAJ_FORM, GetApplicationIcon( ), 0);
    fl_set_form_atclose(PDB_TRAJ_FORM, CB_WritePDBAlongPathWindow_OnClose, 0);
}

static void g3d_delete_pdb_traj_form(void)
{
  if(PDB_TRAJ_FORM != NULL) {
    fl_free_object(pdb_numberframes_obj);
    fl_free_object(pdb_dir_obj);
    fl_free_object(pdb_OKbutton_obj);
    fl_free_object(pdb_CANCELbutton);
#ifdef ENERGY
    fl_free_object(RADIO_AMBER );
    fl_free_object(RADIO_MOVE3D);
    fl_free_object(RADIO_NONE);
    fl_free_object(RADIO_RRT);
#endif

    fl_free_form(PDB_TRAJ_FORM);
    PDB_TRAJ_FORM = NULL;
  }
}

/*****************************************************************/
/* Bouton pour lancer un test d'accesibilite                     */
/*****************************************************************/

static void CB_test_lig_btn_obj(FL_OBJECT *ob, long arg)
{
  ////////  OK //////////
  int       it      = p3d_get_desc_number(P3D_TRAJ);
  int       ir      = p3d_get_desc_curnum(P3D_ROBOT);
  p3d_traj *traj= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);

  if (fl_get_button(ob)) {
    CB_specific_search_obj(ob,0);
    if(p3d_GetDiffuStoppedByWeight()) {
      //bio_loop_graph_statistics();
      bio_search_max_weight_in_curr_rrt();
      CB_specific_search_obj(ob,0);
    }
    traj= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
    if(p3d_get_desc_number(P3D_TRAJ) > it) {
      printf("\nSMOOTHING PATH ...\n");
      CB_start_optim_obj(ob,0);
      fl_set_button(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,1);
      CB_showtraj_obj(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,0);
      // write a pdbfile to display the positions of the ligand's barycenter for all the nodes in the tree
      //bio_write_baryplot();
      // identify the exhausted nodes
      p3d_identify_exhausted_nodes();
    }
    fl_set_button(ob,0);
    traj = NULL;//to delete
  }
  else {
    CB_stop_obj(ob,0);
    CB_del_param_obj(ob,0);
  }

}

/**
 * CB_ComputeBestVdW_obj
 * Compute and print on the screen the maximal VdW
 * value for which the current path is not in collision
 */
static void CB_ComputeBestVdW_obj(FL_OBJECT *obj, long arg) {
  p3d_traj *traj= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  p3d_rob* robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  double VdwSaved =  GetPrevVdw();
  double VdwCurrent;
  int IsInvalidTraj  = TRUE, isCurrentCol;
  int ntest = 0;
  int indCurrRobot = p3d_get_desc_curnum(P3D_ROBOT);
  configPt q;

  if(traj == NULL) {
    printf("VdW computation Error: no current trajectory\n");
    return;
  }
  q = p3d_get_robot_config(robotPt);
  fl_deactivate_object(obj);

  p3d_col_test_all();
  isCurrentCol = biocol_robot_report(indCurrRobot);
  PrintInfo(("robot currently in col: %d\n",isCurrentCol ));
  VdwCurrent = 1.;
  bio_resize_molecules(VdwCurrent/GetPrevVdw());
  bio_true_resize_molecules();
  SetPrevVdw(VdwCurrent);
  IsInvalidTraj = isBioTrajInCol(robotPt,traj, &ntest);
  while((IsInvalidTraj == TRUE)&&(VdwCurrent>0)) {
    bio_resize_molecules(VdwCurrent/GetPrevVdw());
    bio_true_resize_molecules();
    SetPrevVdw(VdwCurrent);
    IsInvalidTraj = isBioTrajInCol(robotPt,traj, &ntest);
    VdwCurrent =VdwCurrent -0.01;
  }
  printf("Max VdW value: %f\n", VdwCurrent +0.01);
  bio_resize_molecules(VdwSaved/GetPrevVdw());
  bio_true_resize_molecules();
  SetPrevVdw(VdwSaved);
  fl_activate_object(obj);

  p3d_set_and_update_robot_conf(q);
  p3d_destroy_config(robotPt, q);
}

static void CB_ComputeVdWProfile_obj(FL_OBJECT *obj, long arg) {
 p3d_traj *trajPt= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
 pp3d_localpath localpathPt;
 double VdWq;
 pp3d_rob robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
 int i, njnt = robotPt->njoints,  end_localpath =0;
 double u=0, du, umax, dMaxDraw;
 double *distances;
 configPt q;
 double currentLength = 0.;

 if(trajPt == NULL) {
   printf("VdW Traj Profile cost Error: no current trajectory\n");
   return;
 }
  fl_deactivate_object(obj);

 localpathPt = trajPt->courbePt;
 PrintInfo(("Max VdW path Profile:\n"));
 distances = MY_ALLOC(double, njnt+1);
 while (localpathPt != NULL){
    umax = localpathPt->range_param;
    while (end_localpath < 2){
      //warning : the step is based
      //on the graphic dmax
      dMaxDraw = p3d_get_env_graphic_dmax();
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      VdWq  = p3d_ComputeVdWMaxConf(q);
      PrintInfo(("%f\t %f\n",currentLength,VdWq));
      p3d_destroy_config(robotPt, q);


      for (i=0; i<=njnt; i++){
	distances[i] = dMaxDraw;
      }
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
      u+=du;
      currentLength+=du;
      if (u > umax-EPS6){
	u = umax;
	end_localpath++;
      }
    }
    localpathPt = localpathPt->next_lp;
    end_localpath = 0;
    u = 0;
 }
 MY_FREE(distances, double, njnt+1);
 fl_activate_object(obj);
}

static void g3d_create_test_lig_btn_obj(void)
{
  TEST_LIG_BTN_OBJ = fl_add_button(FL_PUSH_BUTTON,20,590,110,30,"Compute Ligand Path");
  fl_set_call_back(TEST_LIG_BTN_OBJ,CB_test_lig_btn_obj,0);
}

static void g3d_delete_test_lig_btn_obj(void)
{
  fl_free_object(TEST_LIG_BTN_OBJ);
}


/*****************************************************************/
/* Bouton pour lancer un test pour un loop                       */
/*****************************************************************/

static void CB_test_loop_btn_obj(FL_OBJECT *ob, long arg)
{
  //int       it      = p3d_get_desc_number(P3D_TRAJ);
  //int       ir      = p3d_get_desc_curnum(P3D_ROBOT);

  int nopt;
  p3d_rob *robPt;
  p3d_matrix4 *ref_frame=NULL, *mob_frame=NULL;

  robPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  if (fl_get_button(ob)) {
    if(TRACK_LOOP_FLAG) {
      bio_track_loop_from_FbFe_sequence();
    }
    else {
      bio_set_init_jnt_coordinates();
      CB_specific_search_obj(ob,0);
      p3d_identify_farther_nodes();
      if(robPt->GRAPH->nnode > 2) {
	//	if(p3d_get_frames_for_metric(robPt,&ref_frame,&mob_frame)) {
	if(p3d_GetRefAndMobFrames(robPt,&ref_frame,&mob_frame)) {
	  // identify the exhausted nodes
	  p3d_identify_farther_nodes();
	}
	else {
	  bio_search_max_weight_in_curr_rrt();
	  CB_specific_search_obj(ob,0);
	  nopt = p3d_get_NB_OPTIM();
	  p3d_set_NB_OPTIM(10);
	  printf("\nSMOOTHING PATH ...\n");
	  CB_start_optim_obj(ob,0);
	  p3d_set_NB_OPTIM(nopt);
	  //fl_set_button(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,1);
	  //CB_showtraj_obj(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,0);
	  printf("\nOK : A NEW PATH HAS BEEN COMPUTED\n\n");
	}
      }
      else {
	printf("\nFAIL : PATH CANNOT BE COMPUTED\n\n");
      }
      fl_set_button(ob,0);
    }
  }
}

static void g3d_create_test_loop_btn_obj(void)
{
  TEST_LOOP_BTN_OBJ = fl_add_button(FL_PUSH_BUTTON,130,590,110,30,"Compute Loop Path");
  fl_set_call_back(TEST_LOOP_BTN_OBJ,CB_test_loop_btn_obj,0);
}

static void g3d_delete_test_loop_btn_obj(void)
{
  fl_free_object(TEST_LOOP_BTN_OBJ);
}

/*****************************************************************/
/* Bouton pour arreter l'exploration                             */
/*****************************************************************/

static void CB_stop_comput_path_obj(FL_OBJECT *ob, long arg)
{
  CB_stop_obj(ob,0);
  if(fl_get_button(TEST_LIG_BTN_OBJ))
    fl_set_button(TEST_LIG_BTN_OBJ,0);
}

static void g3d_create_stop_comput_path_obj(void)
{
  STOP_COMPUT_PATH_OBJ = fl_add_button(FL_TOUCH_BUTTON,350,590,30,30,"STOP");
  fl_set_call_back(STOP_COMPUT_PATH_OBJ,CB_stop_comput_path_obj,0);
}

static void g3d_delete_stop_comput_path_obj(void)
{
  fl_free_object(STOP_COMPUT_PATH_OBJ);
}

/*****************************************************************/
/* Input to define the path length (to stop rrt exploration)     */
/*****************************************************************/

static void CB_path_length_obj(FL_OBJECT *ob, long arg)
{
  double   pathLength = atof(fl_get_input(ob));
  //if you set a positive maximal weight (i.e a path length) as
  // stopping condition  it automatically set the flag  IS_WEIGHT_STOP_COND
  //  to TRUE

  if(pathLength <= 0.) {
    p3d_SetIsWeightStopCondition(FALSE);
    p3d_SetIsWeightedChoice(FALSE);
    return;
  }
  p3d_SetStopWeightAndSign(pathLength,p3d_get_w_inc_dir());
  p3d_SetIsWeightStopCondition(TRUE);
  p3d_SetIsWeightedChoice(TRUE);
  fl_set_button(WITH_GOAL_DIR_OBJ,FALSE);
  ENV.setBool(Env::expandToGoal,false);
}

static void g3d_create_path_length_obj(void)
{
  PATH_LENGTH_OBJ = fl_add_input(FL_NORMAL_INPUT,310,590,35,30,"Path Length\n (A)");
  fl_set_call_back(PATH_LENGTH_OBJ,CB_path_length_obj,0);
}

static void g3d_delete_path_length_obj(void)
{
  fl_free_object(PATH_LENGTH_OBJ);
}


/***********************************/
/* virer les collisions embetantes */
/***********************************/

// necessite l'appel a la fonction bio_all_molecules_col_with_report() avt emploi
void afficher_lescollisions(void)
{
  int i = 0;
  int col_number;
  p3d_poly **p1,**p2;

  biocol_report(&col_number, &p1, &p2);
  printf("col_number %d \n",col_number);

  for(i=0;i<col_number;i++)
    {
      printf("joints en collision : %s   %s\n",p1[i]->poly->name,p2[i]->poly->name);
    }

}

static void test_affichage(void)
{
  int nb_col = 0;

  nb_col = bio_all_molecules_col_with_report();

  puts("test de collisions faits");

  printf("nb_col : %d \n",nb_col);
  afficher_lescollisions();
}


/***********************************/
/* refresh displays                */
/***********************************/

static void bio_draw_allwin(int ncol)
{
  int i = 0;
  int j = 0;
  int nb_col;
  p3d_poly **p1,**p2;

  int encoreUnJoint = 1;
  int num_jnt_rob;
  p3d_jnt *jnt_AA;
  char AAtype[4];
  double color_vect[4];

  if( !fl_get_button(AFFICH_BIO_BTN_OBJ) )
    {
      g3d_set_draw_coll(ncol);
      g3d_draw_allwin();
    }
  else
    {
      biocol_report(&nb_col, &p1, &p2);
      // affiche a nouveau dans la bonne couleur les atomes
      for(j=0;j<nb_poly_en_col;j++)
	{
	  switch(poly_en_col[j]->type)
	    {
	    case CARBON: p3d_poly_set_color(poly_en_col[j],Green,NULL);
	      break;
	    case OXYGEN: p3d_poly_set_color(poly_en_col[j],Red,NULL);
	      break;
	    case OXYGEN_H: p3d_poly_set_color(poly_en_col[j],Red,NULL);
	      break;
	    case NITROGEN : p3d_poly_set_color(poly_en_col[j],Blue,NULL);
	      break;
	    case NITROGEN_H : p3d_poly_set_color(poly_en_col[j],Blue,NULL);
	      break;
	    case HYDROGEN : p3d_poly_set_color(poly_en_col[j],White,NULL);
	      break;
	    case SULPHUR : p3d_poly_set_color(poly_en_col[j],Yellow,NULL);
	      break;
	    case SULPHUR_H : p3d_poly_set_color(poly_en_col[j],Yellow,NULL);
	      break;
	    case BROMINE :
	      color_vect[0] = 153; color_vect[0] = 83; color_vect[2] = 4; color_vect[3] = 1;
	      p3d_poly_set_color(poly_en_col[j],Any,color_vect);
	      break;
	    default : p3d_poly_set_color(poly_en_col[j],Brown,NULL);
	    }
	} // fin for j

      nb_poly_en_col = 0;

      // on parcourt l' acide amine courant pour tout remettre en jaune :
      if(AA_courant != 0)
	{
	  num_jnt_rob = get_AAfirstjnt_number(AA_courant);
	  jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];

	  while(encoreUnJoint)
	    {
	      for(i=0;i<jnt_AA->o->np;i++)
		{
		  p3d_poly_set_color(jnt_AA->o->pol[i],Yellow,NULL);
		}
	      if(num_jnt_rob < XYZ_ROBOT->njoints)
		{
		  num_jnt_rob++;
		  jnt_AA = XYZ_ROBOT->joints[num_jnt_rob];
		  if(jnt_AA->o)
		    {
		      get_AAtype_from_name(jnt_AA->o->name, AAtype);
		      // on a encore un joint si jnt_AA->name est different de omega
		      // et que le joint n'appartient pas au ligand
		      encoreUnJoint = ( (jnt_AA->name[0]!='o') && ( !(strcmp(AAtype,"LIG")==0) ) );
		    }
		  else encoreUnJoint = 0;
		}
	      else encoreUnJoint = 0;
	    }
	}

      // affiche en noir les atomes en collision
      for(i=0;(i<nb_col)&&(i<(NB_MAX_COL_AFFICH/2));i++)
	{
	  p3d_poly_set_color(p1[i],Black,NULL);
	  poly_en_col[nb_poly_en_col] = p1[i];
	  nb_poly_en_col++;
	  p3d_poly_set_color(p2[i],Black,NULL);
	  poly_en_col[nb_poly_en_col] = p2[i];
	  nb_poly_en_col++;
	}
      g3d_set_draw_coll(0);
      g3d_draw_allwin();
    }
}


#ifdef ENERGY
//Use of Energy minimization before saving the PDB files
void CB_RadioUseEnergyMinimization_OnChange(FL_OBJECT *ob, long arg)
{
	//Some verification should be done as to whether the BioEnergy module has been initialized
	//  before the user uses this saving module
	switch( arg )
	{
		case 0:
			//Some action when the user clicks on the radio button for the use of Energy minimization
			// saying that no minimization is wanted
			break;
		case 1:
			//Some action when the user clicks on the radio button for the use of Energy minimization
			// with saving of the PDB files from Amber
			break;
		case 2:
			//Some action when the user clicks on the radio button for the use of Energy minimization
			// with saving of the PDB files from BioMove3D
			break;
		case 3:
			//Some action when the user clicks on the radio button for the use of RRT minimization
			break;
		default:
			PrintError(("Unknown argument !!"));
			break;
	}
}
#endif



/***********************************/
/* ligand type selection           */
/***********************************/

static void CB_ligand_type_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_choice(ob);
  int ncol = 0;

  if(val == 2)
    init_ligand_autocol_from_file();
  else
    init_ligand_autocol(val-2);

  /* collision checking */
  if(G3D_ACTIVE_CC)
    {	ncol = p3d_col_test_all(); }

  bio_draw_allwin(ncol);
}



static void g3d_create_ligand_type_obj(void)
{
  int i = 0;
  FL_OBJECT *obj;

  // PROVISOIREMENT ICI !!!!
  int N_LIGTYPES = 24;
  char ligtype[24][20] = {"NO LIGAND","SELECT FILE","ph(X)Et","C9H17O2","2-C9H17O2","sugar","BCL-NewSubs","glyco_1-3","glyco_1-6","suc-AS","BCL-octyl","BCL-para","BCL-meta","BCL-etcl","BCL-ortho","Maltose_1-4","glcB13","glcB12","glcB14","glcB16","Malt14-AMBER","BCL-phet","suc-AS_noH","cpl"};
  ////

  obj = fl_add_box(FL_FLAT_BOX,280,280,100.0,20,"Ligand Type");
  LIGAND_TYPE_OBJ = fl_add_choice(FL_NORMAL_CHOICE,280,300,100.0,40.0,"");
  fl_set_call_back(LIGAND_TYPE_OBJ,CB_ligand_type_obj,0);

  for(i=0; i<N_LIGTYPES; i++) {
    fl_addto_choice(LIGAND_TYPE_OBJ,ligtype[i]);
  }
}


static void g3d_delete_ligand_type_obj(void)
{
  fl_free_object(LIGAND_TYPE_OBJ);
}


/***********************************/
/* init ligand autocol             */
/***********************************/

static void init_ligand_autocol(int index_ligtype)
{
  int subrobot_lig = num_subrobot_ligand(XYZ_ROBOT);

  ////////////////  OUT OF HERE !!! //////////////////////

  // POUR LIGANS LIPASE_INSA
  int npairs_l1 = 54;
  int nrigid1_l1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      1,1,1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,
		      3,3,3,3,3,3,3,
		      4,4,4,4,4,4,4,4,4,4,4,4};
  int natom1_l1[]  = {0,0,0,0,1,3,3,3,3,4,5,5,5,5,5,5,6,7,
		      0,0,0,0,1,1,1,2,2,2,
		      0,0,0,0,0,1,1,
		      0,0,0,0,0,0,0,
		      0,0,0,0,1,1,1,1,2,2,2,2};
  int nrigid2_l1[] = {1,1,1,2,1,1,1,1,2,1,1,1,1,2,2,3,1,1,
		      2,2,3,4,2,2,3,2,2,3,
		      3,4,4,4,5,3,4,
		      4,4,4,5,5,5,5,
		      5,5,5,5,5,5,5,5,5,5,5,5};
  int natom2_l1[]  = {0,1,2,0,0,0,1,2,0,0,0,1,2,0,1,0,0,0,
		      0,1,0,0,0,1,0,0,1,0,
		      0,0,1,2,0,0,0,
		      0,1,2,0,1,2,3,
		      0,1,2,3,0,1,2,3,0,1,2,3};

  // POUR LIGANS LIPASE_CANDIDA-a8
  int npairs_l2 = 120;
  int nrigid1_l2[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		      3,3,3,3,3,3,3,3,3,3,3,3,
		      4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		      5,
		      7,7,7,7,7,7,7,7,7,7,
		      8,8,8,8};
  int natom1_l2[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,
		      0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,2,2,
		      0,
		      0,0,0,0,1,1,1,2,2,2,
		      0,0,1,1};
  int nrigid2_l2[] = {1,1,1,2,2,3,3,3,4,4,4,5,5,5,5,6,7,1,1,1,2,2,3,4,5,1,1,1,2,2,3,4,5,
		      2,2,3,3,3,4,5,6,6,6,2,3,3,3,6,2,3,3,3,6,
		      3,4,4,4,5,5,5,5,7,7,7,8,4,4,4,5,5,5,5,7,
		      6,6,6,6,6,6,6,6,6,6,6,6,
		      5,5,5,5,7,7,7,8,8,9,5,7,7,7,8,5,7,7,7,8,
		      7,
		      8,8,9,9,8,8,9,8,8,9,
		      9,9,9,9};
  int natom2_l2[]  = {0,1,2,0,1,0,1,2,0,1,2,0,1,2,3,0,0,0,1,2,0,1,0,0,0,0,1,2,0,1,0,0,0,
		      0,1,0,1,2,0,0,0,1,2,0,0,1,2,0,0,0,1,2,0,
		      0,0,1,2,0,1,2,3,0,1,2,0,0,1,2,0,1,2,3,0,
		      0,1,2,3,0,1,2,3,0,1,2,3,
		      0,1,2,3,0,1,2,0,1,0,0,0,1,2,0,0,0,1,2,0,
		      0,
		      0,1,0,1,0,1,0,0,1,0,
		      0,1,0,1};

  // POUR LIGANS LIPASE_CANDIDA-a3
  int npairs_l3 = 127;
  int nrigid1_l3[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		      3,3,3,3,3,3,3,3,3,3,3,3,
		      4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		      6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		      7,
		      8,8,8,8};
  int natom1_l3[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,
		      0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,
		      0,
		      0,0,1,1};
  int nrigid2_l3[] = {1,1,1,2,2,2,3,3,3,4,4,5,6,7,1,1,1,2,2,2,3,4,1,1,1,2,2,2,3,4,
		      2,2,2,3,3,3,4,5,5,5,5,2,3,3,3,5,5,5,5,2,3,3,3,5,5,5,5,
		      3,4,4,6,6,6,7,7,7,7,8,4,4,6,7,4,4,6,7,
		      5,5,5,5,5,5,5,5,5,5,5,5,
		      6,6,6,7,7,7,7,8,8,9,6,6,6,7,7,7,7,8,
		      7,7,7,7,8,8,9,9,7,8,8,9,7,8,8,9,
		      8,
		      9,9,9,9};
  int natom2_l3[]  = {0,1,2,0,1,2,0,1,2,0,1,0,0,0,0,1,2,0,1,2,0,0,0,1,2,0,1,2,0,0,
		      0,1,2,0,1,2,0,0,1,2,3,0,0,1,2,0,1,2,3,0,0,1,2,0,1,2,3,
		      0,0,1,0,1,2,0,1,2,3,0,0,1,0,0,0,1,0,0,
		      0,1,2,3,0,1,2,3,0,1,2,3,
		      0,1,2,0,1,2,3,0,1,0,0,1,2,0,1,2,3,0,
		      0,1,2,3,0,1,0,1,0,0,1,0,0,0,1,0,
		      0,
		      0,1,0,1};

  // POUR LIGANS GLY
  int npairs_l4 = 20;
  int nrigid1_l4[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int natom1_l4[]  = {0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,5,6,7};
  int nrigid2_l4[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  int natom2_l4[]  = {0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,0,0,0};

  // POUR LIGANS AVENTIS
  int npairs_l5 = 59;
  int nrigid1_l5[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,
		      3,3,3,3,
		      4,4,4,4};
  int natom1_l5[]  = {0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,7,8,9,10,11,
		      0,0,0,0,0,0,1,1,
		      0,0,0,0,0,1,1,
		      0,0,0,0,
		      0,0,0,0};
  int nrigid2_l5[] = {1,1,2,2,3,4,5,1,1,2,2,3,4,6,1,1,2,3,1,1,2,3,1,2,2,4,1,2,2,4,1,1,1,2,2,2,
		      2,3,5,5,5,5,3,5,
		      4,6,6,6,6,4,6,
		      5,5,5,5,
		      6,6,6,6};
  int natom2_l5[]  = {0,1,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,
		      0,0,0,1,2,3,0,0,
		      0,0,1,2,3,0,0,
		      0,1,2,3,
		      0,1,2,3};

  // POUR LIGANS GLYCO 1-3 (noHp)
  int npairs_l6 = 121;
  int nrigid1_l6[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                      1,1,1,1,1,1,1,1,1,1,
                      2,3,5,5,5,
                      6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
                      8,9,9,9,9,10};
  int natom1_l6[]  = {0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,5,5,5,6,6,6,6,6,6,6,7,7,7,8,8,8,8,8,8,8,9,9,10,10,10,10,10,
                      0,0,0,0,0,0,0,0,0,0,
                      0,0,0,1,2,
                      0,0,0,1,1,1,1,1,1,2,2,2,3,4,4,4,4,4,4,5,5,5,5,6,6,7,7,7,7,7,7,7,8,8,8,8,8,9,9,9,10,10,10,
		      0,0,0,1,2,0};
  int nrigid2_l6[] = {1,2,3,6,1,2,3,4,5,6,6,6,6,1,2,3,4,6,1,2,3,5,5,5,6,7,1,2,4,5,1,2,4,1,3,4,5,5,5,7,1,3,5,2,3,4,5,5,5,7,2,4,3,5,5,5,7,
                      2,3,6,6,6,6,6,6,6,8,
                      4,5,7,7,7,
                      8,9,10,8,9,9,9,11,12,8,10,11,8,9,9,9,10,11,12,8,9,10,11,8,10,8,9,9,9,10,11,12,9,9,9,11,12,8,10,11,9,10,11,
                      10,11,12,12,12,11};
  int natom2_l6[]  = {0,0,0,0,0,0,0,0,0,0,1,2,3,0,0,0,0,0,0,0,0,0,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,0,0,0,0,0,0,0,0,1,2,0,0,0,0,0,1,2,0,
                      0,0,0,1,2,3,4,5,6,0,
                      0,0,0,0,0,
                      0,0,0,0,0,1,2,0,0,0,0,0,0,0,1,2,0,0,0,0,0,0,0,0,0,0,0,1,2,0,0,0,0,1,2,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0};

  // POUR LIGANS GLYCO 1-6 (noHp)
  int npairs_l7 = 121;
  int nrigid1_l7[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                      1,1,1,1,1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
 		      3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		      4,5,6,8 ,9 ,9 ,9 ,9 ,10};
  int natom1_l7[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2,
                      0,0,0,0,0,0,0,0,0,0,0,0,
		      0,0,0,0,1,1,1,2,2,2,3,4,4,4,5,5,5,5,6,6,7,7,7,7,8,8,9,9,9,10,10,10,
 		      0,0,0 ,1,1,1,1,1 ,1 ,2,2 ,2 ,3,4,4,4,4 ,4 ,4 ,5,5,5 ,5 ,6,6 ,7,7,7,7 ,7 ,7 ,7,8,8,8,8 ,8 ,9,9 ,9 ,10,10,10,
		      0,0,0,0 ,0 ,0 ,1 ,2 ,0 };
  int nrigid2_l7[] = {1,2,2,2,2,2,2,2,3,3,3,3,4,1,2,2,2,2,3,1,2,2,2,2,3,
                      2,2,2,2,3,3,3,3,3,3,3,8,
		      3,4,5,6,4,5,7,4,6,7,4,5,6,7,4,5,6,7,4,6,4,5,6,7,5,7,4,6,7,5,6,7,
 		      8,9,10,8,9,9,9,11,12,8,10,11,8,9,9,9,10,11,12,8,9,10,11,8,10,8,9,9,9,10,11,12,9,9,9,11,12,8,10,11,9,10,11,
                      6,7,7,10,11,12,12,12,11};
  int natom2_l7[]  = {0,0,1,2,3,4,5,6,0,1,2,3,0,0,0,1,2,3,0,0,0,1,2,3,0,
                      0,1,2,3,0,1,2,3,4,6,7,0,
		      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 		      0,0,0,0,0,1,2,0,0,0,0,0,0,0,1,2,0,0,0,0,0,0,0,0,0,0,0,1,2,0,0,0,0,1,2,0,0,0,0,0,0,0,0,
		      0,0,0,0,0,0,0,0,0};



/*   // POUR LIGANS GLYCO 1-3 */
/*   int npairs_l6 = 153; */
/*   int nrigid1_l6[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, */
/*                       1,1,1,1,1,1,1,1,1,1, */
/*                       2,3,5,5,5,5,5,5, */
/*                       6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6, */
/*                       8,9,9,9,9,9,9,9,10}; */
/*   int natom1_l6[]  = {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,6,6,6,6,6,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,10,10,10,10,10, */
/*                       0,0,0,0,0,0,0,0,0,0, */
/*                       0,0,0,0,1,1,2,2, */
/*                       0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,7,7,7,7,7,7,7,7,7,8,8,8,8,8,9,9,9,9,10,10,10,10,0,0,0,0,1,1,2,2,0}; */
/*   int nrigid2_l6[] = {1,2,3,6,1,2,2,3,3,4,5,6,6,6,6,1,2,2,3,4,4,6,1,2,3,3,5,5,5,6,7,1,2,2,4,4,5,1,2,2,4,1,3,3,4,5,5,5,7,7,1,3,3,5,2,3,4,4,5,5,5,7,2,4,4,3,5,5,5,7, */
/*                       2,3,6,6,6,6,6,6,6,8, */
/*                       4,5,7,7,7,7,7,7, */
/*                       8,8,9,10,8,9,9,9,11,12,8,8,10,10,11,8,9,9,9,10,11,11,12,12,8,8,9,10,10,11,11,8,8,10,8,9,9,9,10,10,11,11,12,9,9,9,11,12,8,10,10,11,9,10,11,11, */
/*                       10,11,12,12,12,12,12,12,11}; */
/*   int natom2_l6[]  = {0,0,0,0,0,0,1,0,1,0,0,0,1,2,3,0,0,1,0,0,1,0,0,0,0,1,0,1,2,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,1,2,0,1,0,0,1,0,0,0,0,1,0,1,2,0,0,0,1,0,0,1,2,0, */
/*                       0,0,0,1,2,3,4,5,6,0, */
/*                       0,0,0,1,0,1,0,1, */
/*                       0,1,0,0,0,0,1,2,0,0,0,1,0,1,0,0,0,1,2,0,0,1,0,1,0,1,0,0,1,0,1,0,1,0,0,0,1,2,0,1,0,1,0,0,1,2,0,0,0,0,1,0,0,0,0,1, */
/*                       0,0,0,1,0,1,0,1,0}; */

/*   // POUR LIGANS GLYCO 1-6 */
/*   int npairs_l7 = 153; */
/*   int nrigid1_l7[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, */
/*                       1,1,1,1,1,1,1,1,1,1,1,1, */
/* 		      2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2, */
/* 		      3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, */
/* 		      4,5,6,8,9,9,9,9,9,9,9,10}; */
/*   int natom1_l7[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2, */
/*                       0,0,0,0,0,0,0,0,0,0,0,0, */
/* 		      0,0,0,0,0,1,1,1,1,2,2,2,2,2,3,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,7,7,7,7,7,7,7,8,8,8,9,9,9,9,10,10,10,10, */
/* 		      0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,7,7,7,7,7,7,7,7,7,8,8,8,8,8,9,9,9,9,10,10,10,10, */
/* 		      0,0,0,0,0,0,0,1,1,2,2,0}; */
/*   int nrigid2_l7[] = {1,2,2,2,2,2,2,2,3,3,3,3,4,1,2,2,2,2,3,1,2,2,2,2,3, */
/*                       2,2,2,2,3,3,3,3,3,3,3,8, */
/* 		      3,4,4,5,6,4,5,5,7,4,4,6,6,7,4,5,5,6,7,7,4,4,5,6,6,7,7,4,4,6,4,5,5,6,6,7,7,5,5,7,4,6,6,7,5,6,7,7, */
/* 		      8,8,9,10,8,9,9,9,11,12,8,8,10,10,11,8,9,9,9,10,11,11,12,12,8,8,9,10,10,11,11,8,8,10,8,9,9,9,10,10,11,11,12,9,9,9,11,12,8,10,10,11,9,10,11,11, */
/* 		      6,7,7,10,11,12,12,12,12,12,12,11}; */
/*   int natom2_l7[]  = {0,0,1,2,3,4,5,6,0,1,2,3,0,0,0,1,2,3,0,0,0,1,2,3,0, */
/*                       0,1,2,3,0,1,2,3,4,6,7,0, */
/* 		      0,0,1,0,0,0,0,1,0,0,1,0,1,0,0,0,1,0,0,1,0,1,0,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0,0,0,0,1, */
/* 		      0,1,0,0,0,0,1,2,0,0,0,1,0,1,0,0,0,1,2,0,0,1,0,1,0,1,0,0,1,0,1,0,1,0,0,0,1,2,0,1,0,1,0,0,1,2,0,0,0,0,1,0,0,0,0,1, */
/* 		      0,0,0,0,0,0,1,0,1,0,1,0}; */

  ///////////////////////////////////////////////////////////

  // POUR LIGANS AS_GWEN
  int npairs_l8 = 80;
  int nrigid1_l8[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
                     };
  int natom1_l8[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      0,0,0,0,0,1,2,2,2,2,2,3,4,7,7,7,7,8,8,8,8,11,11,11,11,12,13,
		      0,0,0,0,1,1,1,1,1,2,2,2,2,2,2,2,2,3,3,3,3,3,4,5,6,6,6,6,6,7,8,9,9,9,9
                     };
  int nrigid2_l8[] = {1,1,1,1,1,1,1,1,2,2,2,2,2,2,4,4,4,4,
		      2,2,2,3,4,2,2,3,3,3,3,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		      4,4,4,4,4,4,4,4,5,4,4,4,4,5,5,5,5,4,5,5,5,5,4,4,4,5,5,5,5,5,5,5,5,5,5
                     };
  int natom2_l8[]  = {0,1,2,3,4,5,6,7,0,1,2,3,4,5,0,1,2,3,
		      0,1,2,0,0,0,0,0,1,2,3,0,0,0,1,2,3,0,1,2,3,0,1,2,3,0,0,
		      0,1,2,3,0,1,2,3,0,0,1,2,3,0,1,2,3,0,0,1,2,3,0,0,0,0,1,2,3,0,0,0,1,2,3
                     };

  ///////////////////////////////////////////////////////////

  // POUR LIGANS LIPASE - OCTYL
  int npairs_l9 = 144;
  int nrigid1_l9[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		      1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		      2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		      3,3,3,3,3,3,3,3,
		      4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		      5,5,5,5,5,5,
		      6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		      7,7,7,7,7,7,7,7,7,7,
		      8,8,8,8,8,8,8,8,8,8,8,8,
		      9,9,9,9,9,9,9,9,9,9,9,9,9
                     };
  int natom1_l9[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,2,2,
		      0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,1,1,2,2,
		      0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,0,0,
		      0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,0,0,1,1,1,1,
		      0,0,0,0,1,1,1,1,2,2,2,2,
		      0,0,0,0,0,0,0,0,0,0,0,0,0
                     };
  int nrigid2_l9[] = {1,1,1,2,2,2,3,3,3,4,4,4,5,6,1,1,1,2,2,2,3,4,1,1,1,2,2,2,3,4,
		      2,2,2,3,3,3,4,5,7,2,3,3,3,5,2,3,3,3,5,
		      3,4,4,4,6,6,6,8,4,4,4,6,4,4,4,6,
		      5,7,7,9,5,7,5,7,
		      6,6,6,8,8,8,10,6,6,6,8,6,6,6,8,
		      7,7,9,9,9,11,
		      8,8,8,10,10,10,10,8,8,8,10,8,8,8,10,
		      9,9,9,11,11,11,9,9,9,11,
		      10,10,10,10,10,10,10,10,10,10,10,10,
		      11,11,11,11,11,11,11,11,11,11,11,11,11
                     };
  int natom2_l9[]  = {0,1,2,0,1,2,0,1,2,0,1,2,0,0,0,1,2,0,1,2,0,0,0,1,2,0,1,2,0,0,
		      0,1,2,0,1,2,0,0,0,0,0,1,2,0,0,0,1,2,0,
		      0,0,1,2,0,1,2,0,0,1,2,0,0,1,2,0,
		      0,0,1,0,0,0,0,0,
		      0,1,2,0,1,2,0,0,1,2,0,0,1,2,0,
		      0,1,0,1,2,0,
		      0,1,2,0,1,2,3,0,1,2,0,0,1,2,0,
		      0,1,2,0,1,2,0,1,2,0,
		      0,1,2,3,0,1,2,3,0,1,2,3,
		      0,1,2,3,4,5,6,0,1,2,0,1,2
                     };

  ///////////////////////////////////////////////////////////

  // POUR LIGANS LIPASE - PARA
  int npairs_l10 = 70;
  int nrigid1_l10[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,
		       3,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,
		       5,5,5,5,5,5,5,5,5,5,5,5
                      };
  int natom1_l10[]  = {0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,3,4,4,4,4,4,5,5,5,5,6,7,7,7,7,7,8,9,
		       0,0,0,0,1,1,1,2,2,2,
		       0,0,0,0,0,1,1,
		       0,0,0,0,0,0,0,
		       0,0,0,0,1,1,1,1,2,2,2,2
                       };
  int nrigid2_l10[] = {1,1,1,2,3,1,1,1,3,3,4,1,2,2,2,2,1,1,1,1,2,3,2,2,2,2,2,1,2,2,2,2,1,2,
		       3,3,4,5,3,3,4,3,3,4,
		       4,5,5,5,6,4,5,
		       5,5,5,6,6,6,6,
		       6,6,6,6,6,6,6,6,6,6,6,6
                       };
  int natom2_l10[]  = {0,1,2,0,0,0,1,2,0,1,0,0,0,1,2,3,0,0,1,2,0,0,0,1,2,3,0,0,0,1,2,3,0,0,
		       0,1,0,0,0,1,0,0,1,0,
		       0,0,1,2,0,0,0,
		       0,1,2,0,1,2,3,
		       0,1,2,3,0,1,2,3,0,1,2,3
                       };

  // POUR LIGANS LIPASE - META
  int npairs_l11 = 70;
  int nrigid1_l11[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,
		       3,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,
		       5,5,5,5,5,5,5,5,5,5,5,5
                      };
  int natom1_l11[]  = {0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,3,3,3,3,3,4,4,4,4,5,5,6,7,7,8,
		       0,0,0,0,1,1,1,2,2,2,
		       0,0,0,0,0,1,1,
		       0,0,0,0,0,0,0,
		       0,0,0,0,1,1,1,1,2,2,2,2
                       };
  int nrigid2_l11[] = {1,1,1,3,1,1,1,2,3,3,4,1,1,1,2,2,2,2,3,1,2,2,2,2,2,2,2,2,1,2,1,1,2,2,
		       3,3,4,5,3,3,4,3,3,4,
		       4,5,5,5,6,4,5,
		       5,5,5,6,6,6,6,
		       6,6,6,6,6,6,6,6,6,6,6,6
                       };
  int natom2_l11[]  = {0,1,2,0,0,1,2,0,0,1,0,0,1,2,0,1,2,3,0,0,0,1,2,3,0,1,2,3,0,0,0,0,0,8,
		       0,1,0,0,0,1,0,0,1,0,
		       0,0,1,2,0,0,0,
		       0,1,2,0,1,2,3,
		       0,1,2,3,0,1,2,3,0,1,2,3
                       };

  // POUR LIGANS LIPASE - ETCL
  int npairs_l12 = 54;
  int nrigid1_l12[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,
		       2,2,2,2,2,2,2,
		       3,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,4,4,4,4,4
                      };
  int natom1_l12[]  = {0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,4,5,6,
		       0,0,0,0,1,1,1,2,2,2,
		       0,0,0,0,0,1,1,
		       0,0,0,0,0,0,0,
		       0,0,0,0,1,1,1,1,2,2,2,2
                       };
  int nrigid2_l12[] = {1,1,1,2,2,3,1,1,1,2,1,1,1,2,1,1,1,1,
		       2,2,3,4,2,2,3,2,2,3,
		       3,4,4,4,5,3,4,
		       4,4,4,5,5,5,5,
		       5,5,5,5,5,5,5,5,5,5,5,5
                       };
  int natom2_l12[]  = {0,1,2,0,1,0,0,1,2,0,0,1,2,0,0,0,0,0,
		       0,1,0,0,0,1,0,0,1,0,
		       0,0,1,2,0,0,0,
		       0,1,2,0,1,2,3,
		       0,1,2,3,0,1,2,3,0,1,2,3
                       };

  // POUR LIGANS LIPASE - META
  int npairs_l13 = 69;
  int nrigid1_l13[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,1,
		       3,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,
		       5,5,5,5,5,5,5,5,5,5,5,5
                      };
  int natom1_l13[]  = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,5,6,7,
		       0,0,0,0,0,1,1,1,2,2,2,
		       0,0,0,0,0,1,1,
		       0,0,0,0,0,0,0,
		       0,0,0,0,1,1,1,1,2,2,2,2
                       };
  int nrigid2_l13[] = {1,1,1,2,2,2,2,3,3,4,1,1,1,2,2,2,2,3,1,1,1,2,3,1,2,2,2,2,1,1,2,2,
		       2,3,3,4,5,3,3,4,3,3,4,
		       4,5,5,5,6,4,5,
		       5,5,5,6,6,6,6,
		       6,6,6,6,6,6,6,6,6,6,6,6
                       };
  int natom2_l13[]  = {0,1,2,0,1,2,3,0,1,0,0,1,2,0,1,2,3,0,0,1,2,0,0,0,0,1,2,3,0,0,0,0,
		       0,0,1,0,0,0,1,0,0,1,0,
		       0,0,1,2,0,0,0,
		       0,1,2,0,1,2,3,
		       0,1,2,3,0,1,2,3,0,1,2,3
                       };

  // POUR MALTODEXTRIN - MALTOSE_1-4
  int npairs_l14 = 35;
  int nrigid1_l14[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		       2,2,2,2,2,2,2,2
                      };
  int natom1_l14[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,
		       0,0,0,0,1,2,2,2,3,5,5,6,6,8,
		       0,0,1,2,2,5,5,6
                       };
  int nrigid2_l14[] = {1,1,1,1,1,1,2,2,2,2,2,2,4,
		       2,2,2,3,2,2,3,3,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,4
                       };
  int natom2_l14[]  = {0,1,2,3,4,5,0,1,2,3,4,5,0,
		       0,1,2,0,0,0,0,1,0,0,1,0,1,0,
		       0,1,0,0,1,0,1,0
                       };

  // POUR glcB13
  int npairs_l15 = 36;
  int nrigid1_l15[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,
		       3,3,3,3,3,3,3,3,3
                      };
  int natom1_l15[]  = {0,0,0,0,0,1,1,2,2,2,2,3,3,4,5,5,5,6,6,7,7,
		       0,0,0,0,0,0,
		       0,1,1,3,3,5,6,6,8
                       };
  int nrigid2_l15[] = {1,2,3,3,3,1,3,1,2,2,3,1,2,1,1,2,2,1,2,2,2,
		       3,3,3,3,3,3,
		       4,4,4,4,4,4,4,4,4
                       };
  int natom2_l15[]  = {0,0,0,1,2,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,1,
		       0,1,2,3,4,5,
		       0,0,1,0,1,0,0,1,0
                       };

  // POUR glcB12
  int npairs_l16 = 36;
  int nrigid1_l16[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,
		       3,3,3,3,3,3,3,3,3
                      };
  int natom1_l16[]  = {0,0,0,0,1,1,1,2,2,2,3,3,3,4,5,6,6,6,7,7,8,
		       0,0,0,0,0,0,
		       0,1,1,3,3,5,6,6,8
                       };
  int nrigid2_l16[] = {1,3,3,3,1,2,3,1,2,3,1,2,2,1,1,1,2,2,2,2,2,
		       3,3,3,3,3,3,
		       4,4,4,4,4,4,4,4,4
                       };
  int natom2_l16[]  = {0,0,1,2,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,
		       0,1,2,3,4,5,
		       0,0,1,0,1,0,0,1,0
                       };

  // POUR glcB14
  int npairs_l17 = 35;
  int nrigid1_l17[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,
		       3,3,3,3,3,3,3,3,3
                      };
  int natom1_l17[]  = {0,0,0,0,0,0,1,1,1,1,2,2,2,3,3,3,4,5,6,
		       0,0,0,0,0,0,0,
		       0,1,1,3,3,5,6,6,8
                       };
  int nrigid2_l17[] = {1,2,2,3,3,3,1,2,2,3,1,2,3,1,2,2,1,1,2,
		       2,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,4,4
                       };
  int natom2_l17[]  = {0,0,1,0,1,2,0,0,1,0,0,0,0,0,0,1,0,0,0,
		       0,0,1,2,3,4,5,
		       0,0,1,0,1,0,0,1,0
                       };

  // POUR glcB16
  int npairs_l18 = 29;
  int nrigid1_l18[] = {0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,1,1,1,
		       2,2,2,2,2,2
                      };
  int natom1_l18[]  = {0,0,0,0,0,0,0,0,0,0,
		       0,0,0,1,1,1,2,3,3,5,6,6,8,
		       0,0,0,0,0,0
                       };
  int nrigid2_l18[] = {1,1,1,1,1,1,2,4,4,4,
		       2,3,4,2,3,3,2,3,3,3,3,3,3,
		       4,4,4,4,4,4
                       };
  int natom2_l18[]  = {0,1,2,3,4,5,0,0,1,2,
		       0,0,0,0,0,1,0,0,1,0,0,1,0,
		       0,1,2,3,4,5
                       };

  // POUR maltodextrin_lig_AMBER
  int npairs_l19 = 78;
  int nrigid1_l19[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1, 1, 1, 1,
		       2,2,2,2,2,2,2,2,2,2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2
                      };
  int natom1_l19[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       0,0,0,0,0,0,0,0,1,1,2,2,3,3,3,3,3,3,4,4,7,7,7,7,8,8,8,8,8,8,9,9,9,9,9,9,10,11,12,12,
		       0,3,3,3,3,6,7,7,7,7,10,10,10,10,11,11,11,11,12,13
                       };
  int nrigid2_l19[] = {1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,3,
		       2,2,2,2,3,3,3,3,2,3,2,3,2,3,3,3,3,4,4,4,3,3,3,3,3,3,3,3,4,4,3,4,4,4,4,4, 4, 4, 4, 4,
		       5,5,5,5,5,5,5,5,5,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5
                       };
  int natom2_l19[]  = {0,1,2,3,4,5,6,7,8,0,1,2,3,4,5,6,7,0,
		       0,1,2,3,0,1,2,3,0,0,0,0,0,0,1,2,3,0,0,1,0,1,2,3,0,1,2,3,0,1,0,0,1,2,3,4, 0, 0, 0, 1,
		       0,0,1,2,3,0,0,1,2,3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 0
                       };

  // POUR LIGANS LIPASE - PHET
  int npairs_l20 = 83;
  int nrigid1_l20[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		       2,2,2,2,2,2,2,2,2,2,2,2,
		       3,3,3,3,3,3,3,3,3,3,3,3,3,
		       4,4,4,4,4,4,4,
		       6,6,6,6,6,6,6,6,6,6,6,6
                      };
  int natom1_l20[]  = {0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,4,5,6,
		       0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,
		       0,0,0,0,0,0,0,0,0,1,1,1,
		       0,0,0,0,0,1,1,1,1,1,1,1,1,
		       0,0,0,0,0,0,0,
		       0,0,0,0,1,1,1,1,2,2,2,2
                       };
  int nrigid2_l20[] = {1,1,2,2,3,3,3,4,5,1,1,2,3,1,1,2,3,1,1,1,1,
		       2,2,3,3,3,4,5,5,5,5,6,2,2,3,3,3,4,5,
		       3,3,3,4,5,6,6,6,7,3,4,6,
		       4,5,5,5,5,5,5,5,5,5,5,5,5,
		       6,6,6,7,7,7,7,
		       7,7,7,7,7,7,7,7,7,7,7,7
                       };
  int natom2_l20[]  = {0,1,0,1,0,1,2,0,0,0,1,0,0,0,1,0,0,0,0,0,0,
		       0,1,0,1,2,0,0,1,2,3,0,0,1,0,1,2,0,0,
		       0,1,2,0,0,0,1,2,0,0,0,0,
		       0,0,1,2,3,0,1,2,3,0,1,2,3,
		       0,1,2,0,1,2,3,
		       0,1,2,3,0,1,2,3,0,1,2,3
                       };

  // POUR LIGAN AS_noH
  int npairs_l21 = 46;
  int nrigid1_l21[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		       2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
                       };
  int natom1_l21[]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,
		       0,0,0,0,0,1,1,1,2,3,3,4,6,6,8,
		       0,0,0,1,1,1,1,2,2,2,3,3,3,4,4,4,5,6
                       };
  int nrigid2_l21[] = {1,1,1,1,1,1,2,2,2,2,2,4,4,
		       2,2,2,3,4,2,3,3,2,3,3,3,3,3,3,
		       4,4,5,4,4,5,5,4,4,5,4,5,5,4,5,5,4,5
                       };
  int natom2_l21[]  = {0,1,2,3,4,5,0,1,2,3,5,0,1,
		       0,1,2,0,0,0,0,1,0,0,1,0,0,1,0,
		       0,1,0,0,1,0,1,0,1,0,0,0,1,0,0,1,0,0
                       };

  // POUR CPL (ONLY 2 BONDS !!!!)
  int npairs_l22 = 35;
  int nrigid1_l22[] = {0,0,0,0,0,0,0,0,0,
		       1,1,1,1,1,1,
		       2,2,2,2,2,
		       4,4,4,4,4,4,4,4,4,4,4,
		       5,5,5,5
                       };
  int natom1_l22[]  = {0,0,0,0,0,0,0,1,1,
		       0,0,1,1,1,4,
		       0,0,0,0,1,
		       0,0,0,0,1,1,2,2,2,2,5,
		       0,0,0,1
                       };
  int nrigid2_l22[] = {1,1,1,1,2,2,4,1,2,
		       2,3,3,3,3,3,
		       4,4,4,5,4,
		       5,5,6,7,5,6,5,6,6,6,6,
		       7,7,7,7
                       };
  int natom2_l22[]  = {0,1,2,3,0,1,0,0,0,
		       0,0,0,1,2,0,
		       0,1,2,0,0,
		       0,1,0,0,0,0,0,0,1,2,0,
		       0,1,2,0
                       };



  /* ATTENTION A BIEN METTRE LE BON NUMERO DU BON ROBOT !!!!!!!!!!!! */
  printf("numero subrobot du ligand : %d  \n",subrobot_lig);

  switch(index_ligtype) {
  case -1:
    // do nothing
    break;
  case 1:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l1,nrigid1_l1,natom1_l1,nrigid2_l1,natom2_l1);
    break;
  case 2:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l2,nrigid1_l2,natom1_l2,nrigid2_l2,natom2_l2);
    break;
  case 3:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l3,nrigid1_l3,natom1_l3,nrigid2_l3,natom2_l3);
    break;
  case 4:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l4,nrigid1_l4,natom1_l4,nrigid2_l4,natom2_l4);
    break;
  case 5:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l5,nrigid1_l5,natom1_l5,nrigid2_l5,natom2_l5);
    break;
  case 6:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l6,nrigid1_l6,natom1_l6,nrigid2_l6,natom2_l6);
    break;
  case 7:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l7,nrigid1_l7,natom1_l7,nrigid2_l7,natom2_l7);
    break;
  case 8:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l8,nrigid1_l8,natom1_l8,nrigid2_l8,natom2_l8);
    break;
  case 9:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l9,nrigid1_l9,natom1_l9,nrigid2_l9,natom2_l9);
    break;
  case 10:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l10,nrigid1_l10,natom1_l10,nrigid2_l10,natom2_l10);
    break;
  case 11:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l11,nrigid1_l11,natom1_l11,nrigid2_l11,natom2_l11);
    break;
  case 12:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l12,nrigid1_l12,natom1_l12,nrigid2_l12,natom2_l12);
    break;
  case 13:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l13,nrigid1_l13,natom1_l13,nrigid2_l13,natom2_l13);
    break;
  case 14:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l14,nrigid1_l14,natom1_l14,nrigid2_l14,natom2_l14);
    break;
  case 15:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l15,nrigid1_l15,natom1_l15,nrigid2_l15,natom2_l15);
    break;
  case 16:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l16,nrigid1_l16,natom1_l16,nrigid2_l16,natom2_l16);
    break;
  case 17:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l17,nrigid1_l17,natom1_l17,nrigid2_l17,natom2_l17);
    break;
  case 18:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l18,nrigid1_l18,natom1_l18,nrigid2_l18,natom2_l18);
    break;
  case 19:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l19,nrigid1_l19,natom1_l19,nrigid2_l19,natom2_l19);
    break;
  case 20:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l20,nrigid1_l20,natom1_l20,nrigid2_l20,natom2_l20);
    break;
  case 21:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l21,nrigid1_l21,natom1_l21,nrigid2_l21,natom2_l21);
    break;
  case 22:
    if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs_l22,nrigid1_l22,natom1_l22,nrigid2_l22,natom2_l22);
    break;
  }
}

/***********************************/

static void init_ligand_autocol_from_file(void)
{
  int subrobot_lig = num_subrobot_ligand(XYZ_ROBOT);
  FILE *fBCD;
  char  *bcdFileName = NULL;
  const char *tempPtr = NULL;
  int npairs;
  int *nrigid1,*natom1,*nrigid2,*natom2;

  // Select .bcd file corresponding to the ligand
  tempPtr = fl_show_fselector("Select a BCD file corresponding to the ligand:",".","*.bcd", "");

  //Check whether the file can be opened or not
  if ( tempPtr == NULL )
    return;	//The user has pressed CANCEL in the file selector window

  bcdFileName = strdup( tempPtr );
  if (! (fBCD = fopen(bcdFileName, "r") ) ) {
    fl_show_alert("The file could not be opened !","File does not exist or you do not have the right to open it.",
		  bcdFileName,1);
    return;
  }

  read_desc_int(fBCD,1,&npairs);
  nrigid1 = MY_ALLOC(int,npairs);
  natom1 = MY_ALLOC(int,npairs);
  nrigid2 = MY_ALLOC(int,npairs);
  natom2 = MY_ALLOC(int,npairs);

  read_desc_int(fBCD,npairs,nrigid1);
  read_desc_int(fBCD,npairs,natom1);
  read_desc_int(fBCD,npairs,nrigid2);
  read_desc_int(fBCD,npairs,natom2);

  if(subrobot_lig >= 0) bio_set_autocol(subrobot_lig,npairs,nrigid1,natom1,nrigid2,natom2);

  MY_FREE(nrigid1,int,npairs);
  MY_FREE(natom1,int,npairs);
  MY_FREE(nrigid2,int,npairs);
  MY_FREE(natom2,int,npairs);

  fclose(fBCD);

}

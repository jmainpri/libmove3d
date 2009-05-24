#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "Bio-pkg.h"
#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#endif


extern MENU_FILTER *FILTER_FORM;  // KINEO-DEV :doit etre declare dans un .h !!
extern FL_OBJECT  *robotparams_obj; // KINEO-DEV :doit etre declare dans un .h !!
int IsGraphMovie = FALSE;


extern int robotSelectPositionFlag;

static FL_OBJECT  *mpeg_OKbutton_obj;
static FL_OBJECT  *mpeg_Graph_OKbutton_obj;
static FL_OBJECT  *mpeg_imagerate_obj;
static FL_OBJECT  *mpeg_imagecompress_obj;


MENU_ROBOT *ROBOTS_FORM;
FL_FORM    *MPEG_FORM = NULL;
static FL_FORM    *DMAX_FORM = NULL;
static FL_OBJECT  *DMAX_FRAME = NULL;
static FL_OBJECT  *DMAX_INPUT = NULL;
static FL_OBJECT  *DMAX_EXIT = NULL;

/* fonctions externes appelees par les boutons */
//extern double p3d_get_env_graphic_dmax(void);
//extern int p3d_filter_needs_init(void);
//extern void p3d_filter_switch_draw_robot_box(void);
//extern void FORMfilter_init_temp_jntdata(void);

//extern void g3d_kin_constraints_form(void);

static void g3d_create_printconfig_obj(void);
static void CB_printconfig_obj(FL_OBJECT *ob, long arg);

/* Debut modification Fabien */
static void g3d_delete_printconfig_obj(void);

static void CB_config_obj(FL_OBJECT *ob, long arg);
static void g3d_create_config_obj(void);
static void g3d_delete_config_obj(void);
static void CB_new_config_obj(FL_OBJECT *ob, long arg);
static void g3d_create_new_config_obj(void);
static void g3d_delete_new_config_obj(void);
static void CB_del_config_obj(FL_OBJECT *ob, long arg);
static void g3d_create_del_config_obj(void);
static void g3d_delete_del_config_obj(void);
static void CB_set_config_obj(FL_OBJECT *ob, long arg);
static void g3d_create_set_config_obj(void);
static void g3d_delete_set_config_obj(void);
/* Fin modification Fabien */

/* Ajout bouton compute Nic */
static void CB_compute_obj(FL_OBJECT *ob, long arg);
static void g3d_create_compute_obj(void);
static void g3d_delete_compute_obj(void);

/* Ajout bouton track traj (EF) */
static void CB_set_track_obj(FL_OBJECT *ob, long arg);
static void g3d_create_track_obj(void);
static void g3d_delete_track_obj(void);


static void CB_goto_obj(FL_OBJECT *ob, long arg);
static void g3d_create_goto_obj(void);
static void CB_position_obj(FL_OBJECT *ob, long arg);
static void g3d_create_position_obj(void);
static void CB_radius_obj(FL_OBJECT *ob, long arg);
static void g3d_create_radius_obj(void);
static void CB_dmax_obj(FL_OBJECT *ob, long arg);
static void g3d_create_dmax_obj(void);
static int CB_dmax_OnClose(FL_FORM *form, void* arg);
static void CB_dmax_input(FL_OBJECT *ob, long arg);
static void CB_dmax_exit(FL_OBJECT *ob, long arg);
static void CB_dmax_button_obj(FL_OBJECT *ob, long arg);
static void g3d_create_dmax_button_obj(void);
static void CB_trajmove_obj(FL_OBJECT *ob, long arg);
static void g3d_create_trajmove_obj(void);
static void CB_trajnum_obj(FL_OBJECT *ob, long arg);
static void g3d_create_trajnum_obj(void);

static void CB_addtraj_obj(FL_OBJECT *ob, long arg);
static void g3d_create_addtraj_obj(void);
static void CB_begtraj_obj(FL_OBJECT *ob, long arg);
static void g3d_create_begtraj_obj(void);
static void CB_endtraj_obj(FL_OBJECT *ob, long arg);
static void g3d_create_endtraj_obj(void);

//static void CB_showtraj_obj(FL_OBJECT *ob, long arg);
static void g3d_create_showtraj_obj(void);
static void CB_movietraj_obj(FL_OBJECT *ob, long arg);
static void CB_moviegraph_obj(FL_OBJECT *ob, long arg);

static void g3d_create_movietraj_obj(void);

static void g3d_create_mpeg_form(void);
static void g3d_delete_mpeg_form(void);
static void CB_mpeg_obj(FL_OBJECT *ob, long arg);
static void CB_mpeg_imagerate(FL_OBJECT *ob, long arg);
static void CB_mpeg_imagecompress(FL_OBJECT *ob, long arg);

static void CB_writepath_obj(FL_OBJECT *ob, long arg);
static void g3d_create_writepath_obj(void);
static void CB_loadpath_obj(FL_OBJECT *ob, long arg);
static void g3d_create_loadpath_obj(void);

static void CB_display_filterbox_obj(FL_OBJECT *ob, long arg);
static void g3d_create_display_filterbox_obj(void);
static void CB_adapt_filterbox_obj(FL_OBJECT *ob, long arg);
static void g3d_create_adapt_filterbox_obj(void);
static void CB_kine_constraints_obj(FL_OBJECT *ob, long arg);
static void g3d_create_kine_constraints_obj(void);

static void g3d_delete_goto_obj(void);
static void g3d_delete_position_obj(void);
static void g3d_delete_radius_obj(void);
static void g3d_delete_dmax_obj(void);
static void g3d_delete_dmax_button_obj(void);
static void g3d_delete_trajmove_obj(void);
static void g3d_delete_trajnum_obj(void);

static void g3d_delete_addtraj_obj(void);
static void g3d_delete_begtraj_obj(void);
static void g3d_delete_endtraj_obj(void);

static void g3d_delete_showtraj_obj(void);

static void g3d_delete_display_filterbox_obj(void);
static void g3d_delete_adapt_filterbox_obj(void);
static void g3d_delete_kine_constraints_obj(void);

static void g3d_delete_writepath_obj(void);
static void g3d_delete_loadpath_obj(void);
static void g3d_delete_movietraj_obj(void);

static int  default_drawtraj_fct(void);



static int lastchoice = 1;  /* variable pour indiquer l'etat de goto_obj */ 
static configPt last_p_deg[20]; /* variable pour guarder la derniere conf. valide d'un robot */
                               /* 20 == max.num.robots ??? */
static int x_FR[20],y_FR[20];
static int n_f_t[20];
static double sliderHeight;


/*
 *  Position the robot at the beginning of a path
 *
 */

void position_robot_at_beginning(int ir, p3d_traj *trajPt)
{
  ROBOTS_FORM[ir].pos_on_traj = 0;
}


/************************/
/* Robot form creation  */
/************************/
// modif Juan
int calc_real_dof(void)
{
  int nrd;
  int njnt,i,j,k;
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  nrd = robotPt->nb_user_dof;
  njnt = p3d_get_robot_njnt();

  if(njnt > MAX_NJNTS_IN_ROBOTFORM) {
    return 0;
  }

  for(i=0; i<=njnt; i++) {
    for(j=0; j<robotPt->joints[i]->dof_equiv_nbr; j++) {
      k = robotPt->joints[i]->index_dof + j;
      if((! p3d_jnt_get_dof_is_user(robotPt->joints[i], j)) &&
   (robotPt->cntrt_manager->in_cntrt[k] == 1)) {
  nrd++;
      }
    }
  }

  return nrd;
}
// fmodif Juan

void g3d_create_robot_form(int ir)
{int n;
 double s;

  p3d_sel_desc_num(P3D_ROBOT,ir);

  n = calc_real_dof();
  if(n > 30){
    sliderHeight = 10.0;
  }else{
    sliderHeight = 20.0;
  }
  s = n*sliderHeight+251.0;

  ROBOTS_FORM[ir].ROBOT_FORM = fl_bgn_form(FL_UP_BOX,400.0,s);
  g3d_create_position_obj(); /* Cree le menu des positions */
  g3d_create_goto_obj(); /* Cree le choix goto/current */
  g3d_create_trajmove_obj(); /* Cree les fleches sur le menu des positions */
  g3d_create_trajnum_obj(); /* Cree le menu des trajectoires*/

  g3d_create_begtraj_obj(); /* Cree le bouton BegTraj */
  g3d_create_addtraj_obj(); /* Cree le bouton AddTraj */
  g3d_create_endtraj_obj(); /* Cree le bouton EndTraj */
  g3d_create_showtraj_obj(); /* Cree le bouton ShowTraj */
  g3d_create_movietraj_obj(); /* Cree le bouton ShowTraj */
  g3d_create_writepath_obj(); /* Cree le bouton LPC */
  g3d_create_loadpath_obj(); /* Cree le bouton Load LPC */

  g3d_create_compute_obj(); /* Cree le bouton Compute trajectory */
  g3d_create_track_obj();   /* Cree le bouton Track Traj */

  /* Debut Modification Thibaut */
  g3d_create_printconfig_obj(); /* Cree le bouton PrintConfig */
  /* Fin Modification Thibaut */

  /* Debut Modification Fabien */
  g3d_create_config_obj(); /* Cree le menu des configurations */
  g3d_create_set_config_obj(); /* Cree le bouton d'initialisation d'une 
          configuration */
  //  g3d_create_get_config_obj(); /* Cree le bouton d'affichage d'une 
  //          configuration */
  //  g3d_create_save_config_obj(); /* Cree le bouton de sauvegarde des paramatres
  //           du robot */
  g3d_create_new_config_obj(); /* Cree le bouton d'ajout d'une 
          configuration */
  g3d_create_del_config_obj(); /* Cree le bouton de suppression d'une 
          configuration */
  /* Fin Modification Fabien */

  g3d_create_display_filterbox_obj(); /* (Carl) Cree le bouton DisplayFilterBox */
  g3d_create_adapt_filterbox_obj(); /* (Carl) : adaptation des limites de joints 
                   et de la boite de filtrage */
  g3d_create_kine_constraints_obj(); /* Cree le bouton KinematicConstraints */

  g3d_create_radius_obj(); /* Cree le bouton Radius*/
  g3d_create_dmax_button_obj();/* Cree le bouton Pas max.*/
  g3d_create_dmax_obj(); /* Cree le slider Pas max.*/
  fl_end_form();

  g3d_create_mpeg_form();  /* Cree la fenetre MPEG */

  if(n_f_t[ir])
    fl_set_form_position(ROBOTS_FORM[ir].ROBOT_FORM,x_FR[ir],y_FR[ir]);     
}

/**********************************************************************/



double g3d_get_qi(int i)
{p3d_rob *robotPt;
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  return(robotPt->ROBOT_POS[i]);
}

double g3d_get_qf(int i)
{p3d_rob *robotPt;
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  return(robotPt->ROBOT_GOTO[i]);
}

/* ajoute une trajectoire aux trajectoires du robot disponibles */
void g3d_add_traj(char *name,int i){
 char str[100];
 int ir = p3d_get_desc_curnum(P3D_ROBOT); 
 p3d_traj *trajPt = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);

  sprintf(str,"%s (%d)",name,i-1);
  fl_addto_choice(ROBOTS_FORM[ir].g3d_trajnum_obj,str);
  fl_set_choice(ROBOTS_FORM[ir].g3d_trajnum_obj,i);
  /* the robot is positioned at the beginning of the trajectory */
  position_robot_at_beginning(ir, trajPt);
}

/*************************************************************/
/* choix de la position courante ou but */
static void CB_goto_obj(FL_OBJECT *ob, long arg)
{
  int   val = fl_get_choice(ob);
  configPt p=NULL, p_deg=NULL; /* the same vector expressed in radian and in degree */
  configPt pp=NULL;
  int nb_dof, i, ir;
  p3d_rob *robotPt=NULL;
  int ncol=0, *ikSol = NULL;
  
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  nb_dof = p3d_get_robot_ndof();  

  /* copy current or goal configuration into p depending on 
     val. */
  if((val == 1)||(val == 2)) {
    if(val == 1){
      p = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
      robotSelectPositionFlag = 0;
      ikSol = robotPt->ikSolPos;
    }else{
      p = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
      robotSelectPositionFlag = 1;
      ikSol = robotPt->ikSolGoto;
    }
    // modif Juan
    //  if position cannot be updated because of constraints
    //  a valid conformation is searched locally
//     if(!p3d_set_and_update_robot_conf(p)) {
    if(!p3d_set_and_update_this_robot_conf_multisol(robotPt, p, NULL, 0, ikSol)){
      if(p3d_local_conf_correction(robotPt,p)) {
  if(val == 1) {
    p3d_copy_config_into(robotPt,p,&(robotPt->ROBOT_POS));
    printf("WARNING : ROBOT_POS locally corrected for constraits satisfaction\n");
//     print_config(robotPt, p);
  }
  else {
    p3d_copy_config_into(robotPt,p,&(robotPt->ROBOT_GOTO));
    printf("WARNING : ROBOT_GOTO locally corrected for constraits satisfaction\n");
  }
      }
      else {
  if(val == 1) {
    printf("WARNING : ROBOT_POS don't satisfy constraits\n");
  }
  else {
    printf("WARNING : ROBOT_GOTO don't satisfy constraits\n");
  }
      }
    }
    // fmodif Juan
        
    /* vector p with angles expressed in degree */
    p_deg = p3d_copy_config_rad_to_deg(robotPt, p);
    
    fl_freeze_form(ROBOTS_FORM[ir].ROBOT_FORM);
    
    /* for each degree of freedom, set the slider to the position
       corresponding to vector p */
    if(calc_real_dof() > 0) {
      for(i=0; i<nb_dof; i++){
  if(ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL) {
    /* angles to be expressed in degree here */
    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
  }
      }
    }
           
    if(G3D_ACTIVE_CC)
      { ncol = p3d_col_test_all(); }
     
    fl_unfreeze_form(ROBOTS_FORM[ir].ROBOT_FORM); 
    
    g3d_set_draw_coll(ncol);
    g3d_draw_allwin_active();
    
    /* desallocate vectors p and p_deg */
    p3d_destroy_config(robotPt, p);     
    p3d_destroy_config(robotPt, p_deg); 
    lastchoice = val;
  }
  else {      /* SWITCH */
    p = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
    pp = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
    p3d_copy_config_into(robotPt, pp, &(robotPt->ROBOT_POS));
    p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_GOTO));

    p3d_destroy_config(robotPt, p);
    p3d_destroy_config(robotPt, pp);

    fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,lastchoice); 
    CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,0);
  }
}


static void g3d_create_goto_obj(void)
{
 int       ir;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 

  ROBOTS_FORM[ir].GOTO_OBJ = fl_add_choice(FL_NORMAL_CHOICE,10.0,30.,150.0,40.0,"");
  fl_addto_choice(ROBOTS_FORM[ir].GOTO_OBJ,"Current Position");
  fl_addto_choice(ROBOTS_FORM[ir].GOTO_OBJ,"Goto    Position");
  fl_addto_choice(ROBOTS_FORM[ir].GOTO_OBJ,"     SWITCH     ");
  fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,1);
  fl_set_call_back(ROBOTS_FORM[ir].GOTO_OBJ,CB_goto_obj,0);
}

/**********************************************************************/

/* cree le menu des ddl */
static void CB_position_obj(FL_OBJECT *ob, long arg)
{
  double val = fl_get_slider_value(ob);
  configPt p=NULL, p_deg=NULL;
  int i,nb_dof,ir,i_dof;
  p3d_rob *robotPt;
  p3d_jnt *jntPt;
  int ncol=0;
  int I_can;
  int nreshoot;   // <-modif Juan
  int *ikSol = NULL;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  nb_dof = p3d_get_robot_ndof();

  p = p3d_alloc_config(robotPt);
  p_deg = p3d_alloc_config(robotPt);

  if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg);
    ikSol = robotPt->ikSolPos;
  }
  else{
    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO, &p_deg);
    ikSol = robotPt->ikSolGoto;
  }
  p_deg[arg] = val;
  p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);

  /*update the configuration of the current robot */
  /* sustitution pour la partie de la fonction dans l'anciene version de Move3D */
  /*   p3d_set_and_update_robot_conf(p); */ /* <- dans la fonction  remplacee */

  p3d_set_robot_config(robotPt, p);

//   I_can = p3d_update_robot_pos();
  I_can = p3d_update_this_robot_pos_multisol(robotPt, NULL, 0, ikSol);
  
  if (robotPt->cntrt_manager->cntrts != NULL) { 
    // modif Juan
    nreshoot = 0;
    if(!I_can && p3d_get_RLG()) {
      if(robotPt->cntrt_manager->in_cntrt[arg] == 1) {
  while(!I_can && (nreshoot < 100)) {
    if(p3d_random_loop_generator_without_parallel(robotPt, p)) {
      I_can = p3d_update_robot_pos();
    }
    nreshoot++;
  }  
      }
    }
    // fmodif Juan
    if(I_can) {
      p3d_get_robot_config_into(robotPt, &p);
      p3d_get_robot_config_deg_into(robotPt, &p_deg);
      for(i=0; i<nb_dof; i++) {
  if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
    { fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]); }
      }
      p3d_copy_config_into(robotPt, p_deg, &last_p_deg[ir]);
    } else {
      p3d_copy_config_into(robotPt, last_p_deg[ir], &p_deg);
      p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);
      for(i=0; i<nb_dof; i++) {
  if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
    { fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]); }
      }
      jntPt = p3d_robot_dof_to_jnt(robotPt, arg, &i_dof);
      p3d_jnt_set_dof_deg(jntPt, i_dof, p_deg[arg]);  /* ceneccasy lines for some cases of cntrts !!! */
      p3d_update_this_robot_pos_without_cntrt(robotPt);
    }
  }
  
  /* collision checking */
  if(g3d_get_KCD_CHOICE_IS_ACTIVE()) {
    if(G3D_ACTIVE_CC)
      { ncol = p3d_col_test_choice(); }
  } else {
    if(G3D_ACTIVE_CC)
      { ncol = p3d_col_test_all(); }
  }
  g3d_set_draw_coll(ncol);
  /* update the field current position or goal position of the 
     current robot depending on field GOTO_OBJ */
  if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
    p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_POS));
  }
  else{
    p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_GOTO));
  }
  
  /* update trajectory if track_traj mode is activated */
  if (ROBOTS_FORM[ir].TRACK_TRAJ)
    {
      CB_stop_obj(ob,0);
      CB_stop_optim_obj(ob,0);
      if (!p3d_trackCurrentTraj(robotPt,3, 0.001, p3d_get_d0(),p3d_get_QUICK_DESCENT()))
  {
    CB_specific_search_obj(ob,0);
  }
      p3d_set_and_update_robot_conf(p);
    }
  
#ifdef HRI_PLANNER
  if(BTSET!=NULL){
    for(i=0; i<BTSET->human_no; i++){
      if(BTSET->human[i]->HumanPt==robotPt && BTSET->human[i]->exists){
	BTSET->changed = TRUE;
	hri_bt_refresh_all(BTSET);
	break;
      }
    }
  }
  
  if(INTERPOINT!=NULL){
    for(i=0; i<INTERPOINT->human_no; i++){
      if(INTERPOINT->human[i]->HumanPt==robotPt && INTERPOINT->human[i]->exists){
	INTERPOINT->changed = TRUE;
	hri_bt_3drefresh_all(INTERPOINT);
	break;
      }
    }
  }
#endif

  g3d_draw_allwin_active();
  p3d_destroy_config(robotPt, p);
  p3d_destroy_config(robotPt, p_deg);
}

static void g3d_create_position_obj(void)
{ 
  double     f_min=0.0, f_max=0.0;
  char      str[30];
  int       i, j, k, ir, ord;
  int njnt, nb_dof;
  configPt robot_pos_deg;
  p3d_rob *robotPt;
  p3d_jnt * jntPt;
 

  nb_dof = p3d_get_robot_ndof();
  njnt = p3d_get_robot_njnt();
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  /* convert current robot position into degree to initialize
     sliders */
  last_p_deg[ir] = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_POS);
  robot_pos_deg = last_p_deg[ir];

  if(calc_real_dof() > 0) {
    ord = 0;
    for(i=0; i<=njnt; i++) {
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
  k = jntPt->index_dof + j;
  if((p3d_jnt_get_dof_is_user(jntPt, j)) ||
     (robotPt->cntrt_manager->in_cntrt[k] == 1)) {   // modif Juan
    p3d_jnt_get_dof_bounds_deg(jntPt, j, &f_min, &f_max);
    ROBOTS_FORM[ir].POSITION_OBJ[k] = 
      fl_add_valslider(FL_HOR_SLIDER,110.0,70.+sliderHeight*ord,280.0,sliderHeight,"");
    /***** Carole : on remet le min a gauche et le max a droite ******/
    fl_set_slider_bounds(ROBOTS_FORM[ir].POSITION_OBJ[k],
             f_min,f_max);  
    fl_set_object_lsize(ROBOTS_FORM[ir].POSITION_OBJ[k],FL_MEDIUM_SIZE);
    strcpy(str, p3d_jnt_get_dof_name(jntPt, j));
    fl_add_box(FL_UP_BOX,10.0,70.+sliderHeight*ord,100.0,sliderHeight,str);
    
    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[k],
            robot_pos_deg[k]);
    fl_set_call_back(ROBOTS_FORM[ir].POSITION_OBJ[k],CB_position_obj,k);
    if (robotPt->cntrt_manager->in_cntrt[k] == 2)
      { fl_set_slider_size(ROBOTS_FORM[ir].POSITION_OBJ[k],1.0); }
    ord = ord+1;
  } else
    { ROBOTS_FORM[ir].POSITION_OBJ[k] = NULL; }
      }
    }
  }
}

/*******************************************************/

/* permet de modifier le rayon de braquage du robot */
static void  CB_radius_obj(FL_OBJECT *ob, long arg)
{double val = fl_get_slider_value(ob);

 p3d_set_robot_radius(val); 
}

static void g3d_update_radius_obj(int ir, double val)
{
  if (val>EPS6) {
    fl_set_slider_bounds(ROBOTS_FORM[ir].RADIUS_OBJ,0.05*val,10.*val); 
    fl_set_slider_size(ROBOTS_FORM[ir].RADIUS_OBJ,0.1); 
    fl_set_slider_value(ROBOTS_FORM[ir].RADIUS_OBJ,val);
    fl_activate_object(ROBOTS_FORM[ir].RADIUS_OBJ);
  } else {
    fl_set_slider_bounds(ROBOTS_FORM[ir].RADIUS_OBJ,0,1);
    fl_set_slider_size(ROBOTS_FORM[ir].RADIUS_OBJ,1.0); 
    fl_set_slider_value(ROBOTS_FORM[ir].RADIUS_OBJ,0.);
    fl_deactivate_object(ROBOTS_FORM[ir].RADIUS_OBJ);
  }
}

static void g3d_create_radius_obj(void)
{
  int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
  double val = p3d_get_robot_radius();
  
  n = calc_real_dof();
  
  fl_add_box(FL_UP_BOX,10.0,116.+sliderHeight*(n+3),100.0,30.0,"Radius");
  ROBOTS_FORM[ir].RADIUS_OBJ = fl_add_valslider(FL_HOR_SLIDER,110.0,116.+
            sliderHeight*(n+3),280.0,30.0,"");
  g3d_update_radius_obj(ir, val);
  
  fl_set_call_back(ROBOTS_FORM[ir].RADIUS_OBJ,CB_radius_obj,0);
}

/*******************************************************/

/*******************************************************/

/* imprime la configuration courante dans le track-shell */
static void CB_printconfig_obj(FL_OBJECT *ob, long arg)
{int   ir = p3d_get_desc_curnum(P3D_ROBOT), val = fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ);
 double *q;
 pp3d_rob r = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

 q = p3d_get_robot_config_deg(r);
 if ((val == 1) && (p3d_equal_config(r, q, r->ROBOT_POS)))
   { printf("\nCurrent configuration of %s:\n",r->name); }
 else if ((val == 2) && (p3d_equal_config(r, q, r->ROBOT_GOTO)))
   { printf("\nGoal configuration of %s:\n",r->name); }
 else
   { printf("\nIntermediate configuration of %s:\n",r->name); }
 print_config(r, q);
 p3d_destroy_config(r, q);
}

static void g3d_create_printconfig_obj(void)
{int n,ir = p3d_get_desc_curnum(P3D_ROBOT);

  n = calc_real_dof();

  ROBOTS_FORM[ir].PRINT_CONFIGURATION_OBJ = fl_add_button(FL_NORMAL_BUTTON,295.0,93.+sliderHeight*(n+3),95.0,20.0,"Print Configuration");

  fl_set_call_back(ROBOTS_FORM[ir].PRINT_CONFIGURATION_OBJ,CB_printconfig_obj,0);
}
/* Fin Modification Thibaut */

/* Debut modification Fabien */

static void g3d_delete_printconfig_obj(void)
{int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].PRINT_CONFIGURATION_OBJ);
}

/*---------------------------------------------------------------------------
   Menu de choix des configurations */

static void CB_config_obj(FL_OBJECT *ob, long arg)
{
  int   ir, val = fl_get_choice(ob)-1;
  pp3d_rob robotPt;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if ((val>=0) && (val<robotPt->nconf)) {
    robotPt->confcur = robotPt->conf[val];
    if(robotPt->ikSol != NULL){
      p3d_destroy_specific_iksol(robotPt->cntrt_manager, robotPt->ikSol);
    }
    p3d_copy_iksol(robotPt->cntrt_manager, (robotPt->conf[val])->ikSol, &(robotPt->ikSol));
    p3d_set_and_update_robot_conf_multisol(robotPt->confcur->q, robotPt->confcur->ikSol);
    switch (fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ))
      {
      case 1 : { // Current position
  p3d_copy_config_into(robotPt, robotPt->confcur->q,
           &(robotPt->ROBOT_POS));
  if(robotPt->ikSolPos != NULL){
    p3d_destroy_specific_iksol(robotPt->cntrt_manager, robotPt->ikSolPos);
  }
  p3d_copy_iksol(robotPt->cntrt_manager, robotPt->confcur->ikSol, &(robotPt->ikSolPos));
      } break;
      case 2 : { // Goto position
  p3d_copy_config_into(robotPt, robotPt->confcur->q,
           &(robotPt->ROBOT_GOTO));
  if(robotPt->ikSolGoto != NULL){
    p3d_destroy_specific_iksol(robotPt->cntrt_manager, robotPt->ikSolGoto);
  }
  p3d_copy_iksol(robotPt->cntrt_manager, robotPt->confcur->ikSol, &(robotPt->ikSolGoto));
      } break;
      }
    CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,0);
    g3d_draw_allwin_active();
  }
}

static void g3d_create_config_obj(void)
{
  int       n, ir, i;
  pp3d_rob  r;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  r = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  n = calc_real_dof();

  ROBOTS_FORM[ir].CONFIG_OBJ = fl_add_choice(FL_NORMAL_CHOICE,10.0,53.+sliderHeight*(n+3),150.0,40.0,"");
  for(i=0; i<r->nconf; i++) {
    fl_addto_choice(ROBOTS_FORM[ir].CONFIG_OBJ,r->conf[i]->name);
    if (r->conf[i] == r->confcur)
      { fl_set_choice(ROBOTS_FORM[ir].CONFIG_OBJ,i+1); }
  }
  fl_set_call_back(ROBOTS_FORM[ir].CONFIG_OBJ,CB_config_obj,0);
}

static void g3d_delete_config_obj(void)
{int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].CONFIG_OBJ);
}


/*---------------------------------------------------------------------------
   Bouton permettant d'enregistrer une configuration */

static void CB_set_config_obj(FL_OBJECT *ob, long arg)
{
  int ir, len, i, num;
  pp3d_rob robotPt;
  char *name;
  const char *tmp;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if (robotPt->confcur != NULL) {
    switch (fl_get_button_numb(ob))
      {
      case 1 : { // bouton de gauche
  p3d_destroy_config(robotPt, robotPt->confcur->q);
  robotPt->confcur->q = p3d_get_robot_config(robotPt);
      } break;
      case 3 : { // bouton de droite Change le nom
  tmp = fl_show_input("Config name :", robotPt->confcur->name);
  if (tmp != NULL) {
    name = strdup(tmp);
    if (name == NULL) {
      fprintf(stderr, "Pas assez de memoire dans CB_set_config_obj !\n");
      return;
    }
    /* suppression des espaces */
    len = strlen(name);
    for(i=len-1; (i>=0) && (name[i] == ' '); i--) 
      { name[i] = '\0'; }
    for(; i>=0; i--) {
      if (name[i] == ' ')
        { name[i] = '_'; }
    }
    /* Verification de l'unicite*/
    num = 0;
    for(i=0; i<robotPt->nconf; i++) {
      if (robotPt->conf[i] == robotPt->confcur)
        { num = i+1; }
      else {
        if (strcmp(name, robotPt->conf[i]->name) == 0) {
    printf("Nom deja utilise!\n");
    free(name);
    return;
        }
      }
    }
    free(robotPt->confcur->name);
    robotPt->confcur->name = name;
    fl_replace_choice(ROBOTS_FORM[ir].CONFIG_OBJ, num, name);
  }
      } break;
      }
    g3d_draw_allwin_active();
  }
}

static void g3d_create_set_config_obj(void)
{
  int       n, ir;
  pp3d_rob  robotPt;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  n = calc_real_dof();
  ROBOTS_FORM[ir].SET_CONFIG_OBJ = fl_add_button(FL_NORMAL_BUTTON,10.0,93.+sliderHeight*(n+3),150.0,20.0,"Modif config");
  fl_set_call_back(ROBOTS_FORM[ir].SET_CONFIG_OBJ,CB_set_config_obj,0);
  if (robotPt->nconf<=0) {
    fl_deactivate_object(ROBOTS_FORM[ir].SET_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].SET_CONFIG_OBJ,FL_INACTIVE_COL);
  }
}

static void g3d_delete_set_config_obj(void)
{int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].SET_CONFIG_OBJ);
}

/*---------------------------------------------------------------------------
  Bouton permettant de calculer une trajectoire                 */

static void CB_compute_obj(FL_OBJECT *ob, long arg)
{
  int       ir      = p3d_get_desc_curnum(P3D_ROBOT); 
  int       it      = p3d_get_desc_number(P3D_TRAJ);
  pp3d_rob  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  configPt  qcur;
  if (fl_get_button(ROBOTS_FORM[ir].COMPUTE_OBJ))
    { 
      //Modif Mokhtar : A quoi ca sert ???
//       fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,1);
//       CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,1);

      CB_specific_search_obj(ob,0);
      if(p3d_get_desc_number(P3D_TRAJ) > it) {
  CB_start_optim_obj(ob,0);
  fl_set_button(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,1);
  CB_showtraj_obj( ROBOTS_FORM[ir].SHOWTRAJ_OBJ,0);
  qcur = p3d_get_robot_config(robotPt);
  p3d_copy_config_into(robotPt, qcur, &(robotPt->ROBOT_POS));
  p3d_copy_config_into(robotPt, qcur, &(robotPt->ROBOT_GOTO));
  p3d_destroy_config(robotPt, qcur);
      }
      fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,2);
      CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,2);
      fl_set_button(ROBOTS_FORM[ir].COMPUTE_OBJ,0);
    }
  else
    {
      CB_stop_obj(ob,0);
      CB_stop_optim_obj(ob,0);
      fl_set_button(ROBOTS_FORM[ir].COMPUTE_OBJ,1);
    }
}

static void g3d_delete_compute_obj(void)
{int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].COMPUTE_OBJ);
}

static void g3d_create_compute_obj(void)
{
  int       n, ir;
  pp3d_rob  robotPt;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  n = calc_real_dof();
  ROBOTS_FORM[ir].COMPUTE_OBJ = fl_add_button(FL_PUSH_BUTTON,100.0,25.+sliderHeight*(n+3),200.0,20.0,"Compute Trajectory");
  fl_set_call_back(ROBOTS_FORM[ir].COMPUTE_OBJ,CB_compute_obj,0);
}

/* Bouton permettant de suivre la trajectoire par bande elastique (EF) */
static void CB_set_track_obj(FL_OBJECT *ob, long arg)
{
  int       ir      = p3d_get_desc_curnum(P3D_ROBOT); 
  pp3d_rob  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  configPt  qcur    = p3d_get_robot_config(robotPt);
  ROBOTS_FORM[ir].TRACK_TRAJ = !(ROBOTS_FORM[ir].TRACK_TRAJ);
  fl_set_button(ROBOTS_FORM[ir].TRACK_OBJ, ROBOTS_FORM[ir].TRACK_TRAJ);
  if (ROBOTS_FORM[ir].TRACK_TRAJ) {
    if (!p3d_trackCurrentTraj(robotPt, 3, 0.001, p3d_get_d0(), p3d_get_QUICK_DESCENT())) {
      CB_specific_search_obj(ob, 0);
      CB_start_optim_obj(ob, 0);
      g3d_draw_allwin_active();
    }
    p3d_set_and_update_robot_conf(qcur);
  } else {
    CB_stop_obj(ob, 0);
    CB_stop_optim_obj(ob, 0);
  }
  p3d_destroy_config(robotPt, qcur);
}

static void g3d_create_track_obj(void)
{
  int       n, ir;
  pp3d_rob  robotPt;
  
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  n = calc_real_dof();
  ROBOTS_FORM[ir].TRACK_OBJ = fl_add_button(FL_PUSH_BUTTON,10.0,25.+sliderHeight*(n+3),50.0,20.0,"Track Traj");
  fl_set_call_back(ROBOTS_FORM[ir].TRACK_OBJ,CB_set_track_obj,0);
  ROBOTS_FORM[ir].TRACK_TRAJ = FALSE;
}
static void g3d_delete_track_obj(void)
{
  int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].TRACK_OBJ);
}
/*---------------------------------------------------------------------------
   Bouton permettant d'ajouter une nouvelle configuration au robot */

static void CB_new_config_obj(FL_OBJECT *ob, long arg)
{
  char name[31];
  int val, ir, num, i;
  pp3d_rob robotPt;
  configPt q;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  /* Choix d'un nom de configuration unique */
  num = 1;
  for(i=0; i<robotPt->nconf; i++) {
    if (sscanf(robotPt->conf[i]->name, "config_%i", &val) > 0)
      { num = MAX(num, val+1); }
  }
  sprintf(name, "config_%i", num);
  q = p3d_get_robot_config(robotPt);

  /* Ajout de la configuration */
  switch (fl_get_button_numb(ob)){
    case 1 : { // bouton de gauche : Ajoute en debut de liste
      p3d_set_new_robot_config(name, q, robotPt->ikSol, NULL); /* Ajoute en debut de liste */
      num = 1;
    } break;
    case 3 : { // bouton de droite : Insertion
      p3d_set_new_robot_config(name, q, robotPt->ikSol, robotPt->confcur);
      num = fl_get_choice(ROBOTS_FORM[ir].CONFIG_OBJ) + 1;
    } break;
  }
  fl_addto_choice(ROBOTS_FORM[ir].CONFIG_OBJ, name);
  /* Mise a jour de la liste */
  for(i=fl_get_choice_maxitems(ROBOTS_FORM[ir].CONFIG_OBJ); i>num; i--) {
    fl_replace_choice(ROBOTS_FORM[ir].CONFIG_OBJ, i, robotPt->conf[i-1]->name);
  }
  fl_replace_choice(ROBOTS_FORM[ir].CONFIG_OBJ, num, name);
  fl_set_choice(ROBOTS_FORM[ir].CONFIG_OBJ, num);
  p3d_destroy_config(robotPt, q);

  /* Activation des boutons */
  if (robotPt->nconf>0) {
    fl_activate_object(ROBOTS_FORM[ir].SET_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].SET_CONFIG_OBJ,FL_BLACK);
    fl_activate_object(ROBOTS_FORM[ir].DEL_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].DEL_CONFIG_OBJ,FL_BLACK);
  }
}

static void g3d_create_new_config_obj(void)
{
  int       n, ir;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  n = calc_real_dof();
  ROBOTS_FORM[ir].NEW_CONFIG_OBJ = fl_add_button(FL_NORMAL_BUTTON,160.0,93.+sliderHeight*(n+3),70.0,20.0,"New config");
  fl_set_call_back(ROBOTS_FORM[ir].NEW_CONFIG_OBJ,CB_new_config_obj,0);
}

static void g3d_delete_new_config_obj(void)
{int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].NEW_CONFIG_OBJ);
}


/*---------------------------------------------------------------------------
   Bouton permettant de supprimer une configuration existante */

static void CB_del_config_obj(FL_OBJECT *ob, long arg)
{
  int       ir, num, max;
  pp3d_rob  robotPt;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  if (robotPt->confcur != NULL) {
    p3d_del_config(robotPt, robotPt->confcur);
    num = fl_get_choice(ROBOTS_FORM[ir].CONFIG_OBJ);
    fl_delete_choice(ROBOTS_FORM[ir].CONFIG_OBJ, num);
    max = fl_get_choice_maxitems(ROBOTS_FORM[ir].CONFIG_OBJ);
    if (max>0) {
      fl_set_choice(ROBOTS_FORM[ir].CONFIG_OBJ, MIN(num, max));
    }
  }
  if (robotPt->nconf<=0) {
    fl_deactivate_object(ROBOTS_FORM[ir].SET_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].SET_CONFIG_OBJ,FL_INACTIVE_COL);
    fl_deactivate_object(ROBOTS_FORM[ir].DEL_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].DEL_CONFIG_OBJ,FL_INACTIVE_COL);
  }
}

static void g3d_create_del_config_obj(void)
{
  int       n, ir;
  pp3d_rob  robotPt;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  n = calc_real_dof();
  ROBOTS_FORM[ir].DEL_CONFIG_OBJ = fl_add_button(FL_NORMAL_BUTTON,230.0,93.+sliderHeight*(n+3),65.0,20.0,"Delete conf.");
  fl_set_call_back(ROBOTS_FORM[ir].DEL_CONFIG_OBJ,CB_del_config_obj,0);
  if (robotPt->nconf<=0) {
    fl_deactivate_object(ROBOTS_FORM[ir].DEL_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].DEL_CONFIG_OBJ,FL_INACTIVE_COL);
  }
}

static void g3d_delete_del_config_obj(void)
{int       ir;
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].DEL_CONFIG_OBJ);
}


/* Fin modification Fabien */

/* permet de regarder a la boite de filtrage  */
static void  CB_display_filterbox_obj(FL_OBJECT *ob, long arg)
{ 
/* int ir = p3d_get_desc_curnum(P3D_ROBOT); */

/*  if(!p3d_filter_needs_init()) { */
/*     p3d_filter_switch_draw_robot_box(); */
/*     g3d_draw_allwin_active(); */
/*  } */
/*   else */
/*     fl_set_button(ROBOTS_FORM[ir].DISPLAY_FILTERBOX_OBJ,0); */

  printf("FilterBox functions have been disabled\n");

}

static void g3d_create_display_filterbox_obj(void)
{
 int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
 n = calc_real_dof();

 ROBOTS_FORM[ir].DISPLAY_FILTERBOX_OBJ = fl_add_button(FL_PUSH_BUTTON,230.0,53.+sliderHeight*(n+3),65.0,37.0,"Show\nFilter Box");  /* modif Fabien -> changement du placement du botton       (c'etais  160.0,50.+20.0*(n+1),90.0,20.0)*/
 fl_set_call_back(ROBOTS_FORM[ir].DISPLAY_FILTERBOX_OBJ,CB_display_filterbox_obj,0);
}
/*******************************************************/

/* permet d'adapter la boite de filtrage, en changeant 
   les limites de certains DDLs  */
static void CB_adapt_filterbox_obj(FL_OBJECT *ob, long arg)
{
/*   int val = fl_get_button(ob); */
/*   int ir = p3d_get_desc_curnum(P3D_ROBOT);  */

/*   if(val)  */
/*     { */
/*       FORMfilter_init_temp_jntdata(); */
/*       fl_show_form(FILTER_FORM[ir].ROBOT_FORM,FL_PLACE_SIZE,TRUE, */
/*       p3d_get_desc_curname(P3D_ROBOT)); */
/*       fl_set_button(FILTER_FORM[ir].OK_OBJ,0); */
/*       fl_set_button(FILTER_FORM[ir].CANCEL_OBJ,0); */
/*     } */
/*   else */
/*     fl_hide_form(FILTER_FORM[ir].ROBOT_FORM); */

  printf("FilterBox functions have been disabled\n");

}

static void g3d_create_adapt_filterbox_obj(void)
{
 int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
 n = calc_real_dof();

 ROBOTS_FORM[ir].ADAPT_FILTERBOX_OBJ = fl_add_button(FL_PUSH_BUTTON,295.0,53.+sliderHeight*(n+3),95.0,37.0,"Adapt Joint Limits\nand Filter Box"); /*  modif Fabien -> changement du placement du botton (c'etait 250.0,50.+20.0*(n+1),150.0,20.0)*/
 fl_set_call_back(ROBOTS_FORM[ir].ADAPT_FILTERBOX_OBJ,CB_adapt_filterbox_obj,0);
}

/***************************************************************/
/* permet travailler a Move3D avec quelques contraintes cinematiques dans le robot  */
static void CB_kine_constraints_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

  if(val) {
    g3d_kin_constraints_form();
  }
}

static void g3d_create_kine_constraints_obj(void)
{
 int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
 n = calc_real_dof();

 ROBOTS_FORM[ir].KINE_CONSTRAINTS_OBJ = fl_add_button(FL_PUSH_BUTTON,160.0,53.0+sliderHeight*(n+3),70.0,37.0,"Kinematic\nConstraints"); /* modif Fabien -> changement du placement du botton (c'etait 160.0,71.0+20.0*(n+1),240.0,20.0) */
 fl_set_call_back(ROBOTS_FORM[ir].KINE_CONSTRAINTS_OBJ,CB_kine_constraints_obj,0);
}

/***************************************************************/

/* permet de modifier le pas d'affichage d'une trajectoire */
static void CB_dmax_input(FL_OBJECT *ob, long arg){
  int ir = p3d_get_desc_curnum(P3D_ROBOT), val = atoi(fl_get_input(DMAX_INPUT));
  p3d_set_env_graphic_dmax(val);
  fl_set_slider_value(ROBOTS_FORM[ir].DMAX_OBJ, val);
}
static void CB_dmax_exit(FL_OBJECT *ob, long arg){
  fl_hide_form(DMAX_FORM);
  g3d_fl_free_object(DMAX_INPUT);
  g3d_fl_free_object(DMAX_EXIT);
  g3d_fl_free_object(DMAX_FRAME);
  fl_free_form(DMAX_FORM);
}
static int CB_dmax_OnClose(FL_FORM *form, void * arg){
  fl_trigger_object(DMAX_EXIT);
  return FL_IGNORE;
}
static void  CB_dmax_button_obj(FL_OBJECT *ob, long arg){
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  char buffer [10];

  DMAX_FORM = fl_bgn_form(FL_FLAT_BOX,100,90);
  g3d_create_frame(&DMAX_FRAME,FL_NO_FRAME,80,60,"",(void**)&DMAX_FORM,1);
  g3d_create_input(&DMAX_INPUT,FL_NORMAL_INPUT,30,20,"Pas\nmax.",(void**)&DMAX_FRAME,0);
  fl_get_slider_value(ROBOTS_FORM[ir].DMAX_OBJ);
  sprintf(buffer, "%d", (int)fl_get_slider_value(ROBOTS_FORM[ir].DMAX_OBJ));
  fl_set_input(DMAX_INPUT, buffer);
  fl_set_input_color(DMAX_INPUT, 0, 0);
  fl_set_object_callback(DMAX_INPUT,CB_dmax_input,0);
  g3d_create_button(&DMAX_EXIT,FL_PUSH_BUTTON,40,20,"Exit",(void**)&DMAX_FRAME,0);
  fl_set_object_callback(DMAX_EXIT,CB_dmax_exit,0);
  fl_end_form();
  fl_set_form_atclose(DMAX_FORM, CB_dmax_OnClose, NULL);
  fl_show_form(DMAX_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_dmax_button_obj(void){
  int n = calc_real_dof(), ir = p3d_get_desc_curnum(P3D_ROBOT);

  ROBOTS_FORM[ir].DMAX_BUTTON_OBJ = fl_add_button(FL_NORMAL_BUTTON,10.0,146.+sliderHeight*(n+3),100.0,30.0,"Pas max.");
  fl_set_call_back(ROBOTS_FORM[ir].DMAX_BUTTON_OBJ,CB_dmax_button_obj,0);
}


static void  CB_dmax_obj(FL_OBJECT *ob, long arg)
{double val = fl_get_slider_value(ob);

 p3d_set_env_graphic_dmax(val); 
}

static void g3d_create_dmax_obj(void)
{
 int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
 double val = p3d_get_env_graphic_dmax();
 double vmin,v,vmax;

 n = calc_real_dof();
 v = p3d_get_env_graphic_dmax();

 vmax = 10*v;
 vmin = 0.1*v;

//  fl_add_box(FL_UP_BOX,10.0,146.+sliderHeight*(n+3),100.0,30.0,"Pas max.");
 ROBOTS_FORM[ir].DMAX_OBJ = fl_add_valslider(FL_HOR_SLIDER,110.0,146.+sliderHeight*(n+3),280.0,30.0,"");
 fl_set_slider_bounds(ROBOTS_FORM[ir].DMAX_OBJ,vmin,vmax);
 fl_set_object_lsize(ROBOTS_FORM[ir].DMAX_OBJ,FL_MEDIUM_SIZE);
 fl_set_slider_value(ROBOTS_FORM[ir].DMAX_OBJ,val);
 fl_set_call_back(ROBOTS_FORM[ir].DMAX_OBJ,CB_dmax_obj,0);
}

/***************************************************************************/

/* 
 *  Visualization of a local path
 *
 *  Input:  the robot, the local path, the maximal distance each 
 *          body can move between each picture
 *
 *  Description: draws the beginning of the local path. The next 
 *          configuration to draw is computed in such a way that 
 *          each body does not move by more than the distance given
 *          as input.
 */

int draw_localpath(p3d_rob *robotPt, p3d_localpath *localpathPt,
       double dmax)
{
  double u=0, du, umax; /* parameters along the local path */
  configPt q, qsave;
  int njnt = robotPt->njoints;
  double *distances;
  int i, end_localpath = 0;

  umax = localpathPt->range_param;
  distances = MY_ALLOC(double, njnt+1);

  /* current position of robot is saved */
  qsave = p3d_get_robot_config(robotPt);

  while (end_localpath < 2){
    /* position of the robot corresponding to parameter u */
    q = localpathPt->config_at_param(robotPt, localpathPt, u);
    p3d_set_robot_config(robotPt, q);

    /* collision checking */
    g3d_set_draw_coll(p3d_col_test_all());
    g3d_draw_allwin_active();

    /* each body is allowed to move by distance dmax */
    for (i=0; i<=njnt; i++){
      distances[i] = dmax;
    }

    du = localpathPt->stay_within_dist(robotPt, localpathPt,
               u, FORWARD, distances);
    u+=du;
    if (u > umax){
      u = umax;
      end_localpath++;
    }
  }
  /* The initial position of the robot is recovered */
  p3d_set_robot_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  return FALSE;
}

/*
 *  move the current robot on its current trajectory
 *
 *  Input:  an argument
 *
 *  Description: move the current robot on its current trajectory and 
 *          displays the new position. The different values of arg
 *          correspond to the following actions
 *             arg = 0         move twice backward
 *             arg = 1         move once backward
 *             arg = 2         move once forward
 *             arg = 3         move twice forward
 *          for each move, any point on the robot does not move by
 *          more than the distance dmax stored in the current environment.
 */

static void CB_trajmove_obj(FL_OBJECT *ob, long arg)
{
  int   i,njnt,ir;
  configPt q;
  p3d_rob *robotPt;
  double dmax = p3d_get_env_graphic_dmax();
  double *distances, localparam;
  p3d_traj    *trajPt;
  whichway dir;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  trajPt = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ); 
  njnt = p3d_get_robot_njnt(); 

  distances = MY_ALLOC(double, njnt+1);
  if((arg==0)||(arg==3)) 
    dmax*=2.0;

  for (i=0; i<=njnt; i++){
    distances[i] = dmax;
  }
  
  ROBOTS_FORM[ir].g3d_trajnum = p3d_get_desc_number(P3D_TRAJ)-1;
  if(ROBOTS_FORM[ir].g3d_trajnum>=0) {
    /* position of the robot on the current trajectory */
    localparam = ROBOTS_FORM[ir].pos_on_traj;

    if(trajPt != NULL) {
      if(arg >= 2) {
  /* move forward on the trajectory */
  dir = FORWARD;
      }
      else {
  /* move backward on the trajectory */
  dir = BACKWARD;
    }
      p3d_traj_stay_within_dist(trajPt, 
        dir, 
        distances, 
        &localparam); 
      /* store the new position */
      ROBOTS_FORM[ir].pos_on_traj = localparam;
      q = p3d_config_at_param_along_traj(trajPt,localparam);
      if(lastchoice == 1) {
  p3d_copy_config_into(robotPt, q, &(robotPt->ROBOT_POS));
  p3d_ikSol_at_param_along_traj(trajPt,localparam, &(robotPt->ikSolPos));
  fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,1);
  CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,0);      }
      else {
  p3d_copy_config_into(robotPt, q, &(robotPt->ROBOT_GOTO));
  p3d_ikSol_at_param_along_traj(trajPt,localparam, &(robotPt->ikSolGoto));
  fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,2); 
  CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,0);      }
      p3d_destroy_config(robotPt, q);
    }
  }
  MY_FREE(distances, double, njnt+1);
}


static void g3d_create_trajmove_obj(void)
{
 double x,y;
 int i,ir = p3d_get_desc_curnum(P3D_ROBOT);

  x = 160.0;
  y = 30.;

  ROBOTS_FORM[ir].g3d_trajmove_obj[0]=fl_add_button(FL_TOUCH_BUTTON,x+10.0,y,20.0,40.0,"@<<");
  ROBOTS_FORM[ir].g3d_trajmove_obj[1]=fl_add_button(FL_TOUCH_BUTTON,x+30.0,y,20.0,40.0,"@<");
  ROBOTS_FORM[ir].g3d_trajmove_obj[2]=fl_add_button(FL_TOUCH_BUTTON,x+190.0,y,20.0,40.0,"@>");
  ROBOTS_FORM[ir].g3d_trajmove_obj[3]=fl_add_button(FL_TOUCH_BUTTON,x+210.0,y,20.0,40.0,"@>>");
  for(i=0;i<4;i++) fl_set_call_back(ROBOTS_FORM[ir].g3d_trajmove_obj[i],CB_trajmove_obj,i);
}

/**********************************************************************/

/* choix de la trajectoire courante */
static void CB_trajnum_obj(FL_OBJECT *ob, long arg)
{
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  configPt q_init, q_end;
  p3d_rob *robotPt;
  pp3d_traj trajPt;
 
  ROBOTS_FORM[ir].g3d_trajnum = fl_get_choice(ob)-1;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  if(ROBOTS_FORM[ir].g3d_trajnum >=0) {
    /* set current trajectory to according to choice*/
    trajPt = (p3d_traj*) p3d_sel_desc_num(P3D_TRAJ,ROBOTS_FORM[ir].g3d_trajnum);
    ROBOTS_FORM[ir].pos_on_traj = 0;
    /* get configuration at beginning and end of trajectory */
    p3d_ends_and_length_traj(trajPt, &q_init, &q_end);
      /* write it into ROBOT_POS */
    p3d_copy_config_into(robotPt, q_init, &(robotPt->ROBOT_POS));
    p3d_copy_config_into(robotPt, q_end, &(robotPt->ROBOT_GOTO)); 
    fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,2); 
    fl_redraw_form(ROBOTS_FORM[ir].ROBOT_FORM);
    lastchoice = 1;

    g3d_set_win_drawer(G3D_WIN, g3d_draw_trace);
    g3d_draw_allwin_active();
    g3d_set_win_drawer(G3D_WIN, g3d_draw);
   
    p3d_destroy_config(robotPt, q_init);
    p3d_destroy_config(robotPt, q_end);
  } 
}

static void g3d_create_trajnum_obj(void)
{double  x,y;
 int i,ir = p3d_get_desc_curnum(P3D_ROBOT);

  x = 260.0;
  y = 30. ;
  ROBOTS_FORM[ir].g3d_trajnum_obj = fl_add_choice(FL_NORMAL_CHOICE,210.0,y,140.0,40.0,"");

    for(i=0;i<p3d_get_desc_number(P3D_TRAJ);i++)
      g3d_add_traj("Traj",i+1);
    fl_set_choice(ROBOTS_FORM[ir].g3d_trajnum_obj,1);
    fl_set_call_back(ROBOTS_FORM[ir].g3d_trajnum_obj,CB_trajnum_obj,0);
}

/**************************************************************************/

// static FL_FORM    *PLANNER_CHOICE_FORM = NULL;
// static FL_OBJECT  *SEARCH_LINEAR_OBJ;
// static FL_OBJECT  *SEARCH_PLATEFORM_OBJ;
// static FL_OBJECT  *SEARCH_ARM_OBJ;
// static FL_OBJECT  *SEARCH_LOCAL_OBJ;
// static FL_OBJECT  *SEARCH_MANHAT_OBJ;


/* permet le choix du planificateur local dans BegTraj */
#if 0
static 
void CB_buttons_obj(FL_OBJECT *ob, long arg)
{int ir = p3d_get_desc_curnum(P3D_ROBOT);

  if(arg == 0){ROBOTS_FORM[ir].PLANNER_CHOICE = 1;}
  if(arg == 1){ROBOTS_FORM[ir].PLANNER_CHOICE = 2;}
  if(arg == 2){ROBOTS_FORM[ir].PLANNER_CHOICE = 3;}
  if(arg == 3){ROBOTS_FORM[ir].PLANNER_CHOICE = 4;}
  if(arg == 4){ROBOTS_FORM[ir].PLANNER_CHOICE = 5;}
  fl_hide_form(PLANNER_CHOICE_FORM);
  fl_free_object(SEARCH_LOCAL_OBJ);
  fl_free_object(SEARCH_LINEAR_OBJ);
  fl_free_object(SEARCH_PLATEFORM_OBJ);
  fl_free_object(SEARCH_ARM_OBJ);
  fl_free_object(SEARCH_MANHAT_OBJ);
  fl_free_form(PLANNER_CHOICE_FORM);
} 

static 
void g3d_create_planner_choice_form(void)
{
  PLANNER_CHOICE_FORM = fl_bgn_form(FL_UP_BOX,150.0,120.0);

  SEARCH_LOCAL_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,10,55,30,"R&S + arm ");
  fl_set_object_color(SEARCH_LOCAL_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_LOCAL_OBJ,CB_buttons_obj,0);
  SEARCH_LINEAR_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,30,55,30,"Linear");
  fl_set_object_color(SEARCH_LINEAR_OBJ,FL_MCOL,FL_GREEN);
  fl_set_button(SEARCH_LINEAR_OBJ,1);
  fl_set_call_back(SEARCH_LINEAR_OBJ,CB_buttons_obj,1);
  SEARCH_PLATEFORM_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,50,55,30,"Plateform");
  fl_set_object_color(SEARCH_PLATEFORM_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_PLATEFORM_OBJ,CB_buttons_obj,2);
  SEARCH_ARM_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,70,55,30,"R&S");
  fl_set_object_color(SEARCH_ARM_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_ARM_OBJ,CB_buttons_obj,3);
  SEARCH_MANHAT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,90,55,30,"Manhattan");
  fl_set_object_color(SEARCH_MANHAT_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_MANHAT_OBJ,CB_buttons_obj,3);
  fl_end_form();
  fl_show_form(PLANNER_CHOICE_FORM,FL_PLACE_SIZE,TRUE,"LocalPlanner");
}
#endif

/* permet de commencer a entrer une trajectoire manuelle */
static configPt qi, qf;
static p3d_localpath *tman;
static configPt ROBOT_START;
/* Modif Fabien -> Gestion boutons trajectoire manuelle */
/* static int MANUAL_TRAJ = 0; */
/* Fin Modif */

static void CB_begtraj_obj(FL_OBJECT *ob, long arg)
{ int  i,ir,val,nrob;
  p3d_rob *robotPt;
  configPt q;

  /* g3d_create_planner_choice_form(); */

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  q = p3d_alloc_config(robotPt);
  ROBOT_START = p3d_alloc_config(robotPt);

  val = fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ);

//edit mokhtar code du if et du else identiques !!!
  
//   if(val == 1){
    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &q);
    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &ROBOT_START);
//   } else {
//     p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &q);
//     p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &ROBOT_START);
//   }

  qi = q;
  tman = NULL;
  /* Modif Fabien : Gestion boutons trajectoire manuelle */
  /* MANUAL_TRAJ = 1; */
  nrob = p3d_get_desc_number(P3D_ROBOT);
  for(i=0; i<nrob; i++) { /* Interdi l'utilisation de differents robots au   */
    if (i!= ir) {         /* cours de la construction d'une meme trajectoire */
      fl_deactivate_object(ROBOTS_FORM[i].ENDTRAJ_OBJ);
      fl_set_object_lcol(ROBOTS_FORM[i].ENDTRAJ_OBJ,FL_INACTIVE_COL);
      fl_deactivate_object(ROBOTS_FORM[i].ADDTRAJ_OBJ);
      fl_set_object_lcol(ROBOTS_FORM[i].ADDTRAJ_OBJ,FL_INACTIVE_COL);
    }
    fl_deactivate_object(ROBOTS_FORM[i].BEGTRAJ_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[i].BEGTRAJ_OBJ,FL_INACTIVE_COL);
  }
  fl_activate_object(ROBOTS_FORM[ir].ENDTRAJ_OBJ);
  fl_set_object_lcol(ROBOTS_FORM[ir].ENDTRAJ_OBJ,FL_BLACK);
  fl_activate_object(ROBOTS_FORM[ir].ADDTRAJ_OBJ);
  fl_set_object_lcol(ROBOTS_FORM[ir].ADDTRAJ_OBJ,FL_BLACK);
 /* Fin modif Fabien */
  
  p3d_beg_desc(P3D_TRAJ,(char*) "Manual traj");
}

static void g3d_create_begtraj_obj(void)
{
 int ir = p3d_get_desc_curnum(P3D_ROBOT);

  ROBOTS_FORM[ir].BEGTRAJ_OBJ = fl_add_button(FL_NORMAL_BUTTON,
            10.0,10.,50.0,20.0,
            "Beg Traj");
  fl_set_call_back(ROBOTS_FORM[ir].BEGTRAJ_OBJ,CB_begtraj_obj,0);
}


/* permet de terminer une trajectoire manuelle */
static void CB_endtraj_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  int ntest = 0, ir,i, nrob;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  p3d_end_desc();
  /* Modif Fabien : Gestion trajectoire nulle */
  if (robotPt->tcur->courbePt == NULL) {
    printf("COURBE VIDE !\n"); 
    p3d_del_traj(robotPt->tcur);    
  } else {
  /* fin Modif Fabien */
    if(p3d_col_test_traj(robotPt, robotPt->tcur->courbePt, &ntest)){
      printf("COURBE NON VALIDE !\n");}
    else{printf("Courbe valide\n");}
    g3d_add_traj("Manual",p3d_get_desc_number(P3D_TRAJ));
    g3d_show_tcur_rob(robotPt,default_drawtraj_fct);
    
    p3d_copy_config_into(robotPt, ROBOT_START, &(robotPt->ROBOT_POS));
    p3d_copy_config_into(robotPt, qf, &(robotPt->ROBOT_GOTO));

    fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ, 2);
  }
  p3d_destroy_config(robotPt, ROBOT_START);
  p3d_destroy_config(robotPt, qf);
    
  /* Modif Fabien : Gestion boutons trajectoire manuelle */
  nrob = p3d_get_desc_number(P3D_ROBOT);
  for(i=0; i<nrob; i++) { /* Autorise la creation d'une nouvelle trajectoire */
    fl_activate_object(ROBOTS_FORM[i].BEGTRAJ_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[i].BEGTRAJ_OBJ,FL_BLACK);
  }
  fl_deactivate_object(ROBOTS_FORM[ir].ADDTRAJ_OBJ);
  fl_set_object_lcol(ROBOTS_FORM[ir].ADDTRAJ_OBJ,FL_INACTIVE_COL);
  fl_deactivate_object(ROBOTS_FORM[ir].ENDTRAJ_OBJ);
  fl_set_object_lcol(ROBOTS_FORM[ir].ENDTRAJ_OBJ,FL_INACTIVE_COL);
  /*MANUAL_TRAJ = 0; */
  /* Fin modif Fabien */
}

static void g3d_create_endtraj_obj(void)
{
 int ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
 ROBOTS_FORM[ir].ENDTRAJ_OBJ = fl_add_button(FL_NORMAL_BUTTON,
            110.0,10.,50.0,20.0,
            "End Traj");
 /* Modif Fabien : Gestion bouton trajectoire manuelle */
 fl_deactivate_object(ROBOTS_FORM[ir].ENDTRAJ_OBJ);
 fl_set_object_lcol(ROBOTS_FORM[ir].ENDTRAJ_OBJ,FL_INACTIVE_COL);
 /* Fin modif Fabien */
  fl_set_call_back(ROBOTS_FORM[ir].ENDTRAJ_OBJ,CB_endtraj_obj,0);
}


/* permet d'ajouter une configuration intermediaire dans la trajectoire 
   manuelle */
static void CB_addtraj_obj(FL_OBJECT *ob, long arg)
{
  int   ir,val;
  p3d_rob *robotPt;
  p3d_localpath *localpathPt;
  configPt q;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  q = p3d_alloc_config(robotPt);

  val = fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ);

  if(val == 1) {
    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &q);
  } else {
    p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &q);
  } 

  /* the local planner function corresponding to 
     ROBOTS_FORM[ir].PLANNER_CHOICE is called */

  localpathPt = p3d_local_planner(robotPt,qi,q);

  if (localpathPt != NULL)   p3d_add_desc_courbe(localpathPt);

  p3d_destroy_config(robotPt, qi);
  qi = q;
  qf = q;
 
}

static void g3d_create_addtraj_obj(void)
{
 int ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
  ROBOTS_FORM[ir].ADDTRAJ_OBJ = fl_add_button(FL_NORMAL_BUTTON,
            60.0,10.,50.0,20.0,
            "Add Traj");
 /* Modif Fabien : Gestion bouton trajectoire manuelle */
 fl_deactivate_object(ROBOTS_FORM[ir].ADDTRAJ_OBJ);
 fl_set_object_lcol(ROBOTS_FORM[ir].ADDTRAJ_OBJ,FL_INACTIVE_COL);
 /* Fin modif Fabien */
  fl_set_call_back(ROBOTS_FORM[ir].ADDTRAJ_OBJ,CB_addtraj_obj,0);
}

/*************************************************************/

/* trace la trajectoire courante */

static int traj_play = TRUE;

static int 
default_drawtraj_fct(void)
{
  fl_check_forms();
  return(traj_play);
}

void CB_showtraj_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  int val =  fl_get_button(ob); 
  if(val) {
    traj_play = TRUE;
    g3d_show_tcur_rob(robotPt,default_drawtraj_fct);
    fl_set_button(ob,0);
    ////////// BIO /////////
#ifdef BIO
    if(bio_get_flag_evaluate_traj() == 1)
      bio_evaluate_traj(NULL);
#endif
    ////////////////////////
  }

  else {traj_play = FALSE;}
}

static void g3d_create_showtraj_obj(void)
{void CB_showtraj_obj(FL_OBJECT *ob, long arg);
 int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 ROBOTS_FORM[ir].SHOWTRAJ_OBJ = fl_add_button(FL_PUSH_BUTTON,
            170.0,10.,50.0,20.0,
            "Show Traj");
  fl_set_call_back(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,CB_showtraj_obj,0);
}


/* Debut modification Fabien */
/* traj declarations : begin */
typedef enum { LPC_TRAJ_FORMAT, M3D_TRAJ_FORMAT } save_format;
static save_format traj_format = M3D_TRAJ_FORMAT;  

static FILE *LPC_fd = NULL;
static double lpc_rank_nr = 0.0; /* timing of path */
/* traj declarations : end */

/* ************************ */
/*  TRAJECTORY FILE         */
/* ************************ */
static int file_name_exists(const char * file)
{
  if((LPC_fd=fopen(file,"r"))) {
    fclose(LPC_fd);
    return(TRUE);
  }
  return(FALSE);
}

static void CB_save_format(void * data)
{
  FD_FSELECTOR *fs;
  char * pattern, * file, * end;

  fs = fl_get_fselector_fdstruct();
  pattern = (char *)fl_get_pattern();
  file = (char *)fl_get_filename();
  switch(traj_format)
    {
    case M3D_TRAJ_FORMAT : {
      fl_set_object_label(fs->appbutt[0], "Change to\nMove3d format");
      strcpy(pattern, "*.lpc");
      end = strstr(file, ".trj");
      if (end == NULL) 
  { file[0] = '\0'; }
      else
  { strcpy(end, ".lpc"); }
      traj_format = LPC_TRAJ_FORMAT;
    } break;
    case LPC_TRAJ_FORMAT : {
      fl_set_object_label(fs->appbutt[0], "Change to\nLPC format");
      strcpy(pattern, "*.trj");
      end = strstr(file, ".lpc");
      if (end == NULL) 
  { file[0] = '\0'; }
      else
  { strcpy(end, ".trj"); }
      traj_format = M3D_TRAJ_FORMAT;
    } break;
    }
  fl_set_object_label(fs->patbutt, pattern);
  fl_set_input(fs->input, file);
  fl_refresh_fselector();
}

static const char * select_traj_file(void)
{
  const char * lpc_file = NULL;
  char c_dir_name[200];
  char LPC_file[200];
  int  c_sz=0; 

  fl_check_forms();

  p3d_get_directory(c_dir_name);
  c_sz = (int) strlen(c_dir_name); 
  if(c_dir_name[c_sz-1] != '/'){strcat(c_dir_name,"/");} 
  strcat(c_dir_name,"PATH");

  switch(traj_format) 
    {
    case M3D_TRAJ_FORMAT : {
  /* define filename : current robot, current environment */
      sprintf(LPC_file,"%s_%s_path%i.trj",p3d_get_desc_curname(P3D_ROBOT),
        p3d_get_desc_curname(P3D_ENV),
        MAX(p3d_get_desc_curnum(P3D_TRAJ), 0));
      fl_add_fselector_appbutton("Change to\nLPC format", 
         CB_save_format, NULL);
      lpc_file = fl_show_fselector("P3D_PATH filename",c_dir_name,"*.trj",
           LPC_file);
      fl_remove_fselector_appbutton("Change to\nLPC format");
    } break;
    case LPC_TRAJ_FORMAT : {
  /* define filename : current robot, current environment */
      sprintf(LPC_file,"%s_%s_path%i.lpc",p3d_get_desc_curname(P3D_ROBOT),
        p3d_get_desc_curname(P3D_ENV),
        MAX(p3d_get_desc_curnum(P3D_TRAJ), 0));
      fl_add_fselector_appbutton("Change to\nMove3D format",
         CB_save_format, NULL);
      lpc_file = fl_show_fselector("P3D_PATH filename",c_dir_name,"*.lpc",
           LPC_file);
      fl_remove_fselector_appbutton("Change to\nMove3D format");
    } break;
    }    

  return(lpc_file);
}


/* **************** */
/* WRITE AN LPC FILE */
/* **************** */

static int lpc_start_writepath_fct(const char * file)
{
  if(file_name_exists(file)) {
    PrintError(("writepath LPC format : file %s exists \n",file));
    return(FALSE);
  }
  /* open file in order to write */
  if(!(LPC_fd = fopen(file,"w"))){
    PrintError(("writepath LPC format : can't open %s\n",file));
    return(FALSE);
  }
  lpc_rank_nr = 0.0;
  return(TRUE);
}

static int lpc_stop_writing(void)
{
  fl_check_forms();
  fclose(LPC_fd);
  return(TRUE);
}

static int lpc_write_a_conf_fct(char *mov_obj_name)
{
  p3d_rob *robotPt;
  int i, j;
  double v;
  p3d_jnt * jntPt;

  /* add current configuration to the LPC file */
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  fprintf(LPC_fd,"%f ( %s",lpc_rank_nr,mov_obj_name);
  for(i=0; i<robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
  v = p3d_jnt_get_dof_deg(jntPt, j);
  fprintf(LPC_fd," %f",v);
      }
    }
  }
  fprintf(LPC_fd," ) ");

  return(TRUE);
}

int lpc_writepath_fct(void)
{
  char *mov_obj_name = NULL;

  fl_check_forms();
  /* add current configuration to the LPC file */
  mov_obj_name = p3d_get_robot_name();
  lpc_write_a_conf_fct(mov_obj_name);
  /* to be extended: call function again for all other moving objects */
  /*   ->   lpc_write_a_conf_fct(other_mov_obj_name); */
  /* close the line */
  fprintf(LPC_fd," \n");
  lpc_rank_nr += 1.0;
  return(TRUE);
}


/* **************** */
/* READ AN LPC FILE */
/* **************** */
static int p3d_read_a_config(configPt *q1)
{
  int eof_sign = 9999;
  int q1_ok = TRUE;
  int ndof, i, j, k;
  double v;
  char letter = 'c';
  char mov_obj_name[300];
  p3d_rob * robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt = robotPt->joints[0];

  ndof = robotPt->nb_user_dof;
  eof_sign = fscanf(LPC_fd,"%lf",&v); /* ignore this rank number */
  if(eof_sign == -1){
    q1_ok = FALSE;
  }
  else{
    do{
      letter = getc(LPC_fd);
    }while(letter==' ');
    if(letter!='('){
      printf("read_path LPC format : missing ( \n");
      q1_ok = FALSE;
    }
    else{
      eof_sign = fscanf(LPC_fd,"%s",mov_obj_name);
      if(eof_sign == -1)  
  q1_ok = FALSE;
  else{   
    for(i=0; i<robotPt->njoints; i++) {
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
        k = jntPt->index_dof+j;
        if (p3d_jnt_get_dof_is_user(jntPt, j)) {
    if ((letter!='\n')&&(letter!=')')&&(q1_ok)) {
      eof_sign = fscanf(LPC_fd,"%lf",&v);
      if(eof_sign == -1)
        { q1_ok = FALSE; }
      (*q1)[k]=v;
      letter = getc(LPC_fd);
    } else {
      q1_ok = FALSE;
      PrintError(("read_path LPC format : too few DOF params in file\n"));
    }
        } else
    { (*q1)[i]=p3d_jnt_get_dof_deg(jntPt, j); }
      }
    }
    do {
      letter = getc(LPC_fd);
    } while(letter==' ');
    if(letter!=')') {
      q1_ok = FALSE;
      PrintError(("read_path LPC format : 3. too many DOF params in file\n"));
    }
    do{
      letter = getc(LPC_fd);
    }while(letter==' ');
    if(letter!='\n') {
      q1_ok = FALSE;
      PrintError(("read_path LPC format : we didn't reach eol \n"));
    }
  }
    }
  }
  return q1_ok;
}

/* p3d_read_path precondition: file exists */
void p3d_read_path(const char *file)
{
  int ntrj;
  int q0_ok = TRUE;
  int q1_ok = TRUE;
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  configPt q0, q1;
  p3d_localpath *localPathPt;
  char      str[50],sti[20];
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  
  if(!(LPC_fd = fopen(file,"r"))){
    PrintError(("read_path LPC format : can't open %s\n",file));
  }
  else{
    q0 = p3d_alloc_config(robotPt);
    q0_ok=p3d_read_a_config(&q0);
    p3d_convert_config_deg_to_rad(robotPt, q0, &q0);
    if(q0_ok){
      p3d_copy_config_into(robotPt, q0, &(robotPt->ROBOT_POS));
      /* make a path : */
      q1 = p3d_alloc_config(robotPt);
      /* make a name */
      strcpy(str, "globtrj."); 
      sprintf(sti,"%s",robotPt->name); strcat(str,sti);
      sprintf(sti,"%s","."); strcat(str,sti);
      ntrj = p3d_get_desc_number(P3D_TRAJ)+1;
      sprintf(sti,"%d",ntrj); strcat(str,sti);
      
      q1_ok = p3d_read_a_config(&q1);
      /* begin the description of a new path */
      if(q1_ok){
  p3d_beg_desc(P3D_TRAJ,str);
      }
      /* create the connections */
      while(q1_ok){
  /* translate q1 angles in degree */
  p3d_convert_config_deg_to_rad(robotPt, q1, &q1);
  /* create data structure for local connection */
  localPathPt = p3d_linear_localplanner(robotPt, q0, q1, NULL);
  /* add local connection to path */
  if(localPathPt)
    p3d_add_desc_courbe(localPathPt);
  /* copy q1 into q0 */
  p3d_copy_config_into(robotPt, q1, &q0);
  q1_ok = p3d_read_a_config(&q1);
  /* if q1 is not ok, set q1_ok to FALSE and */
  /* close description of the path */
  if(!q1_ok){
    p3d_copy_config_into(robotPt, q0, &(robotPt->ROBOT_GOTO));
    fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,1); 
    CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,0);
    p3d_end_desc();
    g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    g3d_draw_allwin_active();
  }
      }
      if(q1)
  p3d_destroy_config(robotPt, q1);
    }
    if(q0)
      p3d_destroy_config(robotPt, q0);
  }
}


static void CB_writepath_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  int val =  fl_get_button(ob); 
  const char * file;

  if(val) {
    file = select_traj_file();
    if (file != NULL) {
      printf("Creating a trajectory file!\n");
      switch(traj_format) 
  {
  case M3D_TRAJ_FORMAT : {
    p3d_save_traj(file,(p3d_traj *) p3d_get_desc_curid(P3D_TRAJ));
  } break;
  case LPC_TRAJ_FORMAT : {
    if(lpc_start_writepath_fct(file)) {
      g3d_show_tcur_rob(robotPt,lpc_writepath_fct);
      lpc_stop_writing();
    }
  } break;
  }
      printf("Done\n");
    }
    fl_set_button(ob,0);
  }
}

static void g3d_create_writepath_obj(void)
{
 int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 ROBOTS_FORM[ir].WRITEPATH_OBJ = fl_add_button(FL_PUSH_BUTTON,
            280.0,10.,50.0,20.0,
            "Save traj");
  fl_set_call_back(ROBOTS_FORM[ir].WRITEPATH_OBJ,CB_writepath_obj,0);
}


  
static void CB_loadpath_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  int ir;
  const char * file;
  configPt qi, qf;
  pp3d_traj trajPt;

  file = select_traj_file();
  if (file!=NULL) {
    switch(traj_format) 
      {
      case M3D_TRAJ_FORMAT : {
  if (p3d_read_traj(file)) {
    trajPt = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
    ir = p3d_get_desc_curnum(P3D_ROBOT);
    g3d_add_traj(p3d_get_desc_curname(P3D_TRAJ),
           p3d_get_desc_number(P3D_TRAJ));
    qi = p3d_alloc_config(robotPt);
    qf = p3d_alloc_config(robotPt);
    p3d_ends_and_length_traj(trajPt, &qi, &qf);   
    p3d_copy_config_into(robotPt, qf, &(robotPt->ROBOT_GOTO));
    p3d_copy_config_into(robotPt, qi, &(robotPt->ROBOT_POS));
    fl_set_choice(ROBOTS_FORM[ir].GOTO_OBJ,1); 
    CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ,0);
    g3d_draw_allwin_active();
    p3d_destroy_config(robotPt, qi);
    p3d_destroy_config(robotPt, qf);
  }
      } break;
      case LPC_TRAJ_FORMAT : {
  p3d_read_path(file);
      } break;
      }
    printf("Done\n");
    fl_set_button(ob,0);
  }

  g3d_draw_allwin_active();
  
  MY_ALLOC_INFO("Apres lecture de la trajectoire");

  fl_set_button(ob, 0);
}

static void g3d_create_loadpath_obj(void)
{
  int ir = p3d_get_desc_curnum(P3D_ROBOT); 

  ROBOTS_FORM[ir].LOAD_PATH_OBJ = 
    fl_add_button(FL_PUSH_BUTTON,230.0,10.0,50.0,20.0,"Load traj");
  fl_set_object_callback(ROBOTS_FORM[ir].LOAD_PATH_OBJ,CB_loadpath_obj,0);
} 
/* Fin modification Fabien */

/*************************************************************/

/* creation d'un movie anime a partir de la trajectoire */

static int movie_count = 0;
/* static int movie_count_real = 0; */
static int image_rate;
static int image_compress;

 

void MovieDrawGraph(void) {

  char str[512];
  char file[64];
  if((++movie_count)%image_rate == 0) {
    if(movie_count < 10) sprintf(file,"0000%d.jpg",movie_count);//miff
    else if(movie_count < 100) sprintf(file,"000%d.jpg",movie_count);
    else if(movie_count < 1000) sprintf(file,"00%d.jpg",movie_count);
    else sprintf(file,"0%d.jpg",movie_count);
    sprintf(str,"import -silent -quiet -window %d -quality %d %s/video/%s",g3d_win_id(G3D_WIN),image_compress,getenv("HOME_MOVE3D"),file);//for solaris use : sprintf(str,"/usr/local/imagetools/bin/old/import -silent -window %d -quality %d %s",g3d_win_id(G3D_WIN),image_compress,file);
    system(str);
  }


}

static int movie_drawtraj_fct(void)
{
  char str[512];
  char file[64];

  fl_check_forms();
  if((++movie_count)%image_rate == 0) {
    if(movie_count < 10) sprintf(file,"0000%d.jpg",movie_count);//miff
    else if(movie_count < 100) sprintf(file,"000%d.jpg",movie_count);
    else if(movie_count < 1000) sprintf(file,"00%d.jpg",movie_count);
    else sprintf(file,"0%d.jpg",movie_count);
    sprintf(str,"import -silent -quiet -window %d -quality %d %s/video/%s",g3d_win_id(G3D_WIN),image_compress,getenv("HOME_MOVE3D"),file);//for solaris use : sprintf(str,"/usr/local/imagetools/bin/old/import -silent -window %d -quality %d %s",g3d_win_id(G3D_WIN),image_compress,file);
    system(str);
  }
  return(traj_play);
}

static void g3d_create_mpeg_form(void)
{
  char str[4];
  
  if (MPEG_FORM == NULL) { /* Create one for all the robots */
    MPEG_FORM = fl_bgn_form(FL_UP_BOX,200.0,100.0);

    mpeg_OKbutton_obj = fl_add_button(FL_PUSH_BUTTON,
              150.0,40.0,40.0,20.0,
              "OK");
 
    fl_set_call_back(mpeg_OKbutton_obj,CB_movietraj_obj,0); 
    
    mpeg_Graph_OKbutton_obj = fl_add_button(FL_PUSH_BUTTON,
              150.0,70.0,45.0,20.0,
              "G oK");
    fl_set_call_back(mpeg_Graph_OKbutton_obj,CB_moviegraph_obj,0); 
    



    mpeg_imagerate_obj = fl_add_input(FL_NORMAL_INPUT,
              90.0,40.0,40.0,20.0,
              "One image per");
    image_rate = 1;
    sprintf(str,"%d",image_rate);
    fl_set_input(mpeg_imagerate_obj,str);
    fl_set_call_back(mpeg_imagerate_obj,CB_mpeg_imagerate,0); 
    
    
    fl_add_box(FL_UP_BOX,10.0,10.0,70.0,20.0,"Compression");
    mpeg_imagecompress_obj = fl_add_valslider(FL_HOR_SLIDER,80.0,10.0,
                110.0,20.0,"");

    image_compress = 90;
    fl_set_slider_bounds(mpeg_imagecompress_obj,0.0,100.0);
    fl_set_slider_value(mpeg_imagecompress_obj,(double)image_compress);
    fl_set_slider_precision(mpeg_imagecompress_obj,0);
    fl_set_call_back(mpeg_imagecompress_obj,CB_mpeg_imagecompress,0); 
    
    fl_end_form();
  }
}

static void CB_mpeg_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

  if(val) 
    fl_show_form(MPEG_FORM,FL_PLACE_SIZE,TRUE,"MPEG");
  else
    fl_hide_form(MPEG_FORM);
}

static void CB_mpeg_imagerate(FL_OBJECT *ob, long arg)
{
  int val  = atoi(fl_get_input(ob));
  char tmp_str[20];

  if(val <= 0) {
    printf("\nAt least 1/1 image!\n");
    sprintf(tmp_str, "%i", image_rate);
    fl_set_input(ob, tmp_str);
  }
  else
    image_rate = val;

  printf("MPEG image rate: 1/%d\n",image_rate);
}

static void CB_mpeg_imagecompress(FL_OBJECT *ob, long arg)
{
  int val = fl_get_slider_value(ob);
  image_compress = val;
}

static void CB_moviegraph_obj(FL_OBJECT *ob, long arg)
{
  int val =  fl_get_button(ob);
 char str[512];
  if(!val) {
    IsGraphMovie = FALSE;
    // when push for the second time, compress the jpg files.
    sprintf(str,"cd %s/video;mencoder mf://*.jpg -quiet -ovc xvid -xvidencopts bitrate=800 -oac copy -o m3d.avi",getenv("HOME_MOVE3D"));//change to video directory then compress jpg files to AVI video for more parameters and video format see man pages of mencoder
    system(str);
    sprintf(str,"rm %s/video/*.jpg",getenv("HOME_MOVE3D"));//removing jpg files
    system(str);
    return;
  }
  movie_count = 0;
  IsGraphMovie = TRUE;
}

static void CB_movietraj_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  int val =  fl_get_button(ob); 
  char str[512];
  p3d_localpath *lp=NULL;
  int        nb = 0;

  if(val) {
    fl_set_button(ROBOTS_FORM[robotPt->num].MOVIETRAJ_OBJ,0);
    CB_mpeg_imagerate(mpeg_imagerate_obj,0);
    fl_set_button(mpeg_OKbutton_obj,0);
    fl_hide_form(MPEG_FORM);
  }

  if(!robotPt->tcur){
    return;
  }
  lp = robotPt->tcur->courbePt;
  if(lp == NULL){
    return;
  }
  while (lp) {nb++; lp= lp->next_lp;}
  if(val) {
    traj_play = TRUE;
    sprintf(str, "Create a MPEG movie with %d images (%d curves)!\n",
      g3d_show_tcur_rob(robotPt,default_drawtraj_fct)/image_rate,nb);
    if(!fl_show_question(str,1)) {
      fl_set_button(ob,0);
      return;
    }
    movie_count = 0;
    g3d_show_tcur_rob(robotPt,movie_drawtraj_fct);
    //sprintf(str,"%s/bin/script/mpeg_make *.miff m3d.mpg",getenv("HOME_MOVE3D"));
    sprintf(str,"cd %s/video;mencoder mf://*.jpg -quiet -ovc xvid -xvidencopts bitrate=800 -oac copy -o m3d.avi",getenv("HOME_MOVE3D"));//change to video directory then compress jpg files to AVI video for more parameters and video format see man pages of mencoder
    system(str);
//     sprintf(str,"rm %s/video/*.jpg",getenv("HOME_MOVE3D"));//removing jpg files
    system(str);
    fl_set_button(ob,0);
  }
  else {traj_play = FALSE;}
}

static void g3d_create_movietraj_obj(void)
{
 int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 ROBOTS_FORM[ir].MOVIETRAJ_OBJ = fl_add_button(FL_PUSH_BUTTON,
            340.0,10.,50.0,20.0,
            "MPEG");
 fl_set_call_back(ROBOTS_FORM[ir].MOVIETRAJ_OBJ,CB_mpeg_obj,0);
}

/*****************************************************************/

/* fonctions de destruction des objets forms */
void g3d_delete_robot_form(int ir)
{int nr =  p3d_get_desc_curnum(P3D_ROBOT);

 int nsld=0; 
 FL_OBJECT  *obje;

  extern FL_OBJECT * robotparams_obj;

  if (fl_get_button(robotparams_obj) && (ir == nr))
    {
      while((obje=((FL_OBJECT *)ROBOTS_FORM[ir].POSITION_OBJ[nsld])) == NULL)
       { nsld++; }
      fl_get_winorigin(FL_ObjWin(obje),&x_FR[ir],&y_FR[ir]);
      /*   x_FR[ir] -= 10; */
      /*   y_FR[ir] -= 70; */
      n_f_t[ir] = 1;
      fl_hide_form(ROBOTS_FORM[ir].ROBOT_FORM);
    }

  if(fl_get_button(ROBOTS_FORM[ir].MOVIETRAJ_OBJ)) { 
    fl_set_button(ROBOTS_FORM[ir].MOVIETRAJ_OBJ,0);
    fl_hide_form(MPEG_FORM);
  }    
  g3d_delete_mpeg_form();

  p3d_sel_desc_num(P3D_ROBOT,ir);

  g3d_delete_position_obj();
  g3d_delete_goto_obj(); 
  g3d_delete_trajmove_obj(); 
  g3d_delete_trajnum_obj(); 
  g3d_delete_begtraj_obj(); 
  g3d_delete_addtraj_obj(); 
  g3d_delete_endtraj_obj();
  g3d_delete_showtraj_obj(); 
  g3d_delete_movietraj_obj(); 
  g3d_delete_writepath_obj(); 
  g3d_delete_loadpath_obj(); 

  /* Debut modification Fabien */
  g3d_delete_printconfig_obj();
  g3d_delete_config_obj();
  g3d_delete_set_config_obj();
  g3d_delete_new_config_obj();
  g3d_delete_del_config_obj();
  /* Fin modification Fabien */
  g3d_delete_compute_obj();
  g3d_delete_track_obj();

  g3d_delete_display_filterbox_obj();
  g3d_delete_adapt_filterbox_obj();
  g3d_delete_kine_constraints_obj();

  g3d_delete_radius_obj();
  g3d_delete_dmax_button_obj();
  g3d_delete_dmax_obj();

  fl_free_form(ROBOTS_FORM[ir].ROBOT_FORM);

  p3d_sel_desc_num(P3D_ROBOT,nr);
}


static void g3d_delete_goto_obj(void)
{int       ir;

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  fl_free_object(ROBOTS_FORM[ir].GOTO_OBJ);
}

static void g3d_delete_position_obj(void)
{ 
  int       ndof,i,ir;
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  ndof = robotPt->nb_dof;

  for(i=0;i<ndof;i++) {
    if(ROBOTS_FORM[ir].POSITION_OBJ[i]!=NULL)
      { fl_free_object(ROBOTS_FORM[ir].POSITION_OBJ[i]); }
  }
  p3d_destroy_config(robotPt, last_p_deg[ir]);
}

static void g3d_delete_radius_obj(void)
{
  int ir = p3d_get_desc_curnum(P3D_ROBOT); 

  fl_free_object(ROBOTS_FORM[ir].RADIUS_OBJ);
}

static void g3d_delete_dmax_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT);
 
 fl_free_object(ROBOTS_FORM[ir].DMAX_OBJ);
}

static void g3d_delete_dmax_button_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT);

fl_free_object(ROBOTS_FORM[ir].DMAX_BUTTON_OBJ);
}

static void g3d_delete_trajmove_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT);

  fl_free_object(ROBOTS_FORM[ir].g3d_trajmove_obj[0]);
  fl_free_object(ROBOTS_FORM[ir].g3d_trajmove_obj[1]);
  fl_free_object(ROBOTS_FORM[ir].g3d_trajmove_obj[2]);
  fl_free_object(ROBOTS_FORM[ir].g3d_trajmove_obj[3]);
}

static void g3d_delete_trajnum_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT);

  fl_free_object(ROBOTS_FORM[ir].g3d_trajnum_obj);
}


static void g3d_delete_begtraj_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT);

  fl_free_object(ROBOTS_FORM[ir].BEGTRAJ_OBJ);
}

static void g3d_delete_endtraj_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
 fl_free_object(ROBOTS_FORM[ir].ENDTRAJ_OBJ);
}

static void g3d_delete_addtraj_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
  fl_free_object(ROBOTS_FORM[ir].ADDTRAJ_OBJ);
}

static void g3d_delete_showtraj_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 fl_free_object(ROBOTS_FORM[ir].SHOWTRAJ_OBJ);
}


static void g3d_delete_display_filterbox_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 fl_free_object(ROBOTS_FORM[ir].DISPLAY_FILTERBOX_OBJ);
}

static void g3d_delete_adapt_filterbox_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 fl_free_object(ROBOTS_FORM[ir].ADAPT_FILTERBOX_OBJ);
}

static void g3d_delete_kine_constraints_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 fl_free_object(ROBOTS_FORM[ir].KINE_CONSTRAINTS_OBJ);
}

static void g3d_delete_writepath_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 fl_free_object(ROBOTS_FORM[ir].WRITEPATH_OBJ);
}

static void g3d_delete_loadpath_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

  if ( ROBOTS_FORM[ir].LOAD_PATH_OBJ )
  {
    fl_free_object(ROBOTS_FORM[ir].LOAD_PATH_OBJ);
  }
}

static void g3d_delete_movietraj_obj(void)
{int ir = p3d_get_desc_curnum(P3D_ROBOT); 

 fl_free_object(ROBOTS_FORM[ir].MOVIETRAJ_OBJ);
}

static void g3d_delete_mpeg_form(void)
{
  if (MPEG_FORM != NULL) {
    //fl_hide_form(MPEG_FORM);
    fl_free_object(mpeg_OKbutton_obj);
    fl_free_object(mpeg_imagerate_obj);
    fl_free_object(mpeg_imagecompress_obj);

    fl_free_form(MPEG_FORM);
    MPEG_FORM = NULL;
  }
}



/*************************************************************************/

/* Debut modification Fabien */

/* Met a jour la liste des configurations */
static void FORMrobot_update_config()
{
  int i, nconf, ir;
  pp3d_rob robotPt;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  /* Effacement de la liste */
  nconf = fl_get_choice_maxitems(ROBOTS_FORM[ir].CONFIG_OBJ);
  for(i=nconf; i>0; i--)
    { fl_delete_choice(ROBOTS_FORM[ir].CONFIG_OBJ, i); }

  /* Ajout des configurations */
  for(i=0; i<robotPt->nconf; i++) {
    fl_addto_choice(ROBOTS_FORM[ir].CONFIG_OBJ,robotPt->conf[i]->name);
    if (robotPt->conf[i] == robotPt->confcur)
      { fl_set_choice(ROBOTS_FORM[ir].CONFIG_OBJ,i+1); }
  }

  /* Mise a jour des boutons */
  if(robotPt->nconf>0) {
    fl_activate_object(ROBOTS_FORM[ir].SET_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].SET_CONFIG_OBJ, FL_BLACK);
    fl_activate_object(ROBOTS_FORM[ir].DEL_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].DEL_CONFIG_OBJ, FL_BLACK);
  } else {
    fl_deactivate_object(ROBOTS_FORM[ir].SET_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].SET_CONFIG_OBJ, FL_INACTIVE_COL);
    fl_deactivate_object(ROBOTS_FORM[ir].DEL_CONFIG_OBJ);
    fl_set_object_lcol(ROBOTS_FORM[ir].DEL_CONFIG_OBJ, FL_INACTIVE_COL);
  }
}

static void FORMrobot_update_traj()
{
  int i, ntraj, ir;
  pp3d_rob robotPt;

  ir = p3d_get_desc_curnum(P3D_ROBOT);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  /* Effacement de la liste */
  ntraj = fl_get_choice_maxitems(ROBOTS_FORM[ir].g3d_trajnum_obj);
  for(i=ntraj; i>0; i--)
    { fl_delete_choice(ROBOTS_FORM[ir].g3d_trajnum_obj, i); }

  /* Ajout des trajectoires */
  if (robotPt->nt>0) {
    for(i=0; i<robotPt->nt; i++) {
      p3d_sel_desc_num(P3D_TRAJ, i);
      g3d_add_traj(p3d_get_desc_curname(P3D_TRAJ), i+1);
    }
  }
}

/* Met a jour la fenenre a partir des informations du robot */
void FORMrobot_update(int ir)
{
  double val;
  int rcur,nb_dof,i;
  p3d_rob *robotPt;

  rcur = p3d_get_desc_curnum(P3D_ROBOT);
  p3d_sel_desc_num(P3D_ROBOT, ir);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  /* Mise a jour des sliders de position */
  CB_goto_obj(ROBOTS_FORM[ir].GOTO_OBJ, 0);
  /* Mise a jour des configurations */
  FORMrobot_update_config();
  /* Mise a jour des trajectoires */
  FORMrobot_update_traj();
  /* Mise a jour du slider de radius */
  val = p3d_get_robot_radius();
  g3d_update_radius_obj(ir, val);
  nb_dof = p3d_get_robot_ndof();
  for(i=0;i<nb_dof;i++) {
    if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL) {
      if(robotPt->cntrt_manager->in_cntrt[i] == 2)
  { fl_set_slider_size(ROBOTS_FORM[ir].POSITION_OBJ[i],1.0); }
      else
  { fl_set_slider_size(ROBOTS_FORM[ir].POSITION_OBJ[i],0.1); }
    }
  }
  p3d_sel_desc_num(P3D_ROBOT, rcur);
}



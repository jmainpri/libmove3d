#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "UserAppli-pkg.h"
#include "Bio-pkg.h"
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif
#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#endif
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
#endif
#ifdef QT_LIBRARY
#include "../qtWindow/cppToQt.hpp"
#endif

typedef void (*fct_interface)(void);

typedef struct {
  const char * button_name;
  fct_interface creator, show, hide, destructor;
} fct_option_interface;


G3D_Window *G3D_WIN;
int        G3D_ACTIVE_CC = TRUE;

extern int GOTO_READ;	// KINEO-DEV :doit �re d�lar�dans un .h !!
extern int START_READ;	// KINEO-DEV :doit �re d�lar�dans un .h !!


/* repertoire courant des fichiers de donnees et fichier courant*/
static char    DATA_FILE[200];

// objets correspondant a des boutons
extern     MENU_ROBOT *ROBOTS_FORM;			// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_FORM *ENV_FORM;				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_FORM *USER_APPLI_FORM;			// KINEO-DEV :doit �re d�lar�dans un .h !!
#ifdef ENERGY
extern     FL_FORM *BIO_ENERGY_FORM;
#endif
extern     FL_FORM *PLANNER_FORM;				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_FORM* DIFFUSION_FORM;
extern     FL_OBJECT  *ORIENTED_OBJ;			// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_FORM *STEERING_FORM;				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SEARCH_DMAX_PARAM_OBJ; 	// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SEARCH_DRAW_OPTIM_OBJ;	// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SEARCH_DRAW_OBJ; 		// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SEARCH_PRINT_OBJ;		// KINEO-DEV :doit �re d�lar�dans un .h !!

extern     FL_FORM *BIO_COLLISION_FORM;			// KINEO-DEV :doit �re d�lar�dans un .h !!

extern     FL_OBJECT  *SEARCH_GLOBAL_PARAM_OBJ;	// KINEO-DEV :doit �re d�lar�dans un .h !!

extern     FL_FORM    *STEERING_FORM;			// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  **BUTTON_TAB_OBJ;			// KINEO-DEV :doit �re d�lar�dans un .h !!

extern     FL_OBJECT  *SEARCH_COMPCO_PARAM_OBJ;	// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *STRAT1_OBJ; 				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *STRAT2_OBJ;				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SORT1_OBJ; 				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SORT2_OBJ;				// KINEO-DEV :doit �re d�lar�dans un .h !!
extern     FL_OBJECT  *SEARCH_NBTRY_PARAM_OBJ;	// KINEO-DEV :doit �re d�lar�dans un .h !!

#ifdef GRASP_PLANNING
extern FL_FORM *GRASP_PLANNING_FORM;
#endif
#ifdef HRI_PLANNER
extern FL_FORM  *HRI_PLANNER_FORM;
#endif

#ifdef RRT_ACT
FL_OBJECT *rrt_obj;
extern     FL_FORM *RRT_FORM;		// KINEO-DEV :doit �re d�lar�dans un .h !!
#endif

const fct_option_interface array_option_interface[] = {
#ifdef HRI_PLANNER
  { "HRI Motion Planner",
    g3d_create_hri_planner_form,
    g3d_show_hri_planner_form,
    g3d_hide_hri_planner_form,
    g3d_delete_hri_planner_form,
  },
#endif
#ifdef GRASP_PLANNING
  { "Grasp Planning",
    g3d_create_grasp_planning_form,
    g3d_show_grasp_planning_form,
    g3d_hide_grasp_planning_form,
    g3d_delete_grasp_planning_form,
  },
#endif
};

#define NB_OPTION_INTERFACE (sizeof(array_option_interface) / sizeof(fct_option_interface))

FL_FORM    *MAIN_FORM;

/* variables pour gerer les environnements multiples */
typedef struct scene {
  int saved; /* sauve ou non */
  /* les parametres de la fenetre */
  double x,y,z,az,el,zo,size;
  p3d_vector4  up;
  GLdouble   sx,sy,sz,sel,saz,szo;
  p3d_vector4  sup;
  int FILAIRE,CONTOUR,GOURAUD;
  /* autres parametres */
  double pas_max;
  int local_planner, global_planner, sampling;
  int sorting, node_max, nb_try, node_compco;
  double DMAX,vmin,vmax;
  int DRAW_OPTIM,DRAW_GRAPH;

} p3d_scene;

static p3d_scene saved_scene[P3D_MAX_ENV];

static double h;
static double w;

static double vmin,vmax;

/**********************/
/* MAIN form creation */
/**********************/

static void CB_robotcur_obj(FL_OBJECT *ob, long arg);
static void g3d_create_robotcur_obj(void);
static void CB_robotparams_obj(FL_OBJECT *ob, long arg);
static void g3d_create_robotparams_obj(void);
static void CB_envcur_obj(FL_OBJECT *ob, long arg);
static void g3d_create_envcur_obj(void);
static void CB_envparams_obj(FL_OBJECT *ob, long arg);
static void g3d_create_envparams_obj(void);
static void g3d_create_planner_obj(void);
static void g3d_create_steering_obj(void);
static void CB_load_obj(FL_OBJECT *ob, long arg);
static void g3d_create_load_obj(void);
static void CB_CC_obj(FL_OBJECT *ob, long arg);
static void g3d_create_CC_obj(void);
static void CB_exit_obj(FL_OBJECT *ob, long arg);
static void g3d_create_exit_obj(void);
static void g3d_create_user_obj(void);
static void g3d_create_Diffusion_obj(void);
static void g3d_create_bio_collision_obj(void);
/* Debut modification Fabien */
static void CB_load_scenario_obj(FL_OBJECT * ob, long arg);
static void g3d_create_load_scenario_obj(void);
static void CB_save_scenario_obj(FL_OBJECT * ob, long arg);
static void g3d_create_save_scenario_obj(void);
static void g3d_create_option_interface_obj(void);
/* Fin modification Fabien */

static void g3d_delete_main_form(void);
static void g3d_delete_robotcur_obj(void);
static void g3d_delete_robotparams_obj(void);
static void g3d_delete_envcur_obj(void);
static void g3d_delete_envparams_obj(void);
static void g3d_delete_planner_obj(void);
static void g3d_delete_load_obj(void);
static void g3d_delete_CC_obj(void);
static void g3d_delete_exit_obj(void);
static void g3d_delete_user_obj(void);
static void g3d_delete_Diffusion_obj(void);
static void g3d_delete_steering_obj(void);
static void g3d_delete_bio_collision_obj(void);
/* D�ut modification Fabien */
static void g3d_delete_load_scenario_obj(void);
static void g3d_delete_save_scenario_obj(void);
/* Fin modification Fabien */

static void save_scene(int env_num);
static int scene_is_saved(int num_env);
static void charge_scene(int env_num);
static void del_all_scenes(int nenv);

static int CB_OnApplicationClose( FL_FORM *form, void *argument );
static void g3d_create_robots_forms(int nr);

/*****************************************************************/
void g3d_create_main_form(void)
{
  double x1,x2,y1,y2,z1,z2,ampl=0.;
  unsigned int i;
  int nr;
  pp3d_matrix4 g3d_fct_mobcam_form(void);

  w = G3D_WINSIZE_WIDTH;
  h = G3D_WINSIZE_HEIGHT;

  if(p3d_get_desc_number(P3D_ENV)) {
    p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
    ampl = MAX(MAX(x2-x1,y2-y1),z2-z1);
    x1 = .5*(x1+x2); y1 = .5*(y1+y2); z1 = .5*(z1+z2);
  }

  p3d_get_filename(DATA_FILE);

  for(i=0;i<P3D_MAX_ENV;i++){
    saved_scene[i].saved = FALSE;
    //    XYZ_TAB_GRAPH[i] = NULL; Modification Fabien
  }
  /* on ouvre une fenetre graphique => cree un objet form + un canvas opengl*/
  G3D_WIN = g3d_new_win("Move3D",(int) w, (int) h,ampl);
  fl_set_form_icon((FL_FORM*) G3D_WIN->form, GetApplicationIcon( ), 0);

  g3d_set_win_center(G3D_WIN, x1,y1,z1);
  /* on fixe la fonction de calback pour la fenetre graphique */

#ifndef QT_GL
  g3d_set_win_drawer(G3D_WIN, g3d_draw);
#else
  g3d_set_win_drawer(G3D_WIN, NULL);
#endif
  /* on fixe la fonction de calback pour la camera movile */
  g3d_set_win_fct_mobcam(G3D_WIN, g3d_fct_mobcam_form);
  /* Definition de la forme principale */
  MAIN_FORM = fl_bgn_form(FL_UP_BOX,250,260+50*NB_OPTION_INTERFACE);
  g3d_create_envparams_obj(); /* cree le bouton du menu environnement Env */
  g3d_create_envcur_obj(); /* cree le menu deroulant a cote de Env */

  g3d_create_robotparams_obj();/* cree le bouton du menu robot Robot */
  g3d_create_robotcur_obj();/* cree le menu deroulant a cote de Robot */

  g3d_create_load_obj();/* cree le bouton du menu LoadEnv */
  g3d_create_CC_obj();/* cree le bouton du menu Desactivate CC */
  g3d_create_planner_obj();/* cree le bouton du menu  */
  g3d_create_steering_obj(); /* cree le bouton Steering methods */
  g3d_create_Diffusion_obj();/* cree le bouton Delete env*/
  g3d_create_exit_obj();/* cree le bouton Exit */
  g3d_create_user_obj();/* cree le bouton pour les applications d'un user */
  g3d_create_bio_collision_obj();
  /* D�ut modification Fabien */
  g3d_create_load_scenario_obj(); // cree le bouton de chargement d'un scenario
  g3d_create_save_scenario_obj(); // cree le bouton de sauvegarde d'un scenario
  g3d_create_option_interface_obj(); // create the interface option
  /* Fin modification Fabien */
  fl_end_form();

  /* Creation des autres formes */
  g3d_create_env_form();
  // comment user appli
 g3d_create_user_appli_form();
#ifdef ENERGY
  g3d_create_bio_energy_form();
#endif
#ifdef RRT_ACT
  g3d_create_rrt_form();
#endif
  g3d_create_planner_form();
  g3d_create_diffusion_form();
  g3d_create_steering_form();
#ifdef BIO
  g3d_create_bio_collision_form();
#endif

#ifdef QT_LIBRARY
  // This is a pipe to use qt on XForm (X) objects
  fl_add_io_callback(qt_fl_pipe[0], FL_READ, read_pipe, NULL);
#endif

 /* Option interface */
  for(i=0; i<NB_OPTION_INTERFACE; i++) {
    if (array_option_interface[i].creator != NULL)
      { (*array_option_interface[i].creator)(); }
  }
  fl_set_form_icon(MAIN_FORM, GetApplicationIcon( ), 0);

  /* Affichage de la forme principale */
  fl_show_form(MAIN_FORM,FL_PLACE_SIZE,TRUE,"Move3D");

  /* pour plus de confort on remplit deja les positions POS/GOTO des robots */
  /* (sert si par exemple on a plusieurs robots mais qu on ne veut planifier */
  /* de chemin que pour un seul...) et on alloue tous les menus robot */
  nr = p3d_get_desc_number(P3D_ROBOT);
  g3d_create_robots_forms(nr);

  //Set a handler for the "ApplicationClose" event
  fl_set_atclose(CB_OnApplicationClose, 0);
}

void g3d_loop(void)
{
  while(1)
    { fl_do_forms(); }
}

/**********************************************************************/

/* choix du robot courant */
static FL_OBJECT  *robotcur_obj;
FL_OBJECT  *robotparams_obj;

static void CB_robotcur_obj(FL_OBJECT *ob, long arg)
{int val = fl_get_choice(ob);
 int rcur= p3d_get_desc_curnum(P3D_ROBOT);
 float x,y;
 p3d_localplanner_type lpl_type;
 pp3d_rob robotPt;

  p3d_sel_desc_num(P3D_ROBOT,val-1);
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  printf("ancien dmax : %f nouveau dmax robot %s : %f\n", XYZ_ENV->dmax,
	 robotPt->name, sqrt(SQR(robotPt->BB.xmin-robotPt->BB.xmax) +
			     SQR(robotPt->BB.ymin-robotPt->BB.ymax) +
			     SQR(robotPt->BB.zmin-robotPt->BB.zmax))/10.);

  if(val != 0) {
    if(rcur != (val-1)) {
      if(fl_get_button(robotparams_obj)) fl_hide_form(ROBOTS_FORM[rcur].ROBOT_FORM);
      x = ROBOTS_FORM[rcur].ROBOT_FORM->x;
      y = ROBOTS_FORM[rcur].ROBOT_FORM->y;
      if(fl_get_button(robotparams_obj)) {
	fl_set_form_position(ROBOTS_FORM[val-1].ROBOT_FORM, (FL_COORD) x, (FL_COORD) y);
	fl_set_form_icon(ROBOTS_FORM[val-1].ROBOT_FORM, GetApplicationIcon( ), 0);
	fl_show_form(ROBOTS_FORM[val-1].ROBOT_FORM,FL_PLACE_POSITION,TRUE,
		     p3d_get_desc_curname(P3D_ROBOT));
      }

      fl_freeze_form(PLANNER_FORM);
      fl_set_button(ORIENTED_OBJ, p3d_get_ORIENTED());
      fl_unfreeze_form(PLANNER_FORM);
      fl_freeze_form(STEERING_FORM);
      lpl_type = p3d_local_get_planner();
      fl_set_button(BUTTON_TAB_OBJ[lpl_type],1);
      fl_unfreeze_form(STEERING_FORM);

      /*  printf("Planificateur courant: %s\n",
	  p3d_local_getname_planner(p3d_local_get_planner())); */
      g3d_draw_allwin_active(); // Modification Fabien
    }
  }
}

static void g3d_create_robotcur_obj(void)
{void CB_robot_obj(FL_OBJECT *ob, long arg);
 int  i,nrob,rcur;

 robotcur_obj = fl_add_choice(FL_NORMAL_CHOICE,100,210,140,40,""); // Modification Fabien
  nrob = p3d_get_desc_number(P3D_ROBOT);
  rcur = p3d_get_desc_curnum(P3D_ROBOT);

  for(i=0;i<nrob;i++) {
    p3d_sel_desc_num(P3D_ROBOT,i);
    fl_addto_choice(robotcur_obj,p3d_get_desc_curname(P3D_ROBOT));
  }
  p3d_sel_desc_num(P3D_ROBOT,rcur);
  fl_set_choice(robotcur_obj,rcur+1);
  fl_set_call_back(robotcur_obj,CB_robotcur_obj,0);

}

/* affiche le menu position du robot courant */
static void CB_robotparams_obj(FL_OBJECT *ob, long arg)
{int val = fl_get_button(ob);
 int ir = p3d_get_desc_curnum(P3D_ROBOT);

  fl_set_form_icon(ROBOTS_FORM[ir].ROBOT_FORM, GetApplicationIcon( ), 0);
  if(val) fl_show_form(ROBOTS_FORM[ir].ROBOT_FORM,FL_PLACE_SIZE,TRUE,
    p3d_get_desc_curname(P3D_ROBOT));
  else    fl_hide_form(ROBOTS_FORM[ir].ROBOT_FORM);
}

static void g3d_create_robotparams_obj(void)
{void CB_robot_obj(FL_OBJECT *ob, long arg);

  robotparams_obj = fl_add_button(FL_PUSH_BUTTON,
				  10,210,90,40, "Robot"); // Modification Fabien
  fl_set_call_back(robotparams_obj,CB_robotparams_obj,0);
}


/**********************************************************************/

/*choix de l'environnement courant */
static FL_OBJECT  *envcur_obj;
FL_OBJECT  *envparams_obj;

static void CB_envcur_obj(FL_OBJECT *ob, long arg)
{int   val = fl_get_choice(ob),env_num;
 int  nr,ir,r;
 double x,y;
 p3d_rob *rob;
 int p3d_numcoll;

  if(val != 0) {
      if(fl_get_button(envparams_obj)) fl_hide_form(ENV_FORM);
      x = ENV_FORM->x;
      y = ENV_FORM->y;
      fl_free_form(ENV_FORM);
      g3d_create_env_form();
      if(fl_get_button(envparams_obj)) {
	fl_set_form_position(ENV_FORM, (FL_COORD) x, (FL_COORD) y);
	fl_set_form_icon(ENV_FORM, GetApplicationIcon( ), 0);
	fl_show_form(ENV_FORM,FL_PLACE_POSITION,TRUE,
		     p3d_get_desc_curname(P3D_ENV));
      }

      /* en fait on doit reprendre la fonction load_env.... */
      fl_freeze_form(MAIN_FORM);

      env_num = p3d_get_desc_curnum(P3D_ENV);
      /* on stocke l'environnement courant a tous le coups (changements de point de vue...)*/
      save_scene(env_num+1);

      /* on detruit les menus robot */
      nr = p3d_get_desc_number(P3D_ROBOT);
      ir = p3d_get_desc_curnum(P3D_ROBOT);
      if(fl_get_button(robotparams_obj)){fl_hide_form(ROBOTS_FORM[ir].ROBOT_FORM);fl_set_button(robotparams_obj,0);}
      for(ir=0;ir<nr;ir++){
	g3d_delete_robot_form(ir);
	//g3d_delete_filter_form(ir);
      }
      free(ROBOTS_FORM);
      //free(FILTER_FORM);
      /* on les enleve du menu choix des robots */
      for(ir=0;ir<nr;ir++) {
	p3d_sel_desc_num(P3D_ROBOT,ir);
	fl_delete_choice(robotcur_obj,fl_get_choice(robotcur_obj));
      }

      /* detruction of collision detector data */
      p3d_col_stop_all();

      /* on reinitialise la fenetre */
      p3d_col_init_coll();
      p3d_BB_set_mode_close();

      /* on charge le nouveau */
      charge_scene(val);
      /* construction of data for default collision detector */
      p3d_col_start_last();

      /* on recree les menus robot */
      nr = p3d_get_desc_number(P3D_ROBOT);
      r = p3d_get_desc_curnum(P3D_ROBOT);
      g3d_create_robots_forms(nr);

      for(ir=0;ir<nr;ir++) {
	p3d_sel_desc_num(P3D_ROBOT,ir);
	fl_addto_choice(robotcur_obj,p3d_get_desc_curname(P3D_ROBOT));
      }
      p3d_sel_desc_num(P3D_ROBOT,r);
      fl_set_choice(robotcur_obj,r+1);

      if(fl_get_button(robotparams_obj))
	{
		fl_set_form_icon(ROBOTS_FORM[r].ROBOT_FORM, GetApplicationIcon( ), 0);
		fl_show_form(ROBOTS_FORM[r].ROBOT_FORM,FL_PLACE_SIZE,TRUE,
						   p3d_get_desc_curname(P3D_ROBOT));
	}

      rob = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

      /* on actualise le menu planificateur */
      fl_freeze_form(PLANNER_FORM);

      fl_set_slider_bounds(SEARCH_DMAX_PARAM_OBJ,vmin,vmax);
      fl_set_slider_value(SEARCH_DMAX_PARAM_OBJ,p3d_get_DMAX());

      fl_set_slider_value(SEARCH_GLOBAL_PARAM_OBJ,p3d_get_NB_NODES());
      fl_set_slider_value(SEARCH_COMPCO_PARAM_OBJ,p3d_get_COMP_NODES());
      fl_set_slider_value(SEARCH_NBTRY_PARAM_OBJ,ENV.getInt(Env::NbTry));

      fl_set_button(SEARCH_DRAW_OPTIM_OBJ,ENV.getBool(Env::drawTraj));
      fl_set_button(SEARCH_DRAW_OBJ,ENV.getBool(Env::drawGraph));


      /* Carl 2/1/2001: */
      /* we re-initialise the slider for volume */
      g3d_reinit_tolerance_slider();
      /* we re-initialise the slider for tolerance */
      g3d_reinit_volume_slider();
      /* : Carl 2/1/2001 */
      /* modif Pepijn june 2001 */
      g3d_reinit_dmax_slider();


      fl_freeze_form(STEERING_FORM);
      fl_set_button(BUTTON_TAB_OBJ[p3d_local_get_planner()],1);
      fl_unfreeze_form(STEERING_FORM);

      switch(p3d_get_MOTION_PLANNER()){
       case P3D_BASIC:
	 fl_set_button(STRAT1_OBJ,1);
	 break;
       case P3D_ISOLATE_LINKING:
	 fl_set_button(STRAT2_OBJ,1);
	 break;
       default:
	 printf("CB_envcur_obj : no global planner set...\n");
	 break;
      }

      //  if(p3d_get_SAMPLING_CHOICE()==P3D_GAUSSIAN_SAMPLING){fl_set_button(SAMPLE_GAUSSIAN_OBJ,1);}

      switch(p3d_get_SORTING()){
	  case P3D_NB_CONNECT:
	    fl_set_button(SORT1_OBJ,1);
	    break;
	  case P3D_DIST_NODE:
	    fl_set_button(SORT2_OBJ,1);
	    break;
	  default:
	   printf("CB_envcur_obj : no sorting strategy set...\n");
	 break;
      }

      fl_unfreeze_form(PLANNER_FORM);

      /* on teste la position courante */
      if(G3D_ACTIVE_CC){
	p3d_numcoll = p3d_col_test_all();
      }

      MY_ALLOC_INFO("P3D_TOTAL_SIZE");


      fl_set_choice(envcur_obj,val);

      /* on reinitialise certaines variables globales */
      G3D_ACTIVE_CC = 1;

      g3d_reinit_graphics();

      fl_unfreeze_form(MAIN_FORM);
      g3d_refresh_allwin_active();
      g3d_draw_allwin_active();
  }

}

static void g3d_create_envcur_obj(void)
{void CB_env_obj(FL_OBJECT *ob, long arg);
 int  i,nenv,ecur;
 char *name;

  envcur_obj = fl_add_choice(FL_NORMAL_CHOICE,100,160,140,40,""); // Modification Fabien
  nenv = p3d_get_desc_number(P3D_ENV);
  ecur = p3d_get_desc_curnum(P3D_ENV);
  for(i=0;i<nenv;i++) {
    p3d_sel_desc_num(P3D_ENV,i);
    name = p3d_get_desc_curname(P3D_ENV);
    fl_addto_choice(envcur_obj,name);
  }

  p3d_sel_desc_num(P3D_ENV,ecur);
  fl_set_choice(envcur_obj,ecur+1);
  fl_set_call_back(envcur_obj,CB_envcur_obj,0);
}

/* on affiche le menu env */
static void CB_envparams_obj(FL_OBJECT *ob, long arg)
{int val = fl_get_button(ob);

fl_set_form_icon(ENV_FORM, GetApplicationIcon( ), 0);
  if(val) fl_show_form(ENV_FORM,FL_PLACE_SIZE,TRUE,
		       p3d_get_desc_curname(P3D_ENV));
  else    fl_hide_form(ENV_FORM);
}

static void g3d_create_envparams_obj(void)
{void CB_env_obj(FL_OBJECT *ob, long arg);

  envparams_obj = fl_add_button(FL_PUSH_BUTTON,
				  10,10,70,40, "Collision \nChecker");
  fl_set_call_back(envparams_obj,CB_envparams_obj,0);
}

/**********************************************************************/

/* on affiche le menu rrt */
#ifdef RRT_ACT

static void CB_rrt_obj(FL_OBJECT *ob, long arg)
{
  int val =  fl_get_button(ob);
fl_set_form_icon(RRT_FORM, GetApplicationIcon( ), 0);
  if(val) fl_show_form(RRT_FORM,FL_PLACE_SIZE,TRUE,
		       "RRT Planner");
  else    fl_hide_form(RRT_FORM);
}

static void g3d_create_rrt_obj(void)
{
  rrt_obj = fl_add_button(FL_PUSH_BUTTON,
			      90,30,70,20,
			      "RRT planner");
  fl_set_call_back(rrt_obj,CB_rrt_obj,0);
}
#endif

/**********************************************************************/

/* on affiche le menu planner */
FL_OBJECT *planner_obj;

static void CB_planner_obj(FL_OBJECT *ob, long arg)
{
  int val =  fl_get_button(ob);
fl_set_form_icon(PLANNER_FORM, GetApplicationIcon( ), 0);
  if(val) fl_show_form(PLANNER_FORM,FL_PLACE_SIZE,TRUE,
		       "Motion Planner");
  else    fl_hide_form(PLANNER_FORM);
}

static void g3d_create_planner_obj(void)
{
#ifdef RRT_ACT
  g3d_create_rrt_obj();
  planner_obj = fl_add_button(FL_PUSH_BUTTON,
			      90,10,70,20,
			      "Planner");
#else
  planner_obj = fl_add_button(FL_PUSH_BUTTON,
			      90,10,70,40,
			      "Planner");
#endif
  fl_set_call_back(planner_obj,CB_planner_obj,0);
}

/**********************************************************************/

/* on affiche le menu steering methods */
FL_OBJECT *steering_obj;

static void CB_steering_obj(FL_OBJECT *ob, long arg)
{
  int val =  fl_get_button(ob);
fl_set_form_icon(STEERING_FORM, GetApplicationIcon( ), 0);
  if(val) fl_show_form(STEERING_FORM,FL_PLACE_SIZE,TRUE,
		       "Steering Methods");
  else    fl_hide_form(STEERING_FORM);
}

static void g3d_create_steering_obj(void)
{
 steering_obj = fl_add_button(FL_PUSH_BUTTON,
			      170,10,70,40,
			      "Steering\nmethods");
  fl_set_call_back(steering_obj,CB_steering_obj,0);
}

/**********************************************************************/

/* charge un nouvel environnement */
static FL_OBJECT  *load_obj;

static void CB_load_obj(FL_OBJECT *ob, long arg)
{const char *file=NULL;
 int  nr,ir,r,env_num;
 double x1,x2,y1,y2,z1,z2,ampl=0.;
 p3d_rob *rob;
 int p3d_numcoll;
 char c_dir_name[200]; // Modification Fabien

 if(XYZ_NUM_ENV == P3D_MAX_ENV-1){printf("ERROR : TOO MANY ENV DEFINED...\n"); return;}

 p3d_get_directory(c_dir_name); // Modification Fabien
 file = fl_show_fselector("P3D_ENV filename",c_dir_name, "*.p3d","");

 if(file){

   p3d_rw_scenario_delete_name(); // Modification Fabien
   fl_freeze_form(MAIN_FORM);

   /* on stocke l'environnement courant */
   env_num = p3d_get_desc_curnum(P3D_ENV);
   save_scene(env_num+1);
   XYZ_GRAPH = NULL;

   /* on detruit les menus robot */
   nr = p3d_get_desc_number(P3D_ROBOT);
   ir = p3d_get_desc_curnum(P3D_ROBOT);
   /* si le menu robot est visible, on le cache */
   if(fl_get_button(robotparams_obj)){fl_hide_form(ROBOTS_FORM[ir].ROBOT_FORM);}
   /*   on detruit tous les menus */
   for(ir=0;ir<nr;ir++){
     p3d_sel_desc_num(P3D_ROBOT,ir);
     g3d_delete_robot_form(ir);
     //g3d_delete_filter_form(ir);
     /* et on enleve les robots du menu choix des robots */
     fl_delete_choice(robotcur_obj,fl_get_choice(robotcur_obj));
   }
   free(ROBOTS_FORM);

   /* on libere le menu filtre */
   //free(FILTER_FORM);

   /* detruction of collision detector data */
   p3d_col_stop_all();

   /* on charge le nouvel environnement */
   p3d_read_desc((char *)file);
   printf("Loading done...\n");

   /* on re-initialise le detecteur de collision */
   p3d_col_init_coll();
   p3d_BB_set_mode_close();

   /* construction of data for default collision detector */
   p3d_col_start_last();

   /* on met a jour le data_file */
   strcpy(DATA_FILE,file);

   /* on recree les menus robot */
   nr = p3d_get_desc_number(P3D_ROBOT);
   r = p3d_get_desc_curnum(P3D_ROBOT);
   g3d_create_robots_forms(nr);

   /* on remet les robots dans le choix des robots courants */
   for(ir=0;ir<nr;ir++) {
     p3d_sel_desc_num(P3D_ROBOT,ir);
     fl_addto_choice(robotcur_obj,p3d_get_desc_curname(P3D_ROBOT));
   }
   p3d_sel_desc_num(P3D_ROBOT,r);
   fl_set_choice(robotcur_obj,r+1);

   if(fl_get_button(robotparams_obj))
	{
		fl_set_form_icon(ROBOTS_FORM[r].ROBOT_FORM, GetApplicationIcon( ), 0);
		fl_show_form(ROBOTS_FORM[r].ROBOT_FORM,FL_PLACE_SIZE,TRUE,
						   p3d_get_desc_curname(P3D_ROBOT));
	}
   rob = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

   /* on actualise le menu planificateur (distance max de l'environnement) */
   fl_freeze_form(PLANNER_FORM);

   p3d_set_DMAX(p3d_calc_DMAX(rob));
   vmax=p3d_get_DMAX()*5;
   vmin=p3d_get_DMAX()/1000;
   fl_set_slider_bounds(SEARCH_DMAX_PARAM_OBJ,vmin,vmax);
   fl_set_slider_value(SEARCH_DMAX_PARAM_OBJ,p3d_get_DMAX());

   fl_set_slider_value(SEARCH_GLOBAL_PARAM_OBJ,200);
   fl_set_slider_value(SEARCH_COMPCO_PARAM_OBJ,1000);
   fl_set_slider_value(SEARCH_NBTRY_PARAM_OBJ,1000);

   fl_set_button(SEARCH_DRAW_OPTIM_OBJ,0);
   fl_set_button(SEARCH_DRAW_OBJ,0);

   /* Carl 2/1/2001: */
   /* we re-initialise the slider for volume */
   g3d_reinit_tolerance_slider();
   /* we re-initialise the slider for tolerance */
   g3d_reinit_volume_slider();
   /* : Carl 2/1/2001 */
   /* modif Pepijn june 2001 */
   g3d_reinit_dmax_slider();

   /* ICI ON DEVRAIT AVOIR LE PLANIFICATEUR LOCAL !!! */

   p3d_set_MOTION_PLANNER(P3D_BASIC);
   fl_set_button(STRAT1_OBJ,1);

   //p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
   //fl_set_button(SAMPLE_GAUSSIAN_OBJ,0);

   p3d_set_SORTING(P3D_NB_CONNECT);
   fl_set_button(SORT1_OBJ,1);

   fl_unfreeze_form(PLANNER_FORM);

   fl_freeze_form(STEERING_FORM);
   fl_set_button(BUTTON_TAB_OBJ[p3d_local_get_planner()],1);
   fl_unfreeze_form(STEERING_FORM);

   /* on teste les positions courantes */
   if(G3D_ACTIVE_CC){
     p3d_numcoll = p3d_col_test_all();
   }

   MY_ALLOC_INFO("P3D_TOTAL_SIZE");

   /* on remet l'environnement courant dans le menu */
   p3d_sel_desc_num(P3D_ENV,XYZ_NUM_ENV-1);
   fl_addto_choice(envcur_obj,p3d_get_desc_curname(P3D_ENV));

   fl_set_choice(envcur_obj,XYZ_NUM_ENV);

   /* on reinitialise certaines variables globales */
   G3D_ACTIVE_CC = 1;

   h = G3D_WINSIZE;
   w = G3D_WINSIZE;
   if(p3d_get_desc_number(P3D_ENV)) {
     p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
     ampl = MAX(MAX(x2-x1,y2-y1),z2-z1);
   }
   g3d_resize_allwin_active(w,h,ampl);

   ENV.setBool(Env::drawTraj,false);
   fl_set_button(SEARCH_DRAW_OPTIM_OBJ,0);

   ENV.setBool(ENV.drawGraph,false);
   fl_set_button(SEARCH_DRAW_OBJ,0);

   fl_unfreeze_form(MAIN_FORM);
   fl_set_button(load_obj,0);

   g3d_refresh_allwin_active();
   g3d_draw_allwin_active();
 }
 else{
   /* on remet le bouton load_env a 0 */
   fl_set_button(load_obj,0);
   g3d_refresh_allwin_active();
   g3d_draw_allwin_active();
   return;
 }

}

static
void g3d_create_load_obj(void)
{
  load_obj = fl_add_button(FL_NORMAL_BUTTON,
			   10,60,70,40,
			   "Load Env");

  fl_set_call_back(load_obj,CB_load_obj,0);
}

/*******************************************************/

/* on desactive/active le collision checker */

static FL_OBJECT  *CC_obj;

static void  CB_CC_obj(FL_OBJECT *ob, long arg)
{

  if(G3D_ACTIVE_CC){G3D_ACTIVE_CC = 0;}
  else{G3D_ACTIVE_CC = 1;}

  g3d_draw_allwin_active();
}

static
void g3d_create_CC_obj(void)
{
  CC_obj = fl_add_button(FL_PUSH_BUTTON,
			 10,160,90,40, // Modification Fabien
			   "Deactivate\nCC");

  fl_set_call_back(CC_obj,CB_CC_obj,0);
}


/**********************************************************************/

/* on sort du programme */
static FL_OBJECT  *exit_obj;
       FL_OBJECT  *user_obj;
/* FL_OBJECT  *userparams_obj; */

static void CB_user_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

  FL_FORM *XXX_FORM;

#ifdef ENERGY
  XXX_FORM = BIO_ENERGY_FORM;
#else
  XXX_FORM = USER_APPLI_FORM;
#endif

#ifdef ANIMATION
  anim_interface();
#else
	if(val)
	{
		fl_set_form_icon(XXX_FORM, GetApplicationIcon( ), 0);
		fl_show_form(XXX_FORM,FL_PLACE_SIZE,TRUE,
			p3d_get_desc_curname(P3D_ENV));
	}
  else
    fl_hide_form(XXX_FORM);
#endif
}

static void CB_exit_obj(FL_OBJECT *ob, long arg)
{int i,env_number;

 //desalloc array for the multisolution constraint. Allocation in the main function(file move3d.c)
 p3d_destroy_iksol(XYZ_ROBOT->cntrt_manager);

 g3d_delete_main_form();

 p3d_rw_scenario_delete_name();           // Modification Fabien
 p3d_col_env_free_memory_traj_col_tab();  // Modification Fabien
 p3d_autocol_destroy_datas();

 /* destruction of data for collision detectors */
 p3d_col_stop_all();

 env_number = XYZ_NUM_ENV;

 /* on detruit les scenes stockees */
 del_all_scenes(env_number);

 /* on detruit les environnements courants */
 for(i=0;i<env_number;i++){
   p3d_sel_desc_num(P3D_ENV,0);
   p3d_del_desc(P3D_ENV);
 }
 p3d_BB_clear();
 if (XYZ_ENV != NULL) {
   MY_FREE(XYZ_ENV, p3d_env, XYZ_MAX_NUM_ENV);
   XYZ_ENV = NULL;
 }
 if (XYZ_TAB_ENV != NULL)
   { MY_FREE(XYZ_TAB_ENV, pp3d_env, XYZ_MAX_NUM_ENV); }

 MY_ALLOC_INFO("After p3d_del_desc");
 basic_alloc_debugoff();

 exit(0);
}

static void g3d_create_user_obj(void)
{
#ifdef ENERGY
  user_obj = fl_add_button(FL_PUSH_BUTTON,
			   170,110,35,40,
			   "Bio-E");
#else
  user_obj = fl_add_button(FL_PUSH_BUTTON,
			   170,110,35,40, // Modification Fabien
			   "User\nAppli.");
#endif

  fl_set_call_back(user_obj,CB_user_obj,0);
}

static void g3d_create_exit_obj(void)
{
  exit_obj = fl_add_button(FL_TOUCH_BUTTON,
			   170,60,70,40,
			   "Exit");
  fl_set_call_back(exit_obj,CB_exit_obj,0);
}


/**********************************************************************/
/* on sort du programme */
FL_OBJECT  *bio_collision_obj;

static void CB_bio_collision_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

	if(val)
	{
		fl_set_form_icon(BIO_COLLISION_FORM, GetApplicationIcon( ), 0);
		fl_show_form(BIO_COLLISION_FORM,FL_PLACE_SIZE,TRUE,
			p3d_get_desc_curname(P3D_ENV));
	}
  else
    fl_hide_form(BIO_COLLISION_FORM);
}


static void g3d_create_bio_collision_obj(void)
{
  bio_collision_obj = fl_add_button(FL_PUSH_BUTTON,
				    205,110,35,40,
				    "BIO");
  fl_set_call_back(bio_collision_obj,CB_bio_collision_obj,0);
}

/**********************************************************************/

/* Create the diffusion window */
FL_OBJECT *Diffusion_obj;

static void CB_Diffusion_obj(FL_OBJECT *ob, long arg) {
  int val =  fl_get_button(ob);
  fl_set_form_icon(DIFFUSION_FORM, GetApplicationIcon(), 0);
  if(val) fl_show_form(DIFFUSION_FORM,FL_PLACE_SIZE,TRUE,
		       "Diffusion Planner");
  else   fl_hide_form(DIFFUSION_FORM);
}

static void g3d_create_Diffusion_obj(void)
{
  Diffusion_obj = fl_add_button(FL_PUSH_BUTTON,
			  90,60,70,40,
			  "Diffusion");
  fl_set_call_back(Diffusion_obj,CB_Diffusion_obj,0);
}

/**********************************************************************/
/* D�ut modification Fabien */
/* charge un sc�ario */
static FL_OBJECT  *load_scenario_obj;

void read_scenario_by_name(const char *file)
{
  int  nrob, ir;
  //  pp3d_rob robotPt;
  p3d_localplanner_type lpl_type;
  /* on lit le scenario */
  if (ROBOTS_FORM != NULL) { // if no form exist, we cannot update it
    if (file){
      //  		rcur = p3d_get_desc_curnum(P3D_ROBOT);
      //  		robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
      if (p3d_read_scenario(file)){ // returns false on file no found
        fl_deactivate_all_forms();
        /* on met �jour les fen�res */
        nrob = p3d_get_desc_number(P3D_ROBOT);
        for(ir=0; ir<nrob; ir++) {
          FORMrobot_update(ir);
        }

        lpl_type = p3d_local_get_planner();
        fl_set_button(BUTTON_TAB_OBJ[lpl_type],1);
        fl_activate_all_forms();
        g3d_draw_allwin_active();
      }
    }
  } else{
    PrintError(("Cannot load scenario before Form was created.\n"));
  }
}

static void CB_load_scenario_obj(FL_OBJECT *ob, long arg)
{
  const char *file;
  /* nom par d�aut */
  p3d_rw_scenario_init_name();

  file = fl_show_fselector("Scenario filename", p3d_rw_scenario_get_path(),
			   "*.sce", p3d_rw_scenario_get_name());
  read_scenario_by_name(file);
  fl_set_button(load_scenario_obj,0);
}

static void g3d_create_load_scenario_obj(void)
{
  load_scenario_obj = fl_add_button(FL_PUSH_BUTTON, 10,110,70,40,
				    "Load Scenario");
  fl_set_call_back(load_scenario_obj,CB_load_scenario_obj,0);
}

static void g3d_delete_load_scenario_obj(void)
{ fl_free_object(load_scenario_obj); }


/**********************************************************************/
/* sauve un scenario */
static FL_OBJECT  *save_scenario_obj;

static void CB_save_scenario_obj(FL_OBJECT *ob, long arg)
{
  const char *file;
  int ir;

  /* nom par d�aut */
  p3d_rw_scenario_init_name();
  ir = p3d_get_desc_curnum(P3D_ROBOT);
  file = fl_show_fselector("Scenario filename", p3d_rw_scenario_get_path(),
			   "*.sce", p3d_rw_scenario_get_name());
  if(file){
    /* on sauve le sc�ario */
    p3d_save_scenario(file);
  }
  fl_set_button(save_scenario_obj,0);
}

static void g3d_create_save_scenario_obj(void)
{
  save_scenario_obj = fl_add_button(FL_PUSH_BUTTON, 90,110,70,40,
				    "Save Scenario");
  fl_set_call_back(save_scenario_obj,CB_save_scenario_obj,0);
}

static void g3d_delete_save_scenario_obj(void)
{ fl_free_object(save_scenario_obj); }

/* Fin modification Fabien */


/**********************************************************************/
/* Show the optional windows interfaces */

FL_OBJECT  * option_interface_obj[NB_OPTION_INTERFACE];

/*--------------------------------------------------------------------------*/
/*! \brief Create the button to call the window
 *  \internal
 */
static void CB_option_interface_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(ob)) {
    if ((arg>=0) && ( ((unsigned int) arg) < NB_OPTION_INTERFACE ) &&
	(array_option_interface[arg].show != NULL))
      { (*array_option_interface[arg].show)(); }
  } else {
    if ((arg>=0) && ( ((unsigned int) arg) < NB_OPTION_INTERFACE ) &&
	(array_option_interface[arg].hide != NULL))
      { (*array_option_interface[arg].hide)(); }
  }
}

/*--------------------------------------------------------------------------*/
/*! \brief Create the button to call all the windows
 *  \internal
 */
static void g3d_create_option_interface_obj(void)
{
  unsigned int i;

  for(i=0; i<NB_OPTION_INTERFACE; i++) {
    option_interface_obj[i] =
      fl_add_button(FL_PUSH_BUTTON, 10,260+50*i,230,40,
		    array_option_interface[i].button_name);
    fl_set_call_back(option_interface_obj[i],CB_option_interface_obj,i);
  }
}

/*****************************************************************/

/* fonctions de desallocation des objets forms */
static
void g3d_delete_main_form(void)
{int nr,ir;

  g3d_del_win(G3D_WIN);

  fl_hide_form(MAIN_FORM);

  g3d_delete_env_form();
  g3d_delete_user_appli_form();
  g3d_delete_planner_form();
  g3d_delete_steering_form();

#ifdef BIO
  g3d_delete_bio_collision_form();
#endif

  nr = p3d_get_desc_number(P3D_ROBOT);
  for(ir=0;ir<nr;ir++){
    //g3d_delete_filter_form(ir);
    g3d_delete_robot_form(ir);
  }
  free(ROBOTS_FORM);
  //free(FILTER_FORM);

  g3d_delete_envparams_obj();
  g3d_delete_envcur_obj();

  g3d_delete_robotparams_obj();
  g3d_delete_robotcur_obj();

  g3d_delete_load_obj();
  g3d_delete_CC_obj();
  g3d_delete_planner_obj();
  g3d_delete_steering_obj();
  g3d_delete_Diffusion_obj();
  g3d_delete_exit_obj();
  g3d_delete_user_obj();
  g3d_delete_bio_collision_obj();
  g3d_delete_load_scenario_obj(); // Modification Fabien
  g3d_delete_save_scenario_obj(); // Modification Fabien

  fl_free_form(MAIN_FORM);

}


static
void g3d_delete_envparams_obj(void)
{
  fl_free_object(envparams_obj);
}


static
void g3d_delete_envcur_obj(void)
{
  fl_free_object(envcur_obj);
}

static
void g3d_delete_robotparams_obj(void)
{
  fl_free_object(robotparams_obj);
}

static
void g3d_delete_robotcur_obj(void)
{
  fl_free_object(robotcur_obj);
}

static
void g3d_delete_load_obj(void)
{
  fl_free_object(load_obj);
}

static
void g3d_delete_CC_obj(void)
{
  fl_free_object(CC_obj);
}

static
void g3d_delete_planner_obj(void)
{
 fl_free_object(planner_obj);
}

static
void g3d_delete_steering_obj(void)
{
 fl_free_object(steering_obj);
}


static
void g3d_delete_Diffusion_obj(void)
{
  fl_free_object(Diffusion_obj);
}

static void g3d_delete_exit_obj(void)
{
  fl_free_object(exit_obj);
}

static void g3d_delete_user_obj(void)
{
  fl_free_object(user_obj);
}

static void g3d_delete_bio_collision_obj(void)
{
  fl_free_object(bio_collision_obj);
}


/********************* Gestion des environnements multiples *******************/


static void save_scene(int env_num)
{G3D_Window *win;
 int i;

 /* printf("on remplit la case %d de  saved_scene\n",env_num-1); */

  saved_scene[env_num-1].pas_max = p3d_get_env_dmax();
  saved_scene[env_num-1].local_planner = p3d_local_get_planner();
  saved_scene[env_num-1].global_planner = p3d_get_MOTION_PLANNER();
  saved_scene[env_num-1].sampling = p3d_get_SAMPLING_CHOICE();
  saved_scene[env_num-1].sorting = p3d_get_SORTING();
  saved_scene[env_num-1].node_max = p3d_get_NB_NODES();
  saved_scene[env_num-1].nb_try = ENV.getInt(Env::NbTry);
  saved_scene[env_num-1].node_compco = p3d_get_COMP_NODES();
  saved_scene[env_num-1].DMAX = p3d_get_DMAX();
  saved_scene[env_num-1].vmax = saved_scene[env_num-1].DMAX*5;
  saved_scene[env_num-1].vmin = saved_scene[env_num-1].DMAX/1000;

  saved_scene[env_num-1].DRAW_OPTIM = ENV.getBool(Env::drawTraj);
  saved_scene[env_num-1].DRAW_GRAPH = ENV.getBool(Env::drawGraph);

  win = g3d_get_cur_win();
  saved_scene[env_num-1].x = win->x;
  saved_scene[env_num-1].y = win->y;
  saved_scene[env_num-1].z = win->z;
  saved_scene[env_num-1].az = win->az;
  saved_scene[env_num-1].el = win->el;
  saved_scene[env_num-1].zo = win->zo;
  for(i=0;i<4;i++){
    saved_scene[env_num-1].up[i] = win->up[i];
  }
  saved_scene[env_num-1].sx = win->sx;
  saved_scene[env_num-1].sy = win->sy;
  saved_scene[env_num-1].sz = win->sz;
  saved_scene[env_num-1].saz = win->saz;
  saved_scene[env_num-1].sel = win->sel;
  saved_scene[env_num-1].szo = win->szo;
  for(i=0;i<4;i++){
    saved_scene[env_num-1].sup[i] = win->sup[i];
  }

  saved_scene[env_num-1].size = win->size;
  saved_scene[env_num-1].FILAIRE = win->FILAIRE;
  saved_scene[env_num-1].CONTOUR = win->CONTOUR;
  saved_scene[env_num-1].GOURAUD = win->GOURAUD;

  saved_scene[env_num-1].saved = TRUE;

/*   printf("save scene : environnement %s point de vue sauve : %f %f %f %f %f %f\n", */
/* 	 p3d_get_desc_curname(P3D_ENV),win->x,win->y,win->z,win->az,win->el,win->zo); */
}

static int scene_is_saved(int num_env)
{
  return(saved_scene[num_env-1].saved);
}


static void del_all_scenes(int nenv)
{int i;

 for(i=0;i<nenv;i++){
   //   if(XYZ_TAB_GRAPH[i]){p3d_del_graph(XYZ_TAB_GRAPH[i]);} Modification Fabien
   saved_scene[i].saved = FALSE;
 }

}



static void charge_scene(int env_num)
{
  G3D_Window *win;
  int i;

   /* printf("on lit la case %d de  saved_scene\n",env_num-1); */
   if(!scene_is_saved(env_num)){printf("ERREUR : scene not saved\n");return;}

  /* on charge l'environnement courant */
  p3d_sel_desc_num(P3D_ENV,env_num-1);

/*   p3d_sel_desc_num(P3D_ROBOT,saved_scene[env_num-1].rcur_num); */
/*   p3d_sel_desc_num(P3D_OBSTACLE,saved_scene[env_num-1].ocur_num); */

  GOTO_READ = TRUE; START_READ = TRUE;

  //  if(XYZ_TAB_GRAPH[env_num-1]){   Modification Fabien
  //    XYZ_GRAPH = XYZ_TAB_GRAPH[env_num-1];
  //  }
  //  else{XYZ_GRAPH = NULL;}

  p3d_set_env_dmax(saved_scene[env_num-1].pas_max);
  p3d_local_set_planner((p3d_localplanner_type) saved_scene[env_num-1].local_planner);
  p3d_set_MOTION_PLANNER(saved_scene[env_num-1].global_planner);

  p3d_set_SAMPLING_CHOICE(saved_scene[env_num-1].sampling);
  p3d_set_SORTING(saved_scene[env_num-1].sorting);
  p3d_set_NB_NODES(saved_scene[env_num-1].node_max);
  ENV.setInt(Env::NbTry,saved_scene[env_num-1].nb_try);
  ENV.setInt(Env::maxNodeCompco,saved_scene[env_num-1].node_compco);
  p3d_set_DMAX(saved_scene[env_num-1].DMAX);
  vmin = saved_scene[env_num-1].vmin;
  vmax = saved_scene[env_num-1].vmax;
  ENV.setBool(Env::drawTraj, saved_scene[env_num-1].DRAW_OPTIM);
  ENV.setBool(Env::drawGraph, saved_scene[env_num-1].DRAW_GRAPH);


  win = g3d_get_cur_win();

  win->size = saved_scene[env_num-1].size;
  g3d_resize_allwin_active(G3D_WINSIZE,G3D_WINSIZE,win->size);
  /* g3d_set_win_camera(win, ,,,5*win->size,20,30,,,1); */

  win->x = saved_scene[env_num-1].x;
  win->y = saved_scene[env_num-1].y;
  win->z = saved_scene[env_num-1].z;
  win->az = saved_scene[env_num-1].az;
  win->el = saved_scene[env_num-1].el;
  win->zo = saved_scene[env_num-1].zo;
  for(i=0;i<4;i++){
    win->up[i] = saved_scene[env_num-1].up[i];
  }
  win->sx = saved_scene[env_num-1].sx;
  win->sy = saved_scene[env_num-1].sy;
  win->sz = saved_scene[env_num-1].sz;
  win->saz = saved_scene[env_num-1].saz;
  win->sel = saved_scene[env_num-1].sel;
  win->szo = saved_scene[env_num-1].szo;
  for(i=0;i<4;i++){
    win->sup[i] = saved_scene[env_num-1].sup[i];
  }

  win->FILAIRE = saved_scene[env_num-1].FILAIRE;
  win->CONTOUR = saved_scene[env_num-1].CONTOUR;
  win->GOURAUD = saved_scene[env_num-1].GOURAUD;

/*   printf("charge scene : environnement %s point de vue charge : %f %f %f %f %f %f %f %f %f\n", */
/* 	 p3d_get_desc_curname(P3D_ENV),win->x,win->y,win->z,win->zo,win->az,win->el,win->up[0],win->up[1],win->up[2]); */
}


static void g3d_create_robots_forms(int nr)
{
  int r,ir;
  p3d_rob *rob;
  configPt q=NULL;

  r = p3d_get_desc_curnum(P3D_ROBOT);

  ROBOTS_FORM = (MENU_ROBOT *) malloc(nr*sizeof(MENU_ROBOT));
  //FILTER_FORM = (MENU_FILTER *) malloc(nr*sizeof(MENU_FILTER));

  for(ir=0; ir < nr; ir ++){
	if (ir >= nr)
	{
		printf(" ERROR in file: %s  at line: %d \n. ir=%d >= nr=%d \n", __FILE__, __LINE__, ir, nr );
		exit(10);
	}
    p3d_sel_desc_num(P3D_ROBOT,ir);
    rob = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    /* on remplit les position current/goto du robot */
    q = p3d_get_robot_config(rob);
    if(!START_READ){
      p3d_copy_config_into(rob, q, &(rob->ROBOT_POS));
    }
    if(!GOTO_READ){
      p3d_copy_config_into(rob, q, &(rob->ROBOT_GOTO));
    }
    p3d_destroy_config(rob, q);
    /* on cree le menu robot du robot ir */
    g3d_create_robot_form(ir);
    /* on lui donne un planificateur local par defaut */
    ROBOTS_FORM[ir].PLANNER_CHOICE=P3D_LINEAR_PLANNER;
    /* on cree son menu filter box */
    // NOTE : FilterBox functions (create, delete) have been disabled\n");
    //g3d_create_filterbox_form(ir);
  }
  p3d_sel_desc_num(P3D_ROBOT,r);
}


//This method is called when the user closes ANY of the application's forms.
// So any special treatment before closing should be called from here
static int CB_OnApplicationClose( FL_FORM *form, void *argument )
{
	int i = 0;
	FL_FORM *tempForm;

	//From this method one can return FL_IGNORE to prevent the window from closing
	//  or FL_OK to allow the window to close.


	#ifdef ENERGY
	if (form == BIO_ENERGY_FORM)
	{
		fl_hide_form( form );
		fl_set_button( user_obj, 0);
		return (FL_IGNORE);
	}
	#endif

	if (form == BIO_COLLISION_FORM)
	{
		fl_hide_form( form );
		fl_set_button( bio_collision_obj, 0);
		return (FL_IGNORE);
	}

	i = p3d_get_desc_curnum(P3D_ROBOT);
	if ( form == ROBOTS_FORM[i].ROBOT_FORM )
	{
		fl_hide_form( form );
		fl_set_button( robotparams_obj, 0);
		return (FL_IGNORE);
	}

	if ( form == PLANNER_FORM )
	{
		fl_hide_form( form );
		fl_set_button( planner_obj, 0);
		return (FL_IGNORE);
	}

	//If no other window
	tempForm =fl_get_app_mainform();
   	fl_set_app_mainform(form);
	if ( fl_show_question("Do you really want to close the application ?", 0) )
	{
		//Actually, instead of just brutally closing the application
		// We could click on the "Exit" button instead
		//  fl_trigger_object(exit_obj);
		fl_set_app_mainform(tempForm);
		return FL_OK;
	}
   	fl_set_app_mainform(tempForm);

	return(FL_IGNORE);
}


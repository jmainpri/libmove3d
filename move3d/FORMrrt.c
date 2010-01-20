#include "Graphic-pkg.h"
#include "Rrt-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"

/***************************/
/* creation de l'interface */
/***************************/

FL_FORM               *RRT_FORM = NULL; /* def extern in FORMmain.c */

/*******************************************/

static FL_OBJECT  *extend_button_obj;
static FL_OBJECT  *step_slider_obj;
static FL_OBJECT  *K_slider_obj;
static FL_OBJECT  *draw_tree_button_obj;
static FL_OBJECT  *K_button_obj;
static FL_OBJECT  *step_text_obj;
static FL_OBJECT  *save_tree_button_obj;
static FL_OBJECT  *load_tree_button_obj;
static FL_OBJECT  *stop_button_obj;
static FL_OBJECT  *reset_button_obj;
static FL_OBJECT  *dist_choice_obj;
static FL_OBJECT  *connect_choice_obj;
static FL_OBJECT  *extend_choice_obj;
static FL_OBJECT  *cal_distance_obj;


/*******************************************************/
/* Callback functions associated to the button *********/
/*******************************************************/

static void extend_button(FL_OBJECT *ob, long arg);
static void save_tree_button(FL_OBJECT *ob, long arg);
static void load_tree_button(FL_OBJECT *ob, long arg);
static void draw_tree_button(FL_OBJECT *ob, long arg);
static void stop_button(FL_OBJECT *ob, long arg);
static void reset_button(FL_OBJECT *ob, long arg);
static char *filename (void);
static int fct_graph(void);
static int fct_stop(void);
#if 0
static void cal_distance(FL_OBJECT *ob, long arg);
#endif

/*******************************/
/* variables (dessin et arret) */
/*******************************/
//extern G3D_Window *G3D_WIN;
int DRAW_TREE = FALSE;
int STOP = FALSE;



/******************************************/
/* The two functions called by move3d     */
/******************************************/
   
void g3d_create_rrt_form(void)
{
  RRT_FORM = fl_bgn_form(FL_UP_BOX,400.0,240.0);

  /* a button object and the associated callback function */
  extend_button_obj = fl_add_button(FL_NORMAL_BUTTON,
				  10.0,10.0,120.0,30.0,
				  "extend RRT");
  fl_set_call_back(extend_button_obj,extend_button,0); 

  /* a button object and the associated callback function */
  save_tree_button_obj = fl_add_button(FL_NORMAL_BUTTON,
				  10.0,50.0,120.0,30.0,
				  "save RRT");
  fl_set_call_back(save_tree_button_obj,save_tree_button,0); 

  /* a button object and the associated callback function */
  load_tree_button_obj = fl_add_button(FL_NORMAL_BUTTON,
				  140.0,50.0,120.0,30.0,
				  "load RRT");
  fl_set_call_back(load_tree_button_obj,load_tree_button,0); 

  /* a push button object and the associated callback function */
  draw_tree_button_obj = fl_add_button(FL_PUSH_BUTTON,
				  270.0,10.0,120.0,30.0,
				  "draw tree");
  fl_set_call_back(draw_tree_button_obj,draw_tree_button,0); 

  /* a button object and the associated callback function */
  stop_button_obj = fl_add_button(FL_NORMAL_BUTTON,
				  140.0,10.0,120.0,30.0,
				  "stop"); 
  fl_set_call_back(stop_button_obj,stop_button,0); 

  /* a button object and the associated callback function */
  reset_button_obj = fl_add_button(FL_NORMAL_BUTTON,
				  270.0,50.0,120.0,30.0,
				  "reset"); 
  fl_set_call_back(reset_button_obj,reset_button,0); 

  /* a button object and the associated callback function */
  cal_distance_obj = fl_add_choice(FL_NORMAL_CHOICE,
				   10.0,90.0,185.0,30.0,
				  ""); 
  fl_addto_choice(cal_distance_obj,"start normal | rebond | box");
  fl_set_choice(cal_distance_obj,1); /* set the choice default */

  /* a slider object */
  K_slider_obj = fl_add_valslider(FL_HOR_SLIDER, 
				  60.0,210.0,320.0,20.0,
				  "");
  fl_set_slider_bounds(K_slider_obj,
		       100.0,100000.0); /* the minmax slider values */ 
  fl_set_slider_value(K_slider_obj,1000.0); /* set slider value */
  fl_set_slider_precision(K_slider_obj,0); /* precision of the value */

 /* a slider object */
  step_slider_obj = fl_add_valslider(FL_HOR_SLIDER, 
				  60.0,170.0,320.0,20.0,
				  "");
  fl_set_slider_bounds(step_slider_obj,
		       1.0,20.0); /* the minmax slider values */
  fl_set_slider_value(step_slider_obj,10.0); /* set slider value */
  fl_set_slider_precision(step_slider_obj,0); /* precision of the value */
 
  /* push button object */ 
  K_button_obj = fl_add_button(FL_PUSH_BUTTON,
				  10.0,210.0,40.0,20.0,
				  "K");

  /* texte sous la forme d'un bouton */ 
  step_text_obj = fl_add_button(FL_NORMAL_BUTTON,
				  10.0,170.0,40.0,20.0,
				  "STEP");
  /* choice object for changing the distance function */
  dist_choice_obj = fl_add_choice(FL_NORMAL_CHOICE,
				  10.0,130.0,185.0,30.0, "");
  fl_addto_choice(dist_choice_obj,"xyz distance | max moving distance");
  fl_addto_choice(dist_choice_obj,"curve distance | max q change");
  fl_addto_choice(dist_choice_obj,"axe distance");
  fl_set_choice(dist_choice_obj,5); /* set the choice default */

  /* choice object for changing the connection function */
  connect_choice_obj = fl_add_choice(FL_NORMAL_CHOICE,
				     205.0,130.0,185.0,30.0, "");
  fl_addto_choice(connect_choice_obj,"link to tree if node closed");
  fl_addto_choice(connect_choice_obj,"link with nearest neighbor");
  fl_addto_choice(connect_choice_obj,"link with nearest neighbor if node closed");
  fl_set_choice(connect_choice_obj,3); /* set the choice default */

  /* choice object for changing the connection function */
  extend_choice_obj = fl_add_choice(FL_NORMAL_CHOICE,205.0,90.0,185.0,30.0, "");
  fl_addto_choice(extend_choice_obj,"extend linearly (config space)");
  fl_addto_choice(extend_choice_obj,"extend linearly (on localpath)");
  fl_addto_choice(extend_choice_obj,"extend linearly & decrease step");
  fl_addto_choice(extend_choice_obj,"extend surface");
  fl_set_choice(extend_choice_obj,1); /* set the choice default */

  fl_end_form();
}
  

void g3d_delete_rrt_form(void)
{
  /* on cache la form avant de la détruire */
  fl_hide_form(RRT_FORM);

  fl_free_object(extend_button_obj); 
  fl_free_object(step_slider_obj); 
  fl_free_object(K_slider_obj); 
  fl_free_object(draw_tree_button_obj);
  fl_free_object(K_button_obj);
  fl_free_object(step_text_obj);
  fl_free_object(save_tree_button_obj);
  fl_free_object(load_tree_button_obj);
  fl_free_object(stop_button_obj);
  fl_free_object(reset_button_obj);
  fl_free_object(dist_choice_obj);
  fl_free_object(connect_choice_obj);
  fl_free_object(extend_choice_obj);
  fl_free_object(cal_distance_obj);
  fl_free_form(RRT_FORM);
  printf("fin !\n");
}

/*******************************************************/
/* Callback functions specific to the rrt application  */
/*******************************************************/


static void extend_button(FL_OBJECT *ob, long arg)
{  
  int Knum;
  int pas  = fl_get_slider_value(step_slider_obj);
  int dist_choice = fl_get_choice(dist_choice_obj);
  int connect_choice = fl_get_choice(connect_choice_obj);
  int ext_choice = fl_get_choice(extend_choice_obj);
  int start_choice = fl_get_choice(cal_distance_obj);
  int chemin;
  rrt_graph *graph;

  graph = get_rrt_graph();
  if(fl_get_button(K_button_obj))
    Knum  = fl_get_slider_value(K_slider_obj);
  else
    Knum = 2147483640;

  STOP = FALSE;
  chemin = start(pas, Knum, fct_stop, dist_choice,
		 connect_choice, ext_choice, fct_graph, start_choice);
  if (chemin)
    g3d_add_traj("rrt search",p3d_get_desc_number(P3D_TRAJ));
}


/********************************************/
/* draw of the tree if the button is pushed */
/********************************************/

static void draw_tree_button(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(draw_tree_button_obj))
    { 
      g3d_set_win_drawer(G3D_WIN, draw_rrt);
      DRAW_TREE = TRUE; 
      /* visualize the scene  */
      g3d_draw_allwin_active();
    }
  else
    {
      g3d_set_win_drawer(G3D_WIN, g3d_draw);
      DRAW_TREE = FALSE;
      /* visualize the scene  */
      g3d_draw_allwin_active();
    }
}


/*************************************/
/* arret si bouton stop a ete touche */     
/*************************************/

static void stop_button(FL_OBJECT *ob, long arg)
{
  /* arret si bouton stop a ete touche */     
  STOP = TRUE; 
}

/***********************************************/
/* detruire arbre si bouton reset a ete touche */     
/***********************************************/

static void reset_button(FL_OBJECT *ob, long arg)
{
  reset();
}

/***************/
/* save a tree */
/***************/

static void save_tree_button(FL_OBJECT *ob, long arg)
{
  char *file;

  file = filename();
  save_tree(file);
}


/***************/
/* load a tree */
/***************/

static void load_tree_button(FL_OBJECT *ob, long arg)
{
  char *file;

  file = filename();  
  load_tree(file);

}


/**********************************************/
/* genere l'interface pour choisir un fichier */
/**********************************************/

static char *filename (void)
{
char *file;
char c_dir_name[200];
char DEF_FILE[200];
char *env_name; 
int c_sz=0;

  env_name = p3d_get_desc_curname(P3D_ENV);
  strcpy(DEF_FILE,env_name);
  strcat(DEF_FILE,".rrt");
  p3d_get_directory(c_dir_name);
  c_sz = (int) strlen(c_dir_name); 
  if(c_dir_name[c_sz-1] != '/'){strcat(c_dir_name,"/");}   
  
  file = (char*)fl_show_file_selector("RRT filename",c_dir_name,
				      "*.rrt",DEF_FILE);

  return (file);
}


/*************************************************************/
/* function call by rrt, it checks form and stops eventually */
/*************************************************************/

static int fct_stop(void)
{
  fl_check_forms();
  if(STOP)
    return(FALSE);
  else
    return(TRUE);
}


/***************************************************/
/* function call by rrt, draw the graph eventually */
/***************************************************/

static int fct_graph(void)
{
  if (DRAW_TREE)    
    g3d_draw_allwin_active();    
  if(DRAW_TREE)
    return(TRUE);
  else
    return(FALSE);
}


/****************************************************/
/* calculate the distance between q init and q goal */
/****************************************************/

#if 0
static void cal_distance(FL_OBJECT *ob, long arg)
{
  int dist_choice = fl_get_choice(dist_choice_obj);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */

  print_distance(r->ROBOT_POS,r->ROBOT_GOTO,dist_choice);
}
#endif

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

typedef struct {
  char name[64];
  int default_joints[GIK_MAX_JOINT_NO];
  int default_joints_no;
  int actual_joints[GIK_MAX_JOINT_NO];
  int actual_joints_no;
  int active_joint;
} PD_gik_task;

/* ----------- VARIABLES ------------- */
static int gik_active_task_no = 0; /* By default Look at task is active */
p3d_rob * GIK_target_robot = NULL;

// static int GIK_NOT_SUPPORTED_ROBOT = FALSE;

int GIK_VIS = 1;
double GIK_PRECISION = 0.05;
int GIK_STEP = 100;
double GIK_FORCE;

void hri_gik_set_visstep(int step)
{
  GIK_VIS = step;
}

PD_gik_task PDGIKTASK[GIK_MAX_TASK_NO];

/* ----------- FORM VARIABLES ------------- */
FL_FORM * GIK_FORM = NULL;

static FL_OBJECT * GIK_SELECTED_JOINTS_OBJ[GIK_MAX_JOINT_NO];
static FL_OBJECT * GIKTASKGROUP;
static FL_OBJECT * GIKTASK_ACTIVATE_OBJ[GIK_MAX_TASK_NO];
static FL_OBJECT * GIK_TASK_RESET;
static FL_OBJECT * GIK_ACTIVE_JOINTS_OBJ[GIK_MAX_TASK_NO];
static FL_OBJECT * GIK_PRIORITY_OBJ[GIK_MAX_TASK_NO];
static FL_OBJECT * GIKRUNGROUP;
FL_OBJECT * GIK_RUN_OBJ_EXT;  /* These variables are also accessed by hri_planner interface */
FL_OBJECT * GIK_TARGET_ROBOT_OBJ_EXT;
FL_OBJECT * GIK_VIS_OBJ_EXT;
FL_OBJECT * GIK_PRECISION_OBJ_EXT;
FL_OBJECT * GIK_STEP_OBJ_EXT;
extern FL_OBJECT * GIK_RUN_OBJ;  /* These variables are from hri_planner interface */
extern FL_OBJECT * GIK_TARGET_ROBOT_OBJ;
extern FL_OBJECT * GIK_VIS_OBJ;
extern FL_OBJECT * GIK_PRECISION_OBJ;
extern FL_OBJECT * GIK_STEP_OBJ;
/*
 FL_OBJECT * neck1; // 16
 FL_OBJECT * neck2;   // 17
 FL_OBJECT * lshoulder1; // 32
 FL_OBJECT * lshoulder2; // 33
 FL_OBJECT * lshoulder3; // 34
 FL_OBJECT * lelbow; // 35
 FL_OBJECT * lwrist1; // 36
 FL_OBJECT * lwrist2; // 37
 FL_OBJECT * rshoulder1; // 19
 FL_OBJECT * rshoulder2; // 20
 FL_OBJECT * rshoulder3; // 21
 FL_OBJECT * relbow; // 22
 FL_OBJECT * rwrist1; // 23
 FL_OBJECT * rwrist2; // 24
 FL_OBJECT * waist1; // 14
 FL_OBJECT * waist2; // 15
 FL_OBJECT * lpelvis1; // 8
 FL_OBJECT * lpelvis2; // 9
 FL_OBJECT * lpelvis3; // 10
 FL_OBJECT * lknee; // 11
 FL_OBJECT * lankle1; // 12
 FL_OBJECT * lankle2; // 13
 FL_OBJECT * rpelvis1; // 2
 FL_OBJECT * rpelvis2; // 3
 FL_OBJECT * rpelvis3; // 4
 FL_OBJECT * rknee; // 5
 FL_OBJECT * rankle1; // 6
 FL_OBJECT * rankle2; // 7
 */

/* ------------- FUNCTIONS --------- */
static void g3d_create_gik_hrp2_jointsel_objects(void);
static void CB_gik_select_joint_obj(FL_OBJECT *obj, long arg);

static void g3d_create_gik_tasks(void);
static void CB_giktask_activate_obj(FL_OBJECT *obj, long arg);
static void CB_gik_reset_button_obj(FL_OBJECT *obj, long arg);

static void g3d_create_gik_run(void);

static void gik_fill_defaultgiktask(void);
static void gik_reset_actual_joints(PD_gik_task * PDGIKTASK);
static void gik_initialize_current_tasks( p3d_rob * robotPt );

void g3d_create_gik_jointsel_form ( void )
{
  GIK_FORM = fl_bgn_form(FL_UP_BOX,500.0,400.0);
#ifdef HRI_HRP2
  printf("Your robot is HRP-2.\n");
  g3d_create_gik_hrp2_jointsel_objects();
  g3d_create_gik_tasks();
  g3d_create_gik_run();
#else
  printf("Your robot is not supported by the GIK Interface.\n");
//  GIK_NOT_SUPPORTED_ROBOT = TRUE;
#endif

  fl_end_form();
}

void g3d_show_gik_jointsel_form ( void )
{
  fl_show_form(GIK_FORM,FL_PLACE_SIZE,TRUE, "GIK Joint Selection");
}

void g3d_hide_gik_jointsel_form ( void )
{
  fl_hide_form(GIK_FORM);
}

void g3d_delete_gik_jointsel_form ( void )
{
  fl_free_form(GIK_FORM);
}

/* ------------- This function is for HRP-2 only ------------------ */
static void g3d_create_gik_hrp2_jointsel_objects ( void )
{
  FL_OBJECT * obj;
  int i;
  int xframe = 10, yframe = 15;
  int hpx = xframe+60, hpy = yframe+20;

  fl_add_labelframe(FL_BORDER_FRAME,xframe,yframe,150,280,"HRP-2 Joints Selection");

  for (i=0; i<GIK_MAX_JOINT_NO; i++){
    GIK_SELECTED_JOINTS_OBJ[i] = NULL;
  }
  for (i=0; i<GIK_MAX_TASK_NO; i++){
    GIK_ACTIVE_JOINTS_OBJ[i] = NULL;
  }

  /* hpx = 70, hpy = 35 */
  GIK_SELECTED_JOINTS_OBJ[16] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,16);
  GIK_SELECTED_JOINTS_OBJ[17] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy+20,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,17);
  GIK_ACTIVE_JOINTS_OBJ[0] = obj = fl_add_box(FL_ROUNDED_BOX,hpx+25,hpy+12,5,5,""); /* active "look at" joint */
  GIK_SELECTED_JOINTS_OBJ[32] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+35,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,32);
  GIK_SELECTED_JOINTS_OBJ[33] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+40,hpy+35,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,33);
  GIK_SELECTED_JOINTS_OBJ[34] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+45,hpy+55,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,34);
  GIK_SELECTED_JOINTS_OBJ[35] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+50,hpy+85,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,35);
  GIK_SELECTED_JOINTS_OBJ[36] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+50,hpy+115,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,36);
  GIK_SELECTED_JOINTS_OBJ[37] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+50,hpy+135,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,37);
  GIK_ACTIVE_JOINTS_OBJ[1] = obj = fl_add_box(FL_ROUNDED_BOX,hpx+61,hpy+180,5,5,""); /* active "Left hand point at" joint */
  GIK_ACTIVE_JOINTS_OBJ[3] = obj = fl_add_box(FL_ROUNDED_BOX,hpx+61,hpy+160,5,5,""); /* active "Left hand reach at" joint */
  GIK_SELECTED_JOINTS_OBJ[19] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+35,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,19);
  GIK_SELECTED_JOINTS_OBJ[20] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-40,hpy+35,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,20);
  GIK_SELECTED_JOINTS_OBJ[21] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-45,hpy+55,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,21);
  GIK_SELECTED_JOINTS_OBJ[22] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-50,hpy+85,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,22);
  GIK_SELECTED_JOINTS_OBJ[23] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-50,hpy+115,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,23);
  GIK_SELECTED_JOINTS_OBJ[24] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-50,hpy+135,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,24);
  GIK_ACTIVE_JOINTS_OBJ[2] = obj = fl_add_box(FL_ROUNDED_BOX,hpx-39,hpy+180,5,5,""); /* active "Right hand point at" joint */
  GIK_ACTIVE_JOINTS_OBJ[4] = obj = fl_add_box(FL_ROUNDED_BOX,hpx-39,hpy+160,5,5,""); /* active "Right hand reach at" joint */
  GIK_SELECTED_JOINTS_OBJ[14] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy+60,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,14);
  GIK_SELECTED_JOINTS_OBJ[15] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy+80,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,15);
  GIK_SELECTED_JOINTS_OBJ[8] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+10,hpy+105,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,8);
  GIK_SELECTED_JOINTS_OBJ[9] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+125,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,9);
  GIK_SELECTED_JOINTS_OBJ[10] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+145,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,10);
  GIK_SELECTED_JOINTS_OBJ[11] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+175,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,11);
  GIK_SELECTED_JOINTS_OBJ[12] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+205,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,12);
  GIK_SELECTED_JOINTS_OBJ[13] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+225,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,13);
  GIK_SELECTED_JOINTS_OBJ[2] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-10,hpy+105,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,2);
  GIK_SELECTED_JOINTS_OBJ[3] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+125,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,3);
  GIK_SELECTED_JOINTS_OBJ[4] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+145,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,4);
  GIK_SELECTED_JOINTS_OBJ[5] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+175,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,5);
  GIK_SELECTED_JOINTS_OBJ[6] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+205,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,6);
  GIK_SELECTED_JOINTS_OBJ[7] = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+225,30,30,"");
  fl_set_object_color(obj,FL_MCOL,FL_GREEN);
  fl_set_object_callback(obj,CB_gik_select_joint_obj,7);
}

static void CB_gik_select_joint_obj(FL_OBJECT *obj, long arg)
{
  int i,temp;

  int * selected_joints_nb = &PDGIKTASK[gik_active_task_no].actual_joints_no;
  int * selected_joints = PDGIKTASK[gik_active_task_no].actual_joints;

  if(*selected_joints_nb == 0){
    selected_joints[0] = arg;
    (*selected_joints_nb) = 1;
    return;
  }

  for(i=0; i<*selected_joints_nb; i++){
    if(selected_joints[i]>arg){
      for(;i<*selected_joints_nb+1;i++){
        temp = selected_joints[i];
        selected_joints[i] = arg;
        arg = temp;
      }
      (*selected_joints_nb)++;
      break;
    }
    if(selected_joints[i]==arg){
      for(;i<*selected_joints_nb;i++){
        selected_joints[i] = selected_joints[i+1];
      }
      (*selected_joints_nb)--;
      break;
    }
    if(i==*selected_joints_nb-1){
      selected_joints[i+1] = arg;
      (*selected_joints_nb)++;
      break;
    }
  }
  printf("selected joints:");
  for(i=0; i<*selected_joints_nb; i++)	printf(" %d",selected_joints[i]);
  printf("\n");
}

/* --- TASK GROUP --- */
static void g3d_create_gik_tasks(void)
{
  int i,j;
  int xframe = 170, yframe = 15;

  fl_add_labelframe(FL_BORDER_FRAME,xframe,yframe,300,200,"Tasks");

  GIKTASKGROUP = fl_bgn_group();

  gik_fill_defaultgiktask();

  for(i=0; i<GIK_MAX_TASK_NO; i++){
    GIKTASK_ACTIVATE_OBJ[i] = fl_add_checkbutton(FL_RADIO_BUTTON,xframe+10,yframe+10+i*20,100,20,PDGIKTASK[i].name);
    fl_set_call_back(GIKTASK_ACTIVATE_OBJ[i],CB_giktask_activate_obj,i);
  }

  GIK_TASK_RESET = fl_add_button(FL_NORMAL_BUTTON,xframe+10,yframe+20+(i*20),130,25,"Revert to default Joints");
  fl_set_call_back(GIK_TASK_RESET,CB_gik_reset_button_obj,0);

  fl_add_labelframe(FL_BORDER_FRAME,xframe+160,yframe+20,130,160,"Priority Order");
  for(i=0; i<GIK_MAX_TASK_NO; i++){
    GIK_PRIORITY_OBJ[i] = fl_add_choice(FL_NORMAL_CHOICE,xframe+170,yframe+40+(25*i),110,20,"");
    fl_addto_choice(GIK_PRIORITY_OBJ[i],"None");
    for(j=0; j<GIK_MAX_TASK_NO; j++){
      fl_addto_choice(GIK_PRIORITY_OBJ[i],PDGIKTASK[j].name);
    }
  }

  fl_end_group();

  CB_giktask_activate_obj(GIKTASK_ACTIVATE_OBJ[gik_active_task_no],gik_active_task_no);
  fl_set_button(GIKTASK_ACTIVATE_OBJ[gik_active_task_no],TRUE);
}

static void CB_giktask_activate_obj(FL_OBJECT *obj, long arg)
{
  int i;

  for (i=0; i<GIK_MAX_JOINT_NO; i++){
    if(GIK_SELECTED_JOINTS_OBJ[i] != NULL)
      fl_set_button(GIK_SELECTED_JOINTS_OBJ[i],FALSE);
  }

  for(i=0; i<GIK_MAX_TASK_NO; i++){
    if(GIK_ACTIVE_JOINTS_OBJ[i] != NULL)
      fl_set_object_color(GIK_ACTIVE_JOINTS_OBJ[i],FL_MCOL,FL_COL1);
  }

  for (i=0;	i<PDGIKTASK[arg].actual_joints_no; i++) {
    fl_set_button(GIK_SELECTED_JOINTS_OBJ[PDGIKTASK[arg].actual_joints[i]],TRUE);
  }

  gik_active_task_no = arg;
  printf("Active Task is no %d with joints",gik_active_task_no);
  for(i=0; i<PDGIKTASK[gik_active_task_no].actual_joints_no; i++)
    printf(" %d",PDGIKTASK[gik_active_task_no].actual_joints[i]);
  printf("\n");

  fl_set_object_color(GIK_ACTIVE_JOINTS_OBJ[gik_active_task_no],FL_RED,FL_COL1);
}

static void CB_gik_reset_button_obj(FL_OBJECT *obj, long arg)
{
  gik_reset_actual_joints(&PDGIKTASK[gik_active_task_no]);

  CB_giktask_activate_obj(GIKTASK_ACTIVATE_OBJ[gik_active_task_no],gik_active_task_no);
}


/* ----- GIK RUN Group --- */
static void g3d_create_gik_run(void)
{
  int i=0;
  int framex = 170, framey = 230;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  HRI_GIK = hri_gik_create_gik();

  fl_add_labelframe(FL_BORDER_FRAME,framex,framey,300,60,"Run");

  GIKRUNGROUP = fl_bgn_group();

  GIK_RUN_OBJ_EXT = fl_add_button(FL_NORMAL_BUTTON,framex+10,framey+30,70,25,"Run GIK");
  GIK_TARGET_ROBOT_OBJ_EXT = fl_add_choice(FL_NORMAL_CHOICE,framex+10,framey+7,70,20,"");
  GIK_VIS_OBJ_EXT = fl_add_valslider(FL_HOR_SLIDER,framex+150,framey+7,100,15,"");
  GIK_PRECISION_OBJ_EXT = fl_add_valslider(FL_HOR_SLIDER,framex+150,framey+23,100,15,"");
  GIK_STEP_OBJ_EXT = fl_add_valslider(FL_HOR_SLIDER,framex+150,framey+39,100,15,"");
  fl_add_text(FL_NORMAL_TEXT,framex+90,framey+5,60,20,"Vis. Step");
  fl_add_text(FL_NORMAL_TEXT,framex+90,framey+21,60,20,"Precision");
  fl_add_text(FL_NORMAL_TEXT,framex+90,framey+37,60,20,"Iterations");

  for(i=0; i<env->nr; i++){
    fl_addto_choice(GIK_TARGET_ROBOT_OBJ_EXT,env->robot[i]->name);
  }

  fl_set_call_back(GIK_RUN_OBJ_EXT,CB_gik_run_obj,1);
  fl_set_call_back(GIK_TARGET_ROBOT_OBJ_EXT,CB_gik_target_robot_obj,1);

  fl_set_object_callback(GIK_VIS_OBJ_EXT,CB_gik_vis_obj,1);
  fl_set_slider_step(GIK_VIS_OBJ_EXT,10);
  fl_set_slider_bounds(GIK_VIS_OBJ_EXT,1,500);
  fl_set_slider_value(GIK_VIS_OBJ_EXT,GIK_VIS);

  fl_set_object_callback(GIK_PRECISION_OBJ_EXT,CB_gik_precision_obj,1);
  fl_set_slider_step(GIK_PRECISION_OBJ_EXT,0.01);
  fl_set_slider_bounds(GIK_PRECISION_OBJ_EXT,0,1);
  fl_set_slider_value(GIK_PRECISION_OBJ_EXT,GIK_PRECISION);

  fl_set_object_callback(GIK_STEP_OBJ_EXT,CB_gik_step_obj,1);
  fl_set_slider_step(GIK_STEP_OBJ_EXT,10);
  fl_set_slider_bounds(GIK_STEP_OBJ_EXT,1,500);
  fl_set_slider_value(GIK_STEP_OBJ_EXT,GIK_STEP);

  fl_end_group();
}

void CB_gik_target_robot_obj(FL_OBJECT *obj, long arg)
{
  int val = fl_get_choice(obj);
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  GIK_target_robot = env->robot[val-1];

  if(obj == GIK_TARGET_ROBOT_OBJ)
    fl_set_choice(GIK_TARGET_ROBOT_OBJ_EXT,val);
  if(obj == GIK_TARGET_ROBOT_OBJ_EXT)
    fl_set_choice(GIK_TARGET_ROBOT_OBJ,val);

  printf("%s is selected as GIK target\n",GIK_target_robot->name);
}

void CB_gik_run_obj(FL_OBJECT *obj, long arg)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob * robotPt;
  int i;
  p3d_vector3 Tcoord[3];
  configPt q;

  if (HRI_GIK == NULL) {
    PrintError(("Gik not initialized\n"));
    return;
  }

  for(i=0; i<env->nr; i++){
    if( strstr(env->robot[i]->name,"HRP2ROBOT") ){
      robotPt = env->robot[i];
    }
  }
  Tcoord[0][0] = Tcoord[1][0] = Tcoord[2][0] = (GIK_target_robot->BB.xmax+GIK_target_robot->BB.xmin)/2;
  Tcoord[0][1] = Tcoord[1][1] = Tcoord[2][1] = (GIK_target_robot->BB.ymax+GIK_target_robot->BB.ymin)/2;
  Tcoord[0][2] = Tcoord[1][2] = Tcoord[2][2] = (GIK_target_robot->BB.zmax+GIK_target_robot->BB.zmin)/2;

  printf("Going to %f,%f,%f\n",Tcoord[0][0], Tcoord[0][1], Tcoord[0][2]);

  q = p3d_get_robot_config(robotPt);

  if(HRI_GIK->GIKInitialized){
    hri_gik_destroy_gik_data(HRI_GIK);
    hri_gik_uninitialize_gik(HRI_GIK);
  }

  gik_initialize_current_tasks(robotPt);

  hri_gik_compute(robotPt, HRI_GIK, GIK_STEP, GIK_PRECISION, FALSE, GIK_FORCE, Tcoord, NULL, &q, NULL);

  p3d_set_and_update_this_robot_conf(robotPt,q);

  p3d_destroy_config(robotPt,q);

  g3d_draw_allwin_active();
}

static void gik_initialize_current_tasks( p3d_rob * robotPt )
{
  int i,j,k,l,val,task_no=0;
  int ** jm;
  int priority_form_val[GIK_MAX_TASK_NO];
  int priority_form_val_l=0;
  int linelength;

  for(i=0; i<GIK_MAX_TASK_NO; i++){
    val = fl_get_choice(GIK_PRIORITY_OBJ[i]);
    if(val==1) continue;
    for(j=0; j<priority_form_val_l; j++){
      if(priority_form_val[j] == val-2){
        break;
      }
    }
    if(j==priority_form_val_l){
      priority_form_val[priority_form_val_l]=val-2;
      priority_form_val_l++;
    }
  }

  /* Allocation and fill with zeros */
  jm = MY_ALLOC(int*,priority_form_val_l);
  for(i=0; i<priority_form_val_l; i++){
    jm[i] = MY_ALLOC(int,GIK_MAX_JOINT_NO);
    for(j=0; j<GIK_MAX_JOINT_NO; j++){
      jm[i][j] = 0;
    }
  }

  /* Filling the joint matrix regardless of additional zeros */
  for(i=0; i<priority_form_val_l; i++){
    for(j=0; j<PDGIKTASK[priority_form_val[i]].actual_joints_no; j++){
      jm[i][PDGIKTASK[priority_form_val[i]].actual_joints[j]] = PDGIKTASK[priority_form_val[i]].actual_joints[j];
    }
    jm[i][PDGIKTASK[priority_form_val[i]].active_joint] = PDGIKTASK[priority_form_val[i]].active_joint;
  }


  /* printf("Joint matrix result:\n");
   for(i=0; i<priority_form_val_l; i++){
   for(j=0; j<GIK_MAX_JOINT_NO; j++){
   printf(" %2d", jm[i][j]);
   }
   printf("\n");
   }	*/

  /* post processing to remove all-zero columns */
  linelength = GIK_MAX_JOINT_NO;
  for(j=0; j<linelength; j++){
    for(i=0; i<priority_form_val_l; i++){
      if(jm[i][j]!=0){
        break;
      }
    }
    if(i==priority_form_val_l){
      /* remove jth column from jm */
      for(k=0; k<priority_form_val_l; k++){
        for(l=j; l<linelength-1; l++){
          jm[k][l] = jm[k][l+1];
        }
      }
      linelength--;
      j--;
    }
  }

  /*	printf("Joint matrix result:\n");
   for(i=0; i<priority_form_val_l; i++){
   for(j=0; j<linelength; j++){
   printf(" %2d", jm[i][j]);
   }
   printf("\n");
   }
   */
  //printf("joints  %i\n",linelength);
  hri_gik_initialize_gik(HRI_GIK,robotPt,FALSE,linelength);
  for(i=0; i<priority_form_val_l; i++){
    hri_gik_add_task(HRI_GIK, 3, linelength, i+1, jm[i], PDGIKTASK[priority_form_val[i]].active_joint);
  }
  //	PDGIKTASK[gik_active_task_no].actual_joints[PDGIKTASK[gik_active_task_no].actual_joints_no] = PDGIKTASK[gik_active_task_no].active_joint;
  //	hri_gik_add_task(HRI_GIK, 3, PDGIKTASK[gik_active_task_no].actual_joints_no + 1, 1, PDGIKTASK[gik_active_task_no].actual_joints, PDGIKTASK[gik_active_task_no].active_joint);

  for(i=0; i<task_no; i++){
    MY_FREE(jm[i],int,GIK_MAX_JOINT_NO);
  }
  MY_FREE(jm,int*,task_no);

}

void CB_gik_vis_obj(FL_OBJECT *obj, long arg)
{
  GIK_VIS = fl_get_slider_value(obj);
  if(obj == GIK_VIS_OBJ)
    fl_set_slider_value(GIK_VIS_OBJ_EXT,GIK_VIS);
  if(obj == GIK_VIS_OBJ_EXT)
    fl_set_slider_value(GIK_VIS_OBJ,GIK_VIS);
}

void CB_gik_precision_obj(FL_OBJECT *obj, long arg)
{
  GIK_PRECISION = fl_get_slider_value(obj);
  if(obj == GIK_PRECISION_OBJ)
    fl_set_slider_value(GIK_PRECISION_OBJ_EXT,GIK_PRECISION);
  if(obj == GIK_PRECISION_OBJ_EXT)
    fl_set_slider_value(GIK_PRECISION_OBJ,GIK_PRECISION);
}

void CB_gik_step_obj(FL_OBJECT *obj, long arg)
{
  GIK_STEP = fl_get_slider_value(obj);
  if(obj == GIK_STEP_OBJ)
    fl_set_slider_value(GIK_STEP_OBJ_EXT,GIK_STEP);
  if(obj == GIK_STEP_OBJ_EXT)
    fl_set_slider_value(GIK_STEP_OBJ,GIK_STEP);
}



/*------------------ FORM SPECIFIC UTILITY FUNCTIONS ------------------*/

static void gik_fill_defaultgiktask(void)
{
  int i;

  strcpy(PDGIKTASK[0].name,"Look at");
  PDGIKTASK[0].default_joints[0] = 16;
  PDGIKTASK[0].default_joints[1] = 17;
  PDGIKTASK[0].active_joint = 18; /* active joint */
  PDGIKTASK[0].default_joints_no = 2;

  strcpy(PDGIKTASK[1].name,"Left Hand Point at");
  PDGIKTASK[1].default_joints[0] = 32;
  PDGIKTASK[1].default_joints[1] = 33;
  PDGIKTASK[1].default_joints[2] = 34;
  PDGIKTASK[1].default_joints[3] = 35;
  PDGIKTASK[1].default_joints[4] = 36;
  PDGIKTASK[1].default_joints[5] = 37;
  PDGIKTASK[1].active_joint = 45; /* active joint */
  PDGIKTASK[1].default_joints_no = 6;

  strcpy(PDGIKTASK[2].name,"Right Hand Point at");
  PDGIKTASK[2].default_joints[0] = 19;
  PDGIKTASK[2].default_joints[1] = 20;
  PDGIKTASK[2].default_joints[2] = 21;
  PDGIKTASK[2].default_joints[3] = 22;
  PDGIKTASK[2].default_joints[4] = 23;
  PDGIKTASK[2].default_joints[5] = 24;
  PDGIKTASK[2].active_joint = 46; /* active joint */
  PDGIKTASK[2].default_joints_no = 6;

  strcpy(PDGIKTASK[3].name,"Left Hand Reach");
  PDGIKTASK[3].default_joints[0] = 32;
  PDGIKTASK[3].default_joints[1] = 33;
  PDGIKTASK[3].default_joints[2] = 34;
  PDGIKTASK[3].default_joints[3] = 35;
  PDGIKTASK[3].default_joints[4] = 36;
  PDGIKTASK[3].default_joints[5] = 37;
  PDGIKTASK[3].active_joint = 47; /* active joint */
  PDGIKTASK[3].default_joints_no = 6;

  strcpy(PDGIKTASK[4].name,"Right Hand Reach");
  PDGIKTASK[4].default_joints[0] = 19;
  PDGIKTASK[4].default_joints[1] = 20;
  PDGIKTASK[4].default_joints[2] = 21;
  PDGIKTASK[4].default_joints[3] = 22;
  PDGIKTASK[4].default_joints[4] = 23;
  PDGIKTASK[4].default_joints[5] = 24;
  PDGIKTASK[4].active_joint = 48; /* active joint */
  PDGIKTASK[4].default_joints_no = 6;

  /* Assigning default joints to actual joints */
  for(i=0; i<GIK_MAX_TASK_NO; i++){
    gik_reset_actual_joints(&PDGIKTASK[i]);
  }
}


static void gik_reset_actual_joints(PD_gik_task * PDGIKTASK)
{
  int i;

  for(i=0; i<PDGIKTASK->default_joints_no; i++){
    PDGIKTASK->actual_joints[i] = PDGIKTASK->default_joints[i];
  }
  PDGIKTASK->actual_joints_no = PDGIKTASK->default_joints_no;
}




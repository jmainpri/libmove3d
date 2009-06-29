#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#define GIK_MAX_JOINT_NO 50
#define GIK_MAX_TASK_NO 5

typedef struct {
	char name[64];
	int default_joints[GIK_MAX_JOINT_NO];
	int default_joints_no;
	int actual_joints[GIK_MAX_JOINT_NO];
	int actual_joints_no;
	int active_joint;
} PD_gik_task;

/* ----------- VARIABLES ------------- */
static int gik_active_task_no = 0;

/* ----------- FORM VARIABLES ------------- */
FL_FORM * GIK_FORM = NULL;

static FL_OBJECT * GIK_SELECTED_JOINTS_OBJ[GIK_MAX_JOINT_NO];
static FL_OBJECT * GIKTASKGROUP;
static FL_OBJECT * GIKTASK_ACTIVATE_OBJ[GIK_MAX_TASK_NO];
static FL_OBJECT * GIK_TASK_RESET;
static FL_OBJECT * GIK_ACTIVE_JOINTS_OBJ[GIK_MAX_TASK_NO];
/*	FL_OBJECT * neck1; // 16
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
 FL_OBJECT * rankle2; // 7 */


static PD_gik_task PDGIKTASK[GIK_MAX_TASK_NO];
//static int selected_joints[GIK_MAX_JOINT_NO]; /* Array containing a list of selected joint numbers */
//static int selected_joints_nb = 0;

static void g3d_create_gik_hrp2_jointsel_objects(void);
static void CB_gik_select_joint_obj(FL_OBJECT *obj, long arg);

static void g3d_create_gik_tasks(void);
static void CB_giktask_activate_obj(FL_OBJECT *obj, long arg);
static void CB_gik_reset_button_obj(FL_OBJECT *obj, long arg);

static void gik_fill_defaultgiktask(void);
static void gik_reset_actual_joints(PD_gik_task * PDGIKTASK);

void g3d_create_gik_jointsel_form ( void )
{	
	GIK_FORM = fl_bgn_form(FL_UP_BOX,400.0,400.0);
#ifdef HRP2
	g3d_create_gik_hrp2_jointsel_objects();
#else
	printf("Your robot is not supported by the GIK Interface.\n");
#endif

	g3d_create_gik_tasks();

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
/*	GIK_JOINTSEL_FORM_OBJ->neck1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,16);
  GIK_JOINTSEL_FORM_OBJ->neck2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy+20,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,17);
  GIK_JOINTSEL_FORM_OBJ->lshoulder1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+35,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,32);
	GIK_JOINTSEL_FORM_OBJ->lshoulder2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+40,hpy+35,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,33);
	GIK_JOINTSEL_FORM_OBJ->lshoulder3 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+45,hpy+55,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,34);
	GIK_JOINTSEL_FORM_OBJ->lelbow = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+50,hpy+85,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,35);
	GIK_JOINTSEL_FORM_OBJ->lwrist1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+50,hpy+115,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,36);
  GIK_JOINTSEL_FORM_OBJ->lwrist2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+50,hpy+135,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,37);
	GIK_JOINTSEL_FORM_OBJ->rshoulder1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+35,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,19);
	GIK_JOINTSEL_FORM_OBJ->rshoulder2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-40,hpy+35,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,20);
	GIK_JOINTSEL_FORM_OBJ->rshoulder3 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-45,hpy+55,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,21);
	GIK_JOINTSEL_FORM_OBJ->relbow = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-50,hpy+85,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,22);
	GIK_JOINTSEL_FORM_OBJ->rwrist1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-50,hpy+115,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,23);
  GIK_JOINTSEL_FORM_OBJ->rwrist2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-50,hpy+135,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,24);
	GIK_JOINTSEL_FORM_OBJ->waist1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy+60,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,14);
  GIK_JOINTSEL_FORM_OBJ->waist2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy+80,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,15);
  GIK_JOINTSEL_FORM_OBJ->lpelvis1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+10,hpy+105,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,8);
	GIK_JOINTSEL_FORM_OBJ->lpelvis2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+125,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,9);
	GIK_JOINTSEL_FORM_OBJ->lpelvis3 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+145,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,10);
	GIK_JOINTSEL_FORM_OBJ->lknee = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+175,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,11);
	GIK_JOINTSEL_FORM_OBJ->lankle1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+205,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,12);
  GIK_JOINTSEL_FORM_OBJ->lankle2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx+20,hpy+225,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,13);
	GIK_JOINTSEL_FORM_OBJ->rpelvis1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-10,hpy+105,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,2);
  GIK_JOINTSEL_FORM_OBJ->rpelvis2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+125,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,3);
  GIK_JOINTSEL_FORM_OBJ->rpelvis3 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+145,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,4);
  GIK_JOINTSEL_FORM_OBJ->rknee = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+175,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,5);
	GIK_JOINTSEL_FORM_OBJ->rankle1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+205,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,6);
  GIK_JOINTSEL_FORM_OBJ->rankle2 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx-20,hpy+225,30,30,"");
	fl_set_object_color(obj,FL_MCOL,FL_GREEN);
	fl_set_object_callback(obj,CB_gik_select_joint_obj,7); */
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
	int i;
	int xframe = 170, yframe = 15;
	
	fl_add_labelframe(FL_BORDER_FRAME,xframe,yframe,150,200,"Tasks");

  GIKTASKGROUP = fl_bgn_group();

	gik_fill_defaultgiktask();

	for(i=0; i<GIK_MAX_TASK_NO; i++){
		GIKTASK_ACTIVATE_OBJ[i] = fl_add_checkbutton(FL_RADIO_BUTTON,xframe+10,yframe+10+i*20,100,20,PDGIKTASK[i].name);
		fl_set_call_back(GIKTASK_ACTIVATE_OBJ[i],CB_giktask_activate_obj,i);
	}
	
	GIK_TASK_RESET = fl_add_button(FL_NORMAL_BUTTON,xframe+10,yframe+20+(i*20),130,25,"Revert to default Joints");
	fl_set_call_back(GIK_TASK_RESET,CB_gik_reset_button_obj,0);
	
	fl_end_group();
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
	PDGIKTASK[1].active_joint = 37; /* active joint */
	PDGIKTASK[1].default_joints_no = 6;
	
	strcpy(PDGIKTASK[2].name,"Right Hand Point at");
	PDGIKTASK[2].default_joints[0] = 19;
	PDGIKTASK[2].default_joints[1] = 20;
	PDGIKTASK[2].default_joints[2] = 21;
	PDGIKTASK[2].default_joints[3] = 22;
	PDGIKTASK[2].default_joints[4] = 23;
	PDGIKTASK[2].default_joints[5] = 24;
	PDGIKTASK[2].active_joint = 24; /* active joint */
	PDGIKTASK[2].default_joints_no = 6;
	
	strcpy(PDGIKTASK[3].name,"Left Hand Reach");
	PDGIKTASK[3].default_joints[0] = 32;
	PDGIKTASK[3].default_joints[1] = 33;
	PDGIKTASK[3].default_joints[2] = 34;
	PDGIKTASK[3].default_joints[3] = 35;
	PDGIKTASK[3].default_joints[4] = 36;
	PDGIKTASK[3].default_joints[5] = 37;
	PDGIKTASK[3].active_joint = 37; /* active joint */
	PDGIKTASK[3].default_joints_no = 6;
	
	strcpy(PDGIKTASK[4].name,"Right Hand Reach");
	PDGIKTASK[4].default_joints[0] = 19;
	PDGIKTASK[4].default_joints[1] = 20;
	PDGIKTASK[4].default_joints[2] = 21;
	PDGIKTASK[4].default_joints[3] = 22;
	PDGIKTASK[4].default_joints[4] = 23;
	PDGIKTASK[4].default_joints[5] = 24;
	PDGIKTASK[4].active_joint = 24; /* active joint */
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




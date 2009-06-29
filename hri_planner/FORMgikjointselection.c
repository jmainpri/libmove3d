#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#define GIK_MAX_JOINT_NO 30
#define GIK_MAX_TASK_NO 6

/**** Forms and Objects ****/
typedef struct {
	FL_FORM * joints;
	void * vdata;
	char * cdata;
	long  ldata;
#ifdef HRP2
	FL_OBJECT * neck1;
	FL_OBJECT * neck2;
	FL_OBJECT * lshoulder1;
	FL_OBJECT * lshoulder2;
	FL_OBJECT * lshoulder3;
	FL_OBJECT * lelbow;
	FL_OBJECT * lwrist1;
	FL_OBJECT * lwrist2;
	FL_OBJECT * rshoulder1;
	FL_OBJECT * rshoulder2;
	FL_OBJECT * rshoulder3;
	FL_OBJECT * relbow;
  FL_OBJECT * rwrist1;
	FL_OBJECT * rwrist2;
	FL_OBJECT * waist1;
	FL_OBJECT * waist2;
	FL_OBJECT * lpelvis1;
	FL_OBJECT * lpelvis2;
	FL_OBJECT * lpelvis3;
	FL_OBJECT * lknee;
	FL_OBJECT * lankle1;
	FL_OBJECT * lankle2;
	FL_OBJECT * rpelvis1;
	FL_OBJECT * rpelvis2;
	FL_OBJECT * rpelvis3;
	FL_OBJECT * rknee;
	FL_OBJECT * rankle1;
	FL_OBJECT * rankle2;
#endif
} FD_joints;
typedef struct {
	FL_OBJECT * TASK_OBJ;
	char name[64];
	int default_joints[GIK_MAX_JOINT_NO];
	int default_joints_no;
	int active_joint;
} FD_gik_task;

static FL_OBJECT * GIKTASKGROUPFR;
static FL_OBJECT * GIKTASKGROUP;
static FD_gik_task GIKTASK[GIK_MAX_TASK_NO];

FD_joints * GIK_JOINTSEL_FORM_OBJ;

static int selected_joints[GIK_MAX_JOINT_NO]; /* Array containing a list of selected joint numbers */
static int selected_joints_nb = 0;

static void g3d_create_gik_hrp2_jointsel_objects(void);
static void CB_gik_select_joint_obj(FL_OBJECT *obj, long arg);

static void g3d_create_gik_tasks(void);
static void gik_fill_defaultgiktask(void);

void g3d_create_gik_jointsel_form ( void )
{
	GIK_JOINTSEL_FORM_OBJ = MY_ALLOC(FD_joints,1);

	GIK_JOINTSEL_FORM_OBJ->joints = fl_bgn_form(FL_UP_BOX,400.0,400.0);

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
	fl_show_form(GIK_JOINTSEL_FORM_OBJ->joints,FL_PLACE_SIZE,TRUE, "GIK Joint Selection");
}

void g3d_hide_gik_jointsel_form ( void )
{
	fl_hide_form(GIK_JOINTSEL_FORM_OBJ->joints);
}

void g3d_delete_gik_jointsel_form ( void )
{
	fl_free_form(GIK_JOINTSEL_FORM_OBJ->joints);
}

/* ------------- This function is for HRP-2 only ------------------ */
static void g3d_create_gik_hrp2_jointsel_objects ( void )
{
	FL_OBJECT * obj;
	int xframe = 10, yframe = 15;
	int hpx = xframe+60, hpy = yframe+20;

	fl_add_labelframe(FL_BORDER_FRAME,xframe,yframe,150,280,"HRP-2 Joints Selection");

	/* hpx = 70, hpy = 35 */
	GIK_JOINTSEL_FORM_OBJ->neck1 = obj = fl_add_roundbutton(FL_PUSH_BUTTON,hpx,hpy,30,30,"");
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
	fl_set_object_callback(obj,CB_gik_select_joint_obj,7);
}

static void CB_gik_select_joint_obj(FL_OBJECT *obj, long arg)
{
	int i,temp;

	if(selected_joints_nb == 0){
		selected_joints[0] = arg;
		selected_joints_nb = 1;
		return;
	}

	for(i=0; i<selected_joints_nb; i++){
		if(selected_joints[i]>arg){
			for(;i<selected_joints_nb+1;i++){
				temp = selected_joints[i];
				selected_joints[i] = arg;
				arg = temp;
			}
			selected_joints_nb++;
			break;
		}
		if(selected_joints[i]==arg){
			for(;i<selected_joints_nb;i++){
				selected_joints[i] = selected_joints[i+1];
			}
			selected_joints_nb--;
			break;
		}
		if(i==selected_joints_nb-1){
			selected_joints[i+1] = arg;
			selected_joints_nb++;
			break;
		}
	}
	printf("selected joints:");
	for(i=0; i<selected_joints_nb; i++)	printf(" %d",selected_joints[i]);
	printf("\n");
}

static void g3d_create_gik_tasks(void)
{
	int i,j;
	int xframe = 170, yframe = 15;
	GIKTASKGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,xframe,yframe,150,200,"Tasks");

  GIKTASKGROUP = fl_bgn_group();

	gik_fill_defaultgiktask();

	for(i=0; i<GIK_MAX_TASK_NO-1; i++){
		GIKTASK[i].TASK_OBJ = fl_add_choice(FL_NORMAL_CHOICE,xframe+30,yframe+(i+1)*20,100,20,"--");
		for(j=0; j<GIK_MAX_TASK_NO; j++){
			fl_addto_choice(GIKTASK[i].TASK_OBJ, GIKTASK[j].name);
		}
		//fl_set_call_back(GIKTASK[i].TASK_OBJ,CB_gik_select_task,i);
	}
	fl_end_group();
}

static void gik_fill_defaultgiktask(void)
{
	strcpy(GIKTASK[0].name,"None");
	strcpy(GIKTASK[1].name,"Look at");
	GIKTASK[1].default_joints[0] = 16;
	GIKTASK[1].default_joints[1] = 17;
	GIKTASK[1].active_joint = 18; /* active joint */
	GIKTASK[1].default_joints_no = 2;
	strcpy(GIKTASK[2].name,"Left Hand Point at");
	strcpy(GIKTASK[3].name,"Right Hand Point at");
	strcpy(GIKTASK[4].name,"Left Hand Reach");
	strcpy(GIKTASK[5].name,"Right Hand Reach");
}



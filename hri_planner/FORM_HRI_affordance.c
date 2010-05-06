#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#define LOCAL_COMPUTATION_EPSILON (1e-9)
/* #define BH  Change this definition to JIDO, HRP2 or BH if you use these robots */


//extern int CALCULATE_AFFORDANCE;
FL_FORM  *HRI_AFFORDANCE_FORM = NULL;
extern FL_FORM  *PSP_PARAMETERS_FORM;
extern double PSP_PS_TRSHLD;

static FL_OBJECT  *BT_SHOW_DIRECT_REACHABILITY_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_BENDING_REACHABILITY_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_TURNING_AROUND_REACHABLE_OBJ; //AKP
static FL_OBJECT  *BT_CALCULATE_AFFORDANCE_OBJ;//AKP
static FL_OBJECT  *BT_HRP2_REACH_TARGET_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_HRP2_GIK_SOL_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_TURN_AROUND_REACH_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_DIRECT_REACH_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_2D_DIRECT_REACH_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_VISIBILE_PLACE_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_HRP2_HUM_COMMON_VISIBLE_OBJ;
static FL_OBJECT  *BT_SHOW_3D_HRP2_HUM_COMMON_REACH_OBJ;
static FL_OBJECT  *BT_SHOW_2D_HRP2_HUM_COMMON_VISIBLE_OBJ;
static FL_OBJECT  *BT_SHOW_2D_HRP2_HUM_COMMON_REACH_OBJ;
static FL_OBJECT  *BT_HRP2_PUT_OBJECT_OBJ;


static FL_OBJECT  *BT_SHOW_3D_DIRECT_REACH_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_BENDING_REACH_HUM_OBJ;

static FL_OBJECT  *BT_SHOW_OBSTACLE_CELLS_OBJ;
static FL_OBJECT  *BT_CREATE_HRP2_ROBOT_OBJ;
static FL_OBJECT  *BT_UPDATE_HRP2_STATE_OBJ;

int CALCULATE_AFFORDANCE=0;
int SHOW_HRP2_GIK_SOL=0;
extern int PERSPECTIVE_WIN_ENABLED;


int Affordances_Found=0; 
int SHOW_2D_COMMON_REACH_HRP2_HUMAN=0;
int SHOW_3D_COMMON_REACH_HRP2_HUMAN=0;
int SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=0;
int SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=0;
int SHOW_HRP2_HUMAN_COMMON_REACHABLE_VISIBLE=0;

int SHOW_2D_BENDING_REACHABLE_HUM=0;
int SHOW_3D_BENDING_REACHABLE_HUM=0;
int SHOW_2D_DIRECT_REACHABLE_HUM=0;
int SHOW_3D_DIRECT_REACHABLE_HUM=0;
int SHOW_2D_DIRECT_REACHABLE_HRP2=0;
int SHOW_3D_DIRECT_REACHABLE_HRP2=0;
int SHOW_2D_VISIBLE_PLACES_FOR_HRP2=0;
int SHOW_3D_VISIBLE_PLACES_FOR_HRP2=0;
int SHOW_2D_VISIBLE_PLACE_HUM=0;
int SHOW_3D_VISIBLE_PLACE_HUM=0;
int SHOW_2D_TURNING_AROUND_REACHABLE_HUM=0;
int SHOW_3D_TURNING_AROUND_REACHABLE_HUM=0;
int SHOW_OBSTACLE_CELLS=0;
int SHOW_2D_VISIBLE_PLACE_STANDING_HUM=0;
int SHOW_3D_VISIBLE_PLACE_STANDING_HUM=0;

extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting
int CANDIDATE_POINTS_FOR_TASK_FOUND=0;
extern int HRP2_CURRENT_TASK;//1 for take object, 2 for put object, 3 for return to rest position

/*static void fct_draw(void)
{
  if(G3D_DRAW_GRAPH) 
    g3d_draw_allwin_active();
} 
*/

#ifdef USE_HRP2_GIK
static void CB_create_HRP2_robot_obj(FL_OBJECT *ob, long arg)
{
 
 //////////create_HRP2_robot(HRP2_CURRENT_STATE);
 create_HRP2_robot_for_GIK_in_Move3d(HRP2_CURRENT_STATE);

 fl_check_forms();
 g3d_draw_allwin_active();

}
#endif

static void CB_update_HRP2_state_obj(FL_OBJECT *ob, long arg)
{
 if(HRP2_CURRENT_STATE==1)//Sitting
 HRP2_CURRENT_STATE=2;//Half sitting
 else
 HRP2_CURRENT_STATE=1;


 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_obstacle_cells_obj(FL_OBJECT *ob, long arg)
{
 if(SHOW_OBSTACLE_CELLS==0)
 SHOW_OBSTACLE_CELLS=1;
 else
 SHOW_OBSTACLE_CELLS=0;

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_bending_reach_hum_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_BENDING_REACHABLE_HUM==1)
  SHOW_2D_BENDING_REACHABLE_HUM=0;
  else
  SHOW_2D_BENDING_REACHABLE_HUM=1;
 break;

 case 1:
  if(SHOW_3D_BENDING_REACHABLE_HUM==1)
  SHOW_3D_BENDING_REACHABLE_HUM=0;
  else
  SHOW_3D_BENDING_REACHABLE_HUM=1;
 break;
  
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_dir_reach_active_obj(FL_OBJECT *ob, long arg)
{
 switch (arg)
 {
 case 0:
  if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
  SHOW_2D_DIRECT_REACHABLE_HUM=0;
  else
  SHOW_2D_DIRECT_REACHABLE_HUM=1;
 break;
   
 case 1:
  if(SHOW_3D_DIRECT_REACHABLE_HUM==1)
  SHOW_3D_DIRECT_REACHABLE_HUM=0;
  else
  SHOW_3D_DIRECT_REACHABLE_HUM=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_dir_reach_HRP2_active_obj(FL_OBJECT *ob, long arg)
{
 switch (arg)
 {
 case 0:
  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
  SHOW_2D_DIRECT_REACHABLE_HRP2=0;
  else
  SHOW_2D_DIRECT_REACHABLE_HRP2=1;
 break;
   
 case 1:
  if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
  SHOW_3D_DIRECT_REACHABLE_HRP2=0;
  else
  SHOW_3D_DIRECT_REACHABLE_HRP2=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_show_visible_place_HRP2_active_obj(FL_OBJECT *ob, long arg)
{
 switch (arg)
 {
 case 0:
  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
  SHOW_2D_VISIBLE_PLACES_FOR_HRP2=0;
  else
  SHOW_2D_VISIBLE_PLACES_FOR_HRP2=1;
 break;
   
 case 1:
  if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)
  SHOW_3D_VISIBLE_PLACES_FOR_HRP2=0;
  else
  SHOW_3D_VISIBLE_PLACES_FOR_HRP2=1;
 break;
 }
 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_visible_place_hum_active_obj(FL_OBJECT *ob, long arg)
{

 switch(arg)
 {
 case 0:
  if(SHOW_2D_VISIBLE_PLACE_HUM==1)
  SHOW_2D_VISIBLE_PLACE_HUM=0;
  else
  SHOW_2D_VISIBLE_PLACE_HUM=1;
 break;
 
 case 1:
  if(SHOW_3D_VISIBLE_PLACE_HUM==1)
  SHOW_3D_VISIBLE_PLACE_HUM=0;
  else
  SHOW_3D_VISIBLE_PLACE_HUM=1;
 break;

 case 2:
  if(SHOW_2D_VISIBLE_PLACE_STANDING_HUM==1)
  SHOW_2D_VISIBLE_PLACE_STANDING_HUM=0;
  else
  SHOW_2D_VISIBLE_PLACE_STANDING_HUM=1;
 break;

 case 3:
  if(SHOW_3D_VISIBLE_PLACE_STANDING_HUM==1)
  SHOW_3D_VISIBLE_PLACE_STANDING_HUM=0;
  else
  SHOW_3D_VISIBLE_PLACE_STANDING_HUM=1;
 break;

 }
 
 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_turn_around_reach_place_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
  SHOW_2D_TURNING_AROUND_REACHABLE_HUM=0;
  else
  SHOW_2D_TURNING_AROUND_REACHABLE_HUM=1;
 break;
 case 1:
  if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM==1)
  SHOW_3D_TURNING_AROUND_REACHABLE_HUM=0;
  else
  SHOW_3D_TURNING_AROUND_REACHABLE_HUM=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_common_reach_HRP2_hum_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_COMMON_REACH_HRP2_HUMAN==1)
  SHOW_2D_COMMON_REACH_HRP2_HUMAN=0;
  else
  SHOW_2D_COMMON_REACH_HRP2_HUMAN=1;
 break;
 case 1:
  if(SHOW_3D_COMMON_REACH_HRP2_HUMAN==1)
  SHOW_3D_COMMON_REACH_HRP2_HUMAN=0;
  else
  SHOW_3D_COMMON_REACH_HRP2_HUMAN=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_show_common_visible_HRP2_hum_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN==1)
  SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=0;
  else
  SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=1;
 break;
 case 1:
  if(SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN==1)
  SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=0;
  else
  SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}



static void CB_calculate_affordance_active_obj(FL_OBJECT *ob, long arg)
{
 
//// if(CALCULATE_AFFORDANCE==1)
//// {
//// CALCULATE_AFFORDANCE=0;
//// Affordances_Found=0; 
//// }
//// else
//// {
 //////////update_human_state(1);
 //////////virtually_update_human_state_new(1);
 //////find_affordance();
 find_affordance_new();
 CALCULATE_AFFORDANCE=1;
 //g3d_draw_env();
 
 ////find_affordance();
 ////find_human_affordance();
 
 g3d_draw_env();
Affordances_Found=1; 
 ////fl_check_forms();
 ////g3d_draw_allwin_active();

   ////show_affordance();
    
//// }

 fl_check_forms();
 g3d_draw_allwin_active();
}

#ifdef USE_HRP2_GIK
static void CB_show_hrp2_gik_sol_obj(FL_OBJECT *ob, long arg)
{
 if(SHOW_HRP2_GIK_SOL==0)
 {
  SHOW_HRP2_GIK_SOL=1;
  show_gik_sol();
 }
 else
 SHOW_HRP2_GIK_SOL=0;
}
#endif

#ifdef USE_HRP2_GIK
static void CB_hrp2_reach_target_obj(FL_OBJECT *ob, long arg)
{
 int hand=2; //1 for left, 2 for right
 M3D_GIK_TEST(hand);
 return;
}
#endif

#ifdef USE_HRP2_GIK
static void CB_hrp2_put_object_obj(FL_OBJECT *ob, long arg)
{
/*
////********tmp testing for look at loop
p3d_vector3 point_to_look, prev_point_to_look;
int ctr=0;
 //////create_HRP2_look_at_constraint();
while(1)
 {
    point_to_look[0] = ACBTSET->object->joints[1]->abs_pos[0][3];
    point_to_look[1] = ACBTSET->object->joints[1]->abs_pos[1][3];
    point_to_look[2] = ACBTSET->object->joints[1]->abs_pos[2][3];

int use_body_part=1;//0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet
  if(ctr==0)
  {
 
int look_res=HRP2_look_at_point(point_to_look, use_body_part);
prev_point_to_look[0]=point_to_look[0];
prev_point_to_look[1]=point_to_look[1];
prev_point_to_look[2]=point_to_look[2];
execute_current_HRP2_GIK_solution(0);
ctr++;
  }
  else
  {
   if(fabs(prev_point_to_look[0]-point_to_look[0])<0.05&&fabs(prev_point_to_look[1]-point_to_look[1])<0.05&&fabs(prev_point_to_look[2]-point_to_look[2])<0.05)
   {
    
   }
   else
   {
   int look_res=HRP2_look_at_point(point_to_look, use_body_part);
   prev_point_to_look[0]=point_to_look[0];
   prev_point_to_look[1]=point_to_look[1];
   prev_point_to_look[2]=point_to_look[2];
   execute_current_HRP2_GIK_solution(0);
   
   } 
   ////ctr++;
  }
fl_check_forms();
 }

 /////delete_HRP2_look_at_constraint();
return ;
////*******END tmp testing for look at loop
*/

 ////put_object_for_human_to_take();

//////////int hand_by_reach=2;

int for_hand=2;//1 for left hand, 2 for right hand

double *hand_pos;


hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

printf(" Before calling HRP2_look_at_bottle(), hand_pos=(%lf, %lf, %lf)\n",hand_pos[0],hand_pos[1], hand_pos[2]);
HRP2_look_at_bottle();

hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
printf(" After calling HRP2_look_at_bottle(), hand_pos=(%lf, %lf, %lf)\n",hand_pos[0],hand_pos[1], hand_pos[2]);

execute_current_HRP2_GIK_solution(0);


hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
printf(" After execute_current_HRP2_GIK_solution, hand_pos=(%lf, %lf, %lf)\n",hand_pos[0],hand_pos[1], hand_pos[2]);



////return;
for_hand=2;//1 for left hand, 2 for right hand
int res=HRP2_find_collision_free_path_to_take_object_new();
////////int res=HRP2_find_collision_free_path_to_take_object();
if(res==0)
return; 
execute_current_HRP2_GIK_solution(0);


double hand_clench_val=0.4;
printf(" Before calling HRP2_grasp_object\n");
HRP2_grasp_object(for_hand,hand_clench_val);
printf(" After calling HRP2_grasp_object, now calling execute_current_HRP2_GIK_solution\n");
execute_current_HRP2_GIK_solution(0);


////////HRP2_put_object_for_human_to_take();
/////////HRP2_show_object_to_human();
/////////HRP2_hide_object_from_human();


res=HRP2_put_object_for_human_to_take_new();

/////////res=HRP2_hide_object_from_human_new();
/////////res=HRP2_show_object_to_human_new();
if(res==0)
return;

CANDIDATE_POINTS_FOR_TASK_FOUND=1;
execute_current_HRP2_GIK_solution(1);//1 because need to move the bottle also


/*
hand_clench_val=0.3;
HRP2_release_object(for_hand,hand_clench_val);
execute_current_HRP2_GIK_solution(0);
*/
/*
HRP2_return_hand_to_rest_position();
execute_current_HRP2_GIK_solution(0);
*/
}
#endif


static void g3d_create_show_obstacle_cells_obj(void)
{
 BT_SHOW_OBSTACLE_CELLS_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,400,550,50,20,"Show Obstacle cells");
	
  fl_set_call_back(BT_SHOW_OBSTACLE_CELLS_OBJ,CB_show_obstacle_cells_obj,0);

}

static void g3d_create_show_2D_visible_place_hum_obj(void)
{
 BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,50,50,20,"Show Visible Places for Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ,CB_show_visible_place_hum_active_obj,0);

}

static void g3d_create_show_3D_visible_place_hum_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,70,50,20,"Show Visible Places for Human in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_HUM_OBJ,CB_show_visible_place_hum_active_obj,1);

}

static void g3d_create_show_2D_visible_place_standing_hum_obj(void)
{
 BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,260,50,50,20,"Show Visible Places for Standing Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM_OBJ,CB_show_visible_place_hum_active_obj,2);

}


static void g3d_create_show_3D_visible_place_standing_hum_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,260,70,50,20,"Show Visible Places for Standing Human in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM_OBJ,CB_show_visible_place_hum_active_obj,3);

}

static void g3d_create_show_dir_reach_obj(void)
{
 BT_SHOW_DIRECT_REACHABILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,90,50,20,"Show Human Direct Reachability on Plane");
	
  fl_set_call_back(BT_SHOW_DIRECT_REACHABILITY_OBJ,CB_show_dir_reach_active_obj,0);

}

static void g3d_create_show_3D_dir_reach_hum_obj(void)
{
 BT_SHOW_3D_DIRECT_REACH_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,110,50,20,"Show Human Direct Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_DIRECT_REACH_HUM_OBJ,CB_show_dir_reach_active_obj,1);

}


static void g3d_create_show_bending_reach_obj(void)
{
 BT_SHOW_BENDING_REACHABILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,130,50,20,"Show Human Bending Reachability on Plane");
	
  fl_set_call_back(BT_SHOW_BENDING_REACHABILITY_OBJ,CB_show_bending_reach_hum_active_obj,0);

}



static void g3d_create_show_3D_bending_reach_hum_obj(void)
{
  BT_SHOW_3D_BENDING_REACH_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,150,50,20,"Show Human Bending Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_BENDING_REACH_HUM_OBJ,CB_show_bending_reach_hum_active_obj,1);

}

static void g3d_create_show_turn_around_reach_place_obj(void)
{
 BT_SHOW_TURNING_AROUND_REACHABLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,170,50,20,"Show Human Turn Around Reachability on Plane ");
	
  fl_set_call_back(BT_SHOW_TURNING_AROUND_REACHABLE_OBJ,CB_show_turn_around_reach_place_active_obj,0);

}


static void g3d_create_show_3D_turn_around_reach_hum_obj(void)
{
  BT_SHOW_3D_TURN_AROUND_REACH_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,190,50,20,"Show Human Turn Around Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_TURN_AROUND_REACH_HUM_OBJ,CB_show_turn_around_reach_place_active_obj,1);

}


static void g3d_create_show_2D_visible_place_HRP2_obj(void)
{
 BT_SHOW_VISIBILE_PLACE_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,210,50,20,"Show Visible Places for HRP2 on planes");
	
  fl_set_call_back(BT_SHOW_VISIBILE_PLACE_HRP2_OBJ,CB_show_visible_place_HRP2_active_obj,0);

}

static void g3d_create_show_3D_visible_place_HRP2_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,230,50,20,"Show Visible Places for HRP2 in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_HRP2_OBJ,CB_show_visible_place_HRP2_active_obj,1);

} 


static void g3d_create_show_dir_reach_HRP2_obj(void)
{
 BT_SHOW_2D_DIRECT_REACH_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,250,50,20,"Show Direct Reachability of HRP2 on planes");
	
  fl_set_call_back(BT_SHOW_2D_DIRECT_REACH_HRP2_OBJ,CB_show_dir_reach_HRP2_active_obj,0);

}

static void g3d_create_show_3D_dir_reach_HRP2_obj(void)
{
 BT_SHOW_3D_DIRECT_REACH_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,270,50,20,"Show Direct Reachability of HRP2 in 3D");
	
  fl_set_call_back(BT_SHOW_3D_DIRECT_REACH_HRP2_OBJ,CB_show_dir_reach_HRP2_active_obj,1);

}

static void g3d_show_2D_HRP2_hum_common_visible_obj(void)
{
 BT_SHOW_2D_HRP2_HUM_COMMON_VISIBLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,290,50,20,"Show Common Visibility for HRP2 and Human on Plane");
	
 fl_set_call_back(BT_SHOW_2D_HRP2_HUM_COMMON_VISIBLE_OBJ,CB_show_common_visible_HRP2_hum_active_obj,0);

}

static void g3d_show_3D_HRP2_hum_common_visible_obj(void)
{
 BT_SHOW_3D_HRP2_HUM_COMMON_VISIBLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,310,50,20,"Show Common Visibility for HRP2 and Human in 3D");
	
 fl_set_call_back(BT_SHOW_3D_HRP2_HUM_COMMON_VISIBLE_OBJ,CB_show_common_visible_HRP2_hum_active_obj,1);

}

static void g3d_show_2D_HRP2_hum_common_reachable_obj(void)
{
 BT_SHOW_2D_HRP2_HUM_COMMON_REACH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,330,50,20,"Show Common Reachability for HRP2 and Human on Plane");
	
 fl_set_call_back(BT_SHOW_2D_HRP2_HUM_COMMON_REACH_OBJ,CB_show_common_reach_HRP2_hum_active_obj,0);

}

static void g3d_show_3D_HRP2_hum_common_reachable_obj(void)
{
 BT_SHOW_3D_HRP2_HUM_COMMON_REACH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,350,50,20,"Show Common Reachability for HRP2 and Human in 3D");
	
 fl_set_call_back(BT_SHOW_3D_HRP2_HUM_COMMON_REACH_OBJ,CB_show_common_reach_HRP2_hum_active_obj,1);

}

#ifdef USE_HRP2_GIK
static void g3d_create_HRP2_robot_obj(void)
{
 BT_CREATE_HRP2_ROBOT_OBJ=fl_add_button(FL_NORMAL_BUTTON,50,380,100,30,"Create HRP2 robot");
	
 fl_set_call_back(BT_CREATE_HRP2_ROBOT_OBJ,CB_create_HRP2_robot_obj,0);
 
}
#endif
  
#ifdef USE_HRP2_GIK
static void g3d_update_HRP2_state_obj(void)
{
 BT_UPDATE_HRP2_STATE_OBJ=fl_add_checkbutton(FL_PUSH_BUTTON,200,380,50,20,"Change HRP2 state as Half Sitting");
	
 fl_set_call_back(BT_UPDATE_HRP2_STATE_OBJ,CB_update_HRP2_state_obj,0);
 
}
#endif

static void g3d_create_calculate_affordance_obj(void)
{
 BT_CALCULATE_AFFORDANCE_OBJ = fl_add_button(FL_NORMAL_BUTTON,50,410,150,30,"Calculate Various Affordance");
	
  fl_set_call_back(BT_CALCULATE_AFFORDANCE_OBJ,CB_calculate_affordance_active_obj,0);

}

#ifdef USE_HRP2_GIK
void g3d_create_hrp2_reach_target_obj(void)
{
BT_HRP2_REACH_TARGET_OBJ = fl_add_button(FL_NORMAL_BUTTON,50,450,100,35,"RUN HRP2 GIK");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HRP2_REACH_TARGET_OBJ,CB_hrp2_reach_target_obj,0);
}
#endif

#ifdef USE_HRP2_GIK
void g3d_create_show_hrp2_gik_sol_obj(void)
{
BT_SHOW_HRP2_GIK_SOL_OBJ=fl_add_button(FL_NORMAL_BUTTON,200,450,150,35,"Show HRP2 gik solution");
fl_set_call_back(BT_SHOW_HRP2_GIK_SOL_OBJ,CB_show_hrp2_gik_sol_obj,0);
}
#endif

#ifdef USE_HRP2_GIK
void g3d_create_put_object_obj(void)
{
BT_HRP2_PUT_OBJECT_OBJ = fl_add_button(FL_NORMAL_BUTTON,50,490,100,35,"Put Object");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HRP2_PUT_OBJECT_OBJ,CB_hrp2_put_object_obj,0);
       
}
#endif

void g3d_show_HRI_affordance_form(void)
{ 
  fl_show_form(HRI_AFFORDANCE_FORM,
	       FL_PLACE_SIZE,TRUE, "HRI Affordance");
}

void g3d_hide_HRI_affordance_form(void)
{ 
  fl_hide_form(HRI_AFFORDANCE_FORM);
}

void g3d_delete_HRI_affordance_form(void)
{

  ////g3d_delete_bitmap_init_obj();
  
  ////g3d_delete_motion_capture_obj();
  
  g3d_delete_psp_parameters_form();
	
  fl_free_form(HRI_AFFORDANCE_FORM);
}





/************************/
/* Option form creation */
/************************/
void g3d_create_HRI_affordance_form(void)
{
  HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,600.0,600.0);
	
  //g3d_create_find_model_q(); 
	
  //g3d_create_bitmap_init_obj();

  //////////g3d_create_hrp2_reach_target_obj();
  //////////g3d_create_show_hrp2_gik_sol_obj();
  

//For 2D Affordance analysis
  g3d_create_show_dir_reach_obj();
  g3d_create_show_bending_reach_obj();
  g3d_create_show_2D_visible_place_hum_obj();
  g3d_create_show_turn_around_reach_place_obj();
  g3d_create_calculate_affordance_obj();
  g3d_create_show_dir_reach_HRP2_obj();
  g3d_create_show_2D_visible_place_HRP2_obj();

//FOR 3D Affordance analysis
  g3d_create_show_3D_dir_reach_hum_obj();
  g3d_create_show_3D_bending_reach_hum_obj();
  g3d_create_show_3D_visible_place_hum_obj();
  //////////g3d_create_show_2D_visible_place_standing_hum_obj();
  //////////g3d_create_show_3D_visible_place_standing_hum_obj();
  g3d_create_show_3D_turn_around_reach_hum_obj();

  g3d_create_show_3D_dir_reach_HRP2_obj();
  //g3d_create_show_3d_bending_reach_HRP2_obj();
  g3d_create_show_3D_visible_place_HRP2_obj();
  //g3d_create_show_3d_turn_around_reach_HRP2_obj();
     

//FOR 2D decision making;
   g3d_show_2D_HRP2_hum_common_reachable_obj();
   g3d_show_2D_HRP2_hum_common_visible_obj();
   
//FOR 3D decision making;
   g3d_show_3D_HRP2_hum_common_reachable_obj();
   g3d_show_3D_HRP2_hum_common_visible_obj();

   

   //////////g3d_create_show_obstacle_cells_obj();

   #ifdef USE_HRP2_GIK
   //////////g3d_create_HRP2_robot_obj();
   //////////g3d_update_HRP2_state_obj();
   //////////g3d_create_put_object_obj();
   #endif

  fl_end_form();

  //g3d_create_psp_parameters_form();
       
}






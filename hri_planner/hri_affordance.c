#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Rrt-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"
#include "include/hri_bitmap_util.h"
#include "include/hri_bitmap_draw.h"
#include "include/hri_bitmap_cost.h"
#include "include/hri_bitmap_bin_heap.h"

//#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"

/* similar to M_SQRT2 in math.h*/
#ifndef M_SQRT3
#define M_SQRT3 1.732050807568877294
#endif

#ifndef M_SQRT5
#define M_SQRT5 2.236067977499789696 
#endif

//AKP 
extern hri_bitmapset * ACBTSET;
extern struct gik_solution curr_gik_sol; // To store the gik solution between two segments in the path. It will be reset with each call of HRP2_hand_reach
struct gik_solution HRP2_GIK_sol;// It will store entire configuration to reach from the start to the goal of the entire path
extern struct SOLUTION_CONFIGS_FOR_HRP2 cur_gik_sol_configs;//It will store the final set of configurations to be executed on HRP2 
//AKP
int PERSPECTIVE_WIN_ENABLED=0;
int ONLINE_TRACKING_FLAG=0;
// AKP for storing the information about surfaces in the environment
struct env_surfaces curr_surfaces_in_env; 
//struct surface_grid_cell surf_grid[100][100]; 
//int grid_i_max=0;// max 1st index of surf_grid
//int grid_j_max=0;// max 2nd index of surf_grid
double surf_grid_samp_rate=0.05;//0.05;//0.1;// Sampling rate of the surface for constructing the grid
//int current_surface_index; // index i of the curr_surfaces_in_env.flat_surf[i] of env_surfaces

extern int CALCULATE_AFFORDANCE;

extern int SHOW_2D_COMMON_REACH_HRP2_HUMAN;
extern int SHOW_3D_COMMON_REACH_HRP2_HUMAN;
extern int SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN;
extern int SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN;;
extern int SHOW_HRP2_HUMAN_COMMON_REACHABLE_VISIBLE;
extern ChppGikStandingRobot* attStandingRobot;

extern int SHOW_2D_BENDING_REACHABLE_HUM;
extern int SHOW_3D_BENDING_REACHABLE_HUM;
extern int SHOW_2D_DIRECT_REACHABLE_HUM;
extern int SHOW_3D_DIRECT_REACHABLE_HUM;
extern int SHOW_2D_DIRECT_REACHABLE_HRP2;
extern int SHOW_3D_DIRECT_REACHABLE_HRP2;
extern int SHOW_2D_VISIBLE_PLACES_FOR_HRP2;
extern int SHOW_3D_VISIBLE_PLACES_FOR_HRP2;
extern int SHOW_2D_VISIBLE_PLACE_HUM;
extern int SHOW_2D_VISIBLE_PLACE_STANDING_HUM;
extern int SHOW_3D_VISIBLE_PLACE_HUM;
extern int SHOW_3D_VISIBLE_PLACE_STANDING_HUM;
extern int SHOW_2D_TURNING_AROUND_REACHABLE_HUM;
extern int SHOW_3D_TURNING_AROUND_REACHABLE_HUM;

extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting

struct grid_3D grid_around_HRP2;
int HRP2_GIK_MANIP=0;// Just to set the type of the bitmap
int BT_AFFORDANCE_VISIBILITY=1;//For the bitmap which is used for calculating visibility on 3d grid

candidate_poins_for_task candidate_points_to_put;
candidate_poins_for_task candidate_points_to_show;
candidate_poins_for_task candidate_points_to_hide;
////point_co_ordi candidate_points_to_put[100];

int no_candidate_points_to_put=0;
int grid_3d_affordance_calculated=0;

p3d_env *envPt;

point_co_ordi FOV_end_point_vertices[1000][8];//For every set there will be 8 vertices in the order mentioned in the function gpsp_computeFrustumVertices() in the file g3d_draw_camera.c
int no_FOV_end_point_vertices=0;
p3d_matrix4 frustum_transformation_mat;

int HRP2_GIK_path_calculated=0;

point_co_ordi right_hand_rest_pos;//To store the position of the right hand in the rest position

point_co_ordi point_of_curr_collision;// To stoe current point in 3d for which GIK solution is having collision

extern unsigned int NO_DOF_HRP2;

extern double M3D_to_HRP2_GIK_sitting_Z_shift;//0.622+0.16;//0.510-0.112=0.398//This is the value which needs to be added to the M3D z value of robot to synchronize with the z value of HRP2_GIK because for M3D the z is the height of foot whereas for HRP2_GIK z is the height of the waist. 

int THUMB_UP_CONSTRAINT=0; 
int HRP2_CURRENT_TASK=0;//1 for take object, 2 for put object, 3 for return to rest position
int SKIP_FIRST_CONFIG=0;
extern double attSamplingPeriod; // //Set a sampling period: 5ms second is the value used on HRP2, It should be 5e-3
int HRP2_HAND_spline_path_calculated=0;

extern vectorN combined_weights;

extern  std::vector<CjrlGikStateConstraint*> state_constraint_tasks;

extern ChppGikPositionConstraint *pc;

point_co_ordi sphere_surface_pts[25000];
int no_sphere_surface_pts=0;

/*
int ROBOTj_RSHOULDER=19;//29;
   int ROBOTj_LSHOULDER=32;//42;
int ROBOTj_SHOULDER=4;//For Jido
*/

double z_val_of_grasp_point;

extern int CANDIDATE_POINTS_FOR_TASK_FOUND;

//To tmp store the current positions of the hand returned by GIK, remove this to save space and time
double HRP2_hand_pos_sequence[3000][3];
int no_HRP2_hand_pos=0;
point_co_ordi agent_eye_pos;//To store the eye position for calculating visibility, for debuging only

int HUMAN_HAS_MOVED=0;//For updating the mightability maps
int JIDO_HAS_MOVED=0;//For updating the mightability maps
int HRP2_HAS_MOVED=0;//For updating the mightability maps
int NEED_HUMAN_VISIBILITY_UPDATE=0; //For updating the mightability maps
int NEED_HRP2_VISIBILITY_UPDATE=0; //For updating the mightability maps
int NEED_JIDO_VISIBILITY_UPDATE=0; //For updating the mightability maps
////int NEED_HUMAN_CURRENT_VISIBILITY_UPDATE=0;// For updating the visibility from the current head orientation


//================================

int execute_Mightability_Map_functions()
{
 
   if(Affordances_Found==1)
   {
   update_robots_and_objects_status();
   int expansion=1;
   ////////update_3D_grid_for_Mightability_Maps_new(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP);
   
   update_Mightability_Maps();
   show_3d_grid_Bounding_box_for_HRP2_GIK();
   if(SHOW_OBSTACLE_CELLS==1)
   show_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP);
   ////show_3d_grid_for_HRP2_GIK();
   //show_3d_grid_affordances();  
   ////Affordances_Found=0;
/*
   int ROBOTj_RSHOULDER=19;
    double hum_R_shoulder_pos[3];
     hum_R_shoulder_pos[0]= ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_R_shoulder_pos[1] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_R_shoulder_pos[2] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
   g3d_drawDisc(hum_R_shoulder_pos[0], hum_R_shoulder_pos[1], hum_R_shoulder_pos[2], 0.05, 4, NULL);
  */ 
   ////g3d_drawDisc(point_to_look[0], point_to_look[1], point_to_look[2], 0.1, 4, NULL);
   }
   
   if(grid_3d_affordance_calculated==1)
   {
   show_3d_grid_affordances_new();  
   } 
    
    //show_HRP2_gik_sol(); 
  if(HRP2_GIK_path_calculated==1)
   {
   hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
   }

  if(HRP2_HAND_spline_path_calculated==1)
   {
   show_spline_path_for_HRP2_hand();
   }

  
   if(CANDIDATE_POINTS_FOR_TASK_FOUND==1)
   {
   show_weighted_candidate_points_to_put_obj();
   /////////show_weighted_candidate_points_to_show_obj();
   /////////show_weighted_candidate_points_to_hide_obj();
   } 

g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.02, 4, NULL);

}

void get_Frustum_Vertices(float l, float r, float b, float t, float n, float f)
{
point_co_ordi frustumVertices[8];
    float ratio;
    float farLeft;
    float farRight;
    float farBottom;
    float farTop;
	int  projectionMode = 0;
	// perspective mode
    if(projectionMode == 0)
        ratio     = f / n;
    // orthographic mode
    else
        ratio = 1;
    farLeft   = l * ratio;
    farRight  = r * ratio;
    farBottom = b * ratio;
    farTop    = t * ratio;
	
    // compute 8 vertices of the frustum
    // near top right
    frustumVertices[0].x = r;
    frustumVertices[0].y = t;
    frustumVertices[0].z = -n;
	
    // near top left
    frustumVertices[1].x = l;
    frustumVertices[1].y = t;
    frustumVertices[1].z = -n;
	
    // near bottom left
    frustumVertices[2].x = l;
    frustumVertices[2].y = b;
    frustumVertices[2].z = -n;
	
    // near bottom right
    frustumVertices[3].x = r;
    frustumVertices[3].y = b;
    frustumVertices[3].z = -n;
	
    // far top right
    frustumVertices[4].x = farRight;
    frustumVertices[4].y = farTop;
    frustumVertices[4].z = -f;
	
    // far top left
    frustumVertices[5].x = farLeft;
    frustumVertices[5].y = farTop;
    frustumVertices[5].z = -f;
	
    // far bottom left
    frustumVertices[6].x = farLeft;
    frustumVertices[6].y = farBottom;
    frustumVertices[6].z = -f;
	
    // far bottom right
    frustumVertices[7].x = farRight;
    frustumVertices[7].y = farBottom;
    frustumVertices[7].z = -f;
	
   
    //AKP: To store the frustumVertices
    int i=0;
    for(i=0;i<8&&no_FOV_end_point_vertices<1000;i++)
    {
     //printf(" get_Frustum_Vertices, i= %d\n",i);
     p3d_vector4 point, point2;
     point2[0] = frustumVertices[i].x;
     point2[1] = frustumVertices[i].y;
     point2[2] = frustumVertices[i].z;
     point2[3] = 1.0;
     //p3d_rob *rob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
     //p3d_matvec4Mult(rob->o[rob->cam_body_index]->jnt->abs_pos,point2,point);
     p3d_matvec4Mult(frustum_transformation_mat,point2,point);
     FOV_end_point_vertices[no_FOV_end_point_vertices][i].x=point[0];
     FOV_end_point_vertices[no_FOV_end_point_vertices][i].y=point[1];
     FOV_end_point_vertices[no_FOV_end_point_vertices][i].z=point[2];
    ////FOV_end_point_vertices[no_FOV_end_point_vertices][i].x=frustumVertices[i].x;
    ////FOV_end_point_vertices[no_FOV_end_point_vertices][i].y=frustumVertices[i].y;
    ////FOV_end_point_vertices[no_FOV_end_point_vertices][i].z=frustumVertices[i].z;
    }
    no_FOV_end_point_vertices++;
    //printf(" no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
}

int get_points_on_FOV_screen(p3d_rob *r)
{
  ////////printf("get_points_on_FOV_screen, r->cam_body_index =%d\n",r->cam_body_index);
  p3d_obj *objPt = r->o[r->cam_body_index]; 
  
  p3d_jnt *jntPt = objPt->jnt;
  //p3d_jnt *jntPt = r->joints[1];
  
  p3d_matrix4 mattemp,mattemp2;
  int i,j;
  switch ( r->cam_axe) {
  case 0:
    p3d_mat4Pos(mattemp,r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], M_PI-r->cam_tilt,r->cam_pan,0);
    break;
  case 1:
    p3d_mat4Pos(mattemp,r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], M_PI_2+r->cam_pan,0,-r->cam_tilt);
    p3d_mat4Pos(mattemp2,0,0,0, 0,0,M_PI_2);
    break;
  case 2:
    p3d_mat4Pos(mattemp,r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], 0, -M_PI_2-r->cam_pan, r->cam_tilt);
    break;
  default:
    break;
  }
  

  
  
  for(i=0 ; i<=3 ; i++){
    for(j=0 ; j<=3 ; j++){
      
      frustum_transformation_mat[i][j]=jntPt->abs_pos[i][j];
      
    }
  }
  //matrix[14]=0.0;
  //float degYang = (r->cam_v_angle * 180.0/M_PI);//good
  float degYang = (((r->cam_h_angle*2.0)/3.0) * 180.0/M_PI);
  
  
  ////g3d_draw_cone(r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], r->cam_min_range, r->cam_max_range, r->cam_v_angle, r->cam_h_angle, r->cam_axe, r->cam_pan, r->cam_tilt);
  
  p3d_matrix4 tmp_res;
  p3d_mat4Mult(frustum_transformation_mat,mattemp,tmp_res);
  p3d_mat4Copy(tmp_res,frustum_transformation_mat);
  
  if ( r->cam_axe==1)
    {
      
      p3d_mat4Mult(frustum_transformation_mat,mattemp2,tmp_res);
      p3d_mat4Copy(tmp_res,frustum_transformation_mat);
      ////p3d_matvec4Mult(frustum_trans_mat,mattemp2,frustum_trans_mat);
    }
  //perspectiveGL(degYang, r->cam_h_angle/r->cam_v_angle ,0.001, 8.0);//original
  
  
  ////perspectiveGL(degYang, 3.0/2.0 ,0.001, 2.0);
  
  GLdouble fovY=degYang; 
  GLdouble aspect=3.0/2.0;
  GLdouble zNear=0.001;
  GLdouble zFar=2.0; 
  GLdouble fW, fH;
  //	Note:	tan( double ) uses radians but OpenGL works in degrees so we convert
  //			degrees to radians by dividing by 360 then multiplying by pi.
  fH = tanf( (fovY / 2.0) / 180.0 * M_PI ) * zNear;	
  // Same as fH = tan( fovY / 360 * pi ) * zNear;
  //	Calculate the distance from 0 of the x clipping plane based on the aspect ratio.
  //fW = tanf( (aspect / 2.0) / 180.0 * M_PI ) * zNear;
  fW = fH * aspect;
  //printf("fh %f   fw %f\n",fH, fW);	
  //glFrustum( -fW, fW, -fH, fH, zNear, zFar );
  
  //glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
  //g3d_set_color_mat(Any,tBluev);
  get_Frustum_Vertices(-fW, fW, -fH, fH, zNear, zFar );	
  
  return 1;

}



int check_inside_polygon(int no_vertices, point_co_ordi *vertices, point_co_ordi point)//the order of vertices should be clockwise or counter clockwise 
{
 //Equation of one edge is (y-y1)-((y2-y1)/(x2-x1))*(x-x1)=0;
 int i=0;
 int ok=0;
 for(i=0;i<no_vertices;i++)
 {
  double sign_of_next_vertex=(vertices[(i+2)%no_vertices].y-vertices[i%no_vertices].y)-((vertices[(i+1)%no_vertices].y-vertices[i%no_vertices].y)/(vertices[(i+1)%no_vertices].x-vertices[i%no_vertices].x))/(vertices[(i+2)%no_vertices].x-vertices[(i)%no_vertices].x);
  double sign_of_point=(point.y-vertices[i%no_vertices].y)-((vertices[(i+1)%no_vertices].y-vertices[i%no_vertices].y)/(vertices[(i+1)%no_vertices].x-vertices[i%no_vertices].x))/(point.x-vertices[(i)%no_vertices].x);
  if((sign_of_next_vertex>0&&sign_of_point>0)||(sign_of_next_vertex<0&&sign_of_point<0))
  {
  ok=1;
  }
  else
  {
  ok=0;
  return 0;
  }
  
 }

 if(ok==1)
 return 1;

}

int is_point_visible_by_st_line(configPt from_point, configPt to_point, int visibility_test_for)// Finds if there exists a collision free straight line from from_point to to_point 
{
//printf(" Finding st line path from (%lf,%lf,%lf) to (%lf,%lf,%lf) \n",from_point[6],from_point[7], from_point[8], to_point[6], to_point[7], to_point[8]);

if(visibility_test_for==1)// 1 means test for human
 {
int ntest;
   p3d_localpath * localpath;

////p3d_col_activate_robots();
////p3d_col_activate_rob_env(ACBTSET->visball);
p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->human[ACBTSET->actual_human]->HumanPt);
p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->robot);
double v = p3d_get_env_dmax();
//printf(" current v = %lf \n",v);
p3d_set_env_dmax(0.05);


        if( (localpath = p3d_local_planner(ACBTSET->visball, from_point, to_point)) )
        { 
	if(p3d_unvalid_localpath_test(ACBTSET->visball, localpath, &ntest))
         {
          p3d_set_env_dmax(v);
	  destroy_list_localpath(ACBTSET->visball, localpath);
	  ////printf("\nThere exits no straight line path from (%lf, %lf) to (%lf, %lf)\n",from_point[6],from_point[7],to_point[6],to_point[7]);
          ////fflush(stdout);
	  return 0;
	 }
        }
      p3d_set_env_dmax(v);
      destroy_list_localpath(ACBTSET->visball, localpath);
    	p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->human[ACBTSET->actual_human]->HumanPt);
  ////printf("\nThere exits straight line path from (%lf, %lf) to (%lf, %lf)\n",from_point[6],from_point[7],to_point[6],to_point[7]);
      ////    fflush(stdout);
  return 1;
 }	
 if(visibility_test_for==2)// 2 means test for HRP2
 {
int ntest;
   p3d_localpath * localpath;

////p3d_col_activate_robots();
////p3d_col_activate_rob_env(ACBTSET->visball);
p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->robot);
double v = p3d_get_env_dmax();
//printf(" current v = %lf \n",v);
p3d_set_env_dmax(0.05);


        if( (localpath = p3d_local_planner(ACBTSET->visball, from_point, to_point)) )
        { 
	if(p3d_unvalid_localpath_test(ACBTSET->visball, localpath, &ntest))
         {
          p3d_set_env_dmax(v);
	  destroy_list_localpath(ACBTSET->visball, localpath);
	  ////printf("\nThere exits no straight line path from (%lf, %lf) to (%lf, %lf)",from_point[6],from_point[7],to_point[6],to_point[7]);
          ////fflush(stdout);
	  return 0;
	 }
        }
      p3d_set_env_dmax(v);
      destroy_list_localpath(ACBTSET->visball, localpath);
    	p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->robot);
  ////printf("\nThere exits straight line path from (%lf, %lf) to (%lf, %lf)",from_point[6],from_point[7],to_point[6],to_point[7]);
      ////    fflush(stdout);
  return 1;
 }	
}

int is_point_in_fov(p3d_rob* robot, p3d_vector4 p)
{
  
  p3d_rob* rtemp = PSP_ROBOT;
  PSP_ROBOT = robot;
  //p3d_vector4 objectCenter;
  //double tempAngH = robot->cam_h_angle;
  //double tempAngW = robot->cam_h_angle;
  
  //p3d_get_robot_center(object, objectCenter); 
  //robot->cam_h_angle = angleH;
  //robot->cam_v_angle = angleW;
  int plane;
  G3D_Window *win = g3d_get_win_by_name("Perspective");
  g3d_refresh_win(win);
  int is_in_FOV=0;
  for(plane = 0; plane < 6; plane++ ) // for all perspective window frustum planes
    {
      // if the point is in the negative side of the frustum plan means that it i maybe inside the frustum box
      if(win->vs.frustum[plane][0] * (p[0]) + win->vs.frustum[plane][1] * (p[1])
	 + win->vs.frustum[plane][2] * (p[2]) + win->vs.frustum[plane][3] > 0 ) 
	{
	  is_in_FOV=1;
	}
      else
	{
	  is_in_FOV= 0;
	  plane=1000;
	}
    }
  PSP_ROBOT = rtemp;
  
  
  return is_in_FOV;
}

int is_point_on_surface_visible(point_co_ordi point, int visibility_test_for) 
{
  if(visibility_test_for==1)// 1 means test for human
 {
  

    configPt hum_cur_pos  = p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[0][3];//hum_cur_pos[6];
    hum_head_pos[7] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[1][3];//hum_cur_pos[7];
    hum_head_pos[8] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[2][3]+0.1;
    
    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    point_to_look[6] = point.x;
    point_to_look[7] = point.y;
    point_to_look[8] = point.z;
    


  p3d_vector4 objCenter,robCenter;
  int i,j;
 
  p3d_vector4 pointHead,pointDisc, pointAhead;
  double cone_length=1.5;
  double disttocenter;
  //p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;

  p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
 
  //mocap_get_disc_point(pointDisc);
  psu_get_point_ahead_cam(currRob, cone_length, pointAhead); 

 
  //p3d_get_object_center(currRob->o[currRob->cam_body_index], pointHead);

  pointHead[0]=hum_head_pos[6];
  pointHead[1]=hum_head_pos[7];
  pointHead[2]=hum_head_pos[8];

  objCenter[0]=point_to_look[6];
  objCenter[1]=point_to_look[7];
  objCenter[2]=point_to_look[8];
  //lx1 = pointHead [0];
  //ly1 = pointHead [1];
  //lz1 = pointHead [2];
  //printf("point Head %f %f %f \n", pointHead [0],pointHead [1],pointHead [2] );
 // lx2 = pointAhead [0];
 // ly2 = pointAhead [1];
 // lz2 = pointAhead [2];
  //printf("point ahead %f %f %f \n", pointAhead [0],pointAhead [1],pointAhead [2] );

 // psp_deselect_all();
  //  sphereActive = 1;  
  //Static Obstacles
 
	//  p3d_get_object_center(o,objCenter);
	
	 // if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	 //   {
	     
		  
		  
	  

   //if (p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, M_PI/1.5, &disttocenter))
  //// printf("currRob->cam_h_angle=%lf\n",currRob->cam_h_angle);
   if(is_point_in_fov(currRob, objCenter)) //*****Commented temporary because not to restrict in FOV
   {
    
    int is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, visibility_test_for); 
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);
  
   return is_visible;
   }
   /*
   p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
   if(psp_is_object_visible(currRob, ACBTSET->visball, 80))
   return 1;
   else
   return 0;
   */
   /*else
   {
   p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);
   return 0; 
   }*/
 }

  if(visibility_test_for==2)// 2 means test for HRP2
 {
  
    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
    
    
    
    configPt rob_head_pos  = MY_ALLOC(double,ACBTSET->visball->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    p3d_get_robot_config_into(ACBTSET->visball,&rob_head_pos);

    rob_head_pos[6] = ACBTSET->robot->joints[ROBOTj_LOOK]->abs_pos[0][3];
    rob_head_pos[7] = ACBTSET->robot->joints[ROBOTj_LOOK]->abs_pos[1][3];//hum_cur_pos[7];
    rob_head_pos[8] = ACBTSET->robot->joints[ROBOTj_LOOK]->abs_pos[2][3];
    
    configPt point_to_look   = MY_ALLOC(double,ACBTSET->visball->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    p3d_get_robot_config_into(ACBTSET->visball,&point_to_look );
    
    point_to_look[6] = point.x;
    point_to_look[7] = point.y;
    point_to_look[8] = point.z;
    


  p3d_vector4 objCenter,robCenter;
  int i,j;
 
  p3d_vector4 pointHead,pointDisc, pointAhead;
  double cone_length=1.5;
  double disttocenter;
  //p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;

  p3d_rob * currRob=ACBTSET->robot;
 
  //mocap_get_disc_point(pointDisc);
  psu_get_point_ahead_cam(currRob, cone_length, pointAhead); 

 
  //p3d_get_object_center(currRob->o[currRob->cam_body_index], pointHead);

  pointHead[0]=rob_head_pos[6];
  pointHead[1]=rob_head_pos[7];
  pointHead[2]=rob_head_pos[8];

  objCenter[0]=point_to_look[6];
  objCenter[1]=point_to_look[7];
  objCenter[2]=point_to_look[8];
  //lx1 = pointHead [0];
  //ly1 = pointHead [1];
  //lz1 = pointHead [2];
  //printf("point Head %f %f %f \n", pointHead [0],pointHead [1],pointHead [2] );
 // lx2 = pointAhead [0];
 // ly2 = pointAhead [1];
 // lz2 = pointAhead [2];
  //printf("point ahead %f %f %f \n", pointAhead [0],pointAhead [1],pointAhead [2] );

 // psp_deselect_all();
  //  sphereActive = 1;  
  //Static Obstacles
 
	//  p3d_get_object_center(o,objCenter);
	
	 // if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	 //   {
	     
		  
		  
	  

   //if (p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, M_PI/1.5, &disttocenter))
   if(is_point_in_fov(currRob, objCenter))//*****Commented temporary because not to restrict in FOV
   {
    
    int is_visible=is_point_visible_by_st_line(rob_head_pos, point_to_look, visibility_test_for); 
    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
    MY_FREE(rob_head_pos, double,ACBTSET->visball->nb_dof); 
    MY_FREE(point_to_look, double,ACBTSET->visball->nb_dof); 

   
   return is_visible;
   }
   ////else
   ////return 0;
   /*else
   {
   p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);
   return 0; 
   }*/
 }
 

}

//double rob_shoulder_pos
int is_point_directly_reachable_by_HRP2(point_co_ordi point)
{
    int by_LHand=0;
    int by_RHand=0;
    double maxi_reach_dist=0.75;
    
   /*
    int ROBOTj_RSHOULDER=19;//29;
    int ROBOTj_LSHOULDER=32;//42;
   */

    //configPt hum_cur_pos  = p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
    configPt hum_shoulder_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    
    // Reachability Test by right hand
    hum_shoulder_pos[6] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[7] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[8] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    
    configPt point_to_reach = p3d_get_robot_config(ACBTSET->visball);  
    point_to_reach[6] = point.x;
    point_to_reach[7] = point.y;
    point_to_reach[8] = point.z;
    
    
    double dist=sqrt((hum_shoulder_pos[6]-point_to_reach[6])*(hum_shoulder_pos[6]-point_to_reach[6])+(hum_shoulder_pos[7]-point_to_reach[7])*(hum_shoulder_pos[7]-point_to_reach[7])+(hum_shoulder_pos[8]-point_to_reach[8])*(hum_shoulder_pos[8]-point_to_reach[8]));
    
    if(dist<maxi_reach_dist)
    by_RHand=1;
    

    // Reachability Test by left hand
    hum_shoulder_pos[6] = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[7] = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[8] =ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively

    dist=sqrt((hum_shoulder_pos[6]-point_to_reach[6])*(hum_shoulder_pos[6]-point_to_reach[6])+(hum_shoulder_pos[7]-point_to_reach[7])*(hum_shoulder_pos[7]-point_to_reach[7])+(hum_shoulder_pos[8]-point_to_reach[8])*(hum_shoulder_pos[8]-point_to_reach[8]));

    if(dist<maxi_reach_dist)
    by_LHand=1;
    
    p3d_destroy_config(ACBTSET->visball, hum_shoulder_pos);
    //p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_reach);    

    if(by_RHand==1&&by_LHand==1)
    return 3; // Reachable by both hands
    if(by_RHand==1)
    return 2; // reachable by right hand only
    if(by_LHand==1)
    return 1; // reachable by left hand only
    else
    return 0; // not reachable
 
}


int is_point_directly_reachable_by_human(point_co_ordi point)
{
    int by_LHand=0;
    int by_RHand=0;
    double maxi_reach_dist=0.5;
    

    //configPt hum_cur_pos  = p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
    configPt hum_shoulder_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    
    // Reachability Test by right hand
    hum_shoulder_pos[6] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[7] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[8] =ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    
    configPt point_to_reach = p3d_get_robot_config(ACBTSET->visball);  
    point_to_reach[6] = point.x;
    point_to_reach[7] = point.y;
    point_to_reach[8] = point.z;
    
    
    double dist=sqrt((hum_shoulder_pos[6]-point_to_reach[6])*(hum_shoulder_pos[6]-point_to_reach[6])+(hum_shoulder_pos[7]-point_to_reach[7])*(hum_shoulder_pos[7]-point_to_reach[7])+(hum_shoulder_pos[8]-point_to_reach[8])*(hum_shoulder_pos[8]-point_to_reach[8]));
    
    if(dist<maxi_reach_dist)
    by_RHand=1;
    

    // Reachability Test by left hand
    hum_shoulder_pos[6] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[7] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[8] =ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively

    dist=sqrt((hum_shoulder_pos[6]-point_to_reach[6])*(hum_shoulder_pos[6]-point_to_reach[6])+(hum_shoulder_pos[7]-point_to_reach[7])*(hum_shoulder_pos[7]-point_to_reach[7])+(hum_shoulder_pos[8]-point_to_reach[8])*(hum_shoulder_pos[8]-point_to_reach[8]));

    if(dist<maxi_reach_dist)
    by_LHand=1;
    
    p3d_destroy_config(ACBTSET->visball, hum_shoulder_pos);
    //p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_reach);    

    if(by_RHand==1&&by_LHand==1)
    return 3; // Reachable by both hands
    if(by_RHand==1)
    return 2; // reachable by right hand only
    if(by_LHand==1)
    return 1; // reachable by left hand only
    else
    return 0; // not reachable
 
}


int is_point_on_surface_directly_reachable(point_co_ordi point, int reachability_test_for)
{
 if(reachability_test_for==1)// 1 means test for human
 {
  int by_LHand=0;
    int by_RHand=0;
    double maxi_reach_dist=0.5;
    

    //configPt hum_cur_pos  = p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
    configPt hum_shoulder_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    
    // Reachability Test by right hand
    hum_shoulder_pos[6] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[7] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[8] =ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    
    configPt point_to_reach = p3d_get_robot_config(ACBTSET->visball);  
    point_to_reach[6] = point.x;
    point_to_reach[7] = point.y;
    point_to_reach[8] = point.z;
    
    
    double dist=sqrt((hum_shoulder_pos[6]-point_to_reach[6])*(hum_shoulder_pos[6]-point_to_reach[6])+(hum_shoulder_pos[7]-point_to_reach[7])*(hum_shoulder_pos[7]-point_to_reach[7])+(hum_shoulder_pos[8]-point_to_reach[8])*(hum_shoulder_pos[8]-point_to_reach[8]));
    
    if(dist<maxi_reach_dist)
    by_RHand=1;
    

    // Reachability Test by left hand
    hum_shoulder_pos[6] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[7] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    hum_shoulder_pos[8] =ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively

    dist=sqrt((hum_shoulder_pos[6]-point_to_reach[6])*(hum_shoulder_pos[6]-point_to_reach[6])+(hum_shoulder_pos[7]-point_to_reach[7])*(hum_shoulder_pos[7]-point_to_reach[7])+(hum_shoulder_pos[8]-point_to_reach[8])*(hum_shoulder_pos[8]-point_to_reach[8]));

    if(dist<maxi_reach_dist)
    by_LHand=1;
    
    p3d_destroy_config(ACBTSET->visball, hum_shoulder_pos);
    //p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_reach);    

    if(by_RHand==1&&by_LHand==1)
    return 3; // Reachable by both hands
    if(by_RHand==1)
    return 2; // reachable by right hand only
    if(by_LHand==1)
    return 1; // reachable by left hand only
    else
    return 0; // not reachable
    
 }

 if(reachability_test_for==2)// 2 means test for HRP2
 {
    int by_LHand=0;
    int by_RHand=0;
    int hand_by_reach=1;//1 for left hand, 2 for right hand
    int HRP2_state=HRP2_CURRENT_STATE; //1 for sitting on the chair, 2 for standing
    double maxi_reach_dist=0.5;
    p3d_vector3 target_in_global_frame;
    target_in_global_frame[0]=point.x;
    target_in_global_frame[1]=point.y;
    target_in_global_frame[2]=point.z;
    double task_duration=3.0;//in s
    int thumb_up_constraint=1;//Means the x axis of the hand will be made parallel to the z axis og global frame
    vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    by_LHand=HRP2_hand_reach(target_in_global_frame, hand_by_reach, task_duration, HRP2_state,thumb_up_constraint,use_body_part);
    //printf("Reachable by left hand=%d\n",by_LHand);
 
    int show_cur_gik_sol=0; 
    int i_sol_ctr=0;
   if(show_cur_gik_sol==1)
   {
    for(i_sol_ctr=0;i_sol_ctr<curr_gik_sol.no_configs;i_sol_ctr++)
    {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[i_sol_ctr]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }

    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);//Restore to initial position
    }
 
    attStandingRobot->staticState ( backupConfig );
    
    
  ////g3d_draw_env();
  fl_check_forms();
  ////g3d_draw_allwin_active();
    hand_by_reach=2;//For right hand
   //// by_RHand=HRP2_hand_reach(target_in_global_frame, hand_by_reach, task_duration, HRP2_state, thumb_up_constraint);
    //printf("Reachable by right hand=%d\n",by_RHand);  
  backupConfig = attStandingRobot->robot()->currentConfiguration();
   use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
   by_RHand=HRP2_hand_reach(target_in_global_frame, hand_by_reach, task_duration, HRP2_state, thumb_up_constraint, use_body_part);
    //printf("Reachable by right hand=%d\n",by_RHand);  

    //show_cur_gik_sol=1; 
    i_sol_ctr=0;
   if(show_cur_gik_sol==1)
   {
    for(i_sol_ctr=0;i_sol_ctr<curr_gik_sol.no_configs;i_sol_ctr++)
    {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[i_sol_ctr]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  ////g3d_draw_env();
  fl_check_forms();
  ////g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }

    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);//Restore to initial position
    }
 
    attStandingRobot->staticState ( backupConfig );
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
    
    if(by_RHand==1&&by_LHand==1)
    return 3; // Reachable by both hands
    if(by_RHand==1)
    return 2; // reachable by right hand only
    if(by_LHand==1)
    return 1; // reachable by left hand only
    else
    return 0; // not reachable
    
 }
}

int update_surface_grid_for_HRP2_without_GIK(double sampling_rate, flat_surface *cur_sur)
{
  configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos); 

 if(cur_sur->surface_shape==3) // rectangular surface
 {
//printf(" Inside update_surface_grid \n");
int cx,cy;
for(cx=0;cx<100;cx++)
for(cy=0;cy<100;cy++)
 {
cur_sur->surf_grid[cx][cy].visible_by_HRP2=-1;

 }

    p3d_traj_test_type tmp_col_type = p3d_col_env_get_traj_method();
    p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE);

//choose_test_traj=

  int gx=0;
  int gy=0;
  double i=cur_sur->BR_x_min;// min value of the bounding rectangle
  for(;i<=cur_sur->BR_x_max;i+=surf_grid_samp_rate)
  {
    gy=0;
   point_co_ordi point;
    point.x=i;
   double j=cur_sur->BR_y_min;
   for(;j<=cur_sur->BR_y_max;j+=surf_grid_samp_rate)
   {
    
    point.y=j;
    int inside=check_inside_polygon(4, cur_sur->vertices, point);//the order of vertices should be clockwise or counter clockwise 
    if(inside==1)
    {
     cur_sur->surf_grid[gx][gy].inside=1; // grid cell is inside the surface boundary   
   
    } 
    else
    {
     cur_sur->surf_grid[gx][gy].inside=-1; // even the cell is inside the bounding box, it is outside of the actual surface boundary
    } 
//   ACBTSET->actual_human=0;
   point.z=cur_sur->BR_z+0.05; // 0.05 To avoid collision with table top
   int is_visible=is_point_on_surface_visible(point, 2); // 2 means test for HRP2
    
    if(is_visible==1)
    {
     cur_sur->surf_grid[gx][gy].visible_by_HRP2=1;    
   
    } 
    else
    {
     //cur_sur->surf_grid[gx][gy].visible=-1;
    } 
   
   if(fabs(rob_cur_pos[6]-point.x)>1.5||fabs(rob_cur_pos[7]-point.y)>1.5||fabs(rob_cur_pos[8]-point.z)>1.5)   // Surely not reachable by HRP2 from its current position
   {
  
   }
   else
   {
   /////p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos); // Restore the robot's initial position before calculating the reachability for next point; 
   //////int state=1; //AKP : 1 is sitting, 2 is standing 
   //////create_HRP2_robot(state);//Everytime restore the original position of the robot
   //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
   
   ////int is_reachable_directly=is_point_on_surface_directly_reachable(point, 2); //*****NOTE:This function uses GIK to test reachability with HRP2, 2 means test for HRP2

   int is_reachable_directly=is_point_directly_reachable_by_HRP2(point);

    if(is_reachable_directly==3)
    {
     cur_sur->surf_grid[gx][gy].reachable_by_HRP2_RHand=1;    
     cur_sur->surf_grid[gx][gy].reachable_by_HRP2_LHand=1;    
    } 
    else
    {
     if(is_reachable_directly==2)
     {
     cur_sur->surf_grid[gx][gy].reachable_by_HRP2_RHand=1;    
   
     }
     else
     {
      if(is_reachable_directly==1)
      {
      cur_sur->surf_grid[gx][gy].reachable_by_HRP2_LHand=1;    
   
      }
      else
      {
      cur_sur->surf_grid[gx][gy].reachable_by_HRP2_LHand=-1;
      cur_sur->surf_grid[gx][gy].reachable_by_HRP2_RHand=-1;
      } 
     }
     
    } 
   }//End else
  

   gy++;
   }
   gx++;
  }
 cur_sur->grid_i_max=gx;
 cur_sur->grid_j_max=gy;
 
p3d_col_env_set_traj_method(tmp_col_type); //Restoring the actual collision check type
 }

 MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);

}

int update_surface_grid_for_HRP2_with_GIK(double sampling_rate, flat_surface *cur_sur)
{
  configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos); 

 if(cur_sur->surface_shape==3) // rectangular surface
 {
//printf(" Inside update_surface_grid \n");
int cx,cy;
for(cx=0;cx<100;cx++)
for(cy=0;cy<100;cy++)
 {
cur_sur->surf_grid[cx][cy].visible_by_HRP2=-1;

 }

    p3d_traj_test_type tmp_col_type = p3d_col_env_get_traj_method();
    p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE);

//choose_test_traj=

  int gx=0;
  int gy=0;
  double i=cur_sur->BR_x_min;// min value of the bounding rectangle
  for(;i<=cur_sur->BR_x_max;i+=surf_grid_samp_rate)
  {
    gy=0;
   point_co_ordi point;
    point.x=i;
   double j=cur_sur->BR_y_min;
   for(;j<=cur_sur->BR_y_max;j+=surf_grid_samp_rate)
   {
    
    point.y=j;
    int inside=check_inside_polygon(4, cur_sur->vertices, point);//the order of vertices should be clockwise or counter clockwise 
    if(inside==1)
    {
     cur_sur->surf_grid[gx][gy].inside=1; // grid cell is inside the surface boundary   
   
    } 
    else
    {
     cur_sur->surf_grid[gx][gy].inside=-1; // even the cell is inside the bounding box, it is outside of the actual surface boundary
    } 
//   ACBTSET->actual_human=0;
   point.z=cur_sur->BR_z+0.05; // 0.05 To avoid collision with table top
   int is_visible=is_point_on_surface_visible(point, 2); // 2 means test for HRP2
    
    if(is_visible==1)
    {
     cur_sur->surf_grid[gx][gy].visible_by_HRP2=1;    
   
    } 
    else
    {
     //cur_sur->surf_grid[gx][gy].visible=-1;
    } 
   
   if(fabs(rob_cur_pos[6]-point.x)>1.5||fabs(rob_cur_pos[7]-point.y)>1.5||fabs(rob_cur_pos[8]-point.z)>1.5)   // Surely not reachable by HRP2 from its current position
   {
  
   }
   else
   {
   /////p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos); // Restore the robot's initial position before calculating the reachability for next point; 
   //////int state=1; //AKP : 1 is sitting, 2 is standing 
   //////create_HRP2_robot(state);//Everytime restore the original position of the robot
   //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
   
   int is_reachable_directly=is_point_on_surface_directly_reachable(point, 2); // 2 means test for HRP2
   
    if(is_reachable_directly==3)
    {
     cur_sur->surf_grid[gx][gy].reachable_by_HRP2_RHand=1;    
     cur_sur->surf_grid[gx][gy].reachable_by_HRP2_LHand=1;    
    } 
    else
    {
     if(is_reachable_directly==2)
     {
     cur_sur->surf_grid[gx][gy].reachable_by_HRP2_RHand=1;    
   
     }
     else
     {
      if(is_reachable_directly==1)
      {
      cur_sur->surf_grid[gx][gy].reachable_by_HRP2_LHand=1;    
   
      }
      else
      {
      cur_sur->surf_grid[gx][gy].reachable_by_HRP2_LHand=-1;
      cur_sur->surf_grid[gx][gy].reachable_by_HRP2_RHand=-1;
      } 
     }
     
    } 
   }//End else
  

   gy++;
   }
   gx++;
  }
 cur_sur->grid_i_max=gx;
 cur_sur->grid_j_max=gy;
 
p3d_col_env_set_traj_method(tmp_col_type); //Restoring the actual collision check type
 }

 MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);

}

int update_surface_grid_based_on_curr_pos(double sampling_rate, flat_surface *cur_sur)
{
 if(cur_sur->surface_shape==3) // rectangular surface
 {
//printf(" Inside update_surface_grid \n");
int cx,cy;
for(cx=0;cx<100;cx++)
for(cy=0;cy<100;cy++)
 {
cur_sur->surf_grid[cx][cy].visible=-1;

 }

    p3d_traj_test_type tmp_col_type = p3d_col_env_get_traj_method();
    p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE);

//choose_test_traj=

  int gx=0;
  int gy=0;
  double i=cur_sur->BR_x_min;// min value of the bounding rectangle
  for(;i<=cur_sur->BR_x_max;i+=surf_grid_samp_rate)
  {
    gy=0;
   point_co_ordi point;
    point.x=i;
   double j=cur_sur->BR_y_min;
   for(;j<=cur_sur->BR_y_max;j+=surf_grid_samp_rate)
   {
    
    point.y=j;
    int inside=check_inside_polygon(4, cur_sur->vertices, point);//the order of vertices should be clockwise or counter clockwise 
    if(inside==1)
    {
     cur_sur->surf_grid[gx][gy].inside=1; // grid cell is inside the surface boundary   
   
    } 
    else
    {
     cur_sur->surf_grid[gx][gy].inside=-1; // even the cell is inside the bounding box, it is outside of the actual surface boundary
    } 
   ACBTSET->actual_human=0;
   point.z=cur_sur->BR_z+0.05; // 0.05 To avoid collision with table top
    int is_visible=is_point_on_surface_visible(point, 1); // 1 means test for human
    
    if(is_visible==1)
    {
     ////printf(" setting cur_sur->surf_grid[%d][%d].visible=1 \n",gx,gy);
     cur_sur->surf_grid[gx][gy].visible=1;    
   
    } 
    else
    {
     //cur_sur->surf_grid[gx][gy].visible=-1;
    } 

   
   int is_reachable_directly=is_point_on_surface_directly_reachable(point, 1); // 1 means test for human
    
    if(is_reachable_directly==3)
    {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand=1;    
     cur_sur->surf_grid[gx][gy].reachable_by_LHand=1;    
    } 
    else
    {
     if(is_reachable_directly==2)
     {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand=1;    
   
     }
     else
     {
      if(is_reachable_directly==1)
      {
      cur_sur->surf_grid[gx][gy].reachable_by_LHand=1;    
   
      }
      else
      {
      cur_sur->surf_grid[gx][gy].reachable_by_LHand=-1;
      cur_sur->surf_grid[gx][gy].reachable_by_RHand=-1;
      } 
     }
     
    } 
    
  

   gy++;
   }
   gx++;
  }
 cur_sur->grid_i_max=gx;
 cur_sur->grid_j_max=gy;
 
p3d_col_env_set_traj_method(tmp_col_type); //Restoring the actual collision check type
 }
}


int update_surface_grid_by_bending_human_at_curr_pos(double sampling_rate, flat_surface *cur_sur)
{
 if(cur_sur->surface_shape==3) // rectangular surface
 {

//printf(" Inside update_surface_grid_by_bending_human_at_curr_pos \n");
int cx,cy;
for(cx=0;cx<100;cx++)
for(cy=0;cy<100;cy++)
 {
cur_sur->surf_grid[cx][cy].reachable_by_LHand_by_bending=-1;
cur_sur->surf_grid[cx][cy].reachable_by_RHand_by_bending=-1;
 }
p3d_traj_test_type tmp_col_type = p3d_col_env_get_traj_method();
    p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE);

int collision =0 ;
configPt hum_tmp_pos=p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
double pitch_ang=hum_tmp_pos[14]; 
double orig_pitch_ang=pitch_ang;

for(;pitch_ang<M_PI/2.2&&collision==0;pitch_ang+=0.087)
 {
//printf(" pitch_ang = %lf\n",pitch_ang);
hum_tmp_pos[14]=pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[14]=hum_tmp_pos[14];
int kcd_with_report=0;


p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
int collision_test_res = p3d_col_test_robot(human,kcd_with_report);
////printf(" At pitch = %lf, The collision_test_res=%d \n", pitch_ang,collision_test_res);
 if(collision_test_res>0)
 {
 ////printf(" There is collision with human, collision_test_res=%d \n", collision_test_res);
 collision=1;
 }
 else
 {

  int gx=0;
  int gy=0;
  double i=cur_sur->BR_x_min;// min value of the bounding rectangle
  for(;i<=cur_sur->BR_x_max;i+=surf_grid_samp_rate)
  {
    gy=0;
   point_co_ordi point;
    point.x=i;
   double j=cur_sur->BR_y_min;
   for(;j<=cur_sur->BR_y_max;j+=surf_grid_samp_rate)
   {
      point.y=j;
    int inside=check_inside_polygon(4, cur_sur->vertices, point);//the order of vertices should be clockwise or counter clockwise 
    if(inside==1)
    {
     cur_sur->surf_grid[gx][gy].inside=1; // grid cell is inside the surface boundary   
   
    } 
    else
    {
     cur_sur->surf_grid[gx][gy].inside=-1; // even the cell is inside the bounding box, it is outside of the actual surface boundary
    } 
   ACBTSET->actual_human=0;
   point.z=cur_sur->BR_z+0.05; // 0.05 To avoid collision with table top
    
  
    
   
    int is_reachable_by_bending=is_point_on_surface_directly_reachable(point, 1); // 1 means test for human
    //if(is_reachable_by_bending==1)
    //printf("is_reachable_by_bending = %d \n",is_reachable_by_bending);
    if(is_reachable_by_bending==3)
     {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_bending=1;    
     cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending=1;    
     } 
    else
     {
     if(is_reachable_by_bending==2)
      {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_bending=1;    
   
      }
     else
      {
      if(is_reachable_by_bending==1)
       {
      cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending=1;    
   
       }
      else
       {
      //if(cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending>0)
      ////cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending=-1;
      ////cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_bending=-1;
       } 
      }
     }
    
   gy++;
   }
   gx++;
  }
 cur_sur->grid_i_max=gx;
 cur_sur->grid_j_max=gy;
  }//end else of if(collision_test_res>0)



  } //end for(;pitch_ang<0.785&&collision==0;pitch_ang+=0.087)

p3d_col_env_set_traj_method(tmp_col_type); //Restoring the actual collision check type
hum_tmp_pos[14]=orig_pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[14]=hum_tmp_pos[14];
p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
 }//end if(cur_sur->surface_shape==3) // rectangular surface
}

int update_surface_grid_by_bending_human_at_turned_pos(double sampling_rate, flat_surface *cur_sur)
{
 if(cur_sur->surface_shape==3) // rectangular surface
 {

//printf(" Inside update_surface_grid_by_bending_human_at_curr_pos \n");
int cx,cy;
/*for(cx=0;cx<100;cx++)
for(cy=0;cy<100;cy++)
 {
cur_sur->surf_grid[cx][cy].reachable_by_LHand_by_bending=-1;
cur_sur->surf_grid[cx][cy].reachable_by_RHand_by_bending=-1;
 }
*/
p3d_traj_test_type tmp_col_type = p3d_col_env_get_traj_method();
    p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE);

int collision =0 ;
configPt hum_tmp_pos=p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
double pitch_ang=hum_tmp_pos[14]; 
double orig_pitch_ang=pitch_ang;

for(;pitch_ang<M_PI/3.0&&collision==0;pitch_ang+=0.17)
 {
//printf(" pitch_ang = %lf\n",pitch_ang);
hum_tmp_pos[14]=pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[14]=hum_tmp_pos[14];
int kcd_with_report=0;


p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
int collision_test_res = p3d_col_test_robot(human,kcd_with_report);
////printf(" At pitch = %lf, The collision_test_res=%d \n", pitch_ang,collision_test_res);
 if(collision_test_res>0)
 {
 ////printf(" There is collision with human, collision_test_res=%d \n", collision_test_res);
 collision=1;
 }
 else
 {

  int gx=0;
  int gy=0;
  double i=cur_sur->BR_x_min;// min value of the bounding rectangle
  for(;i<=cur_sur->BR_x_max;i+=surf_grid_samp_rate)
  {
    gy=0;
   point_co_ordi point;
    point.x=i;
   double j=cur_sur->BR_y_min;
   for(;j<=cur_sur->BR_y_max;j+=surf_grid_samp_rate)
   {
      point.y=j;
    int inside=check_inside_polygon(4, cur_sur->vertices, point);//the order of vertices should be clockwise or counter clockwise 
    if(inside==1)
    {
     cur_sur->surf_grid[gx][gy].inside=1; // grid cell is inside the surface boundary   
   
    } 
    else
    {
     cur_sur->surf_grid[gx][gy].inside=-1; // even the cell is inside the bounding box, it is outside of the actual surface boundary
    } 
   ACBTSET->actual_human=0;
   point.z=cur_sur->BR_z+0.05; // 0.05 To avoid collision with table top
    
  
    
   
    int is_reachable_by_bending=is_point_on_surface_directly_reachable(point, 1); // 1 means test for human
    //if(is_reachable_by_bending==1)
    //printf("is_reachable_by_bending = %d \n",is_reachable_by_bending);
    if(is_reachable_by_bending==3)
     {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_turning_around_bending=1;    
     cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_turning_around_bending=1;    
     } 
    else
     {
     if(is_reachable_by_bending==2)
      {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_turning_around_bending=1;    
   
      }
     else
      {
      if(is_reachable_by_bending==1)
       {
      cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_turning_around_bending=1;    
   
       }
      else
       {
      //if(cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending>0)
      ////cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending=-1;
      ////cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_bending=-1;
       } 
      }
     }
    
   gy++;
   }
   gx++;
  }
 cur_sur->grid_i_max=gx;
 cur_sur->grid_j_max=gy;
  }//end else of if(collision_test_res>0)



  } //end for(;pitch_ang<0.785&&collision==0;pitch_ang+=0.087)

p3d_col_env_set_traj_method(tmp_col_type); //Restoring the actual collision check type
hum_tmp_pos[14]=orig_pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[14]=hum_tmp_pos[14];
p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
 }//end if(cur_sur->surface_shape==3) // rectangular surface
}



int update_surface_grid_by_turning_human_at_curr_pos(double sampling_rate, flat_surface *cur_sur)
{
 if(cur_sur->surface_shape==3) // rectangular surface
 {

//printf(" Inside update_surface_grid_by_bending_human_at_curr_pos \n");
int cx,cy;
for(cx=0;cx<100;cx++)
for(cy=0;cy<100;cy++)
 {
cur_sur->surf_grid[cx][cy].reachable_by_LHand_by_turning_around=-1;
cur_sur->surf_grid[cx][cy].reachable_by_RHand_by_turning_around=-1;
cur_sur->surf_grid[cx][cy].reachable_by_LHand_by_turning_around_bending=-1;
cur_sur->surf_grid[cx][cy].reachable_by_RHand_by_turning_around_bending=-1;
 }
p3d_traj_test_type tmp_col_type = p3d_col_env_get_traj_method();
    p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE);

int collision =0 ;
configPt hum_tmp_pos=p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
double turn_ang=hum_tmp_pos[11]; 
double orig_turn_ang=turn_ang;

for(;turn_ang<2*M_PI&&collision==0;turn_ang+=0.5)
 {
//printf(" pitch_ang = %lf\n",pitch_ang);
hum_tmp_pos[11]=turn_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[11]=hum_tmp_pos[11];
int kcd_with_report=0;


p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
int collision_test_res = p3d_col_test_robot(human,kcd_with_report);
////printf(" At pitch = %lf, The collision_test_res=%d \n", pitch_ang,collision_test_res);
 if(collision_test_res>0)
 {
 ////printf(" There is collision with human, collision_test_res=%d \n", collision_test_res);
 collision=1;
 }
 else
 {

  int gx=0;
  int gy=0;
  double i=cur_sur->BR_x_min;// min value of the bounding rectangle
  for(;i<=cur_sur->BR_x_max;i+=surf_grid_samp_rate)
  {
    gy=0;
   point_co_ordi point;
    point.x=i;
   double j=cur_sur->BR_y_min;
   for(;j<=cur_sur->BR_y_max;j+=surf_grid_samp_rate)
   {
      point.y=j;
    int inside=check_inside_polygon(4, cur_sur->vertices, point);//the order of vertices should be clockwise or counter clockwise 
    if(inside==1)
    {
     cur_sur->surf_grid[gx][gy].inside=1; // grid cell is inside the surface boundary   
   
    } 
    else
    {
     cur_sur->surf_grid[gx][gy].inside=-1; // Although the cell is inside the bounding box, it is outside of the actual surface boundary
    } 
   ACBTSET->actual_human=0;
   point.z=cur_sur->BR_z+0.05; // 0.05 To avoid collision with table top
    
  
    
   
    int is_reachable_by_bending=is_point_on_surface_directly_reachable(point, 1); // 1 means test for human
    //if(is_reachable_by_bending==1)
    //printf("is_reachable_by_bending = %d \n",is_reachable_by_bending);
    if(is_reachable_by_bending==3)
     {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_turning_around=1;    
     cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_turning_around=1;    
     } 
    else
     {
     if(is_reachable_by_bending==2)
      {
     cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_turning_around=1;    
   
      }
     else
      {
      if(is_reachable_by_bending==1)
       {
      cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_turning_around=1;    
   
       }
      else
       {
      //if(cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending>0)
      ////cur_sur->surf_grid[gx][gy].reachable_by_LHand_by_bending=-1;
      ////cur_sur->surf_grid[gx][gy].reachable_by_RHand_by_bending=-1;
       } 
      }
     }
    
   gy++;
   }
   gx++;
  }
 cur_sur->grid_i_max=gx;
 cur_sur->grid_j_max=gy;
  }//end else of if(collision_test_res>0)


  update_surface_grid_by_bending_human_at_turned_pos(sampling_rate, cur_sur);

  } //end for(;pitch_ang<0.785&&collision==0;pitch_ang+=0.087)

p3d_col_env_set_traj_method(tmp_col_type); //Restoring the actual collision check type
hum_tmp_pos[11]=orig_turn_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[11]=hum_tmp_pos[11];
p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
 }//end if(cur_sur->surface_shape==3) // rectangular surface
}

int initialize_surfaces_in_env()
{


  envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4 objCenter,robCenter;
  int i,j;
  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int contObj =0;
  p3d_vector4 pointHead,pointDisc, pointAhead;
 

 curr_surfaces_in_env.total_no_of_surfaces=0;
 int ns=0;
    //Robot body parts
  
    for(i=0;i<nr;i++)
    {
      r = envPt->robot[i];
      if (strcasestr(r->name,"table"))
     {
 	//// printf("envPt->robot[%d]->name =%s\n", i, envPt->robot[i]->name);
     ////printf(" envPt->robot[%d]->BB.xmin, xmax, ymin, ymax, zmin, zmax : ( %lf, %lf, %lf, %lf, %lf, %lf )\n",i,envPt->robot[i]->BB.xmin,envPt->robot[i]->BB.xmax,envPt->robot[i]->BB.ymin,envPt->robot[i]->BB.ymax, envPt->robot[i]->BB.zmin,envPt->robot[i]->BB.zmax);	

		curr_surfaces_in_env.flat_surf[ns].no_vertices=4;
  		curr_surfaces_in_env.flat_surf[ns].surface_shape=3;//Rectangle

//AKP NOTE : Take care that vertices should be in clockwise or anticlockwise order
// TO DO : AKP : WARNING : Now assuming that the rectangle sides are parallel to the x and y axes, so the vertices are directly assigned as the bounding box values. In future make changes to assign the vertices of such rectangles wgich are not parallel to the axes

  		 curr_surfaces_in_env.flat_surf[ns].vertices[0].x=envPt->robot[i]->BB.xmin;
 		 curr_surfaces_in_env.flat_surf[ns].vertices[0].y=envPt->robot[i]->BB.ymin;
 		 curr_surfaces_in_env.flat_surf[ns].vertices[0].z=envPt->robot[i]->BB.zmax;
 
		  curr_surfaces_in_env.flat_surf[ns].vertices[1].x=envPt->robot[i]->BB.xmin;
		  curr_surfaces_in_env.flat_surf[ns].vertices[1].y=envPt->robot[i]->BB.ymax;
		  curr_surfaces_in_env.flat_surf[ns].vertices[1].z=envPt->robot[i]->BB.zmax;
 
		  curr_surfaces_in_env.flat_surf[ns].vertices[2].x=envPt->robot[i]->BB.xmax;
		  curr_surfaces_in_env.flat_surf[ns].vertices[2].y=envPt->robot[i]->BB.ymax;
		  curr_surfaces_in_env.flat_surf[ns].vertices[2].z=envPt->robot[i]->BB.zmax;
 
  		curr_surfaces_in_env.flat_surf[ns].vertices[3].x=envPt->robot[i]->BB.xmax;
 		 curr_surfaces_in_env.flat_surf[ns].vertices[3].y=envPt->robot[i]->BB.ymin;
 		 curr_surfaces_in_env.flat_surf[ns].vertices[3].z=envPt->robot[i]->BB.zmax;

 		 curr_surfaces_in_env.flat_surf[ns].BR_x_min=envPt->robot[i]->BB.xmin;
		  curr_surfaces_in_env.flat_surf[ns].BR_x_max=envPt->robot[i]->BB.xmax;
 		 curr_surfaces_in_env.flat_surf[ns].BR_y_min=envPt->robot[i]->BB.ymin;;
		  curr_surfaces_in_env.flat_surf[ns].BR_y_max=envPt->robot[i]->BB.ymax;;

 		 curr_surfaces_in_env.flat_surf[ns].BR_z=envPt->robot[i]->BB.zmax;
		  curr_surfaces_in_env.total_no_of_surfaces++;
                  ns++;
     }
    /*
      for(j=0;j<r->no;j++)
	{
	  o = r->o[j];
           printf("envPt->robot[%d]->o[%d]->name =%s, envPt->robot[%d]->o[%d]->np =%d\n", i,j,  envPt->robot[i]->o[j]->name,i,j,envPt->robot[i]->o[j]->np);
	  if (strcasestr(o->name,"table"))
	    {
	      //p3d_get_object_center(o,objCenter);
	      printf(" envPt->robot[%d]->o[%d]->BB.xmin, xmax, ymin, ymax, zmin, zmax : ( %lf, %lf, %lf, %lf, %lf, %lf )\n",i,j,envPt->robot[i]->o[j]->BB.xmin,envPt->robot[i]->o[j]->BB.xmax,envPt->robot[i]->o[j]->BB.ymin,envPt->robot[i]->o[j]->BB.ymax, envPt->robot[i]->o[j]->BB.zmin,envPt->robot[i]->o[j]->BB.zmax);	

              
	    }
	}
    */
      //define if robot is near or not? here or in observation? od we need a different list
      // if ((ContObjTmp/r->no)>.4)
      // 

    }

// 1st surface
/*
  curr_surfaces_in_env.flat_surf[0].no_vertices=4;
  curr_surfaces_in_env.flat_surf[0].surface_shape=3;//Rectangle

  curr_surfaces_in_env.flat_surf[0].vertices[0].x=0.5+4.6;
  curr_surfaces_in_env.flat_surf[0].vertices[0].y=0.5-3;
  curr_surfaces_in_env.flat_surf[0].vertices[0].z=0.75;
 
  curr_surfaces_in_env.flat_surf[0].vertices[1].x=0.5+4.6;
  curr_surfaces_in_env.flat_surf[0].vertices[1].y=-0.5-3;
  curr_surfaces_in_env.flat_surf[0].vertices[1].z=0.75;
 
  curr_surfaces_in_env.flat_surf[0].vertices[2].x=-0.5+4.6;
  curr_surfaces_in_env.flat_surf[0].vertices[2].y=-0.5-3;
  curr_surfaces_in_env.flat_surf[0].vertices[2].z=0.75;
 
  curr_surfaces_in_env.flat_surf[0].vertices[3].x=-0.5+4.6;
  curr_surfaces_in_env.flat_surf[0].vertices[3].y=0.5-3;
  curr_surfaces_in_env.flat_surf[0].vertices[3].z=0.75;

  curr_surfaces_in_env.flat_surf[0].BR_x_min=-0.5+4.6;
  curr_surfaces_in_env.flat_surf[0].BR_x_max=0.5+4.6;
  curr_surfaces_in_env.flat_surf[0].BR_y_min=-0.5-3;
  curr_surfaces_in_env.flat_surf[0].BR_y_max=0.5-3;

  curr_surfaces_in_env.flat_surf[0].BR_z=0.75;
  curr_surfaces_in_env.total_no_of_surfaces=1;

//2nd surface
curr_surfaces_in_env.flat_surf[1].no_vertices=4;
  curr_surfaces_in_env.flat_surf[1].surface_shape=3;//Rectangle

  curr_surfaces_in_env.flat_surf[1].vertices[0].x=1.2+4.6;
  curr_surfaces_in_env.flat_surf[1].vertices[0].y=1.75-3;
  curr_surfaces_in_env.flat_surf[1].vertices[0].z=0.5;
 
  curr_surfaces_in_env.flat_surf[1].vertices[1].x=1.2+4.6;
  curr_surfaces_in_env.flat_surf[1].vertices[1].y=0.75-3;
  curr_surfaces_in_env.flat_surf[1].vertices[1].z=0.5;
 
  curr_surfaces_in_env.flat_surf[1].vertices[2].x=0.2+4.6;
  curr_surfaces_in_env.flat_surf[1].vertices[2].y=0.75-3;
  curr_surfaces_in_env.flat_surf[1].vertices[2].z=0.5;
 
  curr_surfaces_in_env.flat_surf[1].vertices[3].x=0.2+4.6;
  curr_surfaces_in_env.flat_surf[1].vertices[3].y=1.75-3;
  curr_surfaces_in_env.flat_surf[1].vertices[3].z=0.5;

  curr_surfaces_in_env.flat_surf[1].BR_x_min=0.2+4.6;
  curr_surfaces_in_env.flat_surf[1].BR_x_max=1.2+4.6;
  curr_surfaces_in_env.flat_surf[1].BR_y_min=0.75-3;
  curr_surfaces_in_env.flat_surf[1].BR_y_max=1.75-3;

  curr_surfaces_in_env.flat_surf[1].BR_z=0.5;
  curr_surfaces_in_env.total_no_of_surfaces=2;
  
  */
  
}


int show_3d_grid_affordances_new()
{
/*
 point_co_ordi shoulder_pos;
shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);

 shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Red, NULL);
*/
/*
point_co_ordi neck_pos;
neck_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    neck_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    neck_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(neck_pos.x, neck_pos.y, neck_pos.z, .1, Blue, NULL);
*/
 ////printf("Inside show_3d_grid_affordances()\n");
  /*printf("Inside show_3d_grid_affordances_new()\n");
  point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
      
    // Reachability Test by right hand
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
 
  no_sphere_surface_pts=0;
    find_reachable_sphere_surface(1,1);
double interval=grid_around_HRP2.GRID_SET->pace/1.0;
 for(int sp_ctr=0;sp_ctr<no_sphere_surface_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
      g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
       }
       //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
      }
   
    
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Red, NULL);
  
  
  no_sphere_surface_pts=0;
    find_reachable_sphere_surface(2,1);

 for(int sp_ctr=0;sp_ctr<no_sphere_surface_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
      g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
       }
      //// g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
      }
*/

/////////g3d_drawDisc(agent_eye_pos.x, agent_eye_pos.y,agent_eye_pos.z, 0.1, Red, NULL);

 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
       ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
      
       int show_showing_points=0;

       
       if(SHOW_3D_VISIBLE_PLACE_HUM==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        ////else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
        }
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         } 
       }

       if(SHOW_3D_VISIBLE_PLACE_STANDING_HUM==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_standing_human==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        ////else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_standing_human_neck_turn==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation_standing==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        } 
       }
       
       if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        ////else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       } 
      
       
       if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        ////else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       }        


       if(SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN==1)
       {        
        //printf("**** SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=%d, SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=%d\n",SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN,SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN);
        
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
       }
       
       if(SHOW_3D_DIRECT_REACHABLE_HUM==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       }


       if(SHOW_3D_BENDING_REACHABLE_HUM==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       }

       if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_turning_around_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_turning_around_bending==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_turning_around_bending==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_turning_around_bending==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       }
       

       if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       }

       if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        
        
       }

       if(SHOW_3D_COMMON_REACH_HRP2_HUMAN==1)
       {
        //printf("SHOW_3D_COMMON_REACH_HRP2_HUMAN=%d\n",SHOW_3D_COMMON_REACH_HRP2_HUMAN);
        if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)||(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)))//Points which are reachable by both hands of both, human and HRP2
        {
         g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        else
        {
          if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)))//Points which are reachable by both hands of HRP2 and atleast one hand of human
         {
         g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
         else
         { 
         if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1))//Point which is reachable by either hand of both HRP2 and human
          {
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
          }
         }
        }
       }


       //======START for 2D on table surfaces========//
      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
      {
       ////printf(" is_horizontal_surface\n");
       if(SHOW_2D_VISIBLE_PLACE_HUM==1)
       {        
        
       
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        ////else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);
         }
        }
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
        }  

       }
       
       if(SHOW_2D_VISIBLE_PLACE_STANDING_HUM==1)
       {        
        
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_standing_human==1)
        {
        ////printf(" visible by human\n");
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_standing_human_neck_turn==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);
        }

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation_standing==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
        }
       }

       if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);
        }
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
        }
       } 

       if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
        }
       } 
      
       if(SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN==1)
       {        
        //printf("**** SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=%d, SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=%d\n",SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN,SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN);
        
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
       }
       
       if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
         }
        }
       }

        if(SHOW_2D_BENDING_REACHABLE_HUM==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
         }
        }
       }

      if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_turning_around_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_turning_around_bending==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_turning_around_bending==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_turning_around_bending==1)
         {
         ////printf(" RHand reach by turn and bend Drawing disc\n"); 
        g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
         }
        }
       }
        
       if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
         }
        }
       }
    
       if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
       {        

       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
        }
       }

       if(SHOW_2D_COMMON_REACH_HRP2_HUMAN==1)
       {
        ////////printf("inside SHOW_3D_COMMON_REACH_HRP2_HUMAN=%d\n",SHOW_3D_COMMON_REACH_HRP2_HUMAN);
        if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1))//Points which are reachable by both hands of both, human and HRP2
        {
         g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
        else
        {
         if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1))//Point which is reachable by either hand of both HRP2 and human
         {
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
         }
        }
       } 
      } 
       //======END for 2D on table surfaces ========//

      int show_horizontal_surfaces=0;
      if(show_horizontal_surfaces==1)
      {  
       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
        {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world+0.0, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
      }
 
       if(show_showing_points==1) // To calculate the candidate points to show some object to human
       {
        if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1))
        {
          
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
         
          /*
          p3d_vector3 target_in_global_frame;
          target_in_global_frame[0]=cell_x_world;
          target_in_global_frame[1]=cell_y_world;
          target_in_global_frame[2]=cell_z_world;
          int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
          int hand_by_reach=2; //1 for left, 2 for right hand
          double task_duration=3.0;//in s
          int thumb_up_constraint=1;//Means the x axis of the hand will be made parallel to the z axis og global frame
          vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
          int reachable_by_R_hand=HRP2_hand_reach(target_in_global_frame, hand_by_reach, task_duration, HRP2_state,thumb_up_constraint);
          attStandingRobot->staticState ( backupConfig );
          if(reachable_by_R_hand==1)
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
          else
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         */
          
           
        } 
       }
        //fl_check_forms();
       //g3d_draw_allwin_active();
    ////}
    ////else
    ////{
    ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
    ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
    ////} 
    //  fl_check_forms();
   }
  } 
 }
//fl_check_forms();
  //     g3d_draw_allwin_active();
}


int show_3d_grid_affordances()
{
 printf("Inside show_3d_grid_affordances()\n");
 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
       ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
       int show_visibility=0;
       int show_HRP2_reachability=0;
       int show_human_reachability=0;
       int show_showing_points=1;

       if(show_visibility==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
         }
        }
       }
       if(show_HRP2_reachability==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
         }
        }
       }
       
       if(show_human_reachability==1)
       {        

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
        {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
        }
        else
        {
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1)
         {
       //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         }
   
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
         {
        //// printf(" Drawing disc\n"); 
        g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
         }
        }
       }
 
       if(show_showing_points==1) // To calculate the candidate points to show some object to human
       {
        if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1))
        {
          
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
         
          /*
          p3d_vector3 target_in_global_frame;
          target_in_global_frame[0]=cell_x_world;
          target_in_global_frame[1]=cell_y_world;
          target_in_global_frame[2]=cell_z_world;
          int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
          int hand_by_reach=2; //1 for left, 2 for right hand
          double task_duration=3.0;//in s
          int thumb_up_constraint=1;//Means the x axis of the hand will be made parallel to the z axis og global frame
          vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
          int reachable_by_R_hand=HRP2_hand_reach(target_in_global_frame, hand_by_reach, task_duration, HRP2_state,thumb_up_constraint);
          attStandingRobot->staticState ( backupConfig );
          if(reachable_by_R_hand==1)
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
          else
          g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
         */
          
           
        } 
       }
        //fl_check_forms();
       //g3d_draw_allwin_active();
    ////}
    ////else
    ////{
    ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
    ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
    ////} 
    //  fl_check_forms();
   }
  } 
 }
//fl_check_forms();
  //     g3d_draw_allwin_active();
}

int update_3d_grid_reachability_for_human_new()
{

  
  int for_agent=1; //1 for human, 2 for HRP2

  

   printf("Inside update_3d_grid_reachability_for_human_new()\n");
   point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
configPt hum_tmp_pos=p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
//////////double yaw_ang=hum_tmp_pos[11]; 
double yaw_ang=hum_tmp_pos[HUMANq_TORSO_PAN]; 
double orig_yaw_ang=yaw_ang;
//////////double pitch_ang=hum_tmp_pos[14]; 
double pitch_ang=hum_tmp_pos[HUMANq_TORSO_TILT]; 
double orig_pitch_ang=pitch_ang;

int for_actual_pitch=1;
int for_actual_yaw=1;

int turn_collision=0;
//double yaw_
////for(;yaw_ang<2.0*M_PI&&turn_collision==0;yaw_ang+=0.5)
int turn_human=1;
double maxi_left_turn=M_PI/2.0;
double maxi_right_turn=M_PI/2.0;
double curr_left_turn=0.0;
double curr_right_turn=0.0;

int init_yaw=1;

double interval=grid_around_HRP2.GRID_SET->pace-0.005;///1.5;

while(turn_human==1)
 {
////////printf(" **** yaw_ang = %lf\n",yaw_ang);

   ////pitch_ang=hum_tmp_pos[14]; 
   ////orig_pitch_ang=pitch_ang;
   int collision=0;
   
pitch_ang=orig_pitch_ang;
hum_tmp_pos[HUMANq_TORSO_TILT]=pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];
  
for(;pitch_ang<0.785&&collision==0;pitch_ang+=0.25)
  {
//////printf(" **** for yaw_ang = %lf, pitch_ang = %lf\n",yaw_ang, pitch_ang);

hum_tmp_pos[HUMANq_TORSO_TILT]=pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
   {
 //////// printf(" There is collision with human, for pitch_ang = %lf res=%d \n", pitch_ang, res);
  for_actual_pitch=0;
  collision=1;
  hum_tmp_pos[HUMANq_TORSO_TILT]=orig_pitch_ang; 
  pitch_ang=orig_pitch_ang;
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];

  break;
   }

   shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
    int for_hand=1;//1 for left, 2 for right
    no_sphere_surface_pts=0;
    int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
////double interval=grid_around_HRP2.GRID_SET->pace/1.5;
 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
      
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
     
      if(cell_x<0||cell_x>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
      {
       break;
      } 
 
      int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y<0||cell_y>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
      {
       break;
      } 
 
      int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z<0||cell_z>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
      {
       break;
      } 
 
   
      if(for_actual_pitch==1&&for_actual_yaw==1)
        {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_LHand=1;
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=1;  
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=1; 
        }
      else
        {
         if(for_actual_pitch==0&&for_actual_yaw==1)
         {
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=1;  
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=1;
         }
         else
         {
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=1;  
         }
        }
      ////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
       }
       //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
      }
   
    
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Red, NULL);
  
    for_hand=2;//1 for left, 2 for right
    no_sphere_surface_pts=0;
    no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);

 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 

      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;

      if(cell_x<0||cell_x>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
      {
       break;
      } 
 
     int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y<0||cell_y>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
      {
       break;
      } 
 
      int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
      if(cell_z<0||cell_z>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
      {
       break;
      } 

        
      
      
   
       if(for_actual_pitch==1&&for_actual_yaw==1)
        {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_RHand=1;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=1;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=1; 
        }
      else
        {
         if(for_actual_pitch==0&&for_actual_yaw==1)
         {
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=1; 
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=1; 
         }
         else
         {
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=1;  
         }
        }
      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].affordances.reachable_by_human_RHand=1;   

      //g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
       }
      //// g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
      }
 
 
 
 for_actual_pitch=0;
 

  }//loop for pitch

 
 for_actual_yaw=0;
 if(curr_left_turn<=maxi_left_turn)
  {
  
  curr_left_turn+=0.5;
  ////////printf(" curr_left_turn = %lf\n",curr_left_turn);
  yaw_ang+=0.5;
  }
 else
  {
  if(curr_left_turn>maxi_left_turn&&curr_right_turn<=maxi_right_turn)
   {
   
   curr_right_turn+=0.5;
    //////printf(" curr_right_turn = %lf\n",curr_right_turn);
    if(init_yaw==1)
    {
   yaw_ang=orig_yaw_ang; 
   init_yaw=0;
    }
   yaw_ang-=0.5;
   }
  else
   {
   turn_human=0;
   }
  }
 
hum_tmp_pos[HUMANq_TORSO_PAN]=yaw_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_PAN]=hum_tmp_pos[HUMANq_TORSO_PAN];
   
hum_tmp_pos[HUMANq_TORSO_TILT]=orig_pitch_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
   {
  //////printf(" There is collision with human, for yaw_ang = %lf and pitch_ang = %lf res=%d \n", yaw_ang, pitch_ang, res);
  turn_collision=1;
  turn_human=0;
   break;
   }





 }//Loop for yaw 
 
hum_tmp_pos[HUMANq_TORSO_PAN]=orig_yaw_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_PAN]=hum_tmp_pos[HUMANq_TORSO_PAN];
////p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);

hum_tmp_pos[HUMANq_TORSO_TILT]=orig_pitch_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];
p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);

return 1;

 

}

int update_3d_grid_reachability_for_human_standing_new()
{

  
  int for_agent=1; //1 for human, 2 for HRP2

  

   printf("Inside update_3d_grid_reachability_for_human_standing_new()\n");
   point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
configPt hum_tmp_pos=p3d_get_robot_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt);
double yaw_ang=hum_tmp_pos[HUMANq_TORSO_PAN]; 
double orig_yaw_ang=yaw_ang;
double pitch_ang=hum_tmp_pos[HUMANq_TORSO_TILT]; 
double orig_pitch_ang=pitch_ang;

int for_actual_pitch=1;
int for_actual_yaw=1;

int turn_collision=0;
//double yaw_
////for(;yaw_ang<2.0*M_PI&&turn_collision==0;yaw_ang+=0.5)
int turn_human=1;
double maxi_left_turn=M_PI/2.0;
double maxi_right_turn=M_PI/2.0;
double curr_left_turn=0.0;
double curr_right_turn=0.0;

int init_yaw=1;

double interval=grid_around_HRP2.GRID_SET->pace-0.005;///1.5;

while(turn_human==1)
 {
////////printf(" **** yaw_ang = %lf\n",yaw_ang);

   ////pitch_ang=hum_tmp_pos[14]; 
   ////orig_pitch_ang=pitch_ang;
   int collision=0;
   
pitch_ang=orig_pitch_ang;
hum_tmp_pos[HUMANq_TORSO_TILT]=pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];
  
for(;pitch_ang<0.785&&collision==0;pitch_ang+=0.25)
  {
//////printf(" **** for yaw_ang = %lf, pitch_ang = %lf\n",yaw_ang, pitch_ang);

hum_tmp_pos[HUMANq_TORSO_TILT]=pitch_ang; // Around 5 degrees
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
   {
 //////// printf(" There is collision with human, for pitch_ang = %lf res=%d \n", pitch_ang, res);
  for_actual_pitch=0;
  collision=1;
  hum_tmp_pos[HUMANq_TORSO_TILT]=orig_pitch_ang; 
  pitch_ang=orig_pitch_ang;
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];

  break;
   }

   shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
    int for_hand=1;//1 for left, 2 for right
    no_sphere_surface_pts=0;
    int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
////double interval=grid_around_HRP2.GRID_SET->pace/1.5;
 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
      
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
     
      if(cell_x<0||cell_x>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
      {
       break;
      } 
 
      int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y<0||cell_y>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
      {
       break;
      } 
 
      int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z<0||cell_z>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
      {
       break;
      } 
 
   
      if(for_actual_pitch==1&&for_actual_yaw==1)
        {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_standing=1;
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_standing_bending=1;  
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_standing_turning_around_bending=1; 
        }
      else
        {
         if(for_actual_pitch==0&&for_actual_yaw==1)
         {
         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_standing_bending=1;  
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_standing_turning_around_bending=1; 
         }
         else
         {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_standing_turning_around_bending=1; 
         }
        }
      ////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
       }
       //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
      }
   
    
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Red, NULL);
  
    for_hand=2;//1 for left, 2 for right
    no_sphere_surface_pts=0;
    no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);

 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 

      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;

      if(cell_x<0||cell_x>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
      {
       break;
      } 
 
     int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y<0||cell_y>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
      {
       break;
      } 
 
      int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
      if(cell_z<0||cell_z>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
      {
       break;
      } 

        
      
      
   
       if(for_actual_pitch==1&&for_actual_yaw==1)
        {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_standing=1;
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_standing_bending=1;  
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_standing_turning_around_bending=1; 
        }
      else
        {
         if(for_actual_pitch==0&&for_actual_yaw==1)
         {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_standing_bending=1;  
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_standing_turning_around_bending=1; 
         }
         else
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_standing_turning_around_bending=1; 
         }
        }
      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].affordances.reachable_by_human_RHand=1;   

      //g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
       }
      //// g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
      }
 
 
 
 for_actual_pitch=0;
 

  }//loop for pitch

 
 for_actual_yaw=0;
 if(curr_left_turn<=maxi_left_turn)
  {
  
  curr_left_turn+=0.5;
  ////////printf(" curr_left_turn = %lf\n",curr_left_turn);
  yaw_ang+=0.5;
  }
 else
  {
  if(curr_left_turn>maxi_left_turn&&curr_right_turn<=maxi_right_turn)
   {
   
   curr_right_turn+=0.5;
    //////printf(" curr_right_turn = %lf\n",curr_right_turn);
    if(init_yaw==1)
    {
   yaw_ang=orig_yaw_ang; 
   init_yaw=0;
    }
   yaw_ang-=0.5;
   }
  else
   {
   turn_human=0;
   }
  }
 
hum_tmp_pos[HUMANq_TORSO_PAN]=yaw_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_PAN]=hum_tmp_pos[HUMANq_TORSO_PAN];
   
hum_tmp_pos[HUMANq_TORSO_TILT]=orig_pitch_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
   {
  //////printf(" There is collision with human, for yaw_ang = %lf and pitch_ang = %lf res=%d \n", yaw_ang, pitch_ang, res);
  turn_collision=1;
  turn_human=0;
   break;
   }





 }//Loop for yaw 
 
hum_tmp_pos[HUMANq_TORSO_PAN]=orig_yaw_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_PAN]=hum_tmp_pos[HUMANq_TORSO_PAN];
////p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);

hum_tmp_pos[HUMANq_TORSO_TILT]=orig_pitch_ang; 
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_TILT]=hum_tmp_pos[HUMANq_TORSO_TILT];
p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_tmp_pos);

return 1;

 

}


int update_3d_grid_reachability_for_human()
{
  configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */
  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&rob_cur_pos); 
 
  printf("Inside update_3D_grid_based_on_current_human_pos()\n");
 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
      //// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
       point_co_ordi point;
        point.x=cell_x_world;
        point.y=cell_y_world;
        point.z=cell_z_world;
    if(fabs(rob_cur_pos[6]-point.x)>1.5||fabs(rob_cur_pos[7]-point.y)>1.5||fabs(rob_cur_pos[8]-point.z)>3.0||point.z<0.3)   // Surely not reachable by HRP2 from its current position
    {
  
    }
   else
    {
   /////p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos); // Restore the robot's initial position before calculating the reachability for next point; 
   //////int state=1; //AKP : 1 is sitting, 2 is standing 
   //////create_HRP2_robot(state);//Everytime restore the original position of the robot
   //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
   
   ////int is_reachable_directly=is_point_on_surface_directly_reachable(point, 2); // 2 means test for HRP2
    int is_reachable_directly=is_point_directly_reachable_by_human(point);
    
    if(is_reachable_directly==3)
    {
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand=1;    
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand=1;    
    } 
    else
    {
     if(is_reachable_directly==2)
     {
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand=1;    
   
     }
     else
     {
      if(is_reachable_directly==1)
      {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand=1;    
   
      }
      else
      {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand=-1;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand=-1;
      } 
     }
     
    } 
   }//End else
  
   }
  } 
 }

  

 MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);

}



int find_reachable_sphere_surface(int for_hand, int for_agent)
{

   point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
   no_sphere_surface_pts=0;
   
    ////int for_hand=1;//1 for left, 2 for right
    int shoulder_indx; 
    if(for_agent==1)//for human
    {
    if(for_hand==1)//1 for left, 2 for right
     {
   shoulder_indx=HUMANj_LSHOULDER;
     }
    else
     {
     if(for_hand==2)
      {
     shoulder_indx=HUMANj_RSHOULDER;
      }
     }
    }
    else
    {
    if(for_agent==2)//for HRP2
     {
    if(for_hand==1)//1 for left, 2 for right
      {
   shoulder_indx=ROBOTj_LSHOULDER;
      }
    else
      {
     if(for_hand==2)
       {
     shoulder_indx=ROBOTj_RSHOULDER;
       }
      }
     }
     else
     {
       if(for_agent==3)//for JIDO
      {
   
     shoulder_indx=ROBOTj_RSHOULDER;
   
      }
     }  
    }

    double r=0;    
    double curr_yaw=0;

    double shoulder_back_limit;// The maxi possible angle away from the front axis of the sgent
    double shoulder_front_limit;// The maxi possible angle crossing the front axis of the agent

    if(for_agent==1)//For human
    {
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[shoulder_indx]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[shoulder_indx]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[shoulder_indx]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    r=0.7;//Maximum reach boundary for human
    curr_yaw=ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TORSO_PAN];   

    shoulder_back_limit=M_PI/2.0+M_PI/6.0;// The maxi possible angle away from the front axis of human
    shoulder_front_limit=M_PI/3.0;// The maxi possible angle crossing the front axis of human

    }
    else
    {
     if(for_agent==2)//for HRP2
     {
    shoulder_pos.x = ACBTSET->robot->joints[shoulder_indx]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->robot->joints[shoulder_indx]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->robot->joints[shoulder_indx]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     r=0.7;//Maximum reach boundary for HRP2
     curr_yaw=ACBTSET->robot->ROBOT_POS[11];   

    shoulder_back_limit=M_PI/2.0;// The maxi possible angle away from the front axis of HRP2
    shoulder_front_limit=M_PI/3.0;// The maxi possible angle crossing the front axis of HRP2

     }
     else
     {
      if(for_agent==3)//for JIDO
      {
    shoulder_pos.x = ACBTSET->robot->joints[shoulder_indx]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->robot->joints[shoulder_indx]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->robot->joints[shoulder_indx]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     r=1.25;//Maximum reach boundary for JIDO
     curr_yaw=ACBTSET->robot->ROBOT_POS[11];   

    shoulder_back_limit=5.0*M_PI/6.0;// The maxi possible angle away from the front axis of JIDO
    shoulder_front_limit=4.0*M_PI/6.0;// The maxi possible angle crossing the front axis of JIDO

    for_hand=2;//Just to comply with rest of the code
      }
     }
    }

    
    double theta, phi;
    
    ////printf("curr_yaw=%lf\n",curr_yaw);
    double phi_st, phi_end;
    //phi_st=phi;
    
    
    if(for_hand==2)//For right hand
    {
    phi_st=curr_yaw-shoulder_back_limit;
    phi_end=curr_yaw+shoulder_front_limit;
    }
    if(for_hand==1)
    {
    phi_st=curr_yaw-shoulder_front_limit;
    phi_end= (curr_yaw)+(shoulder_back_limit);
    
    }
    /*
     if(phi_st>2.0*M_PI)
     {
     phi_st=phi_st-2.0*M_PI;
     }*/
   
    double increment=grid_around_HRP2.GRID_SET->pace/r;  
    ////////printf("phi_st=%lf, phi_end=%lf, increment=%lf\n",phi_st,phi_end, increment);
     //// phi=hum_yaw;
     
     
      for(phi=phi_st;phi<=phi_end;phi+=increment)
      { 
       for(theta=0.0;theta<=M_PI;theta+=increment)
       {
    sphere_surface_pts[no_sphere_surface_pts].x=shoulder_pos.x+r*sin(theta)*cos(phi);
    sphere_surface_pts[no_sphere_surface_pts].y=shoulder_pos.y+r*sin(theta)*sin(phi);
    sphere_surface_pts[no_sphere_surface_pts].z=shoulder_pos.z+r*cos(theta);
   //////// printf("no_sphere_surface_pts =%d\n",no_sphere_surface_pts);
    no_sphere_surface_pts++;
    
       }
      } 
////////printf("no_sphere_surface_pts =%d\n",no_sphere_surface_pts);
  return no_sphere_surface_pts;
}

int update_3d_grid_reachability_for_JIDO_new()
{

  
  int for_agent=3; //1 for human, 2 for HRP2, 3 for Jido
  
  

   printf("Inside update_3d_grid_reachability_for_JIDO_new()\n");
   point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
      
    // Reachability Test by left hand
    shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
 
    int for_hand=2;//Although JIDO has only one arm, but just to comply with find_reachable_sphere_surface() function, we should always pass 2
    no_sphere_surface_pts=0;
    int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
double interval=grid_around_HRP2.GRID_SET->pace/1.5;
 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
       
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2;
      double y2;
      double z2;
      int cell_x;
      int cell_y;
      int cell_z;
 
      x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  

      if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
        {
      y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 

      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
         { 
        z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
        cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 

        if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
          {
        ////printf(" Cell (%d, %d, %d) is reachabke by JIDO\n", cell_x,cell_y,cell_z);
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_JIDO_Hand=1;   
          }
         }
      ////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
        }
       }

       //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
     }
   
    
 
return 1;

 

}


int update_3d_grid_reachability_for_HRP2_new()
{

  
  int for_agent=2; //1 for human, 2 for HRP2

  

   printf("Inside update_3d_grid_reachability_for_HRP2_new()\n");
   point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
      
    // Reachability Test by left hand
    shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
 
    int for_hand=1;//1 for left, 2 for right
    no_sphere_surface_pts=0;
    int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
double interval=grid_around_HRP2.GRID_SET->pace/1.5;
 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
      int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
   
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_LHand=1;   
      ////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
       }
       //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
      }
   
    
    // Reachability Test by right hand
    shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
    for_hand=2;//1 for left, 2 for right
    no_sphere_surface_pts=0;
    no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);

 for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
      double t=0;
       for(;t<1;t+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
      double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
      double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
      int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
   
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_RHand=1;   

      //g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
       }
      //// g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
      }
 
return 1;

 

}


int update_3d_grid_reachability_for_HRP2()
{
  configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos); 
 
  printf("Inside update_3D_grid_based_on_current_human_pos()\n");
 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
      //// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
       point_co_ordi point;
        point.x=cell_x_world;
        point.y=cell_y_world;
        point.z=cell_z_world;
    if(fabs(rob_cur_pos[6]-point.x)>1.5||fabs(rob_cur_pos[7]-point.y)>1.5||fabs(rob_cur_pos[8]-point.z)>3.0||point.z<0.3)   // Surely not reachable by HRP2 from its current position
    {
  
    }
   else
    {
   /////p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos); // Restore the robot's initial position before calculating the reachability for next point; 
   //////int state=1; //AKP : 1 is sitting, 2 is standing 
   //////create_HRP2_robot(state);//Everytime restore the original position of the robot
   //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
   
   ////int is_reachable_directly=is_point_on_surface_directly_reachable(point, 2); // 2 means test for HRP2
    int is_reachable_directly=is_point_directly_reachable_by_HRP2(point);
   
    if(is_reachable_directly==3)
    {
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand=1;    
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand=1;    
    } 
    else
    {
     if(is_reachable_directly==2)
     {
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand=1;    
   
     }
     else
     {
      if(is_reachable_directly==1)
      {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand=1;    
   
      }
      else
      {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand=-1;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand=-1;
      } 
     }
     
    } 
   }//End else
  
   }
  } 
 }

 MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);

}



int update_3d_grid_visibility(int type)//1 means human, 2 means HRP2, 3 means JIDO
{
printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
point_co_ordi eye_pos;
   
eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
    hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
    hum_head_pos[8] = eye_pos.z;
 
    agent_eye_pos.x=eye_pos.x;       
    agent_eye_pos.y=eye_pos.y;
    agent_eye_pos.z=eye_pos.z;

    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

 double interval=grid_around_HRP2.GRID_SET->pace/2.0;
   printf("interval=%lf, no_FOV_end_point_vertices=%d\n",interval,no_FOV_end_point_vertices);
 
int visible_ctr=0;  
int not_visible_ctr=0;
 int j=0;
    for(j=0;j<no_FOV_end_point_vertices;j++)
    {

    int i=4;
    for(i=4;i<8;i++)
     {
   //// if(i<4)
    //// {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
    //// }  
    ////else
    // {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
 double t=0;

      for(;t<1;t+=interval)
      {
      
      double x;
      double y;
      double z;
      
      if(i<7)
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
        } 
      else
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
        }
       ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
      ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
      int obs_found=0;
      double t2=0.1;//Just to avoid very checking very near to human
      
      for(;t2<1;t2+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*eye_pos.x+t2*x;
      double y2=(1-t2)*eye_pos.y+t2*y;
      double z2=(1-t2)*eye_pos.z+t2*z; 
      
  /*  double x2=(1-t2)*x+t*eye_pos.x;
      double y2=(1-t2)*y+t*eye_pos.y;
      double z2=(1-t2)*z+t*eye_pos.z; 
  */
       //////////hri_bitmap_cell* curr_cell=NULL;
  
       ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

       //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
       int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       int cell_y;
       int cell_z;

       int cell_valid=0;       

       if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
       {
      
 
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
       {
       cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
        {
       ////curr_cell=
       ////curr_cell->x=cell_x;
       ////curr_cell->y=cell_y;
       ////curr_cell->z=cell_z;
       cell_valid=1;
        } 
       }
      }

      ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

       ////////if(curr_cell!=NULL)
        if(cell_valid==1)
        { 
        /*
        if(type==1)//means for human
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=0;
         }
        else
         {
        if(type==2)//means for HRP2
          {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=0;
          }
        else
          {
           if(type==3)//means for JIDO
           {
           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=0;
           } 
          }
         }
         */
        int is_visible=1; 
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
         {
       ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
       ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
        ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
          ////////is_visible=0;
 

          //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          /*
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
          {

          point_to_look[6] = x2;
          point_to_look[7] = y2;
          point_to_look[8] = z2;
          
          //Using perspective taking function
          if(type==1) //for human
          {
          p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
          }
          else
          {
           if(type==2) //for HRP2
           {
          p3d_rob * currRob=ACBTSET->robot;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
           } 
          }
          
          //Using collison test by local path method  
          //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
          
          if(is_visible==1)
          {  
          }
          else
          {
          is_visible=0;
          ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          }
          }
          //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          else*/
          ////////{
          is_visible=0;
          obs_found=1;
          //printf(" Not Visible ");
          not_visible_ctr++;
          ////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          ////////}
         }
        //else
         if(obs_found==0&&is_visible==1)
         {
         if(type==1)//means for human
          {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
          }
        else
          {
        if(type==2)//means for HRP2
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
           }
           else
           {
           if(type==3)//means for JIDO
            {
           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=1;
            } 
           }
          } 
         ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
         visible_ctr++;
         //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
         } 
        }
       }
      }
     //}

     }
    }
  
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);   
    printf(" Visible_ctr=%d, not_visible_ctr=%d\n",visible_ctr,not_visible_ctr); 
}

int update_3d_grid_straight_visibility(int type)//1 means human, 2 means HRP2, 3 means JIDO
{
printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
point_co_ordi eye_pos;
   
eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
    hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
    hum_head_pos[8] = eye_pos.z;
 
    agent_eye_pos.x=eye_pos.x;       
    agent_eye_pos.y=eye_pos.y;
    agent_eye_pos.z=eye_pos.z;

    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

 double interval=grid_around_HRP2.GRID_SET->pace/2.0;
   printf("interval=%lf, no_FOV_end_point_vertices=%d\n",interval,no_FOV_end_point_vertices);
 
int visible_ctr=0;  
int not_visible_ctr=0;
 int j=0;
    for(j=0;j<no_FOV_end_point_vertices;j++)
    {

    int i=4;
    for(i=4;i<8;i++)
     {
   //// if(i<4)
    //// {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
    //// }  
    ////else
    // {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
 double t=0;

      for(;t<1;t+=interval)
      {
      
      double x;
      double y;
      double z;
      
      if(i<7)
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
        } 
      else
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
        }
       ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
      ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
      int obs_found=0;
      double t2=0.1;//Just to avoid very checking very near to human
      
      for(;t2<1;t2+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*eye_pos.x+t2*x;
      double y2=(1-t2)*eye_pos.y+t2*y;
      double z2=(1-t2)*eye_pos.z+t2*z; 
      
  /*  double x2=(1-t2)*x+t*eye_pos.x;
      double y2=(1-t2)*y+t*eye_pos.y;
      double z2=(1-t2)*z+t*eye_pos.z; 
  */
       //////////hri_bitmap_cell* curr_cell=NULL;
  
       ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

       //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
       int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       int cell_y;
       int cell_z;

       int cell_valid=0;       

       if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
       {
      
 
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
       {
       cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
        {
       ////curr_cell=
       ////curr_cell->x=cell_x;
       ////curr_cell->y=cell_y;
       ////curr_cell->z=cell_z;
       cell_valid=1;
        } 
       }
      }

      ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

       ////////if(curr_cell!=NULL)
        if(cell_valid==1)
        { 
        /*
        if(type==1)//means for human
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=0;
         }
        else
         {
        if(type==2)//means for HRP2
          {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;
          }
        else
          {
           if(type==3)//means for JIDO
           {
           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;
           } 
          }
         }
         */

        int is_visible=1; 
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
         {
       ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
       ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
        ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
          ////////is_visible=0;
 

          //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          /*
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
          {

          point_to_look[6] = x2;
          point_to_look[7] = y2;
          point_to_look[8] = z2;
          
          //Using perspective taking function
          if(type==1) //for human
          {
          p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
          }
          else
          {
           if(type==2) //for HRP2
           {
          p3d_rob * currRob=ACBTSET->robot;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
           } 
          }
          
          //Using collison test by local path method  
          //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
          
          if(is_visible==1)
          {  
          }
          else
          {
          is_visible=0;
          ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          }
          }
          //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          else*/
          ////////{
          is_visible=0;
          obs_found=1;
          //printf(" Not Visible ");
          not_visible_ctr++;
          ////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          ////////}
         }
        //else
         if(obs_found==0&&is_visible==1)
         {
         if(type==1)//means for human
          {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
          }
        else
          {
        if(type==2)//means for HRP2
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
           }
           else
           {
           if(type==3)//means for JIDO
            {
           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=1;
            } 
           }
          } 
         ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
         visible_ctr++;
         //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
         } 
        }
       }
      }
     //}

     }
    }
  
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);   
    printf(" Visible_ctr=%d, not_visible_ctr=%d\n",visible_ctr,not_visible_ctr); 
}

int update_3d_grid_visibility_standing(int type)//1 means human, 2 means HRP2
{
printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
point_co_ordi eye_pos;
   
eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
    hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
    hum_head_pos[8] = eye_pos.z;
    
    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

 double interval=grid_around_HRP2.GRID_SET->pace/2.0;
   printf("interval=%lf\n",interval);
 
int visible_ctr=0;  
 int j=0;
    for(j=0;j<no_FOV_end_point_vertices;j++)
    {

    int i=4;
    for(i=4;i<8;i++)
     {
   //// if(i<4)
    //// {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
    //// }  
    ////else
    // {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
 double t=0;

      for(;t<1;t+=interval)
      {
      
      double x;
      double y;
      double z;
      
      if(i<7)
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
        } 
      else
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
        }
       ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
      ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
      int obs_found=0;
      double t2=0.1;//Just to avoid very checking very near to human
      for(;t2<1;t2+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*eye_pos.x+t2*x;
      double y2=(1-t2)*eye_pos.y+t2*y;
      double z2=(1-t2)*eye_pos.z+t2*z; 
      
  /*  double x2=(1-t2)*x+t*eye_pos.x;
      double y2=(1-t2)*y+t*eye_pos.y;
      double z2=(1-t2)*z+t*eye_pos.z; 
  */
       //////////hri_bitmap_cell* curr_cell=NULL;
  
       ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

       //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
       int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       int cell_y;
       int cell_z;

       int cell_valid=0;       

       if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
       {
      
 
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
       {
       cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
        {
       ////curr_cell=
       ////curr_cell->x=cell_x;
       ////curr_cell->y=cell_y;
       ////curr_cell->z=cell_z;
      cell_valid=1;
        } 
       }
      }

      ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

       ////////if(curr_cell!=NULL)
        if(cell_valid==1)
        { 
        /*
        if(type==1)//means for human
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=0;
         }
        else
         {
        if(type==2)//means for HRP2
          {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2=0;
          }
         }
        */
        int is_visible=1; 
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
         {
       ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
       ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
        ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
          ////////is_visible=0;
 

          //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          /*
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
          {

          point_to_look[6] = x2;
          point_to_look[7] = y2;
          point_to_look[8] = z2;
          
          //Using perspective taking function
          if(type==1) //for human
          {
          p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
          }
          else
          {
           if(type==2) //for HRP2
           {
          p3d_rob * currRob=ACBTSET->robot;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
           } 
          }
          
          //Using collison test by local path method  
          //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
          
          if(is_visible==1)
          {  
          }
          else
          {
          is_visible=0;
          ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          }
          }
          //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          else*/
          ////////{
          obs_found=1;
          is_visible=0;
          ////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          ////////}
         }
        //else
         if(obs_found==0&&is_visible==1)
         {
         if(type==1)//means for human
          {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
          }
        else
          {
        if(type==2)//means for HRP2
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
           }
          } 
         ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
         visible_ctr++;
         //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
         } 
        }
       }
      }
     //}

     }
    }
  
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);   
    printf(" Visible_ctr=%d\n",visible_ctr); 
}



int update_3d_grid_visibility_by_neck_turn(int type)//1 means human, 2 means HRP2, 3 means JIDO
{
printf(" Inside update_3d_grid_visibility_by_neck_turn(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
point_co_ordi eye_pos;
   
eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
    hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
    hum_head_pos[8] = eye_pos.z;
    
    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

 double interval=grid_around_HRP2.GRID_SET->pace/2.0;
   printf("interval=%lf\n",interval);
 
int visible_ctr=0;  
 int j=0;
    for(j=0;j<no_FOV_end_point_vertices;j++)
    {

    int i=4;
    for(i=4;i<8;i++)
     {
   //// if(i<4)
    //// {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
    //// }  
    ////else
    // {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
 double t=0;

      for(;t<1;t+=interval)
      {
      
      double x;
      double y;
      double z;
      
      if(i<7)
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
        } 
      else
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
        }
       ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
      ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
      int obs_found=0;
      double t2=0.1;//Just to avoid very checking very near to human
      for(;t2<1;t2+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*eye_pos.x+t2*x;
      double y2=(1-t2)*eye_pos.y+t2*y;
      double z2=(1-t2)*eye_pos.z+t2*z; 
      
  /*  double x2=(1-t2)*x+t*eye_pos.x;
      double y2=(1-t2)*y+t*eye_pos.y;
      double z2=(1-t2)*z+t*eye_pos.z; 
  */
       //////////hri_bitmap_cell* curr_cell=NULL;
  
       ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

       //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
       //////////if(curr_cell!=NULL)
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       int cell_y;
       int cell_z;

       int cell_valid=0;       

       if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
       {
      
 
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
       {
       cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
        {
       ////curr_cell=
       ////curr_cell->x=cell_x;
       ////curr_cell->y=cell_y;
       ////curr_cell->z=cell_z;
      cell_valid=1;
        } 
       }
      }

      ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

       ////////if(curr_cell!=NULL)
        if(cell_valid==1)
        { 
        /*
        if(type==1)//means for human
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=0;
         }
        else
         {
        if(type==2)//means for HRP2
          {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=0;
          }
        else
          //{
           if(type==3)//means for JIDO
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=0;
           }
          //}
         }*/

        int is_visible=1; 
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
         {
       ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
       ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
        ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
          //////////is_visible=0;
 

          //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          /*
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
          {

          point_to_look[6] = x2;
          point_to_look[7] = y2;
          point_to_look[8] = z2;
          
          //Using perspective taking function
          if(type==1) //for human
          {
          p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
          }
          else
          {
           if(type==2) //for HRP2
           {
          p3d_rob * currRob=ACBTSET->robot;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
           } 
          }
          
          //Using collison test by local path method  
          //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
          
          if(is_visible==1)
          {  
          }
          else
          {
          is_visible=0;
          ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          }
          }
          //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          else*/
          ////////{
          is_visible=0;
          obs_found=1;
          ////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          ////////}
         }
        //else
         if(obs_found==0&&is_visible==1)
         {
         if(type==1)//means for human
          {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
          }
        else
          {
        if(type==2)//means for HRP2
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
           }
          else
           {
           if(type==3)//means for JIDO
            {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=1;
            }
           }
          } 
         ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
         visible_ctr++;
         //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
         } 
        }
       }
      }
     //}

     }
    }
  
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);   
  printf(" Visible_ctr=%d\n",visible_ctr); 
}


int update_3d_grid_straight_visibility_standing(int type)//1 means human, 2 means HRP2
{
printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
point_co_ordi eye_pos;
   
eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
    hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
    hum_head_pos[8] = eye_pos.z;
    
    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

 double interval=grid_around_HRP2.GRID_SET->pace/2.0;
   printf("interval=%lf\n",interval);
 
int visible_ctr=0;  
 int j=0;
    for(j=0;j<no_FOV_end_point_vertices;j++)
    {

    int i=4;
    for(i=4;i<8;i++)
     {
   //// if(i<4)
    //// {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
    //// }  
    ////else
    // {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
 double t=0;

      for(;t<1;t+=interval)
      {
      
      double x;
      double y;
      double z;
      
      if(i<7)
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
        } 
      else
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
        }
       ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
      ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
      int obs_found=0;
      double t2=0.1;//Just to avoid very checking very near to human
      for(;t2<1;t2+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*eye_pos.x+t2*x;
      double y2=(1-t2)*eye_pos.y+t2*y;
      double z2=(1-t2)*eye_pos.z+t2*z; 
      
  /*  double x2=(1-t2)*x+t*eye_pos.x;
      double y2=(1-t2)*y+t*eye_pos.y;
      double z2=(1-t2)*z+t*eye_pos.z; 
  */
       //////////hri_bitmap_cell* curr_cell=NULL;
  
       ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

       //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
       //////////if(curr_cell!=NULL)
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       int cell_y;
       int cell_z;

       int cell_valid=0;       

       if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
       {
      
 
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
       {
       cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
        {
       ////curr_cell=
       ////curr_cell->x=cell_x;
       ////curr_cell->y=cell_y;
       ////curr_cell->z=cell_z;
      cell_valid=1;
        } 
       }
      }

      ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

       ////////if(curr_cell!=NULL)
        if(cell_valid==1)
        { 
        /*
        if(type==1)//means for human
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;
         }
        else
         {
        if(type==2)//means for HRP2
          {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation_standing=0;
          }
         }
        */
        int is_visible=1; 
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
         {
       ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
       ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
        ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
          //////////is_visible=0;
 

          //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          /*
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
          {

          point_to_look[6] = x2;
          point_to_look[7] = y2;
          point_to_look[8] = z2;
          
          //Using perspective taking function
          if(type==1) //for human
          {
          p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
          }
          else
          {
           if(type==2) //for HRP2
           {
          p3d_rob * currRob=ACBTSET->robot;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
           } 
          }
          
          //Using collison test by local path method  
          //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
          
          if(is_visible==1)
          {  
          }
          else
          {
          is_visible=0;
          ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          }
          }
          //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          else*/
          ////////{
          obs_found=1;
          is_visible=0;
          ////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          ////////}
         }
        //else
         if(obs_found==0&&is_visible==1)
         {
         if(type==1)//means for human
          {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
          }
        else
          {
        if(type==2)//means for HRP2
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
           }
          } 
         ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
         visible_ctr++;
         //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
         } 
        }
       }
      }
     //}

     }
    }
  
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);   
  printf(" Visible_ctr=%d\n",visible_ctr); 
}


int update_3d_grid_visibility_by_neck_turn_standing(int type)//1 means human, 2 means HRP2
{
printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
point_co_ordi eye_pos;
   
eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
    configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
    hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
    hum_head_pos[8] = eye_pos.z;
    
    configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

 double interval=grid_around_HRP2.GRID_SET->pace/2.0;
   printf("interval=%lf\n",interval);
 
int visible_ctr=0;  
 int j=0;
    for(j=0;j<no_FOV_end_point_vertices;j++)
    {

    int i=4;
    for(i=4;i<8;i++)
     {
   //// if(i<4)
    //// {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
    //// }  
    ////else
    // {
    ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
 double t=0;

      for(;t<1;t+=interval)
      {
      
      double x;
      double y;
      double z;
      
      if(i<7)
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
        } 
      else
        {
      x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
      y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
      z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
        }
       ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
      ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
      int obs_found=0;
      double t2=0.1;//Just to avoid very checking very near to human
      for(;t2<1;t2+=interval) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*eye_pos.x+t2*x;
      double y2=(1-t2)*eye_pos.y+t2*y;
      double z2=(1-t2)*eye_pos.z+t2*z; 
      
  /*  double x2=(1-t2)*x+t*eye_pos.x;
      double y2=(1-t2)*y+t*eye_pos.y;
      double z2=(1-t2)*z+t*eye_pos.z; 
  */
       //////////hri_bitmap_cell* curr_cell=NULL;
  
       ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

       //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
       //////////if(curr_cell!=NULL)
      int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       int cell_y;
       int cell_z;

       int cell_valid=0;       

       if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
       {
      
 
      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
       {
       cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
       if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
        {
       ////curr_cell=
       ////curr_cell->x=cell_x;
       ////curr_cell->y=cell_y;
       ////curr_cell->z=cell_z;
      cell_valid=1;
        } 
       }
      }

      ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

       ////////if(curr_cell!=NULL)
        if(cell_valid==1)
        { 
         /*
        if(type==1)//means for human
         {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=0;
         }
        else
         {
        if(type==2)//means for HRP2
          {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2_neck_turn=0;
          }
         }
         */
        int is_visible=1; 
        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
         {
       ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
       ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
        ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
          //////////is_visible=0;
 

          //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          /*
         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
          {

          point_to_look[6] = x2;
          point_to_look[7] = y2;
          point_to_look[8] = z2;
          
          //Using perspective taking function
          if(type==1) //for human
          {
          p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
          }
          else
          {
           if(type==2) //for HRP2
           {
          p3d_rob * currRob=ACBTSET->robot;
          p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
          is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
           } 
          }
          
          //Using collison test by local path method  
          //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
          
          if(is_visible==1)
          {  
          }
          else
          {
          is_visible=0;
          ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          }
          }
          //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          else*/
          ////////{
          obs_found=1;
          is_visible=0;
          ////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
          ////////}
         }
        //else
         if(obs_found==0&&is_visible==1)
         {
         if(type==1)//means for human
          {
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
          }
        else
          {
        if(type==2)//means for HRP2
           {
          grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2_neck_turn=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
           }
          } 
         ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
         visible_ctr++;
         //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
         } 
        }
       }
      }
     //}

     }
    }
  
    p3d_destroy_config(ACBTSET->visball, hum_head_pos);
    p3d_destroy_config(ACBTSET->visball, point_to_look);   
  printf(" Visible_ctr=%d\n",visible_ctr); 
}


int update_3D_grid_based_on_current_human_pos()
{
 printf("Inside update_3D_grid_based_on_current_human_pos()\n");
 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
       point_co_ordi point;
        point.x=cell_x_world;
        point.y=cell_y_world;
        point.z=cell_z_world;
        //AKP: Following function is very slow for testing visibility for many points  
        int is_visible=is_point_on_surface_visible(point, 1); // 1 means test for Human
        if(is_visible==1)
        {
       //// printf(" point visible \n");
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human=1;
        }
         
       //g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
        //fl_check_forms();
       //g3d_draw_allwin_active();
    ////}
    ////else
    ////{
    ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
    ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
    ////} 
    //  fl_check_forms();
   }
  } 
 }
}

int human_state_updated=0;
int update_human_state(int state) //1 means sitting 0 means standing
{
 
 printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
 if(human_state_updated==0)
 { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  configPt config;
  config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
 hri_human * human=ACBTSET->human[ACBTSET->actual_human];
 ////int state=1;
 
	//  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c7;
	//  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c1;
	//  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c2;
	//  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c3;
	//  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c4;
	//  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c5;
	//  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c6;
    config[8] = human->state[state].c7;
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    // Right Hand 
    config[66] = config[6] + cos(config[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34+0.1;
    // Left Hand 
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34+0.1;
    human->actual_state=state;
	////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

	p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);

	p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
ACBTSET->changed = TRUE;

	if(BTSET!=NULL)
		hri_bt_refresh_all(BTSET);
	if(INTERPOINT!=NULL){
		hri_bt_3drefresh_all(INTERPOINT);
	}
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
	////g3d_draw_allwin_active();
  human_state_updated=0;
 }
 
printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);

}
/*
int update_human_state_new(int state) //1 means sitting 0 means standing
{
 
 printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
 if(human_state_updated==0)
 { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  configPt config;
  config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
 hri_human * human=ACBTSET->human[ACBTSET->actual_human];
 ////int state=1;
 
	//  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c7;
	//  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c1;
	//  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c2;
	//  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c3;
	//  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c4;
	//  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c5;
	//  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c6;
     
    if(strcasestr(human->HumanPt->name,"superman"))
    {
    config[8] = human->state[state].c7;
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    // Right Hand 
    config[66] = config[6] + cos(config[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34+0.1;
    // Left Hand 
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34+0.1;
    }
    else
    {
     if(strcasestr(human->HumanPt->name,"achile"))
     {
   
      config[8] = human->state[state].c7;
      config[32] = human->state[state].c3;
      config[35] = human->state[state].c4;
      config[39] = human->state[state].c1;
      config[42] = human->state[state].c2;
     }   
    } 
    human->actual_state=state;
	////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

	p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);

	p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
ACBTSET->changed = TRUE;

	
 ////g3d_draw_env();
 ////fl_check_forms();
 ////g3d_draw_allwin_active();
	////g3d_draw_allwin_active();
  human_state_updated=0;
 }
 
printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);

}
*/

int virtually_update_human_state(int state) //1 means sitting 0 means standing
{
 
 printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
 if(human_state_updated==0)
 { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  configPt config;
  config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
 hri_human * human=ACBTSET->human[ACBTSET->actual_human];
 ////int state=1;
 
	//  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c7;
	//  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c1;
	//  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c2;
	//  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c3;
	//  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c4;
	//  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c5;
	//  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c6;
 config[8] = human->state[state].c7;
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    // Right Hand 
    config[66] = config[6] + cos(config[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34+0.1;
    // Left Hand 
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34+0.1;
 human->actual_state=state;
	////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

	p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);

	p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
ACBTSET->changed = TRUE;

	if(BTSET!=NULL)
		hri_bt_refresh_all(BTSET);
	if(INTERPOINT!=NULL){
		hri_bt_3drefresh_all(INTERPOINT);
	}
 ////////g3d_draw_env();
 ////////fl_check_forms();
 ////////g3d_draw_allwin_active();
	////g3d_draw_allwin_active();
  human_state_updated=0;
 }
 
printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);

}


int virtually_update_human_state_new(int state) //1 means sitting 0 means standing
{
 
 printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
 if(human_state_updated==0)
 { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  configPt config;
  config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
 hri_human * human=ACBTSET->human[ACBTSET->actual_human];
 ////int state=1;
 
    if(strcasestr(human->HumanPt->name,"superman"))
    {
    config[8] = human->state[state].c7;
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    // Right Hand 
    config[66] = config[6] + cos(config[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34+0.1;
    // Left Hand 
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34+0.1;
    }
    else
    {
     if(strcasestr(human->HumanPt->name,"achile"))
     {
   
      config[8] = human->state[state].c7;
      config[32] = human->state[state].c3;
      config[35] = human->state[state].c4;
      config[39] = human->state[state].c1;
      config[42] = human->state[state].c2;
     }   
    } 
 human->actual_state=state;
	////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

	p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);

	p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
ACBTSET->changed = TRUE;

	/*if(BTSET!=NULL)
		hri_bt_refresh_all(BTSET);
	if(INTERPOINT!=NULL){
		hri_bt_3drefresh_all(INTERPOINT);
	}*/
 ////////g3d_draw_env();
 ////////fl_check_forms();
 ////////g3d_draw_allwin_active();
	////g3d_draw_allwin_active();
  human_state_updated=0;
 }
 
printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);

}

int find_affordance_new()
{
ChronoOn();
////M3D_GIK_TEST();
 ////printf(" Inside find_affordance \n");

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
 {
  printf(" There is collision with human, res=%d \n", res);
  //return 0;
 }
kcd_with_report=0;
 res = p3d_col_test_self_collision(human,kcd_with_report);
 if(res>0)
 {
  printf(" There is self collision with human, res=%d \n", res);
  //return 0;
 }

int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
///////create_HRP2_robot(HRP2_state);

p3d_rob *cur_rob=ACBTSET->robot;

	
	for(int j=0;j<cur_rob->no;j++)
	{
	p3d_obj *o = cur_rob->o[j];
	//if (strstr(o->name,"head") || strstr(o->name,"HEAD") || strstr(o->name,"hand") || strstr(o->name,"HAND"))
	// {
	////p3d_get_object_center(o,objCenter);
        printf("%s\n",o->name);
					// }	  
	}
			
			//define if robot is near or not? here or in observation? od we need a different list
			// if ((ContObjTmp/r->no)>.4)
			// 
			
		


 kcd_with_report=0;
 //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
 res = p3d_col_test_self_collision(cur_rob,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
 if(res>0)
 {
  printf(" There is self collision with robot, res=%d \n", res);
  ////return 0;
 }

kcd_with_report=0;
  res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
 if(res>0)
 {
  printf(" There is collision with robot, res=%d \n", res);
  ////return 0;
 }
 

 if(ONLINE_TRACKING_FLAG==0)
 initialize_surfaces_in_env(); // AKP Note: Comment it if using motion capture button


 
////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 

printf(" Before create_3d_grid_for_HRP2_GIK() \n");
ChronoPrint("***");
create_3d_grid_for_HRP2_GIK();
ChronoPrint("Time for create_3d_grid_for_HRP2_GIK()");
printf(" After create_3d_grid_for_HRP2_GIK() \n");
ChronoOff();

printf(" **** 3D grid dimension is (%d x %d x %d) cells.  \n",grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz);

double cur_h_angle;
double interval;

/*
//////////////////////tmp for jido
ChronoOn();
grid_3d_affordance_calculated=1; 


////p3d_rob *cur_rob=ACBTSET->robot;
cur_h_angle=cur_rob->cam_h_angle;
interval=grid_around_HRP2.GRID_SET->pace;
printf("cur_h_angle=%lf\n",cur_h_angle);
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
 
get_points_on_FOV_screen(cur_rob);
}
cur_rob->cam_h_angle=cur_h_angle;
////ChronoOn();
update_3d_grid_visibility(3); //2 for JIDO

ChronoOff();

update_3d_grid_reachability_for_JIDO_new();


return 1;
///////////////////////end tmp for jido
*/


ChronoOn();

virtually_update_human_state_new(1);// Sitting

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);

//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}

human->cam_h_angle=cur_h_angle;
////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
////ChronoOn();	    

update_3d_grid_visibility(1); //1 for human

ChronoPrint("TIME of 3D Visibility calculation for sitting Human from current position for current head orientation");

//Now making the head straight along of axis of torso
configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */

 p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

double fixed_pitch=M_PI/8.0;

double yaw=0.0;
double pitch=fixed_pitch;
double orig_pan=hum_cur_pos[HUMANq_PAN];
double orig_tilt=hum_cur_pos[HUMANq_TILT];
printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_straight_visibility(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

//Now turning the head only

hum_cur_pos[HUMANq_PAN]=0.0;
yaw=M_PI/3.0;
pitch=fixed_pitch;
////orig_pan=hum_cur_pos[HUMANq_PAN];
////printf(" Original pan = %lf \n",orig_pan); 
hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
////hum_cur_pos[HUMANq_PAN]=0.0;// Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

double tu,ts;
ChronoPrint("TIME of 3D Visibility calculation for sitting Human");
ChronoTimes(&tu,&ts);
printf(" %lf, %lf \n",tu,ts);
ChronoOff();

///////////////3D visibility calculation for making human virtually standing//////////////////////////
/*
ChronoOn();
virtually_update_human_state_new(0);// Standing
cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle for standing human=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}

human->cam_h_angle=cur_h_angle;
////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
////ChronoOn();	    

update_3d_grid_visibility_standing(1); //1 for human

ChronoPrint("TIME of 3D Visibility calculation for standing Human from current position");

//Now making the head straight
hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); 

 p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


yaw=0.0;
pitch=fixed_pitch;
orig_pan=hum_cur_pos[HUMANq_PAN];
orig_tilt=hum_cur_pos[HUMANq_TILT];
printf(" Original pan = %lf \n",orig_pan); 
hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_straight_visibility_standing(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


//Now turning the head only
////hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); 

 ////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


yaw=M_PI/3.0;
pitch=fixed_pitch;
////orig_pan=hum_cur_pos[HUMANq_PAN];
////printf(" Original pan = %lf \n",orig_pan); 

hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);


ChronoPrint("TIME of 3D Visibility calculation for standing Human");
ChronoTimes(&tu,&ts);
////printf(" %lf, %lf \n",tu,ts);
////////update_human_state(1);// Again make it to Sitting
virtually_update_human_state_new(1);// // Again make it to Sitting
ChronoOff();
*/
///////////////END of 3D visibility calculation for making human virtually standing//////////////////////////


ChronoOn();
grid_3d_affordance_calculated=1; 


////p3d_rob *cur_rob=ACBTSET->robot;
cur_h_angle=cur_rob->cam_h_angle;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}
cur_rob->cam_h_angle=cur_h_angle;
////ChronoOn();
#ifdef HRI_JIDO
update_3d_grid_visibility(3); //3 for JIDO
#elif defined(HRI_HRP2)
update_3d_grid_visibility(2); //2 for HRP2
#endif


//Now making the head straight
configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

orig_pan=rob_cur_pos[ROBOTq_PAN];
orig_tilt=rob_cur_pos[ROBOTq_TILT];
printf(" Original pan = %lf \n",orig_pan); 

yaw=0.0;
pitch=fixed_pitch;

rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame

p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

cur_h_angle=cur_rob->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}

#ifdef HRI_JIDO
update_3d_grid_straight_visibility(3); //3 for JIOD
#elif defined(HRI_HRP2)
update_3d_grid_straight_visibility(2); //2 for HRP2
#endif
cur_rob->cam_h_angle=cur_h_angle;

rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


//Now turning the head only
p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
yaw=M_PI/4.0;
pitch=fixed_pitch;
rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

cur_h_angle=cur_rob->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}

#ifdef HRI_JIDO
update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
#elif defined(HRI_HRP2)
update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
#endif

cur_rob->cam_h_angle=cur_h_angle;

rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

rob_cur_pos[ROBOTq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);

ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

cur_h_angle=cur_rob->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}
#ifdef HRI_JIDO
update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
#elif defined(HRI_HRP2)
update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
#endif
//////////update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
cur_rob->cam_h_angle=cur_h_angle;

rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

MY_FREE(rob_cur_pos,double,ACBTSET->robot->nb_dof);


ChronoPrint("TIME of 3D Visibility calculation for ROBOT");
////ChronoOff();
////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
////create_HRP2_robot(HRP2_state);
////g3d_draw_env();
//// fl_check_forms();
//// g3d_draw_allwin_active();


ChronoOff();
ChronoOn();
//// create_HRP2_robot(HRP2_state);
#ifdef HRI_JIDO
update_3d_grid_reachability_for_JIDO_new();
#elif defined(HRI_HRP2)
update_3d_grid_reachability_for_HRP2_new();
#endif
//////////update_3d_grid_reachability_for_HRP2_new();
ChronoPrint("****TIME of 3D reachability calculation for ROBOT\n");



////return 1;
ChronoOff();

ChronoOn();

update_3d_grid_reachability_for_human_new();
/*
//////////update_human_state(0);// Standing
virtually_update_human_state_new(0);// Standing
update_3d_grid_reachability_for_human_standing_new();

////////update_human_state(1);
virtually_update_human_state_new(1);// Sitting
*/
ChronoPrint("****TIME of 3D reachability calculation for human\n");

ChronoOff();


return 1;

//return 1;
/*
ChronoPrint("Time before calculating affordance on surfaces ");
 //current_surface_index=0;
 int i=0;
 for(i=0;i<curr_surfaces_in_env.total_no_of_surfaces;i++)
 {
 printf(" For surface %d\n",i);
 ChronoPrint("Time Before update_surface_grid_based_on_curr_pos");
 update_surface_grid_based_on_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);

 ChronoPrint("Time Before update_surface_grid_by_bending_human_at_curr_pos");
 update_surface_grid_by_bending_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);

 ChronoPrint("Time Before update_surface_grid_by_turning_human_at_curr_pos");
 update_surface_grid_by_turning_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 
 

 
 
 HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
 //////ChronoPrint(" Time before create_HRP2_robot");
 //////create_HRP2_robot(HRP2_state);
 //////ChronoPrint(" Time after create_HRP2_robot and before update_surface_grid_for_HRP2_without_GIK");
 ////update_surface_grid_for_HRP2_with_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 update_surface_grid_for_HRP2_without_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 ChronoPrint(" Time after update_surface_grid_for_HRP2_without_GIK");

 }
*/
////ChronoPrint("TIME for all affordance calculation");
ChronoOff();
////create_3d_grid_for_HRP2_GIK();
////update_3D_grid_based_on_current_human_pos();
//// grid_3d_affordance_calculated=1; 
 
 //update_surface_grid(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[1]);
}

int update_Mightability_Maps()
{
double total_time=0.0;
ChronoOn();
////M3D_GIK_TEST();
 ////printf(" Inside find_affordance \n");

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

/*
int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
 {
  printf(" There is collision with human, res=%d \n", res);
  //return 0;
 }
kcd_with_report=0;
 res = p3d_col_test_self_collision(human,kcd_with_report);
 if(res>0)
 {
  printf(" There is self collision with human, res=%d \n", res);
  //return 0;
 }
*/
int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
///////create_HRP2_robot(HRP2_state);

p3d_rob *cur_rob=ACBTSET->robot;

	/*
	for(int j=0;j<cur_rob->no;j++)
	{
	p3d_obj *o = cur_rob->o[j];
	//if (strstr(o->name,"head") || strstr(o->name,"HEAD") || strstr(o->name,"hand") || strstr(o->name,"HAND"))
	// {
	////p3d_get_object_center(o,objCenter);
        printf("%s\n",o->name);
					// }	  
	}
	*/		
			//define if robot is near or not? here or in observation? od we need a different list
			// if ((ContObjTmp/r->no)>.4)
			// 
			
		

/*
 kcd_with_report=0;
 //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
 res = p3d_col_test_self_collision(cur_rob,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
 if(res>0)
 {
  printf(" There is self collision with robot, res=%d \n", res);
  ////return 0;
 }

kcd_with_report=0;
  res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
 if(res>0)
 {
  printf(" There is collision with robot, res=%d \n", res);
  ////return 0;
 }
 */
/*
 if(ONLINE_TRACKING_FLAG==0)
 initialize_surfaces_in_env(); // AKP Note: Comment it if using motion capture button
*/

 
////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
////update_robots_status();

printf(" Before update_3D_grid_for_Mightability_Maps_new() \n");
ChronoPrint("***");
////update_3d_grid_for_Mightability_Maps();
int expansion=1;
update_3D_grid_for_Mightability_Maps_new(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP);

////if(HUMAN_HAS_MOVED==1)
 ////{
int cell_x,cell_y,cell_z;
  for(cell_x=0; cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx; cell_x++)
  {
     for(cell_y=0; cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny; cell_y++)
   {
       for(cell_z=0; cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz; cell_z++)
    {
 
     if(HUMAN_HAS_MOVED==0&&NEED_HUMAN_VISIBILITY_UPDATE==1)
     {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;
      
     }
      
     if(HUMAN_HAS_MOVED==1)
     {  
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;

     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_LHand=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_RHand=0;
     
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=0;  
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=0;  

     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=0; 
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=0;
     
     NEED_HUMAN_VISIBILITY_UPDATE=1; //To update visibility also
     }

     if(HRP2_HAS_MOVED==0&&NEED_HRP2_VISIBILITY_UPDATE==1)
     {
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=0;
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;
      
     }
 
     if(HRP2_HAS_MOVED==1)
     {  
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;

     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_LHand=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_RHand=0;
     
     NEED_HRP2_VISIBILITY_UPDATE=1; //To update visibility also
     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=0;  
     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=0;  

     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=0; 
     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=0;
     }
    

     if(JIDO_HAS_MOVED==0&&NEED_JIDO_VISIBILITY_UPDATE==1)
     {
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;
     }

     if(JIDO_HAS_MOVED==1)
     {  
     
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=0;
     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;

     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_JIDO_Hand=0;
     
     NEED_JIDO_VISIBILITY_UPDATE=1; //To update visibility also
     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=0;  
     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=0;  

     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=0; 
     ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=0;
     }
    }
   }
  }   
//// }
ChronoPrint("Time for update_3D_grid_for_Mightability_Maps_new()");
printf(" After update_3D_grid_for_Mightability_Maps_new() \n");
double tu,ts;
ChronoTimes(&tu,&ts);
printf(" tu=%lf, ts=%lf \n",tu,ts);
total_time+=tu;//In sec
ChronoOff();

printf(" **** 3D grid dimension is (%d x %d x %d) cells.  \n",grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz);

double cur_h_angle;
double interval=grid_around_HRP2.GRID_SET->pace;
double yaw;
double pitch;
double orig_pan;
double orig_tilt;
double fixed_pitch=M_PI/8.0;

printf(" NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);
/*
//////////////////////tmp for jido
ChronoOn();
grid_3d_affordance_calculated=1; 


////p3d_rob *cur_rob=ACBTSET->robot;
cur_h_angle=cur_rob->cam_h_angle;
interval=grid_around_HRP2.GRID_SET->pace;
printf("cur_h_angle=%lf\n",cur_h_angle);
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
 
get_points_on_FOV_screen(cur_rob);
}
cur_rob->cam_h_angle=cur_h_angle;
////ChronoOn();
update_3d_grid_visibility(3); //2 for JIDO

ChronoOff();

update_3d_grid_reachability_for_JIDO_new();


return 1;
///////////////////////end tmp for jido
*/

if(NEED_HUMAN_VISIBILITY_UPDATE==1)
 {

ChronoOn();

virtually_update_human_state_new(1);// Sitting

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);

//int i_h_a=0;
////////interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}

human->cam_h_angle=cur_h_angle;
////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
////ChronoOn();	    

update_3d_grid_visibility(1); //1 for human

ChronoPrint("TIME of 3D Visibility calculation for sitting Human from current position");

////return 1;

//Now making the head straight
configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */

 p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


double yaw=0.0;
double pitch=fixed_pitch;
double orig_pan=hum_cur_pos[HUMANq_PAN];
double orig_tilt=hum_cur_pos[HUMANq_TILT];
printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw/pan angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_straight_visibility(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


//Now turning the head only
//////configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */

 //////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


yaw=M_PI/3.0;
pitch=fixed_pitch;
//////orig_pan=hum_cur_pos[HUMANq_PAN];
//////printf(" Original pan = %lf \n",orig_pan); 
hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame

p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

////double tu,ts;
ChronoPrint("TIME for all 3D Visibility calculation for sitting Human");
ChronoTimes(&tu,&ts);
printf(" tu=%lf, ts=%lf \n",tu,ts);
total_time+=tu;//In sec
ChronoOff();
 
////return 1;
///////////////3D visibility calculation for making human virtually standing//////////////////////////
/*
ChronoOn();
virtually_update_human_state_new(0);// Standing
cur_h_angle=human->cam_h_angle;
printf(" cur_h_angle for standing human=%lf\n",cur_h_angle);
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}

human->cam_h_angle=cur_h_angle;
////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
////ChronoOn();	    

update_3d_grid_visibility_standing(1); //1 for human

ChronoPrint("TIME of 3D Visibility calculation for standing Human from current position");

//Now making the head straight
hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

 p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


yaw=0.0;
pitch=fixed_pitch;

orig_pan=hum_cur_pos[HUMANq_PAN];
orig_tilt=hum_cur_pos[HUMANq_TILT];
printf(" Original pan = %lf \n",orig_pan); 
hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_straight_visibility_standing(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];



//Now turning the head only
//////hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

 //////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


yaw=M_PI/3.0;
pitch=fixed_pitch;
//////orig_pan=hum_cur_pos[HUMANq_PAN];
//////printf(" Original pan = %lf \n",orig_pan); 
hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame

p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_PAN]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame

p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}
update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
human->cam_h_angle=cur_h_angle;

hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);


ChronoPrint("TIME for all 3D Visibility calculation for standing Human");
ChronoTimes(&tu,&ts);
////printf(" %lf, %lf \n",tu,ts);
virtually_update_human_state_new(1);// Again make it to Sitting
ChronoTimes(&tu,&ts);
printf(" tu=%lf, ts=%lf \n",tu,ts);
total_time+=tu;//In sec
ChronoOff();
*/
NEED_HUMAN_VISIBILITY_UPDATE=0;
///////////////END of 3D visibility calculation for making human virtually standing//////////////////////////

 }//END if(NEED_HUMAN_VISIBILITY_UPDATE==1)
////////return 1;



if(NEED_HRP2_VISIBILITY_UPDATE==1||NEED_JIDO_VISIBILITY_UPDATE==1)
 {
printf(" Need visibility update for robot\n");
ChronoOn();
grid_3d_affordance_calculated=1; 


////p3d_rob *cur_rob=ACBTSET->robot;
cur_h_angle=cur_rob->cam_h_angle;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}
cur_rob->cam_h_angle=cur_h_angle;
////ChronoOn();
#ifdef HRI_JIDO
update_3d_grid_visibility(3); //3 for JIDO
#elif defined(HRI_HRP2)
update_3d_grid_visibility(2); //2 for HRP2
#endif


//Now making the head straight
configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

orig_pan=rob_cur_pos[ROBOTq_PAN];
orig_tilt=rob_cur_pos[ROBOTq_TILT];
yaw=0.0;
pitch=fixed_pitch;
printf(" Original pan = %lf, tilt = %lf \n",orig_pan, orig_tilt); 

rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=pitch; // Human Pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

cur_h_angle=cur_rob->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}

#ifdef HRI_JIDO
update_3d_grid_straight_visibility(3); //3 for JIOD
#elif defined(HRI_HRP2)
update_3d_grid_straight_visibility(2); //2 for HRP2
#endif
cur_rob->cam_h_angle=cur_h_angle;

rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

//Now turning the head only
////configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

 ////p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


yaw=M_PI/4.0;
pitch=fixed_pitch;
////orig_pan=rob_cur_pos[ROBOTq_PAN];
////printf(" Original pan = %lf \n",orig_pan); 
rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=pitch; // Human Pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

cur_h_angle=cur_rob->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}

#ifdef HRI_JIDO
update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
#elif defined(HRI_HRP2)
update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
#endif

cur_rob->cam_h_angle=cur_h_angle;

rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

rob_cur_pos[ROBOTq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=pitch; // Human Pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);

ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

cur_h_angle=cur_rob->cam_h_angle;
//int i_h_a=0;
interval=grid_around_HRP2.GRID_SET->pace;
no_FOV_end_point_vertices=0;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}
#ifdef HRI_JIDO
update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
#elif defined(HRI_HRP2)
update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
#endif
//////////update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
cur_rob->cam_h_angle=cur_h_angle;

rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Pitch angle relative to the human body frame
p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

MY_FREE(rob_cur_pos,double,ACBTSET->robot->nb_dof);


ChronoPrint("TIME of 3D Visibility calculation for ROBOT");
////ChronoOff();
////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
////create_HRP2_robot(HRP2_state);
////g3d_draw_env();
//// fl_check_forms();
//// g3d_draw_allwin_active();

ChronoTimes(&tu,&ts);
printf(" tu=%lf, ts=%lf \n",tu,ts);
total_time+=tu;//In sec
ChronoOff();

 NEED_HRP2_VISIBILITY_UPDATE=0;
 NEED_JIDO_VISIBILITY_UPDATE=0;

 }

 
ChronoOn();
//// create_HRP2_robot(HRP2_state);

#ifdef HRI_JIDO
if(JIDO_HAS_MOVED==1)
 {
 update_3d_grid_reachability_for_JIDO_new();
 JIDO_HAS_MOVED=0;
 }
#elif defined(HRI_HRP2)
if(HRP2_HAS_MOVED==1)
 {
  update_3d_grid_reachability_for_HRP2_new();
  HRP2_HAS_MOVED=0;
 }
#endif
//////////update_3d_grid_reachability_for_HRP2_new();
ChronoPrint("****TIME of 3D reachability calculation for ROBOT\n");

ChronoTimes(&tu,&ts);
printf(" tu=%lf, ts=%lf \n",tu,ts);
total_time+=tu;//In sec

////return 1;
ChronoOff();

ChronoOn();

if(HUMAN_HAS_MOVED==1)
 {
 update_3d_grid_reachability_for_human_new();
 /*
 virtually_update_human_state_new(0);// Standing
update_3d_grid_reachability_for_human_standing_new();
 virtually_update_human_state_new(1);
 */
 HUMAN_HAS_MOVED=0;
 }

ChronoPrint("****TIME of 3D reachability calculation for human\n");
 
ChronoTimes(&tu,&ts);
printf(" tu=%lf, ts=%lf \n",tu,ts);
total_time+=tu;//In sec

ChronoOff();

printf(" <<<<<<<<<< Total Time for updating all the Mightability Maps=%lf s >>>>>>>>>>\n",total_time);

return 1;

//return 1;
/*
ChronoPrint("Time before calculating affordance on surfaces ");
 //current_surface_index=0;
 int i=0;
 for(i=0;i<curr_surfaces_in_env.total_no_of_surfaces;i++)
 {
 printf(" For surface %d\n",i);
 ChronoPrint("Time Before update_surface_grid_based_on_curr_pos");
 update_surface_grid_based_on_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);

 ChronoPrint("Time Before update_surface_grid_by_bending_human_at_curr_pos");
 update_surface_grid_by_bending_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);

 ChronoPrint("Time Before update_surface_grid_by_turning_human_at_curr_pos");
 update_surface_grid_by_turning_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 
 

 
 
 HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
 //////ChronoPrint(" Time before create_HRP2_robot");
 //////create_HRP2_robot(HRP2_state);
 //////ChronoPrint(" Time after create_HRP2_robot and before update_surface_grid_for_HRP2_without_GIK");
 ////update_surface_grid_for_HRP2_with_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 update_surface_grid_for_HRP2_without_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 ChronoPrint(" Time after update_surface_grid_for_HRP2_without_GIK");

 }
*/
////ChronoPrint("TIME for all affordance calculation");
////ChronoOff();
////create_3d_grid_for_HRP2_GIK();
////update_3D_grid_based_on_current_human_pos();
//// grid_3d_affordance_calculated=1; 
 
 //update_surface_grid(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[1]);
}

int find_affordance()
{
ChronoOn();
////M3D_GIK_TEST();
 ////printf(" Inside find_affordance \n");

int kcd_with_report=0;
p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

int res = p3d_col_test_robot(human,kcd_with_report);
 if(res>0)
 {
  printf(" There is collision with human, res=%d \n", res);
  //return 0;
 }
kcd_with_report=0;
 res = p3d_col_test_self_collision(human,kcd_with_report);
 if(res>0)
 {
  printf(" There is self collision with human, res=%d \n", res);
  //return 0;
 }

int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
///////create_HRP2_robot(HRP2_state);

p3d_rob *cur_rob=ACBTSET->robot;

	
	for(int j=0;j<cur_rob->no;j++)
	{
	p3d_obj *o = cur_rob->o[j];
	//if (strstr(o->name,"head") || strstr(o->name,"HEAD") || strstr(o->name,"hand") || strstr(o->name,"HAND"))
	// {
	////p3d_get_object_center(o,objCenter);
        printf("%s\n",o->name);
					// }	  
	}
			
			//define if robot is near or not? here or in observation? od we need a different list
			// if ((ContObjTmp/r->no)>.4)
			// 
			
		


 kcd_with_report=0;
 //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
 res = p3d_col_test_self_collision(cur_rob,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
 if(res>0)
 {
  printf(" There is self collision with robot, res=%d \n", res);
  ////return 0;
 }

kcd_with_report=0;
  res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
 if(res>0)
 {
  printf(" There is collision with robot, res=%d \n", res);
  ////return 0;
 }
 

 
/*
 printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
 if(human_state_updated==0)
 { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  configPt config;
  config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
 hri_human * human=ACBTSET->human[ACBTSET->actual_human];
 int state=1;
 
	//  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c7;
	//  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c1;
	//  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c2;
	//  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c3;
	//  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c4;
	//  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c5;
	//  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c6;
 config[8] = human->state[state].c7;
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    // Right Hand 
    config[66] = config[6] + cos(config[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34+0.1;
    // Left Hand 
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34+0.1;
 human->actual_state=state;
	////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

	p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);

	p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
ACBTSET->changed = TRUE;

	if(BTSET!=NULL)
		hri_bt_refresh_all(BTSET);
	if(INTERPOINT!=NULL){
		hri_bt_3drefresh_all(INTERPOINT);
	}
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
	////g3d_draw_allwin_active();
  human_state_updated=0;
 }
 
printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
*/

 if(ONLINE_TRACKING_FLAG==0)
 initialize_surfaces_in_env(); // AKP Note: Comment it if using motion capture button


 
////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 

printf(" Before create_3d_grid_for_HRP2_GIK() \n");
ChronoPrint("***");
create_3d_grid_for_HRP2_GIK();
ChronoPrint("Time for create_3d_grid_for_HRP2_GIK()");
printf(" After create_3d_grid_for_HRP2_GIK() \n");

double cur_h_angle=human->cam_h_angle;
//int i_h_a=0;
double interval=grid_around_HRP2.GRID_SET->pace;
while(human->cam_h_angle>0.001)
{ 
human->cam_h_angle-=interval;
get_points_on_FOV_screen(human);
}

human->cam_h_angle=cur_h_angle;
////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
////ChronoOn();	    

update_3d_grid_visibility(1); //1 for human
double tu,ts;
ChronoPrint("TIME of 3D Visibility calculation for Human");
ChronoTimes(&tu,&ts);
printf(" %lf, %lf \n",tu,ts);
////ChronoOff();

grid_3d_affordance_calculated=1; 

no_FOV_end_point_vertices=0;
////p3d_rob *cur_rob=ACBTSET->robot;
cur_h_angle=cur_rob->cam_h_angle;
while(cur_rob->cam_h_angle>0.001)
{ 
cur_rob->cam_h_angle-=interval;
get_points_on_FOV_screen(cur_rob);
}
cur_rob->cam_h_angle=cur_h_angle;
////ChronoOn();
update_3d_grid_visibility(2); //2 for HRP2
ChronoPrint("TIME of 3D Visibility calculation for HRP2");
////ChronoOff();
////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
////create_HRP2_robot(HRP2_state);
////g3d_draw_env();
//// fl_check_forms();
//// g3d_draw_allwin_active();

//// create_HRP2_robot(HRP2_state);
update_3d_grid_reachability_for_HRP2();

update_3d_grid_reachability_for_human();

//return 1;

ChronoPrint("Time before calculating affordance on surfaces ");
 //current_surface_index=0;
 int i=0;
 for(i=0;i<curr_surfaces_in_env.total_no_of_surfaces;i++)
 {
 printf(" For surface %d\n",i);
 ChronoPrint("Time Before update_surface_grid_based_on_curr_pos");
 update_surface_grid_based_on_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);

 ChronoPrint("Time Before update_surface_grid_by_bending_human_at_curr_pos");
 update_surface_grid_by_bending_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);

 ChronoPrint("Time Before update_surface_grid_by_turning_human_at_curr_pos");
 update_surface_grid_by_turning_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 
 

 /*
 if(ACBTSET->human[ACBTSET->actual_human]->actual_state==1)//if actually the human is sitting
 {
 update_human_state(0); // Make him virtually standing
 update_surface_grid_by_bending_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 update_human_state(1); // Make him sitting again
 }
 */
 
 HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
 //////ChronoPrint(" Time before create_HRP2_robot");
 //////create_HRP2_robot(HRP2_state);
 //////ChronoPrint(" Time after create_HRP2_robot and before update_surface_grid_for_HRP2_without_GIK");
 ////update_surface_grid_for_HRP2_with_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 update_surface_grid_for_HRP2_without_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
 ChronoPrint(" Time after update_surface_grid_for_HRP2_without_GIK");

 }
ChronoPrint("TIME for all affordance calculation");
ChronoOff();
////create_3d_grid_for_HRP2_GIK();
////update_3D_grid_based_on_current_human_pos();
//// grid_3d_affordance_calculated=1; 
 
 //update_surface_grid(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[1]);
}

// AKP: To draw a cylinder
void g3d_draw_vertical_cylinder(double x, double y, double z, double radius, double height, double slice_distance, int color, double *color_vect)
{
 double i=z;
 for(;i<z+height; i+=slice_distance)
 {
 g3d_drawDisc(x, y, i, radius, color, color_vect);
 }
}


//AKP : To show the different affordance values 
int show_affordance()
{

int show_affordance_type=2;//1 for showing by cylinders, 2 for drwaing vertical line and a circle at top


    int HUMANj_WAIST=7;
    int HUMANj_HIP=34;
    // Reachability Test by right hand
    double hum_waist_pos_x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    double hum_waist_pos_y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[1][3]; // AKP:
    double hum_waist_pos_z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[2][3]; // AKP:

    ////g3d_drawDisc(hum_waist_pos_x, hum_waist_pos_y, hum_waist_pos_z, 0.5, 0.5, NULL);   

int current_surface_index=0;
for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
{

int no_vert=0;
for(;no_vert<curr_surfaces_in_env.flat_surf[current_surface_index].no_vertices-1;no_vert++)
 {
  g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].z,4,NULL);
 }
g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].z,4,NULL);


////printf(" Inside show_affordance grid_i_max =%d , grid_j_max = %d \n", curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max, curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max);
 int i=0;
 for(;i<curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max;i++)
 {
  int j=0;
  int show_cell=0;
  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int visible_by_human=0;
  int visible_by_HRP2=0;
  //printf("\n");
  for(;j<curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max;j++)
  {
   show_cell=0;
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   double x=i*surf_grid_samp_rate;
   double y=j*surf_grid_samp_rate;
   x+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_x_min;
   y+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_y_min;
   double z=curr_surfaces_in_env.flat_surf[current_surface_index].BR_z;

   double cur_z_st=z;
   double height=0.1;//0.3;
   double cur_z_end=z;
   //double cur_z_end=;
   //printf("%d ", curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible);
   ////printf("SHOW_VISIBLE_PLACE=%d\n",SHOW_VISIBLE_PLACE);
   if(SHOW_2D_VISIBLE_PLACE_HUM==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible==1)
    {
 
   //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
   //cur_z_st+=cur_z_end;
   //cur_z_end+=cur_z_st;
    show_cell=1;
    visible_by_human=1;
     ////printf(" Cell %d,%d is visible \n",i,j);
    ////g3d_drawDisc(x, y, z+0.01, surf_grid_samp_rate/2.0, 3, NULL);
    
   
   if(show_affordance_type==1)
     {
  
   g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0, height, 0.01, 3, NULL);
   cur_z_st+=height;
     }
   cur_z_end+=height;
    
    }
   }
   if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible_by_HRP2==1)
    {
 
   //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
   //cur_z_st+=cur_z_end;
   //cur_z_end+=cur_z_st;
    show_cell=1;
    //g3d_drawDisc(x, y, z+0.01, surf_grid_samp_rate/2.0, 3, NULL);
    
   
   if(show_affordance_type==1)
     {
  
   g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0, height, 0.01, 3, NULL);
   cur_z_st+=height;
     }
   cur_z_end+=height;
    
    }
   }
   if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_HRP2_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    if(show_affordance_type==1)
     {
   
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.0075, height, 0.01, 4, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_HRP2_hand==1)//Already reachable by right hand
     reachable_by_HRP2_hand=3; //Reachable by both hands
     else
     reachable_by_HRP2_hand=2; //Reachable by left hand only
  
   ////  g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
     if(show_affordance_type==1)
     {
    
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.015, height, 0.01, 6, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   }
   if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    if(show_affordance_type==1)
     {
   
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.0075, height, 0.01, 4, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only
  
   ////  g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
     if(show_affordance_type==1)
     {
    
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.015, height, 0.01, 6, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   }
   if(SHOW_2D_BENDING_REACHABLE_HUM==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
      reachable_by_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
     if(show_affordance_type==1)
     {
   
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.0075, height, 0.01, 5, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only

   //// g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
    if(show_affordance_type==1)
     {
    
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.015, height, 0.01, 2, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   }
   if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
   {
  /* if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_turning_around==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
    g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.0075, height, 0.01, 5, NULL);
    cur_z_st+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_turning_around==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
    g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.015, height, 0.01, 2, NULL);
    cur_z_st+=height;
    }
   */
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
    show_cell=1;
  reachable_by_hand=1;
 ////   g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    if(show_affordance_type==1)
     {
    
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.0075, height, 0.01, 5, NULL);
    cur_z_st+=height;
     }
    cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
    show_cell=1;

      if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only

  ////  g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
    if(show_affordance_type==1)
     {
    
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.015, height, 0.01, 2, NULL);
    cur_z_st+=height;
     }
    cur_z_end+=height;
    }
   }
  if(show_affordance_type==2)//By line
   {
    if(show_cell==1)
    { 
    ////cur_z_end=z+0.01;
    if(visible_by_human==1)
     {
    ////printf(" Cell %d,%d is visible \n",i,j);
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
    
     }
    cur_z_end=z+0.01;
    g3d_drawOneLine(x,y,z+0.01,x,y,cur_z_end,4,NULL);
    if(reachable_by_hand==1)
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    else
     {
     if(reachable_by_hand==2)
      {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
      }
     else
      {
     if(reachable_by_hand==3) //Reachable by both hands
       { 
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
       }
     //else
     //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
      }
     }
    cur_z_end=z+0.01;
   // g3d_drawOneLine(x,y,z+0.01,x,y,cur_z_end,4,NULL);
    if(reachable_by_HRP2_hand==1)
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    else
     {
     if(reachable_by_HRP2_hand==2)
      {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
      }
     else
      {
     if(reachable_by_HRP2_hand==3) //Reachable by both hands
       { 
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
       }
     //else
     //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
      }
     }
    }
   
   }
  }
 }
}//END for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
/*
int no_path_pts=0;
for(no_path_pts=0;no_path_pts<cur_manipulation_path.total_no_pts;no_path_pts++)
 {
 g3d_drawDisc(cur_manipulation_path.path_points[no_path_pts].x, cur_manipulation_path.path_points[no_path_pts].y, cur_manipulation_path.path_points[no_path_pts].z, 0.01, 4, NULL);
 }
*/

}

//AKP : To show the different affordance values 
int show_affordance_new()
{
no_candidate_points_to_put=0;

int current_surface_index=0;
for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
{

int no_vert=0;
for(;no_vert<curr_surfaces_in_env.flat_surf[current_surface_index].no_vertices-1;no_vert++)
 {
  g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].z,4,NULL);
 }
g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].z,4,NULL);


////printf(" Inside show_affordance grid_i_max =%d , grid_j_max = %d \n", curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max, curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max);
 int i=0;
 for(;i<curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max;i++)
 {
  int j=0;
  int show_cell=0;
  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;
  //printf("\n");
  for(;j<curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max;j++)
  {
   show_cell=0;
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   visible_by_human=0;
   visible_by_HRP2=0;
   reachable_by_turning_around=0;
   reachable_by_bending=0;

   double x=i*surf_grid_samp_rate;
   double y=j*surf_grid_samp_rate;
   x+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_x_min;
   y+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_y_min;
   double z=curr_surfaces_in_env.flat_surf[current_surface_index].BR_z;

   double cur_z_st=z;
   double height=0.1;//0.3;
   double cur_z_end=z;
   //double cur_z_end=;
   //printf("%d ", curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible);
   ////printf("SHOW_VISIBLE_PLACE=%d\n",SHOW_VISIBLE_PLACE);
    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible==1)
    {
 
   //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
   //cur_z_st+=cur_z_end;
   //cur_z_end+=cur_z_st;
    show_cell=1;
    visible_by_human=1;

    }
    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible_by_HRP2==1)
    {
 
   //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
   //cur_z_st+=cur_z_end;
   //cur_z_end+=cur_z_st;
    show_cell=1;
    visible_by_HRP2=1;
    }

    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_hand=1;
    }
    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only
  
    }
     if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
      reachable_by_bending=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_bending==1)//Already reachable by right hand
     reachable_by_bending=3; //Reachable by both hands
     else
     reachable_by_bending=2; //Reachable by left hand only
    }
     if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
    show_cell=1;
  reachable_by_turning_around=1;
 ////   g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
    show_cell=1;

      if(reachable_by_turning_around==1)//Already reachable by right hand
     reachable_by_turning_around=3; //Reachable by both hands
     else
     reachable_by_turning_around=2; //Reachable by left hand only
    }

   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_HRP2_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
   
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_HRP2_hand==1)//Already reachable by right hand
     reachable_by_HRP2_hand=3; //Reachable by both hands
     else
     reachable_by_HRP2_hand=2; //Reachable by left hand only
    }
    
   if(SHOW_2D_VISIBLE_PLACE_HUM==1&&visible_by_human==1)
    {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
    
    }

    if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1&&visible_by_HRP2==1)
    {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
    
    }

    if(SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN==1&&visible_by_human==1&&visible_by_HRP2==1)
    {
     //printf("SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=%d, SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=%d\n",SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN,SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN);
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Green, NULL);
    }

    if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
    {
     if(reachable_by_hand==1)
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    else
     {
     if(reachable_by_hand==2)
      {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
      }
     else
      {
     if(reachable_by_hand==3) //Reachable by both hands
       { 
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
       }
     //else
     //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
      }
     }
    }
     
    if(SHOW_2D_BENDING_REACHABLE_HUM==1)
    {
     if(reachable_by_bending==1)
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    else
     {
     if(reachable_by_bending==2)
      {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
      }
     else
      {
     if(reachable_by_bending==3) //Reachable by both hands
       { 
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
       }
     //else
     //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
      }
     }
    }
   
    if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
    {
     if(reachable_by_turning_around==1)
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    else
     {
     if(reachable_by_turning_around==2)
      {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
      }
     else
      {
     if(reachable_by_turning_around==3) //Reachable by both hands
       { 
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
       }
     //else
     //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
      }
     }
    }
    
   if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
    {
     if(reachable_by_HRP2_hand==1)
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    else
     {
     if(reachable_by_HRP2_hand==2)
      {
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
      }
     else
      {
     if(reachable_by_HRP2_hand==3) //Reachable by both hands
       { 
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
       }
     //else
     //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
      }
     }
    }
    
   if(SHOW_2D_COMMON_REACH_HRP2_HUMAN==1)
    {
    
    if(reachable_by_HRP2_hand==3&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3)) //Reachable by both hands of both human and HRP2
     {
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Green, NULL);
     }
    else
     {
    if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
      {
        g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Red, NULL);
      }
     else
      {
       if((reachable_by_HRP2_hand==3)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
       {
        g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Yellow, NULL);
       }
       else
       {
        if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3))
        {
        g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Blue, NULL);
        }
       } 
      }
     }
    } 
   
    if(SHOW_HRP2_HUMAN_COMMON_REACHABLE_VISIBLE==1)
    {
    if(visible_by_HRP2==1&&visible_by_human==1)
     {
    if(reachable_by_HRP2_hand==3&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3)) //Reachable by both hands of both human and HRP2
      {
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
     
     candidate_points_to_put.point[no_candidate_points_to_put].x=x;
     candidate_points_to_put.point[no_candidate_points_to_put].y=y;
     candidate_points_to_put.point[no_candidate_points_to_put].z=z;
     candidate_points_to_put.no_points++;
     no_candidate_points_to_put++;
     
      }
    if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
      {
     g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
     
     candidate_points_to_put.point[no_candidate_points_to_put].x=x;
     candidate_points_to_put.point[no_candidate_points_to_put].y=y;
     candidate_points_to_put.point[no_candidate_points_to_put].z=z;
     candidate_points_to_put.no_points++;
     no_candidate_points_to_put++;
      }
     } 
    }

  
   
   
  }
 }
}//END for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
/*
int no_path_pts=0;
for(no_path_pts=0;no_path_pts<cur_manipulation_path.total_no_pts;no_path_pts++)
 {
 g3d_drawDisc(cur_manipulation_path.path_points[no_path_pts].x, cur_manipulation_path.path_points[no_path_pts].y, cur_manipulation_path.path_points[no_path_pts].z, 0.01, 4, NULL);
 }
*/

}


/*
int show_common_reachable_places()
{


int current_surface_index=0;
for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
{

int no_vert=0;
for(;no_vert<curr_surfaces_in_env.flat_surf[current_surface_index].no_vertices-1;no_vert++)
 {
  g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].z,4,NULL);
 }
g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].z,4,NULL);


////printf(" Inside show_affordance grid_i_max =%d , grid_j_max = %d \n", curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max, curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max);
 int i=0;
 for(;i<curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max;i++)
 {
  int j=0;
  int show_cell=0;
  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;

  //printf("\n");
  for(;j<curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max;j++)
  {
   show_cell=0;
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   double x=i*surf_grid_samp_rate;
   double y=j*surf_grid_samp_rate;
   x+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_x_min;
   y+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_y_min;
   double z=curr_surfaces_in_env.flat_surf[current_surface_index].BR_z;

   double cur_z_st=z;
   double height=0.1;//0.3;
   double cur_z_end=z;
   //double cur_z_end=;
   //printf("%d ", curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible);
   
   
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_HRP2_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    if(show_affordance_type==1)
     {
   
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.0075, height, 0.01, 4, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_HRP2_hand==1)//Already reachable by right hand
     reachable_by_HRP2_hand=3; //Reachable by both hands
     else
     reachable_by_HRP2_hand=2; //Reachable by left hand only
  
   ////  g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
     if(show_affordance_type==1)
     {
    
    g3d_draw_vertical_cylinder( x, y, cur_z_st, surf_grid_samp_rate/2.0-0.015, height, 0.01, 6, NULL);
    cur_z_st+=height;
     }
     cur_z_end+=height;
    }
   
   if(SHOW_DIRECT_REACHABLE==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
     cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only
  
   
     cur_z_end+=height;
    }
   }
   if(SHOW_BENDING_REACHABLE==1)
   {
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
      reachable_by_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
     cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only

   //// g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
    
     cur_z_end+=height;
    }
   }
   if(SHOW_TURNING_AROUND_REACHABLE==1)
   {
  
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
    show_cell=1;
  reachable_by_hand=1;
 ////   g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
    cur_z_end+=height;
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
    show_cell=1;

      if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only

  ////  g3d_drawDisc(x, y, z+0.03, surf_grid_samp_rate/2.0-0.015, 6, NULL);
    
    cur_z_end+=height;
    }
   }
  if(show_affordance_type==2)//By line
   {
    if(show_cell==1)
    { 
   //// g3d_drawDisc(x, y, z+0.01, surf_grid_samp_rate/2.0, 3, NULL);
    cur_z_end=z+0.01;
    g3d_drawOneLine(x,y,z+0.01,x,y,cur_z_end,4,NULL);
    if((reachable_by_hand==1||reachable_by_hand==2)&&(reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2))
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
    if((reachable_by_hand==3)&&(reachable_by_HRP2_hand==3))
    g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
    }
   
   
   }
  }
 }
}//END for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)


}
*/

void update_human_state_old(int state) //1 means sitting 0 means standing
{
  configPt q;
  q = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */
  //p3d_rob* robotPt;
  printf(" Inside update_human_state \n");
	
  ACBTSET->human[ACBTSET->actual_human]->actual_state = state;
  //q = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);  
  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c7;
  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c1;
  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c2;
  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c3;
  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c4;
  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c5;
  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c6;
	
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,q);

  
  /*robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_sel_desc_name(P3D_ROBOT,ACBTSET->human[ACBTSET->actual_human]->HumanPt->name);
	
  p3d_set_and_update_robot_conf(q); 
  p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
  */
      
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[8]  = q[8];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[43] = q[43];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[44] = q[44];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[46] = q[46];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[47] = q[47];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[50] = q[50];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[53] = q[53];

  
  //    ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[6]  = q[6];
   //   ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[7]  = q[7];
    //  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[11] = q[11];
      //if(fabs(forehead_pos.theta-hum_cur_pos[11])>M_PI/4)
     // ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[64] = q[64];
     // ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[65] = q[65];

  //p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, q);
  MY_FREE(q, double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);  /* Freeing temporary robot config structure */
	
  
}

/*
void update_human_state(int state) //1 means sitting 0 means standing
{
  configPt q;
  p3d_rob* robotPt;
  printf(" Inside update_human_state \n");
	
  ACBTSET->human[ACBTSET->actual_human]->actual_state = state;
  q = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);  
  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c7;
  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c1;
  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c2;
  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c3;
  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c4;
  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c5;
  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c6;
	
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_sel_desc_name(P3D_ROBOT,ACBTSET->human[ACBTSET->actual_human]->HumanPt->name);
	
  p3d_set_and_update_robot_conf(q); 
  p3d_sel_desc_name(P3D_ROBOT,robotPt->name);

      
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[8]  = q[8];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[43] = q[43];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[44] = q[44];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[46] = q[46];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[47] = q[47];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[50] = q[50];
      ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[53] = q[53];

  
  //    ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[6]  = q[6];
   //   ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[7]  = q[7];
    //  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[11] = q[11];
      //if(fabs(forehead_pos.theta-hum_cur_pos[11])>M_PI/4)
     // ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[64] = q[64];
     // ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[65] = q[65];

  p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, q);
	
  
}

*/

// ****** AKP : Automatic surface detection *****////
/*
int detect_horizontal_surfaces()
{
   curr_surfaces_in_env.total_no_of_surfaces=0;

AKP_env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

double env_x1=AKP_env->box.x1;
double env_y1=AKP_env->box.y1;
double env_x2=AKP_env->box.x2;
double env_y2=AKP_env->box.y2;curr_surfaces_in_env.flat_surf[current_surface_index].

  double x_coord;
  double y_coord;
  double z_coord=2;

  //env_x1=-15;
  //env_y1=-12;
  //env_x2=15;
  //env_y2=12;

  for(x_coord=env_x1;x_coord<=env_x2;x_coord+=0.05)
  {
   for(y_coord=env_y1;y_coord<=env_y2;y_coord+=0.05)
   {
    ////printf(" For x_coord=%lf, y_coord=%lf \n", x_coord, y_coord);
     
    current_cell = hri_bt_get_cell(bitmap,(int)((x_coord-ACBTSET->realx)/ACBTSET->pace),
					 (int)((y_coord-ACBTSET->realy)/ACBTSET->pace),
					 (int)(0));  
    
    if(current_cell == NULL) 
    {
    ////printf(" For x_coord=%lf, y_coord=%lf current cell is NULL\n", x_coord, y_coord);
    ////PrintWarning(("AKP Warning: Point for finding Obstacle does not exist\n"));
    //bitmapset->pathexist = FALSE;
    //return -4;
    }
    else
    {
     
     if(ACBTSET->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].val <0) //|| bitmapset->bitmap[BT_COMBINED]->calculate_cell_value(bitmapset,current_cell->x,current_cell->y,current_cell->z)<0)
     {
      
     }
    }
   }
  }
}

*/


double get_cell_value_3D_grid(hri_bitmapset * btset, int x, int y, int z)
{
  if(btset == NULL){
    PrintWarning(("Cant get obstacle value: btset=null"));
    return -1;
  }
  if(btset->bitmap[HRP2_GIK_MANIP] == NULL){
    PrintWarning(("Cant get obstacle value: bitmap=null"));
    return -1;
  }

  /*if(btset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val == -1)
    return -1;
  else
    return 0;
  */
   return btset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val;
}

int hri_bt_initialize_affordance_data(hri_bitmapset* bitmapset,int bt_type)
{
  int x,y,z;
  for(x=0; x<bitmapset->bitmap[bt_type]->nx; x++)
  {
     for(y=0; y<bitmapset->bitmap[bt_type]->ny; y++)
   {
       for(z=0; z<bitmapset->bitmap[bt_type]->nz; z++)
    {
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human = 0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn = 0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human_torso_neack_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_standing_human = 0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_standing_human_neck_turn = 0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_standing_human_torso_neack_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;

  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_human_LHand = 0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_human_RHand = 0;
  
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending=0; 

  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_turning_around=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_turning_around=0;

  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=0;  

  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_standing=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_standing=0;

  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_standing_bending=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_standing_bending=0;

  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_standing_turning_around=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_standing_turning_around=0;


  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_standing_turning_around_bending=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_standing_turning_around_bending=0;


//For HRP2
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_HRP2=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_HRP2_torso_neck_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_standing_HRP2=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_standing_HRP2_neck_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_standing_HRP2_torso_neck_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation_standing=0;

//For Jido
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand=0; 
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_JIDO=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn=0;
  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;
    }
   }
  }   
}


hri_bitmapset* create_3D_grid(int BB_length,int BB_width,int BB_height, double sampling_rate)
{
  ChronoPrint("<<<<<<<<Entering create_3D_grid");
  int dimx,dimy, dimz;
  configPt humanConf;
  double hx,hy,hz;
  double Ccoord[6];
  
  double xsize=6,ysize=6,zsize=3;

  hri_bitmapset * btset;
  btset = hri_bt_create_bitmaps();

  btset->pace = sampling_rate;

  dimx = (int)(BB_length/sampling_rate);
  dimy = (int)(BB_width/sampling_rate);
  dimz = (int)(BB_height/sampling_rate);

  btset->n = 1;
  btset->bitmap = MY_ALLOC(hri_bitmap*,btset->n);
  
  
  btset->bitmap[HRP2_GIK_MANIP]=hri_bt_create_empty_bitmap(dimx, dimy, dimz, sampling_rate, HRP2_GIK_MANIP, get_cell_value_3D_grid); // AKP : get_cell_value_3D_grid is the function name passed as the argument and will be assigned to the function calculate_cell_value of the btset.
   ////btset->bitmap[BT_AFFORDANCE_VISIBILITY]=hri_bt_create_empty_bitmap(dimx, dimy, dimz, sampling_rate, BT_AFFORDANCE_VISIBILITY, get_cell_value_3D_grid); // AKP : get_cell_value_3D_grid is the function name passed as the argument and will be assigned to the function calculate_cell_value of the btset.
  int i=0;

  for(;i<btset->n;i++)
  {
  if(btset->bitmap[i] == NULL)
   {
    printf("AKP WARNING : Could not create the empty bitmap %d inside the function create_3D_grid, so returning with NULL \n",i);
    return NULL;
   }
  }
  // create all cells
  for(i=0;i<btset->n;i++)
  {
  hri_bt_create_data(btset->bitmap[i]); // AKP : Creates necessary data field for an empty bitmap and populate with the value 0
  hri_bt_initialize_affordance_data(btset,i);
  }
  
  

  btset->path = NULL;
  btset->pathexist = FALSE;
  btset->combine_type = BT_COMBINE_SUM; /* default value */
  btset->changed = FALSE;

  

  //hri_exp_fill_obstacles(btset);
 ChronoPrint(">>>>>>>>>>>>Returning create_3D_grid");
  return btset;
}

//AKP : For populating the obstacles in the bitmap 

int create_obstacles_for_HRP2_GIK_manip( hri_bitmapset* btset )
{
  int i;
  p3d_env* env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int expand_rate, minimum_expand_rate;
  configPt robotq;
	
  if(btset == NULL)
    return FALSE;
  
  
  
  if(btset->robot == NULL)
    {
      expand_rate = 0;
      PrintWarning(("WARNING: btset->robot NULL\n"));
    }
  /*else {
    robotq = p3d_get_robot_config(btset->robot);
    //  expand_rate = (bitmapset->robot->BB.xmax-bitmapset->robot->BB.xmin>bitmapset->robot->BB.ymax-bitmapset->robot->BB.ymin)? 
    //       ((bitmapset->robot->BB.xmax-bitmapset->robot->BB.xmin)/(bitmapset->bitmap[BT_OBSTACLES]->pace*2)): 
    //       ((bitmapset->robot->BB.ymax-bitmapset->robot->BB.ymin)/(bitmapset->bitmap[BT_OBSTACLES]->pace*2)); 
    if(DISTANCE2D(btset->robot->BB.xmax,btset->robot->BB.ymax,robotq[ROBOTq_X],robotq[ROBOTq_Y]) >
       DISTANCE2D(btset->robot->BB.xmin,btset->robot->BB.ymin,robotq[ROBOTq_X],robotq[ROBOTq_Y]))
      
      expand_rate = (btset->robot->BB.xmax-robotq[ROBOTq_X] > btset->robot->BB.ymax-robotq[ROBOTq_Y])?
	((btset->robot->BB.xmax-robotq[ROBOTq_X])/btset->pace):
	((btset->robot->BB.ymax-robotq[ROBOTq_Y])/btset->pace);
    
    else
      expand_rate = (btset->robot->BB.xmin-robotq[ROBOTq_X] > btset->robot->BB.ymin-robotq[ROBOTq_Y])?
	((robotq[ROBOTq_X]-btset->robot->BB.xmin)/btset->pace):
	((robotq[ROBOTq_Y]-btset->robot->BB.ymin)/btset->pace);	 
    
    p3d_destroy_config(btset->robot,robotq);
    //expand_rate--;		
  }


expand_rate=1;

  hri_bt_reset_bitmap_data(btset->bitmap[BT_OBSTACLES]);
  minimum_expand_rate = (int) (0.40/btset->pace) - 1; 	
  
  printf("expand rate %i min rate %i \n",expand_rate, minimum_expand_rate);

  if (expand_rate <=  minimum_expand_rate )
    expand_rate += minimum_expand_rate;
*/
  
  double expand_rate_for_HRP2_GIK=0.01;
  double minimum_expand_rate_for_HRP2_GIK=0.005;

  for(i=0; i<env->no ; i++){  
    printf(" Inserting OBJECT %s \n",env->o[i]->name); 
    hri_bt_insert_obs(btset,btset->bitmap[HRP2_GIK_MANIP], env->o[i], env, minimum_expand_rate_for_HRP2_GIK, -2, 1);
    // potential 3d collision
    //hri_bt_insert_obs(btset,btset->bitmap[HRP2_GIK_MANIP], env->o[i], env, safe_expand_rate, BT_OBST_POTENTIAL_OBJECT_COLLISION, 0);
    //hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, expand_rate_for_voronoi, -1)  ;
    /* if(hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, expand_rate, -1)) */
    /* printf("Obstacle placed\n"); */
  }  

 /* for(i=0; i<env->no ; i++){                             
    // hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, (int)(expand_rate/2)+1, -2);
     hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, minimum_expand_rate_for_voronoi, -2);
    
  }*/

  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  for(i=0; i<env->nr; i++){
    
     int is_robot=0;
     int is_human=0;
     int is_HRP2_chair=0;
     //printf(" Inserting ROBOT %s \n",env->robot[i]->name);
     if(!strncmp("HRP2ROBOT",env->robot[i]->name,9))
     is_robot=1;
     if((!strncmp("human",env->robot[i]->name,5))||(!strncmp("HUMAN",env->robot[i]->name,5)))
     is_human=1;
     if(!strncmp("HRP2CHAIR",env->robot[i]->name,9))
     is_HRP2_chair=1;
    
     if(is_robot==0&&is_human==0&& is_HRP2_chair==0)
     {
   ////  if(dynamic_obj[i].is_curr_visible==1)
      ////{
        printf(" Inserting ROBOT %s \n",env->robot[i]->name);
       hri_bt_insert_obsrobot(btset, btset->bitmap[HRP2_GIK_MANIP], env->robot[i], env, minimum_expand_rate_for_HRP2_GIK, -2, 1);
     //hri_bt_insert_1obs2bitmaprobot(btset,btset->bitmap[BT_OBSTACLES],env->robot[i] , env, minimum_expand_rate_for_voronoi, -2);
      ////printf("Obstacles updated for %s\n",env->robot[i]->name);
      ////printf(" BB : %lf, %lf, %lf, %lf, %lf, %lf \n",env->robot[i]->BB.xmin,env->robot[i]->BB.xmax,env->robot[i]->BB.ymin,env->robot[i]->BB.ymax,env->robot[i]->BB.zmin,env->robot[i]->BB.zmax);
      ////}
     
    }
  }
  
  return TRUE;  
}

int update_HRP2_in_bitmap(p3d_rob* rob)
{
 //double i=0;
 
 //for(i=rob->BB.xmin;i<rob->BB.xmax;i
 
}

int disactivate_collision_among_parts_of_HRP2_RHAND()
{
  printf(" Inside disactivate_collision_among_parts_of_HRP2_RHAND()\n");
  envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 int nr = envPt->nr;
  p3d_obj **o, *tmp_o;
  p3d_rob *r;
  int x,y,z;
  
  int hand_part_ctr=0;
 

  int r_ctr=0;
  
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {
  
  r = envPt->robot[r_ctr];
   /////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
   if(strcasecmp(r->name,"HRP2ROBOT")==0)//Exact match
   {
   r_ctr=100000; 
   printf("HRP2ROBOT has been found, Now disactivating collision among its Right hand parts\n");
   o = MY_ALLOC(p3d_obj*,r->no);
   int r_o_ctr=0; 
   for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
    {
    tmp_o = r->o[r_o_ctr];
    
    /////printf(" tmp_o->name = %s \n",tmp_o->name);
    if(strcasecmp(tmp_o->name,"HRP2ROBOT.RHAND")>=15)//Atleast 15 char match
     {
    o[hand_part_ctr] = r->o[r_o_ctr];
    hand_part_ctr++;
     }
    }
   }  
  }
 printf(" hand_part_ctr=%d\n",hand_part_ctr);
 int i=0;
 int j=0;
 for(i=0;i<hand_part_ctr;i++)
 {
  for(j=0;j<hand_part_ctr;j++)
  {
  if(i!=j)
   {
  printf(" Disactivating collision between %s & %s \n",o[i]->name,o[j]->name);
  p3d_col_deactivate_pair_of_objects(o[i], o[j]);
   }
  }
 }
MY_FREE(o, p3d_obj*, r->no);
}

int make_cells_around_HRP2_RHNAD_as_non_obstacle(hri_bitmapset * bitmapset,  int bt_type, double expansion) //TODO : the argument expansion is in m not in terms of no. of cell, and is used as the expansion around BB coordinates, write code to use it
{
  double increment=3.0/4.0*bitmapset->pace;
  envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int x,y,z;
  hri_bitmap * bitmap;

  bitmap = bitmapset->bitmap[bt_type];

  int r_ctr=0;
  
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {
  
  r = envPt->robot[r_ctr];
  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
  int r_o_ctr=0; 
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   if(strcasecmp(o->name,"HRP2ROBOT.RHAND")>=15)
    {
    printf(" o->name = %s \n",o->name);
    printf(" Hand part found ... making the cell values around this hand part= 0\n");
   double BBx;
   for(BBx=o->BB.xmin-expansion;BBx<o->BB.xmax+expansion;BBx+=increment)
     {
    
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin-expansion;BBy<o->BB.ymax+expansion&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin-expansion;BBz<o->BB.zmax+expansion&&y>0&&y<bitmap->ny;BBz+=increment)
      {
     
      z=(BBz- bitmapset->realz)/bitmapset->pace;  
      if(z>0&&z<bitmap->nz)
       {
        printf(" Making cell (%d,%d,%d) as value 0 \n",x,y,z); 
        bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  
       }
      }
     }
    }
   }
  }
 } 
}

int show_exact_obstacles_for_HRP2_GIK_manip(hri_bitmapset * bitmapset, int bt_type)
{
  if(bt_type==HRP2_GIK_MANIP)
 {
  hri_bitmap * bitmap;
bitmap = bitmapset->bitmap[bt_type];
   int x,y,z;
  for(x=0; x<bitmap->nx; x++)
  {
     for(y=0; y<bitmap->ny; y++)
   {
       for(z=0; z<bitmap->nz; z++)
    {
   if(bitmapset->bitmap[bt_type]->data[x][y][z].val < 0)  
     {
      double tmp_x  = x*bitmapset->pace+bitmapset->realx;
	double tmp_y  = y*bitmapset->pace+bitmapset->realy;
	 double tmp_z  = z*bitmapset->pace+bitmapset->realz;
     if(bitmapset->bitmap[bt_type]->data[x][y][z].val == -3)  
     g3d_drawDisc(tmp_x, tmp_y, tmp_z, 0.02, 3, NULL);
     else
     g3d_drawDisc(tmp_x, tmp_y, tmp_z, 0.02, 4, NULL);
     }
    }
   }
  }  
  g3d_drawDisc(point_of_curr_collision.x, point_of_curr_collision.y,point_of_curr_collision.z, 0.1,4, NULL);
 } 
}

robots_status robots_status_for_Mightability_Maps[100];
int update_robots_and_objects_status()
{
   envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  //double cur_x;
  //double cur_y;
  //double cur_z;
  HUMAN_HAS_MOVED=0;
 
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {
  r = envPt->robot[r_ctr];

  //double cur_x=r->ROBOT_POS[6];
   if(strcasestr(r->name,"visball"))
   {
   }
  else
   {
  if(robots_status_for_Mightability_Maps[r_ctr].has_moved==0)
    {
  
  if(fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]-r->ROBOT_POS[6])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]-r->ROBOT_POS[7])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]-r->ROBOT_POS[8])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[9]-r->ROBOT_POS[9])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[10]-r->ROBOT_POS[10])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[11]-r->ROBOT_POS[11])>=0.01)
     {
    
    robots_status_for_Mightability_Maps[r_ctr].has_moved=1;
      
    printf(" >>>> Robot = %s has moved.\n",r->name);
    /*
    robots_status_for_Mightability_Maps[r_ctr].prev_x=robots_status_for_Mightability_Maps[r_ctr].curr_x;
    robots_status_for_Mightability_Maps[r_ctr].prev_y=robots_status_for_Mightability_Maps[r_ctr].curr_y;
    robots_status_for_Mightability_Maps[r_ctr].prev_z=robots_status_for_Mightability_Maps[r_ctr].curr_z;
    
    robots_status_for_Mightability_Maps[r_ctr].curr_x=r->ROBOT_POS[6];
    robots_status_for_Mightability_Maps[r_ctr].curr_y=r->ROBOT_POS[7];
    robots_status_for_Mightability_Maps[r_ctr].curr_z=r->ROBOT_POS[8];
    */
      if(strcasestr(r->name,"HUMAN"))
      {
       HUMAN_HAS_MOVED=1;
      }
      else
      {
       if(strcasestr(r->name,"HRP2"))
       {
       HRP2_HAS_MOVED=1;
       }
       else
       {
        if(strcasestr(r->name,"JIDO"))
        {
        JIDO_HAS_MOVED=1;
        }
       } 
      }
     }
     
    }
   }
  }
}

int update_3D_grid_for_Mightability_Maps(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  configPt visq;
  configPt cur_rob_pos;
  hri_bitmap * bitmap;
  double increment=3.0/4.0*bitmapset->pace;
  visq = p3d_get_robot_config(bitmapset->visball);
  bitmap = bitmapset->bitmap[bt_type];
   envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
   int x,y,z;

  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {
   if(robots_status_for_Mightability_Maps[r_ctr].has_moved==1)
   {
  r = envPt->robot[r_ctr];

  cur_rob_pos=MY_ALLOC(double,r->nb_dof); 
  p3d_get_robot_config_into(r,&cur_rob_pos);
  
  printf(" Robot name = %s \n",r->name);

    NEED_HUMAN_VISIBILITY_UPDATE=1;
   #ifdef HRI_JIDO
    NEED_JIDO_VISIBILITY_UPDATE=1;
   #elif defined(HRI_HRP2)
   NEED_HRP2_VISIBILITY_UPDATE=1;
   #endif
  
  printf(" inside update_3D_grid_for_Mightability_Maps, NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);
  ////**** First making previously occupied cells as free cells  
  p3d_set_and_update_this_robot_conf(r, robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
  p3d_col_deactivate_rob_rob(bitmapset->visball,r);//To make a cell occupied only if there is collision of visball with other objects not with robot
  int r_o_ctr=0; 
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   double BBx;
   for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
    {
    visq[6] = BBx;
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     visq[7]  = BBy;
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
      {
      visq[8]  = BBz;
      z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

      if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
       {
        ////bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle

        p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
       int kcd_with_report=0;
       int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
       if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
        {
         //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
       bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
        
        
        
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
                 {
                 }
                 else
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
              }
             } 
            }
           }
          } 
         } 
        }
       else
        {
        
      /* 
       bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //No obstacle
        
      
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //No obstacle
              }
             } 
            }
           }
          } 
         } 
         */
       
        }
       }
      }
     }
    }
   }
   p3d_col_activate_rob_rob(bitmapset->visball,r);
   p3d_set_and_update_this_robot_conf(r, cur_rob_pos);
  ////*****END making the previously occupied cells as free cells  

  ////*****Now creating exact obstacle at new place of robot
  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
  r_o_ctr=0; 
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   double BBx;
   for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
    {
    visq[6] = BBx;
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     visq[7]  = BBy;
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
      {
      visq[8]  = BBz;
      z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

      if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
       {
        p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
       int kcd_with_report=0;
       int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
       if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
        {
         //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
       bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
       
      
       //else
        //{
        //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
       // }
          
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
                 {
                 }
                 else
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
              }
             } 
            }
           }
          } 
         } 
        }
       ////else
       ////{
       ////printf("Inside BB but no collison\n");
       ////exit(0);
       ////}
       }
      }   
     }
    }
    }
 
   robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
   ////p3d_set_and_update_this_robot_conf(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config,r->ROBOT_POS);
  ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]=r->ROBOT_POS[6];
  ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]=r->ROBOT_POS[7];
  ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]=r->ROBOT_POS[8];
  p3d_copy_config_into(r,r->ROBOT_POS, &(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config));
  ////////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
   }
   
  }
  

  
}


int update_3D_grid_for_Mightability_Maps_new(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  configPt visq;
  configPt cur_rob_pos;
  hri_bitmap * bitmap;
  double increment=3.0/4.0*bitmapset->pace;
  visq = p3d_get_robot_config(bitmapset->visball);
  bitmap = bitmapset->bitmap[bt_type];
   envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
   int x,y,z;

  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {
    
   if(robots_status_for_Mightability_Maps[r_ctr].has_moved==1)
   {
  r = envPt->robot[r_ctr];

  cur_rob_pos=MY_ALLOC(double,r->nb_dof); 
  p3d_get_robot_config_into(r,&cur_rob_pos);
  
  printf(" Movable Object name = %s \n",r->name);
   
   NEED_HUMAN_VISIBILITY_UPDATE=1;
   #ifdef HRI_JIDO
    NEED_JIDO_VISIBILITY_UPDATE=1;
   #elif defined(HRI_HRP2)
   NEED_HRP2_VISIBILITY_UPDATE=1;
   #endif
  
  printf(" inside update_3D_grid_for_Mightability_Maps_new(), NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);
    
  ////**** First making previously occupied cells as free cells  
  p3d_set_and_update_this_robot_conf(r, robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
  ////////p3d_col_deactivate_rob_rob(bitmapset->visball,r);//To make a cell occupied only if there is collision of visball with other objects not with robot
  int r_o_ctr=0; 
  int cell_ctr_tmp=0;
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   double BBx;
   for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
    {
    visq[6] = BBx;
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     visq[7]  = BBy;
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
      {
      visq[8]  = BBz;
      z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

      if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
       {
        ////bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
 
        /*if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
        {
         printf(" Cell belongs to bottle only\n");
        }*/

        cell_ctr_tmp++;
         //////////printf(" cell_ctr=%d: %d, %d\n",cell_ctr_tmp,bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr],bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);
        if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==0)//NOT the cell just above the  horizontal surface of table
        {
        //////////printf(" Not horizontal surface\n");
        ////printf(" cell_ctr=%d, %d, %d\n",cell_ctr_tmp,bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr],bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);

        ////////////if((bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==0)||(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==0)||(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1))
         if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1)
          {  
           bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
           ////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;

          if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
           {
            bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0; 

           if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==0)
            {
             //////////printf(" Cell belongs to the displaced object only and no. of other close object=0\n");
             bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
            
            }
            else//No of close object is >=1
            {
             if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1)//No. of close obj =1 
             {
              if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The close object is this object itself, so remove the object from the close as well as from the belonging object list
              { 
               //////////printf(" Cell belongs to the displaced object only\n");
               bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle

               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
              }
              else//There is one close object but it is not this object, so just remove this object from the close object list
              {
             
              bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
              }
             }
             else//No. of close obj >1, 
             {
              if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//One of the close object is this object itself, so remove the object from the close object list
              { 
              
               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
              }
             
             } 
            
            } 
           }
           else
           {
          //bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
          bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects--;
           } 
          }//End if the cell belongs to this object
          else //The cell does not belong to this object
          {
           if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The cell close to this object, so remove from the list of close objects
           {
            bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   ////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
            
            if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1)//This object is the only object close to the cell, so make the cell free
            {
             bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
             bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
            }
            else
            {
             bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
            }
           }
            

          }// End else of if the cell belongs to this object
        }//End if NOT the cell just above the  horizontal surface of table
        
        
        
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 //if((bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)||(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1))
               if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.is_horizontal_surface==0)//NOT the cell just above the horizontal surface of table
               {
                if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1)
                {  
                bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
                ////bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belong_to_objects--;

                if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
                 {
                 bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belongs_to_objects=0; 

                 if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects==0)
                  {
                  //////////printf(" Cell belongs to the displaced object only\n");
                  bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //no obstacle
            
                  }
                  else//No of close object is >=1
                  {
                  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects==1)//No. of close obj =1 
                   {
                    if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The close object is this object itself, so remove the object from the close as well as from the belonging object list
                    { 
                     //////////printf(" Cell belongs to the displaced object only\n");
                     bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //no obstacle

                     bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects=0;
                    }
                    else//There is one close object but it is not this object, so just remove this object from the close object list
                    {
             
                     bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
                    }
                   }
                   else//No. of close obj >1, 
                   {
                   if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//One of the close object is this object itself, so remove the object from the close object list
                    { 
              
                     bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
                    }
                    
                   } 
            
                  } 
                 }
                 else
                 {
                  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belongs_to_objects--;
                 }
                }//End if the cell belongs to this object
                else //The cell does not belong to this object
                {
                if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The cell close to this object, so remove from the list of close objects
                 {
                 bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   ////bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
            
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects==1)//This object is the only object close to the cell, so make the cell free
                  {
                   bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //no obstacle
                   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects=0;
                  }
                 else
                  {
                   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
                  }
                 }
                }// End else of if the cell belongs to this object 
               }
              }
             } 
            }
           }
          } 
         } 
       
       
        }
       }
      }
     }
    }
   
   ////////p3d_col_activate_rob_rob(bitmapset->visball,r);
   p3d_set_and_update_this_robot_conf(r, cur_rob_pos);
   MY_FREE(cur_rob_pos, double,r->nb_dof); 
  ////*****END making the previously occupied cells as free cells  

  ////*****Now creating exact obstacle at new place of robot

  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
  r_o_ctr=0; 
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   double BBx;
   for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
    {
    visq[6] = BBx;
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     visq[7]  = BBy;
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
      {
      visq[8]  = BBz;
      z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

      if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
       {
        p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
       int kcd_with_report=0;
       int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
       if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
        {
         //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
       bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
       
       if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0)//the cell does not already belong to this robot index
         { 
       bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;
       bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
         }
         bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;
       //else
        //{
        //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
       // }
          
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                 
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
                 {
                   
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
                  { 
                 bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
                 bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
                  }
                 }
                 else
                 {
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
                
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
                  { 
                 bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
                 bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
                  }
                 }
                bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
              }
             } 
            }
           }
          } 
         } 
        }
       ////else
       ////{
       ////printf("Inside BB but no collison\n");
       ////exit(0);
       ////}
       }
      }   
     }
    }
    }
   robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
   ////p3d_set_and_update_this_robot_conf(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config,r->ROBOT_POS);
  ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]=r->ROBOT_POS[6];
  ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]=r->ROBOT_POS[7];
  ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]=r->ROBOT_POS[8];
  p3d_copy_config_into(r,r->ROBOT_POS, &(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config));
  ////////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
   }
   
  }
  

  
}

int create_exact_obstacles_for_HRP2_GIK_manip_fast(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  if(bt_type==HRP2_GIK_MANIP)
 {
   configPt visq;
  hri_bitmap * bitmap;
  double increment=3.0/4.0*bitmapset->pace;
  visq = p3d_get_robot_config(bitmapset->visball);
  bitmap = bitmapset->bitmap[bt_type];
  envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt->no;
  int nr = envPt->nr;

   int x,y,z;
  for(x=0; x<bitmap->nx; x++)
  {
     for(y=0; y<bitmap->ny; y++)
   {
       for(z=0; z<bitmap->nz; z++)
    {
  bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  
  int nr_ctr=0;
    for(;nr_ctr<nr;nr_ctr++)
     {
     bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
     bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
     bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
     bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.near_horizontal_surface=0;
     }
     bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
     bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;  
    }
   }
  }   
  
  
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {

  r = envPt->robot[r_ctr];

  robots_status_for_Mightability_Maps[r_ctr].rob_prev_config=MY_ALLOC(double,r->nb_dof); 
  p3d_get_robot_config_into(r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
  ////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
  robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
 
  printf(" Robot name = %s \n",r->name);
  int surf_z;
  int is_tabel=0;
   if (strcasestr(r->name,"table"))
   {
        //int surf_z=z;
   printf(" Cell belongs to table %s \n",r->name);

   surf_z=((r->BB.zmax - bitmapset->realz)/bitmapset->pace)+1;//One cell above the surface
   is_tabel=1;
   printf(" surf_z = %d \n",surf_z);
   }
  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
  int r_o_ctr=0; 
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   double BBx;
   for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
    {
    visq[6]  = BBx;
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     visq[7]  = BBy;
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
      {
      visq[8]  = BBz;
      z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

      if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
       {
        p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
       int kcd_with_report=0;
       int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
       if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
        {
         //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
       bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
      
       if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0)//the cell does not already belong to this robot index
        { 
       bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index 
       bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
        }

          bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index 

   
      
        if(is_tabel==1&&(r->BB.zmax-BBz)<=bitmapset->pace)
        {
        bitmapset->bitmap[bt_type]->data[x][y][surf_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;//Belongs to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
         /*
        if(surf_z+1<bitmap->nz)
         {
        bitmapset->bitmap[bt_type]->data[x][y][surf_z+1].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x][y][surf_z+1].val = -2;  //To avoid the plan path close to the table surface
         }
        if(x-1>0&&x-1<bitmap->nx&&y+1>0&&y+1<bitmap->ny)
         {
        bitmapset->bitmap[bt_type]->data[x-1][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x-1][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
        if(y+1>0&&y+1<bitmap->ny)
         {
        bitmapset->bitmap[bt_type]->data[x][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
        if(x+1>0&&x+1<bitmap->nx&&y+1>0&&y+1<bitmap->ny)
         {
        bitmapset->bitmap[bt_type]->data[x+1][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x+1][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
        if(x-1>0&&x-1<bitmap->nx)
         {
        bitmapset->bitmap[bt_type]->data[x-1][y][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x-1][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
        if(x+1>0&&x+1<bitmap->nx)
         {
         bitmapset->bitmap[bt_type]->data[x+1][y][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
         bitmapset->bitmap[bt_type]->data[x+1][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
        if(x-1>0&&x-1<bitmap->nx&&y-1>0&&y-1<bitmap->ny)
         {
        bitmapset->bitmap[bt_type]->data[x-1][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x-1][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
         } 
        if(y-1>0&&y-1<bitmap->ny) 
         {
        bitmapset->bitmap[bt_type]->data[x][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
        if(x+1>0&&x+1<bitmap->nx&&y-1>0&&y-1<bitmap->ny)
         {
        bitmapset->bitmap[bt_type]->data[x+1][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
        bitmapset->bitmap[bt_type]->data[x+1][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
         }
         */
        }
        
       //else
        //{
        //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
       // }
          
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
                 {
                  
                  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
                  { 
                  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index  
                  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
                  }
                    bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index   
                 }
                 else
                 {
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
                 
                  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
                  { 
                  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index  
                  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
                  }
                bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index   
                 }
              }
             } 
            }
           }
          } 
         } 
        }
       ////else
       ////{
       ////printf("Inside BB but no collison\n");
       ////exit(0);
       ////}
       }
      }   
     }
    }
   }
  }
  

  int o_ctr=0;
  for(o_ctr=0;o_ctr<no;o_ctr++)
  {
  
   o = envPt->o[r_ctr];
  //// printf(" o->name = %s, \n",o->name);
   double BBx;
   for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
    {
    visq[6]  = BBx;
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     visq[7]  = BBy;
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
      {
      visq[8]  = BBz;
      z=(BBz- bitmapset->realz)/bitmapset->pace;  
 
       
      if(z>0&&z<bitmap->nz)
       {
       p3d_set_and_update_this_robot_conf(bitmapset->visball, visq); 
       int kcd_with_report=0;
       int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
       if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
        {
           //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
       bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;  
          
       int i=0; 
        for(i=x-expansion;i<=x+expansion;i++)
         {
            //printf(" for i = %d\n",i);
         if(i>=0&&i<bitmap->nx)
          {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
           {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
            {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
             {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
              {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  
              }
             }  
            }
           }
          } 
         } 
        }
       }
      }   
     }
    
   }
  }
  p3d_destroy_config(bitmapset->visball, visq);
 } 
}

int create_exact_obstacles_for_HRP2_GIK_manip(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  configPt visq;
  hri_bitmap * bitmap;
  int x,y,z;
  if(bt_type==-1) //Need to create for all bitmap types
  {
  

  visq = p3d_get_robot_config(bitmapset->visball);

  //p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->human[bitmapset->actual_human]->HumanPt);
  

  for(x=0; x<bitmap->nx; x++)
   {
     for(y=0; y<bitmap->ny; y++)
    {
       for(z=0; z<bitmap->nz; z++)
     {
      int btm_ctr=0;
      
      for(btm_ctr=0;btm_ctr<bitmapset->n;btm_ctr++)
      {
       ////printf(" x,y,z = (%d,%d,%d), btm_ctr=%d\n",x,y,z,btm_ctr);
       bitmap = bitmapset->bitmap[btm_ctr]; 
       if(bitmap->type==HRP2_GIK_MANIP)
       p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->robot);
       
	 visq[6]  = x*bitmapset->pace+bitmapset->realx;
	 visq[7]  = y*bitmapset->pace+bitmapset->realy;
	 visq[8]  = z*bitmapset->pace+bitmapset->realz;
	 p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
	 //if(p3d_col_test_robot_statics(bitmapset->visball,FALSE))
         if(p3d_col_test_robot(bitmapset->visball,FALSE))
         {
           //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
           bitmapset->bitmap[btm_ctr]->data[x][y][z].val = -2;  
          
           int i=0; 
           for(i=x-expansion;i<=x+expansion;i++)
           {
            //printf(" for i = %d\n",i);
            if(i>=0&&i<bitmap->nx)
            {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
             {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
              {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
               {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
                {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 bitmapset->bitmap[btm_ctr]->data[i][j][k].val = -2;  
                }
               } 
              }
             }
            } 
           } 
          }
	 else
          {
	   bitmapset->bitmap[btm_ctr]->data[x][y][z].val = 0;
          }
         if(bitmap->type==HRP2_GIK_MANIP)
         p3d_col_activate_rob_rob(bitmapset->visball,bitmapset->robot); 
        }
       }
     }
  }
  p3d_destroy_config(bitmapset->visball, visq);


//p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->human[ACBTSET->actual_human]->HumanPt);

  return TRUE;
    
   
  }
  else //need to create for the specific bitmap type provided
  {
  bitmap = bitmapset->bitmap[bt_type];

  visq = p3d_get_robot_config(bitmapset->visball);

  //p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->human[bitmapset->actual_human]->HumanPt);
 
  ////AKP NOTE : Below is the old version in which the collision test has been disactivated with HRP2 for the purpose of letting the A* planner to find the collision free path from the hand to the goal. But now instead of this now using a function to make the cell values 0 around the hand to be used for manipulation
  ////if(bt_type==HRP2_GIK_MANIP)
  ////p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->robot);

  for(x=0; x<bitmap->nx; x++){
     for(y=0; y<bitmap->ny; y++){
       for(z=0; z<bitmap->nz; z++){
	 visq[6]  = x*bitmapset->pace+bitmapset->realx;
	 visq[7]  = y*bitmapset->pace+bitmapset->realy;
	 visq[8]  = z*bitmapset->pace+bitmapset->realz;
	 p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
	 //if(p3d_col_test_robot_statics(bitmapset->visball,FALSE))
         if(p3d_col_test_robot(bitmapset->visball,FALSE))
         {
           //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
           bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;  
          
           int i=0; 
           for(i=x-expansion;i<=x+expansion;i++)
           {
            //printf(" for i = %d\n",i);
            if(i>=0&&i<bitmap->nx)
            {
             int j=0;
             for(j=y-expansion;j<=y+expansion;j++)
             {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<bitmap->ny)
              {
               int k=0;
                for(k=z-expansion;k<=z+expansion;k++)
               {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<bitmap->nz)
                {
                // printf(" Populating %d, %d, %d \n",i,j,k);
                 bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  
                }
               } 
              }
             }
            } 
           } 
          }
	 else
          {
	   bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;
          } 
       }
     }
  }
  p3d_destroy_config(bitmapset->visball, visq);

////if(bt_type==HRP2_GIK_MANIP)
 //// p3d_col_activate_rob_rob(bitmapset->visball,bitmapset->robot);
//p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->human[ACBTSET->actual_human]->HumanPt);

  return TRUE;
 }
}

int create_3d_grid_for_HRP2_GIK()
{
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
 double BB_half_length=1.5; // along x axis
 double BB_half_width=1.5; // along y axis
 double BB_half_height=2.0; // along z axis
/*
 double BB_half_length=0.5; // along x axis
 double BB_half_width=0.5; // along y axis
 double BB_half_height=2; // along z axis
*/
 //Bounding box coordinates are in global frame
 grid_around_HRP2.grid_BB.min_x=rob_cur_pos[6]-BB_half_length+0.5;
 grid_around_HRP2.grid_BB.max_x=rob_cur_pos[6]+BB_half_length+0.5;
 grid_around_HRP2.grid_BB.min_y=rob_cur_pos[7]-BB_half_width;
 grid_around_HRP2.grid_BB.max_y=rob_cur_pos[7]+BB_half_width;
 grid_around_HRP2.grid_BB.min_z=rob_cur_pos[8]-0.05;//-BB_length;//No need to go underground :)
 grid_around_HRP2.grid_BB.max_z=rob_cur_pos[8]+BB_half_height;
   
  double BB_length=grid_around_HRP2.grid_BB.max_x-grid_around_HRP2.grid_BB.min_x;
  double BB_width=grid_around_HRP2.grid_BB.max_y-grid_around_HRP2.grid_BB.min_y;
  double BB_height=grid_around_HRP2.grid_BB.max_z-grid_around_HRP2.grid_BB.min_z;

 if(grid_around_HRP2.GRID_SET != NULL)
 hri_bt_destroy_bitmapset(grid_around_HRP2.GRID_SET);

 double sampling_rate=0.05;

 grid_around_HRP2.GRID_SET=create_3D_grid(BB_length,BB_width,BB_height,sampling_rate);
 
 hri_bt_change_bitmap_position(grid_around_HRP2.GRID_SET,grid_around_HRP2.grid_BB.min_x,grid_around_HRP2.grid_BB.min_y,grid_around_HRP2.grid_BB.min_z);

 int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also

 //***** AKP : To populate bitmap based on the exact obstacles' position, not based on their bounding Boxes..
/* 
 ChronoPrint("<<<<<<<<Calling create_exact_obstacles_for_HRP2_GIK_manip for HRP2_GIK_MANIP bitmap");
 create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); 
 ChronoPrint("<<<<<<<<Calling create_exact_obstacles_for_HRP2_GIK_manip for BT_AFFORDANCE_VISIBILITY bitmap");
 create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
 ChronoPrint("<<<<<<<<Returned from create_exact_obstacles_for_HRP2_GIK_manip ");
 */
 //***** AKP: Uncomment below to populate the bitmap based on the bounding box of the objects' and robots'
 ////create_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET); 

 ChronoPrint("<<<<<<<<Calling create_exact_obstacles_for_HRP2_GIK_manip");
//// create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); //-1 is for populating every bitmap type 
 create_exact_obstacles_for_HRP2_GIK_manip_fast(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); //-1 is for populating every bitmap type 
 ChronoPrint("<<<<<<<<Returned from create_exact_obstacles_for_HRP2_GIK_manip");

 MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
}

int show_3d_grid_Bounding_box_for_HRP2_GIK()
{
//Line id A onwards in alphabetical order. See notebook for fig
g3d_drawOneLine(
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z,
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z,
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z,
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z,
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

g3d_drawOneLine(
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
        grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);


}

int SHOW_HRP2_ENTIRE_GIK_SOL=1;
double qs_tmp[3];
double qf_tmp[3];
int show_HRP2_gik_sol()
{
 int ctr=100;
 while(SHOW_HRP2_ENTIRE_GIK_SOL==1)
 {
 int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[0]);

 }

  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[0]);

}

point_co_ordi spline_path_btwn_prev_n_curr_exec[3000];
int no_spline_points_btwn_prev_n_curr_exec=0;

point_co_ordi resultant_spline[5000];
int no_spline_points=0;

int show_spline_path_for_HRP2_hand()
{
 int i=0;
 for(i=0;i<no_spline_points_btwn_prev_n_curr_exec-1;i++)
 {
 ////////g3d_drawOneLine(resultant_spline[i].x,resultant_spline[i].y, resultant_spline[i].z, resultant_spline[i+1].x,resultant_spline[i+1].y, resultant_spline[i+1].z, Red, NULL);
 g3d_drawOneLine(spline_path_btwn_prev_n_curr_exec[i].x,spline_path_btwn_prev_n_curr_exec[i].y, spline_path_btwn_prev_n_curr_exec[i].z, spline_path_btwn_prev_n_curr_exec[i+1].x,spline_path_btwn_prev_n_curr_exec[i+1].y, spline_path_btwn_prev_n_curr_exec[i+1].z, Red, NULL);
  
 }

 
 for(i=0;i<no_HRP2_hand_pos-1;i++)
 {
  g3d_drawOneLine(HRP2_hand_pos_sequence[i][0],HRP2_hand_pos_sequence[i][1], HRP2_hand_pos_sequence[i][2], HRP2_hand_pos_sequence[i+1][0],HRP2_hand_pos_sequence[i+1][1], HRP2_hand_pos_sequence[i+1][2], Blue, NULL);
  
 }
}

double parallel_const_time=0.0;
double parallel_const_task_duration=1.5;//in s
int parallel_constraint_interpolation_created=0;
double parallel_const_sampling_period=10e-3;

int find_HRP2_GIK_sol_for_hand_orientation(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach, int state, int use_body_part)
{
   
   printf(" Inside find_HRP2_GIK_sol_for_hand_orientation() \n");
   configPt cur_rob_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&cur_rob_pos);

  if(state==2)
  {
   initialize_constraints_stack_for_standing_HRP2();// AKP WARNING TODO : Need to make the nsfc and com constraint variables global as done for pc otherwise it will give segmentation fault
  } 

  create_HRP2_hand_pos_constraint(hand_by_reach); 
  create_HRP2_hand_parallel_constraint(hand_by_reach);
  ////create_HRP2_look_at_constraint();

  HRP2_get_joint_mask(hand_by_reach, state, use_body_part); // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body 
 int i=0;
  
 

  state_constraint_tasks.clear();  
   
     
      double start_time=0.0;
 
      int priority=1;// expecting at priority 0 the hand position constraint will be there in the stack
      get_parallel_constraint_interpolation(req_hand_orientation_in_global_frame, hand_by_reach, parallel_const_task_duration, start_time, parallel_const_sampling_period, priority);
      parallel_constraint_interpolation_created=1;
      
    double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(hand_by_reach);//1 for left, 2 for right hand;
 p3d_vector3 target_in_global_frame;

 target_in_global_frame[0]=hand_pos[0];
 target_in_global_frame[1]=hand_pos[1];
 target_in_global_frame[2]=hand_pos[2];

    // if( parallel_constraint_interpolation_created==1)
    // {
      while(parallel_const_time<(parallel_const_task_duration-parallel_const_sampling_period))
      {
      parallel_const_time+=parallel_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_parallel_constraint_interpolated_element(parallel_const_time);

       push_HRP2_hand_pos_constraint(target_in_global_frame, hand_by_reach); // hand_by_reach =1 for left hand, 2 for
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      get_HRP2_GIK_sol();
          
   int check_collision=1;
  if(check_collision==1)
    {
   hrp2_to_M3D_ConfigPt(&cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1],cur_rob_pos);

  p3d_set_and_update_this_robot_conf(ACBTSET->robot,cur_rob_pos);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, res=%d \n", res);
       ////point_of_curr_collision.x=target_in_global_frame[0];
      //// point_of_curr_collision.y=target_in_global_frame[1];
      //// point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_hand_parallel_constraint();
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       cur_gik_sol_configs.no_configs--;
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, res=%d \n", res);
       ////point_of_curr_collision.x=target_in_global_frame[0];
       ////point_of_curr_collision.y=target_in_global_frame[1];
      //// point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_hand_parallel_constraint();
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       cur_gik_sol_configs.no_configs--;
       return 0;
       }
     }
 
    }//End while()
  MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
  delete_HRP2_hand_pos_constraint(); 
  ////if(HRP2_CURRENT_TASK==2&&impose_look_at_constraint==1)// for put object, 
  ////delete_HRP2_look_at_constraint();
  delete_HRP2_hand_parallel_constraint();

  return 1;
} 


int find_HRP2_GIK_sol_for_spline_path(int hand_by_reach, int state, int use_body_part, int maintain_hand_orientation)
{
   int impose_look_at_constraint=0;
   int impose_look_at_human_constraint=0;
   int impose_look_at_hand_path_constraint=0;

   printf(" Inside find_HRP2_GIK_sol_for_spline_path() with no_spline_points=%d\n",no_spline_points);
  configPt cur_rob_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&cur_rob_pos);
   
configPt rob_actual_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&rob_actual_pos);

  if(state==2)
  {
   initialize_constraints_stack_for_standing_HRP2();// AKP WARNING TODO : Need to make the nsfc and com constraint variables global as done for pc otherwise it will give segmentation fault
  } 

  create_HRP2_hand_pos_constraint(hand_by_reach); 
  create_HRP2_hand_parallel_constraint(hand_by_reach);
  create_HRP2_look_at_constraint();

  HRP2_get_joint_mask(hand_by_reach, state, use_body_part); // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body 
 int i=0;
 
 double look_at_const_timer=0.0;
  
   double look_at_const_sampling_period=10e-3;
 
 int look_at_constraint_interpolation_created=0;
 int skip_look_const_ctr=0;
 int look_at_human_executing=0;
 int look_at_hand_executing=0;
  
 double *hand_curr_x_orientation;
 p3d_vector3 req_hand_orientation_in_global_frame;

   if(maintain_hand_orientation==1)
      {
       hand_curr_x_orientation=get_HRP2_hand_x_axis_orientation_in_global_frame(hand_by_reach);//1 for left, 2 for right hand
       req_hand_orientation_in_global_frame[0]=hand_curr_x_orientation[0];
       req_hand_orientation_in_global_frame[1]=hand_curr_x_orientation[1];
       req_hand_orientation_in_global_frame[2]=hand_curr_x_orientation[2];
      } 

 int tmp_config_ctr=0;
 int prev_collision_free_config=0;
 double wait_ctr=0.0;
 double wait_cur_state=4;//in s

 vectorN collision_free_config = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model

 

 for(i=0;i<no_spline_points-1;i++)
 {
  p3d_vector3 target_in_global_frame;
  target_in_global_frame[0]=resultant_spline[i].x;
  target_in_global_frame[1]=resultant_spline[i].y;
  target_in_global_frame[2]=resultant_spline[i].z;

  state_constraint_tasks.clear();  

////  printf(" Pushing hand_pos_constraint %d \n",i);
    //if(HRP2_CURRENT_TASK!=2) //For put object
   //////// int hand_pos_push_res=push_HRP2_hand_pos_constraint(target_in_global_frame, hand_by_reach); // hand_by_reach =1 for left hand, 2 for right hand,
     /*if(HRP2_CURRENT_TASK==2&&impose_look_at_constraint==1)// for put object, 
     {
     ////skip_look_const_ctr++;
     ////if(skip_look_const_ctr>500)
      ////{
     printf(" Pushing look_at_constraint for path point %d \n",i);
     int look_at_push_res= push_HRP2_look_at_constraint(target_in_global_frame);
      ////}
     } 
     */

     

     /*
     ////if(HRP2_CURRENT_TASK==10) 
    
     ////if(i>=no_spline_points/2)
     {
      
      if(parallel_constraint_interpolation_created==0)
      {
      p3d_vector3 req_hand_orientation_in_global_frame;
  //The desired orientation of the x axis of wrist frame in the global frame, we want it to be parallel to the z axis of the global frame, i.e. in thumb up orientation
      ////req_hand_orientation_in_global_frame[0]=-0.205737;
      ////req_hand_orientation_in_global_frame[1]=0.321115;
      ////req_hand_orientation_in_global_frame[2]=0.924423 ;
      
      req_hand_orientation_in_global_frame[0]=0;
      req_hand_orientation_in_global_frame[1]=0;
      req_hand_orientation_in_global_frame[2]=1;
   
     
      double start_time=0.0;
 
      int priority=0;
      get_parallel_constraint_interpolation(req_hand_orientation_in_global_frame, hand_by_reach, parallel_const_task_duration, start_time, parallel_const_sampling_period, priority);
      parallel_constraint_interpolation_created=1;
      }
     }  
    
     if( parallel_constraint_interpolation_created==1)
     {
      if(parallel_const_time<(parallel_const_task_duration-parallel_const_sampling_period))
      {
      parallel_const_time+=parallel_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_parallel_constraint_interpolated_element(parallel_const_time);
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      }
      else
      {
       ////printf("Maintaining the last parallel constraint by pushing it with parallel_const_time =%lf\n",parallel_const_time); 
       push_parallel_constraint_interpolated_element(parallel_const_time);

      }
     }
      */
      if(maintain_hand_orientation==1)
      {
      ////printf(" Maintaining the current hand orientation by pushing the parellel constraint (%lf, %lf, %lf)\n",req_hand_orientation_in_global_frame[0],req_hand_orientation_in_global_frame[1],req_hand_orientation_in_global_frame[2]);
      push_HRP2_hand_parallel_constraint(req_hand_orientation_in_global_frame, hand_by_reach);
      }

     int hand_pos_push_res=push_HRP2_hand_pos_constraint(target_in_global_frame, hand_by_reach); // hand_by_reach =1 for left hand, 2 for right hand,
     
     if(HRP2_CURRENT_TASK==2&&wait_ctr>wait_cur_state)
     {
      
      ////printf("impose_look_at_human_constraint=%d, impose_look_at_hand_path_constraint=%d, look_at_human_executing=%d, look_at_hand_executing=%d\n",impose_look_at_human_constraint, impose_look_at_hand_path_constraint, look_at_human_executing, look_at_hand_executing);
      if(impose_look_at_human_constraint==0&&look_at_hand_executing==0&&impose_look_at_hand_path_constraint==0)
      {
      impose_look_at_human_constraint=1;
      look_at_human_executing=1;
      printf(" Look at Human ...\n");
      ////wait_cur_state=0.5;
      }
      else
      {
      if(impose_look_at_human_constraint==1&&look_at_human_executing==0&&impose_look_at_hand_path_constraint==0)
       {
      impose_look_at_human_constraint=0;
      impose_look_at_hand_path_constraint=1;
      look_at_hand_executing=1;
      look_at_constraint_interpolation_created=0;
      printf(" Look at Hand ...\n");
      ////wait_cur_state=2.0;
       }
      }
      wait_ctr=0; 
     }

     if(impose_look_at_human_constraint==1)
     {
      double look_at_const_task_duration=1.5;//in s
      if(look_at_constraint_interpolation_created==0)
      {
      p3d_vector3 look_at_point_in_global_frame;
      double start_time=0;
      double hum_head_pos[3];
      int priority=2;//Assuming 0 as hand pos constraint, 1 for hand parallel constraint
      hum_head_pos[0] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[0][3];//hum_cur_pos[6];
      hum_head_pos[1] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[1][3];//hum_cur_pos[7];
      hum_head_pos[2] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[2][3];
      
      look_at_point_in_global_frame[0]=hum_head_pos[0];
      look_at_point_in_global_frame[1]=hum_head_pos[1];
      look_at_point_in_global_frame[2]=hum_head_pos[2];

      get_HRP2_look_at_constraint_interpolation(look_at_point_in_global_frame,  look_at_const_task_duration, start_time, look_at_const_sampling_period, priority);
      look_at_constraint_interpolation_created=1;
      look_at_const_timer=0;
      }

      if(look_at_const_timer<(look_at_const_task_duration-look_at_const_sampling_period))
      {
      look_at_const_timer+=look_at_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_HRP2_look_at_constraint_interpolated_element(look_at_const_timer);
      look_at_human_executing=1;
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      }
      else
      {
       //impose_look_at_human_constraint=0;
       look_at_human_executing=0;
      }
      
     }

     if(impose_look_at_hand_path_constraint==1)
     {
      double look_at_const_task_duration=1.5;//in s
      if(look_at_constraint_interpolation_created==0)
      {
      p3d_vector3 look_at_point_in_global_frame;
      double start_time=0;
      
      int priority=2;//Assuming 0 as hand pos constraint, 1 for hand parallel constraint
     
      look_at_point_in_global_frame[0]=target_in_global_frame[0];
      look_at_point_in_global_frame[1]=target_in_global_frame[1];
      look_at_point_in_global_frame[2]=target_in_global_frame[2];

      get_HRP2_look_at_constraint_interpolation(look_at_point_in_global_frame,  look_at_const_task_duration, start_time, look_at_const_sampling_period, priority);
      look_at_constraint_interpolation_created=1;
      look_at_const_timer=0;
      }

      if(look_at_const_timer<(look_at_const_task_duration-look_at_const_sampling_period))
      {
      look_at_const_timer+=look_at_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_HRP2_look_at_constraint_interpolated_element(look_at_const_timer);
      look_at_hand_executing=1;
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      }
      else
      {
       impose_look_at_hand_path_constraint=0;
       look_at_constraint_interpolation_created=0;
       look_at_hand_executing=0;
      }
      
     }
     collision_free_config = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
     wait_ctr+=look_at_const_sampling_period;
    
   ////if((no_spline_points-i)<10)
  ////{
 //// printf(" Pushing look_at_constraint for path point %d \n",i);
////  int look_at_push_res= push_HRP2_look_at_constraint(target_in_global_frame);
////  }
  ////printf(" Calling get_HRP2_GIK_sol()\n");
  /*
  double parallel_const_task_duration=2.0;
   if(i>=no_spline_points/2&&parallel_const_time<parallel_const_task_duration)
   {
    if(parallel_constraint_interpolation_created==0)
    {
     p3d_vector3 req_hand_orientation_in_global_frame;
  //The desired orientation of the x axis of wrist frame in the global frame, we want it to be parallel to the z axis of the global frame, i.e. in thumb up orientation
      req_hand_orientation_in_global_frame[0]=0;
      req_hand_orientation_in_global_frame[1]=0;
      req_hand_orientation_in_global_frame[2]=1;
   
     
      double start_time=0.0;
 
      int priority=1;// expecting at priority 0 the hand position constraint will be there in the stack
      get_parallel_constraint_interpolation(req_hand_orientation_in_global_frame, hand_by_reach, parallel_const_task_duration, start_time, parallel_const_sampling_period, priority);
      parallel_constraint_interpolation_created=1;
    } 

     parallel_const_time+=parallel_const_sampling_period;

    printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
    push_parallel_constraint_interpolated_element(parallel_const_time);
    printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);

    ////parallel_const_time+=parallel_const_sampling_period;

   } 
  */
   get_HRP2_GIK_sol();
   
   ////////double *hand_pos;
   
   ////////hand_pos=get_HRP2_hand_center_in_global_frame(hand_by_reach);//1 for left, 2 for right hand;
   ////////printf(" hand_pos=(%lf, %lf, %lf)\n", hand_pos[0], hand_pos[1], hand_pos[2]);
   ////////MY_FREE(hand_pos, double, 3);
   
   int check_collision=1;
  if(check_collision==1)
    {
   if(tmp_config_ctr%100==0||tmp_config_ctr>=no_spline_points-5)
     {
   hrp2_to_M3D_ConfigPt(&cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1],cur_rob_pos);

 
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,cur_rob_pos);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, for the configuration no. %d, res=%d \n", tmp_config_ctr, res);
       printf("with look_at_hand_executing = %d, May be this collision is due to look at hand constraint, so trying to remove this constraint and recalculating. \n",look_at_hand_executing);
        if(look_at_hand_executing==1) //May be this collision is due to look at hand constraint, so try to remove this constraint and recalculate
        {
         impose_look_at_hand_path_constraint=0;
       look_at_constraint_interpolation_created=0;
       look_at_hand_executing=0;
        
        attStandingRobot->staticState ( collision_free_config );//Restoring the previous collision free configuration configuration
         i=prev_collision_free_config-1;
        }
        else
        {
       point_of_curr_collision.x=target_in_global_frame[0];
       point_of_curr_collision.y=target_in_global_frame[1];
       point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_look_at_constraint();
       delete_HRP2_hand_parallel_constraint();
       delete_HRP2_hand_pos_constraint(); 

       //Restore the actual position of the robot
       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       ////////cur_gik_sol_configs.no_configs--;
       //TODO : Delete the configurations till current config to the prev_collision_free_config for freeing the memory
       cur_gik_sol_configs.no_configs=prev_collision_free_config;
       return 0;
        }
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, for the configuration no. %d, res=%d \n", tmp_config_ctr, res);
       point_of_curr_collision.x=target_in_global_frame[0];
       point_of_curr_collision.y=target_in_global_frame[1];
       point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_hand_pos_constraint(); 
       delete_HRP2_look_at_constraint();
       delete_HRP2_hand_parallel_constraint();

       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       ////////cur_gik_sol_configs.no_configs--;
        //TODO : Delete the configurations till current config to the prev_collision_free_config for freeing the memory
       cur_gik_sol_configs.no_configs=prev_collision_free_config;
       return 0;
       }
       prev_collision_free_config=cur_gik_sol_configs.no_configs;
       
      }
/////AKP NOTE : this is tmp storage of hand pos as every configuration, comment this part for making the syatem fast
     double *hand_pos;
     hand_pos=get_HRP2_hand_center_in_global_frame(hand_by_reach);//1 for left, 2 for right hand;
     HRP2_hand_pos_sequence[no_HRP2_hand_pos][0]=hand_pos[0];
     HRP2_hand_pos_sequence[no_HRP2_hand_pos][1]=hand_pos[1]; 
     HRP2_hand_pos_sequence[no_HRP2_hand_pos][2]=hand_pos[2];
     no_HRP2_hand_pos++;
     MY_FREE(hand_pos,double, 3);
//////////////////////      


     }
 tmp_config_ctr++;
 }

  //Restoring the actual position of the robot
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
  MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
  MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
  delete_HRP2_hand_pos_constraint(); 
  ////if(HRP2_CURRENT_TASK==2&&impose_look_at_constraint==1)// for put object, 
  delete_HRP2_look_at_constraint();
  delete_HRP2_hand_parallel_constraint();

  return 1;
} 

point_co_ordi start_hand_pos;

int find_spline_path_for_via_points(point_co_ordi via_points[500], int no_via_points)
{
  

 int continuity_constraint_type=2;//1 means first derivative, i.e. velocity should be continuous everywhere, 2 means second derivative i.e. acceleration should also be continuous everywhere 
 point_co_ordi init_vel, final_vel;
  init_vel.x=0;
  init_vel.y=0;
  init_vel.z=0;
  final_vel.x=0;
  final_vel.y=0;
  final_vel.z=0;

  double total_time=4.0;//in s Total time for start to goal point 
 
   
  ////double sampling_period=attSamplingPeriod; //Set as sampling period used on HRP2, It should be 5e-3
  
  double sampling_period=10e-3;

  int no_spline_pts=get_cubic_spline_by_Hermite_polynomial(via_points, no_via_points, init_vel, final_vel, continuity_constraint_type, sampling_period, total_time, resultant_spline);

  no_spline_points+=no_spline_pts;//Adding in the global counter
  return no_spline_pts; //But returning the no. of pts in call of this function
  

 //return 0;
}

int find_spline_path_for_HRP2_hand(hri_bitmapset * btset, hri_bitmap* bitmap, int hand_by_reach)
{
  point_co_ordi curr_path[500];
  int no_pts=0;
  hri_bitmap_cell* current;
  //int i,j,color;
  //double task_duration=1.0;//1.5;
  if(bitmap == NULL)
    return 0;
  printf(" Inside move_HRP2_hand_with_GIK_on_path with bitmap->searched = %d \n",bitmap->searched);
  if(bitmap->searched) {

    // vertical lines on start and goal
    //g3d_drawOneLine(
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    //g3d_drawOneLine(
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    while(current != bitmap->search_start && current != NULL)
    {
     
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
     current = current->parent;
    }

     //Storing the start cell
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;

     //Storing the actual starting position of the hand, which might be different from the position of the starting cell due to discreatization 
     curr_path[no_pts].x=start_hand_pos.x;
     curr_path[no_pts].y=start_hand_pos.y;
     curr_path[no_pts].z=start_hand_pos.z;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
   


    printf(" No. of cells in the path = %d \n",no_pts);

    
    
    int i=0;
    int j=no_pts-1;
    int i_ctr=0;
    for(i=0;i<=j;i++)
    {
     double x=curr_path[i].x;
     double y=curr_path[i].y;
     double z=curr_path[i].z;
     curr_path[i].x=curr_path[j].x;
     curr_path[i].y=curr_path[j].y;
     curr_path[i].z=curr_path[j].z;
     curr_path[j].x=x;
     curr_path[j].y=y;
     curr_path[j].z=z;
     j--;
    }
    

    /*int i=0;
    int j=no_pts-1;
    for(i=0;i<no_pts;i++)
    {
        
    }*/

 int continuity_constraint_type=2;//1 means first derivative, i.e. velocity should be continuous everywhere, 2 means second derivative i.e. acceleration should also be continuous everywhere 
 point_co_ordi init_vel, final_vel;
  init_vel.x=0;
  init_vel.y=0;
  init_vel.z=0;
  final_vel.x=0;
  final_vel.y=0;
  final_vel.z=0;

  double total_time=4.0;//in s Total time for start to goal point 
 
   
  ////double sampling_period=attSamplingPeriod; //Set as sampling period used on HRP2, It should be 5e-3
  
  double sampling_period=10e-3;
  int no_spline_pts=get_cubic_spline_by_Hermite_polynomial(curr_path, no_pts, init_vel, final_vel, continuity_constraint_type, sampling_period, total_time, resultant_spline);
  no_spline_points=no_spline_pts;
  return 1;
  }//END if(bitmap->searched)
 return 0;
}

int move_HRP2_hand_with_GIK_on_path(hri_bitmapset * btset, hri_bitmap* bitmap, int hand_by_reach)
{
SKIP_FIRST_CONFIG=0;
   vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
 

        int j=0;

        printf(" Inside move_HRP2_hand_with_GIK_on_path(), NO_DOF_HRP2=%d and their valuse are : \n",NO_DOF_HRP2);
        for (  j=6;j<NO_DOF_HRP2;j++ )
        {
        
        
        printf(" backupConfig(%d)=%lf\n",j,backupConfig(j));
        
        }
  //int hand_by_reach=1;//1 for left hand, 2 for right hand
  //double maxi_reach_dist=0.5;
  p3d_vector3 target_in_global_frame;
  p3d_vector3 curr_path[1000];
  int no_pts=0;
  hri_bitmap_cell* current;
  //int i,j,color;
  double task_duration=1.0;//1.5;
  if(bitmap == NULL)
    return 0;
  printf(" Inside move_HRP2_hand_with_GIK_on_path with bitmap->searched = %d \n",bitmap->searched);
  if(bitmap->searched) {

    // vertical lines on start and goal
    //g3d_drawOneLine(
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    //g3d_drawOneLine(
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    while(current != bitmap->search_start && current != NULL)
    {
     
     curr_path[no_pts][0]=current->x*btset->pace+btset->realx;
     curr_path[no_pts][1]=current->y*btset->pace+btset->realy;
     curr_path[no_pts][2]=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
     current = current->parent;
    }
    
    printf(" No. of cells in the path = %d \n",no_pts);
    int i=no_pts-1;
    int i_ctr=0;
    for(;i>=0;i--)
    {
     printf(" Calling HRP2_hand_reach() for %dth path point\n",i); 
     int HRP2_state=HRP2_CURRENT_STATE; //1 for sitting on the chair, 2 for standing
     
    //// if(fabs(curr_path[i][2]-curr_path[0][2])<0.1&&HRP2_CURRENT_TASK!=3)
    ////  THUMB_UP_CONSTRAINT=1;

     int thumb_up_constraint=THUMB_UP_CONSTRAINT;//1 Means the x axis of the hand will be made parallel to the z axis of global frame
    //if(i<=5)
    //thumb_up_constraint=1;// Put the parallel constraint at the few last segments of the path
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
   
    
    int reachable=HRP2_hand_reach(curr_path[i], hand_by_reach, task_duration, HRP2_state, thumb_up_constraint, use_body_part);
    
    
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
    printf(" reachable = %d \n",reachable);
    if(reachable==0)
     {
     printf(" ****** AKP Warning: The GIK solution to reach next cell could not be found, so returning\n");
     attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
     
     return 0;
     }
       //i_ctr++;

     int j=0;
     if(i_ctr>0)
     {
      j=1; //Because the 0th configuration will be the end configuration of previous motion
      SKIP_FIRST_CONFIG=1; //Will be used in HRP2_hand_reach for skipping first configuration which will be the end configuration of previous motion
     }

     for(;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
   printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, res=%d \n", res);
       point_of_curr_collision.x=curr_path[i][0];
       point_of_curr_collision.y=curr_path[i][1];
       point_of_curr_collision.z=curr_path[i][2]; 
       attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, res=%d \n", res);
       point_of_curr_collision.x=curr_path[i][0];
       point_of_curr_collision.y=curr_path[i][1];
       point_of_curr_collision.z=curr_path[i][2]; 
       attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
       return 0;
       }
 
   

    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
    printf(" Updated robot with total configuration j =%d \n",j);
    i_ctr++;
    if(i_ctr>=1)
     {
     task_duration=0.5;//task_duration=0.25;
     }
    //return 1; 
    }
    
printf(" Restoring the original configuration of HRP2 GIK model in BioMove3d\n");
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
        
    return 1;
   }

}

int show_weighted_candidate_points_to_put_obj()
{
  int i=0;
  for(i=0;i<candidate_points_to_put.no_points;i++)
  {
   ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
   g3d_drawDisc(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, 4, NULL);
   ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
  g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z, candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z+candidate_points_to_put.weight[i]/2.0, Green, NULL);
  }
}

int show_weighted_candidate_points_to_show_obj()
{
  int i=0;
  for(i=0;i<candidate_points_to_show.no_points;i++)
  {
   ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
   g3d_drawDisc(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z, .02, 4, NULL);
   ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
  g3d_drawOneLine(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y,candidate_points_to_show.point[i].z, candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z+candidate_points_to_show.weight[i]/2.0, Green, NULL);
  }
}


int show_weighted_candidate_points_to_hide_obj()
{
  int i=0;
  for(i=0;i<candidate_points_to_hide.no_points;i++)
  {
   ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
   g3d_drawDisc(candidate_points_to_hide.point[i].x, candidate_points_to_hide.point[i].y, candidate_points_to_hide.point[i].z, .02, 4, NULL);
   ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
  
   g3d_drawOneLine(candidate_points_to_hide.point[i].x, candidate_points_to_hide.point[i].y,candidate_points_to_hide.point[i].z, candidate_points_to_hide.point[i].x, candidate_points_to_hide.point[i].y, candidate_points_to_hide.point[i].z+candidate_points_to_hide.weight[i]/2.0, Green, NULL);
  }
}


int reverse_sort_weighted_candidate_points_to_put_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  printf("Inside sort_weighted_candidate_points_to_put_obj(), candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);

  printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_put.no_points;i++)
  {
  printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
  }  

  for (int i = 0; i < candidate_points_to_put.no_points; i++) 
  {
   int curmax = i;
   for (int j = i+1; j < candidate_points_to_put.no_points; j++) 
   {
    if(candidate_points_to_put.weight[j]>candidate_points_to_put.weight[curmax])
    {
     curmax=j;
    } 
   }
   if (curmax != i)
   {
    double tmp_wt=candidate_points_to_put.weight[i];
    double tmp_x=candidate_points_to_put.point[i].x;
    double tmp_y=candidate_points_to_put.point[i].y;
    double tmp_z=candidate_points_to_put.point[i].z;
    candidate_points_to_put.weight[i]=candidate_points_to_put.weight[curmax];
    candidate_points_to_put.point[i].x=candidate_points_to_put.point[curmax].x;
    candidate_points_to_put.point[i].y=candidate_points_to_put.point[curmax].y;
    candidate_points_to_put.point[i].z=candidate_points_to_put.point[curmax].z;
 
    candidate_points_to_put.weight[curmax]=tmp_wt;
    candidate_points_to_put.point[curmax].x=tmp_x;
    candidate_points_to_put.point[curmax].y=tmp_y;
    candidate_points_to_put.point[curmax].z=tmp_z;

   }
  }
   
  printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_put.no_points;i++)
  {
  printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
  }  
 
}

int reverse_sort_weighted_candidate_points_to_show_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  printf("Inside reverse_sort_weighted_candidate_points_to_show_obj(), candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);

  printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_show.no_points;i++)
  {
  printf(" candidate_points_to_show.weight[%d]=%lf\n",i, candidate_points_to_show.weight[i]); 
  }  

  for (int i = 0; i < candidate_points_to_show.no_points; i++) 
  {
   int curmax = i;
   for (int j = i+1; j < candidate_points_to_show.no_points; j++) 
   {
    if(candidate_points_to_show.weight[j]>candidate_points_to_show.weight[curmax])
    {
     curmax=j;
    } 
   }
   if (curmax != i)
   {
    double tmp_wt=candidate_points_to_show.weight[i];
    double tmp_x=candidate_points_to_show.point[i].x;
    double tmp_y=candidate_points_to_show.point[i].y;
    double tmp_z=candidate_points_to_show.point[i].z;
    candidate_points_to_show.weight[i]=candidate_points_to_show.weight[curmax];
    candidate_points_to_show.point[i].x=candidate_points_to_show.point[curmax].x;
    candidate_points_to_show.point[i].y=candidate_points_to_show.point[curmax].y;
    candidate_points_to_show.point[i].z=candidate_points_to_show.point[curmax].z;
 
    candidate_points_to_show.weight[curmax]=tmp_wt;
    candidate_points_to_show.point[curmax].x=tmp_x;
    candidate_points_to_show.point[curmax].y=tmp_y;
    candidate_points_to_show.point[curmax].z=tmp_z;

   }
  }
   
  printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_show.no_points;i++)
  {
  printf(" candidate_points_to_show.weight[%d]=%lf\n",i, candidate_points_to_show.weight[i]); 
  }  
 
}

int reverse_sort_weighted_candidate_points_to_hide_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  printf("Inside reverse_sort_weighted_candidate_points_to_hide_obj(), candidate_points_to_hide.no_points=%d\n",candidate_points_to_hide.no_points);

  printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_hide.no_points;i++)
  {
  printf(" candidate_points_to_hide.weight[%d]=%lf\n",i, candidate_points_to_hide.weight[i]); 
  }  

  for (int i = 0; i < candidate_points_to_hide.no_points; i++) 
  {
   int curmax = i;
   for (int j = i+1; j < candidate_points_to_hide.no_points; j++) 
   {
    if(candidate_points_to_hide.weight[j]>candidate_points_to_hide.weight[curmax])
    {
     curmax=j;
    } 
   }
   if (curmax != i)
   {
    double tmp_wt=candidate_points_to_hide.weight[i];
    double tmp_x=candidate_points_to_hide.point[i].x;
    double tmp_y=candidate_points_to_hide.point[i].y;
    double tmp_z=candidate_points_to_hide.point[i].z;
    candidate_points_to_hide.weight[i]=candidate_points_to_hide.weight[curmax];
    candidate_points_to_hide.point[i].x=candidate_points_to_hide.point[curmax].x;
    candidate_points_to_hide.point[i].y=candidate_points_to_hide.point[curmax].y;
    candidate_points_to_hide.point[i].z=candidate_points_to_hide.point[curmax].z;
 
    candidate_points_to_hide.weight[curmax]=tmp_wt;
    candidate_points_to_hide.point[curmax].x=tmp_x;
    candidate_points_to_hide.point[curmax].y=tmp_y;
    candidate_points_to_hide.point[curmax].z=tmp_z;

   }
  }
   
  printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_hide.no_points;i++)
  {
  printf(" candidate_points_to_hide.weight[%d]=%lf\n",i, candidate_points_to_hide.weight[i]); 
  }  
 
}


int assign_weights_on_candidte_points_to_put_obj()
{
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_matInvertXform(ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos, hum_pos_inverse);

   
  int i=0;
  double Amplitude=1;
  double yaw_mean=0;
  double pitch_mean=0;
  double sig_yaw=M_PI;

  for(i=0;i<candidate_points_to_put.no_points;i++)
  {
  point_in_global_frame[0] = candidate_points_to_put.point[i].x;
  point_in_global_frame[1] = candidate_points_to_put.point[i].y;
  point_in_global_frame[2] = candidate_points_to_put.point[i].z;
  point_in_global_frame[3] = 1;
  
  
  ////Assigning weight wrt human axis
  p3d_matvec4Mult(hum_pos_inverse, point_in_global_frame, point_in_human_frame);
  
  p3d_psp_cartesian2spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
  
  
  ////////candidate_points_to_put.weight[i]+=(1.0-(fabs(relative_yaw))/M_PI);

  ////////candidate_points_to_put.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_x*sig_x)+((pitch_mean-relative_yaw)*(pitch_mean-relative_yaw)/2.0*sig_y*sig_y))));

  candidate_points_to_put.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)));

  printf(" relative_yaw=%lf, weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_put.weight[i]);

  
  ////Assigning weight based on the closeness to the point which is at dist 0.3 m from the human 
  double human_pos[3];
  double sig_hum_dist=0.7;
  double mean_dist=0.3;
  human_pos[0]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3];
  human_pos[1]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3];
  human_pos[2]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3];
  ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
  double point_to_human_dist=sqrt((human_pos[0]-point_in_global_frame[0])*(human_pos[0]-point_in_global_frame[0])+(human_pos[1]-point_in_global_frame[1])*(human_pos[1]-point_in_global_frame[1])+(human_pos[2]-point_in_global_frame[2])*(human_pos[2]-point_in_global_frame[2]));
  printf(" point to human dist = %lf \n", point_to_human_dist);
  candidate_points_to_put.weight[i]+=Amplitude/2.0*exp(-(((mean_dist-point_to_human_dist)*(mean_dist-point_to_human_dist)/2.0*sig_hum_dist*sig_hum_dist)));
  

  ////Assigning weight based on the closeness to the bottle current position
  double bottle_pos[3];
  double sig_dist=2.0;
  bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
  double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
  candidate_points_to_put.weight[i]+=Amplitude/1.5*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));

  /*if(fabs(point_to_bottle_dist)<0.00001)
  candidate_points_to_put.weight[i]+=1;
  else
  candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
  */
  } 
  
 
  

  
}

int assign_weights_on_candidte_points_to_show_obj()
{
  
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_matInvertXform(ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos, hum_pos_inverse);

  configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */

 p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


////double yaw=M_PI/3.0;

////double orig_pan=hum_cur_pos[HUMANq_PAN]; 
  int i=0;
  double Amplitude=1.0;
  double yaw_mean=hum_cur_pos[HUMANq_PAN];
  double pitch_mean=hum_cur_pos[HUMANq_TILT];
  double sig_yaw=M_PI/2.0;
  double sig_pitch=M_PI/2.0;
  double max_weight=0;

  for(i=0;i<candidate_points_to_show.no_points;i++)
  {
  point_in_global_frame[0] = candidate_points_to_show.point[i].x;
  point_in_global_frame[1] = candidate_points_to_show.point[i].y;
  point_in_global_frame[2] = candidate_points_to_show.point[i].z;
  point_in_global_frame[3] = 1;
  
  
  ////Assigning weight wrt human axis
  p3d_matvec4Mult(hum_pos_inverse, point_in_global_frame, point_in_human_frame);
  
  p3d_psp_cartesian2spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
  
  
  ////////candidate_points_to_put.weight[i]+=(1.0-(fabs(relative_yaw))/M_PI);

  ////printf(" For point %d \n",i);
    candidate_points_to_show.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)+((pitch_mean-relative_pitch)*(pitch_mean-relative_pitch)/2.0*sig_pitch*sig_pitch)));

   if(max_weight<candidate_points_to_show.weight[i])
   max_weight=candidate_points_to_show.weight[i];
  ////////candidate_points_to_show.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)));
  
  printf(" relative_yaw=%lf, relative_pitch=%lf, weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,relative_pitch, i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);

  /*
  ////Assigning weight based on the closeness to the point which is at dist 0.3 m from the human 
  double human_pos[3];
  double sig_hum_dist=0.7;
  double mean_dist=0.3;
  human_pos[0]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3];
  human_pos[1]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3];
  human_pos[2]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3];
  ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
  double point_to_human_dist=sqrt((human_pos[0]-point_in_global_frame[0])*(human_pos[0]-point_in_global_frame[0])+(human_pos[1]-point_in_global_frame[1])*(human_pos[1]-point_in_global_frame[1])+(human_pos[2]-point_in_global_frame[2])*(human_pos[2]-point_in_global_frame[2]));
  printf(" point to human dist = %lf \n", point_to_human_dist);
  candidate_points_to_put.weight[i]+=Amplitude/2.0*exp(-(((mean_dist-point_to_human_dist)*(mean_dist-point_to_human_dist)/2.0*sig_hum_dist*sig_hum_dist)));
  */

  /*
  ////Assigning weight based on the closeness to the bottle current position
  double bottle_pos[3];
  double sig_dist=2.0;
  bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
  double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
  candidate_points_to_show.weight[i]+=max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  */
  /*if(fabs(point_to_bottle_dist)<0.00001)
  candidate_points_to_put.weight[i]+=1;
  else
  candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
  */
  } 
  
  printf(" max_weight=%lf\n",max_weight);
  for(i=0;i<candidate_points_to_show.no_points;i++)
  {
  point_in_global_frame[0] = candidate_points_to_show.point[i].x;
  point_in_global_frame[1] = candidate_points_to_show.point[i].y;
  point_in_global_frame[2] = candidate_points_to_show.point[i].z;
  point_in_global_frame[3] = 1;
  ////Assigning weight based on the closeness to the bottle current position
  double bottle_pos[3];
  double sig_dist=2.0;
  bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
  double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
  printf(" Old weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
  candidate_points_to_show.weight[i]+=2.0*max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  
  printf(" New weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
  }
  
MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  
}


int assign_weights_on_candidte_points_to_hide_obj()
{
  
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_matInvertXform(ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos, hum_pos_inverse);

  configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); /* Allocation of temporary robot configuration */

 p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


////double yaw=M_PI/3.0;

////double orig_pan=hum_cur_pos[HUMANq_PAN]; 
  int i=0;
  double Amplitude=1.0;
  double yaw_mean=hum_cur_pos[HUMANq_PAN];
  double pitch_mean=hum_cur_pos[HUMANq_TILT];
  double sig_yaw=M_PI/2.0;
  double sig_pitch=M_PI/2.0;
  double max_weight=0;
////int i=0;
  double bottle_pos[3];
  double sig_dist=2.0;
  bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];

  for(i=0;i<candidate_points_to_hide.no_points;i++)
  {
  point_in_global_frame[0] = candidate_points_to_hide.point[i].x;
  point_in_global_frame[1] = candidate_points_to_hide.point[i].y;
  point_in_global_frame[2] = candidate_points_to_hide.point[i].z;
  point_in_global_frame[3] = 1;
  ////Assigning weight based on the closeness to the bottle current position
  
  ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
  double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
  ////printf(" Old weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
  candidate_points_to_hide.weight[i]=Amplitude*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  
  printf(" New weight for candidate point %d to hide with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_hide.weight[i]);
  }
  
MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  
}


int find_candidate_points_to_hide_obj_new()
{
 printf(" Inside find_candidate_points_to_hide_obj_new()\n");
////no_candidate_points_to_show=0;
candidate_points_to_hide.no_points=0;

 int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
       ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
    {
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   visible_by_human=0;
   visible_by_HRP2=0;
   reachable_by_turning_around=0;
   reachable_by_bending=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==0)
     {
      printf("* Not Visible by human\n");
      ////////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2_neck_turn==1)
      ////////////{
       ////////////printf("** Visible by HRP2 neck turn\n");
       ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
       ////{
       //// printf("*** Bending reachable by by human \n");

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
        {
         printf("**** Reachable by HRP2 \n");
         candidate_points_to_hide.point[candidate_points_to_hide.no_points].x=cell_x_world;
         candidate_points_to_hide.point[candidate_points_to_hide.no_points].y=cell_y_world;
         candidate_points_to_hide.point[candidate_points_to_hide.no_points].z=cell_z_world;
         candidate_points_to_hide.weight[candidate_points_to_hide.no_points]=0.0;
         candidate_points_to_hide.no_points++;

        } 
       ////} 
      ////////////} 
     } 
    }   
   }  
  }
 }
      
printf(" candidate_points_to_hide.no_points=%d\n",candidate_points_to_hide.no_points);
 return candidate_points_to_hide.no_points;

}


int find_candidate_points_to_show_obj_new()
{
 printf(" Inside find_candidate_points_to_show_obj_new()\n");
////no_candidate_points_to_show=0;
candidate_points_to_show.no_points=0;

 int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
       ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].is_horizontal_surface==1)
    {
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   visible_by_human=0;
   visible_by_HRP2=0;
   reachable_by_turning_around=0;
   reachable_by_bending=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
     {
      printf("* Visible by human\n");
      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
      {
       printf("** Visible by HRP2\n");
       ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
       {
       //// printf("*** Bending reachable by by human \n");

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
        {
         printf("**** Reachable by HRP2 \n");
         candidate_points_to_show.point[candidate_points_to_show.no_points].x=cell_x_world;
         candidate_points_to_show.point[candidate_points_to_show.no_points].y=cell_y_world;
         candidate_points_to_show.point[candidate_points_to_show.no_points].z=cell_z_world;
         candidate_points_to_show.weight[candidate_points_to_show.no_points]=0;
         candidate_points_to_show.no_points++;

        } 
       } 
      } 
     } 
    }   
   }  
  }
 }
      
printf(" candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);
 return candidate_points_to_show.no_points;

}


int find_candidate_points_on_plane_to_put_obj_new()
{
 printf(" \n****Inside find_candidate_points_on_plane_to_put_obj_new()\n");
no_candidate_points_to_put=0;
candidate_points_to_put.no_points=0;

 int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    ////{
       ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
    {
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   visible_by_human=0;
   visible_by_HRP2=0;
   reachable_by_turning_around=0;
   reachable_by_bending=0;

       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
     {
      printf("* Visible by human\n");
      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1)
      {
       ////printf("** Visible by HRP2\n");
       if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)
       {
        printf("*** Bending reachable by by human \n");

        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
        {
         printf("**** Reachable by HRP2 \n");
         candidate_points_to_put.point[candidate_points_to_put.no_points].x=cell_x_world;
         candidate_points_to_put.point[candidate_points_to_put.no_points].y=cell_y_world;
         candidate_points_to_put.point[candidate_points_to_put.no_points].z=cell_z_world;
         candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
         candidate_points_to_put.no_points++;

        } 
       } 
      } 
     } 
    }   
   }  
  }
 }
      
printf(" candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);
 return candidate_points_to_put.no_points;

}



int find_candidate_points_on_plane_to_put_obj()
{
no_candidate_points_to_put=0;
candidate_points_to_put.no_points=0;

int current_surface_index=0;
for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
{

 int i=0;
 for(;i<curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max;i++)
 {
  int j=0;
  int show_cell=0;
  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;
  //printf("\n");
  for(;j<curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max;j++)
  {
   show_cell=0;
   reachable_by_hand=0;
   reachable_by_HRP2_hand=0;
   visible_by_human=0;
   visible_by_HRP2=0;
   reachable_by_turning_around=0;
   reachable_by_bending=0;

   double x=i*surf_grid_samp_rate;
   double y=j*surf_grid_samp_rate;
   x+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_x_min;
   y+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_y_min;
   double z=curr_surfaces_in_env.flat_surf[current_surface_index].BR_z;

   double cur_z_st=z;
   double height=0.1;//0.3;
   double cur_z_end=z;
   //double cur_z_end=;
   //printf("%d ", curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible);
   ////printf("SHOW_VISIBLE_PLACE=%d\n",SHOW_VISIBLE_PLACE);
    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible==1)
    {
 
   //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
   //cur_z_st+=cur_z_end;
   //cur_z_end+=cur_z_st;
    show_cell=1;
    visible_by_human=1;

    }
    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible_by_HRP2==1)
    {
 
   //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
   //cur_z_st+=cur_z_end;
   //cur_z_end+=cur_z_st;
    show_cell=1;
    visible_by_HRP2=1;
    }

    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_hand=1;
    }
    if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_hand==1)//Already reachable by right hand
     reachable_by_hand=3; //Reachable by both hands
     else
     reachable_by_hand=2; //Reachable by left hand only
  
    }
     if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
      reachable_by_bending=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_bending==1)//Already reachable by right hand
     reachable_by_bending=3; //Reachable by both hands
     else
     reachable_by_bending=2; //Reachable by left hand only
    }
     if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_RHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
    show_cell=1;
  reachable_by_turning_around=1;
 ////   g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_LHand_by_turning_around_bending==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
    show_cell=1;

      if(reachable_by_turning_around==1)//Already reachable by right hand
     reachable_by_turning_around=3; //Reachable by both hands
     else
     reachable_by_turning_around=2; //Reachable by left hand only
    }

   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_RHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
     show_cell=1;
     reachable_by_HRP2_hand=1;
    //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
   
    }
   if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_LHand==1)
    {
    //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
     show_cell=1;
     if(reachable_by_HRP2_hand==1)//Already reachable by right hand
     reachable_by_HRP2_hand=3; //Reachable by both hands
     else
     reachable_by_HRP2_hand=2; //Reachable by left hand only
    }
    
  
   
    ////if(visible_by_HRP2==1&&visible_by_human==1)
    if(visible_by_human==1) 
     {
    if(reachable_by_HRP2_hand==3&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3)) //Reachable by both hands of both human and HRP2
      {
     ////g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
     
     candidate_points_to_put.point[candidate_points_to_put.no_points].x=x;
     candidate_points_to_put.point[candidate_points_to_put.no_points].y=y;
     candidate_points_to_put.point[candidate_points_to_put.no_points].z=z;
     candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
     candidate_points_to_put.no_points++;
     ////candidate_points_to_put[no_candidate_points_to_put].x=x;
     ////candidate_points_to_put[no_candidate_points_to_put].y=y;
     ////candidate_points_to_put[no_candidate_points_to_put].z=z;
     ////no_candidate_points_to_put++;
    
     
      }
    if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
      {
     ////g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
     candidate_points_to_put.point[candidate_points_to_put.no_points].x=x;
     candidate_points_to_put.point[candidate_points_to_put.no_points].y=y;
     candidate_points_to_put.point[candidate_points_to_put.no_points].z=z;
     candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
     candidate_points_to_put.no_points++;

     ////candidate_points_to_put.point[no_candidate_points_to_put].x=x;
     ////candidate_points_to_put.point[no_candidate_points_to_put].y=y;
     ////candidate_points_to_put.point[no_candidate_points_to_put].z=z;
     ////no_candidate_points_to_put++;
      }
     } 
    }
   }
  }
 
}

  
int make_cells_around_point_obstacle_free(double hand_pos[3], int expansion)
{ 
 printf(" Inside  make_cells_around_point_obstacle_free=(%lf,%lf,%lf)\n",hand_pos[0],hand_pos[1],hand_pos[2]);
hri_bitmap_cell* current_cell=NULL;
 current_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], hand_pos[0], hand_pos[1], hand_pos[2]);

       if(current_cell!=NULL)
        { 
        printf(" Inside  make_cells_around_point_obstacle_free, cell=(%d,%d,%d)\n",current_cell->x,current_cell->y,current_cell->z);
        grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[current_cell->x][current_cell->y][current_cell->z].val=0;
        

         
           int i=0; 
           for(i=current_cell->x-expansion;i<=current_cell->x+expansion;i++)
           {
            //printf(" for i = %d\n",i);
            if(i>=0&&i<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
            {
             int j=0;
             for(j=current_cell->y-expansion;j<=current_cell->y+expansion;j++)
             {
              //printf(" for i, j = %d %d\n",i, j);
              if(j>=0&&j<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
              {
               int k=0;
                for(k=current_cell->z-expansion;k<=current_cell->z+expansion;k++)
               {
              //printf(" for i, j, k = %d %d %\n",i, j, k);
              if(k>=0&&k<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
                {
                 ////printf(" Populating cell (%d, %d, %d) with 0\n",i,j,k);
                 grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[i][j][k].val=0;
                 
        
                }
               } 
              }
             }
            } 
           }
     } 
    else
     {
     printf(" Can't get cell corresponding to the hand\n");
     } 
}

/*int store_cur_GIK_sol_for_real_HRP2()
{
int i=0;
for(i=0;i<HRP2_GIK_sol.no_configs;i++)
 {
  
 }
 
}*/

int move_HRP2_one_cell_in_curr_path()
{
 
}

int synchronize_HRP2_GIK_model(ghrp2_config_t *hrp2_config)
{

 vectorN HRP2_conf(NO_DOF_HRP2);

        HRP2_conf(0)=hrp2_config->zmp[0];
        HRP2_conf(1)=hrp2_config->zmp[1];
        HRP2_conf(2)=hrp2_config->zmp[2];

        HRP2_conf(3)=hrp2_config->waistRpy[0];
        HRP2_conf(4)=hrp2_config->waistRpy[1];
        HRP2_conf(5)=hrp2_config->waistRpy[2];
        int i=0;

        for (  i=6;i<NO_DOF_HRP2;i++ )
        HRP2_conf(i)=hrp2_config->angles[i];

attStandingRobot->staticState ( HRP2_conf );

   
}

int execute_current_HRP2_GIK_solution(int with_bottle)
{
 printf(" Inside execute_current_HRP2_GIK_solution()\n");
 int i=0;
 configPt bottle_cur_pos;
 configPt rob_cur_pos;
 rob_cur_pos= MY_ALLOC(double,ACBTSET->robot->nb_dof);  

 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

 if(with_bottle==1)
 {
 bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);  

 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 }
/*
 for(i=0;i<HRP2_GIK_sol.no_configs;i++)
   {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  if(with_bottle==1)
  {
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);
  }
  
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
 }*/

 printf(" Before executing the configs, cur_gik_sol_configs.no_configs=%d, rob_cur_pos=(%lf, %lf, %lf)\n",cur_gik_sol_configs.no_configs,rob_cur_pos[6],rob_cur_pos[7],rob_cur_pos[8]);
 for(i=0;i<cur_gik_sol_configs.no_configs;i++)
 {
  ////printf("Executing for i=%d\n",i);
  hrp2_to_M3D_ConfigPt(&cur_gik_sol_configs.gik_sol[i],rob_cur_pos);
  ////printf("After hrp2_to_M3D_ConfigPt()\n");
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos);
  if(with_bottle==1)
  {
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]-0.06; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);
  
  }
 NEED_HRP2_VISIBILITY_UPDATE=1;
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 }
 ////printf(" Before synchronize_HRP2_GIK_model()\n");
 ////synchronize_HRP2_GIK_model(&cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1]);
 ////printf(" After synchronize_HRP2_GIK_model()\n");

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

vectorN first_config(NO_DOF_HRP2);   
vectorN HRP2_conf(NO_DOF_HRP2);

        HRP2_conf(0)=rob_cur_pos[6];
        HRP2_conf(1)=rob_cur_pos[7];
        ////////if(HRP2_CURRENT_STATE==1)//for sitting
        ////////HRP2_conf(2)=rob_cur_pos[8]+ M3D_to_HRP2_GIK_sitting_Z_shift;
        if(HRP2_CURRENT_STATE==1)//for sitting
        HRP2_conf(2)=M3D_to_HRP2_GIK_sitting_Z_shift;
        if(HRP2_CURRENT_STATE==2)//for half sitting
        HRP2_conf(2)=0.6487;

first_config(0)=HRP2_conf(0);
first_config(1)=HRP2_conf(1);
first_config(2)=HRP2_conf(2);


        HRP2_conf(3)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].waistRpy[0];
        HRP2_conf(4)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].waistRpy[1];
        HRP2_conf(5)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].waistRpy[2];
 
first_config(3)=cur_gik_sol_configs.gik_sol[0].waistRpy[0]; 
first_config(4)=cur_gik_sol_configs.gik_sol[0].waistRpy[1];
first_config(5)=cur_gik_sol_configs.gik_sol[0].waistRpy[2];
       
        int j=0;

        for (  j=6;j<NO_DOF_HRP2;j++ )
        {
        HRP2_conf(j)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].angles[j-6];
        first_config(j)=cur_gik_sol_configs.gik_sol[0].angles[j-6];
        }

        printf(" After executing, the current state of HRP2 is :\n");
        
        for(j=0;j< NO_DOF_HRP2;j++)
        {
        
        printf(" first_config(%d)=%lf, backupConfig(%d)=%lf, HRP2_conf(%d) = %lf\n",j,first_config(j),j,backupConfig(j),j,HRP2_conf(j));
        
        }

attStandingRobot->staticState ( HRP2_conf );
////attStandingRobot->staticState ( first_config );

 if(with_bottle==1)
 {
  MY_FREE(bottle_cur_pos, double,ACBTSET->object->nb_dof); 
 }

  MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
 
printf(" After executing the configs, rob_cur_pos=(%lf, %lf, %lf)\n",rob_cur_pos[6],rob_cur_pos[7],rob_cur_pos[8]);

printf(" Returning from execute_current_HRP2_GIK_solution()\n");

return 1;
  
}

int HRP2_grasp_object(int for_hand,double hand_clench_val)
{

////curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

 ////////double clench=0.4;
double clench=hand_clench_val;


//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
Hand_Clench ( for_hand, clench );
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
//Hand_Clench_without_GIK ( for_hand, clench );

/* AKP NOTE: WARNING Below is bug don't uncomment

 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
   //Restore the actual robot configuration
   p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/

}

int HRP2_release_object(int for_hand, double hand_clench_val)
{
HRP2_GIK_sol.no_configs=0;
 ////////double clench=0.3;
double clench=hand_clench_val;
//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
Hand_Clench ( for_hand, clench );
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration

//Hand_Clench_without_GIK ( for_hand, clench );

/* AKP NOTE: WARNING Below is bug don't uncomment

     int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
//Restore the actual robot configuration
   p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/
}

p3d_vector3 point_to_look;

int HRP2_look_at_bottle()
{
    ////configPt bottle_pos   = MY_ALLOC(double,ACBTSET->object->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    ////p3d_get_robot_config_into(ACBTSET->visball,&bottle_pos );
    
  
    
    point_to_look[0] = ACBTSET->object->joints[1]->abs_pos[0][3];
    point_to_look[1] = ACBTSET->object->joints[1]->abs_pos[1][3];
    point_to_look[2] = ACBTSET->object->joints[1]->abs_pos[2][3];
    printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",point_to_look[0],point_to_look[1],point_to_look[2]);

    ////MY_FREE(bottle_pos, double,ACBTSET->object->nb_dof); 
 
    double task_duration=2.0; //in s
    int state=HRP2_CURRENT_STATE;
    int use_body_part=1;//0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet

  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


    HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
HRP2_look_at(point_to_look, task_duration, state, use_body_part);
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    

return 1;    


   
}


int HRP2_look_at_point(p3d_vector3 point_to_look, int use_body_part)//use_body_part=0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet
{
    ////configPt bottle_pos   = MY_ALLOC(double,ACBTSET->object->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    ////p3d_get_robot_config_into(ACBTSET->visball,&bottle_pos );
    
  
    
    
    printf("Point to look is = (%lf, %lf, %lf)\n",point_to_look[0],point_to_look[1],point_to_look[2]);

    ////MY_FREE(bottle_pos, double,ACBTSET->object->nb_dof); 
 
    double task_duration=2.0; //in s
    int state=HRP2_CURRENT_STATE;
    

  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


    HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
HRP2_look_at(point_to_look, task_duration, state, use_body_part);
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    

return 1;    


   
}


int HRP2_look_at_bottle_old()
{
    ////configPt bottle_pos   = MY_ALLOC(double,ACBTSET->object->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    ////p3d_get_robot_config_into(ACBTSET->visball,&bottle_pos );
    
  
    
    point_to_look[0] = ACBTSET->object->joints[1]->abs_pos[0][3];
    point_to_look[1] = ACBTSET->object->joints[1]->abs_pos[1][3];
    point_to_look[2] = ACBTSET->object->joints[1]->abs_pos[2][3];
    printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",point_to_look[0],point_to_look[1],point_to_look[2]);

    ////MY_FREE(bottle_pos, double,ACBTSET->object->nb_dof); 
 
    double task_duration=2.0; //in s
    int state=HRP2_CURRENT_STATE;
    int use_body_part=1;//0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet

  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


    HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
HRP2_look_at(point_to_look, task_duration, state, use_body_part);
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
//AKP NOTE: Warning code below is bug, don't uncomment it

printf("HRP2_GIK_sol.no_configs=%d\n",HRP2_GIK_sol.no_configs);

int j=0;

 for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
////  printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
/*
    int kcd_with_report=0;
 //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision detected with robot at configuration %d, res=%d \n",j, res);
       int pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        int step_back_sol=10;
        int ctr=0;
        for(;pr_sol_ctr>=0&&ctr<step_back_sol;ctr++)
        {
         ////p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[pr_sol_ctr]);
         //// g3d_draw_env();
         //// fl_check_forms();
         //// g3d_draw_allwin_active();
         MY_FREE(HRP2_GIK_sol.configs[pr_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
         HRP2_GIK_sol.no_configs--;
         pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        } 
        //Restoring the actual configuration
        p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
	printf(" Updated robot with total configuration =%d \n",HRP2_GIK_sol.no_configs);
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot at configuration %d, res=%d \n", j, res);
       int pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        int step_back_sol=2;
        int ctr=0;
        for(;pr_sol_ctr>=0&&ctr<step_back_sol;ctr++)
        {
         ////p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[pr_sol_ctr]);
         MY_FREE(HRP2_GIK_sol.configs[pr_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
         HRP2_GIK_sol.no_configs--;
         pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        } 
         //Restoring the actual configuration
        p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
        printf(" Updated robot with total configuration=%d \n",HRP2_GIK_sol.no_configs);
       return 0;
       }
 */

    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  ////g3d_draw_env();
  ////fl_check_forms();
  ////g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
      //Restoring the actual configuration
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
    printf(" Updated robot with total configuration j =%d \n",j);
return 1;    


   
}

point_co_ordi curr_AStar_path[500];
int no_pts_curr_AStar_path=0;

int get_AStar_path(hri_bitmapset * btset, hri_bitmap* bitmap)
{
  point_co_ordi curr_path[500];
  int no_pts=0;
  hri_bitmap_cell* current;
  //int i,j,color;
  //double task_duration=1.0;//1.5;
  if(bitmap == NULL)
    return 0;
  printf(" Inside move_HRP2_hand_with_GIK_on_path with bitmap->searched = %d \n",bitmap->searched);
  if(bitmap->searched) {

    // vertical lines on start and goal
    //g3d_drawOneLine(
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    //g3d_drawOneLine(
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    while(current != bitmap->search_start && current != NULL)
    {
     
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
     current = current->parent;
    }

     //Storing the start cell
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;

    /* //Storing the actual starting position of the hand, which might be different from the position of the starting cell due to discreatization 
     curr_path[no_pts].x=start_hand_pos.x;
     curr_path[no_pts].y=start_hand_pos.y;
     curr_path[no_pts].z=start_hand_pos.z;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
   */


    printf(" No. of cells in the path = %d \n",no_pts);
 
   //Storing the hand pos as first point
   //////// curr_AStar_path[no_pts_curr_AStar_path].x=start_hand_pos.x;
   ////////  curr_AStar_path[no_pts_curr_AStar_path].y=start_hand_pos.y;
   ////////  curr_AStar_path[no_pts_curr_AStar_path].z=start_hand_pos.z;
   ////////  no_pts_curr_AStar_path++;

    //Storing the hand pos as first point
    curr_AStar_path[no_pts_curr_AStar_path].x=start_hand_pos.x;
     curr_AStar_path[no_pts_curr_AStar_path].y=start_hand_pos.y;
     curr_AStar_path[no_pts_curr_AStar_path].z=start_hand_pos.z;
     no_pts_curr_AStar_path++;


    int j=no_pts-1;
    
   
    for(;j>=0;j--)
    {
     curr_AStar_path[no_pts_curr_AStar_path].x=curr_path[j].x;
     curr_AStar_path[no_pts_curr_AStar_path].y=curr_path[j].y;
     curr_AStar_path[no_pts_curr_AStar_path].z=curr_path[j].z;
     no_pts_curr_AStar_path++;
    } 
   return no_pts_curr_AStar_path;
  }
  else
  return 0;
   
}

int HRP2_find_collision_free_path_to_take_object_new()//In this version, the entire path is divided into three phases, first reach near to the bottle, then orient the hand then again plan a path from the new hand position to the bottle while maintaining the orientation
{
/*
find_candidate_points_on_plane_to_put_obj_new();
assign_weights_on_candidte_points_to_put_obj();
reverse_sort_weighted_candidate_points_to_put_obj();
CANDIDATE_POINTS_FOR_TASK_FOUND=1;
return 0;
*/

/*
find_candidate_points_to_show_obj_new();
assign_weights_on_candidte_points_to_show_obj();
reverse_sort_weighted_candidate_points_to_show_obj();
CANDIDATE_POINTS_FOR_TASK_FOUND=1;
return 0;
*/

/*
find_candidate_points_to_hide_obj_new();
assign_weights_on_candidte_points_to_hide_obj();
reverse_sort_weighted_candidate_points_to_hide_obj();
CANDIDATE_POINTS_FOR_TASK_FOUND=1;
return 0;
*/

 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_find_collision_free_path_to_take_object(), before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 ////create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 

 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 //As now the robot is in rest pos, the hand position will be stored for returning to rest pos
 right_hand_rest_pos.x=hand_pos[0];
 right_hand_rest_pos.y=hand_pos[1];
 right_hand_rest_pos.z=hand_pos[2];

//****This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 printf(" **** Storing right_hand_rest_pos: (%lf, %lf,%lf) \n",right_hand_rest_pos.x,right_hand_rest_pos.y,right_hand_rest_pos.z);
 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3]+0.1;
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

double expansion=0.05;//in m
double cur_hand_pos[3];
cur_hand_pos[0]=qs[0];
cur_hand_pos[1]=qs[1];
cur_hand_pos[2]=qs[2];
printf(" Calling  make_cells_around_HRP2_RHNAD_as_non_obstacle() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

expansion=1;//no of cells around the cell corresponding to the point qf
make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 

expansion=1;//no of cells around the cell corresponding to the point qf
 make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

//qf[2]+=0.1;
printf(" Finding path from hand to bottle \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to bottle could not be found \n");
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;

    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_path_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path-3); //-3 is Just to reach near the bottle, not exactly at bottle before that hand has to be properly oriented
    printf(" After find_spline_path_for_via_points() to reach near the bottle, no_path_pts=%d, no_spline_points=%d\n",no_path_pts, no_spline_points);
    //Storing the spline points for showing the path
    no_spline_points_btwn_prev_n_curr_exec=0;
 
    for(int tmp_i=0;tmp_i<no_path_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(no_path_pts==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to reach near the bottle \n");
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
    }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=0;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to reach near the bottle \n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
    
 
 printf("****Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
/////////***********///////////
     
     p3d_vector3 req_hand_orientation_in_global_frame;
     req_hand_orientation_in_global_frame[0]=0;
     req_hand_orientation_in_global_frame[1]=0;
     req_hand_orientation_in_global_frame[2]=1;
     
     GIK_path_res=find_HRP2_GIK_sol_for_hand_orientation(req_hand_orientation_in_global_frame,for_hand, state, use_body_part);
     
     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to orient the HRP2 hand for grasping the bottle \n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
     

    printf("NOW finding the path from current correctly oriented hand to the bottle\n");
    //NOW finding the path from current correctly oriented hand to the bottle
     hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 
     qs[0]=hand_pos[0]; 
     qs[1]=hand_pos[1];
     qs[2]=hand_pos[2];

 //This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3]+0.1;
  ////////z_val_of_grasp_point=qf[2];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */



////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

   
  cur_hand_pos[0]=qs[0];
  cur_hand_pos[1]=qs[1];
  cur_hand_pos[2]=qs[2];
  ////printf(" Calling make_cells_around_point_obstacle_free() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 ////make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

////expansion=1;
//// make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

printf(" Finding path from new correctly oriented hand position to bottle \n");
 val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from new correctly oriented hand position to bottle could not be found \n");
  attStandingRobot->staticState ( backupConfig );
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

/*
//Add the exact position of the bottle as the end point as the above path is based on the nearest cell
     curr_AStar_path[no_pts_curr_AStar_path].x=qf[0];
     curr_AStar_path[no_pts_curr_AStar_path].y=qf[1];
     curr_AStar_path[no_pts_curr_AStar_path].z=qf[2];
     no_pts_curr_AStar_path++;
*/

    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
     no_path_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path-1);//Find path one cell before the bottle
    
   //Storing the spline points for showing the path
    
   
    for(int tmp_i=0;tmp_i<no_path_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  ////////no_spline_points++;
    }
    if(no_path_pts==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to reach the bottle for grasping \n");
    attStandingRobot->staticState ( backupConfig );
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
    }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to reach the bottle for grasping.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
    

/////****** Now finding the st. line interpolated path between the cell very near to the bottle to the bottle

hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right h

point_co_ordi tmp_via_pts[2];
tmp_via_pts[0].x=hand_pos[0];
tmp_via_pts[0].y=hand_pos[1];
tmp_via_pts[0].z=hand_pos[2];



tmp_via_pts[1].x=ACBTSET->object->joints[1]->abs_pos[0][3];
tmp_via_pts[1].y=ACBTSET->object->joints[1]->abs_pos[1][3];
tmp_via_pts[1].z=ACBTSET->object->joints[1]->abs_pos[2][3]+0.1;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
z_val_of_grasp_point=qf[2];

double sampling_period=5e-3; 
no_spline_points=0;
double t2=0.0;
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;



   
    resultant_spline[no_spline_points].x=x2;
    resultant_spline[no_spline_points].y=y2;
    resultant_spline[no_spline_points].z=z2;
    no_spline_points++;

    spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
    spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
    spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
    no_spline_points_btwn_prev_n_curr_exec++;
     }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to reach the bottle for grasping.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
 printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

/////////***********///////////

attStandingRobot->staticState ( backupConfig );
p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

if(cur_gik_sol_configs.no_configs>0)
return cur_gik_sol_configs.no_configs;
else
return 0;

////show_HRP2_gik_sol();
 /*
int i=0;
//while(1)
//{
        

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);

  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
*/

//store_cur_GIK_sol_for_real_HRP2();
 
}

int HRP2_find_collision_free_path_to_take_object()
{
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_find_collision_free_path_to_take_object(), before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 ////create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 

 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 //As now the robot is in rest pos, the hand position will be stored for returning to rest pos
 right_hand_rest_pos.x=hand_pos[0];
 right_hand_rest_pos.y=hand_pos[1];
 right_hand_rest_pos.z=hand_pos[2];

  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 printf(" **** Storing right_hand_rest_pos: (%lf, %lf,%lf) \n",right_hand_rest_pos.x,right_hand_rest_pos.y,right_hand_rest_pos.z);
 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

double expansion=0.025;//in m
double cur_hand_pos[3];
cur_hand_pos[0]=qs[0];
cur_hand_pos[1]=qs[1];
cur_hand_pos[2]=qs[2];
printf(" Calling make_cells_around_point_obstacle_free() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

printf(" Finding path from hand to bottle \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to bottle could not be found \n");
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to take the bottle \n");
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
    }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     int maintain_hand_orientation=0;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     return 0;
     }
    
 printf("****Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
/////////***********///////////


     

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
 /*
int i=0;
//while(1)
//{
        

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);

  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
*/

//store_cur_GIK_sol_for_real_HRP2();
 
}

int make_cells_around_point_as_obstacle(hri_bitmapset *btset, int bt_type, point_co_ordi point, int extension)
{
hri_bitmap* bitmap;

  bitmap = btset->bitmap[bt_type];

int x,y,z;

x=(point.x-btset->realx)/btset->pace;  
y=(point.y-btset->realy)/btset->pace;
z=(point.z-btset->realz)/btset->pace;
bitmap->data[x][y][z].val =-3;  //-3 to indicate that it has been marked as obstacle due to collision during path planning using GIK, it is not actual obstacle

 if((x>0&&x<bitmap->nx)&&(y>0&&y<bitmap->ny)&&(z>0&&z<bitmap->nz))
 {
  int x1=x-extension;
  
  for(x1=x-extension; x1<x+extension; x1++)
  {
   int y1=y-extension; 
   for(y1=y-extension;y1<y+extension;y1++) 
   {
   int z1=z-extension; 
   for(z1=z-extension;z1<z+extension;z1++) 
   
        ////printf(" Making cell (%d,%d,%d) as value 0 \n",x,y,z); 
   bitmap->data[x1][y1][z1].val =-3;  //-3 to indicate that it has been marked as obstacle due to collision during path planning using GIK, it is not actual obstacle
   } 
  }
 }
}



int HRP2_return_hand_to_rest_position()
{
 HRP2_CURRENT_TASK=3;
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 qf[0]=right_hand_rest_pos.x;
 qf[1]=right_hand_rest_pos.y;
 qf[2]=right_hand_rest_pos.z;

 int no_trials=0;
 int path_found=0;
 int break_loop=0;
 while(no_trials<10&&break_loop==0)
 {

 printf(" **** Using right_hand_rest_pos: (%lf, %lf,%lf) \n",right_hand_rest_pos.x,right_hand_rest_pos.y,right_hand_rest_pos.z);
  
int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);

  if(path_res>0)
  {
printf("**** Path found for returning to rest position for the right hand of HRP2\n");
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;

 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
 
//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

////AKP NOTE : Following function call is for straight line interpolation
////   int col_free_GIK_path=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);


//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
    no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to return to rest position \n");
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }
    
     int maintain_hand_orientation=0;
    int col_free_GIK_path=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
attStandingRobot->staticState ( backupConfig );

     if(col_free_GIK_path==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to return to rest position\n");
     return 0;
     }
    
/////////////////END for using spline



p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
   if(col_free_GIK_path==0)
   {
   printf(" no_trials=%d\n",no_trials);
   printf(" *** Could not found collision free path for returning the right hand to the rest position, trying another path\n");
   int extension=1;
   make_cells_around_point_as_obstacle(grid_around_HRP2.GRID_SET, HRP2_GIK_MANIP, point_of_curr_collision,extension);
   
   }
   else
   {
   path_found=1;
   break_loop=1;
   }
  }
 no_trials++;
 }
 if(path_found==1)
 {
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];
////show_HRP2_gik_sol();

printf(" HRP2_GIK_sol.no_configs=%d\n",HRP2_GIK_sol.no_configs);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
   {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
    }
 */
 return 1;

 }
 else
 {
  printf(" AKP WARNING : Could not found collision free path for returning the right hand to the rest position\n");
  return 0;
 }

 
 
  
 

}



int HRP2_put_object_for_human_to_take_new()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_put_object_for_human_to_take_new() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

MY_FREE(hand_pos, double, 3);




  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 


ChronoOff();

ChronoOn();



//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
no_pts_curr_AStar_path=0;
no_spline_points=0;
 vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;
no_spline_points_btwn_prev_n_curr_exec=0;


find_candidate_points_on_plane_to_put_obj_new();
printf("***candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);
if(candidate_points_to_put.no_points==0)
{
 ChronoOff();
printf("AKP WARNING: candidate_points_to_put.no_points=%d, so not trying to put the bottle.\n",candidate_points_to_put.no_points);
return 0;
}

assign_weights_on_candidte_points_to_put_obj();
reverse_sort_weighted_candidate_points_to_put_obj();


 //////***** Finding a path to first lift the hand with bottle

printf(" Finding path to lift the hand with bottle \n");

point_co_ordi tmp_via_pts[2];
tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];



tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z+0.06;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];

double sampling_period=5e-3; 
no_spline_points=0;
double t2=0.0;
printf("  qs = (%lf, %lf, %lf), qf = (%lf, %lf, %lf)  \n", qs[0], qs[1], qs[2], qf[0], qf[1], qf[2]);
    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
  if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }

   int maintain_hand_orientation=1;
   int  GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lift the bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lift the hand with bottle


  double *new_hand_pos;
new_hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
printf(" Earlier qs was (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);
 qs[0]=new_hand_pos[0];
 qs[1]=new_hand_pos[1];
 qs[2]=new_hand_pos[2];
MY_FREE(new_hand_pos, double, 3);
printf(" After lifting hand new qs is (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);

//////// find_candidate_points_on_plane_to_put_obj_new();
////////assign_weights_on_candidte_points_to_put_obj();
////////reverse_sort_weighted_candidate_points_to_put_obj();

//This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=qs[0];
  start_hand_pos.y=qs[1];
  start_hand_pos.z=qs[2];
////////////////////////////
 configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 
 p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;





///////////////////////
vectorN backupConfig_for_next_iteration  = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model after lifting the bottle, it will be used to restore the configuration, if the path to next iteration is not found, so that testing for the next candidate point could be done
printf("*****candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);

 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  

  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  ////////qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  qf[2]=qs[2]; //To maintain the z value 
printf(" Testing for candidate_points_to_put %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
   
    bottle_cur_pos[6]=qf[0];
  bottle_cur_pos[7]=qf[1];
  bottle_cur_pos[8]=candidate_points_to_put.point[test_p_ctr].z-0.06;
  
   int OK_to_Show=0;
   p3d_set_and_update_this_robot_conf(ACBTSET->object, bottle_cur_pos);
   if(psp_is_object_visible(currRob, ACBTSET->object, 95, FALSE))
   {
   OK_to_Show=0;
   }
   else
   {
   OK_to_Show=1;
   }

   if(OK_to_Show==1)
   {
 int expansion=0;
//AKP NOTE: TODO Undo the cells arround obstacle free if need to test for next point
 make_cells_around_point_obstacle_free(qf,expansion); 


  printf(" Finding path from hand to cnadidate point \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to candidate point %d could not be found \n", test_p_ctr);
 ////return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
      ////////  cur_gik_sol_configs.no_configs=0;
      ////////  cur_gik_sol_configs.gik_sol[0].time=0.0; 
////////p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

////////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path); 

    //Storing the spline points for showing the path
    ////////no_spline_points_btwn_prev_n_curr_exec=0;
    int prev_no_spline_points_btwn_prev_n_curr_exec=no_spline_points_btwn_prev_n_curr_exec;

    for(int tmp_i=0;tmp_i<no_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(no_pts>0)
    {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_put_found=1;
  HRP2_HAND_spline_path_calculated=1;
   printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=1;

    double prev_no_conf=cur_gik_sol_configs.no_configs;
    double prev_time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time; 

    int prev_no_HRP2_hand_pos=no_HRP2_hand_pos;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );
printf("****Inside HRP2_put_object_for_human_to_take_new(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to put the bottle for candidate point %d\n",test_p_ctr);
    no_spline_points_btwn_prev_n_curr_exec=prev_no_spline_points_btwn_prev_n_curr_exec;
    no_HRP2_hand_pos=prev_no_HRP2_hand_pos;
    cur_gik_sol_configs.no_configs=prev_no_conf;
    cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time=prev_time; 
  
    printf(" So Resetting the valid config and time as : cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf \n",cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    
      attStandingRobot->staticState ( backupConfig_for_next_iteration );
     ////attStandingRobot->staticState ( backupConfig );
     ////return 0;
     }
     else
     {

   

   point_to_put_found=1;
  ////////attStandingRobot->staticState ( backupConfig );
  break;
  
     }
    }
  else
    {
   printf("Fail to find path for candidate_points_to_put.point[%d] \n",test_p_ctr);
    }
    
  }//End if(OK_to_Show)
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n"); 
 attStandingRobot->staticState ( backupConfig );
 return 0;
 }


 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];



no_spline_points=0;
 


 //////***** Finding a path to lower the hand with bottle

printf(" Finding path to lower the hand with bottle \n");

tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];

tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z-0.04;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];


no_spline_points=0;
 t2=0.0;
printf("  qs = (%lf, %lf, %lf), qf = (%lf, %lf, %lf) \n", qs[0], qs[1], qs[2], qf[0], qf[1], qf[2]);
    //Storing the spline points for showing the path
    
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
   
    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lower the hand with bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lower the hand with bottle
 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;

    
   ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

   printf(" Returning from HRP2_put_object_for_human_to_take_new();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
attStandingRobot->staticState ( backupConfig );

return 1;
////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}


int HRP2_hide_object_from_human_new()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_hide_object_from_human_new() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

MY_FREE(hand_pos, double, 3);




  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 


ChronoOff();

ChronoOn();

find_candidate_points_to_hide_obj_new();
assign_weights_on_candidte_points_to_hide_obj();
reverse_sort_weighted_candidate_points_to_hide_obj();

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
no_pts_curr_AStar_path=0;
no_spline_points=0;
 vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;
no_spline_points_btwn_prev_n_curr_exec=0;
 //////***** Finding a path to first lift the hand with bottle

printf(" Finding path to lift the hand with bottle \n");

point_co_ordi tmp_via_pts[2];
tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];



tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z+0.06;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];

double sampling_period=5e-3; 
no_spline_points=0;
double t2=0.0;
printf("  qs (%lf, %lf, %lf) = \n", qs[0], qs[1], qs[2]);
    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_hide_object_from_human_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
  if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }

   int maintain_hand_orientation=1;
   int  GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lift the bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("**** after calling find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lift the hand with bottle


  double *new_hand_pos;
new_hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
printf(" Earlier qs was (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);
 qs[0]=new_hand_pos[0];
 qs[1]=new_hand_pos[1];
 qs[2]=new_hand_pos[2];
MY_FREE(new_hand_pos, double, 3);
printf(" After lifting hand new qs is (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);

//////// find_candidate_points_on_plane_to_put_obj_new();
////////assign_weights_on_candidte_points_to_put_obj();
////////reverse_sort_weighted_candidate_points_to_put_obj();

//This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=qs[0];
  start_hand_pos.y=qs[1];
  start_hand_pos.z=qs[2];

 
 int test_p_ctr=0;
///////////////////////////


 int point_to_hide_found=0;
 

 configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 
 p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

vectorN backupConfig_for_next_iteration  = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model after lifting the bottle, it will be used to restore the configuration, if the path to next iteration is not found, so that testing for the next candidate point could be done
printf(" ****candidate_points_to_hide.no_points=%d\n",candidate_points_to_hide.no_points);
 for(;test_p_ctr<candidate_points_to_hide.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_hide %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);

  
  qf[0]=candidate_points_to_hide.point[test_p_ctr].x;
  qf[1]=candidate_points_to_hide.point[test_p_ctr].y;
  qf[2]=candidate_points_to_hide.point[test_p_ctr].z;
  bottle_cur_pos[6]=qf[0];
  bottle_cur_pos[7]=qf[1];
  bottle_cur_pos[8]=qf[2];

  qf[2]=qs[2];
  ////qf[2]=qs[2]; //To maintain the z value 
  
   int OK_to_Hide=0;
   p3d_set_and_update_this_robot_conf(ACBTSET->object, bottle_cur_pos);
   if(psp_is_object_visible(currRob, ACBTSET->object, 5, FALSE))
   {
   OK_to_Hide=0;
   }
   else
   {
   OK_to_Hide=1;
   }

   point_to_hide_found=0;

   if(OK_to_Hide==1)
   { 
 int expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

point_to_hide_found=0;

 int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to candidate point %d could not be found \n", test_p_ctr);
 ////return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
      ////////  cur_gik_sol_configs.no_configs=0;
      ////////  cur_gik_sol_configs.gik_sol[0].time=0.0; 
////////p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

////////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path); //-3 is Just to reach near the bottle, not exactly at bottle before that hand has to be properly oriented

    //Storing the spline points for showing the path
    ////////no_spline_points_btwn_prev_n_curr_exec=0;
    int prev_no_spline_points_btwn_prev_n_curr_exec=no_spline_points_btwn_prev_n_curr_exec;

    for(int tmp_i=0;tmp_i<no_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(no_pts>0)
    {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_hide_found=1;
  HRP2_HAND_spline_path_calculated=1;
   printf("****Inside HRP2_hide_object_from_human(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=1;

    double prev_no_conf=cur_gik_sol_configs.no_configs;
    double prev_time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time; 
    int prev_no_HRP2_hand_pos=no_HRP2_hand_pos;

    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );
printf("****Inside HRP2_hide_object_from_human(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to hide the bottle for candidate point %d\n",test_p_ctr);
    cur_gik_sol_configs.no_configs=prev_no_conf;
    cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time=prev_time; 
    no_spline_points_btwn_prev_n_curr_exec=prev_no_spline_points_btwn_prev_n_curr_exec;
     no_HRP2_hand_pos=prev_no_HRP2_hand_pos;

     attStandingRobot->staticState ( backupConfig_for_next_iteration );
     ////return 0;
     }
     else
     {

   

   point_to_hide_found=1;
  ////////attStandingRobot->staticState ( backupConfig );
  break;
  
     }
    }
  else
    {
   printf("Fail to find path for candidate_points_to_put.point[%d] \n",test_p_ctr);
    }
  }//END if(OK_to_Hide==1)
 }




///////////////////

 

if(point_to_hide_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n"); 
 attStandingRobot->staticState ( backupConfig );
 return 0;
 }


 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

MY_FREE(hand_pos, double, 3);

no_spline_points=0;
 


 //////***** Finding a path to lower the hand with bottle

printf(" Finding path to lower the hand with bottle \n");

tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];

tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z-0.04;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];


no_spline_points=0;
 t2=0.0;
printf("  qs (%lf, %lf, %lf) = \n", qs[0], qs[1], qs[2]);
    //Storing the spline points for showing the path
    
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
   
    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lower the hand with bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lower the hand with bottle
 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;

    
   ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

   printf(" Returning from HRP2_put_object_for_human_to_take_new();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
attStandingRobot->staticState ( backupConfig );
return 1;


}


int HRP2_put_object_for_human_to_take()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 


ChronoOff();

ChronoOn();

 find_candidate_points_on_plane_to_put_obj_new();
assign_weights_on_candidte_points_to_put_obj();
reverse_sort_weighted_candidate_points_to_put_obj();

 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_put %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  ////////qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  qf[2]=qs[2]; //To maintain the z value 

   
 int expansion=0;
 make_cells_around_point_obstacle_free(qf,expansion); 



  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_put_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_put.point[%d] \n",test_p_ctr);
  }
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n");
 return 0;
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to put the bottle \n");
    attStandingRobot->staticState ( backupConfig );
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }

    int maintain_hand_orientation=1;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);


attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to put the bottle \n");
      ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

     return 0;
     }
    
   ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

printf(" Returning from HRP2_put_object_for_human_to_take();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}

int HRP2_show_object_to_human_new()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
ChronoOff();
ChronoOn();
 find_candidate_points_to_show_obj_new();
assign_weights_on_candidte_points_to_show_obj();
reverse_sort_weighted_candidate_points_to_show_obj();
vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;

vectorN backupConfig_for_next_iteration  = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model after lifting the bottle, it will be used to restore the configuration, if the path to next iteration is not found, so that testing for the next candidate point could be done

 int point_to_show_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_show.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_show %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
  qf[0]=candidate_points_to_show.point[test_p_ctr].x;
  qf[1]=candidate_points_to_show.point[test_p_ctr].y;
  qf[2]=candidate_points_to_show.point[test_p_ctr].z;
  ////qf[2]=qs[2]; //To maintain the z value 

   
 int expansion=0;
 make_cells_around_point_obstacle_free(qf,expansion); 

no_spline_points=0;

  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_show.point[%d] \n",test_p_ctr);
  
  

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
       cur_gik_sol_configs.no_configs=0;
       cur_gik_sol_configs.gik_sol[0].time=0; 
       p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation


    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }

////////////////////////////////////////////////
no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
      ////////  cur_gik_sol_configs.no_configs=0;
      ////////  cur_gik_sol_configs.gik_sol[0].time=0.0; 
////////p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation


    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path); //-3 is Just to reach near the bottle, not exactly at bottle before that hand has to be properly oriented

      no_spline_points_btwn_prev_n_curr_exec=0;
      int prev_no_spline_points_btwn_prev_n_curr_exec=no_spline_points_btwn_prev_n_curr_exec;

    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }
    if(no_pts>0)
    {
  printf("Path found for candidate_points_to_show.point[%d] \n",test_p_ctr);
  point_to_show_found=1;
  HRP2_HAND_spline_path_calculated=1;
   printf("****Inside HRP2_show_object_to_human_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=1;

    double prev_no_conf=cur_gik_sol_configs.no_configs;
    double prev_time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time; 
     int prev_no_HRP2_hand_pos=no_HRP2_hand_pos;
    
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );
printf("****Inside HRP2_put_object_for_human_to_take_new(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to put the bottle for candidate point %d\n",test_p_ctr);
    cur_gik_sol_configs.no_configs=prev_no_conf;
    cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time=prev_time; 
    no_spline_points_btwn_prev_n_curr_exec=prev_no_spline_points_btwn_prev_n_curr_exec;
     prev_no_HRP2_hand_pos=prev_no_HRP2_hand_pos;

     attStandingRobot->staticState ( backupConfig_for_next_iteration );
     ////return 0;
     }
      else
     {

   point_to_show_found=1;
  ////////attStandingRobot->staticState ( backupConfig );
  break;
  
     }
    }
  else
    {
   printf("Fail to find path for candidate_points_to_show.point[%d] \n",test_p_ctr);
    }
  }//End if(path_res>0)
 }//End for(;test_p_ctr<candidate_points_to_show.no_points;test_p_ctr++)




///////////////////

 

if(point_to_show_found==0)
 {
 printf(" **** AKP Warning : No feasible point to show the object has been found.\n"); 
 attStandingRobot->staticState ( backupConfig );
 return 0;
 }
else
 {
  printf(" **** Feasible point to show the object has been found.\n"); 
  attStandingRobot->staticState ( backupConfig );
  return 1;
 } 

 

}

int HRP2_show_object_to_human()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
ChronoOff();
ChronoOn();
 find_candidate_points_to_show_obj_new();
assign_weights_on_candidte_points_to_show_obj();
reverse_sort_weighted_candidate_points_to_show_obj();

 int point_to_show_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_show.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_show %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
  qf[0]=candidate_points_to_show.point[test_p_ctr].x;
  qf[1]=candidate_points_to_show.point[test_p_ctr].y;
  qf[2]=candidate_points_to_show.point[test_p_ctr].z;
  ////qf[2]=qs[2]; //To maintain the z value 

   
 int expansion=0;
 make_cells_around_point_obstacle_free(qf,expansion); 



  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_show.point[%d] \n",test_p_ctr);
  point_to_show_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_show.point[%d] \n",test_p_ctr);
  }
 }

if(point_to_show_found==0)
 {
 printf(" **** AKP Warning : No feasible point to show the object has been found.\n");
 return 0;
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to show the bottle \n");
    attStandingRobot->staticState ( backupConfig );
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }

    int maintain_hand_orientation=1;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);

attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to show the bottle \n");
     return 0;
     }
    
   ChronoPrint("\n<<<<<< Time for finding final solution for show ");
ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

printf(" Returning from HRP2_show_object_to_human();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}

int HRP2_hide_object_from_human()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_hide_object_from_human(), before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 ChronoOff();
ChronoOn();
 find_candidate_points_to_hide_obj_new();
assign_weights_on_candidte_points_to_hide_obj();
reverse_sort_weighted_candidate_points_to_hide_obj();

 int point_to_hide_found=0;
 int test_p_ctr=0;

 configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 
 p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

 for(;test_p_ctr<candidate_points_to_hide.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_hide %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);

  
  qf[0]=candidate_points_to_hide.point[test_p_ctr].x;
  qf[1]=candidate_points_to_hide.point[test_p_ctr].y;
  qf[2]=candidate_points_to_hide.point[test_p_ctr].z;
  bottle_cur_pos[6]=qf[0];
  bottle_cur_pos[7]=qf[1];
  bottle_cur_pos[8]=qf[2];

  qf[2]=qs[2];
  ////qf[2]=qs[2]; //To maintain the z value 
  
   int OK_to_Hide=0;
   p3d_set_and_update_this_robot_conf(ACBTSET->object, bottle_cur_pos);
   if(psp_is_object_visible(currRob, ACBTSET->object, 5, FALSE))
   {
   OK_to_Hide=0;
   }
   else
   {
   OK_to_Hide=1;
   }

   point_to_hide_found=0;

   if(OK_to_Hide==1)
   { 
 int expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

point_to_hide_found=0;

  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_hide.point[%d] \n",test_p_ctr);
  point_to_hide_found=1;
  ////break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_hide.point[%d] \n",test_p_ctr);
   point_to_hide_found=0;
  }
  }
 //////}

//////if(point_to_show_found==0)
////// {
////// printf(" **** AKP Warning : No feasible point to hide the object has been found.\n");
////// return 0;
////// } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
  if(point_to_hide_found==1)
  {
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to hide the bottle \n");
    attStandingRobot->staticState ( backupConfig );
    point_to_hide_found=0;
    //////return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }

    int maintain_hand_orientation=1;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);

attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to hide the bottle \n");
     point_to_hide_found=0;
     //////return 0;
     }
     else
     {
      point_to_hide_found=1;
      break;
     }
   }
 }

 if(point_to_hide_found==0)
 {
 printf(" **** AKP Warning : No feasible point to hide the object has been found.\n");
 return 0;
 } 
    
   
ChronoPrint("\n <<<<<<<<>>>>>>>>>>>Time for getting the solution to hide object\n");
ChronoOff();
p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

printf(" Returning from HRP2_hide_object_from_human();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}

int put_object_for_human_to_take_old() //Which is actually a combination of first grasp object by HRP2 then it will put the object
{
 
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" put_object_for_human_to_take_old, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 

 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

int expansion=1;
double cur_hand_pos[3];
cur_hand_pos[0]=qs[0];
cur_hand_pos[1]=qs[1];
cur_hand_pos[2]=qs[2];
printf(" Calling make_cells_around_point_obstacle_free() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

printf(" Finding path from hand to bottle \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to bottle could not be found \n");
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
int i=0;
//while(1)
//{

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

//}  

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 find_candidate_points_on_plane_to_put_obj();
 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  printf(" Testing for %d \n",test_p_ctr);
  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_put[%d] \n",test_p_ctr);
  point_to_put_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_put[%d] \n",test_p_ctr);
  }
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n");
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);

i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  /*envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  bottle_cur_pos[6]=hand_pos[0];
  bottle_cur_pos[7]=hand_pos[1];
  bottle_cur_pos[8]=hand_pos[2];
*/
  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }


 

}


int grid_created=0;
int show_3d_grid_for_HRP2_GIK()
{
if(grid_created==0)
 {
 grid_created=1;
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

 qs[0]= rob_cur_pos[6]+0.1;
 qs[1]= rob_cur_pos[7]-0.9;
 qs[2]= rob_cur_pos[8]+0.8;
 
 qf[0]=qs[0]+0.5;
 qf[1]=qs[1]+1.75;
 qf[2]=qs[2]+0.35;
  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

create_3d_grid_for_HRP2_GIK();

 
//printf("
Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;
move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
int i=0;
//while(1)
//{

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

//}  

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  printf(" Testing for %d \n",test_p_ctr);
  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_put_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_put[%d] \n",test_p_ctr);
  }
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n");
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);

i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  /*envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  bottle_cur_pos[6]=hand_pos[0];
  bottle_cur_pos[7]=hand_pos[1];
  bottle_cur_pos[8]=hand_pos[2];
*/
  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

/* 
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;
    clench=5;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );


  j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }

i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
 
  p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
 
*/

 }

 /*show_3d_grid_Bounding_box_for_HRP2_GIK();
 g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 show_HRP2_gik_sol();
 */
 ////find_HRP2_GIK_collision_free_path(qs, qf, grid_around_HRP2.GRID_SET, 1);
 
 /*
     int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
  
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/
 //show_gik_sol();

 int show_obstacle_cells=1;
 if(show_obstacle_cells==1)
 {
 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    {
       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
       g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
        //fl_check_forms();
       //g3d_draw_allwin_active();
    }
    ////else
    ////{
    ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
    ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
    ////} 
    //  fl_check_forms();
   }
  } 
 }
 }//End if(show_obstacle_cells==1)
}

/*********************ASTAR**************************************/
/*!
 * \brief Calculate the cost of a cell when reached from a different cell
 * sets cell-> val unless for collision in soft obstacle
 * \param cell the cell
 *
 * \return FALSE and sets cell value to a negative number in case of a collision
 */
/****************************************************************/
int Calculate_AStar_Cell_Value(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell )
{
  configPt qc,q_o;
  double saved[3];

 
      cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);
      cell->q = qc;
   


  return cell->val >= 0; // non-negative means no collision
}

/*********************ASTAR***************************************/
/*!
 * \brief A* search: calculate neighbours
 *
 * all neighbors not opened yet will be opened, all openedneighbors will
 * all opened neighbors will be updated if they are cheaper to reach by the center cell
 *
 * \param bitmap the bitmap
 * \param center_cell whose neighbours
 * \param final_cell  goal cell
 * \param reached TRUE if we reach to the goal
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int Calculate_AStar_neigh_costs(hri_bitmapset* btset, hri_bitmap* bitmap, hri_bitmap_cell* center_cell, hri_bitmap_cell* final_cell, int bitmap_index)
{
 //// printf("Inside Calculate_AStar_neigh_costs\n");
  int i,j,k; // loop xyz coordinates
  int x, y, z; // center cell coordinates
  int refx1, refy1, refx2, refy2; // 2step reference cell xy coordinates
  hri_bitmap_cell *current_cell, *overstepped1, *overstepped2;
  double new_cell_g, overstep_val, distance;

  if(center_cell == NULL)
  { 
    printf(" AKP Warning: center_cell = NULL, so returning false \n");
    return FALSE;
  }
  else{
    x=center_cell->x; y=center_cell->y; z=center_cell->z;
  }

  if( ! on_map(center_cell->x, center_cell->y, center_cell->z, bitmap) ) {
    //PrintError(("cant get cell\n"));
    return FALSE;
  }

  /* iterate over all direct non-closed neighbors in bitmap */
  for(i=-1; i<2; i++){ // -1 to 1
    for(j=-1; j<2; j++){ // -1 to 1
      for(k=-1; k<2; k++){// -1 to 1
     // printf(" Checking cell (%d, %d, %d) \n",x+i, y+j, z+k);  
        if(i==0 && j==0 && k==0) continue; // center cell
        if (! on_map(x+i, y+j, z+k, bitmap)) {
          //printf(" Cell is not in the map \n");
          continue;
        }

        if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
          //PrintError(("Can't get cell\n"));
          return FALSE;
        }

        if(btset->bitmap[bitmap_index]->data[x+i][y+j][z+k].val < 0 )
        {
       // printf(" cell (%d, %d, %d) is at obstacle\n",x+i, y+j, z+k);  
        ////printf(" cell is at obstacle \n");
        continue; /* Is the cell in obstacle? */
        }

        /* closed cells already have a minimum path to start, and all neighbors opened */
        if(current_cell->closed) 
        {
       // printf(" Current cell is already closed \n");
        continue;
        }
        /* open cells might have less g cost for going through current node */
        int manhattan_distance = ABS(i) + ABS(j) + ABS(k);
         if (manhattan_distance == 1) {
           distance= 1; // normal grid step
         } else if(manhattan_distance == 2) {
           distance = M_SQRT2; // 2d diagonal step
         } else if(manhattan_distance == 3) {
           distance = M_SQRT3; // 3d diagonal step
         }
        if (current_cell->open) {
        // printf(" current cell is open \n");
          new_cell_g = hri_bt_A_CalculateCellG(btset, current_cell, center_cell, distance);

          if(current_cell->g > new_cell_g){
            current_cell->g =  new_cell_g;
            current_cell->parent = center_cell;
            hri_bt_A_update_cell_OL(current_cell);
          } else {
            continue;
          }

        } else { // cell was neither open nor closed
          //printf(" current cell is neither open nor closed \n");
          if (Calculate_AStar_Cell_Value(btset, bitmap, current_cell, center_cell) == FALSE)
            continue;// leave untouched
          current_cell->h = hri_bt_dist_heuristic(bitmap,current_cell->x,current_cell->y,current_cell->z);
          // TK:dead code never used because of if before
//          if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].val == BT_OBST_POTENTIAL_COLLISION
//              && btset->manip == BT_MANIP_NAVIGATION) {
//            fromcellno = get_direction(current_cell, center_cell);
//            if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].obstacle[fromcellno]==TRUE) // it was !=, PRAGUE
//              continue;
//          }

          current_cell->g = hri_bt_A_CalculateCellG(btset, current_cell, center_cell, distance);
          current_cell->parent = center_cell;

           //printf("Calling hri_bt_A_insert_OL(), g=%f now\n",current_cell->g); 
          hri_bt_A_insert_OL(current_cell);
          current_cell->open = TRUE;
        }
      }
    }
  } // end for immediate neighbors

  /**
   * as an optimisation in 2D to basic A* over the grid, we also search the 2 step
   * diagonals that would be reached by going 1 step horizontal or vertical, and one step diagonal.
   *
   * We use the value and open properties use for single grid steps calculated earlier.
   */
  if (bitmap->nz == 1) {
    k = 0;
    for(i=-2; i<3; i++){ // -2 to 2
      for(j=-2; j<3; j++){ // -2 to 2
        //        for(k=-2; k<3; j++){ // -2 to 2
        if (abs(i)!=2 && abs(j)!=2 && abs(k)!=2) continue; // not a 2 step border cell
        if (i==0 || j==0 || k==0) {
          // in plane on center cell, target cells are 3 manhattan steps away
          if (abs(i) + abs(j) + abs(k) != 3) {
            continue;
          }
        } else { // 3D case
          if (abs(i) + abs(j) + abs(k) != 4) {
            continue;
          }
        }
        if (! on_map(x+i, y+j, z+k, bitmap)) {
          continue;
        }


        if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
          PrintError(("Can't get cell\n"));
          return FALSE;
        }

        if(btset->bitmap[bitmap_index]->data[x+i][y+j][z+k].val < 0) continue; /* Is the cell in obstacle? */

        /* closed cells already have a minimum path to start, and all neighbors opened */
        if(current_cell->closed) continue;

        // 2D code, 2 reference points stepped over
        //ref1 is horizontal / vertical stepped over cell, ref2 diagonally stepped over cell
        if (i == -2) {
          refx1 = -1;
        } else if (i == 2) {
          refx1 = 1;
        } else {
          refx1 = 0;
        }
        if (j == -2) {
          refy1 = -1;
        } else if (j == 2) {
          refy1 = 1;
        } else {
          refy1 = 0;
        }
        if (i < 0) {
          refx2 = -1;
        } else  {
          refx2 = 1;
        }
        if (j < 0) {
          refy2 = -1;
        } else  {
          refy2 = 1;
        }
        overstepped1 = hri_bt_get_cell(bitmap, x+refx1, y+refy1, z+k);
        overstepped2 = hri_bt_get_cell(bitmap, x+refx2, y+refy2, z+k);

        // only continue if both overstepped cells have been opened, and add the max val to g later
        if (overstepped1->open && overstepped2->open) {
          overstep_val =  (overstepped1->val + overstepped2->val) / 2 ;
        } else {
          /* TODO: For more completeness, check the 2 step localpath for collisions,
           * and calculate equivalent cost for a double step */
          continue;
        }

        /* open cells might have less g cost for going through current node */
        if (current_cell->open) {
          new_cell_g = overstep_val + hri_bt_A_CalculateCellG(btset, current_cell, center_cell, M_SQRT5);

          if(current_cell->g > new_cell_g){
            current_cell->g =  new_cell_g;
            current_cell->parent = center_cell;
            hri_bt_A_update_cell_OL(current_cell);
          } else {
            continue;
          }

        } else { // cell was neither open nor closed
          if (Calculate_AStar_Cell_Value(btset, bitmap, current_cell, center_cell) == FALSE)
            continue;// leave untouched
          current_cell->h = hri_bt_dist_heuristic(bitmap,current_cell->x,current_cell->y,current_cell->z);

          current_cell->g = overstep_val + hri_bt_A_CalculateCellG(btset, current_cell, center_cell, M_SQRT5);
          current_cell->parent = center_cell;

          /*  printf("It is g=%f now\n",current_cell->g); */
          ////printf("Calling hri_bt_A_insert_OL(), g=%f now\n",current_cell->g); 
          hri_bt_A_insert_OL(current_cell);
          current_cell->open = TRUE;
        }
        //        }
      }
    } // end for distant diagonal neighbors
  } // end if 2d then smooth
////printf(" Returning true\n");
  return TRUE;
}

double calculate_AStar_path(hri_bitmapset *btset, hri_bitmap* bitmap, int bitmap_index)
{

  hri_bitmap_cell * current_cell;;
  int reached = FALSE;

  

  if(bitmap->search_start == NULL || bitmap->search_goal == NULL){
    printf("AKP Warning Inside calculate_AStar_path(), but start/final cell is NULL\n");
    return -1;
  }
  if(bitmap->search_start == bitmap->search_goal) {
  printf("AKP Warning Inside calculate_AStar_path(), but bitmap->search_start == bitmap->search_goal\n");
    return 0;
  }

  current_cell = bitmap->search_start;

  hri_bt_init_BinaryHeap(bitmap); // ALLOC 

  if(!hri_bt_close_cell(bitmap,current_cell)){
    PrintError(("cant close start cell!\n"));
    return -1;
  }

  printf(" Calling Calculate_AStar_neigh_costs for current_cell (%d,%d,%d)\n",current_cell->x,current_cell->y,current_cell->z);
  Calculate_AStar_neigh_costs(btset, bitmap, current_cell, bitmap->search_goal,bitmap_index);
  int ctr=0;

  while(!reached) {
   ////printf(" ctr = %d\n", ctr);
   ctr++;
    if( hri_bt_A_Heap_size()==0){
      PrintError(("hri_bt_A_Heap_size()=0, A*:no path found!"));
      hri_bt_destroy_BinaryHeap();
      return -1;
    }
    current_cell = hri_bt_A_remove_OL();
    if(current_cell == bitmap->search_goal) {
      reached = TRUE;
      break;
    }
    hri_bt_close_cell(bitmap, current_cell);
    Calculate_AStar_neigh_costs(btset, bitmap, current_cell, bitmap->search_goal,bitmap_index);
  }
  bitmap->searched = TRUE;

  printf("\ncost: %f \n", hri_bt_A_CalculateCellG(btset, bitmap->search_goal, bitmap->search_goal->parent, getCellDistance(bitmap->search_goal, bitmap->search_goal->parent)));


  // TK: This line looks like a bug, as bitmapset definitions do not allow bitmaps to change type
  //  bitmap->type = BT_PATH;

  hri_bt_destroy_BinaryHeap();


  return hri_bt_A_CalculateCellG(btset, bitmap->search_goal, bitmap->search_goal->parent, getCellDistance(bitmap->search_goal, bitmap->search_goal->parent));
}

void  hri_bt_reset_GIK_path(hri_bitmapset * btset)
{
  hri_bitmap* bitmap;

  bitmap = btset->bitmap[HRP2_GIK_MANIP];

  if(bitmap == NULL){
    PrintError(("Try to erase a path from non path bitmap\n"));
    return;
  }

  bitmap->search_start = NULL;
  bitmap->search_goal = NULL;
  bitmap->current_search_node = NULL;
  bitmap->searched = FALSE;

  int x,y,z;

  if(bitmap == NULL)
    return;

  for(x=0; x<bitmap->nx; x++){
    for(y=0; y<bitmap->ny; y++){
      for(z=0; z<bitmap->nz; z++){
        ////bitmap->data[x][y][z].val = 0;
        bitmap->data[x][y][z].h = -1;
        bitmap->data[x][y][z].g = 0;
        bitmap->data[x][y][z].parent = NULL;
        bitmap->data[x][y][z].closed = FALSE;
        bitmap->data[x][y][z].open   = FALSE;
        bitmap->data[x][y][z].x = x;
        bitmap->data[x][y][z].y = y;
        bitmap->data[x][y][z].z = z;
        bitmap->data[x][y][z].locked = FALSE;
      }
    }
  }
  ////hri_bt_reset_bitmap_data(bitmap);

  btset->pathexist = FALSE;

}

int Find_AStar_Path(double qs[3], double qf[3], hri_bitmapset* bitmapset, int manip)
{
  ////p3d_graph *G;
  ////p3d_list_node *graph_node;
  printf(" Inside Find_AStar_Path with qs=(%lf,%lf,%lf) and qf=(%lf,%lf,%lf) \n",qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);

  hri_bitmap* bitmap;
  bitmap = bitmapset->bitmap[HRP2_GIK_MANIP];
 
  //// if(!BTGRAPH)     G = hri_bt_create_graph(bitmapset->robot);
  ////else             G = BTGRAPH;

  ////bitmapset->robot->GRAPH = G;
  ////graph_node = G->nodes; 
   
  ////configPt q_s = p3d_get_robot_config(bitmapset->robot);
  ////G->search_start  = p3d_APInode_make(G,q_s);
  ////p3d_insert_node(G,G->search_start);
  ////p3d_create_compco(G,G->search_start);
  ////G->search_start->type = ISOLATED;

  ////configPt  q_g = p3d_get_robot_config(G->rob);

  ////G->search_goal  = p3d_APInode_make(G,q_g);
  ////p3d_insert_node(G,G->search_goal);
  ////p3d_create_compco(G,G->search_goal);
  ////G->search_goal->type = ISOLATED;

  hri_bitmap_cell* new_search_start=NULL;
  hri_bitmap_cell* new_search_goal=NULL;
  
  new_search_start  =  hri_bt_get_closest_cell(bitmapset, bitmap, qs[0], qs[1], qs[2]);
  new_search_goal  =   hri_bt_get_closest_cell(bitmapset, bitmap, qf[0], qf[1], qf[2]);

  if(new_search_start == NULL) {
    printf("Search start cell does not exist\n");
    bitmapset->pathexist = FALSE;
    return -4;
  }
  if(new_search_goal == NULL ){
    printf("Search goal cell does not exist\n");
    bitmapset->pathexist = FALSE;
    return -4;
  }  

// reset the path, also clears data from earlier failed attempts when !pathexist
  hri_bt_reset_GIK_path(bitmapset);
 

  printf(" search_start cell (%d, %d, %d) \n",new_search_start->x,new_search_start->y,new_search_start->z);
  printf(" search_goal cell (%d, %d, %d) \n",new_search_goal->x,new_search_goal->y,new_search_goal->z);
  bitmap->search_start = new_search_start;
  bitmap->search_goal  = new_search_goal;

  printf("Values at start and goal cells are %lf, %lf\n",bitmap->data[new_search_start->x][new_search_start->y][new_search_start->z].val,bitmap->data[new_search_goal->x][new_search_goal->y][new_search_goal->z].val);

 if( bitmap->data[new_search_start->x][new_search_start->y][new_search_start->z].val<0)
  {
  printf(" AKP Warning : Start cell is at obstacle, so can't find path \n");
  return -1;
  }
 
 if( bitmap->data[new_search_goal->x][new_search_goal->y][new_search_goal->z].val<0)
  {
  printf(" AKP Warning : Goal cell is at obstacle, so can't find path \n");
  return -1;
  }
  

  /******** calculating the path costs *************/
  double result = calculate_AStar_path(bitmapset, bitmap, HRP2_GIK_MANIP);

 


  if (result > -1) {
    bitmapset->pathexist = TRUE;
    return result;
  } else {
    bitmapset->pathexist = FALSE;
    return -5;
  }
}



int get_Hermite_polynomial_points(point_co_ordi st_point, point_co_ordi end_point, point_co_ordi st_vel, point_co_ordi end_vel, double sampling_rate, point_co_ordi *resultant_points)
{


 double t1;
 int no_pts=0;
 double t;
 for(t=0;t<=1;t+=sampling_rate)
 {
 t1=t;

 resultant_points[no_pts].x=st_point.x*(t1-1)*(t1-1)*(2*t1+1)+end_point.x*t1*t1*(3-2*t1)+st_vel.x*t1*(t1-1)*(t1-1)-end_vel.x*t1*t1*(1-t1);
 resultant_points[no_pts].y=st_point.y*(t1-1)*(t1-1)*(2*t1+1)+end_point.y*t1*t1*(3-2*t1)+st_vel.y*t1*(t1-1)*(t1-1)-end_vel.y*t1*t1*(1-t1);
 resultant_points[no_pts].z=st_point.z*(t1-1)*(t1-1)*(2*t1+1)+end_point.z*t1*t1*(3-2*t1)+st_vel.z*t1*(t1-1)*(t1-1)-end_vel.z*t1*t1*(1-t1);
 no_pts++;
 }
return no_pts;
/* for( i=0;i<=2;i+=.005)
 {
 float x=(1-i)*(1-i)*(1-i)*x1+3*(1-i)*(1-i)*i*x2+3*(1-i)*i*i*x3+i*i*i*x4;
 float y=(1-i)*(1-i)*(1-i)*y1+3*(1-i)*(1-i)*i*y2+3*(1-i)*i*i*y3+i*i*i*y4;
 putpixel(x,y,GREEN);
 delay(10);
 }  */
/*
 for(i=0;i<=1;i+=.005)
 {
 float x=(1-i)*(1-i)*(1-i)*x1+8*(1-i)*(1-i)*i*x2+3*(1-i)*i*i*x3+i*i*i*x4;
 float y=(1-i)*(1-i)*(1-i)*y1+8*(1-i)*(1-i)*i*y2+8*(1-i)*i*i*y3+i*i*i*y4;
 putpixel(x,y,GREEN);
 delay(20);
 }
*/

}

int get_cubic_spline_by_Hermite_polynomial(point_co_ordi point[100], int n, point_co_ordi init_vel, point_co_ordi final_vel, int continuity_constraint_type, double sampling_period, double total_time, point_co_ordi *resultant_spline)
{
 FILE *vel_profile=fopen("/home/akpandey/velocity_profile.txt","w");
 //float
 printf(" Inside get_cubic_spline_by_Hermite_polynomial() with n=%d \n",n);
 
 point_co_ordi tmp_resultant_spline[500];
 
 point_co_ordi st_point, end_point;
 point_co_ordi st_vel, end_vel;
 st_vel.x=init_vel.x;
 st_vel.y=init_vel.y;
 st_vel.z=init_vel.z;
 
 double speed=sqrt((st_vel.x*st_vel.x)+(st_vel.y*st_vel.y)+(st_vel.z*st_vel.z));
 fprintf(vel_profile, "st_vel_x=%lf\nst_vel_y=%lf\nst_vel_z=%lf\nspeed=%lf\n", st_vel.x, st_vel.y, st_vel.z, speed);
 
 
 if(continuity_constraint_type==2) // means the second derivative i.e. the accelaration should also be continuous everywhere
 {
 end_vel.x=3.0/4.0*(point[2].x-point[0].x);
 end_vel.y=3.0/4.0*(point[2].y-point[0].y);
 end_vel.z=3.0/4.0*(point[2].z-point[0].z);
 }
 else
 {
 if(continuity_constraint_type==1) // means the first derivative i.e. the velocity should be continuous everywhere
  {
  end_vel.x=0.5*(point[2].x-point[0].x);
  end_vel.y=0.5*(point[2].y-point[0].y);
  end_vel.z=0.5*(point[2].z-point[0].z);
  }
 }
 //end_vel.x=0.0;
 //end_vel.y=0.0;

 //double t_scaling=0.5;

int total_no_of_curves=n-1;//total no of curves will be 1 less than the total no of points

////double time_for_single_segment=total_time/total_no_of_curves;
////printf(" time_for_single_segment=%lf\n",time_for_single_segment);

double sampling_interval=sampling_period;
printf(" sampling_interval for cubic spline = %lf \n", sampling_interval); 
 //float sampling_interval=0.1;
 int i=0;
 int total_no_pts=no_spline_points;//This counter will set to be the already stored points in the resultant_spline
 for(i=0;i<n-1;i++)
 {
 st_point.x=point[i].x;
 st_point.y=point[i].y;
 st_point.z=point[i].z;
 end_point.x=point[i+1].x;
 end_point.y=point[i+1].y;
 end_point.z=point[i+1].z;

  //co_ordi tmp_st_vel, tmp_end_vel;
 //draw_Hermite_polynomial(st_point, end_point, st_vel, end_vel, t_scaling, GREEN);
 printf(" For segment %d, Calling get_Hermite_polynomial_points() \n", i);

  if(i==0)
  sampling_interval=sampling_period/2.0;
  else
  sampling_interval=sampling_period;

  int no_pts=get_Hermite_polynomial_points(st_point, end_point, st_vel, end_vel, sampling_interval, tmp_resultant_spline);
  printf(" Got no_pts=%d for segment %d \n",no_pts, i);
 int j=0; 
 for(;j<no_pts;j++)
  {
  resultant_spline[total_no_pts].x=tmp_resultant_spline[j].x; 
  resultant_spline[total_no_pts].y=tmp_resultant_spline[j].y;
  resultant_spline[total_no_pts].z=tmp_resultant_spline[j].z;
  total_no_pts++;
  } 
 
 
 st_vel.x=end_vel.x;
 st_vel.y=end_vel.y;
 st_vel.z=end_vel.z;

 double speed=sqrt((st_vel.x*st_vel.x)+(st_vel.y*st_vel.y)+(st_vel.z*st_vel.z));
  fprintf(vel_profile, "st_vel_x=%lf\nst_vel_y=%lf\nst_vel_z=%lf\nspeed=%lf\n", st_vel.x, st_vel.y, st_vel.z, speed);
  
 if(i==n-3)
 {
 end_vel.x=final_vel.x;
 end_vel.y=final_vel.y;
 end_vel.z=final_vel.z;
 }
 else
 {
 
 if(continuity_constraint_type==2) // means the second derivative i.e. the accelaration should also be continuous everywhere
 {
 end_vel.x=3.0/4.0*(point[i+3].x-point[i+1].x);
 end_vel.y=3.0/4.0*(point[i+3].y-point[i+1].y);
 end_vel.z=3.0/4.0*(point[i+3].z-point[i+1].z);
 }
 else
 {
 if(continuity_constraint_type==1) // means the first derivative i.e. the velocity should be continuous everywhere
  {
  end_vel.x=0.5*(point[i+3].x-point[i+1].x);
  end_vel.y=0.5*(point[i+3].y-point[i+1].y);
  end_vel.z=0.5*(point[i+3].z-point[i+1].z);
  }
 } 
 // end_vel.x=0.0;
 //end_vel.y=0.0;

 }

 }
 
 fclose(vel_profile);
 printf(" Returning total_no_pts = %d, from get_cubic_spline_by_Hermite_polynomial() \n",total_no_pts); 
return total_no_pts;

}


//int find_3D_voronoi()


///// AKP : END Affordance analysis

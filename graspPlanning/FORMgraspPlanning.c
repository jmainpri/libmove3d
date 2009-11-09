#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "../other_libraries/gbM/src/Proto_gbModeles.h"
#include <list>
#include <string>
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"

static char OBJECT_GROUP_NAME[256]="jido-ob_lin"; // "jido-ob"; //


static bool display_grasps= false;
static p3d_rob *ROBOT= NULL; // the robot
static p3d_rob *HAND_ROBOT= NULL; // the hand robot
static p3d_obj *OBJECT= NULL; // the object to grasp
static p3d_polyhedre *POLYHEDRON= NULL; // the polyhedron associated to the object
static gpHand_properties HAND;  // information about the used hand
static gpArm_type ARM_TYPE= GP_PA10; // type of the robot's arm
static p3d_vector3 CMASS; // object's center of mass
static p3d_matrix3 IAXES; // object's main inertia axes
static double IAABB[6]; // bounding box aligned on the object's inertia axes
static std::list<gpGrasp> GRASPLIST;
static gpGrasp GRASP;   // the current grasp
static std::list<gpPose> POSELIST, POSELIST2;
static gpPose POSE;
static bool LOAD_LIST= false;
static bool INIT_IS_DONE= false;
static double DMAX_FAR= 0.05;
static double DMAX_NEAR= 0.003;

static unsigned int CNT= 0;
static configPt *PATH= NULL;
static int NB_CONFIGS= 0;

void draw_trajectory(configPt* configs, int nb_configs);
void draw_grasp_planner();
void draw_test();
void key1();
void key2();

// extern int GP_Init(char *objectName);
// extern p3d_cntrt* GP_GetArmCntrt(p3d_rob *robotPt);
// extern int GP_ComputeGraspList(char *objectName);
// extern configPt GP_FindGraspConfig(bool &needs_to_move);
// extern int GP_FindPath();
// extern int GP_FindPathForArmOnly();
// extern configPt* GP_GetTrajectory(p3d_rob *robotPt, p3d_traj *traj, int &nb_configs);
// extern configPt* GP_GetAllTrajectoriesAsOne(p3d_rob *robotPt, int &nb_configs);
// extern int GP_ConcateneAllTrajectories(p3d_rob *robotPt);
// extern void GP_Reset();
// extern void Gp_ResetGraph();

double COLOR_TAB[15][3]= {  {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0} , {1.0, 0.5, 0.5}, {0.5, 1.0, 0.5}, {0.5, 0.5, 1.0}, {1.0, 0.25, 0.5}, {1.0, 0.5, 0.25}, {0.25, 1.0, 0.5}, {0.5, 1.0, 0.25}, {0.25, 0.5, 1.0}, {0.5, 0.25, 1.0}  };

#define NB_POINTS_MAX 10000
static unsigned int NB_POINTS= 500;
static p3d_vector3 POINTS[NB_POINTS_MAX];
static p3d_vector3 RAND_POINT;
static bool INSIDE= false;
static gpConvexHull3D *chull= NULL;

/* --------- FORM VARIABLES ------- */
FL_FORM  * GRASP_PLANNING_FORM = NULL;
static FL_OBJECT * MOTIONGROUP;
static FL_OBJECT * BT_GRASP_OBJ;
static FL_OBJECT * BT_GO_AND_GRASP_OBJ;
static FL_OBJECT * BT_ARM_ONLY_OBJ;
static FL_OBJECT * BT_CAMERA_OBJ;
static FL_OBJECT * BT_RESET_OBJ;
static FL_OBJECT * BT_TEST_OBJ;
static FL_OBJECT * BT_DISPLAY_GRASPS_OBJ;
static FL_OBJECT * BT_LOAD_GRASP_LIST_OBJ;
/* ------------------------------------------ */


/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_grasp_planning_group(void);
static void CB_grasp_planner_obj(FL_OBJECT *obj, long arg);
static void CB_go_and_grasp_obj(FL_OBJECT *obj, long arg);
static void CB_arm_only_obj(FL_OBJECT *obj, long arg);
static void CB_camera_obj(FL_OBJECT *obj, long arg);
static void CB_reset_obj(FL_OBJECT *obj, long arg);
static void CB_test_obj(FL_OBJECT *obj, long arg);
static void CB_display_grasps_obj(FL_OBJECT *obj, long arg);
static void CB_load_grasp_list_obj(FL_OBJECT *obj, long arg);
/* ------------------------------------------ */


/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_grasp_planning_form(void)
{
  GRASP_PLANNING_FORM = fl_bgn_form(FL_UP_BOX, 150, 440);

  g3d_create_grasp_planning_group();
  fl_end_form();
}

void g3d_show_grasp_planning_form(void)
{
  fl_show_form(GRASP_PLANNING_FORM, FL_PLACE_SIZE, TRUE, "Grasp Planning");
}

void g3d_hide_grasp_planning_form(void)
{
  fl_hide_form(GRASP_PLANNING_FORM);
}

void g3d_delete_grasp_planning_form(void)
{
  fl_free_form(GRASP_PLANNING_FORM);
}
/* ------------------------------------------ */



/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_grasp_planning_group(void)
{
  int x, y, dy, w, h;
  FL_OBJECT *obj;

  obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 410, "Grasp planning");

  MOTIONGROUP = fl_bgn_group();

  x= 15;
  y= 30;
  w= 120;
  h= 40;
  dy= h + 10;
  BT_GRASP_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Grasp planner");
  BT_GO_AND_GRASP_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + dy, w, h, "Go and grasp the object");
  BT_ARM_ONLY_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "Grasp the object");
  BT_CAMERA_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 3*dy, w, h, "Camera (+screenshot)");
  BT_RESET_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + 4*dy, w, h, "Reset");
  BT_TEST_OBJ  = fl_add_button(FL_NORMAL_BUTTON, x, y + 5*dy, w, h, "Test");
  BT_DISPLAY_GRASPS_OBJ  = fl_add_button(FL_RADIO_BUTTON, x, y + 6*dy, w, h, "Display grasps");
  BT_LOAD_GRASP_LIST_OBJ  = fl_add_button(FL_RADIO_BUTTON, x, y + 7*dy, w, h, "Load grasp list");

  fl_set_call_back(BT_GRASP_OBJ, CB_grasp_planner_obj, 1);
  fl_set_call_back(BT_GO_AND_GRASP_OBJ, CB_go_and_grasp_obj, 2);
  fl_set_call_back(BT_ARM_ONLY_OBJ, CB_arm_only_obj, 2);
  fl_set_call_back(BT_CAMERA_OBJ, CB_camera_obj, 3);
  fl_set_call_back(BT_RESET_OBJ, CB_reset_obj, 1);
  fl_set_call_back(BT_TEST_OBJ, CB_test_obj, 1);
  fl_set_call_back(BT_DISPLAY_GRASPS_OBJ, CB_display_grasps_obj, 1);
  fl_set_object_color(BT_DISPLAY_GRASPS_OBJ,FL_MCOL,FL_GREEN);
  fl_set_button(BT_DISPLAY_GRASPS_OBJ, FALSE);
  fl_set_call_back(BT_LOAD_GRASP_LIST_OBJ, CB_load_grasp_list_obj, 1);
  fl_set_object_color(BT_LOAD_GRASP_LIST_OBJ,FL_MCOL,FL_GREEN);
  fl_set_button(BT_LOAD_GRASP_LIST_OBJ, FALSE);

  fl_end_group();
}

void draw_trajectory(configPt* configs, int nb_configs)
{
  int i;

  if(configs==NULL || nb_configs<=0)
  {  return;  }

  g3d_set_color_mat(Red, NULL);
  for(i=0; i<nb_configs; i++)
  {  gpDraw_solid_sphere(configs[i][6], configs[i][7], 1.0, 0.07, 10);  }

  g3d_set_color_mat(Green, NULL);
  glBegin(GL_LINES);
  for(i=1; i<nb_configs; i++)
  {
    glVertex3d(configs[i-1][6], configs[i-1][7], 1);
    glVertex3d(configs[i][6], configs[i][7], 1);
  }
  glEnd();
}


void redraw()
{
  g3d_win *win= NULL;

  win= g3d_get_cur_win();
  win->fct_draw2= &(draw_grasp_planner);
  g3d_draw_allwin();
  g3d_draw_allwin_active();
}

void init_graspPlanning(char *objectName)
{
  if(p3d_col_get_mode()!=p3d_col_mode_pqp)
  {
    printf("The collision detector MUST be PQP to use graspPlanning module.\n");
    printf("Program must quit.\n");
    exit(0);
  }

  ROBOT= p3d_get_robot_by_name(GP_ROBOT_NAME);

  HAND_ROBOT= NULL;

  //HAND_ROBOT= gpFind_hand_robot(HAND);

  HAND_ROBOT= HAND.initialize();

  if(ROBOT==NULL)
  {
    printf("There is no robot named \"%s\".\n", GP_ROBOT_NAME);
    printf("Program must quit.\n");
    exit(0);
  }
  if(HAND_ROBOT==NULL)
  {
    printf("There is no robot corresponding to one of the defined hand robots.\n");
    printf("Program must quit.\n");
    exit(0);
  }


  OBJECT= p3d_get_obst_by_name(objectName);

  if(OBJECT==NULL)
  {
    printf("There is no object with name \"%s\".\n",objectName);
    printf("Program must quit.\n");
    exit(0);
  }

  POLYHEDRON= OBJECT->pol[0]->poly;
  poly_build_planes(POLYHEDRON);

  Mass_properties mass_prop;
  gpCompute_mass_properties(POLYHEDRON, &mass_prop);
  gpCompute_inertia_axes(&mass_prop, IAXES);
  p3d_vectCopy(mass_prop.r, CMASS);
  gpInertia_AABB(POLYHEDRON, CMASS, IAXES, IAABB);

  printf("center of mass: \n\t %f %f %f \n", CMASS[0], CMASS[1], CMASS[2] );
  printf("inertia axes: \n\t %f %f %f \n", IAXES[0][0], IAXES[0][1], IAXES[0][2] );
  printf("\t %f %f %f \n", IAXES[1][0], IAXES[1][1], IAXES[1][2] );
  printf("\t %f %f %f \n", IAXES[2][0], IAXES[2][1], IAXES[2][2] );
}


void draw_grasp_planner()
{
  p3d_draw_robot_joints((p3d_rob*)(p3d_get_desc_curid(P3D_ROBOT)), 0.1);
  return; 
//   p3d_vector3 cp1, cp2;
//   p3d_rob *rob1= p3d_get_robot_by_name("gripper_robot");
//   p3d_rob *rob2= p3d_get_robot_by_name("robot");
//  if( pqp_robot_robot_distance(rob1, rob2, cp1, cp2)>0)
//  {
//    gpDraw_solid_sphere(cp1[0], cp1[1], cp1[2], 0.1, 6);
//    gpDraw_solid_sphere(cp2[0], cp2[1], cp2[2], 0.1, 6);
//    g3d_drawOneLine(cp1[0], cp1[1], cp1[2],cp2[0], cp2[1], cp2[2], Red, NULL);
//  }

  int cnt= 0;
  for(std::list<gpPose>::iterator iter= POSELIST.begin(); iter!=POSELIST.end(); iter++)
  {
//     if(cnt==1)
    (*iter).draw(0.03);
//break;
    cnt++;
  }

// std::list<gpVector3D> samples;
//  gpSample_horizontal_faces(p3d_get_obst_by_name("box1"), 0.1, samples);
//  for(std::list<gpVector3D>::iterator iter= samples.begin(); iter!=samples.end(); iter++)
//  {
//     (*iter).draw(1,0,0);
//  }

  static bool firstTime= true;
  if(firstTime)
  {
   gpFind_poses_on_object(OBJECT, p3d_get_obst_by_name("box7"), POSELIST, 0.05, 15, POSELIST2);
   printf("%d new poses\n", POSELIST2.size());
   firstTime= false;
  }

  cnt= 0;
  for(std::list<gpPose>::iterator iter= POSELIST2.begin(); iter!=POSELIST2.end(); iter++)
  {
   (*iter).draw(0.03);
   cnt++;
   if(cnt>200)
   {  printf("only the first 200 poses are displayed\n");
     break;
   }
  }


  if(chull!=NULL)
  {
    glPushMatrix();
    glTranslatef(0,0,3);
    chull->draw(false);
    glPopMatrix();


    glDisable(GL_LIGHTING);
    if(INSIDE)
    { glColor3f(0, 0, 1); }
   else
    { glColor3f(1, 0, 0); }
    g3d_drawSphere(RAND_POINT[0], RAND_POINT[1], RAND_POINT[2], 0.15, Red, NULL);
    glEnable(GL_LIGHTING);
  }


//  p3d_rob *robotPt= (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);


//   p3d_matrix4 Twrist, T, Tinv;
//
//   if(strcmp(robotPt->name,"robot")==0)
//   {
//     gpForward_geometric_model_PA10(robotPt, Twrist, false);
//     p3d_matInvertXform(HAND.Thand_wrist, Tinv);
//     p3d_matMultXform(Twrist, Tinv, T);
//   }
//   else
//   {
//    gpGet_wrist_frame(robotPt, Twrist);
//    gpGet_wrist_frame(robotPt, T);
//   }
//
//   draw_frame(Twrist, 0.2);
//   draw_frame(T, 0.1);
//   HAND.draw(T);

  //p3d_matrix4 Tend_eff;
  //gpForward_geometric_model_PA10(ROBOT, Tend_eff);
 // draw_trajectory(PATH, NB_CONFIGS);

  //p3d_vector3 p1, p2;
  p3d_matrix4 pose;
  float mat[16];

  GRASP.draw(0.03);

//  p3d_jnt *j1= get_robot_jnt_by_name(ROBOT, armJoint)

  if(OBJECT!=NULL)
  {
    p3d_get_obj_pos(OBJECT, pose);
    p3d_matrix4_to_OpenGL_format(pose, mat);
/*
    p2[0]= pose[0][3];
    p2[1]= pose[1][3];
    p2[2]= pose[2][3] + 0.05;
    p1[0]= p2[0] + 0.2;
    p1[1]= p2[1] + 0.2;
    p1[2]= p2[2] + 0.5;
    draw_arrow(p1, p2, 1.0, 0.0, 0.0);
*/
  }

  // display all the grasps from the list:
  if(display_grasps)
  {
    for(std::list<gpGrasp>::iterator iter= GRASPLIST.begin(); iter!=GRASPLIST.end(); iter++)
    {   (*iter).draw(0.03);    }
  }


/*
double q[4];
printf("%f %f %f \n", randomPoint[0], randomPoint[1], randomPoint[2]);
if(gpSAHfinger_inverse_kinematics(Twrist, HAND, randomPoint, q, 4))
{
   gpSet_SAHfinger_joint_angles(HAND_ROBOT, HAND, q, 4);
   g3d_set_color_mat(Green, NULL);
glPushMatrix();
  glTranslatef(randomPoint[0], randomPoint[1], randomPoint[2]);
  gpDraw_solid_sphere(0.01, 15);
glPopMatrix();
}
else
{
   g3d_set_color_mat(Red, NULL);
glPushMatrix();
  glTranslatef(randomPoint[0], randomPoint[1], randomPoint[2]);
  gpDraw_solid_sphere(0.01, 15);
glPopMatrix();
}
*/
/*
   glDisable(GL_LIGHTING);
   glPointSize(5);
   glPushMatrix();
   glMultMatrixf(mat);
   for(int i=0; i<POLYHEDRON->nb_faces; i++)
   {
     ind= POLYHEDRON->the_faces[i].the_indexs_points;

     surf_points= sample_triangle_surface(POLYHEDRON->the_points[ind[0]-1], POLYHEDRON->the_points[ind[1]-1], POLYHEDRON->the_points[ind[2]-1], 0.005, &nb_samples);

     glColor3f(1, 0, 0);
     glBegin(GL_POINTS);
       for(int j=0; j<nb_samples; j++)
         glVertex3dv(surf_points[j]);
     glEnd();

     free(surf_points);
     surf_points= NULL;
   }
   glPopMatrix();
   glEnable(GL_LIGHTING);
*/

  return;
}

void draw_test()
{

}

void key1()
{
  CNT++;
 if(CNT>751) CNT= 751;
  printf("CNT= %d\n", CNT);
}

void key2()
{
if(CNT>0)
  CNT--;
  printf("CNT= %d\n", CNT);
}

//! Planification de prise dans le cas d'un objet n'ayant pas besoin d'être décomposé
//! en composantes convexes.
static void CB_grasp_planner_obj(FL_OBJECT *obj, long arg)
{
  unsigned int i;
  static unsigned int count= 1;
  configPt qhand= NULL, qgrasp= NULL;
  p3d_matrix4 objectPose;
  p3d_vector3 objectCenter;
  std::list<gpGrasp>::iterator igrasp;
  G3D_Window *win = NULL;

  //compute the grasp list:
  if(!INIT_IS_DONE)
  {
    init_graspPlanning(GP_OBJECT_NAME_DEFAULT);
    INIT_IS_DONE= true;

    if(ROBOT->nbCcCntrts!=0)
    {
      p3d_desactivateCntrt(ROBOT, ROBOT->ccCntrts[0]);
    }

    gpGrasp_generation(HAND_ROBOT, OBJECT, 0, CMASS, IAXES, IAABB, HAND, HAND.translation_step, HAND.nb_directions, HAND.rotation_step, GRASPLIST);

    printf("Before collision filter: %d grasps.\n", GRASPLIST.size());
    gpGrasp_collision_filter(GRASPLIST, HAND_ROBOT, OBJECT, HAND);
    printf("After collision filter: %d grasps.\n", GRASPLIST.size());
    gpGrasp_stability_filter(GRASPLIST);
    printf("After stability filter: %d grasps.\n", GRASPLIST.size());

    gpGrasp_context_collision_filter(GRASPLIST, HAND_ROBOT, OBJECT, HAND);
    printf("For the current collision context: %d grasps.\n", GRASPLIST.size());
    p3d_col_deactivate_robot(HAND_ROBOT);
  }

  if(GRASPLIST.empty())
  {
    printf("No grasp was found.\n");
    return;
  }

  i= 0;
  for(igrasp=GRASPLIST.begin(); igrasp!=GRASPLIST.end(); igrasp++)
  {
    GRASP= (*igrasp);
    i++;
    if(i>=count)
     break;
  }
  count++;
  if(count>GRASPLIST.size())
  {  count= 1;  }


  p3d_get_obj_pos(OBJECT, objectPose);
  objectCenter[0]= objectPose[0][3] + CMASS[0];
  objectCenter[1]= objectPose[1][3] + CMASS[1];
  objectCenter[2]= objectPose[2][3] + CMASS[2];


  //set hand configuration (for hand robot):
  qhand= p3d_alloc_config(HAND_ROBOT);
  gpInverse_geometric_model_freeflying_hand(HAND_ROBOT, objectPose, GRASP.frame, HAND, qhand);
  qhand[8]= -1; //to put the hand far under the floor
  gpDeactivate_hand_collisions(HAND_ROBOT);
  p3d_set_and_update_this_robot_conf(HAND_ROBOT, qhand);
  p3d_destroy_config(HAND_ROBOT, qhand);
  qhand= NULL;
  gpSet_grasp_configuration(HAND_ROBOT, HAND, GRASP);
  if(qhand!=NULL)
  {  p3d_destroy_config(HAND_ROBOT, qhand);  }

  //find a configuration for the whole robot (mobile base + arm):
  configPt qend= NULL;
  if(ROBOT!=NULL)
  {
    for(i=0; i<150; i++)
    {
      qgrasp= gpRandom_robot_base(ROBOT, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter, ARM_TYPE);

      if(qgrasp==NULL)
      {  break;  }

      qend= NULL;
      qend= gpFind_grasp_from_base_configuration(ROBOT, OBJECT, GRASPLIST, ARM_TYPE, qgrasp, GRASP, HAND);

      if(qend!=NULL)
      {
        p3d_set_and_update_this_robot_conf(ROBOT, qend);
        XYZ_ENV->cur_robot= ROBOT;
        p3d_set_ROBOT_GOTO(qend);
        p3d_destroy_config(ROBOT, qend);
        qend= NULL;
        break;
      }
      p3d_destroy_config(ROBOT, qgrasp);
      qgrasp= NULL;
    }
  }

  if(qgrasp!=NULL)
  {  p3d_destroy_config(ROBOT, qgrasp);  }

  if(i==150)
  {  printf("No platform configuration was found.\n");  }
  else
  {  printf("Grasp planning was successfull.\n");  }

  win= g3d_get_cur_win();
  win->fct_draw2= &(draw_grasp_planner);
  win->x= objectPose[0][3];   win->y= objectPose[1][3];   win->z= objectPose[2][3];
  g3d_draw_allwin();
  g3d_draw_allwin_active();

  return;
}



//! Centers the camera on the object position and takes a screenshot.
static void CB_camera_obj(FL_OBJECT *obj, long arg)
{
  static int count= 0;
  static int firstTime= true;
  char filename[128];

  if(firstTime)
  {
    firstTime= false;
    init_graspPlanning(GP_OBJECT_NAME_DEFAULT);
  }

  gpCompute_stable_poses(OBJECT, CMASS, POSELIST);
  printf("%d poses computed\n", POSELIST.size());
  if(!POSELIST.empty())
  {
    POSE= POSELIST.back();
  }

  for(std::list<gpPose>::iterator iter= POSELIST.begin(); iter!=POSELIST.end(); iter++)
  {   (*iter).print();    }

  sprintf(filename, "screenshot-%d.ppm", count++);
  g3d_export_GL_display(filename);

  G3D_Window *win = g3d_get_cur_win();
  win->fct_draw2= &(draw_grasp_planner);
  win->fct_key1= &(key1);
  win->fct_key2= &(key2);
  g3d_draw_allwin();
}



static void CB_reset_obj(FL_OBJECT *obj, long arg)
{
  GP_Reset();

  printf("GraspPlanning static global values have been reset.\n");
}

static void CB_go_and_grasp_obj(FL_OBJECT *obj, long arg)
{
  bool needs_to_move, so_far_so_good= true;
  int result, path_found;
  double x, y, theta, q1, q2, q3, q4, q5, q6;
  std::vector<double> qhand;
  configPt qstart= NULL, qfinal= NULL, qinter1= NULL, qinter2= NULL, qinter3= NULL, qfar= NULL;
  p3d_rob *robotPt= NULL;
  p3d_cntrt* cntrt_arm = NULL;
  robotPt= p3d_get_robot_by_name(GP_ROBOT_NAME);
  XYZ_ENV->cur_robot= robotPt;

  // initializes everything:
  GP_Init(GP_OBJECT_NAME_DEFAULT);

  redraw();

  cntrt_arm= GP_GetArmCntrt(robotPt);

  if(cntrt_arm==NULL)
  {
   printf("FATAL_ERROR : arm_IK constraint does not exist\n");
   return;
  }

  /* Deactivate the arm_IK constrint */
  p3d_desactivateCntrt(robotPt, cntrt_arm);

  //alloc all configs:
  qstart= p3d_alloc_config(robotPt);
  qfinal= p3d_alloc_config(robotPt);
  qinter1= p3d_alloc_config(robotPt);
  qinter2= p3d_alloc_config(robotPt);
  qinter3= p3d_alloc_config(robotPt);
  qfar= p3d_alloc_config(HAND_ROBOT);

  p3d_get_robot_config_into(robotPt, &qstart);
	p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qstart);
  g3d_draw_allwin_active();

  // computes the grasp list:
  if(!LOAD_LIST)
  {
    result= GP_ComputeGraspList(GP_OBJECT_NAME_DEFAULT);
    gpSave_grasp_list(GRASPLIST, "./graspPlanning/graspList_new.xml");
  }
  else // or loads it:
  {
    result= gpLoad_grasp_list("./graspPlanning/graspList.xml", GRASPLIST);
    if(result==0)
      {
        printf("Can not load a grasp list.\n");
        return;
      }

    if(!GRASPLIST.empty())
    {
      if(GRASPLIST.front().hand_type!=HAND.type)
      {
        printf("The loaded grasp list does not correspond to the current hand type.\n");
        return;
      }
    }
  }


  // move away the hand robot:
  qfar= p3d_alloc_config(HAND_ROBOT);
  qfar[7]= -100; //to put the hand far under the floor
  qfar[8]= -1; //to put the hand far under the floor
  p3d_set_and_update_this_robot_conf(HAND_ROBOT, qfar);
  p3d_destroy_config(HAND_ROBOT, qfar);

  qfinal= GP_FindGraspConfig(needs_to_move);

  p3d_set_and_update_this_robot_conf(robotPt, qstart);
  if(p3d_col_test())
  {
    printf("Start configuration is colliding.\n");
    return;
  }


  if(qfinal!=NULL)
  {
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qfinal);
     printf("Grasp configuration list was successfully computed.\n");
     XYZ_ENV->cur_robot= robotPt;
     p3d_set_ROBOT_GOTO(qfinal);
  }
  else
  {
    printf("No grasp configuration was found.\n");
    so_far_so_good= false;
    redraw();

    //free all configs:
    p3d_destroy_config(robotPt, qstart);
    p3d_destroy_config(robotPt, qfinal);
    p3d_destroy_config(robotPt, qinter1);
    p3d_destroy_config(robotPt, qinter2);
    p3d_destroy_config(robotPt, qinter3);
    return;
  }

  p3d_set_and_update_this_robot_conf(robotPt, qfinal);
  if(p3d_col_test())
  {
    printf("Final configuration is colliding.\n");
    return;
  }

  redraw();

  //if the robot needs to move, we have to introduce three intermediate configurations:
  //    qstart =  (qbase0 ; qarm0 ; qhand0)
  // -> qinter1=  (qbase0 ; qarm_folded ; qhand0)
  // -> qinter2=  (qbase1 ; qarm_folded ; qhand0)
  // -> qinter3=  (qbase1 ; qarm1 ; qhand0)
  // -> qfinal =  (qbase1 ; qarm1 ; qhand1)
  if(needs_to_move)
  {
    // get platform final configuration and arm final configuration:
    p3d_set_and_update_this_robot_conf(robotPt, qfinal);
    gpGet_platform_configuration(robotPt, x, y, theta);
    gpGet_arm_configuration(robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6);
    gpGet_hand_configuration(robotPt, HAND, qhand);

    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    if(HAND.type==GP_GRIPPER) gpOpen_hand(robotPt, HAND);
    result= gpFold_arm(robotPt, ARM_TYPE);
    if(result==0)
    {
      printf("The arm can not be folded.\n");
    }

    p3d_get_robot_config_into(robotPt, &qinter1);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qinter1);
    gpSet_platform_configuration(robotPt, x, y, theta);
    p3d_get_robot_config_into(robotPt, &qinter2);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qinter2);

    gpSet_arm_configuration(robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6);
    p3d_get_robot_config_into(robotPt, &qinter3);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qinter3);

    if(p3d_col_test())
    {
      printf("The robot can not open its hand/gripper in its final arm and base configuration.\n");
      p3d_copy_config_into(robotPt, qfinal, &qinter3);
    }


    printf("qstart    qinter1    qinter2    qinter3    qfinal\n");
    for(int i=6; i<robotPt->nb_dof; i++)
    {
      printf("%f %f %f %f %f\n", qstart[i], qinter1[i], qinter2[i], qinter3[i], qfinal[i]);
    }

    //test all the intermediate configurations:
    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    g3d_draw_allwin_active();
    if(p3d_col_test()) // if collision
    {
      printf("The start configuration is in collision.\n");
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qinter1);
    g3d_draw_allwin_active();
    if(p3d_col_test()) // if collision
    {
      printf("The arm can not be folded without collision.\n");
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qinter2);
    g3d_draw_allwin_active();
    if(p3d_col_test()) // if collision
    {
      printf("qinter2 is colliding.\n");
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qinter3);
    g3d_draw_allwin_active();
    if(p3d_col_test()) // if collision
    {
      printf("qinter3 is colliding.\n");
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qfinal);
    g3d_draw_allwin_active();
    if(p3d_col_test()) // if collision
    {
      printf("qfinal is colliding.\n");
      goto END_GO_AND_GRASP;
    }


    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    p3d_copy_config_into(robotPt, qstart, &(robotPt->ROBOT_POS));
    p3d_activateCntrt(robotPt, cntrt_arm);
    g3d_draw_allwin_active();
    p3d_set_ROBOT_START(qstart);
    p3d_set_ROBOT_GOTO(qinter1);

    p3d_set_env_dmax(DMAX_FAR);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-hand", 1) ;
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, OBJECT_GROUP_NAME, 1) ;
    path_found= GP_FindPath();
    if(!path_found)
    {
      printf("The planner could not find a path to fold the arm.\n");
      so_far_so_good= false;
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qinter1);
    p3d_copy_config_into(robotPt, qinter1, &(robotPt->ROBOT_POS));
    g3d_draw_allwin_active();

    setAndActivateTwoJointsFixCntrt(robotPt, robotPt->objectJnt, robotPt->baseJnt);
    p3d_desactivateCntrt(robotPt, cntrt_arm);

    p3d_realloc_iksol(robotPt->cntrt_manager);

    p3d_set_ROBOT_START(qinter1);
    p3d_set_ROBOT_GOTO(qinter2);

    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-base", 1) ;
    path_found= GP_FindPath();
    if(!path_found)
    {
      printf("The planner could not find a path to go to the object.\n");
      so_far_so_good= false;
      goto END_GO_AND_GRASP;
    }
    desactivateTwoJointsFixCntrt(robotPt, robotPt->objectJnt, robotPt->baseJnt);
    p3d_desactivateCntrt(robotPt, cntrt_arm);

    p3d_set_and_update_this_robot_conf(robotPt, qinter2);
    p3d_copy_config_into(robotPt, qinter2, &(robotPt->ROBOT_POS));
    p3d_copy_config_into(robotPt, qinter3, &(robotPt->ROBOT_GOTO));
    p3d_activateCntrt(robotPt, cntrt_arm);
    g3d_draw_allwin_active();
    p3d_set_ROBOT_START(qinter2);
    p3d_set_ROBOT_GOTO(qinter3);

    p3d_set_env_dmax(DMAX_NEAR);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, OBJECT_GROUP_NAME, 1);
    gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND);
    path_found= GP_FindPath();
    if(!path_found)
    {
      printf("The planner could not find a path to reach the object with the arm.\n");
      so_far_so_good= false;
      goto END_GO_AND_GRASP;
    }
    p3d_desactivateCntrt(robotPt, cntrt_arm);

    p3d_set_and_update_this_robot_conf(robotPt, qinter3);
    p3d_copy_config_into(robotPt, qinter3, &(robotPt->ROBOT_POS));
    g3d_draw_allwin_active();
    p3d_set_ROBOT_START(qinter3);
    p3d_set_ROBOT_GOTO(qfinal);
    gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-hand", 1) ;

    path_found= GP_FindPath();
    if(!path_found)
    {
      printf("The planner could not find a path to close the robot's hand.\n");
      so_far_so_good= false;
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    p3d_copy_config_into(robotPt, qstart, &(robotPt->ROBOT_POS));
    p3d_activateCntrt(robotPt, cntrt_arm);
    g3d_draw_allwin_active();

    GP_ConcateneAllTrajectories(robotPt);
    robotPt->tcur= robotPt->t[0];
  }
  //if the robot does not need to move, we have to introduce one intermediate configurations:
  //    qstart =  (qbase0 ; qarm0 ; qhand0)
  // -> qinter1=  (qbase0 ; qarm1 ; qhand_inter=open)
  // -> qfinal =  (qbase0 ; qarm1 ; qhand1)
  else
  {
    // get arm final configuration:
    p3d_set_and_update_this_robot_conf(robotPt, qfinal);
    g3d_draw_allwin_active();
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qfinal);
    gpGet_arm_configuration(robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6);

    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    gpOpen_hand(robotPt, HAND);
    g3d_draw_allwin_active();
    gpSet_arm_configuration(robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qstart);
    p3d_get_robot_config_into(robotPt, &qinter1);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qinter1);
    if(p3d_col_test())
    {
      printf("The robot can not open its hand/gripper in its final arm and base configuration.\n");
      p3d_copy_config_into(robotPt, qfinal, &qinter1);
    }

    p3d_set_env_dmax(DMAX_FAR);
    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    g3d_draw_allwin_active();
    p3d_set_ROBOT_START(qstart);
    p3d_set_ROBOT_GOTO(qinter1);
    p3d_activateCntrt(robotPt, cntrt_arm);

    p3d_set_env_dmax(DMAX_NEAR);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-hand", 1) ;
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, OBJECT_GROUP_NAME, 1) ;
    gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND);
    path_found= GP_FindPath();
    if(!path_found)
    {
      printf("The planner could not find a path to reach the object with the arm.\n");
      so_far_so_good= false;
      goto END_GO_AND_GRASP;
    }
    p3d_set_and_update_this_robot_conf(robotPt, qinter1);
    g3d_draw_allwin_active();
    p3d_set_ROBOT_START(qinter1);
    p3d_set_ROBOT_GOTO(qfinal);
    gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-hand", 1) ;
    path_found= GP_FindPath();
    if(!path_found)
    {
      printf("The planner could not find a path to close the robot's hand.\n");
      so_far_so_good= false;
      goto END_GO_AND_GRASP;
    }

    p3d_set_and_update_this_robot_conf(robotPt, qstart);
    g3d_draw_allwin_active();
    GP_ConcateneAllTrajectories(robotPt);
  }

  PATH= GP_GetTrajectory(robotPt, robotPt->t[0], NB_CONFIGS);
  printf("path found: %d configs \n", NB_CONFIGS);

  gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND);
  p3d_copy_config_into(robotPt, qstart, &(robotPt->ROBOT_POS));

END_GO_AND_GRASP:
  //free all configs:
  p3d_destroy_config(robotPt, qstart);
  p3d_destroy_config(robotPt, qfinal);
  p3d_destroy_config(robotPt, qinter1);
  p3d_destroy_config(robotPt, qinter2);
  p3d_destroy_config(robotPt, qinter3);

  if(so_far_so_good)
  { printf("ALL IS DONE: SUCCESS.\n"); }
  else
  { printf("ALL IS DONE: THERE WAS SOMETHING WRONG.\n"); }

  return;
}

static void CB_arm_only_obj(FL_OBJECT *obj, long arg)
{
  GP_FindPathForArmOnly();
  return;
}


static void CB_test_obj(FL_OBJECT *obj, long arg)
{
// gpExport_for_coldman(p3d_get_robot_by_name("robot"));
gpExport_for_coldman( (p3d_rob*)(p3d_get_desc_curid(P3D_ROBOT)) );
 redraw();
 return;
  printf("Nothing happened...\n");
//   p3d_rob *robotPt= p3d_get_robot_by_name("robot");
//   print_config(robotPt, robotPt->ROBOT_GOTO);
p3d_matrix4 T, curT;
p3d_mat4Copy(p3d_mat4IDENTITY, T);

T[0][3]= 2.5;
T[1][3]= 400.0;
T[2][3]= 0.4;
// p3d_obj *obst= p3d_get_obst_by_name("object");
// p3d_obj *obst= p3d_get_obst_by_name("box1");
p3d_obj *obst= p3d_get_obst_by_name("Stones");
// set_obst_pos_by_mat(obst,T);
// set_thing_pos(P3D_OBSTACLE, obst, 3, 0, 2, 0, 0, 0);
//      p3d_mat4Copy(T, obst->pol[0]->poly->pos);

if(obst!=NULL)
{
#ifdef PQP
pqp_get_obj_pos(obst, curT);
 p3d_mat4Print(curT, "curT");
 pqp_set_obj_pos(obst, T, 1);
#endif
}


T[0][3]= 1.5;
T[1][3]= 0.0;
T[2][3]= 1.4;
obst= p3d_get_obst_by_name("object");
if(obst!=NULL)
{
#ifdef PQP
pqp_get_obj_pos(obst, curT);
 p3d_mat4Print(curT, "curT");
 pqp_set_obj_pos(obst, T, 1);
#endif
}


p3d_col_test_all();
// p3d_polyhedre *poly= obst->pol[0]->poly;
// poly->nb_faces= 50;
// for(int i=0; i<poly->nb_points; i++)
// {
//   poly->the_points[i][0]*= p3d_random(0.5, 2.5);
//   poly->the_points[i][1]*= p3d_random(0.5, 2.5);
//   poly->the_points[i][2]*= p3d_random(0.5, 2.5);
// }
//
return;
  unsigned int i;
  unsigned int n= 11;


  NB_POINTS= (unsigned int) p3d_random(500, NB_POINTS_MAX);
  NB_POINTS= 2*n;

  for(i=0; i<n; i++)
  {
    POINTS[2*i][0]= 2*cos(i*2*M_PI/((float) n));
    POINTS[2*i][1]= 2*sin(i*2*M_PI/((float) n));
    POINTS[2*i][2]= 2;

//     POINTS[2*i][0]= p3d_random(-1, 5);
//     POINTS[2*i][1]= p3d_random(-1, 5);
//     POINTS[2*i][2]= p3d_random(1, 2);

    POINTS[2*i+1][0]= 2*cos(i*2*M_PI/((float) n));
    POINTS[2*i+1][1]= 2*sin(i*2*M_PI/((float) n));
    POINTS[2*i+1][2]= -2;

//     POINTS[2*i+1][0]= p3d_random(-1, 5);
//     POINTS[2*i+1][1]= p3d_random(-1, 5);
//     POINTS[2*i+1][2]= p3d_random(-1, 1);
  }


//   for(i=0; i<NB_POINTS; i++)
//   {
//     POINTS[i][0]= p3d_random(-1, 5);
//     POINTS[i][1]= p3d_random(-1, 5);
//     POINTS[i][2]= p3d_random(-1, 5);
//   }
//   RAND_POINT[0]= p3d_random(-6, 6);
//   RAND_POINT[1]= p3d_random(-6, 6);
//   RAND_POINT[2]= p3d_random(-6, 6);
//
//   POINTS[0][0]= -3; POINTS[0][1]= -3;  POINTS[0][2]= -3;
//   POINTS[1][0]=  3; POINTS[1][1]= -3;  POINTS[1][2]= -3;
//   POINTS[2][0]=  3; POINTS[2][1]=  3;  POINTS[2][2]= -3;
//   POINTS[3][0]= -3; POINTS[3][1]=  3;  POINTS[3][2]= -3;
//
//   POINTS[4][0]= -3; POINTS[4][1]= -3;  POINTS[4][2]=  3;
//   POINTS[5][0]=  3; POINTS[5][1]= -3;  POINTS[5][2]=  3;
//   POINTS[6][0]=  3; POINTS[6][1]=  3;  POINTS[6][2]=  3;
//   POINTS[7][0]= -3; POINTS[7][1]=  3;  POINTS[7][2]=  3;

  NB_POINTS= 5000;
  double x;
  for(i=0; i<NB_POINTS; i++)
  {
    x= p3d_random(0, 6);
    if(x<1)
    {
      POINTS[i][0]= -5;
      POINTS[i][1]= p3d_random(-5, 5);
      POINTS[i][2]= p3d_random(-5, 5);
    }
    else if(x<2)
    {
      POINTS[i][0]= 5;
      POINTS[i][1]= p3d_random(-5, 5);
      POINTS[i][2]= p3d_random(-5, 5);
    }
    else if(x<3)
    {
      POINTS[i][0]= p3d_random(-5, 5);
      POINTS[i][1]= -5;
      POINTS[i][2]= p3d_random(-5, 5);
    }
    else if(x<4)
    {
      POINTS[i][0]= p3d_random(-5, 5);
      POINTS[i][1]= 5;
      POINTS[i][2]= p3d_random(-5, 5);
    }
    else if(x<5)
    {
      POINTS[i][0]= p3d_random(-5, 5);
      POINTS[i][1]= p3d_random(-5, 5);
      POINTS[i][2]= -5;
    }
    else
    {
      POINTS[i][0]= p3d_random(-5, 5);
      POINTS[i][1]= p3d_random(-5, 5);
      POINTS[i][2]= 5;
    }
//     POINTS[i][0]= p3d_random(-5, 5);
//     POINTS[i][1]= p3d_random(-5, 5);
//     POINTS[i][2]= p3d_random(-5, 5);
  }



  //chull= new gpConvexHull3D(POINTS, NB_POINTS);

  OBJECT= p3d_get_obst_by_name("object");

  if(OBJECT==NULL)
  {
    printf("There is no object with name \"%s\".\n","object");
    printf("Program must quit.\n");
    exit(0);
  }
  POLYHEDRON= OBJECT->pol[0]->poly;
  chull= new gpConvexHull3D(POLYHEDRON->the_points, POLYHEDRON->nb_points);
  chull->compute(false, 0.003, true);
  chull->print();
  printf("largest_ball_radius= %f\n", chull->largest_ball_radius());

  redraw();
}

static void CB_display_grasps_obj(FL_OBJECT *obj, long arg)
{
  display_grasps= !display_grasps;

  if(display_grasps)
  {  fl_set_button(BT_DISPLAY_GRASPS_OBJ, TRUE);  }
  else
  {  fl_set_button(BT_DISPLAY_GRASPS_OBJ, FALSE); }

  redraw();
}


static void CB_load_grasp_list_obj(FL_OBJECT *obj, long arg)
{
  LOAD_LIST= !LOAD_LIST;

  if(LOAD_LIST)
  {  fl_set_button(BT_LOAD_GRASP_LIST_OBJ, TRUE);  }
  else
  {  fl_set_button(BT_LOAD_GRASP_LIST_OBJ, FALSE); }

  redraw();
}


/////////////////////FUNCTIONS USED BY THE GENOM MODULE: /////////////////////////////
p3d_cntrt* GP_GetArmCntrt(p3d_rob *robotPt)
{
  int i;
  p3d_cntrt* cntrt_arm = NULL;

  if(robotPt==NULL)
  {
    printf("%s: %d: GP_GetArmCntrt(): input p3d_rob* is NULL.\n", __FILE__,__LINE__);
    return NULL;
  }

  for(i=0; i<robotPt->cntrt_manager->ncntrts; i++)
  {
    cntrt_arm = robotPt->cntrt_manager->cntrts[i];
    if (strcmp(cntrt_arm->namecntrt, "p3d_pa10_6_arm_ik")==0)
    {  break;  }
  }
  if(i==robotPt->cntrt_manager->ncntrts)
  {
    printf("%s: %d: GP_GetArmCntrt(): fatal error: arm_IK constraint does not exist.\n", __FILE__,__LINE__);
    return NULL;
  }

  return cntrt_arm;
}



int GP_Init(char *objectName)
{
  unsigned int i;

  if(!INIT_IS_DONE)
  {
    init_graspPlanning(objectName);

    // deactivate collisions for all robots except for the two of them needed by the grasp planner:
    for(i=0; i<(unsigned int) XYZ_ENV->nr; i++)
    {
      if(XYZ_ENV->robot[i]==ROBOT || XYZ_ENV->robot[i]==HAND_ROBOT)
      {   continue;    }
      else
      {  p3d_col_deactivate_robot(XYZ_ENV->robot[i]);  }
    }

    INIT_IS_DONE= true;
  }

  return 1;
}



//! Computes a list of grasps (for the hand only)
//! that will make the hand/gripper grasp the specified object.
//! \param objectName name of the object to be grasped by the robot
//! \return 1 in case of success, 0 otherwise
int GP_ComputeGraspList(char *objectName)
{
  GP_Init(objectName);

  printf("Collisions are deactivated for other robots.\n");

  gpGrasp_generation(HAND_ROBOT, OBJECT, 0, CMASS, IAXES, IAABB, HAND, HAND.translation_step, HAND.nb_directions, HAND.rotation_step, GRASPLIST);

  printf("Before collision filter: %d grasps.\n", GRASPLIST.size());
  gpGrasp_collision_filter(GRASPLIST, HAND_ROBOT, OBJECT, HAND);
  printf("After collision filter: %d grasps.\n", GRASPLIST.size());
  gpGrasp_stability_filter(GRASPLIST);
  printf("After stability filter: %d grasps.\n", GRASPLIST.size());

  gpGrasp_context_collision_filter(GRASPLIST, HAND_ROBOT, OBJECT, HAND);
  printf("For the current collision context: %d grasps.\n", GRASPLIST.size());
  p3d_col_deactivate_robot(HAND_ROBOT);

  redraw();


  if(GRASPLIST.empty())
  {
    printf("GP_ComputeGraspList(): No grasp was found.\n");
    return 0;
  }

  return 1;
}


//! Finds a suitable grasp configuration for the whole robot from the previously computed
//! grasp list.
//! This function is meant to be used with the genom module (maybe partly) dedicated to grasp planning.
//! \param needs_to_move will be filled with true if the configuration of the mobile base is different
//! in the computed robot configuration and in the current robot configuration, filled with false otherwise
//! \return the found configuration vector in case of success, NULL in case of failure
configPt GP_FindGraspConfig(bool &needs_to_move)
{
  if(!INIT_IS_DONE)
  {
    printf("GP_FindGraspConfig(): grasp planner needs to be initialized first.\n");
    return NULL;
  }

  if(GRASPLIST.empty())
  {
    printf("GP_FindGraspConfig(): The grasp list is empty.\n");
    return NULL;
  }

  unsigned int i, nb_iters_max;
  p3d_vector3 objectCenter;
  p3d_matrix4 objectPose;
  configPt qcurrent= NULL, qbase= NULL, qresult= NULL;

  //we first check if the robot can grasp the object from its current position:
  //find a configuration for the current robot base configuration:
  qcurrent = p3d_alloc_config(ROBOT);
  p3d_get_robot_config_into(ROBOT, &qcurrent);

  qresult= gpFind_grasp_from_base_configuration(ROBOT, OBJECT, GRASPLIST, ARM_TYPE, qcurrent, GRASP, HAND);

  p3d_destroy_config(ROBOT, qcurrent);

  if(qresult!=NULL)
  {
    GRASP.print();
    needs_to_move= false;

    // as the real Jido's gripper can only be completely opened or completely closed,
    // we set it to max opening:
//     if(HAND.type==GP_GRIPPER)
//     {
//       p3d_set_and_update_this_robot_conf(ROBOT, qresult);
//       gpOpen_hand(ROBOT, HAND);
//       p3d_get_robot_config_into(ROBOT, &qresult);
//       p3d_set_and_update_this_robot_conf(ROBOT, qcurrent);
//     }

    return qresult;
  }



// we must try to find a valid base configuration:
  needs_to_move= true;
  p3d_get_obj_pos(OBJECT, objectPose);
  objectCenter[0]= objectPose[0][3] + CMASS[0];
  objectCenter[1]= objectPose[1][3] + CMASS[1];
  objectCenter[2]= objectPose[2][3] + CMASS[2];

  nb_iters_max= 300;
  for(i=0; i<nb_iters_max; i++)
  {
    qbase= gpRandom_robot_base(ROBOT, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter, ARM_TYPE);
    if(qbase==NULL)
    {  break;  }

    qresult= NULL;
    qresult= gpFind_grasp_from_base_configuration(ROBOT, OBJECT, GRASPLIST, ARM_TYPE, qbase, GRASP, HAND);

    if(qresult!=NULL)
    {
      break;
    }
    p3d_destroy_config(ROBOT, qbase);
    qbase= NULL;
  }
  if(qbase!=NULL)
  {  p3d_destroy_config(ROBOT, qbase);  }

  if(i==nb_iters_max)
  {
    printf("GP_FindGraspConfig: No valid platform configuration was found.\n");
    return NULL;
  }

  // as the real Jido's gripper can only be completely opened or completely closed,
  // we set it to max opening:
//   if(HAND.type==GP_GRIPPER)
//   {
//     p3d_set_and_update_this_robot_conf(ROBOT, qresult);
//     gpOpen_hand(ROBOT, HAND);
//     p3d_get_robot_config_into(ROBOT, &qresult);
//     p3d_set_and_update_this_robot_conf(ROBOT, qcurrent);
//   }


  return qresult;
}

int GP_FindPathForArmOnly()
{
  bool needs_to_move, so_far_so_good= true;
  int result, path_found;
  std::vector<double> qhand;
  configPt qstart= NULL, qfinal= NULL, qfar= NULL;
  p3d_rob *robotPt= NULL;
  p3d_cntrt* cntrt_arm = NULL;
  robotPt= p3d_get_robot_by_name(GP_ROBOT_NAME);
  XYZ_ENV->cur_robot= robotPt;

  // initializes everything:
  GP_Init(GP_OBJECT_NAME_DEFAULT);

  redraw();

  cntrt_arm= GP_GetArmCntrt(robotPt);

  if(cntrt_arm==NULL)
  {
   printf("FATAL_ERROR : arm_IK constraint does not exist\n");
   return 0;
  }

  /* Deactivate the arm_IK constrint */
  p3d_desactivateCntrt(robotPt, cntrt_arm);

  //alloc all configs:
  qstart= p3d_alloc_config(robotPt);
  qfinal= p3d_alloc_config(robotPt);
  qfar= p3d_alloc_config(HAND_ROBOT);

  gpOpen_hand(robotPt, HAND);
  p3d_get_robot_config_into(robotPt, &qstart);
	p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qstart);

  p3d_set_and_update_this_robot_conf(robotPt, qstart);
  if(p3d_col_test())
  {
    printf("GP_FindPathForArmOnly(): Start configuration is colliding.\n");
    so_far_so_good= false;
    goto END_ARM_ONLY;
  }


  // computes the grasp list:
  if(!LOAD_LIST)
  {
    result= GP_ComputeGraspList(GP_OBJECT_NAME_DEFAULT);
    gpSave_grasp_list(GRASPLIST, "./graspPlanning/graspList_new.xml");
  }
  else // or loads it:
  {
    result= gpLoad_grasp_list("./graspPlanning/graspList.xml", GRASPLIST);
    if(result==0)
      {
        printf("Can not load a grasp list.\n");
        so_far_so_good= false;
        goto END_ARM_ONLY;
      }

    if(!GRASPLIST.empty())
    {
      if(GRASPLIST.front().hand_type!=HAND.type)
      {
        printf("The loaded grasp list does not correspond to the current hand type.\n");
        so_far_so_good= false;
        goto END_ARM_ONLY;
      }
    }
  }

  if(GRASPLIST.empty())
  {
    printf("Could not compute any grasp.\n");
    so_far_so_good= false;
    goto END_ARM_ONLY;
  }

  // move away the hand robot:
  qfar= p3d_alloc_config(HAND_ROBOT);
  qfar[7]= -100;
  qfar[8]= -1; //to put the hand far under the floor
  p3d_set_and_update_this_robot_conf(HAND_ROBOT, qfar);

  qfinal= GP_FindGraspConfig(needs_to_move);

  if(qfinal!=NULL)
  {
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qfinal);
     p3d_set_and_update_this_robot_conf(robotPt, qfinal);
     gpOpen_hand(robotPt, HAND);
     p3d_get_robot_config_into(robotPt, &qfinal);
     if(p3d_col_test())
     {
      printf("The robot can not open its hand in final configuration without collision.\n");
      so_far_so_good= false;
      goto END_ARM_ONLY;
     }
     printf("Grasp configuration list was successfully computed.\n");
  }
  else
  {
    printf("No grasp configuration was found.\n");
    so_far_so_good= false;
    goto END_ARM_ONLY;
  }

  if(needs_to_move)
  {
    printf("The robot can not reach the object from its current position. It needs to move.\n");
    so_far_so_good= false;
    goto END_ARM_ONLY;
  }

  p3d_set_and_update_this_robot_conf(robotPt, qstart);
  p3d_copy_config_into(robotPt, qstart, &(robotPt->ROBOT_POS));
  p3d_copy_config_into(robotPt, qfinal, &(robotPt->ROBOT_GOTO));
  p3d_activateCntrt(robotPt, cntrt_arm);
  g3d_draw_allwin_active();
  XYZ_ENV->cur_robot= robotPt;
  p3d_set_ROBOT_START(qstart);
  p3d_set_ROBOT_GOTO(qfinal);


  p3d_set_env_dmax(DMAX_NEAR);
  p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
  p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, OBJECT_GROUP_NAME, 1);
  p3d_activateCntrt(robotPt, cntrt_arm);
  path_found= GP_FindPath();
  if(!path_found)
  {
    printf("The planner could not find a valid path for the arm.\n");
    so_far_so_good= false;
    goto END_ARM_ONLY;
  }


END_ARM_ONLY:
  p3d_destroy_config(robotPt, qstart);
  p3d_destroy_config(robotPt, qfinal);
  p3d_destroy_config(HAND_ROBOT, qfar);

  if(so_far_so_good)
  {
    printf("ALL IS DONE: SUCCESS.\n");
    return 1;
  }
  else
  {
    printf("ALL IS DONE: THERE WAS SOMETHING WRONG.\n");
    return 0;
  }

}


//! Creates and fills an array of configPt with the configuration steps of the given trajectory.
//! \param robotPt pointer to the robot
//! \param traj pointer to the trajectory (that must be compatible with the robot)
//! \param nb_configs will be filled with the size of the array
//! \return the configuration array
configPt* GP_GetTrajectory(p3d_rob *robotPt, p3d_traj *traj, int &nb_configs)
{
  nb_configs= 0;
  if(robotPt==NULL)
  {
    PrintInfo(("GP_GetTrajectory: robot is NULL.\n"));
    return NULL;
  }
  if(traj==NULL)
  {
    PrintInfo(("GP_GetTrajectory: traj is NULL.\n"));
    return NULL;
  }


  bool traj_found_in_robot= false;
  double umax; // parameters along the local path
  int i, *ikSol= NULL;
  pp3d_localpath localpathPt;
  configPt *configs= NULL;

  for(i= 0; i<robotPt->nt; i++)
  {
    if(robotPt->t[i]==traj)
    {
      traj_found_in_robot= true;
      break;
    }
  }
  if(traj_found_in_robot==false)
  {
    PrintInfo(("GP_GetTrajectory: traj may not belong to the robot.\n"));
  }


  localpathPt = traj->courbePt;
  //distances = MY_ALLOC(double, njnt+1);

  i= 0;
  while(localpathPt!=NULL)
  {
    (nb_configs)++;
    localpathPt= localpathPt->next_lp;
  }
  (nb_configs)++;
  configs= (configPt *) malloc(nb_configs*sizeof(configPt));

  localpathPt = traj->courbePt;
  i= 0;
  while(localpathPt != NULL)
  {
    umax= localpathPt->range_param;

    if(i==0)
    {
      configs[i]= localpathPt->config_at_param(robotPt, localpathPt, 0);
      if(!ikSol || !p3d_compare_iksol(robotPt->cntrt_manager, localpathPt->ikSol, ikSol))
      {
	p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &ikSol);
	if(p3d_get_ik_choice() != IK_NORMAL)
        {  p3d_print_iksol(robotPt->cntrt_manager, localpathPt->ikSol);  }
      }
      p3d_set_and_update_this_robot_conf_multisol(robotPt, configs[i], NULL, 0, localpathPt->ikSol);
      i++;
    }

    configs[i] = localpathPt->config_at_param(robotPt, localpathPt, umax);
    if(!ikSol || !p3d_compare_iksol(robotPt->cntrt_manager, localpathPt->ikSol, ikSol))
    {
      p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &ikSol);
      if(p3d_get_ik_choice() != IK_NORMAL)
      {   p3d_print_iksol(robotPt->cntrt_manager, localpathPt->ikSol);  }
    }
    p3d_set_and_update_this_robot_conf_multisol(robotPt, configs[i], NULL, 0, localpathPt->ikSol);
    i++;

    localpathPt = localpathPt->next_lp;
  }


  return configs;
}

//! Creates and fills a array of configPt with the configuration steps of the all the trajectories contained
//! in the trajectory array of a robot.
//! \param robotPt pointer to the robot
//! \param nb_configs will be filled with the size of the array
//! \return the configuration array
configPt* GP_GetAllTrajectoriesAsOne(p3d_rob *robotPt, int &nb_configs)
{
  nb_configs= 0;
  if(robotPt==NULL)
  {
    PrintInfo(("GP_GetTrajectory: robot is NULL.\n"));
    return NULL;
  }

  int i, j, n;
  configPt* configs= NULL, *result= NULL;
  std::list<configPt> cfg_list;
  std::list<configPt>::iterator iter;

  for(i=0; i<robotPt->nt; i++)
  {
    n= 0;
    configs= GP_GetTrajectory(robotPt, robotPt->t[i], n);
    for(j=0; j<n; j++)
    {  cfg_list.push_back(configs[j]);   }
    free(configs);
  }

  nb_configs= cfg_list.size();
  result= (configPt *) malloc(nb_configs*sizeof(configPt));

  i= 0;
  for(iter=cfg_list.begin(); iter!=cfg_list.end(); iter++)
  {
    result[i]= (*iter);
    i++;
  }

  return result;
}

//! Concatenes all the current trajectories of the robot into the first one.
//! NB: only the first trajectory will remain (and grown up); the others are destroyed.
//! \param robotPt pointer to the robot
//! \return 1 in case of success, 0 otherwise
int GP_ConcateneAllTrajectories(p3d_rob *robotPt)
{
  if(robotPt==NULL)
  {
    PrintInfo(("GP_ConcateneAllTrajectories: robot is NULL.\n"));
    return 0;
  }
  if(robotPt->nt==0)
  {
    PrintInfo(("GP_ConcateneAllTrajectories: the robot has no trajectory.\n"));
    return 0;
  }

  int i;
  pp3d_localpath localpathPt, end;

  for(i=0; i<robotPt->nt-1; i++)
  {
    localpathPt = robotPt->t[i]->courbePt;
    while(localpathPt!=NULL)
    {
      end= localpathPt;
      localpathPt = localpathPt->next_lp;
    }
    end->next_lp= robotPt->t[i+1]->courbePt;
    robotPt->t[i+1]->courbePt->prev_lp= end;
  }

  for(i=1; i<robotPt->nt; i++)
  {  free(robotPt->t[i]);  }
  robotPt->nt= 1;

  robotPt->tcur= robotPt->t[0];
  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));

  return 1;
}

//! Computes a path for the arm and hand of the robot to drive it from ROBOT_POS to ROBOT_GOTO configurations.
//! that will make the robot grasp the specified object.
//! \param platform_motion indicates if the platform motion is allowed or not; if not, the platform DOFs will be
//! blocked before calling the motion planner
//! \param arm_motion indicates if the arm motion is allowed or not; if not, the arm DOFs will be
//! blocked before calling the motion planner
//! \param hand_motion indicates if the hand motion is allowed or not; if not, the hand DOFs will be
//! blocked before calling the motion planner
//! \return 1 in case of success, 0 otherwise
int GP_FindPath()
{
  if(!INIT_IS_DONE)
  {
    printf("GP_FindPath(): grasp planner needs to be initialized first.\n");
    return 0;
  }

  int i, result;

  // deactivate collisions for all other robots:
  for(i=0; i<XYZ_ENV->nr; i++)
  {
    if(XYZ_ENV->robot[i]==ROBOT)
    {   continue;    }
    else
    {  p3d_col_deactivate_robot(XYZ_ENV->robot[i]);  }
  }

  printf("Collision are desactivated for other robots\n");

  ENV.setBool(Env::biDir,true);
  ENV.setInt(Env::NbTry, 100000);
  ENV.setInt(Env::MaxExpandNodeFail, 30000);
  ENV.setInt(Env::maxNodeCompco, 100000);
  ENV.setExpansionMethod(Env::Connect);

//   print_config(ROBOT, ROBOT->ROBOT_POS);
//   print_config(ROBOT, ROBOT->ROBOT_GOTO);

  if(p3d_equal_config(ROBOT, ROBOT->ROBOT_POS, ROBOT->ROBOT_GOTO))
  {
    printf("GP_FindPath(): Start and goal configurations are the same.\n");
    return 1;
  }

  p3d_set_and_update_this_robot_conf(ROBOT, ROBOT->ROBOT_POS);
  result= p3d_specific_search("out.txt");

  // optimizes the trajectory:
  CB_start_optim_obj(NULL, 0);

  // reactivate collisions for all other robots:
  for(i=0; i<XYZ_ENV->nr; i++)
  {
    if(XYZ_ENV->robot[i]==HAND_ROBOT || XYZ_ENV->robot[i]==ROBOT)
    {   continue;    }
    else
    {  p3d_col_activate_robot(XYZ_ENV->robot[i]);  }
  }
  printf("Collision are re-activated for other robots\n");

  p3d_SetTemperatureParam(1.0);

  deleteAllGraphs();

  return result;
}

void GP_Reset()
{
  g3d_win *win= NULL;

  for(int i=0; i<NB_CONFIGS; i++)
  { p3d_destroy_config(ROBOT, PATH[i]);  }
  free(PATH);
  PATH= NULL;
  NB_CONFIGS= 0;

  if(ROBOT!=NULL)
  {
    while(ROBOT->nt!=0)
    {   p3d_destroy_traj(ROBOT, ROBOT->t[0]);  }
    FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  }


  ROBOT= NULL;
  HAND_ROBOT= NULL;
  OBJECT= NULL;
  POLYHEDRON= NULL;
  GRASPLIST.clear();

  INIT_IS_DONE= false;

  deleteAllGraphs();

  //reinit all the initial collision context:
  #ifdef PQP
  pqp_create_collision_pairs();
  #endif

  win= g3d_get_cur_win();
  win->fct_draw2= NULL;
  win->fct_key1 = NULL;
  win->fct_key2 = NULL;
}


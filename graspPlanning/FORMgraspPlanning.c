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



static p3d_rob *ROBOT= NULL; //le robot
static p3d_rob *HAND_ROBOT= NULL; //le robot
//static p3d_polyhedre *OBJECT= NULL; //l'objet à saisir
static p3d_obj *OBJECT= NULL; //l'objet à saisir
static p3d_polyhedre *POLYHEDRON= NULL; //le polyèdre associé à l'objet
static gpHand_properties HAND; 
//static msMesh *MESH= NULL;     //maillage utilisé pour la segmentation
static p3d_vector3 CMASS; //centre de masse de l'objet
static p3d_matrix3 IAXES; //axes d'inertie de l'objet
static double IAABB[6]; //bounding box alignée sur les axes d'inertie de l'objet
static std::list<gpGrasp> GRASPLIST;
static gpGrasp GRASP;   // la prise courante
//static gpGrasp *GRASP= NULL;  
//p3d_matrix4 *SAMPLES= NULL;
//int NB_SAMPLES=0;

//static p3d_matrix4 FRAME;
//static p3d_matrix4 WRISTFRAME;
//static p3d_matrix4 TH01, TH02, TH03, TH04, TH05, TH06;
//static int COUNT= 0;
//static Gb_6rParameters ARM_PARAMETERS;

static bool firstTime_grasp_planner= true;
//static bool firstTime_preACD= true;
//static bool firstTime_progressive_ACD= true;
static bool firstTime_test= true;
static unsigned int CNT= 0;


void draw_grasp_planner();
void draw_test();
void key1();
void key2();


extern configPt GP_FindConfigToGraspV2(char* objectName);
extern void GP_Reset();
extern void Gp_ResetGraph() ;
extern configPt* GP_FindPathToGrasp( int *nb_configs);

double COLOR_TAB[15][3]= {  {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0} , {1.0, 0.5, 0.5}, {0.5, 1.0, 0.5}, {0.5, 0.5, 1.0}, {1.0, 0.25, 0.5}, {1.0, 0.5, 0.25}, {0.25, 1.0, 0.5}, {0.5, 1.0, 0.25}, {0.25, 0.5, 1.0}, {0.5, 0.25, 1.0}  };



/* --------- FORM VARIABLES ------- */
FL_FORM  * GRASP_PLANNING_FORM = NULL;
static FL_OBJECT * MOTIONGROUP;
static FL_OBJECT * BT_GRASP_OBJ;
static FL_OBJECT * BT_PREACD_OBJ;
static FL_OBJECT * BT_CAMERA_OBJ;
static FL_OBJECT * BT_RESET_OBJ;
static FL_OBJECT * BT_TEST_OBJ;
/* ------------------------------------------ */


/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_grasp_planning_group(void); 
static void CB_grasp_planner_obj(FL_OBJECT *obj, long arg);
static void CB_preacd_obj(FL_OBJECT *obj, long arg);
static void CB_camera_obj(FL_OBJECT *obj, long arg);
static void CB_reset_obj(FL_OBJECT *obj, long arg);
static void CB_test_obj(FL_OBJECT *obj, long arg);
/* ------------------------------------------ */


/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_grasp_planning_form(void)
{
  GRASP_PLANNING_FORM = fl_bgn_form(FL_UP_BOX, 150, 400);

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

  obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 80, 140, 260, "Grasp planning");

  MOTIONGROUP = fl_bgn_group();

  x= 15;
  y= 90;
  w= 120;
  h= 40;
  dy= h + 10;
  BT_GRASP_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Grasp planner");
  BT_PREACD_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + dy, w, h, "PreACD");
  BT_CAMERA_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "Camera");
  BT_RESET_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + 3*dy, w, h, "Reset");
  BT_TEST_OBJ  = fl_add_button(FL_NORMAL_BUTTON, x, y + 4*dy, w, h, "Test");

  fl_set_call_back(BT_GRASP_OBJ, CB_grasp_planner_obj, 1);
  fl_set_call_back(BT_PREACD_OBJ, CB_preacd_obj, 2);
  fl_set_call_back(BT_CAMERA_OBJ, CB_camera_obj, 3);
  fl_set_call_back(BT_RESET_OBJ, CB_reset_obj, 1);
  fl_set_call_back(BT_TEST_OBJ, CB_test_obj, 1);

  fl_end_group();
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
  //p3d_matrix4 Tend_eff;
  //gpForward_geometric_model_PA10(ROBOT, Tend_eff);

  //p3d_vector3 p1, p2;
  p3d_matrix4 Twrist, pose;
  float mat[16];

  GRASP.draw(0.03);

  p3d_draw_robot_joints(ROBOT, 0.2);
  p3d_draw_robot_joints(HAND_ROBOT, 0.05);
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


//   for(std::list<gpGrasp>::iterator iter= GRASPLIST.begin(); iter!=GRASPLIST.end(); iter++)
//   {
//     (*iter).draw(0.03);
//   } 

/*
double q[4];
printf("%f %f %f \n", randomPoint[0], randomPoint[1], randomPoint[2]);
if(gpSAHfinger_inverse_kinematics(Twrist, HAND, randomPoint, q, 4))
{
   gpSet_SAHfinger_joint_angles(HAND_ROBOT, HAND, q, 4);
   g3d_set_color_mat(Green, NULL);
glPushMatrix();
  glTranslatef(randomPoint[0], randomPoint[1], randomPoint[2]);
  draw_solid_sphere(0.01, 15);
glPopMatrix();
}
else
{
   g3d_set_color_mat(Red, NULL);
glPushMatrix();
  glTranslatef(randomPoint[0], randomPoint[1], randomPoint[2]);
  draw_solid_sphere(0.01, 15);
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
  if(firstTime_grasp_planner)
  {
    init_graspPlanning(GP_OBJECT_NAME_DEFAULT);
    firstTime_grasp_planner= false;
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
  p3d_set_and_update_this_robot_conf(HAND_ROBOT, qhand);
  p3d_destroy_config(HAND_ROBOT, qhand);
  qhand= NULL;
  gpSet_grasp_config(HAND_ROBOT, HAND, GRASP);
  if(qhand!=NULL)
  {  p3d_destroy_config(HAND_ROBOT, qhand);  }

  //find a configuration for the whole robot (mobile base + arm):
  configPt qend= NULL;
  if(ROBOT!=NULL)
  {
    for(i= 0; i<150; i++)
    {
      qgrasp= gpRandom_robot_base(ROBOT, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter);
      if(qgrasp==NULL)
      {  break;  }
    
      qend= NULL;
      qend= gpFind_grasp_from_base_configuration(ROBOT, OBJECT, GRASPLIST, GP_PA10, qgrasp, GRASP, HAND);
      
      if(qend!=NULL)
      {   
        p3d_set_and_update_this_robot_conf(ROBOT, qend);
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


  win= g3d_get_cur_win();
  win->fct_draw2= &(draw_grasp_planner);
  win->x= objectPose[0][3];   win->y= objectPose[1][3];   win->z= objectPose[2][3];
  g3d_draw_allwin();
  g3d_draw_allwin_active();

  return;
}



static void CB_preacd_obj(FL_OBJECT *obj, long arg)
{

}



//! Centers the camera on the object position.
static void CB_camera_obj(FL_OBJECT *obj, long arg)
{

  static int firstTime= true;
  if(firstTime)
  {
    firstTime= false;
    init_graspPlanning(GP_OBJECT_NAME_DEFAULT);
  }

  p3d_matrix4 Twrist, T2, T3, base_frame, inv_base_frame;

//  pqp_print_collision_pairs();

  gpGet_wrist_frame(HAND_ROBOT, Twrist);
  gpGet_arm_base_frame(ROBOT, base_frame);

  p3d_matInvertXform(base_frame, inv_base_frame);

  p3d_mat4Mult(inv_base_frame, Twrist, T2); 

  p3d_mat4Mult(T2, HAND.Thand_wrist, T3);
  

  //gpGrasps_from_grasp_frame_SAHand(HAND_ROBOT, OBJECT, 0, Twrist, HAND, GRASPLIST);
  configPt q= NULL;
  q= p3d_alloc_config(ROBOT);
  int result;
  result= gpInverse_geometric_model_PA10(ROBOT, T3, q);
  printf("MGI: %d\n",result);
  if(result==1)
  {
     p3d_set_and_update_this_robot_conf(ROBOT, q);
  }    
  p3d_destroy_config(ROBOT, q);

  q= p3d_alloc_config(HAND_ROBOT);
  result= gpInverse_geometric_model_freeflying_hand(HAND_ROBOT, p3d_mat4IDENTITY, p3d_mat4IDENTITY, HAND, q);
  printf("MGI: %d\n",result);
  if(result==1)
  {
     p3d_set_and_update_this_robot_conf(HAND_ROBOT, q);
  }    
  p3d_destroy_config(HAND_ROBOT, q);


//p3d_obj *o1= p3d_get_body_by_name("SAHandRight_robot.hand.finger2.fingertip");

//printf("col_pair= %d\n", pqp_is_collision_pair_activated(o1, OBJECT));
  G3D_Window *win = g3d_get_cur_win();
  win->fct_draw2= &(draw_grasp_planner);
  win->fct_key1= &(key1);
  win->fct_key2= &(key2);

  //win->x= POLYHEDRON->pos[0][3];   win->y= POLYHEDRON->pos[1][3];   win->z= POLYHEDRON->pos[2][3];

  g3d_draw_allwin();
}


static void CB_reset_obj(FL_OBJECT *obj, long arg)
{
  ROBOT= NULL;
  OBJECT= NULL;
  POLYHEDRON= NULL;
  GRASPLIST.clear();

  firstTime_grasp_planner= true;
  firstTime_test= true;


  G3D_Window *win = g3d_get_cur_win();
  g3d_draw_allwin();
  win->fct_draw2= NULL;
  win->fct_key1 = NULL;
  win->fct_key2 = NULL;

  printf("GraspPlanning static global values have been reset.\n");
}

static void CB_test_obj(FL_OBJECT *obj, long arg)
{
  configPt q= NULL, qhand= NULL;
  static unsigned int i, count= 1;
  p3d_matrix4 objectPose, objectPose_inv, Twrist, Twrist_grasp_frame_inv;

  std::list<gpGrasp>::iterator pgrasp;

  if(firstTime_test)
  {
   init_graspPlanning(GP_OBJECT_NAME_DEFAULT);
   firstTime_test= false;
   //gpGrasp_generation(HAND_ROBOT, OBJECT, 0, CMASS, IAXES, IAABB, HANDC, 0.015, 8, 2*M_PI/3.0, GRASPLIST);
   gpGrasp_generation(HAND_ROBOT, OBJECT, 0, CMASS, IAXES, IAABB, HAND, 0.09, 8, 2*M_PI/2.0, GRASPLIST);

   gpGrasp_collision_filter(GRASPLIST, HAND_ROBOT, OBJECT, HAND);
   printf("apres collision %d grasps\n", GRASPLIST.size());
   gpGrasp_stability_filter(GRASPLIST);
   printf("apres stabilite %d grasps\n", GRASPLIST.size());

    gpGrasp_context_collision_filter(GRASPLIST, HAND_ROBOT, OBJECT, HAND);
   printf("apres collision contexte %d grasps\n", GRASPLIST.size());
   p3d_col_deactivate_robot(HAND_ROBOT);
  }

  i= 0;
  for(pgrasp=GRASPLIST.begin(); pgrasp!=GRASPLIST.end(); pgrasp++)
  {
    GRASP= (*pgrasp);
    i++;
    if(i>=count)
     break;
  }
  count++;
  if(count>GRASPLIST.size())
  {  count= 1;  }
  GRASP.print();

  if(!GRASPLIST.empty())
  {
    p3d_get_obj_pos(OBJECT, objectPose);
    p3d_matInvertXform(objectPose, objectPose_inv);

    qhand= p3d_alloc_config(HAND_ROBOT);

    p3d_matInvertXform(HAND.Tgrasp_frame_hand, Twrist_grasp_frame_inv);
    p3d_mat4Mult(GRASP.frame, Twrist_grasp_frame_inv, Twrist);


    gpInverse_geometric_model_freeflying_hand(HAND_ROBOT, objectPose, GRASP.frame, HAND, qhand);
    p3d_set_and_update_this_robot_conf(HAND_ROBOT, qhand);
    p3d_destroy_config(HAND_ROBOT, qhand);
    gpSet_grasp_config(HAND_ROBOT, HAND, GRASP);

    if(ROBOT!=NULL)
    {
      q= p3d_get_robot_config(ROBOT);
      configPt qend= NULL;
      qend= gpFind_grasp_from_base_configuration(ROBOT, OBJECT, GRASPLIST, GP_PA10, ROBOT->ROBOT_POS, GRASP, HAND);
    
      if(qend!=NULL)
      {   
        p3d_set_and_update_this_robot_conf(ROBOT, qend);
        print_config(ROBOT, qend);
        p3d_destroy_config(ROBOT, qend);
      }
      p3d_destroy_config(ROBOT, q);
    }
  }

  G3D_Window *win = g3d_get_cur_win();
  g3d_draw_allwin();

  win->fct_draw2= &(draw_grasp_planner);
  win->fct_key1 = &(key1);

  win->x= POLYHEDRON->pos[0][3];   win->y= POLYHEDRON->pos[1][3];   win->z= POLYHEDRON->pos[2][3];

  g3d_draw_allwin();
  g3d_draw_allwin_active();
}





/////////////////////FUNCTIONS USED BY THE  GENOM MODULE: /////////////////////////////
configPt GP_FindConfigToGrasp(char *objectName)
{
  int i;
  int gripper_jnt_index, finger_dof;
  p3d_rob *robotPt= ROBOT;
  configPt qcurrent= NULL, qend= NULL;


  if(firstTime_grasp_planner)
  {
    init_graspPlanning(GP_OBJECT_NAME_DEFAULT);
    firstTime_grasp_planner= false;

    // deactivate collisions for all robots except for the two of them needed by the grasp planner:
    for(i=0; i<XYZ_ENV->nr; i++) 
    {
      if(XYZ_ENV->robot[i]==robotPt || XYZ_ENV->robot[i]==HAND_ROBOT)
      {   continue;    }
      else
      {  p3d_col_deactivate_robot(XYZ_ENV->robot[i]);  }
    }
    printf("Collision are deactivated for other robots.\n");

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
    return NULL;
  }

  //find a configuration for the current robot base configuration:
  qcurrent = p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &qcurrent);

  qend= gpFind_grasp_from_base_configuration(robotPt, OBJECT, GRASPLIST, GP_PA10, qcurrent, GRASP, HAND);
      
  if(qend!=NULL)
  {   
    p3d_set_and_update_this_robot_conf(robotPt, qend);
    GRASP.print();
    //print_config(robotPt, qend);
    g3d_set_draw_coll(TRUE);
    g3d_draw_allwin_active();
  }
  else
  {
    printf("There is no valid grasp for this configuration of the mobile base.\n");
  }

  p3d_destroy_config(robotPt, qcurrent);
  qcurrent= NULL;

 
  // reactivate collisions for all robots except for HAND_ROBOT:
  for(i=0; i<XYZ_ENV->nr; i++) 
  {
    if(XYZ_ENV->robot[i]==HAND_ROBOT)
    {   continue;    }
    else
    {  p3d_col_activate_robot(XYZ_ENV->robot[i]);  }
  }
  printf("Collision are reactivated for all robots.\n");


  gripper_jnt_index= get_robot_jnt_index_by_name(robotPt, GP_FINGERJOINT);
  finger_dof = robotPt->joints[gripper_jnt_index]->index_dof;
  robotPt->ROBOT_POS[finger_dof]= HAND.max_opening_jnt_value;
  robotPt->ROBOT_POS[finger_dof+1]= -1.0*HAND.max_opening_jnt_value;

  return qend;
}

configPt* GP_FindPathToGrasp(int *nb_configs)
{
  int i, ir, it;
  p3d_rob *robotPt= NULL;
  configPt *configs= NULL;

  robotPt= p3d_get_robot_by_name(GP_ROBOT_NAME);

  // deactivate collisions for all other robots:
  for(i=0; i<XYZ_ENV->nr; i++) 
  {
    if(XYZ_ENV->robot[i]==robotPt)
    {   continue;    }
    else
    {  p3d_col_deactivate_robot(XYZ_ENV->robot[i]);  }
  }

  printf("Collision are desactivated for other robots\n");

  p3d_specific_search("out.txt");



  //////////////////////////////////////////////////////////////
  // OPTIMISATION
  ///////////////////////////////////////////////////////////
  p3d_traj *traj = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
  ir = p3d_get_desc_curnum(P3D_ROBOT);
  int  ntest=0, nb_optim=0;
  double gain,gaintot=1.;

  if(p3d_get_desc_number(P3D_TRAJ) > it)
  {
     if(!traj)
     {
       printf("Optimize : ERREUR : no current traj\n");
       return NULL;
     }
 
    if(traj->nlp <= 1)
    {
       printf("Optimization not possible: current trajectory contains one or zero local path\n");
       return NULL;
    }

    ChronoOn();
    while(fabs(gaintot-1.0) < EPS6 && nb_optim < 30)
    {
      for(i=1;i<=p3d_get_NB_OPTIM();i++)
      {
         if(p3d_optim_traj(traj,&gain, &ntest))
         {
	  gaintot = gaintot*(1.- gain);
	 // position the robot at the beginning of the optimized trajectory 
	  position_robot_at_beginning(ir, traj);
        }
      }
      nb_optim++;
    }

    if (fabs(gaintot-1.0) > EPS6)
    {
      // the curve has been optimized 
      p3d_simplify_traj(traj);
    }
    gaintot = (1.-gaintot)*100.;
    printf("La courbe a ete optimisee de %f%%\n",gaintot);
    printf("nb collision test : %d\n", ntest);
    ChronoPrint("");

    // When retrieving statistics;
   //  Commit Jim; date: 01/10/2008
    double tu = 0.0, ts = 0.0;
    ChronoTimes(&tu, &ts);
    if(getStatStatus())
    {
      XYZ_GRAPH->stat->postTime += tu;
    }

    ChronoOff();
    position_robot_at_beginning(ir, traj);
  }
  //////////////////////////////////////////////////////////////
  // FIN  OPTIMISATION
  ///////////////////////////////////////////////////////////
  for(i=0; i<XYZ_ENV->nr; i++) 
  {
    if(XYZ_ENV->robot[i]==HAND_ROBOT)
    {   continue;    }
    else
    {  p3d_col_deactivate_robot(XYZ_ENV->robot[i]);  }
  }
  printf("Collision are re-activated for other robots\n");

  *nb_configs= 0;

  //robot->ROBOT_INTPOS = p3d_copy_config(robot, robot->ROBOT_POS);

  double umax; // parameters along the local path 
  int *ikSol= NULL;
  pp3d_localpath localpathPt;
  i= 0;


  if(robotPt->tcur==NULL)
  {
    PrintInfo(("GP_read_tcur_rob: no current trajectory\n"));
    return NULL;
  }
  localpathPt = robotPt->tcur->courbePt;
  //distances = MY_ALLOC(double, njnt+1);


  while(localpathPt!=NULL)
  {
    (*nb_configs)++;
    localpathPt= localpathPt->next_lp;
  }
  (*nb_configs)++;
  printf("CB_test GraspPlanning -> nb_configs= %d\n", (*nb_configs));
  configs= (configPt *) malloc((*nb_configs)*sizeof(configPt));
  //for(i=0; i<(*nb_configs); i++)
  // {  configs[i]= p3d_alloc_config(robotPt);   }

  localpathPt = robotPt->tcur->courbePt;
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

  //CB_showtraj_obj(NULL,1);

  return configs;
}


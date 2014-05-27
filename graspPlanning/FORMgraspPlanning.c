/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include <time.h>
#include <list>
#include <string>
#include <algorithm>
#include "GraspPlanning-pkg.h"
#include "Collision-pkg.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../localpath/proto/p3d_multiLocalPath_proto.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include "Manipulation.h"


static char ObjectName[]= "PaperDog";
static char RobotName[]= "JIDO_ROBOT";

//! needed by the user interface
static bool DISPLAY_GRASPS= false;
static bool INIT_IS_DONE= false;
static p3d_rob *HAND_ROBOT= NULL; // pointer to the hand robot
static p3d_rob *OBJECT= NULL; // pointer to the object to grasp
static gpHand_properties HAND_PROP;  // information about the used hand
static std::list<gpGrasp> GRASP_LIST;
static std::list<class gpDoubleGrasp> DOUBLE_GRASP_LIST;
static gpGrasp GRASP;   // the current grasp
static gpDoubleGrasp DOUBLEGRASP;
int initialize_grasp_planner();
void draw_grasp_planner();

gpConvexHull chull;
gpConvexHull3D chull3d;
std::vector<gpVector3D> POINTS;
p3d_vector3 E1, E2, E3;
p3d_vector3 A1, B1, C1, D1, A2, B2, C2, D2;
p3d_matrix4 FRAME;

#include "ManipulationPlanner.hpp"
static ManipulationPlanner *manipulation= NULL;


void dynamic_grasping();
void contact_points();


/* --------- FORM VARIABLES ------- */
FL_FORM  * GRASP_PLANNING_FORM = NULL;
static FL_OBJECT *MOTIONGROUP;
static FL_OBJECT *BT_1_OBJ;
static FL_OBJECT *BT_2_OBJ;
static FL_OBJECT *BT_3_OBJ;
static FL_OBJECT *BT_4_OBJ;
static FL_OBJECT *BT_5_OBJ;
static FL_OBJECT *BT_6_OBJ;
static FL_OBJECT *BT_7_OBJ;
static FL_OBJECT *BT_8_OBJ;
static FL_OBJECT *BT_9_OBJ;
static FL_OBJECT *BT_10_OBJ;
/* ------------------------------------------ */


/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_grasp_planning_group ( void );
static void CB_SAHandRight(FL_OBJECT *obj, long arg);
static void CB_double_grasp(FL_OBJECT *obj, long arg);
static void CB_test(FL_OBJECT *obj, long arg);
static void CB_display_grasps(FL_OBJECT *obj, long arg);
static void CB_select_object(FL_OBJECT *obj, long arg);
static void CB_select_hand_robot(FL_OBJECT *obj, long arg);
static void CB_browse_grasps(FL_OBJECT *obj, long arg);
static void CB_delete_grasp(FL_OBJECT *obj, long arg);
static void CB_add_grasp(FL_OBJECT *obj, long arg);
static void CB_save_grasps(FL_OBJECT *obj, long arg);
/* ------------------------------------------ */


/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_grasp_planning_form(void)
{
  GRASP_PLANNING_FORM = fl_bgn_form(FL_UP_BOX, 220, 580);

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

/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_grasp_planning_group(void)
{
  int x, y, dy, w, h, shift;
  FL_OBJECT *frame= NULL, *box= NULL;

  frame = fl_add_labelframe(FL_ENGRAVED_FRAME, 18, 235, 185, 330, "Interface");
  box = fl_add_box(FL_ENGRAVED_FRAME, 23, 250, 175, 310, "");
  fl_set_object_color(box, FL_GREEN, FL_COL1);


  MOTIONGROUP = fl_bgn_group();

  x= 30;
  y= 20;
  w= 160;
  h= 40;
  dy= h + 10;
  shift= 40;
  BT_1_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Test");
  BT_2_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + dy, w, h, "SAHandRight");
  BT_3_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "Double Grasp");
  BT_4_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + 3*dy, w, h, "Display grasps");


  BT_5_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + shift + 4*dy, w, h, "Selected Object: none");
  BT_6_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + shift + 5*dy, w, h, "Selected Hand Robot: none");
  BT_7_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + shift + 6*dy, w, h, "Browse Grasps");
  BT_8_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + shift + 7*dy, w, h, "Delete Grasp");
  BT_9_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + shift + 8*dy, w, h, "Add Grasp");
  BT_10_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y + shift + 9*dy, w, h, "Save Grasps");


  fl_set_call_back(BT_1_OBJ, CB_test, 0);
  fl_set_call_back(BT_2_OBJ, CB_SAHandRight, 0);
  fl_set_call_back(BT_3_OBJ, CB_double_grasp, 0);
  fl_set_call_back(BT_4_OBJ, CB_display_grasps, 0);

  fl_set_object_color(BT_4_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_5_OBJ, CB_select_object, 0);
  fl_set_call_back(BT_6_OBJ, CB_select_hand_robot, 0);
  fl_set_call_back(BT_7_OBJ, CB_browse_grasps, 0);
  fl_set_call_back(BT_8_OBJ, CB_delete_grasp, 0);
  fl_set_call_back(BT_9_OBJ, CB_add_grasp, 0);
  fl_set_call_back(BT_10_OBJ, CB_save_grasps, 0);

  fl_end_group();
}


void redraw()
{
  g3d_win *win= NULL;

  win= g3d_get_cur_win();
  win->fct_draw2= &(draw_grasp_planner);

  win->vs.fov= 30;
  g3d_set_projection_matrix(win->vs.projection_mode);

  g3d_draw_allwin();
  g3d_draw_allwin_active();
}

/*
static void cost(gdouble ** f, GtsCartesianGrid g, guint k0, gpointer data)
{
  p3d_rob* robot= (p3d_rob*) data;

  if(robot==NULL)
  {
    printf("%s: %d: g3d_draw_robot_kinematic_chain(): input robot is NULL.\n", __FILE__, __LINE__);
    return;
  }

  int k;
  double d, dmin;
  p3d_vector3 p1, p2, diff;
  p3d_vector3 p, closestPoint;

  gdouble x, y, z = g.z;
  guint i, j;
  gdouble  t = (1. + sqrt(5.))/2.;

  for (i = 0, x = g.x; i < g.nx; i++, x += g.dx) {
    for (j = 0, y = g.y; j < g.ny; j++, y += g.dy) {
       p[0]= x;
       p[1]= y;
       p[2]= z;

       dmin= 1e9;
       for(k=0; k<=robot->njoints; k++)
       {
         if(robot->joints[k]->prev_jnt!=NULL && robot->joints[k]->prev_jnt!=robot->joints[0])
         {
           p3d_mat4ExtractTrans(robot->joints[k]->prev_jnt->abs_pos, p1);
           p3d_mat4ExtractTrans(robot->joints[k]->abs_pos, p2);
           d= gpPoint_to_line_segment_distance(p, p1, p2, closestPoint);
           //p3d_vectSub(p, closestPoint, diff);
         //  d= p3d_vectNorm(diff);
           if(d < dmin) { dmin= d;}
           //g3d_draw_cylinder(p1, p2, radius/10.0, 15);
         }
       }
      
      f[i][j] = dmin;
    }
  }



  return;
}*/

/*
static void sphere(gdouble ** f, GtsCartesianGrid g, guint k, gpointer data)
{
  gdouble x, y, z = g.z;
  guint i, j;
  gdouble  t = (1. + sqrt(5.))/2.;

  for (i = 0, x = g.x; i < g.nx; i++, x += g.dx)
    for (j = 0, y = g.y; j < g.ny; j++, y += g.dy) {
      gdouble t4 = t*t*t*t;
      gdouble p1 = x*x - t4*y*y;
      gdouble p2 = y*y - t4*z*z;
      gdouble p3 = z*z - t4*x*x;
      gdouble p4 = x*x*x*x + y*y*y*y + z*z*z*z 
  - 2.*x*x*y*y - 2.*y*y*z*z - 2.*x*x*z*z;
      gdouble tmp = x*x + y*y + z*z - 1.;
      gdouble q1 = tmp*tmp;
      gdouble tmp1 = x*x + y*y + z*z - (2. - t);
      gdouble q2 = tmp1*tmp1;
      
      f[i][j] = 8.*p1*p2*p3*p4 + (3. + 5.*t)*q1*q2;
    }
}
*/

void draw_grasp_planner()
{   
//   chull3d.draw();
  g3d_draw_cylinder(E1, E2, 0.005, 8);
  g3d_draw_cylinder(E2, E3, 0.005, 8);

  glBegin(GL_QUADS);
    glVertex3dv(A1); glVertex3dv(B1); glVertex3dv(D1); glVertex3dv(C1);
    glVertex3dv(A2); glVertex3dv(B2); glVertex3dv(D2); glVertex3dv(C2);
  glEnd();

  g3d_draw_frame(FRAME, 0.1);

  p3d_vector3 com;
  p3d_vectCopy(XYZ_ENV->cur_robot->o[0]->pol[0]->poly->cmass, com);
  g3d_draw_solid_sphere(com[0], com[1], com[2], 0.003, 4);

  for(unsigned int i=0; i<POINTS.size(); ++i)
  {
    g3d_draw_solid_sphere(POINTS[i][0], POINTS[i][1], 0.0, 0.003, 4);
  }

  G3D_Window *win;
//   p3d_rob *hand_robot= p3d_get_robot_by_name((char*)(GP_GRIPPER_ROBOT_NAME));
//   p3d_rob *hand_robot= p3d_get_robot_by_name((char*)(GP_SAHAND_RIGHT_ROBOT_NAME));
//   p3d_rob *object_robot= p3d_get_robot_by_name(ObjectName);

  win = g3d_get_cur_win();

  //display all the grasps of the list:
  if(DISPLAY_GRASPS)
  {
    for(std::list<gpGrasp>::iterator iter= GRASP_LIST.begin(); iter!=GRASP_LIST.end(); ++iter)
    { ( *iter ).draw ( 0.005 );    }
  }

//   p3d_rob *object= (p3d_rob*)p3d_get_robot_by_name(ObjectName);
//   g3d_draw_poly_curvature(object->o[0]->pol[0]->poly);


// double q[4];
// gpHand_properties handProp;
// p3d_vector3 bestForceDirection, position, normal, p2;
// p3d_matrix4 Twrist;
// handProp.initialize(GP_SAHAND_RIGHT);
// p3d_rob *hand_robot= (p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
// gpGet_wrist_frame(hand_robot, Twrist);
// for(int k=1; k<=4; ++k)
// {if(k!=2) continue;
//   gpGet_SAHfinger_joint_angles(hand_robot, handProp, q, k, 0);
//   gpSAHfinger_main_force_direction(Twrist, handProp, q, k, bestForceDirection);
//   gpSAHfinger_forward_kinematics(Twrist, handProp, q, position, normal, k);
//   
//   for(int i=0; i<3; ++i)
//   {  p2[i]= position[i] + 0.05*bestForceDirection[i];  }
// p3d_vectCopy(bestForceDirection, GRASP.contacts[1].forceDirection);
//   glColor3f(1.0, 0.0, 0.0);
// }

  GRASP.draw(0.05);

// prop.draw(p3d_mat4IDENTITY);
// gpDraw_reachable_points((p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob *)p3d_get_robot_by_name("Horse"), handProp);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), handProp, 1);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), handProp, 2);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), handProp, 3);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), handProp, 4);
return;


//dynamic_grasping(); 
}




// static void CB_gripper_obj ( FL_OBJECT *obj, long arg )
// { printf("CB_gripper\n");
//   unsigned int i;
//   static unsigned int count= 0, firstTime= TRUE;
//   p3d_rob *robot= NULL;
//   
//   if(firstTime)
//   {
//     firstTime= 0;
//     gpGet_grasp_list(ObjectName, GP_PR2_GRIPPER, GRASP_LIST);
//   }
// 
//   i= 0;
//   for (std::list<gpGrasp>::iterator iter=GRASP_LIST.begin(); iter!=GRASP_LIST.end(); iter++ )
//   {
//     GRASP= ( *iter );
//     i++;
//     if ( i>count )
//     {  break; }
//   }
//   count++;
//   if ( count>GRASP_LIST.size() )
//   {  count= 0;  }
// 
//   GRASP.print();
//   std::string robotName= GP_PR2_GRIPPER_ROBOT_NAME;
//   robot= (p3d_rob*)p3d_get_robot_by_name( (char*)(robotName.c_str()) );
//   gpSet_robot_hand_grasp_configuration(robot, (p3d_rob*)p3d_get_robot_by_name(ObjectName), GRASP);
//   configPt q= p3d_get_robot_config( robot );
//   p3d_copy_config_into( robot, q, &robot->ROBOT_POS );
//   p3d_destroy_config( robot, q );
// 
//   redraw();
//   return;
// }


static void CB_SAHandRight(FL_OBJECT *obj, long arg)
{ 
  printf("CB_SAHandRight\n");
  unsigned int i;
  static unsigned int count= 0, firstTime= TRUE;
  p3d_rob *robot= (p3d_rob*)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
  p3d_rob *object= (p3d_rob*)p3d_get_robot_by_name(ObjectName);
  

  if(firstTime)
  {
    firstTime= 0;
    gpGet_grasp_list(ObjectName, GP_SAHAND_RIGHT, GRASP_LIST);
//     gpReduce_grasp_list_size(GRASP_LIST, GRASP_LIST, 12);
  }

  i= 0;
  for (std::list<gpGrasp>::iterator iter=GRASP_LIST.begin(); iter!=GRASP_LIST.end(); iter++ )
  {
    GRASP= ( *iter );
    i++;
    if ( i>count )
    {  break; }
  }
  count++;
  if ( count>GRASP_LIST.size() )
  {  count= 0;  }

  gpSet_robot_hand_grasp_configuration(robot, object, GRASP);

//    gpSet_robot_hand_grasp_open_configuration(robot, object, GRASP);
  p3d_copy_config_into(robot, p3d_get_robot_config(robot), &robot->ROBOT_POS);
  printf("grasp %d\n",GRASP.ID);

  redraw();
  return;
}


static void CB_double_grasp(FL_OBJECT *obj, long arg)
{
  printf("CB_double_grasp\n");

  static int unsigned firstTime= TRUE, count= 0;
  unsigned int i;
  gpHand_properties handProp;
  p3d_rob *hand1, *hand2, *object, *justin;
  std::list<gpGrasp> graspList1, graspList2;

  hand1= p3d_get_robot_by_name("JIDO_GRIPPER");
  hand2= p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);

  object= p3d_get_robot_by_name(ObjectName);
  justin= p3d_get_robot_by_name("ROBOT");

  if(firstTime)
  {  
   gpGet_grasp_list(ObjectName, GP_GRIPPER, graspList1);
   gpGet_grasp_list(ObjectName, GP_SAHAND_RIGHT, graspList2);


   gpReduce_grasp_list_size(graspList1, graspList1, 50);
   gpReduce_grasp_list_size(graspList2, graspList2, 50);

   gpDouble_grasp_generation(hand1, hand2, object, graspList1, graspList2, DOUBLE_GRASP_LIST);
   firstTime= false;  
   printf("%d double grasps\n",DOUBLE_GRASP_LIST.size());
  }

  std::list<gpDoubleGrasp>::iterator iter;
  i= 0;
  for ( iter=DOUBLE_GRASP_LIST.begin(); iter!=DOUBLE_GRASP_LIST.end(); iter++ )
  {
    DOUBLEGRASP= ( *iter );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>DOUBLE_GRASP_LIST.size() )
  {  count= 1;  }

  //gpCompute_grasp_open_config(justin, DOUBLEGRASP, object, 2);

  gpSet_robot_hand_grasp_configuration(hand1, object, DOUBLEGRASP.grasp1);
  gpSet_robot_hand_grasp_configuration(hand2, object, DOUBLEGRASP.grasp2);

  redraw();
}


struct gpBorderEdge
{
  double length;
  gpVector3D e1, e2;
};

bool gpCmpBorderEdge(const gpBorderEdge &e1, const gpBorderEdge &e2)
{  return (e1.length > e2.length); }

void compute_object_border()
{
  double dz, minZ;
  p3d_vector3 p;
  std::vector<unsigned int> edgeIndices;
  p3d_polyhedre *poly= NULL;
  std::vector<double> coords(2);
  std::vector< std::vector<double> > input_points;
  gpBorderEdge edge;
  std::vector<gpBorderEdge> edges;
  double dotU, dotV, dotW, minDotU, maxDotV;
  p3d_vector3 e1, e2, u, v, w= {0,0,1}, diff;

  poly= XYZ_ENV->cur_robot->o[0]->pol[0]->poly;

  gpCompute_mass_properties(poly);

  POINTS.resize(poly->nb_points);
  for(unsigned int i=0; i<poly->nb_points; ++i)
  {
    if( (i==0) ||  (poly->the_points[i][2] < minZ) )
    {  minZ= poly->the_points[i][2]; }
  }
  dz= 0.03;
  for(unsigned int i=0; i<poly->nb_points; ++i)
  {
    if(poly->the_points[i][2] > minZ + dz)
    {  continue;  }
    coords[0]= poly->the_points[i][0];
    coords[1]= poly->the_points[i][1];
    input_points. push_back(coords);
  }
  POINTS.resize(input_points.size());
  for(unsigned int i=0; i<input_points.size(); ++i)
  {
    POINTS[i][0]= input_points[i][0];
    POINTS[i][1]= input_points[i][1];
    POINTS[i][2]= minZ+dz; 
 }
  chull.setPoints(input_points);
  chull.compute(false, 0.0, true);

  for(unsigned int i=0; i<chull.nbFaces(); ++i)
  {
    edgeIndices= chull.hull_faces[i].vertices();
    edge.e1.set(input_points[edgeIndices[0]][0], input_points[edgeIndices[0]][1], minZ+dz);
    edge.e2.set(input_points[edgeIndices[1]][0], input_points[edgeIndices[1]][1], minZ+dz);
    edge.length= sqrt( pow(edge.e2.x-edge.e1.x,2) + pow(edge.e2.y-edge.e1.y,2) + pow(edge.e2.z-edge.e1.z,2) );
    edges.push_back(edge);
  }

  std::sort(edges.begin(), edges.end(),  gpCmpBorderEdge);
  edge= edges.front();
  edge= edges.at(1);
  for(unsigned int i=0; i<3; ++i)
  {
    E1[i]= edge.e1[i];
    E2[i]= edge.e2[i];
  }

  p3d_vectSub(E2, E1, u);
  p3d_vectNormalize(u, u);
  p3d_vectXprod(w, u, v);

  std::vector<float> dots;
  for(unsigned int i=0; i<poly->nb_points; ++i)
  {
    p3d_vectCopy(poly->the_points[i], p);
    if( p[2] > minZ + dz)
    {  continue; }
    p[2]= minZ + dz;
    p3d_vectSub(p, E1, diff);
    dotU= p3d_vectDotProd(diff, u);
    dotV= p3d_vectDotProd(diff, v);
    dots.push_back(dotU);
  }

//   std::sort(dots.begin(), dots.end(),  std::greater <float>());
  std::sort(dots.begin(), dots.end());
  minDotU= dots[1];


  for(unsigned int i=0; i<3; ++i)
  {
    E2[i]= E1[i] + minDotU*u[i];
    E3[i]= E1[i] + minDotU*u[i] - v[i];

    A1[i]= E1[i];
    B1[i]= E2[i];
    C1[i]= E1[i];
    D1[i]= E2[i];

    A2[i]= E2[i];
    B2[i]= E3[i];
    C2[i]= E2[i];
    D2[i]= E3[i];
  }
  A1[2]= B1[2]= A2[2]= B2[2]= minZ;
  C1[2]= D1[2]= C2[2]= D2[2]= minZ+dz;


  p3d_vectSub(E3, E2, u);
  p3d_vectNormalize(u, u);
  p3d_vectSub(E1, E2, v);
  p3d_vectNormalize(v, v);
  p3d_vectXprod(u, v, w);
  p3d_vectNormalize(w, w);
  p3d_mat4Copy(p3d_mat4IDENTITY, FRAME);
  for(unsigned int i=0; i<3; ++i)
  {
    FRAME[i][0]= u[i];     FRAME[i][1]= v[i];     FRAME[i][2]= w[i];  FRAME[i][3]= E2[i];
  }
  FRAME[2][3]= minZ;

  p3d_matrix4 Tobj, Tobj_inv, T;
  p3d_get_freeflyer_pose(XYZ_ENV->cur_robot, Tobj);
  p3d_matInvertXform(Tobj, Tobj_inv);
  p3d_mat4Mult(Tobj_inv, FRAME, T);
  p3d_mat4Print(T, "T");
//   FILE *file;
//   file= fopen("vertices","w");
//   for(unsigned int i=0; i<poly->nb_points; ++i)
//   {
//     fprintf(file, "    p3d_add_desc_vert %f %f %f\n",poly->the_points[i][0]-poly->cmass[0],poly->the_points[i][1]-poly->cmass[1],poly->the_points[i][2]-poly->cmass[2]);
//   }
//   fclose(file);
}

static void CB_test(FL_OBJECT *obj, long arg)
{
//p3d_export_as_OFF(XYZ_ENV->cur_robot->o[0]->pol[0]->poly);
 gpExport_robot_for_coldman(XYZ_ENV->cur_robot);
//  chull3d.setPoints(XYZ_ENV->cur_robot->o[0]->pol[0]->poly->the_points, XYZ_ENV->cur_robot->o[0]->pol[0]->poly->nb_points);
//  chull3d.compute(false, -1.0, true); 
// 
//  chull3d.setPoints(XYZ_ENV->cur_robot->o[0]->pol[0]->poly->the_points, XYZ_ENV->cur_robot->o[0]->pol[0]->poly->nb_points);
//  chull3d.compute(false, -1.0, true); 
//  compute_object_border();
//   gpCompute_stable_placements((p3d_rob*)p3d_get_robot_by_name(ObjectName), POSELIST);
  redraw();
  return;
  p3d_rob *object= (p3d_rob*)p3d_get_robot_by_name(ObjectName);
  
  p3d_compute_mean_curvature(object->o[0]->pol[0]->poly);
redraw();return;

  gpHand_properties handProp;
//   p3d_rob *robot= (p3d_rob*)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
//   p3d_rob *object= (p3d_rob*)p3d_get_robot_by_name(ObjectName);
//   p3d_matrix4 T, Tobject, grasp_frame;
//   p3d_vector3 zAxis;
//   double alpha, beta, gamma;
  std::vector<double> config, openConfig;
//   double height= 0.21;
  std::list<gpGrasp> graspList;
  gpGrasp grasp; 
  std::string filename;

  filename= "../graspPlanning/graspLists/SAHandRight/"+ std::string(ObjectName) + "Grasps.xml";


/*
  for(int i=0; i<24; ++i)
  {
//     if(i<4)
//     {
//       alpha= i*M_PI_2;
//       beta= 0.0;
//       gamma= 0.0;
//     }
//     else
//     {
//       alpha= (i-4)*M_PI_2;
//       beta= 0.0;
//       gamma= M_PI;
//     }
//     p3d_mat4Pos(T, 0, 0, 0.5*height, alpha, beta, gamma);
//     p3d_mat4ExtractColumnZ(T, zAxis);
//     T[0][3]-= 0.5*height*zAxis[0];
//     T[1][3]-= 0.5*height*zAxis[1];
//     T[2][3]-= 0.5*height*zAxis[2];
    if(i<12)
    {
      alpha= 0.0;
      beta= 0.0;
      gamma= i*2.0*M_PI/12.0;
      p3d_mat4Pos(T, 0, 0, 0, alpha, beta, gamma);
    }
    else
    {
      alpha= 0.0;
      beta= M_PI;
      gamma= (i-12)*2.0*M_PI/12.0;
      height= 0.15;
      p3d_mat4Pos(T, 0, 0, 0.5*height, alpha, beta, gamma);
      p3d_mat4ExtractColumnZ(T, zAxis);
      T[0][3]-= 0.5*height*zAxis[0];
      T[1][3]-= 0.5*height*zAxis[1];
      T[2][3]-= 0.5*height*zAxis[2];
    }
    p3d_set_freeflyer_pose(object, T);
    addCurrentGraspToList(GRASP_LIST);
  }
*/

// std::cout << filename << std::endl;
//   gpSave_grasp_list(GRASP_LIST, filename);

return;
}


void dynamic_grasping()
{
  static bool firstTime= true;
  int i, result;
  p3d_vector3 objectCenter;
  p3d_matrix4 objectPose;
  g3d_win *win= NULL;
  static p3d_rob *robot     = NULL;
  static p3d_rob *object    = NULL;
  static p3d_rob *hand_robot = NULL;
  static gpHand_properties handProp;

  if(firstTime)
  {
    firstTime= false;  
    
    result= gpGet_grasp_list(ObjectName, GP_SAHAND_RIGHT, GRASP_LIST);
    
    if(result==GP_ERROR)
    {  return;  }

    object= p3d_get_robot_by_name(ObjectName);
    if(object==NULL)
    {  return;  }

    robot= p3d_get_robot_by_name(RobotName);
    if(robot==NULL)
    {  return;  }

    gpCompute_mass_properties(object->o[0]->pol[0]->poly);
    
    handProp.initialize(GRASP_LIST.front().hand_type);
    hand_robot= p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
    if(hand_robot!=NULL)
    {  p3d_set_freeflyer_pose2(hand_robot, 10, 0, -10, 0, 0, 0);  }
  }


/*
#ifdef LIGHT_PLANNER
 if ( robot!=NULL )
 {
  if ( robot->nbCcCntrts!=0 )
  {  p3d_desactivateCntrt ( robot, robot->ccCntrts[0] );    }
 }
#endif*/

  p3d_get_body_pose(object, 0, objectPose);


  objectCenter[0]= objectPose[0][3] + object->o[0]->pol[0]->poly->cmass[0];
  objectCenter[1]= objectPose[1][3] + object->o[0]->pol[0]->poly->cmass[1];
  objectCenter[2]= objectPose[2][3] + object->o[0]->pol[0]->poly->cmass[2];

  win= g3d_get_cur_win();
  win->vs.x= objectCenter[0];   win->vs.y= objectCenter[1];   win->vs.z= objectCenter[2];

  if(GRASP_LIST.empty())
  {
    printf("No grasp was found.\n");
    return;
  }

  configPt qcur= NULL, qgrasp= NULL, qend= NULL;
  qcur= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &qcur);
  qend= NULL;
  p3d_get_robot_config_into(robot, &qcur);
  qend= gpFind_grasp_from_base_configuration(robot, object, GRASP_LIST, GP_PA10, qcur, GRASP, handProp);

  if(qend!=NULL)
  {
    p3d_set_and_update_this_robot_conf(robot, qend);
//           XYZ_ENV->cur_robot= robot;
    p3d_copy_config_into(robot, qend, &robot->ROBOT_POS);
    p3d_destroy_config(robot, qend);
    qend= NULL; 
    XYZ_ENV->cur_robot= object;
    return;
  }  

/*
  i= 0;
  for ( igrasp=GRASP_LIST.begin(); igrasp!=GRASP_LIST.end(); igrasp++ )
  {
    GRASP= ( *igrasp );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>GRASP_LIST.size() )
  {  count= 1;  }*/
  //p3d_release_object(robot);

  //find a configuration for the whole robot (mobile base + arm):

 
  if(robot!=NULL)
  {
    for(i=0; i<50; ++i)
    {
        qgrasp= gpRandom_robot_base(robot, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter);

        if ( qgrasp==NULL )
        {  break;  }

        qend= NULL;
        qend= gpFind_grasp_from_base_configuration(robot, object, GRASP_LIST, GP_PA10, qgrasp, GRASP, handProp);

        if ( qend!=NULL )
        {
          p3d_set_and_update_this_robot_conf(robot, qend);
//           XYZ_ENV->cur_robot= robot;
          p3d_copy_config_into(robot, qend, &robot->ROBOT_POS);
          p3d_destroy_config(robot, qend);
          qend= NULL;
          break;
        }
        p3d_destroy_config ( robot, qgrasp );
        qgrasp= NULL;
   }
   if(qgrasp!=NULL)
   {  p3d_destroy_config ( robot, qgrasp );  }
   if ( i==250 )
   {  printf ( "No platform configuration was found.\n" );  }
   else
   {  printf ( "Grasp planning was successfull.\n" );  }
  }

//   gpSet_robot_hand_grasp_configuration(SAHandRight_robot, object, GRASP);

   XYZ_ENV->cur_robot= object;

  //p3d_set_object_to_carry(robot, "Horse");
  //p3d_grab_object(robot, 0);

  return;
}

void contact_points()
{
 bool firstTime= true;
 unsigned int i,j;
 static gpHand_properties handProp;
 p3d_vector3 center;
 p3d_matrix4 Tobject, Twrist, T;
 p3d_rob *object= NULL;
 p3d_rob *robot_hand= NULL; 
 std::list<gpContact> contactList, points;
 std::list<gpContact>::iterator iter;
 static gpKdTree kdtree;
 GLfloat mat[16];
 
 object= p3d_get_robot_by_name("Horse");
 robot_hand= p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
 
 if(object==NULL || robot_hand==NULL)
 { return; }
 
 if(firstTime)
 {
   firstTime= false;   
   handProp.initialize(GP_SAHAND_RIGHT);
   gpSample_obj_surface(object->o[0], 0.005, 0, contactList);
   kdtree.build(contactList);   
 }

 p3d_get_first_joint_pose(robot_hand, Twrist);
 p3d_get_first_joint_pose(object, Tobject);
 
 p3d_to_gl_matrix(Tobject, mat);
 
 glPushAttrib(GL_LIGHTING_BIT | GL_POINT_BIT | GL_LINE_BIT);
 glPointSize(7);
 glLineWidth(1);
 
 //gpDraw_SAHfinger_manipulability_ellipsoid(robot_hand, handProp, 2);

 glPushMatrix();
 //glMultMatrixf(mat);

 
 float clock1;
 clock1= clock();
 for(int k=0; k<10000; ++k)
 for(i=1; i<=1; ++i) //for each finger:
 {
   p3d_mat4Mult(Twrist, handProp.Twrist_finger[i], T);
   
   if(i==0) glColor3f(1.0, 0.0, 0.0);
   else if(i==1) glColor3f(0.0, 1.0, 0.0);
   else if(i==2) glColor3f(0.0, 0.0, 1.0);
   else glColor3f(1.0, 1.0, 0.0);

   for(j=0; j<handProp.workspace.size(); ++j)
   {
     p3d_xformPoint(T, handProp.workspace[j].center, center);
     glColor3f(1, 0, 0);
     points.clear();
     kdtree.sphereIntersection(center, handProp.workspace[j].radius, points);
 /*
     glDisable(GL_LIGHTING);
     
     if(LEVEL==1 || LEVEL==3)
     {
       glBegin(GL_POINTS);
       for(iter=points.begin(); iter!=points.end(); iter++)
       { 
         glVertex3dv(iter->position);
       }
       glEnd();
     }
     
     glEnable(GL_LIGHTING);
     glEnable(GL_BLEND);
     if(LEVEL==2 || LEVEL==3 )
     {
       glEnable(GL_CULL_FACE);
       glEnable(GL_BLEND);
       glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
       glDepthMask(GL_FALSE);
       glColor4f(0, 1, 0, 0.7);
       glPushMatrix();
        glTranslatef(center[0], center[1], center[2]);
        g3d_draw_solid_sphere(handProp.workspace[j].radius, 20);
      //  g3d_draw_wire_ellipsoid(handProp.workspace[j].radius, handProp.workspace[j].radius, handProp.workspace[j].radius);
       glPopMatrix();
     glDepthMask(GL_TRUE);
     glDisable(GL_CULL_FACE);
     }*/
   }
 }

 float elapsedTime= (clock()-clock1)/CLOCKS_PER_SEC;
 
 printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, (int)(elapsedTime/60.0), (int)(elapsedTime - 60*((int)(elapsedTime/60.0))) );

 
 glPopMatrix();
 glPopAttrib();
}

//!#######################################################################################################
//!########################################### user interface ############################################
//!#######################################################################################################
static void CB_display_grasps(FL_OBJECT *obj, long arg)
{
  DISPLAY_GRASPS= !DISPLAY_GRASPS;

  if(DISPLAY_GRASPS)
  {  fl_set_button(BT_4_OBJ, TRUE);  }
  else
  {  fl_set_button(BT_4_OBJ, FALSE); }
  redraw();
}


//! Initializes the data for calling the grasp planner.
int initialize_grasp_planner()
{
  gpHand_type hand_type;

  if(OBJECT==NULL)
  {
    printf("%s: %d: initialize_grasp_planner(): Select an object first.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(HAND_ROBOT==NULL)
  {
    printf("%s: %d: initialize_grasp_planner(): Select a hand robot first.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if( strcmp(HAND_ROBOT->name, GP_GRIPPER_ROBOT_NAME)==0 )
  {  hand_type= GP_GRIPPER;  }
  else if( strcmp(HAND_ROBOT->name, GP_SAHAND_RIGHT_ROBOT_NAME)==0 )
  {  hand_type= GP_SAHAND_RIGHT;  }
  else if( strcmp(HAND_ROBOT->name, GP_SAHAND_LEFT_ROBOT_NAME)==0 )
  {  hand_type= GP_SAHAND_LEFT;  }
  else if( strcmp(HAND_ROBOT->name, GP_PR2_GRIPPER_ROBOT_NAME)==0 )
  {  hand_type= GP_PR2_GRIPPER;  }
  else
  {
    printf("%s: %d: initialize_grasp_planner(): The selected robot (\"%s\") has an unrecognized name.\n",__FILE__,__LINE__,HAND_ROBOT->name);
    printf("It should be one of: %s %s %s %s.\n", GP_GRIPPER_ROBOT_NAME, GP_SAHAND_RIGHT_ROBOT_NAME, GP_SAHAND_LEFT_ROBOT_NAME, GP_PR2_GRIPPER_ROBOT_NAME);
    printf("Be sure that you have selected the hand robot.\n");
    return GP_ERROR;
  }

  HAND_PROP.initialize(hand_type);
  INIT_IS_DONE= true;

  return GP_OK;
}

//! Selects the next object among the simple freeflyer robots of the environment.
//! \return a pointer to the selected object, NULL if no object was found
p3d_rob* select_object()
{
  std::vector<p3d_rob*> freeflyers;

  // first get all the simple freeflyer robots (potential objects):
  for(int i=0; i<XYZ_ENV->nr; ++i)
  {
    if(XYZ_ENV->robot[i]->njoints!=1)
    { continue; }
    if(XYZ_ENV->robot[i]->joints[1]->type!=P3D_FREEFLYER)
    { continue; }
    freeflyers.push_back(XYZ_ENV->robot[i]);
  }
  if(freeflyers.empty())
  {
    printf("%s: %d: select_object(): There is no possible objects (simple freeflyer robots).\n",__FILE__,__LINE__);
    return NULL;
  }

  if(OBJECT==NULL)
  {
    OBJECT= freeflyers[0];
  }
  else
  {
    for(unsigned int i=0; i<freeflyers.size(); ++i)
    {
      if(freeflyers[i]==OBJECT)
      {
        if( i < freeflyers.size() - 1 )
        {
          OBJECT= freeflyers[i+1];
          break;
        }
        else
        {
          OBJECT= freeflyers[0];
          break;
        }
      }
    }
  }

  return OBJECT;
}


static void CB_select_object(FL_OBJECT *obj, long arg)
{
  char newLabel[64];

  strcpy (newLabel,"Selected Object: ");

  OBJECT= select_object();
  if(OBJECT!=NULL)
  {
    strcat(newLabel, OBJECT->name);
    GRASP_LIST.clear();
    INIT_IS_DONE= false;
  }

  fl_set_object_label(BT_5_OBJ, newLabel);
}

//! Selects the next hand robot among the robots that have one of the names defined in graspPlanning.h
//! \return a pointer to the selected robot, NULL if no robot was found
p3d_rob* select_hand_robot()
{
  std::vector<p3d_rob*> handRobots;

  // first get all the robots with specific names:
  for(int i=0; i<XYZ_ENV->nr; ++i)
  {
    if( strcmp(XYZ_ENV->robot[i]->name,GP_GRIPPER_ROBOT_NAME)==0 || strcmp(XYZ_ENV->robot[i]->name,GP_SAHAND_RIGHT_ROBOT_NAME)==0 || strcmp(XYZ_ENV->robot[i]->name,GP_SAHAND_LEFT_ROBOT_NAME)==0 || strcmp(XYZ_ENV->robot[i]->name,GP_PR2_GRIPPER_ROBOT_NAME)==0 )
    { 
      handRobots.push_back(XYZ_ENV->robot[i]);
      continue;
    }
  }
  if(handRobots.empty())
  {
    printf("%s: %d: select_hand_robot(): There is no robot that could be a robot hand.\n",__FILE__,__LINE__);
    printf("It should be one of: %s %s %s %s.\n", GP_GRIPPER_ROBOT_NAME, GP_SAHAND_RIGHT_ROBOT_NAME, GP_SAHAND_LEFT_ROBOT_NAME, GP_PR2_GRIPPER_ROBOT_NAME);
    return NULL;
  }

  if(HAND_ROBOT==NULL)
  {
    HAND_ROBOT= handRobots[0];
  }
  else
  {
    for(unsigned int i=0; i<handRobots.size(); ++i)
    {
      if(handRobots[i]==HAND_ROBOT)
      {
        if( i < handRobots.size() - 1 )
        {
          HAND_ROBOT= handRobots[i+1];
          break;
        }
        else
        {
          HAND_ROBOT= handRobots[0];
          break;
        }
      }
    }
  }

  return HAND_ROBOT;
}

static void CB_select_hand_robot(FL_OBJECT *obj, long arg)
{
  char newLabel[64];

  strcpy (newLabel,"Selected Hand: ");

  HAND_ROBOT= select_hand_robot();
  if(HAND_ROBOT!=NULL)
  {
    strcat(newLabel, HAND_ROBOT->name);
    GRASP_LIST.clear();
    INIT_IS_DONE= false;
  }

  fl_set_object_label(BT_6_OBJ, newLabel);
}

//! Interface function:
//! -The first time it is called, a grasp list file is loaded and the first grasp is selected.
//! -For each following call, the next element of the list is loaded.
static void CB_browse_grasps(FL_OBJECT *obj, long arg)
{
  unsigned int i;
  static unsigned int count= 0;

  if(!INIT_IS_DONE)
  {
    if(initialize_grasp_planner()==GP_ERROR)
    {
      printf("%s: %d: CB_browse_grasps(): Can not initialize.\n",__FILE__,__LINE__);
      return;
    }
    gpGet_grasp_list(OBJECT->name, HAND_PROP.type, GRASP_LIST);
  }

  i= 0;
  for (std::list<gpGrasp>::iterator iter=GRASP_LIST.begin(); iter!=GRASP_LIST.end(); iter++ )
  {
    GRASP= (*iter);
    i++;
    if(i>count)
    {  break; }
  }
  count++;
  if(count >= GRASP_LIST.size())
  {  count= 0;  }
  if(GRASP.hand_type == GP_HAND_NONE){
    GRASP = gpGrasp(HAND_PROP);
  }
  gpSet_robot_hand_grasp_configuration(HAND_ROBOT, OBJECT, GRASP);
//   gpSet_robot_hand_grasp_open_configuration(HAND_ROBOT, OBJECT, GRASP);
  p3d_copy_config_into(HAND_ROBOT, p3d_get_robot_config(HAND_ROBOT), &HAND_ROBOT->ROBOT_POS);

  printf("Selected grasp: #%d\n",GRASP.ID);
//   GRASP.print();

  redraw();
  return;
}

//! Interface function:
//! Deletes the currently selected grasp.
static void CB_delete_grasp(FL_OBJECT *obj, long arg)
{
  std::list<gpGrasp>::iterator igrasp;
  for(igrasp=GRASP_LIST.begin(); igrasp!=GRASP_LIST.end(); ++igrasp )
  {
    if(igrasp->ID==GRASP.ID)
    {
      igrasp= GRASP_LIST.erase(igrasp);
      break;
    }
  }
  printf("Delete grasp #%d\n",GRASP.ID);
  if(igrasp!=GRASP_LIST.end())
  {
    GRASP= *igrasp;
    gpSet_robot_hand_grasp_configuration(HAND_ROBOT, OBJECT, GRASP);
  }
  printf("Delete grasp #%d\n",GRASP.ID);
  redraw();
}


//! Computes the current grasp frame and hand config (for SAHand right) 
int addCurrentGraspToList(p3d_rob *hand_robot, p3d_rob *object, gpHand_properties &handProp, std::list<gpGrasp> &graspList)
{
  if(hand_robot==NULL)
  {
    printf("%s: %d: addCurrentGraspToList(): Input robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
  if(object==NULL)
  {
    printf("%s: %d: addCurrentGraspToList(): Input object is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  p3d_matrix4 grasp_frame, Tobject, Tobject_inv, Trobot, Tgrasp_frame_hand_inv, Ttmp;
  gpGrasp grasp; 
  std::vector<double> config;

  grasp.autoGen= false;
  grasp.object_name= object->name;
  grasp.hand_type= handProp.type;
  grasp.stability= 1.0;
  grasp.quality= 1.0;

  p3d_get_freeflyer_pose(hand_robot, Trobot);
  p3d_get_freeflyer_pose(object, Tobject);

  p3d_matInvertXform(Tobject, Tobject_inv);
  p3d_matInvertXform(handProp.Tgrasp_frame_hand, Tgrasp_frame_hand_inv);

  p3d_matMultXform(Tobject_inv, Trobot, Ttmp);
  p3d_matMultXform(Ttmp, Tgrasp_frame_hand_inv, grasp_frame);

  gpGet_hand_configuration(hand_robot, handProp, 0, config);

  grasp.config= config;
  p3d_mat4Copy(grasp_frame, grasp.frame); 
  grasp.ID= graspList.size()+1;

  grasp.computeOpenConfig(hand_robot, object, false);

  graspList.push_back(grasp);

  return 0;
}

//! Interface function:
//! Adds a grasp to the list from the current hand configuration.
static void CB_add_grasp(FL_OBJECT *obj, long arg)
{
  if(!INIT_IS_DONE)
  {
    if(initialize_grasp_planner()==GP_ERROR)
    {
      printf("%s: %d: CB_add_grasp(): Can not initialize.\n",__FILE__,__LINE__);
      return;
    }
  }

  addCurrentGraspToList(HAND_ROBOT, OBJECT, HAND_PROP, GRASP_LIST);
}

//! Interface function:
//! Saves the current grasp list into a file.
static void CB_save_grasps(FL_OBJECT *obj, long arg)
{
  if(GRASP_LIST.empty())
  {
    printf("%s: %d: CB_save_grasps(): The grasp list is empty.\n",__FILE__,__LINE__);
    return;
  }
  std::string pathName, handFolderName, graspListFile;
  DIR *directory= NULL;


  pathName= std::string(getenv("HOME_MOVE3D")) + std::string("/graspPlanning/graspLists/");
  handFolderName= pathName + gpHand_type_to_folder_name (HAND_PROP.type);

  // look for a directory for the chosen hand:
  directory= opendir(handFolderName.c_str());
  if(directory==NULL)
  {
    // directory needs to be created:
    if(mkdir(handFolderName.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH ) ==-1)
    {
      printf("%s: %d: CB_save_grasps(): failed to create directory \"%s\".\n", __FILE__, __LINE__, handFolderName.c_str() );
      return;
    }
  }
  else
  {
    closedir(directory);
  }

  pathName= std::string(getenv("HOME_MOVE3D")) + std::string("/graspPlanning/graspLists/");
  handFolderName= pathName + gpHand_type_to_folder_name(GRASP_LIST.front().hand_type);
  graspListFile= handFolderName  + std::string ( "/" ) + std::string(GRASP_LIST.front().object_name) + std::string("Grasps_new.xml");

  gpSave_grasp_list(GRASP_LIST, graspListFile);
}

//!#######################################################################################################
//!#######################################################################################################


void test_manipulation()
{
  p3d_rob * robotPt= p3d_get_robot_by_name("JIDOKUKA_ROBOT");
  manipulation= new ManipulationPlanner(robotPt);

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  std::vector<double>  objStart, objGoto;
  ManipulationData configs(manipulation->robot());
//   p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name(ObjectName);
//   p3d_matrix4 T, tAtt;
  gpGrasp grasp;
//   p3d_matrix4 handFrame;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();

  manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(),  objStart, objGoto, ObjectName, (char*)"", (char*)"", grasp, confs, smTrajs);

}


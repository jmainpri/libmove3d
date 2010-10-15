#include <time.h>
#include <list>
#include <string>
#include "GraspPlanning-pkg.h"
#include "Collision-pkg.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../localpath/proto/p3d_multiLocalPath_proto.h"
#include <sys/stat.h>
#include <sys/types.h>

#include "Manipulation.h"


// static char ObjectName[]= "Horse";
static char ObjectName[]= "GREY_TAPE";
static char RobotName[]= "JIDO_ROBOT";
static char HandRobotName[]= "SAHandRight_robot";
static bool display_grasps= false;
static p3d_rob *ROBOT= NULL; // the robot
static p3d_rob *HAND_ROBOT= NULL; // the hand robot
static p3d_rob *OBJECT= NULL; // the object to grasp
static p3d_polyhedre *POLYHEDRON= NULL; // the polyhedron associated to the object
static gpHand_properties HAND_PROP;  // information about the used hand
static gpArm_type ARM_TYPE= GP_PA10; // type of the robot's arm
static p3d_vector3 CMASS; // object's center of mass
static p3d_matrix3 IAXES; // object's main inertia axes
static double IAABB[6]; // bounding box aligned on the object's inertia axes
static std::list<gpGrasp> GRASPLIST;
static std::list<class gpDoubleGrasp> DOUBLEGRASPLIST;
static std::list<gpContact> CONTACTLIST;
static std::list<gpContact> CONTACTLIST2;
static std::list<gpVector3D> SAMPLES;
static std::list<gpVector3D> CLOSESTPOINTS;
static std::vector<gpVector3D> POINTS;
static std::vector<gpSphere> SPHERES;
static std::vector<gpHTMatrix> GFRAMES;
// static p3d_vector3 CENTER= {0.02,-0.05,0.1};
// static double RADIUS= 0.07;
static gpKdTree KDTREE;
// static gpKdTreeTris *KDTREETRIS= NULL;
static int LEVEL= 0;
static gpGrasp GRASP;   // the current grasp
static gpDoubleGrasp DOUBLEGRASP;
static std::list<gpPlacement> POSELIST, POSELIST2;
static gpPlacement POSE;
static bool LOAD_LIST= false;
//static double DMAX_FAR= 0.05;
static double DMAX_NEAR= 0.003;
static p3d_vector3 Oi= {0.0,0.0,0.0}, Of= {0.0,0.0,0.0};
static p3d_vector3 Ai= {0.0,0.0,0.0}, Af= {0.0,0.0,0.0}, Bi= {0.0,0.0,0.0}, Bf= {0.0,0.0,0.0};
// static p3d_vector3 E= {0.0,0.0,0.0};
static bool GRID= false;

// static unsigned int CNT= 0;
static configPt *PATH= NULL;
static int NB_CONFIGS= 0;

#ifdef MULTILOCALPATH
static char OBJECT_GROUP_NAME[256]="jido-ob_lin"; // "jido-ob"; //
#endif
static bool INIT_IS_DONE= false;

void draw_trajectory ( configPt* configs, int nb_configs );
void draw_grasp_planner();
void dynamic_grasping();
void contact_points();
void key1();
void key2();

static gpConvexHull3D *chull= NULL;

/* --------- FORM VARIABLES ------- */
FL_FORM  * GRASP_PLANNING_FORM = NULL;
static FL_OBJECT * MOTIONGROUP;
static FL_OBJECT * BT_GRASP_OBJ;
static FL_OBJECT * BT_1_OBJ;
static FL_OBJECT * BT_2_OBJ;
static FL_OBJECT * BT_3_OBJ;
static FL_OBJECT * BT_4_OBJ;
static FL_OBJECT * BT_5_OBJ;
static FL_OBJECT * BT_DISPLAY_GRASPS_OBJ;
static FL_OBJECT * BT_LOAD_GRASP_LIST_OBJ;
/* ------------------------------------------ */


/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_grasp_planning_group ( void );
static void CB_grasp_planner_obj ( FL_OBJECT *obj, long arg );
//static void CB_1_obj ( FL_OBJECT *obj, long arg );
static void CB_gripper_obj ( FL_OBJECT *obj, long arg );
static void CB_SAHandRight_obj ( FL_OBJECT *obj, long arg );
static void CB_SAHandLeft_obj ( FL_OBJECT *obj, long arg );
static void CB_double_grasp_obj ( FL_OBJECT *obj, long arg );
static void CB_test_obj ( FL_OBJECT *obj, long arg );
static void CB_display_grasps_obj ( FL_OBJECT *obj, long arg );
static void CB_load_grasp_list_obj ( FL_OBJECT *obj, long arg );
/* ------------------------------------------ */


/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_grasp_planning_form ( void )
{
	GRASP_PLANNING_FORM = fl_bgn_form ( FL_UP_BOX, 150, 440 );

	g3d_create_grasp_planning_group();
	fl_end_form();
}

void g3d_show_grasp_planning_form ( void )
{
	fl_show_form ( GRASP_PLANNING_FORM, FL_PLACE_SIZE, TRUE, "Grasp Planning" );
}

void g3d_hide_grasp_planning_form ( void )
{
	fl_hide_form ( GRASP_PLANNING_FORM );
}

void g3d_delete_grasp_planning_form ( void )
{
	fl_free_form ( GRASP_PLANNING_FORM );
}
/* ------------------------------------------ */



/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_grasp_planning_group ( void )
{
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe ( FL_ENGRAVED_FRAME, 5, 15, 140, 410, "Grasp planning" );

	MOTIONGROUP = fl_bgn_group();

	x= 15;
	y= 30;
	w= 120;
	h= 40;
	dy= h + 10;
	BT_GRASP_OBJ = fl_add_button ( FL_NORMAL_BUTTON, x, y, w, h, "Grasp planner" );
	BT_1_OBJ= fl_add_button ( FL_NORMAL_BUTTON, x, y + dy, w, h, "Gripper" );
	BT_2_OBJ= fl_add_button ( FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "SAHandRight" );
	BT_3_OBJ= fl_add_button ( FL_NORMAL_BUTTON, x, y + 3*dy, w, h, "SAHandLeft" );
	BT_4_OBJ = fl_add_button ( FL_NORMAL_BUTTON, x, y + 4*dy, w, h, "Double Grasp" );
	BT_5_OBJ  = fl_add_button ( FL_NORMAL_BUTTON, x, y + 5*dy, w, h, "Test" );
	BT_DISPLAY_GRASPS_OBJ  = fl_add_button ( FL_RADIO_BUTTON, x, y + 6*dy, w, h, "Display grasps" );
	BT_LOAD_GRASP_LIST_OBJ  = fl_add_button ( FL_RADIO_BUTTON, x, y + 7*dy, w, h, "Load grasp list" );

	fl_set_call_back ( BT_GRASP_OBJ, CB_grasp_planner_obj, 1 );
	fl_set_call_back ( BT_1_OBJ, CB_gripper_obj, 2 );
	fl_set_call_back ( BT_2_OBJ, CB_SAHandRight_obj, 3 );
	fl_set_call_back ( BT_3_OBJ, CB_SAHandLeft_obj, 1 );
	fl_set_call_back ( BT_4_OBJ, CB_double_grasp_obj, 1 );
	fl_set_call_back ( BT_5_OBJ, CB_test_obj, 1 );
	fl_set_call_back ( BT_DISPLAY_GRASPS_OBJ, CB_display_grasps_obj, 1 );
	fl_set_object_color ( BT_DISPLAY_GRASPS_OBJ,FL_MCOL,FL_GREEN );
	fl_set_button ( BT_DISPLAY_GRASPS_OBJ, FALSE );
	fl_set_call_back ( BT_LOAD_GRASP_LIST_OBJ, CB_load_grasp_list_obj, 1 );
	fl_set_object_color ( BT_LOAD_GRASP_LIST_OBJ,FL_MCOL,FL_GREEN );
	fl_set_button ( BT_LOAD_GRASP_LIST_OBJ, FALSE );

	fl_end_group();
}

void draw_trajectory ( configPt* configs, int nb_configs )
{
	int i;

	if ( configs==NULL || nb_configs<=0 )
		{  return;  }

	g3d_set_color ( Red, NULL );
	for ( i=0; i<nb_configs; i++ )
		{  g3d_draw_solid_sphere ( configs[i][6], configs[i][7], 1.0, 0.07, 10 );  }

	g3d_set_color ( Green, NULL );
	glBegin ( GL_LINES );
	for ( i=1; i<nb_configs; i++ )
	{
		glVertex3d ( configs[i-1][6], configs[i-1][7], 1 );
		glVertex3d ( configs[i][6], configs[i][7], 1 );
	}
	glEnd();
}


void redraw()
{
  g3d_win *win= NULL;

  win= g3d_get_cur_win();
  win->fct_draw2= & ( draw_grasp_planner );
  win->fct_key1= & ( key1 );
  win->fct_key2= & ( key2 );

  win->vs.fov= 30;
  g3d_set_projection_matrix(win->vs.projection_mode);

  g3d_draw_allwin();
  g3d_draw_allwin_active();
}

int init_graspPlanning ( char *objectName )
{
// 	int i;

	if ( p3d_col_get_mode() !=p3d_col_mode_pqp )
	{
		printf ( "The collision detector MUST be PQP to use graspPlanning module.\n" );
		printf ( "The graspPlanning module will not work.\n" );
		return GP_ERROR;
	}

	ROBOT= p3d_get_robot_by_name ( (char *)GP_ROBOT_NAME );

	HAND_ROBOT= NULL;

	//HAND_ROBOT= gpFind_hand_robot(HAND_PROP);

	HAND_ROBOT= HAND_PROP.initialize();

	if ( ROBOT==NULL )
	{
		printf ( "A robot named \"%s\" is required for some computations.\n", GP_ROBOT_NAME );
//     return GP_ERROR;
	}
	if ( HAND_ROBOT==NULL )
	{
		printf ( "There is no robot corresponding to one of the defined hand robots.\n" );
		printf ( "The graspPlanning module will not work.\n" );
		return GP_ERROR;
	}


	OBJECT= p3d_get_robot_by_name ( objectName );

	if ( OBJECT==NULL )
	{
		printf ( "%s: %d: There is no robot (object) with name \"%s\".\n", __FILE__, __LINE__, objectName );
		return GP_ERROR;
	}

	POLYHEDRON= OBJECT->o[0]->pol[0]->poly;
	poly_build_planes ( POLYHEDRON );

// 	Mass_properties mass_prop;
// 	gpCompute_mass_properties ( POLYHEDRON, &mass_prop );
// 	gpCompute_inertia_axes ( &mass_prop, IAXES );
// 	p3d_vectCopy ( mass_prop.r, CMASS );
// 	gpInertia_AABB ( POLYHEDRON, CMASS, IAXES, IAABB );
        gpCompute_mass_properties(POLYHEDRON);


	printf ( "center of mass: \n\t %f %f %f\n", POLYHEDRON->cmass[0], POLYHEDRON->cmass[1], POLYHEDRON->cmass[2]);
	printf ( "volume: \n\t %f\n", POLYHEDRON->volume);
	printf ( "inertia axes: \n\t %f %f %f \n", POLYHEDRON->inertia_axes[0][0], POLYHEDRON->inertia_axes[0][1], POLYHEDRON->inertia_axes[0][2] );
	printf ( "\t %f %f %f \n", POLYHEDRON->inertia_axes[1][0], POLYHEDRON->inertia_axes[1][1], POLYHEDRON->inertia_axes[1][2] );
	printf ( "\t %f %f %f \n", POLYHEDRON->inertia_axes[2][0], POLYHEDRON->inertia_axes[2][1], POLYHEDRON->inertia_axes[2][2] );

	return GP_OK;
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
// chull->draw();
  G3D_Window *win;
//   p3d_rob *hand_robot= p3d_get_robot_by_name((char*)(GP_GRIPPER_ROBOT_NAME));
  p3d_rob *hand_robot= p3d_get_robot_by_name((char*)(GP_SAHAND_RIGHT_ROBOT_NAME));
  p3d_rob *object_robot= p3d_get_robot_by_name(ObjectName);

  win = g3d_get_cur_win();

  //display all the grasps from the list:
  if(display_grasps)
  {
    for(std::list<gpGrasp>::iterator iter= GRASPLIST.begin(); iter!=GRASPLIST.end(); ++iter)
    { ( *iter ).draw ( 0.005 );    }
  }

  glPushAttrib(GL_ENABLE_BIT);
  glEnable(GL_LIGHTING);
  for(std::list<gpGrasp>::iterator iter= GRASPLIST.begin(); iter!=GRASPLIST.end(); ++iter)
  { 
    gpSet_robot_hand_grasp_configuration(hand_robot, object_robot, *iter);

    win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
    g3d_draw_robot(hand_robot->num, win);
  }
  glPopAttrib();



//   GRASP.draw(0.05);
//   DOUBLEGRASP.draw(0.5);

  return;

//dynamic_grasping(); 

//return;
int i= 0;
double color[4];
glShadeModel(GL_SMOOTH);
glPushMatrix();
    glTranslatef(-4,0,0);
  for ( std::list<gpPlacement>::iterator iter= POSELIST.begin(); iter!=POSELIST.end(); iter++ )
  {
    glTranslatef(0.15,0,0);
    g3d_rgb_from_int(i++, color); 
    glColor4dv(color);
    ( *iter ).draw ( 0.03 );
  }
glPopMatrix();

return;

p3d_rob *object= p3d_get_robot_by_name("Ball");
static p3d_vector3 exchange= {0,0,0};
if(GRID==true)
{
glColor3f(1,0,1);
g3d_draw_solid_sphere(exchange[0], exchange[1], exchange[2], 0.05, 20);
findBestExchangePositionGraphic(object, Oi, Of, Ai, Af, Bi, Bf, exchange); return;
}


//    chull->draw();
return;
   int draw(bool wireframe= false);
	int cnt2= 0;
	for ( std::list<gpPlacement>::iterator iter= POSELIST.begin(); iter!=POSELIST.end(); iter++ )
	{ if(cnt2==0)
		( *iter ).draw ( 0.03 );
		cnt2++;
	}
return;




/*
glColor3f(1,0,0);
g3d_draw_solid_sphere(Oi[0], Oi[1], Oi[2], 0.1, 10);
glColor3f(0,1,0);
g3d_draw_solid_sphere(Of[0], Of[1], Of[2], 0.1, 10);


glColor3f(1,0,0);
g3d_draw_solid_sphere(Ai[0], Ai[1], Ai[2], 0.1, 10);
glColor3f(0,1,0);
g3d_draw_solid_sphere(Af[0], Af[1], Af[2], 0.1, 10);

glColor3f(1,0,0);
g3d_draw_solid_sphere(Bi[0], Bi[1], Bi[2], 0.1, 10);
glColor3f(0,1,0);
g3d_draw_solid_sphere(Bf[0], Bf[1], Bf[2], 0.1, 10);
*/

// dynamic_grasping();
return;

gpHand_properties prop;
prop.initialize(GP_SAHAND_RIGHT);
// prop.draw(p3d_mat4IDENTITY);
gpDraw_reachable_points((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), (p3d_rob *)p3d_get_robot_by_name("Horse"), prop);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), prop, 1);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), prop, 2);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), prop, 3);
// gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), prop, 4);
return;
gpSAHandInfo data;
gpDraw_SAHfinger_outer_workspace(data, 0.1);
return;



p3d_rob *jido= p3d_get_robot_by_name("HRP2TABLE");
p3d_rob *GREY_TAPE= p3d_get_robot_by_name("GREY_TAPE");
  p3d_vector3 a, b, amin, bmin;
p3d_col_robot_environment_distance(GREY_TAPE, a, b);

glColor3f(1,0,0);
g3d_drawSphere(a[0], a[1], a[2], 0.02);
g3d_drawSphere(b[0], b[1], b[2], 0.02);
g3d_drawOneLine(a[0], a[1], a[2],b[0], b[1], b[2], Red, NULL);

double d, dmin= 1e6;
for(int i=0; i<XYZ_ENV->nr; ++i)
{
  if( XYZ_ENV->robot[i]==GREY_TAPE || XYZ_ENV->robot[i]==jido )
  {  continue;  }
  d=  p3d_col_robot_robot_distance(GREY_TAPE, XYZ_ENV->robot[i], a, b);
  if(d < dmin) 
  { 
   dmin= d;
   p3d_vectCopy(a, amin);
   p3d_vectCopy(b, bmin);
 }
}
g3d_drawOneLine(amin[0], amin[1], amin[2],bmin[0], bmin[1], bmin[2], Green, NULL);
// d= MIN(distToEnv,distToRobots);
// d= distToRobots;


return;
double result;
// p3d_matrix4 camera_frame, T1, T2, Tinv;
// p3d_get_freeflyer_pose(objet, camera_frame);
p3d_jnt * tilt= NULL;
tilt= p3d_get_robot_jnt_by_name(jido, (char*) "Tilt");


// g3d_win *wiin= g3d_get_cur_win();


// wiin->vs.cullingEnabled= 1;
// g3d_set_camera_parameters_from_frame(tilt->abs_pos, wiin->vs);

g3d_does_robot_hide_object(tilt->abs_pos, 60, jido, GREY_TAPE, &result);

 return;

//dynamic_grasping();
//contact_points(); return;


   // g3d_screenshot();
	return;
	
/*	
 GtsCartesianGrid g;
 GtsSurface * surface;
 gdouble iso= 0.05;
 gboolean verbose = FALSE, tetra = FALSE, dual = FALSE;
 GtsIsoCartesianFunc func = cost;
 gpointer data;
//cost(gdouble ** f, GtsCartesianGrid g, guint k, gpointer data)
 
 data= XYZ_ENV->cur_robot;
 // interval is [-10:10][-10:10][-10:10]
 g.nx= g.ny= g.nz= 30;
 g.x = -0.2; g.dx = 0.4/(gdouble) (g.nx - 1);
 g.y = -0.3; g.dy = 0.6/(gdouble) (g.ny - 1);
 g.z = 0.0; g.dz = 0.5/(gdouble) (g.nz - 1);


iso= ((gdouble) LEVEL -1 )/450.0;
printf("iso= %f\n",iso);
 surface= gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(), gts_vertex_class());

 gts_isosurface_cartesian (surface, g, func, data, iso);

  glColor3f(0.0, 1.0, 1.0);
  glBegin(GL_TRIANGLES);
  gts_surface_foreach_face(surface, (GtsFunc) draw_face_GTS, NULL);
  glEnd();

  gts_object_destroy(GTS_OBJECT(surface));

return;
*/

p3d_polyhedre *poly= NULL;
p3d_rob *horse= p3d_get_robot_by_name("Horse");
poly= horse->o[0]->pol[0]->poly;
g3d_draw_p3d_polyhedre(poly);
//p3d_coarsen_surface_GTS(poly, 700);
//p3d_draw_surface_GTS(poly);
glPushMatrix();
glTranslatef(0.0, 0.0, 0.2);
p3d_draw_mean_curvature_GTS(poly);
glPopMatrix();
// gpHand_properties handData;
// handData.initialize(GP_SAHAND_RIGHT);
//   gpDraw_workspace_object_intersection((p3d_rob *)p3d_get_robot_by_name("Horse"), (p3d_rob *)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), handData);

//   static int count= 1;
//   char filename[128];
//   sprintf(filename, "/home/jpsaut/BioMove3Dgit/BioMove3D/video/screenshot-00000%d.ppm", count);
//   g3d_export_OpenGL_display(filename);
// GRASP.draw(0.05, 20);
// return;
//  g3d_draw_wire_ellipsoid(0.5, 2, 1);return;

// g3d_draw_ellipsoid(1, 2, 3, 30); return;

// gpHand_properties handData;
// handData.initialize(GP_SAHAND_RIGHT);
//   gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), handData, 1);
//   gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), handData, 2);
//   gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), handData, 3);
//   gpDraw_SAHfinger_manipulability_ellipsoid((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), handData, 4);


// glEnable(GL_SMOOTH);
//   p3d_rob *horse= p3d_get_robot_by_name("Horse");
//   g3d_draw_p3d_polyhedre(horse->o[0]->pol[0]->poly); return;
  



return;
// gpHand_properties data;
// p3d_matrix4 frame;
// data.initialize(GP_SAHAND_RIGHT);
// p3d_rob *hand1= p3d_get_robot_by_name("SAHandRight_robot");
// if(hand1!=NULL) gpGet_wrist_frame(hand1, frame);
// data.draw(frame);
// GRASP.draw(0.03);

if(LEVEL<21)
  KDTREE.draw(LEVEL);
return;


// gpHand_properties handData;
// handData.initialize(GP_SAHAND_RIGHT);
// p3d_matrix4 Tg;
// p3d_mat4Copy(p3d_mat4IDENTITY, Tg);
// Tg[2][3]= 2; 
// handData.draw(Tg);

//   g3d_set_color(Red, NULL);
//   g3d_draw_solid_sphere(Oi[0],Oi[1],Oi[2], 0.08, 10);
//   g3d_set_color(Green, NULL);
//   g3d_draw_solid_sphere(Of[0],Of[1],Of[2], 0.08, 20);


//   g3d_set_color(Red, NULL);
//   g3d_draw_solid_sphere(Ai[0],Ai[1],Ai[2], 0.08, 10);
//   g3d_draw_solid_sphere(Bi[0],Bi[1],Bi[2], 0.08, 10);
// 
//   g3d_set_color(Green, NULL);
//   g3d_draw_solid_sphere(Af[0],Af[1],Af[2], 0.08, 20);
//   g3d_draw_solid_sphere(Bf[0],Bf[1],Bf[2], 0.08, 20);

  g3d_set_color(Violet, NULL);
//   g3d_draw_solid_sphere(E[0],E[1],E[2], 0.08, 20);


  return;

//   p3d_obj *obj = p3d_get_obst_by_name("test");
//   p3d_rob *rob = p3d_get_robot_by_name("Horse");
glPushMatrix();
glTranslatef(0,0,0.6);
  g3d_draw_p3d_polyhedre(XYZ_ENV->cur_robot->o[0]->pol[0]->poly);
glPopMatrix();
   GRASP.draw ( 0.03 );

return;


	for ( unsigned int i=0; i<GFRAMES.size(); i++ )
	{
		GFRAMES[i].draw();
	}

	p3d_rob *robot= p3d_get_robot_by_name ( (char *)GP_OBJECT_NAME_DEFAULT );
// 	p3d_obj * object= p3d_get_robot_body_by_name ( robot, (char *)GP_OBJECT_NAME_DEFAULT );
//   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//    g3d_draw_p3d_polyhedre(object->pol[0]->poly);
//   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	p3d_matrix4 T;
	p3d_get_body_pose ( robot, 0, T );
	g3d_draw_frame ( T,0.2 );


	return;



	glEnable ( GL_LIGHTING );
//   if(KDTREE!=NULL) KDTREE->draw(LEVEL);
// 	if ( KDTREETRIS!=NULL ) KDTREETRIS->draw ( LEVEL );
//   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
// g3d_draw_solid_sphere(CENTER[0],CENTER[1],CENTER[2],RADIUS, 45);
//   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glPopMatrix();
	return;

// 	p3d_jnt *jnt= NULL;

	//g3d_draw_robot_joints((p3d_rob*)(p3d_get_desc_curid(P3D_ROBOT)), 0.1);


	gpDraw_inertia_AABB ( CMASS, IAXES, IAABB );

	glPushMatrix();
	glTranslatef ( 0, 0, 3 );
	if ( POLYHEDRON!=NULL )
		g3d_draw_p3d_polyhedre ( POLYHEDRON );
	glPopMatrix();

	return;


	int cnt= 0;
	for ( std::list<gpPlacement>::iterator iter= POSELIST.begin(); iter!=POSELIST.end(); iter++ )
	{
//     if(cnt==1)
		( *iter ).draw ( 0.03 );
//break;
		cnt++;
	}

	static bool firstTime= true;
	if ( firstTime )
	{
// 		gpFind_poses_on_object ( OBJECT, p3d_get_obst_by_name ( (char *)"box7" ), POSELIST, 0.05, 15, POSELIST2 );
// 		printf ( "%d new poses\n", POSELIST2.size() );
// 		firstTime= false;
	}

	cnt= 0;
	for ( std::list<gpPlacement>::iterator iter= POSELIST2.begin(); iter!=POSELIST2.end(); iter++ )
	{
		( *iter ).draw ( 0.03 );
		cnt++;
		if ( cnt>200 )
		{
			printf ( "only the first 200 poses are displayed\n" );
			break;
		}
	}
}

void draw_test()
{

}

void key1()
{
//   if(LEVEL <= KDTREE.depth())
  LEVEL++;
  printf("LEVEL= %d\n", LEVEL);
}

void key2()
{
  //if(LEVEL>0)
  LEVEL--;
  printf("LEVEL= %d\n", LEVEL);
}


static void CB_grasp_planner_obj ( FL_OBJECT *obj, long arg )
{
  int result;
  p3d_vector3 objectCenter;
  p3d_matrix4 objectPose;
  g3d_win *win= NULL;
  p3d_rob *robot= NULL, *object= NULL;
  p3d_rob *hand_robot= NULL;
  gpHand_properties handProp;
  std::list<gpGrasp>::iterator igrasp;

  result= gpGet_grasp_list_SAHand(ObjectName, 1, GRASPLIST);
//   result= gpGet_grasp_list_gripper(ObjectName, GRASPLIST);
  igrasp= GRASPLIST.begin();
   while(igrasp!=GRASPLIST.end()) {
   if( igrasp->areContactsTooCloseToEdge(30*DEGTORAD, 0.02) ) {
     igrasp= GRASPLIST.erase(igrasp);
     continue;
   }
   igrasp++;
  }

  if(result==GP_ERROR)
  {  return;  }

  object= p3d_get_robot_by_name(ObjectName);
  if(object==NULL)
  {  
    printf("%s: %d: there is no robot named \"%s\" \n", __FILE__, __LINE__, ObjectName);
    return;
  }

  robot= p3d_get_robot_by_name(RobotName);
  if(robot==NULL)
  {  
    printf("%s: %d: there is no robot named \"%s\" \n", __FILE__, __LINE__, RobotName);
    return;
  }

 // hand_robot= p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
  hand_robot= p3d_get_robot_by_name(HandRobotName);
  if(hand_robot==NULL)
  {  
    printf("%s: %d: there is no robot named \"%s\" \n", __FILE__, __LINE__, HandRobotName);
    return;
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
  gpCompute_mass_properties(object->o[0]->pol[0]->poly);

  objectCenter[0]= objectPose[0][3] + object->o[0]->pol[0]->poly->cmass[0];
  objectCenter[1]= objectPose[1][3] + object->o[0]->pol[0]->poly->cmass[1];
  objectCenter[2]= objectPose[2][3] + object->o[0]->pol[0]->poly->cmass[2];

  win= g3d_get_cur_win();
  win->vs.x= objectCenter[0];   win->vs.y= objectCenter[1];   win->vs.z= objectCenter[2];

  if(GRASPLIST.empty())
  {
    printf ( "No grasp was found.\n" );
    return;
  }

/*
  i= 0;
  for ( igrasp=GRASPLIST.begin(); igrasp!=GRASPLIST.end(); igrasp++ )
  {
    GRASP= ( *igrasp );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>GRASPLIST.size() )
  {  count= 1;  }*/
  //p3d_release_object(robot);

//////////////////////////////////////////////////////é
//   p3d_vector3 wrist_direction;
  p3d_vector3 verticalAxis;
//   p3d_matrix4 gframe_object, gframe_world;


  verticalAxis[0]= 0.0;
  verticalAxis[1]= 0.0;
  verticalAxis[2]= -1.0;

//   igrasp= GRASPLIST.begin();
//   while(igrasp!=GRASPLIST.end())
//   {
//     p3d_mat4Copy(igrasp->frame, gframe_object);
//     p3d_mat4Mult(objectPose, gframe_object, gframe_world); //passage repere objet -> repere monde
//     p3d_mat4ExtractColumnZ(gframe_world, wrist_direction);
//     p3d_vectNormalize(wrist_direction, wrist_direction);
// 
//     igrasp->quality= p3d_vectDotProd(wrist_direction, verticalAxis);
//     igrasp++;
//   }
//   GRASPLIST.sort();
//   GRASPLIST.reverse();
//////////////////////////////////////////////////////


  //find a configuration for the whole robot (mobile base + arm):
  configPt qcur= NULL, qgrasp= NULL, qend= NULL;
  qcur= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &qcur);
  handProp.initialize(GRASPLIST.front().hand_type);
//   p3d_vector3 axis;
//   p3d_matrix4 T;
//   p3d_mat4Copy(p3d_mat4IDENTITY, T);
// 
//   axis[0]= 0;
//   axis[1]= 0;
//   axis[2]= 1;
//   p3d_mat4Rot(T, axis, 5*M_PI/8.0);
//   T[0][3]= 0;
//   T[1][3]= 0;
//   T[2][3]= -0.268;
//   handProp.setThand_wrist(T);
  handProp.setArmType(ARM_TYPE);

  p3d_col_deactivate_robot(hand_robot);

  qgrasp= p3d_get_robot_config(robot);

  if(robot!=NULL)
  {
    for(int i=0; i<50; ++i)
    {
//         qgrasp= gpRandom_robot_base(robot, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter);

        if(qgrasp==NULL)
        {  break;  }

        qend= NULL;
        qend= gpFind_grasp_from_base_configuration(robot, object, GRASPLIST, ARM_TYPE, qgrasp, GRASP, handProp);

        if(qend!=NULL)
        { 
          p3d_set_and_update_this_robot_conf(robot, qend);
          p3d_copy_config_into(robot, qend, &robot->ROBOT_POS);
          p3d_destroy_config(robot, qend);
          qend= NULL;
          break;
        }
        p3d_destroy_config(robot, qgrasp);
        qgrasp= NULL;
   }
/*
   if ( i==250 )
   {  printf ( "No platform configuration was found.\n" );  }
   else
   {  printf ( "Grasp planning was successfull.\n" );  }*/
  }

//   gpSet_robot_hand_grasp_configuration(SAHandRight_robot, object, GRASP);

//   XYZ_ENV->cur_robot= cur_robot;

  //p3d_set_object_to_carry(robot, "Horse");
  //p3d_grab_object(robot, 0);

  redraw();

  return;
}




static void CB_gripper_obj ( FL_OBJECT *obj, long arg )
{ printf("CB_gripper\n");
  unsigned int i;
  static unsigned int count= 0, firstTime= TRUE;
  
  if(firstTime)
  {
    firstTime= 0;
    gpGet_grasp_list_gripper(ObjectName, GRASPLIST);
    gpReduce_grasp_list_size(GRASPLIST, GRASPLIST, 35);
  }

  i= 0;
  for (std::list<gpGrasp>::iterator iter=GRASPLIST.begin(); iter!=GRASPLIST.end(); iter++ )
  {
    GRASP= ( *iter );
    i++;
    if ( i>count )
    {  break; }
  }
  count++;
  if ( count>GRASPLIST.size() )
  {  count= 0;  }

  std::string robotName= GP_GRIPPER_ROBOT_NAME;

  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name((char*)(GP_GRIPPER_ROBOT_NAME)), (p3d_rob*)p3d_get_robot_by_name(ObjectName), GRASP);

  redraw();
  return;
}


static void CB_SAHandLeft_obj ( FL_OBJECT *obj, long arg )
{ printf("CB_SAHandLeft\n");
  unsigned int i;
  static unsigned int count= 0, firstTime= TRUE;
  
  if(firstTime)
  {
    firstTime= 0;
    gpGet_grasp_list_SAHand(ObjectName, 2, GRASPLIST);
  }

  i= 0;
  for (std::list<gpGrasp>::iterator iter=GRASPLIST.begin(); iter!=GRASPLIST.end(); iter++ )
  {
    GRASP= ( *iter );
    i++;
    if ( i>count )
    {  break; }
  }
  count++;
  if( count>GRASPLIST.size() )
  {  count= 0;  }

  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name(GP_SAHAND_LEFT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name(ObjectName), GRASP);

  redraw();
  return;
}


static void CB_SAHandRight_obj ( FL_OBJECT *obj, long arg )
{ 
printf("CB_SAHandRight\n");
  unsigned int i;
  static unsigned int count= 0, firstTime= TRUE;
  
  if(firstTime)
  {
    firstTime= 0;
    gpGet_grasp_list(ObjectName, GP_SAHAND_RIGHT, GRASPLIST);
    gpReduce_grasp_list_size(GRASPLIST, GRASPLIST, 7);
  }

  i= 0;
  for (std::list<gpGrasp>::iterator iter=GRASPLIST.begin(); iter!=GRASPLIST.end(); iter++ )
  {
    GRASP= ( *iter );
    i++;
    if ( i>count )
    {  break; }
  }
  count++;
  if ( count>GRASPLIST.size() )
  {  count= 0;  }

  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name(ObjectName), GRASP);

  redraw();
  return;

/*
	bool needs_to_move, so_far_so_good= true;
	int result, path_found;
	double x, y, theta, q1, q2, q3, q4, q5, q6;
	std::vector<double> qhand;
	configPt qstart= NULL, qfinal= NULL, qinter1= NULL, qinter2= NULL, qinter3= NULL, qfar= NULL;
	p3d_rob *robotPt= NULL;
	p3d_cntrt* cntrt_arm = NULL;
	robotPt= p3d_get_robot_by_name ((char *) GP_ROBOT_NAME );
	XYZ_ENV->cur_robot= robotPt;

	// initializes everything:
	GP_Init ( (char *)GP_OBJECT_NAME_DEFAULT );

	redraw();

	cntrt_arm= GP_GetArmCntrt ( robotPt );

	if ( cntrt_arm==NULL )
	{
		printf ( "FATAL_ERROR : arm_IK constraint does not exist\n" );
		return;
	}

	// Deactivate the arm_IK constrint
	p3d_desactivateCntrt ( robotPt, cntrt_arm );

	//alloc all configs:
	qstart= p3d_alloc_config ( robotPt );
	qfinal= p3d_alloc_config ( robotPt );
	qinter1= p3d_alloc_config ( robotPt );
	qinter2= p3d_alloc_config ( robotPt );
	qinter3= p3d_alloc_config ( robotPt );
	qfar= p3d_alloc_config ( HAND_ROBOT );

	p3d_get_robot_config_into ( robotPt, &qstart );
#ifdef LIGHT_PLANNER
	p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qstart);
#endif
	g3d_draw_allwin_active();

	// computes the grasp list:
	if ( !LOAD_LIST )
	{
		result= GP_ComputeGraspList ( (char *)GP_OBJECT_NAME_DEFAULT );
		gpSave_grasp_list ( GRASPLIST, "./graspPlanning/graspList_new.xml" );
	}
	else // or loads it:
	{
		result= gpLoad_grasp_list ( "./graspPlanning/graspList.xml", GRASPLIST );
		if ( result==0 )
		{
			printf ( "Can not load a grasp list.\n" );
			return;
		}

		if ( !GRASPLIST.empty() )
		{
			if ( GRASPLIST.front().hand_type!=HAND_PROP.type )
			{
				printf ( "The loaded grasp list does not correspond to the current hand type.\n" );
				return;
			}
		}
	}


	// move away the hand robot:
	qfar= p3d_alloc_config ( HAND_ROBOT );
	qfar[7]= -100; //to put the hand far under the floor
	qfar[8]= -1; //to put the hand far under the floor
	p3d_set_and_update_this_robot_conf ( HAND_ROBOT, qfar );
	p3d_destroy_config ( HAND_ROBOT, qfar );

	qfinal= GP_FindGraspConfig ( needs_to_move );

	p3d_set_and_update_this_robot_conf ( robotPt, qstart );
	if ( p3d_col_test() )
	{
		printf ( "Start configuration is colliding.\n" );
		return;
	}


	if ( qfinal!=NULL )
	{
#ifdef LIGHT_PLANNER
	p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qfinal);
#endif
		printf ( "Grasp configuration list was successfully computed.\n" );
		XYZ_ENV->cur_robot= robotPt;
		p3d_set_ROBOT_GOTO ( qfinal );
	}
	else
	{
		printf ( "No grasp configuration was found.\n" );
		so_far_so_good= false;
		redraw();

		//free all configs:
		p3d_destroy_config ( robotPt, qstart );
		p3d_destroy_config ( robotPt, qfinal );
		p3d_destroy_config ( robotPt, qinter1 );
		p3d_destroy_config ( robotPt, qinter2 );
		p3d_destroy_config ( robotPt, qinter3 );
		return;
	}

	p3d_set_and_update_this_robot_conf ( robotPt, qfinal );
	if ( p3d_col_test() )
	{
		printf ( "Final configuration is colliding.\n" );
		return;
	}

	redraw();

	//if the robot needs to move, we have to introduce three intermediate configurations:
	//    qstart =  (qbase0 ; qarm0 ; qhand0)
	// -> qinter1=  (qbase0 ; qarm_folded ; qhand0)
	// -> qinter2=  (qbase1 ; qarm_folded ; qhand0)
	// -> qinter3=  (qbase1 ; qarm1 ; qhand0)
	// -> qfinal =  (qbase1 ; qarm1 ; qhand1)
	if ( needs_to_move )
	{
		// get platform final configuration and arm final configuration:
		p3d_set_and_update_this_robot_conf ( robotPt, qfinal );
		gpGet_platform_configuration ( robotPt, x, y, theta );
		gpGet_arm_configuration ( robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6 );
		gpGet_hand_configuration ( robotPt, HAND_PROP, 0, qhand );

		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		if ( HAND_PROP.type==GP_GRIPPER ) gpOpen_hand ( robotPt, HAND_PROP );
		result= gpFold_arm ( robotPt, ARM_TYPE );
		if ( result==0 )
		{
			printf ( "The arm can not be folded.\n" );
		}

		p3d_get_robot_config_into ( robotPt, &qinter1 );
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qinter1);
#endif

		gpSet_platform_configuration ( robotPt, x, y, theta );
		p3d_get_robot_config_into ( robotPt, &qinter2 );
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qinter2);
#endif

		gpSet_arm_configuration ( robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6 );
		p3d_get_robot_config_into ( robotPt, &qinter3 );
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qinter3);
#endif

		if ( p3d_col_test() )
		{
			printf ( "The robot can not open its hand/gripper in its final arm and base configuration.\n" );
			p3d_copy_config_into ( robotPt, qfinal, &qinter3 );
		}


		printf ( "qstart    qinter1    qinter2    qinter3    qfinal\n" );
		for ( int i=6; i<robotPt->nb_dof; i++ )
		{
			printf ( "%f %f %f %f %f\n", qstart[i], qinter1[i], qinter2[i], qinter3[i], qfinal[i] );
		}

		//test all the intermediate configurations:
		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		g3d_draw_allwin_active();
		if ( p3d_col_test() ) // if collision
		{
			printf ( "The start configuration is in collision.\n" );
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qinter1 );
		g3d_draw_allwin_active();
		if ( p3d_col_test() ) // if collision
		{
			printf ( "The arm can not be folded without collision.\n" );
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qinter2 );
		g3d_draw_allwin_active();
		if ( p3d_col_test() ) // if collision
		{
			printf ( "qinter2 is colliding.\n" );
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qinter3 );
		g3d_draw_allwin_active();
		if ( p3d_col_test() ) // if collision
		{
			printf ( "qinter3 is colliding.\n" );
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qfinal );
		g3d_draw_allwin_active();
		if ( p3d_col_test() ) // if collision
		{
			printf ( "qfinal is colliding.\n" );
			goto END_GO_AND_GRASP;
		}


		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		p3d_copy_config_into ( robotPt, qstart, & ( robotPt->ROBOT_POS ) );
		p3d_activateCntrt ( robotPt, cntrt_arm );
		g3d_draw_allwin_active();
		p3d_set_ROBOT_START ( qstart );
		p3d_set_ROBOT_GOTO ( qinter1 );

		p3d_set_env_dmax ( DMAX_FAR );
#ifdef MULTILOCALPATH
		p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, "jido-hand", 1 ) ;
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, OBJECT_GROUP_NAME, 1 ) ;
#endif
		path_found= GP_FindPath();
		if ( !path_found )
		{
			printf ( "The planner could not find a path to fold the arm.\n" );
			so_far_so_good= false;
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qinter1 );
		p3d_copy_config_into ( robotPt, qinter1, & ( robotPt->ROBOT_POS ) );
		g3d_draw_allwin_active();

#ifdef LIGHT_PLANNER
		setAndActivateTwoJointsFixCntrt ( robotPt, robotPt->curObjectJnt, robotPt->baseJnt );
		p3d_desactivateCntrt ( robotPt, cntrt_arm );
#endif

		p3d_realloc_iksol ( robotPt->cntrt_manager );

		p3d_set_ROBOT_START ( qinter1 );
		p3d_set_ROBOT_GOTO ( qinter2 );

#ifdef MULTILOCALPATH
		p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, "jido-base", 1 ) ;
#endif
		path_found= GP_FindPath();
		if ( !path_found )
		{
			printf ( "The planner could not find a path to go to the object.\n" );
			so_far_so_good= false;
			goto END_GO_AND_GRASP;
		}
#ifdef LIGHT_PLANNER
		desactivateTwoJointsFixCntrt ( robotPt, robotPt->curObjectJnt, robotPt->baseJnt );
		p3d_desactivateCntrt ( robotPt, cntrt_arm );
#endif

		p3d_set_and_update_this_robot_conf ( robotPt, qinter2 );
		p3d_copy_config_into ( robotPt, qinter2, & ( robotPt->ROBOT_POS ) );
		p3d_copy_config_into ( robotPt, qinter3, & ( robotPt->ROBOT_GOTO ) );
#ifdef LIGHT_PLANNER
		p3d_activateCntrt ( robotPt, cntrt_arm );
#endif

		g3d_draw_allwin_active();
		p3d_set_ROBOT_START ( qinter2 );
		p3d_set_ROBOT_GOTO ( qinter3 );

		p3d_set_env_dmax ( DMAX_NEAR );
#ifdef MULTILOCALPATH
		p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, OBJECT_GROUP_NAME, 1 );
#endif
//     gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND_PROP);
		path_found= GP_FindPath();
		if ( !path_found )
		{
			printf ( "The planner could not find a path to reach the object with the arm.\n" );
			so_far_so_good= false;
			goto END_GO_AND_GRASP;
		}
		p3d_desactivateCntrt ( robotPt, cntrt_arm );

		p3d_set_and_update_this_robot_conf ( robotPt, qinter3 );
		p3d_copy_config_into ( robotPt, qinter3, & ( robotPt->ROBOT_POS ) );
		g3d_draw_allwin_active();
		p3d_set_ROBOT_START ( qinter3 );
		p3d_set_ROBOT_GOTO ( qfinal );
//     gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND_PROP);
#ifdef MULTILOCALPATH
		p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, "jido-hand", 1 );
#endif

		path_found= GP_FindPath();
		if ( !path_found )
		{
			printf ( "The planner could not find a path to close the robot's hand.\n" );
			so_far_so_good= false;
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		p3d_copy_config_into ( robotPt, qstart, & ( robotPt->ROBOT_POS ) );
#ifdef LIGHT_PLANNER
		p3d_activateCntrt ( robotPt, cntrt_arm );
#endif
		g3d_draw_allwin_active();

		GP_ConcateneAllTrajectories ( robotPt );
		robotPt->tcur= robotPt->t[0];
	}
	//if the robot does not need to move, we have to introduce one intermediate configurations:
	//    qstart =  (qbase0 ; qarm0 ; qhand0)
	// -> qinter1=  (qbase0 ; qarm1 ; qhand_inter=open)
	// -> qfinal =  (qbase0 ; qarm1 ; qhand1)
	else
	{
		// get arm final configuration:
		p3d_set_and_update_this_robot_conf ( robotPt, qfinal );
		g3d_draw_allwin_active();
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qfinal);
#endif
		gpGet_arm_configuration ( robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6 );

		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		gpOpen_hand ( robotPt, HAND_PROP );
		g3d_draw_allwin_active();
		gpSet_arm_configuration ( robotPt, ARM_TYPE, q1, q2, q3, q4, q5, q6 );
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qstart);
#endif
		p3d_get_robot_config_into ( robotPt, &qinter1 );
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qinter1);
#endif
		if ( p3d_col_test() )
		{
			printf ( "The robot can not open its hand/gripper in its final arm and base configuration.\n" );
			p3d_copy_config_into ( robotPt, qfinal, &qinter1 );
		}

		p3d_set_env_dmax ( DMAX_FAR );
		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		g3d_draw_allwin_active();
		p3d_set_ROBOT_START ( qstart );
		p3d_set_ROBOT_GOTO ( qinter1 );
#ifdef LIGHT_PLANNER
		p3d_activateCntrt ( robotPt, cntrt_arm );
#endif

		p3d_set_env_dmax ( DMAX_NEAR );
#ifdef MULTILOCALPATH
		p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, "jido-hand", 1 );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, OBJECT_GROUP_NAME, 1 );
#endif
//     gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND_PROP);
		path_found= GP_FindPath();
		if ( !path_found )
		{
			printf ( "The planner could not find a path to reach the object with the arm.\n" );
			so_far_so_good= false;
			goto END_GO_AND_GRASP;
		}
		p3d_set_and_update_this_robot_conf ( robotPt, qinter1 );
		g3d_draw_allwin_active();
		p3d_set_ROBOT_START ( qinter1 );
		p3d_set_ROBOT_GOTO ( qfinal );
//     gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND_PROP);
#ifdef MULTILOCALPATH
		p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
		p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, "jido-hand", 1 );
#endif
		path_found= GP_FindPath();
		if ( !path_found )
		{
			printf ( "The planner could not find a path to close the robot's hand.\n" );
			so_far_so_good= false;
			goto END_GO_AND_GRASP;
		}

		p3d_set_and_update_this_robot_conf ( robotPt, qstart );
		g3d_draw_allwin_active();
		GP_ConcateneAllTrajectories ( robotPt );
	}

	PATH= GP_GetTrajectory ( robotPt, robotPt->t[0], NB_CONFIGS );
	printf ( "path found: %d configs \n", NB_CONFIGS );

//   gpDeactivate_object_fingertips_collisions(robotPt, OBJECT, HAND_PROP);
	p3d_copy_config_into ( robotPt, qstart, & ( robotPt->ROBOT_POS ) );

END_GO_AND_GRASP:
	//free all configs:
	p3d_destroy_config ( robotPt, qstart );
	p3d_destroy_config ( robotPt, qfinal );
	p3d_destroy_config ( robotPt, qinter1 );
	p3d_destroy_config ( robotPt, qinter2 );
	p3d_destroy_config ( robotPt, qinter3 );

	if ( so_far_so_good )
		{ printf ( "ALL IS DONE: SUCCESS.\n" ); }
	else
		{ printf ( "ALL IS DONE: THERE WAS SOMETHING WRONG.\n" ); }

	return;
*/
}


static void CB_double_grasp_obj( FL_OBJECT *obj, long arg )
{
  printf("CB_double_grasp\n");

  static int unsigned firstTime= TRUE, count= 0;
  unsigned int i;
  gpHand_properties handProp;
  p3d_rob *hand1, *hand2, *object, *justin;
  std::list<gpGrasp> graspList1, graspList2;

  hand1= p3d_get_robot_by_name("JIDO_GRIPPER");
  hand2= p3d_get_robot_by_name("SAHandRight_robot");

  object= p3d_get_robot_by_name(ObjectName);
  justin= p3d_get_robot_by_name("ROBOT");

  if(firstTime)
  {  
   gpGet_grasp_list(ObjectName, GP_GRIPPER, graspList1);
   gpGet_grasp_list(ObjectName, GP_SAHAND_RIGHT, graspList2);


   gpReduce_grasp_list_size(graspList1, graspList1, 50);
   gpReduce_grasp_list_size(graspList2, graspList2, 50);

   gpDouble_grasp_generation(hand1, hand2, object, graspList1, graspList2, DOUBLEGRASPLIST);
   firstTime= false;  
   printf("%d double grasps\n",DOUBLEGRASPLIST.size());
  }

  std::list<gpDoubleGrasp>::iterator iter;
  i= 0;
  for ( iter=DOUBLEGRASPLIST.begin(); iter!=DOUBLEGRASPLIST.end(); iter++ )
  {
    DOUBLEGRASP= ( *iter );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>DOUBLEGRASPLIST.size() )
  {  count= 1;  }

  //gpCompute_grasp_open_config(justin, DOUBLEGRASP, object, 2);

  gpSet_robot_hand_grasp_configuration(hand1, object, DOUBLEGRASP.grasp1);
  gpSet_robot_hand_grasp_configuration(hand2, object, DOUBLEGRASP.grasp2);

  redraw();
}


static void CB_test_obj ( FL_OBJECT *obj, long arg )
{


// p3d_vector3 points[20];
// for(int i=0; i<20; ++i)
// {
//   points[i][0]= p3d_random(-1, 1);
//   points[i][1]= p3d_random(-1, 1);
//   points[i][2]= p3d_random(0.5, 1);
// }
// chull= new gpConvexHull3D(points, 20);
// chull->voronoi(true);
// redraw();
return;

// gpExport_bodies_for_coldman((p3d_rob *)p3d_get_robot_by_name("hrp2"));
gpExport_bodies_for_coldman(XYZ_ENV->cur_robot);
redraw(); return;
p3d_rob *mug= (p3d_rob *)p3d_get_robot_by_name("mug");
gpCompute_stable_placements(mug, POSELIST); 
std::list<p3d_rob*> robotList;
// gpFind_placements_on_object(mug, (p3d_rob *)p3d_get_robot_by_name("table"), robotList, POSELIST, 0.6, 1, 0.01, POSELIST2);



p3d_set_object_to_carry((p3d_rob*)p3d_get_robot_by_name("JIDO_ROBOT"), "Horse");
p3d_grab_object2((p3d_rob*)p3d_get_robot_by_name("JIDO_ROBOT"), 0);
redraw(); return;

g3d_win *curwin= g3d_get_cur_win();
g3d_print_win_camera(curwin->vs);
g3d_set_win_camera(curwin->vs, 0.426241, 0.067929, 0.728624, 3.216148, 2.872153, 0.879167, 0.000000, 0.000000, 1.000000);
// if(firstTime2==1) { firstTime2= 0;  redraw(); return; }
GRID= !GRID;

// static int first= 1;
// if(first) { redraw(); first= 0; return; }
// 

// chull= new gpConvexHull3D(POLYHEDRON->the_points, POLYHEDRON->nb_points);
// chull->compute(false, 0.0001, false);
// std::cout << "faces " << chull->nbFaces() << " poses " << POSELIST.size() << std::endl;
// redraw(); return;
//    chull->compute(false, -1, false);
// gpSAHandInfo data;
// std::vector<gpSphere> spheres;
// gpSAHfinger_workspace_approximation(data, DEGTORAD*2, 0.0, 50,  spheres);
//   redraw(); return;
//  static int firstTime= 1;
//  std::list<gpGrasp>::iterator igrasp;
//  if(firstTime)
//  {
//   gpGet_grasp_list_gripper("GREY_TAPE", GRASPLIST);
//   igrasp= GRASPLIST.begin();
//   while(igrasp!=GRASPLIST.end()) {
//    if( igrasp->areContactsTooCloseToEdge(30*DEGTORAD, 0.02) ) {
//      igrasp= GRASPLIST.erase(igrasp);
//      continue;
//    }
//    igrasp++;
//   }
//   std::cout << GRASPLIST.size() << std::endl;
//   firstTime= 0;
//   redraw();
//  }
//  else
//  {
//   gpReduce_grasp_list_size(GRASPLIST, GRASPLIST, 10);
//   std::cout << GRASPLIST.size() << std::endl;
//   redraw();
//  }
// 
//   return; 
// 
//   p3d_vector3 points[5000];
//   for(int i=0; i<5000; ++i)
//   {
// //     points[i][0]= p3d_random(-0.2, 0.2);
// //     points[i][1]= p3d_random(0.3, 0.6);
// //     points[i][2]= p3d_random(0.65, 0.95);
// 
//     points[i][0]= p3d_random(100, 350);
//     points[i][1]= p3d_random(350, 450);
//     points[i][2]= p3d_random(0, 0);
//   }
//   p3d_set_collision_cloud( points, 5000);
// 
// return;
// 
// redraw();
// return;

// g3d_win *win= NULL;
// win= g3d_get_cur_win();
// 
// p3d_export_robot_as_point_cloud(XYZ_ENV->cur_robot, 0.001, (char *)"SAHandRight_robot.hand.palm", NULL);
// p3d_export_robot_as_one_body(XYZ_ENV->cur_robot,XYZ_ENV->cur_robot->ROBOT_POS);
// // p3d_export_robot_as_point_cloud(XYZ_ENV->cur_robot, 0.001, (char *)"SAHandRight_robot.hand.finger2", NULL);
// // p3d_export_robot_as_point_cloud(XYZ_ENV->cur_robot, 0.001, (char *)"SAHandRight_robot.hand.finger3", NULL);
// // p3d_export_robot_as_point_cloud(XYZ_ENV->cur_robot, 0.001, (char *)"SAHandRight_robot.hand.finger4", NULL);
// 
// return;
// p3d_rob *jido= p3d_get_robot_by_name("JIDO_ROBOT");
// p3d_rob *objet= p3d_get_robot_by_name("BLACK_TAPE");
// double result;
// // p3d_matrix4 camera_frame, T1, T2, Tinv;
// // p3d_get_freeflyer_pose(objet, camera_frame);
// p3d_jnt * tilt= NULL;
// tilt= p3d_get_robot_jnt_by_name(jido, (char*) "Tilt");
// g3d_does_robot_hide_object(tilt->abs_pos, 60, jido, objet, &result);
// printf("result= %f\n", result);
// 
//  return;

// 	g3d_draw_allwin();
// 	g3d_draw_allwin_active();
// g3d_draw_env();

// GLfloat mat[16];
//p3d_to_gl_matrix(camera_frame,  mat);


// g3d_does_robot_hide_object(camera_frame, 60, jido, objet, &result);

//p3d_set_robot_display_mode(jido, P3D_ROB_GREEN_DISPLAY);

//p3d_export_as_OFF(mug->o[0]->pol[0]->poly);
// return;


// p3d_polyhedre *poly= NULL;
// p3d_rob *horse= p3d_get_robot_by_name("Horse");
// poly= horse->o[0]->pol[0]->poly;
// p3d_create_surface_GTS(poly);
// // p3d_compute_mean_curvature_GTS(poly);
// 
// redraw();
// return;
//  
//   win->vs.displayFrame= FALSE;
// pqp_deactivate_all_collisions();
// redraw(); return;

//   p3d_matrix3 M, U, V;
//   p3d_vector3 S;
// 
//   M[0][0]= 0.5;  M[0][1]= 0.8;   M[0][2]= -0.9; 
//   M[1][0]= 0.1;  M[1][1]= 0.1;   M[1][2]= 0.4; 
//   M[2][0]= 0.3;  M[2][1]= -0.7;  M[2][2]= 0.5; 
// 
//   p3d_mat3Print(M,"M");
//   p3d_mat3SVD(M, U, S, V);
//   p3d_mat3Print(U,"U");
//   p3d_mat3Print(V,"V");
//   printf("S %f %f %f\n",S[0],S[1],S[2]);
// return ;
// redraw(); return;
// gpSAHandInfo info;
// std::vector<gpSphere> spheres;
// gpSAHfinger_workspace_approximation(info, 1.0*DEGTORAD, 0.001, 30, spheres);

//    gpSample_obj_surface(((p3d_rob*)p3d_get_robot_by_name("Horse"))->o[0], 0.005, 0, CONTACTLIST);
//   KDTREE.build(CONTACTLIST);
//   redraw();
//  return;
/*
//p3d_export_robot_as_multipart_OBJ((p3d_rob *)p3d_get_robot_by_name("SAHandLeft_robot"), NULL);
//p3d_export_robot_as_multipart_OBJ((p3d_rob *)p3d_get_robot_by_name("SAHandRight_robot"), NULL);
return;
//   gpSwap_ghost_and_graphic_bodies((p3d_rob *)p3d_get_robot_by_name("SAHandLeft_robot"));return;
 static int firstTime= 1;
 if(firstTime)
 { 
   gpGet_grasp_list_SAHand("Horse", 2, GRASPLIST);
   firstTime= 0;
 }
 if(!GRASPLIST.empty())
 {
   gpSet_robot_hand_grasp_configuration((p3d_rob *)p3d_get_robot_by_name("SAHandLeft_robot"), ((p3d_rob*)p3d_get_robot_by_name("Horse")), GRASPLIST.front());
   GRASP=GRASPLIST.front();
   GRASPLIST.pop_front();
 }
redraw();
return;
// gpGet_grasp_list_gripper("DuploBox", GRASPLIST);
// redraw();
// return;


  int i= 0;

  static int count= 0;

  if(firstTime)
  {
   firstTime= 0;
    gpGet_grasp_list_SAHand("Horse", 1, GRASPLIST);
  }

  for (std::list<gpGrasp>::iterator iter=GRASPLIST.begin(); iter!=GRASPLIST.end(); iter++ )
  {
    GRASP= ( *iter );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>GRASPLIST.size() )
          {  count= 1;  }

 gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name("SAHandRight_robot"), (p3d_rob*)p3d_get_robot_by_name("Horse"), GRASP);

  redraw();
  return;*/
 static int firstTime= TRUE;
//   static Manipulation *manipulation= NULL;
  p3d_matrix4 T;
  p3d_rob *justin= NULL, *object= NULL;


  justin= p3d_get_robot_by_name("ROBOT");
  object= p3d_get_robot_by_name("Horse");
 // object_i= p3d_get_robot_by_name("Horse_i");
 // object_f= p3d_get_robot_by_name("Horse_f");

  if(firstTime)
  {
  p3d_desactivateCntrt(justin, justin->ccCntrts[0]);
  p3d_desactivateCntrt(justin, justin->ccCntrts[1]);

  p3d_set_and_update_this_robot_conf(justin, justin->ROBOT_GOTO);
  p3d_mat4Copy(justin->ccCntrts[0]->pasjnts[justin->ccCntrts[0]->npasjnts -1]->abs_pos, T);
  Af[0]= T[0][3];  Af[1]= T[1][3];  Af[2]= T[2][3];
  p3d_mat4Copy(justin->ccCntrts[1]->pasjnts[justin->ccCntrts[1]->npasjnts -1]->abs_pos, T);
  Bf[0]= T[0][3];  Bf[1]= T[1][3];  Bf[2]= T[2][3];

  p3d_set_and_update_this_robot_conf(justin, justin->ROBOT_POS);
  p3d_mat4Copy(justin->ccCntrts[0]->pasjnts[justin->ccCntrts[0]->npasjnts -1]->abs_pos, T);
  Ai[0]= T[0][3];  Ai[1]= T[1][3];  Ai[2]= T[2][3];
  p3d_mat4Copy(justin->ccCntrts[1]->pasjnts[justin->ccCntrts[1]->npasjnts -1]->abs_pos, T);
  Bi[0]= T[0][3];  Bi[1]= T[1][3];  Bi[2]= T[2][3];

  p3d_set_and_update_this_robot_conf(object, object->ROBOT_GOTO);
  p3d_mat4Copy(object->joints[1]->abs_pos, T);
  Of[0]= T[0][3];  Of[1]= T[1][3];  Of[2]= T[2][3];
  p3d_set_and_update_this_robot_conf(object, object->ROBOT_POS);
  p3d_mat4Copy(object->joints[1]->abs_pos, T);
  Oi[0]= T[0][3];  Oi[1]= T[1][3];  Oi[2]= T[2][3];




//     p3d_set_and_update_this_robot_conf(object, object->ROBOT_GOTO);
// // //   p3d_set_and_update_this_robot_conf(object, object_f->ROBOT_POS);
//     p3d_mat4Copy(object->joints[1]->abs_pos, T);
//     Of[0]= T[0][3];  Of[1]= T[1][3];  Of[2]= T[2][3];
//     p3d_set_and_update_this_robot_conf(object, object->ROBOT_POS);
// // //   p3d_set_and_update_this_robot_conf(object, object_i->ROBOT_POS);
//     p3d_mat4Copy(object->joints[1]->abs_pos, T);
//     Oi[0]= T[0][3];  Oi[1]= T[1][3];  Oi[2]= T[2][3];
//    Oi[0]=  0.59;
//    Oi[1]= -0.92 ;
//    Oi[2]=  0.636185;
// 
//    Of[0]= 0.89; 
//    Of[1]=  0.15 ;
//    Of[2]= 0.616185 ;
p3d_matrix4 Ti, Tf;
p3d_mat4Copy(p3d_mat4IDENTITY, Ti);
p3d_mat4Copy(p3d_mat4IDENTITY, Tf);
Ti[0][3]= Oi[0];
Ti[1][3]= Oi[1];
Ti[2][3]= Oi[2];

Tf[0][3]= Of[0];
Tf[1][3]= Of[1];
Tf[2][3]= Of[2];
//    manipulation= new Manipulation(justin);
// manipulation->findAllArmsGraspsConfigs(Ti, Tf);
// manipulation->computeDoubleGraspConfigList();

//       manipulation->computeRegraspTask(p3d_copy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS), p3d_copy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO), "",0);
//       manipulation->printStatDatas();

//    manipulation->computeRegraspTask(p3d_copy_config(justin,justin->ROBOT_POS),p3d_copy_config(justin,justin->ROBOT_GOTO),"",0);
   firstTime= FALSE;
  }

//  manipulation->drawDoubleGraspConfigs();

redraw();
return;
/*
  p3d_set_and_update_this_robot_conf(object, object->ROBOT_POS);
  p3d_get_body_pose(object, 0, objectStartPos);
  p3d_set_and_update_this_robot_conf(object, object->ROBOT_GOTO);
  p3d_get_body_pose(object, 0, objectEndPos);
//   p3d_get_body_pose(object_i, 0, objectStartPos);
//   p3d_get_body_pose(object_f, 0, objectEndPos);

  closestWrist = getClosestWristToTheObject(manipulation._robot);

  if (manipulation._handsDoubleGraspsConfigs.size() > 0) {
    p3d_copy_config_into(manipulation._robot, startConfig, &(manipulation._robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(manipulation._robot, manipulation._robot->ROBOT_POS);
    dgData = (*manipulation._handsDoubleGraspsConfigs.begin());
    doubleGrasp = dgData->getDoubleGrasp();
    //Get the datas corresponding to the double grasp
    if (closestWrist == 0) {
      firstGrasp = doubleGrasp.grasp1;
      secondGrasp = doubleGrasp.grasp2;
    }else {
      firstGrasp = doubleGrasp.grasp2;
      secondGrasp = doubleGrasp.grasp1;
    }
    firstGraspData = manipulation._handsGraspsConfig[closestWrist][firstGrasp.ID];
    secondGraspData = manipulation._handsGraspsConfig[1 - closestWrist][secondGrasp.ID];
    prop1.initialize(firstGraspData->getGrasp()->hand_type);
    prop2.initialize(secondGraspData->getGrasp()->hand_type);
    gpDeactivate_hand_selfcollisions(XYZ_ROBOT, 1);
    gpDeactivate_hand_selfcollisions(XYZ_ROBOT, 2);
//     if (offlineFile.compare("")) {
//       p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
//       loadedGraph = XYZ_GRAPH;
//       statDatas.push_back(_robot->GRAPH->nnode);
//       statDatas.push_back(_robot->GRAPH->time);
//     }
    manipulation.InitHandProp(0);
    manipulation.InitHandProp(1);
  }

    manipulation.InitHandProp(0);
    manipulation.InitHandProp(1);
  manipulation.findAllArmsGraspsConfigs(objectStartPos, objectEndPos);
  statDatas.push_back(manipulation._handsGraspsConfig[0].size());
  statDatas.push_back(manipulation._handsGraspsConfig[1].size());


  manipulation.computeExchangeMat(startConfig, gotoConfig);
  manipulation.computeDoubleGraspConfigList();*/
/*
      findAllArmsGraspsConfigs(objectStartPos, objectEndPos);
      statDatas.push_back(_handsGraspsConfig[0].size());
      statDatas.push_back(_handsGraspsConfig[1].size());
      statDatas.push_back(tu);
      //find Double Grasp configurations
      ChronoOn();
      computeExchangeMat(startConfig, gotoConfig);
      computeDoubleGraspConfigList();
      ChronoMicroTimes(&tu, &ts);
      ChronoPrint("Double Grasp configs: ");
      ChronoOff();
      statDatas.push_back(_handsDoubleGraspsConfigs.size());
      statDatas.push_back(tu);   */
////////////////////////////////////////////////////////////////////////////////////




return;
/*
  static int firstTime= TRUE, count= 0;
  int i;
  p3d_matrix4 objectPose;
  configPt qhand= NULL;
  gpHand_properties handProp;
  p3d_rob *SAHandRight_robot, *SAHandLeft_robot, *object, *justin;
  std::list<gpGrasp> graspList1, graspList2;

  SAHandRight_robot= p3d_get_robot_by_name("SAHandRight_robot");
  SAHandLeft_robot= p3d_get_robot_by_name("SAHandLeft_robot");
  object= p3d_get_robot_by_name("Horse");
  justin= p3d_get_robot_by_name("ROBOT");

  if(firstTime)
  {  
   gpGet_grasp_list_SAHand("Horse", 1, graspList1);
   gpGet_grasp_list_SAHand("Horse", 2, graspList2);

   gpDouble_grasp_generation(SAHandRight_robot, SAHandLeft_robot, object, graspList1, graspList2, DOUBLEGRASPLIST);
   firstTime= false;  
   printf("%d double grasps\n",DOUBLEGRASPLIST.size());
  }

  std::list<gpDoubleGrasp>::iterator iter;
  i= 0;
  for ( iter=DOUBLEGRASPLIST.begin(); iter!=DOUBLEGRASPLIST.end(); iter++ )
  {
    DOUBLEGRASP= ( *iter );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>DOUBLEGRASPLIST.size() )
  {  count= 1;  }

  gpCompute_grasp_open_config(justin, DOUBLEGRASP, object, 2);


  p3d_get_body_pose(object, 0, objectPose);
  p3d_mat4Print(objectPose, "objectPose_original");

  Manipulation manipulation(justin);
  configPt doubleGraspConfig;
  doubleGraspConfig= p3d_alloc_config(justin);
  std::vector<gpHand_properties> armsProp(2);
  armsProp.at(0).initialize(DOUBLEGRASP.grasp1.hand_type);
  armsProp.at(1).initialize(DOUBLEGRASP.grasp2.hand_type);
//   manipulation.getCollisionFreeDoubleGraspAndApproach(objectPose, armsProp, DOUBLEGRASP, &doubleGraspConfig);
//   p3d_set_and_update_this_robot_conf(justin, doubleGraspConfig);

  gpSet_robot_hand_grasp_configuration(SAHandRight_robot, object, DOUBLEGRASP.grasp1);
  gpSet_robot_hand_grasp_configuration(SAHandLeft_robot, object, DOUBLEGRASP.grasp2);
//   gpSet_robot_hand_grasp_open_configuration(SAHandLeft_robot, object, DOUBLEGRASP.grasp2);
//   gpSet_robot_hand_grasp_configuration(SAHandRight_robot, object, DOUBLEGRASP.grasp1);
//   gpSet_grasp_open_configuration(justin, DOUBLEGRASP.grasp2, 2);
//   gpSet_grasp_configuration(justin, DOUBLEGRASP.grasp1, 1);

XYZ_ROBOT= object;
p3d_set_and_update_this_robot_conf(object, object->ROBOT_POS);
//   p3d_matrix4 torsoPose;
//   p3d_mat4Copy(p3d_mat4IDENTITY, torsoPose);
// 
//   DOUBLEGRASP.computeBestObjectOrientation(torsoPose, objectPose);
//   p3d_mat4Print(objectPose, "objectPose");
//   p3d_set_freeflyer_pose(object, objectPose);
// 
//   gpSet_robot_hand_grasp_configuration(SAHandRight_robot, object, DOUBLEGRASP.grasp1);
//   gpSet_robot_hand_grasp_configuration(SAHandLeft_robot, object, DOUBLEGRASP.grasp2);
// */

  redraw();
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
    
    result= gpGet_grasp_list_SAHand(ObjectName, 1, GRASPLIST);
    
    if(result==GP_ERROR)
    {  return;  }

    object= p3d_get_robot_by_name(ObjectName);
    if(object==NULL)
    {  return;  }

    robot= p3d_get_robot_by_name(RobotName);
    if(robot==NULL)
    {  return;  }

    gpCompute_mass_properties(object->o[0]->pol[0]->poly);
    
    handProp.initialize(GRASPLIST.front().hand_type);
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

  if(GRASPLIST.empty())
  {
    printf("No grasp was found.\n");
    return;
  }

  configPt qcur= NULL, qgrasp= NULL, qend= NULL;
  qcur= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &qcur);
  qend= NULL;
  p3d_get_robot_config_into(robot, &qcur);
  qend= gpFind_grasp_from_base_configuration(robot, object, GRASPLIST, GP_PA10, qcur, GRASP, handProp);

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
  for ( igrasp=GRASPLIST.begin(); igrasp!=GRASPLIST.end(); igrasp++ )
  {
    GRASP= ( *igrasp );
    i++;
    if ( i>=count )
    {  break; }
  }
  count++;
  if ( count>GRASPLIST.size() )
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
        qend= gpFind_grasp_from_base_configuration(robot, object, GRASPLIST, GP_PA10, qgrasp, GRASP, handProp);

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
 if(LEVEL>=0)
 kdtree.draw(LEVEL);
 
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

static void CB_display_grasps_obj ( FL_OBJECT *obj, long arg )
{
	display_grasps= !display_grasps;

	if ( display_grasps )
		{  fl_set_button ( BT_DISPLAY_GRASPS_OBJ, TRUE );  }
	else
		{  fl_set_button ( BT_DISPLAY_GRASPS_OBJ, FALSE ); }

	redraw();
}


static void CB_load_grasp_list_obj ( FL_OBJECT *obj, long arg )
{
	LOAD_LIST= !LOAD_LIST;

	if ( LOAD_LIST )
		{  fl_set_button ( BT_LOAD_GRASP_LIST_OBJ, TRUE );  }
	else
		{  fl_set_button ( BT_LOAD_GRASP_LIST_OBJ, FALSE ); }

	redraw();
}



/////////////////////FUNCTIONS USED BY THE GENOM MODULE: /////////////////////////////
p3d_cntrt* GP_GetArmCntrt ( p3d_rob *robotPt )
{
	int i;
	p3d_cntrt* cntrt_arm = NULL;

	if ( robotPt==NULL )
	{
		printf ( "%s: %d: GP_GetArmCntrt(): input p3d_rob* is NULL.\n", __FILE__,__LINE__ );
		return NULL;
	}

	for ( i=0; i<robotPt->cntrt_manager->ncntrts; i++ )
	{
		cntrt_arm = robotPt->cntrt_manager->cntrts[i];
		if ( strcmp ( cntrt_arm->namecntrt, "p3d_pa10_6_arm_ik" ) ==0 )
			{  break;  }
	}
	if ( i==robotPt->cntrt_manager->ncntrts )
	{
		printf ( "%s: %d: GP_GetArmCntrt(): fatal error: arm_IK constraint does not exist.\n", __FILE__,__LINE__ );
		return NULL;
	}

	return cntrt_arm;
}



int GP_Init ( char *objectName )
{
	unsigned int i;

	if ( !INIT_IS_DONE )
	{
		init_graspPlanning ( objectName );

		// deactivate collisions for all robots except for the two of them needed by the grasp planner:
		for ( i=0; i< ( unsigned int ) XYZ_ENV->nr; i++ )
		{
			if ( XYZ_ENV->robot[i]==ROBOT || XYZ_ENV->robot[i]==HAND_ROBOT )
				{   continue;    }
			else
				{  p3d_col_deactivate_robot ( XYZ_ENV->robot[i] );  }
		}

		INIT_IS_DONE= true;
	}

	return 1;
}



//! Computes a list of grasps (for the hand only)
//! that will make the hand/gripper grasp the specified object.
//! \param objectName name of the object to be grasped by the robot
//! \return 1 in case of success, 0 otherwise
int GP_ComputeGraspList ( char *objectName )
{
	GP_Init ( objectName );

	printf ( "Collisions are deactivated for other robots.\n" );

	gpGrasp_generation ( HAND_ROBOT, OBJECT, 0, HAND_PROP, HAND_PROP.nb_positions, HAND_PROP.nb_directions, HAND_PROP.nb_rotations, GRASPLIST );

	printf ( "Before collision filter: %d grasps.\n", GRASPLIST.size() );
	gpGrasp_collision_filter ( GRASPLIST, HAND_ROBOT, OBJECT, HAND_PROP );
	printf ( "After collision filter: %d grasps.\n", GRASPLIST.size() );
	gpGrasp_stability_filter ( GRASPLIST );
	printf ( "After stability filter: %d grasps.\n", GRASPLIST.size() );

//         gpGrasp_compute_open_configs(GRASPLIST, HAND_ROBOT, OBJECT, HAND_PROP );

	gpGrasp_context_collision_filter ( GRASPLIST, HAND_ROBOT, OBJECT, HAND_PROP );
	printf ( "For the current collision context: %d grasps.\n", GRASPLIST.size() );
	p3d_col_deactivate_robot ( HAND_ROBOT );

	redraw();


	if ( GRASPLIST.empty() )
	{
		printf ( "GP_ComputeGraspList(): No grasp was found.\n" );
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
configPt GP_FindGraspConfig ( bool &needs_to_move )
{
	if ( !INIT_IS_DONE )
	{
		printf ( "GP_FindGraspConfig(): grasp planner needs to be initialized first.\n" );
		return NULL;
	}

	if ( GRASPLIST.empty() )
	{
		printf ( "GP_FindGraspConfig(): The grasp list is empty.\n" );
		return NULL;
	}

	unsigned int i, nb_iters_max;
	p3d_vector3 objectCenter;
	p3d_matrix4 objectPose;
	configPt qcurrent= NULL, qbase= NULL, qresult= NULL;

	//we first check if the robot can grasp the object from its current position:
	//find a configuration for the current robot base configuration:
	qcurrent = p3d_alloc_config ( ROBOT );
	p3d_get_robot_config_into ( ROBOT, &qcurrent );

	qresult= gpFind_grasp_from_base_configuration ( ROBOT, OBJECT, GRASPLIST, ARM_TYPE, qcurrent, GRASP, HAND_PROP );

	p3d_destroy_config ( ROBOT, qcurrent );

	if ( qresult!=NULL )
	{
		GRASP.print();
		needs_to_move= false;

		// as the real Jido's gripper can only be completely opened or completely closed,
		// we set it to max opening:
//     if(HAND_PROP.type==GP_GRIPPER)
//     {
//       p3d_set_and_update_this_robot_conf(ROBOT, qresult);
//       gpOpen_hand(ROBOT, HAND_PROP);
//       p3d_get_robot_config_into(ROBOT, &qresult);
//       p3d_set_and_update_this_robot_conf(ROBOT, qcurrent);
//     }

		return qresult;
	}



// we must try to find a valid base configuration:
      needs_to_move= true;
      p3d_get_body_pose ( OBJECT, 0, objectPose );
      objectCenter[0]= objectPose[0][3] + CMASS[0];
      objectCenter[1]= objectPose[1][3] + CMASS[1];
      objectCenter[2]= objectPose[2][3] + CMASS[2];

      nb_iters_max= 300;
      for ( i=0; i<nb_iters_max; i++ )
      {
        qbase= gpRandom_robot_base ( ROBOT, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter);
        if ( qbase==NULL )
                {   break;   }

        qresult= NULL;
        qresult= gpFind_grasp_from_base_configuration ( ROBOT, OBJECT, GRASPLIST, ARM_TYPE, qbase, GRASP, HAND_PROP );

        if ( qresult!=NULL )
                {   break;   }
        p3d_destroy_config ( ROBOT, qbase );
        qbase= NULL;
      }
      if ( qbase!=NULL )
      {  p3d_destroy_config ( ROBOT, qbase );  }

      if ( i==nb_iters_max )
      {
         printf ( "GP_FindGraspConfig: No valid platform configuration was found.\n" );
         return NULL;
      }

	// as the real Jido's gripper can only be completely opened or completely closed,
	// we set it to max opening:
//   if(HAND_PROP.type==GP_GRIPPER)
//   {
//     p3d_set_and_update_this_robot_conf(ROBOT, qresult);
//     gpOpen_hand(ROBOT, HAND_PROP);
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
	robotPt= p3d_get_robot_by_name ( GP_ROBOT_NAME );
	XYZ_ENV->cur_robot= robotPt;

	// initializes everything:
	GP_Init ((char *) GP_OBJECT_NAME_DEFAULT );

	redraw();

	cntrt_arm= GP_GetArmCntrt ( robotPt );

	if ( cntrt_arm==NULL )
	{
		printf ( "FATAL_ERROR : arm_IK constraint does not exist\n" );
		return 0;
	}

	/* Deactivate the arm_IK constrint */
	p3d_desactivateCntrt ( robotPt, cntrt_arm );

	//alloc all configs:
	qstart= p3d_alloc_config ( robotPt );
	qfinal= p3d_alloc_config ( robotPt );
	qfar= p3d_alloc_config ( HAND_ROBOT );

	gpOpen_hand ( robotPt, HAND_PROP );
	p3d_get_robot_config_into ( robotPt, &qstart );
#ifdef LIGHT_PLANNER
	p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qstart);
	//p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint ( robotPt, qstart );
#endif

	p3d_set_and_update_this_robot_conf ( robotPt, qstart );
	if ( p3d_col_test() )
	{
		printf ( "GP_FindPathForArmOnly(): Start configuration is colliding.\n" );
		so_far_so_good= false;
		goto END_ARM_ONLY;
	}


	// computes the grasp list:
	if ( !LOAD_LIST )
	{
		result= GP_ComputeGraspList ( (char*) GP_OBJECT_NAME_DEFAULT );
		gpSave_grasp_list ( GRASPLIST, "./graspPlanning/graspList_new.xml" );
	}
	else // or loads it:
	{
		result= gpLoad_grasp_list ( "./graspPlanning/graspList.xml", GRASPLIST );
		if ( result==0 )
		{
			printf ( "Can not load a grasp list.\n" );
			so_far_so_good= false;
			goto END_ARM_ONLY;
		}

		if ( !GRASPLIST.empty() )
		{
			if ( GRASPLIST.front().hand_type!=HAND_PROP.type )
			{
				printf ( "The loaded grasp list does not correspond to the current hand type.\n" );
				so_far_so_good= false;
				goto END_ARM_ONLY;
			}
		}
	}

	if ( GRASPLIST.empty() )
	{
		printf ( "Could not compute any grasp.\n" );
		so_far_so_good= false;
		goto END_ARM_ONLY;
	}

	// move away the hand robot:
	qfar= p3d_alloc_config ( HAND_ROBOT );
	qfar[7]= -100;
	qfar[8]= -1; //to put the hand far under the floor
	p3d_set_and_update_this_robot_conf ( HAND_ROBOT, qfar );

	qfinal= GP_FindGraspConfig ( needs_to_move );

	if ( qfinal!=NULL )
	{
#ifdef LIGHT_PLANNER
		p3d_update_virtual_object_config_for_arm_ik_constraint(robotPt, 0, qfinal);
		//p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint ( robotPt, qfinal );
#endif
		p3d_set_and_update_this_robot_conf ( robotPt, qfinal );
		gpOpen_hand ( robotPt, HAND_PROP );
		p3d_get_robot_config_into ( robotPt, &qfinal );
		if ( p3d_col_test() )
		{
			printf ( "The robot can not open its hand in final configuration without collision.\n" );
			so_far_so_good= false;
			goto END_ARM_ONLY;
		}
		printf ( "Grasp configuration list was successfully computed.\n" );
	}
	else
	{
		printf ( "No grasp configuration was found.\n" );
		so_far_so_good= false;
		goto END_ARM_ONLY;
	}

	if ( needs_to_move )
	{
		printf ( "The robot can not reach the object from its current position. It needs to move.\n" );
		so_far_so_good= false;
		goto END_ARM_ONLY;
	}

	p3d_set_and_update_this_robot_conf ( robotPt, qstart );
	p3d_copy_config_into ( robotPt, qstart, & ( robotPt->ROBOT_POS ) );
	p3d_copy_config_into ( robotPt, qfinal, & ( robotPt->ROBOT_GOTO ) );
	p3d_activateCntrt ( robotPt, cntrt_arm );
	g3d_draw_allwin_active();
	XYZ_ENV->cur_robot= robotPt;
	p3d_set_ROBOT_START ( qstart );
	p3d_set_ROBOT_GOTO ( qfinal );


	p3d_set_env_dmax ( DMAX_NEAR );
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan ( robotPt );
	p3d_multiLocalPath_set_groupToPlan_by_name ( robotPt, OBJECT_GROUP_NAME, 1 );
#endif
	p3d_activateCntrt ( robotPt, cntrt_arm );
	path_found= GP_FindPath();
	if ( !path_found )
	{
		printf ( "The planner could not find a valid path for the arm.\n" );
		so_far_so_good= false;
		goto END_ARM_ONLY;
	}


END_ARM_ONLY:
	p3d_destroy_config ( robotPt, qstart );
	p3d_destroy_config ( robotPt, qfinal );
	p3d_destroy_config ( HAND_ROBOT, qfar );

	if ( so_far_so_good )
	{
		printf ( "ALL IS DONE: SUCCESS.\n" );
		return 1;
	}
	else
	{
		printf ( "ALL IS DONE: THERE WAS SOMETHING WRONG.\n" );
		return 0;
	}

}


//! Creates and fills an array of configPt with the configuration steps of the given trajectory.
//! \param robotPt pointer to the robot
//! \param traj pointer to the trajectory (that must be compatible with the robot)
//! \param nb_configs will be filled with the size of the array
//! \return the configuration array
configPt* GP_GetTrajectory ( p3d_rob *robotPt, p3d_traj *traj, int &nb_configs )
{
	nb_configs= 0;
	if ( robotPt==NULL )
	{
		PrintInfo ( ( "GP_GetTrajectory: robot is NULL.\n" ) );
		return NULL;
	}
	if ( traj==NULL )
	{
		PrintInfo ( ( "GP_GetTrajectory: traj is NULL.\n" ) );
		return NULL;
	}


	bool traj_found_in_robot= false;
	double umax; // parameters along the local path
	int i, *ikSol= NULL;
	pp3d_localpath localpathPt;
	configPt *configs= NULL;

	for ( i= 0; i<robotPt->nt; i++ )
	{
		if ( robotPt->t[i]==traj )
		{
			traj_found_in_robot= true;
			break;
		}
	}
	if ( traj_found_in_robot==false )
	{
		PrintInfo ( ( "GP_GetTrajectory: traj may not belong to the robot.\n" ) );
	}


	localpathPt = traj->courbePt;
	//distances = MY_ALLOC(double, njnt+1);

	i= 0;
	while ( localpathPt!=NULL )
	{
		( nb_configs ) ++;
		localpathPt= localpathPt->next_lp;
	}
	( nb_configs ) ++;
	configs= ( configPt * ) malloc ( nb_configs*sizeof ( configPt ) );

	localpathPt = traj->courbePt;
	i= 0;
	while ( localpathPt != NULL )
	{
		umax= localpathPt->range_param;

		if ( i==0 )
		{
			configs[i]= localpathPt->config_at_param ( robotPt, localpathPt, 0 );
			if ( !ikSol || !p3d_compare_iksol ( robotPt->cntrt_manager, localpathPt->ikSol, ikSol ) )
			{
				p3d_copy_iksol ( robotPt->cntrt_manager, localpathPt->ikSol, &ikSol );
				if ( p3d_get_ik_choice() != IK_NORMAL )
					{  p3d_print_iksol ( robotPt->cntrt_manager, localpathPt->ikSol );  }
			}
			p3d_set_and_update_this_robot_conf_multisol ( robotPt, configs[i], NULL, 0, localpathPt->ikSol );
			i++;
		}

		configs[i] = localpathPt->config_at_param ( robotPt, localpathPt, umax );
		if ( !ikSol || !p3d_compare_iksol ( robotPt->cntrt_manager, localpathPt->ikSol, ikSol ) )
		{
			p3d_copy_iksol ( robotPt->cntrt_manager, localpathPt->ikSol, &ikSol );
			if ( p3d_get_ik_choice() != IK_NORMAL )
				{   p3d_print_iksol ( robotPt->cntrt_manager, localpathPt->ikSol );  }
		}
		p3d_set_and_update_this_robot_conf_multisol ( robotPt, configs[i], NULL, 0, localpathPt->ikSol );
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
configPt* GP_GetAllTrajectoriesAsOne ( p3d_rob *robotPt, int &nb_configs )
{
	nb_configs= 0;
	if ( robotPt==NULL )
	{
		PrintInfo ( ( "GP_GetTrajectory: robot is NULL.\n" ) );
		return NULL;
	}

	int i, j, n;
	configPt* configs= NULL, *result= NULL;
	std::list<configPt> cfg_list;
	std::list<configPt>::iterator iter;

	for ( i=0; i<robotPt->nt; i++ )
	{
		n= 0;
		configs= GP_GetTrajectory ( robotPt, robotPt->t[i], n );
		for ( j=0; j<n; j++ )
			{  cfg_list.push_back ( configs[j] );   }
		free ( configs );
	}

	nb_configs= cfg_list.size();
	result= ( configPt * ) malloc ( nb_configs*sizeof ( configPt ) );

	i= 0;
	for ( iter=cfg_list.begin(); iter!=cfg_list.end(); iter++ )
	{
		result[i]= ( *iter );
		i++;
	}

	return result;
}

//! Concatenes all the current trajectories of the robot into the first one.
//! NB: only the first trajectory will remain (and grown up); the others are destroyed.
//! \param robotPt pointer to the robot
//! \return 1 in case of success, 0 otherwise
int GP_ConcateneAllTrajectories ( p3d_rob *robotPt )
{
	if ( robotPt==NULL )
	{
		PrintInfo ( ( "GP_ConcateneAllTrajectories: robot is NULL.\n" ) );
		return 0;
	}
	if ( robotPt->nt==0 )
	{
		PrintInfo ( ( "GP_ConcateneAllTrajectories: the robot has no trajectory.\n" ) );
		return 0;
	}

	int i;
	pp3d_localpath localpathPt, end;

	for ( i=0; i<robotPt->nt-1; i++ )
	{
		localpathPt = robotPt->t[i]->courbePt;
		while ( localpathPt!=NULL )
		{
			end= localpathPt;
			localpathPt = localpathPt->next_lp;
		}
		end->next_lp= robotPt->t[i+1]->courbePt;
		robotPt->t[i+1]->courbePt->prev_lp= end;
	}

	for ( i=1; i<robotPt->nt; i++ )
		{  free ( robotPt->t[i] );  }
	robotPt->nt= 1;

	robotPt->tcur= robotPt->t[0];
	FORMrobot_update ( p3d_get_desc_curnum ( P3D_ROBOT ) );

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
	if ( !INIT_IS_DONE )
	{
		printf ( "GP_FindPath(): grasp planner needs to be initialized first.\n" );
		return 0;
	}

	int i, result;

	// deactivate collisions for all other robots:
	for ( i=0; i<XYZ_ENV->nr; i++ )
	{
		if ( XYZ_ENV->robot[i]==ROBOT )
			{   continue;    }
		else
			{  p3d_col_deactivate_robot ( XYZ_ENV->robot[i] );  }
	}

	printf ( "Collision are desactivated for other robots\n" );

	ENV.setBool ( Env::biDir,true );
	ENV.setInt ( Env::NbTry, 100000 );
	ENV.setInt ( Env::MaxExpandNodeFail, 30000 );
	ENV.setInt ( Env::maxNodeCompco, 100000 );
	ENV.setExpansionMethod ( Env::Connect );

//   print_config(ROBOT, ROBOT->ROBOT_POS);
//   print_config(ROBOT, ROBOT->ROBOT_GOTO);

	if ( p3d_equal_config ( ROBOT, ROBOT->ROBOT_POS, ROBOT->ROBOT_GOTO ) )
	{
		printf ( "GP_FindPath(): Start and goal configurations are the same.\n" );
		return 1;
	}

	p3d_set_and_update_this_robot_conf ( ROBOT, ROBOT->ROBOT_POS );
	result= p3d_specific_search ( (char *) "out.txt" );

	// optimizes the trajectory:
	CB_start_optim_obj ( NULL, 0 );

	// reactivate collisions for all other robots:
	for ( i=0; i<XYZ_ENV->nr; i++ )
	{
		if ( XYZ_ENV->robot[i]==HAND_ROBOT || XYZ_ENV->robot[i]==ROBOT )
			{   continue;    }
		else
			{  p3d_col_activate_robot ( XYZ_ENV->robot[i] );  }
	}
	printf ( "Collision are re-activated for other robots\n" );

	p3d_SetTemperatureParam ( 1.0 );

#ifdef LIGHT_PLANNER
	deleteAllGraphs();
#endif

	return result;
}

void GP_Reset()
{
	g3d_win *win= NULL;

	for ( int i=0; i<NB_CONFIGS; i++ )
		{ p3d_destroy_config ( ROBOT, PATH[i] );  }
	free ( PATH );
	PATH= NULL;
	NB_CONFIGS= 0;

	if ( ROBOT!=NULL )
	{
		while ( ROBOT->nt!=0 )
			{   p3d_destroy_traj ( ROBOT, ROBOT->t[0] );  }
		FORMrobot_update ( p3d_get_desc_curnum ( P3D_ROBOT ) );
	}


	ROBOT= NULL;
	HAND_ROBOT= NULL;
	OBJECT= NULL;
	POLYHEDRON= NULL;
	GRASPLIST.clear();

	INIT_IS_DONE= false;

#ifdef LIGHT_PLANNER
	deleteAllGraphs();
#endif

	//reinit all the initial collision context:
#ifdef PQP
	pqp_create_collision_pairs();
#endif

	win= g3d_get_cur_win();
	win->fct_draw2= NULL;
	win->fct_key1 = NULL;
	win->fct_key2 = NULL;
}



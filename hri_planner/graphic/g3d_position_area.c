#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
//#include "Util-pkg.h"
#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#endif




 /* Internal Functions */
static void g3d_draw_SemiDisc(float radMax, float radMin, double Vangle, int perception, int modeling);
static void g3d_draw_a_Box(double x1, double y1, double z1, double x2, double y2, double z2);
static void g3d_draw_objDisc(float radMax, float radMin, int perception, int modeling);

//static double rad2_angleOf(double x1, double y1, double x2, double y2);
double get_robot_angle_rad(p3d_rob *r);

/* External Functions */
void g3d_draw_rob_pos_area();
void g3d_draw_obj_pos_area(p3d_obj *objPt);
void g3d_draw_srchball_pos_area(psp_searchball *srchballpt);

int p3d_is_pos_area_showed(p3d_rob *r);
int p3d_is_in_pos_area(p3d_rob *r, double x, double y, int isrand);
double linearDistance(double x1, double y1, double x2, double y2);
double rad_angleOf(double x1, double y1, double x2, double y2);
//int p3d_set_robot_pos_area(p3d_rob *r, double max, double angle);
int p3d_set_robot_pos_area(p3d_rob *r, double min, double max, double angle);
/*********************************************************/
/* Function to draw a Positioning Area of a robot        */
/*********************************************************/
void g3d_draw_rob_pos_area()//p3d_rob *r)
{
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt *jntPt = r->joints[1];//objPt->jnt;
//  double *color_vect;
//  p3d_vector4 point;
  GLfloat matrix[16];
  int i,j;

  for(i=0 ; i<=3 ; i++){
    for(j=0 ; j<=3 ; j++){
      matrix[4*j+i]=jntPt->abs_pos[i][j];
    }
  }
  
  //g3d_drawSphere(r->BB.xmin,r->BB.ymin,r->BB.zmin, .3, tBlue,color_vect);
  // p3d_get_robot_center(r,point);
  //g3d_drawSphere(point[0],point[1],point[2], .3, tBlue,color_vect);
  //g3d_draw_a_Box(r->BB.xmin,r->BB.ymin,r->BB.zmin,r->BB.xmax,r->BB.ymax,r->BB.zmax);
  
  matrix[14]=0.0;//translation in z
  glPushMatrix();
  glMultMatrixf(matrix);  
  g3d_draw_SemiDisc(r->max_pos_range, r->min_pos_range,r->angle_range,1,1);  
  //g3d_draw_SemiDisc(3.0,1.0);  
  glPopMatrix();

}

/*********************************************************/
/* Function to draw a Positioning Area of an object      */
/*********************************************************/
void g3d_draw_obj_pos_area(p3d_obj *objPt)
{

  p3d_vector4 point;
  GLfloat matrix[16];
  int i;

  p3d_get_object_center(objPt,point);

    for(i = 0 ; i< 16; i++){
      matrix[i] = 0.;
    }
    
    matrix[0] = 1.;
    matrix[5] = 1.;
    matrix[10] = 1.;
    matrix[15] = 1.;

    matrix[12] = point[0];
    matrix[13] = point[1];
  //matrix[14]=0.0;//translation in z

  glPushMatrix();
  glMultMatrixf(matrix);  
  g3d_draw_objDisc(objPt->max_pos_range, objPt->min_pos_range,1,1);
  glPopMatrix();

}


/*********************************************************/
/* Function to draw a Positioning Area of an object      */
/*********************************************************/

void g3d_draw_srchball_pos_area(psp_searchball *srchballpt)
{

  GLfloat matrix[16];
  int i;

    for(i = 0 ; i< 16; i++){
      matrix[i] = 0.;
    }
    
    matrix[0] = 1.;
    matrix[5] = 1.;
    matrix[10] = 1.;
    matrix[15] = 1.;

    matrix[12] = srchballpt->position[0];
    matrix[13] = srchballpt->position[1];
  //matrix[14]=0.0;//translation in z

  glPushMatrix();
  glMultMatrixf(matrix);  
  g3d_draw_objDisc(srchballpt->distMax, srchballpt->distMin,1,1);
  glPopMatrix();

}


/*********************************************************/
/* Function to draw a SemiDisc of Positioning Area       */
/*********************************************************/

static void g3d_draw_SemiDisc(float radMax, float radMin, double Vangle, int perception, int modeling)
{
  double angle, maxangle; 
  GLint circle_points = 320; 
  int i;
  double *color_vect = NULL;
  double angle90 = circle_points/4;
  double angle270 =(angle90*3);
  double limtmp;
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
 
  if (perception)
  {
      //Semidisc base: Red one
      g3d_set_color_mat(tRed,color_vect);
      
      glBegin(GL_POLYGON); 
      // First Disc quater in the fourth cartesian cuadrant relative to the robot
      for (i = angle270; i <= circle_points; i++) {    
        angle = 2*M_PI*i/circle_points; 
        glVertex3d(radMax*cos(angle), radMax*sin(angle),0.0); 
        
      }
      maxangle = angle;
      // Second Disc quater in the first cartesian cuadrant relative to the robot
      for (i = 0; i <= angle90; i++) {
        angle = 2*M_PI*i/circle_points; 
        glVertex3d(radMax*cos(angle), radMax*sin(angle),0.0); 
      } 
    
      glEnd();
  }
  
  if (modeling)
  {
      //++++++++++ Modelling area: Green one +++++++++++++++++++++
      
      g3d_set_color_mat(tGreen,color_vect);
      glBegin(GL_QUAD_STRIP); 
      limtmp=(((2*M_PI)-(Vangle/2))*circle_points)/(2*M_PI);
      // First Disc quater in the fourth cartesian cuadrant relative to the robot
      for (i = limtmp; i <= circle_points; i++) {    
        angle = (2*M_PI)*i/circle_points; 
        glVertex3d(radMax*cos(angle), radMax*sin(angle),0.01); 
        glVertex3d(radMin*cos(angle), radMin*sin(angle),0.01); 
      }
      // Second Disc quater in the first cartesian cuadrant relative to the robot
      for (i = 0; i <= ((Vangle/2)*circle_points)/(2*M_PI); i++) {    
        angle = (2*M_PI)*i/circle_points; 
        glVertex3d(radMax*cos(angle), radMax*sin(angle),0.01); 
        glVertex3d(radMin*cos(angle), radMin*sin(angle),0.01); 
      } 

      glEnd();
  }

}


/****************************************************************/
/* Function to draw a Disc of Positioning Area  for an object   */
/****************************************************************/

static void g3d_draw_objDisc(float radMax, float radMin, int perception, int modeling)
{
  double angle;//, maxangle; 
  GLint circle_points = 640; 
  int i;
  double *color_vect = NULL;
  //double angle90 = circle_points/4;
  //double angle270 =(angle90*3);
  //double limtmp;
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
 
  if (perception)
  {
      //Disc base: Red one
      //g3d_set_color_mat(tRed,color_vect);
      g3d_drawDisc(.0,.0,.0, radMin, tRed, color_vect);
  }
  
  if (modeling)
  {
      //++++++++++ Modelling area: Green one +++++++++++++++++++++
      
      g3d_set_color_mat(tGreen,color_vect);
      glBegin(GL_QUAD_STRIP); 
      for (i = 0; i < circle_points; i++) {    
        angle = (2*M_PI)*i/circle_points; 
        glVertex3d(radMax*cos(angle), radMax*sin(angle),0.01); 
        glVertex3d(radMin*cos(angle), radMin*sin(angle),0.01); 
      }
      glEnd();
  }

}


/*********************************************************/
/* Boolean Functions to draw or not robot's view field or
positioning area                                         */
/*********************************************************/
int p3d_is_pos_area_showed(p3d_rob *r )
{
  //p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if (r->show_pos_area == 1)
    return 1;
  else
    return 0;
}

int p3d_show_rob_pos_area( p3d_rob *r )
{
  r->show_pos_area = TRUE;
  return r->show_pos_area;
}

int p3d_hide_rob_pos_area( p3d_rob *r )
{
  r->show_pos_area = FALSE;
  return r->show_pos_area;
}


int p3d_set_robot_pos_area(p3d_rob *r, double min, double max, double angle)
{
  if (min<max){
  	r->max_pos_range = max;
	r->min_pos_range = min;
	r->angle_range = max;
	return 1;
  }
  return 0;
}

int p3d_set_allhumans_standard_pos_area(p3d_rob *r)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob *currobotPt;
  int i;
  double smax, smin;

  for(i=0; i<envPt->nr; i++){
    currobotPt=envPt->robot[i];
    if(strstr(currobotPt->name,"human")) //for all humans
    {
	smax = r->cam_max_range;
	smin = r->cam_min_range; //needed to be adapted to robot cone of viee
	
    }
  }
  return TRUE;
}



int p3d_is_in_pos_area(p3d_rob *r, double x, double y, int isrand)
{
  double angle, amax, amin, arad;
  double rx, ry;
  if (isrand)
    {
      //for random
      rx = ry = 0.0;
    }
  else
    {
      //for ordered
      rx = r->joints[1]->p0.x;
      ry = r->joints[1]->p0.y;
    }

  arad = linearDistance(0.0,0.0,x,y);

  if (arad <= r->max_pos_range && arad >= r->min_pos_range)
    {
     // printf("---          ---\n");
      angle = rad_angleOf(0.0,0.0,x,y);
     // printf("--- Angle: %f ---\n",angle);
      amax = r->angle_range/2;
      amin = -r->angle_range/2;
      if (amin<0)
        {	
          amin+=2*M_PI;//angle=2*M_PI-angle;//maybe is incorrect
          if (angle<M_PI)
	        angle+=2*M_PI;
          if (amax<2*M_PI)
	        amax+=2*M_PI;
        }
      if (angle<=amax && angle>=amin)
	{
	  return 1;
	}
        //printf("Angle: %f is not in [%f , %f]\n",angle, amin, amax);
    } 
  return 0;
 
}

int p3d_is_in_obj_pos_area(p3d_obj *o, double x, double y)
{
  double arad;
  arad = linearDistance(0.0,0.0,x,y);
  if (arad <= o->max_pos_range && arad >= o->min_pos_range)
    {
       return 1;
    } 
  return 0;
 
}

/*********************************************************/
/* Auxiliar Math Functions to know positioning area      */
/*********************************************************/

double linearDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
}

double rad_angleOf(double x1, double y1, double x2, double y2)
{
  double alfa;
  
  alfa=atan2(y2-y1,x2-x1);
//  printf("--- Alfa: %f --- 1 %f,%f - 2 %f,%f\n",alfa,x1,y1,x2,y2);
 /* if ((x2-x1)<0)
 //   {
// 	alfa+=M_PI;
 //   }
  else
    {*/
      if (alfa<0)
	alfa+=2*M_PI;
 //   }
  return alfa;
}

double rad_angleOf_PIMED(double x1, double y1, double x2, double y2)
{
  double alfa;
  
  alfa=atan((y2-y1)/(x2-x1));
  if ((x2-x1)<0)
    {
 	alfa+=M_PI;
    }
  else
    {
      if (alfa<0)
	alfa+=2*M_PI;
    }
  return alfa;
}

double rad2_angleOf(double x1, double y1, double x2, double y2)
{
  double alfa;
  alfa = atan((y2-y1)/(x2-x1));
  if ((x2-x1)<0)
  {
    alfa+=M_PI;
  }
  return alfa;
}


double get_robot_angle_rad(p3d_rob *r)
{
  p3d_vector4 v_aux,v_aux2; 
  double x,y;
  //p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

      v_aux[0]=0.0;
      v_aux[1]=0.0;
      v_aux[2]=0.0;
      v_aux[3]=0;

      p3d_matvec4Mult(r->joints[1]->abs_pos,v_aux,v_aux2); 
      
      x=v_aux2[0];
      y=v_aux2[1];
      v_aux[0]=1.0;
      v_aux[1]=1.0;

      p3d_matvec4Mult(r->joints[1]->abs_pos,v_aux,v_aux2);
      
      return rad_angleOf(x,y,v_aux2[0],v_aux2[1]);

}


static void g3d_draw_a_Box(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double *color_vect = NULL;
	g3d_set_color_mat(tBlue,color_vect);
	glBegin(GL_LINE_LOOP);
	{      
		glVertex3d(x1,y1,z2);
		glVertex3d(x1,y2,z2);
		glVertex3d(x2,y2,z2);
		glVertex3d(x2,y1,z2);
	}
	glEnd();
       
	glBegin(GL_LINE_LOOP);
	{      
		glVertex3d(x1,y1,z1);
		glVertex3d(x1,y2,z1);
		glVertex3d(x2,y2,z1);
		glVertex3d(x2,y1,z1);
	}
	glEnd();
       
	glBegin(GL_LINES);
	{      
		glVertex3d(x1,y1,z1);  
		glVertex3d(x1,y1,z2);
	 
		glVertex3d(x2,y1,z1);  
		glVertex3d(x2,y1,z2);
	 
		glVertex3d(x2,y2,z1);  
		glVertex3d(x2,y2,z2);
	 
		glVertex3d(x1,y2,z1);  
		glVertex3d(x1,y2,z2);
	}
	glEnd();	
	
}

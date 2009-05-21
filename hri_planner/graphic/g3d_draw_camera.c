#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
//#include "Util-pkg.h"
#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#endif

/* Modif Luis */

/* Internal Functions */
static void g3d_draw_cone(double x,double y,double z, double r, double rmax, double Vangle, double  Hangle, int axe, double pan, double tilt );

//static double dec_angle(double angle, double decrementation);

/* External Functions */
void g3d_draw_rob_cone(); 
int p3d_is_view_field_showed();
void set_robot_camera_body(p3d_rob *r, int body);
void p3d_set_rob_cam_parameters(p3d_rob *r, double x, double y, double z, double min, double max, double Vangle, double Hangle, double body, int axe, double pan, double tilt);

void p3d_rotVector4_in_axe(p3d_vector4 point, float theta, int axe, p3d_vector4 result);
/********************************************************************************************/

/*********************************************************/
/* Function to draw the camera view                      */
/*********************************************************/


void g3d_draw_rob_cone()
{
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_obj *objPt = r->o[r->cam_body_index]; 
  p3d_jnt *jntPt = objPt->jnt;
   //p3d_jnt *jntPt = r->joints[1];
  GLfloat matrix[16];
  int i,j;


  for(i=0 ; i<=3 ; i++){
    for(j=0 ; j<=3 ; j++){
      matrix[4*j+i]=jntPt->abs_pos[i][j];
    }
  }
  //matrix[14]=0.0;

  glPushMatrix();
  glMultMatrixf(matrix); 
  g3d_draw_cone(r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], r->cam_min_range, r->cam_max_range, r->cam_v_angle, r->cam_h_angle, r->cam_axe, r->cam_pan, r->cam_tilt);
  //g3d_draw_cone(0.2,0.0,0.5,3.0,0.0,0.0); 
   
  
  glPopMatrix();

}


/*********************************************************/
/* Function to draw a wired cone (Actually a Pyramid)    */
/*********************************************************/

static void g3d_draw_cone(double x,double y,double z, double r, double rmax, double Vangle, double  Hangle, int axe, double pan, double tilt )
{
 
  double *color_vect;
  //double widecam=0.05; // Small square size
  //double widecam2=1.0; // Big square size
  double z2,x2,y2;
  //double refSize;
  double xt[8],yt[8],zt[8];
  double auxAngleH=Hangle/2, auxAngleV=Vangle/2;
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  g3d_set_color_mat(tBlue,color_vect);

  
  switch (axe)
    {
    case 0:// X
      z2 = z + r * cos(pan) * cos(tilt);
      y2 = y + r * sin(tilt);
      x2 = x + r * sin(pan);

//Four vertex of the small Base of the pyramid [the cutting plane of de camera]
      zt[0] = z + 0.1 * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      yt[0] = y + 0.1 * sin(tilt + auxAngleV);
      xt[0] = x + 0.1 * sin(pan  + auxAngleH);

      zt[1] = z + 0.1 * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      yt[1] = y + 0.1 * sin(tilt + auxAngleV);
      xt[1] = x + 0.1 * sin(pan  - auxAngleH);

      zt[2] = z + 0.1 * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      yt[2] = y + 0.1 * sin(tilt - auxAngleV);
      xt[2] = x + 0.1 * sin(pan  - auxAngleH);

      zt[3] = z + 0.1 * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      yt[3] = y + 0.1 * sin(tilt - auxAngleV);
      xt[3] = x + 0.1 * sin(pan  + auxAngleH);

//Four vertex of the big Base of the pyramid
      zt[4] = z + r * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      yt[4] = y + r * sin(tilt + auxAngleV);
      xt[4] = x + r * sin(pan  + auxAngleH);

      zt[5] = z + r * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      yt[5] = y + r * sin(tilt + auxAngleV);
      xt[5] = x + r * sin(pan  - auxAngleH);

      zt[6] = z + r * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      yt[6] = y + r * sin(tilt - auxAngleV);
      xt[6] = x + r * sin(pan  - auxAngleH);

      zt[7] = z + r * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      yt[7] = y + r * sin(tilt - auxAngleV);
      xt[7] = x + r * sin(pan  + auxAngleH);

     break;
    case 1:// Y
      y2 = y + r * cos(pan) * cos(tilt);
      x2 = x + r * sin(tilt);
      z2 = z + r * sin(pan);

//Four vertex of the small Base of the pyramid [the cutting plane of de camera]
      yt[0] = y + 0.1 * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      xt[0] = x + 0.1 * sin(tilt + auxAngleV);
      zt[0] = z + 0.1 * sin(pan  + auxAngleH);

      yt[1] = y + 0.1 * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      xt[1] = x + 0.1 * sin(tilt + auxAngleV);
      zt[1] = z + 0.1 * sin(pan  - auxAngleH);

      yt[2] = y + 0.1 * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      xt[2] = x + 0.1 * sin(tilt - auxAngleV);
      zt[2] = z + 0.1 * sin(pan  - auxAngleH);

      yt[3] = y + 0.1 * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      xt[3] = x + 0.1 * sin(tilt - auxAngleV);
      zt[3] = z + 0.1 * sin(pan  + auxAngleH);

//Four vertex of the big Base of the pyramid
      yt[4] = y + r * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      xt[4] = x + r * sin(tilt + auxAngleV);
      zt[4] = z + r * sin(pan  + auxAngleH);

      yt[5] = y + r * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      xt[5] = x + r * sin(tilt + auxAngleV);
      zt[5] = z + r * sin(pan  - auxAngleH);

      yt[6] = y + r * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      xt[6] = x + r * sin(tilt - auxAngleV);
      zt[6] = z + r * sin(pan  - auxAngleH);

      yt[7] = y + r * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      xt[7] = x + r * sin(tilt - auxAngleV);
      zt[7] = z + r * sin(pan  + auxAngleH);
      break;
    case 2:// Z
    default:
      x2 = x + r * cos(pan) * cos(tilt);
      y2 = y + r * sin(tilt);
      z2 = z + r * sin(pan);
//Four vertex of the small Base of the pyramid [the cutting plane of de camera]
      xt[0] = x + 0.1 * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      yt[0] = y + 0.1 * sin(tilt + auxAngleV);
      zt[0] = z + 0.1 * sin(pan  + auxAngleH);

      xt[1] = x + 0.1 * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      yt[1] = y + 0.1 * sin(tilt + auxAngleV);
      zt[1] = z + 0.1 * sin(pan  - auxAngleH);

      xt[2] = x + 0.1 * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      yt[2] = y + 0.1 * sin(tilt - auxAngleV);
      zt[2] = z + 0.1 * sin(pan  - auxAngleH);

      xt[3] = x + 0.1 * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      yt[3] = y + 0.1 * sin(tilt - auxAngleV);
      zt[3] = z + 0.1 * sin(pan  + auxAngleH);

//Four vertex of the big Base of the pyramid
      xt[4] = x + r * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      yt[4] = y + r * sin(tilt + auxAngleV);
      zt[4] = z + r * sin(pan  + auxAngleH);

      xt[5] = x + r * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      yt[5] = y + r * sin(tilt + auxAngleV);
      zt[5] = z + r * sin(pan  - auxAngleH);

      xt[6] = x + r * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      yt[6] = y + r * sin(tilt - auxAngleV);
      zt[6] = z + r * sin(pan  - auxAngleH);

      xt[7] = x + r * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      yt[7] = y + r * sin(tilt - auxAngleV);
      zt[7] = z + r * sin(pan  + auxAngleH);
      break;
    }

  
//Drawing only four "conne's" planes in lines

//The small Base 
  glBegin(GL_LINE_LOOP); 
  glVertex3f(xt[0],yt[0],zt[0]);  // bottom Left
  glVertex3f(xt[1],yt[1],zt[1]);  // bottom Right
  glVertex3f(xt[2],yt[2],zt[2]);  // top right
  glVertex3f(xt[3],yt[3],zt[3]);  // top left
  glEnd();

//The Big Base
  glBegin(GL_LINE_LOOP); 
  glVertex3f(xt[4],yt[4],zt[4]);  // bottom Left
  glVertex3f(xt[5],yt[5],zt[5]);  // bottom Right
  glVertex3f(xt[6],yt[6],zt[6]);  // top right
  glVertex3f(xt[7],yt[7],zt[7]);  // top left
  glEnd();

//The right side
  glBegin(GL_LINE_LOOP); 
  glVertex3f(xt[5],yt[5],zt[5]);  // bottom right
  glVertex3f(xt[6],yt[6],zt[6]);  // top right
  glVertex3f(xt[2],yt[2],zt[2]);  // top right small
  glVertex3f(xt[1],yt[1],zt[1]);  // bottom Right small
  glEnd();

//The left side
  glBegin(GL_LINE_LOOP); 
  glVertex3f(xt[0],yt[0],zt[0]);  // bottom Left
  glVertex3f(xt[3],yt[3],zt[3]);  // top left
  glVertex3f(xt[7],yt[7],zt[7]);  // top left Big
  glVertex3f(xt[4],yt[4],zt[4]);  // bottom Left Big
  glEnd();
  

  glBegin(GL_LINE); //center line
  glVertex3f(x,y,z);  // center small
  glVertex3f(x2,y2,z2);  // center Big
  glEnd();
}

/*********************************************************/
/* Boolean Functions to draw or not robot's view field   */
/*********************************************************/

int p3d_is_view_field_showed(p3d_rob *r)
{
  //p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if (r->show_view_field == 1)
     return r->show_view_field;
  else
     return 0;
}


void p3d_set_visible_robot_view_field(p3d_rob *r, int state)
{
  r->show_view_field = state;
}

void p3d_set_visible_robot_pos_area(p3d_rob *r, int state)
{
  r->show_pos_area = state;
}
/*
static double dec_angle(double angle, double decrementation)
{
  double temp = angle - decrementation;
  if (temp<0)
    temp+=2*M_PI;
  return temp;
}

static double aug_angle(double angle, double augmentation)
{
  double temp = angle + augmentation;
  if (temp<0)
    temp+=2*M_PI;
  return temp;
}
*/

void p3d_set_rob_cam_parameters(p3d_rob *r, double x, double y, double z, double min, double max, double Vangle, double Hangle, double body, int axe, double pan, double tilt)
{
  r->cam_pos[0]     = x;
  r->cam_pos[1]     = y;
  r->cam_pos[2]     = z;
  r->cam_min_range  = min;
  r->cam_max_range  = max;  
  r->cam_v_angle    = Vangle;
  r->cam_h_angle    = Hangle;
  r->cam_body_index = body;
  r->cam_axe        = axe;
  r->cam_pan        = pan;
  r->cam_tilt       = tilt;
  
  switch (axe)
    {
    case 0:// X
      r->cam_dir[2] = z +  min * cos(pan) * cos(tilt);
      r->cam_dir[1] = y +  min * sin(tilt);
      r->cam_dir[0] = x +  min * sin(pan);
      break;
    case 1:// Y
      r->cam_dir[1] = y +  min * cos(pan) * cos(tilt);
      r->cam_dir[0] = x +  min * sin(tilt);
      r->cam_dir[2] = z +  min * sin(pan);
      break;
    case 2:// Z
      r->cam_dir[0] = x +  min * cos(pan) * cos(tilt);
      r->cam_dir[1] = y +  min * sin(tilt);
      r->cam_dir[2] = z +  min * sin(pan);
      break;    

    }  

}

void p3d_update_rob_cam_parameters(p3d_rob *r)
{

  switch (r->cam_axe)
    {
    case 0:// X
      r->cam_dir[2] = r->cam_pos[2] +  r->cam_min_range * cos(r->cam_pan) * cos(r->cam_tilt);
      r->cam_dir[1] = r->cam_pos[1] +  r->cam_min_range * sin(r->cam_tilt);
      r->cam_dir[0] = r->cam_pos[0] +  r->cam_min_range * sin(r->cam_pan);
      break;
    case 1:// Y
      r->cam_dir[1] = r->cam_pos[1] +  r->cam_min_range * cos(r->cam_pan) * cos(r->cam_tilt);
      r->cam_dir[0] = r->cam_pos[0] +  r->cam_min_range * sin(r->cam_tilt);
      r->cam_dir[2] = r->cam_pos[2] +  r->cam_min_range * sin(r->cam_pan);
      break;
    case 2:// Z
      r->cam_dir[0] = r->cam_pos[0] +  r->cam_min_range * cos(r->cam_pan) * cos(r->cam_tilt);
      r->cam_dir[1] = r->cam_pos[1] +  r->cam_min_range * sin(r->cam_tilt);
      r->cam_dir[2] = r->cam_pos[2] +  r->cam_min_range * sin(r->cam_pan);
      break;  

    }  

}
void set_robot_camera_body(p3d_rob *r, int body)
{

  r->cam_body_index = body;
//  r->cam_axe = axe;
}

void p3d_rotVector4_in_axe(p3d_vector4 point, float theta, int axe, p3d_vector4 result)
{
  
  p3d_vector4 unitvect;
  p3d_matrix4 matrot;
  float t = 1-cos(theta);
  float c = cos(theta);
  float s = sin(theta);
  switch (axe)
    {
    case 0:// X
      unitvect [0] = 1;
      unitvect [1] = 0;
      unitvect [2] = 0;
      break;
    case 1:// Y
      unitvect [0] = 0;
      unitvect [1] = 1;
      unitvect [2] = 0;
      break;
    case 2:// Z
      unitvect [0] = 0;
      unitvect [1] = 0;
      unitvect [2] = 1;
      break;  
    }
  unitvect [3] = 0;
  matrot[0][0] = t * unitvect[0] + c;
  matrot[0][1] = t * unitvect[0] * unitvect[1] - s * unitvect[2];
  matrot[0][2] = t * unitvect[0] + unitvect[2] + s * unitvect[1];
  matrot[0][3] = 0.0;
  
  matrot[1][0] = t * unitvect[0] * unitvect[1] + s * unitvect[2];
  matrot[1][1] = t * unitvect[1] + c;
  matrot[1][2] = t * unitvect[1] + unitvect[2] - s * unitvect[0];
  matrot[1][3] = 0.0;

  matrot[2][0] = t * unitvect[0] * unitvect[2] - s * unitvect[1];
  matrot[2][1] = t * unitvect[1] + unitvect[2] + s * unitvect[0];
  matrot[2][2] = t * unitvect[2] + c;
  matrot[2][3] = 0.0;

  matrot[3][0] = 0.0;
  matrot[3][1] = 0.0;
  matrot[3][2] = 0.0;
  matrot[3][3] = 1.0;

  p3d_matvec4Mult(matrot,point,result);
}

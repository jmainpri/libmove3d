#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
//#include "Util-pkg.h"
//#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#include "iostream"
//#endif

/* Modif Luis */

/* Internal Functions */
static void g3d_draw_cone(double x,double y,double z, double r, double rmax, double Vangle, double  Hangle, int axe, double pan, double tilt );
static void perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar );
static double tBluev[4] = {0.0, 0.0, 1.0, 0.3};

//static double dec_angle(double angle, double decrementation);

/* External Functions */
void g3d_draw_rob_cone(); 
int p3d_is_view_field_showed();
void set_robot_camera_body(p3d_rob *r, int body);
void p3d_set_rob_cam_parameters(p3d_rob *r, double x, double y, double z, double min, double max, double Vangle, double Hangle, double body, int axe, double pan, double tilt);

void p3d_rotVector4_in_axe(p3d_vector4 point, float theta, int axe, p3d_vector4 result);
/********************************************************************************************/


void g3d_draw_rob_cone(p3d_rob *r)
{
  //p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_obj *objPt = r->o[r->cam_body_index]; 
  p3d_jnt *jntPt = objPt->jnt;
   //p3d_jnt *jntPt = r->joints[1];
  GLfloat matrix[16], matrix2[16], matrix3[16];
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
      matrix[4*j+i]=jntPt->abs_pos[i][j];
	  matrix2[4*j+i]=mattemp[i][j];
		if ( r->cam_axe==1)
			  matrix3[4*j+i]=mattemp2[i][j];
   }
  }
  //matrix[14]=0.0;
	//float degYang = (r->cam_v_angle * 180.0/M_PI);//good
   float degYang = (((r->cam_h_angle*2.0)/3.0) * 180.0/M_PI);
  glPushMatrix();
  glMultMatrixf(matrix);
	//g3d_draw_cone(r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], r->cam_min_range, r->cam_max_range, r->cam_v_angle, r->cam_h_angle, r->cam_axe, r->cam_pan, r->cam_tilt);
  glMultMatrixf(matrix2);
	if ( r->cam_axe==1)
		glMultMatrixf(matrix3);
	//perspectiveGL(degYang, r->cam_h_angle/r->cam_v_angle ,0.001, 8.0);//original
	perspectiveGL(degYang, 3.0/2.0 ,0.001, 8.0);

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
  double z2,x2,y2,rmin;
  //double refSize;
  double xt[8],yt[8],zt[8];
  double auxAngleH=Hangle/2, auxAngleV=Vangle/2;
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  g3d_set_color_mat(tBlue,color_vect);

	rmin = 0;
  switch (axe)
    {
    case 0:// X
      z2 = z + r * cos(pan) * cos(tilt);
      y2 = y + r * sin(tilt);
      x2 = x + r * sin(pan);

//Four vertex of the small Base of the pyramid [the cutting plane of de camera]
      zt[0] = z + rmin * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      yt[0] = y + rmin * sin(tilt + auxAngleV);
      xt[0] = x + rmin * sin(pan  + auxAngleH);

      zt[1] = z + rmin * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      yt[1] = y + rmin * sin(tilt + auxAngleV);
      xt[1] = x + rmin * sin(pan  - auxAngleH);

      zt[2] = z + rmin * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      yt[2] = y + rmin * sin(tilt - auxAngleV);
      xt[2] = x + rmin * sin(pan  - auxAngleH);

      zt[3] = z + rmin * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      yt[3] = y + rmin * sin(tilt - auxAngleV);
      xt[3] = x + rmin * sin(pan  + auxAngleH);

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
      yt[0] = y + rmin * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      xt[0] = x + rmin * sin(tilt + auxAngleV);
      zt[0] = z + rmin * sin(pan  + auxAngleH);

      yt[1] = y + rmin * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      xt[1] = x + rmin * sin(tilt + auxAngleV);
      zt[1] = z + rmin * sin(pan  - auxAngleH);

      yt[2] = y + rmin * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      xt[2] = x + rmin * sin(tilt - auxAngleV);
      zt[2] = z + rmin * sin(pan  - auxAngleH);

      yt[3] = y + rmin * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      xt[3] = x + rmin * sin(tilt - auxAngleV);
      zt[3] = z + rmin * sin(pan  + auxAngleH);

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
      xt[0] = x + rmin * cos(pan  + auxAngleH) * cos(tilt + auxAngleV);
      yt[0] = y + rmin * sin(tilt + auxAngleV);
      zt[0] = z + rmin * sin(pan  + auxAngleH);

      xt[1] = x + rmin * cos(pan  - auxAngleH) * cos(tilt + auxAngleV);
      yt[1] = y + rmin * sin(tilt + auxAngleV);
      zt[1] = z + rmin * sin(pan  - auxAngleH);

      xt[2] = x + rmin * cos(pan  - auxAngleH) * cos(tilt - auxAngleV);
      yt[2] = y + rmin * sin(tilt - auxAngleV);
      zt[2] = z + rmin * sin(pan  - auxAngleH);

      xt[3] = x + rmin * cos(pan  + auxAngleH) * cos(tilt - auxAngleV);
      yt[3] = y + rmin * sin(tilt - auxAngleV);
      zt[3] = z + rmin * sin(pan  + auxAngleH);

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

/////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// 3D vector
///////////////////////////////////////////////////////////////////////////////
typedef struct Vector3
	{
		float x;
		float y;
		float z;
		
		// ctors
		Vector3() : x(0), y(0), z(0) {};
		Vector3(float x, float y, float z) : x(x), y(y), z(z) {};
		
		// utils functions
		float       length() const;                         //
		float       distance(const Vector3& vec) const;     // distance between two vectors
		Vector3&    normalize();                            //
		float       dot(const Vector3& vec) const;          // dot product
		Vector3     cross(const Vector3& vec) const;        // cross product, same as *operator
		
		// operators
		Vector3     operator-() const;                      // unary operator (negate)
		Vector3     operator+(const Vector3& rhs) const;    // add rhs
		Vector3     operator-(const Vector3& rhs) const;    // subtract rhs
		Vector3&    operator+=(const Vector3& rhs);         // add rhs and update this object
		Vector3&    operator-=(const Vector3& rhs);         // subtract rhs and update this object
		Vector3     operator*(const float scale) const;     // scale
		Vector3     operator*(const Vector3& rhs) const;    // cross product
		Vector3&    operator*=(const float scale);          // scale and update this object
		Vector3&    operator*=(const Vector3& rhs);         // cross product and update this object
		Vector3     operator/(const float scale) const;     // inverse scale
		Vector3&    operator/=(const float scale);          // scale and update this object
		bool        operator==(const Vector3& rhs) const;   // exact compare, no epsilon
		bool        operator!=(const Vector3& rhs) const;   // exact compare, no epsilon
		float       operator[](int index) const;            // subscript operator v[0], v[1]
		float&      operator[](int index);                  // subscript operator v[0], v[1]
		
		friend Vector3 operator*(const float a, const Vector3 vec);
		friend std::ostream& operator<<(std::ostream& os, Vector3& vec);
	}Vector3;



// fast math routines from Doom3 SDK
inline float invSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;          // get bits for floating value
    i = 0x5f3759df - (i>>1);    // gives initial guess
    x = *(float*)&i;            // convert bits back to float
    x = x * (1.5f - xhalf*x*x); // Newton step
    return x;
}


///////////////////////////////////////////////////////////////////////////////
// inline functions for Vector3
///////////////////////////////////////////////////////////////////////////////
inline Vector3 Vector3::operator-() const {
    return Vector3(-x, -y, -z);
}

inline Vector3 Vector3::operator+(const Vector3& rhs) const {
    return Vector3(x+rhs.x, y+rhs.y, z+rhs.z);
}

inline Vector3 Vector3::operator-(const Vector3& rhs) const {
    return Vector3(x-rhs.x, y-rhs.y, z-rhs.z);
}

inline Vector3& Vector3::operator+=(const Vector3& rhs) {
x += rhs.x; y += rhs.y; z += rhs.z; return *this;
}

inline Vector3& Vector3::operator-=(const Vector3& rhs) {
x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this;
}

inline Vector3 Vector3::operator*(const float a) const {
    return Vector3(x*a, y*a, z*a);
}

inline Vector3 Vector3::operator*(const Vector3& rhs) const {
    return Vector3(y*rhs.z - z*rhs.y, z*rhs.x - x*rhs.z, x*rhs.y - y*rhs.x);
}

inline Vector3& Vector3::operator*=(const float a) {
x *= a; y *= a; z *= a; return *this;
}

inline Vector3 Vector3::operator/(const float a) const {
    return Vector3(x/a, y/a, z/a);
}

inline Vector3& Vector3::operator/=(const float a) {
x /= a; y /= a; z /= a; return *this;
}

inline bool Vector3::operator==(const Vector3& rhs) const {
return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
}

inline bool Vector3::operator!=(const Vector3& rhs) const {
return (x != rhs.x) || (y != rhs.y) || (z != rhs.z);
}

inline float Vector3::operator[](int index) const {
    return (&x)[index];
}

inline float& Vector3::operator[](int index) {
    return (&x)[index];
}

inline float Vector3::length() const {
    return sqrtf(x*x + y*y + z*z);
}

inline float Vector3::distance(const Vector3& vec) const {
    return sqrtf((vec.x-x)*(vec.x-x) + (vec.y-y)*(vec.y-y) + (vec.z-z)*(vec.z-z));
}

inline Vector3& Vector3::normalize() {
    float invLength = invSqrt(x*x + y*y + z*z);
    x *= invLength;
    y *= invLength;
    z *= invLength;
    return *this;
}

inline float Vector3::dot(const Vector3& rhs) const {
    return (x*rhs.x + y*rhs.y + z*rhs.z);
}

inline Vector3 Vector3::cross(const Vector3& rhs) const {
    return Vector3(y*rhs.z - z*rhs.y, z*rhs.x - x*rhs.z, x*rhs.y - y*rhs.x);
}

inline Vector3 operator*(const float a, const Vector3 vec) {
    return Vector3(a*vec.x, a*vec.y, a*vec.z);
}

inline std::ostream& operator<<(std::ostream& os, Vector3& vec) {
    std::cout << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}
// END OF VECTOR3 ////////////////////////////////////////////////////////////


static Vector3 frustumVertices[8];
static Vector3 frustumNormals[6];

static void gpsp_computeFrustumVertices(float l, float r, float b, float t, float n, float f)
{
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
	
    // compute normals
    frustumNormals[0] = (frustumVertices[5] - frustumVertices[1]) * (frustumVertices[2] - frustumVertices[1]);
    frustumNormals[0].normalize();
	
    frustumNormals[1] = (frustumVertices[3] - frustumVertices[0]) * (frustumVertices[4] - frustumVertices[0]);
    frustumNormals[1].normalize();
	
    frustumNormals[2] = (frustumVertices[6] - frustumVertices[2]) * (frustumVertices[3] - frustumVertices[2]);
    frustumNormals[2].normalize();
	
    frustumNormals[3] = (frustumVertices[4] - frustumVertices[0]) * (frustumVertices[1] - frustumVertices[0]);
    frustumNormals[3].normalize();
	
    frustumNormals[4] = (frustumVertices[1] - frustumVertices[0]) * (frustumVertices[3] - frustumVertices[0]);
    frustumNormals[4].normalize();
	
    frustumNormals[5] = (frustumVertices[7] - frustumVertices[4]) * (frustumVertices[5] - frustumVertices[4]);
    frustumNormals[5].normalize();
}


/*
 Replaces gluPerspective. Sets the frustum to perspective mode.
 fovY	- Field of vision in degrees in the y direction
 aspect	- Aspect ratio of the viewport
 zNear	- The near clipping distance
 zFar	- The far clipping distance
 */

static void perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar )
{

	//	Half of the size of the x and y clipping planes.
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
	glEnable(GL_BLEND);
	glDepthMask(GL_FALSE);
	//glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
	//g3d_set_color_mat(Any,tBluev);
	gpsp_drawFrustum(-fW, fW, -fH, fH, zNear, zFar );	
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
}




void gpsp_drawFrustum(float l, float r, float b, float t, float n, float f)
{
	
	gpsp_computeFrustumVertices(l, r, b, t, n, f);

	
	int projectionMode = 0;
    float colorLine1[4]  = { 0.7f, 0.7f, 0.7f, 0.7f };
    float colorLine2[4]  = { 0.2f, 0.2f, 0.2f, 0.7f };
	float colorPlane1[4] = { 0.5f, 0.5f, 0.5f, 0.7f };
	
    // draw lines
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
    if(projectionMode == 0)
    {
        glBegin(GL_LINES);
        glColor4fv(colorLine2);
        glVertex3f(0, 0, 0);
        glColor4fv(colorLine1);
		//g3d_set_color_mat(Any,tBluev);
        glVertex3fv(&frustumVertices[4].x);
		
        glColor4fv(colorLine2);
        glVertex3f(0, 0, 0);
        glColor4fv(colorLine1);
		//g3d_set_color_mat(Any,tBluev);
        glVertex3fv(&frustumVertices[5].x);
		
        glColor4fv(colorLine2);
        glVertex3f(0, 0, 0);
        glColor4fv(colorLine1);
		//g3d_set_color_mat(Any,tBluev);
        glVertex3fv(&frustumVertices[6].x);
		
        glColor4fv(colorLine2);
        glVertex3f(0, 0, 0);
        glColor4fv(colorLine1);
		//g3d_set_color_mat(Any,tBluev);
        glVertex3fv(&frustumVertices[7].x);
        glEnd();
    }
    else
    {
        glColor4fv(colorLine1);
        glBegin(GL_LINES);
        glVertex3fv(&frustumVertices[0].x);
        glVertex3fv(&frustumVertices[4].x);
        glVertex3fv(&frustumVertices[1].x);
        glVertex3fv(&frustumVertices[5].x);
        glVertex3fv(&frustumVertices[2].x);
        glVertex3fv(&frustumVertices[6].x);
        glVertex3fv(&frustumVertices[3].x);
        glVertex3fv(&frustumVertices[7].x);
        glEnd();
    }
	
    glColor4fv(colorLine1);
    glBegin(GL_LINE_LOOP);
    glVertex3fv(&frustumVertices[4].x);
    glVertex3fv(&frustumVertices[5].x);
    glVertex3fv(&frustumVertices[6].x);
    glVertex3fv(&frustumVertices[7].x);
    glEnd();
	
    glColor4fv(colorLine1);
    glBegin(GL_LINE_LOOP);
    glVertex3fv(&frustumVertices[0].x);
    glVertex3fv(&frustumVertices[1].x);
    glVertex3fv(&frustumVertices[2].x);
    glVertex3fv(&frustumVertices[3].x);
    glEnd();
	
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
	glColor4fv(colorPlane1);
    // frustum is transparent.
    // Draw the frustum faces twice: backfaces first then frontfaces second.
    for(int i = 0; i < 2; ++i)
    {
        if(i == 0)
        {
            // for inside planes
            glCullFace(GL_FRONT);
            glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
        }
        else
        {
            // draw outside planes
            glCullFace(GL_BACK);
            glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
        }

		//
		g3d_set_color_mat(Any,tBluev);
        glBegin(GL_QUADS);
        // left
        glNormal3fv(&frustumNormals[0].x);
        glVertex3fv(&frustumVertices[1].x);
        glVertex3fv(&frustumVertices[5].x);
        glVertex3fv(&frustumVertices[6].x);
        glVertex3fv(&frustumVertices[2].x);
        // right
		//glColor4fv(colorPlane1);

        glNormal3fv(&frustumNormals[1].x);
        glVertex3fv(&frustumVertices[0].x);
        glVertex3fv(&frustumVertices[3].x);
        glVertex3fv(&frustumVertices[7].x);
        glVertex3fv(&frustumVertices[4].x);
		//glColor4fv(colorPlane1);

        // bottom
        glNormal3fv(&frustumNormals[2].x);
        glVertex3fv(&frustumVertices[2].x);
        glVertex3fv(&frustumVertices[6].x);
        glVertex3fv(&frustumVertices[7].x);
        glVertex3fv(&frustumVertices[3].x);
        // top
        glNormal3fv(&frustumNormals[3].x);
        glVertex3fv(&frustumVertices[0].x);
        glVertex3fv(&frustumVertices[4].x);
        glVertex3fv(&frustumVertices[5].x);
        glVertex3fv(&frustumVertices[1].x);
        // front
        glNormal3fv(&frustumNormals[4].x);
        glVertex3fv(&frustumVertices[0].x);
        glVertex3fv(&frustumVertices[1].x);
        glVertex3fv(&frustumVertices[2].x);
        glVertex3fv(&frustumVertices[3].x);
        // back
        glNormal3fv(&frustumNormals[5].x);
        glVertex3fv(&frustumVertices[7].x);
        glVertex3fv(&frustumVertices[6].x);
        glVertex3fv(&frustumVertices[5].x);
        glVertex3fv(&frustumVertices[4].x);
        glEnd();
    }
}


void gpsp_draw_robots_fov(G3D_Window  *win)
{

	if (win->win_perspective) // These features are not shown in a perspective window
		return;

	p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	p3d_rob *currobotPt;
	int i;
	for(i=0; i<envPt->nr; i++){
		currobotPt=envPt->robot[i];
		if (p3d_is_view_field_showed(currobotPt))
			g3d_draw_rob_cone(currobotPt);

	}	
	
	
}


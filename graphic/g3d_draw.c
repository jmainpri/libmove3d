#include "P3d-pkg.h"

#include "Graphic-pkg.h"

// RGB Couleurs utilisees 
double Whitev[4] =   { 1.0, 1.0, 1.0, 1.0 };
double Blackv[4] =   { 0.0, 0.0, 0.0, 1.0 };
double Bluev[4]  =   { 0.0, 0.0, 1.0, 1.0 };
double Redv[4]   =   { 1.0, 0.0, 0.0, 1.0 };
double Yellowv[4]=   { 1.0, 1.0, 0.0, 1.0 };
double Greenv[4] =   { 0.0, 1.0, 0.0, 1.0 };
double Greyv[4]  =   { 0.7, 0.7, 0.7, 1.0 };
double Brownv[4] =   { 1.0, 1.0, 0.5, 1.0 };
double Violetv[4]=   { 1.0, 0.0, 1.0, 1.0 };
double Blue2v[4] =   { 0.0, 1.0, 1.0, 1.0 };
double Skinv[4]  =   { 1.0, 0.81, 0.81, 1.0 };
double DGreyv[4] =   { 0.2, 0.2, 0.2, 1.0 };
double DSkinv[4] =   { 1.0, 0.5, 0.5, 1.0 };
double DBrownv[4]=   { 0.5, 0.5, 0.25, 1.0 };
double DGreenv[4]=   { 0.0, 0.25, 0.0, 1.0 };
double Orangev[4]=   { 1.0, 0.65, 0.0, 1.0 };


// RGBA couleurs utilisees pour la transparence
double tWhitev[4]  =  { 1.0, 1.0, 1.0, 0.5 };
double tBlackv[4]  =  { 0.0, 0.0, 0.0, 0.5 };
double tBluev[4]   =  { 0.0, 0.0, 1.0, 0.5 };
double tRedv[4]    =  { 1.0, 0.0, 0.0, 0.5 };
double tYellowv[4] =  { 1.0, 1.0, 0.0, 0.5 };
double tGreenv[4]  =  { 0.0, 1.0, 0.0, 0.5 };
double tGreyv[4]   =  { 0.7, 0.7, 0.7, 0.5 };
double tBrownv[4]  =  { 1.0, 1.0, 0.5, 0.5 };
double tVioletv[4] =  { 1.0, 0.0, 1.0, 0.5 };
double tBlue2v[4]  =  { 0.0, 1.0, 1.0, 0.4 };
double tSkinv[4]   =  { 1.0, 0.81, 0.81, 0.5 };
double tDGreyv[4]  =  { 0.2, 0.2, 0.2, 0.5 };
double tDSkinv[4]  =  { 1.0, 0.5, 0.5, 0.5 };
double tDBrownv[4] =  { 0.5, 0.5, 0.25, 0.5 };
double tDGreenv[4] =  { 0.0, 0.25, 0.0, 0.5 };
double tOrangev[4] =  { 1.0, 0.65, 0.0, 0.5 };

extern GLfloat matrix_pos_absGL[16];

static double G3D_COLOR_ARRAY[15][3]= {  {1,0,0}, {0,1,0}, {0,0,1}, {1,1,0}, {1,0,1}, {0,1,1} , {1,0.5,0.5}, {0.5,1,0.5}, {0.5,0.5,1}, {1,0.25,0.5}, {1,0.5,0.25}, {0.25,1.0,0.5}, {0.5,1,0.25}, {0.25,0.5,1}, {0.5,0.25,1}  };
static const int G3D_COLOR_ARRAY_SIZE= 15; 


/*******************************************************************************/

static
int mod(int a, int b) {
  int res;

  res = a - b*(int)floor(a/b);
  return(res);
}

/*******************************************************************************/
//! Converts a p3d pose matrix to an OpenGL one.
//! \param T the input p3d_matrix4
//! \param mat a float array that will be filled with the converted matrix
void p3d_to_gl_matrix(p3d_matrix4 T, GLfloat mat[16])
{
  mat[0]= T[0][0];      mat[4]= T[0][1];      mat[8]=  T[0][2];      mat[12]= T[0][3];
  mat[1]= T[1][0];      mat[5]= T[1][1];      mat[9]=  T[1][2];      mat[13]= T[1][3];
  mat[2]= T[2][0];      mat[6]= T[2][1];      mat[10]= T[2][2];      mat[14]= T[2][3];
  mat[3]=       0;      mat[7]=       0;      mat[11]=       0;      mat[15]=       1;
}

/*******************************************************************************/
//! @ingroup graphic 
//! Get the current display size and displays the text in
//! the right/bottom corner
//! \param string contains a string to draw
void g3d_draw_text(char* string)
{
	GLint viewport[4];
	
	glGetIntegerv(GL_VIEWPORT, viewport);
	
	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, viewport[2], 0, viewport[3], -1, 1);
	
	glTranslatef(viewport[2]-150,50, 0);
	glScalef(15.0, 15.0,1.0);
	YsDrawUglyFont(string, -1);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

/*******************************************************************************/
//! @ingroup graphic 
//! Sets the current OpenGL color from a color index or an RGBA vector.
//! \param color color index (see the #define in include/p3d_type.h)
//! \param color_vect a vector of RGBA components (used only if color==Any)
void g3d_set_color_vect(int color, GLdouble color_vect[4]) {

  switch(color) {
    case Blue:
      glColor4dv(Bluev);
      break;
    case Yellow:
      glColor4dv(Yellowv);
      break;
    case Red:
      glColor4dv(Redv);
      break;
    case Green:
      glColor4dv(Greenv);
      break;
    case White:
      glColor4dv(Whitev);
      break;
    case Grey:
      glColor4dv(Greyv);
      break;
    case Brown:
      glColor4dv(Brownv);
      break;
    case Skin:
      glColor4dv(Skinv);
      break;
    case Blue2:
      glColor4dv(Blue2v);
      break;
    case DGrey:
      glColor4dv(DGreyv);
      break;
    case DSkin:
      glColor4dv(DSkinv);
      break;
    case DBrown:
      glColor4dv(DBrownv);
      break;
    case DGreen:
      glColor4dv(DGreenv);
      break;
    case Black:
      glColor4dv(Blackv);
      break;
    case Violet:
      glColor4dv(Violetv);
      break;
    case Orange:
      glColor4dv(Orangev);
      break;

      /* Tranparence */
    case tBlue:
      glColor4dv(tBluev);
      break;
    case tYellow:
      glColor4dv(tYellowv);
      break;
    case tRed:
      glColor4dv(tRedv);
      break;
    case tGreen:
      glColor4dv(tGreenv);
      break;
    case tWhite:
      glColor4dv(tWhitev);
      break;
    case tGrey:
      glColor4dv(tGreyv);
      break;
    case tBrown:
      glColor4dv(tBrownv);
      break;
    case tSkin:
      glColor4dv(tSkinv);
      break;
    case tBlue2:
      glColor4dv(tBlue2v);
      break;
    case tDGrey:
      glColor4dv(tDGreyv);
      break;
    case tDSkin:
      glColor4dv(tDSkinv);
      break;
    case tDBrown:
      glColor4dv(tDBrownv);
      break;
    case tDGreen:
      glColor4dv(tDGreenv);
      break;
    case tBlack:
      glColor4dv(tBlackv);
      break;
    case tViolet:
      glColor4dv(tVioletv);
      break;
    case tOrange:
      glColor4dv(tOrangev);
      break;

    case Any:
      glColor4dv(color_vect);
    break;
  }
}

//! @ingroup graphic 
//! Fills a array with the RGBA coordinates of the specified color index.
//! \param color color index (see the #define in include/p3d_type.h)
//! \param color_vect the vector that will be filled with RGBA components
void g3d_get_color_vect(int color, GLdouble color_vect[4]) {
  unsigned int i;
  switch(color) {
    case Blue:
      for(i=0; i<4; i++)
      {  color_vect[i]= Bluev[i];  }
      break;
    case Yellow:
      for(i=0; i<4; i++)
      {  color_vect[i]= Yellowv[i];  }
      break;
    case Red:
      for(i=0; i<4; i++)
      {  color_vect[i]= Redv[i];  }
      break;
    case Green:
      for(i=0; i<4; i++)
      {  color_vect[i]= Greenv[i];  }
      break;
    case White:
      for(i=0; i<4; i++)
      {  color_vect[i]= Whitev[i];  }
      break;
    case Grey:
      for(i=0; i<4; i++)
      {  color_vect[i]= Greyv[i];  }
      break;
    case Brown:
      for(i=0; i<4; i++)
      {  color_vect[i]= Brownv[i];  }
      break;
    case Skin:
      for(i=0; i<4; i++)
      {  color_vect[i]= Skinv[i];  }
      break;
    case Blue2:
      for(i=0; i<4; i++)
      {  color_vect[i]= Blue2v[i];  }
      break;
    case DGrey:
      for(i=0; i<4; i++)
      {  color_vect[i]= DGreyv[i];  }
      break;
    case DSkin:
      for(i=0; i<4; i++)
      {  color_vect[i]= DSkinv[i];  }
      break;
    case DBrown:
      for(i=0; i<4; i++)
      {  color_vect[i]= DBrownv[i];  }
      break;
    case DGreen:
      for(i=0; i<4; i++)
      {  color_vect[i]= DGreenv[i];  }
      break;
    case Black:
      for(i=0; i<4; i++)
      {  color_vect[i]= Blackv[i];  }
      break;
    case Violet:
      for(i=0; i<4; i++)
      {  color_vect[i]= Violetv[i];  }
      break;
    case Orange:
      for(i=0; i<4; i++)
      {  color_vect[i]= Orangev[i];  }
      break;

      /* Transparence */
    case tBlue:
      for(i=0; i<4; i++)
      {  color_vect[i]= tBluev[i];  }
      break;
    case tYellow:
      for(i=0; i<4; i++)
      {  color_vect[i]= tYellowv[i];  }
      break;
    case tRed:
      for(i=0; i<4; i++)
      {  color_vect[i]= tRedv[i];  }
      break;
    case tGreen:
      for(i=0; i<4; i++)
      {  color_vect[i]= tGreenv[i];  }
      break;
    case tWhite:
      for(i=0; i<4; i++)
      {  color_vect[i]= tWhitev[i];  }
      break;
    case tGrey:
      for(i=0; i<4; i++)
      {  color_vect[i]= tGreyv[i];  }
      break;
    case tBrown:
      for(i=0; i<4; i++)
      {  color_vect[i]= tBrownv[i];  }
      break;
    case tSkin:
      for(i=0; i<4; i++)
      {  color_vect[i]= tSkinv[i];  }
      break;
    case tBlue2:
      for(i=0; i<4; i++)
      {  color_vect[i]= tBlue2v[i];  }
      break;
    case tDGrey:
      for(i=0; i<4; i++)
      {  color_vect[i]= tDGreyv[i];  }
      break;
    case tDSkin:
      for(i=0; i<4; i++)
      {  color_vect[i]= tDSkinv[i];  }
      break;
    case tDBrown:
      for(i=0; i<4; i++)
      {  color_vect[i]= tDBrownv[i];  }
      break;
    case tDGreen:
      for(i=0; i<4; i++)
      {  color_vect[i]= tDGreenv[i];  }
      break;
    case tBlack:
      for(i=0; i<4; i++)
      {  color_vect[i]= tBlackv[i];  }
      break;
    case tViolet:
      for(i=0; i<4; i++)
      {  color_vect[i]= tVioletv[i];  }
      break;
    case tOrange:
      for(i=0; i<4; i++)
      {  color_vect[i]= tOrangev[i];  }
      break;
  }
}

//! @ingroup graphic 
//! This function does the same thing than g3d_set_color_vect().
void g3d_set_color(int color, double color_vect[4]) {
  double color_array[4];

  if(color!=Any)
  {
    g3d_get_color_vect(color, color_array);
  }
  else
  {
    color_array[0]= color_vect[0];
    color_array[1]= color_vect[1];
    color_array[2]= color_vect[2];
    color_array[3]= color_vect[3];
  }

  glColor4dv(color_array);
}

GLdouble m_color_vect[4];

//! @ingroup graphic 
//! Stores one color to be called latter to draw a robot for example
//! \param draw_custom TRUE or FALSE
void g3d_set_custom_color_draw(p3d_rob* r, int draw_custom)
{
  r->draw_custom_color = draw_custom;
}

//! @ingroup graphic 
//! Stores one color to be called latter to draw a robot for example
//! \param color_vect the vector that will be filled with RGBA components
void g3d_set_custom_color_vect(GLdouble color_vect[4])
{
  m_color_vect[0] = color_vect[0];
  m_color_vect[1] = color_vect[1];
  m_color_vect[2] = color_vect[2];
  m_color_vect[3] = color_vect[3];
}

//! @ingroup graphic 
//! Stores one color to be called latter to draw a robot for example
//! \param color_vect the vector that will be filled with RGBA components
void g3d_get_custom_color_vect(GLdouble color_vect[4])
{
  color_vect[0] = m_color_vect[0];
  color_vect[1] = m_color_vect[1];
  color_vect[2] = m_color_vect[2];
  color_vect[3] = m_color_vect[3];
}

//! @ingroup graphic
/****************************************************************************************************/
void g3d_drawDisc(double x,double y,double z, float r, int color, GLdouble color_vect[4]) {
  double angle;
  GLint circle_points = 30;
  int i;

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  g3d_set_color(color,color_vect);

  glBegin(GL_POLYGON);
  for (i = 0; i < circle_points; i++) {
    angle = 2*M_PI*i/circle_points;
    glVertex3d(x+r*cos(angle), y+r*sin(angle),z);
  }
  glEnd();
}

//! @ingroup graphic
//! Draws a sphere without changing the current color.
void g3d_drawSphere(double x,double y,double z, float r) {
  GLint circle_points = 8;
  double angle1=M_PI/circle_points, angle2=2*M_PI/circle_points;
  int i,j;


  for (i=0;i<=circle_points-2;i++) {

    if(i==0) {
      for (j=0;j<=circle_points-1;j++) {
        glBegin(GL_POLYGON);
        glVertex3d(x,y,z+r);
        glVertex3d(x+r*cos(M_PI/2.-i*angle1)*cos(j*angle2),y+r*cos(M_PI/2.-i*angle1)*sin(j*angle2),z+r*sin(M_PI/2.-i*angle1));
        glVertex3d(x+r*cos(M_PI/2.-i*angle1)*cos((j+1)*angle2),y+r*cos(M_PI/2.-i*angle1)*sin((j+1)*angle2),z+r*sin(M_PI/2.-i*angle1));
        glEnd();
      }
    }
    if(i==circle_points-2) {
      for (j=0;j<=circle_points-1;j++) {
        glBegin(GL_POLYGON);
        glVertex3d(x+r*cos(M_PI/2.-i*angle1)*cos(j*angle2),y+r*cos(M_PI/2.-i*angle1)*sin(j*angle2),z+r*sin(M_PI/2.-i*angle1));
        glVertex3d(x+r*cos(M_PI/2.-i*angle1)*cos((j+1)*angle2),y+r*cos(M_PI/2.-i*angle1)*sin((j+1)*angle2),z+r*sin(M_PI/2.-i*angle1));
        glVertex3d(x,y,z-r);
        glEnd();
      }
    } else {
      for (j=0;j<=circle_points-1;j++) {
        glBegin(GL_POLYGON);
        glVertex3d(x+r*cos(M_PI/2.-i*angle1)*cos(j*angle2),y+r*cos(M_PI/2.-i*angle1)*sin(j*angle2),z+r*sin(M_PI/2.-i*angle1));
        glVertex3d(x+r*cos(M_PI/2.-i*angle1)*cos((j+1)*angle2),y+r*cos(M_PI/2.-i*angle1)*sin((j+1)*angle2),z+r*sin(M_PI/2.-i*angle1));
        glVertex3d(x+r*cos(M_PI/2.-(i+1)*angle1)*cos((j+1)*angle2),y+r*cos(M_PI/2.-(i+1)*angle1)*sin((j+1)*angle2),z+r*sin(M_PI/2.-(i+1)*angle1));
        glVertex3d(x+r*cos(M_PI/2.-(i+1)*angle1)*cos(j*angle2),y+r*cos(M_PI/2.-(i+1)*angle1)*sin(j*angle2),z+r*sin(M_PI/2.-(i+1)*angle1));
        glEnd();
      }
    }
  }
}

//! @ingroup graphic
//! Draws a sphere with the specified color.
void g3d_drawColorSphere(double x,double y,double z, float r, int color, GLdouble color_vect[4]) {
  g3d_set_color(color, color_vect);
  g3d_drawSphere(x,y,z, r);
}


//! @ingroup graphic
void g3d_drawCircle(double x,double y, double r, int color, double *color_vect, double width) {
  GLint circle_points = 30;
  int i;
  double a;
  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);
  g3d_set_color_vect(color,color_vect);
  glLineWidth(width);
  glBegin(GL_LINE_LOOP);
  for (i=0;i<circle_points;i++) {
    a = 2*M_PI*i/circle_points;
    glVertex2d(x+r*cos(a), y+r*sin(a));
  }
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

}

/********************************************************/
/* Fonction tracant une ligne 3D de couleur color       */
/********************************************************/
//! @ingroup graphic
void g3d_drawOneLine(double x1,double y1,double z1,double x2,double y2,double z2,int color,double *color_vect) {

  glPushAttrib(GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);

  g3d_set_color_vect(color, color_vect);

  glBegin(GL_LINES);
   glVertex3d(x1, y1, z1);
   glVertex3d(x2, y2, z2);
  glEnd();

  glPopAttrib();
}

//! Draws a simple rectangle with its normal vector parallel to z axis.
//! \param bottomLeftCornerX x coordinate of the bottom left corner of the rectangle when seen from the top
//! \param bottomLeftCornerX y coordinate of the bottom left corner of the rectangle when seen from the top
//! \param z z coordinate of the rectangle
//! \param dimX side length of the rectangle along x axis
//! \param dimY side length of the rectangle along y axis
void g3d_draw_rectangle(float bottomLeftCornerX, float bottomLeftCornerY, float z, float dimX, float dimY)
{
  glPushAttrib(GL_ENABLE_BIT);
	
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
	
  glBegin(GL_QUADS);
	glNormal3f(0.0, 0.0, 1.0);
	glVertex3f(bottomLeftCornerX, bottomLeftCornerY, z);
	glNormal3f(0.0, 0.0, 1.0);
	glVertex3f(bottomLeftCornerX + dimX, bottomLeftCornerY, z);
	glNormal3f(0.0, 0.0, 1.0);
	glVertex3f(bottomLeftCornerX + dimX, bottomLeftCornerY + dimY, z);
	glNormal3f(0.0, 0.0, 1.0);
	glVertex3f(bottomLeftCornerX, bottomLeftCornerY + dimY, z);
  glEnd();
	
  glPopAttrib();
}

//! Draws a tesselated rectangle with its normal vector parallel to z axis.
//! \param bottomLeftCornerX x coordinate of the bottom left corner of the rectangle when seen from the top
//! \param bottomLeftCornerX y coordinate of the bottom left corner of the rectangle when seen from the top
//! \param z z coordinate of the rectangle
//! \param dimX side length of the rectangle along x axis
//! \param dimY side length of the rectangle along y axis
//! \param delta side length of the small squares used to tesselate the rectangle
void g3d_draw_tesselated_rectangle(float bottomLeftCornerX, float bottomLeftCornerY, float z, float dimX, float dimY, float delta)
{
  unsigned int i, j, nx, ny;
  float x1, x2, y, xmax, ymax;
	
  xmax= bottomLeftCornerX + dimX;
  ymax= bottomLeftCornerY + dimY;
	
  nx= (unsigned int) ceil(dimX/delta);
  ny= (unsigned int) ceil(dimY/delta);
	
  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);
	
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glShadeModel(GL_SMOOTH);
	
  for(i=0; i<=nx; i++)
  {
    x1= bottomLeftCornerX + i*delta;
    if(x1>xmax)
    {  break;  }
    x2= bottomLeftCornerX + (i+1)*delta;
    if(x2>xmax)
    {  x2= xmax;  }
		
    glBegin(GL_TRIANGLE_STRIP);
		for(j=0; j<=ny; j++)
		{
			y= bottomLeftCornerY + j*delta;
			
			if(y>ymax)
			{  y= ymax; }
			
			glNormal3f(0.0f, 0.0f, 1.0f);
			glTexCoord2f(x1/dimX, y/dimY);
			glVertex3f(x1, y, z);
			glTexCoord2f(x2/dimX, y/dimY);
			glVertex3f(x2, y, z);
		}
    glEnd();
  }
	
  glPopAttrib();
}


//! Displays an axis-aligned wire box.
//! \param xmin smallest coordinate of the box along X-axis
//! \param xmax biggest coordinate of the box along X-axis
//! \param ymin smallest coordinate of the box along Y-axis
//! \param ymax biggest coordinate of the box along Y-axis
//! \param zmin smallest coordinate of the box along Z-axis
//! \param zmax biggest coordinate of the box along Z-axis
void g3d_draw_AA_box(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) {
	
  glPushAttrib(GL_LINE_BIT);
  glLineWidth(3);
	
  glColor3f(0.0, 0.0, 0.0);
	
  glBegin(GL_LINES);
  glVertex3f(xmin, ymin, zmin);
  glVertex3f(xmin, ymin, zmax);
	
  glVertex3f(xmax, ymin, zmin);
  glVertex3f(xmax, ymin, zmax);
	
  glVertex3f(xmax,  ymax, zmin);
  glVertex3f(xmax,  ymax, zmax);
	
  glVertex3f(xmin, ymax, zmin);
  glVertex3f(xmin, ymax, zmax);
  glEnd();
	
	
  glBegin(GL_LINE_LOOP);
  glVertex3f(xmin, ymin, zmin);
  glVertex3f(xmax, ymin, zmin);
  glVertex3f(xmax, ymax, zmin);
  glVertex3f(xmin, ymax, zmin);
  glEnd();
	
  glBegin(GL_LINE_LOOP);
  glVertex3f(xmin, ymin, zmax);
  glVertex3f(xmax, ymin, zmax);
  glVertex3f(xmax, ymax, zmax);
  glVertex3f(xmin, ymax, zmax);
  glEnd();
	
  glPopAttrib();
}

//! @ingroup graphic
void g3d_drawSphMoveObj(p3d_matrix4 mat ,double length){
  p3d_vector3 axis;
  double t = 0;
  p3d_mat4ExtractRot(mat, axis, &t);
  glPushMatrix();
  glTranslatef(mat[0][3],mat[1][3],mat[2][3]);
  glRotatef((180/M_PI)*t, axis[0], axis[1], axis[2]);
  glLoadName(-4);

  glPushMatrix();
  glRotatef(90,0,1,0);
  g3d_drawCircle(0,0, length, Red, NULL, 2);
  glPopMatrix();

  glLoadName(-5);
  glPushMatrix();
  glRotatef(90,1,0,0);
  g3d_drawCircle(0,0, length, Green, NULL, 2);
  glPopMatrix();

  glLoadName(-6);
  g3d_drawCircle(0,0, length, Blue, NULL, 2);
  glPopMatrix();
}


/**
 *
 * @param frame the Rep position and orientation
 * @param length the lenght of each arrow
 * @param axis the axis to draw : for x = 1 for y = 2 for z = 4. To draw two axis, add their Id
 */
//! @ingroup graphic
void g3d_drawRepMoveObj(p3d_matrix4 frame ,double length, int axis) {
  p3d_vector3 origin, xAxis, yAxis, zAxis;

  origin[0]= frame[0][3];
  origin[1]= frame[1][3];
  origin[2]= frame[2][3];

  xAxis[0]=  origin[0] + length*frame[0][0];
  xAxis[1]=  origin[1] + length*frame[1][0];
  xAxis[2]=  origin[2] + length*frame[2][0];

  yAxis[0]=  origin[0] + length*frame[0][1];
  yAxis[1]=  origin[1] + length*frame[1][1];
  yAxis[2]=  origin[2] + length*frame[2][1];

  zAxis[0]=  origin[0] + length*frame[0][2];
  zAxis[1]=  origin[1] + length*frame[1][2];
  zAxis[2]=  origin[2] + length*frame[2][2];

  switch(axis){
    case 1:{
      glLoadName(-1);
      g3d_draw_arrow(origin, xAxis, 1, 0, 0);
      break;
    }
    case 2:{
      glLoadName(-1);
      g3d_draw_arrow(origin, yAxis, 0.0, 1.0, 0.0);
      break;
    }
    case 3:{
      glLoadName(-1);
      g3d_draw_arrow(origin, xAxis, 1, 0, 0);
      glLoadName(-2);
      g3d_draw_arrow(origin, yAxis, 0.0, 1.0, 0.0);
      break;
    }
    case 4:{
      glLoadName(-1);
      g3d_draw_arrow(origin, zAxis, 0.0, 0.0, 1.0);
      break;
    }
    case 5:{
      glLoadName(-1);
      g3d_draw_arrow(origin, xAxis, 1, 0, 0);
      glLoadName(-2);
      g3d_draw_arrow(origin, zAxis, 0.0, 0.0, 1.0);
      break;
    }
    case 6:{
      glLoadName(-1);
      g3d_draw_arrow(origin, yAxis, 0.0, 1.0, 0.0);
      glLoadName(-2);
      g3d_draw_arrow(origin, zAxis, 0.0, 0.0, 1.0);
      break;
    }
    case 7:{
      glLoadName(-1);
      g3d_draw_arrow(origin, xAxis, 1, 0, 0);
      glLoadName(-2);
      g3d_draw_arrow(origin, yAxis, 0.0, 1.0, 0.0);
      glLoadName(-3);
      g3d_draw_arrow(origin, zAxis, 0.0, 0.0, 1.0);
      break;
    }
  }
}

//! @ingroup graphic
//! Draws a 3D arrow.
//! \param p1 arrow's starting point
//! \param p2 arrow's ending point
//! \param red red component of the arrow's color
//! \param green green component of the arrow's color
//! \param blue blue component of the arrow's color
void g3d_draw_arrow(p3d_vector3 p1, p3d_vector3 p2, double red, double green, double blue)
{
   double length, cone_height;
   p3d_vector3 p;
   p[0]= p2[0] - p1[0];
   p[1]= p2[1] - p1[1];
   p[2]= p2[2] - p1[2];
   length= sqrt( p[0]*p[0] + p[1]*p[1] + p[2]*p[2] );
   p[0]/= length;
   p[1]/= length;
   p[2]/= length;

   cone_height= 0.4* length;

   glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
   glLineWidth(5);
   glDisable(GL_LIGHTING);
   glColor3d(red, green, blue);
   glBegin(GL_LINES);
      glVertex3d(p1[0], p1[1], p1[2]);
      glVertex3d(p2[0]-cone_height*p[0], p2[1]-cone_height*p[1], p2[2]-cone_height*p[2]);
   glEnd();
   glEnable(GL_LIGHTING);

   double color[]= {red, green, blue};
   g3d_set_color(Any, color);

   glPushMatrix();
     glTranslatef(p2[0]-0.05*length*p[0], p2[1]-0.05*length*p[1], p2[2]-0.05*length*p[2]);
     if( sqrt(p[0]*p[0]+p[1]*p[1]) > 1e-9 )
     {  glRotatef((180.0/M_PI)*asin(p[2]) - 90, p[1], -p[0], 0);  }
     else
     {
        if( p[2] < 0 )
        { glRotatef(180, 1, 0, 0); }
     }
     g3d_draw_solid_cone(0.3*cone_height, cone_height, 6);
   glPopMatrix();

   glPopAttrib();
}

//! @ingroup graphic
//! Cette fonction dessine un cone solide -dont les facettes sont
//! remplies- d'axe z et dont la pointe est en (0,0,0).
//! A utiliser dans une fonction d'affichage OpenGL.
void g3d_draw_solid_cone(double radius, double height, int nbSegments)
{
   int i, j;
   double *sint, *cost, z, dz, dr;
   double alpha= atan(height/radius);
   double ca= cos(alpha);
   double sa= sin(alpha);
   g3d_circle_table(&sint, &cost, -nbSegments);
   z= height/2;
   int nbSegments2= nbSegments;

   dz= height/nbSegments2;
   dr= radius*dz/height;
   //Les triangles des cÃ´tes:
   glBegin(GL_TRIANGLE_STRIP);
     for(i=0; i<nbSegments2; i++)
     {
       for(j=nbSegments; j>=0; j--)
       {
         glNormal3d(cost[j]*sa, sint[j]*sa, ca);
         glTexCoord2d(1-j/(double)nbSegments, 1.0f);
         glVertex3d(cost[j]*i*dr, sint[j]*i*dr, z-i*dz);
         glNormal3d(cost[j]*sa, sint[j]*sa, ca);
         glTexCoord2d(1-j/(double)nbSegments, 0.0f);
         glVertex3d(cost[j]*(i+1)*dr, sint[j]*(i+1)*dr, z-(i+1)*dz);
       }
     }
   glEnd();

   //Les triangles du dessous:
   glBegin(GL_TRIANGLE_FAN);
    glNormal3d(0, 0, -1);
    glTexCoord2d(0, 0.0f);
    glVertex3d(0, 0, -z);
    for(i=0; i<=nbSegments; i++)
    { glTexCoord2d(1-i/(double)nbSegments, 1.0f);
      glVertex3d(cost[i]*radius, sint[i]*radius, -z);
    }
   glEnd();

   free(sint);
   free(cost);
}

//! @ingroup graphic
/********************************************************/
/* Fonction tracant un repere de taille a materialisant */
/* l'objet o et de couleur num(modulo 6)                */
/********************************************************/
void g3d_draw_rep_obj(p3d_jnt *jnt,double a,int num) {
  double a1,a9;
  GLfloat        matrix[16];
  int i,j;

  int color;

  /* choice of color */
  //color = mod(num,6)+2;  // modif Juan (no red frames in RRT)
  color = mod(num,6);

  switch(color) {
    case 0:
      g3d_set_color_vect(Red,NULL);
      break;
    case 1:
      g3d_set_color_vect(Green,NULL);
      break;
    case 2:
      g3d_set_color_vect(Blue,NULL);
      break;
    case 3:
      g3d_set_color_vect(Yellow,NULL);
      break;
    case 4:
      g3d_set_color_vect(Blue2,NULL);
      break;
    case 5:
      g3d_set_color_vect(Violet,NULL);
      break;
    case 6:
      g3d_set_color_vect(Black,NULL);
      break;
  }

  for(i=0 ; i<=3 ; i++) {
    for(j=0 ; j<=3 ; j++) {
      matrix[4*j+i]=jnt->abs_pos[i][j];
    }
  }

  a1 = 0.1*a; a9 = 0.9*a;

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT | GL_LINE_BIT);

  glPushMatrix();
  glMultMatrixf(matrix);

  glDisable(GL_LIGHTING);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glVertex3d(0,0,0);    glVertex3d(0,0,a);
  glVertex3d(0,0,0);    glVertex3d(a,0,0);
  glVertex3d(0,0,0);    glVertex3d(0,a,0);
  glEnd();
  glLineWidth(1.0);

  glEnable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glBegin(GL_POLYGON);
  glVertex3d(0,0,a); glVertex3d(-a1,0,a9); glVertex3d(a1,0,a9);
  glEnd();

  glBegin(GL_POLYGON);
  glVertex3d(a,0,0); glVertex3d(a9,0,-a1); glVertex3d(a9,0,a1);
  glEnd();

  glBegin(GL_POLYGON);
  glVertex3d(0,a,0); glVertex3d(0,a9,-a1); glVertex3d(0,a9,a1);
  glEnd();

  glPopMatrix();

  glPopAttrib();
}

//! @ingroup graphic
//! Fonction tracant une boite en couleur donnee
void g3d_draw_simple_box(double x1,double x2,double y1,
                    double y2,double z1,double z2, int color, int fill, double width) {
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  if(fill == 0) {
    /* filaire */
    g3d_set_color(color, NULL);
  }
  
  glPushAttrib(GL_LINE_BIT);
  glLineWidth(width);
  glBegin(GL_LINES);
  glVertex3f(x1, y1, z1);
  glVertex3f(x1, y1, z2);

  glVertex3f(x2, y1, z1);
  glVertex3f(x2, y1, z2);

  glVertex3f(x2,  y2, z1);
  glVertex3f(x2,  y2, z2);

  glVertex3f(x1, y2, z1);
  glVertex3f(x1, y2, z2);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y1, z1);
  glVertex3f(x2, y2, z1);
  glVertex3f(x1, y2, z1);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(x1, y1, z2);
  glVertex3f(x2, y1, z2);
  glVertex3f(x2, y2, z2);
  glVertex3f(x1, y2, z2);
  glEnd();

  glPopAttrib();
}

//! @ingroup graphic
//! Fonction tracant une boite en couleur donnee
void g3d_draw_a_box(double x1,double x2,double y1,
                    double y2,double z1,double z2, int c, int fill) {
  double namplx,namply,namplz,tempx,tempy,tempz;
  int i,n=5;
  int blend = 0; /* Flag pour le reglage des parametres de transparence*/

  namplx=(x2-x1)/n;
  namply=(y2-y1)/n;
  namplz=(z2-z1)/n;

  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);


  if(fill == 0) {
    /* filaire */
    if(c!=Any) 
    g3d_set_color_vect ( c, NULL );

    /* Activation de la fonction de transparence */
    if(blend) {
      glPushMatrix();
      glEnable(GL_BLEND);
      glDepthMask(GL_FALSE);
      glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
    }
  }
  glLineWidth(1);
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

  glBegin(GL_LINES);
  {
    for(i=1;i<=n-1;i++) {
      tempx=x1+i*namplx;
      glVertex3d(tempx,y1,z1);
      glVertex3d(tempx,y2,z1);
      glVertex3d(tempx,y1,z2);
      glVertex3d(tempx,y2,z2);
      glVertex3d(tempx,y1,z1);
      glVertex3d(tempx,y1,z2);
      glVertex3d(tempx,y2,z1);
      glVertex3d(tempx,y2,z2);

      tempy=y1+i*namply;
      glVertex3d(x1,tempy,z1);
      glVertex3d(x2,tempy,z1);
      glVertex3d(x1,tempy,z2);
      glVertex3d(x2,tempy,z2);
      glVertex3d(x1,tempy,z1);
      glVertex3d(x1,tempy,z2);
      glVertex3d(x2,tempy,z1);
      glVertex3d(x2,tempy,z2);

      tempz=z1+i*namplz;
      glVertex3d(x1,y1,tempz);
      glVertex3d(x1,y2,tempz);
      glVertex3d(x2,y1,tempz);
      glVertex3d(x2,y2,tempz);
      glVertex3d(x1,y1,tempz);
      glVertex3d(x2,y1,tempz);
      glVertex3d(x1,y2,tempz);
      glVertex3d(x2,y2,tempz);
    }
  }
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  /* desactivation du mode de transparence*/
  if(blend) {
    glDepthMask (GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();
  }
}

//! @ingroup graphic
/***************************************************************************/
/* Fonction qui teste si la box englobante appartient au frustrum de vue   */
/*                                                                         */
/* IN => o : pointeur sur l'objet courant                                  */
/*       win: pointeur sur la structure fenetre courante                   */
/*                                                                         */
/* OUT => 1 : la boite est partiellement ou totalement dans le frustum     */
/*     => 0 : sinon                                                        */
/***************************************************************************/
int BoxInFrustum_obj(p3d_obj *o,G3D_Window *win) {
  int plan;
  double xmin,xmax,ymin,ymax,zmin,zmax;
  float bboxcenter_x,bboxcenter_y,bboxcenter_z;
  float halfsize_x,halfsize_y,halfsize_z;

  /***** recuperation du bounding box du polyhedre *****/

  p3d_get_BB_obj(o,&xmin,&xmax,&ymin,&ymax,&zmin,&zmax);

  /**** calcul du centre de la bounding box et des demi-distances au centre ****/
  bboxcenter_x = (xmax + xmin)/2.;
  bboxcenter_y = (ymax + ymin)/2.;
  bboxcenter_z = (zmax + zmin)/2.;

  halfsize_x = (xmax - bboxcenter_x);
  halfsize_y = (ymax - bboxcenter_y);
  halfsize_z = (zmax - bboxcenter_z);

  for(plan = 0; plan < 6; plan++ ) // pour tous les plans
  {
    // si un des points est dans le frustum, alors on peut passer au plan suivant
    if(win->vs.frustum[plan][0] * (bboxcenter_x - halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y - halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z - halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x + halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y - halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z - halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x - halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y + halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z - halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x + halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y + halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z - halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x - halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y - halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z + halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x + halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y - halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z + halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x - halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y + halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z + halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;

    if(win->vs.frustum[plan][0] * (bboxcenter_x + halfsize_x) + win->vs.frustum[plan][1] * (bboxcenter_y + halfsize_y)
        + win->vs.frustum[plan][2] * (bboxcenter_z + halfsize_z) + win->vs.frustum[plan][3] > 0 ) continue;
    return 0;
  }
  return 1;
}





//! @ingroup graphic
/*******************************************************************************/
/* Fonction qui teste si une sphere englobante appartient au frustrum de vue   */
/*                                                                             */
/* IN => win : pointeur sur la structure fenetre courante                      */
/*       x,y,z : coordonees absolues du centre de la sphere                    */
/*       radius : rayon de la sphere a afficher                                */
/*                                                                             */
/* OUT => 1 : si la sphere est patiellement ou totalement dans le frustum      */
/*        0: sinon                                                             */
/*******************************************************************************/
int SphereInFrustum( G3D_Window *win, float x, float y, float z, float radius ) {
  int p;
  int dedans = 1;
  int c = 0;  /* le nombre de fois qu on est dans le frustum :
     si on y est 6 fois, on est completement
     si on y est 0 fois, on est en dehors
     sinon, on est partiellement dedans */
  float d;

  for( p = 0; (p < 6) && dedans; p++ )  // pour tous les plans
  {
    d = win->vs.frustum[p][0] * x +
        win->vs.frustum[p][1] * y +
        win->vs.frustum[p][2] * z +
        win->vs.frustum[p][3];

    if( d <= -radius )  // si on est en dehors d un des plans
      dedans = 0;   // on est en dehors
    if( d > radius )    // sinon on incremente le nombre de fois qu on est dans le plan
      c++;
  }
  return ((c <= 6)&&(c > 0)) ? 1 : 0; // voir plus haut pour comprendre ce test
}

//! @ingroup graphic
/*******************************************************/
/* Fonction qui permet d'extraire le frustrum de vue   */
/*                                                     */
/* IN => win : pointeur vers la structure fenetre      */
/*******************************************************/
void g3d_extract_frustum(G3D_Window *win) {

  float proj[16];
  float modl[16];
  float clip[16];
  float t;



  glGetFloatv( GL_MODELVIEW_MATRIX, modl);
  glGetFloatv( GL_PROJECTION_MATRIX, proj);

  /* On combine les 2, en multipliant la matrice de projection par celle de modelisation*/
  clip[ 0] = modl[ 0] * proj[ 0] + modl[ 1] * proj[ 4] +
             modl[ 2] * proj[ 8] + modl[ 3] * proj[12];
  clip[ 1] = modl[ 0] * proj[ 1] + modl[ 1] * proj[ 5] +
             modl[ 2] * proj[ 9] + modl[ 3] * proj[13];
  clip[ 2] = modl[ 0] * proj[ 2] + modl[ 1] * proj[ 6] +
             modl[ 2] * proj[10] + modl[ 3] * proj[14];
  clip[ 3] = modl[ 0] * proj[ 3] + modl[ 1] * proj[ 7] +
             modl[ 2] * proj[11] + modl[ 3] * proj[15];

  clip[ 4] = modl[ 4] * proj[ 0] + modl[ 5] * proj[ 4] +
             modl[ 6] * proj[ 8] + modl[ 7] * proj[12];
  clip[ 5] = modl[ 4] * proj[ 1] + modl[ 5] * proj[ 5] +
             modl[ 6] * proj[ 9] + modl[ 7] * proj[13];
  clip[ 6] = modl[ 4] * proj[ 2] + modl[ 5] * proj[ 6] +
             modl[ 6] * proj[10] + modl[ 7] * proj[14];
  clip[ 7] = modl[ 4] * proj[ 3] + modl[ 5] * proj[ 7] +
             modl[ 6] * proj[11] + modl[ 7] * proj[15];

  clip[ 8] = modl[ 8] * proj[ 0] + modl[ 9] * proj[ 4] +
             modl[10] * proj[ 8] + modl[11] * proj[12];
  clip[ 9] = modl[ 8] * proj[ 1] + modl[ 9] * proj[ 5] +
             modl[10] * proj[ 9] + modl[11] * proj[13];
  clip[10] = modl[ 8] * proj[ 2] + modl[ 9] * proj[ 6] +
             modl[10] * proj[10] + modl[11] * proj[14];
  clip[11] = modl[ 8] * proj[ 3] + modl[ 9] * proj[ 7] +
             modl[10] * proj[11] + modl[11] * proj[15];

  clip[12] = modl[12] * proj[ 0] + modl[13] * proj[ 4] +
             modl[14] * proj[ 8] + modl[15] * proj[12];
  clip[13] = modl[12] * proj[ 1] + modl[13] * proj[ 5] +
             modl[14] * proj[ 9] + modl[15] * proj[13];
  clip[14] = modl[12] * proj[ 2] + modl[13] * proj[ 6] +
             modl[14] * proj[10] + modl[15] * proj[14];
  clip[15] = modl[12] * proj[ 3] + modl[13] * proj[ 7] +
             modl[14] * proj[11] + modl[15] * proj[15];

  /* Extrait le plan de DROITE */
  win->vs.frustum[0][0] = clip[ 3] - clip[ 0];
  win->vs.frustum[0][1] = clip[ 7] - clip[ 4];
  win->vs.frustum[0][2] = clip[11] - clip[ 8];
  win->vs.frustum[0][3] = clip[15] - clip[12];

  /* Calcul des normales */
  t = sqrt( win->vs.frustum[0][0] * win->vs.frustum[0][0] +
            win->vs.frustum[0][1] * win->vs.frustum[0][1] +
            win->vs.frustum[0][2] * win->vs.frustum[0][2] );

  win->vs.frustum[0][0] /= t;
  win->vs.frustum[0][1] /= t;
  win->vs.frustum[0][2] /= t;
  win->vs.frustum[0][3] /= t;

  /* Extrait le plan de GAUCHE */
  win->vs.frustum[1][0] = clip[ 3] + clip[ 0];
  win->vs.frustum[1][1] = clip[ 7] + clip[ 4];
  win->vs.frustum[1][2] = clip[11] + clip[ 8];
  win->vs.frustum[1][3] = clip[15] + clip[12];

  /* Calcul des normales */
  t = sqrt( win->vs.frustum[1][0] * win->vs.frustum[1][0] +
            win->vs.frustum[1][1] * win->vs.frustum[1][1] +
            win->vs.frustum[1][2] * win->vs.frustum[1][2] );

  win->vs.frustum[1][0] /= t;
  win->vs.frustum[1][1] /= t;
  win->vs.frustum[1][2] /= t;
  win->vs.frustum[1][3] /= t;

  /* Extrait le plan du BAS */
  win->vs.frustum[2][0] = clip[ 3] + clip[ 1];
  win->vs.frustum[2][1] = clip[ 7] + clip[ 5];
  win->vs.frustum[2][2] = clip[11] + clip[ 9];
  win->vs.frustum[2][3] = clip[15] + clip[13];

  /* Calcul des normales */
  t = sqrt( win->vs.frustum[2][0] * win->vs.frustum[2][0] +
            win->vs.frustum[2][1] * win->vs.frustum[2][1] +
            win->vs.frustum[2][2] * win->vs.frustum[2][2] );

  win->vs.frustum[2][0] /= t;
  win->vs.frustum[2][1] /= t;
  win->vs.frustum[2][2] /= t;
  win->vs.frustum[2][3] /= t;

  /* Extrait le plan du HAUT */
  win->vs.frustum[3][0] = clip[ 3] - clip[ 1];
  win->vs.frustum[3][1] = clip[ 7] - clip[ 5];
  win->vs.frustum[3][2] = clip[11] - clip[ 9];
  win->vs.frustum[3][3] = clip[15] - clip[13];

  /* Calcul des normales */
  t = sqrt( win->vs.frustum[3][0] * win->vs.frustum[3][0] +
            win->vs.frustum[3][1] * win->vs.frustum[3][1] +
            win->vs.frustum[3][2] * win->vs.frustum[3][2] );

  win->vs.frustum[3][0] /= t;
  win->vs.frustum[3][1] /= t;
  win->vs.frustum[3][2] /= t;
  win->vs.frustum[3][3] /= t;

  /* Extrait le plan ELOIGNE */
  win->vs.frustum[4][0] = clip[ 3] - clip[ 2];
  win->vs.frustum[4][1] = clip[ 7] - clip[ 6];
  win->vs.frustum[4][2] = clip[11] - clip[10];
  win->vs.frustum[4][3] = clip[15] - clip[14];

  /* Calcul des normales */
  t = sqrt( win->vs.frustum[4][0] * win->vs.frustum[4][0] +
            win->vs.frustum[4][1] * win->vs.frustum[4][1] +
            win->vs.frustum[4][2] * win->vs.frustum[4][2] );

  win->vs.frustum[4][0] /= t;
  win->vs.frustum[4][1] /= t;
  win->vs.frustum[4][2] /= t;
  win->vs.frustum[4][3] /= t;

  /* Extrait le plan PROCHE */
  win->vs.frustum[5][0] = clip[ 3] + clip[ 2];
  win->vs.frustum[5][1] = clip[ 7] + clip[ 6];
  win->vs.frustum[5][2] = clip[11] + clip[10];
  win->vs.frustum[5][3] = clip[15] + clip[14];

  /* Calcul des normales */
  t = sqrt( win->vs.frustum[5][0] * win->vs.frustum[5][0] +
            win->vs.frustum[5][1] * win->vs.frustum[5][1] +
            win->vs.frustum[5][2] * win->vs.frustum[5][2] );

  win->vs.frustum[5][0] /= t;
  win->vs.frustum[5][1] /= t;
  win->vs.frustum[5][2] /= t;
  win->vs.frustum[5][3] /= t;



}


//! @ingroup graphic
//! Does not seem to work anymore.
/*******************************************************/
/* Fonction calculant le maillage a appliquer pour     */
/* afficher une primitive                              */
/*                                                     */
/* IN => p : pointeur sur le polyhedre a dessiner      */
/*    => win: pointeur sur la structure fenetre        */
/*                                                     */
/* OUT => la resolution de l'objet a afficher          */
/*******************************************************/
int g3d_calcule_resolution(G3D_Window *win,p3d_poly *p) {
  float distance;
  int resolution;
  float vectx,vecty,vectz;


  if (p->entity_type == SPHERE_ENTITY ) {


    vectx = matrix_pos_absGL[0] * p->pos_rel_jnt[0][3] +
            matrix_pos_absGL[4] * p->pos_rel_jnt[1][3] +
            matrix_pos_absGL[8] * p->pos_rel_jnt[2][3] +
            matrix_pos_absGL[12];

    vecty = matrix_pos_absGL[1] * p->pos_rel_jnt[0][3] +
            matrix_pos_absGL[5] * p->pos_rel_jnt[1][3] +
            matrix_pos_absGL[9] * p->pos_rel_jnt[2][3] +
            matrix_pos_absGL[13];

    vectz = matrix_pos_absGL[2] * p->pos_rel_jnt[0][3] +
            matrix_pos_absGL[6] * p->pos_rel_jnt[1][3] +
            matrix_pos_absGL[10] * p->pos_rel_jnt[2][3] +
            matrix_pos_absGL[14];

    distance = win->vs.frustum[5][0] * vectx +
               win->vs.frustum[5][1] * vecty +
               win->vs.frustum[5][2] * vectz +
               win->vs.frustum[5][3];

    if ((distance - p->primitive_data->radius) /p->primitive_data->radius >= 155.0) {
      resolution = 1; /** pour appeler la liste opengl de resolution BASSE **/
    } else {
      if ((distance - p->primitive_data->radius)/p->primitive_data->radius >= 80.0) {
        resolution = 0; /** pour appeler la liste opengl de resolution MOYENNE **/
      } else {
        resolution = 2; /** pour appeler la liste opengl de resolution HAUTE **/
      }
    }
  } else {resolution = 0;}

  return(resolution);
}



//! @ingroup graphic
/** Fonction dessinant un polyhedre dont la forme est 
* consideree comme primitive                         
*                                                     
* IN => p : pointeur sur le polyhedre a dessiner      
*    => fill : type de dessin a appliquer             
*    => win : pointeur sur la structure fenetre       */
void g3d_draw_primitive(G3D_Window *win,p3d_poly *p, int fill) {
  GLfloat pos_rel_jnt_OGL[16];
  double x_box_length;
  double y_box_length;
  double z_box_length;
  int i_cpti;
  int i_cptj;
  int resolution; /** 0:MED  1:LO  2:HI **/

  //resolution = g3d_calcule_resolution(win,p); /*** calcul du detail de maillage a appliquer **/
  resolution= 0; // force to medium resolution as g3d_calcule_resolution() does not work


  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
  glPushMatrix();

  /* !!! OpenGL considere que une valeur en [i][j] de notre matrice c'est [j][i] !!!! !!!*/
  for(i_cpti = 0; i_cpti < 4; i_cpti++)
    for(i_cptj = 0; i_cptj < 4; i_cptj++)
      pos_rel_jnt_OGL[(4 * i_cptj) + i_cpti] = (float)p->pos_rel_jnt[i_cpti][i_cptj];

  glMultMatrixf(pos_rel_jnt_OGL);

  switch(p->entity_type) {

    case SPHERE_ENTITY: {
        glEnable(GL_NORMALIZE);
        glScalef(p->primitive_data->radius,p->primitive_data->radius,p->primitive_data->radius);
        break;
      }

    case CUBE_ENTITY: {
        x_box_length = p->primitive_data->x_length/2.;
        y_box_length = p->primitive_data->y_length/2.;
        z_box_length = p->primitive_data->z_length/2.;

    	// flat shading looks better than smooth one for cube primitive:
    	if(fill==2)
        {  fill=1;  }
        glShadeModel(GL_SMOOTH);
        glEnable(GL_NORMALIZE);
        glTranslatef(- x_box_length,- y_box_length, z_box_length);
        glScalef(p->primitive_data->x_length,p->primitive_data->y_length,p->primitive_data->z_length);
        break;
      }

    case BOX_ENTITY: {
        /*       x_box_length = p->primitive_data->x_length/2.; */
        /*       y_box_length = p->primitive_data->y_length/2.; */
        /*       z_box_length = p->primitive_data->z_length/2.; */
    	
    	// flat shading looks better than smooth one for box primitive:
    	if(fill==2)
        {  fill=1;  }
        glShadeModel(GL_SMOOTH);
        glEnable(GL_NORMALIZE);
        /*       glTranslatef(- x_box_length,- y_box_length, z_box_length); */
        glScalef(p->primitive_data->x_length,p->primitive_data->y_length,p->primitive_data->z_length);
        break;
      }

    case CYLINDER_ENTITY: {
        glEnable(GL_NORMALIZE);
        /*       glTranslatef(0.0,0.0,- (p->primitive_data->height/2.0)); */
        glScalef(p->primitive_data->radius, p->primitive_data->radius, p->primitive_data->height);
        break;
      }

    case CONE_ENTITY: {
        glEnable(GL_NORMALIZE);
        glTranslatef(0.0,0.0,- (p->primitive_data->height/2.0));
        glScalef(1.0,1.0,p->primitive_data->height);
        break;
      }

  } 

  switch (fill) {

    case 0: {
        glCallList(p->listfil + resolution);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        break;
      }

    case 1: {
        glCallList(p->list + resolution);
        break;
      }

    case 2: {
        if(p->listgour !=-1) {
          glCallList(p->listgour + resolution);
        }
        break;
      }
  }


  glPopMatrix();
  glPopAttrib();
}




//! @ingroup graphic
/** Fonction dessinant un polyhedre plein ou en filaire 
*                                                      
* IN => p : pointeur sur le polyhedre courant         
*    => win : pointeur sur la structure fenetre       
*    => coll :                                        
*    => fill : type de rendu a effectuer              */
void g3d_draw_poly(p3d_poly *p,G3D_Window *win, int coll,int fill) {
  GLdouble color_vect[4];
  int blend = 0;  /* pour activer ou non la transparence */

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);
  
  if(fill && !win->vs.allIsBlack) {
    switch(coll) {
      case 2:
        g3d_get_custom_color_vect(color_vect);
        break;
      case 1:
        g3d_get_color_vect(Red, color_vect);
      break;
      case 0:
        if(p->color!=Any) {
         g3d_get_color_vect(p->color, color_vect);
        }
        else {
          color_vect[0]= p->color_vect[0];
          color_vect[1]= p->color_vect[1];
          color_vect[2]= p->color_vect[2];
          color_vect[3]= p->color_vect[3];
        }
        blend = ( (color_vect[3]==1) ? 0 : 1);
        if(blend==0)
        {   color_vect[3]= 1.0;  }
        else
        {   color_vect[3]= p->color_vect[3];  }
      break;
    }
    glColor4dv(color_vect);
    glEnable(GL_DEPTH_TEST);

    if(win->vs.transparency_mode==G3D_NO_TRANSPARENCY) {
      blend= 0;
      color_vect[3]= 1.0;
    }

    /* Activation de la transparence */
    if(blend) {
      //glPushMatrix();
      glEnable(GL_BLEND);
      glDepthMask(GL_FALSE);
      if(p->color != Any ){
        glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
      }else{
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      }
    }
  }
  else {/*on va dessiner en filaire*/
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    /* modif de david Brunet */
    glColor4dv(Blackv);
    if(win->vs.FILAIRE) {
      switch(coll) {
        case 1:
          g3d_get_color_vect(Red, color_vect);
        break;
        case 0:
          if(p->color!=Any) {
           g3d_get_color_vect(p->color, color_vect);
          } 
          else {
            color_vect[0]= p->color_vect[0];
            color_vect[1]= p->color_vect[1];
            color_vect[2]= p->color_vect[2];
            color_vect[3]= p->color_vect[3];
          }
          break;
      }
    }
    glColor4dv(color_vect);
  }
 
  if(win->vs.allIsBlack && coll==0) {
    glColor4d(0.0, 0.0, 0.0, 1.0);
  }

  switch(p->display_mode) {
    case POLY_DEFAULT_DISPLAY: 
    break;
    case POLY_NO_DISPLAY: 
       glPopAttrib();
       return;
    break;
    case POLY_UNLIT_BLACK_DISPLAY: 
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(0.0, 0.0, 0.0);
    break;
    case POLY_UNLIT_RED_DISPLAY:  
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(1.0, 0.0, 0.0);
    break;
    case POLY_RED_DISPLAY:  
      glColor3f(1.0, 0.0, 0.0);
    break;
    case POLY_UNLIT_GREEN_DISPLAY: 
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(0.0, 1.0, 0.0);
    break;
    case POLY_GREEN_DISPLAY: 
      glColor3f(0.0, 1.0, 0.0);
    break;
    case POLY_UNLIT_BLUE_DISPLAY:
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(0.0, 0.0, 1.0);
    break;
    case POLY_BLUE_DISPLAY:
      glColor3f(0.0, 0.0, 1.0);
    break;
    case POLY_UNLIT_CUSTOM_COLOR_DISPLAY:
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor4dv(p->custom_color);
    break;
    case POLY_CUSTOM_COLOR_DISPLAY:
      glColor4dv(p->custom_color);
    break;
  }

  /******************************************************************/
  /*                                                                */
  /*    Cas d'une PRIMITIVE (voir si on ne peut pas generaliser)    */
  /******************************************************************/
  if ((p->entity_type == SPHERE_ENTITY) || (p->entity_type == CUBE_ENTITY) || (p->entity_type == BOX_ENTITY) || (p->entity_type == CYLINDER_ENTITY)/*
        ||(p->entity_type == CONE_ENTITY)*/) {

    g3d_draw_primitive(win,p,fill);

  }
  else {

    /******************************************************************/
    /*                                                                */
    /*    Cas d'un POLYHEDRE QUELCONQUE                               */
    /******************************************************************/
    glDisable(GL_NORMALIZE);
    if(fill == 1) {

      /* Cas d'un polyhedre quelconque */

      glCallList(p->list);

    } else if(fill == 2 && p->listgour != -1) {

      /* cas d'un polyhedre quelconque */

      glCallList(p->listgour);

    }
    else {
      glCallList(p->listfil);
      glEnable(GL_LIGHTING);
      glEnable(GL_LIGHT0);
    }
		
  }


  /* Desactivation du mode transparence */
  if(blend) {
    glDepthMask (GL_TRUE);
    glDisable(GL_BLEND);
  //  glPopMatrix();
  }

  glPopAttrib();
}

//! @ingroup graphic
/** Fonction dessinant un polyhedre plein ou en filaire avec une couleur 
*                                                     
* IN => p : pointeur sur le polyhedre courant         
*    => win : pointeur sur la structure fenetre       
*    => coll :                                        
*    => fill : type de rendu a effectuer              */
void g3d_draw_poly_with_color(p3d_poly *p,G3D_Window *win,int coll,int fill,double color) {
 
  GLdouble color_vect[4];
  int colorint;
  int blend = 0;  /* pour activer ou non la transparence */

  glPushAttrib(GL_LIGHTING | GL_ENABLE_BIT);

  if(fill && !win->vs.allIsBlack) {
    switch(coll) {
      case 3:
	  g3d_get_color_vect(Blue, color_vect);
      break;
      case 2:		
	  g3d_get_color_vect(Green, color_vect);		
      break;
      case 1:
          g3d_get_color_vect(Red, color_vect);
      break;
      case 0: 
	colorint = (int)color; 
        if(colorint!=Any) {
          g3d_get_color_vect(colorint, color_vect);
        }
        else {
            color_vect[0]= p->color_vect[0];
            color_vect[1]= p->color_vect[1];
            color_vect[2]= p->color_vect[2];
            color_vect[3]= p->color_vect[3];
        }
      break;
    }
    glColor4dv(color_vect);
    blend = color_vect[3] == 1 ? 0 : 1;

    if(win->vs.transparency_mode==G3D_NO_TRANSPARENCY) {
      blend= 0;
      color_vect[3]= 1.0;
    }

    glEnable(GL_DEPTH_TEST);

    /* Activation de la transparence */
    if(blend) {
      glPushMatrix();
      glEnable(GL_BLEND);
      glDepthMask(GL_FALSE);
      if(color != Any ){
        glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
      }else{
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      }
    }

  }

  else {/*on va dessiner en filaire*/
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    if(win->vs.FILAIRE) {
      switch(coll) {
        case 1:
          g3d_get_color_vect(Red, color_vect);
        break;
        case 0:
	  colorint = (int)color;
          if(colorint!=Any) {
            g3d_get_color_vect(colorint, color_vect);
          }
          else {
              color_vect[0]= p->color_vect[0];
              color_vect[1]= p->color_vect[1];
              color_vect[2]= p->color_vect[2];
              color_vect[3]= p->color_vect[3];
          }
        break;
      }
      glColor4dv(color_vect);
    }

  }


  switch(p->display_mode) {
    case POLY_DEFAULT_DISPLAY: 
    break;
    case POLY_NO_DISPLAY: 
       glPopAttrib();
       return;
    break;
    case POLY_UNLIT_BLACK_DISPLAY: 
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(0.0, 0.0, 0.0);
    break;
    case POLY_UNLIT_RED_DISPLAY: 
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(1.0, 0.0, 0.0);
    break;
    case POLY_RED_DISPLAY: 
      glColor3f(1.0, 0.0, 0.0);
    break;
    case POLY_UNLIT_GREEN_DISPLAY: 
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(0.0, 1.0, 0.0);
    break;
    case POLY_GREEN_DISPLAY: 
      glColor3f(0.0, 1.0, 0.0);
    break;
    case POLY_UNLIT_BLUE_DISPLAY: 
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor3f(0.0, 0.0, 1.0);
    break;
    case POLY_BLUE_DISPLAY: 
      glColor3f(0.0, 0.0, 1.0);
    break;
    case POLY_UNLIT_CUSTOM_COLOR_DISPLAY:
      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glColor4dv(p->custom_color);
    break;
    case POLY_CUSTOM_COLOR_DISPLAY:
      glColor4dv(p->custom_color);
    break;
  }

  if(win->vs.allIsBlack==TRUE && coll==0) {
    glColor4d(0.0, 0.0, 0.0, 1.0);
  }

  /******************************************************************/
  /*                                                                */
  /*    Cas d'une PRIMITIVE (voir si on ne peut pas generaliser)    */
  /******************************************************************/
  if ((p->entity_type == SPHERE_ENTITY) || (p->entity_type == CUBE_ENTITY) || (p->entity_type == BOX_ENTITY) || (p->entity_type == CYLINDER_ENTITY)/*
        ||(p->entity_type == CONE_ENTITY)*/) {

    g3d_draw_primitive(win,p,fill);

  }
  else {

    /******************************************************************/
    /*                                                                */
    /*    Cas d'un POLYHEDRE QUELCONQUE                               */
    /******************************************************************/

    if(fill == 1) {

      /* Cas d'un polyhedre quelconque */

      glCallList(p->list);

    } else if(fill == 2 && p->listgour != -1) {

      /* cas d'un polyhedre quelconque */

      glCallList(p->listgour);

    }
    else {
      glCallList(p->listfil);
      glEnable(GL_LIGHTING);
      glEnable(GL_LIGHT0);
    }

  }



  /* if(fill != 2 && p->listgour != -1){  WHY???????*/
  /*       glDeleteLists(p->listgour,1); */
  /*       p->listgour = -1; */
  /*     } */

  /* Desactivation du mode transparence */
  if(blend) {
    glDepthMask (GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();
  }

  glPopAttrib();
}

/* Debut Modification Thibaut */
/*************************************************************/
/* Fonction dessinant les contours d'un polyhedre en couleur */
/*************************************************************/
void g3d_draw_poly_special(p3d_poly *p,G3D_Window *win,int color) {
  int i,j,nvert,nface;
  double x,y,z;


  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);

  nface = p3d_get_nb_faces(p->poly);
  glLineWidth(2.0);
  g3d_set_color_vect(color,NULL);



  /******************************************************************/
  /*                                                                */
  /*    Cas d'une PRIMITIVE (voir si on ne peut pas generaliser)    */
  /******************************************************************/
  if ((p->entity_type == SPHERE_ENTITY) || (p->entity_type == CUBE_ENTITY) ||
      (p->entity_type == BOX_ENTITY) || (p->entity_type == CYLINDER_ENTITY)
      /*||(p->entity_type == CONE_ENTITY)*/) {

    g3d_draw_primitive(win,p,0);

  }
  else {
    /* Cas d'un polyhedre non primitive */

    for(i=1;i<=nface;i++) {
      nvert=p3d_get_nb_points_in_face(p->poly,i);
      glBegin(GL_LINE_LOOP);
      for(j=1;j<=nvert;j++) {
        p3d_get_point_in_pos_in_face(p->poly,i,j,&x,&y,&z);
        move_point(p->pos_rel_jnt,&x,&y,&z,1);
        glVertex3d(x,y,z);
      }
      glEnd();
    }
  }

  glLineWidth(1.0);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}
/* Fin Modification Thibaut */

/********************************************************************************/


/*******************************************************/
/* Fonction construisant les listes opengl de tous les */
/* polyhedres d'un environnement (pour accelerer les   */
/* affichages) en gouraud (fill = 2)                   */
/*******************************************************/
void g3d_init_all_poly_gouraud(void) {
  p3d_poly *p;


  int i_list_goursphere = -1;/* si != -1 alors une display list de sphere a deja ete creee*/

  int i_list_gourcylindre = -1;/* si != -1 alors une display list de cylindre a deja ete creee*/


  int i_list_gourbox = -1;/* si != -1 alors une display list de box ou cube a deja ete creee*/




  p=p3d_poly_get_first();

  /* si on n'a pas deja fait l'init gouraud d'aucun poly */
  if (p->listgour == -1) {

    while(p!=NULL) {

      switch (p->entity_type) {

        case SPHERE_ENTITY: {
            /* si on a pas deja initialise de sphere, on la cree */
            if (i_list_goursphere == -1) {
              g3d_init_poly(p,2);
              i_list_goursphere = p->listgour;
            }
            /* sinon on va se baser sur celle deja existante*/
            else {
              p->listgour = i_list_goursphere;
            }

            break;
          }

        case CUBE_ENTITY: {
            /* si on a pas deja initialise de box, on la cree */
            if (i_list_gourbox == -1) {
              g3d_init_poly(p,2);
              i_list_gourbox = p->listgour;
            }

            /* sinon on va se baser sur celle deja existante*/
            else {
              p->listgour = i_list_gourbox;
            }
            break;
          }

        case BOX_ENTITY: {
            /* si on a pas deja initialise de box, on la cree */
            if (i_list_gourbox == -1) {
              g3d_init_poly(p,2);
              i_list_gourbox = p->listgour;
            }

            /* sinon on va se baser sur celle deja existante*/
            else {
              p->listgour = i_list_gourbox;
            }
            break;
          }

        case CYLINDER_ENTITY: {
            /* si on a pas deja initialise de cylindre, on le cree */
            if (i_list_gourcylindre == -1) {
              g3d_init_poly(p,2);
              i_list_gourcylindre = p->listgour;
            }

            /* sinon on va se baser sur celui deja existant*/
            else {
              p->listgour = i_list_gourcylindre;
            }
            break;
          }



        default: {
            /***** CAS DU POLY QUELCONQUE ******/

            g3d_init_poly(p,2);

            break;
          }
      }

      p=p3d_poly_get_next();
    }
  }

}




/*******************************************************/
/* Fonction construisant les listes opengl de tous les */
/* polyhedres d'un environnement (pour accelerer les   */
/* affichages) en filaire et poly (fill = 0 et 1)      */
/*******************************************************/
void g3d_init_all_poly(void) {
  p3d_poly *p;

  int i_list_sphere = -1; /* si != -1 alors une display list de sphere a deja ete creee*/
  int i_list_filsphere = -1;

  int i_list_cylindre = -1;/* si != -1 alors une display list de cylindre a deja ete creee*/
  int i_list_filcylindre = -1;

  int i_list_box = -1;/* si != -1 alors une display list de box ou cube a deja ete creee*/
  int i_list_filbox = -1;



  p=p3d_poly_get_first();

  while(p!=NULL) {

    switch (p->entity_type) {

      case SPHERE_ENTITY: {
          /* si on a pas deja initialise de sphere, on la cree */
          if (i_list_sphere == -1) {
            g3d_init_poly(p,1);
            g3d_init_poly(p,0);
            i_list_sphere = p->list;
            i_list_filsphere = p->listfil;
          }

          /* sinon on va se baser sur celle deja existante*/
          else {
            p->list = i_list_sphere;
            p->listfil = i_list_filsphere;
          }
          break;
        }

      case CUBE_ENTITY: {
          /* si on a pas deja initialise de box, on la cree */
          if (i_list_box == -1) {
            g3d_init_poly(p,1);
            g3d_init_poly(p,0);
            i_list_box = p->list;
            i_list_filbox = p->listfil;
          }

          /* sinon on va se baser sur celle deja existante*/
          else {
            p->list = i_list_box;
            p->listfil = i_list_filbox;
          }
          break;
        }

      case BOX_ENTITY: {
          /* si on a pas deja initialise de box, on la cree */
          if (i_list_box == -1) {
            g3d_init_poly(p,1);
            g3d_init_poly(p,0);
            i_list_box = p->list;
            i_list_filbox = p->listfil;
          }

          /* sinon on va se baser sur celle deja existante*/
          else {
            p->list = i_list_box;
            p->listfil = i_list_filbox;
          }
          break;
        }

      case CYLINDER_ENTITY: {
          /* si on a pas deja initialise de cylindre, on le cree */
          if (i_list_cylindre == -1) {
            g3d_init_poly(p,1);
            g3d_init_poly(p,0);
            i_list_cylindre = p->list;
            i_list_filcylindre = p->listfil;
          }

          /* sinon on va se baser sur celui deja existant*/
          else {
            p->list = i_list_cylindre;
            p->listfil = i_list_filcylindre;
          }
          break;
        }


      default: {
          /***** CAS DU POLY QUELCONQUE ******/
          g3d_init_poly(p,1);
          g3d_init_poly(p,0);

          break;
        }
    }

    p=p3d_poly_get_next();
  }


}

/******************************************************
 Delete opengl list(s) for each polyedron in environment

  mode: - see g3d_delete_poly() -
*******************************************************/
void g3d_delete_all_poly(int mode) {
  p3d_poly *p;

  p=p3d_poly_get_first();

  while(p!=NULL) {
    g3d_delete_poly(p,mode);
    p=p3d_poly_get_next();
  }
}

/* fonction test (affichant tous les polyhedres) */
/* static */
/* void g3d_display_poly(void) */
/* { */
/*   p3d_poly *p; */

/*   p=p3d_poly_get_first(); */
/*   while(p!=NULL)  */
/*     {  */
/*       if(p->TYPE != P3D_GHOST){ */
/*  g3d_draw_poly(p,win,0,1);  */
/*  g3d_draw_poly(p,win,0,0);  */
/*       } */
/*       p=p3d_poly_get_next(); */
/*     } */

/* } */


/********************************************************/
/* Fonction de Callback pour gestion d'erreur de GLU    */
/*                                                      */
/*IN => errorCode : code d'erreur a traiter par GL      */
/********************************************************/
void GLUerrorCallback(GLenum errorCode) {
  const GLubyte *estring;

  estring = gluErrorString(errorCode);
  fprintf(stderr, "Quadric Error: %s\n", estring);
  exit(0);
}



void g3d_init_box2(p3d_poly *p, int fill) {

  double a2=1./2.,b2=1./2.,c2=1./2.;
  p3d_poly *poly ;
  int face[4];

  poly=p3d_poly_beg_poly(p->poly->name,p->TYPE);

  poly->entity_type = BOX_ENTITY;

  p3d_poly_add_vert(poly,-a2,-b2, c2);
  p3d_poly_add_vert(poly,-a2, b2, c2);
  p3d_poly_add_vert(poly, a2, b2, c2);
  p3d_poly_add_vert(poly, a2,-b2, c2);

  p3d_poly_add_vert(poly,-a2,-b2,-c2);
  p3d_poly_add_vert(poly,-a2, b2,-c2);
  p3d_poly_add_vert(poly, a2, b2,-c2);
  p3d_poly_add_vert(poly, a2,-b2,-c2);

  face[0]=1;
  face[1]=4;
  face[2]=3;
  face[3]=2;
  p3d_poly_add_face(poly, face, 4);

  face[0]=5;
  face[1]=6;
  face[2]=7;
  face[3]=8;
  p3d_poly_add_face(poly, face, 4);


  face[0]=1;
  face[1]=5;
  face[2]=8;
  face[3]=4;
  p3d_poly_add_face(poly, face, 4);

  face[0]=4;
  face[1]=8;
  face[2]=7;
  face[3]=3;
  p3d_poly_add_face(poly, face, 4);

  face[0]=2;
  face[1]=3;
  face[2]=7;
  face[3]=6;
  p3d_poly_add_face(poly, face, 4);

  face[0]=1;
  face[1]=2;
  face[2]=6;
  face[3]=5;
  p3d_poly_add_face(poly, face, 4);

  p3d_poly_end_poly(poly);

  g3d_init_polyquelconque(poly,fill);

  if (fill == 1) {
    p->list = poly->list;
  } else {
    if (fill == 2) {
      p->listgour = poly->listgour;
    } else {
      p->listfil = poly->listfil;
    }
  }
  p3d_poly_del_poly (poly);

}



/********************************************************/
/* Fonction qui fait l'initialisation d'un box de pour  */
/*  pour affichage par GL (PAR VERTEX ARRAY)            */
/*                                                      */
/* IN => p : pointeur sur le polyhedre a dessiner       */
/*    => fill : type de dessin a appliquer              */
/********************************************************/
void g3d_init_box(p3d_poly* p, int fill) {
  static double length = 1.7320508; /*norme du vecteur normal moyen a un point du cube  = racine de 3*/
  double inv_length = 1./length;


  static GLdouble vrtx_ptr[] = {0.,0.,0.,
                                1.,0.,0.,
                                1.,1.,0.,
                                0.,1.,0.,
                                1.,0.,-1.,
                                1.,1.,-1.,
                                0.,1.,-1.,
                                0.,0.,-1.};


  static GLubyte index_ptr[] = {0,
                                1,
                                2,
                                3,
                                4,
                                5,
                                6,
                                7};


  GLdouble nrml_ptr[] = {-inv_length, -inv_length, inv_length,
                         inv_length, -inv_length, inv_length,
                         inv_length, inv_length, inv_length,
                         -inv_length, inv_length, inv_length,
                         inv_length, -inv_length, -inv_length,
                         inv_length, inv_length, -inv_length,
                         -inv_length, inv_length, -inv_length,
                         -inv_length, -inv_length, -inv_length};

  static GLubyte index_lin[] = {0,1,
                                1,2,
                                2,3,
                                3,0,
                                1,4,
                                4,5,
                                5,2,
                                4,7,
                                7,6,
                                6,5,
                                7,0,
                                3,6};

  static GLubyte index_ord[] = {0,1,2,3,
                                1,4,5,2,
                                4,7,6,5,
                                7,0,3,6,
                                3,2,5,6,
                                0,7,4,1};


  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
  glEnableClientState(GL_INDEX_ARRAY);


  glVertexPointer(3,GL_DOUBLE,0,vrtx_ptr);
  glNormalPointer(GL_DOUBLE,0,nrml_ptr);
  glIndexPointer(GL_UNSIGNED_BYTE,0,index_ptr);

  if ((fill == 1) && (p->list == -1)) {
    p->list = glGenLists(1);


    glNewList(p->list, GL_COMPILE);
    glDrawElements(GL_QUADS,24,GL_UNSIGNED_BYTE,index_ord);
  } else {
    if ((fill == 2) && (p->listgour == -1)) {
      p->listgour = glGenLists(1);


      glNewList(p->listgour, GL_COMPILE);
      glDrawElements(GL_QUADS,24,GL_UNSIGNED_BYTE,index_ord);
    } else {
      if((!fill) && (p->listfil == -1)) {
        p->listfil = glGenLists(1);

        glNewList(p->listfil, GL_COMPILE);
        glDrawElements(GL_LINES,24,GL_UNSIGNED_BYTE,index_lin);
      } else {
        PrintInfo(("erreur : fill : %d list : %d listfil : %d listgour : %d\n",fill,p->list,p->listfil,p->listgour));
      }
    }
  }

  glEndList();
}




/********************************************************/
/* Fonction qui fait l'initialisation d'un quadric de   */
/* type cylindre unitaire pour affichage par GL         */
/* !!! ATTENTION !!! si la primitive est un cone, le    */
/* quadric n'est pas de taille unitaire                 */
/*                                                      */
/* IN => p : pointeur sur le polyhedre a dessiner       */
/*    => fill : type de dessin a appliquer              */
/*    => base_radius : rayon du bas du cylindre         */
/*    => top_radius : rayon du haut du cylindre         */
/*                                                      */
/********************************************************/
void g3d_init_cylindreGLU2(p3d_poly* p, int fill, float base_radius,float top_radius) {



  GLint slices = 15; /* Nombre de tranches "verticales" pour mailler le cylindre */
  double r = 1.0;
  double l = 1.0;
  double dt=2*M_PI/slices,l2=l/2.;
  p3d_poly *poly ;
  int i,face1[slices],face2[slices],face[4];

  poly=p3d_poly_beg_poly(p->poly->name,p->TYPE);
  for(i=0;i<=slices-1;i++) {
    p3d_poly_add_vert(poly,r*cos(i*dt),r*sin(i*dt), l2);
    p3d_poly_add_vert(poly,r*cos(i*dt),r*sin(i*dt),-l2);
  }
  /* On cree les faces bouchant le "tube" */
  for(i=0;i<=slices-1;i++) {
    face1[i]=2*i+1;
    face2[i]=2*(slices-i);
  }
  p3d_poly_add_face(poly, face1, slices);
  p3d_poly_add_face(poly, face2, slices);

  /* on cree les facettes du tubes sauf la derniere */
  /* (reliant les derniers points aux premiers) */
  for(i=0;i<=slices-2;i++) {
    face[0]=2*i+1;
    face[1]=2*i+2;
    face[2]=2*i+4;
    face[3]=2*i+3;
    p3d_poly_add_face(poly, face, 4);
  }

  /* on cree la derniere face */
  face[0]=2*slices-1;
  face[1]=2*slices;
  face[2]=2;
  face[3]=1;
  p3d_poly_add_face(poly, face, 4);

  p3d_poly_end_poly(poly);

  g3d_init_polyquelconque(poly,fill);

  if (fill == 1) {
    p->list = poly->list;
  } else {
    if (fill == 2) {
      p->listgour = poly->listgour;
    } else {
      p->listfil = poly->listfil;
    }
  }
  p3d_poly_del_poly(poly);
}












/********************************************************/
/* Fonction qui fait l'initialisation d'un quadric de   */
/* type cylindre unitaire pour affichage par GL         */
/* !!! ATTENTION !!! si la primitive est un cone, le    */
/* quadric n'est pas de taille unitaire                 */
/*                                                      */
/* IN => p : pointeur sur le polyhedre a dessiner       */
/*    => fill : type de dessin a appliquer              */
/*    => base_radius : rayon du bas du cylindre         */
/*    => top_radius : rayon du haut du cylindre         */
/*                                                      */
/********************************************************/
void g3d_init_cylindreGLU(p3d_poly* p, int fill, float base_radius,float top_radius) {
  GLUquadricObj *ps_gluqobj;
  GLUquadricObj *ps_gluqtop; /*** objet quadric pour disque du dessus ***/
  GLUquadricObj *ps_gluqbot; /*** objet quadric pour disque du dessous ***/


  GLint slices = 15; /* Nombre de tranches "verticales" pour mailler le cylindre */

  ps_gluqobj = gluNewQuadric();
  ps_gluqtop = gluNewQuadric();
  ps_gluqbot = gluNewQuadric();

  gluQuadricCallback(ps_gluqobj, GLU_ERROR, (void (*)()) GLUerrorCallback);
  gluQuadricCallback(ps_gluqtop, GLU_ERROR, (void (*)()) GLUerrorCallback);
  gluQuadricCallback(ps_gluqbot, GLU_ERROR, (void (*)()) GLUerrorCallback);

  gluQuadricOrientation(ps_gluqbot,GLU_INSIDE); /* les normales pointent vers z negatif */

  if ((fill == 1) && (p->list == -1)) {
    p->list = glGenLists(1);
    gluQuadricDrawStyle(ps_gluqobj, GLU_FILL);
    gluQuadricNormals(ps_gluqobj, GLU_FLAT);

    gluQuadricDrawStyle(ps_gluqtop, GLU_FILL);
    gluQuadricNormals(ps_gluqtop, GLU_FLAT);

    gluQuadricDrawStyle(ps_gluqbot, GLU_FILL);
    gluQuadricNormals(ps_gluqbot, GLU_FLAT);

    glNewList(p->list, GL_COMPILE);
  } else {
    if ((fill == 2) && (p->listgour == -1)) {
      p->listgour = glGenLists(1);
      gluQuadricDrawStyle(ps_gluqobj, GLU_FILL); /* smooth shaded */
      gluQuadricNormals(ps_gluqobj, GLU_SMOOTH);

      gluQuadricDrawStyle(ps_gluqtop, GLU_FILL); /* smooth shaded */
      gluQuadricNormals(ps_gluqtop, GLU_SMOOTH);

      gluQuadricDrawStyle(ps_gluqbot, GLU_FILL); /* smooth shaded */
      gluQuadricNormals(ps_gluqbot, GLU_SMOOTH);

      glNewList(p->listgour, GL_COMPILE);
    } else {
      if((!fill) && (p->listfil == -1)) {
        p->listfil = glGenLists(1);
        gluQuadricDrawStyle(ps_gluqobj, GLU_LINE); /* wireframe */
        gluQuadricNormals(ps_gluqobj, GLU_NONE);

        gluQuadricDrawStyle(ps_gluqtop, GLU_SILHOUETTE); /* wireframe */
        gluQuadricNormals(ps_gluqtop, GLU_NONE);

        gluQuadricDrawStyle(ps_gluqbot, GLU_SILHOUETTE); /* wireframe */
        gluQuadricNormals(ps_gluqbot, GLU_NONE);

        glNewList(p->listfil, GL_COMPILE);
      } else {
        PrintInfo(("erreur : fill : %d list : %d listfil : %d listgour : %d\n",fill,p->list,p->listfil,p->listgour));
      }
    }
  }

  /* creation d'un cylindre de taille et rayon "unite" */
  gluCylinder(ps_gluqobj, base_radius, top_radius, 1.0, slices, 1);

  gluDisk(ps_gluqbot,0.0,base_radius,slices,1);/* on place le disque du dessous */

  glPushMatrix();
  glTranslatef(0.0,0.0,1.0); /* on se positionne pour placer le disque du dessus */
  gluDisk(ps_gluqtop,0.0,top_radius,slices,1);
  glPopMatrix();
  glEndList();

}




/********************************************************/
/* Fonction qui fait l'initialisation d'un quadric de   */
/* type sphere pour affichage par GL                    */
/*                                                      */
/* IN => p : pointeur sur le polyhedre a dessiner       */
/*    => fill : type de dessin a appliquer              */
/*                                                      */
/********************************************************/
void g3d_init_sphereGLU(p3d_poly* p, int fill) {
  GLUquadricObj *ps_gluqobj;


  GLint slices_lo = 6; /* Nbre de tranches "verticales" pour mailler la sphere LOW RESOLUTION*/
  GLint stacks_lo = 3; /* Nbre de tranches "horizontales" pour mailler la sphere LOW RESOLUTION*/
  GLint slices = 20; /* Nbre de tranches "verticales" pour mailler la sphere MED RESOLUTION*/
  GLint stacks = 20; /* Nbre de tranches "horizontales" pour mailler la sphere MED RESOLUTION*/
  GLint slices_hi = 10; /* Nbre de tranches "verticales" pour mailler la sphere HI RESOLUTION*/
  GLint stacks_hi = 8; /* Nbre de tranches "horizontales" pour mailler la sphere HI RESOLUTION*/

  ps_gluqobj = gluNewQuadric();

  gluQuadricCallback(ps_gluqobj, GLU_ERROR, (void (*)()) GLUerrorCallback);

  if ((fill == 1) && (p->list == -1)) {
    p->list = glGenLists(3);
    gluQuadricDrawStyle(ps_gluqobj, GLU_FILL);
    gluQuadricNormals(ps_gluqobj, GLU_FLAT);

    glNewList(p->list, GL_COMPILE);
    gluSphere(ps_gluqobj, 1.0, slices, stacks); /** sphere at MED RESOLUTION **/
    glEndList();

    glNewList(p->list+1, GL_COMPILE);
    gluSphere(ps_gluqobj, 1.0, slices_lo, stacks_lo); /** sphere at LOW RESOLUTION **/
    glEndList();

    glNewList(p->list+2, GL_COMPILE);
    gluSphere(ps_gluqobj, 1.0, slices_hi, stacks_hi); /** sphere at HIGH RESOLUTION **/
    glEndList();

  } else {
    if ((fill == 2) && (p->listgour == -1)) {
      p->listgour = glGenLists(3);
      gluQuadricDrawStyle(ps_gluqobj, GLU_FILL); /* smooth shaded */
      gluQuadricNormals(ps_gluqobj, GLU_SMOOTH);

      glNewList(p->listgour, GL_COMPILE);
      gluSphere(ps_gluqobj, 1.0, slices, stacks); /** sphere at MED RESOLUTION **/
      glEndList();

      glNewList(p->listgour+1, GL_COMPILE);
      gluSphere(ps_gluqobj, 1.0, slices_lo, stacks_lo); /** sphere at LOW RESOLUTION **/
      glEndList();

      glNewList(p->listgour+2, GL_COMPILE);
      gluSphere(ps_gluqobj, 1.0, slices_hi, stacks_hi); /** sphere at HIGH RESOLUTION **/
      glEndList();
    } else {
      if((!fill) && (p->listfil == -1)) {
        p->listfil = glGenLists(3);
        gluQuadricDrawStyle(ps_gluqobj, GLU_LINE); /* wireframe */
        gluQuadricNormals(ps_gluqobj, GLU_NONE);

        glNewList(p->listfil, GL_COMPILE);
        gluSphere(ps_gluqobj, 1.0, slices, stacks);/** sphere at MED RESOLUTION **/
        glEndList();

        glNewList(p->listfil+1, GL_COMPILE);
        gluSphere(ps_gluqobj, 1.0, slices_lo, stacks_lo);/** sphere at LOW RESOLUTION **/
        glEndList();

        glNewList(p->listfil+2, GL_COMPILE);
        gluSphere(ps_gluqobj, 1.0, slices_hi, stacks_hi);/** sphere at HIGH RESOLUTION **/
        glEndList();

      } else {
        PrintInfo(("erreur : fill : %d list : %d listfil : %d listgour : %d\n",fill,p->list,p->listfil,p->listgour));
      }
    }
  }
  gluDeleteQuadric(ps_gluqobj);
}


/********************************************************/
/* Fonction qui fait l'initialisation d'un polyhedre    */
/* quelconque (non primitive) pour affichage par GL     */
/*                                                      */
/* IN => p : pointeur sur le polyhedre a dessiner       */
/*    => fill : type de dessin a appliquer              */
/********************************************************/
void g3d_init_polyquelconque(p3d_poly *p, int fill) {
  int i,j,nvert,nface;
  double x,y,z;
  int index;
  p3d_vector3 *norm_tab=NULL,norm;
  /* Debut Modification Thibaut */
  p3d_vector3 norm_tmp;
  /* Fin Modification Thibaut */

  nface = p3d_get_nb_faces(p->poly);

  /* Generation des tables de Display List OpenGL selon le type de shading */
  if((fill == 1) && (p->list == -1)) {
    p->list = glGenLists(1);
    glNewList(p->list,GL_COMPILE);
  }
  else {
    if((fill == 2) && (p->listgour == -1)) {
      p->listgour = glGenLists(1);
      glNewList(p->listgour,GL_COMPILE);

    } else {
      if((!fill) && (p->listfil == -1)) {
        p->listfil = glGenLists(1);
        glNewList(p->listfil,GL_COMPILE);

      } else {
        PrintInfo(("erreur : fill : %d list : %d listfil : %d listgour : %d\n",fill,p->list,p->listfil,p->listgour));
      }
    }
  }

  // for smooth shading, compute the vertex normals:
  p3d_build_planes(p->poly);
  if (fill == 2) {
    glShadeModel(GL_SMOOTH);
    p3d_compute_vertex_normals(p->poly);
    nvert = p3d_get_nb_points(p->poly);
    norm_tab = (p3d_vector3 *)malloc(sizeof(p3d_vector3)*nvert);
    for(i=0; i<nvert; i++) {
      p3d_xformVect(p->pos_rel_jnt, p->poly->vertex_normals[i], norm_tab[i]);
    }
  }



  /*************** DEUXIEME PARTIE *************************************/
  for(i = 1; i <= nface; i++) {

    if((fill == 1) && (p->color != Filaire)) {
      p3d_get_plane_normalv_in_world_pos(p,i,norm_tmp);
      p3d_vectNormalize(norm_tmp,norm);
    }

    nvert=p3d_get_nb_points_in_face(p->poly,i);
      
    if(fill && (p->color != Filaire)) {
      glBegin(GL_POLYGON);
    } else {    	
    	glBegin(GL_LINE_LOOP);
    }
    
    for(j=1;j<=nvert;j++) {

      if((fill == 1) && (p->color != Filaire)) {
        glNormal3dv(norm);
      } else if((fill == 2) && (p->color != Filaire)) {
        index = p3d_get_index_point_in_face(p->poly,i,j);
        p3d_vectNormalize(norm_tab[index-1],norm);
        /*  PrintInfo(("le sommet %d de la face %d est le numero %d\n",j,i,index)); */
        /*  PrintInfo(("sa normale : %f %f %f \n",norm[0],norm[1],norm[2])); */
        glNormal3dv(norm);
      }

      p3d_get_point_in_pos_in_face(p->poly,i,j,&x,&y,&z);
      
      move_point(p->pos_rel_jnt,&x,&y,&z,1);

      glVertex3d(x,y,z);
    }

    glEnd();
  }

  if(fill == 2) {
    /* pour gouraud on libere le tableau de normales */
    free(norm_tab);
  }
  
  glEndList();
  
  

}


/***************************************************************/
/* Fonction construisant la display list opengl d'un polyhedre */
/*                                                             */
/* IN => p : pointeur sur le polyhedre a dessiner              */
/*    => fill : type de rendu a appliquer                      */
/***************************************************************/
void g3d_init_poly(p3d_poly *p, int fill) {
  /*******************************************************/
  /*  Debut modif Jean-Gerard utilisation des primitives */
  /*  de la GLU                                          */
  /*                                                     */
  /*******************************************************/


  /*******************************************************/
  /*     Cas d'une Primitive                             */
  /*                                                     */
  /*******************************************************/

  /* Selon le type de primitive que l'on traite, on cree le quadric adequat */
  switch(p->entity_type) {

    case SPHERE_ENTITY: {
        g3d_init_sphereGLU(p,fill);
        break;
      }

    case CUBE_ENTITY: {
        g3d_init_box(p, fill);
        break;
      }

    case BOX_ENTITY: {
        g3d_init_box2(p, fill);
        break;
      }

    case CYLINDER_ENTITY: {
        g3d_init_cylindreGLU2(p,fill,1.0,1.0);
        break;
      }

      /* case CONE_ENTITY:{  */
      /*       g3d_init_cylindreGLU(p,fill,p->primitive_data->other_radius/2.,p->primitive_data->radius/2.); */

      /*       break; */
      /*     } */

    default: {
        /****** CAS D'UN POLYHEDRE NON PRIMITIVE ******/
        g3d_init_polyquelconque(p,fill);
      }

  }

}


/*******************************************************
 Delete polyhedron opengl list(s)

 mode:
  -1: all list
   0: listfil (filaire)
   1: list
   2: listgour (gouraud)

********************************************************/
void g3d_delete_poly(p3d_poly *p, int mode) {
  switch(mode) {
    case -1:

      if (p->listfil != -1)
        glDeleteLists(p->listfil,1);
      if (p->list != -1)
        glDeleteLists(p->list,1);
      if (p->listgour != -1)
        glDeleteLists(p->listgour,1);
      p->listfil = -1;
      p->list = -1;
      p->listgour = -1;
      break;
    case 0:
      if (p->listfil != -1)
        glDeleteLists(p->listfil,1);
      p->listfil = -1;
      break;
    case 1:
      if (p->list != -1)
        glDeleteLists(p->list,1);
      p->list = -1;
      break;
    case 2:
      if (p->listgour != -1)
        glDeleteLists(p->listgour,1);
      p->listgour = -1;
      break;
  }
}

/*
   Calculate the line segment PaPb that is the shortest route between
   two lines P1P2 and P3P4. Calculate also the values of mua and mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Return FALSE if no solution exists.
*/
int g3d_lineLineIntersect( p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3,
                       p3d_vector3 p4, p3d_vector3 *pa, p3d_vector3 *pb,
                       double *mua, double *mub){
   p3d_vector3 p13,p43,p21;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom;

   p13[0] = p1[0] - p3[0];
   p13[1] = p1[1] - p3[1];
   p13[2] = p1[2] - p3[2];
   p43[0] = p4[0] - p3[0];
   p43[1] = p4[1] - p3[1];
   p43[2] = p4[2] - p3[2];
   if (ABS(p43[0])  < EPS && ABS(p43[1])  < EPS && ABS(p43[2])  < EPS)
      return(FALSE);
   p21[0] = p2[0] - p1[0];
   p21[1] = p2[1] - p1[1];
   p21[2] = p2[2] - p1[2];
   if (ABS(p21[0])  < EPS && ABS(p21[1])  < EPS && ABS(p21[2])  < EPS)
      return(FALSE);

   d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
   d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
   d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
   d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
   d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

   denom = d2121 * d4343 - d4321 * d4321;
   if (ABS(denom) < EPS)
      return(FALSE);
   numer = d1343 * d4321 - d1321 * d4343;

   *mua = numer / denom;
   *mub = (d1343 + d4321 * (*mua)) / d4343;

   (*pa)[0] = p1[0] + *mua * p21[0];
   (*pa)[1] = p1[1] + *mua * p21[1];
   (*pa)[2] = p1[2] + *mua * p21[2];
   (*pb)[0] = p3[0] + *mub * p43[0];
   (*pb)[1] = p3[1] + *mub * p43[1];
   (*pb)[2] = p3[2] + *mub * p43[2];

   return(TRUE);
}

//! @ingroup graphic
//! Fonction d'affichage d'un repere (matrice 4x4).
//! Les axes sont dessines sur une longueur "length".
//! A utiliser dans une fonction d'affichage OpenGL.
void g3d_draw_frame(p3d_matrix4 frame, double length)
{
	p3d_vector3 origin, xAxis, yAxis, zAxis;

	origin[0]= frame[0][3];
	origin[1]= frame[1][3];
	origin[2]= frame[2][3];

	xAxis[0]=  origin[0] + length*frame[0][0];
	xAxis[1]=  origin[1] + length*frame[1][0];
	xAxis[2]=  origin[2] + length*frame[2][0];

	yAxis[0]=  origin[0] + length*frame[0][1];
	yAxis[1]=  origin[1] + length*frame[1][1];
	yAxis[2]=  origin[2] + length*frame[2][1];

	zAxis[0]=  origin[0] + length*frame[0][2];
	zAxis[1]=  origin[1] + length*frame[1][2];
	zAxis[2]=  origin[2] + length*frame[2][2];

	g3d_draw_arrow(origin, xAxis, 1.0, 0.0, 0.0);

	g3d_draw_arrow(origin, yAxis, 0.0, 1.0, 0.0);

	g3d_draw_arrow(origin, zAxis, 0.0, 0.0, 1.0);
}

//! @ingroup graphic
//! Draws a cylinder from its two face centers.
//! \param p1 center of the first face (disc) of the cylinder
//! \param p2 center of the second face (disc) of the cylinder
//! \param radius cylinder's radius
//! \param nbSegments number of segments of the cylinder section
//! \return 1 in case of success, 0 otherwise
int g3d_draw_cylinder(p3d_vector3 p1, p3d_vector3 p2, double radius, unsigned int nbSegments)
{
	unsigned int i, j;
	double alpha, dalpha, norm;
	p3d_vector3 d, u, v, c1, c2, c3, c4, normal;

	p3d_vectSub(p2, p1, d);

	norm= p3d_vectNorm(d);

	if(norm< 1e-6)
	{
		return 0;
	}
	d[0]/= norm;
	d[1]/= norm;
	d[2]/= norm;

 //find a vector orthogonal to d:
	if( fabs(d[2]) <= EPSILON )
	{
		u[0]= 0;
		u[1]= 0;
		u[2]= 1;
	}
	else
	{
		u[0]= 0;
		u[1]= 1;
		u[2]= -d[1]/d[2];
		p3d_vectNormalize(u, u);
	}

 // (u,v) is a basis for the plane orthogonal to the cylinder axis:
	p3d_vectXprod(d, u, v);

	dalpha= 2*M_PI/((float) nbSegments);

	alpha= 0;
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(d[0], d[1], d[2]);
	glVertex3d(p2[0], p2[1], p2[2]);
	for(i=0; i<nbSegments; i++)
	{
		for(j=0; j<3; j++)
		{
			c1[j]= p2[j] + radius*cos(alpha)*u[j] + radius*sin(alpha)*v[j];
			c2[j]= p2[j] + radius*cos(alpha+dalpha)*u[j] + radius*sin(alpha+dalpha)*v[j];
		}
		glVertex3f(c1[0], c1[1], c1[2]);
		glVertex3f(c2[0], c2[1], c2[2]);
		alpha+= dalpha;
	}
	glEnd();

	alpha= 0;
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(-d[0], -d[1], -d[2]);
	glVertex3f(p1[0], p1[1], p1[2]);
	for(i=0; i<nbSegments; i++)
	{
		for(j=0; j<3; j++)
		{
			c1[j]= p1[j] + radius*cos(alpha)*v[j] + radius*sin(alpha)*u[j];
			c2[j]= p1[j] + radius*cos(alpha+dalpha)*v[j] + radius*sin(alpha+dalpha)*u[j];
		}
		glVertex3f(c1[0], c1[1], c1[2]);
		glVertex3f(c2[0], c2[1], c2[2]);
		alpha+= dalpha;
	}
	glEnd();

	alpha= 0;

	glBegin(GL_QUADS);
	for(i=0; i<nbSegments; i++)
	{
		for(j=0; j<3; j++)
		{
			c1[j]= p1[j] + radius*cos(alpha)*v[j] + radius*sin(alpha)*u[j];
			c2[j]= p2[j] + radius*cos(alpha)*v[j] + radius*sin(alpha)*u[j];
			c3[j]= p2[j] + radius*cos(alpha+dalpha)*v[j] + radius*sin(alpha+dalpha)*u[j];
			c4[j]= p1[j] + radius*cos(alpha+dalpha)*v[j] + radius*sin(alpha+dalpha)*u[j];
			normal[j]= cos(alpha)*v[j] + sin(alpha)*u[j];
		}
		glNormal3f(normal[0], normal[1], normal[2]);
		glVertex3f(c1[0], c1[1], c1[2]);
		glVertex3f(c2[0], c2[1], c2[2]);
		glVertex3f(c3[0], c3[1], c3[2]);
		glVertex3f(c4[0], c4[1], c4[2]);
		alpha+= dalpha;
	}
	glEnd();

	return 1;
}

//! @ingroup graphic
//! Returns an RGB color (among a set of predefined colors) from an int.
//! e.g. it can be used to select different colors according to the value of a counter.
//! \param i an integer value
//! \param color an array that will be filled with the RGB values corresponding to the given hue. The fourth element is set to 1
void g3d_rgb_from_int(int i, double color[4])
{
  if(i<0)
  { i= 0; }
  else
  {
    i= i%G3D_COLOR_ARRAY_SIZE;
  }

  color[0]= G3D_COLOR_ARRAY[i][0];
  color[1]= G3D_COLOR_ARRAY[i][1];
  color[2]= G3D_COLOR_ARRAY[i][2];
  color[3]= 1;
}

//! @ingroup graphic
//! Computes an RGB color from a hue value.
//! If the hue parameter varies from 0 to 1, the color will vary from red -> green -> cyan -> blue -> magenta -> red
//! with all intermediate hues.
//! \param hue hue value (must be between 0 and 1)
//! \param color an array that will be filled with the RGB values corresponding to the given hue. The fourth element is set to 1
void g3d_rgb_from_hue(double hue, double color[4])
{
   double x, x1, x2, x3, x4, x5;

   x= hue;
   if(x < 0.0)
   { x= 0.0; }
   if(x > 1.0)
   { x= 1.0; }

   x1= 1.0/6.0;
   x2= 2.0/6.0;
   x3= 0.5;
   x4= 4.0/6.0;
   x5= 5.0/6.0;

   color[3]= 1.0;

   if(x < x1)
   {
     color[0]= 1.0;
     color[1]= x/x1;
     color[2]= 0.0;
   }
   else if(x < x2)
   {
     color[0]= (x2-x)/(x2-x1);
     color[1]= 1.0;
     color[2]= 0.0;
   }
   else if(x < x3)
   {
     color[0]= 0.0;
     color[1]= 1.0;
     color[2]= (x-x2)/(x3-x2);
   }
   else if(x < x4)
   {
     color[0]= 0.0;
     color[1]= (x4-x)/(x4-x3);
     color[2]= 1.0;
   }
   else if(x < x5)
   {
     color[0]= (x-x4)/(x5-x4);
     color[1]= 0.0;
     color[2]= 1.0;
   }
   else
   {
     color[0]= 1.0;
     color[1]= 0.0;
     color[2]= (1.0-x)/(1.0-x5);
   }
}

//! @ingroup graphic
//! Draws the joint frames of a robot.
//! \param robot pointer to the robot
//! \param size length of the frame arrows that will be drawn.
//! \return 1 in case of success, 0 otherwise
int g3d_draw_robot_joints(p3d_rob* robot, double size)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_draw_robot_joints(): input robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;
  for(i=0; i<=robot->njoints; i++)
  {
    g3d_draw_frame(robot->joints[i]->abs_pos, size);
  }

  return 1;
}

//! @ingroup graphic
//! Draws the kinematic chain of a robot (draw the links between successive joints).
//! \param robot pointer to the robot
//! \return 1 in case of success, 0 otherwise
int g3d_draw_robot_kinematic_chain(p3d_rob* robot)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_draw_robot_kinematic_chain(): input robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;
  double radius;
  p3d_vector3 p1, p2, d;

  for(i=0; i<=robot->njoints; i++)
  {
    if(robot->joints[i]->prev_jnt!=NULL && robot->joints[i]->prev_jnt!=robot->joints[0])
    {
      p3d_mat4ExtractTrans(robot->joints[i]->prev_jnt->abs_pos, p1);
      p3d_mat4ExtractTrans(robot->joints[i]->abs_pos, p2);
      p3d_vectSub(p2, p1, d);
      radius= p3d_vectNorm(d);
      g3d_draw_cylinder(p1, p2, radius/10.0, 15);
    }
  }

  return 1;
}

//! @ingroup graphic
//! Draws a p3d_polyhedre.
int g3d_draw_p3d_polyhedre(p3d_polyhedre *polyhedron)
{
  if(polyhedron==NULL)
  {
    printf("%s: %d: g3d_draw_p3d_polyhedre(): p3d_polyhedre is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

// double color_tab[16][3]= { {1,0,0}, {0,1,0}, {0,0,1}, {1,1,0}, {1,0,1}, {0,1,1} , {1,0.5,0.5}, {0.5,1,0.5}, {0.5,0.5,1}, {1,0.25,0.5}, {1,0.5,0.25}, {0.25,1.0,0.5}, {0.5,1,0.25}, {0.25,0.5,1}, {0.5,0.25,1}  };

  unsigned int i, j;
  double t;
//  double color_vect[4]= {0.0, 0.0, 0.0, 1.0};
  p3d_matrix4 pose;
  p3d_vector3 axis;
  p3d_vector3 *points=  polyhedron->the_points;
//   p3d_vector3 *normals=  polyhedron->vertex_normals;
  poly_edge *edges= polyhedron->the_edges;
  p3d_face *faces= polyhedron->the_faces;

  p3d_get_poly_pos( polyhedron, pose );
  p3d_mat4ExtractRot(pose, axis, &t);

  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT | GL_POINT_BIT);

  glPushMatrix();
  // glTranslatef(pose[0][3], pose[1][3], pose[2][3]);
  // glRotatef((180/M_PI)*t,axis[0], axis[1], axis[2]);

//    display vertices:
//    glPointSize(4);
//    glColor3f(1, 0, 0);
//    glDisable(GL_LIGHTING);
//    for(i=0; i<polyhedron->nb_points; i++)
//    {
//      glBegin(GL_POINTS);
//        glVertex3dv(points[i]);
//      glEnd();
//    }

      //display vertex normals:
//    if(normals!=NULL)
//    {
//      g3d_set_color(Red, NULL);
//      glBegin(GL_LINES);
//      for(i=0; i<polyhedron->nb_points; i++)
//      {
//        glVertex3dv( points[i] );
//        glVertex3d( points[i][0] + d*normals[i][0], points[i][1] + d*normals[i][1], points[i][2] + d*normals[i][2] );
//      }
//      glEnd();
//    }


   glEnable(GL_LIGHTING);
//    g3d_set_color(Green, NULL);
  // glDisable(GL_LIGHTING);
//    glShadeModel(GL_SMOOTH);
//   double c;
   for(i=0; i<polyhedron->nb_faces; i++)
   {
     glBegin(GL_POLYGON);
       for(j=0; j<faces[i].nb_points; j++)
       {
         if( faces[i].plane!=NULL )
         {   
            glNormal3dv(faces[i].plane->normale); 
         }
         //c= polyhedron->curvatures[faces[i].the_indexs_points[j]-1];
         //c= faces[i].curvature;

//          color_vect[0]= c;
//          color_vect[1]= c;
//          color_vect[2]= c;
//          g3d_set_color(Any, color_vect);
       //  g3d_rgb_from_hue(c, color_vect);
        // g3d_rgb_from_hue(((double) faces[i].part)/2.0, color_vect);

         //if( c<=0.0) glColor3f(0,0,0); else
       //  glColor3f(color_vect[0],color_vect[1],color_vect[2]);
         glVertex3dv(points[faces[i].the_indexs_points[j]-1]);
       }
     glEnd();

      // display face normal
//       if( faces[i].plane!=NULL )
//       {   
//         glDisable(GL_LIGHTING);
//         glColor3f(1.0, 0.0, 0.0);
//         center[0]= center[1]= center[2]= 0;
//         for(j=0; j<faces[i].nb_points; j++)
//         {
//           center[0]+= points[faces[i].the_indexs_points[j]-1][0];
//           center[1]+= points[faces[i].the_indexs_points[j]-1][1];
//           center[2]+= points[faces[i].the_indexs_points[j]-1][2];
//         }
//         center[0]/= faces[i].nb_points;
//         center[1]/= faces[i].nb_points;
//         center[2]/= faces[i].nb_points;
//          glBegin(GL_LINES);
//           glVertex3dv(center);
//           glVertex3d(center[0]+d*faces[i].plane->normale[0], center[1]+d*faces[i].plane->normale[1], center[2]+d*faces[i].plane->normale[2]);
//         glEnd();
//         glEnable(GL_LIGHTING);
//       }
   }
   glEnable(GL_LIGHTING);


//  glDisable(GL_LIGHTING);
  glLineWidth(1);
  if(edges!=NULL)
  {
    //display edges:
//     glBegin(GL_LINES);
//      for(i=0; i<polyhedron->nb_edges; i++)
//      {
//        glVertex3dv(points[edges[i].point1-1]);
//        glVertex3dv(points[edges[i].point2-1]);
//      }
//     glEnd();
    //display edge normals:
//     glBegin(GL_LINES);
//      for(i=0; i<polyhedron->nb_edges; i++)
//      {
//        glVertex3dv(edges[i].midpoint);
//        glVertex3d(edges[i].midpoint[0]+d*edges[i].normal[0],edges[i].midpoint[1]+d*edges[i].normal[1],edges[i].midpoint[2]+d*edges[i].normal[2] );
//      }
//     glEnd();
  }

   //glDisable(GL_LIGHTING);

  glPopMatrix();

  glPopAttrib();

  return 1;
}

//! @ingroup graphic
//! Computes the coordinates of the points of a unit radius circle discretized in n points.
//! sint et cost each have |n+1| elements.
//! Memory is allocated inside the function and must consequently be freed outside the function.
//! The signe of n defines in which direction the circle is travelled around.
int g3d_circle_table(double **sint, double **cost, const int n)
{
   if(sint==NULL || cost==NULL)
   {
     printf("%s: %d: g3d_circle_table(): NULL input(s) (%p %p).\n", __FILE__, __LINE__,sint,cost);
     return 0;
   }

    int i;
    /* Table size, the sign of n flips the circle direction */
    const int size = abs(n);
    /* Determine the angle between samples */
    const double angle = 2*M_PI/( (double) n);
    /* Allocate memory for n samples, plus duplicate of first entry at the end */
    *sint = (double *) malloc((size+1)*sizeof(double));
    *cost = (double *) malloc((size+1)*sizeof(double));
    /* Bail out if memory allocation fails*/
    if (!(*sint) || !(*cost))
    {
        free(*sint);
        free(*cost);
        printf("%s: %d: g3d_circle_table(): memory allocation error.\n",__FILE__,__LINE__);
    }
    /* Compute cos and sin around the circle */
    for (i=0; i<size; i++)
    {
        (*sint)[i] = sin(angle*i);
        (*cost)[i] = cos(angle*i);
    }
    /* Last sample is duplicate of the first */
    (*sint)[size] = (*sint)[0];
    (*cost)[size] = (*cost)[0];

    return 1;
}

//! @ingroup graphic
//! Draws a sphere with OpenGL functions. The sphere is centered on (0,0,0).
//! \param radius radius of the sphere
//! \param nbSegments number of segments of the discretization of the sphere silhouette
void g3d_draw_solid_sphere(double radius, int nbSegments)
{
    int i, j;
    double r, r0;
    double x, y, z, z0;
    double *sint1, *cost1;
    double *sint2, *cost2;
    int n;
    if(nbSegments%2==0)
    {   n= nbSegments;  }
    else
    {   n= nbSegments+1;  }

    g3d_circle_table(&sint1, &cost1, -n);
    g3d_circle_table(&sint2, &cost2, n);

    for (i=1; i<=n/2; i++)
    {
        z0= cost2[i-1];
        r0= sint2[i-1];
        z = cost2[i];
        r = sint2[i];
        glPushAttrib(GL_LIGHTING_BIT);
        glShadeModel(GL_SMOOTH);
        glBegin(GL_TRIANGLE_STRIP);
          for(j=0; j<=n; j++)
          {
            x= cost1[j];
            y= sint1[j];
            glNormal3d(x*r, y*r, z);
            glTexCoord2d( 1-j/((double) n), 2*i/((double) n) );
            glVertex3d(x*r*radius, y*r*radius, z*radius);
            glNormal3d(x*r0, y*r0, z0);
            glTexCoord2d( 1-j/((double) n), 2*(i-1)/((double) n) );
            glVertex3d(x*r0*radius, y*r0*radius, z0*radius);
          }
       glEnd();
       glPopAttrib();
    }
    free(cost1);
    free(sint1);
    free(cost2);
    free(sint2);
}

//! @ingroup graphic
//! Draws a sphere with OpenGL functions.
//! \param x x coordinate of the sphere center
//! \param y y coordinate of the sphere center
//! \param z z coordinate of the sphere center
//! \param radius radius of the sphere
//! \param nbSegments number of segments of the discretization of the sphere silhouette
void g3d_draw_solid_sphere(double x_, double y_, double z_, double radius, int nbSegments)
{
    int i, j;
    double r, r0;
    double x, y, z, z0;
    double *sint1, *cost1;
    double *sint2, *cost2;
    int n;
    if(nbSegments%2==0)
    {   n= nbSegments;  }
    else
    {   n= nbSegments+1;  }

    g3d_circle_table(&sint1, &cost1, -n);
    g3d_circle_table(&sint2, &cost2, n);

    for (i=1; i<=n/2; i++)
    {
        z0= cost2[i-1];
        r0= sint2[i-1];
        z = cost2[i];
        r = sint2[i];
        glPushAttrib(GL_LIGHTING_BIT);
        glShadeModel(GL_SMOOTH);
        glBegin(GL_TRIANGLE_STRIP);
          for(j=0; j<=n; j++)
          {
            x= cost1[j];
            y= sint1[j];
            glNormal3d(x*r, y*r, z);
            glTexCoord2d( 1-j/((double) n), 2*i/((double) n) );
            glVertex3d(x_ + x*r*radius, y_ + y*r*radius, z_ + z*radius);
            glNormal3d(x*r0, y*r0, z0);
            glTexCoord2d( 1-j/((double) n), 2*(i-1)/((double) n) );
            glVertex3d(x_ + x*r0*radius, y_ + y*r0*radius, z_ + z0*radius);
          }
       glEnd();
       glPopAttrib();
    }
    free(cost1);
    free(sint1);
    free(cost2);
    free(sint2);
}

//! @ingroup graphic
//! Draws a cylinder, centered on (0,0,0) and aligned along z-axis.
//! \param radius cylinder radius
//! \param length cylinder length
//! \param nbSegments number of segments of the discretization of the cylinder's section
void g3d_draw_solid_cylinder(double radius, double length, int nbSegments)
{
  int i;
  float alpha, dalpha, *cost, *sint;
  cost= sint= NULL;

  if(nbSegments < 3)
  { nbSegments= 3; }
  if(nbSegments > 100)
  { nbSegments= 100; }

  cost= new float[nbSegments+1];
  sint= new float[nbSegments+1];

  dalpha= 2*M_PI/((float) nbSegments);
  for(i=0; i<=nbSegments; i++)
  {
    alpha= i*dalpha;
    cost[i]= cos(alpha);
    sint[i]= sin(alpha);
  }

  glBegin(GL_TRIANGLE_FAN);
   glNormal3f(0.0, 0.0, 1.0);
   glVertex3f(0.0, 0.0, 0.5*length);
   for(i=0; i<=nbSegments; i++)
   {  glVertex3f(radius*cost[i], radius*sint[i], 0.5*length);   }
  glEnd();

  glBegin(GL_TRIANGLE_FAN);
   glNormal3f(0.0, 0.0, -1.0);
   glVertex3f(0.0, 0.0, 0.5*length);
   for(i=nbSegments; i>=0; i--)
   {  glVertex3f(radius*cost[i], radius*sint[i], -0.5*length);   }
  glEnd();


  glBegin(GL_TRIANGLE_STRIP);
   for(i=0; i<=nbSegments; i++)
   {
     glNormal3f(cost[i], sint[i], 0.0);
     glVertex3f(radius*cost[i], radius*sint[i],  0.5*length);
     glVertex3f(radius*cost[i], radius*sint[i], -0.5*length);
   }
  glEnd();


  delete [] cost;
  delete [] sint;
}

//! @ingroup graphic
//! Draws the face normals of a body. Each normal is drawn starting from the face center.
//! \param obj pointer to the body
//! \param length length of the displayed normals
//! \return 1 in case of success, 0 otherwise
int g3d_draw_body_normals(p3d_obj *obj, double length)
{
  if(obj==NULL)
  {
    printf("%s: %d: g3d_draw_body_normals(): input p3d_obj is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;
  unsigned int j;
  GLfloat matGL[16];
  p3d_matrix4 T;
  p3d_polyhedre *poly= NULL;
  p3d_vector3 *points=  NULL;
  p3d_face *faces= NULL;

  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT);

   glEnable(GL_LIGHTING);
   g3d_set_color(Green, NULL);

   for(i=0; i<obj->np; i++)
   {
     if(obj->is_used_in_device_flag)
     {    p3d_matMultXform(obj->jnt->abs_pos, obj->pol[i]->pos_rel_jnt, T);     }
     else
     {    p3d_mat4Copy(obj->pol[i]->pos0, T);    }

     p3d_to_gl_matrix(T, matGL);

     glPushMatrix();
     glMultMatrixf(matGL);

     poly= obj->pol[i]->poly;
     points=  poly->the_points;
     faces= poly->the_faces;

     p3d_compute_face_areas_and_centroids(poly);

     glBegin(GL_LINES);
     for(j=0; j<poly->nb_faces; j++)
     {
        if(faces[j].plane==NULL)
        {   p3d_build_plane_face(poly, j+1);  }

        if(faces[j].plane!=NULL)
        {
          glVertex3dv(faces[j].centroid);
          glVertex3d(faces[j].centroid[0]+length*faces[j].plane->normale[0], faces[j].centroid[1]+length*faces[j].plane->normale[1], faces[j].centroid[2]+length*faces[j].plane->normale[2]);
        }
     }
     glEnd();
     glPopMatrix();
   }

  glPopAttrib();

  return 1;
}

//! @ingroup graphic
//! Draws the normals of all the robot bodies.
int g3d_draw_robot_normals(p3d_rob *robot, double length)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_draw_robot_normals(): input p3d_rob is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;

  for(i=0; i<robot->no; i++)
  {
    g3d_draw_body_normals(robot->o[i], length);
  }

  return 1;
}

//! @ingroup graphic
//! Draws the vertex normals of a body.
//! \param obj pointer to the body
//! \param length length of the displayed normals
//! \return 1 in case of success, 0 otherwise
int g3d_draw_body_vertex_normals(p3d_obj *obj, double length)
{
  if(obj==NULL)
  {
    printf("%s: %d: g3d_draw_body_vertex_normals(): input p3d_obj is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;
  unsigned int j;
  GLfloat matGL[16];
  p3d_matrix4 T;
  p3d_polyhedre *poly= NULL;
  p3d_vector3 *points=  NULL;
  p3d_vector3 *vertex_normals=  NULL;

  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT);

   glEnable(GL_LIGHTING);
   g3d_set_color(Green, NULL);

   for(i=0; i<obj->np; i++)
   {
     if(obj->is_used_in_device_flag)
     {    p3d_matMultXform(obj->jnt->abs_pos, obj->pol[i]->pos_rel_jnt, T);     }
     else
     {    p3d_mat4Copy(obj->pol[i]->pos0, T);    }

     p3d_to_gl_matrix(T, matGL);

     glPushMatrix();
     glMultMatrixf(matGL);

     poly= obj->pol[i]->poly;
     points=  poly->the_points;
     vertex_normals=  poly->vertex_normals;

     if(vertex_normals==NULL)
     { 
       p3d_compute_vertex_normals(poly);
       vertex_normals=  poly->vertex_normals;
     }

     glBegin(GL_LINES);
     for(j=0; j<poly->nb_points; j++)
     {
        glVertex3dv(points[j]);
        glVertex3d(points[j][0]+length*vertex_normals[j][0],points[j][1]+length*vertex_normals[j][1],points[j][2]+length*vertex_normals[j][2]);
     }
     glEnd();
     glPopMatrix();
   }

  glPopAttrib();

  return 1;
}

//! @ingroup graphic
//! Draws the vertex normals of all the robot bodies.
int g3d_draw_robot_vertex_normals(p3d_rob *robot, double length)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_draw_robot_vertex_normals(): input p3d_rob is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;

  for(i=0; i<robot->no; i++)
  {
    g3d_draw_body_vertex_normals(robot->o[i], length);
  }

  return 1;
}

//! @ingroup graphic
//! Tests wether or not a p3d_poly is transparent.
//! \return 1 if it is transparent, 0 otherwise
int g3d_is_poly_transparent(p3d_poly *p)
{
  if(p==NULL)
  {
    printf("%s: %d: g3d_is_poly_transparent(): input p3d_poly is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

   GLdouble color_vect[4];

   if(p->color!=Any) {
     g3d_get_color_vect(p->color, color_vect);
   }
   else {
     color_vect[3]= p->color_vect[3];
   }

   if(color_vect[3]!=1.0) {
    return 1;
   }
   else {
    return 0;
  }
}

//! @ingroup graphic
//! Draws an ellipsoid.
//! \param a radius along X axis
//! \param b radius along Y axis
//! \param c radius along Z axis
void g3d_draw_ellipsoid(double a, double b, double c, int nbSegments)
{
  int i, j;
  double r, r0;
  double x, y, z, z0;
  double *sint1, *cost1;
  double *sint2, *cost2;
  double n[3], norm;

  a= fabs(a);
  b= fabs(b);
  c= fabs(c);

  g3d_circle_table(&sint1, &cost1, -nbSegments);
  g3d_circle_table(&sint2, &cost2, 2*nbSegments);

  for (i=1; i<=nbSegments; i++)
  {
      z0 = cost2[i-1];
      r0 = sint2[i-1];
      z = cost2[i];
      r = sint2[i];
      glBegin(GL_TRIANGLE_STRIP);
        for(j=0; j<=nbSegments; j++)
        {
          x = cost1[j];
          y = sint1[j];
          n[0]=x*r/a;
          n[1]=y*r/b;
          n[2]=z/c;
          norm=sqrt(pow(n[0],2)+pow(n[1],2)+pow(n[2],2));
          glNormal3f(n[0]/norm,n[1]/norm,n[2]/norm);
          glTexCoord2f(-j/(float)nbSegments,(i+1)/(float)nbSegments);
          glVertex3f(a*x*r,b*y*r,c*z);
          n[0]=x*r0/a;
          n[1]=y*r0/b;
          n[2]=z0/c;
          norm=sqrt(pow(n[0],2)+pow(n[1],2)+pow(n[2],2));
          glNormal3f(n[0]/norm,n[1]/norm,n[2]/norm);
          glTexCoord2f(-j/(float)nbSegments,(i+1)/(float)nbSegments);
          glVertex3f(a*x*r0,b*y*r0,c*z0);
        }
      glEnd();
  }


  delete [] cost1;
  delete [] sint1;
  delete [] cost2;
  delete [] sint2;
}


//! @ingroup graphic
//! Draws a wireframe ellipsoid.
//! \param a radius along X axis
//! \param b radius along Y axis
//! \param c radius along Z axis
void g3d_draw_wire_ellipsoid(double a, double b, double c)
{
   unsigned int i, j, nalpha;
   unsigned int nx, ny, nz;
   double dx, dy, dz;
   double x, y, z;
   double dalpha;
   double a0, b0, c0;
   a= fabs(a);
   b= fabs(b);
   c= fabs(c);

   dx= MIN(a, MIN(b,c))/10.0;
   dy= dz= dx;
   nx= (unsigned int) (2*a/dx);
   ny= (unsigned int) (2*b/dy);
   nz= (unsigned int) (2*c/dz);

   nalpha= 30;
   dalpha= 2*M_PI/((double) nalpha);

   for(i=0; i<nx; ++i)
   {
     x= -a + i*dx;
     b0= (b/a)*sqrt(a*a - x*x);
     c0= (c/a)*sqrt(a*a - x*x);
     glBegin(GL_LINE_LOOP);
     for(j=0; j<nalpha; ++j)
     {
       y= b0*cos(j*dalpha);
       z= c0*sin(j*dalpha);
       glVertex3d(x, y, z);
     }
     glEnd();
   }

   for(i=0; i<ny; ++i)
   {
     y= -b + i*dy;
     a0= (a/b)*sqrt(b*b - y*y);
     c0= (c/b)*sqrt(b*b - y*y);
     glBegin(GL_LINE_LOOP);
     for(j=0; j<nalpha; ++j)
     {
       x= a0*cos(j*dalpha);
       z= c0*sin(j*dalpha);
       glVertex3d(x, y, z);
     }
     glEnd();
   }

//    for(i=0; i<nz; ++i)
//    {
//      z= -c + i*dz;
//      a0= (a/c)*sqrt(c*c - z*z);
//      b0= (b/c)*sqrt(c*c - z*z);
//      glBegin(GL_LINE_LOOP);
//      for(j=0; j<nalpha; ++j)
//      {
//        x= a0*cos(j*dalpha);
//        y= b0*sin(j*dalpha);
//        glVertex3d(x, y, z);
//      }
//      glEnd();
//    }
   return;
}


//! Draws the collision cloud contained in XYZ_ENV.
//! \return 0 in case of success, 1 otherwise
int g3d_draw_collision_cloud()
{
  int i;

  glPushAttrib(GL_POINT_BIT | GL_LIGHTING_BIT);
  glPointSize(4);
  glDisable(GL_LIGHTING);
  glColor3f(1, 0, 0);
  glBegin(GL_POINTS); 
    for(i=0; i<XYZ_ENV->cloudSize; ++i)
    {
      glVertex3dv(XYZ_ENV->collisionCloud[i]);
    }
  glEnd();

  glPopAttrib();

  return 0;
}


//! Draws a p3d_polyhedre with colors corresponding to its local curvature that must have
//! been computed before by calling with p3d_compute_mean_curvature().
//! \return 0 in case of success, 1 otherwise
int g3d_draw_poly_curvature(p3d_polyhedre *poly)
{
  if(poly==NULL)
  {
    printf("%s: %d: g3d_draw_poly_curvature(): input p3d_polyhedre* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  unsigned int i, j, index;
  p3d_matrix4 pose;
  p3d_get_poly_pos(poly, pose );
  p3d_vector3 axis;
  double t;
  double color[4];

  p3d_mat4ExtractRot(pose, axis, &t);

  glPushAttrib(GL_ENABLE_BIT);

  glDisable(GL_LIGHTING);
  glShadeModel(GL_SMOOTH);


  #ifdef USE_SHADERS
   GLint program;
   glGetIntegerv(GL_CURRENT_PROGRAM, &program);
   if(program!=0)
   { glUseProgram(0); }
  #endif


  glPushMatrix();
//    glTranslatef(pose[0][3], pose[1][3], pose[2][3]);
  glRotatef((180/M_PI)*t,axis[0], axis[1], axis[2]);

   glBegin(GL_TRIANGLES);
   for(i= 0; i<poly->nb_faces; i++)
   {
     if(poly->the_faces[i].nb_points!=3)
     {  continue;  }

     for(j=0; j<poly->the_faces[i].nb_points; j++)
     {
       index= poly->the_faces[i].the_indexs_points[j]-1;
//        if( poly->the_faces[i].plane->normale!=NULL )
//        { glNormal3dv(poly->the_faces[i].plane->normale); }
       glNormal3dv(poly->vertex_normals[index]);
       g3d_rgb_from_hue(poly->curvatures[index], color);
       glColor3dv(color);
       glVertex3dv(poly->the_points[index]);
     }
   }
  glEnd();

  glPopMatrix();

  glPopAttrib();


  #ifdef USE_SHADERS
   if(program!=0)
   { glUseProgram(program); }
  #endif

  return 0;
}


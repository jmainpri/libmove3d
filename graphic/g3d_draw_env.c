#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
int HRI_DRAW_TRAJ;
#endif
#ifdef CXX_PLANNER
#include "../planner_cxx/HRICost/HriCost.hpp"
#endif

int G3D_DRAW_TRACE = FALSE;
int G3D_DRAW_OCUR_SPECIAL;
int G3D_SELECTED_JOINT = -999;
int G3D_SELECTED_ROBOT = -1;
int G3D_MOUSE_ROTATION = 0;
int boxlist;            // liste opengl pour la boite materialisant l'environnment
int p3d_numcoll; // Variables externes pour le CC

int NB_CASES = 10; //nombre de cases du damier

/* VARIABLES EXPORTEES DANS g3d_draw.c POUR TRAITEMENT GRAPHIQUE */
GLfloat matrix_pos_absGL[16]; /* tableau (matrice GL) contenant
la position du joint par rapport au repere global (cf. g3d_draw_object_moved)*/

static void g3d_draw_env(void);
static void g3d_draw_obstacle(G3D_Window *win);
static void g3d_draw_body(int coll, G3D_Window *win);
static void g3d_draw_obj_BB(p3d_obj *o);
static void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win);
static void g3d_draw_object(p3d_obj *o, int coll, G3D_Window *win);
#if 0
static void g3d_draw_obj_BB(p3d_obj *o);
#endif
/* Debut Modification Thibaut */
static void g3d_draw_ocur_special(G3D_Window *win);
/* Fin Modification Thibaut */
#if 0
static void g3d_draw_rob_BB(p3d_rob *r);
#endif
static void g3d_draw_robot_box(void);


/****************************************************************************************************/

void g3d_set_draw_coll(int n) {
  p3d_numcoll = n;
}

void g3d_reinit_graphics(void) {
  pp3d_env env;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


  g3d_delete_all_poly(-1);

  if (boxlist != -1) {
    glDeleteLists(boxlist, 1);
    boxlist = -1;
  }

  if (p3d_get_robotboxlist() != -1) {
    glDeleteLists(p3d_get_robotboxlist(), 1);
    p3d_reset_robotboxlist();
  }

  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  if (env != NULL) {
    env->INIT = 1;
  }
}


#ifdef PLANAR_SHADOWS
// Fonction construisant la matrice de projection (pour OpenGL) des ombres sur le plan
// d'équation fPlane (paramètres (a,b,c,d) tels que l'équation du plan soit a*x+b*y+c*z+d=0).
// Un point p est projeté sur le plan selon la direction fLightPos-p où fLightPos est
// la position de la source de lumiere (source ponctuelle).
// La matrice est recopiée dans le tableau fMatrix.
void buildShadowMatrix(float fMatrix[16], float fLightPos[4], float fPlane[4]) {
  float dotp;

  // Calculate the dot-product between the plane and the light's position
  dotp = fPlane[0] * fLightPos[0] +
         fPlane[1] * fLightPos[1] +
         fPlane[2] * fLightPos[2] +
         fPlane[3] * fLightPos[3];

  // First column
  fMatrix[0]  = dotp - fLightPos[0] * fPlane[0];
  fMatrix[4]  = 0.0f - fLightPos[0] * fPlane[1];
  fMatrix[8]  = 0.0f - fLightPos[0] * fPlane[2];
  fMatrix[12] = 0.0f - fLightPos[0] * fPlane[3];

  // Second column
  fMatrix[1]  = 0.0f - fLightPos[1] * fPlane[0];
  fMatrix[5]  = dotp - fLightPos[1] * fPlane[1];
  fMatrix[9]  = 0.0f - fLightPos[1] * fPlane[2];
  fMatrix[13] = 0.0f - fLightPos[1] * fPlane[3];

  // Third column
  fMatrix[2]  = 0.0f - fLightPos[2] * fPlane[0];
  fMatrix[6]  = 0.0f - fLightPos[2] * fPlane[1];
  fMatrix[10] = dotp - fLightPos[2] * fPlane[2];
  fMatrix[14] = 0.0f - fLightPos[2] * fPlane[3];

  // Fourth column
  fMatrix[3]  = 0.0f - fLightPos[3] * fPlane[0];
  fMatrix[7]  = 0.0f - fLightPos[3] * fPlane[1];
  fMatrix[11] = 0.0f - fLightPos[3] * fPlane[2];
  fMatrix[15] = dotp - fLightPos[3] * fPlane[3];
}




//! Draws a simple rectangle with its normal vector parallel to z axis.
//! \param bottomLeftCornerX x coordinate of the bottom left corner of the rectangle when seen from the top
//! \param bottomLeftCornerX y coordinate of the bottom left corner of the rectangle when seen from the top
//! \param z z coordinate of the rectangle
//! \param dimX side length of the rectangle along x axis
//! \param dimY side length of the rectangle along y axis
void g3d_draw_rectangle(float bottomLeftCornerX, float bottomLeftCornerY, float z, float dimX, float dimY)
{
  GLboolean cullface_enable;

  glGetBooleanv(GL_CULL_FACE, &cullface_enable);
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

  if(cullface_enable)
  {  glEnable(GL_CULL_FACE);  }
  else
  {  glDisable(GL_CULL_FACE);  }

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
  GLboolean cullface_enable;
  GLint smooth;
  unsigned int i, j, nx, ny;
  float x1, x2, y, xmax, ymax;

  xmax= bottomLeftCornerX + dimX;
  ymax= bottomLeftCornerY + dimY;

  nx= (unsigned int) ceil(dimX/delta);
  ny= (unsigned int) ceil(dimY/delta);

  glGetBooleanv(GL_CULL_FACE, &cullface_enable);
  glGetIntegerv(GL_SHADE_MODEL, &smooth);

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


  if(cullface_enable)
  {  glEnable(GL_CULL_FACE);  }
  else
  {  glDisable(GL_CULL_FACE);  }

  if(smooth)
  {  glShadeModel(GL_SMOOTH);  }
  else
  {  glShadeModel(GL_FLAT);  }

}


//! Displays an axis-aligned wire box.
//! \param xmin smallest coordinate of the box along X-axis
//! \param xmax biggest coordinate of the box along X-axis
//! \param ymin smallest coordinate of the box along Y-axis
//! \param ymax biggest coordinate of the box along Y-axis
//! \param zmin smallest coordinate of the box along Z-axis
//! \param zmax biggest coordinate of the box along Z-axis
void g3d_draw_AA_box(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) {
  g3d_set_color_mat(Black, NULL);
  glLineWidth(3);
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
}


//! Draws a floor tiled with rectangles and surrounded by a wire box.
//! \param dx length of a tile along x axis
//! \param dy length of a tile along y axis
//! \param xmin smallest coordinate of the floor along X-axis
//! \param xmax biggest coordinate of the floor along X-axis
//! \param ymin smallest coordinate of the floor along Y-axis
//! \param ymax biggest coordinate of the floor along Y-axis
//! \param zmin smallest coordinate of the box along Z-axis
//! \param zmax biggest coordinate of the box along Z-axis
//! \return 1 in case of success, 0 otherwise
int g3d_draw_floor_tiles(float dx, float dy, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, float shadowContrast)
{
  if( (xmin>=xmax) || (ymin>=ymax) || (zmin>=zmax) )
  {
    printf("%s: %d: g3d_draw_floor_tiles(): some of the box limit values are inconsistent.\n",__FILE__,__LINE__);
    return 0;
  }
  unsigned int i, j, k, nx, ny;
  float space, delta, shiftX, shiftY, dx2, dy2;
  float center[3], p1[4][3], p2[4][3];

  nx= (unsigned int) ceil( (xmax-xmin)/dx );
  ny= (unsigned int) ceil( (ymax-ymin)/dy );
  space= ((dx<dy ? dx : dy)/50.0); //width of the border between the tiles (half of the gap between two adjacent tiles)
   delta= ((dx<dy ? dx : dy)/20.0); //tesselation size



  //quit if there is too many tiles to display:
  if(nx>100 || ny>100)
  {
    printf("%s: %d: g3d_draw_floor_tiles(): too many tiles to display: change input values.\n",__FILE__,__LINE__);
    return 0;
  }

  glShadeModel(GL_SMOOTH);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  //draw the tiles:

  shiftX= ( (xmax-xmin)/dx - floor((xmax-xmin)/dx) )/2.0;
  shiftX= 0.5*dx - (dx-shiftX);

  shiftY= ( (ymax-ymin)/dy - floor((ymax-ymin)/dy) )/2.0;
  shiftY= 0.5*dx - (dy-shiftY);

  for(i=0; i<=nx; i++)
  {
    for(j=0; j<=ny; j++)
      {
        center[0]=  xmin + shiftX + i*dx;
        center[1]=  ymin - shiftY + j*dy;
        center[2]=  zmin;
  
        p1[0][0]= center[0] - 0.5*dx;
        p1[0][1]= center[1] - 0.5*dy; 
        p1[0][2]= center[2]; 
  
        p1[1][0]= center[0] + 0.5*dx;
        p1[1][1]= center[1] - 0.5*dy;
        p1[1][2]= center[2]; 
  
        p1[2][0]= center[0] + 0.5*dx;
        p1[2][1]= center[1] + 0.5*dy; 
        p1[2][2]= center[2]; 
  
        p1[3][0]= center[0] - 0.5*dx;
        p1[3][1]= center[1] + 0.5*dy; 
        p1[3][2]= center[2]; 

        p2[0][0]= p1[0][0] + space;
        p2[0][1]= p1[0][1] + space; 
        p2[0][2]= p1[0][2]; 
  
        p2[1][0]= p1[1][0] - space;
        p2[1][1]= p1[1][1] + space; 
        p2[1][2]= p1[1][2]; 
  
        p2[2][0]= p1[2][0] - space;
        p2[2][1]= p1[2][1] - space; 
        p2[2][2]= p1[2][2]; 
  
        p2[3][0]= p1[3][0] + space;
        p2[3][1]= p1[3][1] - space; 
        p2[3][2]= p1[3][2]; 

        for(k=0; k<4; k++)
        {
          if(p1[k][0]>xmax)   p1[k][0]= xmax;
          if(p1[k][0]<xmin)   p1[k][0]= xmin;
          if(p1[k][1]>ymax)   p1[k][1]= ymax;
          if(p1[k][1]<ymin)   p1[k][1]= ymin;

          if(p2[k][0]>xmax)   p2[k][0]= xmax;
          if(p2[k][0]<xmin)   p2[k][0]= xmin;
          if(p2[k][1]>ymax)   p2[k][1]= ymax;
          if(p2[k][1]<ymin)   p2[k][1]= ymin;
        } 
        
        //draw the inner rectangle of the tile:
        glColor3f(0, 0.8, 1);
        dx2= p2[1][0] - p2[0][0];
        dy2= p2[3][1] - p2[0][1];
        g3d_draw_tesselated_rectangle(p2[0][0], p2[0][1], p2[0][2], dx2, dy2, delta);


        //draw the tile borders:
        glDisable(GL_LIGHTING);
        glColor4f(0.0, 0.0, 0.0, 1.0);
        glBegin(GL_QUADS);
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3fv(p1[0]);
        glVertex3fv(p1[1]);
        glVertex3fv(p2[1]);
        glVertex3fv(p2[0]);

        glVertex3fv(p1[1]);
        glVertex3fv(p1[2]);
        glVertex3fv(p2[2]);
        glVertex3fv(p2[1]);

        glVertex3fv(p1[2]);
        glVertex3fv(p1[3]);
        glVertex3fv(p2[3]);
        glVertex3fv(p2[2]);

        glVertex3fv(p1[3]);
        glVertex3fv(p1[0]);
        glVertex3fv(p2[0]);
        glVertex3fv(p2[3]);
        glEnd();        
        glEnable(GL_LIGHTING);
    }
  }


  glDisable(GL_CULL_FACE);

  return 1;
}


// Dessine un sol composé de dimension length*width, composé d'hexagones de "rayon" r,
// le tout encadré par une boite de hauteur "height".
// Le paramètre shadowContrast sert à régler le contraste de luminosité entre les ombres projetées
// sur le plan du sol et les zones éclairées.
void g3d_draw_hexagonal_floor_tiles(double r, double length, double width, double height, GLfloat shadowContrast) {
  //pour éviter qu'il y ait trop d'hexagones à afficher:
  if (length / r > 30)
    r = length / 30.0;
  if (width / r > 30)
    r = width / 30.0;

  int i, j, k;
  double dx = sqrt(3.0) * r;
  double dy = 2.0 * r;
  int nx = (int)ceil(length / dx);
  int ny = (int)ceil(width / dy);


  double delta = ((dx < dy ? dx : dy) / 50.0);
  double sqrt3_2 = sqrt(3.0) / 2.0;

  double shift = r;
  double r2 = 1.1 * r;
  double p[7][3];

  GLfloat mat_ambient_diffuse[4] = { 0.8, 0.5, 0.1, shadowContrast };
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_ambient_diffuse);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  for (i = 0; i <= nx; i++) {
    if (shift == 0)
      shift = r;
    else
      shift = 0;

    for (j = 0; j <= ny; j++) {
      p[0][0] = -length / 2.0 + i * dx;
      p[0][1] =  -width / 2.0 + j * dy + shift;
      p[0][2] =  0.0;


      p[1][0] = -length / 2.0 + i * dx + r2;
      p[1][1] =  -width / 2.0 + j * dy + shift;
      p[1][2] =  0.0;

      p[2][0] = -length / 2.0 + i * dx + r2 * 0.5;
      p[2][1] =  -width / 2.0 + j * dy + r2 * sqrt3_2 + shift;
      p[2][2] =  0.0;

      p[3][0] = -length / 2.0 + i * dx - r2 * 0.5;
      p[3][1] =  -width / 2.0 + j * dy + r2 * sqrt3_2 + shift;
      p[3][2] =  0.0;

      p[4][0] = -length / 2.0 + i * dx - r2;
      p[4][1] =  -width / 2.0 + j * dy + shift;
      p[4][2] =  0.0;

      p[5][0] = -length / 2.0 + i * dx - r2 * 0.5;
      p[5][1] =  -width / 2.0 + j * dy - r2 * sqrt3_2 + shift;
      p[5][2] =  0.0;

      p[6][0] = -length / 2.0 + i * dx + r2 * 0.5;
      p[6][1] =  -width / 2.0 + j * dy - r2 * sqrt3_2 + shift;
      p[6][2] =  0.0;

      for (k = 0; k < 7; k++) {
        if (p[k][0] > length / 2.0)    p[k][0] = length / 2.0;
        if (p[k][0] < -length / 2.0)   p[k][0] = -length / 2.0;
        if (p[k][1] > width / 2.0)     p[k][1] = width / 2.0;
        if (p[k][1] < -width / 2.0)    p[k][1] = -width / 2.0;
      }

      glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0, 0.0, 1.0);
      glVertex3dv(p[0]);
      glVertex3dv(p[1]);
      glVertex3dv(p[2]);
      glVertex3dv(p[3]);
      glVertex3dv(p[4]);
      glVertex3dv(p[5]);
      glVertex3dv(p[6]);
      glVertex3dv(p[1]);
      glEnd();
    }
  }

  mat_ambient_diffuse[0] = 0.9;
  mat_ambient_diffuse[1] = 0.9;
  mat_ambient_diffuse[2] = 0.9;
  mat_ambient_diffuse[3] = shadowContrast;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_ambient_diffuse);
  glBegin(GL_QUADS);
  glNormal3f(0.0, 0.0, 1.0);
  glVertex3f(-length / 2.0, -width / 2.0, -0.002);
  glVertex3f(length / 2.0, -width / 2.0, -0.002);
  glVertex3f(length / 2.0,  width / 2.0, -0.002);
  glVertex3f(-length / 2.0,  width / 2.0, -0.002);
  glEnd();

  glDisable(GL_CULL_FACE);

  g3d_set_color_mat(Black, NULL);
  glLineWidth(2);

  glBegin(GL_LINES);
  glVertex3f(-length / 2.0 + delta, -width / 2.0 + delta, 0);
  glVertex3f(-length / 2.0 + delta, -width / 2.0 + delta, height);

  glVertex3f(length / 2.0 - delta, -width / 2.0 + delta, 0);
  glVertex3f(length / 2.0 - delta, -width / 2.0 + delta, height);

  glVertex3f(length / 2.0 - delta,  width / 2.0 - delta, 0);
  glVertex3f(length / 2.0 - delta,  width / 2.0 - delta, height);

  glVertex3f(-length / 2.0 + delta, width / 2.0 - delta, 0);
  glVertex3f(-length / 2.0 + delta, width / 2.0 - delta, height);
  glEnd();


  glBegin(GL_LINE_LOOP);
  glVertex3f(-length / 2.0, -width / 2.0, height);
  glVertex3f(length / 2.0, -width / 2.0, height);
  glVertex3f(length / 2.0,  width / 2.0, height);
  glVertex3f(-length / 2.0,  width / 2.0, height);
  glEnd();
}


// Dessine le sol de l'environnement. Utilisée dans les fonctions g3d_draw_planar_shadows()
// et g3d_draw_env().
// Le paramètre shadowContrast sert à régler la densité des ombres projetées sur le plan du mur
// (0 < shadowContrast < 1).
void g3d_draw_floor(GLfloat color[3], GLfloat shadowContrast, int tiles) {
  int nbDigit;
  double size, xmin, xmax, ymin, ymax, zmin, zmax;
  GLfloat mat_ambient_diffuse[4]= { color[0], color[1], color[2], shadowContrast};

  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE, mat_ambient_diffuse);
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  if(tiles==0)
  {
    g3d_draw_tesselated_rectangle(xmin, ymin, zmin, xmax-xmin, ymax-ymin,  (xmax-xmin)/80.0);
    return;
  }



  size = MAX(xmax - xmin, ymax - ymin);
  nbDigit = 0;

  for(;size >= 1; nbDigit++){
    size /= 10;
  }
  size *= 10;
  nbDigit--;
  size = floor(size);
  if(size < 2){
    nbDigit--;
  }
  size = pow(10,nbDigit);

  g3d_draw_floor_tiles(size, size, xmin, xmax, ymin, ymax, zmin, zmax, shadowContrast);
}

// Dessine un des  quatres murs au choix (wall=1,2,3 ou 4) de l'environment box.
// Le paramètre shadowContrast sert à régler la densité des ombres projetées sur le plan du mur
// (0 < shadowContrast < 1).
// quadsPerEdge est un parametre de discretisation des murs.
// Plus il est gran, plus le rendu est beau mais plus il sera lourd en calculs.
void g3d_draw_wall(int wall, GLfloat shadowContrast, int quadsPerEdge) {
  int i;
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);


  if (quadsPerEdge < 1 || quadsPerEdge > 30)
    quadsPerEdge = 16;

  GLfloat mat_ambient_diffuse[4] = { 0.65, 0.65, 0.7, shadowContrast };
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_ambient_diffuse);


  glShadeModel(GL_SMOOTH);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  switch (wall) {
    case 1:
      glNormal3f(0.0f, 1.0f, 0.0f);
      for (i = 0; i < quadsPerEdge; ++i) {
        glBegin(GL_TRIANGLE_STRIP);
        {
          for (int j = 0; j < quadsPerEdge + 1; ++j) {
            glVertex3f(xmin + (xmax - xmin)*(i + 1) / quadsPerEdge, ymin, zmin + (zmax - zmin)*j / quadsPerEdge);
            glVertex3f(xmin + (xmax - xmin)*i / quadsPerEdge, ymin, zmin + (zmax - zmin)*j / quadsPerEdge);
          }
        }
        glEnd();
      }
      break;
    case 2:
      glNormal3f(0.0f, -1.0f, 0.0f);
      for (i = 0; i < quadsPerEdge; ++i) {
        glBegin(GL_TRIANGLE_STRIP);
        {
          for (int j = 0; j < quadsPerEdge + 1; ++j) {
            glVertex3f(xmin + (xmax - xmin)*i / quadsPerEdge, ymax, zmin + (zmax - zmin)*j / quadsPerEdge);
            glVertex3f(xmin + (xmax - xmin)*(i + 1) / quadsPerEdge, ymax, zmin + (zmax - zmin)*j / quadsPerEdge);
          }
        }
        glEnd();
      }
      break;
    case 3:
      glNormal3f(1.0f, 0.0f, 0.0f);
      for (i = 0; i < quadsPerEdge; ++i) {
        glBegin(GL_TRIANGLE_STRIP);
        {
          for (int j = 0; j < quadsPerEdge + 1; ++j) {
            glVertex3f(xmin, ymin + (ymax - ymin)*i / quadsPerEdge, zmin + (zmax - zmin)*j / quadsPerEdge);
            glVertex3f(xmin, ymin + (ymax - ymin)*(i + 1) / quadsPerEdge, zmin + (zmax - zmin)*j / quadsPerEdge);
          }
        }
        glEnd();
      }
      break;
    case 4:
      glNormal3f(-1.0f, 0.0f, 0.0f);
      for (i = 0; i < quadsPerEdge; ++i) {
        glBegin(GL_TRIANGLE_STRIP);
        {
          for (int j = 0; j < quadsPerEdge + 1; ++j) {
            glVertex3f(xmax, ymin + (ymax - ymin)*(i + 1) / quadsPerEdge, zmin + (zmax - zmin)*j / quadsPerEdge);
            glVertex3f(xmax, ymin + (ymax - ymin)*i / quadsPerEdge, zmin + (zmax - zmin)*j / quadsPerEdge);
          }
        }
        glEnd();
      }
      break;
    default:
      printf("%s: %d: g3d_draw_wall(int wall): mauvaise entrée.\n\t", __FILE__, __LINE__);
      printf("L'entrée doit valoir 1,2,3 ou 4");
      break;
  }
  glDisable(GL_CULL_FACE);

}

// Dessine l'"arrière" d'un des quatres murs au choix (wall=1,2,3 ou 4) de l'environment box.
// NB: les normales sont inversées par rapport à celles de g3d_draw_walls().
void g3d_draw_backwall(int wall) {
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  GLfloat mat_ambient_diffuse[4] = { 0.65, 0.65, 0.7, 1 };
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_ambient_diffuse);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  switch (wall) {
    case 1:
      glNormal3f(0.0f, -1.0f, 0.0f);
      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(xmin, ymin, zmin);
      glVertex3f(xmax, ymin, zmin);
      glVertex3f(xmax, ymin, zmax);
      glVertex3f(xmin, ymin, zmax);
      glEnd();
      break;
    case 2:
      glNormal3f(0.0f, 1.0f, 0.0f);
      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(xmin, ymax, zmin);
      glVertex3f(xmin, ymax, zmax);
      glVertex3f(xmax, ymax, zmax);
      glVertex3f(xmax, ymax, zmin);
      glEnd();
      break;
    case 3:
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(xmin, ymin, zmin);
      glVertex3f(xmin, ymin, zmax);
      glVertex3f(xmin, ymax, zmax);
      glVertex3f(xmin, ymax, zmin);
      glEnd();
      break;
    case 4:
      glNormal3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_TRIANGLE_FAN);
      glVertex3f(xmax, ymin, zmin);
      glVertex3f(xmax, ymax, zmin);
      glVertex3f(xmax, ymax, zmax);
      glVertex3f(xmax, ymin, zmax);
      glEnd();
      break;
    default:
      printf("%s: %d: g3d_draw_backwall(): mauvaise entrée.\n\t", __FILE__, __LINE__);
      printf("L'entrée doit valoir 1,2,3 ou 4");
      break;
  }


  g3d_set_color_mat(Black, NULL);
  glLineWidth(3);
  glBegin(GL_LINES);
  glVertex3f(xmin, ymin, zmin);
  glVertex3f(xmin, ymin, zmax);
  glVertex3f(xmax, ymin, zmin);
  glVertex3f(xmax, ymin, zmax);

  glVertex3f(xmax, ymax, zmin);
  glVertex3f(xmax, ymax, zmax);

  glVertex3f(xmin, ymax, zmin);
  glVertex3f(xmin, ymax, zmax);
  glEnd();


  glBegin(GL_LINE_LOOP);
  glVertex3f(xmin, ymin, zmax);
  glVertex3f(xmax, ymin, zmax);
  glVertex3f(xmax, ymax, zmax);
  glVertex3f(xmin, ymax, zmax);
  glEnd();


  glDisable(GL_CULL_FACE);
}



// Dessine les ombres projetées par la lumière (de la structure g3d_window courante)
// sur le sol (plane=0), ou un des quatres murs (plane=1,2,3 ou 4).
void g3d_draw_planar_shadows(int plane, int tiles) {
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);
  G3D_Window *win = NULL;
  win = g3d_get_cur_win();

  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  GLfloat mat_specular[]   = { 0.9f, 0.9f, 0.9f, 1.0f };
  GLfloat mat_emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
  GLfloat high_shininess[] = { 100.0f };

  glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
  glMaterialfv(GL_FRONT, GL_EMISSION,  mat_emission);
  glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

  glClear(GL_STENCIL_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_ALWAYS, 1, 1);
  glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
  glColorMask(0, 0, 0, 0);

  switch (plane) {
    case 0:
      g3d_draw_rectangle(xmin, ymin, zmin, xmax-xmin, ymax-ymin);
      break;
    case 1:
    case 2:
    case 3:
    case 4:
      g3d_draw_wall(plane, win->shadowContrast, 16);
      break;
    default:
      printf("%s: %d: g3d_draw_planar_shadows(int plane): mauvaise entrée.\n\t", __FILE__, __LINE__);
      printf("L'entrée doit valoir 0,1,2,3 ou 4");
      break;
  }


  glClear(GL_DEPTH_BUFFER_BIT);

  glStencilFunc(GL_EQUAL, 1, 1);
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.2, 1.2);
  glColorMask(1, 1, 1, 1);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glColor3f(0.1f, 0.1f, 0.1f); // shadow's color

  glMatrixMode(GL_MODELVIEW);
  GLfloat *projection_matrix;

  switch (plane) {
    case 0:
      projection_matrix = win->floorShadowMatrix;
      break;
    case 1:
    case 2:
    case 3:
    case 4:
      projection_matrix = win->wallShadowMatrix[plane-1];
      break;
  }

  glPushMatrix();
  glMultMatrixf(projection_matrix);
  g3d_draw_robots(win);
  g3d_draw_obstacles(win);
  glPopMatrix();

  glPolygonOffset(0.8, 0.8);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glDisable(GL_STENCIL_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  switch (plane) {
    case 0:
      g3d_draw_floor(win->floorColor, win->shadowContrast, tiles);
    break;
    case 1:
    case 2:
    case 3:
    case 4:
      g3d_draw_wall(plane, win->shadowContrast, 16);
      break;
  }
  glDisable(GL_BLEND);
  glDisable(GL_POLYGON_OFFSET_FILL);

}

#endif

/**********************************************/
/* Fonction reglant les lumieres et affichant */
/* l'environnement                            */
/**********************************************/
void g3d_draw() {
  g3d_set_light();
  g3d_draw_env();

}

/*******************************************************/
/* Fonction initialisant graphiquement l'environnement */
/* si besoin est et l'affichant                        */
/*******************************************************/
extern int G3D_MODIF_VIEW;

static void g3d_draw_env(void) {
  pp3d_env e;
  pp3d_rob robotPt;
  G3D_Window *win;



  win = g3d_get_cur_win();
  e = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if (e->INIT) {
    ChronoOn();
    g3d_init_all_poly();
    boxlist = -1;
    p3d_reset_robotboxlist();
    e->INIT = 0;
    ChronoPrint("INIT TIME");
    ChronoOff();
  }

  if (win->GOURAUD) {
    g3d_init_all_poly_gouraud();
  }

  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);


  if (win->displayFloor) {
    if (win->displayShadows) {

      g3d_draw_planar_shadows(0, win->displayTiles);
      glColorMask(0, 0, 0, 0);
      g3d_draw_rectangle(xmin, ymin, zmin, xmax-xmin, ymax-ymin);

      glColorMask(1, 1, 1, 1);
    } else {
      g3d_draw_floor(win->floorColor, 1.0, win->displayTiles);
    }
  }
  if (win->displayWalls) {
    if (win->displayShadows) {
      g3d_draw_planar_shadows(1, 0);
      g3d_draw_planar_shadows(2, 0);
      g3d_draw_planar_shadows(3, 0);
      g3d_draw_planar_shadows(4, 0);
      glColorMask(0, 0, 0, 0);
      g3d_draw_wall(1, win->shadowContrast, 1);
      g3d_draw_wall(2, win->shadowContrast, 1);
      g3d_draw_wall(3, win->shadowContrast, 1);
      g3d_draw_wall(4, win->shadowContrast, 1);
      glColorMask(1, 1, 1, 1);
    } else {
      g3d_draw_wall(1, win->shadowContrast, 16);
      g3d_draw_wall(2, win->shadowContrast, 16);
      g3d_draw_wall(3, win->shadowContrast, 16);
      g3d_draw_wall(4, win->shadowContrast, 16);
    }

    g3d_draw_AA_box(xmin, xmax, ymin, ymax, zmin, zmax);
  }

  /*   printf("\n OpenGL Version %s \n",glGetString(GL_VERSION)); */

  /* glEnable(GL_CULL_FACE); */
  /*    glCullFace(GL_BACK); */
  /*    glFrontFace(GL_CCW); */

  /* g3d_draw_env_box(); NIC */

  g3d_draw_robot_box();

  g3d_extract_frustum(win);
  g3d_draw_robots(win);
  g3d_draw_obstacles(win);
#ifdef HRI_PLANNER
  gpsp_draw_robots_fov(win);
	psp_draw_elements(win);
#endif	
  g3d_kcd_draw_all_aabbs();     /* draw AABBs around static primitives */
  g3d_kcd_draw_aabb_hier();     /* draw AABB tree on static objects */
  g3d_kcd_draw_robot_obbs();    /* draw all obbs of current robot */
  g3d_kcd_draw_all_obbs();      /* draw all static obbs */

  g3d_kcd_draw_closest_points();
  /* Carl: just a test: KCD */
  /* kcd_init_movable_stuff(); */
  /* g3d_kcd_draw_nearest_bbs();   */ /* test nearest BB */
  /* Carl: end of test: KCD */

#ifdef CXX_PLANNER

  if(ENV.getBool(Env::isCostSpace))
  {
	  if(ENV.getBool(Env::enableHri))
	  {
			  std::vector<double> vect_jim;
			  //hri_zones.getHriDistCost(robotPt,FALSE);
			  vect_jim = hri_zones.getVectJim();

			  for(int i=0;i<vect_jim.size()/6;i++)
			  {
				  g3d_drawOneLine(
						vect_jim[0+6*i],vect_jim[1+6*i],vect_jim[2+6*i],
						vect_jim[3+6*i],vect_jim[4+6*i],vect_jim[5+6*i],
						Red,NULL);
			  }
		  }
	  else
	  {
		  for(int num=0;num<2;num++)
		  {
			  for(int it=0;it<3;it++)
			  {
				  if(vectMinDist[num][it]!=0)
				  {
					  g3d_drawOneLine(
							  vectMinDist[0][0],vectMinDist[0][1],vectMinDist[0][2],
							  vectMinDist[1][0],vectMinDist[1][1],vectMinDist[1][2],
									Red,NULL);
					  break;
				  }
			  }
		  }
	  }
  }
#endif

  /* Debut Modification Thibaut */
  if (G3D_DRAW_OCUR_SPECIAL) g3d_draw_ocur_special(win);
  /* Fin Modification Thibaut */

  /*   p3d_get_robot_pos(&x,&y,&z,&t); */
  /*   p3d_get_BB_rob(r,&x1,&x2,&y1,&y2,&z1,&z2); */
  /*   ampl = sqrt(SQR(x2-x1)+SQR(y2-y1)+SQR(z2-z1)); */

  /*   g3d_set_win_camera(win,x,y,z+0.5*ampl,ampl,180.0+t,20.0); */

  if(XYZ_GRAPH && ENV.getBool(Env::drawGraph)){
	  g3d_draw_graph();
  }

  if (ENV.getBool(Env::drawTraj)) {
    g3d_draw_all_tcur();
  }
  if (G3D_DRAW_TRACE) {
    g3d_draw_trace_all_tcur();
    p3d_set_and_update_robot_conf(robotPt->ROBOT_POS);
    /* collision checking */
    p3d_numcoll = p3d_col_test_all();
    g3d_draw_robot(robotPt->num, win);
    p3d_set_and_update_robot_conf(robotPt->ROBOT_GOTO);
    /* collision checking */
    p3d_numcoll = p3d_col_test_all();
    g3d_draw_robot(robotPt->num, win);
  }


// #ifdef PLANAR_SHADOWS
//     //On dessine la source de lumière sous la forme d'une sphère:
//   glDisable( GL_LIGHTING );
//   glColor3f(1.0, 1.0, 0.0);
//   glPushMatrix();
//   {
//    glLightfv( GL_LIGHT0, GL_POSITION, win->lightPosition );
//    glTranslatef( win->lightPosition[0], win->lightPosition[1], win->lightPosition[2] );
//    g3d_drawSphere(0, 0, 0, 50, Yellow, NULL);
//   }
//   glPopMatrix();
//   glEnable( GL_LIGHTING );
// #endif

  if (G3D_MODIF_VIEW) {
    glPushMatrix();
    glTranslatef(win->x, win->y, win->z);
    g3d_draw_frame();
    glPopMatrix();
  }
  p3d_drawRobotMoveMeshs();
#ifdef PLANAR_SHADOWS
#ifdef HRI_PLANNER
  if (!win->win_perspective) {
#endif
    if (win->fct_draw2 != NULL) win->fct_draw2();
    GLfloat light_ambient[] = { 1, 1, 3, 1.0 };
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLineWidth(1);

//     //Draw 0xyz frame:
//     g3d_set_color_mat(Red, NULL);
//     glBegin(GL_LINES);
//     glVertex3f(0, 0, 0);
//     glVertex3f(1, 0, 0);
//     glEnd();
//     g3d_set_color_mat(Green, NULL);
//     glBegin(GL_LINES);
//     glVertex3f(0, 0, 0);
//     glVertex3f(0, 1, 0);
//     glEnd();
//     g3d_set_color_mat(Blue, NULL);
//     glBegin(GL_LINES);
//     glVertex3f(0, 0, 0);
//     glVertex3f(0, 0, 1);
//     glEnd();
//     glLineWidth(1);

#else
 if (!win->win_perspective) {
#endif
#ifdef HRI_PLANNER
    //hri_hri_inter_point_test();
    g3d_hri_bt_draw_active_bitmaps(BTSET);
    g3d_hri_bt_draw_active_3dbitmaps(INTERPOINT);
    g3d_hri_bt_draw_active_3dbitmaps(OBJSET);
    g3d_hri_bt_draw_targets(BTSET);
    hri_exp_draw_ordered_points();
    if(HRI_DRAW_TRAJ){g3d_draw_all_tcur();}
  } else {
	if (win->draw_mode!=NORMAL)
		g3d_set_light_persp();
    psp_draw_in_perspwin();
  }
#endif

}


/**********************************************************/
/* Fonction tracant tous les obstacles d'un environnement */
/**********************************************************/
void g3d_draw_obstacles(G3D_Window* win) {
  int   no, o, i;

  /** Initialisation de la matrice OpenGL unite **/
  /** IMPORTANT pour le calcul de resolution de primitive **/
  for (i = 0 ; i < 16; i++) {
    matrix_pos_absGL[i] = 0.;
  }

  matrix_pos_absGL[0] = 1.;
  matrix_pos_absGL[5] = 1.;
  matrix_pos_absGL[10] = 1.;
  matrix_pos_absGL[15] = 1.;

  o = p3d_get_desc_curnum(P3D_OBSTACLE);
  no = p3d_get_desc_number(P3D_OBSTACLE);



  if (no) {
    for (i = 0;i < no;i++) {
      p3d_sel_desc_num(P3D_OBSTACLE, i);
      /*  ChronoOn(); */
      g3d_draw_obstacle(win);
      /*  printf("obstacle %d",i); */
      /*  ChronoPrint(""); */
      /*  ChronoOff(); */
    }
    p3d_sel_desc_num(P3D_OBSTACLE, o);
  }



}

/*******************************************************/
/* Fonction tracant tous les robots d'un environnement */
/*******************************************************/
void g3d_draw_robots(G3D_Window *win) {
  int   r, nr, ir;
  p3d_rob *rob;

  r = p3d_get_desc_curnum(P3D_ROBOT);
  nr = p3d_get_desc_number(P3D_ROBOT);

  if (nr) {
    for (ir = 0;ir < nr;ir++) {
      p3d_sel_desc_num(P3D_ROBOT, ir);
      rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
#ifdef HRI_PLANNER
	  if (win->win_perspective){
	    if (win->draw_mode==OBJECTIF){
	      if (rob->caption_selected)
		g3d_draw_robot(ir,win);
	    }
	    else{
	      g3d_draw_robot(ir,win);
	    }
	  }
	  else
#endif
	    /*g3d_draw_rob_BB(rob); */
	    g3d_draw_robot(ir, win);
    }
    p3d_sel_desc_num(P3D_ROBOT, r);

  }
}

/*******************************************/
/* Fonction tracant la boite materialisant */
/* les limites du robot                    */
/*******************************************/
static void g3d_draw_robot_box(void) {
  double x1, x2, y1, y2, z1, z2;
  /* double t1,t2,nampl,temp;*/
  /* int i,n=10;*/
  int robot_box_list;

  if (must_draw_robot_box()) {
    robot_box_list = p3d_get_robotboxlist();
    if (robot_box_list == -1) {

      robot_box_list = glGenLists(1);
      p3d_set_robotboxlist(robot_box_list);
      glNewList(robot_box_list, GL_COMPILE_AND_EXECUTE);

      glDisable(GL_LIGHTING);
      glDisable(GL_LIGHT0);

      p3d_get_filter_box(&x1, &x2, &y1, &y2, &z1, &z2);
      PrintInfo(("-> filter: %f,%f, %f,%f, %f,%f\n",
                 x1, x2, y1, y2, z1, z2));
      g3d_set_color_vect(Red, NULL);

      glBegin(GL_LINE_LOOP);
      {
        glVertex3d(x1, y1, z2);
        glVertex3d(x1, y2, z2);
        glVertex3d(x2, y2, z2);
        glVertex3d(x2, y1, z2);
      }
      glEnd();

      glBegin(GL_LINE_LOOP);
      {
        glVertex3d(x1, y1, z1);
        glVertex3d(x1, y2, z1);
        glVertex3d(x2, y2, z1);
        glVertex3d(x2, y1, z1);
      }
      glEnd();

      glBegin(GL_LINES);
      {
        glVertex3d(x1, y1, z1);
        glVertex3d(x1, y1, z2);

        glVertex3d(x2, y1, z1);
        glVertex3d(x2, y1, z2);

        glVertex3d(x2, y2, z1);
        glVertex3d(x2, y2, z2);

        glVertex3d(x1, y2, z1);
        glVertex3d(x1, y2, z2);
      }
      glEnd();

      glEnable(GL_LIGHTING);
      glEnable(GL_LIGHT0);

      glEndList();
    } else {
      glCallList(robot_box_list);
    }
  }
}

/*******************************************/
/* Fonction tracant la boite materialisant */
/* les limites de l'environnement          */
/*******************************************/
void g3d_draw_env_box(void) {
  double x1, x2, y1, y2, z1, z2, temp;
  int i, n = 10;

  glLineWidth(2);
  if (boxlist == -1) {

    boxlist = glGenLists(1);
    glNewList(boxlist, GL_COMPILE_AND_EXECUTE);

    p3d_get_env_box(&x1, &x2, &y1, &y2, &z1, &z2);

    z1 = z1 + 0.01;

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    g3d_set_color_vect(Black, NULL);

    glBegin(GL_LINE_LOOP);
    {
      glVertex3d(x1, y1, z2);
      glVertex3d(x1, y2, z2);
      glVertex3d(x2, y2, z2);
      glVertex3d(x2, y1, z2);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    {
      glVertex3d(x1, y1, z1);
      glVertex3d(x1, y2, z1);
      glVertex3d(x2, y2, z1);
      glVertex3d(x2, y1, z1);
    }
    glEnd();

    glBegin(GL_LINES);
    {
      glVertex3d(x1, y1, z1);
      glVertex3d(x1, y1, z2);

      glVertex3d(x2, y1, z1);
      glVertex3d(x2, y1, z2);

      glVertex3d(x2, y2, z1);
      glVertex3d(x2, y2, z2);

      glVertex3d(x1, y2, z1);
      glVertex3d(x1, y2, z2);
    }
    glEnd();

    glBegin(GL_LINES);
    {
      n = (int)(x2 - x1);
      for (i = 1;i <= n;i++) {
        temp = x1 + i;
        glVertex3d(temp, y1, z1);
        glVertex3d(temp, y2, z1);
      }
      n = (int)(y2 - y1);
      for (i = 1;i <= n;i++) {
        temp = y1 + i;
        glVertex3d(x1, temp, z1);
        glVertex3d(x2, temp, z1);
      }
    }
    glEnd();

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glEndList();
  } else {
    glCallList(boxlist);
  }
  glLineWidth(1);
}

/***************************************/
/* Fonction tracant l'obstacle courant */
/***************************************/
static
void g3d_draw_obstacle(G3D_Window *win) {
  pp3d_obj o;


  o = (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);


  /*  g3d_draw_obj_BB(o);  */


  /* on teste si l'obstacle est dans le frustum avant de le dessiner */
  if (BoxInFrustum_obj(o, win)) {
    g3d_draw_object(o, 0, win);
  }

}




/***********************************************/
/* Fonction tracant le robot courant en tenant */
/* compte de s'il a percute un obstacle ou non */
/***********************************************/
void g3d_draw_robot(int ir, G3D_Window* win) {
  int nb, b, ib, num;
  int coll = 0;
  /* B Kineo Carl 22.02.2002 */
  /* test */
  /*  p3d_poly *p1 = NULL; */
  /*  p3d_poly *p2 = NULL; */
  /*  p3d_obj *o1 = NULL; */
  /*  p3d_obj *o2 = NULL; */
  /* E Kineo Carl 22.02.2002 */

  b = p3d_get_desc_curnum(P3D_BODY);
  nb = p3d_get_desc_number(P3D_BODY);

  num = p3d_get_desc_curnum(P3D_ROBOT);

  if (p3d_numcoll) {
    coll = p3d_col_does_robot_collide(ir, p3d_numcoll);
  }

  /* B Kineo Carl 22.02.2002 */
  /* test */
  /*   if(coll) */
  /*     { */
  /*       p3d_col_get_report(0,&p1,&p2); */
  /*       printf("G3D: clash found %s, %s\n",p1->poly->name,p2->poly->name); */

  /*       p3d_col_get_report_obj(&o1,&o2); */
  /*       printf("G3D: clash found %s, %s\n",o1->name,o2->name); */
  /*     } */
  /* E Kineo Carl 22.02.2002 */

  for (ib = 0;ib < nb;ib++) {
    p3d_sel_desc_num(P3D_BODY, ib);
    g3d_draw_body(coll, win);
  }
  p3d_sel_desc_num(P3D_BODY,b);

#ifdef HRI_PLANNER
  p3d_rob *r;
  r=(p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if (r==PSP_ROBOT)
    if (win->win_perspective && PSP_DEACTIVATE_AUTOHIDE) // This characteristics are shown in a perspective window
      return;

  if (!win->win_perspective) // This characteristics are not shown in a perspective window
    {
      if (p3d_is_pos_area_showed(r))
	g3d_draw_rob_pos_area();
     // if (p3d_is_view_field_showed(r))
	//g3d_draw_rob_cone();
    }
#endif
}

void p3d_drawRobotMoveMeshs(void) {
  if (G3D_SELECTED_JOINT != -999) {
    p3d_rob* robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    if (robot->num != G3D_SELECTED_ROBOT) {
      return;
    }
    p3d_jnt* jnt = robot->joints[G3D_SELECTED_JOINT];
    p3d_vector3 axis;
    double t = 0, size = 0;
    size = MAX(jnt->o->BB0.xmax - jnt->o->BB0.xmin, jnt->o->BB0.ymax - jnt->o->BB0.ymin);
    size = MAX(size, jnt->o->BB0.zmax - jnt->o->BB0.zmin);
    size /= 2;

    switch (jnt->type) {
      case P3D_ROTATE: {
        glLoadName(-1);
        p3d_mat4ExtractRot(jnt->abs_pos, axis, &t);
        glPushMatrix();
        glTranslatef(jnt->abs_pos[0][3], jnt->abs_pos[1][3], jnt->abs_pos[2][3]);
        glRotatef((180 / M_PI)*t, axis[0], axis[1], axis[2]);
        g3d_drawCircle(0, 0, size + size*0.3, Green, NULL, 2);
        glPopMatrix();
        break;
      }
      case P3D_FREEFLYER: {
        p3d_matrix4 repPos;
        p3d_mat4Copy(jnt->pos0, repPos);
        repPos[0][3] = jnt->abs_pos[0][3];
        repPos[1][3] = jnt->abs_pos[1][3];
        repPos[2][3] = jnt->abs_pos[2][3];
        g3d_drawRepMoveObj(repPos, size + size*0.4, 7);
        g3d_drawSphMoveObj(repPos, size + size*0.1);
        break;
      }
      case P3D_KNEE: {
        p3d_matrix4 repPos;
        p3d_mat4Copy(jnt->pos0, repPos);
        repPos[0][3] = jnt->abs_pos[0][3];
        repPos[1][3] = jnt->abs_pos[1][3];
        repPos[2][3] = jnt->abs_pos[2][3];
        g3d_drawSphMoveObj(repPos, size + size*0.1);
        break;
      }
      case P3D_PLAN: {
        p3d_matrix4 repPos;
        p3d_mat4Copy(jnt->pos0, repPos);
        repPos[0][3] = jnt->abs_pos[0][3];
        repPos[1][3] = jnt->abs_pos[1][3];
        repPos[2][3] = jnt->abs_pos[2][3];
        g3d_drawRepMoveObj(repPos, size + size*0.4, 3);
        glLoadName(-3);
        p3d_mat4ExtractRot(repPos, axis, &t);
        glPushMatrix();
        glTranslatef(jnt->abs_pos[0][3], jnt->abs_pos[1][3], jnt->abs_pos[2][3]);
        glRotatef((180 / M_PI)*t, axis[0], axis[1], axis[2]);
        g3d_drawCircle(0, 0, size + size*0.1, Blue, NULL, 2);
        glPopMatrix();
        break;
      }
      case P3D_TRANSLATE:
        break;//Do nothing
      case P3D_FIXED:
        break;//Do nothing
      case P3D_BASE:
        break;//Do nothing
    }
  }
}
/******************************************************/
/* Fonction tracant le corps courant du robot courant */
/******************************************************/
static
void g3d_draw_body(int coll, G3D_Window* win) {
  pp3d_obj o;

  o = (p3d_obj *) p3d_get_desc_curid(P3D_BODY);
  /* g3d_draw_obj_BB(o); */ /* Carl: KCD test */

//modification JPSaut: Ce test est inutile (OpenGL tronque déjà les triangles situés en dehors du view frustum)
//et bloque parfois l'affichage de corps qui devraient être visibles:
//if (BoxInFrustum_obj(o,win))
  {
    g3d_draw_object_moved(o, coll, win);
  }

}

/*******************************************/
/* Fonction dessinant un objet en position */
/*******************************************/
static
void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win) {
  int i, j;

  /* On cree une matrice compatible avec opengl */
  /* on y met la position du jnt par rapport au repere global*/
  /* on s'en sert dans la fct 'g3d_calcule_resolution' (g3d_draw.c)*/
  /* pour calculer la position de l'objet par rapport au repere global*/
  /* de la scene */
  for (i = 0 ; i <= 3 ; i++) {
    for (j = 0 ; j <= 3 ; j++) {
      matrix_pos_absGL[4*j+i] = o->jnt->abs_pos[i][j];
    }
  }
  glPushMatrix();
  glMultMatrixf(matrix_pos_absGL);
  g3d_draw_object(o, coll, win);
  glPopMatrix();
  if (win->BB == TRUE) {
    g3d_draw_obj_BB(o);
  }
}


/*******************************/
/* Fonction dessinant un objet */
/*******************************/
static
void g3d_draw_object(p3d_obj *o, int coll, G3D_Window *win) {
  int i;

  glLoadName(o->o_id_in_env);

#ifdef HRI_PLANNER
  int colltemp, istrans;
  double colorindex;

  if (PSP_NUM_OBJECTS==0){
    colorindex = 1;
  }
  else{
    colorindex = (PSP_CURR_DRAW_OBJ+1)*(PSP_MAX_COLOR_IDX/(PSP_NUM_OBJECTS*1.0));
   // PSP_DRAW_OBJ_COL_INDEX[PSP_CURR_DRAW_OBJ] = colorindex;
  }


  if(win->draw_mode==OBJECTIF){ //If is indicated to draw only the objective
    if (o->caption_selected){ // if the object if marked as part of the objective
      colltemp = 2;
      for(i=0;i<o->np;i++){
	if (o->pol[i]->TYPE!=P3D_GHOST || win->GHOST == TRUE){
	  if((!win->FILAIRE)&&(!win->GOURAUD)){g3d_draw_poly_with_color(o->pol[i],win,colltemp,1,colorindex);}
	  if((!win->FILAIRE)&&(win->GOURAUD)){g3d_draw_poly_with_color(o->pol[i],win,colltemp,2,colorindex);}
	  if((win->FILAIRE || win->CONTOUR)){g3d_draw_poly_with_color(o->pol[i],win,colltemp,0,colorindex);}
	}
      }
      PSP_CURR_DRAW_OBJ++;
      if (PSP_CURR_DRAW_OBJ>=PSP_NUM_OBJECTS)
	PSP_CURR_DRAW_OBJ = 0;
    }
  }
  else{
    if (win->draw_mode==DIFFERENCE){
      if (o->caption_selected){ // if the object is marked as part of the objective
	colltemp = 2;
      }
      else{
	colltemp = 3;
      }
    }
    else
      colltemp = coll;

    istrans=0;
    if (win->win_perspective && o->trans){
      istrans=1;
    }
    if (!istrans){
      for(i=0;i<o->np;i++){
	if (o->pol[i]->TYPE!=P3D_GHOST || win->GHOST == TRUE){
	  if(colltemp !=2 || colltemp !=3) colorindex = o->pol[i]->color;
	  if((!win->FILAIRE)&&(!win->GOURAUD)){g3d_draw_poly_with_color(o->pol[i],win,colltemp,1,colorindex);}
	  if((!win->FILAIRE)&&(win->GOURAUD)){g3d_draw_poly_with_color(o->pol[i],win,colltemp,2,colorindex);}
	  if((win->FILAIRE || win->CONTOUR)){g3d_draw_poly_with_color(o->pol[i],win,colltemp,0,colorindex);}
	}
      }
      if (colltemp == 2)
	PSP_CURR_DRAW_OBJ++;
      if (PSP_CURR_DRAW_OBJ>=PSP_NUM_OBJECTS)
	PSP_CURR_DRAW_OBJ = 0;
    }
  }
  if (!win->win_perspective){ // This characteristics are not shown in a perspective window
    if (o->show_pos_area){
      g3d_draw_obj_pos_area(o);
      //printf("drawing\n");
    }
  }
#else
  for(i=0;i<o->np;i++){
    if (o->pol[i]->TYPE != P3D_GHOST || win->GHOST == TRUE){
      if((!win->FILAIRE)&&(!win->GOURAUD)){g3d_draw_poly(o->pol[i],win,coll,1);}
      if((!win->FILAIRE)&&(win->GOURAUD)){g3d_draw_poly(o->pol[i],win,coll,2);}
      if((win->FILAIRE || win->CONTOUR)){g3d_draw_poly(o->pol[i],win,coll,0);}
    }
  }
#endif

  /*  for(i=0;i<o->np;i++){ */
  /*    if (o->pol[i]->TYPE!=P3D_GHOST){ */
  /*      if((win->FILAIRE || win->CONTOUR)){ */
  /*        g3d_draw_poly(o->pol[i],win,coll,0); */
  /*      } */
  /*      if(!win->FILAIRE){ */
  /*        if(!win->GOURAUD){ */
  /*   g3d_draw_poly(o->pol[i],win,coll,1); */
  /*        } */
  /*        else{ */
  /*   g3d_draw_poly(o->pol[i],win,coll,2); */
  /*        } */
  /*      } */
  /*    } */
  /*  } */


}

/***************************************************/
/* Fonction tracant la boite englobante d'un objet */
/***************************************************/

static
void g3d_draw_obj_BB(p3d_obj *o) {
  double x1, x2, y1, y2, z1, z2;

  p3d_get_BB_obj(o, &x1, &x2, &y1, &y2, &z1, &z2); /* new Carl 23052001 */
  g3d_draw_a_box(x1, x2, y1, y2, z1, z2, Red, 1);
}

/* Debut Modification Thibaut */
/*************************************************************/
/* Fonction tracant la boite englobante de l'ostacle courant */
/*************************************************************/
static void g3d_draw_ocur_special(G3D_Window *win) {
  double x1, x2, y1, y2, z1, z2;
  int i;
  /*   G3D_Window *win; */

  pp3d_obj oc = (pp3d_obj)p3d_get_desc_curid(P3D_OBSTACLE);

  /*   win =  g3d_get_cur_win(); */

  if (G3D_DRAW_OCUR_SPECIAL == 1) {
    p3d_get_BB_obj(oc, &x1, &x2, &y1, &y2, &z1, &z2); /* new Carl 23052001 */
    /*     x1 = oc->BB.xmin; */
    /*     x2 = oc->BB.xmax; */
    /*     y1 = oc->BB.ymin; */
    /*     y2 = oc->BB.ymax; */
    /*     z1 = oc->BB.zmin; */
    /*     z2 = oc->BB.zmax; */
    g3d_draw_a_box(x1, x2, y1, y2, z1, z2, Red, 0);
  } else {
    for (i = 0;i < oc->np;i++)
      g3d_draw_poly_special(oc->pol[i], win, Red);
  }
}
/* Fin Modification Thibaut */

/***************************************************/
/* Fonction tracant la boite englobante d'un robot */
/***************************************************/
#if 0
static
void g3d_draw_rob_BB(p3d_rob *r) {
  double x1, x2, y1, y2, z1, z2;

  p3d_get_BB_rob(r, &x1, &x2, &y1, &y2, &z1, &z2); /* new Carl 23052001 */
  /*  x1 = r->BB.xmin; */
  /*  x2 = r->BB.xmax; */
  /*  y1 = r->BB.ymin; */
  /*  y2 = r->BB.ymax; */
  /*  z1 = r->BB.zmin; */
  /*  z2 = r->BB.zmax; */
  PrintInfo(("x1=%f,x2=%f,y1=%f,y2=%f,z1=%f,z2=%f\n", x1, x2, y1, y2, z1, z2);
            g3d_draw_a_box(x1, x2, y1, y2, z1, z2, Yellow, 0));
}
#endif

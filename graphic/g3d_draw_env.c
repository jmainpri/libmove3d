#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

#include "Graphic-pkg.h"

#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
int HRI_DRAW_TRAJ;
#endif

#ifdef DPG
#include "../planner/dpg/proto/DpgGrid.h"
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
GLdouble matrix_pos_absGL[16]; /* tableau (matrice GL) contenant
la position du joint par rapport au repere global (cf. g3d_"draw_object"_moved)*/

//static void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win);

#if 0
static void g3d_draw_obj_BB(p3d_obj *o);
#endif
/* Debut Modification Thibaut */
static void g3d_draw_ocur_special(G3D_Window *win);
/* Fin Modification Thibaut */

static void g3d_draw_robot_box(void);
// static void g3d_draw_rob_BB(p3d_rob *r);


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

#ifdef P3D_COLLISION_CHECKING
  if (p3d_get_robotboxlist() != -1) {
    glDeleteLists(p3d_get_robotboxlist(), 1);
    p3d_reset_robotboxlist();
  }
#endif
	
  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  if (env != NULL) {
    env->INIT = 1;
  }
}



// Fonction construisant la matrice de projection (pour OpenGL) des ombres sur le plan
// d'équation fPlane (paramètres (a,b,c,d) tels que l'équation du plan soit a*x+b*y+c*z+d=0).
// Un point p est projeté sur le plan selon la direction fLightPos-p où fLightPos est
// la position de la source de lumiere (source ponctuelle).
// La matrice est recopiée dans le tableau fMatrix.
void buildShadowMatrix(GLdouble fMatrix[16], GLfloat fLightPos[4], GLdouble fPlane[4]) {
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


//! Draws a floor tiled with rectangles and surrounded by a wire box.
//! \param color RGB color of the floor
//! \param dx length of a tile along x axis
//! \param dy length of a tile along y axis
//! \param xmin smallest coordinate of the floor along X-axis
//! \param xmax biggest coordinate of the floor along X-axis
//! \param ymin smallest coordinate of the floor along Y-axis
//! \param ymax biggest coordinate of the floor along Y-axis
//! \param zmin smallest coordinate of the box along Z-axis
//! \param zmax biggest coordinate of the box along Z-axis
//! \return 1 in case of success, 0 otherwise
int g3d_draw_tiled_floor(GLdouble color[3], float dx, float dy, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
  if( (xmin>=xmax) || (ymin>=ymax) || (zmin>=zmax) )
  {
    printf("%s: %d: g3d_draw_tiled_floor(): some of the box limit values are inconsistent.\n",__FILE__,__LINE__);
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
    printf("%s: %d: g3d_draw_tiled_floor(): too many tiles to display: change input values.\n",__FILE__,__LINE__);
    return 0;
  }

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_CULL_FACE);
//   glCullFace(GL_BACK);

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
        glColor4f(color[0], color[1], color[2], 1);
        dx2= p2[1][0] - p2[0][0];
        dy2= p2[3][1] - p2[0][1];
        g3d_draw_tesselated_rectangle(p2[0][0], p2[0][1], p2[0][2], dx2, dy2, delta);

        //draw the tile borders:
        glPushAttrib(GL_LIGHTING_BIT);
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
        glPopAttrib();
    }
  }

  glPopAttrib();

  return 1;
}


// Dessine un sol composé de dimension length*width, composé d'hexagones de "rayon" r,
// le tout encadré par une boite de hauteur "height".
// Le paramètre shadowContrast sert à régler le contraste de luminosité entre les ombres projetées
// sur le plan du sol et les zones éclairées.
void g3d_draw_hexagonal_floor_tiles(double r, double length, double width, double height) {
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

  glColor4f(0.8, 0.5, 0.1, 1.0);
  glPushAttrib(GL_LINE_BIT | GL_ENABLE_BIT);
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


  glColor4f(0.9, 0.9, 0.9, 1);

  glBegin(GL_QUADS);
  glNormal3f(0.0, 0.0, 1.0);
  glVertex3f(-length / 2.0, -width / 2.0, -0.002);
  glVertex3f(length / 2.0, -width / 2.0, -0.002);
  glVertex3f(length / 2.0,  width / 2.0, -0.002);
  glVertex3f(-length / 2.0,  width / 2.0, -0.002);
  glEnd();

  glDisable(GL_CULL_FACE);

  g3d_set_color(Black, NULL);
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

  glPopAttrib();
}

//! @ingroup graphic
//! Draws the environment floor.
//! \param color the floor color
//! \param tiles boolean to display floor tiles or not
void g3d_draw_floor(GLdouble color[3], int tiles) {
  int nbDigit;
  double size, xmin, xmax, ymin, ymax, zmin, zmax;

  glColor4f(color[0], color[1], color[2], 1);
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  if(tiles==0)
  {
    glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    g3d_draw_tesselated_rectangle(xmin, ymin, zmin, xmax-xmin, ymax-ymin,  (xmax-xmin)/80.0);
    glPopAttrib();
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

  g3d_draw_tiled_floor(color, size, size, xmin, xmax, ymin, ymax, zmin, zmax);
}

//! @ingroup graphic
//! Draws one of the environnment box walls.
//! \param wall choice of the wall to display (wall=1,2,3 ou 4).
//! \param color wall color
//! \param quadsPerEdge discretization step of the wall surface (The biggest it is, the nicest is the rendering but the more computationly expensive.)
void g3d_draw_wall(int wall, GLdouble color[3], int quadsPerEdge) {
  int i;
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  if (quadsPerEdge < 1 || quadsPerEdge > 30)
  {  quadsPerEdge = 16;  }

  glColor3dv(color);

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glShadeModel(GL_SMOOTH);

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
      printf("%s: %d: g3d_draw_wall(int wall): wrong input value.\n\t", __FILE__, __LINE__);
      printf("It must be 1,2,3 or 4");
    break;
  }
  
  glPopAttrib();
}

// Dessine l'"arrière" d'un des quatres murs au choix (wall=1,2,3 ou 4) de l'environment box.
// NB: les normales sont inversées par rapport à celles de g3d_draw_wall().
void g3d_draw_backwall(int wall) {
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glShadeModel(GL_SMOOTH);

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


  g3d_set_color(Black, NULL);
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

  glPopAttrib();
}


/**********************************************/
/* Fonction reglant les lumieres et affichant */
/* l'environnement                            */
/**********************************************/
void g3d_draw() {
  g3d_draw_env();
}

/*******************************************************/
/* Fonction initialisant graphiquement l'environnement */
/* si besoin est et l'affichant                        */
/*******************************************************/
extern int G3D_MODIF_VIEW;

void g3d_sky_box(double x, double y, double z)
{
  double d= 20, dz= 10;

  GLdouble colorBottom[3]=  {1.0, 1.0, 1.0};
  GLdouble colorTop[3]= {0.3, 0.4, 1.0};
  double w= 1.0;

  glPushAttrib(GL_LIGHTING_BIT); 

  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);

  glBegin(GL_QUADS);
   glNormal3f(-1, 0, 0);
   glColor3dv(colorBottom);
   glVertex4f(x+d, y+d, z-dz, w);
   glVertex4f(x+d, y-d, z-dz, w);
   glColor3dv(colorTop);
   glVertex4f(x+d, y-d, z+d, w);
   glVertex4f(x+d, y+d, z+d, w);
  glEnd();

  glBegin(GL_QUADS);
   glNormal3f(1, 0, 0);
   glColor3dv(colorBottom);
   glVertex4f(x-d, y-d, z-dz, w);
   glVertex4f(x-d, y+d, z-dz, w);
   glColor3dv(colorTop);
   glVertex4f(x-d, y+d, z+d, w);
   glVertex4f(x-d, y-d, z+d, w);
  glEnd();


  glBegin(GL_QUADS);
   glNormal3f(0, -1, 0);
   glColor3dv(colorBottom);
   glVertex4f(x-d, y+d, z-dz, w);
   glVertex4f(x+d, y+d, z-dz, w);
   glColor3dv(colorTop);
   glVertex4f(x+d, y+d, z+d, w);
   glVertex4f(x-d, y+d, z+d, w);
  glEnd();

  glBegin(GL_QUADS);
   glNormal3f(0, 1, 0);
   glColor3dv(colorBottom);
   glVertex4f(x+d, y-d, z-dz, w);
   glVertex4f(x-d, y-d, z-dz, w);
   glColor3dv(colorTop);
   glVertex4f(x-d, y-d, z+d, w);
   glVertex4f(x+d, y-d, z+d, w);
  glEnd();

  glColor3dv(colorBottom);
  glBegin(GL_QUADS);
   glNormal3f(0, 0, 1);
   glVertex4f(x-d, y-d, z-dz, w);
   glVertex4f(x+d, y-d, z-dz, w);
   glVertex4f(x+d, y+d, z-dz, w);
   glVertex4f(x-d, y+d, z-dz, w);
  glEnd();

  glColor3dv(colorTop);
  glBegin(GL_QUADS);
   glNormal3f(0, 0, -1);
   glVertex4f(x-d, y-d, z+d, w);
   glVertex4f(x-d, y+d, z+d, w);
   glVertex4f(x+d, y+d, z+d, w);
   glVertex4f(x+d, y-d, z+d, w);
  glEnd();

  glPopAttrib(); 
}

//! @ingroup graphic 
//! This function is the main display function called each time an OpenGL window is refreshed.
//! It is preferable to keep only what is really necessary inside it.
//! Define your own win->fct_draw2() and put your additional display inside.
void g3d_draw_env(void) {
  static int firstTime= TRUE;
  pp3d_env e;
  pp3d_rob robotPt;
  G3D_Window *win;
  GLdouble *projection_matrix;

  win = g3d_get_cur_win();
  e = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 
 g3d_extract_frustum(win);

  if (e->INIT) {
    ChronoOn();
    g3d_init_all_poly();
    boxlist = -1;
#ifdef P3D_COLLISION_CHECKING
    p3d_reset_robotboxlist();
#endif
    e->INIT = 0;
    ChronoPrint("INIT TIME");
    ChronoOff();
  }

  if (win->vs.GOURAUD) {
    glShadeModel(GL_SMOOTH);
    g3d_init_all_poly_gouraud();
  }
  else {
    glShadeModel(GL_FLAT);
  }

  // set the background color: 
  glClearColor(win->vs.bg[0], win->vs.bg[1], win->vs.bg[2], 0.0);

  
  // set the light source position to the camera position (a little bit higher though)
  // The render is not good if shadows are displayed, so do it only if shadows
  // are disabled:
  if(win->vs.cameraBoundedLight && !win->vs.displayShadows) {
    win->vs.lightPosition[0]= win->vs.cameraPosition[0];
    win->vs.lightPosition[1]= win->vs.cameraPosition[1];
    win->vs.lightPosition[2]= win->vs.cameraPosition[2];
  }
  
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  //////////////////////BEGINNING OF FUNCTION MAIN CORE///////////////////
  if(firstTime) 
  {
    g3d_init_OpenGL();
    firstTime= FALSE;
  } 
  #ifdef HRI_PLANNER
  glEnable(GL_COLOR_MATERIAL);
  #endif

  g3d_set_default_material();
  g3d_set_light(win->vs);

//deactivate picking until it works perfectly:
  G3D_SELECTED_JOINT= -999; 

  glPushAttrib(GL_ENABLE_BIT);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(win->vs.enableLight)
  {   
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
  }
  else
  {   glDisable(GL_LIGHTING);   }

  if(!win->vs.displayShadows)
  {
    //draw opaque objects first:
    win->vs.transparency_mode= G3D_OPAQUE;
    if(win->vs.cullingEnabled==1) {
      glEnable(GL_CULL_FACE);
    }
    else {
      glDisable(GL_CULL_FACE);
    }

    g3d_draw_robots(win);
    g3d_draw_obstacles(win);
	  

#ifdef HRI_COSTSPACE
    g3d_draw_costspace();
    g3d_draw_hrics();
#endif

   //g3d_sky_box(win->vs.x, win->vs.y, win->vs.z);

    if(win->vs.displayFloor)
    {  g3d_draw_floor(win->vs.floorColor, win->vs.displayTiles);   }

    if(win->vs.displayWalls)
    {
      g3d_draw_wall(1, win->vs.wallColor, 16);
      g3d_draw_wall(2, win->vs.wallColor, 16);
      g3d_draw_wall(3, win->vs.wallColor, 16);
      g3d_draw_wall(4, win->vs.wallColor, 16);
      glDisable(GL_LIGHTING);

      g3d_draw_AA_box(xmin, xmax, ymin, ymax, zmin, zmax);
      glEnable(GL_LIGHTING);
    }
    //draw transparent objects to finish:
    win->vs.transparency_mode= G3D_TRANSPARENT;
    glEnable(GL_CULL_FACE);
    g3d_draw_robots(win);
    g3d_draw_obstacles(win);
    glDisable(GL_CULL_FACE);
  }
  else
  {
    glDisable(GL_STENCIL_TEST);

    win->vs.transparency_mode= G3D_NO_TRANSPARENCY;

    g3d_draw_robots(win);
    g3d_draw_obstacles(win);

///////////////////////////////
// The following commented lines are to be used instead of the three previous ones
// to have shadows plus transparency.
// There is still a little problem: shadows can not be seen through transparent obstacles.
//     glDisable(GL_DEPTH_TEST);
//     g3d_draw_floor(win->vs.floorColor, win->vs.displayTiles);
//     if(win->vs.displayWalls)
//     {  
//      for(int i=1; i<=4; ++i)
//      {   g3d_draw_wall(i, win->vs.wallColor, 16);  }
//     }
//     glEnable(GL_DEPTH_TEST);
// 
//     win->vs.transparency_mode= G3D_OPAQUE;
//     g3d_draw_robots(win);
//     g3d_draw_obstacles(win);
//     glEnable(GL_CULL_FACE);
//     win->vs.transparency_mode= G3D_TRANSPARENT;
//     g3d_draw_robots(win);
//     g3d_draw_obstacles(win);
//     glEnable(GL_CULL_FACE);
//     glColorMask(0,0,0,0);
//     win->vs.transparency_mode= G3D_NO_TRANSPARENCY;
//     g3d_draw_robots(win);
//     g3d_draw_obstacles(win);
//     glColorMask(1,1,1,1);
///////////////////////////////

    glClear(GL_STENCIL_BUFFER_BIT);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 0x2, 0x0);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    g3d_draw_floor(win->vs.floorColor, win->vs.displayTiles);

    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    projection_matrix = win->vs.floorShadowMatrix;
    glStencilFunc(GL_EQUAL, 0x3, 0x2);

    glColorMask(0,0,0,0);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glDisable(GL_LIGHTING);

    glPushMatrix();
      glMultMatrixd(projection_matrix);
      win->vs.allIsBlack= TRUE;
      g3d_draw_robots(win);
      g3d_draw_obstacles(win);
    glPopMatrix();
    glColorMask(1,1,1,1);

    win->vs.allIsBlack= FALSE;

    glStencilFunc(GL_EQUAL, 0x1, 0x1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    g3d_set_dim_light();
    g3d_set_shade_material();
    glEnable(GL_LIGHTING);
    g3d_draw_floor(win->vs.floorColor, win->vs.displayTiles);

    if(win->vs.displayWalls)
    {
      for(int i=1; i<=4; ++i)
      {
        glClear(GL_STENCIL_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 0x2, 0x0);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  
        win->vs.allIsBlack= FALSE;
        g3d_set_light(win->vs);
        g3d_set_default_material();

        if(win->vs.enableLight)
        {   
          glEnable(GL_LIGHT0);
          glEnable(GL_LIGHTING);
        }
        else
        {   glDisable(GL_LIGHTING);   }

        g3d_set_light(win->vs);
        g3d_set_default_material();
        g3d_draw_wall(i, win->vs.wallColor, 16);

        glColorMask(0,0,0,0);
        glDisable(GL_DEPTH_TEST);
        glMatrixMode(GL_MODELVIEW);
        projection_matrix = win->vs.wallShadowMatrix[i-1];
        glStencilFunc(GL_EQUAL, 0x3, 0x2);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glPushMatrix();
          glMultMatrixd(projection_matrix);
          win->vs.allIsBlack= TRUE;
          g3d_draw_robots(win);
          g3d_draw_obstacles(win);
        glPopMatrix();
        glColorMask(1,1,1,1);
        win->vs.allIsBlack= FALSE;

        glStencilFunc(GL_EQUAL, 0x1, 0x1);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
        g3d_set_dim_light();
        g3d_set_shade_material();
        glEnable(GL_LIGHTING);
        g3d_draw_wall(i, win->vs.wallColor, 1);
      }

      g3d_set_light(win->vs);
      g3d_set_default_material();
      glEnable(GL_DEPTH_TEST); 
      glDisable(GL_STENCIL_TEST);
      g3d_draw_AA_box(xmin, xmax, ymin, ymax, zmin, zmax);
    }


    glEnable(GL_DEPTH_TEST); 
    glDisable(GL_STENCIL_TEST);
  }
  glPopAttrib();
  //////////////////////END OF FUNCTION MAIN CORE///////////////////

  if (win->fct_draw2 != NULL) win->fct_draw2();
  
  if(win->vs.displayJoints) {
    g3d_draw_robot_joints(XYZ_ENV->cur_robot, 0.1);
    //g3d_draw_robot_kinematic_chain(XYZ_ENV->cur_robot);
  }
 


#ifdef HRI_PLANNER
  gpsp_draw_robots_fov(win);
  psp_draw_elements(win);
#endif
	
#ifdef P3D_COLLISION_CHECKING

// printf("psp\n");
//  {-0.100170963, -0.972703815, -0.209316134, 10.4761086},
//  {-0.631904542, 0.746246278, -0.20931612, 17.3667202},
//  {-0.717136264, -0.221836269, 0.660684645, 11.4558439},
//  {0.143646017, 0.0444349162, -0.98863107, 10.3555546},
//  {0.839170814, 0.260432333, 0.477459252, 13409.1396},
//  {-0.838386714, -0.259343415, -0.479425579, 31.8722878}

// g3d_extract_frustum(win);
// printf("frustum\n");
// printf(" %f %f %f %f \n",win->vs.frustum[0][0],win->vs.frustum[0][1],win->vs.frustum[0][2],win->vs.frustum[0][3]);
// printf(" %f %f %f %f \n",win->vs.frustum[1][0],win->vs.frustum[1][1],win->vs.frustum[1][2],win->vs.frustum[1][3]);
// printf(" %f %f %f %f \n",win->vs.frustum[2][0],win->vs.frustum[2][1],win->vs.frustum[2][2],win->vs.frustum[2][3]);
// printf(" %f %f %f %f \n",win->vs.frustum[3][0],win->vs.frustum[3][1],win->vs.frustum[3][2],win->vs.frustum[3][3]);
// printf(" %f %f %f %f \n",win->vs.frustum[4][0],win->vs.frustum[4][1],win->vs.frustum[4][2],win->vs.frustum[4][3]);
// printf(" %f %f %f %f \n",win->vs.frustum[5][0],win->vs.frustum[5][1],win->vs.frustum[5][2],win->vs.frustum[5][3]);
	
  g3d_kcd_draw_all_aabbs();     // draw AABBs around static primitives
  g3d_kcd_draw_aabb_hier();     // draw AABB tree on static objects
  g3d_kcd_draw_robot_obbs();    // draw all obbs of current robot
  g3d_kcd_draw_all_obbs();      // draw all static obbs

  g3d_kcd_draw_closest_points();
#endif
	
  #ifdef DPG
  if(XYZ_GRAPH && XYZ_GRAPH->dpgGrid){
     XYZ_GRAPH->dpgGrid->draw();
   }
  #endif

  /* Debut Modification Thibaut */
  if (G3D_DRAW_OCUR_SPECIAL) g3d_draw_ocur_special(win);
  /* Fin Modification Thibaut */

#ifdef P3D_PLANNER
  if(XYZ_GRAPH && ENV.getBool(Env::drawGraph)){
	  g3d_draw_graph();
  }
#endif

  if (ENV.getBool(Env::drawTraj)) {
    g3d_draw_all_tcur();
  }
  if (G3D_DRAW_TRACE) {
    p3d_set_and_update_robot_conf(robotPt->ROBOT_POS);
    /* collision checking */
#ifdef P3D_COLLISION_CHECKING
    p3d_numcoll = p3d_col_test_all();
#endif
    win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
    g3d_draw_robot(robotPt->num, win);
    p3d_set_and_update_robot_conf(robotPt->ROBOT_GOTO);
    /* collision checking */
#ifdef P3D_COLLISION_CHECKING
    p3d_numcoll = p3d_col_test_all();
#endif
    win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
    g3d_draw_robot(robotPt->num, win);
    win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
    g3d_draw_trace_all_tcur();
  }


//     //On dessine la source de lumière sous la forme d'une sphère:
//   glDisable( GL_LIGHTING );
//   glColor3f(1.0, 1.0, 0.0);
//   glPushMatrix();
//   {
//    glLightdv( GL_LIGHT0, GL_POSITION, win->vs.lightPosition );
//    glTranslatef( win->vs.lightPosition[0], win->vs.lightPosition[1], win->vs.lightPosition[2] );
//    g3d_drawColorSphere(0, 0, 0, 50, Yellow, NULL);
//   }
//   glPopMatrix();
//   glEnable( GL_LIGHTING );

  if (G3D_MODIF_VIEW && win->vs.displayFrame) {
    glPushMatrix();
    glTranslatef(win->vs.x, win->vs.y, win->vs.z);
	if(ENV.getBool(Env::drawFrame))
	{
	  g3d_draw_frame();
	}
    glPopMatrix();
  }
  p3d_drawRobotMoveMeshs();



#ifdef HRI_PLANNER
if (!win->win_perspective) {
   //hri_hri_inter_point_test();
   g3d_hri_bt_draw_active_bitmaps(BTSET);
   g3d_hri_bt_draw_active_3dbitmaps(INTERPOINT);
   g3d_hri_bt_draw_active_3dbitmaps(OBJSET);
   g3d_hri_bt_draw_targets(BTSET);
   hri_exp_draw_ordered_points();
   //g3d_hri_display_test();
   if(HRI_DRAW_TRAJ){g3d_draw_all_tcur();}
#ifdef USE_MIGHTABILITY_MAPS
    execute_Mightability_Map_functions();
#endif
 } else {
   if (win->draw_mode!=NORMAL)
     g3d_set_light_persp();
   psp_draw_in_perspwin();
 }
#endif


  if(ENV.getBool(Env::drawLightSource))
  {
       glDisable( GL_LIGHTING );
       glColor3f(1.0, 1.0, 0.0);
       glPushMatrix();
       {
           glLightfv( GL_LIGHT0, GL_POSITION, win->vs.lightPosition );
           glTranslatef( win->vs.lightPosition[0], win->vs.lightPosition[1], win->vs.lightPosition[2] );
           g3d_drawColorSphere(0, 0, 0, 0.10, Yellow, NULL);
       }
       glPopMatrix();
       glEnable( GL_LIGHTING );
   }
	
	/**
	 * Transparent cost grids
	 */
	g3d_draw_grids();
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

      p3d_obj *obstacle = (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);
      if(obstacle->display_mode==P3D_OBJ_NO_DISPLAY) {
        continue;
      }

      /*  ChronoOn(); */
      g3d_draw_obstacle(win);
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

  r = p3d_get_desc_curnum(P3D_ROBOT);
  nr = p3d_get_desc_number(P3D_ROBOT);

  if (nr) {
    for (ir = 0;ir < nr;ir++) {
      p3d_sel_desc_num(P3D_ROBOT, ir);
      p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
      if(rob->display_mode==P3D_ROB_NO_DISPLAY) {
        continue;
      }
 #ifdef HRI_PLANNER
	  if (win->win_perspective){
		if (win->draw_mode==OBJECTIF){
		  if (rob->caption_selected)
			  g3d_draw_robot(ir,win);
		}
		else
		{
		  g3d_draw_robot(ir,win);
		}
	  }
	  else
#endif
//            g3d_draw_rob_BB((p3d_rob *) p3d_get_desc_curid(P3D_ROBOT));
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

#ifdef P3D_COLLISION_CHECKING
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
#endif
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
    printf("creation(%f,%f,%f,%f,%f,%f)\n",x1, x2, y1, y2, z1, z2);

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
    p3d_get_env_box(&x1, &x2, &y1, &y2, &z1, &z2);
    printf("affichage(%f,%f,%f,%f,%f,%f)\n",x1, x2, y1, y2, z1, z2);
  }
  glLineWidth(1);
}

/***************************************/
/* Fonction tracant l'obstacle courant */
/***************************************/
void g3d_draw_obstacle(G3D_Window *win) {
  pp3d_obj o;
  o = (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);

  g3d_draw_object(o, 0, win);
#ifdef HRI_PLANNER	  
  if (!win->win_perspective && o->caption_selected)
    g3d_draw_obj_BB(o);
#endif
  
#ifdef HRI_PLANNER	
  else
	  if ( o->caption_selected)
		  PSP_CURR_DRAW_OBJ++;
#endif
	
}




/***********************************************/
/* Fonction tracant le robot courant en tenant */
/* compte de s'il a percute un obstacle ou non */
/***********************************************/
void g3d_draw_robot(int ir, G3D_Window* win) {
  int nb, b, ib, num;
  int coll = 0;

  b = p3d_get_desc_curnum(P3D_BODY);
  nb = p3d_get_desc_number(P3D_BODY);

  num = p3d_get_desc_curnum(P3D_ROBOT);

#ifdef P3D_COLLISION_CHECKING
  if (p3d_numcoll) {
    coll = p3d_col_does_robot_collide(ir, p3d_numcoll);
  }
#endif
	
  for (ib = 0;ib < nb;ib++) {
    p3d_sel_desc_num(P3D_BODY, ib);
    g3d_draw_body(coll, win);
  }
  p3d_sel_desc_num(P3D_BODY,b);
  
  p3d_rob *r;
  r=(p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
#ifdef HRI_PLANNER
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

#ifdef DPG
  if(ENV.getBool(Env::drawGrid) && r->GRAPH && r->GRAPH->dpgGrid){
    for(int i = 0; i < r->nbDpgCells; i++){
      r->dpgCells[i]->draw(Green, 2);
    }
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
void g3d_draw_body(int coll, G3D_Window* win) {
  pp3d_obj o;

  o = (p3d_obj *) p3d_get_desc_curid(P3D_BODY);

  g3d_draw_object_moved(o, coll, win);
#ifdef HRI_PLANNER
   if (!win->win_perspective && o->caption_selected)
     g3d_draw_obj_BB(o);
#endif
}

/*******************************************/
/* Fonction dessinant un objet en position */
/*******************************************/
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
  glMultMatrixd(matrix_pos_absGL);
  g3d_draw_object(o, coll, win);
  glPopMatrix();
  if (win->vs.BB == TRUE) {
    g3d_draw_obj_BB(o);
  }
}


/*******************************/
/* Fonction dessinant un objet */
/*******************************/
void g3d_draw_object(p3d_obj *o, int coll, G3D_Window *win) {
  int i, transparent;
  int black;
  glLoadName(o->o_id_in_env);

#ifdef HRI_PLANNER
  int colorindex = 0;
  int colltemp, istrans;

  if (PSP_NUM_OBJECTS==0){
    colorindex = 1;
  }
  else{
    colorindex = (PSP_CURR_DRAW_OBJ+1) * (PSP_MAX_COLOR_IDX/(PSP_NUM_OBJECTS*1.0));
    PSP_DRAW_OBJ_COL_INDEX[PSP_CURR_DRAW_OBJ] = colorindex;
  }

  if(win->draw_mode==OBJECTIF){ //If is indicated to draw only the objective
    if (o->caption_selected){ // if the object if marked as part of the objective
      colltemp = 2;

      for(i=0;i<o->np;i++){
                  if (o->pol[i]->TYPE!=P3D_GHOST || win->vs.GHOST == TRUE){

                        //check if the poly is transparent or not to know if we have to display it:
                          if(coll!=0)
                          { transparent= 0;   }
                          else
                          {  transparent= g3d_is_poly_transparent(o->pol[i]);   }
                          if(!transparent && win->vs.transparency_mode==G3D_TRANSPARENT)
                          {  continue; }
                          if(transparent && win->vs.transparency_mode==G3D_OPAQUE)
                          {  continue; }

                          //flat shading display:
                          if( !win->vs.FILAIRE && !win->vs.GOURAUD )
                          { g3d_draw_poly_with_color(o->pol[i],win,colltemp,1,colorindex); }
                          //smooth shading display:
                          if( !win->vs.FILAIRE && win->vs.GOURAUD )
                          { g3d_draw_poly_with_color(o->pol[i],win,colltemp,2,colorindex); }
                          //simple wire display:
                          if( win->vs.FILAIRE && !win->vs.CONTOUR )
                          { g3d_draw_poly_with_color(o->pol[i],win,colltemp,0,colorindex); }
                          //contour display:
                          if(win->vs.CONTOUR)
                          {
                            black= win->vs.allIsBlack;
                            win->vs.allIsBlack= TRUE;
                            g3d_draw_poly_with_color(o->pol[i],win,colltemp,0,colorindex);
                            win->vs.allIsBlack= black;
                          }
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
        if (o->pol[i]->TYPE!=P3D_GHOST || win->vs.GHOST == TRUE){
	  if(colltemp !=2 && colltemp !=3) { colorindex = o->pol[i]->color;  }

         //check if the poly is transparent or not to know if we have to display it:
          if(coll!=0)
          { transparent= 0;   }
          else
          {  transparent= g3d_is_poly_transparent(o->pol[i]);   }
          if(!transparent && win->vs.transparency_mode==G3D_TRANSPARENT)
          {  continue; }
          if(transparent && win->vs.transparency_mode==G3D_OPAQUE)
          {  continue; }

          //flat shading display:
          if( !win->vs.FILAIRE && !win->vs.GOURAUD )
            {g3d_draw_poly_with_color(o->pol[i],win,colltemp,1,colorindex);}
          //smooth shading display:
          if( !win->vs.FILAIRE && win->vs.GOURAUD)
          {g3d_draw_poly_with_color(o->pol[i],win,colltemp,2,colorindex);}
          //simple wire display:
          if( win->vs.FILAIRE && !win->vs.CONTOUR )
          {g3d_draw_poly_with_color(o->pol[i],win,colltemp,0,colorindex);}
          //contour display:
          if(win->vs.CONTOUR)
          {
            black= win->vs.allIsBlack;
            win->vs.allIsBlack= TRUE;
            g3d_draw_poly_with_color(o->pol[i],win,colltemp,0,colorindex);
            win->vs.allIsBlack= black;
          }
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
    }
  }
#else
  for(i=0;i<o->np;i++){
    if (o->pol[i]->TYPE != P3D_GHOST || win->vs.GHOST == TRUE){

      //check if the poly is transparent or not to know if we have to display it:
      if(coll!=0)
      { transparent= 0;   }
      else
      {  transparent= g3d_is_poly_transparent(o->pol[i]);   }
      if(!transparent && win->vs.transparency_mode==G3D_TRANSPARENT)
      {  continue; }
      if(transparent && win->vs.transparency_mode==G3D_OPAQUE)
      {  continue; }

      //flat shading display:
      if((!win->vs.FILAIRE)&&(!win->vs.GOURAUD))
      {g3d_draw_poly(o->pol[i],win,coll,1);}
      //smooth shading display:
      if((!win->vs.FILAIRE)&&(win->vs.GOURAUD))
      {g3d_draw_poly(o->pol[i],win,coll,2);}
      //wire display:
      if((win->vs.FILAIRE && !win->vs.CONTOUR))
      {g3d_draw_poly(o->pol[i],win,coll,0);}
      //contour display:
      if(win->vs.CONTOUR)
      {   
        black= win->vs.allIsBlack;
        win->vs.allIsBlack= TRUE;
        g3d_draw_poly(o->pol[i],win,coll,0);
        win->vs.allIsBlack= black;
      }
    }
  }
#endif
}

/***************************************************/
/* Fonction tracant la boite englobante d'un objet */
/***************************************************/

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

// static
// void g3d_draw_rob_BB(p3d_rob *r) {
//   double x1, x2, y1, y2, z1, z2;
// 
//   p3d_get_BB_rob(r, &x1, &x2, &y1, &y2, &z1, &z2); /* new Carl 23052001 */
//   /*  x1 = r->BB.xmin; */
//   /*  x2 = r->BB.xmax; */
//   /*  y1 = r->BB.ymin; */
//   /*  y2 = r->BB.ymax; */
//   /*  z1 = r->BB.zmin; */
//   /*  z2 = r->BB.zmax; */
//   PrintInfo(("x1=%f,x2=%f,y1=%f,y2=%f,z1=%f,z2=%f\n", x1, x2, y1, y2, z1, z2);
//             g3d_draw_a_box(x1, x2, y1, y2, z1, z2, Yellow, 0));
// }

void showConfig(configPt conf){
  p3d_set_and_update_robot_conf(conf);
  g3d_refresh_allwin_active();
  sleep(1);
}

//! This function is used to know how much a robot (visually) hides an object from a specific viewpoint.
//! \param camera_frame the frame of the viewpoint (the looking direction is X, Y points downward and Z to the left)
//! \param camera_fov the field of view angle of the robot's camera (in degrees)
//! \param robot pointer to the robot
//! \param object pointer to the object
//! \param result return the ratio between the number of object's pixels that are visible
//! and the overall size (in pixels) of the image. So the value is between 0 (invisible object) and 1 (the object
//! occupies all the image).
//! \return 0 in case of success, 1 otherwise
int g3d_does_robot_hide_object(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *robot, p3d_rob *object, double *result)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_does_robot_hide_object(): input robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
  if(object==NULL)
  {
    printf("%s: %d: g3d_does_robot_hide_object(): input object is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
 
  int i, width, height;
  GLint viewport[4];
  int displayFrame, displayJoints, displayShadows, displayWalls, displayFloor, displayTiles, cullingEnabled;
  double fov;
  int count;
  unsigned char *image= NULL;
  float red, green, blue;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");

  // disable the display of all obstacles and of all the robots of no interest:
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_NO_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    if(XYZ_ENV->robot[i]==robot || XYZ_ENV->robot[i]==object) {
       continue;
    }
    else {
      p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_NO_DISPLAY);
    }
  }
  // display our robot and object with specific colors:
  p3d_set_robot_display_mode(robot, P3D_ROB_GREEN_DISPLAY);
  p3d_set_robot_display_mode(object, P3D_ROB_RED_DISPLAY);

  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];
 
  // save the current display options:
  g3d_save_win_camera(win->vs);
  fov            =  win->vs.fov;
  displayFrame   =  win->vs.displayFrame;
  displayJoints  =  win->vs.displayJoints;
  displayShadows =  win->vs.displayShadows;
  displayWalls   =  win->vs.displayWalls;
  displayFloor   =  win->vs.displayFloor;
  displayTiles   =  win->vs.displayTiles;
  cullingEnabled =  win->vs.cullingEnabled;
  red            =  win->vs.bg[0]; 
  green          =  win->vs.bg[1]; 
  blue           =  win->vs.bg[2]; 

  // only keep what is necessary:
  win->vs.fov            = camera_fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled=  1;
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);


  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);


  g3d_draw_win(win);

  // restore the display options:
  g3d_restore_win_camera(win->vs);
  win->vs.fov            = fov;
  win->vs.displayFrame   = displayFrame;
  win->vs.displayJoints  = displayJoints;
  win->vs.displayShadows = displayShadows;
  win->vs.displayWalls   = displayWalls;
  win->vs.displayFloor   = displayFloor;
  win->vs.displayTiles   = displayTiles;
  win->vs.cullingEnabled =  cullingEnabled;
  g3d_set_win_bgcolor(win->vs, red, green, blue);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov

  // reset the display modes of everything
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_DEFAULT_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_DEFAULT_DISPLAY);
  }

//   static int cnt= 0;
//   char name[256];
//   sprintf(name, "/home/jpsaut/BioMove3Dgit/BioMove3D/screenshots/image%i.ppm", cnt++);
//   g3d_export_OpenGL_display(name);


  // get the OpenGL image buffer:
  image = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration

  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  // get the image pixels (from (0,0) position):
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

  // count the pixels corresponding to the object's color:
  count= 0;
  for(i=0; i<width*height; i++)
  {
    if(image[3*i] > 0.8) {
      count++;
    }
  }

  *result= ((double) count)/((double) width*height);

  free(image);

  return 0;
}

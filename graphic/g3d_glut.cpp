/*
 *  glutWindow.cpp
 *  OOMove3D
 *
 *  Created by Jim Mainprice on 25/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#if defined(MACOSX) 
#include <glut.h>
#else
#include <GL/freeglut.h>
#endif

#include "Graphic-pkg.h"
#include "Move3d-pkg.h"

using namespace std;

extern void* GroundCostObj;

void (*draw_opengl)();

//! reshape the OPenGL display
//! changes the matrix when the window changes sizes
void g3d_glut_reshapeGL(int w, int h)
{
  glViewport(0, 0, (GLint) w, (GLint) h);
  //	qglClearColor(QColor::fromCmykF(0.0, 0.0, 0.0, 0.0));
	
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
	
  double size = G3D_WIN->vs.size;
	
  gluPerspective(40.0, (GLdouble) w / (GLdouble) h, size / 10000.0, size * 1000.0);
	
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

//! Initilizes the OpenGL display
//! Sets the matrices at start
void g3d_glut_initializeGL(int w,int h)
{
  glViewport(0,0,(GLint) w,(GLint) h);
  //glClearColor(G3D_WIN->vs.bg[0],G3D_WIN->vs.bg[1],G3D_WIN->vs.bg[2],.0);
	
  //   glMatrixMode(GL_PROJECTION);
  //   glLoadIdentity();
  g3d_set_projection_matrix(G3D_WIN->vs.projection_mode);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
	
  /** on desactive tout mode OpenGL inutile ***/
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_ALPHA_TEST);
	
  glEnable(GL_DEPTH_TEST);
	
  if(G3D_WIN->vs.GOURAUD) 
    {
      glShadeModel(GL_SMOOTH);
    } else 
    {
      glShadeModel(GL_FLAT);
    }
	
	
  if(GroundCostObj)
    {
      cout << "GroundCostObj, vs.displayFloor = false" << endl;
      G3D_WIN->vs.displayFloor = false;
    }
}

bool init = true;

// place on the screen
int _i; int _j;

void g3d_glut_get_win_mouse(int* i, int* j)
{
  //cout << "qt_get_win_mouse" <<  endl;
  *i = _i;
  *j = _j;
}

void g3d_glut_ui_calc_param(g3d_cam_param& p)
{
  //cout << "g3d_glut_ui_calc_param" <<  endl;
  p3d_vector4 up;
	
  calc_cam_param(G3D_WIN, p.Xc, p.Xw);
	
  if (G3D_WIN)
    {
      p3d_matvec4Mult(*G3D_WIN->cam_frame, G3D_WIN->vs.up, up);
    }
  else
    {
      up[0] = 0;
      up[1] = 0;
      up[2] = 1;
    }
	
  p.up[0] = up[0];
  p.up[1] = up[1];
  p.up[2] = up[2];
}


void g3d_glut_paintGL()
{
  if(init)
    {
      g3d_glut_initializeGL(800,600);
      init = false;
    }
	
  glPushMatrix();
	
  //	computeNewVectors(Xc,Xw,up);
  g3d_cam_param p;
  ext_g3d_calc_cam_param(p);
	
  gluLookAt(p.Xc[0], p.Xc[1], p.Xc[2], 
	    p.Xw[0], p.Xw[1], p.Xw[2], 
	    p.up[0], p.up[1], p.up[2]);
  
  //  cout << " Xc = " << p.Xc[0] << " , " << p.Xc[1] << " , " << p.Xc[2] << endl; 
  //  cout << " Xw = " << p.Xw[0] << " , " << p.Xw[1] << " , " << p.Xw[2] << endl; 
  //  cout << " up = " << p.up[0] << " , " << p.up[1] << " , " << p.up[2] << endl;
	
  G3D_WIN->vs.cameraPosition[0]= p.Xc[0];
  G3D_WIN->vs.cameraPosition[1]= p.Xc[1];
  G3D_WIN->vs.cameraPosition[2]= p.Xc[2];  
	
  g3d_draw();
	
  glPopMatrix();
  glutSwapBuffers();
}


//! Creates a Glut window
GlutWindowDisplay::GlutWindowDisplay(int argc, char *argv[])
{
  int argc2 = 0;
  char* argv2 = "";
	
  /* initialisation de GLUT */
  glutInit (&argc2, &argv2);
	
  /* création d'une fenêtre OpenGL RVBA avec en simple mémoire tampon
     avec un tampon de profondeur */
  //glutInitDisplayMode (GLUT_RGB | GLUT_SINGLE | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL | GLUT_STEREO);
  //glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL | GLUT_ALPHA );
  glutInitWindowSize (800, 600);
  glutCreateWindow ("Move3D Glut"); 
}


void GlutWindowDisplay::initDisplay()
{
  cout << __func__ << " in " << __FILE__  << endl;
	
  //Graphic::initDrawFunctions();
  // Initializes the G3D Window
  new qtG3DWindow;
	
  // Initializes Draw Functions
  draw_opengl = g3d_draw_allwin_active;
  ext_g3d_draw_allwin_active = g3d_draw_allwin_active;
  ext_g3d_get_win_mouse = g3d_glut_get_win_mouse;
  ext_g3d_calc_cam_param = g3d_glut_ui_calc_param;
  
  G3D_WIN->vs.displayFloor = true;
  G3D_WIN->vs.displayTiles = true;
  G3D_WIN->vs.GOURAUD = true;
	
  /* initialisation des fonctions callback appelées par glut 
     pour respectivement le redimensionnement de la fenêtre
     et le rendu }de la scène */
  glutReshapeFunc (g3d_glut_reshapeGL);
  glutDisplayFunc (g3d_glut_paintGL);
}


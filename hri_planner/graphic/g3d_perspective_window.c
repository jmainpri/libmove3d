#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"
#include "Hri_planner-pkg.h"


#ifdef __cplusplus
extern "C" {
#endif
#include <GL/glx.h>
/* Evil, but glx seems not to define itself correctly for glcanvas */
#define GLX_H 
#include <forms.h>
#include <glcanvas.h>
#ifdef __cplusplus
}
#endif

/*   Defined for UNIX (XForms & GLX)
 
 repris du gl.c de forms pour pouvoir utiliser des display lists  
 identiques sur les fenetres copiees */
#define MAXATTRIB  34

typedef struct {
  XVisualInfo *xvinfo;
  GLXContext context;
  int direct;
  int glconfig[MAXATTRIB];
} CSPEC;

#define GLPROP(ob)   ((CSPEC *)(ob->c_vdata))


static void recalc_cam_up         (G3D_Window *win, p3d_matrix4 transf);
static void calc_cam_param        (G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);
static void get_pos_cam_matrix    (G3D_Window *win, p3d_matrix4 Transf);
static void get_lookat_vector     (G3D_Window *win, p3d_vector4 Vec);
int  canvas_expose_special (FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud);

p3d_matrix4 WinId = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

int G3D_RESFRESH_PERSPECTIVE = TRUE;
int PSP_REFRESH_BLOCK = TRUE;

/* fonction pour recalculer le vector 'up' de la camera */
/* quand on change la reference                         */
static void recalc_cam_up(G3D_Window *win, p3d_matrix4 transf) {
  p3d_vector4 v_up;
  int i;
	
  p3d_matvec4Mult(transf,win->up,v_up);
  for(i=0;i<4;i++) win->up[i] = v_up[i];
}

G3D_Window *g3d_show_persp_win() 
{ 
  G3D_Window *win = g3d_get_win_by_name((char*)"Move3D");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas); 
  G3D_Window *newwin; 
  //char     str[256]; 
  int        w,h; 
  int        i,j; 
  p3d_vector4 Xc,Xw;
  fl_get_winsize(FL_ObjWin(ob),&w,&h); 
  //sprintf(str,"%s->copy",win->name);

  newwin = g3d_new_win_wo_buttons((char*)"Perspective",w/2,w/3,win->size); /* 1.33 is the standard ratio of camera images */
  
  /* pour associer un context identique au canvas de la fenetre */ 
  FL_OBJECT   *newob = ((FL_OBJECT *)newwin->canvas); 
  XVisualInfo *vi = glXChooseVisual(fl_display,fl_screen,GLPROP(newob)->glconfig); 
  glXDestroyContext(fl_display,GLPROP(newob)->context); 
  GLPROP(newob)->context = glXCreateContext(fl_display,vi, 
																						fl_get_glcanvas_context(ob), 
																						GLPROP(newob)->direct); 
  
  //new->FILAIRE = win->FILAIRE; 
  //new->CONTOUR = win->CONTOUR; 
  newwin->GOURAUD = win->GOURAUD;
  for(i = 0; i < 6; i++){
    for(j = 0; j < 4; j++){
      newwin->frustum[i][j] = win->frustum[i][j];
    }
	}
  g3d_set_win_drawer(newwin,win->fct_draw); 
  g3d_set_win_bgcolor(newwin, win->bg[0],win->bg[1],win->bg[2]); 
  g3d_set_win_fct_mobcam(newwin,win->fct_mobcam); 
  if(win->cam_frame == &WinId) { 
    g3d_set_win_camera(newwin, win->x,win->y,win->z,win->zo,win->az,win->el,win->up[0],win->up[1],win->up[2]); 
  } 
  else { 
    calc_cam_param(win,Xc,Xw); 
    //recalc_mouse_param(newwin,Xc,Xw); 
    for(i=0;i< 4;i++) 
      newwin->up[i] = win->up[i]; 
    recalc_cam_up(newwin,*win->cam_frame); 
    newwin->zo = win->zo; 
  }
	
  return(newwin); 
} 

void g3d_set_win_draw_mode(G3D_Window *w,g3d_window_draw_mode mode)
{
  if (mode != NORMAL)
		g3d_set_win_bgcolor(w,0.0,0.0,0.0);
  else
    g3d_set_win_bgcolor(w,1.0,1.0,1.0);
  
  w->draw_mode = mode;  
	
}


/* fonction pour calculer les parametres de position de la      */ 
/* camera de la facon necessaire pour openGL                    */ 
static void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) 
{ 
  static p3d_matrix4 Txc = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}; 
  p3d_matrix4 m_aux; 
  p3d_vector4 Xx; 
	
	if (win->point_of_view==0) {
		get_lookat_vector(win, Xx); 
		get_pos_cam_matrix(win, Txc); 
		p3d_mat4Mult(*win->cam_frame,Txc,m_aux); 
		p3d_matvec4Mult(m_aux,Xx,Xc); 
		p3d_matvec4Mult(*win->cam_frame,Xx,Xw);  
	} 
  else { 
		if (PSP_ROBOT) {
			p3d_rob *r = PSP_ROBOT;  
			p3d_obj *objPt = r->o[r->cam_body_index]; 
			p3d_jnt *jntPt = objPt->jnt;
			
			//Robot cam
			//Setting up camera position
			Xx[0]=r->cam_pos[0];
			Xx[1]=r->cam_pos[1];
			Xx[2]=r->cam_pos[2];
			Xx[3]=1;
			p3d_matvec4Mult(jntPt->abs_pos,Xx,Xc);
			//Setting up camera orientation
			Xx[0]=r->cam_dir[0];
			Xx[1]=r->cam_dir[1];
			Xx[2]=r->cam_dir[2];
			p3d_matvec4Mult(jntPt->abs_pos,Xx,Xw);
		}
	}
}



static void get_pos_cam_matrix(G3D_Window *win, p3d_matrix4 Transf) {
  /* Caution: ici on change les parametres de translation de la */
  /* matrix, les rest des elementes doivent etre initialises    */
  /* dans la fonction qu'appel                                  */
  Transf[0][3] = win->zo * (cos(win->az)*cos(win->el));
  Transf[1][3] = win->zo * (sin(win->az)*cos(win->el));
  Transf[2][3] = win->zo * sin(win->el);
}

/* fonctions pour copier les donnees relies a la camera de     */
/* la structure G3D_Window de facon utilisable dans operations */
/* avec transformations homogenes                              */
static void get_lookat_vector(G3D_Window *win, p3d_vector4 Vec) {
  Vec[0] = win->x;
  Vec[1] = win->y;
  Vec[2] = win->z;
  Vec[3] = 1.0;
}






int canvas_expose_special(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud) 
{ 
  G3D_Window *g3dwin = (G3D_Window *)ud; 
  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))  
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob)); 
  
	if (g3dwin->draw_mode==NORMAL)
	{	
		glDisable(GL_STENCIL_TEST);
		glDisable(GL_SCISSOR_TEST);
		glDisable(GL_ALPHA_TEST);
  
		glDisable(GL_DEPTH_TEST); 
  // glDisable(GL_SHADING); 
	}
	else
	{
		glDisable(GL_DEPTH_TEST); 
  //glDisable(GL_BLEND); 
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
	}
  
  glViewport(0,0,(GLint)w,(GLint)h);
	//printf("w/2 = %i h/2 = %i \n", w/2, h/2);
  glClearColor(g3dwin->bg[0],g3dwin->bg[1],g3dwin->bg[2],.0); 
  
  
  glMatrixMode(GL_PROJECTION); 
  glLoadIdentity();
	
  if (g3dwin->win_perspective)
	{
		if (PSP_ROBOT)
		{
			//pp3d_rob r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
			//float degXang = (PSP_ROBOT->cam_h_angle*180.0/M_PI);
			//float degYang = (PSP_ROBOT->cam_v_angle * 180.0/M_PI);//good 
			float degYang = (((PSP_ROBOT->cam_h_angle*2.0)/3.0) * 180.0/M_PI);
			//gluPerspective(degYang, PSP_ROBOT->cam_h_angle/PSP_ROBOT->cam_v_angle ,0.1, 10);//good
			gluPerspective(degYang, 3.0/2.0 ,0.1, 10);//real robot camera reference
			//gluPerspective(degYang, degXang/degYang ,g3dwin->size/100.0, 100.0*g3dwin->size); //original one
			//glFrustum(-degYang/2,degYang/2,-degXang/2,degXang/2,PSP_ROBOT->cam_min_range,PSP_ROBOT->cam_max_range);
			//glFrustum(-.5,.5,-.5,.5,0.1,20.0);
		}
	}
  else
	{
		gluPerspective(40.0,(GLdouble)w/(GLdouble)h,g3dwin->size/100.0,100.0*g3dwin->size); 
	}
  
  glMatrixMode(GL_MODELVIEW); 
  glLoadIdentity(); 
	
  if (g3dwin->draw_mode==NORMAL)
  {
  // on desactive tout mode OpenGL inutile 
	  glDisable(GL_STENCIL_TEST);
	  glDisable(GL_SCISSOR_TEST);
	  glDisable(GL_ALPHA_TEST);
  }
  else
	{
		glDisable(GL_DEPTH_TEST); 
        //glDisable(GL_BLEND); 
        glDisable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
		//printf("here\n");
	}
  if(g3dwin->GOURAUD){ 
    glShadeModel(GL_SMOOTH); 
  } 
  else{ 
    glShadeModel(GL_FLAT); 
  }
  
  glXWaitX();
  g3d_draw_win2(g3dwin);
  glXWaitGL();
  /*  */
  return(TRUE); 
}   


void g3d_refresh_win(G3D_Window *w) 
{
  FL_OBJECT  *ob;
  int winw,winh;
  w->list = -1; 
  ob = ((FL_OBJECT *)w->canvas); 
  fl_get_winsize(FL_ObjWin(ob),&winw,&winh); 
	
  canvas_expose_special(ob, NULL, winw,winh, NULL, w);
	
}

extern G3D_Window *G3D_WINDOW_CUR;
extern int G3D_MODIF_VIEW;

void g3d_draw_win2(G3D_Window *win) 
{  
  p3d_vector4 Xc,Xw; 
  p3d_vector4 up; 
	
  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas); 
	
  G3D_WINDOW_CUR = win; 
  PSP_CURR_DRAW_OBJ=0;
  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob)) 
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));
  
	
  glClearColor(win->bg[0],win->bg[1],win->bg[2],.0); 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (win->draw_mode != NORMAL)
		glClearDepth(1.0f);
	else
	{
  //glClear(GL_COLOR_BUFFER_BIT);
	
		if(win->GOURAUD){ 
			glShadeModel(GL_SMOOTH); 
		} 
		else{ 
			glShadeModel(GL_FLAT); 
		}
	}
	
  calc_cam_param(win,Xc,Xw); 
  
  p3d_matvec4Mult(*win->cam_frame,win->up,up); 
  
  glPushMatrix(); 
	
  ///Luis
  ////////////////////////////AQUI WEY 
	//glBlendFunc(GL_SRC_ALPHA,GL_ONE);	
	//glBlendFunc(GL_CONSTANT_COLOR,GL_CONSTANT_COLOR);	
	gluLookAt(Xc[0],Xc[1],Xc[2],Xw[0],Xw[1],Xw[2],up[0],up[1],up[2]); 
	
  
  if(G3D_MODIF_VIEW) { 
    //g3d_draw_frame();
    glPushMatrix(); 
    glTranslatef(win->x,win->y,win->z); 
    g3d_draw_frame(); 
    glPopMatrix(); 
  } 
  
  if(win->fct_draw) (*win->fct_draw)(); 
  glPopMatrix(); 
  // glFinish();
	
	//  if (win->win_perspective)
	//    {
	//if (win->draw_mode != NORMAL)
	//{
	//	glDisable(GL_COLOR_MATERIAL);
	//	glDisable(GL_LIGHTING);
	//	glDisable(GL_LIGHT0);
	//}
	//glDepthFunc(GL_LEQUAL);
	//    }
  if(G3D_RESFRESH_PERSPECTIVE)
  {
	  glFlush();
    glXSwapBuffers(fl_display,fl_get_canvas_id(ob)); 
	  
  }
	  /*glXWaitGL();*/ /**** Jean-Gerard ***/
} 

void g3d_set_light_persp(void)
{
	
  p3d_vector4 Xx,Xc;
	p3d_jnt *jntPt =  PSP_ROBOT->o[PSP_ROBOT->cam_body_index]->jnt;
  Xx[0]=PSP_ROBOT->cam_pos[0];
  Xx[1]=PSP_ROBOT->cam_pos[1];
  Xx[2]=PSP_ROBOT->cam_pos[2];
  Xx[3]=1;
  p3d_matvec4Mult(jntPt->abs_pos,Xx,Xc);
	
	//GLfloat light_position[] = { 20.0, -60.0, 100.0, 1.0 }; 
  GLfloat light_position[] = { Xc[0], Xc[1], Xc[2], 1.0 }; 
  GLfloat light_ambient[] = { 1, 1, 1, 1.0 };
  double x1,y1,x2,y2,z1,z2;
  double xmil=0.,ymil=0.,zmil=0.,ampl=0.,xampl=0.,yampl=0.,zampl=0.;
  double factor = g3d_get_light_factor();
	
  if(p3d_get_desc_number(P3D_ENV)) {
    p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
    xmil=(x2+x1)/2.; ymil=(y2+y1)/2.; zmil=(z2+z1)/2.;
    xampl=(x2-x1)/2.;yampl=(y2-y1)/2.;zampl=(z2-z1)/2.;
    ampl = factor*sqrt(xampl*xampl+yampl*yampl+zampl*zampl);
    light_position[0]=xmil; 
    light_position[1]=ymil; 
    light_position[2]=zmil+0.5*zampl;
  } 
  //glLightfv(GL_LIGHT0, GL_POSITION, light_position); 
  //glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 2./ampl); 
  glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT1);
  
	
}

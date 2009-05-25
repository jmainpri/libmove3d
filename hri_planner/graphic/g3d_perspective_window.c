#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"
#include "Hri_planner-pkg.h"


#ifdef __cplusplus
extern "C" {
#endif
#include "GL/glx.h"
#include "forms.h"
#include "glcanvas.h"
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
static int  canvas_expose_special (FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud);

p3d_matrix4 WinId = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

/* fonction pour recalculer le vector 'up' de la camera */
/* quand on change la reference                         */
static void
recalc_cam_up(G3D_Window *win, p3d_matrix4 transf) {
  p3d_vector4 v_up;
  int i;

  p3d_matvec4Mult(transf,win->up,v_up);
  for(i=0;i<4;i++) win->up[i] = v_up[i];
}

G3D_Window *g3d_show_persp_win() 
{ 
  G3D_Window *win = g3d_get_win_by_name("Move3D");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas); 
  G3D_Window *newwin; 
  //char       str[256]; 
  int        w,h; 
  int        i,j; 
  p3d_vector4 Xc,Xw;
  fl_get_winsize(FL_ObjWin(ob),&w,&h); 
  //sprintf(str,"%s->copy",win->name);
  //wey 
 // new = g3d_new_persp_win("Perspective",w,h,win->size); 
  newwin = g3d_new_win_wo_buttons("Perspective",w/3,380/3,win->size); 
  
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
  for( i = 0; i < 6; i++)
    for(j = 0; j < 4; j++){
      newwin->frustum[i][j] = win->frustum[i][j];
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
static void 
calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) 
{ 
  static p3d_matrix4 Txc = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}; 
  p3d_matrix4 m_aux; 
  p3d_vector4 Xx; 

  //int x,y,z,tet1,tet2,tet3;
  

  if (win->point_of_view==0)
    {
       get_lookat_vector(win, Xx); 
      get_pos_cam_matrix(win, Txc); 
      p3d_mat4Mult(*win->cam_frame,Txc,m_aux); 
      p3d_matvec4Mult(m_aux,Xx,Xc); 
      p3d_matvec4Mult(*win->cam_frame,Xx,Xw);  
    } 
  else
    { 
  //Modified Luis 
      if (PSP_ROBOT)
	{
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



static void
get_pos_cam_matrix(G3D_Window *win, p3d_matrix4 Transf) {
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
static void
get_lookat_vector(G3D_Window *win, p3d_vector4 Vec) {
  Vec[0] = win->x;
  Vec[1] = win->y;
  Vec[2] = win->z;
  Vec[3] = 1.0;
}






static int canvas_expose_special(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud) 
{ 
  G3D_Window *g3dwin = (G3D_Window *)ud; 
  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))  
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob)); 
  
  //glDisable(GL_STENCIL_TEST);
  //glDisable(GL_SCISSOR_TEST);
  //glDisable(GL_ALPHA_TEST);
  
  //glDisable(GL_DEPTH_TEST); 
  // glDisable(GL_SHADING); 
  
  glDisable(GL_DEPTH_TEST); 
  //glDisable(GL_BLEND); 
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  
  
  glViewport(0,0,(GLint)w,(GLint)h); 
  glClearColor(g3dwin->bg[0],g3dwin->bg[1],g3dwin->bg[2],.0); 
  
  
  glMatrixMode(GL_PROJECTION); 
  glLoadIdentity();

  if (g3dwin->win_perspective)
    {
      if (PSP_ROBOT)
	{
	  //pp3d_rob r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	  float degXang = (PSP_ROBOT->cam_h_angle*180.0/M_PI);
	  float degYang = (PSP_ROBOT->cam_v_angle*180.0/M_PI);
	  
	  gluPerspective(degYang,degXang/degYang ,g3dwin->size/100.0, 100.0*g3dwin->size); 
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
 
  // glEnable(GL_CULL_FACE); 
  // glCullFace(GL_BACK); 
  // glFrontFace(GL_CCW); 
  
  // on desactive tout mode OpenGL inutile 
  //glDisable(GL_STENCIL_TEST);
  //glDisable(GL_SCISSOR_TEST);
  //glDisable(GL_ALPHA_TEST);
  
  glDisable(GL_DEPTH_TEST); 
  //glDisable(GL_BLEND); 
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  //printf("here\n");
  if(g3dwin->GOURAUD){ 
    glShadeModel(GL_SMOOTH); 
  } 
  else{ 
    glShadeModel(GL_FLAT); 
  }
  
  glXWaitX();
  //AKIN FIX  g3d_draw_win2(g3dwin);
  glXWaitGL();
  /*  */
  return(TRUE); 
}   


void g3d_refresh_win2(G3D_Window *w) 
{
  FL_OBJECT  *ob;
  int winw,winh;
  w->list = -1; 
  ob = ((FL_OBJECT *)w->canvas); 
  fl_get_winsize(FL_ObjWin(ob),&winw,&winh); 

  canvas_expose_special(ob, NULL, winw,winh, NULL, w);

}

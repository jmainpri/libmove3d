#include "Util-pkg.h"
#include "P3d-pkg.h"

#include "Graphic-pkg.h"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

#include "Move3d-pkg.h"
#include "Collision-pkg.h"

extern void* GroundCostObj;

#include <iostream>

#ifndef PROTO
#ifdef UNIX
#include "GL/glx.h"
#define GLX_H 

#ifdef __cplusplus
extern "C" {
#endif
#include "forms.h"
#include "glcanvas.h"
#ifdef __cplusplus
}
#endif

#endif
#endif

#define MOUSE_BTN_LEFT  256
#define MOUSE_BTN_CENTER  512
#define MOUSE_BTN_RIGHT   1024

#define KBD_CAPSLOCK_ON  2
#define KBD_NUMLOCK_ON   16
// I don't know the value of this one =>  #define KBD_SCROLLLOCK_ON

#define KBD_SHIFT_ON   1
#define KBD_CTRL_ON  4
#define KBD_ALT_ON   8
#define KBD_WIN_ON   64 //Windows key

//Edit Mokhtar Picking
#define BUFSIZE 512
GLuint selectBuf[BUFSIZE];
static int picking = FALSE;
static int picking_x;
static int picking_y;
static int enable_picking= TRUE; /*!<  flag to enable/disable picking */
//end Mokhtar Picking

static double LIGHT_FACTOR = 5;

/*   Globals (for every platform)  */
p3d_matrix4 Id = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};  /* Global use by all platform (used as extern for FORMmobcam) */
int robotSelectPositionFlag = 0; //current position 0 or goto position 1 ?

/*   Statics (for every platform)  */
static int POLY_TYPE = 0;
static int LIST;

G3D_Window *G3D_WINDOW_CUR = NULL; /* la fenetre courante */
G3D_Window *G3D_WINDOW_LST = NULL; /* liste des fenetres graphiques */
static G3D_Window *G3D_WINDOW_CMC = NULL;
int G3D_MODIF_VIEW  = FALSE; /* flag true durant modif viewing */

//static void get_lookat_vector(G3D_Window *win, p3d_vector4 Vec);
//static void get_pos_cam_matrix(G3D_Window *win, p3d_matrix4 Transf);
//static void recalc_mouse_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);
void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);
//static void recalc_cam_up(G3D_Window *win, p3d_matrix4 transf);

//edit Mokhtar Picking functions
static void startPicking(int cursorX, int cursorY);
static int stopPicking();
static int processHits (GLint hits, GLuint buffer[]);


#ifndef NOFORMS
/*   Defined for UNIX (XForms & GLX)

 repris du gl.c de forms pour pouvoir utiliser des display lists
 identiques sur les fenetres copiees */
#define MAXATTRIB  34

typedef struct {
  XVisualInfo *xvinfo;
  GLXContext context;
  int direct;
  int glconfig[MAXATTRIB];
}
	CSPEC;

#define GLPROP(ob)   ((CSPEC *)(ob->c_vdata))
/*  fin reprise du gl.c */

/*   Statics for UNIX (XForms & GLX) */
static int        G3D_GLCONFIG[30] = { // pas utilise... servirait pour le stencil 
GLX_RGBA, GLX_DEPTH_SIZE, 1,
GLX_RED_SIZE, 1, GLX_GREEN_SIZE, 1, GLX_BLUE_SIZE, 1,
GLX_STENCIL_SIZE,1,
GLX_DOUBLEBUFFER,
  
#ifdef ENABLE_ANTIALIASING
GLX_SAMPLE_BUFFERS_ARB, 1,
GLX_SAMPLES_ARB, 4,
#endif
  
None
};

static int canvas_expose(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud);
static int canvas_viewing(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud);

static void button_done(FL_OBJECT *ob, long data);
static void button_unselect(FL_OBJECT *ob, long data);
static void button_copy(FL_OBJECT *ob, long data);
static void button_view_save(FL_OBJECT *ob, long data);
static void button_view_restore(FL_OBJECT *ob, long data);
static void button_view_fil(FL_OBJECT *ob, long data);
static void button_view_cont(FL_OBJECT *ob, long data);
static void button_view_ghost(FL_OBJECT *ob, long data);
#ifdef DPG
static void button_view_grid(FL_OBJECT *ob, long data);
#endif
static void button_view_bb(FL_OBJECT *ob, long data);
static void button_view_gour(FL_OBJECT *ob, long data);
// static void button_freeze(FL_OBJECT *ob, long data);
static void button_screenshot(FL_OBJECT *ob, long data);
static void button_mobile_camera(FL_OBJECT *ob, long data);
static void button_proj(FL_OBJECT *ob, long data);
static void button_joints(FL_OBJECT *ob, long data);
static void button_light(FL_OBJECT *ob, long data);
static void button_floor(FL_OBJECT *ob, long data);
static void button_tiles(FL_OBJECT *ob, long data);
static void button_walls(FL_OBJECT *ob, long data);
static void button_shadows(FL_OBJECT *ob, long data);
static void button_antialiasing(FL_OBJECT *ob, long data);
static void button_shaders(FL_OBJECT *ob, long data);


static G3D_Window *g3d_copy_win(G3D_Window *win);

/** UNIX Global Functions *************************************************************************/




G3D_Window
*g3d_new_win(const char *name,int w, int h, float size) {
  G3D_Window *win = (G3D_Window *)malloc(sizeof(G3D_Window));

  win->vs = g3d_init_viewer_state(size);

#ifndef QT_GL
  FL_FORM    *form= fl_bgn_form(FL_UP_BOX,w+90,h+20);

  FL_OBJECT  *can = fl_add_glcanvas(FL_NORMAL_CANVAS,10,10,w,h,"GL canvas");

  FL_OBJECT *wcop= fl_add_button(FL_NORMAL_BUTTON,w+20,20,60,20,"Copy");

  FL_OBJECT *vsav= fl_add_button(FL_NORMAL_BUTTON,w+20,40,60,30,"Save View");
  FL_OBJECT *vres= fl_add_button(FL_NORMAL_BUTTON,w+20,70,60,40,"Restore \n View");


  FL_OBJECT *vfil= fl_add_button(FL_NORMAL_BUTTON,w+20,110,60,30,"Poly/Line");
  FL_OBJECT *vcont= fl_add_button(FL_NORMAL_BUTTON,w+20,140,60,30,"Contours");
  FL_OBJECT *vGhost= fl_add_button(FL_NORMAL_BUTTON,w+20,170,60,20,"Ghost");
  FL_OBJECT *vBb= fl_add_button(FL_NORMAL_BUTTON,w+20,190,60,20,"BB");
#ifdef DPG
  FL_OBJECT *vGrid= fl_add_button(FL_NORMAL_BUTTON,w+20,210,60,20,"Grid");
  FL_OBJECT *vgour= fl_add_button(FL_NORMAL_BUTTON,w+20,230,60,20,"Smooth");
#else
  FL_OBJECT *vgour= fl_add_button(FL_NORMAL_BUTTON,w+20,210,60,40,"Smooth");
#endif
  FL_OBJECT *screenshot= fl_add_button(FL_NORMAL_BUTTON,w+20,250,60,30,"Screenshot");


  FL_OBJECT *mcamera= fl_add_button(FL_PUSH_BUTTON,w+20,280,60,40,"Mobile\n Camera");
  FL_OBJECT *proj= fl_add_button(FL_NORMAL_BUTTON,w+20,320,60,40,"Perspective\nOrthographic");

  FL_OBJECT *done= fl_add_button(FL_NORMAL_BUTTON,w+20,360,60,20,"Done");

  FL_OBJECT *unselect= fl_add_button(FL_NORMAL_BUTTON,w+20,380,60,40,"Unselect\n Joint");

  //This frame is not automatically resized after a window resize operation, so
  //it is often nicer without it:
  //fl_add_labelframe(FL_BORDER_FRAME,w+15,510,68,90,"Options");

  FL_OBJECT *displayJoints = fl_add_checkbutton(FL_PUSH_BUTTON,w+20,460,60,20,"Joints");
  FL_OBJECT *oplight= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,480,60,20,"Light");
  fl_set_button(oplight, 1);
  FL_OBJECT *opfloor = fl_add_checkbutton(FL_PUSH_BUTTON,w+20,500,60,20,"Floor");
  FL_OBJECT *optiles = fl_add_checkbutton(FL_PUSH_BUTTON,w+20,520,60,20,"Tiles");
  FL_OBJECT *walls= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,540,60,20,"Walls");
  FL_OBJECT *shadows= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,560,60,20,"Shadows");
  FL_OBJECT *antialiasing= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,580,60,20,"Antialiasing"); 
  FL_OBJECT *shaders= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,600,60,20,"Shaders"); 
  fl_set_button(shaders, 1);
  fl_end_form();

#else
  FL_FORM    *form=NULL;
  FL_FORM    *can=NULL;
  FL_FORM    *mcamera=NULL;
#endif


  /* Les parametres de la fenetre */
  win->form       = (void *)form;
  win->canvas     = (void *)can;

  win->fct_draw   = NULL;
  win->next       = NULL;
  win->fct_mobcam   = NULL;
  win->cam_frame  = &Id;
  win->mcamera_but  = (void *)mcamera;


  sprintf(win->name,"%s",name);
  g3d_set_win_camera(win->vs, .0,.0,.0,2*size, INIT_AZ, INIT_EL,.0,.0,1.0);
  g3d_save_win_camera(win->vs);
  g3d_set_win_bgcolor(win->vs,1.0,1.0,1.0);
  win->next = G3D_WINDOW_LST;
  G3D_WINDOW_LST = win;



  if(ENV.getBool(Env::isCostSpace) && (GroundCostObj != NULL)){
          g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  }
  else
  {
          g3d_set_win_bgcolor(win->vs, 1.0, 1.0, 0.8);
  }
  win->fct_draw2= NULL;
  win->fct_key1= NULL;
  win->fct_key2= NULL;


#ifndef QT_GL
  /* Attributs/Handlers du canvas */
  fl_set_glcanvas_attributes(can,G3D_GLCONFIG);
  fl_set_object_gravity(can,FL_NorthWest,FL_SouthEast);

  fl_add_canvas_handler(can,Expose,canvas_expose,(void *)win);
  fl_add_canvas_handler(can,ButtonPress,canvas_viewing,(void *)win);
  fl_add_canvas_handler(can,ButtonRelease,canvas_viewing,(void *)win);
  fl_add_canvas_handler(can,MotionNotify,canvas_viewing,(void *)win);

  fl_add_canvas_handler(can,KeyPress,canvas_viewing,(void *)win);
  fl_add_canvas_handler(can,KeyRelease,canvas_viewing,(void *)win);

  /* Attributs/Handlers des boutons */
  fl_set_object_gravity(wcop,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(vsav,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(vres,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(vfil,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(vcont,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(vGhost,FL_NorthEast,FL_NorthEast);
#ifdef DPG
  fl_set_object_gravity(vGrid,FL_NorthEast,FL_NorthEast);
#endif
  fl_set_object_gravity(vBb,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(vgour,FL_NorthEast,FL_NorthEast);
//   fl_set_object_gravity(wfree,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(screenshot,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(done,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(unselect,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(mcamera,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(proj,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(displayJoints,FL_NorthEast,FL_NorthEast);

  fl_set_object_gravity(oplight,FL_NorthEast,FL_NorthEast);
  fl_set_choice(oplight, 0);
  fl_set_object_gravity(opfloor,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(optiles,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(walls,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(shadows,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(antialiasing,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(shaders,FL_NorthEast,FL_NorthEast);

  fl_set_object_callback(done,button_done,(long)win);
  fl_set_object_callback(unselect,button_unselect,(long)win);
  fl_set_object_callback(wcop,button_copy,(long)win);
  fl_set_object_callback(vsav,button_view_save,(long)win);
  fl_set_object_callback(vres,button_view_restore,(long)win);
  fl_set_object_callback(vfil,button_view_fil,(long)win);
  fl_set_object_callback(vcont,button_view_cont,(long)win);
  fl_set_object_callback(vGhost,button_view_ghost,(long)win);
#ifdef DPG
  fl_set_object_callback(vGrid,button_view_grid,(long)win);
#endif
  fl_set_object_callback(vBb,button_view_bb,(long)win);
  fl_set_object_callback(vgour,button_view_gour,(long)win);
//   fl_set_object_callback(wfree,button_freeze,(long)win);
  fl_set_object_callback(screenshot,button_screenshot,(long)win);
  fl_set_object_callback(mcamera,button_mobile_camera,(long)win);
  fl_set_object_callback(proj,button_proj,(long)win);
  fl_set_object_callback(displayJoints,button_joints,(long)win);

  fl_set_object_callback(oplight,button_light,(long)win);
  fl_set_object_callback(opfloor, button_floor,(long)win);
  fl_set_object_callback(optiles,button_tiles,(long)win);
  fl_set_object_callback(walls,button_walls,(long)win);
  fl_set_object_callback(shadows,button_shadows,(long)win);
  fl_set_object_callback(antialiasing,button_antialiasing,(long)win);
  fl_set_object_callback(shaders,button_shaders,(long)win);

  /* fl_show_form(form,FL_PLACE_FREE,FL_FULLLBORDER,name);*/
  fl_show_form(form,FL_PLACE_MOUSE|FL_FREE_SIZE,FL_FULLBORDER,name);
  /* Pour ne recevoir les events MOTION_NOTIFY que quand le bouton est down */
  fl_remove_selected_xevent(FL_ObjWin(can),PointerMotionMask|PointerMotionHintMask);
#endif


  //Remplissage des matrices de projection sur les plans dans la direction de la lumière.
  //Si la position de la lumière est modifiée, il faudra mettre à jour les matrices.
  g3d_build_shadow_matrices(win->vs);


  G3D_WINDOW_CUR = win;
  return(win);
}

void
g3d_del_win(G3D_Window *win)
/* used in static function for forms
 used in FORMmain */
{
  FL_FORM *form = ((FL_FORM *)win->form);
  FL_OBJECT *can = ((FL_OBJECT *)win->canvas);
  G3D_Window *w = G3D_WINDOW_LST;

  if (!win->form) {
    PrintInfo(("g3d_del_win: wrong arg\n"));
    return;
  }
  fl_hide_form(form);
  fl_free_object(can);
  fl_free_form(form);
  if(w == win) G3D_WINDOW_LST = win->next;
	else  while(w) {if(w->next == win) {w->next = win->next; break;}w=w->next;}
  win->form = NULL;

  g3d_free_viewer_state(win->vs);

  free(win);
}


int
g3d_win_id(G3D_Window *win)
/* used in FORMrobot  */
{
  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas);
  return(FL_ObjWin(ob));
}


void
g3d_refresh_allwin_active(void)
/* used in FORMmain  */
{

#ifdef QT_LIBRARY
    // Problem with the OpenGL code
    return;
#endif

  G3D_Window *w = G3D_WINDOW_LST;
  FL_OBJECT  *ob;
  int winw,winh;

  while(w) {
    
    if(w->vs.ACTIVE == 1) {
      
      w->vs.list = -1;
      
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      ob = ((FL_OBJECT *)w->canvas);
      fl_get_winsize(FL_ObjWin(ob),&winw,&winh);
      
      canvas_expose(ob, NULL, winw, winh, NULL, w);
      
    }
    w = w->next;
  }
}

void g3d_event_win(G3D_Window *g3dwin, int event, int xpos, int ypos, void* data) {
  /* KINEO-DEV: used in win32 part, here just for prototype generation. */
}


/** UNIX Static Functions *************************************************************************/


static G3D_Window *
g3d_copy_win(G3D_Window *win) {
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas);
  G3D_Window *new_win;
  char       str[256];
  int        w,h;
  int        i,j;
  p3d_vector4 Xc,Xw;
  fl_get_winsize(FL_ObjWin(ob),&w,&h);
  sprintf(str,"%s->copy",win->name);
  new_win = g3d_new_win(str,w,h,win->vs.size);

  {
    /* pour associer un context identique au canvas de la fenetre */
    FL_OBJECT   *newob = ((FL_OBJECT *)new_win->canvas);
    XVisualInfo *vi = glXChooseVisual(fl_display,fl_screen,GLPROP(newob)->glconfig);
    glXDestroyContext(fl_display,GLPROP(newob)->context);
    GLPROP(newob)->context = glXCreateContext(fl_display,vi,
                                              fl_get_glcanvas_context(ob),
                                              GLPROP(newob)->direct);
  }

  new_win->vs.FILAIRE = win->vs.FILAIRE;
  new_win->vs.CONTOUR = win->vs.CONTOUR;
  new_win->vs.GOURAUD = win->vs.GOURAUD;
  for( i = 0; i < 6; i++)
    for(j = 0; j < 4; j++) {
      new_win->vs.frustum[i][j] = win->vs.frustum[i][j];
    }
  g3d_set_win_drawer(new_win,win->fct_draw);
  g3d_set_win_bgcolor(new_win->vs, win->vs.bg[0],win->vs.bg[1],win->vs.bg[2]);
  g3d_set_win_fct_mobcam(new_win,win->fct_mobcam);
  if(win->cam_frame == &Id) {
    g3d_set_win_camera(new_win->vs, win->vs.x,win->vs.y,win->vs.z,win->vs.zo,win->vs.az,win->vs.el,win->vs.up[0],win->vs.up[1],win->vs.up[2]);
  } else {
    calc_cam_param(win,Xc,Xw);
    recalc_mouse_param(new_win->vs,Xc,Xw);
    for(i=0;i< 4;i++) new_win->vs.up[i] = win->vs.up[i];
    recalc_cam_up(new_win->vs,*win->cam_frame);
    new_win->vs.zo = win->vs.zo;
  }
  return(new_win);
}

double g3d_get_light_factor(void) {
  return LIGHT_FACTOR;
}

void g3d_set_light_factor(double factor) {
  LIGHT_FACTOR = factor;
}

void g3d_draw_win(G3D_Window *win) {
  p3d_vector4 Xc,Xw;
  p3d_vector4 up;

  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas);

  G3D_WINDOW_CUR = win;

  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));

  calc_cam_param(win,Xc,Xw);

  p3d_matvec4Mult(*win->cam_frame,win->vs.up,up);

  glPushMatrix();
  gluLookAt(Xc[0],Xc[1],Xc[2],Xw[0],Xw[1],Xw[2],up[0],up[1],up[2]);

  win->vs.cameraPosition[0]= Xc[0];
  win->vs.cameraPosition[1]= Xc[1];
  win->vs.cameraPosition[2]= Xc[2];  
	//   if(G3D_MODIF_VIEW) {
	//     glPushMatrix();
        //     glTranslatef(win->vs.x,win->vs.y,win->vs.z);
	//     g3d_draw_frame();
	//     glPopMatrix();
	//   }
  
  if(win->fct_draw) (*win->fct_draw)();


  glPopMatrix();

//   if (win->win_perspective)//G3D_REFRESH_PERSPECTIVE)
  glXSwapBuffers(fl_display,fl_get_canvas_id(ob));
}

void g3d_draw_win_back_buffer(G3D_Window *win) {
  p3d_vector4 Xc,Xw;
  p3d_vector4 up;
  
  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas);
  
  G3D_WINDOW_CUR = win;
  
  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));
  
  calc_cam_param(win,Xc,Xw);
  
  p3d_matvec4Mult(*win->cam_frame,win->vs.up,up);
  
  glPushMatrix();
  gluLookAt(Xc[0],Xc[1],Xc[2],Xw[0],Xw[1],Xw[2],up[0],up[1],up[2]);
  
  win->vs.cameraPosition[0]= Xc[0];
  win->vs.cameraPosition[1]= Xc[1];
  win->vs.cameraPosition[2]= Xc[2];  
	//   if(G3D_MODIF_VIEW) {
	//     glPushMatrix();
  //     glTranslatef(win->vs.x,win->vs.y,win->vs.z);
	//     g3d_draw_frame();
	//     glPopMatrix();
	//   }
  
  if(win->fct_draw) (*win->fct_draw)();
  
  
  glPopMatrix();
  
  //  if (win->win_perspective)//G3D_REFRESH_PERSPECTIVE)
  //  glXSwapBuffers(fl_display,fl_get_canvas_id(ob));
}


static int
canvas_expose(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud) {
  G3D_Window *g3dwin = (G3D_Window *)ud;

  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));

  glViewport(0,0,(GLint)w,(GLint)h);

  g3d_set_projection_matrix(g3dwin->vs.projection_mode);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  g3d_init_OpenGL();
  
  if(g3dwin->vs.GOURAUD) {
    glShadeModel(GL_SMOOTH);
  } else {
    glShadeModel(GL_FLAT);
  }

  glXWaitX(); /*** jean-gerard ***/
  g3d_draw_win(g3dwin);
  glXWaitGL();

  return(TRUE);
}


extern int G3D_SELECTED_JOINT;
extern int G3D_SELECTED_ROBOT;
extern int G3D_MOUSE_ROTATION;

static void g3d_pointViewportProjection(G3D_Window *win, p3d_vector3 jntPoint, double *x, double *y, double *z, int unProject){
  GLdouble modelview_matrix[16], projection_matrix[16];
  GLint viewport[4];
  GLdouble winX, winY, winZ;
  p3d_vector4 Xc, Xw;
  p3d_vector4 up;

  calc_cam_param(win, Xc, Xw);
  p3d_matvec4Mult(*win->cam_frame, win->vs.up, up);
  glPushMatrix();
  gluLookAt(Xc[0], Xc[1], Xc[2], Xw[0], Xw[1], Xw[2], up[0], up[1], up[2]);
  glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix);
  glPopMatrix();
  glGetIntegerv(GL_VIEWPORT, viewport);
  if (!unProject){
    gluProject(jntPoint[0],  jntPoint[1],  jntPoint[2], modelview_matrix,  projection_matrix, viewport, &winX, &winY, &winZ);
  }else{
    gluUnProject(jntPoint[0],  jntPoint[1],  jntPoint[2], modelview_matrix,  projection_matrix, viewport, &winX, &winY, &winZ);
  }
  *x = winX;
  *y = viewport[3]-winY;
  *z = winZ;
}

static void g3d_moveBodyWithMouse(G3D_Window *g3dwin, int *i0, int *j0, int i, int j){
  if(enable_picking==FALSE)
  {  return;  }


  double winX = 0, winY = 0, winZ = 0, nWinX = 0, nWinY = 0, nWinZ = 0, jntValue = 0;
  p3d_rob* robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt* jnt = robot->joints[G3D_SELECTED_JOINT];
  p3d_vector3 jntPoint;
  p3d_vector4 norm;



  //orientation
  switch(G3D_MOUSE_ROTATION-1){
    case 0:{
      if(jnt->type == P3D_KNEE){
        norm[0] = 0;
        norm[1] = -1;
        norm[2] = 0;
        norm[3] = 1;
      }else if(jnt->type == P3D_FREEFLYER || jnt->type == P3D_PLAN){
        norm[0] = 1;
        norm[1] = 0;
        norm[2] = 0;
        norm[3] = 1;
      }else{
        norm[0] = 0;
        norm[1] = 0;
        norm[2] = 1;
        norm[3] = 1;
      }
      break;
		}
    case 1:{
      if(jnt->type == P3D_KNEE){
        norm[0] = 1;
        norm[1] = 0;
        norm[2] = 0;
        norm[3] = 1;
      }else if(jnt->type == P3D_FREEFLYER || jnt->type == P3D_PLAN){
        norm[0] = 0;
        norm[1] = 1;
        norm[2] = 0;
        norm[3] = 1;
      }else{
        norm[0] = 0;
        norm[1] = 0;
        norm[2] = 1;
        norm[3] = 1;
			}
      break;
    }
    case 3:{
      if(jnt->type == P3D_FREEFLYER){
        norm[0] = 0;
        norm[1] = 0;
        norm[2] = 1;
        norm[3] = 1;
      }else{
        norm[0] = 0;
        norm[1] = -1;
        norm[2] = 0;
        norm[3] = 1;
      }
      break;
    }
    case 4:{
			norm[0] = 1;
			norm[1] = 0;
			norm[2] = 0;
			norm[3] = 1;
      break;
    }
    default:{
			norm[0] = 0;
			norm[1] = 0;
			norm[2] = 1;
			norm[3] = 1;
      break;
    }
  }
  //projection d'un point de l'axe
  p3d_vector4 result;
  p3d_matvec4Mult(jnt->abs_pos,norm,result);
  g3d_pointViewportProjection(g3dwin, result, &nWinX, &nWinY, &nWinZ, 0);


  //projection du centre
  p3d_jnt_get_cur_vect_point(jnt, jntPoint);
  g3d_pointViewportProjection(g3dwin, jntPoint, &winX, &winY, &winZ, 0);
  int sign = nWinZ < winZ ? -1 : 1;
  double theta = 0;
  double size = 0;
  if(jnt->type == P3D_ROTATE || jnt->type == P3D_KNEE || (jnt->type == P3D_PLAN && G3D_MOUSE_ROTATION-1 == 2) || (jnt->type == P3D_FREEFLYER && G3D_MOUSE_ROTATION-1 > 2)){
    theta = atan2 (j+*j0-winY, i+*i0-winX) - atan2 (*j0-winY, *i0-winX);
    jntValue = p3d_jnt_get_dof(jnt, G3D_MOUSE_ROTATION - 1) + sign*theta;
  }else{
    p3d_vector3 startPosition = {0,0,0};
    g3d_pointViewportProjection(g3dwin, startPosition, &winX, &winY, &winZ, 0);
    startPosition[0] = winX;
    startPosition[1] = winY;
    startPosition[2] = winZ;
    p3d_vector3 endPosition = {0,0,0};
    endPosition[G3D_MOUSE_ROTATION-1] = 1;
    g3d_pointViewportProjection(g3dwin, endPosition, &winX, &winY, &winZ, 0);
    endPosition[0] = winX;
    endPosition[1] = winY;
    endPosition[2] = winZ;
    p3d_vector3 screen_direction = {endPosition[0] - startPosition[0], endPosition[1] - startPosition[1],endPosition[2] - startPosition[2]};
    double normScrenn_direction = sqrt(SQR(screen_direction[0]) + SQR(screen_direction[1]) + SQR(screen_direction[2]));
    screen_direction[0] = screen_direction[0]/normScrenn_direction;
    screen_direction[1] = screen_direction[1]/normScrenn_direction;
    screen_direction[2] = screen_direction[2]/normScrenn_direction;
    p3d_vector3 mouseVector = {i, j, 0};
    double dotMouseScreen = p3d_vectDotProd(mouseVector, screen_direction);
    screen_direction[0] = screen_direction[0]*dotMouseScreen;
    screen_direction[1] = screen_direction[1]*dotMouseScreen;
    screen_direction[2] = screen_direction[2]*dotMouseScreen;
    endPosition[0] = startPosition[0] + screen_direction[0];
    endPosition[1] = startPosition[1] + screen_direction[1];
    endPosition[2] = startPosition[2] + screen_direction[2];
    g3d_pointViewportProjection(g3dwin, startPosition, &winX, &winY, &winZ, 1);
    g3d_pointViewportProjection(g3dwin, endPosition, &nWinX, &nWinY, &nWinZ, 1);
    switch(G3D_MOUSE_ROTATION-1){
			case 0:{
				if(jnt->type == P3D_FREEFLYER || jnt->type == P3D_PLAN){
					size = nWinX - winX;
				}
				break;
			}
			case 1:{
				if(jnt->type == P3D_FREEFLYER || jnt->type == P3D_PLAN){
					size = winY - nWinY;
				}
				break;
			}
			case 2:{
				if(jnt->type == P3D_FREEFLYER){
					size =  winZ - nWinZ;
				}
				break;
			}
    }
    jntValue = p3d_jnt_get_dof(jnt, G3D_MOUSE_ROTATION - 1) + size;
  }
  *i0 = i+*i0;
  *j0 = j+*j0;
  double vmin = 0, vmax = 0;
  p3d_get_robot_dof_bounds(robot, jnt->index_dof + G3D_MOUSE_ROTATION - 1, &vmin, &vmax);
  jntValue = jntValue < vmin ? vmin : jntValue;
  jntValue = jntValue > vmax ? vmax : jntValue;
  double oldJntValue = p3d_jnt_get_dof(jnt, G3D_MOUSE_ROTATION - 1);
  p3d_jnt_set_dof(jnt, G3D_MOUSE_ROTATION - 1, jntValue);
	//   p3d_update_this_robot_pos_with_partial_reshoot(robot);
  int I_can = p3d_update_this_robot_pos_multisol(robot, NULL, 0, NULL);
  if (robot->cntrt_manager->cntrts != NULL && !I_can) {
    p3d_jnt_set_dof(jnt, G3D_MOUSE_ROTATION - 1, oldJntValue);
    p3d_update_this_robot_pos_multisol(robot, NULL, 0, NULL);
  }
  configPt conf = p3d_get_robot_config(robot);
  int ncol = 0 ;
  if(g3d_get_KCD_CHOICE_IS_ACTIVE()) {
    if(G3D_ACTIVE_CC)
		{ ncol = p3d_col_test_choice(); }
  } else {
    if(G3D_ACTIVE_CC)
		{ ncol = p3d_col_test_all(); }
  }
  g3d_set_draw_coll(ncol);
  /* update the field current position or goal position of the
	 current robot depending on field GOTO_OBJ */
  extern int robotSelectPositionFlag;
  if(robotSelectPositionFlag == 0){
    p3d_copy_config_into(robot, conf, &(robot->ROBOT_POS));
  }
  else{
    p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  }
  FORMrobot_update(robot->num);
}
//ATTENTION
#ifndef HRI_PLANNER
static int
canvas_viewing(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud) {
  G3D_Window   *g3dwin = (G3D_Window *)ud;
  unsigned int key, strippedKey;
  static int   i0,j0;
  static double x,y,z,zo,el,az;
  int          i,j,nodeNb,draw = 0;
  double       x_aux,y_aux,az_aux,rotinc,incinc,zo_inc;
  p3d_list_node *nodes;
  static int shift_key_pressed= 0;
  double translation_step= 10*g3dwin->vs.size/w;
  double rotation_step = 0.1;
  double zoom_step = 1;

  if(!g3dwin->vs.eventsEnabled) {
    return TRUE;
  }

  G3D_MODIF_VIEW = TRUE;

  switch(xev->type) {
		case KeyRelease:
      key = XKeycodeToKeysym(fl_display,xev->xkey.keycode,0);
      if(key==XK_Shift_L || key==XK_Shift_R)
				shift_key_pressed= 0;
			break;
		case KeyPress:
      key = XKeycodeToKeysym(fl_display,xev->xkey.keycode,0);
      switch(key)
		{
        case XK_space: //reset the joint selection
		 G3D_SELECTED_JOINT= -999;
        break;
	case XK_Shift_L: case XK_Shift_R:
		 shift_key_pressed= 1;
        break;
	case XK_e:
		 if(shift_key_pressed)
                    g3d_move_win_camera_forward( g3dwin->vs, 0.1*translation_step);
		 else
                    g3d_move_win_camera_forward( g3dwin->vs, translation_step);
        break;
	case XK_d:
		 if(shift_key_pressed)
                     g3d_move_win_camera_forward( g3dwin->vs, -0.1*translation_step);
		 else
                     g3d_move_win_camera_forward( g3dwin->vs, -translation_step);
        break;
	 case XK_s:
		 if(shift_key_pressed)
                      g3d_move_win_camera_sideways( g3dwin->vs, 0.1*translation_step);
		 else
                      g3d_move_win_camera_sideways( g3dwin->vs, translation_step);
        break;
	 case XK_f:
		      if(shift_key_pressed)
                         g3d_move_win_camera_sideways( g3dwin->vs, -0.1*translation_step);
		      else
                         g3d_move_win_camera_sideways( g3dwin->vs, -translation_step);
        break;
	 case XK_t:
		      if(shift_key_pressed)
                         g3dwin->vs.z+= 0.1*translation_step;
		      else
                          g3dwin->vs.z+= translation_step;
        break;
			case XK_g:
				if(shift_key_pressed)
                                        g3dwin->vs.z-= 0.1*translation_step;
				else
                                        g3dwin->vs.z-= translation_step;
        break;
	case XK_r:
		if(shift_key_pressed)
                    g3d_rotate_win_camera_rz( g3dwin->vs, -0.1*rotation_step);
		else
                    g3d_rotate_win_camera_rz( g3dwin->vs, -rotation_step);
        break;
	case XK_w:
		if(shift_key_pressed)
                     g3d_rotate_win_camera_rz( g3dwin->vs, 0.1*rotation_step);
		else
                     g3d_rotate_win_camera_rz( g3dwin->vs, rotation_step);
        break;
	case XK_v:
		if(shift_key_pressed)
                      g3d_zoom_win_camera( g3dwin->vs, 0.1*zoom_step);
		else
                     g3d_zoom_win_camera( g3dwin->vs, zoom_step);
        break;
	case XK_c:
		if(shift_key_pressed)
                     g3d_zoom_win_camera( g3dwin->vs, -0.1*zoom_step);
		else
                     g3d_zoom_win_camera( g3dwin->vs, -zoom_step);
        break;
	case XK_q:
		if(g3dwin->fct_key1!=NULL)
			g3dwin->fct_key1();
	break;
	case XK_a:
		if(g3dwin->fct_key2!=NULL)
			g3dwin->fct_key2();
	break;

        //déplacement de la source de lumière selon les trois axes:
	case XK_i:
		if(shift_key_pressed)
                 g3dwin->vs.lightPosition[0]+= 0.1*translation_step;
		else
                  g3dwin->vs.lightPosition[0]+= translation_step;
                g3d_build_shadow_matrices(g3dwin->vs);
        break;
	case XK_k:
		if(shift_key_pressed)
                  g3dwin->vs.lightPosition[0]-= 0.1*translation_step;
		else
                  g3dwin->vs.lightPosition[0]-= translation_step;
                g3d_build_shadow_matrices(g3dwin->vs);
        break;
	case XK_j:
		if(shift_key_pressed)
                   g3dwin->vs.lightPosition[1]+= 0.1*translation_step;
		else
                    g3dwin->vs.lightPosition[1]+= translation_step;
                g3d_build_shadow_matrices(g3dwin->vs);
        break;
	case XK_l:
		if(shift_key_pressed)
                    g3dwin->vs.lightPosition[1]-= 0.1*translation_step;
		else
                   g3dwin->vs.lightPosition[1]-= translation_step;
                g3d_build_shadow_matrices(g3dwin->vs);
        break;
	case XK_p:
		if(shift_key_pressed)
                  g3dwin->vs.lightPosition[2]+= 0.1*translation_step;
		else
                  g3dwin->vs.lightPosition[2]+= translation_step;
                g3d_build_shadow_matrices(g3dwin->vs);
        break;
	case XK_m:
		if(shift_key_pressed)
                  g3dwin->vs.lightPosition[2]-= 0.1*translation_step;
		else
                  g3dwin->vs.lightPosition[2]-= translation_step;
                g3d_build_shadow_matrices(g3dwin->vs);
        break;
		}
      g3d_refresh_allwin_active();
			break;


    case MotionNotify:
      fl_get_win_mouse(win,&i,&j,&key);
      if ( picking ) {
				if ( picking_x < i - 2 ||
						picking_x > i + 2 ||
						picking_y < j - 2 ||
						picking_y > j + 2)
					picking = FALSE;
      }
      i -= i0;
      j -= j0;
      //Extract the information related to the mouse buttons from the "key"
      // In this way we will not be bothered by the Alt, Crtl, Shift modifiers
      strippedKey = key & (MOUSE_BTN_LEFT + MOUSE_BTN_CENTER + MOUSE_BTN_RIGHT);
      //If information is needed as to wether the Ctrl, Alt or Shift modifiers are used,
      //   one can use KBD_SHIFT_ON, KBD_CTRL_ON , KBD_ALT_ON, or KBD_WIN_ON like this
      //   ex:  if ( key & KBD_SHIFT_ON ) { //do some action }
      //         printf("mouse strippedkey = %d\n", strippedKey);

      //Take the action decision based on the stripped key.
      switch(strippedKey) {
        case MOUSE_BTN_LEFT: /* zoom */
          if (G3D_MOUSE_ROTATION != 0) {
            g3d_moveBodyWithMouse(g3dwin, &i0, &j0, i,j);
          } else {
                g3dwin->vs.zo = (zo * (double)i)/w + zo;
                if(g3dwin->vs.zo < 0.1) g3dwin->vs.zo = 0.1;
                if(g3dwin->vs.projection_mode==G3D_ORTHOGRAPHIC)
                { g3d_refresh_allwin_active(); return 0; }
          }
          break;
        case MOUSE_BTN_CENTER: /* angle */
          g3dwin->vs.az = (-2*GAIN_AZ * i)/w + az;
          if(g3dwin->vs.az < .0) g3dwin->vs.az = 2*M_PI + g3dwin->vs.az;
          if(g3dwin->vs.az > 2*M_PI) g3dwin->vs.az = g3dwin->vs.az - 2*M_PI;

          g3dwin->vs.el = (GAIN_EL * j)/w + el;
          if(g3dwin->vs.el < -M_PI/2.0) g3dwin->vs.el = -M_PI/2.0;
          if(g3dwin->vs.el > M_PI/2.0) g3dwin->vs.el = M_PI/2.0;
          break;
        case MOUSE_BTN_RIGHT:  /* origin esf*/
          rotinc = (-GAIN_AZ * i)/w;
          incinc = (-GAIN_EL * j)/w;
          /* rotation effects */
          g3dwin->vs.x = x + 2.0*zo*cos(el)*sin(rotinc/2.0)*sin(az+rotinc/2.0);
          g3dwin->vs.y = y - 2.0*zo*cos(el)*sin(rotinc/2.0)*cos(az+rotinc/2.0);
          g3dwin->vs.az = az + rotinc;

          if(g3dwin->vs.az < .0)
            g3dwin->vs.az = 2*M_PI;
          if(g3dwin->vs.az > 2*M_PI)
            g3dwin->vs.az = .0;
          x_aux=g3dwin->vs.x;
          y_aux=g3dwin->vs.y;
          az_aux=g3dwin->vs.az;
          /* incline effects */
          g3dwin->vs.z = z - 2.0 *zo*sin(incinc/2.0)*cos(el+incinc/2.0);
          g3dwin->vs.x = x_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*cos(az_aux);
          g3dwin->vs.y = y_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*sin(az_aux);
          g3dwin->vs.el = el + incinc;
          if(g3dwin->vs.el < -M_PI/2)
            g3dwin->vs.el = -M_PI/2;
          if(g3dwin->vs.el > M_PI/2)
            g3dwin->vs.el = M_PI/2;
          break;
          /* deplacement du  point central de l'image */
          /* - bouton gauche et central -> x          */
          /* - bouton droit et central  -> y          */
          /* - bouton gauche et droit   -> z          */
        case MOUSE_BTN_LEFT + MOUSE_BTN_CENTER: /* origin x */
          g3dwin->vs.x = x + j*g3dwin->vs.size/w;
          /*       g3dwin->vs.x = x + i*g3dwin->vs.size/h; */
          break;
        case MOUSE_BTN_CENTER + MOUSE_BTN_RIGHT: /* origin y */
          g3dwin->vs.y = y + j*g3dwin->vs.size/w;
          /*       g3dwin->vs.y = y + i*g3dwin->vs.size/h; */
          break;
        case MOUSE_BTN_LEFT + MOUSE_BTN_RIGHT: /* origin z */
          g3dwin->vs.z = z + j*g3dwin->vs.size/w;
          /*       g3dwin->vs.z = z + i*g3dwin->vs.size/h; */
          break;
        case MOUSE_BTN_LEFT + MOUSE_BTN_CENTER + MOUSE_BTN_RIGHT: /* reset up */
          g3dwin->vs.up[0]=.0;
          g3dwin->vs.up[1]=.0;
          g3dwin->vs.up[2]=1.0;
          break;
      }
      glXWaitX(); /*** jean-Gerard ***/
      g3d_draw_win(g3dwin);
      glXWaitGL();/*** jean-gerard ***/
      break;

		case ButtonPress:
      fl_get_win_mouse(win,&i0,&j0,&key);
      x=g3dwin->vs.x;
      y=g3dwin->vs.y;
      z=g3dwin->vs.z;
      zo=g3dwin->vs.zo;
      el=g3dwin->vs.el;
      az=g3dwin->vs.az;

      //Mokhtar scroll zoom
      strippedKey = key & (KBD_SHIFT_ON);
      if (strippedKey == KBD_SHIFT_ON){//if shift key is pressed
                                zo_inc = floor(g3dwin->vs.size);//slow zoom
      }else{
                                zo_inc = floor(g3dwin->vs.size);//big zoom
      }
      switch (xev->xbutton.button) {
				case 1:{//click left button
          // activate picking, picking will be cancelled if the mouse is used to change the camera view
          // (and not to pick a vertex)
#ifdef P3D_PLANNER
          if (!XYZ_GRAPH || !ENV.getBool(Env::drawGraph)){
            p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
            startPicking(i0, j0);
            p3d_drawRobotMoveMeshs();
            g3d_draw_allwin_active();
            int bodyNb = stopPicking();
            if (bodyNb < 0 && bodyNb != -999) {
              G3D_MOUSE_ROTATION = -bodyNb;
            }else if (bodyNb >= 0){
              for (int k = 0; k < robotPt->njoints + 1; k++) {
                if (robotPt->joints[k]->o && robotPt->joints[k]->o->o_id_in_env == bodyNb) {
                  if(robotPt->joints[k]->type != P3D_FIXED && robotPt->joints[k]->type != P3D_BASE){
                    G3D_SELECTED_JOINT = robotPt->joints[k]->num;
                    G3D_SELECTED_ROBOT = robotPt->num;
                  }
                  break;
                }
              }
            }
          }
#endif
         // picking = TRUE;
          picking_x = i0;
          picking_y = j0;
          break;
				}
				case 4:{//forward scroll (forward zoom)
          g3dwin->vs.zo += zo_inc;
          draw = 1;
          if(g3dwin->vs.zo < 0) g3dwin->vs.zo = 0;
          break;
				}
				case 5:{//back scroll (back zoom)
          g3dwin->vs.zo -= zo_inc;
          draw = 1;
          if(g3dwin->vs.zo < 0) g3dwin->vs.zo = 0;
          break;
				}
      }
      //update the window
      if (draw){
				glXWaitX(); /*** jean-Gerard ***/
				g3d_draw_win(g3dwin);
				glXWaitGL();/*** jean-gerard ***/
      }
      break;
		case ButtonRelease:
			switch(xev->xbutton.button) {
				case 1:{
					//If currently picking, Give the robot the configuration clicked on the graph
					if ( picking ) {
						fl_get_win_mouse(win,&i0,&j0,&key);
						picking = FALSE;
						if (XYZ_GRAPH && ENV.getBool(Env::drawGraph) && enable_picking==TRUE){
							startPicking(i0,j0);
							//               g3d_draw_graph();
							g3d_draw_allwin_active();
							nodeNb = stopPicking();
							nodes = XYZ_GRAPH->nodes;
							for(;(nodes->next) && (nodes->N->num != nodeNb); nodes = nodes->next);
							//           p3d_set_robot_config(XYZ_ROBOT,nodes->N->q);
              p3d_set_and_update_this_robot_conf_without_cntrt(XYZ_ROBOT,nodes->N->q);
							if(robotSelectPositionFlag){
								p3d_copy_config_into(XYZ_ROBOT, nodes->N->q, &(XYZ_ROBOT->ROBOT_GOTO));
							}else{
								p3d_copy_config_into(XYZ_ROBOT, nodes->N->q, &(XYZ_ROBOT->ROBOT_POS));
							}
							int ncol=0;
							/* collision checking */
							if(g3d_get_KCD_CHOICE_IS_ACTIVE()) {
                if (G3D_ACTIVE_CC) {
                  ncol = p3d_col_test_choice();
                }
							} else {
                if (G3D_ACTIVE_CC) {
                  ncol = p3d_col_test_all();
                }
							}
							g3d_set_draw_coll(ncol);
							g3d_draw_allwin_active();
							g3d_draw_allwin_active();
							p3d_print_node(XYZ_GRAPH, nodes->N);
						}
					}
          if (G3D_MOUSE_ROTATION != 0 || G3D_SELECTED_JOINT != -999){//pick body
            G3D_MOUSE_ROTATION = 0;
            g3d_draw_allwin_active();
          }
				}
					break;
			}
  }

  G3D_MODIF_VIEW = FALSE;
  return(TRUE);
}
#else

static int
canvas_viewing(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud)
{
  G3D_Window   *g3dwin = (G3D_Window *)ud;
  unsigned int key;
	static int   i0,j0;//,idr=-1;
  static double x,y,z,zo,el,az;
  int          i,j;
  double       x_aux,y_aux,az_aux,rotinc,incinc;

  if(!g3dwin->vs.eventsEnabled) {
    return TRUE;
  }

  G3D_MODIF_VIEW = TRUE;

  switch(xev->type) {
      case KeyPress:
           key = XKeycodeToKeysym(fl_display,xev->xkey.keycode,0);
           switch(key)
	   {
	     case XK_q:
               if(g3dwin->fct_key1!=NULL)
		  g3dwin->fct_key1();
	     break;
	     case XK_a:
                if(g3dwin->fct_key2!=NULL)
		   g3dwin->fct_key2();
	     break;
          }
        break;
		case MotionNotify:
			fl_get_win_mouse(win,&i,&j,&key);
//			printf("i = %f\n",(double)i);
//			printf("j = %f\n",(double)j);
//			printf("%d\n",key);
			i -= i0; j -= j0;
			switch(key) {
				case 260: /* Select Object and Displace in X */
                                        //	move_robot(idr,0,j,g3dwin->vs.size/h);
					break;
				case 516: /* Select Object and Displace in Y */
                                        //	move_robot(idr,1,i,g3dwin->vs.size/w);
					break;
				case 1028:/* Select Object and Displace in Z */
                                        //	move_robot(idr,2,j,g3dwin->vs.size/h);
					break;
				case 256: /* zoom */
				case 272:
				case 258:
				case 274:
				case 8464:
				case 16656:
                                        g3dwin->vs.zo = (zo * i)/w + zo;
                                        if(g3dwin->vs.zo < .0) g3dwin->vs.zo = .0;
					break;
				case 512: /* angle */
				case 514:
				case 528:
				case 530:
				case 8720:
				case 16912:
				case 8704:
                                        g3dwin->vs.az = (-2*GAIN_AZ * i)/w + az;
                                        if(g3dwin->vs.az < .0) g3dwin->vs.az = 2*M_PI;
                                        if(g3dwin->vs.az > 2*M_PI) g3dwin->vs.az =.0;
                                        g3dwin->vs.el = (GAIN_EL * j)/w + el;
                                        if(g3dwin->vs.el < -M_PI/2.0) g3dwin->vs.el = -M_PI/2.0;
                                        if(g3dwin->vs.el > M_PI/2.0) g3dwin->vs.el = M_PI/2.0;
					break;
				case 1024:  /* origin esf*/
				case 1026:
				case 1040:
				case 1042:
				case 9232:
				case 17424:
					rotinc = (-GAIN_AZ * i)/w;
					incinc = (-GAIN_EL * j)/w;
					/* rotation effects */
                                        g3dwin->vs.x = x + 2.0*zo*cos(el)*sin(rotinc/2.0)*sin(az+rotinc/2.0);
                                        g3dwin->vs.y = y - 2.0*zo*cos(el)*sin(rotinc/2.0)*cos(az+rotinc/2.0);
                                        g3dwin->vs.az = az + rotinc;

                                        if(g3dwin->vs.az < .0)
                                                g3dwin->vs.az = 2*M_PI;
                                        if(g3dwin->vs.az > 2*M_PI)
                                                g3dwin->vs.az = .0;
                                        x_aux=g3dwin->vs.x;
                                        y_aux=g3dwin->vs.y;
                                        az_aux=g3dwin->vs.az;
					/* incline effects */
                                        g3dwin->vs.z = z - 2.0 *zo*sin(incinc/2.0)*cos(el+incinc/2.0);
                                        g3dwin->vs.x = x_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*cos(az_aux);
                                        g3dwin->vs.y = y_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*sin(az_aux);
                                        g3dwin->vs.el = el + incinc;
                                        if(g3dwin->vs.el < -M_PI/2)
                                                g3dwin->vs.el = -M_PI/2;
                                        if(g3dwin->vs.el > M_PI/2)
                                                g3dwin->vs.el = M_PI/2;
					break;
					/* deplacement du  point central de l'image */
					/* - bouton gauche et central -> x          */
					/* - bouton droit et central  -> y          */
					/* - bouton gauche et droit   -> z          */
				case 768: /* origin x */
				case 770:
				case 784:
				case 8976:
				case 17168:
                                        g3dwin->vs.x = x + j*g3dwin->vs.size/w;
                                        /*       g3dwin->vs.x = x + i*g3dwin->vs.size/h; */
					break;
				case 1536: /* origin y */
				case 1552:
				case 1538:
				case 9744:
				case 17936:
                                        g3dwin->vs.y = y + j*g3dwin->vs.size/w;
                                        /*       g3dwin->vs.y = y + i*g3dwin->vs.size/h; */
					break;
				case 1280: /* origin z */
				case 1282:
				case 1296:
				case 9488:
				case 17680:
                                        g3dwin->vs.z = z + j*g3dwin->vs.size/w;
                                        /*       g3dwin->vs.z = z + i*g3dwin->vs.size/h; */
					break;
				case 1792: /* reset up */
                                        g3dwin->vs.up[0]=.0;
                                        g3dwin->vs.up[1]=.0;
                                        g3dwin->vs.up[2]=1.0;
					break;
			}
			glXWaitX(); /*** jean-Gerard ***/
			g3d_draw_win(g3dwin);
			glXWaitGL();/*** jean-gerard ***/
			break;

		case ButtonPress:
			fl_get_win_mouse(win,&i0,&j0,&key);
//			printf("-------------------------------------\n");
//			printf("i0 = %f\n",(double)i0);
//			printf("j0 = %f\n",(double)j0);
                        x=g3dwin->vs.x;
                        y=g3dwin->vs.y;
                        z=g3dwin->vs.z;
                        zo=g3dwin->vs.zo;
                        el=g3dwin->vs.el;
                        az=g3dwin->vs.az;
			switch(key) {
				case 260: /* Select Object */
					//	idr=select_robot(g3dwin,i0,j0);
					break;
				case 516: /* Select Object*/
					//		idr=select_robot(g3dwin,i0,j0);
					break;
				case 1028:/* Select Object */
					//			idr=select_robot(g3dwin,i0,j0);
					break;
			}
			//select_robot(g3dwin,i0,j0,0);
			//printf("Key %i\n",key);

			break;

  }
  G3D_MODIF_VIEW = FALSE;
  return(TRUE);
}

#endif


/**
  * Next function are for displaying inside Qt
  */
#ifdef QT_GL

G3D_Window * qt_get_cur_g3d_win()
{
	return G3D_WINDOW_CUR;
}


void qt_calc_cam_param()
{
	p3d_vector4 Xc, Xw;
	p3d_vector4 up;

	//	  std::cout << "G3D_WINDOW_CUR->zo = "<< G3D_WINDOW_CUR->zo << std::endl;

	calc_cam_param(G3D_WINDOW_CUR, Xc, Xw);

	/*std::cout << Xc[0] << " " << Xc[1] << " " << Xc[2] << std::endl;
	 std::cout << Xw[0] << " " << Xw[1] << " " << Xw[2] << std::endl;
	 std::cout << up[0] << " " << up[1] << " " << up[2] << std::endl;*/

	JimXc[0] = Xc[0];
	JimXc[1] = Xc[1];
	JimXc[2] = Xc[2];

	JimXw[0] = Xw[0];
	JimXw[1] = Xw[1];
	JimXw[2] = Xw[2];

	if (G3D_WINDOW_CUR)
	{
		p3d_matvec4Mult(*G3D_WINDOW_CUR->cam_frame, G3D_WINDOW_CUR->vs.up, up);
	}
	else
	{
		up[0] = 0;
		up[1] = 0;
		up[2] = 1;
	}

	Jimup[0] = up[0];
	Jimup[1] = up[1];
	Jimup[2] = up[2];
}


void qt_canvas_viewing(int mouse_press, int button)
{
	G3D_Window *g3dwin = G3D_WINDOW_CUR;
	int w = G3D_WINSIZE_WIDTH;
	int h = G3D_WINSIZE_HEIGHT;
	unsigned int key;

	static int i0, j0;//,idr=-1;
	int i, j;

	static double x, y, z, zo, el, az;
	double x_aux, y_aux, az_aux, rotinc, incinc;

	G3D_MODIF_VIEW = TRUE;

	if (mouse_press)
	{
		qt_get_win_mouse(&i0, &j0);
		/*printf("----------------------------------\n", (double) i0);
		 printf("i0 = %d\n", i0);
		 printf("j0 = %d\n", j0);*/
                x = g3dwin->vs.x;
                y = g3dwin->vs.y;
                z = g3dwin->vs.z;
                zo = g3dwin->vs.zo;
                el = g3dwin->vs.el;
                az = g3dwin->vs.az;
	}
	else
	{
		qt_get_win_mouse(&i, &j);

		if (button == 0)
		{
			key = 256;
		}
		else if (button == 1)
		{
			key = 512;
		}
		else if (button == 2)
		{
			key = 1024;
		}

		/*printf("i = %d\n", i);
		 printf("j = %d\n", j);*/
		//			printf("%d\n",key);
		i -= i0;
		j -= j0;
		switch (key)
		{
		case 260: /* Select Object and Displace in X */
                        //	move_robot(idr,0,j,g3dwin->vs.size/h);
			break;
		case 516: /* Select Object and Displace in Y */
                        //	move_robot(idr,1,i,g3dwin->vs.size/w);
			break;
		case 1028:/* Select Object and Displace in Z */
                        //	move_robot(idr,2,j,g3dwin->vs.size/h);
			break;
		case 256: /* zoom */
		case 272:
		case 258:
		case 274:
		case 8464:
		case 16656:
                        g3dwin->vs.zo = (zo * i) / w + zo;
                        if (g3dwin->vs.zo < .0)
			{

                                g3dwin->vs.zo = .0;
			}
                        //			std::cout << "g3dwin->vs.zo = "<< g3dwin->vs.zo << std::endl;
			break;
		case 512: /* angle */
		case 514:
		case 528:
		case 530:
		case 8720:
		case 16912:
                        g3dwin->vs.az = (-2 * GAIN_AZ * i) / w + az;
                        if (g3dwin->vs.az < .0)
                                g3dwin->vs.az = 2 * M_PI;
                        if (g3dwin->vs.az > 2 * M_PI)
                                g3dwin->vs.az = .0;
                        g3dwin->vs.el = (GAIN_EL * j) / w + el;
                        if (g3dwin->vs.el < -M_PI / 2.0)
                                g3dwin->vs.el = -M_PI / 2.0;
                        if (g3dwin->vs.el > M_PI / 2.0)
                                g3dwin->vs.el = M_PI / 2.0;
			break;
		case 1024: /* origin esf*/
		case 1026:
		case 1040:
		case 1042:
		case 9232:
		case 17424:
			rotinc = (-GAIN_AZ * i) / w;
			incinc = (-GAIN_EL * j) / w;
			/* rotation effects */
                        g3dwin->vs.x = x + 2.0 * zo * cos(el) * sin(rotinc / 2.0) * sin(az
					+ rotinc / 2.0);
                        g3dwin->vs.y = y - 2.0 * zo * cos(el) * sin(rotinc / 2.0) * cos(az
					+ rotinc / 2.0);
                        g3dwin->vs.az = az + rotinc;

                        if (g3dwin->vs.az < .0)
                                g3dwin->vs.az = 2 * M_PI;
                        if (g3dwin->vs.az > 2 * M_PI)
                                g3dwin->vs.az = .0;
                        x_aux = g3dwin->vs.x;
                        y_aux = g3dwin->vs.y;
                        az_aux = g3dwin->vs.az;
			/* incline effects */
                        g3dwin->vs.z = z - 2.0 * zo * sin(incinc / 2.0) * cos(el + incinc
					/ 2.0);
                        g3dwin->vs.x = x_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
					/ 2.0) * cos(az_aux);
                        g3dwin->vs.y = y_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
					/ 2.0) * sin(az_aux);
                        g3dwin->vs.el = el + incinc;
                        if (g3dwin->vs.el < -M_PI / 2)
                                g3dwin->vs.el = -M_PI / 2;
                        if (g3dwin->vs.el > M_PI / 2)
                                g3dwin->vs.el = M_PI / 2;
			break;
			/* deplacement du  point central de l'image */
			/* - bouton gauche et central -> x          */
			/* - bouton droit et central  -> y          */
			/* - bouton gauche et droit   -> z          */
		case 768: /* origin x */
		case 770:
		case 784:
		case 8976:
		case 17168:
                        g3dwin->vs.x = x + j * g3dwin->vs.size / w;
                        /*       g3dwin->vs.x = x + i*g3dwin->vs.size/h; */
			break;
		case 1536: /* origin y */
		case 1552:
		case 1538:
		case 9744:
		case 17936:
                        g3dwin->vs.y = y + j * g3dwin->vs.size / w;
                        /*       g3dwin->vs.y = y + i*g3dwin->vs.size/h; */
			break;
		case 1280: /* origin z */
		case 1282:
		case 1296:
		case 9488:
		case 17680:
                        g3dwin->vs.z = z + j * g3dwin->vs.size / w;
                        /*       g3dwin->vs.z = z + i*g3dwin->vs.size/h; */
			break;
		case 1792: /* reset up */
                        g3dwin->vs.up[0] = .0;
                        g3dwin->vs.up[1] = .0;
                        g3dwin->vs.up[2] = 1.0;
			break;
		}
	}
}

void qt_change_mob_frame(G3D_Window* win,pp3d_matrix4 frame)
{
	p3d_matrix4 matr;
	p3d_vector4 Xc,Xcnew,Xw,Xwnew;
	
	calc_cam_param(win,Xc,Xw);
	win->cam_frame = frame;
	p3d_matInvertXform(*win->cam_frame, matr);
	p3d_matvec4Mult(matr, Xc, Xcnew);
	p3d_matvec4Mult(matr, Xw, Xwnew);
	recalc_mouse_param(win->vs,Xcnew,Xwnew);
	recalc_cam_up(win->vs,matr);
}

void qt_reset_mob_frame(G3D_Window* win)
{
	p3d_vector4 Xc,Xcnew,Xw,Xwnew;
	
	calc_cam_param(win,Xc,Xw);
	recalc_mouse_param(win->vs,Xc,Xw);
	recalc_cam_up(win->vs,*win->cam_frame);
	win->cam_frame = &Id;
}


#endif

static void
button_done(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_del_win(win);
}

static void
button_unselect(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  G3D_SELECTED_JOINT= -999;
  g3d_draw_win(win);
}

static void
button_joints(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->vs.displayJoints= !win->vs.displayJoints;
  g3d_draw_win(win);
}


static void
button_light(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->vs.enableLight= !win->vs.enableLight;
  g3d_draw_win(win);
}

static void
button_floor(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->vs.displayFloor= !win->vs.displayFloor;
  g3d_set_win_floor_color(win->vs, 1, 0.87, 0.49);
  g3d_draw_win(win);
}

static void
button_tiles(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->vs.displayTiles= !win->vs.displayTiles;
  g3d_draw_win(win);
}

static void
button_shadows(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->vs.displayShadows= !win->vs.displayShadows;
  g3d_draw_win(win);
}

static void
button_walls(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->vs.displayWalls = !win->vs.displayWalls;
  g3d_draw_win(win);
}

static void
button_antialiasing(FL_OBJECT *ob, long data) {
#ifdef ENABLE_ANTIALIASING
  G3D_Window *win = (G3D_Window *)data;
  win->vs.enableAntialiasing = !win->vs.enableAntialiasing;
  if (win->vs.enableAntialiasing) 
    glEnable(GL_MULTISAMPLE_ARB);
  else
    glDisable(GL_MULTISAMPLE_ARB);
  g3d_draw_win(win);
#else
  printf("Antialiasing flag is FALSE. Enable it in cmake\n");
#endif
}

static void
button_shaders(FL_OBJECT *ob, long data) {
  #ifdef USE_SHADERS
  G3D_Window *win = (G3D_Window *)data;
  win->vs.enableShaders = !win->vs.enableShaders;
  if(win->vs.enableShaders) 
  { g3d_load_next_shader(); }
  else
  { glUseProgram(0); }
 
  g3d_draw_win(win);
  #else
  printf("USE_SHADERS flag is OFF. Enable it in cmake\n");
  #endif
}

static void
button_copy(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_copy_win(win);
}


static void
button_view_save(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_save_win_camera(win->vs);
  g3d_draw_win(win);
}



static void
button_view_restore(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_restore_win_camera(win->vs);
  g3d_draw_win(win);
}



static void
button_view_fil(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;

  if (win->vs.FILAIRE) {
    win->vs.FILAIRE = 0;
  } else {
    win->vs.FILAIRE = 1;
    win->vs.CONTOUR = 0;
    win->vs.GOURAUD = 0;
  }
  win->vs.list = -1;

  g3d_draw_win(win);
}


static void
button_view_cont(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->vs.CONTOUR) {
    win->vs.CONTOUR = 0;
  } else {
    win->vs.CONTOUR = 1;
    win->vs.FILAIRE = 0;
    win->vs.GOURAUD = 0;
  }
  win->vs.list = -1;
  g3d_draw_win(win);
}

static void
button_view_ghost(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->vs.GHOST) {
    win->vs.GHOST = 0;
  } else {
    win->vs.GHOST = 1;
    win->vs.GOURAUD = 0;
  }
  win->vs.list = -1;
  g3d_draw_win(win);
}

#ifdef DPG
static void
button_view_grid(FL_OBJECT *ob, long data) {
  
  G3D_Window *win = (G3D_Window *)data;
  if (ENV.getBool(Env::drawGrid)) {
    ENV.setBool(Env::drawGrid, 0);
  } else {
    ENV.setBool(Env::drawGrid, 1);
  }
  g3d_draw_win(win);
}
#endif


static void
button_view_bb(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->vs.BB) {
    win->vs.BB = 0;
  } else {
    win->vs.BB = 1;
    win->vs.GOURAUD = 0;
  }
  win->vs.list = -1;
  g3d_draw_win(win);
}

static void
button_view_gour(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->vs.GOURAUD) {
    win->vs.GOURAUD = 0;
  } else {
    win->vs.GOURAUD = 1;
  }
  win->vs.list = -1;
  g3d_draw_win(win);
}


// static void
// button_freeze(FL_OBJECT *ob, long data) {
//   G3D_Window *win = (G3D_Window *)data;
//   int i;
//   if (win->vs.ACTIVE) {
//     win->vs.ACTIVE = 0;
//   } else {
//     win->vs.ACTIVE = 1;
//   }
//   for(i = 0;i < 1000;i++)
//     g3d_draw_win(win);
// }

//! @ingroup graphic 
//! This function does all that is needed to take a shot of the current OpenGL window,
//! name it and save it in the good directory.
void g3d_screenshot(char * winname)
{
  if(winname==NULL) {
   printf("%s: %d: input char string is NULL.\n", __FILE__,__LINE__);
   return;
  }

  G3D_Window *win = NULL;

  win= g3d_get_win_by_name(winname);

  if(win==NULL) {
   printf("%s: %d: there is no window named \"%s\".", __FILE__,__LINE__,winname);
   return;
  }

  static int count= 1;
  char pathname[128], basename[128], extname[128], extname2[128];
  char filename[128], filename2[128], command[128];
  char *path= NULL;
  
  path= getenv("HOME_MOVE3D");

  strcpy(basename, "screenshot-");
  if(count < 10) { strcat(basename, "0000"); }
  else if(count < 100) { strcat(basename, "000"); }
  else if(count < 1000) { strcat(basename, "00"); }
  else if(count < 10000) { strcat(basename, "0"); }
 
  
  if(path==NULL)  {
	sprintf(pathname, "./screenshots/");
  }
  else  { 
    sprintf(pathname, "%s/screenshots/", path);
  }
  
  strcpy(filename, pathname);
  strcat(filename, basename);	
  strcpy(filename2, pathname);
  strcpy(filename2, filename);
	
  sprintf(extname, "%d_%s.ppm", count,winname);
  sprintf(extname2, "%d_%s.png", count++,winname);
  
  strcat(filename, extname);	
  strcat(filename2, extname2); 

  win->vs.displayFrame= FALSE;
  //g3d_refresh_allwin_active();
  g3d_draw_win(win);
  g3d_export_OpenGL_display(filename);

  //convert the ppm to lossless png using the convert command:
  sprintf(command,"convert -quality 100 %s %s; rm %s", filename, filename2, filename);
  system(command);

  win->vs.displayFrame= TRUE;	
}

void button_screenshot(FL_OBJECT *ob, long data) { 
  g3d_screenshot((char *)"Move3D");
  return;
}

static void
button_mobile_camera(FL_OBJECT *ob, long data) {
	G3D_Window *win = (G3D_Window *)data;
	p3d_matrix4 matr;
	p3d_vector4 Xc,Xcnew,Xw,Xwnew;

	if(fl_get_button(ob)) {
		G3D_WINDOW_CMC = win;
		calc_cam_param(win,Xc,Xw);
		win->cam_frame = (*win->fct_mobcam)();
		p3d_matInvertXform(*win->cam_frame, matr);
		p3d_matvec4Mult(matr, Xc, Xcnew);
		p3d_matvec4Mult(matr, Xw, Xwnew);
                recalc_mouse_param(win->vs,Xcnew,Xwnew);
                recalc_cam_up(win->vs,matr);
	} else {
		calc_cam_param(win,Xc,Xw);
                recalc_mouse_param(win->vs,Xc,Xw);
                recalc_cam_up(win->vs,*win->cam_frame);
		win->cam_frame = &Id;
	}

	g3d_draw_win(win);
}

static void
button_proj(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  
  switch(win->vs.projection_mode)
  {
    case G3D_PERSPECTIVE:
      win->vs.projection_mode= G3D_ORTHOGRAPHIC;
    break;
    case G3D_ORTHOGRAPHIC:
      win->vs.projection_mode= G3D_PERSPECTIVE;
    break;
  }
  g3d_refresh_allwin_active();
  g3d_draw_win(win);
}

/***************************************************************************************************/
/** WIN32 Global Functions *************************************************************************/

#else

static void g3d_draw_win(G3D_Window *win);


G3D_Window* g3d_new_win(char *name,int w, int h, float size) {
  G3D_Window *win = (G3D_Window *)malloc(sizeof(G3D_Window));



  /* Les parametres de la fenetre */
  win->form       = (void *)NULL;
  win->canvas     = (void *)NULL;
  win->vs.size       = size;
  win->vs.FILAIRE = 0;
  win->vs.CONTOUR = 0;
  win->vs.GOURAUD = 0;
  win->vs.ACTIVE = 1;
  win->vs.list = -1;
  win->fct_draw   = NULL;
  win->next       = NULL;
  win->fct_mobcam   = NULL;
  win->cam_frame  = &Id;
  win->mcamera_but  = (void *)NULL;
  sprintf(win->name,"%s",name);

  g3d_set_win_camera(win, .0,.0,.0,5*size, INIT_AZ, INIT_EL,.0,.0,1.0);
  g3d_save_win_camera(win);
  g3d_set_win_bgcolor(win,1.0,1.0,1.0);
  win->next = G3D_WINDOW_LST;
  G3D_WINDOW_LST = win;

  G3D_WINDOW_CUR = win;

  return(win);
}

void g3d_event_win(G3D_Window *g3dwin, int event, int xpos, int ypos, void* data)
/*
 Added and used by Win32 event
 Code copied from "canvas_viewing"
 */
{
  int          i,j;
  double       x_aux,y_aux,az_aux,rotinc,incinc;

  int w;
  static int   i0,j0;
  static double x,y,z,zo,el,az;



  G3D_MODIF_VIEW = TRUE;

  switch(event) {

    case 11: /* Zoom: Mouse move + Left mouse button down */

      i = xpos - i0;
      j = ypos - j0;
      w = *((int*)data);

      g3dwin->vs.zo = (zo * i)/w + zo;
      if(g3dwin->vs.zo < .0) g3dwin->vs.zo = .0;

      g3d_draw_win(g3dwin);

      break;

    case 12: /* Angle ref: Mouse move + Middle mouse button down */

      i = xpos - i0;
      j = ypos - j0;
      w = *((int*)data);

      g3dwin->vs.az = (-3*GAIN_AZ * i)/w + az;
      if(g3dwin->vs.az < .0) g3dwin->vs.az += 2*M_PI;
      if(g3dwin->vs.az > 2*M_PI) g3dwin->vs.az -= 2*M_PI;
      g3dwin->vs.el = (GAIN_EL * j)/w + el;
      if(g3dwin->vs.el < -M_PI/2.0) g3dwin->vs.el = -M_PI/2.0;
      if(g3dwin->vs.el > M_PI/2.0) g3dwin->vs.el = M_PI/2.0;


      g3d_draw_win(g3dwin);

      break;

    case 13:  /* Origin esf : Mouse move + Right mouse button down */

      i = xpos - i0;
      j = ypos - j0;
      w = *((int*)data);

      rotinc = (+2*GAIN_AZ * i)/w;
      incinc = (-2*GAIN_EL * j)/w;
      /* rotation effects */
      g3dwin->vs.x = x + 2.0*zo*cos(el)*sin(rotinc/2.0)*sin(az+rotinc/2.0);
      g3dwin->vs.y = y - 2.0*zo*cos(el)*sin(rotinc/2.0)*cos(az+rotinc/2.0);
      g3dwin->vs.az = az + rotinc;

      if(g3dwin->vs.az < .0)
        g3dwin->vs.az = 2*M_PI;
      if(g3dwin->vs.az > 2*M_PI)
        g3dwin->vs.az = .0;
      x_aux=g3dwin->vs.x;
      y_aux=g3dwin->vs.y;
      az_aux=g3dwin->vs.az;
      /* incline effects */
      g3dwin->vs.z = z - 2.0 *zo*sin(incinc/2.0)*cos(el+incinc/2.0);
      g3dwin->vs.x = x_aux
			+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*cos(az_aux);
      g3dwin->vs.y = y_aux
			+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*sin(az_aux);
      g3dwin->vs.el = el + incinc;
      if(g3dwin->vs.el < -M_PI/2)
        g3dwin->vs.el = -M_PI/2;
      if(g3dwin->vs.el > M_PI/2)
        g3dwin->vs.el = M_PI/2;

      g3d_draw_win(g3dwin);

      break;

      /* not yet implemented for Win32:

			 deplacement du  point central de l'image
			 - bouton gauche et central -> x
			 - bouton droit et central  -> y
			 - bouton gauche et droit   -> z
			 case 768:    origin x
                         g3dwin->vs.x = x + j*g3dwin->vs.size/w;
                         g3dwin->vs.x = x + i*g3dwin->vs.size/h;
			 break;
			 case 1536:    origin y
                         g3dwin->vs.y = y + j*g3dwin->vs.size/w;
                         g3dwin->vs.y = y + i*g3dwin->vs.size/h;
			 break;
			 case 1280:    origin z
                         g3dwin->vs.z = z + j*g3dwin->vs.size/w;
                         g3dwin->vs.z = z + i*g3dwin->vs.size/h;
			 break;
			 case 1792:    reset up
                         g3dwin->vs.up[0]=.0;
                         g3dwin->vs.up[1]=.0;
                         g3dwin->vs.up[2]=1.0;
			 break;
			 }

			 update view
			 g3d_draw_win(g3dwin);

			 break; */


    case 2: /* One Mouse Button Press */

      /* store initial cursor pos */
      i0 = xpos;
      j0 = ypos;

      /* store previous value */
      x=g3dwin->vs.x;
      y=g3dwin->vs.y;
      z=g3dwin->vs.z;
      zo=g3dwin->vs.zo;
      el=g3dwin->vs.el;
      az=g3dwin->vs.az;
      break;

  }

  G3D_MODIF_VIEW = FALSE;

  return;
}


void g3d_del_win(G3D_Window *win) {
  G3D_Window *w = G3D_WINDOW_LST;

  if(w == win)
    G3D_WINDOW_LST = win->next;
  else {
    while(w) {
      if(w->next == win) {
        w->next = win->next;
        break;
      }
      w=w->next;
    }
  }
  free(win);
}


void g3d_refresh_allwin_active(void) {
  G3D_Window *w = G3D_WINDOW_LST;
  /* FL_OBJECT  *ob; */

  while(w) {

    if(w->ACTIVE == 1) {
      w->list = -1;

      /* =      glMatrixMode(GL_PROJECTION);
			 =      glLoadIdentity();
			 ob = ((FL_OBJECT *)w->canvas);
			 fl_get_winsize(FL_ObjWin(ob),&winw,&winh);
			 gluPerspective(40.0,(GLdouble)winw/(GLdouble)winh,w->size/100.0,100.0*w->size);
			 =      glMatrixMode(GL_MODELVIEW);
			 =      glLoadIdentity();

			 =     glEnable(GL_DEPTH_TEST);
                         =     if(w->vs.GOURAUD){
			 =   glShadeModel(GL_SMOOTH); */
    } else {
      glShadeModel(GL_FLAT);
    }
    /*g3d_draw_win(w);
		 */

    w = w->next;
  }

}
/** WIN32 Static Functions *************************************************************************/


static void
g3d_draw_win(G3D_Window *win) {
  p3d_vector4 Xc,Xw;
  p3d_vector4 up;

  G3D_WINDOW_CUR = win;

  calc_cam_param(win,Xc,Xw);
  p3d_matvec4Mult(*win->cam_frame,win->vs.up,up);

  glPushMatrix();

  glLoadIdentity();

  gluLookAt(Xc[0],Xc[1],Xc[2],Xw[0],Xw[1],Xw[2],up[0],up[1],up[2]);

  if(G3D_MODIF_VIEW && win->vs.displayFrame) {
    glPushMatrix();
    glTranslatef(win->vs.x,win->vs.y,win->vs.z);
    g3d_draw_frame();
    glPopMatrix();
  }

  if(win->fct_draw) (*win->fct_draw)();

  glPopMatrix();

  glFinish();
}


/***********************************************************************************************/

#endif



/* pure opengl or m3d code  */


void
g3d_draw_allwin(void) {
#ifndef QT_GL
  G3D_Window *w = G3D_WINDOW_LST;
  while (w) {
    g3d_draw_win(w);
    w = w->next;
  }
#else
  if(pipe2openGl)
    {
      pipe2openGl->update();
    }
#endif
}

void
g3d_draw_allwin_active(void) {
#ifndef QT_GL
  G3D_Window *w = G3D_WINDOW_LST;
  while (w) {
    if (w->vs.ACTIVE == 1) {
      g3d_draw_win(w);
    }
    w = w->next;
  }
#else
  if(pipe2openGl)
    {
      pipe2openGl->update();
    }
#endif
}


void
g3d_print_allwin(void) {
  G3D_Window *w = G3D_WINDOW_LST;
  while(w) {
    PrintInfo(("%s ",w->name));
    w = w->next;
  }
  PrintInfo(("\n"));
}


void
g3d_resize_win(G3D_Window *win, float w, float h, float size) {
  win->vs.size = size;

  glViewport(0, 0, (GLsizei)w, (GLsizei)h);

  g3d_set_win_camera(win->vs, .0,.0,.0,5*size, INIT_AZ, INIT_EL,.0,.0,1.0);
  g3d_save_win_camera(win->vs);
}


void
g3d_resize_allwin_active(float w, float h, float size) {
  G3D_Window *win = G3D_WINDOW_LST;
  while (win) {
    if (win->vs.ACTIVE == 1) {
      g3d_resize_win(win, w, h, size);
    }
    win = win->next;
  }
}





void
g3d_set_win_fct_mobcam(G3D_Window *win, pp3d_matrix4 (*fct)(void)) {
  win->fct_mobcam = fct;
}

void
g3d_set_mobile_camera_activate(G3D_Window *win, int mode)
// mode != 0 -> activate, else desactivate
{
  p3d_matrix4 matr;
  p3d_vector4 Xc,Xcnew,Xw,Xwnew;

  if(mode != 0) {
    G3D_WINDOW_CMC = win;
    calc_cam_param(win,Xc,Xw);
    win->cam_frame = (*win->fct_mobcam)();
    p3d_matInvertXform(*win->cam_frame, matr);
    p3d_matvec4Mult(matr, Xc, Xcnew);
    p3d_matvec4Mult(matr, Xw, Xwnew);
    recalc_mouse_param(win->vs,Xcnew,Xwnew);
    recalc_cam_up(win->vs,matr);
  } else {
    calc_cam_param(win,Xc,Xw);
    recalc_mouse_param(win->vs,Xc,Xw);
    recalc_cam_up(win->vs,*win->cam_frame);
    win->cam_frame = &Id;
  }

  g3d_draw_win(win);
}



void
g3d_set_win_drawer(G3D_Window *win, void (*fct)(void)) {
  win->fct_draw = fct;
}


void
g3d_init_allwin_booleans(void) {
  G3D_Window *w = G3D_WINDOW_LST;
  while (w) {
    w->vs.FILAIRE = 0;
    w->vs.CONTOUR = 0;
    w->vs.GOURAUD = 0;
    w = w->next;
  }
}


void
g3d_beg_poly(void) {
  switch(POLY_TYPE) {
    case 0:
      glBegin(GL_POLYGON);
      break;
    case 1:
      glBegin(GL_LINE_LOOP);
      break;
    case 2:
      PrintInfo(("case 2\n"));
      LIST=glGenLists(1);
      glNewList(LIST,GL_COMPILE);
      break;
  }
}


void
g3d_end_poly(void) {
  if (POLY_TYPE != 2) {
    glEnd();
    return;
  }
  PrintInfo(("case 2\n"));
  glEndList();
  /*
	 glColor3f (1.0, 1.0, 0.0);
	 glBegin(GL_LINE_LOOP);glCallList(LIST);glEnd();
	 glStencilFunc(GL_EQUAL, 0, 1);
	 glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	 glColor3f (0.0, 0.0, 0.0);
	 */
  glBegin(GL_POLYGON);
  glCallList(LIST);
  glEnd();
  /*
	 glColor3f (1.0, 1.0, 0.0);
	 glStencilFunc(GL_ALWAYS,0,1);
	 glStencilOp(GL_INVERT, GL_INVERT, GL_INVERT);
	 */
  glLineWidth(4.0);
  glBegin(GL_LINE_LOOP);
  glCallList(LIST);
  glEnd();
  glLineWidth(1.0);

  glDeleteLists(LIST,1);
}


G3D_Window *g3d_get_cur_win(void) {
  return(G3D_WINDOW_CUR);
}

G3D_Window *g3d_get_win_by_name(char *s)
{
        if(s==NULL) {
           return NULL;
        }
	G3D_Window *w = G3D_WINDOW_LST;
	while(w)
	{
		if (strcmp(s,w->name)==0)
			return w;
		w = w->next;
	}
	return NULL;
}

/**
  * Gets the current window states
  */
g3d_states& g3d_get_cur_states(){
    return(G3D_WINDOW_CUR->vs);
}

/**
  * Gets the current window states by window name
  */
g3d_states& g3d_get_states_by_name(char *s){
    return (g3d_get_win_by_name(s)->vs);
}

G3D_Window *g3d_get_cmc_win(void)
/* rendre la fenetre ou on travaille avec la camera mobile maintenant */
{
  return(G3D_WINDOW_CMC);
}

/*          Static Functions                */


/* fonction pour calculer les parametres de position de la      */
/* camera de la facon necessaire pour openGL                    */
void
calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) {
  static p3d_matrix4 Txc = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  p3d_matrix4 m_aux;
  p3d_vector4 Xx;


  get_lookat_vector(win->vs, Xx);
  get_pos_cam_matrix(win->vs, Txc);
  
  p3d_mat4Mult(*win->cam_frame,Txc,m_aux);
  p3d_matvec4Mult(m_aux,Xx,Xc);
  p3d_matvec4Mult(*win->cam_frame,Xx,Xw);

}

//Edit Mokhtar
/****************************************************************************/
/** \brief Enable OpenGL picking mode
 \param cursorX Mouse X on the window
 \param cursorY Mouse Y on the window
 */
/****************************************************************************/
static void startPicking(int cursorX, int cursorY) {

  GLint viewport[4];
  int winw,winh;
  G3D_Window *w = G3D_WINDOW_LST;
  FL_OBJECT  *ob;

  glSelectBuffer(BUFSIZE,selectBuf);
  glRenderMode(GL_SELECT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  glGetIntegerv(GL_VIEWPORT,viewport);
  gluPickMatrix(cursorX,viewport[3]-cursorY,5,5,viewport);
  ob = (FL_OBJECT *)w->canvas;
  fl_get_winsize(FL_ObjWin(ob),&winw,&winh);
  g3d_set_projection_matrix(G3D_PERSPECTIVE);


  glMatrixMode(GL_MODELVIEW);
  glInitNames();
  glPushName(-999);

}
/****************************************************************************/
/** \brief Stop picking, reset to render mode and retrun the Node selected
 */
/****************************************************************************/
static int stopPicking() {
  int hits;

  glPopName();
  // restoring the original projection matrix
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);




  glFlush();

  // returning to normal rendering mode
  hits = glRenderMode(GL_RENDER);

	//   if there are hits process them
  if (hits != 0)
    return processHits(hits,selectBuf);
  else{
    return -999;
  }
}
/****************************************************************************/
/** \brief Retrun the first Node number
 \param hits number of hits
 \param buffer the chart where the names are stocked
 */
/****************************************************************************/
static int processHits(GLint hits, GLuint buffer[]) {
  int i;
  GLuint names, *ptr, minZ,*ptrNames, numberOfNames;

  ptr = (GLuint *) buffer;
  minZ = 0xffffffff;
  for (i = 0; i < hits; i++) {
    names = *ptr;
    ptr++;
    if (*ptr < minZ) {
      numberOfNames = names;
      minZ = *ptr;
      ptrNames = ptr+2;
    }

    ptr += names+2;
  }
  return *ptrNames;
}

//! @ingroup graphic 
//! Enables/disables all the mouse interactions that modify parameters
//! used by the planner (like current robot configuration) for all the g3d_windows.
//! \param enabled set to 1/0 to enable/disable picking
void g3d_set_picking(unsigned int enabled)
{
  if(enabled==0)
  {  enable_picking= FALSE;  }
  else
  {  enable_picking= TRUE;  }
}




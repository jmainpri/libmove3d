#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Collision-pkg.h"
#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#endif
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

static void get_lookat_vector(G3D_Window *win, p3d_vector4 Vec);
static void get_pos_cam_matrix(G3D_Window *win, p3d_matrix4 Transf);
static void recalc_mouse_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);
static void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);
static void recalc_cam_up(G3D_Window *win, p3d_matrix4 transf);

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

static int        G3D_GLCONFIG[30] = { /* pas utilise... servirait pour le stencil */
GLX_RGBA, GLX_DEPTH_SIZE, 1,
GLX_RED_SIZE, 1, GLX_GREEN_SIZE, 1, GLX_BLUE_SIZE, 1,
GLX_STENCIL_SIZE,1,
GLX_DOUBLEBUFFER,
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
static void button_freeze(FL_OBJECT *ob, long data);
static void button_mobile_camera(FL_OBJECT *ob, long data);
static void button_joints(FL_OBJECT *ob, long data);
#ifdef PLANAR_SHADOWS
static void button_light(FL_OBJECT *ob, long data);
static void button_floor(FL_OBJECT *ob, long data);
static void button_tiles(FL_OBJECT *ob, long data);
static void button_walls(FL_OBJECT *ob, long data);
static void button_shadows(FL_OBJECT *ob, long data);
#endif

static void g3d_draw_win(G3D_Window *win);
static G3D_Window *g3d_copy_win(G3D_Window *win);

/** UNIX Global Functions *************************************************************************/

#ifdef PLANAR_SHADOWS
//-----------------------------------------------------------------------------
// Name: findPlane()
// Desc: find the plane equation given 3 points
//-----------------------------------------------------------------------------
void g3d_findPlane( GLdouble plane[4], GLdouble v0[3], GLdouble v1[3], GLdouble v2[3] )
{
  GLdouble vec0[3], vec1[3];

  // Need 2 vectors to find cross product
  vec0[0] = v1[0] - v0[0];
  vec0[1] = v1[1] - v0[1];
  vec0[2] = v1[2] - v0[2];

  vec1[0] = v2[0] - v0[0];
  vec1[1] = v2[1] - v0[1];
  vec1[2] = v2[2] - v0[2];

  // Find cross product to get A, B, and C of plane equation
  plane[0] =   vec0[1] * vec1[2] - vec0[2] * vec1[1];
  plane[1] = -(vec0[0] * vec1[2] - vec0[2] * vec1[0]);
  plane[2] =   vec0[0] * vec1[1] - vec0[1] * vec1[0];

  plane[3] = -(plane[0] * v0[0] + plane[1] * v0[1] + plane[2] * v0[2]);
}

// Construit les matrices de projection des ombres sur les plans du sol et des murs.
// Cette fonction doit être appelée chaque fois que la position de la lumière change.
void g3d_build_shadow_matrices(G3D_Window *win)
{
  buildShadowMatrix( win->floorShadowMatrix, win->lightPosition, win->floorPlane );
  buildShadowMatrix( win->wallShadowMatrix[0], win->lightPosition, win->wallPlanes[0] );
  buildShadowMatrix( win->wallShadowMatrix[1], win->lightPosition, win->wallPlanes[1] );
  buildShadowMatrix( win->wallShadowMatrix[2], win->lightPosition, win->wallPlanes[2] );
  buildShadowMatrix( win->wallShadowMatrix[3], win->lightPosition, win->wallPlanes[3] );
}
#endif


G3D_Window
*g3d_new_win(const char *name,int w, int h, float size) {
  G3D_Window *win = (G3D_Window *)malloc(sizeof(G3D_Window));

#ifndef QT_GL
  FL_FORM    *form= fl_bgn_form(FL_UP_BOX,w+90,h+20);
  FL_OBJECT  *can = fl_add_glcanvas(FL_NORMAL_CANVAS,10,10,w,h,"Nico");

  FL_OBJECT *wcop= fl_add_button(FL_NORMAL_BUTTON,w+20,20,60,20,"Copy");

  FL_OBJECT *vsav= fl_add_button(FL_NORMAL_BUTTON,w+20,40,60,40,"Save\nView");
  FL_OBJECT *vres= fl_add_button(FL_NORMAL_BUTTON,w+20,80,60,40,"Restore\n View");


  FL_OBJECT *vfil= fl_add_button(FL_NORMAL_BUTTON,w+20,140,60,40,"Poly/\nLine");
  FL_OBJECT *vcont= fl_add_button(FL_NORMAL_BUTTON,w+20,180,60,40,"Contours");
  FL_OBJECT *vGhost= fl_add_button(FL_NORMAL_BUTTON,w+20,220,60,20,"Ghost");
  FL_OBJECT *vBb= fl_add_button(FL_NORMAL_BUTTON,w+20,240,60,20,"BB");
#ifdef DPG
  FL_OBJECT *vGrid= fl_add_button(FL_NORMAL_BUTTON,w+20,260,60,20,"Grid");
  FL_OBJECT *vgour= fl_add_button(FL_NORMAL_BUTTON,w+20,280,60,20,"Smooth");
#else
  FL_OBJECT *vgour= fl_add_button(FL_NORMAL_BUTTON,w+20,260,60,40,"Smooth");
#endif
  FL_OBJECT *wfree= fl_add_button(FL_PUSH_BUTTON,w+20,320,60,40,"Freeze");

  FL_OBJECT *mcamera= fl_add_button(FL_PUSH_BUTTON,w+20,360,60,40,"Mobile\n Camera");

  FL_OBJECT *done= fl_add_button(FL_NORMAL_BUTTON,w+20,420,60,20,"Done");

  FL_OBJECT *unselect= fl_add_button(FL_NORMAL_BUTTON,w+20,420,60,40,"Unselect\n Joint");

  //This frame is not automatically resized after a window resize operation, so
  //it is often nicer without it:
  //fl_add_labelframe(FL_BORDER_FRAME,w+15,510,68,90,"Options");

  FL_OBJECT *displayJoints = fl_add_checkbutton(FL_PUSH_BUTTON,w+20,460,60,20,"Joints");
#ifdef PLANAR_SHADOWS
  FL_OBJECT *oplight= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,480,60,20,"Light");
  fl_set_button(oplight, 1);
  FL_OBJECT *opfloor = fl_add_checkbutton(FL_PUSH_BUTTON,w+20,500,60,20,"Floor");
  FL_OBJECT *optiles = fl_add_checkbutton(FL_PUSH_BUTTON,w+20,520,60,20,"Tiles");
  FL_OBJECT *walls= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,540,60,20,"Walls");
  FL_OBJECT *shadows= fl_add_checkbutton(FL_PUSH_BUTTON,w+20,560,60,20,"Shadows");
#endif

  fl_end_form();

#else
  FL_FORM    *form=NULL;
  FL_FORM    *can=NULL;
  FL_FORM    *mcamera=NULL;
#endif


  /* Les parametres de la fenetre */
  win->form       = (void *)form;
  win->canvas     = (void *)can;
  win->size       = size;
  win->FILAIRE = 0;
  win->CONTOUR = 0;
  win->GHOST = 0;
  win->BB = 0;
  win->GOURAUD = 0;
  win->ACTIVE = 1;
  win->list = -1;
  win->fct_draw   = NULL;
  win->next       = NULL;
  win->fct_mobcam   = NULL;
  win->cam_frame  = &Id;
  win->mcamera_but  = (void *)mcamera;
  sprintf(win->name,"%s",name);
  g3d_set_win_camera(win, .0,.0,.0,2*size, INIT_AZ, INIT_EL,.0,.0,1.0);
  g3d_save_win_camera(win);
  g3d_set_win_bgcolor(win,1.0,1.0,1.0);
  win->next = G3D_WINDOW_LST;
  G3D_WINDOW_LST = win;
#ifdef PLANAR_SHADOWS
  if(ENV.getBool(Env::isCostSpace) && (GroundCostObj != NULL)){
	  g3d_set_win_bgcolor(win, 0, 0, 0);
  }
  else
  {
	  g3d_set_win_bgcolor(win, 1.0, 1.0, 0.8);
  }
  win->fct_draw2= NULL;
  win->fct_key1= NULL;
  win->fct_key2= NULL;
  win->floorColor[0]= 0.5;
  win->floorColor[1]= 0.9;
  win->floorColor[2]= 0.9;
  win->wallColor[0]= 0.5;
  win->wallColor[1]= 0.5;
  win->wallColor[2]= 0.6;
  win->displayJoints = 0;
  win->enableLight = 1;
  win->displayShadows = 0;
  win->displayWalls = 0;
  win->displayFloor = 0;
  win->displayTiles = 0;
  win->allIsBlack = 0;
#endif
#ifdef HRI_PLANNER
  win->win_perspective = 0;
  win->point_of_view = 0;
  win->draw_mode = NORMAL;
#endif

#ifndef QT_GL
  /* Attributs/Handlers du canvas */
  fl_set_glcanvas_attributes(can,G3D_GLCONFIG);
  fl_set_object_gravity(can,FL_NorthWest,FL_SouthEast);

  fl_add_canvas_handler(can,Expose,canvas_expose,(void *)win);
  fl_add_canvas_handler(can,ButtonPress,canvas_viewing,(void *)win);
  fl_add_canvas_handler(can,ButtonRelease,canvas_viewing,(void *)win);
  fl_add_canvas_handler(can,MotionNotify,canvas_viewing,(void *)win);
#ifdef PLANAR_SHADOWS
  fl_add_canvas_handler(can,KeyPress,canvas_viewing,(void *)win);
  fl_add_canvas_handler(can,KeyRelease,canvas_viewing,(void *)win);
#endif
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
  fl_set_object_gravity(wfree,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(done,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(unselect,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(mcamera,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(displayJoints,FL_NorthEast,FL_NorthEast);
#ifdef PLANAR_SHADOWS
  fl_set_object_gravity(oplight,FL_NorthEast,FL_NorthEast);
  fl_set_choice(oplight, 0);
  fl_set_object_gravity(opfloor,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(optiles,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(walls,FL_NorthEast,FL_NorthEast);
  fl_set_object_gravity(shadows,FL_NorthEast,FL_NorthEast);
#endif

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
  fl_set_object_callback(wfree,button_freeze,(long)win);
  fl_set_object_callback(mcamera,button_mobile_camera,(long)win);
  fl_set_object_callback(displayJoints,button_joints,(long)win);
#ifdef PLANAR_SHADOWS
  fl_set_object_callback(oplight,button_light,(long)win);
  fl_set_object_callback(opfloor, button_floor,(long)win);
  fl_set_object_callback(optiles,button_tiles,(long)win);
  fl_set_object_callback(walls,button_walls,(long)win);
  fl_set_object_callback(shadows,button_shadows,(long)win);
#endif

  /* fl_show_form(form,FL_PLACE_FREE,FL_FULLLBORDER,name);*/
  fl_show_form(form,FL_PLACE_MOUSE|FL_FREE_SIZE,FL_FULLBORDER,name);
  /* Pour ne recevoir les events MOTION_NOTIFY que quand le bouton est down */
  fl_remove_selected_xevent(FL_ObjWin(can),PointerMotionMask|PointerMotionHintMask);
#endif


#ifdef PLANAR_SHADOWS
	//Les plans du sol et des murs vont être ajustés sur les coordonnées de
	//l'environment_box.
	double xmin, xmax, ymin, ymax, zmin, zmax;
	p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

	if( xmin>=xmax || ymin>=ymax || zmin>=zmax)
	{
		printf("%s: %d: g3d_new_win(): mauvais paramètres pour la commande p3d_set_env_box.\n\t", __FILE__, __LINE__);
		printf("Il faut les donner sous la forme xmin ymin zmin xmax ymax zmax.\n");
	}


	GLdouble v0[3], v1[3], v2[3];

	//plan du sol (normale selon Z):
	v0[0]= xmin;      v1[0]= xmax;      v2[0]= xmin;
	v0[1]= ymin;      v1[1]= ymin;      v2[1]= ymax;
	v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmin;
	g3d_findPlane(win->floorPlane, v0, v1, v2);

	//plan du premier mur (normale selon y):
	v0[0]= xmin;      v1[0]= xmin;      v2[0]= xmax;
	v0[1]= ymin;      v1[1]= ymin;      v2[1]= ymin;
	v0[2]= zmin;      v1[2]= zmax;      v2[2]= zmin;
	g3d_findPlane(win->wallPlanes[0], v0, v1, v2);

	//plan du deuxième mur (normale selon -y):
	v0[0]= xmin;      v1[0]= xmax;      v2[0]= xmin;
	v0[1]= ymax;      v1[1]= ymax;      v2[1]= ymax;
	v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmax;
	g3d_findPlane(win->wallPlanes[1], v0, v1, v2);

	//plan du troisième mur (normale selon x):
	v0[0]= xmin;      v1[0]= xmin;      v2[0]= xmin;
	v0[1]= ymin;      v1[1]= ymax;      v2[1]= ymin;
	v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmax;
	g3d_findPlane(win->wallPlanes[2], v0, v1, v2);


	//plan du quatrième mur (normale selon -x):
	v0[0]= xmax;      v1[0]= xmax;      v2[0]= xmax;
	v0[1]= ymin;      v1[1]= ymax;      v2[1]= ymax;
	v0[2]= zmin;      v1[2]= zmax;      v2[2]= zmin;
	g3d_findPlane(win->wallPlanes[3], v0, v1, v2);

	//positionnement de la lumière:
	win->lightPosition[0]= 0.5*(xmin+xmax);
	win->lightPosition[1]= 0.5*(ymin+ymax);
	win->lightPosition[2]= 0.9*(zmin+zmax);
	win->lightPosition[3]= 1.0;

	//Remplissage des matrices de projection sur les plans dans la direction de la lumière.
	//Si la position de la lumière est modifiée, il faudra mettre à jour les matrices.
	g3d_build_shadow_matrices(win);
#endif


  G3D_WINDOW_CUR = win;
  return(win);
}

#ifdef HRI_PLANNER
G3D_Window  *g3d_new_win_wo_buttons(char *name,int w, int h, float size)
{
	G3D_Window *win = (G3D_Window *)malloc(sizeof(G3D_Window));

	FL_FORM    *form= fl_bgn_form(FL_UP_BOX,w+20,h+20);
	FL_OBJECT  *can = fl_add_glcanvas(FL_NORMAL_CANVAS,10,10,w,h,"NoButtons");

	fl_end_form();


	/* Window parameters */
	win->form       = (void *)form;
	win->canvas     = (void *)can;
	win->size       = size;

	win->win_perspective = 1;
	win->point_of_view = 1;
	win->draw_mode = NORMAL;

	win->FILAIRE = 0;
	win->CONTOUR = 0;
	win->GOURAUD = 0;
	win->ACTIVE = 1;
	win->list = -1;
	win->fct_draw   = NULL;
	win->next       = NULL;
	win->fct_mobcam   = NULL;
	win->cam_frame  = &Id;
	win->mcamera_but  = NULL;

	win->enableLight = 1;
	win->displayFloor = 0;
	win->displayTiles = 0;
	win->displayWalls = 0;
	win->displayShadows = 0;

	sprintf(win->name,"%s",name);
	g3d_set_win_bgcolor(win,1.0,1.0,1.0);
	win->next = G3D_WINDOW_LST;
	G3D_WINDOW_LST = win;

	/* Attributs/Handlers du canvas */
	fl_set_glcanvas_attributes(can,G3D_GLCONFIG);
	fl_set_object_gravity(can,FL_NorthWest,FL_SouthEast);

	fl_add_canvas_handler(can,Expose,canvas_expose_special,(void *)win);

	fl_show_form(form,FL_PLACE_MOUSE|FL_FREE_SIZE,FL_FULLBORDER,name);

	G3D_WINDOW_CUR = win;
	return(win);
}
#endif

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
  G3D_Window *w = G3D_WINDOW_LST;
  FL_OBJECT  *ob;
  int winw,winh;

  while(w) {

    if(w->ACTIVE == 1) {

      w->list = -1;

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      ob = ((FL_OBJECT *)w->canvas);
		fl_get_winsize(FL_ObjWin(ob),&winw,&winh);
	
#ifdef HRI_PLANNER
		if(w->win_perspective)
		{
			//printf("refreshing\n");
			canvas_expose_special(ob, NULL, winw, winh, NULL, w);
		}
		else
		{
#endif
			canvas_expose(ob, NULL, winw, winh, NULL, w);
/*      gluPerspective(40.0,(GLdouble)winw/(GLdouble)winh,w->size/100.0,100.0*w->size);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      glEnable(GL_DEPTH_TEST);
      if(w->GOURAUD) {
        glShadeModel(GL_SMOOTH);
      } else {
        glShadeModel(GL_FLAT);
      }
		g3d_draw_win(w);*/
#ifdef HRI_PLANNER			
		}
#endif	
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
  new_win = g3d_new_win(str,w,h,win->size);

  {
    /* pour associer un context identique au canvas de la fenetre */
    FL_OBJECT   *newob = ((FL_OBJECT *)new_win->canvas);
    XVisualInfo *vi = glXChooseVisual(fl_display,fl_screen,GLPROP(newob)->glconfig);
    glXDestroyContext(fl_display,GLPROP(newob)->context);
    GLPROP(newob)->context = glXCreateContext(fl_display,vi,
                                              fl_get_glcanvas_context(ob),
                                              GLPROP(newob)->direct);
  }

  new_win->FILAIRE = win->FILAIRE;
  new_win->CONTOUR = win->CONTOUR;
  new_win->GOURAUD = win->GOURAUD;
  for( i = 0; i < 6; i++)
    for(j = 0; j < 4; j++) {
      new_win->frustum[i][j] = win->frustum[i][j];
    }
  g3d_set_win_drawer(new_win,win->fct_draw);
  g3d_set_win_bgcolor(new_win, win->bg[0],win->bg[1],win->bg[2]);
  g3d_set_win_fct_mobcam(new_win,win->fct_mobcam);
  if(win->cam_frame == &Id) {
    g3d_set_win_camera(new_win, win->x,win->y,win->z,win->zo,win->az,win->el,win->up[0],win->up[1],win->up[2]);
  } else {
    calc_cam_param(win,Xc,Xw);
    recalc_mouse_param(new_win,Xc,Xw);
    for(i=0;i< 4;i++) new_win->up[i] = win->up[i];
    recalc_cam_up(new_win,*win->cam_frame);
    new_win->zo = win->zo;
  }
  return(new_win);
}

double g3d_get_light_factor(void) {
  return LIGHT_FACTOR;
}

void g3d_set_light_factor(double factor) {
  LIGHT_FACTOR = factor;
}
/*
void g3d_set_light() {
  GLdouble light_position[] = { 20.0, -60.0, 100.0, 1.0 };
  GLdouble light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
  GLdouble light_specular[] = { 0.1, 0.1, 0.1, 1.0 };
  double x1,y1,x2,y2,z1,z2,xmil=0.,ymil=0.,zmil=0.,ampl=0.,xampl=0.,yampl=0.,zampl=0.;
  p3d_vector4 Xc,Xw;
  G3D_Window *win = g3d_get_cur_win();

  calc_cam_param(win,Xc,Xw);

#ifdef HRI_PLANNER
  if(win->win_perspective){
    p3d_jnt *jntPt =  PSP_ROBOT->o[PSP_ROBOT->cam_body_index]->jnt;
    Xw[0]=PSP_ROBOT->cam_pos[0];
    Xw[1]=PSP_ROBOT->cam_pos[1];
    Xw[2]=PSP_ROBOT->cam_pos[2];
    Xw[3]=1;
    p3d_matvec4Mult(jntPt->abs_pos,Xw,Xc);
  }

#endif

  if(p3d_get_desc_number(P3D_ENV)) {
    p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
    xmil = (x2 + x1) / 2.;
    ymil = (y2 + y1) / 2.;
    zmil = (z2 + z1) / 2.;
    xampl = (x2 - x1) / 2.;
    yampl = (y2 - y1) / 2.;
    zampl = (z2 - z1) / 2.;
    ampl = 1.5 * sqrt(xampl * xampl + yampl * yampl + zampl * zampl);
    //   light_position[0]=xmil; light_position[1]=ymil; light_position[2]=zmil+0.5*zampl;
    light_position[0] = Xc[0];
    light_position[1] = Xc[1];
    light_position[2] = Xc[2];

  }
#ifdef HRI_PLANNER
	if(win->win_perspective){// && (win->draw_mode != NORMAL)){
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT1);
    return;
  }
#endif
#ifdef PLANAR_SHADOWS
  light_position[0]= win->lightPosition[0];
  light_position[1]= win->lightPosition[1];
  light_position[2]= win->lightPosition[2];
  light_position[3]= win->lightPosition[3];
#endif

  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 2./ampl);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}*/

//! @ingroup graphic
//! Sets the default light parameters.
void g3d_set_light()
{
  G3D_Window *win = g3d_get_cur_win();
  GLfloat light_ambient[4] = { 0.3f, 0.3f, 0.3f, 1.0f };
  GLfloat light_diffuse[4] = { 0.4f, 0.4f, 0.4f, 1.0f };
  GLfloat light_specular[4]= { 0.9f, 0.9f, 0.9f, 1.0f };

  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_POSITION, win->lightPosition);
}

//! @ingroup graphic
//! Sets the light parameters for things that will be displayed in the shadow.
void g3d_set_dim_light()
{
  G3D_Window *win = g3d_get_cur_win();

  GLfloat light_ambient[4] = { 0.3f, 0.2f, 0.2f, 1.0f };
  GLfloat light_diffuse[4] = { 0.3f, 0.3f, 0.3f, 1.0f };
  GLfloat light_specular[4]= { 0.5f, 0.5f, 0.5f, 1.0f };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, win->lightPosition);
}

//! @ingroup graphic
//! Sets the default material parameters for OpenGL.
void g3d_set_default_material()
{
  GLfloat mat_ambient[4] = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat mat_diffuse[4] = { 0.4f, 0.4f, 0.4f, 1.0f };
  GLfloat mat_specular[4]= { 0.8f, 0.8f, 0.8f, 1.0f };
  GLfloat mat_emission[4]= { 0.2f, 0.2f, 0.2f, 1.0f };
  GLfloat shininess = 80.0f;
  
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}

//! @ingroup graphic
//! Set the material parameters for things that are in the shadow (floor or wall part) for OpenGL.
void g3d_set_shade_material()
{
  GLfloat mat_ambient[4]    = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat mat_diffuse[4]    = { 0.4f, 0.4f, 0.4f, 1.0f };
  GLfloat mat_specular[4]   = { 0.2f, 0.2f, 0.2f, 1.0f };
  GLfloat mat_emission[4]   = { 0.2f, 0.2f, 0.2f, 1.0f };
  GLfloat shininess = 40.0f;
    
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}

static void
g3d_draw_win(G3D_Window *win) {
  p3d_vector4 Xc,Xw;
  p3d_vector4 up;


  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas);

  G3D_WINDOW_CUR = win;

  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));

  glClearColor(win->bg[0],win->bg[1],win->bg[2],.0);
//   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


//   if(win->GOURAUD) {
//     glShadeModel(GL_SMOOTH);
//   } else {
//     glShadeModel(GL_FLAT);
//   }

  calc_cam_param(win,Xc,Xw);

  p3d_matvec4Mult(*win->cam_frame,win->up,up);

  glPushMatrix();
  gluLookAt(Xc[0],Xc[1],Xc[2],Xw[0],Xw[1],Xw[2],up[0],up[1],up[2]);


	//   if(G3D_MODIF_VIEW) {
	//     glPushMatrix();
	//     glTranslatef(win->x,win->y,win->z);
	//     g3d_draw_frame();
	//     glPopMatrix();
	//   }


  if(win->fct_draw) (*win->fct_draw)();
  if(win->fct_draw2) (*win->fct_draw2)();

  glPopMatrix();

  //if (win->win_perspective)//G3D_REFRESH_PERSPECTIVE)
  glXSwapBuffers(fl_display,fl_get_canvas_id(ob));
}


static int
canvas_expose(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud) {
  G3D_Window *g3dwin = (G3D_Window *)ud;

  if(glXGetCurrentContext() != fl_get_glcanvas_context(ob))
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));


  glViewport(0,0,(GLint)w,(GLint)h);
  glClearColor(g3dwin->bg[0],g3dwin->bg[1],g3dwin->bg[2],.0);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(40.0,(GLdouble)w/(GLdouble)h,g3dwin->size/1000.0,1000.0*g3dwin->size);

  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();


  /* glEnable(GL_CULL_FACE); */
  /*   glCullFace(GL_BACK); */
  /*   glFrontFace(GL_CCW); */

  /** on desactive tout mode OpenGL inutile ***/
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_ALPHA_TEST);

  glEnable(GL_DEPTH_TEST);
  if(g3dwin->GOURAUD) {
    glShadeModel(GL_SMOOTH);
  } else {
    glShadeModel(GL_FLAT);
  }

  glXWaitX(); /*** jean-gerard ***/
  g3d_draw_win(g3dwin);
  glXWaitGL();

  return(TRUE);
}



//! Moves the camera in its look direction while removing the motions along Z axis.
//! \param d the length of the desired camera motion
inline void g3d_move_win_camera_forward( G3D_Window *win, float d )
{
	win->x = win->x - cos(win->az)*d;
	win->y = win->y - sin(win->az)*d;
}


//! \param d the length of the desired camera motion
inline void g3d_move_win_camera_sideways( G3D_Window *win, float d )
{
	win->x = win->x + sin(win->az)*d;
	win->y = win->y - cos(win->az)*d;
}

//! Rotates the camera around Z axis
//! \param d the angle of the desired camera rotation
inline void g3d_rotate_win_camera_rz( G3D_Window *win, float d )
{
	win->az = win->az - d;
}

//! Performs a camera zoom.
//! \param d the "distance" of the zoom
inline void g3d_zoom_win_camera( G3D_Window *win, float d )
{
	win->zo = win->zo - d;
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
  p3d_matvec4Mult(*win->cam_frame, win->up, up);
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
  double translation_step= 10*g3dwin->size/w;
  double rotation_step = 0.1;
  double zoom_step = 1;


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
					g3d_move_win_camera_forward( g3dwin, 0.1*translation_step);
				else
					g3d_move_win_camera_forward( g3dwin, translation_step);
        break;
			case XK_d:
				if(shift_key_pressed)
					g3d_move_win_camera_forward( g3dwin, -0.1*translation_step);
				else
					g3d_move_win_camera_forward( g3dwin, -translation_step);
        break;
			case XK_s:
				if(shift_key_pressed)
					g3d_move_win_camera_sideways( g3dwin, 0.1*translation_step);
				else
					g3d_move_win_camera_sideways( g3dwin, translation_step);
        break;
			case XK_f:
				if(shift_key_pressed)
					g3d_move_win_camera_sideways( g3dwin, -0.1*translation_step);
				else
					g3d_move_win_camera_sideways( g3dwin, -translation_step);
        break;
			case XK_t:
				if(shift_key_pressed)
					g3dwin->z+= 0.1*translation_step;
				else
					g3dwin->z+= translation_step;
        break;
			case XK_g:
				if(shift_key_pressed)
					g3dwin->z-= 0.1*translation_step;
				else
					g3dwin->z-= translation_step;
        break;
			case XK_r:
				if(shift_key_pressed)
					g3d_rotate_win_camera_rz( g3dwin, -0.1*rotation_step);
				else
					g3d_rotate_win_camera_rz( g3dwin, -rotation_step);
        break;
			case XK_w:
				if(shift_key_pressed)
					g3d_rotate_win_camera_rz( g3dwin, 0.1*rotation_step);
				else
					g3d_rotate_win_camera_rz( g3dwin, rotation_step);
        break;
			case XK_v:
				if(shift_key_pressed)
					g3d_zoom_win_camera( g3dwin, 0.1*zoom_step);
				else
					g3d_zoom_win_camera( g3dwin, zoom_step);
        break;
			case XK_c:
				if(shift_key_pressed)
					g3d_zoom_win_camera( g3dwin, -0.1*zoom_step);
				else
					g3d_zoom_win_camera( g3dwin, -zoom_step);
        break;
#ifdef PLANAR_SHADOWS
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
					g3dwin->lightPosition[0]+= 0.1*translation_step;
				else
					g3dwin->lightPosition[0]+= translation_step;
				g3d_build_shadow_matrices(g3dwin);
        break;
			case XK_k:
				if(shift_key_pressed)
					g3dwin->lightPosition[0]-= 0.1*translation_step;
				else
					g3dwin->lightPosition[0]-= translation_step;
				g3d_build_shadow_matrices(g3dwin);
        break;
			case XK_j:
				if(shift_key_pressed)
					g3dwin->lightPosition[1]+= 0.1*translation_step;
				else
					g3dwin->lightPosition[1]+= translation_step;
				g3d_build_shadow_matrices(g3dwin);
        break;
			case XK_l:
				if(shift_key_pressed)
					g3dwin->lightPosition[1]-= 0.1*translation_step;
				else
					g3dwin->lightPosition[1]-= translation_step;
				g3d_build_shadow_matrices(g3dwin);
        break;
			case XK_p:
				if(shift_key_pressed)
					g3dwin->lightPosition[2]+= 0.1*translation_step;
				else
					g3dwin->lightPosition[2]+= translation_step;
				g3d_build_shadow_matrices(g3dwin);
        break;
			case XK_m:
				if(shift_key_pressed)
					g3dwin->lightPosition[2]-= 0.1*translation_step;
				else
					g3dwin->lightPosition[2]-= translation_step;
				g3d_build_shadow_matrices(g3dwin);
        break;
#endif
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
						g3dwin->zo = (zo * i)/w + zo;
						if(g3dwin->zo < .0) g3dwin->zo = .0;
          }
          break;
        case MOUSE_BTN_CENTER: /* angle */
          g3dwin->az = (-2*GAIN_AZ * i)/w + az;
          if(g3dwin->az < .0) g3dwin->az = 2*M_PI + g3dwin->az;
          if(g3dwin->az > 2*M_PI) g3dwin->az = g3dwin->az - 2*M_PI;

          g3dwin->el = (GAIN_EL * j)/w + el;
          if(g3dwin->el < -M_PI/2.0) g3dwin->el = -M_PI/2.0;
          if(g3dwin->el > M_PI/2.0) g3dwin->el = M_PI/2.0;
          break;
        case MOUSE_BTN_RIGHT:  /* origin esf*/
          rotinc = (-GAIN_AZ * i)/w;
          incinc = (-GAIN_EL * j)/w;
          /* rotation effects */
          g3dwin->x = x + 2.0*zo*cos(el)*sin(rotinc/2.0)*sin(az+rotinc/2.0);
          g3dwin->y = y - 2.0*zo*cos(el)*sin(rotinc/2.0)*cos(az+rotinc/2.0);
          g3dwin->az = az + rotinc;

          if(g3dwin->az < .0)
            g3dwin->az = 2*M_PI;
          if(g3dwin->az > 2*M_PI)
            g3dwin->az = .0;
          x_aux=g3dwin->x;
          y_aux=g3dwin->y;
          az_aux=g3dwin->az;
          /* incline effects */
          g3dwin->z = z - 2.0 *zo*sin(incinc/2.0)*cos(el+incinc/2.0);
          g3dwin->x = x_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*cos(az_aux);
          g3dwin->y = y_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*sin(az_aux);
          g3dwin->el = el + incinc;
          if(g3dwin->el < -M_PI/2)
            g3dwin->el = -M_PI/2;
          if(g3dwin->el > M_PI/2)
            g3dwin->el = M_PI/2;
          break;
          /* deplacement du  point central de l'image */
          /* - bouton gauche et central -> x          */
          /* - bouton droit et central  -> y          */
          /* - bouton gauche et droit   -> z          */
        case MOUSE_BTN_LEFT + MOUSE_BTN_CENTER: /* origin x */
          g3dwin->x = x + j*g3dwin->size/w;
          /*       g3dwin->x = x + i*g3dwin->size/h; */
          break;
        case MOUSE_BTN_CENTER + MOUSE_BTN_RIGHT: /* origin y */
          g3dwin->y = y + j*g3dwin->size/w;
          /*       g3dwin->y = y + i*g3dwin->size/h; */
          break;
        case MOUSE_BTN_LEFT + MOUSE_BTN_RIGHT: /* origin z */
          g3dwin->z = z + j*g3dwin->size/w;
          /*       g3dwin->z = z + i*g3dwin->size/h; */
          break;
        case MOUSE_BTN_LEFT + MOUSE_BTN_CENTER + MOUSE_BTN_RIGHT: /* reset up */
          g3dwin->up[0]=.0;
          g3dwin->up[1]=.0;
          g3dwin->up[2]=1.0;
          break;
      }
      glXWaitX(); /*** jean-Gerard ***/
      g3d_draw_win(g3dwin);
      glXWaitGL();/*** jean-gerard ***/
      break;

		case ButtonPress:
      fl_get_win_mouse(win,&i0,&j0,&key);
      x=g3dwin->x;
      y=g3dwin->y;
      z=g3dwin->z;
      zo=g3dwin->zo;
      el=g3dwin->el;
      az=g3dwin->az;

      //Mokhtar scroll zoom
      strippedKey = key & (KBD_SHIFT_ON);
      if (strippedKey == KBD_SHIFT_ON){//if shift key is pressed
				zo_inc = floor(g3dwin->size);//slow zoom
      }else{
				zo_inc = floor(g3dwin->size);//big zoom
      }
      switch (xev->xbutton.button) {
				case 1:{//click left button
          // activate picking, picking will be cancelled if the mouse is used to change the camera view
          // (and not to pick a vertex)
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
         // picking = TRUE;
          picking_x = i0;
          picking_y = j0;
          break;
				}
				case 4:{//forward scroll (forward zoom)
          g3dwin->zo += zo_inc;
          draw = 1;
          if(g3dwin->zo < 0) g3dwin->zo = 0;
          break;
				}
				case 5:{//back scroll (back zoom)
          g3dwin->zo -= zo_inc;
          draw = 1;
          if(g3dwin->zo < 0) g3dwin->zo = 0;
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

  G3D_MODIF_VIEW = TRUE;

  switch(xev->type) {
			
		case MotionNotify:
			fl_get_win_mouse(win,&i,&j,&key);
//			printf("i = %f\n",(double)i);
//			printf("j = %f\n",(double)j);
//			printf("%d\n",key);
			i -= i0; j -= j0;
			switch(key) {
				case 260: /* Select Object and Displace in X */
					//	move_robot(idr,0,j,g3dwin->size/h);
					break;
				case 516: /* Select Object and Displace in Y */
					//	move_robot(idr,1,i,g3dwin->size/w);
					break;
				case 1028:/* Select Object and Displace in Z */
					//	move_robot(idr,2,j,g3dwin->size/h);
					break;
				case 256: /* zoom */
				case 272:
				case 258:
				case 274:
				case 8464:
				case 16656:
					g3dwin->zo = (zo * i)/w + zo;
					if(g3dwin->zo < .0) g3dwin->zo = .0;
					break;
				case 512: /* angle */
				case 514:
				case 528:
				case 530:
				case 8720:
				case 16912:
				case 8704:
					g3dwin->az = (-2*GAIN_AZ * i)/w + az;
					if(g3dwin->az < .0) g3dwin->az = 2*M_PI;
					if(g3dwin->az > 2*M_PI) g3dwin->az =.0;
					g3dwin->el = (GAIN_EL * j)/w + el;
					if(g3dwin->el < -M_PI/2.0) g3dwin->el = -M_PI/2.0;
					if(g3dwin->el > M_PI/2.0) g3dwin->el = M_PI/2.0;
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
					g3dwin->x = x + 2.0*zo*cos(el)*sin(rotinc/2.0)*sin(az+rotinc/2.0);
					g3dwin->y = y - 2.0*zo*cos(el)*sin(rotinc/2.0)*cos(az+rotinc/2.0);
					g3dwin->az = az + rotinc;

					if(g3dwin->az < .0)
						g3dwin->az = 2*M_PI;
					if(g3dwin->az > 2*M_PI)
						g3dwin->az = .0;
					x_aux=g3dwin->x;
					y_aux=g3dwin->y;
					az_aux=g3dwin->az;
					/* incline effects */
					g3dwin->z = z - 2.0 *zo*sin(incinc/2.0)*cos(el+incinc/2.0);
					g3dwin->x = x_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*cos(az_aux);
					g3dwin->y = y_aux
					+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*sin(az_aux);
					g3dwin->el = el + incinc;
					if(g3dwin->el < -M_PI/2)
						g3dwin->el = -M_PI/2;
					if(g3dwin->el > M_PI/2)
						g3dwin->el = M_PI/2;
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
					g3dwin->x = x + j*g3dwin->size/w;
					/*       g3dwin->x = x + i*g3dwin->size/h; */
					break;
				case 1536: /* origin y */
				case 1552:
				case 1538:
				case 9744:
				case 17936:
					g3dwin->y = y + j*g3dwin->size/w;
					/*       g3dwin->y = y + i*g3dwin->size/h; */
					break;
				case 1280: /* origin z */
				case 1282:
				case 1296:
				case 9488:
				case 17680:
					g3dwin->z = z + j*g3dwin->size/w;
					/*       g3dwin->z = z + i*g3dwin->size/h; */
					break;
				case 1792: /* reset up */
					g3dwin->up[0]=.0;
					g3dwin->up[1]=.0;
					g3dwin->up[2]=1.0;
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
			x=g3dwin->x;
			y=g3dwin->y;
			z=g3dwin->z;
			zo=g3dwin->zo;
			el=g3dwin->el;
			az=g3dwin->az;
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
		p3d_matvec4Mult(*G3D_WINDOW_CUR->cam_frame, G3D_WINDOW_CUR->up, up);
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
		x = g3dwin->x;
		y = g3dwin->y;
		z = g3dwin->z;
		zo = g3dwin->zo;
		el = g3dwin->el;
		az = g3dwin->az;
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
			//	move_robot(idr,0,j,g3dwin->size/h);
			break;
		case 516: /* Select Object and Displace in Y */
			//	move_robot(idr,1,i,g3dwin->size/w);
			break;
		case 1028:/* Select Object and Displace in Z */
			//	move_robot(idr,2,j,g3dwin->size/h);
			break;
		case 256: /* zoom */
		case 272:
		case 258:
		case 274:
		case 8464:
		case 16656:
			g3dwin->zo = (zo * i) / w + zo;
			if (g3dwin->zo < .0)
			{

				g3dwin->zo = .0;
			}
			//			std::cout << "g3dwin->zo = "<< g3dwin->zo << std::endl;
			break;
		case 512: /* angle */
		case 514:
		case 528:
		case 530:
		case 8720:
		case 16912:
			g3dwin->az = (-2 * GAIN_AZ * i) / w + az;
			if (g3dwin->az < .0)
				g3dwin->az = 2 * M_PI;
			if (g3dwin->az > 2 * M_PI)
				g3dwin->az = .0;
			g3dwin->el = (GAIN_EL * j) / w + el;
			if (g3dwin->el < -M_PI / 2.0)
				g3dwin->el = -M_PI / 2.0;
			if (g3dwin->el > M_PI / 2.0)
				g3dwin->el = M_PI / 2.0;
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
			g3dwin->x = x + 2.0 * zo * cos(el) * sin(rotinc / 2.0) * sin(az
					+ rotinc / 2.0);
			g3dwin->y = y - 2.0 * zo * cos(el) * sin(rotinc / 2.0) * cos(az
					+ rotinc / 2.0);
			g3dwin->az = az + rotinc;

			if (g3dwin->az < .0)
				g3dwin->az = 2 * M_PI;
			if (g3dwin->az > 2 * M_PI)
				g3dwin->az = .0;
			x_aux = g3dwin->x;
			y_aux = g3dwin->y;
			az_aux = g3dwin->az;
			/* incline effects */
			g3dwin->z = z - 2.0 * zo * sin(incinc / 2.0) * cos(el + incinc
					/ 2.0);
			g3dwin->x = x_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
					/ 2.0) * cos(az_aux);
			g3dwin->y = y_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
					/ 2.0) * sin(az_aux);
			g3dwin->el = el + incinc;
			if (g3dwin->el < -M_PI / 2)
				g3dwin->el = -M_PI / 2;
			if (g3dwin->el > M_PI / 2)
				g3dwin->el = M_PI / 2;
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
			g3dwin->x = x + j * g3dwin->size / w;
			/*       g3dwin->x = x + i*g3dwin->size/h; */
			break;
		case 1536: /* origin y */
		case 1552:
		case 1538:
		case 9744:
		case 17936:
			g3dwin->y = y + j * g3dwin->size / w;
			/*       g3dwin->y = y + i*g3dwin->size/h; */
			break;
		case 1280: /* origin z */
		case 1282:
		case 1296:
		case 9488:
		case 17680:
			g3dwin->z = z + j * g3dwin->size / w;
			/*       g3dwin->z = z + i*g3dwin->size/h; */
			break;
		case 1792: /* reset up */
			g3dwin->up[0] = .0;
			g3dwin->up[1] = .0;
			g3dwin->up[2] = 1.0;
			break;
		}
	}
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
  win->displayJoints= !win->displayJoints;
  g3d_draw_win(win);
}

#ifdef PLANAR_SHADOWS
static void
button_light(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->enableLight= !win->enableLight;
  g3d_draw_win(win);
}

static void
button_floor(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->displayFloor= !win->displayFloor;
  g3d_draw_win(win);
}

static void
button_tiles(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->displayTiles= !win->displayTiles;
  g3d_draw_win(win);
}

static void
button_shadows(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->displayShadows= !win->displayShadows;
  g3d_draw_win(win);
}

static void
button_walls(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  win->displayWalls = !win->displayWalls;
  g3d_draw_win(win);
}
#endif

static void
button_copy(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_copy_win(win);
}


static void
button_view_save(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_save_win_camera(win);
  g3d_draw_win(win);
}



static void
button_view_restore(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  g3d_restore_win_camera(win);
  g3d_draw_win(win);
}



static void
button_view_fil(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;

  if (win->FILAIRE) {
    win->FILAIRE = 0;
  } else {
    win->FILAIRE = 1;
    win->GOURAUD = 0;
  }
  win->list = -1;

  g3d_draw_win(win);
}


static void
button_view_cont(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->CONTOUR) {
    win->CONTOUR = 0;
  } else {
    win->CONTOUR = 1;
    win->GOURAUD = 0;
  }
  win->list = -1;
  g3d_draw_win(win);
}

static void
button_view_ghost(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->GHOST) {
    win->GHOST = 0;
  } else {
    win->GHOST = 1;
    win->GOURAUD = 0;
  }
  win->list = -1;
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
  if (win->BB) {
    win->BB = 0;
  } else {
    win->BB = 1;
    win->GOURAUD = 0;
  }
  win->list = -1;
  g3d_draw_win(win);
}

static void
button_view_gour(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  if (win->GOURAUD) {
    win->GOURAUD = 0;
  } else {
    win->GOURAUD = 1;
  }
  win->list = -1;
  g3d_draw_win(win);
}


static void
button_freeze(FL_OBJECT *ob, long data) {
  G3D_Window *win = (G3D_Window *)data;
  int i;
  if (win->ACTIVE) {
    win->ACTIVE = 0;
  } else {
    win->ACTIVE = 1;
  }
  for(i = 0;i < 1000;i++)
    g3d_draw_win(win);
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
		recalc_mouse_param(win,Xcnew,Xwnew);
		recalc_cam_up(win,matr);
	} else {
		calc_cam_param(win,Xc,Xw);
		recalc_mouse_param(win,Xc,Xw);
		recalc_cam_up(win,*win->cam_frame);
		win->cam_frame = &Id;
	}

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
  win->size       = size;
  win->FILAIRE = 0;
  win->CONTOUR = 0;
  win->GOURAUD = 0;
  win->ACTIVE = 1;
  win->list = -1;
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

      g3dwin->zo = (zo * i)/w + zo;
      if(g3dwin->zo < .0) g3dwin->zo = .0;

      g3d_draw_win(g3dwin);

      break;

    case 12: /* Angle ref: Mouse move + Middle mouse button down */

      i = xpos - i0;
      j = ypos - j0;
      w = *((int*)data);

      g3dwin->az = (-3*GAIN_AZ * i)/w + az;
      if(g3dwin->az < .0) g3dwin->az += 2*M_PI;
      if(g3dwin->az > 2*M_PI) g3dwin->az -= 2*M_PI;
      g3dwin->el = (GAIN_EL * j)/w + el;
      if(g3dwin->el < -M_PI/2.0) g3dwin->el = -M_PI/2.0;
      if(g3dwin->el > M_PI/2.0) g3dwin->el = M_PI/2.0;


      g3d_draw_win(g3dwin);

      break;

    case 13:  /* Origin esf : Mouse move + Right mouse button down */

      i = xpos - i0;
      j = ypos - j0;
      w = *((int*)data);

      rotinc = (+2*GAIN_AZ * i)/w;
      incinc = (-2*GAIN_EL * j)/w;
      /* rotation effects */
      g3dwin->x = x + 2.0*zo*cos(el)*sin(rotinc/2.0)*sin(az+rotinc/2.0);
      g3dwin->y = y - 2.0*zo*cos(el)*sin(rotinc/2.0)*cos(az+rotinc/2.0);
      g3dwin->az = az + rotinc;

      if(g3dwin->az < .0)
        g3dwin->az = 2*M_PI;
      if(g3dwin->az > 2*M_PI)
        g3dwin->az = .0;
      x_aux=g3dwin->x;
      y_aux=g3dwin->y;
      az_aux=g3dwin->az;
      /* incline effects */
      g3dwin->z = z - 2.0 *zo*sin(incinc/2.0)*cos(el+incinc/2.0);
      g3dwin->x = x_aux
			+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*cos(az_aux);
      g3dwin->y = y_aux
			+ 2.0*zo*sin(incinc/2.0)*sin(el+incinc/2.0)*sin(az_aux);
      g3dwin->el = el + incinc;
      if(g3dwin->el < -M_PI/2)
        g3dwin->el = -M_PI/2;
      if(g3dwin->el > M_PI/2)
        g3dwin->el = M_PI/2;

      g3d_draw_win(g3dwin);

      break;

      /* not yet implemented for Win32:

			 deplacement du  point central de l'image
			 - bouton gauche et central -> x
			 - bouton droit et central  -> y
			 - bouton gauche et droit   -> z
			 case 768:    origin x
			 g3dwin->x = x + j*g3dwin->size/w;
			 g3dwin->x = x + i*g3dwin->size/h;
			 break;
			 case 1536:    origin y
			 g3dwin->y = y + j*g3dwin->size/w;
			 g3dwin->y = y + i*g3dwin->size/h;
			 break;
			 case 1280:    origin z
			 g3dwin->z = z + j*g3dwin->size/w;
			 g3dwin->z = z + i*g3dwin->size/h;
			 break;
			 case 1792:    reset up
			 g3dwin->up[0]=.0;
			 g3dwin->up[1]=.0;
			 g3dwin->up[2]=1.0;
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
      x=g3dwin->x;
      y=g3dwin->y;
      z=g3dwin->z;
      zo=g3dwin->zo;
      el=g3dwin->el;
      az=g3dwin->az;
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

			 glCullFace(GL_BACK);
			 glEnable(GL_CULL_FACE);
			 =     glEnable(GL_DEPTH_TEST);
			 =     if(w->GOURAUD){
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


  glClearColor(win->bg[0],win->bg[1],win->bg[2],.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(win->GOURAUD) {
    glShadeModel(GL_SMOOTH);
  } else {
    glShadeModel(GL_FLAT);
  }

  calc_cam_param(win,Xc,Xw);
  p3d_matvec4Mult(*win->cam_frame,win->up,up);

  glPushMatrix();

  glLoadIdentity();

  gluLookAt(Xc[0],Xc[1],Xc[2],Xw[0],Xw[1],Xw[2],up[0],up[1],up[2]);

  if(G3D_MODIF_VIEW) {

    glPushMatrix();
    glTranslatef(win->x,win->y,win->z);
    g3d_draw_frame();
    glPopMatrix();
  }



  if(win->fct_draw) (*win->fct_draw)();



  glPopMatrix();
  /*  glXWaitGL();    */
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
#ifdef HRI_PLANNER
    if(w->win_perspective)
      g3d_refresh_win(w);
    else
#endif
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
    if (w->ACTIVE == 1) {
#ifdef HRI_PLANNER
if(w->win_perspective)
    g3d_refresh_win(w);
else
#endif
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
  win->size = size;

  glViewport(0, 0, (GLsizei)w, (GLsizei)h);

  g3d_set_win_camera(win, .0,.0,.0,5*size, INIT_AZ, INIT_EL,.0,.0,1.0);
  g3d_save_win_camera(win);
}


void
g3d_resize_allwin_active(float w, float h, float size) {
  G3D_Window *win = G3D_WINDOW_LST;
  while (win) {
    if (win->ACTIVE == 1) {
      g3d_resize_win(win, w, h, size);
    }
    win = win->next;
  }
}


void
g3d_set_win_bgcolor(G3D_Window *win, float r, float v, float b) {
  win->bg[0] = r;
  win->bg[1] = v;
  win->bg[2] = b;
}

void
g3d_set_win_floor_color(G3D_Window *win, float r, float v, float b) {
  win->floorColor[0] = r;
  win->floorColor[1] = v;
  win->floorColor[2] = b;
}

void
g3d_set_win_wall_color(G3D_Window *win, float r, float v, float b) {
  win->wallColor[0] = r;
  win->wallColor[1] = v;
  win->wallColor[2] = b;
}


void
g3d_set_win_camera(G3D_Window *win, float ox,float oy, float oz,
                   float dist, float az, float el,
                   float up0, float up1, float up2) {
  win->x = ox;
  win->y = oy;
  win->z = oz;
  win->zo = dist;
  win->az = az, win->el = el;
  win->up[0] = up0;
  win->up[1] = up1, win->up[2] = up2;
  win->up[3]=0.0;
}

void
g3d_set_win_center(G3D_Window *win, float ox,float oy, float oz) {
  win->x = ox;
  win->y = oy;
  win->z = oz;
}


void
g3d_save_win_camera(G3D_Window *win) {
  int i;

  win->sx = win->x;
  win->sy = win->y;
  win->sz = win->z;
  win->szo = win->zo;
  win->saz = win->az, win->sel = win->el;
  for(i=0;i<4;i++) win->sup[i] = win->up[i];
}

void
g3d_load_saved_camera_params(double* params)
{
  G3D_Window *win = g3d_get_win_by_name((char*)"Move3D");
  int i;
  
  win->sx = params[0];
  win->sy = params[1];
  win->sz = params[2];
  win->szo = params[3];
  win->saz = params[4], win->sel = params[5];
  for(i=0;i<4;i++) win->sup[i] = params[6+i];
}


void
g3d_restore_win_camera(G3D_Window *win) {
  int i;

  win->x = win->sx;
  win->y = win->sy;
  win->z = win->sz;
  win->zo = win->szo;
  win->az = win->saz, win->el = win->sel;
  for(i=0;i<4;i++) win->up[i] = win->sup[i];
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
    recalc_mouse_param(win,Xcnew,Xwnew);
    recalc_cam_up(win,matr);
  } else {
    calc_cam_param(win,Xc,Xw);
    recalc_mouse_param(win,Xc,Xw);
    recalc_cam_up(win,*win->cam_frame);
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
    w->FILAIRE = 0;
    w->CONTOUR = 0;
    w->GOURAUD = 0;
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


void g3d_draw_frame(void) {
//   GLdouble mat_ambient_diffuse[4]= { 0., .0, .0, 1.0 };
  double a= G3D_WINDOW_CUR->size;
  double a1,a9;

  a1 = .1 * a;
  a9 = .9 * a;

  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);
  glColor3d(0.,0.,0.);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glVertex3d(.0, .0, .0);
  glVertex3d(.0, .0, a);
  glVertex3d(.0, .0, .0);
  glVertex3d(a, .0, .0);
  glVertex3d(.0, .0, .0);
  glVertex3d(.0, a, .0);
  glEnd();
  glLineWidth(1.0);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glDisable(GL_CULL_FACE);
  glBegin(GL_POLYGON);
  glVertex3d(.0, .0, a);
  glVertex3d(-a1, .0, a9);
  glVertex3d(a1, .0, a9);
  glEnd();

  glBegin(GL_POLYGON);
  glVertex3d(a, .0, .0);
  glVertex3d(a9, .0, -a1);
  glVertex3d(a9, .0, a1);
  glEnd();

  glBegin(GL_POLYGON);
  glVertex3d(.0, a, .0);
  glVertex3d(.0, a9, -a1);
  glVertex3d(.0, a9, a1);
  glEnd();
  /*  glEnable(GL_CULL_FACE);  */
}


G3D_Window *g3d_get_cur_win(void) {
  return(G3D_WINDOW_CUR);
}

G3D_Window *g3d_get_win_by_name(char *s)
{
	G3D_Window *w = G3D_WINDOW_LST;
	while(w)
	{
		if (strcmp(s,w->name)==0)
			return w;
		w = w->next;
	}
	return NULL;
}


G3D_Window *g3d_get_cmc_win(void)
/* rendre la fenetre ou on travaille avec la camera mobile maintenant */
{
  return(G3D_WINDOW_CMC);
}

/*          Static Functions                */

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

static void
get_pos_cam_matrix(G3D_Window *win, p3d_matrix4 Transf) {
  /* Caution: ici on change les parametres de translation de la */
  /* matrix, les rest des elementes doivent etre initialises    */
  /* dans la fonction qu'appel                                  */
  Transf[0][3] = win->zo * (cos(win->az)*cos(win->el));
  Transf[1][3] = win->zo * (sin(win->az)*cos(win->el));
  Transf[2][3] = win->zo * sin(win->el);
}

/* fonction pour calculer les parametres de position de la      */
/* camera de la facon necessaire pour openGL                    */
static void
calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) {
  static p3d_matrix4 Txc = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  p3d_matrix4 m_aux;
  p3d_vector4 Xx;

#ifdef HRI_PLANNER
  if (win->point_of_view==0){
#endif
		get_lookat_vector(win, Xx);
		get_pos_cam_matrix(win, Txc);

		p3d_mat4Mult(*win->cam_frame,Txc,m_aux);
		p3d_matvec4Mult(m_aux,Xx,Xc);
		p3d_matvec4Mult(*win->cam_frame,Xx,Xw);
#ifdef HRI_PLANNER
	}
  else {
    //Modified Luis
    if (PSP_ROBOT){
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
#endif
}

/* fonction pour recalculer le vector 'up' de la camera */
/* quand on change la reference                         */
static void
recalc_cam_up(G3D_Window *win, p3d_matrix4 transf) {
  p3d_vector4 v_up;
  int i;

  p3d_matvec4Mult(transf,win->up,v_up);
  for(i=0;i<4;i++) win->up[i] = v_up[i];
}

/* fonction qui change les parametres de position de la       */
/* camera qui son controles pour la souris quand on change    */
/* la reference                                               */
static void
recalc_mouse_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) {
  double incx,incy,incz,incd;
  double azim,elev;

  win->x = Xw[0];
  win->y = Xw[1];
  win->z = Xw[2];

  incx = Xc[0] - Xw[0];
  incy = Xc[1] - Xw[1];
  incz = Xc[2] - Xw[2];
  incd = sqrt(SQR(incx)+SQR(incy));
  azim = atan2(incy,incx);
  elev = atan2(incz,incd);
  /* set azim and elev in right range */
  azim = angle_limit_2PI(azim);

  if(elev>M_PI/2) {
    elev = M_PI - elev;
    if(azim>M_PI) azim -= M_PI;
    else azim += M_PI;
  }
  if(elev<-M_PI/2) {
    elev = -M_PI - elev;
    if(azim>M_PI) azim -= M_PI;
    else azim += M_PI;
  }
  win->az = azim;
  win->el = elev;
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
  gluPerspective(40.0,(GLdouble)winw/(GLdouble)winh,w->size/100.0,100.0*w->size);

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

//! @ingroup graphic 
//! Saves the current OpenGL pixel buffer as a ppm (PortablePixMap) image file (uncompressed format).
//! In other words: takes a screenshot of the active OpenGL window.
//! \param filename name of the image file where to save the pixel buffer
//! \return 1 in case of success, 0,otherwise
int g3d_export_OpenGL_display(char *filename)
{
  size_t length;
  unsigned int i,j, width, height, change_name= 0;
  GLint viewport[4];
  char filename2[128]; 
  unsigned char *pixels= NULL;
  unsigned char *pixels_inv= NULL;
  FILE *file= NULL;

  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];

  pixels    = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  pixels_inv= (unsigned char*) malloc(3*width*height*sizeof(unsigned char));

  length= strlen(filename); 

  if(length < 5)
  {  change_name= 1;  }
  else if(filename[length-4]!='.' || filename[length-3]!='p' || filename[length-2]!='p' || filename[length-1]!='m')
  {  change_name= 1;  }

  if(change_name)
  {
    printf("%s: %d: file \"%s\" does not have the good extension (it should be a .ppm file).\n",__FILE__,__LINE__, filename);
    strcpy(filename2, filename);
    strcat(filename2, ".ppm");
    printf("It is renamed \"%s\".\n",filename2);
  }    
  else
  { strcpy(filename2, filename);  }  

  file= fopen(filename2,"w");


  if(file==NULL) 
  {
    printf("%s: %d: can not open \"%s\".\n",__FILE__,__LINE__,filename2);
    fclose(file);
    return 0;
  }

  glReadBuffer(GL_FRONT);

  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  // get the image pixels (from (0,0) position):
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

  // glReadPixels returns an upside-down image.
  // we have to first flip it
  // NB: in pixels the 3 colors of a pixel follows each other immediately (RGBRGBRGB...RGB).
  for(i=0; i<width; i++)
  { 
    for(j=0; j<height; j++)
    { 
      pixels_inv[3*(i+j*width)]  = pixels[3*(i+(height-j)*width)+0];
      pixels_inv[3*(i+j*width)+1]= pixels[3*(i+(height-j)*width)+1];
      pixels_inv[3*(i+j*width)+2]= pixels[3*(i+(height-j)*width)+2];
    }
  } 

  fprintf(file, "P6\n");
  fprintf(file, "# creator: BioMove3D\n");
  fprintf(file, "%d %d\n", width, height);
  fprintf(file, "255\n");

  fwrite(pixels_inv, sizeof(unsigned char), 3*width*height, file);

  fclose(file);

  free(pixels);
  free(pixels_inv);

  return 1;
}

//! @ingroup graphic 
//! Initializes OpenGL main parameters.
void g3d_init_OpenGL()
{
  glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.2, 1.2);

  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glLineWidth(1);
}

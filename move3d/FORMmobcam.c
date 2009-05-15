#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"


extern p3d_matrix4 Id;	// KINEO-DEV :doit être déclaré dans un .h !!

static int num_bod;
static int salida;
static pp3d_matrix4 transf;

static void g3d_create_mobcam_form(void);
static void g3d_delete_mobcam_form(void);
static void CB_bodies_obj(FL_OBJECT *ob, long arg);
static void g3d_create_bodies_obj(void);
static void g3d_delete_bodies_obj(void);

static FL_FORM *MOBCAM_FORM;
static FL_OBJECT   *BODIES_OBJ[MAX_DDLS];

pp3d_matrix4 g3d_fct_mobcam_form(void)
{ 
  G3D_Window *win;
  int x,y,w,h,xmc,ymc;
  FL_OBJECT  *ob,*obje;
  int  n;

  n = num_bod;
  transf = &Id;
  win = g3d_get_cmc_win();
  obje = ((FL_OBJECT *)win->canvas);
  g3d_create_mobcam_form();
  fl_get_wingeometry(FL_ObjWin(obje),&x,&y,&w,&h);
  if((x+w)<1000) xmc = x + w + 70;
  else xmc = x - 100;
  ymc = y + h - 240;
  fl_set_form_position(MOBCAM_FORM,xmc,ymc);
  fl_show_form(MOBCAM_FORM,FL_PLACE_POSITION,TRUE,"");
  fl_deactivate_all_forms();
  fl_activate_form(MOBCAM_FORM);
  salida = 0;
  while(1) {
    ob = fl_check_forms(); /* cette fonction et fl_do_forms ne marchent pas ici */
    if(salida) {           /* donc une variable global est utilice pour sortir du bucle */
      g3d_delete_mobcam_form();
      fl_activate_all_forms();
      return(transf);
    }
  }

}

static void g3d_create_mobcam_form(void)
{int n;
 double s;
 
  n = p3d_get_robot_njnt()+1;
  s = n*20.0+60.0;
 
  MOBCAM_FORM = fl_bgn_form(FL_UP_BOX,100.0,s);
  g3d_create_bodies_obj();
  fl_end_form();
}

static void CB_bodies_obj(FL_OBJECT *ob, long arg)
{pp3d_rob r; 
 pp3d_jnt j;

 if(fl_get_button(ob)){
   r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
   j = r->joints[arg];
   transf = &(j->abs_pos);
   salida = 1;
 }
}

static void g3d_create_bodies_obj(void)
{ void CB_bodies_obj(FL_OBJECT *ob, long arg);
  char cadena[8];
  int  i,njnt,ord;
  pp3d_rob r;

  njnt = p3d_get_robot_njnt()+1;
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  fl_add_box(FL_FLAT_BOX,0.0,0.0,100.0,30.0,"CHOOSE the body which\nattach the camera to");
  fl_add_box(FL_ROUNDED_BOX,0.0,30.0,100.0,18.0,r->name);

  ord = 0;
  for(i=0;i<njnt;i++) {
    sprintf(cadena,"Body %d",i);
    BODIES_OBJ[i] = fl_add_button(FL_PUSH_BUTTON,5.0,50.0+20.0*ord,90.0,20.0,cadena);
    fl_set_object_callback(BODIES_OBJ[i],CB_bodies_obj,i);
    ord=ord+1;
  }
  num_bod = njnt;
}

static void g3d_delete_mobcam_form(void)
{
  fl_hide_form(MOBCAM_FORM);
  g3d_delete_bodies_obj();
  fl_free_form(MOBCAM_FORM);
}

static void g3d_delete_bodies_obj(void)
{ int  i,n;

  n = num_bod;
  for(i=0;i<n;i++) {
    fl_free_object(BODIES_OBJ[i]);
  }
}

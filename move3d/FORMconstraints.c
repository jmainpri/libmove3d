/* NOTE : this file does not consider modifications to handle dofs
 
*/

#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"



extern MENU_ROBOT *ROBOTS_FORM;	// KINEO-DEV :doit �tre d�clar� dans un .h !!
extern int robotSelectPositionFlag;

//extern void g3d_create_robot_form(int ir);
//extern void g3d_delete_robot_form(int ir);

//extern void update_robot_pos(pp3d_jnt j);

static MENU_CONSTRAINTS *CONSTRAINTS_FORM;
static MENU_CONSTRAINTS_SETTING *CONSTRAINTS_SETTING_FORM;

static void g3d_create_constraints_form(void); 
static void g3d_create_cntrt_list_obj(void);
static void g3d_create_cntrt_arrows_obj(void);
static void g3d_create_cntrt_new_obj(void);
static void g3d_create_cntrt_ok_obj(void);
static void CB_cntrt_list_obj(FL_OBJECT *ob, long arg);
static void CB_cntrt_arrows_obj(FL_OBJECT *ob, long arg);
static void CB_cntrt_new_obj(FL_OBJECT *ob, long arg);
static void CB_cntrt_ok_obj(FL_OBJECT *ob, long arg);
static void g3d_delete_cntrt_list_obj(void);
static void g3d_delete_cntrt_arrows_obj(void);
static void g3d_delete_cntrt_new_obj(void);
static void g3d_delete_cntrt_ok_obj(void);
static void g3d_delete_constraints_form(void);
static void g3d_create_constraints_setting_form(void); 
static void g3d_create_cntrt_set_list_obj(void);
static void g3d_create_cntrt_set_done_obj(void);
static void CB_cntrt_set_list_obj(FL_OBJECT *ob, long arg);
static void CB_cntrt_set_done_obj(FL_OBJECT *ob, long arg);
static void g3d_delete_cntrt_set_list_obj(void);
static void g3d_delete_cntrt_set_done_obj(void);
static void g3d_delete_constraints_setting_form(void);

static void g3d_one_cntrt_form(p3d_cntrt *ct, int x, int y);

static void g3d_fixed_jnt_form(p3d_cntrt *ct, int x, int y);
static void g3d_lin_rel_dofs_form(p3d_cntrt *ct, int x, int y);
static void g3d_RRPRlnk_form(p3d_cntrt *ct, int x, int y);
static void g3d_4Rlnk_form(p3d_cntrt *ct, int x, int y);
static void g3d_3RPRlnk_form(p3d_cntrt *ct, int x, int y);
static void g3d_jnt_on_ground_form(p3d_cntrt *ct, int x, int y);
static void g3d_car_front_wheels_form(p3d_cntrt *ct, int x, int y);
static void g3d_planar_closed_chain_form(p3d_cntrt *ct, int x, int y);
static void g3d_R6_arm_ik(p3d_cntrt *ct, int x, int y);
static void g3d_3R_arm_ik_form(p3d_cntrt *ct, int x, int y);

static int ncntrt_st;         /* nombre des contraintes dans le robot dans la creation de la form */
static int ncntrt_af;         /* nombre des contraintes dans le robot (affichage) */
static int ncntrt_tp = 9;     /* nombre des types de contraintes dans le setting */

static int xCF,yCF,ftCF=1;
static int CSF = 0;

/* variables pour guarder les entrees */
static int Jpasiv[MAX_ARGU_CNTRT];
static int Jactiv[MAX_ARGU_CNTRT];
static double Dval[MAX_ARGU_CNTRT];
static int Ival[MAX_ARGU_CNTRT];

/* functions for RLG */
static void g3d_create_RLG_obj(void);
static void g3d_delete_RLG_obj(void);

void g3d_kin_constraints_form(void)
{ p3d_rob *r;
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
 
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  ncntrt_st = r->cntrt_manager->ncntrts;
  if(ncntrt_st<4) ncntrt_af = 4;
  else  ncntrt_af = ncntrt_st;
   
  CONSTRAINTS_FORM = (MENU_CONSTRAINTS *) malloc(sizeof(MENU_CONSTRAINTS)); 
  g3d_create_constraints_form();
  fl_show_form(CONSTRAINTS_FORM->CNTRT_FORM,FL_PLACE_SIZE,TRUE,"KINEMATIC CONSTRAINTS");
  fl_set_button(CONSTRAINTS_FORM->OK_OBJ,0);
  fl_set_button(CONSTRAINTS_FORM->NEW_OBJ,0);
  fl_deactivate_all_forms();
  fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM); 
  fl_set_button(ROBOTS_FORM[ir].KINE_CONSTRAINTS_OBJ,0); 
}


static void g3d_create_constraints_form(void)
{ 
  CONSTRAINTS_FORM->CNTRT_FORM = fl_bgn_form(FL_UP_BOX,300.0,20.0*ncntrt_af+50.0);
  g3d_create_cntrt_list_obj();
  g3d_create_cntrt_arrows_obj();
  g3d_create_cntrt_new_obj();
  g3d_create_cntrt_ok_obj();
  g3d_create_RLG_obj();
  fl_end_form();
  if(!ftCF) fl_set_form_position(CONSTRAINTS_FORM->CNTRT_FORM,xCF-200,yCF-100);  /* regular los parametros de posicion (dependen de ncntrt_af) !!!!!!! */
}


static void CB_cntrt_list_obj(FL_OBJECT *ob, long arg)
{int val =  fl_get_button(ob);
 p3d_rob *r;
 p3d_cntrt *ct;

 r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 ct = r->cntrt_manager->cntrts[arg];
 if(val) {
   if(p3d_update_constraint(ct, 1)) {
     if (ct->enchained != NULL)
       p3d_reenchain_cntrts(ct);
     p3d_col_deactivate_one_cntrt_pairs(ct);
   }
   else {
     fl_set_button(ob,0);
   }
 }
 else {
   ct->active = 0;
   if (ct->enchained != NULL)
     p3d_unchain_cntrts(ct);
   p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
   p3d_col_activate_one_cntrt_pairs(ct);
 }
}

static void g3d_create_cntrt_list_obj(void)
{
 int i,j; 
 p3d_rob *r;
 p3d_cntrt *ct;
 char textline[50];
 char auxtext[6];
 char nart[3];

 r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

 fl_add_box(FL_DOWN_BOX,40.0,10.0,220.0,20.0*ncntrt_af,"");
 
 if(r->cntrt_manager->cntrts == NULL) {
   fl_add_text(FL_NORMAL_TEXT,85.0,30.0,140.0,20.0,"NO KINEMATIC CONSTRAINTS");
 }
 else {
   for(i=0; i<ncntrt_st; i++) {
     ct = r->cntrt_manager->cntrts[i];
     CONSTRAINTS_FORM->LIST_OBJ[i] = fl_add_button(FL_PUSH_BUTTON,10.0,10.0+20.0*i,20.0,20.0,"");
     fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[i],ct->active);
     fl_set_object_color(CONSTRAINTS_FORM->LIST_OBJ[i],FL_TOP_BCOL,FL_RED);
     fl_set_object_callback(CONSTRAINTS_FORM->LIST_OBJ[i],CB_cntrt_list_obj,i);
     strcpy(textline, ct->namecntrt);
     if(strcmp(ct->namecntrt,"p3d_planar_closed_chain")==0) { 
       sprintf(nart,"%d",ct->pasjnts[0]->num);
       strcpy(auxtext, " J");
       strcat(auxtext, nart);
       strcat(textline, auxtext);
       sprintf(nart,"%d",ct->pasjnts[1]->num);
       strcpy(auxtext, " J");
       strcat(auxtext, nart);
       strcat(textline, auxtext);
     } 
     else {
       for(j=0;j<ct->npasjnts;j++) {
	 sprintf(nart,"%d",ct->pasjnts[j]->num);
	 strcpy(auxtext, " J");
	 strcat(auxtext, nart);
	 strcat(textline, auxtext);
       }
       for(j=0;j<ct->nactjnts;j++) {
	 sprintf(nart,"%d",ct->actjnts[j]->num);
	 strcpy(auxtext, " J");
	   strcat(auxtext, nart);
	   strcat(textline, auxtext);
       }
     }
     fl_add_text(FL_NORMAL_TEXT,50.0,13.0+20.0*i,200.0,14.8,textline);
   }
 }
}

static void CB_cntrt_arrows_obj(FL_OBJECT *ob, long arg)
{int val =  fl_get_button(ob);
 p3d_rob *r;
 int x,y;

 if(CSF) {
   fl_set_button(ob,0);
   return;
 }
 r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
 if(val) {
   fl_get_winorigin(FL_ObjWin(ob),&x,&y);
   fl_deactivate_form(CONSTRAINTS_FORM->CNTRT_FORM);    
   g3d_one_cntrt_form(r->cntrt_manager->cntrts[arg], x, y);
   fl_set_button(ob,0);
 }
}

static void g3d_create_cntrt_arrows_obj(void)
{
 int i; 
 p3d_rob *r;
 p3d_cntrt *ct;

 r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 
 if(r->cntrt_manager->cntrts != NULL) {
   for(i=0; i<ncntrt_st; i++) {
     ct = r->cntrt_manager->cntrts[i];
     CONSTRAINTS_FORM->ARROWS_OBJ[i] = fl_add_button(FL_PUSH_BUTTON,270.0,10.0+20.0*i,20.0,20.0,"@->");
     fl_set_button(CONSTRAINTS_FORM->ARROWS_OBJ[i],0);
     fl_set_object_callback(CONSTRAINTS_FORM->ARROWS_OBJ[i],CB_cntrt_arrows_obj,i);
   }
 }
}

static void CB_cntrt_new_obj(FL_OBJECT *ob, long arg)
{
 if(CSF) {
   fl_set_button(CONSTRAINTS_FORM->NEW_OBJ,0); 
   return;
 }

 CONSTRAINTS_SETTING_FORM = (MENU_CONSTRAINTS_SETTING *) malloc(sizeof(MENU_CONSTRAINTS_SETTING)); 
 g3d_create_constraints_setting_form();
 fl_show_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM,FL_PLACE_SIZE,TRUE,"CONSTRAINTS SETTING");
 fl_set_button(CONSTRAINTS_SETTING_FORM->DONE_OBJ,0);
/*  fl_deactivate_form(CONSTRAINTS_FORM->CNTRT_FORM); */
 fl_set_button(CONSTRAINTS_FORM->NEW_OBJ,0); 
}

static void g3d_create_cntrt_new_obj(void)
{

 CONSTRAINTS_FORM->NEW_OBJ = fl_add_button(FL_PUSH_BUTTON,155.0,20.0*ncntrt_af+20.0,40.0,20.0,"NEW");
 fl_set_call_back(CONSTRAINTS_FORM->NEW_OBJ,CB_cntrt_new_obj,0);
}


static void  CB_cntrt_ok_obj(FL_OBJECT *ob, long arg)
{int ir;

 ir = p3d_get_desc_curnum(P3D_ROBOT);

 if(CSF) {
   g3d_delete_constraints_setting_form();
   free(CONSTRAINTS_SETTING_FORM);
 }
 g3d_delete_constraints_form();
 free(CONSTRAINTS_FORM);
 FORMrobot_update(ir);
/*  g3d_delete_robot_form(ir); */
/*  g3d_delete_filter_form(ir); */
/*  g3d_create_robot_form(ir); */
/*  fl_show_form(ROBOTS_FORM[ir].ROBOT_FORM,FL_PLACE_POSITION,TRUE, */
/* 	      p3d_get_desc_curname(P3D_ROBOT));  */
/*  g3d_create_filterbox_form(ir); */
 fl_activate_all_forms();
}

static void g3d_create_cntrt_ok_obj(void)
{

 CONSTRAINTS_FORM->OK_OBJ = fl_add_button(FL_PUSH_BUTTON,205.0,20.0*ncntrt_af+20.0,40.0,20.0,"OK");
 fl_set_call_back(CONSTRAINTS_FORM->OK_OBJ,CB_cntrt_ok_obj,0);
}

/***********************************************************/

static void CB_RLG_obj(FL_OBJECT *ob, long arg)
{
  if(p3d_random_loop_generator_ok()) {
    p3d_set_RLG(!p3d_get_RLG());
    fl_set_button(CONSTRAINTS_FORM->RLG_OBJ,p3d_get_RLG());
  }
  else{
    fl_set_button(CONSTRAINTS_FORM->RLG_OBJ,0);
  }
}

static void g3d_create_RLG_obj(void)
{ 
  CONSTRAINTS_FORM->RLG_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,
		      35.0,20.0*ncntrt_af+15.0,50.0,30.0,
			      "use RLG");
  fl_set_call_back(CONSTRAINTS_FORM->RLG_OBJ,CB_RLG_obj,0);
  fl_set_button(CONSTRAINTS_FORM->RLG_OBJ,p3d_get_RLG());
}

static void g3d_delete_RLG_obj(void)
{ 
  fl_free_object(CONSTRAINTS_FORM->RLG_OBJ);
}

/***********************************************************/


static void g3d_delete_cntrt_list_obj(void)
{int i;
 
 for(i=0; i<ncntrt_st; i++) {
   fl_free_object(CONSTRAINTS_FORM->LIST_OBJ[i]);
   CONSTRAINTS_FORM->LIST_OBJ[i] = NULL;
 }
}

static void g3d_delete_cntrt_arrows_obj(void)
{int i;
 
 for(i=0; i<ncntrt_st; i++) {
   fl_free_object(CONSTRAINTS_FORM->ARROWS_OBJ[i]);
   CONSTRAINTS_FORM->ARROWS_OBJ[i] = NULL;
 }
}

static void g3d_delete_cntrt_new_obj(void)
{
  fl_free_object(CONSTRAINTS_FORM->NEW_OBJ);
  CONSTRAINTS_FORM->NEW_OBJ = NULL;
}

static void g3d_delete_cntrt_ok_obj(void)
{ 
  fl_free_object(CONSTRAINTS_FORM->OK_OBJ);
  CONSTRAINTS_FORM->OK_OBJ = NULL;
}



static void g3d_delete_constraints_form(void)
{ p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  FL_OBJECT  *obje;

  obje = ((FL_OBJECT *)CONSTRAINTS_FORM->NEW_OBJ);
  fl_get_winorigin(FL_ObjWin(obje),&xCF,&yCF);
  ftCF = 0;
  fl_hide_form(CONSTRAINTS_FORM->CNTRT_FORM);

  if(r->cntrt_manager->cntrts != NULL) {
    g3d_delete_cntrt_list_obj();
    g3d_delete_cntrt_arrows_obj();
  }
  g3d_delete_cntrt_new_obj();
  g3d_delete_cntrt_ok_obj();
  g3d_delete_RLG_obj();

  fl_free_form(CONSTRAINTS_FORM->CNTRT_FORM);
}


/* ----------------------------------------------------------------------------- */

static void g3d_create_constraints_setting_form(void)
{
  CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM = fl_bgn_form(FL_UP_BOX,250.0,20.0*ncntrt_tp+90.0);
  g3d_create_cntrt_set_list_obj();
  g3d_create_cntrt_set_done_obj();
  fl_end_form();
  CSF = 1;
}


static void CB_cntrt_set_list_obj(FL_OBJECT *ob, long arg)
{int val =  fl_get_button(ob);
 int x,y;

 if(val) {
   fl_get_winorigin(FL_ObjWin(ob),&x,&y);
   fl_deactivate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM); 
   fl_deactivate_form(CONSTRAINTS_FORM->CNTRT_FORM);
   switch(arg) {
   case 0:
     g3d_fixed_jnt_form(NULL,x,y);
     break;
   case 1:
     g3d_lin_rel_dofs_form(NULL,x,y);
     break;
   case 2:
     g3d_RRPRlnk_form(NULL,x,y);
     break;
   case 3:
     g3d_4Rlnk_form(NULL,x,y);
     break;
   case 4:
     g3d_3RPRlnk_form(NULL,x,y);
     break;
   case 5:
     g3d_jnt_on_ground_form(NULL,x,y);
     break;
   case 6:
     g3d_car_front_wheels_form(NULL,x,y);
     break;
   case 7:
     g3d_planar_closed_chain_form(NULL,x,y);
     break;
   case 8:
     g3d_R6_arm_ik(NULL,x,y);
     break;
   default:
     printf("CB_cntrt_set_list_obj : ERROR : case is not in switch\n");
     return;
   }
   fl_set_button(ob,0);
 }
}

static void g3d_create_cntrt_set_list_obj(void)
{
 int   i,ord; 

 fl_add_box(FL_ROUNDED_BOX,30.0,5.0,150.0,20.0,"TYPE OF CONSTRAINT"); 
 fl_add_box(FL_DOWN_BOX,10.0,30.0,200.0,20.0,"Joint fixed at a value");               /* type de contrainte 0 */
 fl_add_box(FL_DOWN_BOX,10.0,50.0,200.0,20.0,"J_a = K x J_b + C");                    /* type de contrainte 1 */
 fl_add_box(FL_DOWN_BOX,10.0,70.0,200.0,20.0,"RRPR Linkage");                         /* type de contrainte 2 */
 fl_add_box(FL_DOWN_BOX,10.0,90.0,200.0,20.0,"4R Linkage");                           /* type de contrainte 3 */
 fl_add_box(FL_DOWN_BOX,10.0,110.0,200.0,20.0,"3RPR Linkage");                        /* type de contrainte 4 */
 fl_add_box(FL_DOWN_BOX,10.0,130.0,200.0,20.0,"Contact with Ground");                 /* type de contrainte 5 */
 fl_add_box(FL_DOWN_BOX,10.0,150.0,200.0,20.0,"Car Front Wheels");                    /* type de contrainte 6 */
 fl_add_box(FL_DOWN_BOX,10.0,170.0,200.0,20.0,"Planar Closed Chain");                 /* type de contrainte 7 */
 fl_add_box(FL_DOWN_BOX,10.0,190.0,200.0,20.0,"R6 Arm Inverse Kinematics ");          /* type de contrainte 8 */
 fl_add_box(FL_FLAT_BOX,5.0,20.0*ncntrt_tp+35.0,240.0,30.0,"INPORTANT: if one constraint affects to another\n       SET THEM IN ORDER!");   
 ord = 0;
 for(i=1;i<=ncntrt_tp;i++) {
   CONSTRAINTS_SETTING_FORM->LIST_OBJ[i] = fl_add_button(FL_PUSH_BUTTON,210.0,30.0+20.0*ord,35.0,20.0,"Set");
   fl_set_object_callback(CONSTRAINTS_SETTING_FORM->LIST_OBJ[i],CB_cntrt_set_list_obj,i-1);
   ord++;
 }
}

static void  CB_cntrt_set_done_obj(FL_OBJECT *ob, long arg)
{
 g3d_delete_constraints_setting_form();
 free(CONSTRAINTS_SETTING_FORM);
/*  fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM); */
}

static void g3d_create_cntrt_set_done_obj(void)
{

 CONSTRAINTS_SETTING_FORM->DONE_OBJ = fl_add_button(FL_PUSH_BUTTON,105.0,20.0*ncntrt_tp+65.0,40.0,20.0,"Done");
 fl_set_call_back(CONSTRAINTS_SETTING_FORM->DONE_OBJ,CB_cntrt_set_done_obj,0);
}


static void g3d_delete_cntrt_set_list_obj(void)
{int i;

 for(i=1;i<=ncntrt_tp;i++) {
   fl_free_object(CONSTRAINTS_SETTING_FORM->LIST_OBJ[i]);
 }
}

static void g3d_delete_cntrt_set_done_obj(void)
{
  fl_free_object(CONSTRAINTS_SETTING_FORM->DONE_OBJ);
}

static void g3d_delete_constraints_setting_form(void)
{
  fl_hide_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);

  g3d_delete_cntrt_set_list_obj();
  g3d_delete_cntrt_set_done_obj();

  fl_free_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
  CSF = 0;
}


/* ------------------------------------------------------------------ */

static void g3d_one_cntrt_form(p3d_cntrt *ct, int x, int y)
{
  if(strcmp(ct->namecntrt,"p3d_fixed_jnt")==0) {
    g3d_fixed_jnt_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_lin_rel_dofs")==0) {
    g3d_lin_rel_dofs_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_jnt_on_ground")==0) {
    g3d_jnt_on_ground_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_RRPRlnk")==0) {
    g3d_RRPRlnk_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_4Rlnk")==0) {
    g3d_4Rlnk_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_3RPRlnk")==0) {
    g3d_3RPRlnk_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_car_front_wheels")==0) {
    g3d_car_front_wheels_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_planar_closed_chain")==0) {
    g3d_planar_closed_chain_form(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_R6_arm_ik")==0) {
    g3d_R6_arm_ik(ct,x,y);
  }
  else if(strcmp(ct->namecntrt,"p3d_3R_arm_ik")==0) {
    g3d_3R_arm_ik_form(ct,x,y);
  }
  else {
    PrintInfo(("No control window available for this constraint"));
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
  
/* ------------------------------------------------------------------ */
/* ***** fonctions pour chaque contrainte  ***** */ 

/* cntrt_tp0 */
static void g3d_create_cntrt_tp0_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp0_form(void);
static void CB_inputs_tp0_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp0_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp0_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP0_FORM;
static FL_OBJECT *INPUTS_TP0[2];
static FL_OBJECT *OK_TP0;
static FL_OBJECT *CANCEL_TP0;

static void g3d_fixed_jnt_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp0_form(ct);
  fl_set_form_position(CNTRT_TP0_FORM,x,y);     
  fl_show_form(CNTRT_TP0_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp0_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP0_FORM = fl_bgn_form(FL_UP_BOX,95.0,85.0);

  INPUTS_TP0[0] = fl_add_input(FL_NORMAL_INPUT,45.0,5.0,30.0,20.0,"J=");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP0[0],inval);
    fl_deactivate_object(INPUTS_TP0[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP0[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP0[0],CB_inputs_tp0_obj,0); 

  INPUTS_TP0[1] = fl_add_input(FL_NORMAL_INPUT,45.0,35.0,40.0,20.0,"value=");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[0]);
    fl_set_input(INPUTS_TP0[1],inval);
    Dval[0] = ct->argu_d[0];
  }
  fl_set_object_color(INPUTS_TP0[1],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP0[1],CB_inputs_tp0_obj,1); 

  OK_TP0 = fl_add_button(FL_PUSH_BUTTON,4.0,60.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP0,CB_ok_tp0_obj,-1); 
  else fl_set_call_back(OK_TP0,CB_ok_tp0_obj,ct->num); 

  CANCEL_TP0 = fl_add_button(FL_PUSH_BUTTON,52.0,60.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP0,CB_cancel_tp0_obj,-1); 
  else fl_set_call_back(CANCEL_TP0,CB_cancel_tp0_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp0_form(void)
{
  fl_hide_form(CNTRT_TP0_FORM);

  fl_free_object(INPUTS_TP0[0]); 
  fl_free_object(INPUTS_TP0[1]); 
  fl_free_object(OK_TP0);
  fl_free_object(CANCEL_TP0); 
  fl_free_form(CNTRT_TP0_FORM);
}

static void CB_inputs_tp0_obj(FL_OBJECT *ob, long arg)
{
  switch(arg) {
  case 0:
    Jpasiv[0] = atoi(fl_get_input(ob));
    break; 
  case 1:
    Dval[0] = atof(fl_get_input(ob));
    break; 
  }
}

static void CB_ok_tp0_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
/*   p3d_cntrt *ct; */

  if(arg == -1) {
    if(p3d_constraint("p3d_fixed_jnt", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, -1, 1)) {
/*       ct = p3d_get_current_cntrt();  */             /* pas necessaire pour cette cntrt */
/*       p3d_col_deactivate_one_cntrt_pairs(ct); */      
      g3d_delete_cntrt_tp0_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_fixed_jnt", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp0_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }     
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp0_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp0_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp0 */

/* ------------------------------------------------------------------ */
/* cntrt_tp1 */
static void g3d_create_cntrt_tp1_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp1_form(void);
static void CB_inputs_tp1_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp1_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp1_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP1_FORM;
static FL_OBJECT *INPUTS_TP1[4];
static FL_OBJECT *OK_TP1;
static FL_OBJECT *CANCEL_TP1;

static void g3d_lin_rel_dofs_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp1_form(ct);
  fl_set_form_position(CNTRT_TP1_FORM,x,y);     
  fl_show_form(CNTRT_TP1_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp1_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP1_FORM = fl_bgn_form(FL_UP_BOX,95.0,135.0);

  INPUTS_TP1[0] = fl_add_input(FL_NORMAL_INPUT,45.0,5.0,30.0,20.0,"J_a=");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP1[0],inval);
    fl_deactivate_object(INPUTS_TP1[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP1[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP1[0],CB_inputs_tp1_obj,0); 

  INPUTS_TP1[2] = fl_add_input(FL_NORMAL_INPUT,45.0,30.0,30.0,20.0,"J_b=");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP1[2],inval);
    fl_deactivate_object(INPUTS_TP1[2]);
    Jactiv[0] = ct->actjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP1[2],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP1[2],CB_inputs_tp1_obj,2); 

  INPUTS_TP1[1] = fl_add_input(FL_NORMAL_INPUT,45.0,55.0,40.0,20.0,"K=");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[0]);
    fl_set_input(INPUTS_TP1[1],inval);
    Dval[0] = ct->argu_d[0];
  }
  fl_set_object_color(INPUTS_TP1[1],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP1[1],CB_inputs_tp1_obj,1); 

  INPUTS_TP1[3] = fl_add_input(FL_NORMAL_INPUT,45.0,80.0,40.0,20.0,"C=");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[1]);
    fl_set_input(INPUTS_TP1[3],inval);
    Dval[1] = ct->argu_d[1];
  }
  fl_set_object_color(INPUTS_TP1[3],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP1[3],CB_inputs_tp1_obj,3); 

  OK_TP1 = fl_add_button(FL_PUSH_BUTTON,4.0,110.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP1,CB_ok_tp1_obj,-1); 
  else fl_set_call_back(OK_TP1,CB_ok_tp1_obj,ct->num); 

  CANCEL_TP1 = fl_add_button(FL_PUSH_BUTTON,52.0,110.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP1,CB_cancel_tp1_obj,-1); 
  else fl_set_call_back(CANCEL_TP1,CB_cancel_tp1_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp1_form(void)
{
  fl_hide_form(CNTRT_TP1_FORM);

  fl_free_object(INPUTS_TP1[0]); 
  fl_free_object(INPUTS_TP1[1]); 
  fl_free_object(INPUTS_TP1[2]);
  fl_free_object(INPUTS_TP1[3]);
  fl_free_object(OK_TP1);
  fl_free_object(CANCEL_TP1); 
  fl_free_form(CNTRT_TP1_FORM);
}

static void CB_inputs_tp1_obj(FL_OBJECT *ob, long arg)
{
  switch(arg) {
  case 0:
    Jpasiv[0] = atoi(fl_get_input(ob));
    break; 
  case 1:
    Dval[0] = atof(fl_get_input(ob));    
    break; 
  case 2:
    Jactiv[0] = atoi(fl_get_input(ob));
    break; 
  case 3:
    Dval[1] = atof(fl_get_input(ob));    
    break; 
  }    
}

static void CB_ok_tp1_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
/*   p3d_cntrt *ct; */

  if(arg == -1) {
    if(p3d_constraint("p3d_lin_rel_dofs", -1, Jpasiv, -1, Jactiv, 
		      -1, Dval, -1, Ival, -1, 1)) {
/*       ct = p3d_get_current_cntrt();  */             /* pas necessaire pour cette cntrt */
/*       p3d_col_deactivate_one_cntrt_pairs(ct); */      
      g3d_delete_cntrt_tp1_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_lin_rel_dofs", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp1_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }     
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp1_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp1_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp1 */

/* ------------------------------------------------------------------ */
/* cntrt_tp2 */
static void g3d_create_cntrt_tp2_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp2_form(void);
static void CB_inputs_tp2_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp2_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp2_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP2_FORM;
static FL_OBJECT *INPUTS_TP2[3];
static FL_OBJECT *OK_TP2;
static FL_OBJECT *CANCEL_TP2;

static void g3d_RRPRlnk_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp2_form(ct);
  fl_set_form_position(CNTRT_TP2_FORM,x,y);     
  fl_show_form(CNTRT_TP2_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp2_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP2_FORM = fl_bgn_form(FL_UP_BOX,95.0,115.0);

  INPUTS_TP2[0] = fl_add_input(FL_NORMAL_INPUT,55.0,5.0,30.0,20.0,"O is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP2[0],inval);
    fl_deactivate_object(INPUTS_TP2[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP2[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP2[0],CB_inputs_tp2_obj,0); 

  INPUTS_TP2[1] = fl_add_input(FL_NORMAL_INPUT,55.0,30.0,30.0,20.0,"A is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP2[1],inval);
    fl_deactivate_object(INPUTS_TP2[1]);
   Jactiv[0] = ct->actjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP2[1],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP2[1],CB_inputs_tp2_obj,1); 

  INPUTS_TP2[2] = fl_add_input(FL_NORMAL_INPUT,55.0,55.0,30.0,20.0,"C is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[1]->num);
    fl_set_input(INPUTS_TP2[2],inval);
    fl_deactivate_object(INPUTS_TP2[2]);
    Jpasiv[1] = ct->pasjnts[1]->num;
  }
  else
    fl_set_object_color(INPUTS_TP2[2],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP2[2],CB_inputs_tp2_obj,2); 

  OK_TP2 = fl_add_button(FL_PUSH_BUTTON,4.0,90.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP2,CB_ok_tp2_obj,-1); 
  else fl_set_call_back(OK_TP2,CB_ok_tp2_obj,ct->num);  

  CANCEL_TP2 = fl_add_button(FL_PUSH_BUTTON,52.0,90.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP2,CB_cancel_tp2_obj,-1); 
  else fl_set_call_back(CANCEL_TP2,CB_cancel_tp2_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp2_form(void)
{
  fl_hide_form(CNTRT_TP2_FORM);

  fl_free_object(INPUTS_TP2[0]); 
  fl_free_object(INPUTS_TP2[1]); 
  fl_free_object(INPUTS_TP2[2]);
  fl_free_object(OK_TP2);
  fl_free_object(CANCEL_TP2); 
  fl_free_form(CNTRT_TP2_FORM);
}

static void CB_inputs_tp2_obj(FL_OBJECT *ob, long arg)
{ int val = atoi(fl_get_input(ob));

  switch(arg) {
  case 0:
    Jpasiv[0] = val;
    break; 
  case 1:
    Jactiv[0] = val;    
    break; 
  case 2:
    Jpasiv[1] = val;
    break; 
  }
}

static void CB_ok_tp2_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
  p3d_cntrt *ct;

  if(arg == -1) {
    if(p3d_constraint("p3d_RRPRlnk", -1, Jpasiv, -1, Jactiv, 
		      -1, Dval, -1, Ival, -1, 1)) {
      ct = p3d_get_current_cntrt(); 
      p3d_col_deactivate_one_cntrt_pairs(ct);      
      g3d_delete_cntrt_tp2_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_RRPRlnk", -1, Jpasiv, -1, Jactiv, -1, 
		      Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp2_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }       
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp2_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp2_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp2 */

/* ------------------------------------------------------------------ */
/* cntrt_tp3 */
static void g3d_create_cntrt_tp3_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp3_form(void);
static void CB_inputs_tp3_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp3_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp3_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP3_FORM;
static FL_OBJECT *INPUTS_TP3[4];
static FL_OBJECT *OK_TP3;
static FL_OBJECT *CANCEL_TP3;

static void g3d_4Rlnk_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp3_form(ct);
  fl_set_form_position(CNTRT_TP3_FORM,x,y);     
  fl_show_form(CNTRT_TP3_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp3_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP3_FORM = fl_bgn_form(FL_UP_BOX,95.0,140.0);

  INPUTS_TP3[0] = fl_add_input(FL_NORMAL_INPUT,55.0,5.0,30.0,20.0,"O is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP3[0],inval);
    fl_deactivate_object(INPUTS_TP3[0]);
    Jactiv[0] = ct->actjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP3[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP3[0],CB_inputs_tp3_obj,0); 

  INPUTS_TP3[1] = fl_add_input(FL_NORMAL_INPUT,55.0,30.0,30.0,20.0,"A is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP3[1],inval);
    fl_deactivate_object(INPUTS_TP3[1]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP3[1],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP3[1],CB_inputs_tp3_obj,1); 

  INPUTS_TP3[2] = fl_add_input(FL_NORMAL_INPUT,55.0,55.0,30.0,20.0,"B is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[1]->num);
    fl_set_input(INPUTS_TP3[2],inval);
    fl_deactivate_object(INPUTS_TP3[2]);
    Jpasiv[1] = ct->pasjnts[1]->num;
  } 
  else
    fl_set_object_color(INPUTS_TP3[2],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP3[2],CB_inputs_tp3_obj,2); 

  INPUTS_TP3[3] = fl_add_input(FL_NORMAL_INPUT,55.0,80.0,30.0,20.0,"C is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[2]->num);
    fl_set_input(INPUTS_TP3[3],inval);
    fl_deactivate_object(INPUTS_TP3[3]);
    Jpasiv[2] = ct->pasjnts[2]->num;
  } 
  else
    fl_set_object_color(INPUTS_TP3[3],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP3[3],CB_inputs_tp3_obj,3); 

  OK_TP3 = fl_add_button(FL_PUSH_BUTTON,4.0,115.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP3,CB_ok_tp3_obj,-1); 
  else fl_set_call_back(OK_TP3,CB_ok_tp3_obj,ct->num); 

  CANCEL_TP3 = fl_add_button(FL_PUSH_BUTTON,52.0,115.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP3,CB_cancel_tp3_obj,-1); 
  else fl_set_call_back(CANCEL_TP3,CB_cancel_tp3_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp3_form(void)
{
  fl_hide_form(CNTRT_TP3_FORM);

  fl_free_object(INPUTS_TP3[0]); 
  fl_free_object(INPUTS_TP3[1]); 
  fl_free_object(INPUTS_TP3[2]);
  fl_free_object(INPUTS_TP3[3]);
  fl_free_object(OK_TP3);
  fl_free_object(CANCEL_TP3); 
  fl_free_form(CNTRT_TP3_FORM);
}

static void CB_inputs_tp3_obj(FL_OBJECT *ob, long arg)
{ int val = atoi(fl_get_input(ob));

  switch(arg) {
  case 0:
    Jactiv[0] = val;
    break; 
  case 1:
    Jpasiv[0] = val;    
    break; 
  case 2:
    Jpasiv[1] = val;
    break; 
  case 3:
    Jpasiv[2] = val;
    break; 
  }
}

static void CB_ok_tp3_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
  p3d_cntrt *ct;

  if(arg == -1) {
    if(p3d_constraint("p3d_4Rlnk", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, -1, 1)) {
      ct = p3d_get_current_cntrt(); 
      p3d_col_deactivate_one_cntrt_pairs(ct);      
      g3d_delete_cntrt_tp3_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_4Rlnk", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp3_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }       
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp3_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp3_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp3 */

/* ------------------------------------------------------------------ */
/* cntrt_tp4 */
static void g3d_create_cntrt_tp4_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp4_form(void);
static void CB_inputs_tp4_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp4_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp4_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP4_FORM;
static FL_OBJECT *INPUTS_TP4[4];
static FL_OBJECT *OK_TP4;
static FL_OBJECT *CANCEL_TP4;

static void g3d_3RPRlnk_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp4_form(ct);
  fl_set_form_position(CNTRT_TP4_FORM,x,y);     
  fl_show_form(CNTRT_TP4_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp4_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP4_FORM = fl_bgn_form(FL_UP_BOX,95.0,140.0);

  INPUTS_TP4[0] = fl_add_input(FL_NORMAL_INPUT,55.0,5.0,30.0,20.0,"O is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP4[0],inval);
    fl_deactivate_object(INPUTS_TP4[0]);
    Jactiv[0] = ct->actjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP4[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP4[0],CB_inputs_tp4_obj,0); 

  INPUTS_TP4[1] = fl_add_input(FL_NORMAL_INPUT,55.0,30.0,30.0,20.0,"A is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP4[1],inval);
    fl_deactivate_object(INPUTS_TP4[1]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP4[1],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP4[1],CB_inputs_tp4_obj,1); 

  INPUTS_TP4[2] = fl_add_input(FL_NORMAL_INPUT,55.0,55.0,30.0,20.0,"B is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[1]->num);
    fl_set_input(INPUTS_TP4[2],inval);
    fl_deactivate_object(INPUTS_TP4[2]);
    Jactiv[1] = ct->actjnts[1]->num;
  } 
  else
    fl_set_object_color(INPUTS_TP4[2],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP4[2],CB_inputs_tp4_obj,2); 

  INPUTS_TP4[3] = fl_add_input(FL_NORMAL_INPUT,55.0,80.0,30.0,20.0,"C is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[1]->num);
    fl_set_input(INPUTS_TP4[3],inval);
    fl_deactivate_object(INPUTS_TP4[3]);
    Jpasiv[1] = ct->pasjnts[1]->num;
  } 
  else
    fl_set_object_color(INPUTS_TP4[3],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP4[3],CB_inputs_tp4_obj,3); 

  OK_TP4 = fl_add_button(FL_PUSH_BUTTON,4.0,115.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP4,CB_ok_tp4_obj,-1); 
  else fl_set_call_back(OK_TP4,CB_ok_tp4_obj,ct->num); 

  CANCEL_TP4 = fl_add_button(FL_PUSH_BUTTON,52.0,115.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP4,CB_cancel_tp4_obj,-1); 
  else fl_set_call_back(CANCEL_TP4,CB_cancel_tp4_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp4_form(void)
{
  fl_hide_form(CNTRT_TP4_FORM);

  fl_free_object(INPUTS_TP4[0]); 
  fl_free_object(INPUTS_TP4[1]); 
  fl_free_object(INPUTS_TP4[2]);
  fl_free_object(INPUTS_TP4[3]);
  fl_free_object(OK_TP4);
  fl_free_object(CANCEL_TP4); 
  fl_free_form(CNTRT_TP4_FORM);
}

static void CB_inputs_tp4_obj(FL_OBJECT *ob, long arg)
{ int val = atoi(fl_get_input(ob));

  switch(arg) {
  case 0:
    Jactiv[0] = val;
    break; 
  case 1:
    Jpasiv[0] = val;    
    break; 
  case 2:
    Jactiv[1] = val;
    break; 
  case 3:
    Jpasiv[1] = val;
    break; 
  }
}

static void CB_ok_tp4_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
  p3d_cntrt *ct;

  if(arg == -1) {
    if(p3d_constraint("p3d_3RPRlnk", -1, Jpasiv, -1, Jactiv, 
		      -1, Dval, -1, Ival, -1, 1)) {
      ct = p3d_get_current_cntrt(); 
      p3d_col_deactivate_one_cntrt_pairs(ct);      
      g3d_delete_cntrt_tp4_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_deactivate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_3RPRlnk", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp4_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }       
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp4_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp4_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp4 */

/* ------------------------------------------------------------------ */
/* cntrt_tp5 */
static void g3d_create_cntrt_tp5_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp5_form(void);
static void CB_inputs_tp5_obj(FL_OBJECT *ob, long arg);
static void CB_sense_tp5_obj(FL_OBJECT *ob, long arg);
static void CB_twosenses_tp5_obj(FL_OBJECT *ob, long arg);
static void CB_stay_tp5_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp5_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp5_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP5_FORM;
static FL_OBJECT *INPUTS_TP5[6];
static FL_OBJECT *BLOCK_TP5;
static FL_OBJECT *SENSE_TP5;
static FL_OBJECT *TWOSENSES_TP5;
static FL_OBJECT *STAY_TP5;
static FL_OBJECT *OK_TP5;
static FL_OBJECT *CANCEL_TP5;

static void g3d_jnt_on_ground_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp5_form(ct);
  fl_set_form_position(CNTRT_TP5_FORM,x,y);     
  fl_show_form(CNTRT_TP5_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp5_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP5_FORM = fl_bgn_form(FL_UP_BOX,250.0,237.0);

  INPUTS_TP5[0] = fl_add_input(FL_NORMAL_INPUT,190.0,10.0,30.0,20.0,"Controled joints is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP5[0],inval);
    fl_deactivate_object(INPUTS_TP5[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  } 
  else
    fl_set_object_color(INPUTS_TP5[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP5[0],CB_inputs_tp5_obj,0); 

  fl_add_text(FL_NORMAL_TEXT,60.0,28.0,150.0,20.0,"Relative position of sliding point");

  INPUTS_TP5[1] = fl_add_input(FL_NORMAL_INPUT,30.0,45.0,60.0,20.0,"");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->Tatt[0][3]);
    fl_set_input(INPUTS_TP5[1],inval);
    fl_deactivate_object(INPUTS_TP5[1]);
    Dval[2] = ct->Tatt[0][3];
  }  
  else
    fl_set_object_color(INPUTS_TP5[1],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP5[1],CB_inputs_tp5_obj,3); 

  INPUTS_TP5[2] = fl_add_input(FL_NORMAL_INPUT,95.0,45.0,60.0,20.0,"");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->Tatt[1][3]);
    fl_set_input(INPUTS_TP5[2],inval);
    fl_deactivate_object(INPUTS_TP5[2]);
    Dval[3] = ct->Tatt[1][3];
  }  
  else
    fl_set_object_color(INPUTS_TP5[2],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP5[2],CB_inputs_tp5_obj,4); 
  
  INPUTS_TP5[3] = fl_add_input(FL_NORMAL_INPUT,160.0,45.0,60.0,20.0,"");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->Tatt[2][3]);
    fl_set_input(INPUTS_TP5[3],inval);
    fl_deactivate_object(INPUTS_TP5[3]);
    Dval[4] = ct->Tatt[2][3];
  }  
  else
    fl_set_object_color(INPUTS_TP5[3],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP5[3],CB_inputs_tp5_obj,5); 
  
  INPUTS_TP5[4] = fl_add_input(FL_NORMAL_INPUT,190.0,67.0,40.0,20.0,"z of the ground =");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[0]);
    fl_set_input(INPUTS_TP5[4],inval);
    Dval[0] = ct->argu_d[0];
  }  
  fl_set_object_color(INPUTS_TP5[4],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP5[4],CB_inputs_tp5_obj,1); 

  INPUTS_TP5[5] = fl_add_input(FL_NORMAL_INPUT,190.0,89.0,40.0,20.0,"Angle when the body is hanging =");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[1]);
    fl_set_input(INPUTS_TP5[5],inval);
    Dval[1] = ct->argu_d[1];
  }
  fl_set_object_color(INPUTS_TP5[5],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP5[5],CB_inputs_tp5_obj,2); 

  fl_add_text(FL_NORMAL_TEXT,90.0,108.0,100.0,20.0,"Blocked joints");

  BLOCK_TP5 = fl_add_input(FL_NORMAL_INPUT,20.0,125.0,210.0,20.0,"");
/*   if(ct != NULL) { */
  fl_set_input(BLOCK_TP5,"NOT IN DISPLAY");
  fl_deactivate_object(BLOCK_TP5);


  SENSE_TP5 = fl_add_checkbutton(FL_PUSH_BUTTON,60.0,147.0,40.0,20.0,"Turn in negative direction");
  if(ct != NULL) {
    fl_set_button(SENSE_TP5,ct->argu_i[0]);
    Ival[0] = ct->argu_i[0];
  }
  fl_set_call_back(SENSE_TP5,CB_sense_tp5_obj,0); 

  TWOSENSES_TP5 = fl_add_checkbutton(FL_PUSH_BUTTON,60.0,169.0,40.0,20.0,"Change turning sense");
  if(ct != NULL) {
    fl_set_button(TWOSENSES_TP5,ct->argu_i[1]);
    Ival[1] = ct->argu_i[1];
  }
  fl_set_call_back(TWOSENSES_TP5,CB_twosenses_tp5_obj,0); 

  STAY_TP5 = fl_add_checkbutton(FL_PUSH_BUTTON,60.0,191.0,40.0,20.0,"Keep contact with ground");
  if(ct != NULL) {
    fl_set_button(STAY_TP5,ct->argu_i[2]);
    Ival[2] = ct->argu_i[2];
  }
  fl_set_call_back(STAY_TP5,CB_stay_tp5_obj,0); 

  OK_TP5 = fl_add_button(FL_PUSH_BUTTON,80.0,212.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP5,CB_ok_tp5_obj,-1); 
  else fl_set_call_back(OK_TP5,CB_ok_tp5_obj,ct->num); 

  CANCEL_TP5 = fl_add_button(FL_PUSH_BUTTON,130.0,212.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP5,CB_cancel_tp5_obj,-1); 
  else fl_set_call_back(CANCEL_TP5,CB_cancel_tp5_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp5_form(void)
{
  fl_hide_form(CNTRT_TP5_FORM);

  fl_free_object(INPUTS_TP5[0]); 
  fl_free_object(INPUTS_TP5[1]); 
  fl_free_object(INPUTS_TP5[2]); 
  fl_free_object(INPUTS_TP5[3]); 
  fl_free_object(INPUTS_TP5[4]); 
  fl_free_object(INPUTS_TP5[5]); 
  fl_free_object(BLOCK_TP5);
  fl_free_object(SENSE_TP5);
  fl_free_object(TWOSENSES_TP5);
  fl_free_object(STAY_TP5);
  fl_free_object(OK_TP5);
  fl_free_object(CANCEL_TP5); 
  fl_free_form(CNTRT_TP5_FORM);
}

static void CB_inputs_tp5_obj(FL_OBJECT *ob, long arg)
{
  if(arg == 0) {
    Jpasiv[0] = atoi(fl_get_input(ob));
  }
  else {
    arg --;
    Dval[arg] = atof(fl_get_input(ob)); 
  }
} 

static void CB_sense_tp5_obj(FL_OBJECT *ob, long arg)
{
  Ival[0] = fl_get_button(ob); 
}

static void CB_twosenses_tp5_obj(FL_OBJECT *ob, long arg)
{
  Ival[1] = fl_get_button(ob); 
}

static void CB_stay_tp5_obj(FL_OBJECT *ob, long arg)
{
  Ival[2] = fl_get_button(ob); 
}

static void CB_ok_tp5_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
/*   p3d_cntrt *ct; */

  if(arg == -1) {
    if(p3d_constraint("p3d_jnt_on_ground", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, -1, 1)) {
/*       ct = p3d_get_current_cntrt();  */             /* pas necessaire pour cette cntrt */
/*       p3d_col_deactivate_one_cntrt_pairs(ct);  */     
      g3d_delete_cntrt_tp5_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_jnt_on_ground", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp5_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }       
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp5_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp5_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp5 */

/* ------------------------------------------------------------------ */
/* cntrt_tp6 */
static void g3d_create_cntrt_tp6_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp6_form(void);
static void CB_inputs_tp6_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp6_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp6_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP6_FORM;
static FL_OBJECT *INPUTS_TP6[5];
static FL_OBJECT *OK_TP6;
static FL_OBJECT *CANCEL_TP6;

static void g3d_car_front_wheels_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp6_form(ct);
  fl_set_form_position(CNTRT_TP6_FORM,x,y);     
  fl_show_form(CNTRT_TP6_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp6_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP6_FORM = fl_bgn_form(FL_UP_BOX,250.0,165.0);

  INPUTS_TP6[0] = fl_add_input(FL_NORMAL_INPUT,190.0,10.0,30.0,20.0,"Joint corresponding to the steering column is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP6[0],inval);
    fl_deactivate_object(INPUTS_TP6[0]);
    Jactiv[0] = ct->actjnts[0]->num;
  } 
  else
    fl_set_object_color(INPUTS_TP6[0],FL_CYAN,FL_CYAN);
  fl_set_call_back(INPUTS_TP6[0],CB_inputs_tp6_obj,0); 

  INPUTS_TP6[1] = fl_add_input(FL_NORMAL_INPUT,190.0,35.0,30.0,20.0,"Joint corresponding to right wheel is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP6[1],inval);
    fl_deactivate_object(INPUTS_TP6[1]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }  
  else
    fl_set_object_color(INPUTS_TP6[1],FL_CYAN,FL_CYAN);
  fl_set_call_back(INPUTS_TP6[1],CB_inputs_tp6_obj,1); 

  INPUTS_TP6[2] = fl_add_input(FL_NORMAL_INPUT,190.0,60.0,30.0,20.0,"Joint corresponding to left wheel is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[1]->num);
    fl_set_input(INPUTS_TP6[2],inval);
    fl_deactivate_object(INPUTS_TP6[2]);
    Jpasiv[1] = ct->pasjnts[1]->num;
  }
  else
    fl_set_object_color(INPUTS_TP6[2],FL_CYAN,FL_CYAN);
  fl_set_call_back(INPUTS_TP6[2],CB_inputs_tp6_obj,2); 

  INPUTS_TP6[3] = fl_add_input(FL_NORMAL_INPUT,190.0,85.0,40.0,20.0,"Distance between front and rear wheels axes =");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[0]);
    fl_set_input(INPUTS_TP6[3],inval);
    Dval[0] = ct->argu_d[0];
  }
  fl_set_object_color(INPUTS_TP6[3],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP6[3],CB_inputs_tp6_obj,3); 

  INPUTS_TP6[4] = fl_add_input(FL_NORMAL_INPUT,190.0,110.0,40.0,20.0,"Distance between front wheels =");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[1]*2);
    fl_set_input(INPUTS_TP6[4],inval);
    Dval[1] = ct->argu_d[1]*2;
  }
  fl_set_object_color(INPUTS_TP6[4],FL_CYAN,FL_CYAN);  
  fl_set_call_back(INPUTS_TP6[4],CB_inputs_tp6_obj,4); 

  OK_TP6 = fl_add_button(FL_PUSH_BUTTON,80.0,140.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP6,CB_ok_tp6_obj,-1); 
  else fl_set_call_back(OK_TP6,CB_ok_tp6_obj,ct->num); 

  CANCEL_TP6 = fl_add_button(FL_PUSH_BUTTON,130.0,140.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP6,CB_cancel_tp6_obj,-1); 
  else fl_set_call_back(CANCEL_TP6,CB_cancel_tp6_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp6_form(void)
{
  fl_hide_form(CNTRT_TP6_FORM);

  fl_free_object(INPUTS_TP6[0]); 
  fl_free_object(INPUTS_TP6[1]); 
  fl_free_object(INPUTS_TP6[2]); 
  fl_free_object(INPUTS_TP6[3]); 
  fl_free_object(INPUTS_TP6[4]); 
  fl_free_object(CANCEL_TP6); 
  fl_free_form(CNTRT_TP6_FORM);
}

static void CB_inputs_tp6_obj(FL_OBJECT *ob, long arg)
{
  switch(arg) {
  case 0:
    Jactiv[0] = atoi(fl_get_input(ob));
    break;
  case 1:
    Jpasiv[0] = atoi(fl_get_input(ob));
    break; 
  case 2:
    Jpasiv[1] = atoi(fl_get_input(ob));
    break; 
  case 3:
    Dval[0] = atof(fl_get_input(ob));    
    break; 
  case 4:
    Dval[1] = atof(fl_get_input(ob));    
    break;
  }
} 

static void CB_ok_tp6_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
/*   p3d_cntrt *ct; */

  if(arg == -1) {
    if(p3d_constraint("p3d_car_front_wheels", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, -1, 1)) {
/*       ct = p3d_get_current_cntrt();  */             /* pas necessaire pour cette cntrt */
/*       p3d_col_deactivate_one_cntrt_pairs(ct); */      
      g3d_delete_cntrt_tp6_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_car_front_wheels", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp6_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }     
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp6_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp6_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp6 */

/* ------------------------------------------------------------------ */
/* cntrt_tp7 */
static void g3d_create_cntrt_tp7_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp7_form(void);
static void CB_inputs_tp7_obj(FL_OBJECT *ob, long arg);
static void CB_whatcase_tp7_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp7_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp7_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP7_FORM;
static FL_OBJECT *INPUTS_TP7[5];
static FL_OBJECT *WHATCASE_TP7;
static FL_OBJECT *OK_TP7;
static FL_OBJECT *CANCEL_TP7;

static void g3d_planar_closed_chain_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp7_form(ct);
  fl_set_form_position(CNTRT_TP7_FORM,x,y);     
  fl_show_form(CNTRT_TP7_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp7_form(p3d_cntrt *ct)
{ char inval[20];

  CNTRT_TP7_FORM = fl_bgn_form(FL_UP_BOX,250.0,180.0);

  INPUTS_TP7[0] = fl_add_input(FL_NORMAL_INPUT,205.0,5.0,30.0,20.0,"Point of closure in the passive part corresponds to J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP7[0],inval);
    fl_deactivate_object(INPUTS_TP7[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP7[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP7[0],CB_inputs_tp7_obj,0); 

  INPUTS_TP7[1] = fl_add_input(FL_NORMAL_INPUT,205.0,30.0,30.0,20.0,"Point of closure in the active part corresponds to J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[1]->num);
    fl_set_input(INPUTS_TP7[1],inval);
    fl_deactivate_object(INPUTS_TP7[1]);
    Jpasiv[1] = ct->pasjnts[1]->num;
  }
  else
    fl_set_object_color(INPUTS_TP7[1],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP7[1],CB_inputs_tp7_obj,1); 

  INPUTS_TP7[2] = fl_add_input(FL_NORMAL_INPUT,205.0,55.0,30.0,20.0,"First joint (rotation) of the controlled chain is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP7[2],inval);
    fl_deactivate_object(INPUTS_TP7[2]);
    Jactiv[0] = ct->actjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP7[2],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP7[2],CB_inputs_tp7_obj,2); 

  INPUTS_TP7[3] = fl_add_input(FL_NORMAL_INPUT,205.0,80.0,30.0,20.0,"First passive joint (JP1) is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[2]->num);
    fl_set_input(INPUTS_TP7[3],inval);
    fl_deactivate_object(INPUTS_TP7[3]);
    Jpasiv[2] = ct->pasjnts[2]->num;
  }
  else
    fl_set_object_color(INPUTS_TP7[3],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP7[3],CB_inputs_tp7_obj,3); 

  INPUTS_TP7[4] = fl_add_input(FL_NORMAL_INPUT,205.0,105.0,30.0,20.0,"Second passive joint (JP2) is J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[3]->num);
    fl_set_input(INPUTS_TP7[4],inval);
    fl_deactivate_object(INPUTS_TP7[4]);
    Jpasiv[2] = ct->pasjnts[3]->num;
  }
  else
    fl_set_object_color(INPUTS_TP7[4],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP7[4],CB_inputs_tp7_obj,4); 

  WHATCASE_TP7 = fl_add_checkbutton(FL_PUSH_BUTTON,60.0,130.0,40.0,20.0,"Change configuration");
  if(ct != NULL) {
    fl_set_button(WHATCASE_TP7,ct->argu_i[0]);
    Ival[0] = ct->argu_i[0];
  }
  fl_set_call_back(WHATCASE_TP7,CB_whatcase_tp7_obj,0); 

  OK_TP7 = fl_add_button(FL_PUSH_BUTTON,80.0,155.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP7,CB_ok_tp7_obj,-1); 
  else fl_set_call_back(OK_TP7,CB_ok_tp7_obj,ct->num);  

  CANCEL_TP7 = fl_add_button(FL_PUSH_BUTTON,130.0,155.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP7,CB_cancel_tp7_obj,-1); 
  else fl_set_call_back(CANCEL_TP7,CB_cancel_tp7_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp7_form(void)
{
  fl_hide_form(CNTRT_TP7_FORM);

  fl_free_object(INPUTS_TP7[0]); 
  fl_free_object(INPUTS_TP7[1]); 
  fl_free_object(INPUTS_TP7[2]);
  fl_free_object(INPUTS_TP7[3]);
  fl_free_object(INPUTS_TP7[4]);
  fl_free_object(WHATCASE_TP7);
  fl_free_object(OK_TP7);
  fl_free_object(CANCEL_TP7); 
  fl_free_form(CNTRT_TP7_FORM);
}

static void CB_inputs_tp7_obj(FL_OBJECT *ob, long arg)
{ int val = atoi(fl_get_input(ob));

  switch(arg) {
  case 0:
    Jpasiv[0] = val;
    break; 
  case 1:
    Jpasiv[1] = val;
    break; 
  case 2:
    Jactiv[0] = val;    
    break;
  case 3:
    Jpasiv[2] = val;    
    break;
  case 4:
    Jpasiv[3] = val;    
    break;  
  }
}

static void CB_whatcase_tp7_obj(FL_OBJECT *ob, long arg)
{
  Ival[0] = fl_get_button(ob); 
}

static void CB_ok_tp7_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
  p3d_cntrt *ct;

  if(arg == -1) {
    if(p3d_constraint("p3d_planar_closed_chain", -1, Jpasiv, -1, Jactiv, 
		      -1, Dval, -1, Ival, -1, 1)) {
      ct = p3d_get_current_cntrt(); 
      p3d_col_deactivate_one_cntrt_pairs(ct);      
      g3d_delete_cntrt_tp7_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
   }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_planar_closed_chain", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp7_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }       
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp7_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp7_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp7 */

/* ------------------------------------------------------------------ */

/* cntrt_tp8 */
static void g3d_create_cntrt_tp8_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp8_form(void);
static void CB_inputs_tp8_obj(FL_OBJECT *ob, long arg);
static void CB_e1_tp8_obj(FL_OBJECT *ob, long arg);
static void CB_e2_tp8_obj(FL_OBJECT *ob, long arg);
static void CB_e3_tp8_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp8_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp8_obj(FL_OBJECT *ob, long arg);

static FL_FORM   *CNTRT_TP8_FORM;
static FL_OBJECT *INPUTS_TP8[5];
static FL_OBJECT *E1_TP8;
static FL_OBJECT *E2_TP8;
static FL_OBJECT *E3_TP8;
static FL_OBJECT *OK_TP8;
static FL_OBJECT *CANCEL_TP8;

static void g3d_R6_arm_ik(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp8_form(ct);
  fl_set_form_position(CNTRT_TP8_FORM,x,y);     
  fl_show_form(CNTRT_TP8_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp8_form(p3d_cntrt *ct)
{ char inval[20];
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  CNTRT_TP8_FORM = fl_bgn_form(FL_UP_BOX,140.0,240.0);

  INPUTS_TP8[0] = fl_add_input(FL_NORMAL_INPUT,80.0,5.0,30.0,20.0,"Base is at J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP8[0],inval);
    fl_deactivate_object(INPUTS_TP8[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP8[0],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP8[0],CB_inputs_tp8_obj,0); 

  INPUTS_TP8[1] = fl_add_input(FL_NORMAL_INPUT,80.0,30.0,30.0,20.0,"Grip is at J");
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP8[1],inval);
    fl_deactivate_object(INPUTS_TP8[1]);
    Jactiv[0] = ct->actjnts[0]->num;
  }
  else
    fl_set_object_color(INPUTS_TP8[1],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP8[1],CB_inputs_tp8_obj,1); 

  INPUTS_TP8[2] = fl_add_input(FL_NORMAL_INPUT,70.0,55.0,40.0,20.0,"a2 = ");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[0]);
    fl_set_input(INPUTS_TP8[2],inval);
    fl_deactivate_object(INPUTS_TP8[2]);
    Dval[0] = ct->argu_d[0];
  }
  else
    fl_set_object_color(INPUTS_TP8[2],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP8[2],CB_inputs_tp8_obj,2); 

  INPUTS_TP8[3] = fl_add_input(FL_NORMAL_INPUT,70.0,80.0,40.0,20.0,"r4 = ");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[1]);
    fl_set_input(INPUTS_TP8[3],inval);
    fl_deactivate_object(INPUTS_TP8[3]);
    Dval[1] = ct->argu_d[1];
  }
  else
    fl_set_object_color(INPUTS_TP8[3],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP8[3],CB_inputs_tp8_obj,3); 

  INPUTS_TP8[4] = fl_add_input(FL_NORMAL_INPUT,70.0,105.0,40.0,20.0,"Grip Distance");
  if(ct != NULL) {
    sprintf(inval,"%f",ct->argu_d[2]);
    fl_set_input(INPUTS_TP8[4],inval);
    Dval[2] = ct->argu_d[2];
  }
  fl_set_object_color(INPUTS_TP8[4],FL_CYAN,FL_CYAN);      
  fl_set_call_back(INPUTS_TP8[4],CB_inputs_tp8_obj,4); 

  fl_add_box(FL_FLAT_BOX,20.0,130.0,100.0,30.0,"ARM CONFIGURATION");

  E1_TP8 = fl_add_choice(FL_NORMAL_CHOICE,50.0,150.0,60.0,20.0,"");
  fl_addto_choice(E1_TP8,"forward");
  fl_addto_choice(E1_TP8,"backward");
  if(ct != NULL) {
//     if(ct->argu_i[1] == 1)
//       fl_set_choice(E1_TP8,1);
//     else
//       fl_set_choice(E1_TP8,2);
//     Ival[1] = ct->argu_i[1];
    if(robotSelectPositionFlag){//goto
      if(r->ikSolGoto[ct->num] < 5){
        fl_set_choice(E1_TP8,1);
      }else{
        fl_set_choice(E1_TP8,2);
      }
      Ival[0] = r->ikSolGoto[ct->num];
    }else{//current
      if(r->ikSolPos[ct->num] < 5){
        fl_set_choice(E1_TP8,1);
      }else{
        fl_set_choice(E1_TP8,2);
      }
      Ival[0] = r->ikSolPos[ct->num];
    }
  }
	
	if(ct==NULL) return;
  fl_set_call_back(E1_TP8,CB_e1_tp8_obj,ct->num);
  fl_set_object_color(E1_TP8,FL_LEFT_BCOL,FL_CYAN);

  E2_TP8 = fl_add_choice(FL_NORMAL_CHOICE,50.0,170.0,60.0,20.0,"");
  fl_addto_choice(E2_TP8,"up");
  fl_addto_choice(E2_TP8,"down");
  if(ct != NULL) {
//     if(ct->argu_i[0] == 1)
//       fl_set_choice(E2_TP8,1);
//     else
//       fl_set_choice(E2_TP8,2);
//     Ival[0] = ct->argu_i[0];
    if(robotSelectPositionFlag){//goto
      if(r->ikSolGoto[ct->num]%4 < 2){
        fl_set_choice(E1_TP8,1);
      }else{
        fl_set_choice(E1_TP8,2);
      }
      Ival[0] = r->ikSolGoto[ct->num];
    }else{//current
      if(r->ikSolPos[ct->num]%4 < 2){
        fl_set_choice(E1_TP8,1);
      }else{
        fl_set_choice(E1_TP8,2);
      }
      Ival[0] = r->ikSolPos[ct->num];
    }
  }

  fl_set_call_back(E2_TP8,CB_e2_tp8_obj,ct->num);
  fl_set_object_color(E2_TP8,FL_LEFT_BCOL,FL_CYAN);

  E3_TP8 = fl_add_choice(FL_NORMAL_CHOICE,50.0,190.0,60.0,20.0,"");
  fl_addto_choice(E3_TP8,"flip");
  fl_addto_choice(E3_TP8,"flop");  
  if(ct != NULL) {
//     if(ct->argu_i[2] == 1)
//       fl_set_choice(E3_TP8,1);
//     else
//       fl_set_choice(E3_TP8,2);
//     Ival[2] = ct->argu_i[2];
    if(robotSelectPositionFlag){//goto
      if(r->ikSolGoto[ct->num]%2 == 0){
        fl_set_choice(E1_TP8,1);
      }else{
        fl_set_choice(E1_TP8,2);
      }
      Ival[0] = r->ikSolGoto[ct->num];
    }else{//current
      if(r->ikSolPos[ct->num]%2 == 0){
        fl_set_choice(E1_TP8,1);
      }else{
        fl_set_choice(E1_TP8,2);
      }
      Ival[0] = r->ikSolPos[ct->num];
    }
  }

  fl_set_call_back(E3_TP8,CB_e3_tp8_obj,ct->num);
  fl_set_object_color(E3_TP8,FL_LEFT_BCOL,FL_CYAN);

  OK_TP8 = fl_add_button(FL_PUSH_BUTTON,25.0,215.0,40.0,20.0,"OK");
  if(ct == NULL) fl_set_call_back(OK_TP8,CB_ok_tp8_obj,-1); 
  else fl_set_call_back(OK_TP8,CB_ok_tp8_obj,ct->num);  

  CANCEL_TP8 = fl_add_button(FL_PUSH_BUTTON,75.0,215.0,40.0,20.0,"Cancel");
  if(ct == NULL) fl_set_call_back(CANCEL_TP8,CB_cancel_tp8_obj,-1); 
  else fl_set_call_back(CANCEL_TP8,CB_cancel_tp8_obj,ct->num); 

  fl_end_form();
}

static void g3d_delete_cntrt_tp8_form(void)
{
  fl_hide_form(CNTRT_TP8_FORM);

  fl_free_object(INPUTS_TP8[0]); 
  fl_free_object(INPUTS_TP8[1]); 
  fl_free_object(INPUTS_TP8[2]);
  fl_free_object(INPUTS_TP8[3]);
  fl_free_object(INPUTS_TP8[4]);
  fl_free_object(E1_TP8);
  fl_free_object(E2_TP8);
  fl_free_object(E3_TP8);
  fl_free_object(OK_TP8);
  fl_free_object(CANCEL_TP8); 
  fl_free_form(CNTRT_TP8_FORM);
}

static void CB_inputs_tp8_obj(FL_OBJECT *ob, long arg)
{ 
  switch(arg) {
  case 0:
    Jpasiv[0] = atoi(fl_get_input(ob));
    break; 
  case 1:
    Jactiv[0] = atoi(fl_get_input(ob));
    break; 
  case 2:
    Dval[0] = atof(fl_get_input(ob));    
    break;
  case 3:
    Dval[1] = atof(fl_get_input(ob));    
    break;
  case 4:
    Dval[2] = atof(fl_get_input(ob));    
    break;  
  }
}

static void CB_e1_tp8_obj(FL_OBJECT *ob, long arg)
{ int val = fl_get_choice(ob);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if(val != 0) {
//     if(val == 1){
//       Ival[1] = 1;
//     }else{
//       Ival[1] = -1;
//     }
    if(robotSelectPositionFlag){//goto
      if(val == 1){
        Ival[0] = (int)(Ival[0]%4);
        r->ikSolGoto[arg] = (int)(Ival[0]%4);
      }else{
        Ival[0] = (int)(Ival[0]%4 + 4);
        r->ikSolGoto[arg] = (int)(Ival[0]%4+ 4);
      }
    }else{//current
      if(val == 1){
        Ival[0] = (int)(Ival[0]%4);
        r->ikSolPos[arg] = (int)(Ival[0]%4);
      }else{
        Ival[0] = (int)(Ival[0]%4 + 4);
        r->ikSolPos[arg] = (int)(Ival[0]%4+ 4);
      }
    }
  }
}

static void CB_e2_tp8_obj(FL_OBJECT *ob, long arg)
{ int val = fl_get_choice(ob);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if(val != 0) {
//     if(val == 1){
//       Ival[0] = 1;
//     }else{
//       Ival[0] = -1;
//     }
    if(robotSelectPositionFlag){//goto
      if(val == 1){
        Ival[0] = 4*((int)Ival[0]/4)+((int)Ival[0]-1)%2+1;
        r->ikSolGoto[arg] = Ival[0];
      }else{
        Ival[0] = 4*((int)Ival[0]/4)+((int)Ival[0]-1)%2+3;
        r->ikSolGoto[arg] = Ival[0];
      }
    }else{//current
      if(val == 1){
        Ival[0] = 4*((int)Ival[0]/4)+((int)Ival[0]-1)%2+1;
        r->ikSolPos[arg] = Ival[0];
      }else{
        Ival[0] = 4*((int)Ival[0]/4)+((int)Ival[0]-1)%2+3;
        r->ikSolPos[arg] = Ival[0];
      }
    }
  }
}

static void CB_e3_tp8_obj(FL_OBJECT *ob, long arg)
{ int val = fl_get_choice(ob);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if(val != 0) {
//     if(val == 1){
//       Ival[2] = 1;
//     }else{
//       Ival[2] = -1;
//     }
    if(robotSelectPositionFlag){//goto
      if(val == 1){
        Ival[0] = ((int)Ival[0])%2 == 0? ((int)Ival[0])-1:((int)Ival[0]);
        r->ikSolGoto[arg] = Ival[0];
      }else{
        Ival[0] = ((int)Ival[0])%2 == 1? ((int)Ival[0])+1:((int)Ival[0]);
        r->ikSolGoto[arg] = Ival[0];
      }
    }else{//current
      if(val == 1){
        Ival[0] = ((int)Ival[0])%2 == 0? ((int)Ival[0])-1:((int)Ival[0]);
        r->ikSolPos[arg] = Ival[0];
      }else{
        Ival[0] = ((int)Ival[0])%2 == 1? ((int)Ival[0])+1:((int)Ival[0]);
        r->ikSolPos[arg] = Ival[0];
      }
    }
  }
}

static void CB_ok_tp8_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_cntrt *ct;

  if(arg == -1) {
    if(p3d_constraint("p3d_R6_arm_ik", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, -1, 1)) {
      ct = p3d_get_current_cntrt(); 
      p3d_col_deactivate_one_cntrt_pairs(ct);      
      g3d_delete_cntrt_tp8_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
   }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_R6_arm_ik", -1, Jpasiv, -1, Jactiv,
		      -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp8_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
	fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
/* 	CB_cntrt_list_obj(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1); */
      }
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  p3d_update_this_robot_pos(r);
}

static void CB_cancel_tp8_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp8_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}
/* fin cntrt_tp8 */

/* ------------------------------------------------------------------ */

/* cntrt_tp9 */
static void g3d_create_cntrt_tp9_form(p3d_cntrt *ct);
static void g3d_delete_cntrt_tp9_form(void);
static void CB_inputs_tp9_obj(FL_OBJECT *ob, long arg);
static void CB_ok_tp9_obj(FL_OBJECT *ob, long arg);
static void CB_cancel_tp9_obj(FL_OBJECT *ob, long arg);
static int CB_cntrt_tp9_OnClose(FL_FORM *form, void *arg);

static FL_FORM   *CNTRT_TP9_FORM;
static FL_OBJECT *MAIN_FRAME_TP9;
static FL_OBJECT *VALID_FRAME_TP9;
static FL_OBJECT *INPUTS_TP9[10];
static FL_OBJECT *OK_TP9;
static FL_OBJECT *CANCEL_TP9;

static void g3d_3R_arm_ik_form(p3d_cntrt *ct, int x, int y)
{
  g3d_create_cntrt_tp9_form(ct);
  fl_set_form_position(CNTRT_TP9_FORM,x,y);
  fl_show_form(CNTRT_TP9_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_cntrt_tp9_form(p3d_cntrt *ct)
{ char inval[20];
  g3d_create_form(&CNTRT_TP9_FORM,110,280,FL_UP_BOX);
  g3d_create_frame(&MAIN_FRAME_TP9,FL_ENGRAVED_FRAME,-1,-1,"",(void**)&CNTRT_TP9_FORM,1);

  //Passive joint1
  g3d_create_input(&INPUTS_TP9[0],FL_NORMAL_INPUT,33,20,"PasJnt 1",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[0],CB_inputs_tp9_obj,0);
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[0]->num);
    fl_set_input(INPUTS_TP9[0],inval);
    fl_deactivate_object(INPUTS_TP9[0]);
    Jpasiv[0] = ct->pasjnts[0]->num;
  }else{
    fl_set_object_color(INPUTS_TP9[0],FL_CYAN,FL_CYAN);
  }

  //Passive joint2
  g3d_create_input(&INPUTS_TP9[1],FL_NORMAL_INPUT,33,20,"PasJnt 2",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[1],CB_inputs_tp9_obj,1);
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[1]->num);
    fl_set_input(INPUTS_TP9[1],inval);
    fl_deactivate_object(INPUTS_TP9[1]);
    Jpasiv[1] = ct->pasjnts[1]->num;
  }else{
    fl_set_object_color(INPUTS_TP9[1],FL_CYAN,FL_CYAN);
  }

  //Passive joint1
  g3d_create_input(&INPUTS_TP9[2],FL_NORMAL_INPUT,33,20,"PasJnt 3",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[2],CB_inputs_tp9_obj,2);
  if(ct != NULL) {
    sprintf(inval,"%d",ct->pasjnts[2]->num);
    fl_set_input(INPUTS_TP9[2],inval);
    fl_deactivate_object(INPUTS_TP9[2]);
    Jpasiv[2] = ct->pasjnts[2]->num;
  }else{
    fl_set_object_color(INPUTS_TP9[2],FL_CYAN,FL_CYAN);
  }
  
  //Active joint
  g3d_create_input(&INPUTS_TP9[3],FL_NORMAL_INPUT,33,20,"ActJnt",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[3],CB_inputs_tp9_obj,3);
  if(ct != NULL) {
    sprintf(inval,"%d",ct->actjnts[0]->num);
    fl_set_input(INPUTS_TP9[3],inval);
    fl_deactivate_object(INPUTS_TP9[3]);
    Jactiv[0] = ct->actjnts[0]->num;
  }else{
    fl_set_object_color(INPUTS_TP9[3],FL_CYAN,FL_CYAN);
  }
  
  //lenght 1
  g3d_create_input(&INPUTS_TP9[4],FL_NORMAL_INPUT,33,20,"Length 1",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[4],CB_inputs_tp9_obj,4);
  if(ct != NULL) {
    sprintf(inval,"%.2f",ct->argu_d[0]);
    fl_set_input(INPUTS_TP9[4],inval);
    fl_deactivate_object(INPUTS_TP9[4]);
    Dval[0] = ct->argu_d[0];
  }else{
    fl_set_object_color(INPUTS_TP9[4],FL_CYAN,FL_CYAN);
  }
  
  //lenght 2
  g3d_create_input(&INPUTS_TP9[5],FL_NORMAL_INPUT,33,20,"Length 2",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[5],CB_inputs_tp9_obj,5);
  if(ct != NULL) {
    sprintf(inval,"%.2f",ct->argu_d[1]);
    fl_set_input(INPUTS_TP9[5],inval);
    fl_deactivate_object(INPUTS_TP9[5]);
    Dval[1] = ct->argu_d[1];
  }else{
    fl_set_object_color(INPUTS_TP9[5],FL_CYAN,FL_CYAN);
  }
  
  //lenght 3
  g3d_create_input(&INPUTS_TP9[6],FL_NORMAL_INPUT,33,20,"Length 3",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[6],CB_inputs_tp9_obj,6);
  if(ct != NULL) {
    sprintf(inval,"%.2f",ct->argu_d[2]);
    fl_set_input(INPUTS_TP9[6],inval);
    fl_deactivate_object(INPUTS_TP9[6]);
    Dval[2] = ct->argu_d[2];
  }else{
    fl_set_object_color(INPUTS_TP9[6],FL_CYAN,FL_CYAN);
  }
  
  //Singularity Jnt
  g3d_create_input(&INPUTS_TP9[7],FL_NORMAL_INPUT,33,20,"Sing Jnt",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[7],CB_inputs_tp9_obj,7);
  if(ct != NULL) {
    sprintf(inval,"%d",(int)((ct->singularities[0])->singJntVal[0])->jntNum);
    fl_set_input(INPUTS_TP9[7],inval);
    fl_deactivate_object(INPUTS_TP9[7]);
    Dval[3] = ((ct->singularities[0])->singJntVal[0])->jntNum;
  }else{
    fl_set_object_color(INPUTS_TP9[7],FL_CYAN,FL_CYAN);
  }
  
  //Singularity Val
  g3d_create_input(&INPUTS_TP9[8],FL_NORMAL_INPUT,33,20,"Sing Val",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[8],CB_inputs_tp9_obj,8);
  if(ct != NULL) {
    sprintf(inval,"%.2f",((ct->singularities[0])->singJntVal[0])->val[0]);
    fl_set_input(INPUTS_TP9[8],inval);
    Dval[4] = ((ct->singularities[0])->singJntVal[0])->val[0];
  }else{
    fl_set_object_color(INPUTS_TP9[8],FL_CYAN,FL_CYAN);
  }

  //Singularity Val
  g3d_create_input(&INPUTS_TP9[9],FL_NORMAL_INPUT,33,20,"Ik Sol",(void**)&MAIN_FRAME_TP9,0);
  fl_set_call_back(INPUTS_TP9[9],CB_inputs_tp9_obj,9);
  if(ct != NULL) {
    sprintf(inval,"%d",ct->argu_i[0]);
    fl_set_input(INPUTS_TP9[9],inval);
    Ival[0] = ct->argu_i[0];
  }else{
    fl_set_object_color(INPUTS_TP9[9],FL_CYAN,FL_CYAN);
  }
  
  g3d_create_frame(&VALID_FRAME_TP9,FL_NO_FRAME,-1,-1,"",(void**)&CNTRT_TP9_FORM,1);
  //Ok button
  g3d_create_button(&OK_TP9,FL_PUSH_BUTTON,40,20,"Ok",(void**)&VALID_FRAME_TP9,0);
  if(ct == NULL) fl_set_call_back(OK_TP9,CB_ok_tp9_obj,-1);
  else fl_set_call_back(OK_TP9,CB_ok_tp9_obj,ct->num);

  //Cancel button
  g3d_create_button(&CANCEL_TP9,FL_PUSH_BUTTON,40,20,"Cancel",(void**)&VALID_FRAME_TP9,0);
  if(ct == NULL) fl_set_call_back(CANCEL_TP9,CB_cancel_tp9_obj,-1);
  else fl_set_call_back(CANCEL_TP9,CB_cancel_tp9_obj,ct->num);

  fl_end_form();
  fl_set_form_icon(CNTRT_TP9_FORM, GetApplicationIcon(), 0);
  fl_set_form_atclose(CNTRT_TP9_FORM, CB_cntrt_tp9_OnClose, NULL);
}

static void g3d_delete_cntrt_tp9_form(void)
{
  int i = 0;

  fl_hide_form(CNTRT_TP9_FORM);
  for(i = 0; i < 10; i++){
    g3d_fl_free_object(INPUTS_TP9[i]);
  }
  g3d_fl_free_object(MAIN_FRAME_TP9);
  g3d_fl_free_form(CNTRT_TP9_FORM);
}

static void CB_inputs_tp9_obj(FL_OBJECT *ob, long arg)
{
  switch(arg) {
    case 0:{
      Jpasiv[0] = atoi(fl_get_input(ob));
      break;
    }
    case 1:{
      Jpasiv[1] = atoi(fl_get_input(ob));
      break;
    }
    case 2:{
      Jpasiv[2] = atoi(fl_get_input(ob));
      break;
    }
    case 3:{
      Jactiv[0] = atoi(fl_get_input(ob));
      break;
    }
    case 4:{
      Dval[0] = atof(fl_get_input(ob));
      break;
    }
    case 5:{
      Dval[1] = atof(fl_get_input(ob));
      break;
    }
    case 6:{
      Dval[2] = atof(fl_get_input(ob));
      break;
    }
    case 7:{
      Dval[3]= atof(fl_get_input(ob));
      break;
    }
    case 8:{
      Dval[4] = atof(fl_get_input(ob));
      break;
    }
    case 9:{
      Ival[0] = atoi(fl_get_input(ob));
      break;
    }
  }
}

static void CB_ok_tp9_obj(FL_OBJECT *ob, long arg)
{ p3d_rob *r;
/*   p3d_cntrt *ct; */

  if(arg == -1) {
    if(p3d_constraint("p3d_3R_arm_ik", -1, Jpasiv, -1, Jactiv,
          -1, Dval, -1, Ival, -1, 1)) {
      g3d_delete_cntrt_tp9_form();
      g3d_delete_constraints_form();
      free(CONSTRAINTS_FORM);
      g3d_kin_constraints_form();
      fl_activate_all_forms();
      fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
  else {
    r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 0);
    if(p3d_constraint("p3d_3R_arm_ik", -1, Jpasiv, -1, Jactiv,
          -1, Dval, -1, Ival, arg, 1)) {
      p3d_update_jnts_state(r->cntrt_manager,r->cntrt_manager->cntrts[arg], 1);
      g3d_delete_cntrt_tp9_form();
      if(!fl_get_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num])) {
        fl_set_button(CONSTRAINTS_FORM->LIST_OBJ[r->cntrt_manager->cntrts[arg]->num],1);
      }
      fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
    }
    else {
      fl_set_button(ob,0);
    }
  }
}

static void CB_cancel_tp9_obj(FL_OBJECT *ob, long arg)
{
  g3d_delete_cntrt_tp9_form();
  if(arg == -1) {
    fl_activate_form(CONSTRAINTS_SETTING_FORM->CNTRT_SET_FORM);
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
  else {
    fl_activate_form(CONSTRAINTS_FORM->CNTRT_FORM);
  }
}

static int CB_cntrt_tp9_OnClose(FL_FORM *form, void *arg){
  g3d_delete_cntrt_tp9_form();
  //If we return FL_OK, the application will continue to try to shut down itself
  //   if however we return FL_IGNORE, the application will not continue this event
  return FL_IGNORE;
}
/* fin cntrt_tp9 */

/* ------------------------------------------------------------------ */


#include "Graphic-pkg.h"
#include "UserAppli-pkg.h"
#include "GL/glx.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"

FL_FORM *USER_APPLI_FORM = NULL;
static void callbacks(FL_OBJECT *ob, long arg);
static int CB_userAppliForm_OnClose(FL_FORM *form, void *arg);
//PLANNING
static FL_OBJECT  *OFFLINE_FRAME;
static FL_OBJECT  *OFFLINE;
static FL_OBJECT  *OFFLINE_OPEN;
static FL_OBJECT  *OFFLINE_CLOSED;
static FL_OBJECT  *COMPUTE_PATH_FRAME;
static FL_OBJECT  *COMPUTE_PATH;
static FL_OBJECT  *COMPUTE_PATH_OPEN;
static FL_OBJECT  *COMPUTE_PATH_CLOSED;
static FL_OBJECT  *COMPUTE_PATH_NEAR;
//VISUALIZATION
static FL_OBJECT  *VISUALIZATION_FRAME;
static FL_OBJECT  *SHOW_TRAJ;
static FL_OBJECT  *CANVAS;
//SET POS
static FL_OBJECT  *SET_POS_FRAME;
static FL_OBJECT  *SET_INIT_OBJECT_POS;
static FL_OBJECT  *SET_GOTO_OBJECT_POS;
//MISC
static G3D_Window *win;
static int G3D_GLCONFIG[30] = { /* pas utilise... servirait pour le stencil */
  GLX_RGBA, GLX_DEPTH_SIZE, 1,
  GLX_RED_SIZE, 1, GLX_GREEN_SIZE, 1, GLX_BLUE_SIZE, 1,
  GLX_STENCIL_SIZE, 1,
  GLX_DOUBLEBUFFER,
  None
};

extern FL_OBJECT  *user_obj;

void g3d_create_user_appli_form(void){
  win = (G3D_Window *)malloc(sizeof(G3D_Window));
  g3d_create_form(&USER_APPLI_FORM, 300, 400, FL_UP_BOX);
  g3d_create_labelframe(&OFFLINE_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Offline", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&OFFLINE,FL_NORMAL_BUTTON,-1,30.0,"Offline planner",(void**)&OFFLINE_FRAME,0);
  fl_set_call_back(OFFLINE,callbacks,0);
  g3d_create_button(&OFFLINE_OPEN,FL_NORMAL_BUTTON,-1,30.0,"Offline open chain planner",(void**)&OFFLINE_FRAME,0);
  fl_set_call_back(OFFLINE_OPEN,callbacks,1);
  g3d_create_button(&OFFLINE_CLOSED,FL_NORMAL_BUTTON,-1,30.0,"Offline closed chain planner",(void**)&OFFLINE_FRAME,0);
  fl_set_call_back(OFFLINE_CLOSED,callbacks,2);

  g3d_create_labelframe(&COMPUTE_PATH_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Path computing", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&COMPUTE_PATH,FL_NORMAL_BUTTON,-1,30.0,"Compute path",(void**)&COMPUTE_PATH_FRAME,0);
  fl_set_call_back(COMPUTE_PATH,callbacks,3);
  g3d_create_button(&COMPUTE_PATH_OPEN,FL_NORMAL_BUTTON,-1,30.0,"Open chain Path",(void**)&COMPUTE_PATH_FRAME,0);
  fl_set_call_back(COMPUTE_PATH_OPEN,callbacks,4);
  g3d_create_button(&COMPUTE_PATH_CLOSED,FL_NORMAL_BUTTON,-1,30.0,"Closed chain path",(void**)&COMPUTE_PATH_FRAME,0);
  fl_set_call_back(COMPUTE_PATH_CLOSED,callbacks,5);
  g3d_create_button(&COMPUTE_PATH_NEAR,FL_NORMAL_BUTTON,-1,30.0,"Grasp position path",(void**)&COMPUTE_PATH_FRAME,0);
  fl_set_call_back(COMPUTE_PATH_NEAR,callbacks,6);

  g3d_create_labelframe(&VISUALIZATION_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Visualization", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&SHOW_TRAJ,FL_NORMAL_BUTTON,-1,30.0,"Play trajectory",(void**)&VISUALIZATION_FRAME,0);
  fl_set_call_back(SHOW_TRAJ,callbacks,7);

  g3d_create_labelframe(&SET_POS_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Set Object Position", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&SET_INIT_OBJECT_POS,FL_NORMAL_BUTTON,-1,30.0,"Init",(void**)&SET_POS_FRAME,0);
  fl_set_call_back(SET_INIT_OBJECT_POS,callbacks,8);
  g3d_create_button(&SET_GOTO_OBJECT_POS,FL_NORMAL_BUTTON,-1,30.0,"Goto",(void**)&SET_POS_FRAME,0);
  fl_set_call_back(SET_GOTO_OBJECT_POS,callbacks,9);
  
//   double size = 0.0, x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0, z1 = 0.0, z2 = 0.0;
//   extern p3d_matrix4 Id;
//   extern G3D_Window * G3D_WINDOW_LST;
//   if(p3d_get_desc_number(P3D_ENV)) {
//     p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2); 
//     size = MAX(MAX(x2-x1,y2-y1),z2-z1);
//     x1 = .5*(x1+x2); y1 = .5*(y1+y2); z1 = .5*(z1+z2);
//   }
//   
//   CANVAS = fl_add_glcanvas(FL_NORMAL_CANVAS, 10, 10, 640, 480, "Canvas");
//   /* Les parametres de la fenetre */
//   win->form       = (void *)USER_APPLI_FORM;
//   win->canvas     = (void *)CANVAS;
//   win->size       = size;
//   win->FILAIRE = 0;
//   win->CONTOUR = 0;
//   win->GOURAUD = 0;
//   win->ACTIVE = 1;
//   win->list = -1;
//   win->fct_draw   = NULL;
//   win->next       = NULL;
//   win->fct_mobcam   = NULL;
//   win->cam_frame  = &Id;
//   win->mcamera_but  = NULL;
//   sprintf(win->name, "%s", "Move3D");
//   g3d_set_win_camera(win, .0, .0, .0, 2*size, INIT_AZ, INIT_EL, .0, .0, 1.0);
//   g3d_save_win_camera(win);
//   g3d_set_win_bgcolor(win, 1.0, 1.0, 1.0);
//   win->next = G3D_WINDOW_LST;
//   G3D_WINDOW_LST = win;
// 
//   /* Attributs/Handlers du canvas */
//   fl_set_glcanvas_attributes(CANVAS, G3D_GLCONFIG);
//   fl_set_object_gravity(CANVAS, FL_NorthWest, FL_SouthEast);
// 
//   fl_add_canvas_handler(CANVAS, Expose, canvas_expose, (void *)win);
//   fl_add_canvas_handler(CANVAS, ButtonPress, canvas_viewing, (void *)win);
//   fl_add_canvas_handler(CANVAS, ButtonRelease, canvas_viewing, (void *)win);
//   fl_add_canvas_handler(CANVAS, MotionNotify, canvas_viewing, (void *)win);
//   g3d_set_win_drawer(win, g3d_draw);
  
  fl_end_form();
  fl_set_form_atclose(USER_APPLI_FORM, CB_userAppliForm_OnClose, 0);
}

void g3d_delete_user_appli_form(void)
{
  //PLANNING
  g3d_fl_free_object(OFFLINE);
  g3d_fl_free_object(OFFLINE_OPEN);
  g3d_fl_free_object(OFFLINE_CLOSED);
  g3d_fl_free_object(OFFLINE_FRAME);
  g3d_fl_free_object(COMPUTE_PATH);
  g3d_fl_free_object(COMPUTE_PATH_OPEN);
  g3d_fl_free_object(COMPUTE_PATH_CLOSED);
  g3d_fl_free_object(COMPUTE_PATH_NEAR);
  g3d_fl_free_object(COMPUTE_PATH_FRAME);
  //VISUALIZATION
  g3d_fl_free_object(SHOW_TRAJ);
  g3d_fl_free_object(VISUALIZATION_FRAME);
  //SET POS
  g3d_fl_free_object(SET_INIT_OBJECT_POS);
  g3d_fl_free_object(SET_GOTO_OBJECT_POS);
  g3d_fl_free_object(SET_POS_FRAME);
  g3d_fl_free_form(USER_APPLI_FORM);
}

/****************************************************************************/
/** \brief This function is called when the "User appli" window is closed from the X button
  If we do not use this call back, XForms tries to close the entire application
    and we do not want that. Instead we will just click on the Cancel button
 \param *form a pointer on the FL_FORM
 \param *arg argument for the call back function (not used)
 \return FL_IGNORE.
 */
/****************************************************************************/
static int CB_userAppliForm_OnClose(FL_FORM *form, void *arg)
{
  //Call the fonction closing the form.
  g3d_delete_user_appli_form();
  fl_set_button(user_obj,0);//release the button path_Deformation on planner FORM
  //If we return FL_OK, the application will continue to try to shut down itself
  //   if however we return FL_IGNORE, the application will not continue this event
  return FL_IGNORE;
}

static void callbacks(FL_OBJECT *ob, long arg){
  p3d_matrix4 att1 = {{0.173,0,0.984,20},
                    {0.171,0.984,-0.030,-590},
                    {-0.969,0.173,0.171,40},
                    {0,0,0,1}};
  p3d_matrix4 att2 = {{0.173,0,-0.984,20},
                    {-0.171,-0.984,-0.030,590},
                    {-0.969,0.173,-0.171,40},
                    {0,0,0,1}};
  p3d_set_and_update_robot_conf_multisol(XYZ_ROBOT->ROBOT_POS, NULL);
  static p3d_matrix4 objectInitPos, objectGotoPos;
  switch (arg){
    case 0:{
      openChainPlannerOptions();
      globalPlanner();
      closedChainPlannerOptions();
      globalPlanner();
      break;
    }
    case 1:{
      openChainPlannerOptions();
      globalPlanner();
      break;
    }
    case 2:{
      closedChainPlannerOptions();
      globalPlanner();
      break;
    }
    case 3:{
//       p3d_obj *o1Pt, *o2Pt;
//       p3d_kcd_collision_test();
//       p3d_kcd_get_pairObjInCollision ( &o1Pt, &o2Pt );
//       printf("colliding pair: %s %s\n", o1Pt->name, o2Pt->name);
//       p3d_dpgGrid * grid = NULL;
//       grid = p3d_allocDPGGrid();
//       p3d_initDPGGrid(XYZ_ENV, grid);
//       buildEnvEdges(XYZ_ENV);
//       p3d_initStaticGrid(XYZ_ENV, grid);
      
      configPt approachConf = setTwoArmsRobotGraspApproachPos(XYZ_ROBOT, objectInitPos, att1, att2);
      p3d_set_and_update_robot_conf(approachConf);
      g3d_refresh_allwin_active();
      sleep(2);
      
      configPt conf = setBodyConfigForBaseMovement(XYZ_ROBOT, approachConf, XYZ_ROBOT->defaultConf);
      p3d_set_and_update_robot_conf(conf);
      g3d_refresh_allwin_active();
      sleep(2);
      
      
//       pickAndMoveObjectByMat(XYZ_ROBOT, objectInitPos, objectGotoPos, att1, att2);
      break;
    }
    case 4:{
      pickObjectByMat(XYZ_ROBOT, objectInitPos, att1, att2);
      break;
    }
    case 5:{
      moveObjectByMat(XYZ_ROBOT, objectGotoPos, att1, att2);
      break;
    }
    case 6:{
      graspObjectByMat(XYZ_ROBOT, objectInitPos, att1, att2);
      break;
    }
    case 7:{
      viewTraj();
//       checkForCollidingLpAlongPath();
      break;
    }
    case 8:{
      p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
      break;
    }
    case 9:{
      p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
      break;
    }
  }
}

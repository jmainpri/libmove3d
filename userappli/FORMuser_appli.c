#include "Graphic-pkg.h"
#include "UserAppli-pkg.h"
#include "GL/glx.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "P3d-pkg.h"
#include "../lightPlanner/proto/DlrPlanner.h"
#include "../lightPlanner/proto/DlrParser.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/robotPos.h"

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
static FL_OBJECT  *SAVE_TRAJ;
static FL_OBJECT  *USE_LIN;
//SET POS
static FL_OBJECT  *SET_POS_FRAME;
static FL_OBJECT  *SET_INIT_OBJECT_POS;
static FL_OBJECT  *SET_GOTO_OBJECT_POS;
//MISC
static FL_OBJECT  *MISC_FRAME;
static FL_OBJECT  *TESTMODEL;
static FL_OBJECT  *SPECIFIC_MULTI;
static FL_OBJECT  *TESTS;
static G3D_Window *win;

extern FL_OBJECT  *user_obj;

void g3d_create_user_appli_form(void){
  win = (G3D_Window *)malloc(sizeof(G3D_Window));
  g3d_create_form(&USER_APPLI_FORM, 300, 400, FL_UP_BOX);
  g3d_create_labelframe(&OFFLINE_FRAME, FL_ENGRAVED_FRAME, -1, -1, "testLP", (void**)&USER_APPLI_FORM, 1);
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
  g3d_create_button(&SAVE_TRAJ,FL_NORMAL_BUTTON,-1,30.0,"Save Traj.",(void**)&VISUALIZATION_FRAME,0);
  fl_set_call_back(SAVE_TRAJ,callbacks,10);
  g3d_create_button(&USE_LIN,FL_PUSH_BUTTON,-1,30.0,"Linear",(void**)&VISUALIZATION_FRAME,0);
  fl_set_call_back(USE_LIN,callbacks,11);

  g3d_create_labelframe(&SET_POS_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Set Object Position", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&SET_INIT_OBJECT_POS,FL_NORMAL_BUTTON,-1,30.0,"Init",(void**)&SET_POS_FRAME,0);
  fl_set_call_back(SET_INIT_OBJECT_POS,callbacks,8);
  g3d_create_button(&SET_GOTO_OBJECT_POS,FL_NORMAL_BUTTON,-1,30.0,"Goto",(void**)&SET_POS_FRAME,0);
  fl_set_call_back(SET_GOTO_OBJECT_POS,callbacks,9);

  g3d_create_labelframe(&MISC_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Set Object Position", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&TESTMODEL,FL_NORMAL_BUTTON,30.0,30.0,"Dynamic",(void**)&MISC_FRAME,0);
  fl_set_call_back(TESTMODEL,callbacks,12);
  g3d_create_button(&SPECIFIC_MULTI,FL_NORMAL_BUTTON,60.0,30.0,"specific Multi",(void**)&MISC_FRAME,0);
  fl_set_call_back(SPECIFIC_MULTI,callbacks,13);
  g3d_create_button(&TESTS,FL_NORMAL_BUTTON,60.0,30.0,"Tests",(void**)&MISC_FRAME,0);
  fl_set_call_back(TESTS,callbacks,14);

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
  //MISC
  g3d_fl_free_object(TESTMODEL);
  g3d_fl_free_object(SPECIFIC_MULTI);
  g3d_fl_free_object(TESTS);
  g3d_fl_free_object(MISC_FRAME);

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
#ifdef LIGHT_PLANNER
  p3d_matrix4 att1, att2;
  if(XYZ_ROBOT->ccCntrts != NULL){
    p3d_mat4Copy(XYZ_ROBOT->ccCntrts[0]->Tatt, att1);
    p3d_mat4Copy(XYZ_ROBOT->ccCntrts[1]->Tatt, att2);
    p3d_set_and_update_robot_conf_multisol(XYZ_ROBOT->ROBOT_POS, NULL);
  }
  static p3d_matrix4 objectInitPos, objectGotoPos;
  static int isObjectInitPosInitialised = FALSE, isObjectGotoPosInitialised = FALSE;
#endif
  switch (arg){
    case 0:{
      #ifdef DPG
checkForColPath(XYZ_ROBOT, XYZ_ROBOT->tcur, XYZ_GRAPH, XYZ_ROBOT->ROBOT_POS, XYZ_ROBOT->tcur->courbePt);
      #endif
//       nbCollisionPerSecond();
//       nbLocalPathPerSecond();
      break;
    }
    case 1:{
#ifdef LIGHT_PLANNER
      if(!isObjectGotoPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_GOTO);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
        isObjectGotoPosInitialised = TRUE;
      }
      platformCarryObjectByMat(XYZ_ROBOT, objectGotoPos, att1, att2);
#endif
      break;
    }
    case 2:{
#ifdef LIGHT_PLANNER
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
      computeOfflineClosedChain(XYZ_ROBOT, objectInitPos);
#endif
      break;
    }
    case 3:{
//       nbLocalPathPerSecond();
//       nbCollisionPerSecond();
#ifdef LIGHT_PLANNER
      DlrPlanner* planner = new DlrPlanner("./trajFile");
      DlrParser parser("./planner_input.txt", planner);
      parser.parse();
      planner->process();
#endif
      break;
    }
    case 4:{
#ifdef LIGHT_PLANNER
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
			platformGotoObjectByMat(XYZ_ROBOT, objectInitPos, att1, att2);
#endif
      break;
    }
    case 5:{
#ifdef LIGHT_PLANNER
      if(!isObjectGotoPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_GOTO);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
        isObjectGotoPosInitialised = TRUE;
      }
			carryObject(XYZ_ROBOT, objectGotoPos, att1, att2);
#endif
      break;
    }
    case 6:{
#ifdef LIGHT_PLANNER
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
      gotoObjectByMat(XYZ_ROBOT, objectInitPos, att1, att2);
#endif
      break;
    }
    case 7:{
      viewTraj();
      break;
    }
    case 8:{
#ifdef LIGHT_PLANNER
      p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
      isObjectInitPosInitialised = TRUE;
#endif
      break;
    }
    case 9:{
#ifdef LIGHT_PLANNER
      p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
      isObjectGotoPosInitialised = TRUE;
#endif
      break;
    }
    case 10:{
#ifdef LIGHT_PLANNER
      saveTrajInFile("./trajFile.txt", (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ), 1, p3d_get_env_graphic_dmax());
#endif
      break;
    }
    case 11:{
#ifdef LIGHT_PLANNER
      fixJoint(XYZ_ROBOT, XYZ_ROBOT->baseJnt, XYZ_ROBOT->baseJnt->jnt_mat);
//       fixJoint(XYZ_ROBOT, XYZ_ROBOT->objectJnt, XYZ_ROBOT->objectJnt->jnt_mat);
      shootTheObjectArroundTheBase(XYZ_ROBOT, XYZ_ROBOT->baseJnt,XYZ_ROBOT->objectJnt, -1);
      deactivateHandsVsObjectCol(XYZ_ROBOT);
#endif
      break;
    }
    case 12:{
      p3d_rob* robotToMove = XYZ_ENV->robot[1];
      configPt computerConfig = p3d_get_robot_config(robotToMove);
//       computerConfig[10] = -1.85;
      computerConfig[6] = 0.44;
      p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
      g3d_draw_allwin_active();

//       p3d_rob* robotToMove = XYZ_ENV->robot[10];
//       configPt computerConfig = p3d_get_robot_config(robotToMove);
//       computerConfig[10] = -1.9198;
//       p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
//       robotToMove = XYZ_ENV->robot[1];
//       p3d_destroy_config(robotToMove, computerConfig);
//       computerConfig = p3d_get_robot_config(robotToMove);
//       computerConfig[6] = 5.75;
//       computerConfig[7] = -2.97;
//       computerConfig[17] = 0.399337357;
//       computerConfig[18] = -0.122876709938079;
//       computerConfig[19] = -0.430068353883584;
//       computerConfig[20] = -0.0307192254810733;
//       computerConfig[24] = -1.01373245120011;
//       computerConfig[25] = 0.0307192254810733;
//       computerConfig[26] = 1.19804736775424;
//       computerConfig[27] = 1.04445167668118;
//       computerConfig[28] = -0.18431496891401;
//       computerConfig[29] = 1.35164344279973;
//       computerConfig[30] = -0.27647243591772;
//       computerConfig[31] = 0.675821721399865;
//       computerConfig[32] = -0.184314968914011;
//       computerConfig[34] = -0.860136707767169;
//       computerConfig[35] = 0.153595725979646;
//       computerConfig[36] = 0.0307192254810733;
//       computerConfig[41] = -0.0921574844570056;
//       p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
//       p3d_destroy_config(robotToMove, computerConfig);
//       g3d_draw_allwin_active();
      
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//       p3d_destroy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//       XYZ_ROBOT->ROBOT_GOTO = p3d_get_robot_config(XYZ_ROBOT);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//       p3d_destroy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//       XYZ_ROBOT->ROBOT_POS = p3d_get_robot_config(XYZ_ROBOT);
//       double trajLength = p3d_compute_traj_length(XYZ_ROBOT->tcur);
//       int success = false;
//       p3d_rob* robotToMove = XYZ_ENV->robot[10];
//       configPt computerConfig = p3d_get_robot_config(robotToMove);
//       configPt robotConfig = p3d_get_robot_config(XYZ_ROBOT);
//       do{
//         double randomPos = p3d_random(0, trajLength);
//         configPt randomConfig = p3d_config_at_distance_along_traj(XYZ_ROBOT->tcur, randomPos);
//         p3d_set_and_update_this_robot_conf(XYZ_ROBOT, randomConfig);
//         double x, y, z, rx, ry, rz;
//         p3d_mat4ExtractPosReverseOrder(XYZ_ROBOT->joints[10]->abs_pos, &x, &y, &z, &rx, &ry, &rz);
//         computerConfig[6] = x;
//         computerConfig[7] = y;
//         computerConfig[8] = z;
//         computerConfig[9] = rx;
//         computerConfig[10] = ry;
//         computerConfig[11] = rz;
// 
//         p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
//         p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//         success = !p3d_col_test();
//         p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//         success *= !p3d_col_test();
//       }while(success == false);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, robotConfig);
//       g3d_draw_allwin_active();


      
      computerConfig[41] = -0.0921574844570056;
      p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
      p3d_destroy_config(robotToMove, computerConfig);
      
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//       p3d_destroy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//       XYZ_ROBOT->ROBOT_GOTO = p3d_get_robot_config(XYZ_ROBOT);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//       p3d_destroy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//       XYZ_ROBOT->ROBOT_POS = p3d_get_robot_config(XYZ_ROBOT);
//       double trajLength = p3d_compute_traj_length(XYZ_ROBOT->tcur);
//       int success = false;
//       p3d_rob* robotToMove = XYZ_ENV->robot[10];
//       configPt computerConfig = p3d_get_robot_config(robotToMove);
//       configPt robotConfig = p3d_get_robot_config(XYZ_ROBOT);
//       do{
//         double randomPos = p3d_random(0, trajLength);
//         configPt randomConfig = p3d_config_at_distance_along_traj(XYZ_ROBOT->tcur, randomPos);
//         p3d_set_and_update_this_robot_conf(XYZ_ROBOT, randomConfig);
//         double x, y, z, rx, ry, rz;
//         p3d_mat4ExtractPosReverseOrder(XYZ_ROBOT->joints[10]->abs_pos, &x, &y, &z, &rx, &ry, &rz);
//         computerConfig[6] = x;
//         computerConfig[7] = y;
//         computerConfig[8] = z;
//         computerConfig[9] = rx;
//         computerConfig[10] = ry;
//         computerConfig[11] = rz;
// 
//         p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
//         p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//         success = !p3d_col_test();
//         p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//         success *= !p3d_col_test();
//       }while(success == false);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, robotConfig);
//       g3d_draw_allwin_active();


      
      break;
    }
    case 13:{
#ifdef MULTIGRAPH
     p3d_specificSuperGraphLearn();
#endif
      break;
    }
    case 14:{
     p3d_computeTests();
      break;
    }
  }
}

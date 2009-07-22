#include "Graphic-pkg.h"
#include "UserAppli-pkg.h"
#include "GL/glx.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
// #include "Util-pkg.h"

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
static G3D_Window *win;

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
  g3d_create_button(&SAVE_TRAJ,FL_NORMAL_BUTTON,-1,30.0,"Save Traj.",(void**)&VISUALIZATION_FRAME,0);
  fl_set_call_back(SAVE_TRAJ,callbacks,10);
  g3d_create_button(&USE_LIN,FL_PUSH_BUTTON,-1,30.0,"Linear",(void**)&VISUALIZATION_FRAME,0);
  fl_set_call_back(USE_LIN,callbacks,11);

  g3d_create_labelframe(&SET_POS_FRAME, FL_ENGRAVED_FRAME, -1, -1, "Set Object Position", (void**)&USER_APPLI_FORM, 1);
  g3d_create_button(&SET_INIT_OBJECT_POS,FL_NORMAL_BUTTON,-1,30.0,"Init",(void**)&SET_POS_FRAME,0);
  fl_set_call_back(SET_INIT_OBJECT_POS,callbacks,8);
  g3d_create_button(&SET_GOTO_OBJECT_POS,FL_NORMAL_BUTTON,-1,30.0,"Goto",(void**)&SET_POS_FRAME,0);
  fl_set_call_back(SET_GOTO_OBJECT_POS,callbacks,9);

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
  p3d_matrix4 att1, att2;
  p3d_mat4Copy(XYZ_ROBOT->ccCntrts[0]->Tatt, att1);
  p3d_mat4Copy(XYZ_ROBOT->ccCntrts[1]->Tatt, att2);
  p3d_set_and_update_robot_conf_multisol(XYZ_ROBOT->ROBOT_POS, NULL);
  static p3d_matrix4 objectInitPos, objectGotoPos;
  static int isObjectInitPosInitialised = FALSE, isObjectGotoPosInitialised = FALSE;
  switch (arg){
    case 0:{
			disableAutoCol(XYZ_ROBOT);
			p3d_col_activate_rob(XYZ_ROBOT);
//       openChainPlannerOptions();
//       globalPlanner();
//       closedChainPlannerOptions();
//       globalPlanner();
//       switchBBActivationForGrasp();

//         globalUdpClient->sendConfig(p3d_get_robot_config(XYZ_ROBOT) , XYZ_ROBOT->nb_dof);
//         std::string message;
//         while(true){
//           message = globalUdpClient->receive();
//           if(message.empty() == false){
//             std::cout << message << std::endl;
//             break;
//           }
//           std::cout << "Waiting for message" << std::endl;
//         }
//         std::string exitMessage("Exit");
//         globalUdpClient->send(exitMessage);
      break;
    }
    case 1:{
//       openChainPlannerOptions();
//       globalPlanner();
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
      computeOfflineOpenChain(XYZ_ROBOT, objectInitPos);
      break;
    }
    case 2:{
//       closedChainPlannerOptions();
//       globalPlanner();
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
      computeOfflineClosedChain(XYZ_ROBOT, objectInitPos);
      break;
    }
    case 3:{

//       p3d_dpgGrid * grid = NULL;
//       grid = p3d_allocDPGGrid();
//       p3d_initDPGGrid(XYZ_ENV, grid);
//       buildEnvEdges(XYZ_ENV);
//       p3d_initStaticGrid(XYZ_ENV, grid);

     if(!isObjectInitPosInitialised){
       p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
       p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
       isObjectInitPosInitialised = TRUE;
     }
     if(!isObjectGotoPosInitialised){
       p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_GOTO);
       p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
       isObjectGotoPosInitialised = TRUE;
     }
     pickAndMoveObjectByMat(XYZ_ROBOT, objectInitPos, objectGotoPos, att1, att2);
      break;
    }
    case 4:{
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
      pickObjectByMat(XYZ_ROBOT, objectInitPos, att1, att2);
      break;
    }
    case 5:{
      if(!isObjectGotoPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_GOTO);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
        isObjectGotoPosInitialised = TRUE;
      }
      moveObjectByMat(XYZ_ROBOT, objectGotoPos, att1, att2);
      break;
    }
    case 6:{
      if(!isObjectInitPosInitialised){
        p3d_set_and_update_robot_conf(XYZ_ROBOT->ROBOT_POS);
        p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
        isObjectInitPosInitialised = TRUE;
      }
      graspObjectByMat(XYZ_ROBOT, objectInitPos, att1, att2);
//       graspObjectByConf(XYZ_ROBOT, objectInitPos, p3d_copy_config(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS),p3d_copy_config(XYZ_ROBOT,  XYZ_ROBOT->ROBOT_GOTO));
      break;
    }
    case 7:{
      viewTraj();
//       checkForCollidingLpAlongPath();
      break;
    }
    case 8:{
      p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectInitPos);
      isObjectInitPosInitialised = TRUE;
      break;
    }
    case 9:{
      p3d_mat4Copy(XYZ_ROBOT->objectJnt->jnt_mat, objectGotoPos);
      isObjectGotoPosInitialised = TRUE;
      break;
    }
    case 10:{
      saveTrajInFile((p3d_traj*) p3d_get_desc_curid(P3D_TRAJ));
      break;
    }
    case 11:{
      setLinearLp(fl_get_button(ob));
      break;
    }
  }
}

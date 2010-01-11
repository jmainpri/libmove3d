#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

static FL_FORM * ARI = NULL;

static FL_OBJECT * ARI_CHOOSE_FILE;
static FL_OBJECT * ARI_CHOOSE_ROB;
static FL_OBJECT * ARI_FILE_SPEC;
static FL_OBJECT * ARI_CLOSE;

extern p3d_joint_list *  Chain[5];

static void anim_update_file_spec(void) {
  char text[1024] = "";
  char word[30];
  int LChain;
  p3d_joint_list * JointList;

  for (LChain=0; LChain < 5; LChain++) {
    sprintf(word,"Chain %d ", LChain);
    strcat (text,word);
    JointList = Chain[LChain];
    while (JointList != NULL) {
      sprintf(word,"%d ",JointList->NumJoint);
      strcat(text,word);
      JointList = JointList->next;
    }
    strcat(text,"\n");
  }
  fl_set_object_label(ARI_FILE_SPEC, text);
}

static void anim_ari_cb (FL_OBJECT * obj, long arg) {
  int RobotNum;
  p3d_rob * RobotPt;
  char * File;
  switch (arg) {
  case 10 : 
    File = fl_show_fselector("Reactive Definition Filename", p3d_rw_scenario_get_path(),"*.rdef"," ");
    if (File) {
      RobotNum = fl_get_choice(ARI_CHOOSE_ROB) -1;
      RobotPt = p3d_sel_desc_num(P3D_ROBOT,RobotNum);
      anim_reactive_definition_file(File,RobotPt);
      anim_update_file_spec();
      anim_interface_update();
    }
    break;
  case 11 :
    fl_hide_form(ARI);
    fl_free_form(ARI);
    ARI = NULL;
    anim_interface_update();
    break;
  default :
    PrintWarning (("anim_reactive_interface.c -- unknown command"));
    break;
  }
}

void anim_reactive_interface (void) {
  int LRob;

  if (ARI != NULL) {
    fl_hide_form(ARI);
    fl_free_form(ARI);
    ARI = NULL;
  }

  ARI = fl_bgn_form(FL_UP_BOX, 200, 310);

  ARI_CHOOSE_ROB  = fl_add_choice(FL_NORMAL_CHOICE, 100, 10, 80, 20, "Associate to robot : ");
  for(LRob = 0; LRob < p3d_get_desc_number(P3D_ROBOT); LRob++) {
    p3d_sel_desc_num(P3D_ROBOT,LRob);
    fl_addto_choice(ARI_CHOOSE_ROB, p3d_get_desc_curname(P3D_ROBOT));
  }
  
  ARI_CHOOSE_FILE = fl_add_button (FL_NORMAL_BUTTON, 10, 40, 180, 20, "Choose File to Load");
  fl_set_object_callback(ARI_CHOOSE_FILE, anim_ari_cb, 10);
  
  ARI_FILE_SPEC = fl_add_text(FL_NORMAL_TEXT, 10, 70, 180, 100, "");

  ARI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 280, 180, 20, "Close");
  fl_set_object_callback(ARI_CLOSE, anim_ari_cb, 11);

  fl_end_form();
  fl_show_form (ARI, FL_PLACE_SIZE, FL_FULLBORDER, "Reactive Scheme Loader");
}

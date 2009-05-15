#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

static FL_FORM * ANIM_INTERFACE = NULL;
static FL_OBJECT * ANIM_LOAD_FILE;
static FL_OBJECT * ANIM_PROCESS_MCAP;
static FL_OBJECT * ANIM_REACTIVE;
static FL_OBJECT * ANIM_COMPUTE_PATH;
static FL_OBJECT * ANIM_OPTIMIZE_PATH;
static FL_OBJECT * ANIM_SAMPLE_PATH;
static FL_OBJECT * ANIM_CHARAC_TRAJ;
static FL_OBJECT * ANIM_WALK_CONTROL;
static FL_OBJECT * ANIM_REAC_COL;

static FL_OBJECT * ANIM_LOAD_RAP;
static FL_OBJECT * ANIM_PROC_RAP;
static FL_OBJECT * ANIM_RDEF_RAP;
static FL_OBJECT * ANIM_CPATH_RAP;
static FL_OBJECT * ANIM_0PATH_RAP;
static FL_OBJECT * ANIM_SPATH_RAP;
static FL_OBJECT * ANIM_CTRAJ_RAP;
static FL_OBJECT * ANIM_WALK_RAP;
static FL_OBJECT * ANIM_REAC_COL_RAP;


static FL_OBJECT * ANIM_BOX1;
static FL_OBJECT * ANIM_BOX2;
static FL_OBJECT * ANIM_TEXT1;
static FL_OBJECT * ANIM_TEXT2;
static FL_OBJECT * ANIM_TEXT3;

static FL_OBJECT * ANIM_WHOLE_LIB_DEF;
static FL_OBJECT * ANIM_WHOLE_PATH_DEF;

static AnimProb ZANIM;
static int AnimProblemInitialized = 0;

static p3d_rob * anim_find_robot_pointer_from_name (const char RobotName[255]) 
{
  p3d_rob * RobotPt;
  int RobotIndex;
  
  RobotIndex = p3d_get_rob_nid_by_name((char *)RobotName);
  if (RobotIndex <0) {
    PrintWarning (("anim_interface.c -- Le robot %s est associe au fichier de capture de mouvement en cours de lecture. Ce robot n'existe pas. Le programme est interrompu.\n",RobotName));
    exit(101);
  }
  RobotPt    = XYZ_ENV->robot[RobotIndex];
  return RobotPt;
}

static void anim_initialize_animation_problem(void) {
  if (!AnimProblemInitialized) {
    ZANIM.Qi = NULL;
    ZANIM.Qf = NULL;
    ZANIM.Robot = NULL;
    ZANIM.RobotMin = NULL;
    ZANIM.FindTrajResult = NULL;
    ZANIM.OptimizedResult = NULL;
    ZANIM.SamplingRes.NofFrames = 0;
    ZANIM.SamplingRes.AnimBuffer = NULL;
    ZANIM.WalkContRes.NofFrames = 0;
    ZANIM.WalkContRes.AnimBuffer = NULL;
    ZANIM.ColAvoidRes.NofFrames = 0;
    ZANIM.ColAvoidRes.AnimBuffer = NULL;
    ZANIM.ColOpt1Res.NofFrames = 0;
    ZANIM.ColOpt1Res.AnimBuffer = NULL;
    ZANIM.ColOpt2Res.NofFrames = 0;
    ZANIM.ColOpt2Res.AnimBuffer = NULL;
    ZANIM.PosPlanRes.NofFrames = 0;
    ZANIM.PosPlanRes.AnimBuffer = NULL;
    ZANIM.FrameDuration = 0;
    ZANIM.InitRRTGraph = NULL;
    ZANIM.FinalRRTGraph = NULL;
    ZANIM.TrajCharacterized = 0;
  }
  AnimProblemInitialized = 1;
}

static FILE * anim_open_definition_file (const char * FileName) {
  FILE * FilePt;
  if ((FilePt = fopen(FileName,"r"))==NULL) {
    PrintInfo(("anim_interface.c -- Le fichier %s n'a pu etre
 ouvert. Le programme est interrompu \n",FileName));
    exit (112);
  }
  fseek (FilePt,0,0);
  return FilePt;
}

static void anim_update_load_rap (void) {
  int NofAnim = XYZ_ANIM.nof_animation;
  char text[255];
  sprintf(text, "%d MOTION CAPTURES LOADED", NofAnim);
  fl_set_object_label(ANIM_LOAD_RAP, text);
  if (NofAnim == 0)
    fl_set_object_color (ANIM_LOAD_RAP, FL_RED, FL_BLACK);
  else
    fl_set_object_color (ANIM_LOAD_RAP, FL_GREEN, FL_BLACK);
}

static void anim_update_proc_rap (void) {
  int NofAnim = XYZ_ANIM.nof_animation;
  int NofProcessed = 0, LAnim;
  char text[255];

  for (LAnim = 0; LAnim < NofAnim; LAnim++) 
    if (XYZ_ANIM.animation[LAnim]->processed == 1) 
      NofProcessed ++;

  sprintf(text, "%d MOTION CAPTURES PROCESSED", NofProcessed);
  fl_set_object_label(ANIM_PROC_RAP, text);
  if (NofProcessed == 0)
    fl_set_object_color (ANIM_PROC_RAP, FL_RED, FL_BLACK);
  else
    fl_set_object_color (ANIM_PROC_RAP, FL_GREEN, FL_BLACK);
}

static void anim_update_rdef_rap (void) {
  char text[255];
  if (anim_reactive_is_initialized()) {
    sprintf(text, "REACTIVE DOFs ARE DEFINED");
    fl_set_object_label(ANIM_RDEF_RAP, text);
    fl_set_object_color (ANIM_RDEF_RAP, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "REACTIVE DOFs ARE NOT DEFINED");
    fl_set_object_label(ANIM_RDEF_RAP, text);
    fl_set_object_color (ANIM_RDEF_RAP, FL_RED, FL_BLACK);
  }
}

static void anim_load_whole_library_def (void) {
  char * File = NULL;
  FILE * FilePt;
  char FileToLoad[1024];
  char RobotName[1024];
  int LAnim;
  p3d_rob * RobotPt;

  File = fl_show_fselector("Whole Library Definition Filename", p3d_rw_scenario_get_path(),"*.wldef"," ");
  if (File) {
    FilePt = anim_open_definition_file(File);
    
    fscanf (FilePt,"%s", RobotName);
    fscanf (FilePt,"%s", FileToLoad);
    anim_load_mldef(FileToLoad,RobotName);
    anim_interface_update();
    for (LAnim = 0; LAnim < XYZ_ANIM.nof_animation; LAnim ++)
      if (XYZ_ANIM.animation[LAnim]->processed == 0) 
	anim_process_mcap(XYZ_ANIM.animation[LAnim]);
    anim_interface_update();
    fscanf (FilePt,"%s", FileToLoad);  
    RobotPt = anim_find_robot_pointer_from_name(RobotName);
    anim_reactive_definition_file(FileToLoad, RobotPt);
    anim_interface_update();
    fclose(FilePt);
  }
}

static void anim_load_whole_path_def (void) {
  char * File;
  FILE * FilePt;
  char FileToLoad[1024];
  char RobotName[1024];
  int ReadInt1, ReadInt2, ReadInt3;
  float ReadFloat1, ReadFloat2, ReadFloat3;

  File = fl_show_fselector("Whole Path Definition Filename", p3d_rw_scenario_get_path(),"*.wpdef"," ");
  FilePt = anim_open_definition_file(File);
  fscanf (FilePt,"%s", FileToLoad);
  p3d_read_scenario(FileToLoad);
  fscanf (FilePt,"%s", RobotName);  
  ZANIM.Robot = anim_find_robot_pointer_from_name(RobotName);
  fscanf (FilePt,"%s", RobotName);  
  ZANIM.RobotMin = anim_find_robot_pointer_from_name(RobotName);

  anim_compute_path(&ZANIM);
  anim_interface_update();
  fscanf (FilePt,"%d %d", &ReadInt1, &ReadInt2);

  anim_optim_traj_initialize(&ZANIM);
  anim_optim_traj_start(&ZANIM, ReadInt1, ReadInt2);
  anim_interface_update();

  fscanf (FilePt,"%d %d %d %f %f %f", &ReadInt1, &ReadInt2, &ReadInt3, &ReadFloat1, &ReadFloat2, &ReadFloat3);
  anim_sample_path_initialize(&ZANIM);
  ZANIM.SampleOptions.MaxLinearSpeed  = (double)ReadFloat1;
  ZANIM.SampleOptions.MaxAngularSpeed = (double)ReadFloat2;
  ZANIM.SampleOptions.MaxAcceleration = (double)ReadFloat3;
  ZANIM.SampleOptions.FramePerSecond  = ReadInt1;
  ZANIM.SampleOptions.EnableMaxAngularSpeed = ReadInt2;
  ZANIM.SampleOptions.EnableMaxAcceleration = ReadInt3;
  anim_sample_path(&ZANIM);
  anim_interface_update();
  anim_charac_traj(&ZANIM);
  anim_interface_update();
  fclose(FilePt);
}

static void anim_update_cpath_rap (void) {
  char text[255];
  if (ZANIM.RobotMin) {
    if (ZANIM.RobotMin->nt > 0) {
      sprintf(text, "%d PATH COMPUTED", ZANIM.RobotMin->nt);
      fl_set_object_label(ANIM_CPATH_RAP, text);
      fl_set_object_color (ANIM_CPATH_RAP, FL_GREEN, FL_BLACK);
    }
  }
  else {
    sprintf(text, "NO PATH COMPUTED");
    fl_set_object_label(ANIM_CPATH_RAP, text);
    fl_set_object_color (ANIM_CPATH_RAP, FL_RED, FL_BLACK);
  }
}

static void anim_update_opath_rap (void) {
  char text[255];
  if (ZANIM.OptimizedResult != NULL) {
    sprintf(text, "AN OPTIMIZED PATH EXISTS");
    fl_set_object_label(ANIM_0PATH_RAP, text);
    fl_set_object_color (ANIM_0PATH_RAP, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "NO OPTIMIZED PATH");
    fl_set_object_label(ANIM_0PATH_RAP, text);
    fl_set_object_color (ANIM_0PATH_RAP, FL_RED, FL_BLACK);
  }
}

static void anim_update_spath_rap (void) {
  char text[255];
  if (ZANIM.SamplingRes.NofFrames > 0) {
    sprintf(text, "A SAMPLED PATH EXISTS");
    fl_set_object_label(ANIM_SPATH_RAP, text);
    fl_set_object_color (ANIM_SPATH_RAP, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "NO SAMPLED PATH");
    fl_set_object_label(ANIM_SPATH_RAP, text);
    fl_set_object_color (ANIM_SPATH_RAP, FL_RED, FL_BLACK);
  }
}

static void anim_update_ctraj_rap (void) {
  char text[255];
  if (ZANIM.TrajCharacterized > 0) {
    sprintf(text, "VELOCITIES COMPUTED");
    fl_set_object_label(ANIM_CTRAJ_RAP, text);
    fl_set_object_color (ANIM_CTRAJ_RAP, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "VELOCITIES NOT COMPUTED");
    fl_set_object_label(ANIM_CTRAJ_RAP, text);
    fl_set_object_color (ANIM_CTRAJ_RAP, FL_RED, FL_BLACK);
  }
}

static void anim_update_walk_rap (void) {
  char text[255];
  if (ZANIM.WalkContRes.NofFrames > 0) {
    sprintf(text, "WALK ANIMATION ENABLE");
    fl_set_object_label(ANIM_WALK_RAP, text);
    fl_set_object_color (ANIM_WALK_RAP, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "NO WALK ANIMATION");
    fl_set_object_label(ANIM_WALK_RAP, text);
    fl_set_object_color (ANIM_WALK_RAP, FL_RED, FL_BLACK);
  }
}

static void anim_update_reac_col_rap (void) {
  char text[255];
  if (ZANIM.ColAvoidRes.NofFrames > 0) {
    sprintf(text, "REACTIVE AVOIDANCE ENABLE");
    fl_set_object_label(ANIM_REAC_COL_RAP, text);
    fl_set_object_color (ANIM_REAC_COL_RAP, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "NO REACTIVE AVOIDANCE");
    fl_set_object_label(ANIM_REAC_COL_RAP, text);
    fl_set_object_color (ANIM_REAC_COL_RAP, FL_RED, FL_BLACK);
  }
}

void anim_interface_update (void) {
  anim_update_load_rap();
  anim_update_proc_rap();
  anim_update_rdef_rap();
  anim_update_cpath_rap();
  anim_update_opath_rap();
  anim_update_spath_rap();
  anim_update_ctraj_rap();
  anim_update_walk_rap();
  anim_update_reac_col_rap();
}

static void anim_interface_cb (FL_OBJECT * obj, long arg) {
  switch (arg) {
  case 10 : 
    anim_load_file_interface();
    break;
  case 11 : 
    anim_process_mcap_interface();
    break;
  case 12 :
    anim_reactive_interface();
    break;
  case 101 :
    anim_load_whole_library_def();
    break;
  case 13 :
    anim_compute_path_interface(&ZANIM);
    break;
  case 14 : 
    anim_optim_path_interface(&ZANIM);
    break;
  case 15 : 
    anim_sample_path_interface(&ZANIM);
    break;
  case 16 : 
    anim_charac_traj_interface(&ZANIM);
    break;
  case 102 :
    anim_load_whole_path_def();
    break;
  case 17 : 
    anim_walk_controller_interface(&ZANIM);
    break;
  case 18 : 
    anim_reactive_collision_interface(&ZANIM);
    break;
  default :
    PrintWarning (("anim_interface.c -- unknown command"));
    break;
  }
}

void anim_interface (void) {

  anim_initialize_animation_problem();

  if (ANIM_INTERFACE != NULL) {
    fl_hide_form(ANIM_INTERFACE);
    fl_free_form(ANIM_INTERFACE);
    ANIM_INTERFACE = NULL;
  }  

  ANIM_INTERFACE = fl_bgn_form(FL_UP_BOX, 500., 500.);
  
  ANIM_TEXT1 = fl_add_text(FL_NORMAL_TEXT, 10, 10, 480, 20, "MOTION LIBRARY DEFINITION MODULES");
  fl_set_object_color(ANIM_TEXT1, FL_SLATEBLUE, FL_WHITE);
  
  ANIM_LOAD_FILE  = fl_add_button(FL_NORMAL_BUTTON, 10, 40, 180, 20, "Load Motion Captures");
  fl_set_object_callback (ANIM_LOAD_FILE, anim_interface_cb, 10);

  ANIM_LOAD_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 40, 180, 20, "");

  ANIM_PROCESS_MCAP  = fl_add_button(FL_NORMAL_BUTTON, 10, 70, 180, 20, "Process Motion Captures");
  fl_set_object_callback (ANIM_PROCESS_MCAP, anim_interface_cb, 11);

  ANIM_PROC_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 70, 180, 20, "");

  ANIM_REACTIVE  = fl_add_button(FL_NORMAL_BUTTON, 10, 100, 180, 20, "Load Reactive Definition");
  fl_set_object_callback (ANIM_REACTIVE, anim_interface_cb, 12);

  ANIM_RDEF_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 100, 180, 20, "");
  
  ANIM_WHOLE_LIB_DEF = fl_add_button(FL_NORMAL_BUTTON, 410, 40, 80, 80, "DEF FILE");
  fl_set_object_callback (ANIM_WHOLE_LIB_DEF, anim_interface_cb, 101);

  ANIM_BOX1 = fl_add_box(FL_FRAME_BOX,10,130,480,5,"");

  ANIM_TEXT2 = fl_add_text(FL_NORMAL_TEXT, 10, 140, 480, 20, "INITIAL CONFIGURATION SEQUENCE CONSTRUCTION MODULES");
  fl_set_object_color(ANIM_TEXT2, FL_SLATEBLUE, FL_WHITE);

  ANIM_COMPUTE_PATH  = fl_add_button(FL_NORMAL_BUTTON, 10, 170, 180, 20, "Compute Path");
  fl_set_object_callback (ANIM_COMPUTE_PATH, anim_interface_cb, 13);
  
  ANIM_CPATH_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 170, 180, 20, "");

  ANIM_OPTIMIZE_PATH  = fl_add_button(FL_NORMAL_BUTTON, 10, 200, 180, 20, "Optimize Path");
  fl_set_object_callback (ANIM_OPTIMIZE_PATH, anim_interface_cb, 14);

  ANIM_0PATH_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 200, 180, 20, "");

  ANIM_SAMPLE_PATH  = fl_add_button(FL_NORMAL_BUTTON, 10, 230, 180, 20, "Sample Path");
  fl_set_object_callback (ANIM_SAMPLE_PATH, anim_interface_cb, 15);

  ANIM_SPATH_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 230, 180, 20, "");

  ANIM_CHARAC_TRAJ  = fl_add_button(FL_NORMAL_BUTTON, 10, 260, 180, 20, "Charac Path");
  fl_set_object_callback (ANIM_CHARAC_TRAJ, anim_interface_cb, 16);

  ANIM_CTRAJ_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 260, 180, 20, "");

  ANIM_WHOLE_PATH_DEF  = fl_add_button(FL_NORMAL_BUTTON, 410, 170, 80, 110, "DEF FILE");
  fl_set_object_callback (ANIM_WHOLE_PATH_DEF, anim_interface_cb, 102);

  ANIM_BOX2 = fl_add_box(FL_FRAME_BOX,10,290,480,5,"");

  ANIM_TEXT3 = fl_add_text(FL_NORMAL_TEXT, 10, 300, 480, 20, "ANIMATION MODULES");
  fl_set_object_color(ANIM_TEXT3, FL_SLATEBLUE, FL_WHITE);

  ANIM_WALK_CONTROL  = fl_add_button(FL_NORMAL_BUTTON, 10, 330, 180, 20, "Walk Control");
  fl_set_object_callback (ANIM_WALK_CONTROL, anim_interface_cb, 17);

  ANIM_WALK_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 330, 180, 20, "");

  ANIM_REAC_COL = fl_add_button(FL_NORMAL_BUTTON, 10, 360, 180, 20, "Reactive Collision Avoidance");
  fl_set_object_callback (ANIM_REAC_COL, anim_interface_cb, 18);

  ANIM_REAC_COL_RAP = fl_add_text(FL_NORMAL_TEXT, 210, 360, 180, 20, "");
 
  fl_end_form();
  fl_show_form (ANIM_INTERFACE, FL_PLACE_SIZE, FL_FULLBORDER, "Animation Modules Interface");

  anim_interface_update();
}

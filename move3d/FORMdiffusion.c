#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Collision-pkg.h"


extern FL_OBJECT* Diffusion_obj;
extern FL_OBJECT* SEARCH_DRAW_OBJ;

//fl objects used in other (bio) forms
FL_OBJECT* MANHATTAN_CHECK = NULL;
FL_OBJECT* WITH_GOAL_DIR_OBJ = NULL;
FL_OBJECT* DRAW_GRAPH_OBJ = NULL;
FL_OBJECT* MY_RRT_METHOD = NULL;
FL_OBJECT* ML_RRT_METHOD = NULL;



FL_FORM* DIFFUSION_FORM = NULL;




static FL_FORM* DIRECTION_CUSTOMIZED_FORM = NULL;
static FL_FORM* NODE_CUSTOMIZED_FORM = NULL;
static FL_FORM* PROCESS_CUSTOMIZED_FORM = NULL;
static FL_FORM* DISTANCE_CUSTOMIZED_FORM = NULL;
static FL_FORM* MANHATTAN_CUSTOMIZED_FORM = NULL;
static FL_FORM* COSTSPACE_CUSTOMIZED_FORM = NULL;


static FL_OBJECT* ALL_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* CUSTOMIZED_FRAME = NULL;
static FL_OBJECT* BI_OR_MONO_GROUP = NULL;
static FL_OBJECT* BI_DIR_OBJ = NULL;
static FL_OBJECT* MONO_DIR_OBJ = NULL;
static FL_OBJECT* BI_OR_MONO_FRAME = NULL;
static FL_OBJECT* BALANCED_COMP_OBJ = NULL;
static FL_OBJECT* DIFFUSION_METHOD_GROUP = NULL;
static FL_OBJECT* DIFFUSION_METHOD_FRAME = NULL;
static FL_OBJECT* RRT_CONNECT_METHOD = NULL;
static FL_OBJECT* DIRECTION_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* NODE_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* CSPACE_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* PROCESS_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* DISTANCE_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* MANHATTAN_CUSTOMIZED_METHOD = NULL;
static FL_OBJECT* NTRY_NBNODES_FRAME = NULL;
static FL_OBJECT* N_TRY_MAX = NULL;
static FL_OBJECT* NB_NODE_COMP_MAX = NULL;
static FL_OBJECT* RUN_OBJ = NULL;
static FL_OBJECT* STOP_OBJ = NULL;
static FL_OBJECT* RESET_OBJ = NULL;
static FL_OBJECT* RUN_STOP_RESET_FRAME = NULL;
static FL_OBJECT* EXPANSION_CHOICE_OBJ = NULL;
static FL_OBJECT* DELTA_COST_CHOICE = NULL;
static FL_OBJECT* BIAS_FRAME = NULL;
static FL_OBJECT* BIAS_CHECK = NULL;
static FL_OBJECT* EXPANSION_DIRECTION_FRAME = NULL;
static FL_OBJECT* EXTENSION_FRAME = NULL;
static FL_OBJECT* COST_SPACE_FRAME0 = NULL;
static FL_OBJECT* COST_SPACE_FRAME1 = NULL;
static FL_OBJECT* COST_SPACE_FRAME2 = NULL;
static FL_OBJECT* BIAS_RATIO_SLIDER = NULL;
static FL_OBJECT* DYN_PARAM_SLIDER = NULL;
static FL_OBJECT* NODE_CHOICE_OBJ = NULL;
static FL_OBJECT* EXPANSION_NODE_FRAME = NULL;
static FL_OBJECT* KNEAR_SLIDER = NULL;
static FL_OBJECT* MAX_FAIL_SLIDER = NULL;
static FL_OBJECT* MAX_DIST_CHECK = NULL;
static FL_OBJECT* MAX_FAIL_FRAME = NULL;
static FL_OBJECT* DYN_PARAM2_SLIDER = NULL;
static FL_OBJECT* MAX_FAIL_CHECK = NULL;
static FL_OBJECT* MAX_DIST_FRAME = NULL;
static FL_OBJECT* STEP_SLIDER = NULL;
static FL_OBJECT* ADD_CYCLES_CHECK = NULL;
static FL_OBJECT* COST_SLIDER = NULL;
static FL_OBJECT* COST_TRAJ_OBJ = NULL;
static FL_OBJECT* NODE_COST_TRAJ_OBJ = NULL;
static FL_OBJECT* DIFFUSION_PROCESS_CHOICE = NULL;
static FL_OBJECT* DIFFUSION_PROCESS_FRAME = NULL;
static FL_OBJECT* DISTANCE_FRAME = NULL;
static FL_OBJECT* DISTANCE_CHOICE = NULL;
static FL_OBJECT* WEIGHT_ROTA_CHECK = NULL;
static FL_OBJECT* COST_SPACE_CHECK = NULL;
static FL_OBJECT* COST_METHOD_CHOICE = NULL;
static FL_OBJECT* LOCAL_ADAPT_CHECK = NULL;
static FL_OBJECT* GRID_ROADMAP_OBJ = NULL;
static FL_OBJECT* EXTRACT_BPATH_OBJ = NULL;
static FL_OBJECT* ALPHA_COST_SLIDER = NULL;
static FL_OBJECT* OPTIMIZE_COST_TRAJ_OBJ = NULL;
static FL_OBJECT* NB_FAIL_OPTIMIZE_COST_TRAJ_OBJ = NULL;
/* static FL_OBJECT* THRESHOLD_DOWN_SLIDER = NULL; */
static FL_OBJECT* WEIGHT_ROTA_SLIDER = NULL;
static FL_OBJECT* WEIGHT_ROTA_FRAME = NULL;
static FL_OBJECT* MANHATTAN_FRAME = NULL;
static FL_OBJECT* MANHATTAN_RATIO_SLIDER = NULL;
static FL_OBJECT* MAX_PASSI_FAIL_SLIDER = NULL;
static FL_OBJECT* EXT_PASS_FAIL_ACT_CHECK = NULL;
static FL_OBJECT* IS_EXPAND_CONTROL_CHECK = NULL;


static void CB_IsPasExtWhenAct(FL_OBJECT *obj, long arg);
static void CB_MaxPassiveExpandValue(FL_OBJECT *obj, long arg);
static void CB_ManhattanRatioValue(FL_OBJECT *obj, long arg);
static void CB_IsManhattanExp(FL_OBJECT *obj, long arg);
static void CB_DistConfigChoice(FL_OBJECT *obj, long arg);
static void CB_WeightRotationsValue(FL_OBJECT *obj, long arg);
static void CB_IsWeightedRotations(FL_OBJECT *obj, long arg);
static void CB_IsCostFunctSpace(FL_OBJECT *obj, long arg);
static void CB_CostMethodChoice(FL_OBJECT *obj, long arg);
static void CB_IsLocalCostAdapt(FL_OBJECT *obj, long arg);
static void CB_IsExpandControl(FL_OBJECT *obj, long arg);
static void CB_DeltaCostChoice(FL_OBJECT *obj, long arg);
static void CB_GridRoadmap(FL_OBJECT *obj, long arg);
static void CB_BPathExtract(FL_OBJECT *obj, long arg);
/* static void CB_ThresholdDown(FL_OBJECT *obj, long arg); */
static void CB_ExpansionChoice(FL_OBJECT *obj, long arg);
static void CB_ExendStepParam(FL_OBJECT *obj, long arg);
static void CB_CostSpaceParam(FL_OBJECT *obj, long arg);
static void CB_AlphaCostParam(FL_OBJECT *obj, long arg);
static void CB_OptimizeCostTraj(FL_OBJECT *obj, long arg);
static void CB_SetNbFailOptimCostMax(FL_OBJECT *obj, long arg);
static void CB_TrajCost(FL_OBJECT *obj, long arg);
static void CB_TrajNodeCost(FL_OBJECT *obj, long arg);
static void CB_NodeExpChoice(FL_OBJECT *obj, long arg);
static void CB_LambdaSlider(FL_OBJECT *obj, long arg);
static void CB_BiasRatioSlider(FL_OBJECT *obj, long arg);
static void CB_DiffusionMethod_obj(FL_OBJECT *obj, long arg);
static void CB_BiOrMonoDirplanner_obj(FL_OBJECT *obj, long arg);
static void CB_WithGoalExpansion_obj(FL_OBJECT *obj, long arg);
static void CB_SetNTryMax(FL_OBJECT *obj, long arg);
static void CB_SetNbNodeCompMax(FL_OBJECT *obj, long arg);
static void CB_Run(FL_OBJECT *obj, long arg);
static void CB_Stop(FL_OBJECT *obj, long arg);
static void CB_Reset(FL_OBJECT *obj, long arg);
static void CB_DrawGraph(FL_OBJECT *obj, long arg);
static void CB_ExpansionDirectChoice(FL_OBJECT *obj, long arg);
static void CB_IsGoalBias(FL_OBJECT *obj, long arg);
static void CB_IsMaxDist(FL_OBJECT *obj, long arg);
static void CB_GetIsCycles(FL_OBJECT *obj, long arg);
static void CB_IsMaxExpandNodeFail(FL_OBJECT *obj, long arg);
static void CB_ExpandNodeFailValue(FL_OBJECT *obj, long arg);
static void CB_NbNearestExpand(FL_OBJECT *obj, long arg);

static void g3d_create_sized_diffusion_form(int w, int h);
static void g3d_create_DiffusionMethod_obj(void);
static void g3d_delete_DiffusionMethod_obj(void);
static void g3d_create_BiOrMonoDir_Frame_obj(void);
static void g3d_delete_BiOrMonoDir_Frame_obj(void);
static void g3d_create_NTryNbNodes_obj(void);
static void g3d_delete_NTryNbNodes_obj(void);
static void g3d_create_RunStopReset_obj(void);
static void g3d_delete_RunStopReset_obj(void);

static void g3d_create_DirectionCustomizedForm(void);
static void g3d_create_NodeCustomizedForm(void);
static void g3d_create_ProcessCustomizedForm(void);
static void g3d_create_DistanceCustomizedForm(void);
static void g3d_create_ManhattanCustomizedForm(void);
static void g3d_create_CostSpaceCustomizedForm(void);
static void p3d_SetRrtConnectParam(void);

static int IsCustomizedDirectionShown = FALSE;
static int IsCustomizedNodeShown = FALSE;
static int IsCustomizedProcessShown = FALSE;
static int IsCustomizedDistanceShown = FALSE;
static int IsCustomizedManhattanShown = FALSE;
static int IsCustomizedCostSpaceShown = FALSE;

void g3d_create_diffusion_form(void) {
  g3d_create_sized_diffusion_form(456, 240);
}

static void g3d_create_sized_diffusion_form(int w, int h){
  g3d_create_form(&DIFFUSION_FORM,w,h,FL_UP_BOX); 
  g3d_create_BiOrMonoDir_Frame_obj();
  g3d_create_DiffusionMethod_obj();
  g3d_create_NTryNbNodes_obj();
  g3d_create_RunStopReset_obj();
  fl_end_form();
  g3d_create_DirectionCustomizedForm();
  g3d_create_NodeCustomizedForm();
  g3d_create_ProcessCustomizedForm();
  g3d_create_DistanceCustomizedForm();
  g3d_create_ManhattanCustomizedForm();
  g3d_create_CostSpaceCustomizedForm();

}

void g3d_delete_diffusion_form(void) {
  if(fl_get_button(Diffusion_obj)) {
    fl_hide_form(DIFFUSION_FORM);
  }
  g3d_delete_DiffusionMethod_obj();
  g3d_delete_BiOrMonoDir_Frame_obj();
  g3d_delete_NTryNbNodes_obj();
  g3d_delete_RunStopReset_obj();
  g3d_fl_free_form(DIRECTION_CUSTOMIZED_FORM);
  g3d_fl_free_form(NODE_CUSTOMIZED_FORM);
  g3d_fl_free_form(PROCESS_CUSTOMIZED_FORM);
  g3d_fl_free_form(DISTANCE_CUSTOMIZED_FORM);
  g3d_fl_free_form(MANHATTAN_CUSTOMIZED_FORM);
  g3d_fl_free_form(COSTSPACE_CUSTOMIZED_FORM);
  g3d_fl_free_form(DIFFUSION_FORM);
}

static void g3d_create_DirectionCustomizedForm(void) {
  int InitExpanDirMethod = p3d_GetExpansionDirectionMethod();
 
  g3d_create_form(&DIRECTION_CUSTOMIZED_FORM,456, 90,FL_UP_BOX);
  g3d_create_labelframe(&EXPANSION_DIRECTION_FRAME,FL_NO_FRAME, 140, 60,
			"Sampling Direction", (void**) &DIRECTION_CUSTOMIZED_FORM,1);
  EXPANSION_CHOICE_OBJ = fl_add_choice(FL_NORMAL_CHOICE,15.0,25.,
				       120.0,40.0,"");
  fl_addto_choice(EXPANSION_CHOICE_OBJ,"All Space");
  fl_addto_choice(EXPANSION_CHOICE_OBJ,"Dyn.Box");
  fl_set_choice(EXPANSION_CHOICE_OBJ, InitExpanDirMethod);
  fl_set_call_back(EXPANSION_CHOICE_OBJ,CB_ExpansionDirectChoice,0);

  g3d_create_frame(&BIAS_FRAME,FL_NO_FRAME, 140, 60,
			"", (void**) &DIRECTION_CUSTOMIZED_FORM,1);
  g3d_create_checkbutton(&BIAS_CHECK,FL_PUSH_BUTTON,70.0,30.0,"Biased to goal",
			 (void**)&BIAS_FRAME,0);  
  fl_set_object_color(BIAS_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(BIAS_CHECK,p3d_GetIsGoalBias());
  fl_set_call_back(BIAS_CHECK, CB_IsGoalBias,0);

  g3d_create_valslider(&BIAS_RATIO_SLIDER,FL_HOR_SLIDER,100,20.0,"Bias Ratio",
		       (void**)&BIAS_FRAME,0);
  fl_set_slider_step(BIAS_RATIO_SLIDER,0.01);
  fl_set_slider_bounds(BIAS_RATIO_SLIDER,0.,1.);
  fl_set_slider_value(BIAS_RATIO_SLIDER,p3d_GetGoalBiasValue());
  fl_set_call_back(BIAS_RATIO_SLIDER,CB_BiasRatioSlider,0);


  g3d_create_valslider(&DYN_PARAM_SLIDER,FL_HOR_SLIDER,100,20.0,"Dyn. Param",
		       (void**)&DIRECTION_CUSTOMIZED_FORM,1);
  fl_set_slider_step(DYN_PARAM_SLIDER,0.01);
  fl_set_slider_bounds(DYN_PARAM_SLIDER,0.1,50.);
  fl_set_slider_value(DYN_PARAM_SLIDER,p3d_GetLambda());
  fl_set_call_back(DYN_PARAM_SLIDER,CB_LambdaSlider,0);
  fl_end_form();
}

static void g3d_create_NodeCustomizedForm(void) {
  g3d_create_form(&NODE_CUSTOMIZED_FORM,456, 120,FL_UP_BOX); 
  g3d_create_labelframe(&EXPANSION_NODE_FRAME,FL_NO_FRAME, 140, 60,
			"Expansion Node Choice", (void**) &NODE_CUSTOMIZED_FORM,1);
  NODE_CHOICE_OBJ = fl_add_choice(FL_NORMAL_CHOICE,15.0,25.,
				       120.0,40.0,"");
  fl_addto_choice(NODE_CHOICE_OBJ,"Nearest");
  fl_addto_choice(NODE_CHOICE_OBJ,"K-Nearest");
  fl_addto_choice(NODE_CHOICE_OBJ,"Best Score");
  fl_addto_choice(NODE_CHOICE_OBJ,"K-BestScore");
  fl_addto_choice(NODE_CHOICE_OBJ,"Rand in Shell");
  fl_addto_choice(NODE_CHOICE_OBJ,"Random");

  fl_set_choice(NODE_CHOICE_OBJ, p3d_GetExpansionNodeMethod());
  fl_set_call_back(NODE_CHOICE_OBJ,CB_NodeExpChoice,0);


  g3d_create_frame(&MAX_FAIL_FRAME,FL_NO_FRAME, 140, 60,
			"", (void**) &NODE_CUSTOMIZED_FORM,1);
  g3d_create_checkbutton(&MAX_FAIL_CHECK,FL_PUSH_BUTTON,70.0,30.0,"Discard Fail Nodes",
			 (void**)&MAX_FAIL_FRAME,0);  
  fl_set_object_color(MAX_FAIL_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(MAX_FAIL_CHECK, p3d_GetIsMaxExpandNodeFail());
  fl_set_call_back(MAX_FAIL_CHECK, CB_IsMaxExpandNodeFail, 0);

  g3d_create_valslider(&MAX_FAIL_SLIDER,FL_HOR_SLIDER,100,20.0,"Nb. Max fail",
		       (void**)&MAX_FAIL_FRAME,0);
  fl_set_slider_step(MAX_FAIL_SLIDER,1);
  fl_set_slider_bounds(MAX_FAIL_SLIDER,1, 100);
  fl_set_slider_value(MAX_FAIL_SLIDER,p3d_GetMaxExpandNodeFail());
  fl_set_call_back(MAX_FAIL_SLIDER,CB_ExpandNodeFailValue,0);

  g3d_create_frame(&MAX_DIST_FRAME,FL_NO_FRAME, 140, 60,
			"", (void**) &NODE_CUSTOMIZED_FORM,1);
  g3d_create_checkbutton(&MAX_DIST_CHECK,FL_PUSH_BUTTON,70.0,30.0,"Max. Distance",
			 (void**)&MAX_DIST_FRAME,0);  
  fl_set_object_color(MAX_DIST_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(MAX_DIST_CHECK, p3d_GetIsMaxDistNeighbor());
  fl_set_call_back(MAX_DIST_CHECK, CB_IsMaxDist, 0);

  g3d_create_valslider(&DYN_PARAM2_SLIDER,FL_HOR_SLIDER,100,20.0,"Dyn. Param",
		       (void**)&MAX_DIST_FRAME,0);
  fl_set_slider_step(DYN_PARAM2_SLIDER,0.01);
  fl_set_slider_bounds(DYN_PARAM2_SLIDER,0.1,50.);
  fl_set_slider_value(DYN_PARAM2_SLIDER,p3d_GetLambda());
  fl_set_call_back(DYN_PARAM2_SLIDER,CB_LambdaSlider,0);


  g3d_create_valslider(&KNEAR_SLIDER,FL_HOR_SLIDER,100,20.0,"K Best/Near percent.",
		       (void**)&NODE_CUSTOMIZED_FORM,1);
  fl_set_slider_step(KNEAR_SLIDER,5);
  fl_set_slider_bounds(KNEAR_SLIDER,0, 100);
  fl_set_slider_value(KNEAR_SLIDER,p3d_GetNearestExpandPercent());
  fl_set_call_back(KNEAR_SLIDER,CB_NbNearestExpand,0);
  fl_end_form();
}

static void  g3d_create_ProcessCustomizedForm(void) {
  g3d_create_form(&PROCESS_CUSTOMIZED_FORM,456, 120,FL_UP_BOX);
  g3d_create_labelframe(&DIFFUSION_PROCESS_FRAME,FL_NO_FRAME, 140, 60,
			"Diffusion Process", (void**) &PROCESS_CUSTOMIZED_FORM,1);
  DIFFUSION_PROCESS_CHOICE = fl_add_choice(FL_NORMAL_CHOICE,15.0,25.,
				       120.0,40.0,"");
  fl_addto_choice(DIFFUSION_PROCESS_CHOICE, "Connect");
  fl_addto_choice(DIFFUSION_PROCESS_CHOICE, "Extend N times");
  fl_addto_choice(DIFFUSION_PROCESS_CHOICE, "Extend 1 time")
;
  fl_set_choice(DIFFUSION_PROCESS_CHOICE, p3d_GetExpansionChoice());
  fl_set_call_back(DIFFUSION_PROCESS_CHOICE,CB_ExpansionChoice,0);

  g3d_create_frame(&EXTENSION_FRAME,FL_NO_FRAME, 140, 100,
			"", (void**) &PROCESS_CUSTOMIZED_FORM,1);
  g3d_create_valslider(&STEP_SLIDER,FL_HOR_SLIDER,100,20.0,"Extention Step",
		       (void**)&EXTENSION_FRAME,0);
  fl_set_slider_step(STEP_SLIDER,1);
  fl_set_slider_bounds(STEP_SLIDER,1, 100);
  fl_set_slider_value(STEP_SLIDER,p3d_GetExtendStepParam());
  fl_set_call_back(STEP_SLIDER,CB_ExendStepParam,0);

  g3d_create_checkbutton(&ADD_CYCLES_CHECK,FL_PUSH_BUTTON,70.0,30.0,"Add cycles",
			 (void**)&PROCESS_CUSTOMIZED_FORM,1);  
  fl_set_object_color(ADD_CYCLES_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(ADD_CYCLES_CHECK, p3d_GetIsCycles());
  fl_set_call_back(ADD_CYCLES_CHECK, CB_GetIsCycles, 0);

  fl_end_form();
}

static void g3d_create_DistanceCustomizedForm(void) {
  g3d_create_form(&DISTANCE_CUSTOMIZED_FORM,456, 90,FL_UP_BOX);  
  g3d_create_labelframe(&DISTANCE_FRAME,FL_NO_FRAME, 140, 60,
			"Distance Choice", (void**) &DISTANCE_CUSTOMIZED_FORM,1);
  
  DISTANCE_CHOICE = fl_add_choice(FL_NORMAL_CHOICE,15.0,25.,
				       120.0,40.0,"");
  fl_addto_choice(DISTANCE_CHOICE, "Cspace"); 
  fl_addto_choice(DISTANCE_CHOICE, "Active Conf Dist"); 
  fl_addto_choice(DISTANCE_CHOICE, "Lig/Protein Dist");
  fl_addto_choice(DISTANCE_CHOICE, "Mob. Frame Dist");
 
  fl_set_choice(DISTANCE_CHOICE, p3d_GetDistConfigChoice());
  fl_set_call_back(DISTANCE_CHOICE,CB_DistConfigChoice,0);

  g3d_create_frame(&WEIGHT_ROTA_FRAME,FL_NO_FRAME, 140, 60,
			"", (void**) &DISTANCE_CUSTOMIZED_FORM,1);

  g3d_create_checkbutton(&WEIGHT_ROTA_CHECK,FL_PUSH_BUTTON,70.0,30.0,"Weighted Rotations",
			 (void**)&WEIGHT_ROTA_FRAME,0);  
  fl_set_object_color(WEIGHT_ROTA_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(WEIGHT_ROTA_CHECK, p3d_GetIsWeightedRotations());
  fl_set_call_back(WEIGHT_ROTA_CHECK, CB_IsWeightedRotations, 0);

  g3d_create_valslider(&WEIGHT_ROTA_SLIDER,FL_HOR_SLIDER,100,20.0,"Rota Weight ",
		       (void**)&WEIGHT_ROTA_FRAME,0);
  fl_set_slider_step(WEIGHT_ROTA_SLIDER,0.1);
  fl_set_slider_bounds(WEIGHT_ROTA_SLIDER,0.1, 10);
  fl_set_slider_value(WEIGHT_ROTA_SLIDER,p3d_GetWeightRotations());
  fl_set_call_back(WEIGHT_ROTA_SLIDER,CB_WeightRotationsValue,0);

  fl_end_form();
}
static void g3d_create_ManhattanCustomizedForm(void) {
  g3d_create_form(&MANHATTAN_CUSTOMIZED_FORM,456, 90,FL_UP_BOX);
  g3d_create_checkbutton(&MANHATTAN_CHECK,FL_PUSH_BUTTON,-1,-1,"Manhat Expansion",
			 (void**)&MANHATTAN_CUSTOMIZED_FORM,1);  
  fl_set_object_color(MANHATTAN_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(MANHATTAN_CHECK, p3d_GetIsManhatExpansion());
  fl_set_call_back(MANHATTAN_CHECK, CB_IsManhattanExp, 0);

  g3d_create_frame(&MANHATTAN_FRAME,FL_NO_FRAME, 140, 60,
			"", (void**) &MANHATTAN_CUSTOMIZED_FORM,1);

  g3d_create_valslider(&MANHATTAN_RATIO_SLIDER,FL_HOR_SLIDER,100,20.0,"Manhat. Ratio ",
		       (void**)&MANHATTAN_FRAME,0);
  fl_set_slider_step(MANHATTAN_RATIO_SLIDER,0.01);
  fl_set_slider_bounds(MANHATTAN_RATIO_SLIDER,0.01, 1.);
  fl_set_slider_value(MANHATTAN_RATIO_SLIDER,p3d_GetManhattanRatio());
  fl_set_call_back(MANHATTAN_RATIO_SLIDER,CB_ManhattanRatioValue,0);

  g3d_create_valslider(&MAX_PASSI_FAIL_SLIDER,FL_HOR_SLIDER,100,20.0,"Max Passive Fails",
		       (void**)&MANHATTAN_FRAME,0);
  fl_set_slider_step(MAX_PASSI_FAIL_SLIDER,1);
  fl_set_slider_bounds(MAX_PASSI_FAIL_SLIDER,1, 100);
  fl_set_slider_value(MAX_PASSI_FAIL_SLIDER,p3d_GetMaxPassiveExpand());
  fl_set_call_back(MAX_PASSI_FAIL_SLIDER,CB_MaxPassiveExpandValue,0);

  g3d_create_checkbutton(&EXT_PASS_FAIL_ACT_CHECK,FL_PUSH_BUTTON,-1,-1,
			 "Extend Pass. whithout Act.", 
			 (void**)&MANHATTAN_CUSTOMIZED_FORM,1);  
  fl_set_object_color(EXT_PASS_FAIL_ACT_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(EXT_PASS_FAIL_ACT_CHECK, p3d_GetIsPasExtWhenAct());
  fl_set_call_back(EXT_PASS_FAIL_ACT_CHECK, CB_IsPasExtWhenAct, 0);

  fl_end_form();
}

static void g3d_create_CostSpaceCustomizedForm(void) {
  char buffer[10];
  g3d_create_form(&COSTSPACE_CUSTOMIZED_FORM,456, 135,FL_UP_BOX);
  g3d_create_frame(&COST_SPACE_FRAME0,FL_NO_FRAME, 90, -1,
		   "", (void**) &COSTSPACE_CUSTOMIZED_FORM,1);

  g3d_create_checkbutton(&COST_SPACE_CHECK,FL_PUSH_BUTTON,-1,-1,"Cost Space",
			 (void**)&COST_SPACE_FRAME0,0);  
  fl_set_object_color(COST_SPACE_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(COST_SPACE_CHECK, p3d_GetIsCostFuncSpace());
  fl_set_call_back(COST_SPACE_CHECK , CB_IsCostFunctSpace, 0);

  g3d_create_valslider(&COST_SLIDER,FL_HOR_SLIDER,80,20.0,"Cost Parameter",
		       (void**)&COST_SPACE_FRAME0,0);
  fl_set_slider_step(COST_SLIDER,1.);
  fl_set_slider_bounds(COST_SLIDER,0., 5.);
  fl_set_slider_value(COST_SLIDER,p3d_GetCostSpaceParam());
  fl_set_call_back(COST_SLIDER,CB_CostSpaceParam,0);

  g3d_create_frame(&COST_SPACE_FRAME1,FL_NO_FRAME, 110, -1,
		   "", (void**) &COSTSPACE_CUSTOMIZED_FORM,1);

  g3d_create_button(&NODE_COST_TRAJ_OBJ,FL_PUSH_BUTTON,40.0,20.0,
		    "TrajNode C",(void**)&COST_SPACE_FRAME1,0);
  fl_set_call_back(NODE_COST_TRAJ_OBJ,CB_TrajNodeCost,0);

  g3d_create_button(&COST_TRAJ_OBJ,FL_PUSH_BUTTON,40.0,20.0,
		    "Traj C",(void**)&COST_SPACE_FRAME1,0);
  fl_set_call_back(COST_TRAJ_OBJ,CB_TrajCost,0);

  g3d_create_choice(&COST_METHOD_CHOICE,FL_NORMAL_CHOICE,100, 30,"",
			 (void**)&COST_SPACE_FRAME1,0);  

  fl_addto_choice(COST_METHOD_CHOICE , "Threshold");
  fl_addto_choice(COST_METHOD_CHOICE, "T-Urmson");
  fl_addto_choice(COST_METHOD_CHOICE, "T-RRT");
  fl_addto_choice(COST_METHOD_CHOICE, "Cycle T-RRT");
  fl_addto_choice(COST_METHOD_CHOICE, "Monte Carlo");
  fl_addto_choice(COST_METHOD_CHOICE, "Down Search");
  fl_addto_choice(COST_METHOD_CHOICE, "Adapt. Thres");
  
  fl_set_choice(COST_METHOD_CHOICE, p3d_GetCostMethodChoice());
  fl_set_call_back(COST_METHOD_CHOICE , CB_CostMethodChoice, 0);

/*   g3d_create_valslider(&THRESHOLD_DOWN_SLIDER,FL_HOR_SLIDER,80,20.0,"Down/Up Ratio", */
/* 		       (void**)&COSTSPACE_CUSTOMIZED_FORM,1); */
/*   fl_set_slider_step(THRESHOLD_DOWN_SLIDER,1); */
/*   fl_set_slider_bounds(THRESHOLD_DOWN_SLIDER,0, 5); */
/*   fl_set_slider_value(THRESHOLD_DOWN_SLIDER,p3d_GetThresholdDown()); */
/*   fl_set_call_back(THRESHOLD_DOWN_SLIDER,CB_ThresholdDown,0); */

  
  g3d_create_frame(&COST_SPACE_FRAME2,FL_NO_FRAME, 110, -1,
		   "", (void**) &COSTSPACE_CUSTOMIZED_FORM,1);

  g3d_create_button(&GRID_ROADMAP_OBJ,FL_PUSH_BUTTON,-1,20,
		    "Compute grid",(void**)&COST_SPACE_FRAME2,0);
  fl_set_call_back(GRID_ROADMAP_OBJ,CB_GridRoadmap,0);

  g3d_create_button(&EXTRACT_BPATH_OBJ,FL_PUSH_BUTTON, -1,20,
		    "Extract Best P.",(void**)&COST_SPACE_FRAME2,0);
  fl_set_call_back(EXTRACT_BPATH_OBJ,CB_BPathExtract,0);

  g3d_create_checkbutton(&LOCAL_ADAPT_CHECK,FL_PUSH_BUTTON,-1,-1,"Local. adapt",
			 (void**)&COSTSPACE_CUSTOMIZED_FORM,1);  
  fl_set_object_color(LOCAL_ADAPT_CHECK,FL_MCOL,FL_GREEN);
  fl_set_button(LOCAL_ADAPT_CHECK, p3d_GetIsLocalCostAdapt());
  fl_set_call_back(LOCAL_ADAPT_CHECK, CB_IsLocalCostAdapt, 0);


  g3d_create_choice(&DELTA_COST_CHOICE, FL_NORMAL_CHOICE,100, 30,"",
		    (void**)&COSTSPACE_CUSTOMIZED_FORM,1);  

  fl_addto_choice(DELTA_COST_CHOICE, "Work");
  fl_addto_choice(DELTA_COST_CHOICE, "Ave.C.Node");
  fl_addto_choice(DELTA_COST_CHOICE, "C.Node+Dis");
  fl_addto_choice(DELTA_COST_CHOICE, "Boltzmann");

  fl_set_choice(DELTA_COST_CHOICE, p3d_GetDeltaCostChoice());
  fl_set_call_back(DELTA_COST_CHOICE,CB_DeltaCostChoice,0);

  g3d_create_valslider(&ALPHA_COST_SLIDER,FL_HOR_SLIDER,80,20.0,"Alpha",
		       (void**)&COSTSPACE_CUSTOMIZED_FORM,1);
  fl_set_slider_step(ALPHA_COST_SLIDER,0.1);
  fl_set_slider_bounds(ALPHA_COST_SLIDER,0, 1);
  fl_set_slider_value(ALPHA_COST_SLIDER,p3d_GetAlphaValue());
  fl_set_call_back(ALPHA_COST_SLIDER,CB_AlphaCostParam,0);

  g3d_create_button(&OPTIMIZE_COST_TRAJ_OBJ,FL_PUSH_BUTTON,-1,20,
		    "Optimize Traj",(void**)&COSTSPACE_CUSTOMIZED_FORM,1);
  fl_set_call_back(OPTIMIZE_COST_TRAJ_OBJ,CB_OptimizeCostTraj,0);

  g3d_create_input(&NB_FAIL_OPTIMIZE_COST_TRAJ_OBJ,FL_NORMAL_INPUT, 30.0,20.0,"",
		   (void**)&COSTSPACE_CUSTOMIZED_FORM,1);
  sprintf(buffer,"%d", p3d_GetNbFailOptimCostMax());
  fl_set_input(NB_FAIL_OPTIMIZE_COST_TRAJ_OBJ, buffer);
  fl_set_call_back(NB_FAIL_OPTIMIZE_COST_TRAJ_OBJ,CB_SetNbFailOptimCostMax,0);


    IS_EXPAND_CONTROL_CHECK = fl_add_checkbutton(FL_PUSH_BUTTON,  345.0,35.0,30,30.0,
					    "Control Expand");
    fl_set_object_color(IS_EXPAND_CONTROL_CHECK,FL_MCOL,FL_GREEN);
    fl_set_call_back(IS_EXPAND_CONTROL_CHECK, CB_IsExpandControl,0); 
    fl_set_button(IS_EXPAND_CONTROL_CHECK, p3d_GetIsExpandControl());
    fl_end_form();
}

static void CB_SetNbFailOptimCostMax(FL_OBJECT *obj, long arg) {
  p3d_SetNbFailOptimCostMax(atoi(fl_get_input(NB_FAIL_OPTIMIZE_COST_TRAJ_OBJ)));
}


static void CB_IsPasExtWhenAct(FL_OBJECT *obj, long arg) {
int val = fl_get_button(obj);
  p3d_SetIsPasExtWhenAct(val);
}

static void CB_MaxPassiveExpandValue(FL_OBJECT *obj, long arg) {
  int  val = fl_get_slider_value(obj);
  p3d_SetMaxPassiveExpand(val);
}

static void CB_ManhattanRatioValue(FL_OBJECT *obj, long arg) {
  double  val = fl_get_slider_value(obj);
  p3d_SetManhattanRatio(val);
}


static void CB_IsManhattanExp(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsManhatExpansion(val);
}

static void CB_DistConfigChoice(FL_OBJECT *obj, long arg) {
  int val = fl_get_choice(obj);
  p3d_SetDistConfigChoice(val);
}

static void CB_WeightRotationsValue(FL_OBJECT *obj, long arg) {
  double  val = fl_get_slider_value(obj);
  p3d_SetWeightRotations(val);
}

static void CB_IsWeightedRotations(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsWeightedRotations(val);
}

static void CB_IsCostFunctSpace(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  fl_deactivate_object(obj);
  p3d_SetIsCostFuncSpace(val);
  if(XYZ_GRAPH != NULL) {
    p3d_UpdateEdgeGraphCost(XYZ_GRAPH);
  }
  fl_activate_object(obj);

}


static void CB_CostMethodChoice(FL_OBJECT *obj, long arg) {
  int val = fl_get_choice(obj);
  fl_deactivate_object(obj);
  p3d_SetCostMethodChoice(val);
  fl_activate_object(obj);
}


static void CB_IsLocalCostAdapt(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsLocalCostAdapt(val);
}

static void CB_IsExpandControl(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsExpandControl(val);
}



static void CB_DeltaCostChoice(FL_OBJECT *obj, long arg) {
  int val = fl_get_choice(obj);
  fl_deactivate_object(obj);
  p3d_SetDeltaCostChoice(val);
  if(XYZ_GRAPH != NULL) {
    p3d_UpdateEdgeGraphCost(XYZ_GRAPH);
  }
  fl_activate_object(obj);
}

static void CB_ExpansionChoice(FL_OBJECT *obj, long arg) {
  int val = fl_get_choice(obj);
  p3d_SetExpansionChoice(val);
}

static void CB_ExendStepParam(FL_OBJECT *obj, long arg) {
  int  val = fl_get_slider_value(obj);
  p3d_SetExtendStepParam(val);
}

static void CB_CostSpaceParam(FL_OBJECT *obj, long arg) {
  double  val = fl_get_slider_value(obj);
  p3d_SetCostSpaceParam(val);
}

/* static void CB_ThresholdDown(FL_OBJECT *obj, long arg) { */
/*   int  val = fl_get_slider_value(obj); */
/*   p3d_SetThresholdDown(val); */
/* } */

static void CB_TrajNodeCost(FL_OBJECT *obj, long arg) {  
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph *GraphPt = XYZ_GRAPH;
  double cost;
  configPt q;
  
  fl_deactivate_object(obj);
  if(GraphPt != NULL) {
    p3d_PrintNodeSolutionCost(GraphPt);  
  } else {
    q = p3d_get_robot_config(robotPt);
    cost = p3d_GetConfigCost(robotPt, q);
    printf("Cost of the current configuration: %f\n", cost);
    p3d_destroy_config(robotPt, q);
  }
  fl_set_button(obj,0);
  fl_activate_object(obj);
}
static void CB_TrajCost(FL_OBJECT *obj, long arg) {  
  p3d_graph *GraphPt = XYZ_GRAPH;
  fl_deactivate_object(obj);
  if(GraphPt!=NULL) {
    p3d_PrintTrajCost(GraphPt,GraphPt->rob->tcur);  
  }
  fl_set_button(obj,0);
  fl_activate_object(obj);
}
static void CB_GridRoadmap(FL_OBJECT *obj, long arg) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  fl_deactivate_object(obj);
  p3d_CreateDenseRoadmap(robotPt);
  fl_set_button(obj,0);
  fl_activate_object(obj);
}

static void CB_BPathExtract(FL_OBJECT *obj, long arg) {
  fl_deactivate_object(obj);
  p3d_ExtractBestTraj(XYZ_GRAPH);
  fl_set_button(obj,0);
  fl_activate_object(obj);
}

static void CB_AlphaCostParam(FL_OBJECT *obj, long arg) {
  double  val = fl_get_slider_value(obj);
  p3d_SetAlphaValue(val);

}

static void CB_OptimizeCostTraj(FL_OBJECT *obj, long arg) {

  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  p3d_traj* CurrentTrajPt = robotPt->tcur;
  int nFailOptim = 0, nLoopTotMax = 10000, nLoopTot = 0;
  int isOptimSuccess;

  if(CurrentTrajPt == NULL) {
    PrintInfo(("Warning: no current trajectory to optimize\n"));
    fl_set_button(obj,0);
    return;
  }
  fl_deactivate_object(obj);


  PrintInfo(("Initial traj:\n"));
  p3d_PrintTrajCost(robotPt->GRAPH, CurrentTrajPt);

  //Loop done until an optimization failed a given number of times or when it reaches 
  // a maximal number of loops
  while((nFailOptim <p3d_GetNbFailOptimCostMax()) &&(nLoopTot < nLoopTotMax) ) {
    nLoopTot++;
    isOptimSuccess = p3d_OneLoopOptimizeCostTraj(robotPt->GRAPH, CurrentTrajPt);
    if(isOptimSuccess == FALSE) {
      nFailOptim ++;
    } else {
      nFailOptim = 0.;
    }
  }
  PrintInfo(("Optimized traj:\n"));
  p3d_PrintTrajCost(robotPt->GRAPH, CurrentTrajPt);

  fl_set_button(obj,0);
  fl_activate_object(obj);
}

static void CB_IsMaxDist(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsMaxDistNeighbor(val);
}

static void CB_GetIsCycles(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsCycles(val);
}


static void CB_IsMaxExpandNodeFail(FL_OBJECT *obj, long arg){
  int val = fl_get_button(obj);
  p3d_SetIsMaxExpandNodeFail(val);
}

static void CB_ExpandNodeFailValue(FL_OBJECT *obj, long arg) {
  int  val = fl_get_slider_value(obj);
  p3d_SetMaxExpandNodeFail(val);
}

static void CB_NbNearestExpand(FL_OBJECT *obj, long arg) {
  int  val = fl_get_slider_value(obj);
  p3d_SetNearestExpandPercent(val);
}

static void CB_NodeExpChoice(FL_OBJECT *obj, long arg) {
  int  val = fl_get_choice(obj);
  p3d_SetExpansionNodeMethod(val);
  if((val == BEST_SCORE_EXP_METH ) || 
     (val == K_BEST_SCORE_EXP_METH)) {
    p3d_SetIsWeightedChoice(TRUE);
  } else {
    p3d_SetIsWeightedChoice(FALSE);
  }
}

static void CB_LambdaSlider(FL_OBJECT *obj, long arg) {
  double   val = fl_get_slider_value(obj);
  fl_set_slider_value(BIAS_RATIO_SLIDER,p3d_GetGoalBiasValue());
  p3d_SetLambda(val);
  if(obj == DYN_PARAM_SLIDER) {
    fl_set_slider_value(DYN_PARAM2_SLIDER,p3d_GetLambda());
  }
  else if (obj == DYN_PARAM2_SLIDER){
    fl_set_slider_value(DYN_PARAM_SLIDER,p3d_GetLambda());
  }
  else {
    PrintInfo(("Error in seting the value of Lambda sliders\n"));
    }
}


static void CB_BiasRatioSlider(FL_OBJECT *obj, long arg) {
  double   val = fl_get_slider_value(obj);
  p3d_SetGoalBiasValue(val);
}

static void CB_IsGoalBias(FL_OBJECT *obj, long arg) {
  int   val = fl_get_button(obj);
  p3d_SetIsGoalBias(val);
}

static void CB_ExpansionDirectChoice(FL_OBJECT *obj, long arg) {
  int   val = fl_get_choice(obj);
  p3d_SetExpansionDirectionMethod(val);
}


static void CB_DiffusionMethod_obj(FL_OBJECT *obj, long arg) {
  fl_set_form_icon(DIRECTION_CUSTOMIZED_FORM, GetApplicationIcon(), 0);
  fl_set_form_icon(NODE_CUSTOMIZED_FORM, GetApplicationIcon(), 0);
  fl_set_form_icon(PROCESS_CUSTOMIZED_FORM, GetApplicationIcon(), 0);
  fl_set_form_icon(DISTANCE_CUSTOMIZED_FORM, GetApplicationIcon(), 0);
  fl_set_form_icon(MANHATTAN_CUSTOMIZED_FORM, GetApplicationIcon(), 0);
  fl_set_form_icon(COSTSPACE_CUSTOMIZED_FORM, GetApplicationIcon(), 0);


  switch((int)arg){
  case 0:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM);
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {
      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
   if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    p3d_SetRrtConnectParam();
    break;
  case 1:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM);
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {
      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
    if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    break;
  case 2: 
      fl_set_form_position(DIRECTION_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);
      fl_show_form(DIRECTION_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Sampling Direction");  

      IsCustomizedDirectionShown = TRUE;
      fl_set_form_position(NODE_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+DIRECTION_CUSTOMIZED_FORM->h+2*28);
      fl_show_form(NODE_CUSTOMIZED_FORM, FL_PLACE_GEOMETRY, TRUE,
		   "Customize Expansion Node");
      IsCustomizedNodeShown = TRUE;
      fl_set_form_position(PROCESS_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h +3*28 + DIRECTION_CUSTOMIZED_FORM->h
			   + NODE_CUSTOMIZED_FORM->h);
      fl_show_form(PROCESS_CUSTOMIZED_FORM, FL_PLACE_GEOMETRY, TRUE,
		   "Customize Process");
      IsCustomizedProcessShown = TRUE;
      fl_set_form_position(DISTANCE_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h +4*28 + DIRECTION_CUSTOMIZED_FORM->h
			   + NODE_CUSTOMIZED_FORM->h + PROCESS_CUSTOMIZED_FORM->h);
      fl_show_form(DISTANCE_CUSTOMIZED_FORM, FL_PLACE_GEOMETRY, TRUE,
		   "Customize Distances");
      IsCustomizedDistanceShown = TRUE;
      fl_set_form_position(MANHATTAN_CUSTOMIZED_FORM, DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h +5*28 + DIRECTION_CUSTOMIZED_FORM->h
			   + NODE_CUSTOMIZED_FORM->h + PROCESS_CUSTOMIZED_FORM->h + 
			   DISTANCE_CUSTOMIZED_FORM->h);
      fl_show_form(MANHATTAN_CUSTOMIZED_FORM, FL_PLACE_GEOMETRY, TRUE,
		   "Customize Manhattan");
      IsCustomizedManhattanShown = TRUE;
      fl_set_form_position(COSTSPACE_CUSTOMIZED_FORM, DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h +6*28 + DIRECTION_CUSTOMIZED_FORM->h
			   + NODE_CUSTOMIZED_FORM->h + PROCESS_CUSTOMIZED_FORM->h + 
			   DISTANCE_CUSTOMIZED_FORM->h + MANHATTAN_CUSTOMIZED_FORM->h);
      fl_show_form(COSTSPACE_CUSTOMIZED_FORM, FL_PLACE_GEOMETRY, TRUE,
		   "Customize CostSpace");
      IsCustomizedCostSpaceShown = TRUE;
    break;
  case 3:
    if(IsCustomizedDirectionShown == FALSE) {      
      fl_set_form_position(DIRECTION_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);
      fl_show_form(DIRECTION_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Sampling Direction");
      IsCustomizedDirectionShown = TRUE;
    }
    if(IsCustomizedNodeShown == TRUE) {
      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
    if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    break;

  case 4: 
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM); 
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == FALSE) {
      fl_set_form_position(NODE_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);
      fl_show_form(NODE_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Node");
      IsCustomizedNodeShown = TRUE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
    if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    break;

  case 5:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM); 
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {

      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == FALSE) {
      fl_set_form_position(PROCESS_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);
      fl_show_form(PROCESS_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Process");
      IsCustomizedProcessShown = TRUE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
    if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    break;
  case 6:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM); 
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {

      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == FALSE) {
      fl_set_form_position(DISTANCE_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);
      fl_show_form(DISTANCE_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Distance");
      IsCustomizedDistanceShown = TRUE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
    if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    break;
  case 7:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM); 
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {

      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == FALSE) {
      fl_set_form_position(MANHATTAN_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);      
      fl_show_form(MANHATTAN_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Manhattan");
      IsCustomizedManhattanShown = TRUE;
    }
    if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    break;
  case 8:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM); 
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {

      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
    if(IsCustomizedCostSpaceShown == FALSE) {
      fl_set_form_position(COSTSPACE_CUSTOMIZED_FORM,  DIFFUSION_FORM->x,
			   DIFFUSION_FORM->y+DIFFUSION_FORM->h+28);      
      fl_show_form(COSTSPACE_CUSTOMIZED_FORM,FL_PLACE_GEOMETRY,TRUE,
		   "Customize Cost SPace");
      IsCustomizedCostSpaceShown = TRUE;
    }
    break;
  case 9:
    if(IsCustomizedDirectionShown == TRUE) {
      fl_hide_form(DIRECTION_CUSTOMIZED_FORM);
      IsCustomizedDirectionShown = FALSE;
    }
    if(IsCustomizedNodeShown == TRUE) {
      fl_hide_form(NODE_CUSTOMIZED_FORM);
      IsCustomizedNodeShown = FALSE;
    }
    if(IsCustomizedProcessShown == TRUE) {
      fl_hide_form(PROCESS_CUSTOMIZED_FORM);
      IsCustomizedProcessShown = FALSE;
    }
    if(IsCustomizedDistanceShown == TRUE) {
      fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
      IsCustomizedDistanceShown = FALSE;
    }
    if(IsCustomizedManhattanShown == TRUE) {
      fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
      IsCustomizedManhattanShown = FALSE;
    }
   if(IsCustomizedCostSpaceShown == TRUE) {
      fl_hide_form(COSTSPACE_CUSTOMIZED_FORM);
      IsCustomizedCostSpaceShown = FALSE;
    }
    p3d_SetManhattanRrtParam();
    break;
  default:   
    fl_hide_form(DIRECTION_CUSTOMIZED_FORM); 
    IsCustomizedDirectionShown = FALSE;
    fl_hide_form(NODE_CUSTOMIZED_FORM); 
    IsCustomizedNodeShown = FALSE;
    fl_hide_form(PROCESS_CUSTOMIZED_FORM);
    IsCustomizedProcessShown = FALSE;
    fl_hide_form(DISTANCE_CUSTOMIZED_FORM);
    IsCustomizedDistanceShown = FALSE;
    fl_hide_form(MANHATTAN_CUSTOMIZED_FORM);
    IsCustomizedManhattanShown = FALSE;

  }
}

static void g3d_create_NTryNbNodes_obj(void) {  
  char buffer[10];
  g3d_create_frame(&NTRY_NBNODES_FRAME,FL_ENGRAVED_FRAME, 200, -1,
		   "", (void**) &DIFFUSION_FORM,1);
  g3d_create_input(&N_TRY_MAX,FL_NORMAL_INPUT,75.0,30.0,"          N Try Max         ",
		   (void**)&NTRY_NBNODES_FRAME,0);
  sprintf(buffer,"%d", p3d_get_NB_TRY());
  fl_set_input(N_TRY_MAX, buffer);
  fl_set_call_back(N_TRY_MAX,CB_SetNTryMax,0);

  g3d_create_input(&NB_NODE_COMP_MAX,FL_NORMAL_INPUT,75.0,30.0,"   Nb Node Comp Max",
		   (void**)&NTRY_NBNODES_FRAME,0);
  sprintf(buffer,"%d", p3d_get_COMP_NODES());
  fl_set_input(NB_NODE_COMP_MAX, buffer);
  fl_set_call_back(NB_NODE_COMP_MAX,CB_SetNbNodeCompMax,0);

}

static void g3d_delete_NTryNbNodes_obj(void) {
  g3d_fl_free_object(NTRY_NBNODES_FRAME); 
}


static void g3d_create_RunStopReset_obj(void) {
  g3d_create_frame(&RUN_STOP_RESET_FRAME,FL_NO_FRAME, 200, -1,
		   "", (void**) &DIFFUSION_FORM,1);

  g3d_create_button(&RUN_OBJ,FL_PUSH_BUTTON,60.0,30.0,"Run",(void**)&RUN_STOP_RESET_FRAME,0);
  fl_set_call_back(RUN_OBJ,CB_Run,0);

  g3d_create_button(&STOP_OBJ,FL_PUSH_BUTTON,60.0,30.0,"Stop",(void**)&RUN_STOP_RESET_FRAME,0);
  fl_set_call_back(STOP_OBJ,CB_Stop,0);

  g3d_create_button(&RESET_OBJ,FL_PUSH_BUTTON,60.0,30.0,"Reset",(void**)&RUN_STOP_RESET_FRAME,0);
  fl_set_call_back(RESET_OBJ,CB_Reset,0);

  g3d_create_checkbutton(&DRAW_GRAPH_OBJ,FL_PUSH_BUTTON,70.0,30.0,"Draw Graph",
			 (void**)&RUN_STOP_RESET_FRAME,0);  
  fl_set_object_color(DRAW_GRAPH_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(DRAW_GRAPH_OBJ,CB_DrawGraph,0);
  fl_set_button(DRAW_GRAPH_OBJ,FALSE);
}


static void CB_DrawGraph(FL_OBJECT *ob, long arg) {
  G3D_DRAW_GRAPH = !G3D_DRAW_GRAPH;
  g3d_draw_allwin_active();
  fl_set_button(DRAW_GRAPH_OBJ,G3D_DRAW_GRAPH);
  fl_set_button(SEARCH_DRAW_OBJ, G3D_DRAW_GRAPH);
}


static void g3d_delete_RunStopReset_obj(void){
  g3d_fl_free_object(RUN_STOP_RESET_FRAME);
}

FILE* DifficultyFile = NULL;

static void CB_Run(FL_OBJECT *obj, long arg) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  int res = 0;
  p3d_graph* GraphPt;
  char DifficultyFileName[15] = "Difficulty.txt";
  fl_deactivate_object(obj);  
  fl_deactivate_object(RESET_OBJ);
  p3d_SetStopValue(FALSE);
  
  if(!XYZ_GRAPH) { 
    GraphPt = p3d_create_graph();
  }  else {
    GraphPt = XYZ_GRAPH;
  }
  
  
  PrintInfo(("\n\n*************************\n \
Begining of Diffusion search process\n"));
  MY_ALLOC_INFO("Before the graph creation");
  DifficultyFile = fopen(DifficultyFileName, "w");
  res = p3d_RunDiffusion(GraphPt, fct_stop,fct_draw);
  fclose(DifficultyFile);

  PrintInfo(("End of Diffusion search process \n\
 ************************\n"));
  if(!res) {
    PrintInfo(("No solution path: the exploration didn't \
link a start and a goal configuration \n"));
  }
  else { 
    // on construit la trajectoire entre les points etapes 
    if(p3d_graph_to_traj(robotPt)) {	
      g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    } else {
      // printf("Problem during trajectory extraction\n");
      g3d_draw_allwin_active();
    }
  }
  PrintInfo(("\n\n"));
  fl_ringbell(0);
  fl_set_button(obj,0);
  fl_activate_object(RESET_OBJ);
  fl_activate_object(obj);  
}

static void CB_Stop(FL_OBJECT *obj, long arg) {
  fl_deactivate_object(obj);
  p3d_SetStopValue(TRUE);
  fl_set_button(RUN_OBJ,0); 
  fl_activate_object(RUN_OBJ); 
  fl_activate_object(RESET_OBJ);
  fl_set_button(obj,0);
  fl_activate_object(obj); 
  

}

static void CB_Reset(FL_OBJECT *obj, long arg) {
  p3d_SetTemperatureParam(1.);
  fl_deactivate_object(obj);
  p3d_del_graph(XYZ_GRAPH);
  MY_ALLOC_INFO("After the graph destruction");
  g3d_draw_allwin_active();  
  fl_set_button(obj,0);
  fl_activate_object(obj);
}

static void CB_SetNTryMax(FL_OBJECT *obj, long arg) {
  p3d_set_NB_TRY(atoi(fl_get_input(N_TRY_MAX)));
}

static void CB_SetNbNodeCompMax(FL_OBJECT *obj, long arg) {
  p3d_set_COMP_NODES(atoi(fl_get_input(NB_NODE_COMP_MAX)));
}

static void g3d_create_DiffusionMethod_obj(void) { 
 
  g3d_create_frame(&DIFFUSION_METHOD_FRAME,FL_NO_FRAME, -1, -1,
		   "", (void**) &DIFFUSION_FORM,1);
  DIFFUSION_METHOD_GROUP = fl_bgn_group();
  g3d_create_checkbutton(&RRT_CONNECT_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "RRT connect",(void**)&DIFFUSION_METHOD_FRAME,0);
  fl_set_object_color(RRT_CONNECT_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(RRT_CONNECT_METHOD,CB_DiffusionMethod_obj,0);

  g3d_create_checkbutton(&MY_RRT_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "My RRT  ",(void**)&DIFFUSION_METHOD_FRAME,0);
  fl_set_object_color(MY_RRT_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(MY_RRT_METHOD,CB_DiffusionMethod_obj,1);

  g3d_create_labelframe(&CUSTOMIZED_FRAME,FL_ENGRAVED_FRAME, 260, -1,
			"Customize", (void**) &DIFFUSION_METHOD_FRAME,0);

  g3d_create_checkbutton(&ALL_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "All           ",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(ALL_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(ALL_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,2);

  g3d_create_checkbutton(&DIRECTION_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "Sampling",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(DIRECTION_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(DIRECTION_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,3);

  g3d_create_checkbutton(&NODE_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "Node",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(NODE_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(NODE_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,4);

  g3d_create_checkbutton(&PROCESS_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "Process  ",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(PROCESS_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(PROCESS_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,5);

  g3d_create_checkbutton(&DISTANCE_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "Distance ",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(DISTANCE_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(DISTANCE_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,6);

  g3d_create_checkbutton(&MANHATTAN_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "Manhattan",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(MANHATTAN_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(MANHATTAN_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,7);

  g3d_create_checkbutton(&CSPACE_CUSTOMIZED_METHOD,FL_RADIO_BUTTON,-1,-1,
			 "CSpace",(void**)&CUSTOMIZED_FRAME,0);
  fl_set_object_color(CSPACE_CUSTOMIZED_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(CSPACE_CUSTOMIZED_METHOD,CB_DiffusionMethod_obj,8);

  ML_RRT_METHOD= fl_add_checkbutton(FL_RADIO_BUTTON,12,100,30,30,"ML-RRT");
  fl_set_object_color(ML_RRT_METHOD,FL_MCOL,FL_GREEN);
  fl_set_call_back(ML_RRT_METHOD,CB_DiffusionMethod_obj,9);
  DIFFUSION_METHOD_GROUP = fl_end_group();

  fl_set_button(MY_RRT_METHOD,1);
}

static void g3d_delete_DiffusionMethod_obj(void) {
  g3d_fl_free_object(DIFFUSION_METHOD_FRAME);
}


static void CB_BiOrMonoDirplanner_obj(FL_OBJECT *obj, long arg) {
  int IsBidirect = (int)arg;
  p3d_SetIsBidirectDiffu(IsBidirect);
  if(IsBidirect == TRUE) {
    fl_activate_object(BALANCED_COMP_OBJ);
    fl_set_object_lcol(BALANCED_COMP_OBJ,FL_BLACK);
    fl_deactivate_object(WITH_GOAL_DIR_OBJ);  
    fl_set_object_lcol(WITH_GOAL_DIR_OBJ,FL_INACTIVE_COL);
  } else {
    fl_deactivate_object(BALANCED_COMP_OBJ);
    fl_set_object_lcol(BALANCED_COMP_OBJ,FL_INACTIVE_COL);
    fl_activate_object(WITH_GOAL_DIR_OBJ);  
    fl_set_object_lcol(WITH_GOAL_DIR_OBJ,FL_BLACK);
  }

}

static void CB_Balancedplanner_obj(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsBalancedExpansion(val);

}

static void CB_WithGoalExpansion_obj(FL_OBJECT *obj, long arg) {
  int val = fl_get_button(obj);
  p3d_SetIsExpansionToGoal(val);
}


static void g3d_create_BiOrMonoDir_Frame_obj(void) {

  g3d_create_frame(&BI_OR_MONO_FRAME, FL_ENGRAVED_FRAME, 200,-1, "", 
		   (void**) &DIFFUSION_FORM, 1);
  BI_OR_MONO_GROUP = fl_bgn_group();
  g3d_create_checkbutton(&MONO_DIR_OBJ,FL_RADIO_BUTTON,-1,-1,
			 "Mono-Direction",(void**)&BI_OR_MONO_FRAME, 0);
  fl_set_object_color(MONO_DIR_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(MONO_DIR_OBJ,CB_BiOrMonoDirplanner_obj,0);

  g3d_create_checkbutton(&BI_DIR_OBJ,FL_RADIO_BUTTON,-1,-1,
			 "Bi-Direction",(void**)&BI_OR_MONO_FRAME, 0);
  fl_set_object_color(BI_DIR_OBJ,FL_MCOL,FL_GREEN);  
  fl_set_call_back(BI_DIR_OBJ,CB_BiOrMonoDirplanner_obj,1);

  BI_OR_MONO_GROUP = fl_end_group();


  g3d_create_checkbutton(&WITH_GOAL_DIR_OBJ,FL_PUSH_BUTTON,-1,-1,
			 "With goal         ",(void**)&BI_OR_MONO_FRAME, 0);
  fl_set_object_color(WITH_GOAL_DIR_OBJ,FL_MCOL,FL_GREEN);  
  fl_set_call_back(WITH_GOAL_DIR_OBJ,CB_WithGoalExpansion_obj,0); 
  fl_set_button(WITH_GOAL_DIR_OBJ,1);


  g3d_create_checkbutton(&BALANCED_COMP_OBJ,FL_PUSH_BUTTON,-1,-1,
			 "Balanced",(void**)&BI_OR_MONO_FRAME, 0);
  fl_set_object_color(BALANCED_COMP_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BALANCED_COMP_OBJ,CB_Balancedplanner_obj,0); 
  fl_set_button(BALANCED_COMP_OBJ,1);

  if(p3d_GetIsBidirectDiffu() == TRUE) { 
    fl_set_button(BI_DIR_OBJ,1);
    fl_activate_object(BALANCED_COMP_OBJ);  
    fl_set_object_lcol(BALANCED_COMP_OBJ,FL_BLACK);
    fl_deactivate_object(WITH_GOAL_DIR_OBJ);  
    fl_set_object_lcol(WITH_GOAL_DIR_OBJ,FL_INACTIVE_COL);
  }
  else{
    fl_set_button(MONO_DIR_OBJ,1);
    fl_deactivate_object(BALANCED_COMP_OBJ);  
    fl_set_object_lcol(BALANCED_COMP_OBJ,FL_INACTIVE_COL);
    fl_activate_object(WITH_GOAL_DIR_OBJ);  
    fl_set_object_lcol(WITH_GOAL_DIR_OBJ,FL_BLACK);

  }
  fl_set_button(BALANCED_COMP_OBJ, p3d_GetIsBalancedExpansion());
  fl_set_button(WITH_GOAL_DIR_OBJ, p3d_GetIsExpansionToGoal());
}

static void g3d_delete_BiOrMonoDir_Frame_obj(void) {
  g3d_fl_free_object(BI_OR_MONO_FRAME);
}

static void p3d_SetRrtConnectParam(void) {
  p3d_SetExpansionDirectionMethod(GLOBAL_CS_EXP);
  fl_set_choice(EXPANSION_CHOICE_OBJ, GLOBAL_CS_EXP);
  p3d_SetIsGoalBias(FALSE);
  fl_set_button(BIAS_CHECK,FALSE);
  p3d_SetExpansionNodeMethod(NEAREST_EXP_NODE_METH);
  p3d_SetIsWeightedChoice(FALSE);
  fl_set_choice(NODE_CHOICE_OBJ, NEAREST_EXP_NODE_METH);
  p3d_SetIsMaxExpandNodeFail(FALSE);
  fl_set_button(MAX_FAIL_CHECK, FALSE);
  p3d_SetIsMaxDistNeighbor(FALSE);
  fl_set_button(MAX_DIST_CHECK, FALSE);
  p3d_SetExpansionChoice(ONE_NODE_CONNECT_EXP_CHOICE);
  fl_set_choice(DIFFUSION_PROCESS_CHOICE, ONE_NODE_CONNECT_EXP_CHOICE);
  p3d_SetDistConfigChoice(GENERAL_CSPACE_DIST);
  fl_set_choice(DISTANCE_CHOICE, GENERAL_CSPACE_DIST);
  p3d_SetIsWeightedRotations(FALSE);
  fl_set_button(WEIGHT_ROTA_CHECK, FALSE);
  p3d_SetIsCostFuncSpace(FALSE);
  fl_set_button(COST_SPACE_CHECK, FALSE);
  p3d_SetIsManhatExpansion(FALSE);
  fl_set_button(MANHATTAN_CHECK, FALSE);
}

void p3d_SetManhattanRrtParam(void) {
  p3d_SetExpansionDirectionMethod(GLOBAL_CS_EXP);
  fl_set_choice(EXPANSION_CHOICE_OBJ, GLOBAL_CS_EXP);
  p3d_SetIsGoalBias(FALSE);
  fl_set_button(BIAS_CHECK,FALSE);
  p3d_SetExpansionNodeMethod(NEAREST_EXP_NODE_METH);
  p3d_SetIsWeightedChoice(TRUE);
  fl_set_choice(NODE_CHOICE_OBJ, K_BEST_SCORE_EXP_METH);
  p3d_SetIsMaxExpandNodeFail(TRUE);
  fl_set_button(MAX_FAIL_CHECK, TRUE);
  p3d_SetIsMaxDistNeighbor(FALSE);
  fl_set_button(MAX_DIST_CHECK, FALSE);
  p3d_SetExpansionChoice(ONE_NODE_CONNECT_EXP_CHOICE);
  fl_set_choice(DIFFUSION_PROCESS_CHOICE, ONE_NODE_CONNECT_EXP_CHOICE);
  p3d_SetDistConfigChoice(GENERAL_CSPACE_DIST);
  fl_set_choice(DISTANCE_CHOICE, GENERAL_CSPACE_DIST);
  p3d_SetIsWeightedRotations(FALSE);
  fl_set_button(WEIGHT_ROTA_CHECK, FALSE);
  p3d_SetIsCostFuncSpace(FALSE);
  fl_set_button(COST_SPACE_CHECK, FALSE);
  p3d_SetIsManhatExpansion(TRUE);
  fl_set_button(MANHATTAN_CHECK, TRUE);
}

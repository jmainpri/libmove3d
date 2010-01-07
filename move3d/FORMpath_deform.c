//File Created By Mokhtar GHARBI
//On 22/01/2007
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

//FL_OBJECTS declaration
extern FL_FORM * PATH_DEFORM_FORM;
extern FL_OBJECT * PATH_DEFORMATION;//The button creating this form. (in planner Form)
extern FL_OBJECT * SORT1_OBJ; //The check button NbEdges. (in planner Form)
extern FL_OBJECT * SORT2_OBJ; //The check button Distance. (in planner Form)

static FL_OBJECT * RETRACT_FRAME;
static FL_OBJECT * CYCLE_CHECKB;
static FL_OBJECT * RETRACT_CHECKB;
static FL_OBJECT * DISCREET_CHECKB;
static FL_OBJECT * SPACE1;
static FL_OBJECT * NDRAW_COUNTER;
static FL_OBJECT * NBRETRACT_INPUT;
static FL_OBJECT * DOF1_COUNTER;
static FL_OBJECT * DOF2_COUNTER;
static FL_OBJECT * NODE_FRAME;
static FL_OBJECT * ADD_NODE_BUTTON;
static FL_OBJECT * LAST_NODE_BUTTON;
static FL_OBJECT * REPLOT_BUTTON;
static FL_OBJECT * CLEAR_BUTTON;
static FL_OBJECT * NODE_COUNTER;
static FL_OBJECT * PLOT_ALL_CHECKB;
static FL_OBJECT * PLOT_ALL_INPUT;
static FL_OBJECT * FILTER_GRAPH_CHECKB;
static FL_OBJECT * RETRACT_STRAT_FRAME;
static FL_OBJECT * RETRACT_STRAT_GROUP;
static FL_OBJECT * GLOBAL_SEARCH_CHECKB;
static FL_OBJECT * FORW_SEARCH_CHECKB;
static FL_OBJECT * LINEAR_SEARCH_CHECKB;
static FL_OBJECT * VIS_GRAPH_FRAME;
static FL_OBJECT * VIS_GRAPH_XYPLOT;
static FL_OBJECT * DRAW_BUTTON;
static FL_OBJECT * SPACE2;
static FL_OBJECT * CANCEL_BUTTON;
static FL_OBJECT * GVIS_CHECKB;

//static functions declaration
   //create
static void g3d_create_retract_frame(void);
static void g3d_create_gvis_checkb(void);
static void g3d_create_ndraw_counter(void);
static void g3d_create_nbretract_input(void);
static void g3d_create_dof1_counter(void);
static void g3d_create_dof2_counter(void);
static void g3d_create_node_frame(void);
static void g3d_create_retract_strat_frame(void);
static void g3d_create_vis_graph_frame(void);
   //callbacks
static int CB_pathDeformForm_OnClose(FL_FORM *form, void *arg);
static void CB_cancel_button(FL_OBJECT *ob, long arg);
static void CB_cycle_checkb(FL_OBJECT *ob, long arg);
static void CB_retract_checkb(FL_OBJECT *ob,  long arg);
static void CB_discreet_checkb(FL_OBJECT *ob, long arg);
static void CB_ndraw_counter(FL_OBJECT *ob, long arg);
static void CB_dof1_counter(FL_OBJECT *ob, long arg);
static void CB_dof2_counter(FL_OBJECT *ob, long arg);
static void CB_nbretract_input(FL_OBJECT *ob, long arg);
static void CB_add_node_button(FL_OBJECT *ob, long arg);
static void CB_last_node_button(FL_OBJECT *ob, long arg);
static void CB_replot_button(FL_OBJECT *ob, long arg);
static void CB_clear_button(FL_OBJECT *ob, long arg);
static void CB_node_counter(FL_OBJECT *ob, long arg);
static void CB_plot_all_checkb(FL_OBJECT *ob, long arg);
static void CB_plot_all_input(FL_OBJECT *ob, long arg);
static void CB_filter_graph_checkb(FL_OBJECT *ob, long arg);
static void CB_RetractPlan_strategy(FL_OBJECT *ob, long arg);
static void CB_draw_button(FL_OBJECT *ob, long arg);
static void CB_gvis_checkb(FL_OBJECT *ob, long arg);
static void CB_vis_graph_xyplot(FL_OBJECT *ob, long arg);
  //delete
static void g3d_delete_retract_frame(void);
static void g3d_delete_gvis_checkb(void);
static void g3d_delete_ndraw_counter(void);
static void g3d_delete_nbRetract_input(void);
static void g3d_delete_dof1_counter(void);
static void g3d_delete_dof2_counter(void);
static void g3d_delete_node_frame(void);
static void g3d_delete_retract_strat_frame(void);
static void g3d_delete_vis_graph_frame(void);


  //variables
int FLAG_IS_ALLPLOT = FALSE;
  //Other functions
static void p3d_compute_proj_in_form(p3d_graph* G);
static void g3d_plot_proj_in_form(p3d_graph* G);
static void p3d_addNodeInGraph(void);
static void p3d_addLastNodeInGraph(void);
//initialisation
/****************************************************************************/
/** \brief Create the path deformation form.
 */
/****************************************************************************/
void g3d_create_path_deformation_form(void){
  g3d_create_form(&PATH_DEFORM_FORM,320,320,FL_UP_BOX);
  g3d_create_retract_frame();
  g3d_create_gvis_checkb();
  g3d_create_frame(&SPACE1,FL_NO_FRAME,100,30.0,"",(void**)&PATH_DEFORM_FORM,1);
  g3d_create_nbretract_input();
  g3d_create_ndraw_counter();
  g3d_create_dof1_counter();
  g3d_create_dof2_counter();
  g3d_create_node_frame();
  g3d_create_vis_graph_frame();

  fl_end_form();
  fl_set_form_icon(PATH_DEFORM_FORM, GetApplicationIcon(), 0);
  fl_set_form_atclose(PATH_DEFORM_FORM, CB_pathDeformForm_OnClose, NULL);
}

//creating FL_OBJECTS
/****************************************************************************/
/** \brief Create the Retract frame containing Cycle and Retract
 checkbuttons
 */
/****************************************************************************/
static void g3d_create_retract_frame(void){
  g3d_create_frame(&RETRACT_FRAME,FL_ENGRAVED_FRAME,-1,-1,"",(void**)&PATH_DEFORM_FORM,1);
  g3d_create_checkbutton(&CYCLE_CHECKB,FL_PUSH_BUTTON,-1,-1,"Cycle",(void**)&RETRACT_FRAME,0);
  fl_set_object_color(CYCLE_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(CYCLE_CHECKB,CB_cycle_checkb,0);
  if (p3d_get_cycles())
    fl_set_button(CYCLE_CHECKB,1);
  g3d_create_checkbutton(&RETRACT_CHECKB,FL_PUSH_BUTTON,-1,-1,"Retract",(void**)&RETRACT_FRAME,0);
  fl_set_object_color(RETRACT_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(RETRACT_CHECKB,CB_retract_checkb,0);
  if (p3d_get_test_reductib())
    fl_set_button(RETRACT_CHECKB,1);
  g3d_create_checkbutton(&DISCREET_CHECKB,FL_PUSH_BUTTON,-1,-1,"Discreet",(void**)&RETRACT_FRAME,0);
  fl_set_object_color(DISCREET_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(DISCREET_CHECKB,CB_discreet_checkb,0);
  if (p3d_get_is_visibility_discreet())
    fl_set_button(DISCREET_CHECKB,1);
}
/****************************************************************************/
/** @brief Create the Graph visibility checkbox. By enabling Gvis, you can draw the Visibility graph on the XYPLOT.
 */
/****************************************************************************/
static void g3d_create_gvis_checkb(void){
  g3d_create_checkbutton(&GVIS_CHECKB,FL_PUSH_BUTTON,-1,-1,"Enable Gvis",(void**)&PATH_DEFORM_FORM,1);
  fl_set_object_color(GVIS_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(GVIS_CHECKB,CB_gvis_checkb,0);
  if (p3d_get_gvis())
    fl_set_button(GVIS_CHECKB,1);
}
/****************************************************************************/
/** \brief Create the ndraw counter
 */
/****************************************************************************/
static void g3d_create_ndraw_counter(void){
  g3d_create_counter(&NDRAW_COUNTER,FL_SIMPLE_COUNTER,50,15,"ndraw",(void**)&PATH_DEFORM_FORM,1);
  fl_set_counter_value(NDRAW_COUNTER, 100);
  fl_set_counter_bounds(NDRAW_COUNTER, 1,(double)p3d_get_nretract());
  fl_set_counter_precision(NDRAW_COUNTER, 0);
  fl_set_counter_step(NDRAW_COUNTER, 1, 10);
  fl_set_call_back(NDRAW_COUNTER,CB_ndraw_counter,0);
}
/****************************************************************************/
/** \brief Create the nbretract input
 */
/****************************************************************************/
static void g3d_create_nbretract_input(void){
  char buffer [10];
  g3d_create_input(&NBRETRACT_INPUT,FL_NORMAL_INPUT,20,20,"nbRetract",(void**)&PATH_DEFORM_FORM,1);
  sprintf(buffer, "%d", p3d_get_nretract_max());
  fl_set_input(NBRETRACT_INPUT, buffer);
  fl_set_call_back(NBRETRACT_INPUT,CB_nbretract_input,0);
}

/****************************************************************************/
/** \brief Create the dof1 counter
 */
/****************************************************************************/
static void g3d_create_dof1_counter(void){
  p3d_rob *robotPt;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  g3d_create_counter(&DOF1_COUNTER,FL_SIMPLE_COUNTER,50,15,"dof1",(void**)&PATH_DEFORM_FORM,1);
  fl_set_counter_precision(DOF1_COUNTER, 0);
  fl_set_counter_bounds(DOF1_COUNTER, 0,robotPt->nb_user_dof-1);
  fl_set_counter_step(DOF1_COUNTER, 1, 1);
  fl_set_counter_value (DOF1_COUNTER,  p3d_get_dof1());
  fl_set_object_callback(DOF1_COUNTER,CB_dof1_counter,0);
}
/****************************************************************************/
/** \brief Create the dof2 counter
 */
/****************************************************************************/
static void g3d_create_dof2_counter(void){
  p3d_rob *robotPt;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  g3d_create_counter(&DOF2_COUNTER,FL_SIMPLE_COUNTER,50,15,"dof2",(void**)&PATH_DEFORM_FORM,1);
  fl_set_counter_precision(DOF2_COUNTER, 0);
  fl_set_counter_bounds(DOF2_COUNTER, 0, robotPt->nb_user_dof-1);
  fl_set_counter_step(DOF2_COUNTER, 1, 1);
  fl_set_counter_value (DOF2_COUNTER,  p3d_get_dof2());
  fl_set_object_callback(DOF2_COUNTER,CB_dof2_counter,0);
}
/****************************************************************************/
/** \brief Create node frame and this FL_OBJECT componants.
 */
/****************************************************************************/
static void g3d_create_node_frame(void){
  char buffer[10];
  g3d_create_frame(&NODE_FRAME,FL_NO_FRAME,120,-1,"",(void**)&PATH_DEFORM_FORM,1);
  //add Node
  g3d_create_button(&ADD_NODE_BUTTON,FL_NORMAL_BUTTON,55,20,"Add Node",(void**)&NODE_FRAME,0);
  fl_set_call_back(ADD_NODE_BUTTON,CB_add_node_button,0);
  //Last Node
  g3d_create_button(&LAST_NODE_BUTTON,FL_NORMAL_BUTTON,55,20,"Last Node",(void**)&NODE_FRAME,0);
  fl_set_call_back(LAST_NODE_BUTTON,CB_last_node_button,0);
  //Replot
  g3d_create_button(&REPLOT_BUTTON,FL_NORMAL_BUTTON,55,20,"Replot",(void**)&NODE_FRAME,0);
  fl_set_call_back(REPLOT_BUTTON,CB_replot_button,0);
  //Clear
  g3d_create_button(&CLEAR_BUTTON,FL_NORMAL_BUTTON,55,20,"Clear",(void**)&NODE_FRAME,0);
  fl_set_call_back(CLEAR_BUTTON,CB_clear_button,0);
  //Node counter
  g3d_create_counter(&NODE_COUNTER,FL_NORMAL_COUNTER,100,20,"Node",(void**)&NODE_FRAME,0);
  fl_set_counter_value(NODE_COUNTER, 0);
  if(XYZ_GRAPH != NULL) {
    fl_set_counter_bounds(NODE_COUNTER, 0, (double)( XYZ_GRAPH->nnode -1));
  } else {
    fl_set_counter_bounds(NODE_COUNTER, 0, 0);
  }
  fl_set_counter_step(NODE_COUNTER, 1, 10);
  fl_set_counter_precision(NODE_COUNTER, 0);
  fl_set_call_back(NODE_COUNTER,CB_node_counter,0);
  //Plot all Check box
  g3d_create_checkbutton(&PLOT_ALL_CHECKB,FL_PUSH_BUTTON,-1,20,"Plot all",(void**)&NODE_FRAME,0);
  fl_set_object_color(PLOT_ALL_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(PLOT_ALL_CHECKB,CB_plot_all_checkb,0);
  if (FLAG_IS_ALLPLOT)
    fl_set_button(PLOT_ALL_CHECKB,1);
  //Plot all input
  g3d_create_input(&PLOT_ALL_INPUT,FL_NORMAL_INPUT,40,20,"",(void**)&NODE_FRAME,0);
  sprintf(buffer, "%d", p3d_get_Nstep());
  fl_set_input(PLOT_ALL_INPUT, buffer);
  fl_set_call_back(PLOT_ALL_INPUT,CB_plot_all_input,0);
  //Filter graph
  g3d_create_checkbutton(&FILTER_GRAPH_CHECKB,FL_PUSH_BUTTON,-1,20,"Filter graph",(void**)&NODE_FRAME,0);
  fl_set_object_color(FILTER_GRAPH_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(FILTER_GRAPH_CHECKB,CB_filter_graph_checkb,0);
  if (p3d_get_show_cycle_only())
    fl_set_button(FILTER_GRAPH_CHECKB,1);
  //Retract strat frame
  g3d_create_retract_strat_frame();
}
/****************************************************************************/
/** \brief Create retract strat frame and this FL_OBJECT componants.
 */
/****************************************************************************/
static void g3d_create_retract_strat_frame(void){
  g3d_create_labelframe(&RETRACT_STRAT_FRAME,FL_ENGRAVED_FRAME,-1,-1,"Retract Strat",(void**)&NODE_FRAME,0);
  RETRACT_STRAT_GROUP = fl_bgn_group();
  //Glob Search
  g3d_create_checkbutton(&GLOBAL_SEARCH_CHECKB,FL_RADIO_BUTTON,-1,20,"Glob search",(void**)&RETRACT_STRAT_FRAME,0);
  fl_set_object_color(GLOBAL_SEARCH_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(GLOBAL_SEARCH_CHECKB,CB_RetractPlan_strategy,(long)GLOBAL_SEARCH_CHECKB);
  //Forw Search
  g3d_create_checkbutton(&FORW_SEARCH_CHECKB,FL_RADIO_BUTTON,-1,20,"Forw search",(void**)&RETRACT_STRAT_FRAME,0);
  fl_set_object_color(FORW_SEARCH_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(FORW_SEARCH_CHECKB,CB_RetractPlan_strategy,(long)FORW_SEARCH_CHECKB);
  //Linear Search
  g3d_create_checkbutton(&LINEAR_SEARCH_CHECKB,FL_RADIO_BUTTON,-1,20,"Linear search",(void**)&RETRACT_STRAT_FRAME,0);
  fl_set_object_color(LINEAR_SEARCH_CHECKB,FL_MCOL,FL_GREEN);
  fl_set_call_back(LINEAR_SEARCH_CHECKB,CB_RetractPlan_strategy,(long)LINEAR_RETRACT_SEARCH);
  //RETRACT_STRAT_GROUP =
  fl_end_group();
  switch (p3d_get_retract_search_type()){
    case (ALL_RETRACT_SEARCH):{
      fl_set_button(GLOBAL_SEARCH_CHECKB,1);
    break;
    }
    case (FORWARD_RETRACT_SEARCH):{
      fl_set_button(FORW_SEARCH_CHECKB,1);
    break;
    }
    case (LINEAR_RETRACT_SEARCH):{
      fl_set_button(LINEAR_SEARCH_CHECKB,1);
    break;
    }
  }
}
/****************************************************************************/
/** \brief Create visibility graph frame and this FL_OBJECT componants.
 */
/****************************************************************************/
static void g3d_create_vis_graph_frame(void){
  g3d_create_frame(&VIS_GRAPH_FRAME,FL_NO_FRAME,155,-1,"",(void**)&PATH_DEFORM_FORM,1);
  //vis graph
  g3d_create_xyplot(&VIS_GRAPH_XYPLOT,FL_POINTS_XYPLOT,150,150,"",(void**)&VIS_GRAPH_FRAME,0);
  fl_set_xyplot_inspect(VIS_GRAPH_XYPLOT,1);//set the xyplot reagent to mouse events
  fl_set_object_color(VIS_GRAPH_XYPLOT, FL_SLATEBLUE, FL_WHITE);
  fl_set_call_back(VIS_GRAPH_XYPLOT,CB_vis_graph_xyplot,0);
  //Draw
  g3d_create_button(&DRAW_BUTTON,FL_NORMAL_BUTTON,50.0,30.0,"Draw",(void**)&VIS_GRAPH_FRAME,0);
  fl_set_call_back(DRAW_BUTTON,CB_draw_button,0);
  //space 2
  g3d_create_frame(&SPACE2,FL_NO_FRAME,40,-1,"",(void**)&VIS_GRAPH_FRAME,0);
  //Cancel
  g3d_create_button(&CANCEL_BUTTON,FL_NORMAL_BUTTON,50.0,30.0,"Cancel",(void**)&VIS_GRAPH_FRAME,0);
  fl_set_call_back(CANCEL_BUTTON,CB_cancel_button,0);
}

//CallBacks
/****************************************************************************/
/** \brief CallBack for Cycle checkbutton.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_cycle_checkb(FL_OBJECT *ob, long arg){
  int val;
  
  val = fl_get_button(ob);
  p3d_set_cycles(val);
}
/****************************************************************************/
/** \brief CallBack for Retract checkbutton.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_retract_checkb(FL_OBJECT *ob, long arg){
  int val;
  val = fl_get_button(ob);
  p3d_set_test_reductib(val);
}
/****************************************************************************/
/** \brief CallBack for discreet checkbutton.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_discreet_checkb(FL_OBJECT *ob, long arg){
  int val;
  val = fl_get_button(ob);
  p3d_set_is_visibility_discreet(val);
}
/****************************************************************************/
/** \brief CallBack for discreet checkbutton.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_gvis_checkb(FL_OBJECT *ob, long arg){
  int val;
  val = fl_get_button(ob);
  p3d_set_gvis(val);
}
/****************************************************************************/
/** \brief CallBack for ndraw counter.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_ndraw_counter(FL_OBJECT *ob, long arg){
  int index;

  fl_set_counter_bounds(NDRAW_COUNTER, 1,(double)p3d_get_nretract() );
  index = (int) fl_get_counter_value(NDRAW_COUNTER);
  p3d_set_num_draw_proj(index);
  g3d_plot_proj_in_form(XYZ_GRAPH);
}
/****************************************************************************/
/** \brief CallBack for nbretract input.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_nbretract_input(FL_OBJECT *ob, long arg){
  p3d_set_nretract_max(atof(fl_get_input(NBRETRACT_INPUT)) == 0?p3d_get_nretract_max():atof(fl_get_input(NBRETRACT_INPUT)));
}
/****************************************************************************/
/** \brief CallBack for dof1 counter.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_dof1_counter(FL_OBJECT *ob, long arg){
  p3d_rob *robotPt;
  int dof1;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  dof1 = p3d_robot_user_dof_to_dof(robotPt,(int)floor(fl_get_counter_value(DOF1_COUNTER)) );
  p3d_set_dof1(dof1);
  g3d_draw_allwin_active();
}
/****************************************************************************/
/** \brief CallBack for dof2 counter.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_dof2_counter(FL_OBJECT *ob, long arg){
  p3d_rob *robotPt;
  int dof2;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  dof2 = p3d_robot_user_dof_to_dof(robotPt,(int)floor(fl_get_counter_value(DOF2_COUNTER)));
  p3d_set_dof2(dof2);
  g3d_draw_allwin_active();
}
/****************************************************************************/
/** \brief CallBack for add node button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void p3d_addNodeInGraph(void){
  configPt qcurrent = NULL;
  pp3d_rob robotPt;
  p3d_node* current_nodePt;
  p3d_graph* G = NULL;
  double dist;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if(!XYZ_GRAPH) {
    G = p3d_create_graph();
  }else{ 
    G = XYZ_GRAPH;
  }
  qcurrent = p3d_get_robot_config(robotPt);
  current_nodePt = p3d_APInode_make_multisol(G,qcurrent,NULL);
  if(G->start_nodePt== NULL) {
    G->start_nodePt = current_nodePt;
    p3d_insert_node(G,G->start_nodePt);
    p3d_create_compco(G,G->start_nodePt);
    if(G->start_nodePt == NULL){
      PrintInfo(("Probleme with startnode \n"));
    }
  }else{
    if(p3d_APInode_linked(G,G->prev_nodePt,current_nodePt,&dist)) {
      p3d_insert_node(G,current_nodePt);
      p3d_create_edges(G,G->prev_nodePt,current_nodePt,dist);
      p3d_add_node_compco(current_nodePt,G->prev_nodePt->comp, TRUE);
      //current_localpathPt = p3d_local_planner(robotPt, G->prev_nodePt->q, current_nodePt->q);
    }else{
      PrintInfo(("edge would be in collision\n"));
      p3d_destroy_config(robotPt, current_nodePt->q);
      MY_FREE(current_nodePt, p3d_node, 1);
      current_nodePt = NULL;
    }
  }
  if(current_nodePt !=NULL) {
    G->prev_nodePt = current_nodePt;
  }
}

static void CB_add_node_button(FL_OBJECT *ob, long arg){
  p3d_addNodeInGraph();
  g3d_draw_allwin_active();
}
/****************************************************************************/
/** \brief CallBack for Last Node button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void p3d_addLastNodeInGraph(void){
  configPt qcurrent;
  double dist;
  pp3d_rob robotPt;
  p3d_graph* G = NULL;
  p3d_node* current_nodePt;
  p3d_list_edge* list_edge;
  
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if(!XYZ_GRAPH) {
    G = p3d_create_graph();
  }else{
    G = XYZ_GRAPH;
  }

  if (G->start_nodePt != NULL) {
    qcurrent = p3d_get_robot_config(robotPt);
    current_nodePt = p3d_APInode_make_multisol(G,qcurrent,NULL);
    if(p3d_APInode_linked(G,G->prev_nodePt,current_nodePt,&dist)) {
      if (G->traj1Pt == NULL) {
        p3d_insert_node(G,current_nodePt);
        p3d_create_edges(G,G->prev_nodePt,current_nodePt,dist);
        p3d_add_node_compco(current_nodePt,G->prev_nodePt->comp, TRUE);
        G->search_start = G->start_nodePt;
        G->search_goal = current_nodePt;
        G->traj1Pt = p3d_graph_to_traj(robotPt);
        if (G->traj1Pt != NULL) {
          G->prev_nodePt = G->start_nodePt;
          G->last_nodePt = current_nodePt;
          //bloque le 1er path
          current_nodePt->edges->E->unvalid = TRUE;
          list_edge = current_nodePt->edges->E->Nf->edges;
          while (list_edge!= NULL) {
            if(list_edge->E->Nf == current_nodePt) {
              list_edge->E->unvalid = TRUE;
            }
            list_edge = list_edge->next;
          }
        }else{
          PrintInfo(("Path1 not found!\n"));
        }
      }else{
        if(p3d_APInode_linked(G,current_nodePt,G->last_nodePt,&dist)) {
          p3d_insert_node(G,current_nodePt);
          p3d_create_edges(G,G->prev_nodePt,current_nodePt,dist);
          p3d_add_node_compco(current_nodePt,G->prev_nodePt->comp, TRUE);
          p3d_create_edges(G,current_nodePt,G->last_nodePt,dist);
          G->search_start = G->start_nodePt;
          G->search_goal = G->last_nodePt;
          G->traj2Pt = p3d_graph_to_traj(robotPt);

          if (G->traj2Pt != NULL) {
            p3d_compute_proj_in_form(G);
          }else{
            PrintInfo(("Path not found!\n"));
          }
        }else{
          PrintInfo(("edge would be in collision\n"));
          p3d_destroy_config(robotPt, current_nodePt->q);
          MY_FREE(current_nodePt, p3d_node, 1);
          current_nodePt = NULL;
        }
      }
    }else{
      PrintInfo(("edge would be in collision\n"));
      p3d_destroy_config(robotPt, current_nodePt->q);
      MY_FREE(current_nodePt, p3d_node, 1);
      current_nodePt = NULL;
    }
  }
}

static void CB_last_node_button(FL_OBJECT *ob, long arg){
  p3d_addLastNodeInGraph();
  g3d_draw_allwin_active();
}
/****************************************************************************/
/** \brief CallBack for replot button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_replot_button(FL_OBJECT *ob, long arg){
  p3d_graph* G;
  G = XYZ_GRAPH;
  p3d_compute_proj_in_form(G);
}
/****************************************************************************/
/** \brief CallBack for Clear button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_clear_button(FL_OBJECT *ob, long arg){
  pp3d_rob robotPt;
  p3d_compco * comp;
  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  configPt qsave;

  qsave = p3d_get_robot_config(robotPt);
  if( (XYZ_GRAPH != NULL) && (XYZ_GRAPH->start_nodePt!= NULL)) {
    comp = XYZ_GRAPH->start_nodePt->comp;
    p3d_clear_comp(XYZ_GRAPH,comp);
    p3d_remove_compco(XYZ_GRAPH,comp);
    XYZ_GRAPH->start_nodePt->comp = NULL;
    XYZ_GRAPH->start_nodePt = NULL;
    XYZ_GRAPH->last_nodePt = NULL;
    XYZ_GRAPH->prev_nodePt = NULL;
    if(XYZ_GRAPH->traj1Pt!= NULL) {
      p3d_del_traj(XYZ_GRAPH->traj1Pt);//mokhtar
      XYZ_GRAPH->traj1Pt= NULL;
    }
    if(XYZ_GRAPH->traj2Pt!= NULL) {
      p3d_del_traj(XYZ_GRAPH->traj2Pt);//mokhtar
      XYZ_GRAPH->traj2Pt= NULL;
    }
    p3d_set_robot_config(robotPt,qsave);
    p3d_destroy_config(robotPt,qsave);
    g3d_draw_allwin_active();
  }
}
/****************************************************************************/
/** \brief CallBack for Node counter.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_node_counter(FL_OBJECT *ob, long arg){
  int num_node;
  p3d_list_node* list_node;
  int i;
  pp3d_rob  robotPt;
  int max_count;
  configPt q_copy;

  robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if(XYZ_GRAPH != NULL) {
    fl_set_counter_bounds(NODE_COUNTER, 1, (double)( XYZ_GRAPH->nnode));
    num_node = (int) fl_get_counter_value(NODE_COUNTER);
    list_node = XYZ_GRAPH->nodes;
    max_count = MIN(num_node, XYZ_GRAPH->nnode);
    for(i = 1; i<max_count;i++) {
      list_node = list_node->next;
    }
    if(list_node!=NULL){
      q_copy = p3d_copy_config(robotPt,list_node->N->q);
      p3d_set_robot_config(robotPt,q_copy);
      //edit mokhtar création de trajectoires
      p3d_copy_config_into(robotPt, q_copy, &(robotPt->ROBOT_POS));
      // Very strange : g3d_draw_allwin_active() must
      //be called twice to draw correctly the position of the robot
      g3d_draw_allwin_active();
      g3d_draw_allwin_active();
    }
  }else{
    fl_set_counter_bounds(NODE_COUNTER, 0, 0);
  }
}
/****************************************************************************/
/** \brief CallBack for Plot all check button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_plot_all_checkb(FL_OBJECT *ob, long arg){
  FLAG_IS_ALLPLOT = !FLAG_IS_ALLPLOT;
}
/****************************************************************************/
/** \brief CallBack for Plot all input.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_plot_all_input(FL_OBJECT *ob, long arg){
  p3d_set_Nstep(atoi(fl_get_input(PLOT_ALL_INPUT)));
}
/****************************************************************************/
/** \brief CallBack for filter graph check button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_filter_graph_checkb(FL_OBJECT *ob, long arg){
  p3d_set_show_cycle_only(!p3d_get_show_cycle_only());
  g3d_draw_allwin_active();
}
/****************************************************************************/
/** \brief CallBack for retract plan strategy radio button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_RetractPlan_strategy(FL_OBJECT *ob, long arg){
  if(arg == 0){p3d_set_retract_search_type(ALL_RETRACT_SEARCH);}
  if(arg == 1){p3d_set_retract_search_type(FORWARD_RETRACT_SEARCH);}
  if(arg == 2){p3d_set_retract_search_type(LINEAR_RETRACT_SEARCH);}
}
/****************************************************************************/
/** \brief CallBack for XY plot.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
void CB_vis_graph_xyplot(FL_OBJECT *ob, long arg){
  float x = 0,y = 0;
  int i = 0,j1 = 0, j2 = 0, dof1 = p3d_get_dof1(), dof2 = p3d_get_dof2();
  double vmin1, vmin2, vmax1, vmax2;
  p3d_jnt * jntPt1, *jntPt2;
  configPt qnew;

  fl_get_xyplot(VIS_GRAPH_XYPLOT, &x,&y,&i);
  jntPt1 = p3d_robot_dof_to_jnt(XYZ_ROBOT, dof1,  &j1);
  jntPt2 = p3d_robot_dof_to_jnt(XYZ_ROBOT, dof2,  &j2);
  p3d_jnt_get_dof_rand_bounds(jntPt1, j1, &vmin1, &vmax1);
  p3d_jnt_get_dof_rand_bounds(jntPt2, j2, &vmin2, &vmax2);
  qnew = p3d_get_robot_config(XYZ_ROBOT);
  qnew[dof1] = vmin1 + x;
  qnew[dof2] = vmin2 + y;
  p3d_set_and_update_this_robot_conf(XYZ_ROBOT, qnew);
  g3d_draw_allwin();
}

/****************************************************************************/
/** \brief CallBack for draw button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_draw_button(FL_OBJECT *ob, long arg){
  p3d_graph* G;
  int i = 0;

  G = XYZ_GRAPH;
  p3d_compute_2D_Cspace_proj(XYZ_ROBOT, p3d_get_dof1(),p3d_get_dof2(),p3d_get_Nstep());
  g3d_plot_proj_in_form(G);
  for(i = 1; i < p3d_get_nretract_max() + 1; i++) {
    del_plot_file(i);
  }
}
/****************************************************************************/
/** \brief CallBack for the cancel button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_cancel_button(FL_OBJECT *ob, long arg){
  g3d_destroy_pathDeformForm();
  fl_set_button(PATH_DEFORMATION,0);//release the button path_Deformation on planner FORM
}
/****************************************************************************/
/** \brief This function is called when the "Path deformation" window is closed from the X button
  If we do not use this call back, XForms tries to close the entire application
    and we do not want that. Instead we will just click on the Cancel button
 \param *form a pointer on the FL_FORM
 \param *arg argument for the call back function (not used)
 \return FL_IGNORE.
 */
/****************************************************************************/
static int CB_pathDeformForm_OnClose(FL_FORM *form, void *arg)
{
  //Call the fonction closing tthe form.
  g3d_destroy_pathDeformForm();
  fl_set_button(PATH_DEFORMATION,0);//release the button path_Deformation on planner FORM
  //If we return FL_OK, the application will continue to try to shut down itself
  //   if however we return FL_IGNORE, the application will not continue this event
  return FL_IGNORE;
}

//destruction
/****************************************************************************/
/** \brief method is used to destroy the cancel button.
 */
/****************************************************************************/
void g3d_destroy_pathDeformForm(void){
  fl_hide_form(PATH_DEFORM_FORM);
  g3d_delete_retract_frame();
  g3d_delete_gvis_checkb();
  g3d_delete_ndraw_counter();
  g3d_delete_nbRetract_input();
  g3d_delete_dof1_counter();
  g3d_delete_dof2_counter();
  g3d_delete_node_frame();
  g3d_delete_vis_graph_frame();

  g3d_fl_free_form(PATH_DEFORM_FORM);
}
/****************************************************************************/
/** \brief Delete the Retract frame containing Cycle and Retract
 checkbuttons.
 */
/****************************************************************************/
static void g3d_delete_retract_frame(void){
  g3d_fl_free_object(CYCLE_CHECKB);
  g3d_fl_free_object(RETRACT_CHECKB);
  g3d_fl_free_object(DISCREET_CHECKB);
  g3d_fl_free_object(RETRACT_FRAME);
}
/****************************************************************************/
/** \brief Delete the dof2 counter.
 */
/****************************************************************************/
static void g3d_delete_gvis_checkb(void){
  g3d_fl_free_object(GVIS_CHECKB);
}
/****************************************************************************/
/** \brief Delete the ndraw counter.
 */
/****************************************************************************/
static void g3d_delete_ndraw_counter(void){
  g3d_fl_free_object(NDRAW_COUNTER);
}

/****************************************************************************/
/** \brief Delete the nbRetract input.
 */
/****************************************************************************/
static void g3d_delete_nbRetract_input(void){
  g3d_fl_free_object(NBRETRACT_INPUT);
}

/****************************************************************************/
/** \brief Delete the dof1 counter.
 */
/****************************************************************************/
static void g3d_delete_dof1_counter(void){
  g3d_fl_free_object(DOF1_COUNTER);
}
/****************************************************************************/
/** \brief Delete the dof2 counter.
 */
/****************************************************************************/
static void g3d_delete_dof2_counter(void){
  g3d_fl_free_object(DOF2_COUNTER);
}
/****************************************************************************/
/** \brief Delete node frame and this FL_OBJECT componants.
 */
/****************************************************************************/
static void g3d_delete_node_frame(void){
  g3d_fl_free_object(ADD_NODE_BUTTON);
  g3d_fl_free_object(LAST_NODE_BUTTON);
  g3d_fl_free_object(REPLOT_BUTTON);
  g3d_fl_free_object(CLEAR_BUTTON);
  g3d_fl_free_object(NODE_COUNTER);
  g3d_fl_free_object(PLOT_ALL_CHECKB);
  g3d_fl_free_object(PLOT_ALL_INPUT);
  g3d_fl_free_object(FILTER_GRAPH_CHECKB);
  g3d_draw_allwin_active();
  g3d_delete_retract_strat_frame();
  g3d_fl_free_object(NODE_FRAME);
}
/****************************************************************************/
/** \brief Delete retract strat frame and this FL_OBJECT componants.
 */
/****************************************************************************/
static void g3d_delete_retract_strat_frame(void){
  g3d_fl_free_object(GLOBAL_SEARCH_CHECKB);
  g3d_fl_free_object(FORW_SEARCH_CHECKB);
  g3d_fl_free_object(LINEAR_SEARCH_CHECKB);
  fl_free_object(RETRACT_STRAT_GROUP);
  g3d_fl_free_object(RETRACT_STRAT_FRAME);
}
/****************************************************************************/
/** \brief Delete visibility graph frame and this FL_OBJECT componants.
 */
/****************************************************************************/
static void g3d_delete_vis_graph_frame(void){
  g3d_fl_free_object(VIS_GRAPH_XYPLOT);
  g3d_fl_free_object(DRAW_BUTTON);
  g3d_fl_free_object(CANCEL_BUTTON);
  g3d_fl_free_object(SPACE2);
  g3d_fl_free_object(VIS_GRAPH_FRAME);
}


//fonction annexes
static void p3d_compute_proj_in_form(p3d_graph* G) {
  int i,is_projectable;

  if((G!= NULL) && (G->traj1Pt!= NULL) && (G->traj2Pt!= NULL)) {
    for(i = 1; i < p3d_get_nretract_max() + 1; i++) {
      del_plot_file(i);
    }
    if(FLAG_IS_ALLPLOT == TRUE) {
      p3d_set_nretract(1);
      p3d_set_num_draw_proj(1);
      fl_set_counter_value(NDRAW_COUNTER, 1.);
      p3d_compute_traj_project(G->rob, G->traj1Pt,G->traj2Pt, p3d_get_Nstep());
    }else{
      p3d_set_nretract(1);
      is_projectable = p3d_test_projection(G->rob, G->traj1Pt, G->traj2Pt, p3d_get_Nstep());
      PrintInfo(("path deformable? : %d\n", is_projectable));
    }
    g3d_plot_proj_in_form(G);
  }
}

static void g3d_plot_proj_in_form(p3d_graph* G) {
  char buf[128], nomfichier[255], rep[255] = "./plotinfos";
  char * env_name, *recup;
  int index;

  index = p3d_get_num_draw_proj();
  strcpy (nomfichier, rep);
  env_name = p3d_get_desc_curname(P3D_ENV);
  strcat(nomfichier, "."); 
  strcat(nomfichier,env_name);
  strcat(nomfichier, ".");
  snprintf(buf,128, "%d", index);
  recup = strdup(buf);
  strcat(nomfichier, recup);
  fl_set_xyplot_file(VIS_GRAPH_XYPLOT, nomfichier,"","","");
  MY_FREE(recup, char, 1);
}

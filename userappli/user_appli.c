#include "UserAppli-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

#ifdef LIGHT_PLANNER
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"
#endif
static int trueFunction(p3d_rob* robot, p3d_localpath* curLp);
static void p3d_fuseGraphs(p3d_rob* robot, p3d_graph* mainGraph, p3d_graph* subGraph);
#ifdef DPG
static int dynamicTraj(p3d_rob* robot, p3d_localpath* curLp);
#endif
static p3d_edge* p3d_getLpEdge(p3d_rob* robot, p3d_graph* graph, p3d_localpath* lp);

void openChainPlannerOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
//   p3d_set_NB_TRY(1000);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}


void globalPlanner(void) {
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
}

static int trueFunction(p3d_rob* robot, p3d_localpath* curLp) {
  g3d_draw_allwin_active();
  return TRUE;
}

void viewTraj(void) {
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
// #ifdef LIGHT_PLANNER
//   switchBBActivationForGrasp();
// #endif
#ifdef DPG
  g3d_show_tcur_rob(robot, dynamicTraj);
#else
  g3d_show_tcur_rob(robot, trueFunction);
#endif
// #ifdef LIGHT_PLANNER
//   switchBBActivationForGrasp();
// #endif
}

/** ////////// Fin Setters /////////////*/

/** ////////// Fonctions Principales /////////////*/
#ifdef LIGHT_PLANNER
extern double USE_LIN;
extern double SAFETY_DIST;
void computeOfflineOpenChain(p3d_rob* robot, p3d_matrix4 objectInitPos){
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
  CB_del_param_obj(NULL, 0);
  deactivateCcCntrts(robot, -1);
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  fixJoint(robot, robot->curObjectJnt, objectInitPos);
  p3d_update_this_robot_pos(robot);
  configPt conf = p3d_get_robot_config(robot);
  print_config(robot, conf);
  openChainPlannerOptions();
  globalPlanner();
  unFixJoint(robot, robot->curObjectJnt);
  unFixJoint(robot, robot->baseJnt);
}

void computeOfflineClosedChain(p3d_rob* robot, p3d_matrix4 objectInitPos){
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->curObjectJnt, objectInitPos);
  switchBBActivationForGrasp();
}
#endif
/** //////////// Fin Fonctions Principales /////////////*/


/** Tests des localpaths */

void nbCollisionPerSecond(void){
  double tu, ts;
  int nbTested = 0;
  int nbInCol = 0;
  ChronoOn();
  configPt q = p3d_alloc_config(XYZ_ROBOT);

  ChronoTimes(&tu, &ts);
  for (;tu < 5; nbTested++){
    p3d_shoot(XYZ_ROBOT, q, 1);
    p3d_set_and_update_robot_conf(q);
    if (p3d_col_test()){
      nbInCol++;
    }
    ChronoTimes(&tu, &ts);
  }
  ChronoPrint("");
  ChronoTimes(&tu, &ts);
  ChronoOff();
  std::cout << "Percenatge in collision = " << ((double) nbInCol / (double) nbTested) << std::endl;
  std::cout << "NbTests per second = " << (double) ((double) nbTested / tu) << std::endl;
}

void nbLocalPathPerSecond(void){
  double tu = 0, ts, totalTime = 0.0;
  int nbTested = 0;
  int nbTests = 0;
  int nbColTests = 0;
  int nbInCol = 0;
  int nbMaxTests = 0;

  configPt q1 = p3d_alloc_config(XYZ_ROBOT);
  configPt q2 = p3d_alloc_config(XYZ_ROBOT);
  p3d_localpath * lp = NULL;
  
  for (;totalTime < 10; nbTested++){
    p3d_shoot(XYZ_ROBOT, q1, 1);
    p3d_set_and_update_robot_conf(q1);
    if (p3d_col_test()){
      nbTested--;
      continue;
    }
    p3d_destroy_config(XYZ_ROBOT, q1);
    q1 = p3d_get_robot_config(XYZ_ROBOT);
    p3d_shoot(XYZ_ROBOT, q2, 1);
    p3d_set_and_update_robot_conf(q2);
    p3d_destroy_config(XYZ_ROBOT, q2);
    q2 = p3d_get_robot_config(XYZ_ROBOT);
    lp = p3d_local_planner_multisol(XYZ_ROBOT, q1, q2, NULL);
    p3d_destroy_config(XYZ_ROBOT, q2);
    q2 = lp->config_at_param(XYZ_ROBOT, lp, MIN(5*p3d_get_env_dmax(), lp->length_lp));
    lp->destroy(XYZ_ROBOT, lp);
    p3d_set_and_update_robot_conf(q2);
    if (p3d_col_test()){
      nbTested--;
      continue;
    }
    p3d_destroy_config(XYZ_ROBOT, q2);
    q2 = p3d_get_robot_config(XYZ_ROBOT);
    lp = p3d_local_planner_multisol(XYZ_ROBOT, q1, q2, NULL);
    ChronoOn();
    if(p3d_unvalid_localpath_test(XYZ_ROBOT, lp, &nbTests)){
      nbInCol++;
    }
    ChronoTimes(&tu, &ts);
    ChronoOff();
    totalTime += tu;
    nbColTests += nbTests;
    if(nbMaxTests < nbTests){
      nbMaxTests = nbTests;
    }
    nbTests = 0;
  }

  std::cout << "Nb Tested = " << nbTested << std::endl;
  std::cout << "Nb Valid = " << nbTested - nbInCol << std::endl;
  std::cout << "Ratio of Valid/Total = " << (double) (nbTested - nbInCol) / (double) nbTested << std::endl;
  std::cout << "----------------------------------" << std::endl;
  std::cout << "nbColTest/sec = " << (double) ((double)nbColTests / totalTime) << std::endl;
  std::cout << "nbColTest/LP = " << (double) nbColTests / (double) nbTested << std::endl;
  std::cout << "nbColTestMax/LP = " << nbMaxTests << std::endl;
}

static void p3d_globalPDRSequence(void){
  int nbTry = ENV.getInt(Env::NbTry);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
  CB_global_search_obj(NULL,0);
  p3d_set_cycles(1);
  p3d_set_is_visibility_discreet(1);
  p3d_set_test_reductib(1);
  ENV.setInt(Env::NbTry,(int)(nbTry/20));
  CB_global_search_obj(NULL,0);
  ENV.setInt(Env::NbTry,nbTry);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

#ifdef MULTIGRAPH

void p3d_specificSuperGraphLearn(void) {
  double *arraytimes = MY_ALLOC(double, p3d_get_NB_specific());
  int nfail = 0;
  configPt qs = NULL, qg = NULL, qStart = NULL, qGoal = NULL;
  double tu = 0.0, ts = 0.0, mgTime = 0.0, gTime = 0.0;
  p3d_rob *robotPt = (p3d_rob *)(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph * graph = NULL, *tmp = NULL;
  p3d_flatSuperGraphNode *startNode = NULL, *goalNode = NULL;

  p3d_SetDiffuStoppedByWeight(0);
  p3d_SetStopValue(FALSE);

  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  if (ENV.getBool(Env::expandToGoal) == true) {
    qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
  }
  if (!XYZ_GRAPH) {
    graph = p3d_create_graph();
  } else {
    graph = XYZ_GRAPH;
  }

  //compute the specific planner for each part of the system
  for (int j = 0; j < robotPt->mg->nbGraphs; j++) {
    p3d_set_user_drawnjnt(robotPt->mg->mgJoints[j]->joints[robotPt->mg->mgJoints[j]->nbJoints - 1]);
    tmp = p3d_setMultiGraphAndActiveDof(robotPt, j);
    qStart = p3d_copy_config(robotPt, qs);
    if (ENV.getBool(Env::expandToGoal) == true) {
      qGoal = p3d_copy_config(robotPt, qg);
    }
    //save
    configPt qPos = robotPt->ROBOT_POS;
    configPt qGoto = robotPt->ROBOT_GOTO;
    //init
    robotPt->ROBOT_POS = qStart;
    robotPt->ROBOT_GOTO = qGoal;
    //execute
    int success = p3d_specific_search((char*)"");
    //restore
    robotPt->ROBOT_POS = qPos;
    robotPt->ROBOT_GOTO = qGoto;
//     p3d_loopSpecificLearn(robotPt, qStart, qGoal, (char*)"", 0, arraytimes, &nfail);
    if (p3d_graph_to_traj(robotPt)) {
      g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
    }
    gTime += robotPt->mg->graphs[j]->time;
  }
  //create the flat super Graph
  if (graph->nnode == 0) {
    robotPt->GRAPH = NULL;
    XYZ_GRAPH = NULL;
    ChronoOn();
    p3d_setAllDofActive(robotPt);
    p3d_flatMultiGraph(robotPt, 1);
    p3d_setAllDofActive(robotPt);
    ChronoTimes(&tu, &ts);
    ChronoOff();
    mgTime += tu;
    //check if there is solution in this graph or not.
    startNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qs);
    goalNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qg);
    robotPt->mg->fsg->search_start = startNode;
    robotPt->mg->fsg->search_goal = goalNode;
    if (!p3d_graph_search(robotPt->mg->fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH)) {
      //continuer la recherche
      ChronoOn();
      p3d_delFlatSuperGraph(robotPt, robotPt->mg->fsg);
      p3d_flatMultiGraph(robotPt, 0);
      p3d_setAllDofActive(robotPt);
      ChronoTimes(&tu, &ts);
      ChronoOff();
      mgTime += tu;
      startNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qs);
      goalNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qg);
      robotPt->mg->fsg->search_start = startNode;
      robotPt->mg->fsg->search_goal = goalNode;
      if (!p3d_graph_search(robotPt->mg->fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH)) {
        p3d_doIncrementalConstruction(1);
        printf("Continue search\n");
      }
    }
  } else {
    ChronoOn();
    p3d_setAllDofActive(robotPt);
    for (int j = 0; j < robotPt->mg->nbGraphs; j++) {
      p3d_fillFlatMultiGraph(robotPt, NULL, NULL, j, 2);
    }
    p3d_setAllDofActive(robotPt);
    ChronoTimes(&tu, &ts);
    ChronoOff();
    mgTime += tu;
    startNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qs);
    goalNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qg);
    if(robotPt->mg->fsg){
      robotPt->mg->fsg->search_start = startNode;
      robotPt->mg->fsg->search_goal = goalNode;
      //check if there is solution in this graph or not.
      if (!p3d_graph_search(robotPt->mg->fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH)) {
        p3d_doIncrementalConstruction(1);
        printf("Continue search\n");
      }
    }
  }
  if (p3d_doIncrementalConstruction(-1)) {
    p3d_set_multiGraph(TRUE);
    p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
    p3d_setAllDofActive(robotPt);
    p3d_doIncrementalConstruction(0);
  }

  p3d_convertFsgToGraph(graph, robotPt->mg->fsg);
  XYZ_GRAPH = graph;
  robotPt->GRAPH = graph;
  graph->time = gTime;
  graph->mgTime = mgTime;
  p3d_print_info_graph(graph);
  p3d_addStartAndGoalNodeToGraph(qs, qg, NULL, NULL, graph, robotPt);
  if(p3d_graph_to_traj(robotPt)){
    printf("A path is found\n");
    g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
  }
}

void p3d_globalSuperGraphLearn(void){
  p3d_graph * final = NULL;
  double graphsTime = 0.0, tu = 0.0, ts = 0.0;
  int graphsNodes = 0, graphsEdges = 0;
  p3d_rob * robotPt = XYZ_ROBOT;
  int nbNodes[3] = {20,20,20};

  if (!XYZ_GRAPH) final = p3d_create_graph();
  else           final = XYZ_GRAPH;

  for(int j = 0; j < robotPt->mg->nbGraphs; j++){
    p3d_set_user_drawnjnt(robotPt->mg->mgJoints[j]->joints[robotPt->mg->mgJoints[j]->nbJoints - 1]);
    p3d_set_NB_NODES(nbNodes[j]);
    p3d_setMultiGraphAndActiveDof(robotPt, j);
//     p3d_globalPDRSequence();
    CB_global_search_obj(NULL,0);
//     CB_print_obj(NULL,0);
    graphsTime += XYZ_GRAPH->time;
    graphsNodes += XYZ_GRAPH->nnode;
    graphsEdges += XYZ_GRAPH->nedge;
    if(STAT){
      setTotalCountVar(XYZ_GRAPH);
      mergeStat(XYZ_GRAPH->stat, final->stat);
    }
  }

  ChronoOn();
  p3d_setAllDofActive(robotPt);
  p3d_flatMultiGraph(robotPt, 0);
  p3d_setAllDofActive(robotPt);
  p3d_convertFsgToGraph(final, robotPt->mg->fsg);
  ChronoTimes(&tu, &ts);
  ChronoOff();
  final->mgTime += tu;
  final->time += graphsTime;
  XYZ_GRAPH = final;
  robotPt->GRAPH = final;
  p3d_set_user_drawnjnt(-1);
  MY_ALLOC_INFO("After p3d_learn");
  p3d_print_info_graph(final);
  if (STAT){
    final->stat->mgNodes = final->nnode;
    final->nnode = final->stat->planConfAdd;
    final->stat->mgEdges = final->nedge/2;
    final->nedge = final->stat->planEdge*2;
    final->stat->mgTime = final->mgTime;
  }
  printf("Graphs Nodes = %d\n", graphsNodes);
  printf("Graphs Edges = %d\n", graphsEdges);
  printf("SG merge time = %f\n", final->mgTime);
}
#endif

static void p3d_reset_graph(p3d_graph * graph) {
  p3d_del_graph(graph);
  p3d_reinit_array_exhausted_nodes();
  MY_ALLOC_INFO("Apres destruction du graphe");
}
double tmpStat[4];
void p3d_computeTests(void){
//   printf("===========================================\n");
//   printf("GLOBAL\n");
//   printf("===========================================\n");
//   p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
//   p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
//   for(int i = 0; i < 20; i++) {
//     p3d_reset_graph(XYZ_GRAPH);
//     XYZ_GRAPH = NULL;
// //     p3d_resetMultiGraph(XYZ_ROBOT);
//     printf("##########  TEST N %d  ############\n", i+1);
//     CB_global_search_obj(NULL,0);
// //     p3d_globalSuperGraphLearn();
// //         p3d_globalPDRSequence();
//     setTotalCountVar(XYZ_GRAPH);
//     mergeStat(XYZ_GRAPH->stat, XYZ_ENV->stat);
//     printStatsGraph(XYZ_GRAPH->stat, 1);
//   }
//   printStatsEnv(XYZ_ENV->stat, 1);
//   p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
//   p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  double stats[31][7];
  for(int i = 0; i < 31; i++){
    for(int j = 0; j < 7; j++){
      stats[i][j] = 0;
    }
  }



//   p3d_rob* robotToMove = XYZ_ENV->robot[10];
//   configPt saveConfig = p3d_get_robot_config(robotToMove);
//   for(int i = 0; i < 1; i++) {
//     p3d_set_and_update_this_robot_conf(robotToMove, saveConfig);
//     p3d_reset_graph(XYZ_GRAPH);
//     //compute the big roadmap
//     CB_global_search_obj(NULL,0);
//     stats[i][0] = XYZ_GRAPH->time;
//     stats[i][1] = XYZ_GRAPH->nnode;
//     //find a traj in the roadmap
//     p3d_specific_search((char*)"");
//     stats[i][0] = XYZ_GRAPH->time - stats[i][0];
//     stats[i][1] = XYZ_GRAPH->nnode - stats[i][1];
//     //put the object randomly on the traj
//     double trajLength = p3d_compute_traj_length(XYZ_ROBOT->tcur);
//     int success = false;
//     configPt computerConfig = p3d_get_robot_config(robotToMove);
//     configPt robotConfig = p3d_get_robot_config(XYZ_ROBOT);
//     do{
//       double randomPos = p3d_random(0, trajLength);
//       configPt randomConfig = p3d_config_at_distance_along_traj(XYZ_ROBOT->tcur, randomPos);
//       computerConfig[6] = randomConfig[21];
//       computerConfig[7] = randomConfig[22];
//       computerConfig[8] = randomConfig[23];
//       computerConfig[9] = randomConfig[24];
//       computerConfig[10] = randomConfig[25];
//       computerConfig[11] = randomConfig[26];
//       p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//       success = !p3d_col_test();
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//       success *= !p3d_col_test();
//     }while(success == false);
//     p3d_set_and_update_this_robot_conf(XYZ_ROBOT, robotConfig);
//     g3d_draw_allwin_active();
//     //launch the processing
//     checkForColPath(XYZ_ROBOT, XYZ_ROBOT->tcur, XYZ_ROBOT->GRAPH, XYZ_ROBOT->ROBOT_POS, XYZ_ROBOT->tcur->courbePt);
//     stats[i][2] = tmpStat[0] != -1 ? 1 : 0;
//     stats[i][3] = tmpStat[0] != -1 ? tmpStat[0] : 0;
//     stats[i][4] = tmpStat[0] != -1 ? tmpStat[1] : 0;
//     stats[i][5] = tmpStat[2];
//     stats[i][6] = tmpStat[3];
//     //print the results
//     printf("trajectory plan time : %f\n", stats[i][0]);
//     printf("trajectory added nodes : %f\n", stats[i][1]);
//     printf("Needs enrichment : %f\n", stats[i][2]);
//     printf("Time for enrichment : %f\n", stats[i][3]);
//     printf("Nodes added in enrichment : %f\n", stats[i][4]);
//     printf("Strategy Time : %f\n", stats[i][5]);
//   }
//   for(int i = 0; i < 20; i++){
//     stats[21][0] += stats[i][0];
//     stats[21][1] += stats[i][1];
//     stats[21][2] += stats[i][2];
//     stats[21][3] += stats[i][3];
//     stats[21][4] += stats[i][4];
//     stats[21][5] += stats[i][5];
//     stats[21][6] += stats[i][6];
//   }
//   printf("trajectory plan time : %f\n", stats[21][0]/20);
//   printf("trajectory added nodes : %f\n", stats[21][1]/20);
//   printf("Needs enrichment : %f\n", stats[21][2]);
//   printf("Time for enrichment : %f\n", stats[21][3]/stats[21][2]);
//   printf("Nodes added in enrichment : %f\n", stats[21][4]/stats[21][2]);
//   printf("Strategy Time : %f\n", stats[21][5]);
//   printf("Strategy Success : %f\n", stats[21][6]);

//Specific
  p3d_rob* robotToMove = XYZ_ENV->robot[1];
  configPt saveConfig = p3d_get_robot_config(robotToMove);
  for(int i = 0; i < 1; i++) {
    p3d_set_and_update_this_robot_conf(robotToMove, saveConfig);
    p3d_reset_graph(XYZ_GRAPH);
    CB_global_search_obj(NULL,0);
    stats[i][0] = XYZ_GRAPH->time;
    //find a traj
    p3d_specific_search((char*)"");

    stats[i][1] = XYZ_GRAPH->time;
    stats[i][2] = XYZ_GRAPH->nnode;
    //0
//     stats[i][0] = XYZ_GRAPH->time;
//     stats[i][1] = XYZ_GRAPH->nnode;
//     p3d_reset_graph(XYZ_GRAPH);
    //0
    //put the object randomly on the traj
//     double trajLength = p3d_compute_traj_length(XYZ_ROBOT->tcur);
//     int success = false;
//     configPt computerConfig = p3d_get_robot_config(robotToMove);
//     configPt robotConfig = p3d_get_robot_config(XYZ_ROBOT);
//     do{
//       double randomPos = p3d_random(0, trajLength);
//       configPt randomConfig = p3d_config_at_distance_along_traj(XYZ_ROBOT->tcur, randomPos);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, randomConfig);
      double x, y, z, rx, ry, rz;
//       p3d_mat4ExtractPosReverseOrder(XYZ_ROBOT->joints[10]->abs_pos, &x, &y, &z, &rx, &ry, &rz);
//       computerConfig[6] = x;
//       computerConfig[7] = y;
//       computerConfig[8] = z;
//       computerConfig[9] = rx;
//       computerConfig[10] = ry;
//       computerConfig[11] = rz;
// 
//       p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_POS);
//       success = !p3d_col_test();
//       p3d_set_and_update_this_robot_conf(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO);
//       success *= !p3d_col_test();
//     }while(success == false);
//     p3d_set_and_update_this_robot_conf(XYZ_ROBOT, robotConfig);
//     g3d_draw_allwin_active();
      p3d_rob* robotToMove = XYZ_ENV->robot[1];
      configPt computerConfig = p3d_get_robot_config(robotToMove);
      computerConfig[6] = 0.44;
      computerConfig[9] = rx;
      computerConfig[10] = ry;
      computerConfig[11] = rz;

      p3d_set_and_update_this_robot_conf(robotToMove, computerConfig);
      g3d_draw_allwin_active();
    //launch the processing
    //0
//     p3d_specific_search((char*)"");
//     stats[i][2] = XYZ_GRAPH->time;
//     stats[i][3] = XYZ_GRAPH->nnode;
    //0
#ifdef DPG
    int j = 0,  optimized = XYZ_ROBOT->tcur->isOptimized;
    do{
      printf("Test %d", j);
      j++;
      checkForColPath(XYZ_ROBOT, XYZ_ROBOT->tcur, XYZ_GRAPH, XYZ_ROBOT->ROBOT_POS, XYZ_ROBOT->tcur->courbePt, optimized);
    }while(tmpStat[3] != -1 && tmpStat[3] != 0);
#endif
    stats[i][3] = tmpStat[0];
    stats[i][4] = tmpStat[1];
    stats[i][5] = tmpStat[2];
    stats[i][6] = tmpStat[3];
    tmpStat[0] = 0;
    tmpStat[1] = 0;
    tmpStat[2] = 0;
    tmpStat[3] = 0;
    //print the results
    printf("graph plan time : %f\n", stats[i][0]);
    printf("trajectory plan time : %f\n", stats[i][1]);
    printf("trajectory nodes : %f\n", stats[i][2]);
    int needs = stats[i][3] == -1 ? 0:1;
    printf("Needs enrichment : %d\n",needs);
    printf("method trajectory plan time : %f\n", stats[i][3]);
    printf("trajectory added nodes : %f\n", stats[i][4]);
    printf("method plan time : %f\n", stats[i][5]);
    printf("method success : %f\n\n\n", stats[i][6]);
  }
  printf("Roadmap time, trajectory plan time, trajectory nodes, Needs enrichment, method trajectory plan time, trajectory added nodes, method plan time, method success\n");
  for(int i = 0; i < 30; i++){
    
    printf("%f, ", stats[i][0]);
    printf("%f, ", stats[i][1]);
    printf("%f, ", stats[i][2]);
    int needs = stats[i][3] == -1 ? 0:1;
    printf("%d, ", needs);
    printf("%f, ", stats[i][3]);
    printf("%f, ", stats[i][4]);
    printf("%f, ", stats[i][5]);
    printf("%f\n", stats[i][6]);
  }
}
#ifdef DPG
static int dynamicTraj(p3d_rob* robot, p3d_localpath* curLp){
  g3d_draw_allwin_active();
  fl_check_forms();
  configPt currentConf = p3d_get_robot_config(robot);
  return checkForColPath(robot, robot->tcur, robot->GRAPH, currentConf, curLp, robot->tcur->isOptimized);

}

// Return Values : -2 = Error  -1 = a collision is avoided  0 = No traj given  1 = Traj is collision Free.

int checkForColPath(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, configPt current, p3d_localpath* currentLp, int optimized){
  if (!traj) {
    return 0;
  }
  p3d_rob* currentRobot = XYZ_ENV->cur_robot;
  p3d_sel_desc_num(P3D_ROBOT, robot->num);
  int ntest = 0;
  double dist = 0;
  double trajLength =  p3d_compute_traj_length(traj);
  bool graphTrajInCollision = false;
//if the trajectory is optimized, add it into the graph if not yet added
//   if(optimized){
//     p3dAddTrajToGraph(robot, mainGraph, traj);
//   }
  //get the current lp
  if(!currentLp){
    currentLp = p3d_findConfigLocalPathInTraj(robot, traj, current);
  }
  //start form the localpath containing the current config of the robot
  p3d_localpath *cur = currentLp;
  double curDist = 0;
  for(p3d_localpath* tmp = traj->courbePt; tmp && tmp != currentLp; tmp = tmp->next_lp){
    curDist += tmp->length_lp;
  }
  dist = curDist;
  for (; cur != NULL && !graphTrajInCollision && dist < curDist + 0.33 * trajLength ; cur = cur->next_lp) {
    if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {//le lp est en collision
#ifdef GRASP_PLANNING
    genomGetCollideStatus(true);
#endif
      ChronoOn();
      graphTrajInCollision = true;
//       printf((char*)"lp in collision\n");
      configPt box[2];
      p3d_localpath * tmpPrev = cur;
      double tmpDist = dist;
      //find the first node (start of a localpath, or current node)
      if(cur != currentLp){
        //la config initiale de cur ne peux pas etre en collision sinon elle aurait ele detecte dans le localpath precedent
        box[0] = p3d_config_at_distance_along_traj(traj, tmpDist);
        tmpPrev = cur;
      }else{
        box[0] = current;
      }

      //find the last node (end of a localpath)
      p3d_localpath * tmpNext = cur;
      tmpDist = dist + cur->length_lp;
      do {
        box[1] = p3d_config_at_distance_along_traj(traj, tmpDist);
        p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, box[1]);
        if (tmpNext->next_lp != NULL) {
          tmpNext = tmpNext->next_lp;
          tmpDist += tmpNext->length_lp;
        }else{
          //la configuration d'arrivee
          box[1] = p3d_config_at_distance_along_traj(traj, trajLength);
          break;
        }
      } while (p3d_col_test());
      configPt tmpQ = p3d_config_at_distance_along_traj(traj, trajLength);
      if(!p3d_equal_config(robot, box[1], tmpQ)){
        tmpDist -= tmpNext->length_lp;
        tmpNext = tmpNext->prev_lp;
      }
      p3d_destroy_config(robot, tmpQ);

      //tagger les edges comme invalides.
      p3d_edge* edge = p3d_getLpEdge(robot, mainGraph, tmpPrev);
      p3d_unvalid_edge(mainGraph, edge);
      for(p3d_localpath* tmplp = tmpPrev->next_lp; tmplp != tmpNext->next_lp; tmplp = tmplp->next_lp){
        //if there is more than one edge, unvalidate the edges starting and ending at each node of the subpath
        for(p3d_list_edge* le = edge->Nf->edges; le ; le = le->next){
          p3d_unvalid_edge(mainGraph, le->E);
        }
        edge = p3d_getLpEdge(robot, mainGraph, tmplp);
        p3d_unvalid_edge(mainGraph, edge);
      }
//       g3d_draw_allwin_active();
      p3d_addStartAndGoalNodeToGraph(current, robot->ROBOT_GOTO, currentLp->ikSol, robot->ikSolGoto, mainGraph, robot);
      //regarder si il y a un chemin valide deja construit dans le graph. Sinon construire un RRt apres avoir séparé le graph en 2 compco
      if(!p3d_graph_to_traj(robot)){
        //separation du graph
        p3d_separate_graph_for_unvalid_edges(robot->GRAPH);
        //sauvegarde
        int random = p3d_get_RANDOM_CHOICE();
        int sampling = p3d_get_SAMPLING_CHOICE();
        int motion = p3d_get_MOTION_PLANNER();
        int biDirection = ENV.getBool(Env::biDir);
        int nbTry = ENV.getInt(Env::NbTry);
        int comp = ENV.getInt(Env::maxNodeCompco);
        int nbNodes = p3d_get_NB_NODES();
        p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
        p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
        p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
        ENV.setInt(Env::NbTry,10000000);
        ENV.setBool(Env::biDir, true);
        ENV.setInt(Env::maxNodeCompco,10000000);
        p3d_set_NB_NODES(10000);
        configPt qPos = robot->ROBOT_POS;
        configPt qGoto = robot->ROBOT_GOTO;
        robot->ROBOT_POS = box[0];
        robot->ROBOT_GOTO = box[1];
//         XYZ_GRAPH = NULL;
//         robot->GRAPH = NULL;
        p3d_set_tmax((int)XYZ_GRAPH->time+20);
        int success = p3d_specific_search((char*)"");
        p3d_set_tmax(1800);
        //restore
        robot->ROBOT_POS = qPos;
        robot->ROBOT_GOTO = qGoto;
        p3d_set_RANDOM_CHOICE(random);
        p3d_set_SAMPLING_CHOICE(sampling);
        p3d_set_MOTION_PLANNER(motion);
        ENV.setInt(Env::NbTry,nbTry);
        ENV.setBool(Env::biDir, biDirection);
        p3d_set_NB_NODES(nbNodes);
        ENV.setInt(Env::maxNodeCompco,comp);
        tmpStat[0] = XYZ_GRAPH->time;
        tmpStat[1] = XYZ_GRAPH->nnode;
        //Fuse the generated graph with the main one
//         p3d_graph* subGraph = XYZ_GRAPH;
//         p3d_fuseGraphs(robot, mainGraph, subGraph);
//         XYZ_GRAPH = mainGraph;
//         robot->GRAPH = mainGraph;
        if (!success){
          printf("Impossible to avoid collision\n");
          tmpStat[3] = 0;
          return -2;
        }
      }else{
        tmpStat[0] = -1;
      }
      cur = tmpNext;
      dist = tmpDist;
      PrintInfo(("Alternative Edge : "));
      ChronoPrint("");
      double tu = 0.0, ts = 0.0;
      ChronoTimes(&tu, &ts);
      tmpStat[2] += tu;
      ChronoOff();
    }else{
      dist += cur->length_lp;
    }
  }
  if(graphTrajInCollision){ //If it's the optimized traj in collision, we change the trajectory with the graph traj.
    p3d_addStartAndGoalNodeToGraph(current, robot->ROBOT_GOTO, currentLp->ikSol, robot->ikSolGoto, robot->GRAPH, robot);
    if(p3d_traj* newTraj = p3d_graph_to_traj(robot)){
//       g3d_draw_allwin_active();
      tmpStat[3] = 1;
      p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
//       g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
      return -1;
    }else{
      g3d_draw_allwin_active();
      printf("oula Mino il y'a soucis!!! \n");
      tmpStat[3] = 0;
      return -2;
    }
  }else{
    tmpStat[3] = -1; //don't need
    p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
    g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
//     g3d_draw_allwin_active();
//     if(optimized){
//       optimiseTrajectory(100,6);
//     }
#ifdef GRASP_PLANNING
    genomGetCollideStatus(false);
#endif
    return 1;
  }

}
#endif

static p3d_edge* p3d_getLpEdge(p3d_rob* robot, p3d_graph* graph, p3d_localpath* lp){
  p3d_node * startNode = NULL, *goalNode = NULL;
  startNode = p3d_TestConfInGraph(graph, lp->config_at_param(robot, lp, 0));
  goalNode = p3d_TestConfInGraph(graph, lp->config_at_param(robot, lp, lp->length_lp));
  if(startNode && goalNode){
    for(p3d_list_edge* lEdge = startNode->edges; lEdge; lEdge = lEdge->next){
      if (lEdge->E->Nf == goalNode){
        return lEdge->E;
      }
    }
  }
  return NULL;
}

p3d_localpath* p3d_findConfigLocalPathInTraj(p3d_rob* robot, p3d_traj* traj, configPt q){
  configPt q1 = NULL, q2 = NULL;
  int dist1 = 0, dist2 = 0;
  for(p3d_localpath* lp = traj->courbePt; lp; lp = lp->next_lp){
    q1 = lp->config_at_param(robot, lp, 0);
    // take into account the constraints.
    p3d_set_and_update_this_robot_conf_multisol(robot, q1, NULL, 0, lp->ikSol);
    p3d_destroy_config(robot, q1);
    q1 = p3d_get_robot_config(robot);
    dist1 = p3d_dist_q1_q2_multisol(robot ,q1, q, lp->ikSol);
    q2 = lp->config_at_param(robot, lp, lp->length_lp);
    // take into account the constraints.
    p3d_set_and_update_this_robot_conf_multisol(robot, q2, NULL, 0, lp->ikSol);
    p3d_destroy_config(robot, q2);
    q2 = p3d_get_robot_config(robot);
    dist2 = p3d_dist_q1_q2_multisol(robot ,q2, q, lp->ikSol);
    int distTot  = p3d_dist_q1_q2_multisol(robot ,q1, q2, lp->ikSol);
    p3d_destroy_config(robot, q1);
    p3d_destroy_config(robot, q2);
    if(distTot - dist1 - dist2 < EPS6 && distTot - dist1 - dist2 > -EPS6){
      return lp;
    }
  }
  return NULL;
}

p3d_node* p3d_findInsertConnectTrajConfigInGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj, configPt q, p3d_localpath* currentLp){
//   //add the current config of the robot to the graph, connect it and get the new trajectory
//   if(!currentLp){
//     currentLp = p3d_findConfigLocalPathInTraj(robot, traj, q);
//   }
//   p3d_node* currentNode = p3d_addConfToGraph(robot, graph, q, currentLp->ikSol);
//   p3d_node* startNode = p3d_addConfToGraph(robot, graph, q1, currentLp->ikSol);
//   p3d_node* endNode = p3d_addConfToGraph(robot, graph, q2, currentLp->ikSol);
//   int startConnected = false, endConnected = false;
//   for(p3d_list_edge* lEdge = currentNode->edges; lEdge && !startConnected && !endConnected; lEdge = lEdge->next){
//     if(lEdge->E->Nf == startNode){
//       startConnected = true;
//     }
//     if(lEdge->E->Nf == startNode){
//       endConnected = true;
//     }
//   }
//   if(!startConnected || !endConnected){
//     p3d_add_node_compco(currentNode, startNode->comp, TRUE);
//   }
//   if(!startConnected){
//     p3d_create_edges(graph, startNode, currentNode, dist1);//create edges between the two nodes
//   }
//   if(!endConnected){
//     p3d_create_edges(graph, currentNode, endNode, dist2);//create edges between the two nodes
//   }
//   return currentNode;
  return NULL;
}

// p3d_node* p3d_addConfToGraph(p3d_rob* robot, p3d_graph* graph, configPt q, int* ikSol){
//   p3d_node* node = p3d_TestConfInGraph(graph, q);
//   if(!node){
//     node = p3d_APInode_make_multisol(graph, q, ikSol);
//     p3d_insert_node(graph, node);
//   }
//   return node;
// }
// 
// void p3dAddTrajToGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj){
//   p3d_node* initNode = NULL, *endNode = NULL;
//   configPt qInit, qEnd;
//   bool nodeAlreadyConnected = false;
//   for(p3d_localpath* lp = traj->courbePt; lp; lp = lp->next_lp){
//     qInit = lp->config_at_param(robot, lp, 0);
//     // take into account the constraint.
//     p3d_set_and_update_this_robot_conf_multisol(robot, qInit, NULL, 0, lp->ikSol);
//     p3d_destroy_config(robot, qInit);
//     qInit = p3d_get_robot_config(robot);
//     qEnd = lp->config_at_param(robot, lp, lp->length_lp);
//     // take into account the constraint.
//     p3d_set_and_update_this_robot_conf_multisol(robot, qEnd, NULL, 0, lp->ikSol);
//     p3d_destroy_config(robot, qEnd);
//     qEnd = p3d_get_robot_config(robot);
//     nodeAlreadyConnected = false;
//     initNode = NULL;
//     endNode = NULL;
//     initNode = p3d_TestConfInGraph(graph, qInit);
//     if(!initNode){
//       printf("QInit n'est pas dans le graph\n");//If qinit is not already in the graph, there is a problem !!!
//       return;
//     }
//     endNode = p3d_TestConfInGraph(graph, qEnd);
//     if(!endNode){
//       endNode = p3d_addConfToGraph(robot, graph, qEnd, lp->ikSol);
//     }
//     //connect qInit and qEnd if its not connected yet
//     for(p3d_list_edge* lEdge = initNode->edges; lEdge; lEdge = lEdge->next){
//       if(lEdge->E->Nf == endNode){
//         nodeAlreadyConnected = true;
//         break;
//       }
//     }
//     if(!nodeAlreadyConnected){
//       p3d_add_node_compco(endNode, initNode->comp, TRUE);
//       double dist = p3d_dist_q1_q2_multisol(robot, qInit, qEnd, lp->ikSol);//take the distance between the two nodes
//       p3d_create_edges(graph, initNode, endNode, dist);//create edges between the two nodes
//     }
//   }
// }

static void p3d_fuseGraphs(p3d_rob* robot, p3d_graph* mainGraph, p3d_graph* subGraph){
  p3d_node* map[2][subGraph->nnode];
  for(int i = 0; i < subGraph->nnode; i++){
    map[0][i] = map[1][i] = NULL;
  }
  p3d_list_node *subLNode = subGraph->nodes;
  for(int i = 0; subLNode; subLNode = subLNode->next, i++){
    p3d_node* existingNode = p3d_TestConfInGraph(mainGraph, subLNode->N->q);
    if(!existingNode){
      existingNode = p3d_APInode_make_multisol(mainGraph, subLNode->N->q, subLNode->N->iksol);
      p3d_insert_node(mainGraph, existingNode);
      p3d_create_compco(mainGraph, existingNode);
    }
    map[0][i] = subLNode->N;
    map[1][i] = existingNode;
    //find in the map the neighbours of subLNode->N and connect the resulting nodes in the main graph
    for(p3d_list_node *lNode = subLNode->N->neighb; lNode; lNode = lNode->next){
      for(int j = 0; j < i; j++){
        if (map[0][j] == lNode->N){
          //existingNode and map[1][j] have to be connected (and their compco fused)
          int* ikSol = NULL;
          if (existingNode->numcomp < map[1][j]->numcomp) {
            p3d_merge_comp(mainGraph, existingNode->comp, &(map[1][j]->comp));// merge the two compco
            p3d_get_non_sing_iksol(robot->cntrt_manager, existingNode->iksol, map[1][j]->iksol, &ikSol);
            int dist = p3d_dist_q1_q2_multisol(robot, existingNode->q, map[1][j]->q, ikSol);
            p3d_create_edges(mainGraph, existingNode, map[1][j], dist);//link the two nodes
          } else if (existingNode->numcomp > map[1][j]->numcomp){
            p3d_merge_comp(mainGraph, map[1][j]->comp, &(existingNode->comp));// merge the two compco
            p3d_get_non_sing_iksol(robot->cntrt_manager, existingNode->iksol, map[1][j]->iksol, &ikSol);
            int dist = p3d_dist_q1_q2_multisol(robot, existingNode->q, map[1][j]->q, ikSol);
            p3d_create_edges(mainGraph, existingNode, map[1][j], dist);//link the two nodes
          }
        }
      }
    }
  }
}

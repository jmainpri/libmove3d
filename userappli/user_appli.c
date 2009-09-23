#include "UserAppli-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

#ifdef LIGHT_PLANNER
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/robotPos.h"
#endif
static int trueFunction(void);
static void p3d_fuseGraphs(p3d_rob* robot, p3d_graph* mainGraph, p3d_graph* subGraph);

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

static int trueFunction(void) {
  g3d_draw_allwin_active();
  return TRUE;
}

void viewTraj(void) {
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
#ifdef LIGHT_PLANNER
  switchBBActivationForGrasp();
#endif
  g3d_show_tcur_rob(robot, trueFunction);
#ifdef LIGHT_PLANNER
  switchBBActivationForGrasp();
#endif
}

void showConfig(configPt conf){
  p3d_set_and_update_robot_conf(conf);
  g3d_refresh_allwin_active();
  sleep(1);
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
  fixJoint(robot, robot->objectJnt, objectInitPos);
  p3d_update_this_robot_pos(robot);
  configPt conf = p3d_get_robot_config(robot);
  print_config(robot, conf);
  openChainPlannerOptions();
  globalPlanner();
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
}

void computeOfflineClosedChain(p3d_rob* robot, p3d_matrix4 objectInitPos){
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  switchBBActivationForGrasp();
}
#endif
/** //////////// Fin Fonctions Principales /////////////*/


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
    p3d_loopSpecificLearn(robotPt, qStart, qGoal, (char*)"", 0, arraytimes, &nfail);
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

void p3d_computeTests(void){
  printf("===========================================\n");
  printf("GLOBAL\n");
  printf("===========================================\n");
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  for(int i = 0; i < 1; i++) {
    p3d_reset_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    p3d_resetMultiGraph(XYZ_ROBOT);
    printf("##########  TEST N %d  ############\n", i+1);
//     CB_global_search_obj(NULL,0);
    p3d_globalSuperGraphLearn();
//         p3d_globalPDRSequence();
    setTotalCountVar(XYZ_GRAPH);
    mergeStat(XYZ_GRAPH->stat, XYZ_ENV->stat);
    printStatsGraph(XYZ_GRAPH->stat, 1);
  }
  printStatsEnv(XYZ_ENV->stat, 1);
}
#ifdef DPG
//Ne traite pas le cas ou c'est le debut ou la fin du lp qui sont en collision. C'est du changement statique de l'environement. Ne marche pas lors de l'execution. True if a traj is found false otherwise
int checkForCollidingLpAlongPath(void) {
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  p3d_graph* mainGraph = XYZ_GRAPH;
  if (!traj) {
    return FALSE;
  }
  p3d_localpath *cur = traj->courbePt;
  int ntest = 0;
  double dist = 0;
  double nodeDist = 0.0;
  double trajLength =  p3d_compute_traj_length(traj);
  int counter = 0;
  int counterMax = 20;
  bool optimTrajInCollision = false, graphTrajInCollision = false;

  if(traj->isOptimized){//is a otimized trajectory
   //if the optimized traj is in collision use the graph trajectory
    for (; cur != NULL; cur = cur->next_lp){
      if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {
//         cur = traj->trajInGraph;
        optimTrajInCollision = true;
        break;
      }
    }
    //else exit this function
    if(optimTrajInCollision == false){
      return TRUE;
    }else{
      p3dAddTrajToGraph(robot, mainGraph, traj);
    }
  }
  
//Essai : regarder si il n'y a pas un chemin valid deja construit dans le graph
  if(optimTrajInCollision){
    if(p3d_graph_to_traj(robot)){
      g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
      optimTrajInCollision = false;
      if(traj->isOptimized){
        optimiseTrajectory();
      }
    }
  }

  for (; cur != NULL; cur = cur->next_lp) {
    if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {//le lp est en collision
      graphTrajInCollision = true;
      printf((char*)"lp in collision\n");
      configPt box[2], q = p3d_alloc_config(robot);
      p3d_localpath * tmpPrev = cur;
      double tmpDist = dist;
      //find the first node (start of a localpath)
      do {
        box[0] = p3d_config_at_distance_along_traj(traj, tmpDist);
        p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, box[0]);
        if (tmpPrev->prev_lp != NULL){
          tmpPrev = tmpPrev->prev_lp;
          tmpDist -= tmpPrev->length_lp;
        }else{
          //la configuration de depart
          box[0] = p3d_config_at_distance_along_traj(traj, 0);
          break;
        }
      } while (p3d_col_test());
      //save the selected localpath
      configPt tmpQ = p3d_config_at_distance_along_traj(traj, 0);
      if(!p3d_equal_config(robot, box[0], tmpQ)){
        tmpPrev = tmpPrev->next_lp;
      }
      p3d_destroy_config(robot, tmpQ);
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
      //save the selected localpath
      tmpQ = p3d_config_at_distance_along_traj(traj, trajLength);
      if(!p3d_equal_config(robot, box[1], tmpQ)){
        tmpNext = tmpNext->prev_lp;
      }
      p3d_destroy_config(robot, tmpQ);
      //tagger les edges comme invalides.
      p3d_edge* edge = p3d_getLpEdge(robot, robot->GRAPH, tmpPrev);
      p3d_unvalid_edge(robot->GRAPH, edge);
      for(p3d_localpath* tmplp = tmpPrev->next_lp; tmplp != tmpNext->next_lp; tmplp = tmplp->next_lp){
        p3d_edge* edge = p3d_getLpEdge(robot, robot->GRAPH, tmplp);
        edge->unvalid = TRUE;
        if(!robot->GRAPH->oriented){//unvalid the other edge too
          for(p3d_list_edge* lEdge = edge->Nf->edges; lEdge; lEdge = lEdge->next){
            if(lEdge->E->Nf == edge->Ni){
              lEdge->E->unvalid = TRUE;
              break;
            }
          }
        }
      }
      showConfig(box[0]);
      showConfig(box[1]);
      //sauvegarde
      int random = p3d_get_RANDOM_CHOICE();
      int sampling = p3d_get_SAMPLING_CHOICE();
      int motion = p3d_get_MOTION_PLANNER();
      int biDirection = ENV.getBool(Env::biDir);
      int nbTry = ENV.getInt(Env::NbTry);
      int comp = ENV.getInt(Env::maxNodeCompco);
      p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
      p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
      ENV.setInt(Env::NbTry,10000000);
      ENV.setBool(Env::biDir, true);
      ENV.setInt(Env::maxNodeCompco,10000000);
      configPt qPos = robot->ROBOT_POS;
      configPt qGoto = robot->ROBOT_GOTO;
      robot->ROBOT_POS = box[0];
      robot->ROBOT_GOTO = box[1];
      XYZ_GRAPH = NULL;
      robot->GRAPH = NULL;

      int success = p3d_specific_search((char*)"");

      //restore
      robot->ROBOT_POS = qPos;
      robot->ROBOT_GOTO = qGoto;
      p3d_set_RANDOM_CHOICE(random);
      p3d_set_SAMPLING_CHOICE(sampling);
      p3d_set_MOTION_PLANNER(motion);
      ENV.setInt(Env::NbTry,nbTry);
      ENV.setBool(Env::biDir, biDirection);
      ENV.setInt(Env::maxNodeCompco,comp);
      if (!success){
        printf("Impossible to avoid collision\n");
        return FALSE;
      }
      //Fuse the generated graph with the main one
      p3d_graph* subGraph = XYZ_GRAPH;
      p3d_fuseGraphs(robot, mainGraph, subGraph);
      XYZ_GRAPH = mainGraph;
      robot->GRAPH = mainGraph;
      cur = tmpNext;
    }
    dist += cur->length_lp;
  }
  if(graphTrajInCollision || optimTrajInCollision){ //If it's the optimized traj in collision, we change the trajectory withe the graph traj.
    p3d_addStartAndGoalNodeToGraph(robot->ROBOT_POS, robot->ROBOT_GOTO, robot->ikSolPos, robot->ikSolGoto, robot->GRAPH, robot);
    if(p3d_graph_to_traj(robot)){
      g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
      checkForCollidingLpAlongPath();
    }else{
      printf("oula Mino il y'a soucis!!! \n");
      return FALSE;
    }
  }
  if(traj->isOptimized){
    optimiseTrajectory();
  }
  return TRUE;
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

void p3dAddTrajToGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj){
  p3d_node* initNode = NULL, *endNode = NULL;
  configPt qInit, qEnd;
  bool nodeAlreadyConnected = false;
  for(p3d_localpath* lp = traj->courbePt; lp; lp = lp->next_lp){
    qInit = lp->config_at_param(robot, lp, 0);
    qEnd = lp->config_at_param(robot, lp, lp->length_lp);
    nodeAlreadyConnected = false;
    initNode = NULL;
    endNode = NULL;
    initNode = p3d_TestConfInGraph(graph, qInit);
    if(!initNode){
      printf("QInit n'est pas dans le graph\n");//If qinit is not already in the graph, there is a problem !!!
      return;
    }
    endNode = p3d_TestConfInGraph(graph, qEnd);
    if(!endNode){
      endNode = p3d_APInode_make_multisol(graph, qEnd, lp->ikSol);
      p3d_insert_node(graph, endNode);
    }
    //connect qInit and qEnd if its not connected yet
    for(p3d_list_edge* lEdge = initNode->edges; lEdge; lEdge = lEdge->next){
      if(lEdge->E->Nf == endNode){
        nodeAlreadyConnected = true;
        break;
      }
    }
    if(!nodeAlreadyConnected){
      p3d_add_node_compco(endNode, initNode->comp, TRUE);
      double dist = p3d_dist_q1_q2_multisol(robot, qInit, qEnd, lp->ikSol);//take the distance between the two nodes
      p3d_create_edges(graph, initNode, endNode, dist);//create edges between the two nodes
    }
  }
}

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


// void p3d_computeCollisionTime(void){
//   p3d_rob * r = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
//   configPt q = p3d_alloc_config(r);
//   configPt q2 = p3d_alloc_config(r);
//   double tu = 0.0, ts = 0.0, nTu = 0.0;
//   p3d_localpath * localpathPt = NULL;
//   int ntest;
//
// //Shoot
//   ChronoOn();
//   ChronoTimes(&tu, &ts);
//   printf("Time before shoot = %f\n", tu);
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//   }
//   ChronoTimes(&tu, &ts);
//   printf("Time after shoot = %f\n", tu);
//   ChronoOff();
//
// //Collision Shoot
//   ChronoOn();
//   ChronoTimes(&nTu, &ts);
//   printf("Time before col = %f\n", nTu);
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//     p3d_col_test();
//   }
//   ChronoTimes(&nTu, &ts);
//   printf("Time after col = %f\n", nTu);
//   ChronoOff();
//   printf("Collision time = %f over 1000 = %f\n", (nTu - tu)/NB_SHOOTS, (nTu - tu)*1000/NB_SHOOTS);
// 
//   tu = nTu - tu;
// //localPath Creation
//   ChronoOn();
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//     p3d_col_test();
//     p3d_standard_shoot(r, q2,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q2);
//     p3d_get_robot_config_into(r, &q2);
//     p3d_col_test();
//     localpathPt = p3d_local_planner_multisol(r, q, q2, NULL);
//   }
//   ChronoTimes(&nTu, &ts);
//   printf("Time after lp Create = %f\n", nTu - 2*tu);
//   ChronoOff();
//   tu = nTu - 2*tu;
// 
// //localpath Col
//   ChronoOn();
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//     p3d_col_test();
//     p3d_standard_shoot(r, q2,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q2);
//     p3d_get_robot_config_into(r, &q2);
//     p3d_col_test();
//     localpathPt = p3d_local_planner_multisol(r, q, q2, NULL);
//     p3d_unvalid_localpath_test(r, localpathPt, &ntest);
//   }
//   ChronoTimes(&nTu, &ts);
//   printf("Time after lp Create = %f\n", nTu - tu);
//   ChronoOff();
//   printf("Collision time = %f over 1000 = %f\n", (nTu - tu)/NB_SHOOTS, (nTu - tu)*1000/NB_SHOOTS);
// }


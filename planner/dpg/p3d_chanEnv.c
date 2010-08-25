#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "p3d_chanEnv_proto.h"

static p3d_edge* p3d_getLpEdge(p3d_rob* robot, p3d_graph* graph, p3d_localpath* lp);


int checkCollisionsOnPathAndReplan(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, int optimized){
  if (!traj) {
    return 0;
  }
  p3d_rob* currentRobot = XYZ_ENV->cur_robot;
  p3d_sel_desc_num(P3D_ROBOT, robot->num);
  int ntest = 0;
  bool graphTrajInCollision = false;
  for (p3d_localpath *cur = traj->courbePt; cur != NULL && !graphTrajInCollision ; cur = cur->next_lp) {
    if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {//le lp est en collision
      ChronoOn();
      graphTrajInCollision = true;
      configPt startConf = cur->config_at_distance(robot, cur, 0), endConf = NULL;
      
      //find the last node (end of a localpath)
      p3d_localpath * tmpNext = cur;
      do {
        endConf = tmpNext->config_at_distance(robot, tmpNext, tmpNext->length_lp);
        p3d_set_and_update_this_robot_conf(robot, endConf);
        if (tmpNext->next_lp != NULL) {
          tmpNext = tmpNext->next_lp;
        }else{
          break;
        }
      }while (p3d_col_test());
      if(tmpNext){
        tmpNext = tmpNext->prev_lp;
      }
      
      //tagger les edges comme invalides.
      p3d_edge* edge = p3d_getLpEdge(robot, mainGraph, cur);
      p3d_unvalid_edge(mainGraph, edge);
      for(p3d_localpath* tmplp = cur->next_lp; tmplp != tmpNext->next_lp; tmplp = tmplp->next_lp){
        //if there is more than one edge, unvalidate the edges starting and ending at each node of the subpath
        for(p3d_list_edge* le = edge->Nf->edges; le ; le = le->next){
          p3d_unvalid_edge(mainGraph, le->E);
        }
        edge = p3d_getLpEdge(robot, mainGraph, tmplp);
        p3d_unvalid_edge(mainGraph, edge);
      }
      p3d_addStartAndGoalNodeToGraph(startConf, robot->ROBOT_GOTO, cur->ikSol, robot->ikSolGoto, mainGraph, robot);
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
        robot->ROBOT_POS = startConf;
        robot->ROBOT_GOTO = endConf;
        int success = p3d_specific_search((char*)"");
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
        if (!success){
          printf("Impossible to avoid collision\n");
          return -2;
        }
      }
      cur = tmpNext;
      PrintInfo(("Alternative Edge : "));
      ChronoPrint("");
      double tu = 0.0, ts = 0.0;
      ChronoTimes(&tu, &ts);
      ChronoOff();
    }
  }
  if(graphTrajInCollision){ //If it's the optimized traj in collision, we change the trajectory with the graph traj.
    p3d_addStartAndGoalNodeToGraph(robot->ROBOT_POS, robot->ROBOT_GOTO, robot->ikSolPos, robot->ikSolGoto, robot->GRAPH, robot);
    if(p3d_graph_to_traj(robot)){
      p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
      return -1;
    }else{
      printf("oula Mino il y'a soucis!!! \n");
      return -2;
    }
  }else{
    p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
    g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
    return 1;
  }
}

// Return Values : -2 = Error  -1 = a collision is avoided  0 = No traj given  1 = Traj is collision Free.
int replanForCollidingPath(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, configPt current, p3d_localpath* currentLp, int optimized){
  if (!traj || !currentLp) {
    return 0;
  }
  p3d_rob* currentRobot = XYZ_ENV->cur_robot;
  p3d_sel_desc_num(P3D_ROBOT, robot->num);
  int ntest = 0;
  double dist = 0;
  double trajLength =  p3d_compute_traj_length(traj);
  bool graphTrajInCollision = false;
  //start form the localpath containing the current config of the robot
  p3d_localpath *cur = currentLp;
  double curDist = 0;
  for(p3d_localpath* tmp = traj->courbePt; tmp && tmp != currentLp; tmp = tmp->next_lp){
    curDist += tmp->length_lp;
  }
  dist = curDist;
  for (; cur != NULL && !graphTrajInCollision && dist < curDist + 0.33 * trajLength ; cur = cur->next_lp) {
    if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {//le lp est en collision
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
        //Fuse the generated graph with the main one
//         p3d_graph* subGraph = XYZ_GRAPH;
//         p3d_fuseGraphs(robot, mainGraph, subGraph);
//         XYZ_GRAPH = mainGraph;
//         robot->GRAPH = mainGraph;
        if (!success){
          printf("Impossible to avoid collision\n");
          return -2;
        }
      }else{
      }
      cur = tmpNext;
      dist = tmpDist;
      PrintInfo(("Alternative Edge : "));
      ChronoPrint("");
      double tu = 0.0, ts = 0.0;
      ChronoTimes(&tu, &ts);
      ChronoOff();
    }else{
      dist += cur->length_lp;
    }
  }
  if(graphTrajInCollision){ //If it's the optimized traj in collision, we change the trajectory with the graph traj.
    p3d_addStartAndGoalNodeToGraph(current, robot->ROBOT_GOTO, currentLp->ikSol, robot->ikSolGoto, robot->GRAPH, robot);
    if(/*p3d_traj* newTraj = */p3d_graph_to_traj(robot)){
//       g3d_draw_allwin_active();
      p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
//       g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
      return -1;
    }else{
//       g3d_draw_allwin_active();
      printf("oula Mino il y'a soucis!!! \n");
      return -2;
    }
  }else{
    p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
    g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
//     g3d_draw_allwin_active();
//     if(optimized){
//       optimiseTrajectory(100,6);
//     }
    return 1;
  }
}

//return True if there is collision, False otherwise
int checkForCollidingPath(p3d_rob* robot, p3d_traj* traj, p3d_localpath* currentLp){
  if (!traj || !currentLp) {
    return 0;
  }
  p3d_rob* currentRobot = XYZ_ENV->cur_robot;
  p3d_sel_desc_num(P3D_ROBOT, robot->num);
  int ntest = 0;
  double dist = 0;
  double trajLength =  p3d_compute_traj_length(traj);
  //start form the localpath containing the current config of the robot
  p3d_localpath *cur = currentLp;
  double curDist = 0;
  for(p3d_localpath* tmp = traj->courbePt; tmp && tmp != currentLp; tmp = tmp->next_lp){
    curDist += tmp->length_lp;
  }
  dist = curDist;
  for (; cur != NULL && dist < curDist + 0.33 * trajLength ; cur = cur->next_lp) {
    if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {
       p3d_sel_desc_num(P3D_ROBOT, currentRobot->num);
       return true;//there is collision
    }
  }
  return false;//there is no collision
}

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



#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"
#include <stdarg.h>
#include <stdlib.h>

#define DEBUGMULTIGRAPH 0

static void p3d_checkMergeCollision(p3d_rob * r, p3d_node ** node, p3d_multiGraphJoint ** mgJoints, int mgNum, int mode, int nbConfigs, configPt *q);

//Definition of a local structure
typedef struct lpOrder{
  p3d_localpath * currentLp;
  double currentPathLen;
  int mgNum;
}lpOrder;

typedef struct mergedConfigs{
  configPt q;
  int nbMg;
  p3d_multiGraphJoint ** mgJoints;
  struct mergedConfigs * prev;
  struct mergedConfigs * next;
}mergedConfigs;

/**********************************************/
/********* Collisions And Joints **************/
/**********************************************/

/**
 * @brief Activate an independent part and desactive all the others. Only the collision between the selected part and the environment and the part and comunes parts are computed.
 * @param r The robot
 * @param mgNum The number of the independent part to activate.
 */
void p3d_setActiveDof(p3d_rob * r, int mgNum){
  if (mgNum >= 0 && mgNum <= r->mg->nbGraphs){//simple protection contre les erreurs de programation
    p3d_multiGraphJoint* mgJoints = r->mg->mgJoints[mgNum];

    for(int i = mgJoints->nbJoints - 1; i >= 0; i--){//Desactiver les contraintes pour les jnts utilisé et activer les collisions
      p3d_jnt * jnt = r->joints[mgJoints->joints[i]];
      if(jnt->type != P3D_BASE && jnt->type != P3D_FIXED && mgJoints->cntrts[i] != -1){
        p3d_cntrt * ct = r->cntrt_manager->cntrts[mgJoints->cntrts[i]];
        if(p3d_update_constraint(ct, 0)) {
          if (ct->enchained != NULL)
            p3d_unchain_cntrts(ct);
          p3d_update_jnts_state(r->cntrt_manager,ct, 0);
          p3d_col_activate_one_cntrt_pairs(ct);
        }
      }
      p3d_col_activate_obj_env(jnt->o);
    }
    p3d_autocol_activate_rob(r);

    //Activer les contraintes de tous les autres joints de la liste
    for(int i = 0; i < r->njoints + 1; i++){
      if(r->mg->usedJoint[i] == 1){
        int usedJoint = 0;
        for(int j = 0; j < mgJoints->nbJoints; j++){
          if(i == mgJoints->joints[j]){
            usedJoint = 1; //the joint is in the mgJoints
            break;
          }
        }
        if(usedJoint == 0){//Activer la contrainte du joint et desactiver les bodys
          p3d_jnt * jnt = r->joints[i];
          p3d_multiGraphJoint* tmpMgJnt = r->mg->mgJoints[p3d_jointInMultigraph(r, i)];
          p3d_cntrt * ct = NULL;
          for(int j = 0; j < tmpMgJnt->nbJoints; j++){
            if(tmpMgJnt->joints[j] == i && tmpMgJnt->cntrts[j] != -1){
              ct = r->cntrt_manager->cntrts[tmpMgJnt->cntrts[j]];
            }
          }
          if(ct){
//             ct->argu_d[0] = RTOD(jnt->v);
            if(p3d_update_constraint(ct, 1)) {
              if (ct->enchained != NULL)
                p3d_reenchain_cntrts(ct);
              p3d_col_deactivate_one_cntrt_pairs(ct);
            }
          }
          if(jnt->o){
            p3d_col_deactivate_obj_env(jnt->o);
            for(int j = 0; j < r->njoints + 1; j++){
              if((r->joints[j])->o && jnt->o->num != (r->joints[j])->o->num){//objets utilises et differents
                p3d_col_deactivate_obj_obj(jnt->o, (r->joints[j])->o);
              }
            }
          }
        }
      }
    }
    for(int i = 0; i < r->mg->nbGraphs; i++){
      if(i == mgNum){
        r->mg->active[i] = 1;
      }else{
        r->mg->active[i] = 0;
      }
    }
  }
}

/**
 * @brief Activate an indeperdent part against other active parts only.
 * @param r The robot
 * @param mgNum The number of the independent part to activate.
 */
void p3d_activateMgAutocol(p3d_rob * r, int mgNum){
  p3d_multiGraphJoint* mgJoints = r->mg->mgJoints[mgNum];

  for(int i = mgJoints->nbJoints - 1; i >= 0; i--){//Desactiver les contraintes pour les jnts utilisé et activer les collisions
    p3d_jnt * jnt = r->joints[mgJoints->joints[i]];
    if(jnt->type != P3D_BASE && jnt->type != P3D_FIXED && mgJoints->cntrts[i] != -1 && jnt->o != NULL){
      p3d_cntrt * ct = r->cntrt_manager->cntrts[mgJoints->cntrts[i]];
      if(p3d_update_constraint(ct, 0)) {
        if (ct->enchained != NULL)
          p3d_unchain_cntrts(ct);
          p3d_update_jnts_state(r->cntrt_manager,ct, 0);
          p3d_col_activate_one_cntrt_pairs(ct);
      }
// Activer la collision avec les autres parties
      for(int j = 0; j < r->mg->nbGraphs; j++){
        if(r->mg->active[j]){//activer les collisions avec ces joints
          for(int k = 0; k < (r->mg->mgJoints[j])->nbJoints; k++){
            p3d_multiGraphJoint * mgj = r->mg->mgJoints[j];
            if((r->joints[mgj->joints[k]])->o && jnt->o->num != (r->joints[mgj->joints[k]])->o->num){//objets utilises et differents
              if(p3d_isMarkedForautocol(r->num, jnt->o->num, (r->joints[mgj->joints[k]])->o->num) == 1){
                p3d_col_activate_obj_obj(jnt->o, (r->joints[mgj->joints[k]])->o);
              }
            }
          }
        }
      }
  // Activer les autoCollisions de cette meme partie
      for(int j = 0; j < i; j++){
        if(r->joints[mgJoints->joints[j]]->o && p3d_isMarkedForautocol(r->num, jnt->o->num, r->joints[mgJoints->joints[j]]->o->num) == 1){
          p3d_col_activate_obj_obj(jnt->o, r->joints[mgJoints->joints[j]]->o);
        }
      }
    }
  }
  r->mg->active[mgNum] = 1;
}

/**
 * @brief Disactivate an indeperdent part totaly.
 * @param r The robot
 * @param mgNum The number of the independent part to activate.
 */
void p3d_deactivateMgAutocol(p3d_rob * r, int mgNum){
  p3d_multiGraphJoint* mgJoints = r->mg->mgJoints[mgNum];

  for(int i = mgJoints->nbJoints - 1; i >= 0; i--){//Activer les contraintes pour les jnts utilisé et activer les collisions
    p3d_jnt * jnt = r->joints[mgJoints->joints[i]];
    if(jnt->type != P3D_BASE && jnt->type != P3D_FIXED && mgJoints->cntrts[i] != -1 && jnt->o != NULL){
      p3d_cntrt * ct = r->cntrt_manager->cntrts[mgJoints->cntrts[i]];
      if(ct){
//         ct->argu_d[0] = RTOD(jnt->v);
        if(p3d_update_constraint(ct, 1)) {
          if (ct->enchained != NULL)
            p3d_reenchain_cntrts(ct);
          p3d_col_deactivate_one_cntrt_pairs(ct);
        }
      }

      for(int j = 0; j < r->mg->nbGraphs; j++){
        if(r->mg->active[j]){//deactiver les collisions avec ces joints
          for(int k = 0; k < (r->mg->mgJoints[j])->nbJoints; k++){
            p3d_multiGraphJoint * mgj = r->mg->mgJoints[j];
            if((r->joints[mgj->joints[k]])->o && jnt->o->num != (r->joints[mgj->joints[k]])->o->num){//objets utilises et differents
              if(p3d_isMarkedForautocol(r->num, jnt->o->num, (r->joints[mgj->joints[k]])->o->num) == 1){
                p3d_col_deactivate_obj_obj(jnt->o, (r->joints[mgj->joints[k]])->o);
              }
            }
          }
        }
      }
      // Desactiver les autoCollisions de cette meme partie
      for(int j = 0; j < i; j++){
        if(r->joints[mgJoints->joints[j]]->o && p3d_isMarkedForautocol(r->num, jnt->o->num, r->joints[mgJoints->joints[j]]->o->num) == 1){
          p3d_col_deactivate_obj_obj(jnt->o, r->joints[mgJoints->joints[j]]->o);
        }
      }
    }
  }
  r->mg->active[mgNum] = 0;
}

/**
 * @brief Activate all robot collisions.
 * @param r The robot
 */
void p3d_setAllDofActive(p3d_rob * r){
  for(int j = 0; j < r->mg->nbGraphs; j++){
    p3d_multiGraphJoint* mgJoints = r->mg->mgJoints[j];
    for(int i = mgJoints->nbJoints - 1; i >= 0; i--){//Desactiver les contraintes pour les jnts utilisé et activer les collisions
      p3d_jnt * jnt = r->joints[mgJoints->joints[i]];
      if(jnt->type != P3D_BASE && jnt->type != P3D_FIXED && mgJoints->cntrts[i] != -1){
        p3d_cntrt * ct = r->cntrt_manager->cntrts[mgJoints->cntrts[i]];
        if(p3d_update_constraint(ct, 0)) {
          if (ct->enchained != NULL)
            p3d_unchain_cntrts(ct);
          p3d_update_jnts_state(r->cntrt_manager,ct, 0);
          p3d_col_activate_one_cntrt_pairs(ct);
        }
      }
      p3d_col_activate_obj_env(jnt->o);
    }
    r->mg->active[j] = 1;
  }
  p3d_init_rob_col_activ(r->name); //mettre la table des autocollision a 1
  p3d_desactivate_col_check_automatic(); //mettre a -1 les bodys adjacents
  p3d_autocol_activate_rob(r);//traitement des paires
}

/**
 * @brief Disactivate all robot collisions.
 * @param r The robot
 */
void p3d_setAllDofPassive(p3d_rob * r){
  for(int j = 0; j < r->mg->nbGraphs; j++){
    p3d_multiGraphJoint* mgJoints = r->mg->mgJoints[j];
    for(int i = mgJoints->nbJoints - 1; i >= 0; i--){//Activer les contraintes pour les jnts utilisé et desactiver les collisions
      p3d_jnt * jnt = r->joints[mgJoints->joints[i]];
      if(mgJoints->cntrts[i] != -1){
        p3d_cntrt * ct = r->cntrt_manager->cntrts[mgJoints->cntrts[i]];
        if(ct){
          ct->argu_d[0] = RTOD(jnt->dof_data[0].v);
          if(p3d_update_constraint(ct, 1)) {
            if (ct->enchained != NULL)
              p3d_reenchain_cntrts(ct);
            p3d_col_deactivate_one_cntrt_pairs(ct);
          }
        }
      }
      p3d_col_deactivate_obj_env(jnt->o);
    }
    r->mg->active[j] = 0;
  }
  p3d_col_deactivate_rob(r);
//   p3d_desactivate_col_check_all();//mettre a -1 tous les bodys
//   p3d_autocol_activate_rob(r); //traitement des paires
}

/**
 * @brief Select a part randomly from the given robot and active its parts (Désactivate all other parts)
 * @param r The robot
 * @param random The selected part num is returned
 * @return The selected graph
 */
p3d_graph * p3d_setRandomMultiGraphAndActiveDof(p3d_rob * r, int * random){
  *random = (int)p3d_random(0,r->mg->nbGraphs);

  if(DEBUGMULTIGRAPH){
    printf("Selected MG = %d\n", *random);
  }
  if(!r->mg->graphs[*random]){
    r->GRAPH = NULL;
    XYZ_GRAPH = NULL;
    r->mg->graphs[*random] = p3d_create_graph();
  }
  XYZ_GRAPH = r->mg->graphs[*random];
  r->GRAPH = r->mg->graphs[*random];
  p3d_setActiveDof(r,*random);
  return r->mg->graphs[*random];
}

/**
 * @brief Select a part from the given robot and active its parts (Désactivate all other parts)
 * @param r The robot
 * @param mgNum The part num to activate
 * @return The selected graph
 */
p3d_graph * p3d_setMultiGraphAndActiveDof(p3d_rob * r, int mgNum){
  if(DEBUGMULTIGRAPH){
    printf("Selected MG = %d\n", mgNum);
  }
  if(!r->mg->graphs[mgNum]){
    r->GRAPH = NULL;
    XYZ_GRAPH = NULL;
    r->mg->graphs[mgNum] = p3d_create_graph();
  }
  XYZ_GRAPH = r->mg->graphs[mgNum];
  r->GRAPH = r->mg->graphs[mgNum];
  p3d_setActiveDof(r,mgNum);
  return r->mg->graphs[mgNum];
}

/**
 * @brief Check if the given joint ID is used into the multiGraph part.
 * @param r The robot
 * @param jointId The joint Id
 * @return The part number or -1
 */
int p3d_jointInMultigraph(p3d_rob * r, int jointId){
  if(r && jointId != 0){//simple petite protection
    if(r->mg->usedJoint[jointId] == 1){
      for(int i = 0; i < r->mg->nbGraphs; i++){
        p3d_multiGraphJoint* mgJoints = r->mg->mgJoints[i];
        for(int j = 0; j < mgJoints->nbJoints; j++){
          if(mgJoints->joints[j] == jointId){
            return i;
          }
        }
      }
    }
  }
  return -1;
}

static int SelectedMgNum = -1;
int p3d_getSelectedMgNum(){
  return SelectedMgNum;
}
void p3d_setSelectedMgNum(int mgNum){
  SelectedMgNum = mgNum;
}

/**********************************************/
/******** FlatSuperGraph construction *********/
/**********************************************/

/**
 * @brief Fill the Super graph. Create it first before filling it.
 * @param r 
 * @param mode 
 */
void p3d_flatMultiGraph(p3d_rob * r, int mode){
  if (r->mg->fsg == NULL){
    p3d_createRobotFlatSuperGraph(r); //Alloc and graph init
  }
  switch(mode){
    case 0:{
      p3d_fillFlatMultiGraph(r, NULL, NULL, 0, 0);
      break;
    }
    case 1:{
      p3d_fillFlatMultiGraph(r, NULL, NULL, 0, 1);
      break;
    }
  }
}

//mode : 0 merge all, 1 merge traj only, 2 merge only non merged
int p3d_fillFlatMultiGraph(p3d_rob * r, p3d_node ** node,  p3d_multiGraphJoint ** mgJoints, int mgNum, int mode){
  p3d_list_node * ln = NULL;
  p3d_node * n = NULL;
  static int depth = 0;
  p3d_graph * g = NULL;

  if(!node){//si node == NULL => premiere entree dans la fonction
    depth = 0;
    if(!r->mg->involvesCp){//if there is no common part or is fixed
      p3d_setAllDofPassive(r);//desactivation de tous les mg
    }else{
      p3d_setAllDofActive(r);
    }
    node = MY_ALLOC(p3d_node*, r->mg->nbGraphs);
    mgJoints = MY_ALLOC(p3d_multiGraphJoint*, r->mg->nbGraphs);
    for(int i = 0; i < r->mg->nbGraphs; i++){//initilisation des tableaux
      node[i] = NULL;
      mgJoints[i] = NULL;
    }
  }

  depth++;
  if(mgNum == r->mg->nbGraphs){
    mgNum = 0;
  }
  if(depth > r->mg->nbGraphs){
    depth--;
    return 0;
  }
  if(!r->mg->involvesCp){//if there is no common part or is fixed
    p3d_activateMgAutocol(r,mgNum);//activation des joints du MG
  }
  g = r->mg->graphs[mgNum];

  mgJoints[mgNum] = r->mg->mgJoints[mgNum];
  switch(mode){
    case 0: {//merge all
      ln = g->nodes;
      for(; ln; ln = ln->next){
        node[mgNum] = ln->N;
        if(!p3d_fillFlatMultiGraph(r, node, mgJoints, mgNum + 1, mode)){//si fill retourne 0 test de collision du super noeud et connection
          int nbConfigs = 0;
          configPt *q = p3d_mergeMultiGraphNodes(r, depth, node, mgJoints, &nbConfigs);//creation des configurations du robot
          p3d_checkMergeCollision(r, node, mgJoints, mgNum, mode, nbConfigs, q);//collision des super noeuds et leur connexion
        }
      }
      break;
    }
    case 1: {//merge traj only
      n = g->search_start;
      for(; n; n = n->search_to){
        node[mgNum] = n;
        if(!p3d_fillFlatMultiGraph(r, node, mgJoints, mgNum + 1, mode)){//si fill retourne 0 test de collision du super noeud et connection
          int nbConfigs = 0;
          configPt *q = p3d_mergeMultiGraphNodes(r, depth, node, mgJoints, &nbConfigs);//creation des configurations du robot
          p3d_checkMergeCollision(r, node, mgJoints, mgNum, mode, nbConfigs, q);//collision des super noeuds et leur connexion
        }
      }
      break;
    }
    case 2: {//merge only non merged
      ln = g->nodes;
      for(; ln; ln = ln->next){
        if(depth == 1 && ln->N->mergeState != 0){
          //continue
        }else{
          node[mgNum] = ln->N;
          if(!p3d_fillFlatMultiGraph(r, node, mgJoints, mgNum + 1, mode)){//si fill retourne 0 test de collision du super noeud et connection
            int nbConfigs = 0;
            configPt *q = p3d_mergeMultiGraphNodes(r, depth, node, mgJoints, &nbConfigs);//creation des configurations du robot
            p3d_checkMergeCollision(r, node, mgJoints, mgNum, mode, nbConfigs, q);//collision des super noeuds et leur connexion
          }
        }
      }
      break;
    }
    default:{
      printf("Bad merge mode selected !!! \n");
      break;
    }
  }
  if(depth > 1){
    node[mgNum] = NULL;
    if(!r->mg->involvesCp){//if there is no common part or is fixed
      p3d_deactivateMgAutocol(r,mgNum);//desactivation des joints du MG
    }
    depth --;
  }else{
    if(mode == 0){//tag all p3d_nodes as merged
      for(int i = 0; i < r->mg->nbGraphs; i++){
        g = r->mg->graphs[i];
        for(ln = g->nodes; ln; ln = ln->next){
          ln->N->mergeState = 1;
        }
      }
    }
    if(!r->mg->involvesCp){//if there is no common part or is fixed
      p3d_setAllDofActive(r);//reactivation de toutes les parties
    }
  }
  return 1;
}

static void p3d_checkMergeCollision(p3d_rob * r, p3d_node ** node, p3d_multiGraphJoint ** mgJoints, int mgNum, int mode, int nbConfigs, configPt *q){
  for(int i = 0; i < nbConfigs; i++){
    p3d_set_and_update_robot_conf_multisol(q[i], NULL);//set robot Pos
    if(DEBUGMULTIGRAPH){
      g3d_refresh_allwin_active();
    }
    if(!p3d_col_test()){
      //collision check
      if(DEBUGMULTIGRAPH){
        printf("collision free\n");
      }
      //ajout du noeud dans le graph
      p3d_flatSuperGraphNode * fsgNode = p3d_createFlatSuperGraphNode (r,r->mg->fsg, node, q[i]);
      if(fsgNode){
        p3d_addFsgNodeInGraph(r->mg->fsg, fsgNode);
        //connection avec les autres noeuds de la liste
        p3d_testFsgNodesConnection(r, r->mg->fsg, fsgNode);
      }
    }
  }
}

//TODO
void p3d_testFsgNodesConnection(p3d_rob *r, p3d_flatSuperGraph *fsg, p3d_flatSuperGraphNode * node){
  int goodFsgNode = TRUE;
  p3d_fsgListNode * list = fsg->nodes;
//   static int nbConnexions = 0;
  while(list){//pour chaque noeud du super graphe
    if(list->node != node){
      goodFsgNode = TRUE;
      // regarder si ses noeuds sont adjacents a node (Filtre 1)
      for(int i = 0; i < r->mg->nbGraphs && goodFsgNode; i++){//pour chaque Mutligraph (chaque indice du tableau des nodes composant le MGNode)
        p3d_list_node * lnode = node->nodes[i]->neighb;
        goodFsgNode = FALSE;
        while(lnode && !goodFsgNode){//pour chaque voisin de node
          if(lnode->N == list->node->nodes[i] || node->nodes[i] == list->node->nodes[i]){//voir si les deux noeuds sont adjacents ou egaux
            goodFsgNode = TRUE;//si oui passer regarder les autres niveaux
            break;
          }
          goodFsgNode = FALSE;
          lnode = lnode->next;
        }
      }
      //Filtre 2
      if(goodFsgNode){//test de connexion entre les deux super noeuds
        fsg->search_start = node;
        fsg->search_goal = list->node;
        if(p3d_graph_search(fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH) == FALSE){
          //si il n'y a pas de chemin entre les deux deja existant connecter.
          //creation de deux noeuds temporaires
          p3d_node* N1 = p3d_create_node(NULL);
          N1->q = p3d_copy_config(r, list->node->q);
          p3d_node* N2 = p3d_create_node(NULL);
          N2->q =  p3d_copy_config(r, node->q);
          double dist = P3D_HUGE;
          if(p3d_APInode_linked_multisol(NULL, N1, N2, &dist)){//noeuds connectables
            p3d_connectFsgNodes(fsg, node, list->node, dist);
          }else{
            for(int i = 0; i < r->mg->nbGraphs; i++){//ne tagger que les noeuds de la trajectoire
              p3d_graph * graph = r->mg->graphs[i];
              p3d_node * tmpNode = graph->search_start;
              int k = 0;
              for(;tmpNode; tmpNode = tmpNode->search_to){
                if (tmpNode == list->node->nodes[i]){
                  list->node->nodes[i]->needMgCycle = TRUE;
                  k++;
                }
                if (tmpNode == node->nodes[i]){
                  node->nodes[i]->needMgCycle = TRUE;
                  k++;
                }
              }
            }
            fsg->autoColNodes.push_back(p3d_createFlatSuperGraphEdge(node, list->node, -1));
          }
          p3d_APInode_desalloc(NULL, N1);
          p3d_APInode_desalloc(NULL, N2);
        }else{
          if(DEBUGMULTIGRAPH){
            printf("J'ai trouvé un chemin entre %d et %d\n", node->num, list->node->num);
          }
        }
      }
    }
    list = list->next;
  }
}

void p3d_extractMultigraphStartGoal(p3d_rob * r, configPt qs, configPt qg, configPt * q_s, configPt * q_g){
  if(r->mg->nbGraphs != 0){
    for(int i = 0; i < r->mg->nbGraphs; i++){
    q_s[i] = MY_ALLOC(double, (size_t)r->nb_dof);
    q_g[i] = MY_ALLOC(double, (size_t)r->nb_dof);
      for(int j = 0; j < r->nb_dof; j++){
        q_s[i][j] = 0.0;
        q_g[i][j] = 0.0;
      }
    }
    for(int i = 0; i < r->njoints + 1; i++){
      p3d_jnt * jnt = r->joints[i];
      if(r->mg->usedJoint[i] == 1){//used joint check insert it in the right configPt
        int mgId = p3d_jointInMultigraph(r, r->joints[i]->num);
        for(int j = 0; j < jnt->dof_equiv_nbr; j++){
          q_s[mgId][jnt->index_dof+j] = qs[jnt->index_dof+j];
          q_g[mgId][jnt->index_dof+j] = qg[jnt->index_dof+j];
        }
      }else{//insert the jont value in all the configs
        for(int j = 0; j < jnt->dof_equiv_nbr; j++){
          for(int k = 0; k < r->mg->nbGraphs; k++){
            q_s[k][jnt->index_dof+j] = qs[jnt->index_dof+j];
            q_g[k][jnt->index_dof+j] = qg[jnt->index_dof+j];
          }
        }
      }
    }
  }else{
    q_s[0] = p3d_copy_config(r, qs);
    q_g[0] = p3d_copy_config(r, qg);
  }
}

/**********************************************/
/*************** Path Managment ***************/
/**********************************************/

int compareLpLength (const void* a, const void* b){
  lpOrder * lpoa = (lpOrder *)a, * lpob = (lpOrder *)b;
  return lpoa->currentPathLen > lpob->currentPathLen ? 1 : (lpoa->currentPathLen < lpob->currentPathLen ? -1 : 0);
}

p3d_traj * p3d_extractAndMergeMultiGraphPaths(p3d_rob * r){
  p3d_traj ** paths = NULL;
  int nTotLp = 0, *lpMultiGraphId = NULL;
  p3d_localpath ** lpMultiGraph = NULL, *lastLp = NULL, *newLp = NULL;
  lpOrder ** mglp = NULL;
  char str[250], sti[250];
  configPt start, end, *mergeConfigs;

  if(r && r->mg && r->mg->nbGraphs > 1){//simple petite protection
    paths = MY_ALLOC(p3d_traj*, r->mg->nbGraphs);
    for(int i = 0; i < r->mg->nbGraphs; i++){//Compute the paths and the total number of localpaths
      p3d_setMultiGraphAndActiveDof(r, i);
      if ((paths[i] = p3d_graph_to_traj(r))) {
        nTotLp = paths[i]->nlp;
      }else{
        MY_FREE(paths, p3d_traj*, r->mg->nbGraphs);
        return NULL;
      }
    }
    lpMultiGraph = MY_ALLOC(p3d_localpath*, nTotLp);
    lpMultiGraphId = MY_ALLOC(int, nTotLp);
    for(int i = 0; i < nTotLp; i++){
      lpMultiGraph[i] = NULL;
      lpMultiGraphId[i] = 0;
    }
    mglp = MY_ALLOC(lpOrder*, r->mg->nbGraphs);
    for(int i = 0; i < r->mg->nbGraphs; i++){
      mglp[i] = MY_ALLOC(lpOrder, 1);
      mglp[i]->currentLp = paths[i]->courbePt;
      mglp[i]->currentPathLen = mglp[i]->currentLp->length_lp;
      mglp[i]->mgNum = i;
    }
    //path Creation
    strcpy(str, "globtrj.");
    sprintf(sti, "%s", r->name);
    strcat(str, sti);
    sprintf(sti, "%s", ".");
    strcat(str, sti);
    sprintf(sti, "%d", r->nt);
    strcat(str, sti);
    p3d_beg_desc(P3D_TRAJ, str);
    lastLp = r->tcur->courbePt;
    for(int i = 0; i < nTotLp; i++){//while all lp paths are not checked

      qsort(mglp, r->mg->nbGraphs, sizeof(lpOrder*), compareLpLength);//trier par ordre croissant de longeure
      while(lastLp && lastLp->next_lp){//aller au dernier Localpath
        lastLp = lastLp->next_lp;
      }
      if(lastLp){//créeation du nouveau LP en prenant en compte l'ancien
        mergeConfigs = MY_ALLOC(configPt, r->mg->nbGraphs);
        start = lastLp->config_at_param(r,lastLp, lastLp->length_lp);
        for(int j = 0; j < r->mg->nbGraphs; j++){
          if(j == mglp[0]->mgNum){
            mergeConfigs[j] = mglp[0]->currentLp->config_at_param(r,mglp[0]->currentLp, 0);
          }else{
            mergeConfigs[j] = start;
          }
        }
        start = p3d_mergeMultiGraphConfig(r,r->mg->nbGraphs, mergeConfigs, r->mg->mgJoints);
        end = mglp[0]->currentLp->config_at_param(r,mglp[0]->currentLp, mglp[0]->currentLp->length_lp);
        for(int j = 0; j < r->mg->nbGraphs; j++){
          if(j == mglp[0]->mgNum){
            mergeConfigs[j] = mglp[0]->currentLp->config_at_param(r,mglp[0]->currentLp, 0);
          }else{
            mergeConfigs[j] = end;
          }
        }
        end = p3d_mergeMultiGraphConfig(r,r->mg->nbGraphs, mergeConfigs, r->mg->mgJoints);
        newLp = p3d_local_planner_multisol(r, start, end, NULL);
        if(!newLp){
          printf("Un lp généré est null\n");
        }
        MY_FREE(mergeConfigs, configPt, r->mg->nbGraphs);
      }else{
        newLp = mglp[0]->currentLp;
      }
      p3d_add_desc_courbe(newLp);
    }
    p3d_end_desc();
  }
  MY_FREE(paths, p3d_traj*, r->mg->nbGraphs);
  for(int i = 0; i < r->mg->nbGraphs; i++){
    MY_FREE(mglp[i],lpOrder, 1);
  }
  MY_FREE(mglp, lpOrder*, r->mg->nbGraphs);
  MY_FREE(lpMultiGraph, p3d_localpath*, nTotLp);
  MY_FREE(lpMultiGraphId, int, nTotLp);
  return r->tcur;// a changer
}

/**********************************************/
/********** Nodes/configs Managment ***********/
/**********************************************/

configPt p3d_mergeMultiGraphConfig(p3d_rob *r, int nConfig, configPt *configs, p3d_multiGraphJoint ** mgJoints){
   configPt q = NULL;
   if(nConfig > 1 && configs && mgJoints){//simple protection
       q = p3d_copy_config(r, configs[0]);
       for(int i = 1; i < nConfig; i++){//pour tous les noeuds de la liste
           if(configs[i] != NULL) {
               for(int j = 0; j < (mgJoints[i])->nbJoints; j++){//pour tous les joints correspondants au noeud
                   p3d_jnt* joint = r->joints[(mgJoints[i])->joints[j]];
                   for(int k = 0; k < joint->dof_equiv_nbr; k++){//pour tous les Dof du joint
                       q[joint->index_dof+k] = configs[i][joint->index_dof+k];
                   }
               }
           }
       }
       return q;
   }
   return NULL;
}

configPt* p3d_mergeMultiGraphNodes(p3d_rob *r, int nNodes, p3d_node ** nodes, p3d_multiGraphJoint ** mgJoints, int* nbConfigs){
  if(nNodes > 1 && nodes && mgJoints){
    *nbConfigs = 0;
    //commencer par créer une liste de mergedConfigs
    mergedConfigs * rootMC = NULL;
    mergedConfigs * tmpMC = NULL;
    for(int i = 0; i < nNodes; i++){//creation de la liste
      mergedConfigs * mC = MY_ALLOC(mergedConfigs, 1);
      (*nbConfigs)++;
      mC->nbMg = 1;
      mC->mgJoints = MY_ALLOC(p3d_multiGraphJoint *, mC->nbMg);
      mC->q = p3d_copy_config(r, nodes[i]->q);
      mC->mgJoints[0] = mgJoints[i];
      if(!rootMC){
        rootMC = mC;
        mC->prev = NULL;
      }else{
        mC->prev = tmpMC;
        tmpMC->next = mC;
      }
      mC->next = NULL;
      tmpMC = mC;
    }
    //merge de tous les tronc identiques
    tmpMC = rootMC;
    int differentTrunk = FALSE;
    while(tmpMC){//merge de tous les tronc identiques
      mergedConfigs * tmpMC2 = tmpMC->next;
      while(tmpMC2){
        differentTrunk = FALSE;
        for(int i = 0; !differentTrunk && i < r->njoints + 1; i++){
          if(r->mg->usedJoint[i] == 0){
            p3d_jnt* joint = r->joints[i];
            for(int j = 0; !differentTrunk && j < joint->dof_equiv_nbr; j++){ //pour tous les Dof du joint
              if(tmpMC->q[joint->index_dof+j] != tmpMC2->q[joint->index_dof+j]){
                differentTrunk = TRUE;
              }
            }
          }
        }
        if(!differentTrunk){//Si les troncs sont identiques
          //merge des deux config dans tmpMC
          configPt q = p3d_mergeTwoMultiGraphConfigs(r, tmpMC->q, tmpMC2->q, tmpMC->q, tmpMC->nbMg, tmpMC->mgJoints, tmpMC2->nbMg, tmpMC2->mgJoints);
          if(DEBUGMULTIGRAPH){
            print_config(r, q);
          }
          p3d_destroy_config(r,tmpMC->q);
          tmpMC->q = q;
          p3d_multiGraphJoint ** mgJoints = MY_ALLOC(p3d_multiGraphJoint *, tmpMC->nbMg + tmpMC2->nbMg);
          for(int i = 0; i < tmpMC->nbMg ; i++){
            mgJoints[i] = tmpMC->mgJoints[i];
          }
          for(int i = 0; i < tmpMC2->nbMg ; i++){
            mgJoints[tmpMC->nbMg + i] = tmpMC2->mgJoints[i];
          }
          MY_FREE(tmpMC->mgJoints, p3d_multiGraphJoint *, tmpMC->nbMg);
          tmpMC->mgJoints = mgJoints;
          tmpMC->nbMg += tmpMC2->nbMg;
          //destruction de tmpMC2
          mergedConfigs * tmp = tmpMC2;
          tmpMC2 = tmpMC2->next;
          (*nbConfigs)--;
          MY_FREE(tmp->mgJoints, p3d_multiGraphJoint *, tmp->nbMg);
          tmp->nbMg = 0;
          p3d_destroy_config(r,tmp->q);
          if(tmp->prev){
            tmp->prev->next = tmp->next;
            tmp->prev = NULL;
          }
          if(tmp->next){
            tmp->next->prev = tmp->prev;
            tmp->next = NULL;
          }
          MY_FREE(tmp, mergedConfigs, 1);
        }else{
          tmpMC2 = tmpMC2->next;
        }
      }
      tmpMC = tmpMC->next;
    }
    //merge de tous les configs qui restent
    tmpMC = rootMC;
    configPt * q = MY_ALLOC(configPt, (*nbConfigs));
    for(int i = 0; i < (*nbConfigs); i++){
      q[i] = NULL;
    }
    int i = 0;

    if((*nbConfigs) == 1){
      q[0] = rootMC->q;
    }else{
      if(DEBUGMULTIGRAPH){
//          printf("merge de tous les configs qui restent\n");
      }
      while(tmpMC){//merge de tous troncs restants
        if(DEBUGMULTIGRAPH){
//           printf("tmpMC\n");
        }
        mergedConfigs * tmpMC2 = rootMC;
        while(tmpMC2){
          if(DEBUGMULTIGRAPH){
//             printf("tmpMC2\n");
          }
          if(tmpMC2 != tmpMC){
            if(DEBUGMULTIGRAPH){
//               printf("Merge\n");
            }
            q[i] = p3d_mergeTwoMultiGraphConfigs(r, tmpMC->q, tmpMC2->q, tmpMC->q, tmpMC->nbMg, tmpMC->mgJoints, tmpMC2->nbMg, tmpMC2->mgJoints);
            i++;
          }
          tmpMC2 = tmpMC2->next;
        }
        tmpMC = tmpMC->next;
      }
    }
    return q;
  }
  return NULL;
}

configPt p3d_mergeTwoMultiGraphConfigs(p3d_rob *r, configPt q1, configPt q2, configPt trunk, int nbMg1, p3d_multiGraphJoint **mgj1, int nbMg2, p3d_multiGraphJoint **mgj2){
  if(r && mgj1 && mgj2){
    configPt q = p3d_copy_config(r, trunk);
    //premiere config
    for(int i = 0; i < nbMg1; i++){
      for(int j = 0; j < mgj1[i]->nbJoints; j++){//pour tous les joints correspondants
        p3d_jnt* joint = r->joints[mgj1[i]->joints[j]];
        for(int k = 0; k < joint->dof_equiv_nbr; k++){//pour tous les Dof du joint
          q[joint->index_dof+k] = q1[joint->index_dof+k];
        }
      }
    }
    //seconde config
    for(int i = 0; i < nbMg2; i++){
      for(int j = 0; j < mgj2[i]->nbJoints; j++){//pour tous les joints correspondants
        p3d_jnt* joint = r->joints[mgj2[i]->joints[j]];
        for(int k = 0; k < joint->dof_equiv_nbr; k++){//pour tous les Dof du joint
          q[joint->index_dof+k] = q2[joint->index_dof+k];
        }
      }
    }
    return q;
  }
  return NULL;
}

p3d_flatSuperGraphNode * p3d_isConfigInSuperGraph(p3d_rob * robot, p3d_flatSuperGraph *fsg, configPt q){
  if(fsg){
    p3d_fsgListNode * list = fsg->nodes;
    for(; list; list = list->next){
      if(p3d_equal_config(robot, list->node->q , q)){
        return list->node;
      }
    }
  }
  return NULL;
}
/**********************************************/
/********** FlatSuperGraph Managment **********/
/**********************************************/

int p3d_doIncrementalConstruction(int state){
  static int INCR = FALSE;

  if(state != -1){
    INCR = state;
  }
  return INCR;
}

void p3d_addFsgNodeInGraph(p3d_flatSuperGraph *fsg, p3d_flatSuperGraphNode * node){
  p3d_fsgListNode * listNode = p3d_createFsgListNode(node);
  p3d_pushFsgListNode(&(fsg->nodes), listNode);
  fsg->nNodes++;
}

void p3d_connectFsgNodes(p3d_flatSuperGraph *fsg, p3d_flatSuperGraphNode * n1, p3d_flatSuperGraphNode * n2, double dist){
  p3d_flatSuperGraphEdge * fsgEdge = p3d_createFlatSuperGraphEdge(n1, n2, dist);
  p3d_fsgListEdge * listEdgeFsg = p3d_createFsgListEdge(fsgEdge);
  p3d_flatSuperGraphEdge * fsgEdge1 = p3d_createFlatSuperGraphEdge(n1, n2, dist);
  p3d_fsgListEdge * listEdgeN1 = p3d_createFsgListEdge(fsgEdge1);
  p3d_flatSuperGraphEdge * fsgEdge2 = p3d_createFlatSuperGraphEdge(n2, n1, dist);
  p3d_fsgListEdge * listEdgeN2 = p3d_createFsgListEdge(fsgEdge2);

  p3d_pushFsgListEdge(&(fsg->edges), listEdgeFsg);
  fsg->nEdges++;
  p3d_pushFsgListEdge(&(n1->fsgEdges), listEdgeN1);
  n1->nEdges++;
  p3d_pushFsgListEdge(&(n2->fsgEdges), listEdgeN2);
  n2->nEdges++;
}

void p3d_resetMultiGraph(p3d_rob * r){
  p3d_delFlatSuperGraph(r, r->mg->fsg);
  r->mg->fsg = NULL;
  for(int i = 0; r &&  i < r->mg->nbGraphs; i++){
    p3d_del_graph(r->mg->graphs[i]);
    r->mg->graphs[i] = NULL;
  }
}

void p3d_convertFsgToGraph(p3d_graph * graph, p3d_flatSuperGraph *fsg){
  double dist = 0.0;
  if(fsg == NULL){
    printf("Error in SuperGraph conversion the SuperGraph is NULL\n");
    return;
  }
  p3d_fsgListNode * listNode = fsg->nodes;
  p3d_fsgListEdge * listEdge = fsg->edges;
  while(listNode){
    p3d_node * node = p3d_APInode_make_multisol(graph, p3d_copy_config(graph->rob, listNode->node->q), NULL);
    p3d_insert_node(graph, node);
    listNode->node->mergedNode = node;
//    if(!listNode->prev){
    p3d_create_compco(graph, node);
//    }else{
//      p3d_add_node_compco(node,graph->comp, TRUE);
//    }
    listNode = listNode->next;
  }
  while(listEdge){
    dist = p3d_dist_q1_q2_multisol(graph->rob, listEdge->edge->node1->q, listEdge->edge->node2->q, NULL);//take the distance between the two nodes
    p3d_create_edges(graph, listEdge->edge->node1->mergedNode, listEdge->edge->node2->mergedNode, dist);//create edge between the two nodes
    //merge the compcos
    if (listEdge->edge->node1->mergedNode->numcomp < listEdge->edge->node2->mergedNode->numcomp) {
      p3d_merge_comp(graph, listEdge->edge->node1->mergedNode->comp, &(listEdge->edge->node2->mergedNode->comp));// merge the two compco
    } else if (listEdge->edge->node1->mergedNode->numcomp > listEdge->edge->node2->mergedNode->numcomp) {
      p3d_merge_comp(graph, listEdge->edge->node2->mergedNode->comp, &(listEdge->edge->node1->mergedNode->comp));// merge the two compco
    }
    listEdge = listEdge->next;
  }
}

int p3d_isThereEdgeForNodesInFSG(p3d_flatSuperGraph * fsg, p3d_flatSuperGraphNode * n1, p3d_flatSuperGraphNode * n2){
  if(fsg && n1 && n2){
      p3d_fsgListEdge * listEdge = fsg->edges;
  	for(; listEdge; listEdge = listEdge->next){
    	if((listEdge->edge->node1 == n1 && listEdge->edge->node2 == n2) || (listEdge->edge->node1 == n2 && listEdge->edge->node2 == n1)){
      	return TRUE;
      }
    }
  }
  return FALSE;
}

int p3d_isNodesMarkedForCycle(p3d_flatSuperGraph * fsg, p3d_node* node1, p3d_node* node2, int mgNum){
  for(unsigned int i = 0; i < fsg->autoColNodes.size(); i++){
    if(fsg->autoColNodes[i]->node1->nodes[mgNum] == node1){
      if(fsg->autoColNodes[i]->node2->nodes[mgNum] == node2){
        return i;
      }
    }
    if(fsg->autoColNodes[i]->node2->nodes[mgNum] == node1){
      if(fsg->autoColNodes[i]->node1->nodes[mgNum] == node2){
        return i;
      }
    }
  }
  return -1;
}

// int p3d_isValidMgCycle(p3d_rob *r, p3d_node * node, p3d_edge * edge){
//   if(r && node && edge){//simple petite protection de code
//     p3d_flatSuperGraph * fsg = MY_ALLOC(p3d_flatSuperGraph, 1);
//     p3d_initFlatSuperGraph(fsg);
//     
//     p3d_delFlatSuperGraph(r, fsg);
//     MY_FREE(fsg, p3d_flatSuperGraph, 1);
//   }else{
//     printf("The robot, the node or the edge are NULL in p3d_isValidMgCycle\n");
//   }
// }

/**********************************************/
/******* FlatSuperGraph Create and Init *******/
/**********************************************/

void p3d_createRobotFlatSuperGraph (p3d_rob *r){
  if (r->mg->fsg == NULL){
    r->mg->fsg = new p3d_flatSuperGraph;
    p3d_initFlatSuperGraph(r->mg->fsg);
  }
}

void p3d_initFlatSuperGraph (p3d_flatSuperGraph *fsg){
  fsg->nNodes = 0;
  fsg->nodes = NULL;
  fsg->nEdges = 0;
  fsg->edges = NULL;
  fsg->search_start = NULL;
  fsg->search_goal = NULL;
  fsg->search_done = FALSE;
  fsg->search_numdev = 0;
  fsg->search_path = FALSE;
  fsg->search_goal_sol = NULL;
  fsg->search_cost = 0.0;
}

p3d_flatSuperGraphNode * p3d_createFlatSuperGraphNode (p3d_rob *r, p3d_flatSuperGraph *fsg, p3d_node ** nodes, configPt q){
  p3d_flatSuperGraphNode * fsgNode = MY_ALLOC(p3d_flatSuperGraphNode, 1);
  fsgNode->num = fsg->nNodes;
  fsgNode->nNodes = r->mg->nbGraphs;
  if(q != NULL){
    fsgNode->q = q;
  }
  fsgNode->mergedNode = NULL;
  fsgNode->nodes = MY_ALLOC(p3d_node*, fsgNode->nNodes);
  for(int i = 0; i < fsgNode->nNodes; i++){
    fsgNode->nodes[i] = nodes[i];
  }

  fsgNode->nEdges = 0;
  fsgNode->fsgEdges = NULL;
  fsgNode->h = 0;
  fsgNode->g = 0;
  fsgNode->f = 0;
  fsgNode->opened = 0;
  fsgNode->closed = 0;
  fsgNode->search_from = NULL;
  fsgNode->search_to = NULL;
  fsgNode->edge_from = NULL;
  fsgNode->orderCostListEdge = NULL;
  return fsgNode;
}

p3d_flatSuperGraphEdge * p3d_createFlatSuperGraphEdge(p3d_flatSuperGraphNode * fsgN1, p3d_flatSuperGraphNode * fsgN2, double dist){
  if(fsgN1 && fsgN2){
    p3d_flatSuperGraphEdge * fsgEdge = MY_ALLOC(p3d_flatSuperGraphEdge, 1);
    fsgEdge->node1 = fsgN1;
    fsgEdge->node2 = fsgN2;
    fsgEdge->cost = dist;
    return fsgEdge;
  }else{
    return NULL;
  }
}

p3d_fsgListNode * p3d_createFsgListNode(p3d_flatSuperGraphNode * fsgNode){
  p3d_fsgListNode * fgsListNode = MY_ALLOC(p3d_fsgListNode, 1);
  fgsListNode->node = fsgNode;
  fgsListNode->prev = NULL;
  fgsListNode->next = NULL;
  return fgsListNode;
}

p3d_fsgListEdge * p3d_createFsgListEdge(p3d_flatSuperGraphEdge * fsgEdge){
  p3d_fsgListEdge * fgsListEdge = MY_ALLOC(p3d_fsgListEdge, 1);
  fgsListEdge->edge = fsgEdge;
  fgsListEdge->prev = NULL;
  fgsListEdge->next = NULL;
  return fgsListEdge;
}

/***********************************/
/******* FlatSuperGraph List *******/
/***********************************/

//Node

void p3d_pushFsgListNode(p3d_fsgListNode ** list, p3d_fsgListNode * nodeToInsert){
  if(nodeToInsert){
    if(!*list){//first element of the list
      *list = nodeToInsert;
      return;
    }
    p3d_fsgListNode * tmp = *list;
    while(tmp->next){
      tmp = tmp->next;
    }
    tmp ->next = nodeToInsert;
    nodeToInsert->prev = tmp;
  }
}

void p3d_insertAfterFsgListNode(p3d_fsgListNode * prevNode, p3d_fsgListNode * nodeToInsert){
  if(prevNode && nodeToInsert){
    if(prevNode->next){
      p3d_fsgListNode * nextNode = prevNode->next;
      prevNode->next = nodeToInsert;
      nodeToInsert->prev = prevNode;
      nextNode->prev = nodeToInsert;
      nodeToInsert->next = nextNode;
    }else{
      prevNode->next = nodeToInsert;
      nodeToInsert->prev = prevNode;
    }
  }
}

void p3d_insertBeforeFsgListNode(p3d_fsgListNode * nextNode, p3d_fsgListNode * nodeToInsert){
  if(nextNode && nodeToInsert){
    if(nextNode->prev){
      p3d_fsgListNode * prevNode = nextNode->prev;
      prevNode->next = nodeToInsert;
      nodeToInsert->prev = prevNode;
      nextNode->prev = nodeToInsert;
      nodeToInsert->next = nextNode;
    }else{
      nextNode->prev = nodeToInsert;
      nodeToInsert->next = nextNode;
    }
  }
}

void p3d_removeFsgListNode (p3d_fsgListNode *fsgListNode){
  p3d_fsgListNode * tmp = fsgListNode->prev;

  if(!fsgListNode->prev){
    fsgListNode->prev->next = fsgListNode->next;
  }
  if(fsgListNode->next){
    fsgListNode->next->prev = tmp;
  }
}

//Edge

void p3d_pushFsgListEdge(p3d_fsgListEdge ** list, p3d_fsgListEdge * edgeToInsert){
  if(edgeToInsert){
    if(!*list){//first element of the list
      *list = edgeToInsert;
      return;
    }
    p3d_fsgListEdge * tmp = *list;
    while(tmp->next){
      tmp = tmp->next;
    }
    tmp->next = edgeToInsert;
    edgeToInsert->prev = tmp;
  }
}

void p3d_insertAfterFsgListEdge(p3d_fsgListEdge * prevEdge, p3d_fsgListEdge * edgeToInsert){
  if(prevEdge && edgeToInsert){
    if(prevEdge->next){
      p3d_fsgListEdge * nextEdge = prevEdge->next;
      prevEdge->next = edgeToInsert;
      edgeToInsert->prev = prevEdge;
      nextEdge->prev = edgeToInsert;
      edgeToInsert->next = nextEdge;
    }else{
      prevEdge->next = edgeToInsert;
      edgeToInsert->prev = prevEdge;
    }
  }
}

void p3d_insertBeforeFsgListEdge(p3d_fsgListEdge * nextEdge, p3d_fsgListEdge * edgeToInsert){
  if(nextEdge && edgeToInsert){
    if(nextEdge->prev){
      p3d_fsgListEdge * prevEdge = nextEdge->prev;
      prevEdge->next = edgeToInsert;
      edgeToInsert->prev = prevEdge;
      nextEdge->prev = edgeToInsert;
      edgeToInsert->next = nextEdge;
    }else{
      nextEdge->prev = edgeToInsert;
      edgeToInsert->next = nextEdge;
    }
  }
}

void p3d_removeFsgListEdge (p3d_fsgListEdge *fsgListEdge){
  p3d_fsgListEdge * tmp = fsgListEdge->prev;

  if(!fsgListEdge->prev){
    fsgListEdge->prev->next = fsgListEdge->next;
  }
  if(fsgListEdge->next){
    fsgListEdge->next->prev = tmp;
  }
}

/***********************************/
/******* FlatSuperGraph Del ********/
/***********************************/

void p3d_delFlatSuperGraph (p3d_rob * r, p3d_flatSuperGraph * fsg){
  if(fsg){
    p3d_delFsgListNode(r, fsg->nodes);
    fsg->nodes = NULL;
    p3d_delFsgListEdge(fsg->edges);
    fsg->edges = NULL;
    fsg->nNodes = 0;
    fsg->nEdges = 0;
    fsg->autoColNodes.clear();
//     for(int i = 0; i < fsg->nbCoordNeeded; i++){
//       MY_FREE(fsg->coordNeeded[i], p3d_list_edge *, r->mg->nbGraphs);
//     }
//     MY_FREE(fsg->coordNeeded, p3d_list_edge **, (1+fsg->nbCoordNeeded/10)*10);
  }
}

void p3d_delFsgListNode(p3d_rob * r, p3d_fsgListNode * list){
  p3d_fsgListNode * tmp = NULL;
  while(list && list->next){
    tmp = list;
    list = list->next;
    p3d_delNodeFromFsgListNode(r, tmp);
    tmp = NULL;
  }
  p3d_delNodeFromFsgListNode(r, list);
  list = NULL;
}

void p3d_delFsgListEdge(p3d_fsgListEdge * list){
  p3d_fsgListEdge * tmp = NULL;
  while(list && list->next){
    tmp = list;
    list = list->next;
    p3d_delNodeFromFsgListEdge(tmp);
    tmp = NULL;
  }
  p3d_delNodeFromFsgListEdge(list);
  list = NULL;
}

void p3d_delNodeFromFsgListNode(p3d_rob * r, p3d_fsgListNode * node){
  if(node != NULL){
    p3d_delFsgNode(r, node->node);
    node->node = NULL;
  }
  MY_FREE(node, p3d_fsgListNode, 1);
  node = NULL;
}

void p3d_delNodeFromFsgListEdge(p3d_fsgListEdge * node){
  if(node != NULL && node->edge != NULL){
    p3d_delFsgEdge(node->edge);
    node->edge = NULL;
  }
  MY_FREE(node, p3d_fsgListEdge, 1);
  node = NULL;
}

void p3d_delFsgNode(p3d_rob * r, p3d_flatSuperGraphNode * node){
  MY_FREE(node->nodes,p3d_node *, node->nNodes);//the p3d_node will be freed in the same time as the p3d_graph
  p3d_delFsgListEdge(node->fsgEdges);
  MY_FREE(node, p3d_flatSuperGraphNode, 1);
  node = NULL;
}

void p3d_delFsgEdge(p3d_flatSuperGraphEdge * edge){
  MY_FREE(edge, p3d_flatSuperGraphEdge, 1);
  edge = NULL;
}

//old Functions

// //mode : 0 merge all, 1 merge traj only, 2 merge only non merged
// int p3d_fillFlatMultiGraph(p3d_rob * r, p3d_node ** node,  p3d_multiGraphJoint ** mgJoints, int mgNum, int mode){
//   p3d_list_node * ln = NULL;
//   p3d_node * n = NULL;
//   static int depth = 0;
//   if(!node){//si node == NULL => premiere entree dans la fonction
//     depth = 1;
//     p3d_setAllDofPassive(r);//desactivation de tous les mg
//     p3d_activateMgAutocol(r,mgNum);//activation des joints du premier mg
//     node = MY_ALLOC(p3d_node*, r->mg->nbGraphs);
//     mgJoints = MY_ALLOC(p3d_multiGraphJoint*, r->mg->nbGraphs);
//     for(int i = 0; i < r->mg->nbGraphs; i++){//initilisation du tableau
//       node[i] = NULL;
//       mgJoints[i] = NULL;
//     }
//     p3d_graph * g = r->mg->graphs[mgNum];
//     switch(mode){
//       case 0: {//merge all
//         ln = g->nodes;
//         for(; ln; ln = ln->next){
//           node[0] = ln->N;
//           mgJoints[0] = r->mg->mgJoints[mgNum];
//           p3d_fillFlatMultiGraph(r, node, mgJoints, mgNum + 1, mode);
//         }
//         break;
//       }
//       case 1: {//merge traj only
//         n = g->search_start;
//         for(; n; n = n->search_to){
//           node[0] = n;
//           mgJoints[0] = r->mg->mgJoints[mgNum];
//           p3d_fillFlatMultiGraph(r, node, mgJoints, mgNum + 1, mode);
//         }
//         break;
//       }
//       case 2: {//merge only non merged
//         ln = g->nodes;
//         for(; ln; ln = ln->next){
//           if(ln->N->mergeState == 0){
//             node[0] = ln->N;
//             mgJoints[0] = r->mg->mgJoints[mgNum];
//             p3d_fillFlatMultiGraph(r, node, mgJoints, mgNum + 1, mode);
//             ln->N->mergeState = 1;
//           }
//         }
//         break;
//       }
//       default: {//default
//         return 0;
//       }
//     }
//     MY_FREE(node, p3d_node*, r->mg->nbGraphs);
//     p3d_deactivateMgAutocol(r,mgNum);
//     if(mode == 0){//tag all p3d_nodes
//       for(int i = 0; i < r->mg->nbGraphs; i++){
//         g = r->mg->graphs[i];
//         for(ln = g->nodes; ln; ln = ln->next){
//           ln->N->mergeState = 1;
//         }
//       }
//     }
//   }else{
//     depth++;
//     if(mgNum == r->mg->nbGraphs){
//       mgNum = 0;
//     }
//     if(depth > r->mg->nbGraphs){
//       depth--;
//       return 0;
//     }
//     p3d_activateMgAutocol(r,mgNum);//activation des joints du MG
//     p3d_graph * g = r->mg->graphs[mgNum];
//     switch(mode){
//       case 0: {//merge all
//         ln = g->nodes;
//         for(; ln; ln = ln->next){//tester node avec chaque noeud de la trajectoire si il n'y a pas de collision, voir pour un autre mg
//           node[depth - 1] = ln->N;
//           mgJoints[depth - 1] = r->mg->mgJoints[mgNum];
//           int nbConfigs = 0;
//           configPt *q = p3d_mergeMultiGraphNodes(r, depth, node, mgJoints, &nbConfigs);//creation de la configuration du robot
//           p3d_checkMergeCollision(r, node, mgJoints, mgNum, mode, nbConfigs, q);
//         }
//         break;
//       }
//       case 1: {//merge traj only
//         n = g->search_start;
//         for(;n; n = n->search_to){//tester node avec chaque noeud de la trajectoire si il n'y a pas de collision, voir pour un autre mg
//           node[depth - 1] = n;
//           mgJoints[depth - 1] = r->mg->mgJoints[mgNum];
//           int nbConfigs = 0;
//           configPt *q = p3d_mergeMultiGraphNodes(r, depth, node, mgJoints, &nbConfigs);//creation de la configuration du robot
//           p3d_checkMergeCollision(r, node, mgJoints, mgNum, mode, nbConfigs, q);
//         }
//         break;
//       }
//       case 2: {//merge only non merged
//         ln = g->nodes;
//         for(; ln; ln = ln->next){//tester node avec tous les autres noeuds
//           node[depth - 1] = ln->N;
//           mgJoints[depth - 1] = r->mg->mgJoints[mgNum];
//           int nbConfigs = 0;
//           configPt *q = p3d_mergeMultiGraphNodes(r, depth, node, mgJoints, &nbConfigs);//creation de la configuration du robot
//           p3d_checkMergeCollision(r, node, mgJoints, mgNum, mode, nbConfigs, q);
//         }
//         break;
//       }
//     }
//     node[mgNum] = NULL;
//     p3d_deactivateMgAutocol(r,mgNum);//desactivation des joints du MG
//   }
//   depth--;
//   return 1;
// }


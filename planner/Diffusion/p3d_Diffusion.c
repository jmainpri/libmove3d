#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "P3d-pkg.h"

static int  IS_BALANCED_EXPANSION = TRUE;
static int IS_BIDIRECTION_DIFFU = FALSE;
static int IS_EXPANSION_TO_GOAL = TRUE;

/**
 * p3d_SetIsBidirectDiffu
 * Set if the diffusion method is mono or bidirectionnal
 * @param[In]: TRUE if the diffusion method is birectionnal
 */
void p3d_SetIsBidirectDiffu(int IsBidirectDiffu) {
  IS_BIDIRECTION_DIFFU = IsBidirectDiffu;
}

/**
 * p3d_GetIsBidirectDiffu
 * Get if the diffusion method is  mono or bidirectionnal
 * @return: TRUE if the diffusion method is birectionnal
 */
int p3d_GetIsBidirectDiffu(void) {
  return IS_BIDIRECTION_DIFFU;
}

/**
 * p3d_SetIsExpansionToGoal
 * Set if the diffusion method expand toward 
 * a given goal or not
 * @param[In]: TRUE if diffusion method expand toward 
 * a given goal
 */
void p3d_SetIsExpansionToGoal(int IsExpansionToGoal) {
  IS_EXPANSION_TO_GOAL = IsExpansionToGoal;
}

/**
 * p3d_GetIsExpansionToGoal
 * Get if the diffusion method expand toward 
 * a given goal or not
 * @return: TRUE if diffusion method expand toward 
 * a given goal
 */
int p3d_GetIsExpansionToGoal(void) {
  return IS_EXPANSION_TO_GOAL;
}

/**
 * p3d_GetIsBalancedExpansion
 * Get if the expansion process must keep the
 * componant balanced.
 * @return: TRUE if the expansion is balance
 */
int p3d_GetIsBalancedExpansion(void) {
  return IS_BALANCED_EXPANSION;
}

/**
* p3d_SetIsBalancedExpansion
 * Set if the expansion process must keep the
 * componant balanced.
 * @param[In] IsBalancedExpansion: TRUE if the expansion 
 * is balance
 */
void p3d_SetIsBalancedExpansion(int IsBalancedExpansion) {
  IS_BALANCED_EXPANSION = IsBalancedExpansion;
}


/**
 * p3d_DiffuseOneConf
 * Create a new configuration diffused from a first one
 * by sampling a rand configuration in the space.
 * The diffusion step id equal to dMax*extendStepParam
 * (similarly to the one used in the expend version of the 
 * diffusion methods)
 * @param[In] robotPt: the current robot.
 * @param[In] qcurrent: the configuration from which the 
 * diffusion occure 
 * @return: the diffused new configuration 
 */
configPt p3d_DiffuseOneConf(p3d_rob* robotPt, configPt qCurrent) {
  int isExpandDirectionFound, i, j, k;
  double dMax, step, extendStepParam;
  configPt directionConfig, qNew;
  double deltaPath;
  p3d_jnt* jntPt; 
  //pseudo random direction sampled
  directionConfig = p3d_alloc_config(robotPt);
  isExpandDirectionFound = p3d_shoot(robotPt, directionConfig,
				     TRUE);
  if(isExpandDirectionFound == FALSE) {
    PrintInfo(("Error: failed to sample a\
 random direction for diffusion\n"));
     return NULL;
   }
 dMax =  p3d_get_env_dmax();
 extendStepParam = p3d_GetExtendStepParam();
 step = dMax*extendStepParam;

 deltaPath = step/p3d_dist_config(robotPt,qCurrent, directionConfig);
 qNew = p3d_alloc_config(robotPt);

 //expansion process
 for(i=0; i<=robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if(p3d_jnt_get_dof_is_user(jntPt, j)) {
	if(p3d_jnt_is_dof_circular(jntPt, j)) {
	  qNew[k] = qCurrent[k] + 
	    deltaPath*diff_angle(qCurrent[k],directionConfig[k]);
	}
	else {
	  qNew[k] = qCurrent[k] + 
	    deltaPath*(directionConfig[k] - qCurrent[k]); 
	}
      } else {
	qNew[k] = qCurrent[k];
      }
    }
  }
 p3d_destroy_config(robotPt, directionConfig);
 return qNew;
}




/**
 * p3d_ExpandOneStep
 * Function corresponding to one step of expansion for a 
 * componant, based on the principle of diffusion and exploring
 *  trees: select a direction of expansion and a node to expand
 *  then expand the componant from the node in the selected direction.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompToExpandPt: Pointer to the componant to expand
 * @param[In] GoalCompPt: The goal componant we want to reach during 
 * the expansion. This parameter can be used if we want to biase the 
 * expansion process. Can be set  to NULL if we don't want any bias. 
 * @return: The number of created nodes
 */
static int p3d_ExpandOneStep(p3d_graph* GraphPt, p3d_compco* CompToExpandPt, 
		      p3d_compco* GoalCompPt) {

  int NbCreatedNodes = 0;
  int IsExpandDirectionFound = FALSE;
  p3d_node* ExpansionNodePt = NULL;
  configPt DirectionConfig;
  int ArePassiveDofsSampled = TRUE;
  double Ratio = -1.;
  double RandomNum = -1.;
  int savedDistChoice;
  DirectionConfig  = p3d_alloc_config(GraphPt->rob);

  /* selection of the direction of expansion */
  if(p3d_GetIsManhatExpansion() == TRUE) {
    Ratio = p3d_GetManhattanRatio(); 
    RandomNum = p3d_random(0.,1.);
    if(Ratio > RandomNum) {
      /*Decompose expansion into active and passive dofs expansion*/
      ArePassiveDofsSampled = FALSE;
    }
  }
//   printf("RRT before shoot\n");
  IsExpandDirectionFound = SelectExpansionDirection(GraphPt, 
						    CompToExpandPt, GoalCompPt, 
						    DirectionConfig, 
						    ArePassiveDofsSampled);
//   printf("RRT after shoot: direction found = %d\n", IsExpandDirectionFound);
  if (IsExpandDirectionFound  == FALSE) {
    p3d_destroy_config(GraphPt->rob, DirectionConfig);
    return 0;
  }

  if(ArePassiveDofsSampled == FALSE) {
  savedDistChoice = p3d_GetDistConfigChoice();
  p3d_SetDistConfigChoice(ACTIVE_CONFIG_DIST);
  }
 /* selection of the node to expand */

  ExpansionNodePt = SelectExpansionNode(GraphPt, CompToExpandPt, 
					DirectionConfig);
//   printf("RRT found neighbour\n");
  if(ArePassiveDofsSampled == FALSE)  {
    p3d_SetDistConfigChoice(savedDistChoice);
    }
  if(ExpansionNodePt == NULL) {
    //  PrintInfo (("Warning: failed to select a node to expand\n"));
    p3d_destroy_config(GraphPt->rob, DirectionConfig);
    return 0;
  }
  /*Process of node expansion. Can lead to the creation of 
    several new nodes */
  if(ArePassiveDofsSampled == FALSE) {
    p3dCopyPassive(GraphPt->rob, ExpansionNodePt->q, DirectionConfig);
  }
  NbCreatedNodes = ExpandProcess(GraphPt, ExpansionNodePt, DirectionConfig);
//   printf("RRT expend %d\n", NbCreatedNodes);
  if(ArePassiveDofsSampled == FALSE) {
    /* A manhatan expansion is occuring. We must expand the passive dofs*/
   NbCreatedNodes += p3d_PassivExpandProcess(GraphPt, ExpansionNodePt, 
					     NbCreatedNodes);
  }
  p3d_destroy_config(GraphPt->rob, DirectionConfig);
  return NbCreatedNodes;
}



static int p3d_MonteCarloOneStep(p3d_graph* GraphPt, p3d_compco* CompToExpandPt, 
		      p3d_compco* GoalCompPt) {

  int NbCreatedNodes = 0;
  int IsExpandDirectionFound = FALSE;
  p3d_node* ExpansionNodePt = NULL;
  configPt DirectionConfig;

  DirectionConfig  = p3d_alloc_config(GraphPt->rob);


  IsExpandDirectionFound = p3d_RandDirShoot(GraphPt->rob,  DirectionConfig, 
						    TRUE);


  ExpansionNodePt = CompToExpandPt->dist_nodes->N;
  p3d_addConfig(GraphPt->rob, DirectionConfig, ExpansionNodePt->q,
		DirectionConfig);
  NbCreatedNodes = ExpandProcess(GraphPt, ExpansionNodePt, DirectionConfig);
 
  p3d_destroy_config(GraphPt->rob, DirectionConfig);
  return NbCreatedNodes;
}





/**
 * p3d_ExpandCompWithoutGoal 
 * Expand a connect componant without any goal to reach
 * stop when a given amount of nodes has been created or
 * when the user push the stop button 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: Pointer to the componant to expand
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: The number of added nodes
*/
int p3d_ExpandCompWithoutGoal(p3d_graph* GraphPt, p3d_compco* CompPt, 
			      int (*StopFunction)(void), 
			      void (*DrawFunction)(void)) {
  
  int NbNodeMaxComp = -1;
  int NbCurCreatedNodes = 0;
  int NbTotCreatedNodes = 0;
  int NTryCreateNode = 0;
  int nLoop = 0;
  //  int Stop = FALSE;

  if((GraphPt == NULL) ||(CompPt == NULL)) {
    PrintInfo (("Warning: Try to expand a  Graph \
or a CompPt NULL\n"));
    return 0;
  }

  NbNodeMaxComp = p3d_get_COMP_NODES();
  
  while(( CompPt->nnode < NbNodeMaxComp) &&(p3d_GetStopValue() == FALSE)) {
    nLoop++;
   
    if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      NbCurCreatedNodes = p3d_MonteCarloOneStep(GraphPt,CompPt, NULL);
    } else{
      NbCurCreatedNodes =  p3d_ExpandOneStep(GraphPt,CompPt, NULL);
    }
    if(NbCurCreatedNodes != 0) {
      NbTotCreatedNodes += NbCurCreatedNodes;
      NTryCreateNode = 0;
						
      if (DrawFunction) (*DrawFunction)();
 
      /*NbNodeMaxComp need to be refreshed as the user 
	can change its value during the expand process */
      NbNodeMaxComp = p3d_get_COMP_NODES();
    }
    else {
      NTryCreateNode += 1;
      if(NTryCreateNode > p3d_get_NB_TRY()) {
	PrintInfo (("riched the maximal number of consecutive \
 failures to expand a Comp \n"));
	break;
      } 
    }
    /* Function used to stop the exploration process. Typically, 
       used when someone check the stop button*/
    if (StopFunction) {
      if (!(*StopFunction)())
	{
	  PrintInfo(("Componant expansion canceled\n"));
	  p3d_SetStopValue(TRUE);
	}
    }
  }
  return NbTotCreatedNodes;
}

/**
 * p3d_ExpandCompToGoal 
 * Expand a connect componant with a goal to reach 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: Pointer to the componant to expand
 * @param[In] GoalNodePt: Pointer to the Goal Node 
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: The number of added nodes
*/
int p3d_ExpandCompToGoal(p3d_graph* GraphPt, p3d_compco* CompPt,
			 p3d_node* GoalNodePt,
			 int (*StopFunction)(void), 
			 void (*DrawFunction)(void)) {
  
  int IsCompConnectToGoal = FALSE;
  int NbNodeMaxComp = -1;
  int NbCurCreatedNodes = 0;
  int NbTotCreatedNodes = 0;
  int NTryCreateNode = 0;
  if((GraphPt == NULL) ||(CompPt == NULL)) {
    PrintInfo (("Warning: Try to expand a  Graph \
or a CompPt NULL\n"));
    return 0;
  }

  NbNodeMaxComp = p3d_get_COMP_NODES();

  // first connexion attempt without node addition
  if(p3d_GetIsCostFuncSpace() == FALSE ) {
    IsCompConnectToGoal =  p3d_ConnectNodeToComp(GraphPt, 
						     GoalNodePt, CompPt);
  } else {
    IsCompConnectToGoal  = p3d_CostConnectNodeToComp(GraphPt, 
						     GoalNodePt, CompPt);
  } 

  while( (CompPt->nnode < NbNodeMaxComp) && 
	 (IsCompConnectToGoal == FALSE)  && 
	 (p3d_GetStopValue() == FALSE)) {

  /* Function used to stop the exploration process. Typically, 
       used when someone check the stop button*/
    if (StopFunction) {
      if (!(*StopFunction)()) {
	PrintInfo(("Componant expansion canceled\n"));
	p3d_SetStopValue(TRUE);
	break;
      }
    }
    if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      NbCurCreatedNodes = p3d_MonteCarloOneStep(GraphPt,CompPt, GoalNodePt->comp );
    } else{
      NbCurCreatedNodes =  p3d_ExpandOneStep(GraphPt,CompPt, GoalNodePt->comp);
    }
    if(NbCurCreatedNodes != 0) {
      if (DrawFunction) (*DrawFunction)();
      NbTotCreatedNodes += NbCurCreatedNodes;
      NTryCreateNode = 0;
      if(p3d_GetIsCostFuncSpace() == FALSE ) {
	IsCompConnectToGoal =  p3d_ConnectNodeToComp(GraphPt, 
						     GoalNodePt, CompPt);
      } else {
	IsCompConnectToGoal  = p3d_CostConnectNodeToComp(GraphPt, 
							 GoalNodePt, CompPt);
      }   
    }
    else {
      if(GoalNodePt->comp->num == CompPt->num){
	IsCompConnectToGoal = TRUE;
      } else {
	NTryCreateNode += 1;
	if(NTryCreateNode > p3d_get_NB_TRY()) {
	  PrintInfo (("Riched the maximal number of consecutive \
 failures to expand a Comp \n"));
	  p3d_SetStopValue(TRUE);
	} 
	/*NbNodeMaxComp need to be refreshed as the user 
	  can change its value during the expand process */  
	NbNodeMaxComp = p3d_get_COMP_NODES();
      }
    }
  
  }
  if (DrawFunction) (*DrawFunction)();

  return NbTotCreatedNodes;
}

/**
 * p3d_ExpandInitCompToGoal 
 * Expand the initial componant (ie. including 
 * the search start node) and try to connect it to the goal 
 * configuration (search_goal) 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: the number of added nodes
*/
int p3d_ExpandInitCompToGoal(p3d_graph* GraphPt, int (*StopFunction)(void),
			     void (*DrawFunction)(void)) {
  p3d_compco* InitCompPt;
  p3d_node* GoalNodePt;
  int NbAddedNodes = 0; 
 
  if(GraphPt == NULL) {
    PrintInfo (("Warning: Try to expand a NULL graph \n"));
    return FALSE;
  }
  if(p3d_equal_config(GraphPt->rob, GraphPt->search_start->q, 
		      GraphPt->search_goal->q)) {
    PrintInfo(("Tree Expansion failed: root nodes are the same\n"));
    return FALSE;
  }
  InitCompPt = GraphPt->search_start->comp;
  GoalNodePt = GraphPt->search_goal;
  NbAddedNodes = p3d_ExpandCompToGoal(GraphPt, InitCompPt, 
				      GoalNodePt, 
				      StopFunction, 
				      DrawFunction);
  return NbAddedNodes;
}

/**
 * p3d_ExpandInitCompNoGoal 
 * Expand the initial componant (ie. including 
 * the search start node) without goal 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: the number of added nodes
*/
int p3d_ExpandInitCompNoGoal(p3d_graph* GraphPt, int (*StopFunction)(void),
			     void (*DrawFunction)(void)) {
  p3d_compco* InitCompPt;
  int NbAddedNodes = 0; 
 
  if(GraphPt == NULL) {
    PrintInfo (("Warning: Try to expand a NULL graph \n"));
    return FALSE;
  }
  InitCompPt = GraphPt->search_start->comp;
 
  NbAddedNodes = p3d_ExpandCompWithoutGoal(GraphPt, InitCompPt, 
					   StopFunction, DrawFunction);
  return NbAddedNodes;
}


/**
 * p3d_BidirExpandComp 
 * Expand two connect componants and try to connect them
 * The expansion can be balanced or not  
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: Pointer to the componant to expand
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: The number of added nodes
*/
int p3d_BidirExpandComp(p3d_graph* GraphPt, p3d_compco* Comp1Pt, 
			p3d_compco* Comp2Pt,
			int (*StopFunction)(void), 
			void (*DrawFunction)(void)) {
  
  int AreCompConnected = FALSE;
  int NbNodeMaxComp = -1;
  int NbCurCreatedNodes = 0;
  int NbTotCreatedNodes = 0;
  int NTryCreateNode = 0;
  p3d_node* NodeComp1Pt, *NodeComp2Pt;
  if((GraphPt == NULL) ||(Comp1Pt == NULL)||(Comp2Pt == NULL)) {
    PrintInfo (("Warning: Try to expand a  Graph \
or ComponantsPt NULL\n"));
    return FALSE;
  }

  NodeComp1Pt = Comp1Pt->last_node->N;
  NodeComp2Pt = Comp2Pt->last_node->N;
  NbNodeMaxComp = p3d_get_COMP_NODES();
  
  while( (AreCompConnected == FALSE ) &&
	 (Comp1Pt->nnode < NbNodeMaxComp) &&
	 (Comp2Pt->nnode < NbNodeMaxComp) &&
	 (p3d_GetStopValue() == FALSE)) {
    if((p3d_GetIsBalancedExpansion() == FALSE) ||
       (Comp1Pt->nnode < Comp2Pt->nnode +2)) {

    if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      NbCurCreatedNodes = p3d_MonteCarloOneStep(GraphPt,Comp1Pt, NULL);
    } else{
      NbCurCreatedNodes =  p3d_ExpandOneStep(GraphPt,Comp1Pt, Comp2Pt);
    }
      if(NbCurCreatedNodes != 0) {
	NbTotCreatedNodes += NbCurCreatedNodes;
	NTryCreateNode = 0;
	if(p3d_GetIsCostFuncSpace() == FALSE ) {
	AreCompConnected = p3d_ConnectNodeToComp(GraphPt,
						 GraphPt->last_node->N,
						 Comp2Pt);
	} else {
	AreCompConnected = p3d_CostConnectNodeToComp(GraphPt,
						 GraphPt->last_node->N,
						 Comp2Pt);
	}
	if (DrawFunction) (*DrawFunction)();
      }
      else {
	NTryCreateNode += 1;
	if(NTryCreateNode > p3d_get_NB_TRY()) {
	  PrintInfo (("Riched the maximal number of consecutive \
 failures to expand a Comp \n"));
	  p3d_SetStopValue(TRUE);
	}
      }
    }
    if(NodeComp1Pt->comp->num == NodeComp2Pt->comp->num ) {
      AreCompConnected = TRUE;
    }
    if((AreCompConnected == FALSE )&&((p3d_GetIsBalancedExpansion() == FALSE) ||
	(Comp2Pt->nnode < Comp1Pt->nnode +2))) {
      if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
	NbCurCreatedNodes = p3d_MonteCarloOneStep(GraphPt,Comp2Pt, Comp1Pt);
      } else{
	NbCurCreatedNodes =  p3d_ExpandOneStep(GraphPt,Comp2Pt, Comp1Pt);
      }
 
      if(NbCurCreatedNodes != 0) {
	NbTotCreatedNodes += NbCurCreatedNodes;
	NTryCreateNode = 0;
	if(p3d_GetIsCostFuncSpace() == FALSE ) {
	AreCompConnected = p3d_ConnectNodeToComp(GraphPt,
						 GraphPt->last_node->N,
						 Comp1Pt); 
	} else   {
	  AreCompConnected  = p3d_CostConnectNodeToComp(GraphPt,
						 GraphPt->last_node->N,
						 Comp1Pt); 
	  
	}
	if (DrawFunction) (*DrawFunction)();
      }
      else {
	NTryCreateNode += 1;
	if(NTryCreateNode > p3d_get_NB_TRY()) {
	  PrintInfo (("Riched the maximal number of consecutive \
 failures to expand a Comp \n"));
	  p3d_SetStopValue(TRUE);


	}
      }
    }
    /*NbNodeMaxComp need to be refreshed as the user 
      can change its value during the expand process */
    NbNodeMaxComp = p3d_get_COMP_NODES();

    /* Function used to stop the exploration process. Typically, 
       used when someone check the stop button*/
    if (StopFunction) {
      if (!(*StopFunction)())
	{
	  PrintInfo(("Componants bi-expansion canceled\n"));
	  p3d_SetStopValue(TRUE);

	}
    }
  }
  return NbTotCreatedNodes;
}

/**
 * p3d_BiExpandInitGoalComp 
 * Expand the initial and Goal componants and try to connect 
 * them. 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: The total number of created nodes
*/
int p3d_BiExpandInitGoalComp(p3d_graph* GraphPt, int (*StopFunction)(void),
			     void (*DrawFunction)(void)) {
  p3d_compco* InitCompPt;
  p3d_compco* GoalCompPt;
  int NbTotCreatedNodes = 0;

  if(GraphPt == NULL) {
    PrintInfo (("Warning: Try to expand a NULL graph \n"));
    return 0;
  }
  if(p3d_equal_config(GraphPt->rob, GraphPt->search_start->q, 
		      GraphPt->search_goal->q)) {
    PrintInfo(("Tree Expansion failed: root nodes are the same\n"));
    return 0;
  }

  InitCompPt = GraphPt->search_start->comp;
  GoalCompPt = GraphPt->search_goal->comp;
  NbTotCreatedNodes = p3d_BidirExpandComp(GraphPt, InitCompPt, 
					  GoalCompPt, StopFunction, 
					  DrawFunction);
  return NbTotCreatedNodes;
}

/**
 * p3d_RunDiffusion
 * Main function to run a diffusion process based 
 * on the current value of the setting parameter
 * @param[In] GraphPt: Pointeer on the Robot Graph
 * @param[In] fct_stop: stop function used to stop the process.
 * Used in articular when the user push the stop button
 * @param[In] fct_draw: function used to draw the current graph.
 * @return: TRUE if the extremal positions are linked.
 */
int p3d_RunDiffusion(p3d_graph* GraphPt, int (*fct_stop)(void),
		     void (*fct_draw)(void), p3d_node* Ns, p3d_node* Ng) {
  double    tu,ts;
//   p3d_node  *Ns=NULL,*Ng=NULL;
  p3d_rob* RobotPt = GraphPt->rob;
  int nbAddedNodes = 0;

  ChronoOn();
 /* Nodes QS and QG exist ?*/
//   Ns = p3d_TestConfInGraph(GraphPt, ConfigStart);
//   if(Ns == NULL) {
//     Ns = p3d_CreateExtremalNodeFromConf(GraphPt,ConfigStart);
//   } else {
//     p3d_destroy_config(RobotPt, ConfigStart);
//     ConfigStart = NULL;
//   }
//   if((p3d_GetIsBidirectDiffu() == TRUE) || (p3d_GetIsExpansionToGoal()== TRUE)) {
//   
//     Ng = p3d_TestConfInGraph(GraphPt, ConfigGoal); 
//     if(Ng == NULL) {
//       Ng = p3d_CreateExtremalNodeFromConf(GraphPt,ConfigGoal);
//     } else {
//       p3d_destroy_config(RobotPt, ConfigGoal);
//       ConfigGoal = NULL;
//     }
//   }

  p3d_InitRun(GraphPt,Ns, Ng);
  p3d_set_robot_config(RobotPt, Ns->q);
  p3d_update_this_robot_pos_without_cntrt_and_obj(RobotPt);
  if((p3d_GetIsCostFuncSpace() == TRUE) &&   
     (p3d_GetExpansionChoice() == ONE_NODE_CONNECT_EXP_CHOICE)) {
    PrintInfo(("Warning: Connect expansion strategy \
is usually unadapted for cost spaces\n"));
  }

    if(p3d_GetIsBidirectDiffu() == TRUE) {
      nbAddedNodes = p3d_BiExpandInitGoalComp(GraphPt, fct_stop, fct_draw);
    } else if (p3d_GetIsExpansionToGoal() == TRUE) {
      nbAddedNodes = p3d_ExpandInitCompToGoal(GraphPt, fct_stop, fct_draw);
    } else {
      nbAddedNodes = p3d_ExpandInitCompNoGoal(GraphPt, fct_stop, fct_draw);

    }  
    
  ChronoPrint(""); 
  ChronoTimes(&tu,&ts); 
  GraphPt->time = GraphPt->time + tu;
 
   ChronoOff();
   if((p3d_GetIsExpansionToGoal()== FALSE ) || (Ng == NULL ) ){
    return FALSE;
  }
   //  return ((Ns->comp->num == Ng->comp->num) && (nbAddedNodes >=1));
   return (Ns->comp->num == Ng->comp->num);
}


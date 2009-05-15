#ifndef __CEXTRACT__
/**
 * p3d_SetIsWeightedChoice
 * Set if the nodes are weighted or not
 * param[In]: IsWeightedChoice: should be TRUE if
 * the nodes are weighted.
 */
void p3d_SetIsWeightedChoice(int IsWeightedChoice);

/**
 * p3d_GetIsWeightedChoice
 * Get if the nodes are weighted or not
 * @return: TRUE if the nodes are weighted.
 */
int p3d_GetIsWeightedChoice(void);

/**
 * p3d_GetNodeWeightStrategy
 * Get the strategy used to weight a node
 * @return:  the strategy used to weight a node.
 */
int p3d_GetNodeWeightStrategy(void);

/**
 * p3d_SetNodeWeightStrategy
 * Set the strategy used to weight a node
 * @param[In]:  the strategy used to weight a node.
 */
void p3d_SetNodeWeightStrategy(int NodeWeightStrat);

/**
 * p3d_GetNodeWeight
 * Get the weight of a given node depending
 * of the weighting strategy
 * @param[In]: The given node
 * @return: the weight of the node depending of the 
 * weighting strategy. 
 * Note: currently the returned value is different than the
 * NodePt->weight field. Should be modified to remove ambiguity
 */
double p3d_GetNodeWeight(p3d_node* NodePt);

/**
 * p3d_SetNodeWeight
 * Set the weight of a given node depending
 * of the weighting strategy. The function modify
 * the  field NodePt->weight.
 * @param[In]: GraphPt: the robot graph
 * @param[In]: NodePt: the given node
 */
void p3d_SetNodeWeight(p3d_graph* GraphPt, p3d_node* NodePt);

double p3d_get_rate_n_expan_without_w_improve(void);
double ffo_dist_from_root_pos(configPt q);

int p3d_get_w_inc_dir(void);
void p3d_init_root_weight(p3d_graph *G);


void p3d_SetStopWeightAndSign(double stop_weight, int sign_stop_weight);
void p3d_GetStopWeightAndSign(double *stop_weightPt, int *sign_stop_weightPt);

void p3d_SetDiffuStoppedByWeight(double stopped_by_weight);
int p3d_GetDiffuStoppedByWeight(void);

void p3d_SetIsWeightStopCondition(int IsWeightStopCondition);
int p3d_GetIsWeightStopCondition(void);
#endif

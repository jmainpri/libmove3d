/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "RRT.hpp"

using namespace std;
using namespace tr1;

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool RRT::costConnectNodeToComp(
		Node* node,
		Node* compNode)
{
	int SavedIsMaxDis = FALSE;
	Node* node2(NULL);

	switch(p3d_GetNodeCompStrategy()) {
	case K_NEAREST_NODE_COMP:
		/*Connect randomly to one of the k nearest
      nodes of the componant */
		/*todo*/
		break;
	case NEAREST_NODE_COMP:
	default:
		SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
		p3d_SetIsMaxDistNeighbor(FALSE);

		node2 = _Graph->nearestWeightNeighbour(compNode,
				node->getConfiguration(),
				false,
				p3d_GetDistConfigChoice());

		p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);

		//    if( ExpandCostConnect(
		//    		*node,
		//    		node2->getConf(),
		//    		node2,
		//    		Env::costConnect,
		//    		true) )

		LocalPath path(node->getConfiguration(),node2->getConfiguration());

		if( path.getValid() ){

			if(path.length() <= _expan->step())
			{
				int nbCreatedNodes=0;

//				p3d_LinkNodesMergeComp(_Graph->getGraphStruct(),
//				                           node->getNodeStruct(),
//				                           node2->getNodeStruct());

				connectNode(node,path,1.0,node2,node2->getConfiguration()->cost(),nbCreatedNodes);
				cout << "Path Valid Connected" << endl;
				return true;
			}

			if( expandToGoal(
					node,
					node2->getConfiguration()))
			{
				cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
				if(expandProcess(node, node2->getConfiguration(), node2, Env::nExtend) >= 1 )
					return true;
			}
		}
		return false;
	}
	return false;
}

/**
 * CostTestSucceeded
 * Transition Test function to validate the feasability of the motion
 * from the current config with the current cost in function
 * of the previous config and cost
 * this test is currently based on the Metropolis Criterion
 * also referred as the Boltzmann probability when applied to
 * statistical physics or molecular modeling
 * The temperature parameter is adaptively tuned  in function of the
 * failures and successes during the search process.
 * This adaptation can be local to each node or applied globaly to
 * the entire graph.
 * @param[In] G: pointer to the robot graph
 * @param[In] previousNodePt: pointer to the previous node
 * @param[In] currentConfig: current confguration
 * @param[In] PreviousCost: Previous cost (i.e. cost
 * of the previous config)
 * @param[In] CurrentCost: Current node cost
 * @param[In] IsRaisingCost: Give the direction of the
 * cost. TRUE if its easy to succeed test for increasing costs
 * (i.e from goal to init) FALSE otherwise.
 * @return: TRUE if the test succeeded, FALSE otherwise
 */

int Nb_succes=0;

bool RRT::costTestSucceeded(Node* previousNode,
		shared_ptr<Configuration> currentConfig,
		double currentCost) {
	p3d_node* previousNodePt(previousNode->getNodeStruct());
	double ThresholdVal;
	double dist;
	bool success(false);
	configPt previousConfig = previousNodePt->q;
	double temperature, cVertex,cOpt,cMax ;
	double minThreshold = 0.05;

	switch(p3d_GetCostMethodChoice()) {
	case MAXIMAL_THRESHOLD:
		// Literature method:
		// the condition test is only based on
		// an absolute cost which increase continuously.
		success = currentConfig->cost() < p3d_GetCostThreshold();
		break;
	case URMSON_TRANSITION:
		//TODO !
		cVertex = p3d_ComputeUrmsonNodeCost(_Graph->getGraphStruct(), previousNodePt);
		cOpt = _Start->getCompcoStruct()->minCost * (_Start->getConfiguration()->dist(*_Goal->getConfiguration(),
				p3d_GetDistConfigChoice())+1) / (2. * _expan->step());
		cMax = _Start->getCompcoStruct()->maxUrmsonCost;
		ThresholdVal = 1-(cVertex - cOpt)/(cMax - cOpt);
		ThresholdVal = MAX(ThresholdVal, minThreshold);
		//    PrintInfo(("Threshold value : %f,cVertex:%f, cOpt:%f, cMax:%f \n ",ThresholdVal,
		//       cVertex, cOpt, cMax));
		success = p3d_random(0.,1.) < ThresholdVal;
		// cout << "success: " << success << endl;
		break;
		//the same part is used for TRANSITION_RRT and
		//MONTE_CARLO_SEARCH
	case TRANSITION_RRT:
	case TRANSITION_RRT_CYCLE:
	case MONTE_CARLO_SEARCH:

		// IsRaisingCost is FALSE when we are expanding the InitComp:
		// Downhill slopes have better chances to be accepted. If
		// we are expanding the GoalComp tests are inversed: Uphill
		// slopes have better chances to be accepted
		// update: this doesn't work. Inverting the acceptance test means that
		// the tree will grow towards the maxima, whereas it should follow the saddle points

		// Old implementation in case of
		// down hill slopes including a maximal
		// ratio between uphill and downhill slopes
		// this test ratio has been removed
		/*   if(CurrentCost <= PreviousCost) { */
		/*     GlobalNbDown++; */
		/*     (previousNodePt->NbDown)++; */
		/*     if(p3d_GetIsLocalCostAdapt()) { */
		/*       if( (p3d_GetThresholdDown() != 0 )&&  */
		/* 	  (previousNodePt->NbDown > p3d_GetThresholdDown())) { */
		/* 	previousNodePt->NbDown =0; */
		/* 	return FALSE; */
		/*       } else { */
		/* 	return TRUE; */
		/*       } */
		/*     } else { */
		/*       if( (p3d_GetThresholdDown() != 0 )&&  */
		/* 	  (GlobalNbDown > p3d_GetThresholdDown())) { */
		/* 	GlobalNbDown = 0; */
		/* 	return FALSE; */
		/*       } else { */
		/* 	return TRUE; */
		/*       } */
		/*     } */
		/*   } */

		//new simplified test for down hill slopes
		if(currentCost <= previousNodePt->cost) {
			return true;
		}

		//  GlobalNbDown =0;
		// previousNodePt->NbDown =0;

		// In principle, the distance are not necessarly
		// reversible for non-holonomic robots
		dist = p3d_dist_q1_q2(_Robot->getRobotStruct(), currentConfig->getConfigStruct(), previousConfig);
		// dist = p3d_dist_q1_q2(mR, previousConfig, currentConfig);

		// get the value of the auto adaptive temperature.
		temperature = p3d_GetIsLocalCostAdapt() ?
				previousNodePt->temp :
				previousNodePt->comp->temperature;

		if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
			temperature = 0.001*ENV.getDouble(Env::alpha)*ENV.getInt(Env::maxCostOptimFailures);
		}
		/*Main function to test if the next config
      will be selected as a new node.
      The TemperatureParam adjust itself automatically
      in function of the fails and successes of extensions*/


		//Metropolis criterion (ie Boltzman probability)
		//    ThresholdVal = exp((PreviousCost-CurrentCost)/(temperature*dist));
		ThresholdVal = exp((previousNodePt->cost-currentCost)/temperature);


		//    success = ThresholdVal > 0.5;
		success = p3d_random(0.,1.) < ThresholdVal;
		if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
			break;
		}
		//  p3d_EvaluateExpandDiffic(previousNodePt->comp, success);
	}

	if(ENV.getBool(Env::printTemp))
	{
		cout << temperature <<"\t"<< previousNodePt->cost <<"\t"<< currentCost << endl;

		if(success)
		{
			cout << "-----------------------------------------------" << endl;
			cout << "Nb_succes = " << Nb_succes++ << endl;
			cout << "-----------------------------------------------" << endl;
		}
	}

	ENV.setDouble(Env::temperature,temperature);

	return success;
}

bool RRT::costTestSucceededConf(shared_ptr<Configuration>& previousConfig,
		shared_ptr<Configuration>& currentConfig,
		double temperature) {

	double previousCost = previousConfig->cost();
	double currentCost = currentConfig->cost();
	//new simplified test for down hill slopes
	if(currentCost <= previousCost) {
		return true;
	}
	// ATTENTION HERE!!!
	else{
		return false;
	}
	// In principle, the distance are not necessarly
	// reversible for non-holonomic robots
	// dist = p3d_dist_q1_q2(mR->mR, currentConfig.mQ, previousConfig);
	// dist = p3d_dist_q1_q2(mR, previousConfig, currentConfig);

	//Metropolis criterion (ie Boltzman probability)
	//    ThresholdVal = exp((PreviousCost-CurrentCost)/(temperature*dist));
	double ThresholdVal = exp((previousCost-currentCost)/temperature);

	// success = ThresholdVal > 0.5;
	bool success = p3d_random(0.,1.) < ThresholdVal;

	if(ENV.getBool(Env::printTemp)){
		cout << temperature <<"\t"<< previousCost <<"\t"<< currentCost << endl;

	}

	return success;
}

bool RRT::expandToGoal(	Node* expansionNode,
						shared_ptr<Configuration> directionConfig) {


	bool extensionSucceeded(true);

	double positionAlongDirection(0.);
	double temperature = expansionNode->getCompcoStruct()->temperature;
	double extensionCost(0.);

	shared_ptr<Configuration> fromConfig = expansionNode->getConfiguration();
	shared_ptr<Configuration> toConfig;

	shared_ptr<LocalPath> extensionLocalPath;

	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	while ( extensionSucceeded && positionAlongDirection < 1.) {

		LocalPath directionLocalPath(fromConfig,directionConfig);

		extensionSucceeded = _expan->nextStep(directionLocalPath, directionConfig,
				positionAlongDirection, extensionLocalPath, Env::nExtend);

		toConfig = extensionLocalPath->getEnd();

		if (extensionSucceeded && ENV.getBool(Env::isCostSpace) ) {

			extensionCost = toConfig->cost();

			if ( costTestSucceededConf(fromConfig,toConfig,temperature)) {
				if ( extensionCost > fromConfig->cost() ) {
//					temperature = adjustTemperature(true,temperature);
				}
			}
			else {
				//cout << "return false in nExtendToGoal" << endl;
				//cout << endl;
				return false;
			}
		}
		fromConfig = toConfig;
	}
	if(extensionSucceeded)
		cout << "return "<< extensionSucceeded <<" in nExtendToGoal"<< endl;

	//cout << endl;

	return extensionSucceeded;
}

bool RRT::expandCostConnect(Node& expansionNode,
		shared_ptr<Configuration> directionConfig,
		Node* directionNode,
		Env::expansionMethod method,
		bool toGoal) {

	bool extensionSucceeded(false);
	bool failed(false);

	int nbCreatedNodes(0);

	double positionAlongDirection(0.);
	shared_ptr<LocalPath> directionLP;
	shared_ptr<LocalPath> extensionLP;

	shared_ptr<Configuration> fromConfig = expansionNode.getConfiguration();
	shared_ptr<Configuration> toConfig;

	double extensionCost(0.);
	double prevCost(0.);

	bool firstIteration(true);
	bool upHill(false);

	int nb_of_extend=0;

	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	while(!failed && positionAlongDirection < 1.)
	{
		directionLP = shared_ptr<LocalPath>(
				new LocalPath(
						fromConfig,
						directionConfig));

		extensionSucceeded = _expan->nextStep(
				*directionLP,
				directionConfig,
				positionAlongDirection,
				extensionLP,
				method);

		nb_of_extend++;

		failed |= !extensionSucceeded;

		toConfig = extensionLP->getEnd();

		extensionCost = toConfig->cost();
		prevCost = 		fromConfig->cost();

		//cout << "extensionCost["<<nb_of_extend<<"] = "<< extensionCost << endl;
		//cout << "prevCost["<<nb_of_extend<<"] = "<< prevCost << endl;

		// Transition test for cost spaces, increase temperature in case of failure
		// Only for first iteration
		if(firstIteration && !failed)
		{
			// Expansion Control
			if(	ENV.getBool(Env::expandControl) &&
					!_expan->expandControl(*directionLP, 1.0 , expansionNode))
			{
				failed = true;
			}

			// Cost Test Control
			if((!failed) && (extensionCost > prevCost))
			{
				upHill = true;

				if(!costTestSucceeded(&expansionNode, toConfig, extensionCost))
				{
					adjustTemperature(false,&expansionNode);
					failed = true;
				}
				else
				{
					adjustTemperature(true,&expansionNode);
				}

			}
		}

		// Check if it's going down hill for next iterations
		if((!firstIteration) && (!failed))
		{
			if(extensionCost > prevCost){
				failed = true;
			}
		}

		if(failed){
			if(firstIteration){
				_expan->expansionFailed(expansionNode);
			}
			break;
		}

		fromConfig = toConfig;

		if(firstIteration){
			nbCreatedNodes++;
			firstIteration = false;
			if(upHill){
				failed = true;
			}
		}
	}

	if( nbCreatedNodes==1 &&
			( !toGoal  || (toGoal && positionAlongDirection >= 1.0))){
		//cout << "nb_of_extend = " << nb_of_extend << endl;
		//cout << "fromConfig = " << fromConfig->cost() << endl;

		directionLP = shared_ptr<LocalPath>(
				new LocalPath(
						expansionNode.getConfiguration(),
						directionConfig));

		extensionLP = shared_ptr<LocalPath>(
				new LocalPath(
						expansionNode.getConfiguration(),
						fromConfig));

		extensionCost = fromConfig->cost();

		double length = extensionLP->length();
//		ATTENTION
//		TODO Ajouter un champs dans composante connexete
//		expansionNode.getCompcoStruct()->sumLengthEdges += length;

		double positionAlongDirection =
			directionLP->length() == 0. ? 1. : MIN(1., _expan->step() / directionLP->length());
		nbCreatedNodes=0;
		connectNode(&expansionNode,
				*extensionLP,
				positionAlongDirection,
				directionNode,
				extensionCost,
				nbCreatedNodes);

		//	  cout << expansionNode.getConf()->cost() << "\t\t\t" << extensionCost << endl;
		//	  if(expansionNode.getConf()->cost() < extensionCost )
		//		  cout << "Rising" << endl;
		//	  cout << "nb_nodes = " << nb_nodes++ << endl;
	}

	if(nbCreatedNodes>1){
		cout << "ERRROR nbCreatedNodes in CostConnectMethod, nbCreatedNodes = "<< nbCreatedNodes<< endl;
	}

	if( ENV.getBool(Env::isCostSpace) && (p3d_GetCostMethodChoice() == MAXIMAL_THRESHOLD)) {
		p3d_updateCostThreshold();
	}

	return nbCreatedNodes;
}


void RRT::adjustTemperature(bool accepted, Node* node)
{

	double factor =
			exp(log(2.) / pow(10., ENV.getDouble(Env::temperatureRate)));

	if (accepted)
	{
		node->setTemp(node->getTemp() / 2.0);
		node->getCompcoStruct()->temperature /= 2.;
	}
	else
	{
		node->setTemp(node->getTemp() * factor);
		node->getCompcoStruct()->temperature *= factor;
	}
}

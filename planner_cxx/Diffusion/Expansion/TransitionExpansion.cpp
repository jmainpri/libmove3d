/*
 * TransitionExpansion.cpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#include "TransitionExpansion.hpp"

using namespace std;
using namespace tr1;

TransitionExpansion::TransitionExpansion() :
	TreeExpansionMethod()
	{
}

TransitionExpansion::TransitionExpansion(Graph* ptrGraph) :
	TreeExpansionMethod(ptrGraph)
	{
}

TransitionExpansion::~TransitionExpansion()
{
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

bool TransitionExpansion::costTestSucceeded(Node* previousNode,
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
		/*cVertex = p3d_ComputeUrmsonNodeCost(Graph->getGraphStruct(), previousNodePt);
		cOpt = _Start->getCompcoStruct()->minCost * (_Start->getConfiguration()->dist(*_Goal->getConfiguration(),
				p3d_GetDistConfigChoice())+1) / (2. * this->step());
		cMax = _Start->getCompcoStruct()->maxUrmsonCost;*/
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
		dist = p3d_dist_q1_q2(mGraph->getRobot()->getRobotStruct(), currentConfig->getConfigStruct(), previousConfig);
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

bool TransitionExpansion::costTestSucceededConf(shared_ptr<Configuration>& previousConfig,
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

bool TransitionExpansion::expandToGoal(	Node* expansionNode,
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

		extensionSucceeded = this->nextStep(directionLocalPath, directionConfig,
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

bool TransitionExpansion::expandCostConnect(Node& expansionNode,
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

		extensionSucceeded = this->nextStep(
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
					!this->expandControl(*directionLP, 1.0 , expansionNode))
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
				this->expansionFailed(expansionNode);
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
			directionLP->length() == 0. ? 1. : MIN(1., this->step() / directionLP->length());
		nbCreatedNodes=0;

		this->addNode(&expansionNode,
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


void TransitionExpansion::adjustTemperature(bool accepted, Node* node)
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


int TransitionExpansion::expandProcess(Node* expansionNode,
		shared_ptr<Configuration> directionConfig, Node* directionNode,
		Env::expansionMethod method)
{
	bool extensionSucceeded(false);
	bool failed(false);
	int nbCreatedNodes(0);
	Node fromNode(*expansionNode);
	Node* extensionNode(NULL);
	shared_ptr<LocalPath> directionLocalpath;
	double positionAlongDirection(0.);
	shared_ptr<LocalPath> extensionLocalpath;
	double extensionCost(0.);
	bool firstIteration(true);

//	cout << "Cost Expand" << endl;

	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	while (firstIteration || (method == Env::nExtend && !failed
			&& positionAlongDirection < 1.))
	{
		directionLocalpath = shared_ptr<LocalPath> (new LocalPath(
				fromNode.getConfiguration(), directionConfig));

		extensionSucceeded = this->nextStep(*directionLocalpath,
				directionNode, positionAlongDirection, extensionLocalpath,
				method);

		failed |= !extensionSucceeded;

		// Expansion Control
		if (firstIteration && !failed)
		{
			if (ENV.getBool(Env::expandControl)
					&& !this->expandControl(*directionLocalpath,
							positionAlongDirection, *expansionNode))
			{
				failed = true;
			}
		}

		// Transition test for cost spaces, increase temperature in case of failure
		if (!failed && ENV.getBool(Env::isCostSpace))
		{
			extensionCost = extensionLocalpath->getEnd()->cost();

			if (!costTestSucceeded(&fromNode, extensionLocalpath->getEnd(),
					extensionCost))
			{
				adjustTemperature(false, &fromNode);
				failed = true;

				int nbCostFail = ENV.getInt(Env::nbCostTransFailed);
				nbCostFail++;
				ENV.setInt(Env::nbCostTransFailed, nbCostFail);

				if (ENV.getBool(Env::printCostFail))
					cout << "nbCostFail = " << nbCostFail << endl;
			}

			if (!failed && (extensionCost
					> expansionNode->getConfiguration()->cost()))
			{
				adjustTemperature(true, &fromNode);
			}
		}

		// Add node to graph if everything succeeded
		if (!failed)
		{
			extensionNode = addNode(&fromNode, *extensionLocalpath,
					positionAlongDirection, directionNode, extensionCost,
					nbCreatedNodes);
		}
		if (firstIteration && failed)
		{
			this->expansionFailed(*expansionNode);
		}

		if (!failed)
		{
			fromNode = *extensionNode;
		}
		firstIteration = false;

	}

	if (ENV.getBool(Env::isCostSpace) && ENV.getInt(Env::CostMethodChoice)
			== MAXIMAL_THRESHOLD)
	{
		p3d_updateCostThreshold();
	}

	return nbCreatedNodes;
}

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"

int nb_nodes=0;

//((pathDelta < 0.03 || (pathDelta > 0.8 && path.length() < step))))
//      ((pathDelta < 0.1 || (
//  path.length() < step && pathDelta > 0.8 && !(expMethod == Env::nExtend &&
//						 extendStep != 1)))))// && p3d_GetIsExpandControl() && !directionNode && expandControl(N.mN)))

bool RRT::nextStepConfig(Localpath& path, shared_ptr<Configuration>& directionConfig, double& pathDelta,
		shared_ptr<Localpath>& newPath, Env::expansionMethod method)
{

	if(method == Env::connect)
	{
		cout << "Error : Method Doesn't work with connect" << endl;
	}
	else
	{
		pathDelta = path.length() == 0. ? 1. : MIN(1., this->step() / path.length());

		newPath = shared_ptr<Localpath>(
				new Localpath(mG, path.mBegin,
						pathDelta == 1. && directionConfig ?
						directionConfig :
						path.confAtParam(pathDelta * path.path()->range_param)));
	}

	return(newPath->valid());
}

/**
 * ExpandProcess
 *  General function expanding a node toward a direction
 * of expansion. The mode of expansion depends of the
 * expansion choice selected.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] ExpansionNodePt: the node to expand
 * @param[In] DirectionConfig: the direction of expansion
 * @return: the number of nodes created during the expansion
 * process (several nodes can be created during one expansion)
 */
bool RRT::ExpandToGoal(Node& expansionNode,
		shared_ptr<Configuration> directionConfig, Node* directionNode,
		Env::expansionMethod method) {

	bool extensionSucceeded(true);
	shared_ptr<Localpath> extensionLocalpath;
	shared_ptr<Localpath> directionLocalpath;
	double positionAlongDirection(0.);
	double temperature = expansionNode.getComp()->temperature;
	double extensionCost(0.);
	shared_ptr<Configuration> fromConfig = expansionNode.getConfSP();
	shared_ptr<Configuration> toConfig;

	if (method != Env::nExtend ) return false;
	//cout << "NEW TEST !!!" << endl;

	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	while ( extensionSucceeded && positionAlongDirection < 1.) {

		directionLocalpath = shared_ptr<Localpath>(new Localpath(mG,
						fromConfig,directionConfig));

		extensionSucceeded = nextStepConfig(*directionLocalpath, directionConfig,
				positionAlongDirection, extensionLocalpath, method);

		toConfig = extensionLocalpath->mEnd;

		//cout << "Extension succeeded : " <<extensionSucceeded << endl;

		// Transition test for cost spaces, increase temperature in case of failure
		if (extensionSucceeded && p3d_GetIsCostFuncSpace()) {

			extensionCost = toConfig->cost();
			//cout << "extensionCost = " << extensionCost << endl;
			if ( costTestSucceededConf(fromConfig,toConfig,temperature)) {
				if ( extensionCost > fromConfig->cost() ) {
					temperature = adjustTemperature(true,temperature);
					//cout << "extensionCost > fromConfig->cost(), temperature ="<< temperature<< endl;
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

bool RRT::ExpandCostConnect(Node& expansionNode,
	     shared_ptr<Configuration> directionConfig,
	     Node* directionNode,
	     Env::expansionMethod method,
	     bool toGoal) {

  bool extensionSucceeded(false);
  bool failed(false);

  int nbCreatedNodes(0);

  double positionAlongDirection(0.);
  shared_ptr<Localpath> directionLP;
  shared_ptr<Localpath> extensionLP;

  shared_ptr<Configuration> fromConfig = expansionNode.getConfSP();
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
	directionLP = shared_ptr<Localpath>(
			new Localpath(
					mG,
					fromConfig,
					directionConfig));

	extensionSucceeded = nextStepConfig(
			*directionLP,
			directionConfig,
			positionAlongDirection,
			extensionLP,
			method);

	nb_of_extend++;

	failed |= !extensionSucceeded;

	toConfig = extensionLP->mEnd;

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
			!expandControl(*directionLP, 1.0 , expansionNode))
		{
			failed = true;
		}

		// Cost Test Control
		if((!failed) && (extensionCost > prevCost))
		{
			upHill = true;
			if(!costTestSucceeded(&expansionNode, *toConfig, extensionCost))
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
			expansionFailed(expansionNode);
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

	  directionLP = shared_ptr<Localpath>(
	  			new Localpath(
	  					mG,
	  					expansionNode.getConfSP(),
	  					directionConfig));

	  extensionLP = shared_ptr<Localpath>(
	  	  			new Localpath(
	  	  				mG,
	  	  				expansionNode.getConfSP(),
	  	  				fromConfig));

	  extensionCost = fromConfig->cost();

	  double length = extensionLP->length();
	  expansionNode.getComp()->sumLengthEdges += length;

	  double positionAlongDirection =
		  directionLP->length() == 0. ? 1. : MIN(1., step() / directionLP->length());
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

  if(p3d_GetIsCostFuncSpace() && (p3d_GetCostMethodChoice() == MAXIMAL_THRESHOLD)) {
	p3d_updateCostThreshold();
  }

  return nbCreatedNodes;
}

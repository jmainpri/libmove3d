#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

extern double InitCostThreshold;

bool RRT::expandControl(Localpath& path, double positionAlongDirection, Node& compNode)
{
	double radius=0;

	if(!ENV.getBool(Env::useRefiRadius))
	{
		if( ENV.getExpansionMethod() == Env::extend)
		{
			radius = step();
		}
		if(ENV.getExpansionMethod() == Env::costConnect)
		{
			radius = compNode.getComp()->sumLengthEdges / (compNode.getComp()->nnode-1);
		}
	}
	else
	{
		radius = ENV.getDouble(Env::refiRadius);
	}

	if(ENV.getBool(Env::printRadius)){
		cout << "radius = " << radius << endl;
		cout << "path.length() = " << path.length() << endl;
//		cout << "TEST?= " << ((path.length() <= radius)&&positionAlongDirection >= 1.) << endl;
		cout << "Average length = " << compNode.getComp()->sumLengthEdges / (compNode.getComp()->nnode-1) << endl;
		double ratio =  (double)compNode.getComp()->nbRefinNodes / (double)compNode.getComp()->nnode;
		cout << "ratio of RNODES = " << ratio << endl;
		cout << endl;
	}

	if( path.length() <= radius ) // || extensionLocalpath->length() < 0.01 * path->length(); //extensionLocalpath->length() <= this->step();
		{
		if(compNode.getComp()->nbRefinNodes*50 > compNode.getComp()->nnode)
			return(false);
		else
		{
			compNode.getComp()->nbRefinNodes++;
			}
		}
	return(true);
}

void RRT::expansionFailed(Node& node) {
  if(p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  {
    p3d_SetNGood(0);
  }
  node.mN->n_fail_extend++;
  if((ENV.getBool(Env::discardNodes)) &&
     (node.mN != mG->getGraph()->search_start) &&
     (node.mN != mG->getGraph()->search_goal) &&
     (node.mN->n_fail_extend > p3d_GetMaxExpandNodeFail())) {
    node.mN->IsDiscarded = true;
    update_parent_nfails(node.mN);
    mG->getGraph()->n_consec_pb_level ++;
    mG->getGraph()->n_consec_fail_pb_level ++;
  }
}

    //((pathDelta < 0.03 || (pathDelta > 0.8 && path.length() < step))))
      //      ((pathDelta < 0.1 || (
      //  path.length() < step && pathDelta > 0.8 && !(expMethod == Env::nExtend &&
    //						 extendStep != 1)))))// && p3d_GetIsExpandControl() && !directionNode && expandControl(N.mN)))

bool RRT::nextStep(Localpath& path, Node* directionNode, double& pathDelta, shared_ptr<Localpath>& newPath, Env::expansionMethod method)
{

  if(method == Env::connect)
  {
    // create path satisfying connect method
    newPath = shared_ptr<Localpath>(new Localpath(path, pathDelta));
    if(pathDelta == 0.)
    { return(false); }
  }
  else
  {
    pathDelta = path.length() == 0. ? 1. : MIN(1., this->step() / path.length());

    newPath = shared_ptr<Localpath>(
      new Localpath(mG, path.mBegin,
		    pathDelta == 1. && directionNode ?
		    directionNode->getConfSP() :
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
int RRT::ExpandProcess(Node& expansionNode,
			     shared_ptr<Configuration> directionConfig,
			     Node* directionNode,
			     Env::expansionMethod method) {

  if(method == Env::costConnect){
	  return ExpandCostConnect(expansionNode,
		 directionConfig,
		 directionNode,
		 method,
		 false);
  }

  bool extensionSucceeded(false);
  bool failed(false);
  int nbCreatedNodes(0);
  Node* fromNode(&expansionNode);
  Node* extensionNode(NULL);
  shared_ptr<Localpath> directionLocalpath;
  double positionAlongDirection(0.);
  shared_ptr<Localpath> extensionLocalpath;
  double extensionCost(0.);
  bool firstIteration(true);

  // Perform extension toward directionConfig
  // Additional nodes creation in the nExtend case, but without checking for expansion control
  while(firstIteration ||
	(method == Env::nExtend && !failed && positionAlongDirection < 1.))
  {
    directionLocalpath = shared_ptr<Localpath>(new Localpath(mG, fromNode->getConfSP(), directionConfig));
    extensionSucceeded = this->nextStep(*directionLocalpath, directionNode, positionAlongDirection, extensionLocalpath, method);

    failed |= !extensionSucceeded;

    if(failed){
	int nbCollFail = ENV.getInt(Env::nbCollExpanFailed);
	nbCollFail++;
	ENV.setInt(Env::nbCollExpanFailed,nbCollFail);
	if(ENV.getBool(Env::printCollFail)){
		cout << "nbCollFail = " << nbCollFail << endl;
	}
    }

    // Transition test for cost spaces, increase temperature in case of failure
    if(!failed && p3d_GetIsCostFuncSpace())
    {
      extensionCost = extensionLocalpath->mEnd->cost();
      if(!this->costTestSucceeded(fromNode, *extensionLocalpath->mEnd, extensionCost))
      {
	this->adjustTemperature(false, fromNode);
	failed = true;

	int nbCostFail = ENV.getInt(Env::nbCostTransFailed);
	nbCostFail++;
	ENV.setInt(Env::nbCostTransFailed,nbCostFail);
	if(ENV.getBool(Env::printCostFail))
		cout << "nbCostFail = " << nbCostFail << endl;
      }
    }
    // Expansion Control
    if(firstIteration && !failed)
    {
      if(ENV.getBool(Env::expandControl) &&
    		  !expandControl(*directionLocalpath, positionAlongDirection, expansionNode))
    	  failed = true;
    }
    // Add node to graph if everything succeeded
    if(!failed)
    {
      extensionNode = this->connectNode(fromNode, *extensionLocalpath, positionAlongDirection, directionNode, extensionCost, nbCreatedNodes);
      // In cost space, decrease temperature if the accepted cost is higher
      if(p3d_GetIsCostFuncSpace() && extensionCost > fromNode->getCost())
    	  this->adjustTemperature(true, extensionNode);

      // costTemp
      fprintf(traj_file, "%d, %5.6f, %5.20f\n", extensionNode->mN->num, extensionCost , extensionNode->getComp()->temperature);

    }
    if(firstIteration && failed)
      this->expansionFailed(expansionNode);

    fromNode = extensionNode;
    firstIteration = false;
  }

  if(p3d_GetIsCostFuncSpace() && (p3d_GetCostMethodChoice() == MAXIMAL_THRESHOLD)) {
    p3d_updateCostThreshold();
  }

  return nbCreatedNodes;
}

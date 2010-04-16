/*
 * BaseExpansion.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */


#include "BaseExpansion.h"

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

BaseExpansion::BaseExpansion() :
	ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
	MaxExpandNodeFailure(10),
	kNearestPercent(10),
	ExpansionDirectionMethod(GLOBAL_CS_EXP),
	IsDirSampleWithRlg(false)
	{ cout << "no graph in expansion method" << endl; }

BaseExpansion::BaseExpansion(Graph* ptrGraph) :

	ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
	MaxExpandNodeFailure(10),
	kNearestPercent(10),
	ExpansionDirectionMethod(GLOBAL_CS_EXP),
	IsDirSampleWithRlg(false),
	mGraph(ptrGraph) {}

BaseExpansion::~BaseExpansion(){}

double BaseExpansion::step(){
	 return(p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep));
}

bool BaseExpansion::expandControl(LocalPath& path, double positionAlongDirection, Node& compNode)
{
	double radius=0;

	if(!ENV.getBool(Env::useRefiRadius))
	{
		if( ENV.getExpansionMethod() == Env::Extend || ENV.getExpansionMethod() == Env::costConnect)
		{
			radius = step();
		}
//		if(ENV.getExpansionMethod() == Env::costConnect)
//		{
//			radius = compNode.getComp()->sumLengthEdges / (compNode.getComp()->nnode-1);
//		}
	}
	else
	{
		radius = ENV.getDouble(Env::refiRadius);
	}

	if(ENV.getBool(Env::printRadius) ){
		cout << "radius = " << radius << endl;
                cout << "path.length() = " << path.getParamMax() << endl;
		//		cout << "TEST?= " << ((path.length() <= radius)&&positionAlongDirection >= 1.) << endl;
		/**
		 *
		 * ATTENTION TODO Refinement radius mean value
		 *
		 */
//		cout << "Average length = " << compNode.getCompcoStruct()->sumLengthEdges / (compNode.getCompcoStruct()->nnode-1) << endl;
		double ratio =  (double)compNode.getCompcoStruct()->nbRefinNodes / (double)compNode.getCompcoStruct()->nnode;
		cout << "ratio of RNODES = " << ratio << endl;
		cout << endl;
	}

        if( path.getParamMax() <= radius ) // || extensionLocalpath->length() < 0.01 * path->length(); //extensionLocalpath->length() <= this->step();
	{
		if(compNode.getCompcoStruct()->nbRefinNodes*2 > compNode.getCompcoStruct()->nnode)
		{
			return(false);
		}
		else
		{
			compNode.getCompcoStruct()->nbRefinNodes++;
		}
	}
	return(true);
}

void BaseExpansion::expansionFailed(Node& node) {

	if(ExpansionNodeMethod == RANDOM_IN_SHELL_METH)  {
		p3d_SetNGood(0);
	}

	node.getNodeStruct()->n_fail_extend++;

	if((ENV.getBool(Env::discardNodes)) &&
			(node.getNodeStruct() != mGraph->getGraphStruct()->search_start) &&
			(node.getNodeStruct() != mGraph->getGraphStruct()->search_goal) &&
			(node.getNodeStruct()->n_fail_extend > MaxExpandNodeFailure)) {

		node.getNodeStruct()->IsDiscarded = true;
		update_parent_nfails(node.getNodeStruct());

		mGraph->getGraphStruct()->n_consec_pb_level ++;
		mGraph->getGraphStruct()->n_consec_fail_pb_level ++;
	}
}

Node* BaseExpansion::addNode(Node* currentNode, LocalPath& path, double pathDelta,
		Node* directionNode, int& nbCreatedNodes)
{
	if ((pathDelta == 1. && directionNode))
	{
		cout << "MergeComp" << endl;
		mGraph->MergeComp(currentNode, directionNode, path.getParamMax());
		return (directionNode);
	}
	else
	{
		Node* newNode = mGraph->insertNode( currentNode, path );

		nbCreatedNodes++;
		return (newNode);
	}
}

//((pathDelta < 0.03 || (pathDelta > 0.8 && path.length() < step))))
//      ((pathDelta < 0.1 || (
//  path.length() < step && pathDelta > 0.8 && !(expMethod == Env::nExtend &&
//						 extendStep != 1)))))// && p3d_GetIsExpandControl() && !directionNode && expandControl(N.mN)))

/**
 * Gives successive co
 */
bool BaseExpansion::nextStep(LocalPath& path,
		Node* directionNode,
		double& pathDelta,
		shared_ptr<LocalPath>& newPath,
		Env::expansionMethod method)
{

	if( method == Env::Connect )
	{
		// create path satisfying connect method
		newPath = shared_ptr<LocalPath>(new LocalPath(path, pathDelta));
		if(pathDelta == 0.)
		{ return(false); }
	}
	else
	{
                pathDelta = path.getParamMax() == 0. ? 1. : MIN(1., step() / path.getParamMax());

		shared_ptr<Configuration> ptrEnd;

		if(pathDelta == 1. && directionNode)
		{
			ptrEnd = directionNode->getConfiguration();
		}
		else
		{
			ptrEnd = path.configAtParam(pathDelta * path.getLocalpathStruct()->range_param);
		}

		ptrEnd->setConstraints();

		newPath = shared_ptr<LocalPath>(new LocalPath(path.getBegin(),ptrEnd));
	}

	return(newPath->getValid());
}

bool BaseExpansion::nextStep(LocalPath& path,
		shared_ptr<Configuration>& directionConfig,
		double& pathDelta,
		shared_ptr<LocalPath>& newPath,
		Env::expansionMethod method)
{

	if(method == Env::Connect)
	{
		cout << "Error : Method Doesn't work with connect" << endl;
	}
	else
	{
                pathDelta = path.getParamMax() <= 0. ? 1. : MIN(1., this->step() / path.getParamMax());

		newPath = shared_ptr<LocalPath>(
				new LocalPath(path.getBegin(),
						pathDelta == 1. && directionConfig ?
								directionConfig :
				path.configAtParam(pathDelta * path.getLocalpathStruct()->range_param)));
	}

	return(newPath->getValid());
}

/*
 * TreeExpansion.cpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */

#include "BaseExpansionMethod.hpp"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;

bool BaseExpansionMethod::expandControl(Localpath& path, double positionAlongDirection, Node& compNode)
{
	double radius=0;

	if(!ENV.getBool(Env::useRefiRadius))
	{
		if( ENV.getExpansionMethod() == Env::extend || ENV.getExpansionMethod() == Env::costConnect)
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
		if(compNode.getComp()->nbRefinNodes*2 > compNode.getComp()->nnode)
		{
			return(false);
		}
		else
		{
			compNode.getComp()->nbRefinNodes++;
		}
	}
	return(true);
}

void BaseExpansionMethod::expansionFailed(Node& node) {

	if(ExpansionNodeMethod == RANDOM_IN_SHELL_METH)  {
		p3d_SetNGood(0);
	}

	node.mN->n_fail_extend++;

	if((ENV.getBool(Env::discardNodes)) &&
			(node.mN != mGraph->getGraph()->search_start) &&
			(node.mN != mGraph->getGraph()->search_goal) &&
			(node.mN->n_fail_extend > MaxExpandNodeFailure)) {

		node.mN->IsDiscarded = true;
		update_parent_nfails(node.mN);

		mGraph->getGraph()->n_consec_pb_level ++;
		mGraph->getGraph()->n_consec_fail_pb_level ++;
	}
}

//((pathDelta < 0.03 || (pathDelta > 0.8 && path.length() < step))))
//      ((pathDelta < 0.1 || (
//  path.length() < step && pathDelta > 0.8 && !(expMethod == Env::nExtend &&
//						 extendStep != 1)))))// && p3d_GetIsExpandControl() && !directionNode && expandControl(N.mN)))

bool BaseExpansionMethod::nextStep(Localpath& path, Node* directionNode,
		double& pathDelta,
		shared_ptr<Localpath>& newPath,
		Env::expansionMethod method)
{

	if( method == Env::connect )
	{
		// create path satisfying connect method
		newPath = shared_ptr<Localpath>(new Localpath(path, pathDelta));
		if(pathDelta == 0.)
		{ return(false); }
	}
	else
	{
		pathDelta = path.length() == 0. ? 1. : MIN(1., step() / path.length() );

		newPath = shared_ptr<Localpath>(
				new Localpath(path.mBegin,
						pathDelta == 1. && directionNode ?
								directionNode->getConfSP() :
									path.confAtParam(pathDelta * path.path()->range_param)));
	}

	return(newPath->valid());
}

bool BaseExpansionMethod::nextStep(Localpath& path,
		shared_ptr<Configuration>& directionConfig, double& pathDelta,
		shared_ptr<Localpath>& newPath, Env::expansionMethod method)
{

	if(method == Env::connect)
	{
		cout << "Error : Method Doesn't work with connect" << endl;
	}
	else
	{
		pathDelta = path.length() <= 0. ? 1. : MIN(1., this->step() / path.length());

		newPath = shared_ptr<Localpath>(
				new Localpath(path.mBegin,
						pathDelta == 1. && directionConfig ?
								directionConfig :
				path.confAtParam(pathDelta * path.path()->range_param)));
	}

	return(newPath->valid());
}

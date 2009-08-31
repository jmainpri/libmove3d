/*
 * TreeExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_TREE_EXPANSION_HPP
#define P3D_TREE_EXPANSION_HPP

#include "../../PlanningAPI/planningAPI.hpp"
#include "../../qtWindow/qtBase/env.hpp"
#include "BaseExpansionMethod.hpp"

class TreeExpansionMethod : public BaseExpansionMethod {

public:

	TreeExpansionMethod();
	TreeExpansionMethod(Graph* prtGraph);

	~TreeExpansionMethod();

	std::tr1::shared_ptr<Configuration> getExpansionDirection(Node* fromComp,
			Node* toComp,
			bool samplePassive,
			Node*& directionNode);

	std::tr1::shared_ptr<Configuration> selectExpansionDirection(Node* expandComp,
						Node* goalComp,
						bool samplePassive,
						Node*& directionNode);

	Node* getExpansionNode(Node* compNode,
			std::tr1::shared_ptr<Configuration> direction,
			int distance);

	Node* selectExpansionNode(Node* compNode,
			std::tr1::shared_ptr<Configuration> direction,int distance);

};

#endif

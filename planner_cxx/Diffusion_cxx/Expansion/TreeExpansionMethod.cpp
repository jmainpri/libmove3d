/*
 * TreeExpansionMethod.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */


#include "P3d-pkg.h"
#include <iostream>

#include "TreeExpansionMethod.hpp"

using namespace std;

TreeExpansionMethod::TreeExpansionMethod() :
	BaseExpansionMethod() {
}

TreeExpansionMethod::TreeExpansionMethod(Graph* ptrGraph) :
	BaseExpansionMethod(ptrGraph) {
}

TreeExpansionMethod::~TreeExpansionMethod(){
}

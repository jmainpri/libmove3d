/*
 * cppToQt.hpp
 *
 *  Created on: Aug 19, 2009
 *      Author: jmainpri
 */

#ifndef CPPTOQT_HPP_
#define CPPTOQT_HPP_
#include "../qtWindow/qtLibrary.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "UserAppli-pkg.h"
#include "Bio-pkg.h"
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
#endif

#ifdef CXX_PLANNER
#include "../planner_cxx/plannerFunctions.hpp"
#include "../planner_cxx/API/Trajectory/BaseOptimization.hpp"
#include "../planner_cxx/API/Trajectory/CostOptimization.hpp"
#include "../planner_cxx/Greedy/GreedyCost.hpp"
#include "../planner_cxx/API/Search/Dijkstra/dijkstra.hpp"
	#ifdef HRI_PLANNER
	#include "Hri_planner-pkg.h"
	#include "../planner_cxx/HRI_CostSpace/HRICS_HAMP.h"
	#endif
#endif

#ifdef HRI_COSTSPACE
#include "../planner_cxx/HRI_CostSpace/HRICS_Planner.h"
#endif

#ifdef QT_GL
extern G3D_Window *G3D_WIN;
extern QSemaphore* sem;
#endif

void read_pipe(int fd, void* data);

extern int qt_fl_pipe[2];

#endif /* CPPTOQT_HPP_ */

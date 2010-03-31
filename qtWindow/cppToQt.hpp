/*
 * cppToQt.hpp
 *
 *  Created on: Aug 19, 2009
 *      Author: jmainpri
 */

#ifndef CPPTOQT_HPP_
#define CPPTOQT_HPP_
#include "../qtWindow/qtLibrary.h"

#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
#endif

#ifdef CXX_PLANNER
#include "../planner_cxx/plannerFunctions.hpp"
#include "../planner_cxx/API/Trajectory/Smoothing.hpp"
#include "../planner_cxx/API/Trajectory/CostOptimization.hpp"
#include "../planner_cxx/Greedy/GreedyCost.hpp"
#include "../planner_cxx/API/Search/Dijkstra/dijkstra.hpp"
#include "../util/CppApi/MultiRun.hpp"
#endif

#ifdef HRI_COSTSPACE
#include "../planner_cxx/HRI_CostSpace/HRICS_Planner.h"
#endif

#ifdef HRI_PLANNER
#include "../planner_cxx/HRI_CostSpace/HRICS_HAMP.h"
#endif

#include "Graphic-pkg.h"

#ifdef QT_GL
extern G3D_Window *G3D_WIN;
extern QSemaphore* sem;
#endif

#ifndef WITH_XFORMS
void qt_resetGraph();
void qt_drawAllWinActive();
void qt_runDiffusion();
void qt_runPRM();
void qt_shortCut();
void qt_readScenario();
#endif

  /**
    * @ingroup qtWindow
    * @brief Function details the pipe between the XForm thread and the Qt Interface thread
    */
void read_pipe(int fd, void* data);
extern int qt_fl_pipe[2];
extern const char *qt_fileName;

#endif /* CPPTOQT_HPP_ */

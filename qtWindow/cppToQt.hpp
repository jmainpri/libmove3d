/*
 * cppToQt.hpp
 *
 *  Created on: Aug 19, 2009
 *      Author: jmainpri
 */

#ifndef CPPTOQT_HPP_
#define CPPTOQT_HPP_

#include "qtLibrary.h"

#ifdef ENERGY
#include "bio/BioEnergy/include/Energy-pkg.h"
#endif

#ifdef CXX_PLANNER
#include "plannerFunctions.hpp"
#include "API/Trajectory/smoothing.hpp"
#include "API/Trajectory/costOptimization.hpp"
//#include "planner_cxx/Greedy/GreedyCost.hpp"
#include "API/Search/Dijkstra/dijkstra.hpp"
#include "util/CppApi/MultiRun.hpp"
#endif

#ifdef HRI_COSTSPACE
#include "HRI_CostSpace/HRICS_Workspace.h"
#endif

#if defined( HRI_PLANNER ) && defined( HRI_COSTSPACE )
#include "HRI_CostSpace/HRICS_HAMP.h"
#endif

#include "Graphic-pkg.h"

#ifdef QT_GL
extern G3D_Window *G3D_WIN;
extern QSemaphore* sem;
#endif

void qt_resetGraph();
void qt_drawAllWinActive();
void qt_runDiffusion();
void qt_runPRM();
void qt_shortCut();
void qt_optimize();
void qt_readScenario();
void qt_saveScenario();
void qt_readTraj();
#ifdef HRI_COSTSPACE
void qt_load_HRICS_Grid(std::string gridName);
#endif

  /**
    * @ingroup qtWindow
    * @brief Function details the pipe between the XForm thread and the Qt Interface thread
    */
void read_pipe(int fd, void* data);
extern int qt_fl_pipe[2];
extern const char *qt_fileName;

#endif /* CPPTOQT_HPP_ */

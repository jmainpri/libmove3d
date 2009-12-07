//#ifndef PLANNING_API_CPP_
//#define PLANNING_API_CPP_

/**
 * C++ basic headers (they all have name spaces)
 */
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <tr1/memory>

/**
 * Environment has to be included before anything (weird)
 */
#include "../p3d/env.hpp"
/**
 * Basic move3d modules
 */
#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "GroundHeight-pkg.h"
#include "Util-pkg.h"

/**
 * The CPP API so that
 * Robot is first and Graph is last (kind of tricky because its backwards)
 */

#include "3DGrid/grid.h"
#include "Search/AStar/AStar.h"
#include "Search/Dijkstra/dijkstra.hpp"
//#include "Trajectory/CostOptimization.hpp"
//#include "Trajectory/BaseOptimization.hpp"
#include "Trajectory/trajectory.hpp"

#include "Roadmap/graph.hpp"
#include "Roadmap/edge.hpp"
#include "Roadmap/node.hpp"
#include "ConfigSpace/localpath.hpp"
#include "ConfigSpace/configuration.hpp"
#include "Device/robot.hpp"

#include "scene.h"
#include "project.h"

//#endif








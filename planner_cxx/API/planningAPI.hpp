#ifndef PLANNING_API_HPP_
#define PLANNING_API_HPP_

/**
 * C++ basic headers (they all have name spaces)
 */
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <string>
//#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <tr1/memory>

#include "P3d-pkg.h"

/**
 * Environment has to be included before anything (weird)
 */
#include "../p3d/env.hpp"

/**
 * The CPP API so that
 * Robot is first and Graph is last (kind of tricky because its backwards)
 */

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry> 

// Configuration has no dependencies and is used by most other classes
#include "ConfigSpace/configuration.hpp"
#include "ConfigSpace/localpath.hpp"

// Dependency to robot
#include "Device/robot.hpp"

// Graph, Node, Edge, Robot and Localpath have no interdependencies
#include "Roadmap/node.hpp"
#include "Roadmap/edge.hpp"
#include "Roadmap/compco.h"
#include "Roadmap/graph.hpp"

// LocalPathValidTest inherits LocalPath, include it after localpath.hpp
#include "ConfigSpace/localPathValidTest.h"

#include "Search/AStar/AStar.h"
#include "Search/Dijkstra/dijkstra.hpp"

//#include "Trajectory/CostOptimization.hpp"
//#include "Trajectory/Smoothing.hpp"
#include "Trajectory/trajectory.hpp"

#include "scene.h"
#include "project.h"

#endif

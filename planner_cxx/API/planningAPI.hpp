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
USING_PART_OF_NAMESPACE_EIGEN
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>

class Graph;
class Edge;
class Node;
class Robot;
class LocalPath;

// Configuration has no dependencies and is used by most other classes
#include "ConfigSpace/configuration.hpp"

// Graph, Node, Edge, Robot and Localpath have no interdependencies
#include "Roadmap/graph.hpp"
#include "Roadmap/node.hpp"
#include "Device/robot.hpp"
#include "Roadmap/edge.hpp"
#include "ConfigSpace/localpath.hpp"
// LocalPathValidTest inherits LocalPath, include it after localpath.hpp
#include "ConfigSpace/localPathValidTest.h"

#include "Grids/TwoDGrid.hpp"
#include "Grids/ThreeDGrid.h"
#include "Grids/ThreeDPoints.h"
#include "Search/AStar/AStar.h"
#include "Search/Dijkstra/dijkstra.hpp"
//#include "Trajectory/CostOptimization.hpp"
//#include "Trajectory/Smoothing.hpp"
#include "Trajectory/trajectory.hpp"

#include "scene.h"
#include "project.h"

#endif

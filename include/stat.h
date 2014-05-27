/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#ifndef _STAT_H
#define _STAT_H

extern int STAT;

/**
 * Data structure to store statistics on planning algorithms
 *
 */
typedef struct p3d_statistics {
  int loopNum;      // Number of loops
  double preTime;    // Pre-planning time
  double planTime;    // Planning Time
  int planConfTot;    // Total Number of Configurations
  int planConfCol;    // Number of conf in collision
  int planConfFree;    // Number of conf free
  int planConfAdd;    // Number of conf added
  int planConfGuard;   // Number of guardian
  int planConfConn;    // Number of connectors
  int planEdge;       // Number of edges in the graph
  int planCycle;      // Number of cycles in the graph
  double cyclingTime;  // Time spent to construct cycles
  int planNeigCall;    // Number of call to neigh function
  int planLpNum;    // Number of computed local paths
  double planLpLenght;   // Local path length
  int planLpColNum;    // Number of local path collision test
  double planLpStepSize;  // Local path step size
  double postTime;    // Post planning time
  int colNum;     // Number of collisions
  int lenLpNumBeforePost;  // Number of local path before post processing
  int lenLpNumAfterPost;  // Number of local path after post processing
  double lenPathBeforePost;  // Path length before postProcessing
  double lenPathAfterPost;  // Path length after postProcessing
  unsigned long memory;   // Memory usage
#ifdef MULTIGRAPH
  double mgTime;
  int mgNodes;
  int mgEdges;
#endif
}p3d_stat;

#endif /* #ifndef _STAT_H */

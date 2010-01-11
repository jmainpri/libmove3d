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

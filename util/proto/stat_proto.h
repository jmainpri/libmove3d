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
#ifndef __CEXTRACT__

#include "P3d-pkg.h"
// Function of the stats module; Commit Jim; date: 01/10/2008

/**
 * enableStats
 *
 * Enables the retrieval of
 * statistical information on the algorithm
 */
extern void enableStats(void);

/**
 * disableStats
 *
 * Disables the retrieval of
 * statistical information on the algorithm
 */
extern void disableStats(void);

/**
 * getStatStatus
 *
 * Get if the statistics retrieval
 * is local enabled
 * @return: TRUE if the module is enabled
 */
int getStatStatus(void);

/**
 * createStat
 *
 * Allocates a statistic data structure
 * @return: the structure
 */
p3d_stat * createStat(void);

/**
 * destroyStat
 *
 * Frees a statistic data structure
 * @return: the structure
 */
extern void destroyStat(p3d_stat ** s);

/**
 * initStats
 *
 * Resets all fields of the stat data structure to
 */
extern void initStats(p3d_stat * s);

/**
 * setTotalCountVar
 *
 * Copy all graph data into the stat data structure
 */
extern void setTotalCountVar(p3d_graph * graph);

/**
 * setTotalCountVar
 *
 * Merge tow data structures by adding the src to the dest
 */
extern void mergeStat(p3d_stat * src, p3d_stat * dest);

/**
 * getPathStat
 *
 * Gets path statistics and sets it to the stat data structure
 */
extern void getPathStat(p3d_stat * s, p3d_traj * path, int beforePost);

/**
 * printStatsGraph
 *
 * Prints the statistics from a stat structure
 * not all printed statistics are taken from the structure
 */
extern void printStatsGraph(p3d_stat * s, int Print);

/**
 * printStatsEnv
 *
 * Same as the printStatgraph
 * it sets a Average to note that the statistics are global
 * Meant to be printed after several runs of the same algorithm
 */
extern void printStatsEnv(p3d_stat * s, int Print);

/**
 * addStatsToFile
 *
 * Same as the printStatgraph to a file
 * in Comma-separated values (CSV)
 */
extern void addStatToFile(p3d_stat * s , FILE* Stat_output );

/**
 * openStatsFile
 */
extern FILE* openStatFile(FILE* Stat_output , char* s );

/**
 * closeStatsFile
 */
extern void closeStatFile(FILE* Stat_output );

/**
 * saveInStatToFile
 */
extern void saveInStatFile();

#endif /* __CEXTRACT__ */

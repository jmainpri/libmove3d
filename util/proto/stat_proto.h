#ifndef __CEXTRACT__
#include "Planner-pkg.h"

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

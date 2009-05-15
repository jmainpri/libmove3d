#include "Util-pkg.h" 


/**
 * Flag to set if the statistics module
 * is activated or not
 */
int STAT = FALSE;

/**
 * enableStats
 *
 * 
 * Enables the retrieval of
 * statistical information on the algorithm
 */
void enableStats(void){
  STAT = TRUE;
}

/**
 * disableStats
 *
 * Disables the retrieval of
 * statistical information on the algorithm
 */
void disableStats(void){
  STAT = FALSE;
}

/**
 * getStatStatus
 *
 * Get if the statistics retrieval
 * is local enabled
 * @return: TRUE if the module is enabled
 */
int getStatStatus(void){
  return STAT;
}

/**
 * createStat
 *
 * Allocates a statistic data structure
 * @return: the structure
 */
p3d_stat * createStat(void){
  p3d_stat * s = MY_ALLOC(p3d_stat, 1);
  initStats(s);
  return s;
}

/**
 * destroyStat
 *
 * Frees a statistic data structure
 * @return: the structure
 */
void destroyStat(p3d_stat ** s){
  MY_FREE(*s, p3d_stat, 1);
  *s = NULL;
}

/**
 * initStats
 *
 * Resets all fields of the stat data structure to
 */
void initStats(p3d_stat * s){
  if(s){
    s->loopNum = 0;
    s->preTime = 0;
    s->planTime = 0;
    s->planConfTot = 0;
    s->planConfCol = 0;
    s->planConfFree = 0;
    s->planConfAdd = 0;
    s->planConfGuard = 0;
    s->planConfConn = 0;
    s->planNeigCall = 0;
    s->planEdge = 0;
    s->planCycle = 0;
    s->cyclingTime = 0;
    s->planLpNum = 0;
    s->planLpLenght = 0;
    s->planLpColNum = 0;
    s->planLpStepSize = 0;
    s->postTime = 0;
    s->colNum = 0;
    s->lenLpNumBeforePost = 0;
    s->lenLpNumAfterPost = 0;
    s->lenPathBeforePost = 0;
    s->lenPathAfterPost = 0;
    s->memory = 0;
#ifdef MULTIGRAPH
    s->mgTime = 0;
    s->mgNodes = 0;
    s->mgEdges = 0;
#endif
  }
  else PrintInfo(("Warning: statistic data structure not allocated!\n"));
}

/**
 * setTotalCountVar
 *
 * Copy all graph data into the stat data structure
 */
void setTotalCountVar(p3d_graph * graph){
  if(graph){
    p3d_stat * s = graph->stat;
    if(s){
      s->loopNum++;
      s->planConfTot = graph->nb_q;
      s->planConfFree = graph->nb_q_free;
      s->planConfCol = graph->nb_q - graph->nb_q_free;
      s->planConfAdd = graph->nnode;
      s->planEdge = graph->nedge;
      if(!graph->oriented){
        s->planEdge /= 2;
      }
      s->planCycle = s->planEdge - graph->nnode + graph->ncomp;
      s->planLpNum = graph->nb_local_call;
      s->colNum = graph->nb_test_coll;
      s->planLpStepSize += p3d_get_DMAX();
      s->memory = basic_alloc_get_size();
#ifdef MULTIGRAPH
    s->mgTime = graph->mgTime;
#endif
    }
    else PrintInfo(("Warning: statistic data structure not allocated!\n"));
  }
  else PrintInfo(("Warning: graph data structure not allocated!\n"));
}

/**
 * setTotalCountVar
 *
 * Merge tow data structures by adding the src to the dest
 */
void mergeStat(p3d_stat * src, p3d_stat * dest){
  if(dest && src){
    dest->loopNum += src->loopNum;
    dest->preTime += src->preTime;
    dest->planTime += src->planTime;
    dest->planConfTot += src->planConfTot;
    dest->planConfCol += src->planConfCol;
    dest->planConfFree += src->planConfFree;
    dest->planConfAdd += src->planConfAdd;
    dest->planConfGuard += src->planConfGuard;
    dest->planConfConn += src->planConfConn;
    dest->planNeigCall += src->planNeigCall;
    dest->planEdge += src->planEdge;
    dest->planCycle += src->planCycle;
    dest->cyclingTime += src->cyclingTime;
    dest->planLpNum += src->planLpNum;
    dest->planLpLenght += src->planLpLenght;
    dest->planLpColNum += src->planLpColNum;
    dest->planLpStepSize += src->planLpStepSize;
    dest->postTime += src->postTime;
    dest->colNum += src->colNum;
    dest->lenLpNumBeforePost += src->lenLpNumBeforePost;
    dest->lenLpNumAfterPost += src->lenLpNumAfterPost;
    dest->lenPathBeforePost += src->lenPathBeforePost;
    dest->lenPathAfterPost += src->lenPathAfterPost;
    dest->memory += src->memory;
#ifdef MULTIGRAPH
    dest->mgTime += src->mgTime;
    dest->mgNodes += src->mgNodes;
    dest->mgEdges += src->mgEdges;
#endif
  }
}


/**
 * getPathStat
 *
 * Gets path statistics and sets it to the stat data structure
 */
void getPathStat(p3d_stat * s, p3d_traj * path, int beforePost){
  if(path){
    p3d_localpath * lp = path->courbePt;
    int nbLp = 0;
    double pathLen = 0.0;
  
    for(nbLp = 0; lp; lp = lp->next_lp, nbLp++){
      pathLen += lp->length_lp;
    }
  
    if(s){
      if(beforePost){
        s->lenLpNumBeforePost += nbLp;
        s->lenPathBeforePost += pathLen;
      }else{
        s->lenLpNumAfterPost += nbLp;
        s->lenPathAfterPost += pathLen;
      }
    }
    else
      PrintInfo(("\nStatistical structure is uninitialized\n"));
  }
}

/**
 * printStatsGraph
 *
 * Prints the statistics from a stat structure
 * not all printed statistics are taken from the structure
 */
void printStatsGraph(p3d_stat * s, int Print){
  
  setTotalCountVar(XYZ_GRAPH);
  
  if(s && Print){
    PrintInfo(("############# Stat Results #############\n\n"));
    PrintInfo(("Number of loops\t\t\t\t\t\t\t%d\n", s->loopNum));
    PrintInfo(("Pre-planning time\t\t\t\t\t\t%f\n", s->preTime ));
    PrintInfo(("planning time\t\t\t\t\t\t\t%f\n", s->planTime));
    PrintInfo(("Number of conf Total\t\t\t\t\t\t%d\n", s->planConfTot));
    PrintInfo(("Number of conf in collision\t\t\t\t\t%d\n", s->planConfCol));
    PrintInfo(("Number of conf free\t\t\t\t\t\t%d\n", s->planConfFree));
    PrintInfo(("Number of conf added\t\t\t\t\t\t%d\n", s->planConfAdd));
    PrintInfo(("Number of guardian\t\t\t\t\t\t%d\n", s->planConfGuard));
    PrintInfo(("Number of connectors\t\t\t\t\t\t%d\n", s->planConfConn));
    PrintInfo(("Number of call to neigh function\t\t\t\t%d\n", s->planNeigCall));
    PrintInfo(("Number of edges in the graph\t\t\t\t%d\n", s->planEdge));
    PrintInfo(("Number of cycles in the graph\t\t\t\t%d\n", s->planCycle));
    PrintInfo(("Time spent to create cycles in the graph\t\t\t\t%f\n", s->cyclingTime));
    PrintInfo(("Number of computed localpaths\t\t\t\t\t%d\n", s->planLpNum));
    PrintInfo(("Localpath lenght\t\t\t\t\t\t%f\n", s->planLpLenght));
    PrintInfo(("Number of localpath collision test\t\t\t\t%d\n", s->planLpColNum));
    PrintInfo(("Localpath step size\t\t\t\t\t\t%f\n", s->planLpStepSize));
    PrintInfo(("Post planning time\t\t\t\t\t\t%f\n", s->postTime));
    PrintInfo(("Number of collisions\t\t\t\t\t\t%d\n", s->colNum));
    PrintInfo(("Number of configs in paths before postProcessing\t\t%d\n", s->lenLpNumBeforePost + 2*s->loopNum));
    PrintInfo(("Path lenght before postProcessing\t\t\t\t%f\n", s->lenPathBeforePost));
    PrintInfo(("lenght between config before postProcessing\t\t\t%f\n", s->lenPathBeforePost/s->lenLpNumBeforePost));
    PrintInfo(("Number of configs in paths after postProcessing\t\t\t%d\n", s->lenLpNumAfterPost + 2*s->loopNum));
    PrintInfo(("Path lenght after postProcessing\t\t\t\t%f\n", s->lenPathAfterPost));
    PrintInfo(("lenght between config after postProcessing\t\t\t%f\n", s->lenPathAfterPost/s->lenLpNumAfterPost));
    PrintInfo(("Memory usage\t\t\t\t\t\t\t%lu\n", s->memory));
#ifdef MULTIGRAPH
    PrintInfo(("MultiGraph Time\t\t\t\t\t\t%f\n", s->mgTime));
    PrintInfo(("MultiGraph Nodes\t\t\t\t\t\t%d\n", s->mgNodes));
    PrintInfo(("MultiGraph Edges\t\t\t\t\t\t%d\n", s->mgEdges));
#endif
    PrintInfo(("\n############# Stat Results #############\n"));
  }
  else
    PrintInfo(("\nStatistical structure is uninitialized\n"));
}

/**
 * printStatsEnv
 *
 * Same as the printStatgraph
 * it sets a Average to note that the statistics are global
 * Meant to be printed after several runs of the same algorithm
 */
void printStatsEnv(p3d_stat * s, int Print){
	
  if( Print && s ){
    PrintInfo(("############# Stat Results #############\n\n"));
    PrintInfo(("Number of loops\t\t\t\t\t\t\t%d\n", s->loopNum));
    PrintInfo(("Pre-planning time\t\t\t\t\t\t%f\tAverage\t\t%f\n", s->preTime, s->preTime / ((double)s->loopNum+1)));
    PrintInfo(("planning time\t\t\t\t\t\t\t%f\tAverage\t\t%f\n", s->planTime, s->planTime /  ((double)s->loopNum)));
    PrintInfo(("Number of conf Total\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planConfTot,((double)s->planConfTot)/ ((double)s->loopNum)));
    PrintInfo(("Number of conf in collision\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planConfCol,((double)s->planConfCol)/  ((double)s->loopNum)));
    PrintInfo(("Number of conf free\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planConfFree,((double)s->planConfFree)/  ((double)s->loopNum)));
    PrintInfo(("Number of conf added\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planConfAdd,((double)s->planConfAdd)/  ((double)s->loopNum)));
    PrintInfo(("Number of guardian\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planConfGuard,((double)s->planConfGuard)/  ((double)s->loopNum)));
    PrintInfo(("Number of connectors\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planConfConn,((double)s->planConfConn)/  ((double)s->loopNum)));
    PrintInfo(("Number of call to neigh function\t\t\t\t%d\t\tAverage\t\t%f\n", s->planNeigCall,((double)s->planNeigCall)/  ((double)s->loopNum)));
    PrintInfo(("Number of edges in the graph\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planEdge,((double)s->planEdge)/  ((double)s->loopNum)));
    PrintInfo(("Number of cycles in the graph\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planCycle,((double)s->planCycle)/  ((double)s->loopNum)));
    PrintInfo(("Time spent to create cycles in the graph\t\t\t\t\t\t%f\t\tAverage\t\t%f\n", s->cyclingTime,(s->cyclingTime)/  ((double)s->loopNum)));
    PrintInfo(("Number of computed localpaths\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->planLpNum,((double)s->planLpNum)/  ((double)s->loopNum)));
    PrintInfo(("Localpath lenght\t\t\t\t\t\t%f\tAverage\t\t%f\n", s->planLpLenght,((double)s->planLpLenght)/((double)s->planLpNum)));
    PrintInfo(("Number of localpath collision test\t\t\t\t%d\t\tAverage\t\t%f\n", s->planLpColNum, ((double)s->planLpColNum)/((double)s->planLpNum)));
    PrintInfo(("Localpath step size\t\t\t\t\t\t%f\tAverage\t\t%f\n", s->planLpStepSize, s->planLpStepSize/  ((double)s->loopNum)));
    PrintInfo(("Post planning time\t\t\t\t\t\t%f\tAverage\t\t%f\n", s->postTime, s->postTime /  ((double)s->loopNum)));
    PrintInfo(("Number of collisions\t\t\t\t\t\t%d\t\tAverage\t\t%f\n", s->colNum, ((double)s->colNum)/  ((double)s->loopNum)));
    PrintInfo(("Number of configs in paths before postProcessing\t\t%d\t\tAverage\t\t%f\n", s->lenLpNumBeforePost + 2*s->loopNum, ((double)s->lenLpNumBeforePost + 2*s->loopNum)/  ((double)s->loopNum)));
    PrintInfo(("Path lenght before postProcessing\t\t\t\t%f\tAverage\t\t%f\n", s->lenPathBeforePost, ((double)s->lenPathBeforePost)/  ((double)s->loopNum)));
    PrintInfo(("lenght between config before postProcessing\t\t\t%f\n", s->lenPathBeforePost/s->lenLpNumBeforePost));
    PrintInfo(("Number of configs in paths after postProcessing\t\t\t%d\t\tAverage\t\t%f\n", s->lenLpNumAfterPost + 2*s->loopNum, ((double)s->lenLpNumAfterPost + 2*s->loopNum)/  ((double)s->loopNum)));
    PrintInfo(("Path lenght after postProcessing\t\t\t\t%f\tAverage\t\t%f\n", s->lenPathAfterPost, ((double)s->lenPathAfterPost)/  ((double)s->loopNum)));
    PrintInfo(("lenght between config after postProcessing\t\t\t%f\n", s->lenPathAfterPost/s->lenLpNumAfterPost));
    PrintInfo(("Memory usage\t\t\t\t\t\t\t%lu\tAverage\t\t%Lf\n", s->memory, ((long double)s->memory)/((long double)s->loopNum)));
#ifdef MULTIGRAPH
    PrintInfo(("MultiGraph Time\t\t\t\t\t\t%f\tAverage\t\t%f\n", s->mgTime, ((double)s->mgTime)/  ((double)s->loopNum)));
    PrintInfo(("MultiGraph Nodes\t\t\t\t\t\t%d\tAverage\t\t%f\n", s->mgNodes, ((double)s->mgNodes)/  ((double)s->loopNum)));
    PrintInfo(("MultiGraph Edges\t\t\t\t\t\t%d\tAverage\t\t%f\n", s->mgEdges, ((double)s->mgEdges)/  ((double)s->loopNum)));
#endif
    PrintInfo(("\n############# Stat Results #############\n"));
  }
  else
    PrintInfo(("\nStatistical structure is uninitialised\n"));
}

/**
 * addStatsToFile
 *
 * Same as the printStatgraph to a file
 * in Comma-separated values (CSV)
 */
void addStatToFile(p3d_stat * s , FILE* Stat_output ){

    fprintf( Stat_output,"Number of loops,%d\n", s->loopNum);
    fprintf( Stat_output,"Pre-planning time,%f\n", s->preTime );
    fprintf( Stat_output,"planning time,%f\n", s->planTime);
    fprintf( Stat_output,"Number of conf Total,%d\n", s->planConfTot);
    fprintf( Stat_output,"Number of conf in collision,%d\n", s->planConfCol);
    fprintf( Stat_output,"Number of conf free,%d\n", s->planConfFree);
    fprintf( Stat_output,"Number of conf added,%d\n", s->planConfAdd);
    fprintf( Stat_output,"Number of guardian,%d\n", s->planConfGuard);
    fprintf( Stat_output,"Number of connectors,%d\n", s->planConfConn);
    fprintf( Stat_output,"Number of call to neigh function,%d\n", s->planNeigCall);
    fprintf( Stat_output,"Number of edges in the graph,%d\n", s->planEdge);
    fprintf( Stat_output,"Number of cycles in the graph,%d\n", s->planCycle);
    fprintf( Stat_output,"Time spent to create cycles in the graph,%f\n", s->cyclingTime);
    fprintf( Stat_output,"Number of computed localpaths,%d\n", s->planLpNum);
    fprintf( Stat_output,"Localpath lenght,%f\n", s->planLpLenght);
    fprintf( Stat_output,"Number of localpath collision test,%d\n", s->planLpColNum);
    fprintf( Stat_output,"Localpath step size,%f\n", s->planLpStepSize);
    fprintf( Stat_output,"Post planning time,%f\n", s->postTime);
    fprintf( Stat_output,"Number of collisions,%d\n", s->colNum);
    fprintf( Stat_output,"Number of configs in paths before postProcessing,%d\n", s->lenLpNumBeforePost + 2*s->loopNum);
    fprintf( Stat_output,"Path lenght before postProcessing,%f\n", s->lenPathBeforePost);
    fprintf( Stat_output,"lenght between config before postProcessing,%f\n", s->lenPathBeforePost/s->lenLpNumBeforePost);
    fprintf( Stat_output,"Number of configs in paths after postProcessing,%d\n", s->lenLpNumAfterPost + 2*s->loopNum);
    fprintf( Stat_output,"Path lenght after postProcessing,%f\n", s->lenPathAfterPost);
    fprintf( Stat_output,"lenght between config after postProcessing,%f\n", s->lenPathAfterPost/s->lenLpNumAfterPost);
    fprintf( Stat_output,"Memory usage,%lu\n\n", s->memory);
}

/**
 * openStatsFile
 */
FILE* openStatFile(FILE* Stat_output , char* s ){

	Stat_output = fopen( s , "w");
	PrintInfo(("Opening statistics file\n"));
	return Stat_output;
}

/**
 * closeStatsFile
 */
void closeStatFile(FILE* Stat_output ){
	fclose(Stat_output);
	PrintInfo(("Closing statistics file\n"));
}

/**
 * saveInStatToFile
 */
void saveInStatFile(){


	FILE* Stat_output =NULL;

	Stat_output = openStatFile( Stat_output, "statistics.csv" );

	addStatToFile( XYZ_GRAPH->stat , Stat_output );

	closeStatFile(Stat_output);
}

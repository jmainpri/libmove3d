
/****************************************************************************/
/*!		bcd_global.h
 *
 *   bio structures
 *
 ****************************************************************************/

 
#if !defined (BCD_GLOBAL)
#define BCD_GLOBAL
#include "P3d-pkg.h"
#include "Bio-pkg.h"

extern int number_of_bcd_robots; /* number of robots in the bio collision detector */

/* array of robots in the bio collision detector */
extern Robot_structure *bcd_robots[]; 

/* table with the number of robots in a p3d-rob */
extern int num_subrobots[]; /* Stores the number of subrobots of
each p3d_rob */
extern int robot_p3d[]; /* robot_p3d[N] indicates the first
	subrobot corresponding to p3d_rob number N */
  
extern int HYDROGEN_ENVIRONEMENT;



/**********************************************************************************
TYPES AND VARIABLES FOR THE HIERARCHIES
**********************************************************************************/
/**********************************************************************************/

extern check_pair_function check_pair;
extern check_all_function check_all;
 
extern void (*box_collision)(Bboxpt, Bboxpt);
/*trash*/
int giveme_mininhierarchy();

/*************************************************************************************/

extern int charlatan;
#endif





	














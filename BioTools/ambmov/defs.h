#ifndef DEFS_H
#define DEFS_H

/**********************************************************************/
// DEFINITIONS

#define SQR(x) ((x)*(x))
#define PI 3.14159265358979323846
#define FUZZ  0.00001
#define ABS(x)  (((x) >= 0.0) ? (x) : -(x))
#define EQ(x,y) (ABS((x)-(y)) < FUZZ)

/////////////////////////////////
#define OMEGAMAX 190.0*(PI/180.0)
#define OMEGAMIN 170.0*(PI/180.0)
/////////////////////////////////
#define C_VDWR 1.53
#define N_VDWR 1.38
#define O_VDWR 1.35
#define S_VDWR 1.79
#define H_VDWR 1.10

#define Br_VDWR 1.94
#define Cl_VDWR 1.74
#define F_VDWR 1.29

/* #define C_VDWR 1.7 */
/* #define N_VDWR 1.55 */
/* #define O_VDWR 1.52 */
/* #define S_VDWR 1.8 */
/* #define H_VDWR 1.2 */
/////////////////////////////////
#define N_COLOR "Blue"
//#define CA_COLOR "DGreen"
#define C_COLOR "Green"
#define O_COLOR "Red"
#define S_COLOR "Yellow"
#define H_COLOR "White"
/////////////////////////////////

// ATOM NAME STANDARDS
#define INSIGHT_NAMES 0  // default
#define AMBER_NAMES   1

#endif

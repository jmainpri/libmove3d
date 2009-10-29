#ifndef XYT_SYS_H
#define XYT_SYS_H

#ifdef UNIX

	#include <stdio.h>
	#include <sys/types.h>
	#include <stdlib.h>
	#include <time.h>
	
#elif defined WIN32

	#include <stdio.h>
	#include <stdlib.h>
	#include <time.h>
	#include <windows.h>

	#ifdef _DEBUG
		#define __KD_DEBUG
		#define PRINT_INFO
		#define PRINT_ERROR
		#define PRINT_WARNING
		#define PRINT_DEBUG
	#endif

	/* if KCD_CP_IS_SOLID is 1, convex polyhedron as a solid instead of as a collection of convex facets in KCD
  	#define CP_IS_SOLID */

	/* test with AABB around body of robot if KCD_USE_P3D_BODY_AABB is 1 */
	#define USE_P3D_BODY_AABB

	/* KCD with volume: also apply to robot iff KCD_APPROX_ROBOT is 1 */
/*	#define APPROX_ROBOT */

	/* pour compiler sans solide qui fout sa zone dans purify */
/*	#define SOLID_ACT */

	/* pour compiler sans icollide */
/*	#define ICOLLID_ACT */

	/* pour compiler avec gjk afin de le debugger  */
/*	#define GJK_DEBUG */

	/* write a file with OBB tree information (just for debugging) */
/*	#define OBB_EVALUATION */
	
#elif defined VXWORKS

	/* #include <vxWorks.h> */
	#include <stdio.h>
	#include <stdlib.h>
#else
	#error Host platform must be defined !!
#endif

#ifdef QT_LIBRARY

#include <QtCore/QObject>

#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QtCore/QStringList>

#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>

#include <QtGui/QCDEStyle>
#include <QtGui/QCleanlooksStyle>
#include <QtGui/QCommonStyle>
#include <QtGui/QMotifStyle>
#include <QtGui/QPlastiqueStyle>
#include <QtGui/QWindowsStyle>

#include <QtGui/QApplication>
#include <QtGui/QFileDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QLabel>
#include <QtGui/QLayout>
#include <QtGui/QSlider>
#include <QtGui/QCheckBox>
#include <QtGui/QPushButton>
#include <QtGui/QComboBox>
#include <QtGui/QStackedWidget>
#include <QtGui/QSpacerItem>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>

#endif

#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifndef M_PI
#define	M_PI	3.14159265358979323846
#define	M_PI_2	1.57079632679489661923
#endif 

#define M_2PI   6.2831853071795864769

#define EPS1 0.1
#define EPS2 0.01
#define EPS3 0.001
#define EPS4 0.0001
#define EPS5 0.00001
#define EPS6 0.000001
#define EPS10 1e-10
 
#define EPSILON 1e-10

#define P3D_HUGE 1E10



/*
 * Some usefull define
 */

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#ifndef NIL
#define NIL 0
#endif

#ifndef NULL
#define NULL 0
#endif

typedef double Float;

/*
 * Some type definitions
 */
typedef int     *PtrI;          /* pointer to an integer       */
typedef int	(*PtrFct)();	/* pointer to integer function */
typedef float   *PtrF;          /* pointer to a float          */


/*
 * Some usefull macros
 */

/* #define RTOD(r)      (r*57.29578049) */
/* #define DTOR(d)      (d*0.017453292) */
#define RTOD(r)      ((r)*180.0/M_PI)
#define DTOR(d)      ((d)*M_PI/180.0)

#define	FUZZ	     0.00001
#define ABS(x)       (((x) >= 0.0) ? (x) : -(x))
#define MIN(x,y)     (((x) < (y)) ? (x) : (y))
#define MAX(x,y)     (((x) > (y)) ? (x) : (y))
#define SQR(x) ((x)*(x))
#define EQ(x,y)      (ABS((x)-(y)) < FUZZ)
#define LEQ(x,y)     ((x)<(y) || EQ(x,y))
#define GEQ(x,y)     ((x)>(y) || EQ(x,y))
#define LNEQ(x,y)    (!GEQ(x,y))
#define GNEQ(x,y)    (!LEQ(x,y))
#define FEQ(x,y,f)   (ABS((x)-(y)) < f)
#define FLEQ(x,y,f)  ((x)<(y) || FEQ(x,y,f))
#define FGEQ(x,y,f)  ((x)>(y) || FEQ(x,y,f))
#define DOT(u,v)     ((u)[0] * (v)[0] + (u)[1] * (v)[1] + (u)[2] * (v)[2])
#define MAG(u)       sqrt(DOT(u,u))
#ifndef SIGN
#define SIGN(f)      (((f) < 0) ? (-1) : (1))
#endif
#define ROUND(f)     ((int) ( (f) + .5*SIGN(f)))
#define MFLOOR(x)     ((int)x)
#define MCEIL(x)      (x == floor(x) ? x : floor(x)+1)
#define MOD(k,n)     ((k+n)%n)
#define CLAMP(x,min,max)  (((x)<(min)) ? (min) : (((x)>(max)) ? (max) : (x)))
#define INTERP(x,y,t) ((x) + ((y)-(x))*(t))
#define VECEQ(u,v)    (EQ(u[0],v[0]) && EQ(u[1],v[1]) && EQ(u[2],v[2]))
#define FVECEQ(u,v,f) (FEQ(u[0],v[0],f) && FEQ(u[1],v[1],f) && FEQ(u[2],v[2],f))
#define cpvector(u,v) memcpy(u,v,3*sizeof(float))
#define cpmatrix(u,v) memcpy(u,v,12*sizeof(float))



/* Missing prototypes */

extern int      fprintf(FILE *, const char *, ...);
extern int      printf(const char *, ...);

/*
 * for debugging 
 */
#ifdef PRINT_INFO
#define PrintInfo(p) (void)printf p
#else
#define PrintInfo(p) 
#endif
#ifdef PRINT_ERROR
#define PrintError(p) { (void)printf("\n!!!! Error %s (line %i): ",__FILE__,__LINE__); (void)printf p; }
#else
#define PrintError(p) 
#endif
#ifdef PRINT_WARNING
#define PrintWarning(p) (void)printf p
#else
#define PrintWarning(p) 
#endif
#ifdef PRINT_DEBUG
#define PrintDebug(p) { (void)printf("\nDebug %s (line %i): ",__FILE__,__LINE__); (void)printf p; }
#else
#define PrintDebug(p) 
#endif

/*
 * For structure allocation. 
 */

#define MY_ALLOC(type,n)     (type *) basic_alloc(n, sizeof(type))
#define MY_FREE(ptr,type,n)  basic_free((void *)ptr, n, sizeof(type))
#define MY_REALLOC(ptr,type,oldn,newn) (type *)basic_realloc((void *)ptr,oldn,newn,sizeof(type))
#define MY_ALLOC_INFO(p)     basic_alloc_info(p)
#define MY_STRDUP(str)       dyna_str_dup(str)
#define MY_STRFREE(str)      dyna_str_free(str)
#include "../util/proto/basic_alloc_proto.h"


/*
 * For passing parameters to a function
 */
#ifndef VAR_ARGS
#define VAR_ARGS    v_arg1,v_arg2,v_arg3,v_arg4,v_arg5
#endif

typedef int      *ARGS;


#endif


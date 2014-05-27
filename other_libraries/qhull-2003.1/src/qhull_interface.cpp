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
/*<html><pre>  -<a                             href="qh-user.htm"
  >-------------------------------</a><a name="TOP">-</a>
*/

#include <iostream.h>
#include <conio.h>

//--- Include qhull, so it works from with in a C++ source file
//---
//--- In MVC one cannot just do:
//---
//---    extern "C"
//---    {
//---      #include "qhull_a.h"
//---    }
//---
//--- Because qhull_a.h includes math.h, which can not appear
//--- inside a extern "C" declaration.
//---
//--- Maybe that why Numerical recipes in C avoid this problem, by removing
//--- standard include headers from its header files and add them in the
//--- respective source files instead.
//---
//--- [K. Erleben]

#if defined(__cplusplus)
extern "C"
{
#endif
#include <stdio.h>
#include <stdlib.h>
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
#if defined(__cplusplus)
}
#endif

/*********************************************************************/
/*                                                                   */
/*                                                                   */
/*                                                                   */
/*                                                                   */
/*********************************************************************/

void compute_convex_hull(void)
{  
	int dim;  	              /* dimension of points */
	int numpoints;            /* number of points */
	coordT *points;           /* array of coordinates for each point */ 
	boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */ 
	char flags[]= "qhull Tv"; /* option flags for qhull, see qh_opt.htm */
	FILE *outfile= stdout;    /* output from qh_produce_output()			
	                             use NULL to skip qh_produce_output() */ 
	FILE *errfile= stderr;    /* error messages from qhull code */ 
	int exitcode;             /* 0 if no error from qhull */
	facetT *facet;	          /* set by FORALLfacets */
	int curlong, totlong;	  /* memory remaining after qh_memfreeshort */

   	/* initialize dim, numpoints, points[], ismalloc here */
	exitcode= qh_new_qhull (dim, numpoints, points, ismalloc,
							flags, outfile, errfile);
	if (!exitcode) { /* if no error */ 
		/* 'qh facet_list' contains the convex hull */
		FORALLfacets {
			/* ... your code ... */ 
		}
	}
	qh_freeqhull(!qh_ALL);  
	qh_memfreeshort (&curlong, &totlong);
	if (curlong || totlong)
		fprintf (errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", 
		             totlong, curlong);
};

/*********************************************************************/
/*                                                                   */
/*                                                                   */
/*                                                                   */
/*                                                                   */
/*********************************************************************/

void main() 
{ 
	cout << "Hello world" << endl;
	
	cout << "Press any key..." << endl;  
	
	while(!_kbhit());

};

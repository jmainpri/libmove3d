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

   usermem.c
   qh_exit(), qh_free(), and qh_malloc()

   see README.txt  see COPYING.txt for copyright information.

   If you redefine one of these functions you must redefine all of them.
   If you recompile and load this file, then usermem.o will not be loaded
   from qhull.a or qhull.lib

   See libqhull.h for data structures, macros, and user-callable functions.
   See user.c for qhull-related, redefinable functions
   see user.h for user-definable constants
   See userprintf.c for qh_fprintf and qh_fprintf_rbox

   Please report any errors that you fix to qhull@qhull.org
*/

#include "libqhull.h"

#include <stdlib.h>

/*-<a                             href="qh-user.htm#TOC"
  >-------------------------------</a><a name="qh_exit">-</a>

  qh_exit( exitcode )
    exit program

  notes:
    same as exit()
*/
void qh_exit(int exitcode) {
    exit(exitcode);
} /* exit */

/*-<a                             href="qh-user.htm#TOC"
>-------------------------------</a><a name="qh_free">-</a>

qh_free( mem )
free memory

notes:
same as free()
*/
void qh_free(void *mem) {
    free(mem);
} /* free */

/*-<a                             href="qh-user.htm#TOC"
    >-------------------------------</a><a name="qh_malloc">-</a>

    qh_malloc( mem )
      allocate memory

    notes:
      same as malloc()
*/
void *qh_malloc(size_t size) {
    return malloc(size);
} /* malloc */



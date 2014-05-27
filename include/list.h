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
#ifndef LIST_INCLUDE
#define LIST_INCLUDE 1

#ifndef SYS_INCLUDE
#include "p3d_sys.h"
#endif

/************************************************************************/

typedef struct listnode listnode;
typedef listnode *List;

struct listnode {
	int	*data;
	struct listnode *prev, *next;
}LIST_NODE;

#define L_FIRST(L) ((L) ? L_NEXT(L) : NULL)
#define L_NEXT(L) ((L)->next)
#define L_PREV(L) ((L)->prev)
#define LISTDATA(type,L) ((type *)((L)->data))


/* extern int  CLTraverse(); */
/* extern void CLAppend(); */
/* extern void CLInsert(); */
/* extern void CLDeleteNode(); */
/* extern int  CLDeleteData(); */
/* extern void CLKill(); */
/* extern void CLAddData(); */
/* extern List CLFindNode(); */
/* extern PtrI CLFindData(); */
/* extern List CLIterator(); */
/* extern void CLSort(); */
/* extern int  CLLength(); */

#include "list_proto.h"
/* Profiler */
/* #include "Init.prof.h" */
#endif


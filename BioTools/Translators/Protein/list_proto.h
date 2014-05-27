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

#ifndef LIST_PROTO_H
#define LIST_PROTO_H

extern int list_create(int nelems, void*** listPt);
extern void list_copy(void** src_list, int src_nelemt,
			void*** dst_list, int* dst_nelemt);
extern void list_insert(void*** listPt, int *nelems, void *thePt);
extern void list_insert_at_beginning(void*** listPt, int *nelems, void *thePt);
extern int list_remove(void*** listPt, int *nelems, void *thePt);
extern int list_contains(void** listPt, int nelems, void* thePt);
extern int list_find(void** listPt, int nelems, void* thePt, int* indexPt);
extern int list_replace(void** listPt, int nelems, void* old_elem, void* new_elem);
extern void list_free(void ***listPt, int *nelems);

extern void int_list_init(int_list* value_list);
extern void int_list_insert(int_list* value_list, int new_value);
extern void int_list_free(int_list* value_list);
extern int int_list_is_in_list(int_list* value_list, int value);
extern int int_list_get_value(int_list* value_list, int index);

#endif

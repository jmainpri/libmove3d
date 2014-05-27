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
#include <stdlib.h>
#include <stdio.h>

#include "list.h"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

int list_create(int nelems, void*** listPt) {

  int index;

  (*listPt) = (void**)malloc(sizeof(void *) * nelems);

  if (*listPt!=NULL) {
    for (index=0; index<nelems; index++){
      (*listPt)[index]=NULL;
    }
  }

  return (*listPt!=NULL);
}

void list_copy(void** src_list, int src_nelemt,
	       void*** dst_list, int* dst_nelemt) {
  int index;
  *dst_list = (void**)malloc(src_nelemt*sizeof(void*));

  for (index=0; index<src_nelemt; index++) {
    (*dst_list)[index] = src_list[index];
  }
  *dst_nelemt = src_nelemt;
}

void list_insert(void*** listPt, int *nelems, void *thePt) {
  if(*nelems == 0) {
    *listPt = (void **) malloc(sizeof(void *));
  }
  else {
    *listPt = (void **) realloc(*listPt,sizeof(void *) * (*nelems + 1));
  }
  (*listPt)[*nelems] = thePt;
  (*nelems)++;
}

void list_insert_at_beginning(void*** listPt, int *nelems, void *thePt){

  int i;

  if(*nelems == 0) {
    *listPt = (void **) malloc(sizeof(void *));
  }
  else {
    *listPt = (void **) realloc(*listPt,sizeof(void *) * (*nelems + 1));
    for(i=*nelems;i>0;i--){
      (*listPt)[i]=(*listPt)[i-1];
    }
  }
  (*listPt)[0] = thePt;
  (*nelems)++;
}

void list_int_insert(int**listPt, int *nelems, int value) {
  if(*nelems == 0) {
    *listPt = (int*) malloc(sizeof(int));
  }
  else {
    *listPt = (int*) realloc(*listPt,sizeof(int) * (*nelems + 1));
  }
  (*listPt)[*nelems] = value;
  (*nelems)++;
}

int list_remove(void*** listPt, int *nelems, void *thePt) {

  int index=0;
  int found = FALSE;

  
  while(!found && index<*nelems) {
    if ((*listPt)[index]==thePt) {
      found = TRUE;
    }
    else {
      index++;
    }
  }

  if (found) {
    (*listPt)[index] = (*listPt)[*nelems-1];
    *listPt = (void**) realloc(*listPt, (*nelems-1)*sizeof(void*));
    (*nelems)--;
  }

  return found;
}

int list_find(void** listPt, int nelems, void* thePt, int* indexPt) {

  int i=0;
  int found = FALSE;

  while(!found && i<nelems) {
    if (listPt[i]==thePt) {
      found = TRUE;
    }
    else {
      i++;
    }
  }

  if (found)
    *indexPt = i;
    
  return found;
}

int list_contains(void** listPt, int nelems, void* thePt) {

  int i=0;
  int found = FALSE;

  while(!found && i<nelems) {
    if (listPt[i]==thePt) {
      found = TRUE;
    }
    else {
      i++;
    }
  }

  return found;
}


int list_replace(void** listPt, int nelems, void* old_elem, void* new_elem) {
  
  int index;
  int found = FALSE;

  if (list_find(listPt, nelems, old_elem, &index)) {
    found = TRUE;
    listPt[index]=new_elem;
  }

  return found;
  
}

void list_free(void ***listPt, int *nelems) {

  if (*nelems!=0) {
    free(*listPt);
    *listPt = NULL;
  }
  *nelems = 0;
}

/*--------------------------------------------*/
/*        Particular case : INT LIST          */
/*--------------------------------------------*/

void int_list_init(int_list* value_list){
  value_list->n_values = 0;
  value_list->values = NULL;
}

void int_list_insert(int_list* value_list, int new_value){

  if( value_list->n_values== 0) {
    value_list->values = (int *) malloc(sizeof(int));
  }
  else {
    value_list->values = (int *) realloc(value_list->values,sizeof(int) * (value_list->n_values + 1));
  }
  (value_list->values)[value_list->n_values] = new_value;
  (value_list->n_values)++;
}

void int_list_free(int_list* value_list) {

  if (value_list->n_values!=0) {
    free(value_list->values);
    value_list->values = NULL;
  }
  value_list->n_values = 0;
  free(value_list);
}

int int_list_is_in_list(int_list* value_list, int value){

  int i = 0;
  int found = FALSE;

  while(i<value_list->n_values && !found){
    if ((value_list->values)[i]==value)
      found = TRUE;
    else
      i++;
  }

  return found;
}

int int_list_get_value(int_list* value_list, int index){

  return(value_list->values[index]);
}

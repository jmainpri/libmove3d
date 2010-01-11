           /*******************************
	     List Structures
	     Written by T. Simeon 
	     nic@grasp.cis.upenn.edu
	     U.P.E.N.N july 89
	    *******************************/
#include <stdlib.h>
#include "list.h"

/***********************************************************************/

void
CLAppend(p_list, p_data)
     List *p_list;
     void *p_data;
/*
 * Appends the data to the list
 */
{
  register List list = *p_list;
  register List newentry = MY_ALLOC(listnode, 1);
  newentry->data = NULL;
  newentry->prev = NULL;
  newentry->next = NULL;
  
  /* register List newentry = TALLOC(listnode, 1); */
  

  newentry->data = p_data;
  L_PREV(newentry) = list;
  if (list) {
    L_NEXT(newentry) = L_NEXT(list);
    L_NEXT(list) = newentry;
  } else {
    L_NEXT(newentry) = newentry;
  }
  L_PREV(L_NEXT(newentry)) = newentry;
  *p_list = newentry;
}

/***********************************************************************/

void
CLInsert(p_list, p_data)
     List *p_list;
     void *p_data;
/*
 * Inserts the data at the head of the list
 */
{
  CLAppend(p_list, p_data);
  *p_list = L_PREV(*p_list);
}
/***********************************************************************/

List
CLFindNode(list, p_data, p_cmp_f)
     List list;
     void *p_data;
     int  (*p_cmp_f)();
{
  register List l = L_FIRST(list);
  
  if (l) {
    register List first = l;
    do {
      if (p_cmp_f) {
	if (p_cmp_f(LISTDATA(int,l), p_data) != 0) {
	  return l;
	}
      } else {
	if (LISTDATA(int,l) == p_data) {
	  return l;
	}
      }
      l = L_NEXT(l);
    } while (l != first);
  }
  return NIL;
}

/***********************************************************************/

int *
CLFindData(list, p_data, p_cmp_f)
     List list;
     void *p_data;
     int (*p_cmp_f)();
{
  register List l = CLFindNode(list, p_data, p_cmp_f);

  return (l ? LISTDATA(int,l) : NULL);
}

/***********************************************************************/

void
CLDeleteNode(p_list, thenode, p_del_f)
     List *p_list;
     List thenode;
     void (*p_del_f)();
/*
 * Deletes thenode from p_list.
 */
{
  if (L_NEXT(thenode) == thenode) {
    /* only one node in list */
    *p_list = NULL;
  } else {
    L_PREV(L_NEXT(thenode)) = L_PREV(thenode);
    L_NEXT(L_PREV(thenode)) = L_NEXT(thenode);
    if (thenode == *p_list) {
      *p_list = L_PREV(thenode);
    }
  }
  if (p_del_f) {
    (*p_del_f)(LISTDATA(int,thenode));
  }
  MY_FREE(thenode,listnode,1);
}

/***********************************************************************/

int
CLDeleteData(p_list, p_data, p_cmp_f, p_del_f)
     List *p_list;
     void *p_data;
     int (*p_cmp_f)();
     void (*p_del_f)();
/*
 * Deletes the node containing p_data from p_list. The comparison
 * function p_cmp_f should return 1 when a match is found.
 */
{
  register List list = CLFindNode(*p_list, p_data, p_cmp_f);
  if (list) {
    CLDeleteNode(p_list, list, p_del_f);
    return(1);
  } else {
    return(0);
  }
}

/***********************************************************************/

void
CLKill(p_list, p_del_f)
     List *p_list;
     void (*p_del_f)();
{
  List list;
  
  while ((list = L_FIRST(*p_list)) != NULL) {
    CLDeleteNode(p_list, list, p_del_f);
  }
}

/***********************************************************************/

List
CLIterator(list, lastreturn)
     List list, lastreturn;
{

  return (lastreturn == list) ? NULL
                              : (lastreturn) ? L_NEXT(lastreturn)
                                             : L_NEXT(list);
}

/***********************************************************************/

int
CLTraverse(list,func,VAR_ARGS)
     List list;
     int  (*func)();
     ARGS VAR_ARGS;
/* 
 * Applies the given function to each member in the given list.
 * It stops if the function returns a zero value.
 * The function is expected to be of the form 'func(member,args)'
 * The argument list args can be at most 40 bytes.

 */
{
  List first = L_FIRST(list);

  if (first) {
    list = first;
    do {
      if(!func(LISTDATA(int,list),VAR_ARGS)) 
	return(0);
      list = L_NEXT(list);
    } while (list != first);
  }
  return(TRUE);
}

/***********************************************************************/

int
CLLength(list)
     List    list;
{
  int  n;
  List l;

  l = 0;
  n = 0;
  while ((l=CLIterator(list,l)) != NULL) {
    n++;
  }
  return(n);
}

/***********************************************************************/

void
CLAddData(p_list, p_data)
     List *p_list;
     void *p_data;
/*
 * appends the data to the list iff it is not already there.
 */
{
  if (!CLFindNode(*p_list,p_data,0)) {
    CLAppend(p_list,p_data);
  }
}

/***********************************************************************/

void
CLSort(list,cmp)
     List list;
     int  (*cmp)();
{
  List	l;
  int	n;
  int	*data[1000];

  l = 0;
  n = 0;
  while ((l=CLIterator(l,list)) != NULL) {
    data[n++] = l->data;
  }
  
  qsort(data,n,sizeof(data[0]),cmp);
  
  l = 0;
  n = 0;
  while ((l=CLIterator(l,list)) != NULL) {
    l->data = data[n++];
  }
}
    
/***********************************************************************/

















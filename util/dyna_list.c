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
/***************************************************************************/
/*! \file dyna_list.c
 *
 *  \brief Dynamic double chained list
 */
/***************************************************************************/


#include "Util-pkg.h"


/* unitary stack depth */
#define STACK_DEPTH (8)

/*! \brief Default copy constructor */
static void * alloc_data(void *data, size_t size)
{
  void *v;
  v = (void *)MY_ALLOC(char, size);
  if (v) {
    memcpy(v, data, size);
  }
  return v;
}

/*! \brief Default destructor */
static void free_data(void *v, size_t size)
{
  MY_FREE(v, char, size);
}


static dbl_list_node * alloc_node(dbl_clone_function clone_func,
				  void *data, size_t size)
{
  dbl_list_node * N;

  N = MY_ALLOC(dbl_list_node, 1);
  if (N != NULL) {
    memset ((void *)N, 0, sizeof(dbl_list_node));
    if (clone_func == NULL)
      N->data = alloc_data(data, size);
    else
      N->data = clone_func(data);
  }
  return N;
}

static void free_node(dbl_list_node *N, dbl_destroy_function destroy_func, size_t size)
{
  if (destroy_func == NULL)
    free_data(N->data, size);
  else
    destroy_func(N->data);
  MY_FREE(N, dbl_list_node, 1);
}

void * dbl_copy_link(void * data)
{
  return (data); /* On fait pas de copie, on conserve l'original */
}

void dbl_delete_link(void * data)
{
   /* On fait rien puisque seul le pointeur compte. */
}

/*
 * init a list object.
 * return NULL if error.
 */

dbl_list * dbl_list_init(dbl_clone_function clone_func,
		  dbl_destroy_function destroy_func, size_t size)
{
  dbl_list * l;

  l = MY_ALLOC(dbl_list, 1);
  if (l != NULL) {
    l->first = l->last = l->current = NULL;
    l->nb_node = 0;
    l->cur_node = -1;
    l->clone = clone_func;
    l->destroy = destroy_func;
    l->size = size;
    l->state = DBL_OK;

    /* alloc stack */
    l->stack = MY_ALLOC(dbl_list_node *, STACK_DEPTH);
    l->stack_indice = MY_ALLOC(int, STACK_DEPTH);
    l->sp_l = 1;
    /* check alloc */
    if ((l->stack == NULL) || (l->stack_indice == NULL)) {
      MY_FREE(l, dbl_list, 1);
      PrintError(("Not enough memory !!!\n"));
      return NULL;
    }
    l->sp = 0;
    l->sp_modif = 0;
  }
  return l;
}

/*
 * init a list object of pointer.
 * return NULL if error.
 */

dbl_list * dbl_list_pointer_init()
{
  dbl_list * l;

  l = MY_ALLOC(dbl_list, 1);
  if (l != NULL) {
    l->first = l->last = l->current = NULL;
    l->nb_node = 0;
    l->cur_node = -1;
    l->clone = dbl_copy_link;
    l->destroy = dbl_delete_link;
    l->size = 0;
    l->state = DBL_OK;

    /* alloc stack */
    l->stack = MY_ALLOC(dbl_list_node *, STACK_DEPTH);
    l->stack_indice = MY_ALLOC(int, STACK_DEPTH);
    l->sp_l = 1;
    /* check alloc */
    if ((l->stack == NULL) || (l->stack_indice == NULL)) {
      MY_FREE(l, dbl_list, 1);
      PrintError(("Not enough memory !!!\n"));
      return NULL;
    }
    l->sp = 0;
    l->sp_modif = 0;
  }
  return l;
}

/*
 * test if the list is empty
 */
int dbl_list_empty(dbl_list *l)
{ return (l->first == NULL); }


/*
 * check if there is un error
 */
int dbl_list_is_error(dbl_list *l)
{ return (l->state != DBL_OK); }


/*
 * get error code.
 */
int dbl_list_get_error(dbl_list *l)
{ return l->state; }


/*
 * clear error
 */
void dbl_list_clear_error(dbl_list *l)
{ l->state = DBL_OK; }


/*
 * destory list object
 */
void dbl_list_destroy(dbl_list *l)
{
  if (!l)
    return;

  /* remove nodes */
  dbl_list_goto_first(l);

  while(dbl_list_more(l))
    dbl_list_remove(l);

  /* free stack */
  MY_FREE(l->stack, dbl_list_node *, l->sp_l*STACK_DEPTH);
  MY_FREE(l->stack_indice, int, l->sp_l*STACK_DEPTH);

  /* free list */
  MY_FREE(l, dbl_list, 1);
}


/*
 * insert one node after the current
 */
dbl_list_node * dbl_list_insert_after(dbl_list *l, void *data)
{
  dbl_list_node *next, *new_sl;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node((l->clone), data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
    return l->first;
  } else {
    if (l->current == NULL) {
      l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
      return NULL;
    }
    next = l->current->next;
    new_sl = alloc_node((l->clone), data, l->size);
    if (new_sl) {
      new_sl->next = next;
      new_sl->prev = l->current;
      l->current->next = new_sl;

      if (next)
	next->prev = new_sl;

      if (new_sl->next == NULL)
	l->last = new_sl;

      l->nb_node ++;
      l->sp_modif = l->sp;
    }
  }
  return new_sl;
}


/*
 * insert one node before the current
 */
dbl_list_node * dbl_list_insert_before(dbl_list *l, void *data)
{
  dbl_list_node *prev, *new_sl;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node((l->clone), data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
    return l->first;
  } else {
    if (l->current == NULL) {
      l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
      return NULL;
    }
    prev = l->current->prev;
    new_sl = alloc_node((l->clone), data, l->size);

    if (new_sl) {
      if (prev)
	prev->next = new_sl;

      new_sl->prev = prev;
      new_sl->next = l->current;
      l->current->prev = new_sl;

      if (new_sl->prev == NULL)
	{ l->first = new_sl; }

      l->cur_node ++;
      l->nb_node ++;
      l->sp_modif = l->sp;
    }
  }
  return new_sl;
}


/*
 * insert node the head of list
 */
dbl_list_node * dbl_list_append(dbl_list *l, void *data)
{
  dbl_list_node *next;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node((l->clone), data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
  } else {
    next = l->first;
    l->first = alloc_node((l->clone), data, l->size);
    if (l->first) {
      l->first->next = next;
      next->prev = l->first;

      l->nb_node ++;
      if (l->current != NULL)
	{ l->cur_node ++; }
      l->sp_modif = l->sp;
    }
  }
  l->sp_modif = l->sp;
  return l->first;
}


/*
 * insert node the end of list
 */
dbl_list_node * dbl_list_concat(dbl_list *l, void *data)
{
  dbl_list_node *prev;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node((l->clone), data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
  } else {
    prev = l->last;
    l->last = alloc_node((l->clone), data, l->size);
    if (l->last) {
      l->last->prev = prev;
      prev->next = l->last;

      l->nb_node ++;
    }
  }
  return l->last;
}


/*
 * insert node at the end of list if it doesn't exist before
 */
dbl_list_node * dbl_list_add_link(dbl_list *l, void *data)
{
  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node((l->clone), data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
  } else {
    dbl_list_push(l);
    if (dbl_list_find_by_data(l, data, NULL) == NULL) {
      dbl_list_pop(l);
      dbl_list_concat_link(l, data);
    } else
      { dbl_list_pop(l); }
  }
  return l->last;
}

/*
 * insert one node after the current
 */
dbl_list_node * dbl_list_insert_link_after (dbl_list *l, void *data)
{
  dbl_list_node *next, *new_sl;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node(dbl_copy_link, data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
    return l->first;
  } else {
    if (l->current == NULL) {
      l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
      return NULL;
    }
    next = l->current->next;
    new_sl = alloc_node(dbl_copy_link, data, l->size);
    if (new_sl) {
      new_sl->next = next;
      new_sl->prev = l->current;
      l->current->next = new_sl;

      if (next)
	next->prev = new_sl;

      if (new_sl->next == NULL)
	l->last = new_sl;

      l->nb_node ++;
      l->sp_modif = l->sp;
    }
  }
  return new_sl;
}

/*
 * insert one node before the current
 */
dbl_list_node * dbl_list_insert_link_before(dbl_list *l, void *data)
{
  dbl_list_node *prev, *new_sl;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node(dbl_copy_link, data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
    return l->first;
  } else {
    if (l->current == NULL) {
      l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
      return NULL;
    }
    prev = l->current->prev;
    new_sl = alloc_node(dbl_copy_link, data, l->size);

    if (new_sl) {
      if (prev)
	prev->next = new_sl;

      new_sl->prev = prev;
      new_sl->next = l->current;
      l->current->prev = new_sl;

      if (new_sl->prev == NULL)
	l->first = new_sl;

      l->cur_node ++;
      l->nb_node ++;
      l->sp_modif = l->sp;
    }
  }
  return new_sl;
}

/*
 * insert node the head of list
 */
dbl_list_node * dbl_list_append_link(dbl_list *l, void *data)
{
  dbl_list_node *next;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node(dbl_copy_link, data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
  } else {
    next = l->first;
    l->first = alloc_node(dbl_copy_link, data, l->size);
    if (l->first) {
      l->first->next = next;
      next->prev = l->first;

      l->nb_node ++;
      if (l->current != NULL)
	{ l->cur_node ++; }
      l->sp_modif = l->sp;
    }
  }
  l->sp_modif = l->sp;
  return l->first;
}

/*
 * insert node the end of list
 */
dbl_list_node * dbl_list_concat_link(dbl_list *l, void *data)
{
  dbl_list_node *prev;

  if (l->first == NULL) {
    l->first = l->current = l->last = alloc_node(dbl_copy_link, data, l->size);
    if (l->first != NULL) {
      l->cur_node = 0;
      l->nb_node = 1;
    }
  } else {
    prev = l->last;
    l->last = alloc_node(dbl_copy_link, data, l->size);
    if (l->last) {
      l->last->prev = prev;
      prev->next = l->last;

      l->nb_node ++;
    }
  }
  return l->last;
}


/**
 * @brief Insert a node in a sorted list. The list is supposed sorted.
 * @param l The list
 * @param data Data to insert
 * @param (* sortFn)( void *, void * ) Compare fonction
 */

void dbl_list_insert_sorted_link(dbl_list *l, void *data, int (*sortFn)(void *, void *))
{
  int added = 0;

  if (l->first == NULL) {//si la liste est vide
    l->first = l->current = l->last = alloc_node((l->clone), data, l->size);
    if (l->first != NULL) {//si l'opération a reussi.
      l->cur_node = 0;
      l->nb_node = 1;
    }
  } else {
    dbl_list_push(l);//sauver la postion courant de la liste
    if (dbl_list_find_by_data(l, data, NULL) == NULL) {//si la donnée n'existe pas dans la liste
      for (dbl_list_goto_first(l);dbl_list_more(l) && added == 0;dbl_list_next(l)){
        if (!sortFn(data,dbl_list_get_data(l))){//si newListNode->data->cost < cursor->data->cost
          dbl_list_insert_before(l, data);//inserer data dans la position precedente
          added = 1;
        }
      }
      if (!added){//si le noeud n'a pas ete inserer. l'ajouter a la fin de la liste.
        dbl_list_concat_link(l,data);
      }
    }
    dbl_list_pop(l);//restitue la position courante de la liste
  }
}

/*
 * get the adresse of current node
 */
dbl_list_node * dbl_list_get_current(dbl_list *l)
{
  if (l->first)
    return l->current;
  else {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return NULL;
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the indice of the current node
 *  \param l: the list
 *  \return the indice of the current node (-1 if no current node is defined)
 */
int dbl_list_get_current_indice(dbl_list *l)
{
  if (l->first)
    { return l->cur_node; }
  else {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return -1;
  }
}


/*
 * get the of first node
 */
dbl_list_node * dbl_list_first(dbl_list *l)
{
  if (l->first != NULL)
    { return l->first; }

  l->state = DBL_EMPTY;
  return NULL;
}


/*
 * get the adresse of last node
 */
dbl_list_node * dbl_list_last(dbl_list *l)
{
  if (l->first != NULL)
    { return l->last; }

  l->state = DBL_EMPTY;
  return NULL;
}


/*
 * check if there is more node in the list
 */
int dbl_list_more(dbl_list *l)
{ return (l->current != NULL); }


/*
 * check if no more nodes in the list
 */
int dbl_list_no_more(dbl_list *l)
{ return (l->current == NULL); }


/*
 * move to the next node
 */
dbl_list_node * dbl_list_next(dbl_list *l)
{
  if (l->first == NULL)
    { l->state = DBL_EMPTY; }
  else {
    if ( l->current != NULL ) {
      l->current = l->current->next;
      if (l->current != NULL)
        l->cur_node ++;
      else
        l->cur_node = -1;
    }
  }
  return l->current;
}


/*
 * move to the previous node
 */
dbl_list_node * dbl_list_prev(dbl_list *l)
{
  if (l->first == NULL)
    { l->state = DBL_EMPTY; }
  else {
    if (l->current != NULL) {
      l->current = l->current->prev;
      l->cur_node --;
    }
  }
  return l->current;
}


/*
 * clear list
 */
void dbl_list_clear (dbl_list *l)
{
  if (l == NULL) return;

  if (l->first == NULL) return;

  l->sp = 0;
  l->sp_modif = 0;

  /* remove nodes */
  dbl_list_goto_first(l);

  while(dbl_list_more(l))
    dbl_list_remove(l);

  l->first = l->last = l->current = NULL;
  l->cur_node = -1;
  l->nb_node = 0;
}

/*
 * cut the current node
 */
dbl_list_node * dbl_list_remove(dbl_list *l)
{
  dbl_list_node *tmp;

  if (l->current == NULL) {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return NULL;
  }

  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    return NULL;
  } else {
    l->sp_modif = l->sp;

    if (l->first == l->current)
      { l->first = l->first->next; }

    if (l->last == l->current)
      { l->last = l->last->prev; }

    if (l->current->prev != NULL)
      { l->current->prev->next = l->current->next; }

    if (l->current->next != NULL)
      { l->current->next->prev = l->current->prev; }

    tmp = l->current->next;
    free_node(l->current, (l->destroy), l->size);
    l->current = tmp;
    l->nb_node --;

    if (l->current == NULL) {
      l->current = l->last;
      l->cur_node --;
    }

  }
  return l->current;
}

/*! \brief Remove the current node but do not destroy the data
 *
 *  \param l: The list
 *
 *  \return The new current node
 */
dbl_list_node * dbl_list_remove_link(dbl_list *l)
{
  dbl_list_node *tmp;

  if (l->current == NULL) {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return NULL;
  }

  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    return NULL;
  } else {
    l->sp_modif = l->sp;

    if (l->first == l->current)
      { l->first = l->first->next; }

    if (l->last == l->current)
      { l->last = l->last->prev; }

    if (l->current->prev != NULL)
      { l->current->prev->next = l->current->next; }

    if (l->current->next != NULL)
      { l->current->next->prev = l->current->prev; }

    tmp = l->current->next;
    free_node(l->current, (dbl_delete_link), 0);
    l->current = tmp;
    l->nb_node --;

    if (l->current == NULL) {
      l->current = l->last;
      l->cur_node --;
    }

  }
  return l->current;
}


/*
 * cut the node with data
 */
dbl_list_node * dbl_list_remove_data(dbl_list *l, void * data)
{
  dbl_list_node *tmp, *cur;
  int cur_index;

  cur = l->current;
  cur_index = l->cur_node;
  tmp = dbl_list_find_by_data(l, data, NULL);

  if (tmp!=NULL) {
    if (tmp == cur)
      { cur = tmp->next; }
    if (cur_index>=l->cur_node)
      { cur_index --; }
    dbl_list_remove(l);
  }
  l->current = cur;
  l->cur_node = cur_index;
  return cur;
}


/*
 * cut the node with data
 */
dbl_list_node * dbl_list_remove_link_data(dbl_list *l, void * data)
{
  dbl_list_node *tmp, *cur;
  int cur_index;

  cur = l->current;
  cur_index = l->cur_node;
  tmp = dbl_list_find_by_data(l, data, NULL);

  if (tmp!=NULL) {
    if (tmp == cur)
      { cur = tmp->next; }
    if (cur_index>=l->cur_node)
      { cur_index --; }
    dbl_list_remove_link(l);
  }
  l->current = cur;
  l->cur_node = cur_index;
  return cur;
}


/*
 * cut the first node
 */
void dbl_list_remove_first(dbl_list *l)
{
  dbl_list_node *tmp;

  if (l->first == NULL)
    { l->state = DBL_EMPTY; }
  else {

    if (l->current == l->first)
      { l->current = l->current->next; }
    else
      { l->cur_node --; }
    if (l->current == NULL)
      { l->cur_node = -1; }

    if (l->last == l->first)
      { l->last = l->last->next; }

    if (l->first->next)
      { l->first->next->prev = NULL; }

    tmp = l->first->next;
    l->sp_modif = l->sp;

    free_node(l->first, (l->destroy), l->size);
    l->nb_node --;

    l->first = tmp;

  }
}

/*
 * cut the last node
 */
void dbl_list_remove_last(dbl_list *l)
{
  dbl_list_node *tmp;

  if (l->first == NULL)
    l->state = DBL_EMPTY;
  else {

    if (l->current == l->last) {
      l->current = l->current->prev;
      if (l->current == NULL)
	{ l->cur_node = -1; }
      else
	{ l->cur_node --; }
    }

    if (l->first == l->last)
      { l->first = l->first->prev; }

    if (l->last->prev)
      { l->last->prev->next = NULL; }

    l->sp_modif = l->sp;
    tmp = l->last->prev;
    free_node(l->last, (l->destroy), l->size);
    l->nb_node --;
    l->last = tmp;
  }
}


/*--------------------------------------------------------------------------*/
/* !\brief Swap between current & next nodes.
 *
 *  \param l: the list
 *
 *  \return TRUE if succeed.
 */
int dbl_list_swap_next(dbl_list *l)
{
  dbl_list_node *next, *prev;

  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    return FALSE;
  }
  if ((l->current == NULL) || (l->current->next == NULL)) {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return FALSE;
  }
  next = l->current->next;
  prev = l->current->prev;

  l->current->next = next->next;
  if (next->next != NULL)
    { next->next->prev = l->current; }
  else
    { l->last = l->current; }

  l->current->prev = next;
  next->next = l->current;
  next->prev = prev;
  if (prev != NULL)
    { prev->next = next; }
  else
    { l->first = next; }
  l->cur_node ++;
  l->sp_modif = l->sp;

  return TRUE;
}


/*--------------------------------------------------------------------------*/
/* !\brief Swap between current & prev nodes.
 *
 *  \param l: the list
 *
 *  \return TRUE if success.
 */
int dbl_list_swap_prev(dbl_list *l)
{
  dbl_list_node *next, *prev;

  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    return FALSE;
  }
  if ((l->current == NULL) || (l->current->prev == NULL)) {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return FALSE;
  }
  next = l->current->next;
  prev = l->current->prev;

  l->current->prev = prev->prev;
  if (prev->prev != NULL)
    { prev->prev->next = l->current; }
  else
    { l->first = l->current; }

  l->current->next = prev;
  prev->prev = l->current;
  prev->next = next;
  if (next != NULL)
    { next->prev = prev; }
  else
    { l->last = prev; }
  l->cur_node --;
  l->sp_modif = l->sp;

  return TRUE;
}


/*
 * move to the first node
 */
dbl_list_node * dbl_list_goto_first(dbl_list *l)
{
  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    l->cur_node = -1;
  } else
    { l->cur_node = 0; }

  l->current = l->first;
  return l->current;
}


/*
 * move to the last node
 */
dbl_list_node * dbl_list_goto_last(dbl_list *l)
{
  if (l->first == NULL)
    { l->state = DBL_EMPTY; }
  l->cur_node = l->nb_node - 1;

  l->current = l->last;
  return l->current;
}


/*--------------------------------------------------------------------------*/
/*! \brief Set the current position on the element n
 *
 *  \param l: The list
 *  \param n: The indice to reach
 */
dbl_list_node * dbl_list_goto_n(dbl_list *l, int n)
{
  if (l->first == NULL)
    { l->state = DBL_EMPTY; }
  if (n>=l->nb_node) {
    l->cur_node = -1;
    l->current = NULL;
  } else {
    l->cur_node = 0;
    l->current = l->first;
    for(l->cur_node=0; l->cur_node<n; l->cur_node++)
      { l->current = l->current->next; }
  }
  return l->current;
}


/*
 * get the adresse of data
 */
void * dbl_list_get_data(dbl_list *l)
{
  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    return NULL;
  }
  if (l->current == NULL) {
    l->state = DBL_NOT_VALIDE_CURRENT_POSITION;
    return NULL;
  }
  return l->current->data;
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the adresse of nTh node
 *  \param l: the list
 *  \param n: the node number
 *  \return the node adresse
 */
dbl_list_node * dbl_list_get_node_n(dbl_list *l, int n)
{
  int i=0;
  dbl_list_node * node;

  node = l->first;
  while(node != NULL) {
    if (i == n)
      { return node; }
    i++;
    node = node->next;
  }
  return NULL;
}


/*
 * get the adresse of nTh element's data
 */
void * dbl_list_get_data_n(dbl_list *l, int n)
{
  int i=0;
  dbl_list_node * node;

  node = l->first;
  while(node != NULL) {
    if (i == n)
      { return node->data; }
    i++;
    node = node->next;
  }
  return NULL;
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the indice of the data in the list
 *  \param l: the list
 *  \param data: the data searched
 *  \return the indice of the data node (-1 if the data is not found)
 */
int dbl_list_get_data_indice(dbl_list *l, void * data)
{
  int i=0;
  dbl_list_node * node;

  node = l->first;
  while(node != NULL) {
    if (node->data == data)
      { return i; }
    i++;
    node = node->next;
  }
  return -1;
}

/*
 * keep current position in the list
 */
void dbl_list_push(dbl_list *l)
{
  if ((l->sp + 1) >= l->sp_l * STACK_DEPTH) {
    l->sp_l++;
    l->stack = MY_REALLOC(l->stack, dbl_list_node *, (l->sp_l-1) *STACK_DEPTH,
			 (l->sp_l) * STACK_DEPTH);
    l->stack_indice = MY_REALLOC(l->stack_indice, int, (l->sp_l-1) *
				 STACK_DEPTH, (l->sp_l) * STACK_DEPTH);
    if ((l->stack == NULL) || (l->stack_indice == NULL)) {
      PrintError(("Not enough memory !!!\n"));
      l->sp_l = 0;
      l->sp = 0;
      l->sp_modif = 0;
      l->state = DBL_MEMORY_ERROR;
      return;
    }
  }
  l->stack[l->sp] = l->current;
  l->stack_indice[l->sp] = l->cur_node;
  l->sp++;
}

/*
 * pop the last position pushed!!
 */
int dbl_list_pop(dbl_list *l)
{
  dbl_list_node * node;

  if (l->sp == 0) {
    l->state = DBL_STACK_ERROR;
    return FALSE;
  }
  l->sp --;
  l->current = l->stack[l->sp];
  l->cur_node = l->stack_indice[l->sp];
  if (l->sp < l->sp_modif) {
    if (l->current != NULL) {
      node = l->first;
      l->cur_node = 0;
      while ((node != NULL) && (node != l->current)) {
	node = node->next;
	l->cur_node ++;
      }
      if (node == NULL) {
	l->current = NULL;
	l->cur_node = -1;
      }
    }
    l->sp_modif = l->sp;
  }
  return TRUE;
}


/*
 * Flush stack pointer
 */

void dbl_list_flush_stack (dbl_list *l)
{
  if (l != NULL) {
    l->sp_modif = 0;
    l->sp = 0;
  }
}

/*
 * copy list..
 */
dbl_list * dbl_list_copy(const dbl_list *l)
{
  dbl_list *tmp;
  dbl_list_node * current;

  tmp = dbl_list_init(l->clone, l->destroy, l->size);
  if (tmp == NULL)
    { return NULL; }

  current = l->first;

  while(current!=NULL) {
    dbl_list_concat(tmp, current->data);
    current = current->next;
  }

  dbl_list_goto_first(tmp);

  return tmp;
}

/*--------------------------------------------------------------------------*/
/*! \brief Concat all the list element and clone them.
 *
 *  \param l_dest: the destination list
 *  \param l_src:  the source list
 *
 *  \return TRUE if success
 */
int dbl_list_concat_list(dbl_list * l_dest, const dbl_list *l_src)
{
  dbl_list_node * current;

  if (l_dest->size != l_src->size) {
    l_dest->state = DBL_NOT_MATCHING_LIST;
  }

  current = l_src->first;
  dbl_list_push(l_dest);
  while(current!=NULL) {
    dbl_list_concat(l_dest, current->data);
    current = current->next;
  }

  dbl_list_pop(l_dest);

  if (l_dest->state == DBL_OK)
    { return TRUE; }
  return FALSE;
}

/*--------------------------------------------------------------------------*/
/*! \brief Concat all the list element but do only links.
 *
 *  \param l_dest: the destination list
 *  \param l_src:  the source list
 *
 *  \return TRUE if success
 */
int dbl_list_concat_list_link(dbl_list * l_dest, const dbl_list *l_src)
{
  dbl_list_node * current;

  if (l_dest->size != l_src->size) {
    l_dest->state = DBL_NOT_MATCHING_LIST;
  }

  current = l_src->first;
  dbl_list_push(l_dest);

  while(current!=NULL) {
    dbl_list_concat_link(l_dest, current->data);
    current = current->next;
  }

  dbl_list_pop(l_dest);

  if (l_dest->state == DBL_OK)
    { return TRUE; }
  return FALSE;
}


/*
 * count nodes in the list
 */
int dbl_list_count(dbl_list *l)
{ return l->nb_node; }


/*
 * Sort list
 */
void dbl_list_sort (dbl_list *l,  int (*call_func)(void *, void *))
{
  void **p;
  void *q;
  int n, i, j;

  if ((l != NULL) || (l->nb_node != 1)) { /* singular case */
    n = l->nb_node;
    if (n <= 0)
      { l->state = DBL_EMPTY; }
    else {
      /* alloc pointers data tab */
      p = MY_ALLOC(void *,  n);

      if (p == NULL)
	{ l->state = DBL_MEMORY_ERROR; }
      else {
	/* get pointers */
	dbl_list_goto_first (l);
	i = 0;
	/* body */
	while (dbl_list_more (l)) {
	  /* get data */
	  p[i++] = dbl_list_get_data (l);
	  /* next */
	  dbl_list_next (l);
	} /* while */

	/* sorting */
	for (i = 0; i < n - 1; i++) {
	  for ( j = i; j < n; j++) {
	    if (call_func (p[i], p[j])) {
	      q = p[i];
	      p[i] = p[j];
	      p[j] = q;
	    }	/* if */
	  } /* for j */
	} /* for i */


	/* reposition pointers in the list */
	dbl_list_goto_first (l);
	i = 0;
	/* body */
	while (dbl_list_more (l)) {
	  /* get data */
	  l->current->data = p[i++];
	  /* next */
	  dbl_list_next (l);
	} /* while */

	/* reset current */
	dbl_list_goto_first (l);

	/* free */
	MY_FREE(p, void *, n);
      }
    }
  }
}


/*
 * find node using user-defined comparasion function --
 *                      null addresses will be compared
 */

dbl_list_node * dbl_list_find_by_data (dbl_list *l, void *data,
			       int (*FctEqualData)(void *, void *))
{
  void *node_data;

  if (l->first == NULL) {
    l->state = DBL_EMPTY;
    return NULL;
  }

  dbl_list_goto_first (l);
  while (dbl_list_more (l)){
    node_data = l->current->data;
    if (!FctEqualData){
      if (node_data == data){
	l->state = DBL_OK;
	return l->current;
      }
    }else{
      if (FctEqualData (node_data, data)){
	l->state = DBL_OK;
	return l->current;
      }
    }
    dbl_list_next (l);
  }/* while */

  return NULL;
}


/*--------------------------------------------------------------------------*/
/*! \brief Test the equality of two lists
 *
 * \param list1Pt: the first list
 * \param list2Pt: the second list
 * \param FctEqualData: A function to compare two data.
 *        If it return a number <0 then the first element is the previous.
 *        If it return 0  then both are equal.
 *        If it return a number >0 then the second element is the previous.
 *        If it is a NULL pointeur, it compares data pointer adress value
 *
 * \return <0 if \a list1Pt is smaller, 0 if they are equal,
 *         >0 if \a list2Pt is smaller
 */

int dbl_list_test_equal (dbl_list *list1Pt, dbl_list *list2Pt,
			 int (*FctEqualData)(void *, void *))
{
  dbl_list_node * node1Pt, * node2Pt;
  int test;

  node1Pt = list1Pt->first;
  node2Pt = list2Pt->first;
  while((node1Pt != NULL) && (node2Pt != NULL)) {
    if (!FctEqualData){
      if (node1Pt->data < node2Pt->data)
	{ return -1; }
      else if (node1Pt->data > node2Pt->data)
	{ return 1; }
    } else {
      test = FctEqualData(node1Pt->data, node2Pt->data);
      if (test!=0)
	{ return test; }
    }
    node1Pt = node1Pt->next;
    node2Pt = node2Pt->next;
  }
  if (node1Pt != NULL)
    { return 1; }
  if (node2Pt != NULL)
    { return -1; }
  return 0;
}


/*--------------------------------------------------------------------------*/
/*! \brief Test if the data of one sub-list is included in the main list
 *
 * \param main_listPt: the main list
 * \param sub_listPt:  the sub-list list
 * \param FctEqualData: A function to compare two data.
 *        TRUE if the data are equals
 *
 * \return TRUE if the sub-list data are included in the main list
 */

int dbl_list_test_included (dbl_list * main_listPt, dbl_list * sub_listPt,
			    int (*FctEqualData)(void *, void *))
{
  dbl_list_node * sub_nodePt, * main_nodePt;
  int found;

  sub_nodePt  = sub_listPt->first;
  while(sub_nodePt != NULL) {
    main_nodePt = main_listPt->first;
    found = FALSE;
    while(main_nodePt != NULL) {
      if (!FctEqualData){
	if (sub_nodePt->data == main_nodePt->data) {
	  found = TRUE;
	  break;
	}
      } else {
	if (FctEqualData(main_nodePt->data, sub_nodePt->data)) {
	  found = TRUE;
	  break;
	}
      }
      main_nodePt = main_nodePt->next;
    }
    if (!found)
      { return FALSE; }
    sub_nodePt = sub_nodePt->next;
  }
  return TRUE;
}


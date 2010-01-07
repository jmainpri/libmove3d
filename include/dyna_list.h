/***************************************************************************/
/*! \file dyna_list.h
 *
 *  \brief Dynamic double chained list
 */
/***************************************************************************/


#ifndef dyna_list_h
#define dyna_list_h

/*! \brief List errors stats
 */
typedef enum {
  DBL_OK, 
  DBL_EMPTY, 
  DBL_NOT_VALIDE_CURRENT_POSITION, 
  DBL_STACK_ERROR,
  DBL_MEMORY_ERROR,
  DBL_NULL_LIST,
  DBL_NOT_MATCHING_LIST
} dbl_list_state;

typedef void *(*dbl_clone_function)(void*);
typedef void (*dbl_destroy_function)(void*);
typedef int (*dbl_equal_function)(void*, void*);



/*! \bief One node
 */
typedef struct s_dbl_list_node {
  void *data;
  struct s_dbl_list_node *next, *prev;
} dbl_list_node;


/*! \brief List structure
 */
typedef struct s_dbl_list {
  dbl_list_node *first, *last, *current;
  dbl_list_node **stack;
  int * stack_indice;
  int nb_node;
  int cur_node;
  dbl_clone_function clone;
  dbl_destroy_function destroy;
  dbl_list_state state;
  size_t size;
  int sp_modif;
  int sp;
  int sp_l;
} dbl_list;

#define DBL_LIST_FIRST(type, L) ((type *)(((L)->first)?((L)->first->data):NULL))
#define DBL_LIST_LAST(type, L) ((type *)(((L)->last)?((L)->last->data):NULL))
#define DBL_LIST_NEXT(type, L) ((type *)(((L)->current)?(((L)->current->next)?((L)->current->next->data):NULL):NULL))
#define DBL_LIST_PREV(type, L) ((type *)(((L)->current)?(((L)->current->prev)?((L)->current->prev->data):NULL):NULL))
#define DBL_LIST_DATA(type, L) ((type *)(((L)->current)?((L)->current->data):NULL))
#define DBL_LIST_DATA_N(type, L, num) ((type *)(dbl_list_get_data_n(L, num)))

/* Macro d'accès à : dbl_list * dbl_list_init(void *(*dbl_clone_func)(void*), 
 *                              void (*dbl_destroy_func)(void*),size_t size)
 */
#define DBL_LIST_INIT(copy, del, type) (dbl_list_init((dbl_clone_function)(copy), (dbl_destroy_function)(del), sizeof(type)))

/* Macro d'accès à : dbl_list * dbl_list_find_by_data(dbl_list * listPt,
 *                    void * data, int (*dbl_equal_function)(void*,void*)) 
 */
#define DBL_LIST_FIND(L, data, fct) ((dbl_list_find_by_data((L), (data), (dbl_equal_function)(fct))) != NULL)

#endif /* dyna_list_h */

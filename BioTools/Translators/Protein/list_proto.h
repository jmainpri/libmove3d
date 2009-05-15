
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

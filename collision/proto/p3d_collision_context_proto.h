/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

extern void p3d_col_pair_activate_col_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern void p3d_col_pair_activate_dist_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern void p3d_col_pair_activate_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern void p3d_col_pair_activate_col_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_activate_dist_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_activate_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_deactivate_col_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern void p3d_col_pair_deactivate_dist_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern void p3d_col_pair_deactivate_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern void p3d_col_pair_deactivate_col_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_deactivate_dist_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_deactivate_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_deactivate_all ( p3d_collision_pair * pair );
extern int p3d_col_pair_is_test_col_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern int p3d_col_pair_is_calc_dist_env ( p3d_collision_pair * pair, p3d_obj * obj );
extern int p3d_col_pair_is_test_col_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern int p3d_col_pair_is_calc_dist_pair ( p3d_collision_pair * pair, p3d_obj * obj1, p3d_obj * obj2 );
extern void p3d_col_pair_add_into ( p3d_collision_pair * src_pair, p3d_collision_pair * dest_pair );
extern void p3d_col_pair_sub_into ( p3d_collision_pair * src_pair, p3d_collision_pair * dest_pair );
extern void p3d_col_pair_copy_into ( p3d_collision_pair * src_pair, p3d_collision_pair * dest_pair );
extern p3d_collision_pair * p3d_col_pair_create ( void );
extern p3d_collision_pair * p3d_col_pair_copy ( p3d_collision_pair * pair );
extern void p3d_col_pair_destroy ( p3d_collision_pair * pair );
extern p3d_collision_pair * p3d_col_pair_get_cur ( void );
extern void p3d_col_pair_copy_current_into ( p3d_collision_pair * pair );
extern p3d_collision_pair * p3d_col_pair_copy_current ( void );
extern void p3d_col_pair_put ( p3d_collision_pair * pair );
extern void p3d_col_pair_push ( void );
extern void p3d_col_pair_push_and_put ( p3d_collision_pair * pair );
extern void p3d_col_pair_pop ( void );
extern void p3d_col_pair_flush ( void );
extern void p3d_col_pair_clear ( void );
extern void p3d_col_pair_start ( void );
extern void p3d_col_pair_stop ( void );
extern void p3d_col_context_add_elem ( p3d_collision_context ** list_begin, void * param, p3d_collision_pair * pair );
extern void p3d_col_context_merge_elem ( p3d_collision_context ** list_begin, void * param, p3d_collision_pair * pair );
extern void p3d_col_context_del_elem ( p3d_collision_context ** list_begin, void * param, p3d_collision_pair * pair );
extern void p3d_col_context_del_param ( p3d_collision_context ** list_begin, void * param );
extern p3d_collision_context * p3d_col_context_copy ( p3d_collision_context *list_begin );
extern void p3d_col_context_destroy ( p3d_collision_context ** list_begin );
extern void p3d_col_context_add ( p3d_collision_context * src_context, p3d_collision_context ** dest_context );
extern p3d_collision_pair * p3d_col_context_sub ( p3d_collision_context * src_context, p3d_collision_context * dest_context );

#endif /* __CEXTRACT__ */

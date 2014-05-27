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
/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Wed Jun 23 14:30:02 2004
 */
#ifndef __CEXTRACT__

extern p3d_graph * p3d_create_graph ( void );
extern p3d_graph * p3d_create_graph ( p3d_rob* r );
extern p3d_node * p3d_create_node ( p3d_graph * G );
extern void p3d_insert_node ( p3d_graph *G, p3d_node *nodePt );
extern void p3d_merge_check ( p3d_graph * G );
extern void p3d_merge_comp ( p3d_graph *G, p3d_compco *c1, p3d_compco **c2Pt );
extern int p3d_link_node_comp ( p3d_graph *G, p3d_node *N, p3d_compco **compPt );
extern int p3d_link_node_graph ( p3d_node* Node, p3d_graph* Graph );
extern int p3d_link_node_comp_multisol ( p3d_graph *G, p3d_node *N, p3d_compco **compPt );
extern int p3d_link_node_graph_multisol ( p3d_node* Node, p3d_graph* Graph );
extern void p3d_randconfs ( int NMAX, int (*fct_stop)(void), void (*fct_draw)(void) );
extern void p3d_learn ( int NMAX, int (*fct_stop)(void), void (*fct_draw)(void) );
extern void p3d_loopSpecificLearn(p3d_rob *robotPt, configPt qs, configPt qg, char* filePrefix, int loopNb, double * arraytimes, int *nfail);
extern int p3d_specific_search (char* filePrefix);
extern void p3d_expand_graph ( p3d_graph *G, double frac, int (*fct_stop)(void), void (*fct_draw)(void) );
extern p3d_node ** p3d_addStartAndGoalNodeToGraph(configPt qs, configPt qg, int *iksols, int *iksolg, p3d_graph *G, p3d_rob *robotPt);
extern int p3d_specific_learn ( double *qs, double *qg, int *iksols, int *iksolg, int (*fct_stop)(void), void (*fct_draw)(void) );
extern void p3d_create_orphans ( int NMAX, int (*fct_stop)(void), void (*fct_draw)(void) );
extern void p3d_create_linking ( int NMAX, int (*fct_stop)(void), void (*fct_draw)(void) );
extern int p3d_generate_random_free_conf ( p3d_graph *G, int inode, int (*fct_stop)(void), int * fail );
extern int p3d_generate_random_free_conf_multisol ( p3d_graph *G, int (*fct_stop)(void), int * fail );
extern int p3d_generate_random_conf ( p3d_graph *G, int (*fct_stop)(void), int * fail );
extern int p3d_add_basic_node ( p3d_graph *G, int (*fct_stop)(void), int * fail );
extern int p3d_add_isolate_or_linking_node ( p3d_graph *G, int (*fct_stop)(void), void (*fct_draw)(void), int *fail, int type );
extern int BestNode ( void *n1, void *n2 );
//extern int ebtBestNodeByEdge(void *n1, void *n2);
//extern int costBestNode ( void *n1, void *n2 );
//extern int sortCostBestNode(void *n1, void *n2) ;
extern int costBestEdge ( void *e1, void *e2 );
extern int sortCostBestEdge(void *e1, void *e2) ;

extern void p3d_print_graph_compco ( p3d_graph *G );
extern void p3d_order_list_node_nofconnex ( void );
extern int p3d_convert_traj_to_graph ( p3d_traj *traj, p3d_graph *graph, int (*fct_stop)(void) );

//start path deform
extern void del_plot_file(int index);
//extern void save_plot_in_file(int index, double coef1,double coef2);
extern FILE * open_file_to_save_plot(int index);
extern void close_file_to_save_plot(FILE *OFile);
extern int p3d_add_all_prm_node(p3d_graph *G,int (*fct_stop)(void));
extern int p3d_all_link_node(p3d_node* N, p3d_graph* G);
extern int BestPath(void *n1, void *n2);
//end path deform
#endif /* __CEXTRACT__ */

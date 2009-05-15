
#include "kcd.h" /* all types needed below for global variables */

extern obj_prim_interval_table *kcd_o_prm_table;
extern prim_obj_table      *kcd_obj_prim_table;
extern poly_bb_table       *kcd_poly_bb_table;
extern mov_obj_bb_table    *kcd_mo_bb_table;   /* for each mo, the bb around it */
extern link_bb_table_p     *kcd_body_bb_table; /* for each robot or mo-group, 
                     the mo-id and bb-id of the links or mo-s in the group */

extern int *nof_grp_elts; /* for each group of movable objects, the number of movable objects in the group */
extern int *grp_is_active; /* for each group of movable objects, TRUE iff group must be tested for collision */
extern int *ith_static_obj; /* entry nof_sos is input index considerung all objects */

extern id_bb      *flat_obbs,    *spherical_obbs,    *tall_obbs;
extern int      nof_flat_obbs, nof_spherical_obbs, nof_tall_obbs;

extern kcd_bb_p   *all_bbs;
extern int      nof_bbs; /* nof_flat_obbs + nof_spherical_obbs + nof_tall_obbs */
extern int nof_bbs_extra; /* extra allocated memory in all_bbs list, just in case */

extern kcd_bb_p first_bb;
extern kcd_bb_p first_bbs_pile;
extern kcd_bb_p last_bbs_pile;
extern int nr_added_on_pile;


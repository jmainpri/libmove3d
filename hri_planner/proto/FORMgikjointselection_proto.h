/*
 *  FORMgikjointselection_proto.h
 *  XcodeBioMove3D
 *
 *  Created by Akin Sisbot on 25/6/09.
 *
 */

#ifndef __CEXTRACT__

extern void g3d_create_gik_jointsel_form ( void );
extern void g3d_show_gik_jointsel_form ( void );
extern void g3d_hide_gik_jointsel_form ( void );
extern void g3d_delete_gik_jointsel_form ( void );
extern void CB_gik_target_robot_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_vis_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_precision_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_step_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_run_obj(FL_OBJECT *obj, long arg);
#endif /* __CEXTRACT__ */


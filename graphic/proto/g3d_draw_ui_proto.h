
#include "forms.h"


#ifndef __CEXTRACT__
extern void g3d_create_form(FL_FORM** form, int w, int h, int type);
extern int g3d_create_labelframe(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_button(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_choice(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_checkbutton(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_frame(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_input(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_valslider(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_box(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_counter(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_xyplot(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_refresh_form(FL_FORM** form);
extern void g3d_fl_free_object(FL_OBJECT* obj);
extern void g3d_fl_free_form(FL_FORM* obj);
#endif /* __CEXTRACT__ */

extern int hri_is_object_visible(HRI_AGENT * agent,p3d_rob *object, int threshold, int save);
extern int g3d_is_object_visible_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *object, double *result);
extern int g3d_is_object_visible_from_current_viewpoint(g3d_win* win, p3d_rob *object, double *result, int save, char *path);
extern int hri_object_visibility_placement(HRI_AGENT * agent,p3d_rob *object, int * result);
extern int g3d_object_visibility_placement(p3d_matrix4 camera_frame, p3d_rob *object, double Hfov, double Vfov, double Hfoa, double Vfoa, int *result);

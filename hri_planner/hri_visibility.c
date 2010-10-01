#include "Util-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "GL/glx.h"

int hri_is_object_visible(HRI_AGENT * agent, p3d_rob *object, int threshold, int save, int draw_at_end)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  double result;
  int point,fov,visobj;
  
  if(object==NULL || agent==NULL){
    printf("%s: %d: g3d_is_object_visible_from_viewpoint(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  //Change the size of the viewport if you want speed
  if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/3),(GLint)(viewport[3]/3));
  }
  
  g3d_save_win_camera(win->vs);
  g3d_save_state(win, &st);
  
  point = agent->perspective->enable_pointing_draw;
  fov = agent->perspective->enable_vision_draw;
  visobj = agent->perspective->enable_visible_objects_draw;  
  
  // only keep what is necessary:
  win->vs.fov            = agent->perspective->fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled = 1;
  agent->perspective->enable_pointing_draw = FALSE;
  agent->perspective->enable_vision_draw = FALSE;
  agent->perspective->enable_visible_objects_draw = FALSE;
  
#ifdef USE_SHADERS
  g3d_no_shader();
#endif
  
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(agent->perspective->camjoint->abs_pos, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);
  
  //everything is ready now.
  g3d_is_object_visible_from_current_viewpoint(win, object, &result, save, (char*)"/Users/easisbot/Work/BioMove3D/screenshots/");
  
  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);
  agent->perspective->enable_pointing_draw = point;
  agent->perspective->enable_vision_draw = fov;
  agent->perspective->enable_visible_objects_draw = visobj;
  
#ifdef USE_SHADERS
  if (win->vs.enableShaders) {
    g3d_use_shader();
  }
#endif
  
  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
  if(draw_at_end)
    g3d_draw_win(win);
  
  if(100*result>=threshold)
    return TRUE;
  else
    return FALSE;
  
}


//! This function return how much % of the object is visible from a given viewpoint.
//! \param camera_frame the frame of the viewpoint (the looking direction is X, Y points downward and Z to the left)
//! \param camera_fov the field of view angle of the robot's camera (in degrees)
//! \param object pointer to the object
//! \param result return the ratio of the visibility of the object
//! \return TRUE in case of success, FALSE otherwise
int g3d_is_object_visible_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *object, double *result)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  int save = TRUE;
  
  if(object==NULL){
    printf("%s: %d: g3d_is_object_visible_from_viewpoint(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  
  //Change the size of the viewport if you want speed
  if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/3),(GLint)(viewport[3]/3));
  }
  
  g3d_save_win_camera(win->vs);
  g3d_save_state(win, &st);
  
  // only keep what is necessary:
  win->vs.fov            = camera_fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled=  1;
  win->vs.enableLogo = 0;
  
#ifdef USE_SHADERS
  g3d_no_shader();
#endif
  
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);
  
  //everything is ready now.
  g3d_is_object_visible_from_current_viewpoint(win, object,result,TRUE,(char*)"/Users/easisbot/Work/BioMove3D/screenshots/");
  
  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);
  
#ifdef USE_SHADERS
  if (win->vs.enableShaders) {
    g3d_use_shader();
  }
#endif
  
  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
  g3d_draw_win(win);
  
  return TRUE;
}

//! This function return how much % of the object is visible from a given viewpoint.
//! \param camera_frame the frame of the viewpoint (the looking direction is X, Y points downward and Z to the left)
//! \param camera_fov the field of view angle of the robot's camera (in degrees)
//! \param object pointer to the object
//! \param result return the ratio of the visibility of the object
//! \return TRUE in case of success, FALSE otherwise
int g3d_are_given_objects_visible_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, p3d_rob **objects, int objects_nb, double *results)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  int save = TRUE;
  
  if(objects==NULL){
    printf("%s: %d: input objects are NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  
  //Change the size of the viewport if you want speed
  if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/3),(GLint)(viewport[3]/3));
  }
  
  g3d_save_win_camera(win->vs);
  g3d_save_state(win, &st);
  
  // only keep what is necessary:
  win->vs.fov            = camera_fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled=  1;
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);
  
  //everything is ready now.
  g3d_compute_visibility_for_given_objects_in_current_viewpoint(win, objects, objects_nb, results, TRUE, (char*)"/Users/easisbot/Work/BioMove3D/screenshots/");
  
  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);
  
  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
  g3d_draw_win(win);
  
  return TRUE;
}


int g3d_is_object_visible_from_current_viewpoint(g3d_win* win, p3d_rob *object, double *result, int save, char *path)
{
  int idealpixels;
  int visiblepixels;
  unsigned char *image;
  int i, width, height;
  GLint viewport[4];
  static int crntcnt= 0, idlcnt = 0;
  char name[256];  
  
  // disable the display of all obstacles and of all the robots of no interest:
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_NO_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    if(XYZ_ENV->robot[i]==object) {
      continue;
    }
    else {
      p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_NO_DISPLAY);
    }
  }
  // display the object in red
  p3d_set_robot_display_mode(object, P3D_ROB_UNLIT_RED_DISPLAY);
  
  g3d_draw_win_back_buffer(win); //only the object should be drawn in red, everthing else is black
  
  if(save) {
    sprintf(name, "%sidealview%i.ppm",path, crntcnt++);
    g3d_export_OpenGL_display(name);
  }
  
  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];
  
  // get the OpenGL image buffer:
  image = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration
  
  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  
  // get the image pixels (from (0,0) position):
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
  
  // count the pixels corresponding to the object's color:
  idealpixels= 0;
  for(i=0; i<width*height; i++) {
    if(image[3*i]> 0.8) {
      idealpixels++;
    }
  }
  
  if(idealpixels!=0){    
    // display everything in blue except the object which is in red
    for(i=0; i<XYZ_ENV->no; ++i) {
      p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_UNLIT_BLUE_DISPLAY);
    }
    for(i=0; i<XYZ_ENV->nr; ++i) {
      if(XYZ_ENV->robot[i]==object) {
        continue;
      }
      else {
        p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_UNLIT_BLUE_DISPLAY);
      }
    }  
    // display the object in red
    p3d_set_robot_display_mode(object, P3D_ROB_UNLIT_RED_DISPLAY);
    
    g3d_draw_win_back_buffer(win);
    
    if(save){
      //save the image. All is blue, the object is red.
      sprintf(name, "%scurrentview%i.ppm",path, idlcnt++);
      g3d_export_OpenGL_display(name);
    }    
    glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration
    
    // choose 1-byte alignment:
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    // get the image pixels (from (0,0) position):
    
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
    
    visiblepixels= 0;
    for(i=0; i<width*height; i++){
      if(image[3*i]> 0.8) {
        visiblepixels++;
      }
    }
    
    // visiblepixels present the pixels that we see and not blocked by obstructions
    
    *result= ((double) visiblepixels)/((double) idealpixels);
  }
  else{
    //printf("Not looking at the object: %s\n",object->name);
    *result = -1;
  }
  
  free(image);
  
  // reset the display modes of everything
  for(i=0; i<XYZ_ENV->no; ++i){
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_DEFAULT_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i){
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_DEFAULT_DISPLAY);
  }
  
  return TRUE;
}

/* Computes visibility in one image acquisition for given objects */
int g3d_compute_visibility_for_given_objects_in_current_viewpoint(g3d_win* win, p3d_rob **objects, int objects_nb, double *res, int save, char *path)
{
  int idealpixels[objects_nb];
  int visiblepixels[objects_nb];
  unsigned char *image;
  int i, j, width, height;
  GLint viewport[4];
  static int crntcnt = 0;
  char name[256];  
  double color[4]= {0,0,0,1}; 
    
  // disable the display of all obstacles and of all the robots:
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_NO_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_NO_DISPLAY);
  }
  
  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];
  
  // get the OpenGL image buffer:
  image = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration
  
  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  
  // display each object separately and get its pixel count
  for(i=0; i<objects_nb; i++) {
    p3d_set_robot_display_mode(objects[i], P3D_ROB_UNLIT_RED_DISPLAY);
    
    g3d_draw_win_back_buffer(win); //only the object should be drawn in red, everthing else is black
    
    if(save) {
      sprintf(name, "/%sidealview%i.ppm", path, crntcnt++);
      g3d_export_OpenGL_display(name);
    }
    
    // get the image pixels (from (0,0) position):
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
    
    // count the pixels corresponding to the object's color:
    idealpixels[i]= 0;
    visiblepixels[i] = 0;
    for(j=0; j<width*height; j++) {
      if(image[3*j]> 0.8) {
        idealpixels[i]++;
      }
    }
    p3d_set_robot_display_mode(objects[i], P3D_ROB_NO_DISPLAY);
  }
  
  
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_UNLIT_BLUE_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_UNLIT_BLUE_DISPLAY);
  }  
  
  // display all the objects in a single image in the different colors of red
  for(i=0; i<objects_nb; i++) {
    color[0]+=(10.0/255.0);
    p3d_set_robot_display_mode(objects[i], P3D_ROB_UNLIT_CUSTOM_COLOR_DISPLAY, color);
  }
  
  g3d_draw_win_back_buffer(win); //only the object should be drawn in red, everthing else is black
  
  if(save) {
    sprintf(name, "/%scurrentview%i.ppm", path, crntcnt++);
    g3d_export_OpenGL_display(name);
  }
  
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
  
  for(j=0; j<width*height; j++){
    if(image[3*j]>0) {
      visiblepixels[(int)((image[3*j])/10)-1]++;
    }
  }
  
  // visiblepixels present the pixels that we see and not blocked by obstructions
  for (i=0; i<objects_nb; i++) {
    res[i] = ((double) visiblepixels[i])/((double) idealpixels[i]);
  }
  
  free(image);
  
  // reset the display modes of everything
  for(i=0; i<XYZ_ENV->no; ++i){
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_DEFAULT_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i){
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_DEFAULT_DISPLAY);
  }
  
  return TRUE;
}


/****************************************************************/
/*!
 * \brief Converts a cartesian coordinate to a spherical one
 *
 * \param x,y,z point
 * \param rho - distance, phi - elevation,theta - azimuth resulting angles
 * !
 
 */
/****************************************************************/
void p3d_cartesian2spherical(double x, double y, double z, double *rho, double *phi, double *theta)
{
  *rho   = DISTANCE3D(x,y,z,0,0,0);
  *theta = atan2(y,x);
  *phi   = atan2(z,DISTANCE2D(x,y,0,0));
}

/****************************************************************/
/*!
 * \brief Converts a cartesian coordinate to a spherical one with a given origin
 *
 * \param x,y,z point
 * \param originx,originy,originz origin point
 * \param phi,theta resulting angles
 * !
 
 */
/****************************************************************/
void p3d_cartesian2spherical(double x, double y, double z,
                             double originx, double originy, double originz,
                             double *phi, double *theta)
{
  double distance = DISTANCE3D(x,y,z,originx,originy,originz);
  
  *phi = atan2( (y-originy),(x-originx) );
  *theta = acos( (z-originz)/distance );
}


int hri_object_visibility_placement(HRI_AGENT *agent, p3d_rob *object, int *result, double *elevation, double *azimuth)
{
  
  if(object==NULL || agent==NULL){
    printf("%s: %d: hri_object_visibility_placement(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  
  g3d_object_visibility_placement(agent->perspective->camjoint->abs_pos, object, 
                                  DTOR(agent->perspective->fov),DTOR(agent->perspective->fov*0.75),
                                  DTOR(agent->perspective->foa),DTOR(agent->perspective->foa*0.75),
                                  result, elevation, azimuth);  
  return TRUE;
}

int hri_object_pointing_placement(HRI_AGENT *agent, p3d_rob *object, int *result, double *elevation, double *azimuth)
{
  
  if(object==NULL || agent==NULL){
    printf("%s: %d: hri_object_pointing_placement(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  
  g3d_object_visibility_placement(agent->perspective->pointjoint->abs_pos, object, 
                                  DTOR(agent->perspective->point_tolerance),DTOR(agent->perspective->point_tolerance),
                                  DTOR(agent->perspective->point_tolerance),DTOR(agent->perspective->point_tolerance),
                                  result, elevation, azimuth);  
  return TRUE;
}


int g3d_object_visibility_placement(p3d_matrix4 camera_frame, p3d_rob *object, double Hfov, double Vfov, double Hfoa, double Vfoa, int *result, double *phi_result, double *theta_result)
{
  p3d_vector4 objectCenter,objectCenterCamFr;
  p3d_matrix4 invM;
  double rho,phi,theta;
  
  p3d_get_robot_center(object, objectCenter);
  
  p3d_matInvertXform(camera_frame,invM);
  p3d_matvec4Mult(invM, objectCenter, objectCenterCamFr);
  
  p3d_cartesian2spherical(objectCenterCamFr[0],objectCenterCamFr[1],objectCenterCamFr[2],
                          &rho,&phi,&theta);
  //printf("Distance:%f, Elevation: %f, Azimuth: %f\n",rho,RTOD(phi),RTOD(theta));
  
  *phi_result = phi; // Elevation
  *theta_result = theta; // Azimuth
  
  if( (ABS(theta)<Hfoa/2) && (ABS(phi)<Vfoa/2) ){
    // is in foa
    *result = 1;
  }
  else {
    if ((ABS(theta)<Hfov/2) && (ABS(phi)<Vfov/2)) {
      // is in fov
      *result = 2;
    }
    else {
      *result = 3;
    }
  }
  
  return TRUE;
}

void g3d_draw_all_agents_fovs(HRI_AGENTS *agents)
{
  int i;
  
  if(agents != NULL){
    for (i=0; i<agents->all_agents_no; i++) {
      if(agents->all_agents[i]->perspective->enable_vision_draw)
        g3d_draw_agent_fov(agents->all_agents[i]);
      if(agents->all_agents[i]->perspective->enable_pointing_draw)
        g3d_draw_agent_pointing(agents->all_agents[i]);
    }
  }
}


int g3d_draw_agent_pointing(HRI_AGENT *agent)
{
  GLdouble BlueColor[4] =    { 0.0, 0.0, 0.5, 0.7 };
  GLdouble BlueColorT[4] =   { 0.0, 0.0, 0.5, 0.0 }; 
  
  if(agent!=NULL && agent->perspective->enable_pointing_draw){
    
    g3d_draw_visibility_by_frame(agent->perspective->pointjoint->abs_pos,
                                 DTOR(agent->perspective->point_tolerance), DTOR(agent->perspective->point_tolerance),
                                 2, BlueColor, BlueColorT);  
    return TRUE;
  }
  else {
    return FALSE;
  }
  
}

int g3d_draw_agent_fov(HRI_AGENT *agent)
{
  GLdouble GreenColor[4] =   { 0.0, 0.5, 0.0, 0.7 };
  GLdouble GreenColorT[4] =   { 0.0, 0.5, 0.0, 0.0 };
  GLdouble GreyColor[4] =   { 0.5, 0.5, 0.5, 0.5 };
  GLdouble GreyColorT[4] =   { 0.5, 0.5, 0.5, 0.0 };
  
  if(agent!=NULL && agent->perspective->enable_vision_draw){
    
    g3d_draw_visibility_by_frame(agent->perspective->camjoint->abs_pos,
                                 DTOR(agent->perspective->foa),
                                 DTOR(agent->perspective->foa*0.75),
                                 1, GreenColor, GreenColorT);  
    
    g3d_draw_visibility_by_frame(agent->perspective->camjoint->abs_pos,
                                 DTOR(agent->perspective->fov),
                                 DTOR(agent->perspective->fov*0.75),
                                 1, GreyColor, GreyColorT);
    
    return TRUE;
  }
  else {
    return FALSE;
  }
  
}

int g3d_draw_visibility_by_frame(p3d_matrix4 camera_frame, double Hfov, double Vfov, double max_dist, GLdouble source_color[], GLdouble  dest_color[])
{
  double x_source, y_source, z_source;
  p3d_vector4 left_up, left_down, right_up, right_down;
  p3d_vector4 left_up_abs, left_down_abs, right_up_abs, right_down_abs;
  
  x_source = camera_frame[0][3];
  y_source = camera_frame[1][3];
  z_source = camera_frame[2][3];
  
  left_up[0] = max_dist;
  left_up[1] = atan(Hfov/2)*max_dist;
  left_up[2] = atan(Vfov/2)*max_dist;
  left_up[3] = 1;
  
  left_down[0] = max_dist;
  left_down[1] = atan(Hfov/2)*max_dist;
  left_down[2] = -atan(Vfov/2)*max_dist;
  left_down[3] = 1;
  
  right_up[0] = max_dist;
  right_up[1] = -atan(Hfov/2)*max_dist;
  right_up[2] = atan(Vfov/2)*max_dist;
  right_up[3] = 1;
  
  right_down[0] = max_dist;
  right_down[1] = -atan(Hfov/2)*max_dist;
  right_down[2] = -atan(Vfov/2)*max_dist;
  right_down[3] = 1;
  
  p3d_matvec4Mult(camera_frame, left_up, left_up_abs);
  p3d_matvec4Mult(camera_frame, left_down, left_down_abs);
  p3d_matvec4Mult(camera_frame, right_up, right_up_abs);
  p3d_matvec4Mult(camera_frame, right_down, right_down_abs);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  //Left
  glBegin(GL_QUADS);
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);  
  glColor4dv(dest_color);
  glVertex3f(left_up_abs[0],left_up_abs[1],left_up_abs[2]);  
  glColor4dv(dest_color);
  glVertex3f(left_down_abs[0],left_down_abs[1],left_down_abs[2]); 
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);
  glEnd();
  
  //Right
  glBegin(GL_QUADS);
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);  
  glColor4dv(dest_color);
  glVertex3f(right_up_abs[0],right_up_abs[1],right_up_abs[2]); 
  glColor4dv(dest_color);
  glVertex3f(right_down_abs[0],right_down_abs[1],right_down_abs[2]);  
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);
  glEnd();
  
  //Up
  glBegin(GL_QUADS);
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);  
  glColor4dv(dest_color);
  glVertex3f(right_up_abs[0],right_up_abs[1],right_up_abs[2]); 
  glColor4dv(dest_color);
  glVertex3f(left_up_abs[0],left_up_abs[1],left_up_abs[2]);  
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);
  glEnd();
  
  //Down
  glBegin(GL_QUADS);
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);  
  glColor4dv(dest_color);
  glVertex3f(left_down_abs[0],left_down_abs[1],left_down_abs[2]); 
  glColor4dv(dest_color);
  glVertex3f(right_down_abs[0],right_down_abs[1],right_down_abs[2]);  
  glColor4dv(source_color);
  glVertex3f(x_source,y_source,z_source);
  glEnd();
  
  glDisable(GL_BLEND);
  
  return TRUE;
}


int hri_is_object_pointed(HRI_AGENT * agent, p3d_rob *object, int threshold, int save)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  double result;
  
  if(object==NULL || agent==NULL){
    printf("%s: %d: g3d_is_object_visible_from_viewpoint(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  //Change the size of the viewport if you want speed
  if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/3),(GLint)(viewport[3]/3));
  }
  
  g3d_save_win_camera(win->vs);
  g3d_save_state(win, &st);
  
  // only keep what is necessary:
  win->vs.fov            = agent->perspective->point_tolerance;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled =  1;
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(agent->perspective->pointjoint->abs_pos, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);
  
  //everything is ready now.
  g3d_is_object_visible_from_current_viewpoint(win, object,&result,save,(char*)"");
  
  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);
  
  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
  g3d_draw_win(win);
  
  if(100*result>=threshold)
    return TRUE;
  else
    return FALSE;  
}
void DrawSmallCube(double x, double y, double z)
{
  
  /* We tell we want to draw quads */
  glBegin(GL_QUADS);
  
  /* Every four calls to glVertex, a quad is drawn */
  glVertex3f(-1+x, -1+y, -1+z);
  glVertex3f(-1+x, -1+y,  1+z);
  glVertex3f(-1+x,  1+y,  1+z);
  glVertex3f(-1+x,  1+y, -1+z);
  
  glVertex3f( 1+x, -1+y, -1+z);
  glVertex3f( 1+x, -1+y,  1+z);
  glVertex3f( 1+x,  1+y,  1+z);
  glVertex3f( 1+x,  1+y, -1+z);
  
  glVertex3f(-1+x, -1+y, -1+z);
  glVertex3f(-1+x, -1+y,  1+z);
  glVertex3f( 1+x, -1+y,  1+z);
  glVertex3f( 1+x, -1+y, -1+z);
  
  glVertex3f(-1+x,  1+y, -1+z);
  glVertex3f(-1+x,  1+y,  1+z);
  glVertex3f( 1+x,  1+y,  1+z);
  glVertex3f( 1+x,  1+y, -1+z);
  
  glVertex3f(-1+x, -1+y, -1+z);
  glVertex3f(-1+x,  1+y, -1+z);
  glVertex3f( 1+x,  1+y, -1+z);
  glVertex3f( 1+x, -1+y, -1+z);
  
  glVertex3f(-1+x, -1+y,  1+z);
  glVertex3f(-1+x,  1+y,  1+z);
  glVertex3f( 1+x,  1+y,  1+z);
  glVertex3f( 1+x, -1+y,  1+z);
  
  /* No more quads */
  glEnd();
  
}

/* Occlusion test using GPU ARB extensions */
/* It's not functional but it's left here to keep valuable code */
int g3d_is_object_visible_from_current_viewpoint2(g3d_win* win, p3d_rob *object, double *result, int save, char *path)
{
/*  int i;
  
  GLuint queries = -1;
  GLuint sampleCount;
  GLint available;
  GLint bitsSupported;
  
  glDrawBuffer(GL_BACK);
  
  glClearColor(1,1,0,1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glGetQueryiv(GL_SAMPLES_PASSED, GL_QUERY_COUNTER_BITS, &bitsSupported);
  
  glGenQueriesARB(1, &queries);
  
  // Here draw occluders.
  //g3d_draw_win_back_buffer(win);
  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas);
  g3d_drawSphere(0, 0, 0, 2);
  //DrawSmallCube(0,0,0);
  glXSwapBuffers(fl_display,fl_get_canvas_id(ob));
  //DrawSmallCube(0,0,0);
  g3d_drawSphere(0, 0, 0, 2);
  glXSwapBuffers(fl_display,fl_get_canvas_id(ob));
  glFlush();
  return 1;
  //glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  glDepthMask(GL_FALSE);
  // also disable texturing and any fancy shaders
  
  glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queries);
  
  // Draw target object
  //p3d_set_robot_display_mode(XYZ_ENV->robot[num], P3D_ROB_UNLIT_BLUE_DISPLAY);
  //g3d_draw_robot(FALSE, win);
  //DrawSmallCube(2,-1,0);
  g3d_drawSphere(4, -4, 1, 1);
  
  glEndQueryARB(GL_SAMPLES_PASSED_ARB);
  
  
  glFlush();
  
  // Do other work until "most" of the queries are back, to avoid
  // wasting time spinning
  i = 0; // instead of N-1, to prevent the GPU from going idle
  do {
    
    glGetQueryObjectivARB(queries,
                          GL_QUERY_RESULT_AVAILABLE_ARB,
                          &available);
  } while (!available);
  
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDepthMask(GL_TRUE);
  // reenable other state, such as texturing
  
  glGetQueryObjectuivARB(queries, GL_QUERY_RESULT_ARB,
                         &sampleCount);
  if (sampleCount > 0) {
    printf("VISIBLE count:%d\n",sampleCount);    
  }
  else {
    printf("INVISIBLE\n");
 } 
 g3d_drawSphere(4, -4, 1, 1);
 
 */
 
  return TRUE;
}

int hri_compute_agent_sees(HRI_AGENT * agent, int threshold, int save, int draw_at_end)
{
  GLint viewport[4];
  int i;
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  p3d_env *env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  double elevation, azimuth;
  p3d_rob **test_obj_list;
  int obj_idx = 0;
  int placement_res;
  double *vis_res;
  int saved_sees_display_mode;
  int point,fov,visobj;
  
  if(agent==NULL){
    printf("%s: %d: input agent is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  saved_sees_display_mode = agent->perspective->enable_visible_objects_draw;
  agent->perspective->enable_visible_objects_draw = FALSE;
  //Change the size of the viewport if you want speed
  if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/3),(GLint)(viewport[3]/3));
  }
  
  g3d_save_win_camera(win->vs);
  g3d_save_state(win, &st);
  
  // only keep what is necessary:
  win->vs.fov            = agent->perspective->fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled =  1;
  win->vs.enableLogo     =  0;
  agent->perspective->enable_pointing_draw = FALSE;
  agent->perspective->enable_vision_draw = FALSE;
  agent->perspective->enable_visible_objects_draw = FALSE;
  
#ifdef USE_SHADERS
  g3d_no_shader();
#endif
  
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(agent->perspective->camjoint->abs_pos, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);
  
  // everything is ready now.
  // 1- Select the objects that are in fov
  
  test_obj_list = MY_ALLOC(p3d_rob*,env->nr);
  
  for(i=0; i<env->nr; i++) {
    hri_object_visibility_placement(agent, env->robot[i], &placement_res, &elevation, &azimuth);
    switch (placement_res) {
      case 1:
        agent->perspective->currently_sees.vispl[i] = HRI_FOA;
        test_obj_list[obj_idx] = env->robot[i];
        obj_idx++;
        break;
      case 2:
        agent->perspective->currently_sees.vispl[i] = HRI_FOV;
        test_obj_list[obj_idx] = env->robot[i];
        obj_idx++;
        break;
      default:
        agent->perspective->currently_sees.vispl[i] = HRI_OOF;
        agent->perspective->currently_sees.vis[i] = HRI_INVISIBLE;
        break;
    }
  }
    
  // 2- Fetch their visibility
  
  vis_res = MY_ALLOC(double,obj_idx);
  
  g3d_compute_visibility_for_given_objects_in_current_viewpoint(win, test_obj_list, obj_idx, vis_res, save, (char*)"/Users/easisbot/Work/BioMove3D/screenshots/");
  
  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);
 
  agent->perspective->enable_pointing_draw = point;
  agent->perspective->enable_vision_draw = fov;
  agent->perspective->enable_visible_objects_draw = visobj;
  
#ifdef USE_SHADERS
  if (win->vs.enableShaders) {
    g3d_use_shader();
  }
#endif  
  
  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
  if(draw_at_end)
    g3d_draw_win(win);
  
  for(i=0; i<obj_idx; i++) {
    if(100*vis_res[i]>=threshold){
      agent->perspective->currently_sees.vis[test_obj_list[i]->num] = HRI_VISIBLE;
      //printf("%s is VISIBLE\n",test_obj_list[i]->name);
    }
    else {
      agent->perspective->currently_sees.vis[test_obj_list[i]->num] = HRI_INVISIBLE;
      //printf("%s is INVISIBLE\n",test_obj_list[i]->name);
    } 
  }
  
  agent->perspective->enable_visible_objects_draw = saved_sees_display_mode;
  
  return TRUE;
}



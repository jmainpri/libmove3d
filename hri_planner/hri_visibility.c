#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

int hri_is_object_visible(HRI_AGENT * agent,p3d_rob *object, int threshold, int save)
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
  win->vs.fov            = agent->perspective->fov;
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
  g3d_set_camera_parameters_from_frame(agent->perspective->camjoint->abs_pos, win->vs);
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
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);
  
  //everything is ready now.
  g3d_is_object_visible_from_current_viewpoint(win, object,result,TRUE,(char*)"/Users/easisbot/Work/BioMove3D/screenshots");
  
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
  unsigned char *image= NULL;
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
  
  if(save){
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
  for(i=0; i<width*height; i++)
  {
    if(image[3*i]> 0.8) {
      idealpixels++;
    }
  }
  
  if(idealpixels!=0){    
    // display everything in blur except the object which is in red
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
  
  for (i=0; i<agents->all_agents_no; i++) {
    if(agents->all_agents[i]->perspective->enable_vision_draw)
      g3d_draw_agent_fov(agents->all_agents[i]);
    if(agents->all_agents[i]->perspective->enable_pointing_draw)
      g3d_draw_agent_pointing(agents->all_agents[i]);
  }
}
  

int g3d_draw_agent_pointing(HRI_AGENT *agent)
{
  GLdouble GreenColor[4] =   { 0.0, 0.5, 0.0, 0.5 };
  GLdouble GreenColorT[4] =   { 0.0, 0.5, 0.0, 0.0 };  
  
  if(agent!=NULL && agent->perspective->enable_pointing_draw){
    
    g3d_draw_visibility_by_frame(agent->perspective->pointjoint->abs_pos,
                                 DTOR(agent->perspective->point_tolerance), DTOR(agent->perspective->point_tolerance),
                                 2, GreenColor, GreenColorT);  
    return TRUE;
  }
  else {
    return FALSE;
  }

}

int g3d_draw_agent_fov(HRI_AGENT *agent)
{
  GLdouble GreenColor[4] =   { 0.0, 0.5, 0.0, 0.5 };
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


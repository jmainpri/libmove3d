#include "Util-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "GL/glx.h"

// Function pointer to link with a draw function working on the openGL backbuffer
void (*ext_g3d_draw_allwin_active_backbuffer)();

void hri_initialize_visibility()
{
  ext_g3d_draw_allwin_active_backbuffer = (void (*)())(g3d_draw_allwin_active_back_buffer);
}

int hri_is_object_visible(HRI_AGENT * agent, p3d_rob *object, int threshold, int save, int draw_at_end)
{
  g3d_win *win = g3d_get_win_by_name((char*)"Move3D");
  double result;
  int point, fov, visobj;

  if(object==NULL || agent==NULL){
    printf("%s: %d: g3d_is_object_visible_from_viewpoint(): input object is NULL.\n", __FILE__, __LINE__);
    return FALSE;
  }

  point = agent->perspective->enable_pointing_draw;
  fov = agent->perspective->enable_vision_draw;
  visobj = agent->perspective->enable_visible_objects_draw;

  agent->perspective->enable_pointing_draw = FALSE;
  agent->perspective->enable_vision_draw = FALSE;
  agent->perspective->enable_visible_objects_draw = FALSE;


  g3d_is_object_visible_from_viewpoint(agent->perspective->camjoint->abs_pos,
                                       agent->perspective->fov,
                                       object, &result, save);

  agent->perspective->enable_pointing_draw = point;
  agent->perspective->enable_vision_draw = fov;
  agent->perspective->enable_visible_objects_draw = visobj;

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
float elapsed_time_vis = 0;
int g3d_is_object_visible_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *object, double *result, int save)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");

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

  g3d_no_shader();

  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);

  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);

  //everything is ready now.

  g3d_is_object_visible_from_current_viewpoint(win, object, result, save, (char*)"/Users/easisbot/");

  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);

  if (win->vs.enableShaders) {
    g3d_use_shader();
  }

  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov

  return TRUE;
}

//! This function return how much % of the object is visible from a given viewpoint.
//! \param camera_frame the frame of the viewpoint (the looking direction is X, Y points downward and Z to the left)
//! \param camera_fov the field of view angle of the robot's camera (in degrees)
//! \param object pointer to the object
//! \param result return the ratio of the visibility of the object
//! \return TRUE in case of success, FALSE otherwise
int g3d_get_given_entities_pixelpresence_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, HRI_ENTITY **objects, int objects_nb, double *results, int save)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win = g3d_get_win_by_name((char*) "Move3D");

  if(objects==NULL) {
    printf("%s: %d: input objects are NULL.\n", __FILE__, __LINE__);
    return FALSE;
  }

  if(objects_nb == 0) {
    return TRUE;
  }

  // Change the size of the viewport if you want speed
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
  win->vs.cullingEnabled = 1;
  win->vs.enableLogo     = 0;

  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);

  // everything is ready now.
  g3d_get_given_entities_pixelpresence_in_current_viewpoint(win, objects, objects_nb, results, save, (char*)"/home/easisbot/");

  // restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);

  g3d_restore_win_camera(win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov

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

  ext_g3d_draw_allwin_active_backbuffer(); //only the object should be drawn in red, everthing else is black

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

    ext_g3d_draw_allwin_active_backbuffer();

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


int g3d_compute_visibility_for_given_entities(HRI_ENTITY ** ents, HRI_AGENT * agent, HRI_VISIBILITY * res, int ent_nb)
{
  double saved_pan_value, saved_tilt_value;
  configPt q;
  p3d_rob * robot;
  p3d_jnt * panJnt, * tiltJnt;
  HRI_ENTITY ** entities_to_test;
  int * entities_to_test_indexes;
  double * results;
  int i, o_i, j, t;
  double pan_max_head_turning, tilt_max_head_turning, pan_head_turning_iter, tilt_head_turning_iter;
  double visible_pixel_treshold = 0.001;
  double elevation, azimuth;
  int pan_div_no, tilt_div_no;
  int vis_pl;
  int save_images = FALSE;

  //TODO: Do not compute all visibility placements in each step
  //TODO: There should be a more intelligent way of dealing with the tilt angle
  //TODO: Here the fov of an agent is unchanged, since it has to move the head many times. If we can enlarge the fov it'll turn less -> less computation

  robot  = agent->robotPt;
  panJnt  = robot->joints[agent->perspective->pan_jnt_idx];
  tiltJnt = robot->joints[agent->perspective->tilt_jnt_idx];

  results = MY_ALLOC(double, ent_nb); // ALLOC
  entities_to_test = MY_ALLOC(HRI_ENTITY *, ent_nb); // ALLOC
  entities_to_test_indexes = MY_ALLOC(int, ent_nb); // ALLOC

  q = p3d_get_robot_config(robot);

  for(i=0; i<ent_nb; i++) {
    if(ents[i]->disappeared)
      res[i] = HRI_UK_VIS;
    else
      res[i] = HRI_INVISIBLE;
  }

  // SAVE AGENT PARAMETERS
  saved_pan_value = q[panJnt->index_dof];
  saved_tilt_value = q[tiltJnt->index_dof];

  // COMPUTE HOW MANY TIMES AGENTS NEEDS TO TURN HIS HEAD TO SEE ALL THE SCENE
  pan_max_head_turning = RTOD(panJnt->dof_data[0].vmax - panJnt->dof_data[0].vmin);
  pan_div_no = ceil(pan_max_head_turning/agent->perspective->fov);
  pan_head_turning_iter = (pan_max_head_turning-1)/pan_div_no; // -1 to be sure not to reach panJnt->dof_data[0].vmax
  tilt_max_head_turning = RTOD(tiltJnt->dof_data[0].vmax - tiltJnt->dof_data[0].vmin);
  tilt_div_no = ceil(tilt_max_head_turning/(agent->perspective->fov*0.75));
  tilt_head_turning_iter = (tilt_max_head_turning-1)/tilt_div_no; // -1 to be sure not to reach tiltJnt->dof_data[0].vmax

  // FOR EACH TILT
  for(t=0; t<tilt_div_no; t++) {
    q[tiltJnt->index_dof] = tiltJnt->dof_data[0].vmin +  t*tilt_head_turning_iter;    
    
    // 2- TURN HEAD STEP BY STEP
    for(j=0; j<pan_div_no; j++) {
      
      // TURN HEAD
      q[panJnt->index_dof] = panJnt->dof_data[0].vmin + j*pan_head_turning_iter;
      p3d_set_and_update_this_robot_conf(robot, q);
      
      // SELECT IN FOV/FOA OBJECTS
      for(o_i=0, i=0; i<ent_nb; i++) {
	if(res[i] == HRI_INVISIBLE) {
	  if(ents[i]->disappeared)
	    vis_pl= HRI_UK_VIS_PLACE;
	  else
	    hri_object_visibility_placement(agent, ents[i]->robotPt, &vis_pl, &elevation, &azimuth);
	  
	  if( ((vis_pl == HRI_FOV) || (vis_pl == HRI_FOA)) && (res[i] !=  HRI_VISIBLE) ) {
	    entities_to_test[o_i] = ents[i];
	    entities_to_test_indexes[o_i] = i;
	    o_i++;
	  }
	}
      }
      
      // TEST THEIR VISIBILITY
      g3d_get_given_entities_pixelpresence_from_viewpoint(agent->perspective->camjoint->abs_pos, agent->perspective->fov,
							  entities_to_test, o_i, results, save_images);
      //printf("%d. Number of tested entities: %d\n", j+2, o_i);
      
      // EVALUATE AND WRITE THE RESULT
      for(i=0; i<o_i; i++) {
	if(visible_pixel_treshold < results[i]) {
        res[entities_to_test_indexes[i]] = HRI_VISIBLE;
	}
      }
    }
  }
  
  // RESTORE AGENT HEAD POSITION
  q[panJnt->index_dof] = saved_pan_value; 
  q[tiltJnt->index_dof] = saved_tilt_value;
  p3d_set_and_update_this_robot_conf(robot, q);

  MY_FREE(results, double, ent_nb); // FREE
  MY_FREE(entities_to_test, HRI_ENTITY *, ent_nb); // FREE
  MY_FREE(entities_to_test_indexes, int, ent_nb); // FREE

  return TRUE;
}


/* Computes visibility in one image acquisition for given objects */
int g3d_visibility_for_given_objects_in_current_viewpoint_pixelpercentage(g3d_win* win, p3d_rob **objects, int objects_nb, double *res, int save, char *path)
{
  int idealpixels[objects_nb];
  int visiblepixels[objects_nb];
  unsigned char *image;
  int i, j, width, height;
  GLint viewport[4];
  static int crntcnt = 0;
  char name[256];
  double color[4]= {0,0,0,1};

  if(objects_nb == 0) {
    return TRUE;
  }

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

  // check if the image is empty
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

/* Computes visibility in one image acquisition for given objects */
int g3d_get_given_entities_pixelpresence_in_current_viewpoint(g3d_win* win, HRI_ENTITY **objects, int objects_nb,
                                                             double *vis_results, int save, char *path)
{
  unsigned char *image;
  int i, width, height;
  GLint viewport[4];
  static int crntcnt = 0;
  char name[256];
  double color[4]= {0,0,0,1};
  int *visiblepixels;

  if(objects_nb == 0) {
    return TRUE;
  }

  visiblepixels = MY_ALLOC(int, objects_nb); //ALLOC

  for (i=0; i<objects_nb; i++) {
    visiblepixels[i] = 0;
  }

  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];

  // get the OpenGL image buffer:
  image = MY_ALLOC(unsigned char, 3*width*height); //ALLOC
  glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration

  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);

  for(i=0; i<XYZ_ENV->no; i++) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_UNLIT_BLUE_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; i++) {
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_UNLIT_BLUE_DISPLAY);
  }

  // display all the objects in a single image in the different colors of red
  for(i=0; i<objects_nb; i++) {
    color[0]+=(10.0/255.0);
    if((objects[i]->type == HRI_AGENT_PART) || (objects[i]->type == HRI_OBJECT_PART))
      p3d_set_obj_display_mode(objects[i]->partPt, P3D_OBJ_UNLIT_CUSTOM_COLOR_DISPLAY, color);
    else
      p3d_set_robot_display_mode(objects[i]->robotPt, P3D_ROB_UNLIT_CUSTOM_COLOR_DISPLAY, color);
  }

  g3d_draw_win_back_buffer(win); //only the objects should be drawn in red, everthing else is blue

  if(save) {
    sprintf(name, "/%svisibilityview%i.ppm", path, crntcnt++);
    g3d_export_OpenGL_display(name);
  }

  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

  for(i=0; i<width*height; i++){
    if(image[3*i]>0) {
      visiblepixels[(int)((image[3*i])/10)-1]++;
    }
  }

  // visiblepixels present the pixels that we see and not blocked by obstructions
  for (i=0; i<objects_nb; i++) {
    vis_results[i] = ((double)visiblepixels[i])/(width*height);
    //if((objects[i]->type == HRI_AGENT_PART) || (objects[i]->type == HRI_OBJECT_PART))
    //  printf("tested Object name: %s pixel count:%d, vis result: %f\n", objects[i]->partPt->name, visiblepixels[i], vis_results[i]);
    //else
    //  printf("tested Object name: %s pixel count:%d, vis result: %f\n", objects[i]->robotPt->name, visiblepixels[i], vis_results[i]);
  }

  MY_FREE(image, unsigned char, 3*width*height); //FREE
  MY_FREE(visiblepixels, int, objects_nb); //FREE

  // reset the display modes of everything
  for(i=0; i<XYZ_ENV->no; i++){
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_DEFAULT_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; i++){
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

int hri_entity_visibility_placement(HRI_AGENT *agent, HRI_ENTITY *ent, int *result, double *elevation, double *azimuth)
{

  p3d_vector4 entCenter;
  p3d_BB * entBB;
  
  if(ent==NULL || agent==NULL){
    printf("%s: %d: hri_entity_visibility_placement(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }

  if((ent->type == HRI_OBJECT_PART) || (ent->type == HRI_AGENT_PART) )
    entBB = &ent->partPt->BB;
  else
    entBB = &ent->robotPt->BB;

  entCenter[0] = (((entBB->xmax - entBB->xmin)/2) + entBB->xmin);
  entCenter[1] = (((entBB->ymax - entBB->ymin)/2) + entBB->ymin);
  entCenter[2] = (((entBB->zmax - entBB->zmin)/2) + entBB->zmin);
  entCenter[3] = 1.0;

  g3d_object_visibility_placement(agent->perspective->camjoint->abs_pos, entCenter,
                                  DTOR(agent->perspective->fov),DTOR(agent->perspective->fov*0.75),
                                  DTOR(agent->perspective->foa),DTOR(agent->perspective->foa*0.75),
                                  result, elevation, azimuth);
  return TRUE;
}

int hri_object_visibility_placement(HRI_AGENT *agent, p3d_rob *object, int *result, double *elevation, double *azimuth)
{

  p3d_vector4 objectCenter;

  if(object==NULL || agent==NULL){
    printf("%s: %d: hri_object_visibility_placement(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }

  objectCenter[0] = (((object->BB.xmax - object->BB.xmin)/2) + object->BB.xmin);
  objectCenter[1] = (((object->BB.ymax - object->BB.ymin)/2) + object->BB.ymin);
  objectCenter[2] = (((object->BB.zmax - object->BB.zmin)/2) + object->BB.zmin);
  objectCenter[3] = 1.0;

  g3d_object_visibility_placement(agent->perspective->camjoint->abs_pos, objectCenter,
                                  DTOR(agent->perspective->fov),DTOR(agent->perspective->fov*0.75),
                                  DTOR(agent->perspective->foa),DTOR(agent->perspective->foa*0.75),
                                  result, elevation, azimuth);
  return TRUE;
}

int hri_object_pointing_placement(HRI_AGENT *agent, p3d_rob *object, int *result, double *elevation, double *azimuth)
{
  p3d_vector4 objectCenter;
  
  if(object==NULL || agent==NULL){
    printf("%s: %d: hri_object_pointing_placement(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }

  objectCenter[0] = (((object->BB.xmax - object->BB.xmin)/2) + object->BB.xmin);
  objectCenter[1] = (((object->BB.ymax - object->BB.ymin)/2) + object->BB.ymin);
  objectCenter[2] = (((object->BB.zmax - object->BB.zmin)/2) + object->BB.zmin);
  objectCenter[3] = 1.0;
  
  g3d_object_visibility_placement(agent->perspective->pointjoint->abs_pos, objectCenter,
                                  DTOR(agent->perspective->point_tolerance),DTOR(agent->perspective->point_tolerance),
                                  DTOR(agent->perspective->point_tolerance),DTOR(agent->perspective->point_tolerance),
                                  result, elevation, azimuth);
  return TRUE;
}


int g3d_object_visibility_placement(p3d_matrix4 camera_frame, p3d_vector4 objectCenter, double Hfov, double Vfov, double Hfoa, double Vfoa, int *result, double *phi_result, double *theta_result)
{
  p3d_vector4 objectCenterCamFr;
  p3d_matrix4 invM;
  double rho,phi,theta;
  
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
  win->vs.enableLogo     =  0;
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
  win->vs.cullingEnabled =  1;
  win->vs.enableLogo     =  0;
  agent->perspective->enable_pointing_draw = FALSE;
  agent->perspective->enable_vision_draw = FALSE;
  agent->perspective->enable_visible_objects_draw = FALSE;

  g3d_no_shader();

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

  g3d_visibility_for_given_objects_in_current_viewpoint_pixelpercentage(win, test_obj_list, obj_idx, vis_res, save, (char*)"/Users/easisbot/Work/BioMove3D/screenshots/");

  //restore viewport
  if(!save){
    glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  }
  g3d_load_state(win, &st);

  agent->perspective->enable_pointing_draw = point;
  agent->perspective->enable_vision_draw = fov;
  agent->perspective->enable_visible_objects_draw = visobj;

  if (win->vs.enableShaders) {
    g3d_use_shader();
  }

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

/* This function turns the head of the agent towards a given direction (direction is in spherical coord and is relative to the body) */
int hri_turn_agent_head_direction(HRI_AGENT *agent, double elevation, double azimuth)
{
  p3d_rob * robot = NULL;
  p3d_jnt *panJnt = NULL, *tiltJnt = NULL;
  configPt agent_q;

  // MOVE THE AGENT TO LOOK AT THE TARGET
  // Moving the head is done by moving pan and tilt joints only
  // For more compilated structures a better solution should be implemented
  robot  = agent->robotPt;
  panJnt  = robot->joints[agent->perspective->pan_jnt_idx];
  tiltJnt = robot->joints[agent->perspective->tilt_jnt_idx];

  if( (robot==NULL) || (panJnt==NULL) || (tiltJnt==NULL) ) {
    printf("In %s:%d, agent not correctly initialized\n", __FILE__, __LINE__);
    return FALSE;
  }

  agent_q = MY_ALLOC(double, robot->nb_dof); /* ALLOC */

  p3d_get_robot_config_into(robot, &agent_q);

  /* TURN PAN */
  azimuth = azimuth + agent_q[panJnt->index_dof]; // Relative to absolute angle

  if(panJnt->dof_data[0].vmin > azimuth) {
    if(panJnt->dof_data[0].vmax < azimuth - M_2PI) {
      if(ABS(panJnt->dof_data[0].vmin-azimuth) < ABS(panJnt->dof_data[0].vmax-azimuth))
        agent_q[panJnt->index_dof] = panJnt->dof_data[0].vmin;
      else
        agent_q[panJnt->index_dof] = panJnt->dof_data[0].vmax;
    }
    else {
      agent_q[panJnt->index_dof] = azimuth - M_2PI;
    }
  }
  else {
    if(panJnt->dof_data[0].vmax < azimuth) {
      if(panJnt->dof_data[0].vmin > azimuth - M_2PI) {
        if(ABS(panJnt->dof_data[0].vmin-azimuth) < ABS(panJnt->dof_data[0].vmax-azimuth))
          agent_q[panJnt->index_dof] = panJnt->dof_data[0].vmin;
        else
          agent_q[panJnt->index_dof] = panJnt->dof_data[0].vmax;
      }
      else {
        agent_q[panJnt->index_dof] = azimuth- M_2PI;
      }
    }
    else {
      agent_q[panJnt->index_dof] = azimuth;
    }
  }

  /* TURN TILT */
  elevation = elevation + agent_q[tiltJnt->index_dof]; // Relative to absolute angle

  if(tiltJnt->dof_data[0].vmin > elevation) {
    agent_q[tiltJnt->index_dof] = tiltJnt->dof_data[0].vmin;
  }
  else {
    if(tiltJnt->dof_data[0].vmax < elevation)
      agent_q[tiltJnt->index_dof] = tiltJnt->dof_data[0].vmax;
    else
      agent_q[tiltJnt->index_dof] = elevation;
  }

  p3d_set_and_update_this_robot_conf(robot, agent_q);

  MY_FREE(agent_q, double, robot->nb_dof); /* FREE */

  return TRUE;
}


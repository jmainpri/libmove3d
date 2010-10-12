#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Util-pkg.h"

#include "../graphic/proto/g3d_logo.h"

#ifdef USE_SHADERS
 #include <GL/glew.h>

 GLuint G3D_PROGRAMS[100];
 unsigned int G3D_NB_PROGRAMS;
 unsigned int G3D_CURRENT_PROGRAM;
#endif

//! Call this function just after the creation of a g3d_states variable.
g3d_states g3d_init_viewer_state(double size)
{
    g3d_states vs;

    vs.size       = size;
    vs.FILAIRE = 0;
    vs.CONTOUR = 0;
    vs.GHOST = 0;
    vs.BB = 0;
    vs.GOURAUD = 1;
    vs.ACTIVE = 1;
    vs.list = -1;

    vs.fov= 55;
    vs.projection_mode = G3D_PERSPECTIVE;
    vs.transparency_mode = G3D_TRANSPARENT_AND_OPAQUE;

    vs.frustum[0][0]= 0.0;  vs.frustum[0][1]= 0.0;  vs.frustum[0][2]= 0.0;  vs.frustum[0][3]= 0.0;
    vs.frustum[1][0]= 0.0;  vs.frustum[1][1]= 0.0;  vs.frustum[1][2]= 0.0;  vs.frustum[1][3]= 0.0;
    vs.frustum[2][0]= 0.0;  vs.frustum[2][1]= 0.0;  vs.frustum[2][2]= 0.0;  vs.frustum[2][3]= 0.0;
    vs.frustum[3][0]= 0.0;  vs.frustum[3][1]= 0.0;  vs.frustum[3][2]= 0.0;  vs.frustum[3][3]= 0.0;
    vs.frustum[4][0]= 0.0;  vs.frustum[4][1]= 0.0;  vs.frustum[4][2]= 0.0;  vs.frustum[4][3]= 0.0;
    vs.frustum[5][0]= 0.0;  vs.frustum[5][1]= 0.0;  vs.frustum[5][2]= 0.0;  vs.frustum[5][3]= 0.0;

    //cam_frame  = &Id;
    
    vs.floorColor[0]= 0.5;
    vs.floorColor[1]= 0.9;
    vs.floorColor[2]= 0.9;
    vs.wallColor[0]= 0.5;
    vs.wallColor[1]= 0.5;
    vs.wallColor[2]= 0.6;
    vs.eventsEnabled= 1;
    vs.cullingEnabled = 0;
    vs.displayFrame = 1;
    vs.displayJoints = 0;
    vs.enableLight = 1;
    vs.cameraBoundedLight = 1;   
    vs.displayShadows = 0;
    vs.displayWalls = 0;
    vs.displayFloor = 0;
    vs.displayTiles = 0;
    vs.enableAntialiasing = 0;
    #ifdef USE_SHADERS
    vs.enableShaders = 1;
    #else
    vs.enableShaders = 0;
    #endif
    vs.allIsBlack = 0;

    //Les plans du sol et des murs vont être ajustés sur les coordonnées de
    //l'environment_box.
    double xmin, xmax, ymin, ymax, zmin, zmax;
    p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

    if( xmin>=xmax || ymin>=ymax || zmin>=zmax)
    {
            printf("%s: %d: g3d_new_win(): mauvais paramètres pour la commande p3d_set_env_box.\n\t", __FILE__, __LINE__);
            printf("Il faut les donner sous la forme xmin ymin zmin xmax ymax zmax.\n");
    }


    GLdouble v0[3], v1[3], v2[3];

    //plan du sol (normale selon Z):
    v0[0]= xmin;      v1[0]= xmax;      v2[0]= xmin;
    v0[1]= ymin;      v1[1]= ymin;      v2[1]= ymax;
    v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmin;
    g3d_findPlane(vs.floorPlane, v0, v1, v2);

    //plan du premier mur (normale selon y):
    v0[0]= xmin;      v1[0]= xmin;      v2[0]= xmax;
    v0[1]= ymin;      v1[1]= ymin;      v2[1]= ymin;
    v0[2]= zmin;      v1[2]= zmax;      v2[2]= zmin;
    g3d_findPlane(vs.wallPlanes[0], v0, v1, v2);

    //plan du deuxième mur (normale selon -y):
    v0[0]= xmin;      v1[0]= xmax;      v2[0]= xmin;
    v0[1]= ymax;      v1[1]= ymax;      v2[1]= ymax;
    v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmax;
    g3d_findPlane(vs.wallPlanes[1], v0, v1, v2);

    //plan du troisième mur (normale selon x):
    v0[0]= xmin;      v1[0]= xmin;      v2[0]= xmin;
    v0[1]= ymin;      v1[1]= ymax;      v2[1]= ymin;
    v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmax;
    g3d_findPlane(vs.wallPlanes[2], v0, v1, v2);


    //plan du quatrième mur (normale selon -x):
    v0[0]= xmax;      v1[0]= xmax;      v2[0]= xmax;
    v0[1]= ymin;      v1[1]= ymax;      v2[1]= ymax;
    v0[2]= zmin;      v1[2]= zmax;      v2[2]= zmin;
    g3d_findPlane(vs.wallPlanes[3], v0, v1, v2);

    //positionnement de la lumière:
    vs.lightPosition[0]= 0.5*(xmin+xmax);
    vs.lightPosition[1]= 0.5*(ymin+ymax);
    vs.lightPosition[2]= 0.9*(zmin+zmax);
    vs.lightPosition[3]= 1.0;

    vs.logoTexture = 0;
    vs.enableLogo  = 1;

    return vs;
}

//! Call this function before the destruction of a g3d_states variable.
int g3d_free_viewer_state(g3d_states vs)
{
  if(vs.logoTexture!=0) {
    glDeleteTextures(1, &(vs.logoTexture));
  }

  return 0;
}

//-----------------------------------------------------------------------------
// Name: findPlane()
// Desc: find the plane equation given 3 points
//-----------------------------------------------------------------------------
void g3d_findPlane( GLdouble plane[4], GLdouble v0[3], GLdouble v1[3], GLdouble v2[3] )
{
  GLdouble vec0[3], vec1[3];

  // Need 2 vectors to find cross product
  vec0[0] = v1[0] - v0[0];
  vec0[1] = v1[1] - v0[1];
  vec0[2] = v1[2] - v0[2];

  vec1[0] = v2[0] - v0[0];
  vec1[1] = v2[1] - v0[1];
  vec1[2] = v2[2] - v0[2];

  // Find cross product to get A, B, and C of plane equation
  plane[0] =   vec0[1] * vec1[2] - vec0[2] * vec1[1];
  plane[1] = -(vec0[0] * vec1[2] - vec0[2] * vec1[0]);
  plane[2] =   vec0[0] * vec1[1] - vec0[1] * vec1[0];

  plane[3] = -(plane[0] * v0[0] + plane[1] * v0[1] + plane[2] * v0[2]);
}

// Construit les matrices de projection des ombres sur les plans du sol et des murs.
// Cette fonction doit être appelée chaque fois que la position de la lumière change.
void g3d_build_shadow_matrices(g3d_states &vs)
{
  buildShadowMatrix( vs.floorShadowMatrix, vs.lightPosition, vs.floorPlane );
  buildShadowMatrix( vs.wallShadowMatrix[0], vs.lightPosition, vs.wallPlanes[0] );
  buildShadowMatrix( vs.wallShadowMatrix[1], vs.lightPosition, vs.wallPlanes[1] );
  buildShadowMatrix( vs.wallShadowMatrix[2], vs.lightPosition, vs.wallPlanes[2] );
  buildShadowMatrix( vs.wallShadowMatrix[3], vs.lightPosition, vs.wallPlanes[3] );
}

void
g3d_set_win_bgcolor(g3d_states &vs, float r, float v, float b) {
  vs.bg[0] = r;
  vs.bg[1] = v;
  vs.bg[2] = b;
}

void
g3d_set_win_floor_color(g3d_states &vs, float r, float v, float b) {
  vs.floorColor[0] = r;
  vs.floorColor[1] = v;
  vs.floorColor[2] = b;
}

void
g3d_set_win_wall_color(g3d_states &vs, float r, float v, float b) {
  vs.wallColor[0] = r;
  vs.wallColor[1] = v;
  vs.wallColor[2] = b;
}


void
g3d_set_win_camera(g3d_states &vs, float ox,float oy, float oz,
                   float dist, float az, float el,
                   float up0, float up1, float up2) {
  vs.x = ox;
  vs.y = oy;
  vs.z = oz;
  vs.zo = dist;
  vs.az = az;
  vs.el = el;
  vs.up[0] = up0;
  vs.up[1] = up1,
  vs.up[2] = up2;
  vs.up[3]=0.0;
}

void g3d_print_win_camera(g3d_states &vs)
{
  printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n",vs.x, vs.y, vs.z, vs.zo, vs.az, vs.el, vs.up[0], vs.up[1], vs.up[2]);
}

void
g3d_set_win_center(g3d_states &vs, float ox,float oy, float oz) {
  vs.x = ox;
  vs.y = oy;
  vs.z = oz;
}

//! @ingroup graphic
//! Sets the default light parameters.
void g3d_set_light(g3d_states &vs)
{
//   GLfloat light_ambient[4] = { 0.3f, 0.3f, 0.3f, 1.0f };
//   GLfloat light_diffuse[4] = { 0.4f, 0.4f, 0.4f, 1.0f };
//   GLfloat light_specular[4]= { 0.9f, 0.9f, 0.9f, 1.0f };
//   glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
//   glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
//   glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);


    GLfloat ambientLight[] = {0.2f, 0.2f, 0.2f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    GLfloat lightColor[] = {0.6f, 0.6f, 0.6f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor);
    glLightfv(GL_LIGHT0, GL_POSITION, vs.lightPosition);
}


void
g3d_save_win_camera(g3d_states &vs) {
  int i;

  vs.sx = vs.x;
  vs.sy = vs.y;
  vs.sz = vs.z;
  vs.szo = vs.zo;
  vs.saz = vs.az, vs.sel = vs.el;
  for(i=0;i<4;i++) vs.sup[i] = vs.up[i];
}

void
g3d_restore_win_camera(g3d_states &vs) {
  int i;

  vs.x = vs.sx;
  vs.y = vs.sy;
  vs.z = vs.sz;
  vs.zo = vs.szo;
  vs.az = vs.saz, 
  vs.el = vs.sel;
  for(i=0;i<4;i++) vs.up[i] = vs.sup[i];
}

void
g3d_load_saved_camera_params(double* params)
{
  g3d_states vs =  g3d_get_states_by_name((char*)"Move3D");
  int i;

  vs.sx = params[0];
  vs.sy = params[1];
  vs.sz = params[2];
  vs.szo = params[3];
  vs.saz = params[4], vs.sel = params[5];
  for(i=0;i<4;i++) vs.sup[i] = params[6+i];
}

/* fonctions pour copier les donnees relies a la camera de     */
/* la structure G3D_Window de facon utilisable dans operations */
/* avec transformations homogenes                              */
void
get_lookat_vector(g3d_states &vs, p3d_vector4 Vec) {
  Vec[0] = vs.x;
  Vec[1] = vs.y;
  Vec[2] = vs.z;
  Vec[3] = 1.0;
}

void
get_pos_cam_matrix(g3d_states &vs, p3d_matrix4 Transf) {
  /* Caution: ici on change les parametres de translation de la */
  /* matrix, les rest des elementes doivent etre initialises    */
  /* dans la fonction qu'appel                                  */
  Transf[0][3] = vs.zo * (cos(vs.az)*cos(vs.el));
  Transf[1][3] = vs.zo * (sin(vs.az)*cos(vs.el));
  Transf[2][3] = vs.zo * sin(vs.el);
}

//! Moves the camera in its look direction while removing the motions along Z axis.
//! \param d the length of the desired camera motion
void g3d_move_win_camera_forward(g3d_states &vs, float d )
{
        vs.x = vs.x - cos(vs.az)*d;
        vs.y = vs.y - sin(vs.az)*d;
}


//! \param d the length of the desired camera motion
void g3d_move_win_camera_sideways(g3d_states &vs, float d )
{
        vs.x = vs.x + sin(vs.az)*d;
        vs.y = vs.y - cos(vs.az)*d;
}

//! Rotates the camera around Z axis
//! \param d the angle of the desired camera rotation
void g3d_rotate_win_camera_rz(g3d_states &vs, float d )
{
        vs.az = vs.az - d;
}

//! Performs a camera zoom.
//! \param d the "distance" of the zoom
void g3d_zoom_win_camera(g3d_states &vs, float d )
{
   vs.zo = vs.zo - d;
   if(vs.zo < 0.1) vs.zo= 0.1;
}

/* fonction pour recalculer le vector 'up' de la camera */
/* quand on change la reference                         */
void
recalc_cam_up(g3d_states &vs, p3d_matrix4 transf) {
  p3d_vector4 v_up;
  int i;

  p3d_matvec4Mult(transf,vs.up,v_up);
  for(i=0;i<4;i++) vs.up[i] = v_up[i];
}

/* fonction qui change les parametres de position de la       */
/* camera qui son controles pour la souris quand on change    */
/* la reference                                               */
void
recalc_mouse_param(g3d_states &vs, p3d_vector4 Xc, p3d_vector4 Xw) {
  double incx,incy,incz,incd;
  double azim,elev;

  vs.x = Xw[0];
  vs.y = Xw[1];
  vs.z = Xw[2];

  incx = Xc[0] - Xw[0];
  incy = Xc[1] - Xw[1];
  incz = Xc[2] - Xw[2];
  incd = sqrt(SQR(incx)+SQR(incy));
  azim = atan2(incy,incx);
  elev = atan2(incz,incd);
  /* set azim and elev in right range */
  azim = angle_limit_2PI(azim);

  if(elev>M_PI/2) {
    elev = M_PI - elev;
    if(azim>M_PI) azim -= M_PI;
    else azim += M_PI;
  }
  if(elev<-M_PI/2) {
    elev = -M_PI - elev;
    if(azim>M_PI) azim -= M_PI;
    else azim += M_PI;
  }
  vs.az = azim;
  vs.el = elev;
}

//! @ingroup graphic
//! Initializes OpenGL main parameters.
void g3d_init_OpenGL()
{
  glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.2, 1.2);

  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_DEPTH_TEST);
#ifdef ENABLE_ANTIALIASING
  glDisable(GL_MULTISAMPLE_ARB);
#endif
  /** on desactive tout mode OpenGL inutile ***/
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_ALPHA_TEST);


  #ifdef USE_SHADERS
  char *move3Dpath= NULL;
  char path[128];
//   std::list<std::string> vsname, fsname;
  char vsname[128], fsname[128];

  /* initialize  glew */
  static int firstTime= 1;
  
  if(firstTime)
  {
    firstTime= 0;
    glewInit();
      
    if(!g3d_init_extensions())
    { return;  }
  
    move3Dpath= getenv("HOME_MOVE3D");

    if(move3Dpath==NULL)  {
      strcpy(path, "..");
    }
    else  { 
      strcpy(path, move3Dpath);
    }

   G3D_NB_PROGRAMS= 0;
   sprintf(vsname, "%s/graphic/shaders/default.vert", path);
   sprintf(fsname, "%s/graphic/shaders/default.frag", path);
   G3D_PROGRAMS[G3D_NB_PROGRAMS++] = g3d_load_program(vsname, fsname);

   sprintf(vsname, "%s/graphic/shaders/cell_shading.vert", path);
   sprintf(fsname, "%s/graphic/shaders/cell_shading.frag", path);
   G3D_PROGRAMS[G3D_NB_PROGRAMS++] = g3d_load_program(vsname, fsname);

   sprintf(vsname, "%s/graphic/shaders/normals.vert", path);
   sprintf(fsname, "%s/graphic/shaders/normals.frag", path);
   G3D_PROGRAMS[G3D_NB_PROGRAMS++] = g3d_load_program(vsname, fsname);

   sprintf(vsname, "%s/graphic/shaders/default.vert", path);
   sprintf(fsname, "%s/graphic/shaders/depth.frag", path);
   G3D_PROGRAMS[G3D_NB_PROGRAMS++] = g3d_load_program(vsname, fsname);

   G3D_CURRENT_PROGRAM= 0;

   glUseProgram( G3D_PROGRAMS[G3D_CURRENT_PROGRAM] );
  }
  glEnable(GL_MULTISAMPLE_ARB);

  #endif
}

#ifdef USE_SHADERS
int g3d_load_next_shader()
{
  G3D_CURRENT_PROGRAM++;

  if(G3D_CURRENT_PROGRAM > G3D_NB_PROGRAMS-1) {
    G3D_CURRENT_PROGRAM= 0;
  }

  glUseProgram( G3D_PROGRAMS[G3D_CURRENT_PROGRAM] );

  return 0;
}

//! Deactivates the use of shaders.
int g3d_no_shader()
{
  glUseProgram(0);

  return 0;
}

//! Activates the use of shaders.
int g3d_use_shader()
{
  glUseProgram(G3D_PROGRAMS[G3D_CURRENT_PROGRAM]);

  return 0;
}

#endif


//! @ingroup graphic
//! Sets the OpenGL projection matrix used by default by the g3d_windows.
//! Use this function instead of calling directly gluPerspective (unlesss you want some specific parameters)
//! to avoid dispersion of the same setting code.
//! \param mode projection mode (perspective or orthographic)
void g3d_set_projection_matrix(g3d_projection_mode mode)
{
  GLint current_mode;
  GLint viewport[4], width, height;
  GLdouble ratio, d;

  g3d_states vs = g3d_get_cur_states();

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetIntegerv(GL_MATRIX_MODE, &current_mode);

  width = viewport[2];
  height= viewport[3];

  ratio= ((GLdouble) width)/((GLdouble) height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  switch(mode)
  {
    case G3D_PERSPECTIVE:
      gluPerspective(vs.fov, ratio, vs.zo/500.0, 100.0*vs.zo);
    break;
    case G3D_ORTHOGRAPHIC:
      d= vs.zo;
      glOrtho(-ratio*d, ratio*d, -d, d, -10*d, 10*d);
    break;
  }

  glMatrixMode(current_mode);
}

//! @ingroup graphic
//! Saves the current OpenGL pixel buffer as a ppm (PortablePixMap) image file (uncompressed format).
//! In other words: takes a screenshot of the current active OpenGL window.
//! \param filename name of the image file where to save the pixel buffer
//! \return 1 in case of success, 0,otherwise
int g3d_export_OpenGL_display(char *filename)
{
  size_t length;
  unsigned int i, j, width, height, change_name= 0;
  GLint viewport[4];
  char filename2[128];
  unsigned char *pixels= NULL;
  unsigned char *pixels_inv= NULL;
  FILE *file= NULL;

  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];

  pixels    = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  pixels_inv= (unsigned char*) malloc(3*width*height*sizeof(unsigned char));

  length= strlen(filename);

  if(length < 5)
  {  change_name= 1;  }
  else if(filename[length-4]!='.' || filename[length-3]!='p' || filename[length-2]!='p' || filename[length-1]!='m')
  {  change_name= 1;  }

  if(change_name)
  {
    printf("%s: %d: file \"%s\" does not have the good extension (it should be a .ppm file).\n",__FILE__,__LINE__, filename);
    strcpy(filename2, filename);
    strcat(filename2, ".ppm");
    printf("It is renamed \"%s\".\n",filename2);
  }
  else
  { strcpy(filename2, filename);  }

  file= fopen(filename2,"w");


  if(file==NULL)
  {
    printf("%s: %d: can not open \"%s\".\n",__FILE__,__LINE__,filename2);
    fclose(file);
    return 0;
  }

  glReadBuffer(GL_BACK); // use back buffer as we are in a double-buffered configuration

  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  // get the image pixels (from (0,0) position):
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

  // glReadPixels returns an upside-down image.
  // we have to first flip it
  // NB: in pixels the 3 colors of a pixel follows each other immediately (RGBRGBRGB...RGB).
  for(i=0; i<width; i++)
  {
    for(j=0; j<height; j++)
    {
      pixels_inv[3*(i+j*width)]  = pixels[3*(i+(height-1-j)*width)+0];
      pixels_inv[3*(i+j*width)+1]= pixels[3*(i+(height-1-j)*width)+1];
      pixels_inv[3*(i+j*width)+2]= pixels[3*(i+(height-1-j)*width)+2];
    }
  }

  fprintf(file, "P6\n");
  fprintf(file, "# creator: BioMove3D\n");
  fprintf(file, "%d %d\n", width, height);
  fprintf(file, "255\n");

  fwrite(pixels_inv, sizeof(unsigned char), 3*width*height, file);

  fclose(file);

  free(pixels);
  free(pixels_inv);

  return 1;
}

//! @ingroup graphic
//! Sets the light parameters for things that will be displayed in the shadow.
void g3d_set_dim_light()
{
  g3d_states vs = g3d_get_cur_states();

  GLfloat light_ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
  GLfloat light_diffuse[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
  GLfloat light_specular[4]= { 0.3f, 0.3f, 0.3f, 1.0f };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, vs.lightPosition);
}

//! @ingroup graphic
//! Sets the default material parameters for OpenGL.
void g3d_set_default_material()
{
  GLfloat mat_ambient[4] = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat mat_diffuse[4] = { 0.4f, 0.4f, 0.4f, 1.0f };
  GLfloat mat_specular[4]= { 0.8f, 0.8f, 0.8f, 1.0f };
  GLfloat mat_emission[4]= { 0.05f, 0.05f, 0.05f, 1.0f };
  GLfloat shininess = 90.0f;

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

//    GLfloat specularity = 0.3f;
//     GLfloat emissivity = 0.05f;
//     GLfloat shininess = 10.0f;
//     GLfloat materialColor[] = {0.2f, 0.2f, 1.0f, 1.0f};
//     //The specular (shiny) component of the material
//     GLfloat materialSpecular[] = {specularity, specularity, specularity, 1.0f};
//     //The color emitted by the material
//     GLfloat materialEmission[] = {emissivity, emissivity, emissivity, 1.0f};
//
//     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, materialColor);
//     glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
//     glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
//     glMaterialf(GL_FRONT, GL_SHININESS, shininess); //The shininess parameter

      /////////////   From xavier
//  GLfloat mat_ambient[4] = { 0.7f, 0.7f, 0.7f, 1.0f };
//  GLfloat mat_diffuse[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
//  GLfloat mat_specular[4]= { 0.5f, 0.5f, 0.5f, 1.0f };
//  GLfloat mat_emission[4]= { 0.2f, 0.2f, 0.2f, 1.0f };
//  GLfloat shininess = 60.0f;
//
//  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
//  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
//  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
//  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
//  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
//
//     GLfloat specularity = 0.3f;
//     GLfloat emissivity = 0.05f;
//     GLfloat shininess = 25.0f;
//     GLfloat materialColor[] = {0.2f, 0.2f, 1.0f, 1.0f};
//     //The specular (shiny) component of the material
//     GLfloat materialSpecular[] = {specularity, specularity, specularity, 1.0f};
//     //The color emitted by the material
//     GLfloat materialEmission[] = {emissivity, emissivity, emissivity, 1.0f};
//
//     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, materialColor);
//     glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
//     glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
//     glMaterialf(GL_FRONT, GL_SHININESS, shininess); //The shininess parameter

}

//! @ingroup graphic
//! Set the material parameters for things that are in the shadow (floor or wall part) for OpenGL.
void g3d_set_shade_material()
{
  GLfloat mat_ambient[4]    = { 0.5f, 0.5f, 0.5f, 1.0f };
  GLfloat mat_diffuse[4]    = { 0.4f, 0.4f, 0.4f, 1.0f };
  GLfloat mat_specular[4]   = { 0.2f, 0.2f, 0.2f, 1.0f };
  GLfloat mat_emission[4]   = { 0.05f, 0.05f, 0.05f, 1.0f };
  GLfloat shininess = 10.0f;

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}

//! @ingroup graphic
//! The original function of Move3D that displays a frame with black flat arrows.
void g3d_draw_frame(void) {
//   GLdouble mat_ambient_diffuse[4]= { 0., .0, .0, 1.0 };
  g3d_states vs  = g3d_get_cur_states();

  double a= vs.size;
  double a1,a9;

  a1 = .1 * a;
  a9 = .9 * a;

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT | GL_LINE_BIT);

  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);
  glColor3d(0.,0.,0.);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glVertex3d(.0, .0, .0);
  glVertex3d(.0, .0, a);
  glVertex3d(.0, .0, .0);
  glVertex3d(a, .0, .0);
  glVertex3d(.0, .0, .0);
  glVertex3d(.0, a, .0);
  glEnd();
  glLineWidth(1.0);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glDisable(GL_CULL_FACE);
  glBegin(GL_POLYGON);
  glVertex3d(.0, .0, a);
  glVertex3d(-a1, .0, a9);
  glVertex3d(a1, .0, a9);
  glEnd();

  glBegin(GL_POLYGON);
  glVertex3d(a, .0, .0);
  glVertex3d(a9, .0, -a1);
  glVertex3d(a9, .0, a1);
  glEnd();

  glBegin(GL_POLYGON);
  glVertex3d(.0, a, .0);
  glVertex3d(.0, a9, -a1);
  glVertex3d(.0, a9, a1);
  glEnd();

 glPopAttrib();
}

//! @ingroup graphic
//! Sets the camera parameters (focus point position, zoom factor, elevation and azimuth angles) from
//! a frame matrix. The camera will look in the direction of the X axis of the frame, with the Z axis pointing to the left
//! and the Y axis pointg downward.
//! This function is based on the conventions used in calc_cam_param and get_pos_cam_matrix.
//! \param frame desired pose of the camera.
//! \param vs camera parameters output 
//! \return 0 in case of success, 1 otherwise
int g3d_set_camera_parameters_from_frame(p3d_matrix4 frame, g3d_states &vs)
{ 
  p3d_vector3 focus, position, diff;
  double d= 0.2;

  position[0]= frame[0][3];
  position[1]= frame[1][3];
  position[2]= frame[2][3];


  focus[0]= position[0] + d*frame[0][0];
  focus[1]= position[1] + d*frame[1][0];
  focus[2]= position[2] + d*frame[2][0];

  p3d_vectSub(position, focus, diff);

  vs.x= focus[0];
  vs.y= focus[1];
  vs.z= focus[2];

  vs.zo= p3d_vectNorm(diff);
  vs.az= atan2(diff[1], diff[0]);
  vs.el= asin(diff[2]/vs.zo);
  if(diff[2]>0) {
    vs.el= fabs(vs.el);
  }
  else {
    vs.el= -fabs(vs.el);
  } 

  return 0;
}

//! @ingroup graphic
//! Saves the viewer state of a given window
//! This function DOES NOT save all the contents of the state. It's created for a specific use.
//! \param win windows
//! \param st viewer state 
//! \return 1 in case of success, 0 means failure
int g3d_save_state(g3d_win *win, g3d_states *st)
{
  
  if(win==NULL || st==NULL){
    return FALSE;
  }
  
  //save useful things
  st->fov                =  win->vs.fov;
  st->displayFrame       =  win->vs.displayFrame;
  st->displayJoints      =  win->vs.displayJoints;
  st->displayShadows     =  win->vs.displayShadows;
  st->displayWalls       =  win->vs.displayWalls;
  st->displayFloor       =  win->vs.displayFloor;
  st->enableAntialiasing =  win->vs.enableAntialiasing;
  st->enableShaders      =  win->vs.enableShaders;
  st->displayTiles       =  win->vs.displayTiles;
  st->cullingEnabled     =  win->vs.cullingEnabled;
  st->enableLogo         =  win->vs.enableLogo;
  st->bg[0]              =  win->vs.bg[0]; 
  st->bg[1]              =  win->vs.bg[1]; 
  st->bg[2]              =  win->vs.bg[2]; 
  
  return TRUE;
}

//! @ingroup graphic
//! Loads the viewer state of a given window
//! This function DOES NOT load all the contents of the state. It's created for a specific use.
//! \param win windows
//! \param st viewer state 
//! \return 1 in case of success, 0 means failure
int g3d_load_state(g3d_win *win, g3d_states *st)
{
   
  if(win==NULL || st==NULL){
    return FALSE;
  }
  
  //load useful things
  win->vs.fov = st->fov;
  win->vs.displayFrame    = st->displayFrame; 
  win->vs.displayJoints   = st->displayJoints;
  win->vs.displayShadows  = st->displayShadows; 
  win->vs.displayWalls    = st->displayWalls; 
  win->vs.displayFloor    = st->displayFloor;
  win->vs.enableAntialiasing = st->enableAntialiasing;
  win->vs.displayTiles    = st->displayTiles;
  win->vs.cullingEnabled  = st->cullingEnabled;
  win->vs.enableShaders   = st->enableShaders;
  win->vs.enableLogo      = st->enableLogo;
  win->vs.bg[0]           = st->bg[0];
  win->vs.bg[1]           = st->bg[1];
  win->vs.bg[2]           = st->bg[2];
  
  return TRUE;
}

//! Call this function to see what is the latest error reported by OpenGL.
//! \param a additional message (it can be left to NULL)
//! \return O if there was no error, 1 otherwise
int g3d_checkGLerrors(char *message)
{
  int result= 0;
  GLenum error;

  while ((error = glGetError()) != GL_NO_ERROR) {
    if(message==NULL)
    {
      fprintf(stderr, "OpenGL: Error: %s\n", (char *) gluErrorString(error));
    }
    else
    {
      fprintf(stderr, "%s OpenGL: Error: %s\n", message, (char *) gluErrorString(error));
    }
    result= 1;
  }
  
  return result;
}

//! Loads the logo contained in g3d_logo.h
//! \param vs the g3d_states that will contained the texture name
//! \return 0 in case of success, 1 otherwise
int g3d_load_logo_texture(g3d_states &vs)
{
  unsigned char pixel[3];
  glGenTextures(1, &(vs.logoTexture));

  glBindTexture(GL_TEXTURE_2D, vs.logoTexture);


  unsigned char *data= new unsigned char[4*LOGO_WIDTH*LOGO_HEIGHT];

  for(unsigned int i= 0; i<LOGO_WIDTH*LOGO_HEIGHT; ++i)
  {
    LOGO_PIXEL(LOGO_DATA, pixel);
    data[4*i]     = pixel[0];
    data[4*i + 1] = pixel[1];
    data[4*i + 2] = pixel[2];
    data[4*i + 3] = 128;
  }

  //set pixel unpacking mode
  glPixelStorei(GL_UNPACK_SWAP_BYTES, 0);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
  
  glTexImage2D (GL_TEXTURE_2D, 0, 3, LOGO_WIDTH, LOGO_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
  delete [] data;

  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  return 0;
}


//! Displays the LAAS logo.
//! \param offsetX X position of the logo lower-left corner (from the image lower-left corner)
//! \param offsetY Y position of the logo lower-left corner (from the image lower-left corner)
//! \param scale scale factor to apply to the logo (original size in g3d_logo.h)
//! \return 0 in case of success, 1 otherwise
int g3d_display_logo(g3d_states &vs, float offsetX, float offsetY, float scale)
{
  static int firstTime= TRUE;

  if(firstTime==TRUE)
  {
    firstTime= FALSE;
    // create the texture used to display the LAAS logo:
    g3d_load_logo_texture(vs);
   }

  GLint viewport[4];
  int width, height;

  glGetIntegerv(GL_VIEWPORT, viewport);
  width  = viewport[2];
  height = viewport[3];
 #ifdef USE_SHADERS
  g3d_no_shader();
 #endif
  glPushAttrib(GL_ENABLE_BIT | GL_TRANSFORM_BIT);
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
    glLoadIdentity();
    glOrtho(0, width, 0, height, -1, 1);
    glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity(); 
      glEnable(GL_BLEND);
      glDisable(GL_LIGHTING); 
      glDisable(GL_DEPTH_TEST);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);	 
      glEnable(GL_TEXTURE_2D);
       glColor4f(1.0, 1.0, 1.0, 0.5);
      glBindTexture(GL_TEXTURE_2D, vs.logoTexture);
      glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
      glBegin (GL_QUADS);
        glTexCoord2f(1.0f,1.0f);  glVertex2f(offsetX + scale*LOGO_WIDTH, offsetY);
        glTexCoord2f(1.0f,0.0f);  glVertex2f(offsetX + scale*LOGO_WIDTH, offsetY + scale*LOGO_HEIGHT);
        glTexCoord2f(0.0f,0.0f);  glVertex2f(offsetX, offsetY + scale*LOGO_HEIGHT);
        glTexCoord2f(0.0f,1.0f);  glVertex2f(offsetX, offsetY);
      glEnd(); 
 
      glPopMatrix(); 
    glMatrixMode(GL_PROJECTION);
   glPopMatrix();
 glPopAttrib();

 #ifdef USE_SHADERS
  g3d_use_shader();
 #endif

 return 0;
}


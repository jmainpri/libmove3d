/*************************************************************************/
/***********Fonctions derivees des fonctions de base de Move3D************/
/***********     ou pouvant en constituer des extensions      ************/
/*************************************************************************/

#include <math.h>
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "GraspPlanning-pkg.h"
#include <string.h>
#include <string>
#include <stdarg.h>
#include <iostream>

//! @ingroup graspPlanning 
//! Cette fonction s'utilise comme un printf mais ecrit dans le fichier logfile.
//! La premiere fois qu'elle est appelee, elle ecrase le contenu precedent du fichier.
//! This function is used like a printf but writes in a file named "logfile".
//! The first time it is called, the previous content of the file is erased.
void logfile(const char *format,...)
{
  static int firstTime= 1;
  FILE *file= NULL;
  if(firstTime)
  {
     firstTime= 0;
     file= fopen("logfile","w");
  }
  else
  {
     file= fopen("logfile","a+");
  }

  if(file==NULL)
  { printf("%s: %d: logfile(): erreur d'ouverture de fichier.\n", __FILE__, __LINE__);
    return; }

  va_list args;
  va_start(args,format);
  vfprintf(file,format,args);
  va_end(args);

  fclose(file);
}



//! @ingroup graspPlanning 
//! This function just gets the pose of the given object.
int p3d_get_obj_pos(p3d_obj *o, p3d_matrix4 pose)
{
   #ifdef GP_DEBUG
    if(o==NULL)
    {
      printf("%s: %d: p3d_get_obj_pose(): input is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(o->pol[0]==NULL)
    {
      printf("%s: %d: p3d_get_obj_pose(): the field pol[0] of the input object is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(o->pol[0]->poly==NULL)
    {
      printf("%s: %d: p3d_get_obj_pose(): the field pol[0]->poly of the input object is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
   #endif

  p3d_get_poly_pos(o->pol[0]->poly, pose);

  return 1;
}




// Pour afficher une variable de type p3d_polyhedre.
// A utiliser dans une fonction d'affichage OpenGL.
/*
void draw_p3d_polyhedre(p3d_polyhedre *polyhedron)
{
	double color_tab[15][3]= {  {1,0,0}, {0,1,0}, {0,0,1}, {1,1,0}, {1,0,1}, {0,1,1} , {1,0.5,0.5}, {0.5,1,0.5}, {0.5,0.5,1}, {1,0.25,0.5}, {1,0.5,0.25}, {0.25,1.0,0.5}, {0.5,1,0.25}, {0.25,0.5,1}, {0.5,0.25,1}  };


  int i, j;
  p3d_matrix4 pose;
  p3d_get_poly_pos( polyhedron, pose );
  p3d_vector3 axis;
  double t;
  p3d_mat4ExtractRot(pose, axis, &t);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glPointSize(6);
  glPushMatrix();
   glTranslatef(pose[0][3], pose[1][3], pose[2][3]);
   glRotatef((180/M_PI)*t,axis[0], axis[1], axis[2]);


   for(i= 0; i<polyhedron->nb_faces; i++)
   {
		 g3d_set_color_mat(Any, color_tab[i%15]);
     glBegin(GL_POLYGON);
       for(j=0; j<polyhedron->the_faces[i].nb_points; j++)
       {
         if( polyhedron->the_faces[i].plane->normale!=NULL )
          glNormal3dv(polyhedron->the_faces[i].plane->normale);
         glVertex3dv(polyhedron->the_points[polyhedron->the_faces[i].the_indexs_points[j]-1]);
       }
     glEnd();
   }

  glPopMatrix();

}*/


//! @ingroup graspPlanning 
//! Ecrit dans un fichier les indices des triangles voisins de chaque face du polyedre.
int p3d_print_face_neighbours(p3d_polyhedre *polyhedron, char *filename)
{
   #ifdef GP_DEBUG
    if(polyhedron==NULL)
    {  printf("%s: %d: p3d_print_face_neighbours(): entree= NULL.\n",__FILE__,__LINE__);
       return 0; }
   #endif

   int i;
   int nb_faces= polyhedron->nb_faces;
   p3d_face *faces= polyhedron->the_faces;

   FILE *file;
   file= fopen(filename,"w");

   if(file==NULL)
   {
     printf("%s: %d: p3d_print_face_neighbours(): erreur d'ouverture de fichier.\n", __FILE__, __LINE__);
     return 0;
   }

   for(i=0; i<nb_faces; i++)
   {
      fprintf(file, "triangle n°%d:\n\t %d %d %d \n", i, faces[i].neighbours[0], faces[i].neighbours[1], faces[i].neighbours[2]);
   }

   fclose(file);

   return 1;
}

//! @ingroup graspPlanning 
//! Enregistre au format .obj (format wavefront), une structure p3d_polyhedre.
int p3d_save_in_OBJ_format(p3d_polyhedre *polyhedron, char *name)
{
   #ifdef GP_DEBUG
    if(polyhedron==NULL)
    {  printf("%s: %d: save_p3d_polyhedre_in_OBJ_format(): entree= NULL.\n",__FILE__,__LINE__);
       return 0; }
   #endif
   unsigned int i, j;
   poly_index *ind;
   FILE *file= NULL;

   file= fopen(name, "w");
   if(file==NULL)
   { printf("%s: %d: save_p3d_polyhedre_in_OBJ_format(): erreur d'ouverture de fichier.\n",__FILE__,__LINE__);
     return 0;  }


   fprintf(file, "# nb_vertices %d nb_faces %d\n", polyhedron->nb_points, polyhedron->nb_faces);

   for(i=0; i< polyhedron->nb_points; i++)
   {  fprintf(file, "v %f %f %f \n", polyhedron->the_points[i][0], polyhedron->the_points[i][1], polyhedron->the_points[i][2]);   }

   for(i=0; i< polyhedron->nb_faces; i++)
   {
     ind= polyhedron->the_faces[i].the_indexs_points;
     fprintf(file, "f");
     for(j=0; j<polyhedron->the_faces[i].nb_points; j++)
     {
       fprintf(file, " %d", ind[j]);
     }
     fprintf(file, "\n");

  }

  fclose(file);

  return 1;
}


//! @ingroup graspPlanning 
//! Cree un p3d_polyhedre identique a celui donne en parametre.
p3d_polyhedre * p3d_copy_polyhedre(p3d_polyhedre *polyhedron)
{
  static int nbCalls= 0;

  unsigned int i, j;
  char str[200];
  if(strlen(polyhedron->name)>150)
  {
    printf("%s: %d: nom trop long\n", __FILE__, __LINE__);
    return NULL;
  }
  sprintf(str, "%s_copy(%d)", polyhedron->name, nbCalls++);

  p3d_polyhedre *result;
  result= p3d_create_poly(str);

  result->the_points= MY_ALLOC(p3d_vector3, polyhedron->nb_points);
  result->nb_points= polyhedron->nb_points;
  result->the_faces= MY_ALLOC(p3d_face, polyhedron->nb_faces);
  result->nb_faces= polyhedron->nb_faces;

  for(i=0; i<polyhedron->nb_points; i++)
  {
    p3d_vectCopy(polyhedron->the_points[i], result->the_points[i]);
  }

  for(i=0; i<polyhedron->nb_faces; i++)
  {
    result->the_faces[i].nb_points= polyhedron->the_faces[i].nb_points;
    result->the_faces[i].the_indexs_points= MY_ALLOC(poly_index, polyhedron->the_faces[i].nb_points);
    for(j=0; j<polyhedron->the_faces[i].nb_points; j++)
    {
       result->the_faces[i].the_indexs_points[j]= polyhedron->the_faces[i].the_indexs_points[j];
    }
  }

  return result;
}

//! @ingroup graspPlanning 
//! Affiche la face d'indice "index" du polyedre.
//! L'indice doit être compris entre 0 et nb_faces-1
//! A utiliser dans une fonction d'affichage OpenGL.
int p3d_display_face(p3d_polyhedre *polyhedron, unsigned int index)
{
  #ifdef GP_DEBUG
   if(polyhedron==NULL)
   {
     printf("%s: %d: p3d_polyhedre_display_face(): entree NULL.\n", __FILE__, __LINE__);
     return 0;
   }
  #endif

  if(index < 0)
  {
     printf("%s: %d: p3d_polyhedre_display_face(): indice de la face trop petit.\n", __FILE__, __LINE__);
     return 0;
  }
  if(index > polyhedron->nb_faces-1)
  {
     printf("%s: %d: p3d_polyhedre_display_face(): indice de la face trop grand.\n", __FILE__, __LINE__);
     return 0;
  }

  unsigned int i;
  p3d_vector3 *points= polyhedron->the_points;
  p3d_face *faces= polyhedron->the_faces;
  poly_index *ind= faces[index].the_indexs_points;

  p3d_matrix4 pose;
  p3d_get_poly_pos( polyhedron, pose );
  p3d_vector3 axis;
  double t;
  p3d_mat4ExtractRot(pose, axis, &t);

  glDisable(GL_LIGHTING);
  glColor3f(1.0, 0.0, 0.0);


  glPushMatrix();
   glTranslatef(pose[0][3], pose[1][3], pose[2][3]);
   glRotatef((180/M_PI)*t,axis[0], axis[1], axis[2]);
    glBegin(GL_POLYGON);
    for(i=0; i<faces[index].nb_points; i++)
    {
      glVertex3f(points[ind[i]-1][0],  points[ind[i]-1][1], points[ind[i]-1][2]);
    }
    glEnd();

    for(i=0; i<faces[index].nb_points; i++)
    {
      g3d_drawColorSphere(points[ind[i]-1][0],  points[ind[i]-1][1], points[ind[i]-1][2], 0.0005, Green, NULL);
    }

  glPopMatrix();

  glEnable(GL_LIGHTING);

  return 1;
}


//! \deprecated Do not use anymore but the code might be of some interest.
// Test de collision entre un robot et un objet.
// Les tests d'autocollision du robot sont aussi effectues.
// Tous les autres tests de collision (avec les autres objets et robots) ne sont
// pas pris en compte.
// Retourne 0 s'il n'y a pas de collision, 1 sinon.
int p3d_col_test_rob_obj(p3d_rob *robot, p3d_obj *object)
{
  #ifdef GP_DEBUG
   if(robot==NULL || object==NULL)
   {
     printf("%s: %d: p3d_col_test_rob_obj(): entree(s) NULL (%p %p).\n", __FILE__, __LINE__,robot,object);
     return 0;
   }
  #endif

   int i, result;
   kcd_col_handle *handlePt= NULL;

   switch( p3d_col_get_mode() )
   {
     case p3d_col_mode_pqp:
         pqp_robot_obj_collision_test(robot, object);
     break;
     case p3d_col_mode_kcd:
          handlePt= kcd_get_cur_col_handle();

          //On desactive les collisions entre tous les obstacles, sauf l'objet, et les robots:
          for(i=0; i<XYZ_ENV->no; i++)
          {
            if(XYZ_ENV->o[i]!=object)
              p3d_kcd_deactivate_obstacle(XYZ_ENV->o[i]);
          }

          //On desactive les collisions des robots entre eux.
          //Pour desactiver les collisions entre l'objet et les robots qui ne nous
          //interesse pas, on desactive leurs tests de collision avec l'environnement:
          for(i=0; i<XYZ_ENV->nr; i++)
          {
            if(XYZ_ENV->robot[i]==robot)
              continue;

            kcd_deact_collision_robot_pair(handlePt, robot, XYZ_ENV->robot[i]);

            kcd_deact_collision_robot_to_env(handlePt, XYZ_ENV->robot[i]);
          }

          //On effectue le test de collision:
          result= p3d_col_test();

          //On reactive ce qui avait ete desactive:
          for(i=0; i<XYZ_ENV->no; i++)
          {
            if(XYZ_ENV->o[i]!=object)
              p3d_kcd_activate_obstacle(XYZ_ENV->o[i]);
          }

          for(i=0; i<XYZ_ENV->nr; i++)
          {
            if(XYZ_ENV->robot[i]==robot)
              continue;
            kcd_add_collision_robot_pair(handlePt, robot, XYZ_ENV->robot[i]);
            kcd_add_collision_robot_to_env(handlePt, XYZ_ENV->robot[i]);
          }
     break;
     default:
        printf("%s: %d: p3d_col_test_rob_obj(): cette fonction ne marche qu'avec kcd et PQP.\n", __FILE__, __LINE__);
        return 0;
     break;
   }

   return result;
}


//! @ingroup graspPlanning 
//! Fonction pour passer d'une matrice 4x4 du format de la librairie "GB" a celui de move3D.
void Gb_th_matrix4(Gb_th *th, p3d_matrix4 mat)
{
  mat[0][0]= th->vx.x;   mat[0][1]= th->vy.x;   mat[0][2]= th->vz.x;   mat[0][3]= th->vp.x;
  mat[1][0]= th->vx.y;   mat[1][1]= th->vy.y;   mat[1][2]= th->vz.y;   mat[1][3]= th->vp.y;
  mat[2][0]= th->vx.z;   mat[2][1]= th->vy.z;   mat[2][2]= th->vz.z;   mat[2][3]= th->vp.z;
  mat[3][0]= 0;          mat[3][1]= 0;          mat[3][2]= 0;          mat[3][3]= 1;
}

//! @ingroup graspPlanning 
//! Fonction pour passer d'une matrice 4x4 du format de move3D a celui de la librairie "GB" .
void Gb_matrix4_th(p3d_matrix4 mat, Gb_th *th)
{
  th->vx.x = mat[0][0];
  th->vx.y = mat[1][0];
  th->vx.z = mat[2][0];

  th->vy.x = mat[0][1];
  th->vy.y = mat[1][1];
  th->vy.z = mat[2][1];

  th->vz.x = mat[0][2];
  th->vz.y = mat[1][2];
  th->vz.z = mat[2][2];

  th->vp.x = mat[0][3];
  th->vp.y = mat[1][3];
  th->vp.z = mat[2][3];
}


//! @ingroup graspPlanning 
//! Solves the trigonometric equation a*cos(x) + b*sin(x)= c where a,b and c are given and x is the unknown.
//! \param a first parameter of the equation to solve
//! \param b second parameter of the equation to solve
//! \param c third parameter of the equation to solve
//! \param x1 pointer to where the first solution (if it exists) will be copied
//! \param x2 pointer to where the second solution (if it exists) will be copied
//! \return the number of solutions of the equation (0, 1 or 2)
int solve_trigonometric_equation(double a, double b, double c, double *x1, double *x2)
{
   double delta, sqrtDelta, alpha1, alpha2;
   double epsilon= 10e-9;

   // particular case when every x is solution.
   // Return 0 anyway.
   if( fabs(a)<epsilon && fabs(b)<epsilon )
     return 0;

   if( (c*c) > (a*a + b*b) )
   {  return 0; }

   // solve b*sin(x)= c
   if( fabs(a)<epsilon )
   {
     if( fabs(c) > fabs(b) )
     { return 0; }

     if( fabs(c-b) < epsilon )
     {
        (*x1)= M_PI_2;
        return 1;
     }
     if( fabs(c+b) < epsilon )
     {
        (*x1)= -M_PI_2;
        return 1;
     }
     (*x1)= asin(c/b);
     (*x2)= M_PI - (*x1);
     return 2;
   }

   // solve a*cos(x)= c
   if( fabs(b)<epsilon )
   {
     if( fabs(c) > fabs(a) )
     { return 0; }

     if( fabs(c-a) < epsilon )
     {
        (*x1)= 0;
        return 1;
     }
     if( fabs(c+a) < epsilon )
     {
        (*x1)= M_PI;
        return 1;
     }
     (*x1)= acos(c/a);
     (*x2)= 2*M_PI - (*x1);
     return 2;
   }

   // general case:

   delta= 4*b*b*( a*a + b*b - c*c);

   if(delta < 0)
   {  return 0; }

   if( fabs(delta) < epsilon )
   {
     alpha1= a*c / (a*a+b*b);
     (*x1)= atan2( (c-alpha1*a)/b, alpha1 );
     return 1;
   }

   sqrtDelta= sqrt(delta);
   alpha1= ( 2*a*c + sqrtDelta ) / (2*(a*a+b*b));
   alpha2= ( 2*a*c - sqrtDelta ) / (2*(a*a+b*b));



   (*x1)= atan2( (c - a*alpha1)/b, alpha1 );
   (*x2)= atan2( (c - a*alpha2)/b, alpha2 );

// printf("q_2= [ %f  ou %f  ]\n", (*x1)*RADTODEG, (*x2)*RADTODEG);

// printf("3\t a*cos(x1) + b*sin(x1)= c:   %f  =  %f  \n", a*cos((*x1)) + b*sin((*x1)), c);
// printf("3\t a*cos(x2) + b*sin(x2)= c:   %f  =  %f  \n", a*cos((*x2)) + b*sin((*x2)), c);
   return 2;
}



//! @ingroup graspPlanning 
//! Fonction d'affichage d'un repere (matrice 4x4).
//! Les axes sont dessines sur une longueur "length".
//! A utiliser dans une fonction d'affichage OpenGL.
void draw_frame0(p3d_matrix4 frame, double length)
{
   p3d_vector3 origin, xAxis, yAxis, zAxis;

   origin[0]= frame[0][3];
   origin[1]= frame[1][3];
   origin[2]= frame[2][3];

   xAxis[0]=  frame[0][0];
   xAxis[1]=  frame[1][0];
   xAxis[2]=  frame[2][0];

   yAxis[0]=  frame[0][1];
   yAxis[1]=  frame[1][1];
   yAxis[2]=  frame[2][1];

   zAxis[0]=  frame[0][2];
   zAxis[1]=  frame[1][2];
   zAxis[2]=  frame[2][2];

   glLineWidth(4);
   glDisable(GL_LIGHTING);
   glColor3f(1,0,0);
   glBegin(GL_LINES);
     glVertex3d( origin[0], origin[1], origin[2] );
     glVertex3d( origin[0]+ length*xAxis[0], origin[1]+ length*xAxis[1], origin[2]+ length*xAxis[2] );
   glEnd();


   glColor3f(0,1,0);
   glBegin(GL_LINES);
     glVertex3d( origin[0], origin[1], origin[2] );
     glVertex3d( origin[0]+ length*yAxis[0], origin[1]+ length*yAxis[1], origin[2]+ length*yAxis[2] );
   glEnd();

   glColor3f(0,0,1);
   glBegin(GL_LINES);
     glVertex3d( origin[0], origin[1], origin[2] );
     glVertex3d( origin[0]+ length*zAxis[0], origin[1]+ length*zAxis[1], origin[2]+ length*zAxis[2] );
   glEnd();

   glEnable(GL_LIGHTING);
   glLineWidth(2);

}

//! @ingroup graspPlanning 
//! Exports the current scene to a .pov file (for POVRAY ray tracer).
//! Still experimental.
//! \param foldername name of the folder where all the created include files (.inc) will be written
//! \param filename name of the created .pov file
int export_scene_to_POVRAY(char *foldername, char *filename)
{
  if(foldername==NULL || filename==NULL)
  {
    printf("%s: %d: export_scene_to_POVRay(): an input is NULL (%p %p).\n",__FILE__,__LINE__,foldername, filename);
    return 0;
  }

  int i, j, k;
  int mesh_reported= FALSE;
  FILE *file= NULL;
  char name[100], pol_name[100];
  char inc_name[100], inc_name2[100];
  p3d_obj *object;
  p3d_rob *robot;
  G3D_Window *win = g3d_get_cur_win();
//  p3d_vector4 Xc, Xw;
  //p3d_vector4 up;

  //sprintf(name, "%s/%s", foldername, filename);
  sprintf(name, "%s", filename);
  file= fopen(name, "w");
  if(file==NULL)
  {
    printf("%s: %d: export_scene_to_POVRAY(): can't open file %s.\n",__FILE__,__LINE__, name);
    return 0;
  }

  for(i=0; i<XYZ_ENV->no; i++)
  {
    object= XYZ_ENV->o[i];
    printf("object %d: %s\n", i, object->name);
    strcpy(name, object->name);
    for(j=0; name[j]!='\0'; j++)
    {
       if(name[j]=='.')
         name[j]= '_';
    }
    sprintf(inc_name, "%s.inc", name);
    fprintf(file, "#include \"%s\"\n", inc_name);
    sprintf(inc_name2, "./povray/%s", inc_name);
    export_p3d_obj_to_POVRAY(object, inc_name2);
    printf("inc_name: %s\n", name);
  }

  for(i=0; i<XYZ_ENV->nr; i++)
  {
    robot= XYZ_ENV->robot[i];
    for(k=0; k<robot->no; k++)
    {
      object= robot->o[k];
      strcpy(name, object->name);
      for(j=0; name[j]!='\0'; j++)
      {
        if(name[j]=='.')
          name[j]= '_';
      }
      sprintf(inc_name, "%s.inc", name);
      fprintf(file, "#include \"%s\"\n", inc_name);
      sprintf(inc_name2, "./povray/%s", inc_name);
      export_p3d_obj_to_POVRAY(object, inc_name2);
      printf("inc_name: %s\n", name);
    }
  }


  /////////////////////global settings////////////////
  fprintf(file, "global_settings {\n");
  fprintf(file, "\t assumed_gamma 1\n");
  fprintf(file, "\t radiosity {\n");
  fprintf(file, "\t\t pretrace_start 0.08\n");
  fprintf(file, "\t\t pretrace_end  0.04\n");
  fprintf(file, "\t\t count 100//35\n");
  fprintf(file, "\t\t\n");
  fprintf(file, "\t\t nearest_count 10//5\n");
  fprintf(file, "\t\t error_bound 0.2\n");
  fprintf(file, "\t\t recursion_limit 1\n");
  fprintf(file, "\t\t\n");
  fprintf(file, "\t\t low_error_factor .5\n");
  fprintf(file, "\t\t gray_threshold 0.0\n");
  fprintf(file, "\t\t minimum_reuse 0.015\n");
  fprintf(file, "\t\t brightness 1\n");
  fprintf(file, "\t\t normal on\n");
  fprintf(file, "\t\t media on\n");
  fprintf(file, "\t\t adc_bailout 0.01/2\n");
  fprintf(file, "\t }\n");
  fprintf(file, "}\n\n");
 ////////////////////////////////////////////////

  //calc_cam_param(win, Xc, Xw);
 // p3d_matvec4Mult( *(*win->cam_frame), win->vs.up, up);
/*
  Transf[0][3] = win->vs.zo * (cos(win->vs.az)*cos(win->vs.el));
  Transf[1][3] = win->vs.zo * (sin(win->vs.az)*cos(win->vs.el));
  Transf[2][3] = win->vs.zo * sin(win->vs.el);
*/
  p3d_vector3 cam_pos, right;
  cam_pos[0] = win->vs.zo * (cos(win->vs.az)*cos(win->vs.el));
  cam_pos[1] = win->vs.zo * (sin(win->vs.az)*cos(win->vs.el));
  cam_pos[2] = win->vs.zo * sin(win->vs.el);

  right[0] = (*win->cam_frame)[0][0];
  right[1] = (*win->cam_frame)[1][0];
  right[2] = (*win->cam_frame)[2][0];
  p3d_vectNormalize(right, right);

  fprintf(file, "camera {\n");
  fprintf(file, "\t /*location  <1.0, -4, 3.5>*/\n");
  fprintf(file, "\t location  <%f, %f, %f>\n", cam_pos[0], cam_pos[1], cam_pos[2]);
  fprintf(file, "\t /*direction 1.5*y*/\n");
 // fprintf(file, "\t direction  <%f,%f,%f>\n",win->x-cam_pos[0],win->vs.y-cam_pos[1],win->vs.z-cam_pos[2]);
  fprintf(file, "\t /*right  -x*image_width/image_height*/\n");
  fprintf(file, "\t right  <%f, %f, %f>\n", -right[0], -right[1], -right[2] );
  fprintf(file, "\t sky  <%f, %f, %f>\n", win->vs.up[0], win->vs.up[1], win->vs.up[2] );
  fprintf(file, "\t angle  45\n");
  fprintf(file, "\t look_at  <%f %f %f>\n", win->vs.x, win->vs.y, win->vs.z);
  fprintf(file, "}\n\n");

  fprintf(file, "sky_sphere {\n");
  fprintf(file, "\t pigment {\n");
  fprintf(file, "\t\t gradient z\n");
  fprintf(file, "\t\t color_map {\n");
  fprintf(file, "\t\t\t [0.0 rgb <0.6,0.7,1.0>]\n");
  fprintf(file, "\t\t\t [0.7 rgb <0.0,0.1,0.8>]\n");
  fprintf(file, "\t\t }\n");
  fprintf(file, "\t }\n");
  fprintf(file, "}\n\n");

  fprintf(file, "light_source {\n");
  fprintf(file, "\t <0, 0, 0>\n");
  fprintf(file, "\t color rgb <1, 0.7, 1>\n");
  fprintf(file, "\t translate <30, 10, 50>\n");
  fprintf(file, "}\n\n");

  fprintf(file, "plane {\n");
  fprintf(file, "\t z, -0.0\n");
  fprintf(file, "\t texture {\n");
  fprintf(file, "\t\t pigment {\n");
  fprintf(file, "\t\t\t checker\n");
  fprintf(file, "\t\t\t color rgb 1\n");
  fprintf(file, "\t\t\t color blue 1\n");
  fprintf(file, "\t\t\t scale 1\n");
  fprintf(file, "\t\t }\n");
  fprintf(file, "\t }\n");
  fprintf(file, "}\n\n");



  for(i=0; i<XYZ_ENV->no; i++)
  {
    object= XYZ_ENV->o[i];
    mesh_reported= FALSE;
    printf("object %d: %s\n", i, object->name);
    strcpy(name, object->name);
    for(j=0; name[j]!='\0'; j++)
    {
       if(name[j]=='.')
         name[j]= '_';
    }
    for(j=0; j<object->np; j++)
    {
      if(object->pol[j]->entity_type==POLYHEDRON_ENTITY || object->pol[j]->entity_type==CONVEX_POLYHEDRON || object->pol[j]->entity_type==CONCAVE_POLYHEDRON)
      {
        if(mesh_reported==TRUE)
          continue;
        else
          mesh_reported= TRUE;
      }

      switch(object->pol[j]->entity_type)
      {
        case POLYHEDRON_ENTITY: case CONVEX_POLYHEDRON: case CONCAVE_POLYHEDRON:
          sprintf(pol_name, "%s_mesh", name);
        break;
        case SPHERE_ENTITY:
          sprintf(pol_name, "%s_sphere_%d", name, j);
        break;
        case BOX_ENTITY:
          sprintf(pol_name, "%s_box_%d", name, j);
        break;
      }
      fprintf(file, "object { %s\n", pol_name);/*
       fprintf(file, "\t \matrix < %f, %f, %f,\n", object->opos[0][0], object->opos[0][1], object->opos[0][2]);
       fprintf(file, "\t           %f, %f, %f,\n", object->opos[1][0], object->opos[1][1], object->opos[1][2]);
       fprintf(file, "\t           %f, %f, %f,\n", object->opos[2][0], object->opos[2][1], object->opos[2][2]);
       fprintf(file, "\t           %f, %f, %f >\n", object->opos[0][3], object->opos[1][3], object->opos[2][3]);*/
      fprintf(file, "\t pigment {\n");
      switch(object->pol[0]->color)
      {
        case Red:
          fprintf(file, "\t\t color rgb <1,0,0>\n");
        break;
        case Green:
          fprintf(file, "\t\t color rgb <0,1,0>\n");
        break;
        case Blue:
          fprintf(file, "\t\t color rgb <0,0,1>\n");
        break;
        case Any:
          fprintf(file, "\t\t color rgb <%f,%f,%f>\n", object->pol[0]->color_vect[0], object->pol[0]->color_vect[1], object->pol[0]->color_vect[2]);
        break;
        default:
          fprintf(file, "\t\t color rgb <0.5,0.5,0.5>\n");
        break;
      }
      fprintf(file, "\t }\n");
      fprintf(file, "}\n\n");
    }



  }


  for(i=0; i<XYZ_ENV->nr; i++)
  {
    robot= XYZ_ENV->robot[i];
    for(k=0; k<robot->no; k++)
    {
      object= robot->o[k];
      mesh_reported= FALSE;
      strcpy(name, object->name);
      for(j=0; name[j]!='\0'; j++)
      {
        if(name[j]=='.')
          name[j]= '_';
      }
      for(j=0; j<object->np; j++)
      {
        if(object->pol[j]->entity_type==POLYHEDRON_ENTITY || object->pol[j]->entity_type==CONVEX_POLYHEDRON || object->pol[j]->entity_type==CONCAVE_POLYHEDRON)
        {
          if(mesh_reported==TRUE)
            continue;
          else
            mesh_reported= TRUE;
        }

        switch(object->pol[j]->entity_type)
        {
          case POLYHEDRON_ENTITY: case CONVEX_POLYHEDRON: case CONCAVE_POLYHEDRON:
            sprintf(pol_name, "%s_mesh", name);
          break;
          case SPHERE_ENTITY:
            sprintf(pol_name, "%s_sphere_%d", name, j);
          break;
          case BOX_ENTITY:
            sprintf(pol_name, "%s_box_%d", name, j);
          break;
        }
        fprintf(file, "object { %s\n", pol_name);
//         fprintf(file, "\t \matrix < %f, %f, %f,\n", object->opos[0][0], object->opos[0][1], object->opos[0][2]);
//         fprintf(file, "\t           %f, %f, %f,\n", object->opos[1][0], object->opos[1][1], object->opos[1][2]);
//         fprintf(file, "\t           %f, %f, %f,\n", object->opos[2][0], object->opos[2][1], object->opos[2][2]);
//         fprintf(file, "\t           %f, %f, %f >\n", object->opos[0][3], object->opos[1][3], object->opos[2][3]);
/*
        fprintf(file, "\t \matrix < %f, %f, %f,\n", object->jnt->abs_pos[0][0], object->jnt->abs_pos[1][0], object->jnt->abs_pos[2][0]);
        fprintf(file, "\t           %f, %f, %f,\n", object->jnt->abs_pos[0][1], object->jnt->abs_pos[1][1], object->jnt->abs_pos[2][1]);
        fprintf(file, "\t           %f, %f, %f,\n", object->jnt->abs_pos[0][2], object->jnt->abs_pos[1][2], object->jnt->abs_pos[2][2]);
        fprintf(file, "\t           %f, %f, %f >\n", object->jnt->abs_pos[0][3], object->jnt->abs_pos[1][3], object->jnt->abs_pos[2][3]);*/
        fprintf(file, "\t pigment {\n");
        switch(object->pol[0]->color)
        {
          case Red:
            fprintf(file, "\t\t color rgb <1,0,0>\n");
          break;
          case Green:
            fprintf(file, "\t\t color rgb <0,1,0>\n");
          break;
          case Blue:
            fprintf(file, "\t\t color rgb <0,0,1>\n");
          break;
          case Any:
            fprintf(file, "\t\t color rgb <%f,%f,%f>\n", object->pol[0]->color_vect[0], object->pol[0]->color_vect[1], object->pol[0]->color_vect[2]);
          break;
          default:
            fprintf(file, "\t\t color rgb <0.5,0.5,0.5>\n");
          break;
        }
        fprintf(file, "\t }\n");
        fprintf(file, "}\n\n");
      }
/*

    fprintf(file, "object { %s\n", name);
    fprintf(file, "\t \matrix < %f, %f, %f,\n", object->opos[0][0], object->opos[0][1], object->opos[0][2]);
    fprintf(file, "\t           %f, %f, %f,\n", object->opos[1][0], object->opos[1][1], object->opos[1][2]);
    fprintf(file, "\t           %f, %f, %f,\n", object->opos[2][0], object->opos[2][1], object->opos[2][2]);
    fprintf(file, "\t           %f, %f, %f >\n", object->opos[0][3], object->opos[1][3], object->opos[2][3]);

    fprintf(file, "\t pigment {\n");
    fprintf(file, "\t\t color rgb <0.5,0.5,0.5>\n");
    fprintf(file, "\t }\n");
    fprintf(file, "}\n\n");      */
    }
  }



  fclose(file);


  return 1;
}

//! Exports a p3d_obj to a .pov file (for POVRAY ray tracer).
//! Still experimental.
int export_p3d_obj_to_POVRAY(p3d_obj *object, char *filename)
{
  if(object==NULL || filename==NULL)
  {
    printf("%s: %d: export_p3d_obj_to_POVRAY(): an input is NULL (%p %p).\n",__FILE__,__LINE__,object,filename);
    return 0;
  }

  unsigned int i, j;
  int nb_polyhedra, nb_points, nb_faces, previous_nb_points;
  char name[100];
  FILE *file= NULL;
  p3d_vector3  *points= NULL;
  p3d_matrix4 T, Tpose;
  p3d_face  *faces= NULL;
  p3d_polyhedre *polyh= NULL;
  p3d_primitive *prim= NULL;
  p3d_matrix4 Tpolyh;

  file= fopen(filename, "w");
  if(file==NULL)
  {
    printf("%s: %d: export_p3d_obj_to_POVRAY(): can't open file %s.\n",__FILE__,__LINE__, filename);
    return 0;
  }

  if(object->is_used_in_device_flag && object->jnt!=NULL)
  {    p3d_matMultXform(object->jnt->abs_pos, object->BodyWrtPilotingJoint, T);
    ///p3d_mat4Copy(object->jnt->abs_pos, T);
    printf("object joint %s (%s)\n", object->name, object->jnt->name);
  }
  else
  {
    p3d_mat4Copy(p3d_mat4IDENTITY, T);
  }


  fprintf(file, "#declare default_material = texture {\n");
  fprintf(file, "\t pigment {\n");
  switch(object->pol[0]->color)
        {
          case Red:
            fprintf(file, "\t\t color rgb <1,0,0>\n");
          break;
          case Green:
            fprintf(file, "\t\t color rgb <0,1,0>\n");
          break;
          case Blue:
            fprintf(file, "\t\t color rgb <0,0,1>\n");
          break;
          case Any:
            fprintf(file, "\t\t color rgb <%f,%f,%f>\n", object->pol[0]->color_vect[0], object->pol[0]->color_vect[1], object->pol[0]->color_vect[2]);
          break;
          default:
            fprintf(file, "\t\t color rgb <0.5,0.5,0.5>\n");
          break;
        }
//  fprintf(file, "\t\t color rgbf <1.0, 1.0, 1.0, 0.0>\n");
  fprintf(file, "\t }\n");
  fprintf(file, "\t finish {\n");
  fprintf(file, "\t\t ambient rgb <0.0, 0.0, 0.0>\n");
  fprintf(file, "\t\t diffuse 0.700000\n");
  fprintf(file, "\t\t brilliance 1.000000\n");
  fprintf(file, "\t\t metallic 0.000000\n");
  fprintf(file, "\t\t specular 1.000000\n");
  fprintf(file, "\t\t roughness 0.010000\n");
  fprintf(file, "\t }\n");
  fprintf(file, "}\n\n");



  ////////On traite d'abord tous les polyedres en les assemblant en un seul mesh2////////
  nb_points= 0;
  nb_faces= 0;
  nb_polyhedra= 0;
  for(i=0; i<(unsigned int) object->np; i++)
  {
    if(object->pol[i]->entity_type==POLYHEDRON_ENTITY || object->pol[i]->entity_type==CONVEX_POLYHEDRON || object->pol[i]->entity_type==CONCAVE_POLYHEDRON)
    {
      if(nb_polyhedra==0)
         p3d_mat4Copy(object->pol[i]->poly->pos, Tpolyh);
      nb_polyhedra++;
      nb_points+= object->pol[i]->poly->nb_points;
      nb_faces+= object->pol[i]->poly->nb_faces;
    }
  }

  printf("%s nb_points= %d nb_faces= %d nb_polyhedra= %d\n", object->name,nb_points,nb_faces,nb_polyhedra);


  strcpy(name, object->name);
  for(i=0; name[i]!='\0'; i++)
  {
    if(name[i]=='.')
       name[i]= '_';
  }

  if(nb_polyhedra!=0)
  {
      fprintf(file, "#declare %s_mesh = mesh2 {\n", name);

      fprintf(file, "\t vertex_vectors {\n");
      fprintf(file, "\t\t %d,\n", nb_points);

      for(i=0; i<(unsigned int) object->np; i++)
      {
        if(object->pol[i]->entity_type==POLYHEDRON_ENTITY || object->pol[i]->entity_type==CONVEX_POLYHEDRON || object->pol[i]->entity_type==CONCAVE_POLYHEDRON)
        {
          polyh= object->pol[i]->poly;
          points= polyh->the_points;
          for(j=0; j<polyh->nb_points; j++)
          {
            fprintf(file, "\t\t <%f, %f, %f>,\n", points[j][0], points[j][1], points[j][2]);
          }
        }
      }
      fprintf(file, "\t}\n\n");

      fprintf(file, "\t texture_list { 1, texture{ default_material } }\n");

      fprintf(file, "\t face_indices {\n");
      fprintf(file, "\t\t %d,\n", nb_faces);

      previous_nb_points= 0;
      for(i=0; i<(unsigned int) object->np; i++)
      {
        if(object->pol[i]->entity_type==POLYHEDRON_ENTITY || object->pol[i]->entity_type==CONVEX_POLYHEDRON || object->pol[i]->entity_type==CONCAVE_POLYHEDRON)
        {
          polyh= object->pol[i]->poly;
          faces= polyh->the_faces;

          for(j=0; j<polyh->nb_faces; j++)
          {
            if( faces[j].nb_points!=3 )
            {
              printf("%s: %d: export_p3d_obj_to_POVray(): a face is not triangular.\n",__FILE__,__LINE__);
              continue;
            }
            fprintf(file, "\t\t <%d, %d, %d>, 0,\n",faces[j].the_indexs_points[0]-1+previous_nb_points,faces[j].the_indexs_points[1]-1+previous_nb_points, faces[j].the_indexs_points[2]-1+previous_nb_points);
          }
          previous_nb_points+= polyh->nb_points;
        }
      }
      fprintf(file, "\t}\n\n");

      fprintf(file, "\t normal_indices {\n");
      fprintf(file, "\t\t 0\n");
      fprintf(file, "\t}\n\n");
    /*
      fprintf(file, "\t \matrix < %f, %f, %f,\n", Tpose[0][0], Tpose[0][1], Tpose[0][2]);
      fprintf(file, "\t           %f, %f, %f,\n", Tpose[1][0], Tpose[1][1], Tpose[1][2]);
      fprintf(file, "\t           %f, %f, %f,\n", Tpose[2][0], Tpose[2][1], Tpose[2][2]);
      fprintf(file, "\t           %f, %f, %f >\n", Tpose[0][3], Tpose[1][3], Tpose[2][3]);*/
      p3d_matMultXform(object->pol[0]->pos0, T, Tpose);
     // p3d_mat4Mult(T, p3d_mat4IDENTITY, Tpose);
      if(object->jnt!=NULL)
         p3d_mat4Copy(object->jnt->abs_pos, Tpose);
      else
         p3d_mat4Copy(object->pol[0]->pos0, Tpose);
      fprintf(file, "\t matrix < %f, %f, %f,\n", Tpose[0][0], Tpose[1][0], Tpose[2][0]);
      fprintf(file, "\t          %f, %f, %f,\n", Tpose[0][1], Tpose[1][1], Tpose[2][1]);
      fprintf(file, "\t          %f, %f, %f,\n", Tpose[0][2], Tpose[1][2], Tpose[2][2]);
      fprintf(file, "\t          %f, %f, %f >\n", Tpose[0][3], Tpose[1][3], Tpose[2][3]);
      fprintf(file, "}\n\n");
  }

  //////On traite maintenant les primitives///////////:
  for(i=0; i<(unsigned int) object->np; i++)
  {
    switch(object->pol[i]->entity_type)
    {
       case SPHERE_ENTITY:
            prim= object->pol[i]->primitive_data;
            fprintf(file, "#declare %s_sphere_%d  = sphere {\n", name, i);
            fprintf(file, "\t <%f, %f, %f>, // center of sphere \n", 0.0, 0.0, 0.0);
            fprintf(file, "\t %f  // radius\n", prim->radius);
            fprintf(file, "\t texture {\n");
            fprintf(file, "\t\t default_material\n");
            fprintf(file, "\t }\n");
/*
            fprintf(file, "\t \matrix < %f, %f, %f,\n", object->pol[i]->pos0[0][0], object->pol[i]->pos0[0][1], object->pol[i]->pos0[0][2]);
            fprintf(file, "\t           %f, %f, %f,\n", object->pol[i]->pos0[1][0], object->pol[i]->pos0[1][1], object->pol[i]->pos0[1][2]);
            fprintf(file, "\t           %f, %f, %f,\n", object->pol[i]->pos0[2][0], object->pol[i]->pos0[2][1], object->pol[i]->pos0[2][2]);
            fprintf(file, "\t           %f, %f, %f >\n", object->pol[i]->pos0[0][3], object->pol[i]->pos0[1][3], object->pol[i]->pos0[2][3]);*/

            p3d_matMultXform(T, object->pol[i]->pos0, Tpose);
            fprintf(file, "\t matrix < %f, %f, %f,\n", Tpose[0][0], Tpose[1][0], Tpose[2][0]);
            fprintf(file, "\t          %f, %f, %f,\n", Tpose[0][1], Tpose[1][1], Tpose[2][1]);
            fprintf(file, "\t          %f, %f, %f,\n", Tpose[0][2], Tpose[1][2], Tpose[2][2]);
            fprintf(file, "\t          %f, %f, %f >\n", Tpose[0][3], Tpose[1][3], Tpose[2][3]);
            fprintf(file, "}\n");
/*
sphere {
  <0, 0, 0> // center of sphere <X Y Z>
  0.05       // radius of sphere
pigment { color rgb <1.0,1.0,1.0> }
}
*/
       break;
       case BOX_ENTITY:
            prim= object->pol[i]->primitive_data;
            fprintf(file, "#declare %s_%s_%d  = box {\n", name, "box", i);
            fprintf(file, "\t <%f, %f, %f>,  // Near lower left corner\n", -0.5*prim->x_length, -0.5*prim->y_length, -0.5*prim->z_length);
            fprintf(file, "\t <%f, %f, %f>  // Far upper right corner\n", 0.5*prim->x_length, 0.5*prim->y_length, 0.5*prim->z_length);
            fprintf(file, "\t texture {\n");
            fprintf(file, "\t\t default_material\n");
            fprintf(file, "\t }\n");
            p3d_matMultXform(T, object->pol[i]->pos0, Tpose);
            fprintf(file, "\t matrix < %f, %f, %f,\n", Tpose[0][0], Tpose[1][0], Tpose[2][0]);
            fprintf(file, "\t           %f, %f, %f,\n", Tpose[0][1], Tpose[1][1], Tpose[2][1]);
            fprintf(file, "\t           %f, %f, %f,\n", Tpose[0][2], Tpose[1][2], Tpose[2][2]);
            fprintf(file, "\t           %f, %f, %f >\n", Tpose[0][3], Tpose[1][3], Tpose[2][3]);/*
            fprintf(file, "\t  matrix < %f, %f, %f,\n", Tpose[0][0], Tpose[1][0], Tpose[2][0]);
            fprintf(file, "\t           %f, %f, %f,\n", Tpose[0][1], Tpose[1][1], Tpose[2][1]);
            fprintf(file, "\t           %f, %f, %f,\n", Tpose[0][2], Tpose[1][2], Tpose[2][2]);
            fprintf(file, "\t           %f, %f, %f >\n", Tpose[0][3], Tpose[1][3], Tpose[2][3]);*/
            fprintf(file, "}\n");

       break;
       case CYLINDER_ENTITY:
/*
  cylinder {
    <0, 1, 0>,     // Center of one end
    <1, 2, 3>,     // Center of other end
    0.5            // Radius
    open           // Remove end caps
    texture { T_Stone25 scale 4 }
  }
*/
       break;
/*
       default:
          printf("%s: %d: export_p3d_obj_to_POVRAY(): this type of entity (%d) is not supported.\n",__FILE__,__LINE__, entity_type);
       break;*/
    }

  }

  fclose(file);

  return 1;
}


int export_p3d_polyhedre_to_POVRAY(p3d_polyhedre *polyhedron, char *filename)
{
  if(polyhedron==NULL || filename==NULL)
  {
    printf("%s: %d: export_p3d_polyhedre_to_POVRAY(): an input is NULL (%p %p).\n",__FILE__,__LINE__,polyhedron,filename);
    return 0;
  }

  unsigned int i;
  char name[100];
  FILE *file= NULL;
  p3d_vector3  *points= polyhedron->the_points;
  p3d_face  *faces= polyhedron->the_faces;

  file= fopen(filename, "w");
  if(file==NULL)
  {
    printf("%s: %d: export_p3d_polyhedre_to_POVRAY(): can't open file %s.\n",__FILE__,__LINE__, filename);
    return 0;
  }

  fprintf(file, "#declare default_material = texture {\n");
  fprintf(file, "\t pigment {\n");
  fprintf(file, "\t\t color rgbf <1.0, 1.0, 1.0, 0.0>\n");
  fprintf(file, "\t }\n");
  fprintf(file, "\t finish {\n");
  fprintf(file, "\t\t ambient rgb <0.0, 0.0, 0.0>\n");
  fprintf(file, "\t\t diffuse 0.700000\n");
  fprintf(file, "\t\t brilliance 1.000000\n");
  fprintf(file, "\t\t metallic 0.000000\n");
  fprintf(file, "\t\t specular 1.000000\n");
  fprintf(file, "\t\t roughness 0.010000\n");
  fprintf(file, "\t }\n");
  fprintf(file, "}\n\n");

  strcpy(name, polyhedron->name);
  for(i=0; name[i]!='\0'; i++)
  {
    if(name[i]=='.')
       name[i]= '_';
  }

  fprintf(file, "#declare %s  = mesh2 {\n", name);

  fprintf(file, "\t vertex_vectors {\n");
  fprintf(file, "\t\t %d,\n", polyhedron->nb_points);
  for(i=0; i<polyhedron->nb_points; i++)
  {
    fprintf(file, "\t\t <%f, %f, %f>,\n", points[i][0], points[i][1], points[i][2]);
  }
  fprintf(file, "\t}\n\n");

  fprintf(file, "\t texture_list { 1, texture{ default_material } }\n");

  fprintf(file, "\t face_indices {\n");
  fprintf(file, "\t\t %d,\n", polyhedron->nb_faces);
  for(i=0; i<polyhedron->nb_faces; i++)
  {
    if( faces[i].nb_points!=3 )
    {
      printf("%s: %d: export_p3d_polyhedre_to_POVray(): a face is not triangular.\n",__FILE__,__LINE__);
    }
    fprintf(file, "\t\t <%d, %d, %d>, 0,\n",faces[i].the_indexs_points[0]-1,faces[i].the_indexs_points[1]-1, faces[i].the_indexs_points[2]-1);
  }
  fprintf(file, "\t}\n\n");


  fprintf(file, "\t normal_indices {\n");
  fprintf(file, "\t\t 0\n");
  fprintf(file, "\t}\n\n");


  fprintf(file, "}\n\n");


  fclose(file);

  return 1;
}


// int export_primitive_to_POVRAY(char *object_name, int entity_type, p3d_primitive *prim, char *filename)
// {
//   if(object_name==NULL || prim==NULL || filename==NULL)
//   {
//     printf("%s: %d: export_primitive_to_POVRAY(): an input is NULL (%p %p %p).\n",__FILE__,__LINE__,object_name,prim,filename);
//     return 0;
//   }
//
//   int i, j;
//   FILE *file= NULL;
//
//   file= fopen(filename, "w");
//   if(file==NULL)
//   {
//     printf("%s: %d: export_primitive_to_POVRAY(): can't open file %s.\n",__FILE__,__LINE__, filename);
//     return 0;
//   }
//
//   fprintf(file, "#declare default_material = texture {\n");
//   fprintf(file, "\t pigment {\n");
//   fprintf(file, "\t\t color rgbf <1.0, 1.0, 1.0, 0.0>\n");
//   fprintf(file, "\t }\n");
//   fprintf(file, "\t finish {\n");
//   fprintf(file, "\t\t ambient rgb <0.0, 0.0, 0.0>\n");
//   fprintf(file, "\t\t diffuse 0.700000\n");
//   fprintf(file, "\t\t brilliance 1.000000\n");
//   fprintf(file, "\t\t metallic 0.000000\n");
//   fprintf(file, "\t\t specular 1.000000\n");
//   fprintf(file, "\t\t roughness 0.010000\n");
//   fprintf(file, "\t }\n");
//   fprintf(file, "}\n\n");
//
//   switch(entity_type)
//   {
//      case SPHERE_ENTITY:
//
// // sphere {
// //   <0, 0, 0> // center of sphere <X Y Z>
// //   0.05       // radius of sphere
// // pigment { color rgb <1.0,1.0,1.0> }
// // }
//
//      break;
//      case BOX_ENTITY:
//             fprintf(file, "#declare %s  = box {\n", object_name);
//             fprintf(file, "\t <%f, %f, %f>,  // Near lower left corner\n", -0.5*prim->x_length, -0.5*prim->y_length, -0.5*prim->z_length);
//             fprintf(file, "\t <%f, %f, %f>  // Far upper right corner\n", 0.5*prim->x_length, 0.5*prim->y_length, 0.5*prim->z_length);
//             fprintf(file, "\t texture {\n");
//             fprintf(file, "\t\t default_material\n");
//             fprintf(file, "\t }\n");
//             fprintf(file, "}\n");
//
// //   box {
// //     <-1, 0,   -1>,  // Near lower left corner
// //     < 1, 0.5,  3>   // Far upper right corner
// //     texture {
// //       T_Stone25     // Pre-defined from stones.inc
// //       scale 4       // Scale by the same amount in all
// //                     // directions
// //     }
// //     rotate y*20     // Equivalent to "rotate <0,20,0>"
// //   }
//
//      break;
//      case CYLINDER_ENTITY:
//
// //   cylinder {
// //     <0, 1, 0>,     // Center of one end
// //     <1, 2, 3>,     // Center of other end
// //     0.5            // Radius
// //     open           // Remove end caps
// //     texture { T_Stone25 scale 4 }
// //   }
//
//      break;
//      default:
//         printf("%s: %d: export_primitive_to_POVRAY(): this type of entity (%d) is not supported.\n",__FILE__,__LINE__, entity_type);
//      break;
//   }
//
//
//
//   fclose(file);
//
//   return 1;
// }

//! @ingroup graspPlanning 
//! Fonction de calcul d'une suite de points realisant un echantillonnage de la surface du carre
//! ([origin[0];origin[0]+factor],[origin[1];origin[1]+factor]).
//! L'echantillon calcule (copie dans 'result') est le n-ieme de la suite.
//! Computes the n-th sample of an incremental lattice sampling the surface of a square.
//! The method is from
//! "Incremental low-discrepancy lattice methods for motion planning"
//! by S. R. Lindemann and S. M. LaValle
//! In Proc. IEEE International Conference on Robotics and Automation, 2003
//! \param n index of the sample in the sequence
//! \param origin the sampled square is ([origin[0];origin[0]+factor],[origin[1];origin[1]+factor])
//! \param factor the sampled square is ([origin[0];origin[0]+factor],[origin[1];origin[1]+factor])
//! \param result the computed sample
void get_sample2D(int n, p3d_vector2 origin, double factor, p3d_vector2 result)
{
  static p3d_vector2 L[4]= { {0.0,0.0}, {0.5,0.5}, {0.5,0.0}, {0.0,0.5} };

  int index, nextN;
  p3d_vector2 sample;

  index= n%4;
  nextN= (int)(n/4);

  sample[0]= origin[0] + factor*L[index][0];
  sample[1]= origin[1] + factor*L[index][1];

  if( nextN==0 )
  {
    result[0]= sample[0];
    result[1]= sample[1];
    return;
  }
  else
  {
    factor/= 2.0;
  }

  get_sample2D((int) nextN, sample, factor, result);
}

//! @ingroup graspPlanning 
//! Fonction de calcul d'une suite de points realisant un echantillonnage du volume du cube
//! ([origin[0];origin[0]+factor],[origin[1];origin[1]+factor],[origin[2];origin[2]+factor]).
//! L'echantillon calcule (copie dans 'result') est le n-ieme de la suite.
//! Computes the n-th sample of an incremental lattice sampling the volume of a cube.
//! The method is from
//! "Incremental low-discrepancy lattice methods for motion planning"
//! by S. R. Lindemann and S. M. LaValle
//! In Proc. IEEE International Conference on Robotics and Automation, 2003
//! \param n index of the sample in the sequence
//! \param origin the sampled cube is ([origin[0];origin[0]+factor],[origin[1];origin[1]+factor],[origin[2];origin[2]+factor])
//! \param factor he sampled cube is ([origin[0];origin[0]+factor],[origin[1];origin[1]+factor],[origin[2];origin[2]+factor])
//! \param result the computed sample
void get_sample3D(int n, p3d_vector3 origin, double factor, p3d_vector3 result)
{
  static p3d_vector3 L[8]= { {0.0,0.0,0.0}, {0.5,0.0,0.0}, {0.0,0.0,0.5}, {0.5,0.0,0.5}, {0.0,0.5,0.0}, {0.5,0.5,0.0}, {0.0,0.5,0.5}, {0.5,0.5,0.5} };
//   L[0][0]= 0.0;   L[1][0]=  0.5;   L[2][0]=  0.0;   L[3][0]=  0.5;
//   L[0][1]= 0.0;   L[1][1]=  0.0;   L[2][1]=  0.0;   L[3][1]=  0.0;
//   L[0][2]= 0.0;   L[1][2]=  0.0;   L[2][2]=  0.5;   L[3][2]=  0.5;
// 
//   L[4][0]=  0.0;   L[5][0]=  0.5;   L[6][0]=  0.0;   L[7][0]=  0.5;
//   L[4][1]=  0.5;   L[5][1]=  0.5;   L[6][1]=  0.5;   L[7][1]=  0.5;
//   L[4][2]=  0.0;   L[5][2]=  0.0;   L[6][2]=  0.5;   L[7][2]=  0.5;

  int index, nextN;
  p3d_vector3 sample;

  index= n%8;
  nextN= (int)(n/8);

  sample[0]= origin[0] + factor*L[index][0];
  sample[1]= origin[1] + factor*L[index][1];
  sample[2]= origin[2] + factor*L[index][2];

  if( nextN==0 )
  {
    result[0]= sample[0];
    result[1]= sample[1];
    result[2]= sample[2];
    return;
  }
  else
  {
    factor/= 2.0;
  }

  get_sample3D((int) nextN, sample, factor, result);
}


//! @ingroup graspPlanning 
//! Writes the content of the p3d_matrix4 in a float array with the format used by OpenGL (when calling a function
//! like glLoadMatrix or glMultMatrix).
void p3d_matrix4_to_OpenGL_format(p3d_matrix4 source, GLfloat mat[16])
{
  mat[0]= source[0][0];    mat[4]= source[0][1];    mat[8]=  source[0][2];    mat[12]= source[0][3];
  mat[1]= source[1][0];    mat[5]= source[1][1];    mat[9]=  source[1][2];    mat[13]= source[1][3];
  mat[2]= source[2][0];    mat[6]= source[2][1];    mat[10]= source[2][2];    mat[14]= source[2][3];
  mat[3]=            0;    mat[7]=            0;    mat[11]=            0;    mat[15]=            1;
}

//! @ingroup graspPlanning 
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpExport_bodies_for_coldman(p3d_rob *robot)
{
  size_t pos;
  unsigned int k, countM;
  int i, j, shift;
  double color_vect[4];
  p3d_index *indices= NULL;
  p3d_vector3 p1, p2;
  p3d_matrix4 T, T2, Tinv;
  p3d_obj *body;
  FILE *file= NULL;
  char *path= NULL;
  std::string bodyName, pathName, objName, mtlName;

  path= getenv("HOME_MOVE3D");
  
  if(path==NULL)  {
	pathName.assign("./graspPlanning/export/");
  }
  else  { 
    pathName.assign(path);	
    pathName+= "/graspPlanning/export/";
  }
  
  for(i=0; i<robot->no; i++)
  {
    body= robot->o[i];
    bodyName= body->name;
    pos= bodyName.find_last_of('.');
    if(pos!=bodyName.npos)
    { 
      bodyName= bodyName.substr(pos+1, bodyName.length());
    }
    
    // first, write the .obj file:
    objName= pathName + bodyName + ".obj";

    file= fopen(objName.c_str(), "w");
    if(file==NULL)
    { 
      printf("%s: %d: gpExport_bodies_for_coldman(): can not open %s.\n", __FILE__,__LINE__,objName.c_str());
      return GP_ERROR;
    }

    fprintf(file, "# %s\n",  bodyName.c_str());
    fprintf(file, "mtllib %s.mtl\n",  bodyName.c_str());
    fprintf(file, "o unnamed_object1\n");

    for(j=0; j<body->np; j++)
    {
      if(body->pol[j]->p3d_objPt!=body)
      {  continue;  }

      p3d_mat4Copy(body->pol[j]->pos0, T2);
      p3d_matInvertXform(body->jnt->pos0_obs, Tinv);
      p3d_matMultXform(Tinv, T2,  T);

      for(k=0; k<body->pol[j]->poly->nb_points; k++)
      {
        p3d_vectCopy(body->pol[j]->poly->the_points[k], p1);
        p3d_xformPoint(T, p1, p2);
        fprintf(file, "v %f %f %f\n", p2[0], p2[1], p2[2]);
      }
    }

    fprintf(file, "g unnamed_object1\n");
    shift= 0;
    countM= 1;
    for(j=0; j<body->np; j++)
    {
      if(body->pol[j]->p3d_objPt!=body)
      {  continue;  }

      fprintf(file, "usemtl material%d\n", countM);
      countM++;

      for(k=0; k<body->pol[j]->poly->nb_faces; k++)
      {
        indices= body->pol[j]->poly->the_faces[k].the_indexs_points;
        if(body->pol[j]->poly->the_faces[k].nb_points==3)
        {
          fprintf(file, "f %d %d %d\n", indices[0]+shift, indices[1]+shift, indices[2]+shift);
        }
        else
        {
          #ifndef PQP
          printf("%s: %d: gpExport_bodies_for_coldman(): some functions in p3d_pqp are needed to deal with non triangular faces.\n", __FILE__,__LINE__);
          #else
          triangles= pqp_triangulate_face(body->pol[j]->poly, k, &nb_triangles);
          if(triangles!=NULL)
          {
            for(it=0; it<nb_triangles; it++)
            {
              fprintf(file, "f %d %d %d\n", triangles[it][0]+1+shift, triangles[it][1]+1+shift, triangles[it][2]+1+shift);
            }
            free(triangles);
          }
          #endif
        }
      }
      shift+= body->pol[j]->poly->nb_points;
    }

    fclose(file);
    file= NULL;

    // now, write the .mtl file:
    //sprintf(str, "./graspPlanning/export/%s.mtl",  bodyName.c_str());
    mtlName= pathName + bodyName + ".mtl";
    
    file= fopen(mtlName.c_str(), "w");
    if(file==NULL)
    { 
       printf("%s: %d: gpExport_bodies_for_coldman(): can not open %s.\n", __FILE__,__LINE__,mtlName.c_str());
       return GP_ERROR;
    }

    fprintf(file, "# %s material\n",  bodyName.c_str());

    countM= 1;
    for(j=0; j<body->np; j++)
    {
      if(body->pol[j]->p3d_objPt!=body)
      {  continue;  }

      fprintf(file, "newmtl material%d\n", countM);
      countM++;

      fprintf(file, "Ns 100.00\n");
      fprintf(file, "d 1.0\n");
      fprintf(file, "illum 2\n");

      if(body->pol[j]->color_vect==NULL)
      { 
        g3d_get_color_vect(body->pol[j]->color, color_vect);
      }
      else
      { 
        color_vect[0]= body->pol[j]->color_vect[0];
        color_vect[1]= body->pol[j]->color_vect[1];
        color_vect[2]= body->pol[j]->color_vect[2];
      }

      fprintf(file, "Kd %f %f %f\n", 1.0*color_vect[0], 1.0*color_vect[1], 1.0*color_vect[2]);
      fprintf(file, "Ka %f %f %f\n", 0.7*color_vect[0], 0.7*color_vect[1], 0.7*color_vect[2]);
      fprintf(file, "Ks %f %f %f\n", 0.8*color_vect[0], 0.8*color_vect[1], 0.8*color_vect[2]);
      fprintf(file, "Ke %f %f %f\n", 0.2*color_vect[0], 0.2*color_vect[1], 0.2*color_vect[2]);
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Exports the different environment obstacles as separate .obj files with
//! associated .mtl file containing the same colors as the ones defined in the p3d models.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpExport_obstacles_for_coldman()
{
  size_t pos;
  unsigned int k, countM;
  int i, j, shift;
  double color_vect[4];
  p3d_index *indices= NULL;
  p3d_vector3 p1, p2;
  p3d_matrix4 T, Tinv;
  p3d_obj *body;
  char str[128];
  FILE *file= NULL;
  std::string bodyName;

  for(i=0; i<XYZ_ENV->no; i++)
  {
    body= XYZ_ENV->o[i];
    bodyName= body->name;
    pos= bodyName.find_last_of('.');
    if(pos!=bodyName.npos)
    { 
      bodyName= bodyName.substr(pos+1, bodyName.length());
    }

    // first, write the .obj file:
    sprintf(str, "./graspPlanning/export/%s.obj", bodyName.c_str());
    file= fopen(str, "w");
    if(file==NULL)
    { 
      printf("%s: %d: gpExport_obstacles_for_coldman(): can not open %s.\n", __FILE__,__LINE__,str);
      return GP_ERROR;
    }

    fprintf(file, "# %s\n",  bodyName.c_str());
    fprintf(file, "mtllib %s.mtl\n",  bodyName.c_str());
    fprintf(file, "o unnamed_object1\n");

    for(j=0; j<body->np; j++)
    {
      if(body->pol[j]->p3d_objPt!=body)
      {  continue;  }

      if(j==0) 
      {  p3d_mat4Copy(p3d_mat4IDENTITY, T); }
      else
      {
         p3d_matInvertXform(body->pol[0]->pos0, Tinv);
         p3d_matMultXform(Tinv, body->pol[j]->pos0, T);
      }


      for(k=0; k<body->pol[j]->poly->nb_points; k++)
      {
        p3d_vectCopy(body->pol[j]->poly->the_points[k], p1);
        p3d_xformPoint(T, p1, p2);
        fprintf(file, "v %f %f %f\n", p2[0], p2[1], p2[2]);
      }
    }

    fprintf(file, "g unnamed_object1\n");
    shift= 0;
    countM= 1;
    for(j=0; j<body->np; j++)
    {
      if(body->pol[j]->p3d_objPt!=body)
      {  continue;  }

      fprintf(file, "usemtl material%d\n", countM);
      countM++;

      for(k=0; k<body->pol[j]->poly->nb_faces; k++)
      {
        indices= body->pol[j]->poly->the_faces[k].the_indexs_points;
        if(body->pol[j]->poly->the_faces[k].nb_points==3)
        {
          fprintf(file, "f %d %d %d\n", indices[0]+shift, indices[1]+shift, indices[2]+shift);
        }
        else
        {
          #ifndef PQP
          printf("%s: %d: gpExport_obstacles_for_coldman(): some functions in p3d_pqp are needed to deal with non triangular faces.\n", __FILE__,__LINE__);
          #else
          triangles= pqp_triangulate_face(body->pol[j]->poly, k, &nb_triangles);
          if(triangles!=NULL)
          {
            for(it=0; it<nb_triangles; it++)
            {
              fprintf(file, "f %d %d %d\n", triangles[it][0]+1+shift, triangles[it][1]+1+shift, triangles[it][2]+1+shift);
            }
            free(triangles);
          }
          #endif
        }
      }
      shift+= body->pol[j]->poly->nb_points;
    }

    fclose(file);
    file= NULL;

    // now, write the .mtl file:
    sprintf(str, "./graspPlanning/export/%s.mtl",  bodyName.c_str());
    file= fopen(str, "w");
    if(file==NULL)
    { 
       printf("%s: %d: gpExport_obstacles_for_coldman(): can not open %s.\n", __FILE__,__LINE__,str);
       return GP_ERROR;
    }

    fprintf(file, "# %s material\n",  bodyName.c_str());

    countM= 1;
    for(j=0; j<body->np; j++)
    {
      if(body->pol[j]->p3d_objPt!=body)
      {  continue;  }

      fprintf(file, "newmtl material%d\n", countM);
      countM++;

      fprintf(file, "Ns 100.00\n");
      fprintf(file, "d 1.0\n");
      fprintf(file, "illum 2\n");

      if(body->pol[j]->color_vect==NULL)
      { 
        g3d_get_color_vect(body->pol[j]->color, color_vect);
      }
      else
      { 
        color_vect[0]= body->pol[j]->color_vect[0];
        color_vect[1]= body->pol[j]->color_vect[1];
        color_vect[2]= body->pol[j]->color_vect[2];
      }

      fprintf(file, "Kd %f %f %f\n", 1.0*color_vect[0], 1.0*color_vect[1], 1.0*color_vect[2]);
      fprintf(file, "Ka %f %f %f\n", 0.7*color_vect[0], 0.7*color_vect[1], 0.7*color_vect[2]);
      fprintf(file, "Ks %f %f %f\n", 0.8*color_vect[0], 0.8*color_vect[1], 0.8*color_vect[2]);
      fprintf(file, "Ke %f %f %f\n", 0.2*color_vect[0], 0.2*color_vect[1], 0.2*color_vect[2]);
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Computes the axis-aligned bounding box of a polyhedron.
//! \param polyhedron pointer to the polyhedron
//! \param xmin minimal coordinate along X-axis
//! \param xmax maximal coordinate along X-axis
//! \param ymin minimal coordinate along Y-axis
//! \param ymax maximal coordinate along Y-axis
//! \param zmin minimal coordinate along Z-axis
//! \param zmax maximal coordinate along Z-axis
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpPolyhedron_AABB(p3d_polyhedre *polyhedron, double &xmin, double &xmax, double &ymin, double &ymax, double &zmin, double &zmax)
{
   if(polyhedron==NULL)
   {
     printf("%s: %d: gpPolyhedron_AABB(): input p3d_polyhedre is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }

   unsigned int i, j;
   double x, y, z;
   p3d_index *indices= NULL;
   p3d_vector3 *points= NULL;
   p3d_face *faces= NULL;


   points= polyhedron->the_points;
   faces= polyhedron->the_faces;
   // we parse all the faces and not the points because we are only interested 
   // in the face points (some points may belong to no face):
   for(i=0; i<polyhedron->nb_faces; ++i)
   {
     indices= faces[i].the_indexs_points;

     for(j=0; j<faces[i].nb_points; ++j)
     {
       x= points[indices[j]-1][0];
       y= points[indices[j]-1][1];
       z= points[indices[j]-1][2];

       if(i==0 && j==0)
       {
         xmin= xmax= x;
         ymin= ymax= y;
         zmin= zmax= z;
         continue;
       }

       if(x < xmin) {  xmin= x;  }
       if(x > xmax) {  xmax= x;  }
       if(y < ymin) {  ymin= y;  }
       if(y > ymax) {  ymax= y;  }
       if(z < zmin) {  zmin= z;  }
       if(z > zmax) {  zmax= z;  }
     }
   }

   return GP_OK;
}


//! @ingroup graspPlanning 
//! Computes the axis-aligned bounding box of a p3d_obj.
//! \param obj pointer to the p3d_obj
//! \param xmin minimal coordinate along X-axis
//! \param xmax maximal coordinate along X-axis
//! \param ymin minimal coordinate along Y-axis
//! \param ymax maximal coordinate along Y-axis
//! \param zmin minimal coordinate along Z-axis
//! \param zmax maximal coordinate along Z-axis
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpObj_AABB(p3d_obj *obj, double &xmin, double &xmax, double &ymin, double &ymax, double &zmin, double &zmax)
{
   if(obj==NULL)
   {
     printf("%s: %d: gpObj_AABB(): input p3d_obj is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }

   unsigned int i;
   double xmin_i, xmax_i, ymin_i, ymax_i, zmin_i, zmax_i;

   for(i=0; i<(unsigned int) obj->np; ++i)
   {
     gpPolyhedron_AABB(obj->pol[i]->poly, xmin_i, xmax_i, ymin_i, ymax_i, zmin_i, zmax_i);

     if(i==0)
     {
       xmin= xmin_i;
       xmax= xmax_i;
       ymin= ymin_i;
       ymax= ymax_i;
       zmin= zmin_i; 
       zmax= zmax_i; 
     }
     else
     {
       if(xmin_i < xmin) { xmin= xmin_i; }
       if(xmax_i > xmax) { xmax= xmax_i; }
       if(ymin_i < ymin) { ymin= ymin_i; }
       if(ymax_i > ymax) { ymax= ymax_i; }
       if(zmin_i < zmin) { zmin= zmin_i; }
       if(zmax_i > zmax) { zmax= zmax_i; }
     }
   }

   return GP_OK;
}


//! @ingroup graspPlanning 
//! Prints the AABBs of each body of a robot. It is meant to be used to automatically compute
//! P3D_GHOST volumes for the robot.
//! \param robot pointer to the robot
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpPrint_robot_AABBs(p3d_rob *robot)
{
   if(robot==NULL)
   {
     printf("%s: %d: gpPrint_robot_AABBs(): input p3d_rob is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }

   int i;
   double xmin, xmax, ymin, ymax, zmin, zmax;
   double tx, ty, tz, ax, ay, az;

   printf("AABBs for robot \"%s\" \n", robot->name);
   for(i=0; i<robot->no; ++i)
   {
     printf("\t %s: \n", robot->o[i]->name);
//      gpObj_AABB(robot->o[i], xmin, xmax, ymin, ymax, zmin, zmax);
//      printf("\t %s: [ %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ]\n", robot->o[i]->name, xmin, xmax, ymin, ymax, zmin, zmax);

//      p3d_get_body_pose(robot, i, pose);
//      p3d_mat4ExtractPosReverseOrder2(pose, &tx, &ty, &tz, &ax, &ay, &az);
      pqp_top_OBB(robot->o[i], tx, ty, tz, ax, ay, az, xmin, xmax, ymin, ymax, zmin, zmax);
     printf("\t p3d_add_desc_box base1 %f %f %f P3D_GHOST\n",(xmax-xmin),(ymax-ymin),(zmax-zmin));

     printf("\t p3d_set_body_abs_pos %f %f %f %f %f %f\n\n", tx, ty, tz, ax*RADTODEG, ay*RADTODEG, az*RADTODEG);

//      p3d_mat4Print(pose, "pose");
   }

   return GP_OK;
}

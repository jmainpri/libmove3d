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




//! Calcule l'inverse d'une matrice 2x2.
//! \param mat a 2x2 matrix
//! \param invmat the inverse of mat
int p3d_mat2Invert(p3d_matrix2 mat, p3d_matrix2 invmat)
{
   double det= mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1];
   if( fabs(det) < EPSILON)
   {
      #ifdef DEBUG
        //printf("%s: %d: p3d_mat2Invert(): matrice non inversible\n", __FILE__, __LINE__);
      #endif
      return 0;
   }

   invmat[0][0]=   mat[1][1]/det;    invmat[0][1]=  -mat[0][1]/det;
   invmat[1][0]=  -mat[1][0]/det;    invmat[1][1]=   mat[0][0]/det;

   return 1;
}


//! Simple adaptation de la fonction p3d_mat4Rot aux matrices 3x3.
//! Calcule la matrice correspondant a la rotation d'axe et d'angle donnes en parametres.
//! Compute and fill a rotation matrix for a rotation described by an axis and an angle.
//! \param M the computed rotation matrix
//! \param axis the axis of the desired rotation
//! \param t the angle of the desired rotation
void p3d_mat3Rot(p3d_matrix3 M, p3d_vector3 axis, double t)
{
  double norm, c, s, v;
  double x, y, z;

  x = axis[0];
  y = axis[1];
  z = axis[2];

  p3d_mat3Copy(p3d_mat3IDENTITY,M);

  norm = p3d_vectNorm(axis);
  if (norm == 0.0) {
    return;
  }

  x /= norm;
  y /= norm;
  z /= norm;

  c = cos(t);
  s = sin(t);
  v = 1 - c;

  M[0][0] = x*x*v + c;
  M[1][0] = x*y*v + z*s;
  M[2][0] = x*z*v - y*s;
  M[0][1] = x*y*v - z*s;
  M[1][1] = y*y*v + c;
  M[2][1] = y*z*v + x*s;
  M[0][2] = x*z*v + y*s;
  M[1][2] = y*z*v - x*s;
  M[2][2] = z*z*v + c;
}

//! Recopie dans R la matrice de rotation 3x3 extraite d'une matrice de transformation homogene 4x4 M.
//! \param M a 4x4 homogeneous transform input matrix
//! \param R the output matrix filled with the rotation part of M
#ifdef C99
inline
#endif
void p3d_mat4ExtractRotMatrix( p3d_matrix4 M, p3d_matrix3 R)
{
  R[0][0]= M[0][0];  R[0][1]= M[0][1];  R[0][2]= M[0][2];
  R[1][0]= M[1][0];  R[1][1]= M[1][1];  R[1][2]= M[1][2];
  R[2][0]= M[2][0];  R[2][1]= M[2][1];  R[2][2]= M[2][2];
}

//! Remplit la matrice M de sorte qu'elle soit la matrice de transformation correspondant
//! a un translation de vecteur (tx, ty, tz) et a une translation d'axe "axis" et d'angle "angle".
#ifdef C99
inline
#endif
void p3d_mat4TransRot( p3d_matrix4 M, double tx, double ty, double tz, p3d_vector3 axis, double angle)
{
   p3d_mat4Rot(M, axis, angle);
   M[0][3]= tx;
   M[1][3]= ty;
   M[2][3]= tz;
}


//! Extrait de la matrice M le vecteur correspondant a la translation.
#ifdef C99
inline
#endif
void p3d_mat4ExtractTrans ( p3d_matrix4 M, p3d_vector3 v)
{
   v[0]= M[0][3];
   v[1]= M[1][3];
   v[2]= M[2][3];
}


//Cette fonction est maintenant dans p3d_matrix.c
/*
// Calcule l'angle et l'axe d'une rotation a partir d'une matrice de transformation 4x4
// (l'ancienne fonction move3d de base
// ne marche pas pour certains cas singuliers).
// Conversion d'un code en Java de Martin Baker (http://www.euclideanspace.com/index.html).
void p3d_mat4ExtractRot ( p3d_matrix4 M, p3d_vector3 axis, double *angle )
{
  double epsilon = 0.01; // margin to allow for rounding errors

  if ((fabs(M[0][1]-M[1][0])< epsilon)  && (fabs(M[0][2]-M[2][0])< epsilon)  && (fabs(M[1][2]-M[2][1])< epsilon))
  {
    // singularity found
    // first check for identity matrix which must have +1 for all terms in leading diagonal
    // and zero in other terms
    if ( (fabs(M[0][1]+M[1][0]) < 0.1)  && (fabs(M[0][2]+M[2][0]) < 0.1)  && (fabs(M[1][2]+M[2][1]) < 0.1)
      && (fabs(M[0][0]+M[1][1]+M[2][2]-3) < 0.1) )
    {
      // this singularity is identity matrix so angle = 0
      // note epsilon is greater in this case since we only have to distinguish between 0 and 180 degrees
      // zero angle, arbitrary axis
      axis[0]= 1;
      axis[1]= 0;
      axis[2]= 0;
      *angle= 0;
      return;
    }

    // otherwise this singularity is angle = 180
    *angle = M_PI;
    axis[0] = (M[0][0]+1)/2;
    if (axis[0] > 0)
    { // can only take square root of positive number, always true for orthogonal matrix
      axis[0] = sqrt(axis[0]);
    }
    else
    {
      axis[0] = 0; // in case matrix has become de-orthogonalised
    }

    axis[1] = (M[1][1]+1)/2;
    if (axis[1] > 0)
    { // can only take square root of positive number, always true for orthogonal matrix
      axis[1] = sqrt(axis[1]);
    }
    else
    {
      axis[1] = 0; // in case matrix has become de-orthogonalised
    }

    axis[2] = (M[2][2]+1)/2;
    if (axis[2] > 0)
    { // can only take square root of positive number, always true for orthogonal matrix
      axis[2] = sqrt(axis[2]);
    }
    else
    {
      axis[2] = 0; // in case matrix has become de-orthogonalised
    }


    int xZero = (fabs(axis[0])<epsilon);
    int yZero = (fabs(axis[1])<epsilon);
    int zZero = (fabs(axis[2])<epsilon);
    int xyPositive = (M[0][1] > 0);
    int xzPositive = (M[0][2] > 0);
    int yzPositive = (M[1][2] > 0);
    if (xZero && !yZero && !zZero)
    { // implements  last 6 rows of above table
      if (!yzPositive)
        axis[1] = -axis[1];
    }
    else
    {
      if (yZero && !zZero)
      {
	 if (!xzPositive)
          axis[2] = -axis[2];
      }
      else
      {
        if (zZero)
        {
	  if (!xyPositive)
           axis[0] = -axis[0];
        }
      }
    }

    return;
  }


  double s= sqrt((M[2][1] - M[1][2])*(M[2][1] - M[1][2])+(M[0][2] - M[2][0])*(M[0][2] - M[2][0])+(M[1][0] - M[0][1])*(M[1][0] - M[0][1])); // used to normalise

  if (fabs(s) < 0.001) s=1; // prevent divide by zero, should not happen
                            // if matrix is orthogonal and should be
  // caught by singularity test above, but I've left it in just in case
  *angle = acos(( M[0][0] + M[1][1] + M[2][2] - 1)/2);
  axis[0] = (M[2][1] - M[1][2])/s;
  axis[1] = (M[0][2] - M[2][0])/s;
  axis[2] = (M[1][0] - M[0][1])/s;

  return;
}*/



//! This function just gets the pose of the given object.
int p3d_get_obj_pos(p3d_obj *o, p3d_matrix4 pose)
{
   #ifdef DEBUG
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



//! Cette fonction permet de retrouver l'indice d'une liaison dans le tableau des liaisons d'un robot,
//! a partir de son nom.
//! Find the index of a robot joint from its name.
//! It is the index in the robot's joint array and it starts from 0.
//! \param robot pointer to the robot
//! \param name name of the searched joint
//! \return the index of the joint if it is found, 0 otherwise
int get_robot_jnt_index_by_name(p3d_rob* robot, char *name)
{
 #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: get_robot_jnt_index_by_name(): robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
  if(name==NULL)
  {
    printf("%s: %d: get_robot_jnt_index_by_name(): name is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
 #endif

  int i;
  for(i=0; i<=robot->njoints; i++)
  {
     if(robot->joints[i]->name==NULL)
       continue;

     if( strcmp(robot->joints[i]->name, name) == 0 )
     {
        return i;
     }
  }

  printf("%s: %d: get_robot_jnt_index_by_name(): robot \"%s\" has no joint named \"%s\".\n", __FILE__, __LINE__, robot->name, name);
  return 0;

}

//! Finds a robot's joint from its name.
//! \param robot pointer to the robot
//! \param name name of the searched joint
//! \return pointer to the joint if it is found, NULL otherwise
p3d_jnt * get_robot_jnt_by_name(p3d_rob* robot, char *name)
{
 #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: get_robot_jnt_by_name(): robot is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }
  if(name==NULL)
  {
    printf("%s: %d: get_robot_jnt_by_name(): name is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }
 #endif

  int i;
  for(i=0; i<=robot->njoints; i++)
  {
     if(robot->joints[i]->name==NULL)
       continue;

     if( strcmp(robot->joints[i]->name, name) == 0 )
     {
        return robot->joints[i];
     }
  }

  printf("%s: %d: get_robot_jnt_by_name(): robot \"%s\" has no joint named \"%s\".\n", __FILE__, __LINE__, robot->name, name);

  return NULL;
}


//! Draws the joint frames of a robot.
//! \param robot pointer to the robot
//! \param size length of the frame arrows that will be drawn.
//! \return 1 in case of success, 0 otherwise
int p3d_draw_robot_joints(p3d_rob* robot, double size)
{
 #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: p3d_draw_robot_joints(): robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
 #endif

  int i;
  for(i=0; i<=robot->njoints; i++)
  {
    draw_frame(robot->joints[i]->abs_pos, size);
/*    printf("joint: %s\n", robot->joints[i]->name);
    printf("\t %f %f %f\n", robot->joints[i]->pos0[0][3], robot->joints[i]->pos0[1][3], robot->joints[i]->pos0[2][3]);*/
  }

  return 1;
}

//! Finds a robot body from its name.
//! \param robot pointer to the robot
//! \param name name of the searched body (without the prefix "robot_name.")
//! \return pointer to the joint if it is found, NULL otherwise
p3d_obj * get_robot_body_by_name(p3d_rob* robot, char *name)
{
 #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: get_robot_body_by_name(): robot is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }
  if(name==NULL)
  {
    printf("%s: %d: get_robot_body_by_name(): name is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }
 #endif

  int i;
  std::string body_name;
  body_name= std::string(robot->name) + "." + std::string(name);

  for(i=0; i<robot->no; i++)
  {
     if(robot->o[i]->name==NULL)
       continue;

     if( strcmp(robot->o[i]->name, body_name.c_str()) == 0 )
     {
        return robot->o[i];
     }
  }


//   printf("%s: %d: get_robot_body_by_name(): robot \"%s\" has no body named \"%s\".\n", __FILE__, __LINE__, robot->name, name);


  return NULL;
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

int draw_p3d_polyhedre(p3d_polyhedre *polyhedron)
{
 #ifdef DEBUG
  if(polyhedron==NULL)
  {
    printf("%s: %d: draw_p3d_polyhedre(): NULL input pointer.\n", __FILE__, __LINE__);
    return 0;
  }
 #endif

// double color_tab[16][3]= { {1,0,0}, {0,1,0}, {0,0,1}, {1,1,0}, {1,0,1}, {0,1,1} , {1,0.5,0.5}, {0.5,1,0.5}, {0.5,0.5,1}, {1,0.25,0.5}, {1,0.5,0.25}, {0.25,1.0,0.5}, {0.5,1,0.25}, {0.25,0.5,1}, {0.5,0.25,1}  };

  unsigned int i, j;
//   float d= 0.03;
  double t;
  p3d_matrix4 pose;
  p3d_vector3 axis;
  p3d_vector3 *points=  polyhedron->the_points;
  //p3d_vector3 *normals=  polyhedron->normals;
  p3d_face *faces= polyhedron->the_faces;
  //gluPerspective(40.0, 1.2 , 0.01, 100.0);

  p3d_get_poly_pos( polyhedron, pose );
  p3d_mat4ExtractRot(pose, axis, &t);

//   glEnable(GL_LIGHTING);
//   glEnable(GL_LIGHT0);
//   glPointSize(6);

  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT | GL_POINT_BIT);


  glPushMatrix();
  // glTranslatef(pose[0][3], pose[1][3], pose[2][3]);
  // glRotatef((180/M_PI)*t,axis[0], axis[1], axis[2]);
/*
g3d_set_color_mat(Red, NULL);
   glBegin(GL_POINTS);
   for(i= 0; i<polyhedron->nb_points; i++)
   {
     glVertex3dv(polyhedron->the_points[i]);

   }
   glEnd();
*/

/*
   if(normals!=NULL)
   {
     g3d_set_color_mat(Red, NULL);
     glBegin(GL_LINES);
     for(i=0; i<polyhedron->nb_points; i++)
     {
       glVertex3dv( points[i] );
       glVertex3d( points[i][0] + d*normals[i][0], points[i][1] + d*normals[i][1], points[i][2] + d*normals[i][2] );
     }
     glEnd();
   }
*/
//    glPointSize(4);
//    glColor3f(1, 0, 0);
//    glDisable(GL_LIGHTING);
//    for(i=0; i<polyhedron->nb_points; i++)
//    {
//      glBegin(GL_POINTS);
//        glVertex3dv(points[i]);
//      glEnd();
//    }
   glEnable(GL_LIGHTING);
//    g3d_set_color_mat(Green, NULL);
//    glShadeModel(GL_SMOOTH);
   for(i=0; i<polyhedron->nb_faces; i++)
   {
//      if(faces[i].part>0)
//      {  g3d_set_color_mat(Any, color_tab[faces[i].part%15]);  }
     glBegin(GL_POLYGON);
       for(j=0; j<faces[i].nb_points; j++)
       {
         if( faces[i].plane!=NULL )
         {   glNormal3dv(faces[i].plane->normale);  }
         glVertex3dv(points[faces[i].the_indexs_points[j]-1]);
       }
     glEnd();
   }


//    glDisable(GL_LIGHTING);
//    glColor3f(0,0,0);
//    for(i=0; i<polyhedron->nb_faces; i++)
//    {
//      glBegin(GL_LINE_LOOP);
//        for(j=0; j<faces[i].nb_points; j++)
//        {
//          glVertex3dv(points[faces[i].the_indexs_points[j]-1]);
//        }
//      glEnd();
//    }



  glPopMatrix();

  glPopAttrib();

  return 1;
}


//! Calcule, pour chaque face, les indices de ses voisines (les faces partageant une de
//! ses arêtes).
//! Les faces doivent être triangulaires, auquel cas une face a au maximum trois voisines.
//! Un triangle peut avoir moins de trois voisins auquel cas les indices correspondants
//! seront mis a la valeur -1.
//! Les indices des faces voisines sont les indices dans le tableau de faces du polyedre
//! et commencent a 0.
int gpCompute_face_neighbours(p3d_polyhedre *polyhedron)
{
   #ifdef DEBUG
    if(polyhedron==NULL)
    {  printf("%s: %d: gpCompute_face_neighbours(): entree= NULL.\n",__FILE__,__LINE__);
       return 0; }
   #endif

   int i, j, ei, ej;
   int ei1, ej1, ei2, ej2;
   int nb_faces= polyhedron->nb_faces;
   p3d_face *faces= polyhedron->the_faces;

   //met tous les indices des sommetsVoisins a -1
   for(i=0; i<nb_faces; i++)
   {
      faces[i].neighbours[0]= -1;
      faces[i].neighbours[1]= -1;
      faces[i].neighbours[2]= -1;
   }

   //Pour tous les triangles sauf le dernier
   for(i=0; i<nb_faces-1; i++)
   {
     //Pour chaque arête
     for(ei=0; ei<3; ei++)
     {
       //On continue si on connait deja son voisin
       if(faces[i].neighbours[ei]!=-1)
         continue;

       //on recupere les indices des sommets
       //de l'arête:
       ei1= faces[i].the_indexs_points[ei];
       ei2= faces[i].the_indexs_points[(ei+1)%3];

       //Pour les triangles d'indice superieur
       for(j=i+1; j<nb_faces; j++)
       {
          //Pour chaque arête :
          for(ej=0; ej<3; ej++)
          {
              //on recupere les indices des sommets
              //de l'arête:
              ej1= faces[j].the_indexs_points[ej];
              ej2= faces[j].the_indexs_points[(ej+1)%3];

              //Si les arêtes sont les mêmes, les triangles sont voisins:
              if( ( (ei1==ej1)&&(ei2==ej2) )||( (ei1==ej2)&&(ei2==ej1) ) )
              {
                  faces[i].neighbours[ei]= j;
                  faces[j].neighbours[ej]= i;
              }
          }
       }
     }
   }

   return 1;
}


//! Calcule les normales de chaque sommet du polyedre.
//! La normale d'un sommet est la somme des normales de chaque triangle auquel il appartient,
//! ponderee par l'angle forme par les deux arêtes  du triangle auxquelles appartient le sommet.
int p3d_compute_vertex_normals(p3d_polyhedre *polyhedron)
{
   #ifdef DEBUG
    if(polyhedron==NULL)
    {  printf("%s: %d: p3d_compute_vertex_normals(): entree= NULL.\n",__FILE__,__LINE__);
       return 0; }
   #endif

   unsigned int i, j, index0, index1, index2;
   double vertex_angle;
   p3d_vector3 e1, e2;
   p3d_vector3 *points= polyhedron->the_points;
   p3d_face *faces= polyhedron->the_faces;

   if(polyhedron->normals==NULL)
   {
      free(polyhedron->normals);
   }

   polyhedron->normals= (p3d_vector3 *) malloc(polyhedron->nb_points*sizeof(p3d_vector3));

   for(i=0; i<polyhedron->nb_points; i++)
   {
     polyhedron->normals[i][0]= 0.0;
     polyhedron->normals[i][1]= 0.0;
     polyhedron->normals[i][2]= 0.0;
   }

   for(i=0; i<polyhedron->nb_faces; i++)
   {
     printf("face %p: %d points\n", &faces[i], faces[i].nb_points);
     for(j=0; j<faces[i].nb_points; j++)
     {
        printf("\t point %d\n", faces[i].the_indexs_points[j]-1 );
     }

     for(j=0; j<faces[i].nb_points; j++)
     {

       index0= faces[i].the_indexs_points[ j ] - 1;
       index1= faces[i].the_indexs_points[ (j + 1)%faces[i].nb_points ] - 1;
       index2= faces[i].the_indexs_points[ (j + 2)%faces[i].nb_points ] - 1;
      // printf("\t \t indices %d %d %d\n", index0, index1, index2);
       p3d_vectSub(points[index0], points[index1], e1);
       p3d_vectSub(points[index2], points[index1], e2);

       p3d_vectNormalize(e1, e1);
       p3d_vectNormalize(e2, e2);

       vertex_angle= fabs( acos( p3d_vectDotProd(e1, e2) ) );

       polyhedron->normals[index1][0]+= vertex_angle*( faces[i].plane->normale[0] );
       polyhedron->normals[index1][1]+= vertex_angle*( faces[i].plane->normale[1] );
       polyhedron->normals[index1][2]+= vertex_angle*( faces[i].plane->normale[2] );
     }

   }

   for(i=0; i<polyhedron->nb_points; i++)
   {
     p3d_vectNormalize(polyhedron->normals[i], polyhedron->normals[i]);
   }

   return 1;
}


//! Ecrit dans un fichier les indices des triangles voisins de chaque face du polyedre.
int p3d_print_face_neighbours(p3d_polyhedre *polyhedron, char *filename)
{
   #ifdef DEBUG
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

//! Enregistre au format .obj (format wavefront), une structure p3d_polyhedre.
int p3d_save_in_OBJ_format(p3d_polyhedre *polyhedron, char *name)
{
   #ifdef DEBUG
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


//! Affiche la face d'indice "index" du polyedre.
//! L'indice doit être compris entre 0 et nb_faces-1
//! A utiliser dans une fonction d'affichage OpenGL.
int p3d_display_face(p3d_polyhedre *polyhedron, unsigned int index)
{
  #ifdef DEBUG
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

    glColor3f(0.0, 1.0, 0.0);
    for(i=0; i<faces[index].nb_points; i++)
    {
      g3d_drawSphere(points[ind[i]-1][0],  points[ind[i]-1][1], points[ind[i]-1][2], 0.0005, Green, NULL);
    }

  glPopMatrix();

  glEnable(GL_LIGHTING);

  return 1;
}



//! Retourne un pointeur sur le robot dont le nom est donne en parametre.
//! \param name the name of the robot
//! \return a pointer to the robot with the given name
p3d_rob *p3d_get_robot_by_name(char *name)
{
  #ifdef DEBUG
   if(name==NULL)
   {
     printf("%s: %d: p3d_get_robot_by_name(): name is NULL.\n", __FILE__, __LINE__);
     return NULL;
   }
  #endif

   int i, r, nr;
   p3d_rob *robot;
   r = p3d_get_desc_curnum(P3D_ROBOT);
   nr= p3d_get_desc_number(P3D_ROBOT);

   for(i=0; i<nr; i++)
   {
     robot= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
     if(strcmp(name, robot->name)==0)
     {  return(robot);  }
   }

   printf("%s: %d: p3d_get_robot_by_name(): there is no robot named \"%s\".\n", __FILE__, __LINE__, name);

   return NULL;
}


//! \deprecated Do not use anymore but the code might be of some interest.
// Test de collision entre un robot et un objet.
// Les tests d'autocollision du robot sont aussi effectues.
// Tous les autres tests de collision (avec les autres objets et robots) ne sont
// pas pris en compte.
// Retourne 0 s'il n'y a pas de collision, 1 sinon.
int p3d_col_test_rob_obj(p3d_rob *robot, p3d_obj *object)
{
  #ifdef DEBUG
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
     #ifdef PQP
     case p3d_col_mode_pqp:
         pqp_robot_obj_collision_test(robot, object);
     break;
     #endif
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


//! A partir d'une matrice de transformation homogene, cette fonction extrait les parametres de translation
//! et les angles d'Euler associes a la sous-matrice de rotation.
//! La matrice de rotation est supposee avoir ete calculee a partir des angles d'Euler avec
//! la fonction p3d_mat4PosReverseOrder() de move3D i.e. R= transpose(Rz*Ry*Rx) avec
//! Rx, Ry et Rz les matrices de rotation selon x, y et z.
//! Le deuxieme angle de rotation retourne est choisi pour être entre -pi/2 et pi/2.
//! NOTE: cette fonction a due être ajoutee car la fonction de move3D censee faire la même chose
//! ne marche pas dans tous les cas.
void p3d_mat4ExtractPosReverseOrder2(p3d_matrix4 M,
				    double * tx, double * ty, double * tz,
				    double * ax, double * ay, double * az)
{
  double cy;
  double epsilon= 10e-6;

  (*ay)= asin(M[0][2]);
  cy = cos( (*ay) );
  if( (-epsilon < cy)  &&  (cy < epsilon) )
  {
    (*ax) = 0.0;
    (*az)= atan2( M[1][0], M[1][1] );
  }
  else
  {
    (*ax)= -atan2( M[1][2], M[2][2] );
    (*az)= -atan2( M[0][1], M[0][0] );

    if( (*ay)<0 && (*ay)<-M_PI_2 )
      (*ay)= -M_PI - (*ay);

    if( (*ay)>0 && (*ay)>M_PI_2 )
      (*ay)= M_PI - (*ay);
  }

  (*tx) = M[0][3];
  (*ty) = M[1][3];
  (*tz) = M[2][3];
}

//! Fonction pour passer d'une matrice 4x4 du format de la librairie "GB" a celui de move3D.
void Gb_th_matrix4(Gb_th *th, p3d_matrix4 mat)
{
  mat[0][0]= th->vx.x;   mat[0][1]= th->vy.x;   mat[0][2]= th->vz.x;   mat[0][3]= th->vp.x;
  mat[1][0]= th->vx.y;   mat[1][1]= th->vy.y;   mat[1][2]= th->vz.y;   mat[1][3]= th->vp.y;
  mat[2][0]= th->vx.z;   mat[2][1]= th->vy.z;   mat[2][2]= th->vz.z;   mat[2][3]= th->vp.z;
  mat[3][0]= 0;          mat[3][1]= 0;          mat[3][2]= 0;          mat[3][3]= 1;
}

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



//! Cette fonction retourne dans sint et cost les coordonnees
//! des points d'un cercle de rayon 1 discretise en n points.
//! sint et cost ont chacun |n+1| elements.
//! La memoire est reserve dans la fonction. Il faudra donc liberer les tableaux en dehors
//! de la fonction.
//! Le signe de n indique la direction de parcours des points du cercle.
int circle_table(double **sint, double **cost, const int n)
{
  #ifdef DEBUG
   if(sint==NULL || cost==NULL)
   {
     printf("%s: %d: circle_table(): entree(s) NULL (%p %p).\n", __FILE__, __LINE__,sint,cost);
     return 0;
   }
  #endif

    int i;
    /* Table size, the sign of n flips the circle direction */
    const int size = abs(n);
    /* Determine the angle between samples */
    const double angle = 2*M_PI/( (double) n);
    /* Allocate memory for n samples, plus duplicate of first entry at the end */
    *sint = (double *) malloc((size+1)*sizeof(double));
    *cost = (double *) malloc((size+1)*sizeof(double));
    /* Bail out if memory allocation fails*/
    if (!(*sint) || !(*cost))
    {
        free(*sint);
        free(*cost);
        printf("%s: %d: circle_table(): erreur d'allocation memoire.\n",__FILE__,__LINE__);
    }
    /* Compute cos and sin around the circle */
    for (i=0; i<size; i++)
    {
        (*sint)[i] = sin(angle*i);
        (*cost)[i] = cos(angle*i);
    }
    /* Last sample is duplicate of the first */
    (*sint)[size] = (*sint)[0];
    (*cost)[size] = (*cost)[0];

    return 1;
}

//! Now in g3d_draw.c
//! Cette fonction dessine un cône solide -dont les facettes sont
//! remplies- d'axe z et dont la pointe est en (0,0,0).
//! A utiliser dans une fonction d'affichage OpenGL.
/*
void draw_solid_cone(double radius, double height, int nbSegments)
{
   int i, j;
   double *sint, *cost, z, dz, dr;
   double alpha= atan(height/radius);
   double ca= cos(alpha);
   double sa= sin(alpha);
   circle_table(&sint, &cost, -nbSegments);
   z= height/2;
   int nbSegments2= nbSegments;

   dz= height/nbSegments2;
   dr= radius*dz/height;
   //Les triangles des côtes:
   glBegin(GL_TRIANGLE_STRIP);
     for(i=0; i<nbSegments2; i++)
     {
       for(j=nbSegments; j>=0; j--)
       {
         glNormal3d(cost[j]*sa, sint[j]*sa, ca);
         glTexCoord2d(1-j/(double)nbSegments, 1.0f);
         glVertex3d(cost[j]*i*dr, sint[j]*i*dr, z-i*dz);
         glNormal3d(cost[j]*sa, sint[j]*sa, ca);
         glTexCoord2d(1-j/(double)nbSegments, 0.0f);
         glVertex3d(cost[j]*(i+1)*dr, sint[j]*(i+1)*dr, z-(i+1)*dz);
       }
     }
   glEnd();

   //Les triangles du dessous:
   glBegin(GL_TRIANGLE_FAN);
    glNormal3d(0, 0, -1);
    glTexCoord2d(0, 0.0f);
    glVertex3d(0, 0, -z);
    for(i=0; i<=nbSegments; i++)
    { glTexCoord2d(1-i/(double)nbSegments, 1.0f);
      glVertex3d(cost[i]*radius, sint[i]*radius, -z);
    }
   glEnd();

   free(sint);
   free(cost);
}
*/

//! Draws a sphere with OpenGL functions. The sphere is centered on (0,0,0).
//! \param radius radius of the sphere
//! \param nbSegments number of segments of the discretization of the sphere silhouette
void gpDraw_solid_sphere(double radius, int nbSegments)
{
    int i, j;
    double r, r0;
    double x, y, z, z0;
    double *sint1, *cost1;
    double *sint2, *cost2;
    int n;
    if(nbSegments%2==0)
       n= nbSegments;
    else
       n= nbSegments+1;
    circle_table(&sint1, &cost1, -n);
    circle_table(&sint2, &cost2, n);
    for (i=1; i<=n/2; i++)
    {
        z0= cost2[i-1];
        r0= sint2[i-1];
        z = cost2[i];
        r = sint2[i];
        glBegin(GL_TRIANGLE_STRIP);
          for(j=0; j<=n; j++)
          {
            x= cost1[j];
            y= sint1[j];
            glNormal3d(x*r, y*r, z);
            glTexCoord2d( 1-j/((double) n), 2*i/((double) n) );
            glVertex3d(x*r*radius, y*r*radius, z*radius);
            glNormal3d(x*r0, y*r0, z0);
            glTexCoord2d( 1-j/((double) n), 2*(i-1)/((double) n) );
            glVertex3d(x*r0*radius, y*r0*radius, z0*radius);
          }
       glEnd();
    }
    free(cost1);
    free(sint1);
    free(cost2);
    free(sint2);
}

//! Draws a sphere with OpenGL functions.
//! \param x x coordinate of the sphere center
//! \param y y coordinate of the sphere center
//! \param z z coordinate of the sphere center
//! \param radius radius of the sphere
//! \param nbSegments number of segments of the discretization of the sphere silhouette
void gpDraw_solid_sphere(double x_, double y_, double z_, double radius, int nbSegments)
{
    int i, j;
    double r, r0;
    double x, y, z, z0;
    double *sint1, *cost1;
    double *sint2, *cost2;
    int n;
    if(nbSegments%2==0)
       n= nbSegments;
    else
       n= nbSegments+1;
    circle_table(&sint1, &cost1, -n);
    circle_table(&sint2, &cost2, n);
    for (i=1; i<=n/2; i++)
    {
        z0= cost2[i-1];
        r0= sint2[i-1];
        z = cost2[i];
        r = sint2[i];
        glBegin(GL_TRIANGLE_STRIP);
          for(j=0; j<=n; j++)
          {
            x= cost1[j];
            y= sint1[j];
            glNormal3d(x*r, y*r, z);
            glTexCoord2d( 1-j/((double) n), 2*i/((double) n) );
            glVertex3d(x_ + x*r*radius, y_ + y*r*radius, z_ + z*radius);
            glNormal3d(x*r0, y*r0, z0);
            glTexCoord2d( 1-j/((double) n), 2*(i-1)/((double) n) );
            glVertex3d(x_ + x*r0*radius, y_ + y*r0*radius, z_ + z0*radius);
          }
       glEnd();
    }
    free(cost1);
    free(sint1);
    free(cost2);
    free(sint2);
}

//! Draws a cylinder, centered on (0,0,0) and aligned along z-axis.
//! \param radius cylinder radius
//! \param length cylinder length
//! \param nbSegments number of segments of the discretization of the cylinder's section
void gpDraw_solid_cylinder(double radius, double length, int nbSegments)
{
  int i;
  float alpha, dalpha, *cost, *sint;
  cost= sint= NULL;

  if(nbSegments < 3)
  { nbSegments= 3; }
  if(nbSegments > 100)
  { nbSegments= 100; }

  cost= new float[nbSegments+1];
  sint= new float[nbSegments+1];

  dalpha= 2*M_PI/((float) nbSegments);
  for(i=0; i<=nbSegments; i++)
  {
    alpha= i*dalpha;
    cost[i]= cos(alpha);
    sint[i]= sin(alpha);
  }

  glBegin(GL_TRIANGLE_FAN);
   glNormal3f(0.0, 0.0, 1.0);
   glVertex3f(0.0, 0.0, 0.5*length);
   for(i=0; i<=nbSegments; i++)
   {  glVertex3f(radius*cost[i], radius*sint[i], 0.5*length);   }
  glEnd();

  glBegin(GL_TRIANGLE_FAN);
   glNormal3f(0.0, 0.0, -1.0);
   glVertex3f(0.0, 0.0, 0.5*length);
   for(i=nbSegments; i>=0; i--)
   {  glVertex3f(radius*cost[i], radius*sint[i], -0.5*length);   }
  glEnd();


  glBegin(GL_TRIANGLE_STRIP);
   for(i=0; i<=nbSegments; i++)
   {
     glNormal3f(cost[i], sint[i], 0.0);
     glVertex3f(radius*cost[i], radius*sint[i],  0.5*length);
     glVertex3f(radius*cost[i], radius*sint[i], -0.5*length);
   }
  glEnd();


  delete [] cost;
  delete [] sint;
}



//! Draws a cylinder from its two face centers.
//! \param p1 center of the first face (disc) of the cylinder
//! \param p2 center of the second face (disc) of the cylinder
//! \param radius cylinder's radius
//! \param nbSegments number of segments of the cylinder section
//! \return 1 in case of success, 0 otherwise
int gpDraw_cylinder(p3d_vector3 p1, p3d_vector3 p2, double radius, unsigned int nbSegments)
{
  unsigned int i, j;
  double alpha, dalpha, norm;
  p3d_vector3 d, u, v, c1, c2, c3, c4, normal;

  p3d_vectSub(p2, p1, d);

  norm= p3d_vectNorm(d);

  if(norm< 1e-6)
  {
    return 0;
  }  

  d[0]/= norm;
  d[1]/= norm;
  d[2]/= norm;

  //find a vector orthogonal to d:
  if( fabs(d[2]) <= 1e-6 )
  {
    u[0]= 0;
    u[1]= 0;
    u[2]= 1;
  }
  else
  {
     u[0]= 0;
     u[1]= 1;
     u[2]= -d[1]/d[2];
     p3d_vectNormalize(u, u);
  }
 
  // (u,v) is a basis for the plane orthogonal to the cylinder axis:
  p3d_vectXprod(d, u, v);

  dalpha= 2*M_PI/((float) nbSegments);

  alpha= 0;
  glBegin(GL_TRIANGLE_FAN);
  glNormal3d(d[0], d[1], d[2]);
  glVertex3d(p2[0], p2[1], p2[2]);
  for(i=0; i<nbSegments; i++)
  {
    for(j=0; j<3; j++)
    {
      c1[j]= p2[j] + radius*cos(alpha)*u[j] + radius*sin(alpha)*v[j];
      c2[j]= p2[j] + radius*cos(alpha+dalpha)*u[j] + radius*sin(alpha+dalpha)*v[j];
    }
    glVertex3f(c1[0], c1[1], c1[2]);
    glVertex3f(c2[0], c2[1], c2[2]);
    alpha+= dalpha; 
  }
  glEnd();

  alpha= 0;
  glBegin(GL_TRIANGLE_FAN);
  glNormal3f(-d[0], -d[1], -d[2]);
  glVertex3f(p1[0], p1[1], p1[2]);
  for(i=0; i<nbSegments; i++)
  {
    for(j=0; j<3; j++)
    {
      c1[j]= p1[j] + radius*cos(alpha)*v[j] + radius*sin(alpha)*u[j];
      c2[j]= p1[j] + radius*cos(alpha+dalpha)*v[j] + radius*sin(alpha+dalpha)*u[j];
    }
    glVertex3f(c1[0], c1[1], c1[2]);
    glVertex3f(c2[0], c2[1], c2[2]);
    alpha+= dalpha; 
  }
  glEnd();

  alpha= 0;

  glBegin(GL_QUADS);
  for(i=0; i<nbSegments; i++)
  {
    for(j=0; j<3; j++)
    {
      c1[j]= p1[j] + radius*cos(alpha)*v[j] + radius*sin(alpha)*u[j];
      c2[j]= p2[j] + radius*cos(alpha)*v[j] + radius*sin(alpha)*u[j];
      c3[j]= p2[j] + radius*cos(alpha+dalpha)*v[j] + radius*sin(alpha+dalpha)*u[j];
      c4[j]= p1[j] + radius*cos(alpha+dalpha)*v[j] + radius*sin(alpha+dalpha)*u[j];
      normal[j]= cos(alpha)*v[j] + sin(alpha)*u[j];
    }
    glNormal3f(normal[0], normal[1], normal[2]);
    glVertex3f(c1[0], c1[1], c1[2]);
    glVertex3f(c2[0], c2[1], c2[2]);
    glVertex3f(c3[0], c3[1], c3[2]);
    glVertex3f(c4[0], c4[1], c4[2]);
    alpha+= dalpha; 
  }
  glEnd();

  return 1;
}

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


//! Now in g3d_draw.c
//! Fonction dessinant une fleche partant de p1 et se terminant en p2, de couleur
//! dont les composantes RGB sont donnees en parametre.
//! A utiliser dans une fonction d'affichage OpenGL.
/*
void draw_arrow(p3d_vector3 p1, p3d_vector3 p2, double red, double green, double blue)
{
   double length, cone_height;
   p3d_vector3 p;
   p[0]= p2[0] - p1[0];
   p[1]= p2[1] - p1[1];
   p[2]= p2[2] - p1[2];
   length= sqrt( p[0]*p[0] + p[1]*p[1] + p[2]*p[2] );
   p[0]/= length;
   p[1]/= length;
   p[2]/= length;

   cone_height= 0.1* length;
   glLineWidth(5);
   glDisable(GL_LIGHTING);
   glColor3d(red, green, blue);
   glBegin(GL_LINES);
      glVertex3d(p1[0], p1[1], p1[2]);
      glVertex3d(p2[0]-cone_height*p[0], p2[1]-cone_height*p[1], p2[2]-cone_height*p[2]);
   glEnd();
   glEnable(GL_LIGHTING);

   double color[]= {red, green, blue};
   g3d_set_color_mat(Any, color);

   glPushMatrix();
     glTranslatef(p2[0]-0.05*length*p[0], p2[1]-0.05*length*p[1], p2[2]-0.05*length*p[2]);
     if( sqrt(p[0]*p[0]+p[1]*p[1]) > 1e-9 )
     {  glRotatef(RADTODEG*asin(p[2]) - 90, p[1], -p[0], 0);  }
     else
     {
        if( p[2] < 0 )
         glRotatef(180, 1, 0, 0);
     }
     draw_solid_cone(0.3*cone_height, cone_height, 10);
   glPopMatrix();
}
*/

//! Fonction d'affichage d'un repere (matrice 4x4).
//! Les axes sont dessines sur une longueur "length".
//! A utiliser dans une fonction d'affichage OpenGL.
void draw_frame_jp(p3d_matrix4 frame, double length)
{
   p3d_vector3 origin, xAxis, yAxis, zAxis;

   origin[0]= frame[0][3];
   origin[1]= frame[1][3];
   origin[2]= frame[2][3];

   xAxis[0]=  origin[0] + length*frame[0][0];
   xAxis[1]=  origin[1] + length*frame[1][0];
   xAxis[2]=  origin[2] + length*frame[2][0];

   yAxis[0]=  origin[0] + length*frame[0][1];
   yAxis[1]=  origin[1] + length*frame[1][1];
   yAxis[2]=  origin[2] + length*frame[2][1];

   zAxis[0]=  origin[0] + length*frame[0][2];
   zAxis[1]=  origin[1] + length*frame[1][2];
   zAxis[2]=  origin[2] + length*frame[2][2];

   draw_arrow(origin, xAxis, 1.0, 0.0, 0.0);

   draw_arrow(origin, yAxis, 0.0, 1.0, 0.0);

   draw_arrow(origin, zAxis, 0.0, 0.0, 1.0);
}

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
 // p3d_matvec4Mult( *(win->cam_frame), win->up, up);
/*
  Transf[0][3] = win->zo * (cos(win->az)*cos(win->el));
  Transf[1][3] = win->zo * (sin(win->az)*cos(win->el));
  Transf[2][3] = win->zo * sin(win->el);
*/
  p3d_vector3 cam_pos, right;
  cam_pos[0] = win->zo * (cos(win->az)*cos(win->el));
  cam_pos[1] = win->zo * (sin(win->az)*cos(win->el));
  cam_pos[2] = win->zo * sin(win->el);

  right[0] = (*win->cam_frame)[0][0];
  right[1] = (*win->cam_frame)[1][0];
  right[2] = (*win->cam_frame)[2][0];
  p3d_vectNormalize(right, right);

  fprintf(file, "camera {\n");
  fprintf(file, "\t /*location  <1.0, -4, 3.5>*/\n");
  fprintf(file, "\t location  <%f, %f, %f>\n", cam_pos[0], cam_pos[1], cam_pos[2]);
  fprintf(file, "\t /*direction 1.5*y*/\n");
 // fprintf(file, "\t direction  <%f,%f,%f>\n",win->x-cam_pos[0],win->y-cam_pos[1],win->z-cam_pos[2]);
  fprintf(file, "\t /*right  -x*image_width/image_height*/\n");
  fprintf(file, "\t right  <%f, %f, %f>\n", -right[0], -right[1], -right[2] );
  fprintf(file, "\t sky  <%f, %f, %f>\n", win->up[0], win->up[1], win->up[2] );
  fprintf(file, "\t angle  45\n");
  fprintf(file, "\t look_at  <%f %f %f>\n", win->x, win->y, win->z);
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
  static p3d_vector2 L[4];
  L[0][0]=  0.0;   L[1][0]= 0.5;   L[2][0]=  0.5;   L[3][0]=  0.0;
  L[0][1]=  0.0;   L[1][1]= 0.5;   L[2][1]=  0.0;   L[3][1]=  0.5;

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
  static p3d_vector3 L[8];
  L[0][0]= 0.0;   L[1][0]=  0.5;   L[2][0]=  0.0;   L[3][0]=  0.5;
  L[0][1]= 0.0;   L[1][1]=  0.0;   L[2][1]=  0.0;   L[3][1]=  0.0;
  L[0][2]= 0.0;   L[1][2]=  0.0;   L[2][2]=  0.5;   L[3][2]=  0.5;

  L[4][0]=  0.0;   L[5][0]=  0.5;   L[6][0]=  0.0;   L[7][0]=  0.5;
  L[4][1]=  0.5;   L[5][1]=  0.5;   L[6][1]=  0.5;   L[7][1]=  0.5;
  L[4][2]=  0.0;   L[5][2]=  0.0;   L[6][2]=  0.5;   L[7][2]=  0.5;

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

//! Writes the content of the p3d_matrix4 in a float array with the format used by OpenGL (when calling a function
//! like glLoadMatrix or glMultMatrix).
void p3d_matrix4_to_OpenGL_format(p3d_matrix4 source, GLfloat mat[16])
{
  mat[0]= source[0][0];    mat[4]= source[0][1];    mat[8]=  source[0][2];    mat[12]= source[0][3];
  mat[1]= source[1][0];    mat[5]= source[1][1];    mat[9]=  source[1][2];    mat[13]= source[1][3];
  mat[2]= source[2][0];    mat[6]= source[2][1];    mat[10]= source[2][2];    mat[14]= source[2][3];
  mat[3]=            0;    mat[7]=            0;    mat[11]=            0;    mat[15]=            1;
}


int gpExport_for_coldman(p3d_rob *robot)
{
  unsigned int it, k, nb_triangles;
  int i, j, shift;
  p3d_index *indices= NULL;
  p3d_vector3 p1, p2;
  p3d_matrix4 T, T2, Tinv;
  p3d_obj *body;
  char str[128];
  FILE *file= NULL;
  #ifdef PQP
  pqp_triangle *triangles= NULL;
  #endif 

  for(i=0; i<robot->no; i++)
  {
    body= robot->o[i];
//     if(body->BodyWrtPilotingJoint==NULL)
//     {  continue;  }
    sprintf(str, "./graspPlanning/export/%s.obj", body->name);
    file= fopen(str, "w");
    if(file==NULL)
    { 
       printf("%s: %d: gpExport_for_coldman(): can not open %s.\n", __FILE__,__LINE__,str);
       return 0;
    }
    fprintf(file, "# %s\n", body->name);

    for(j=0; j<body->np; j++)
    {
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

    shift= 0;
    for(j=0; j<body->np; j++)
    {
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
          printf("%s: %d: gpExport_for_coldman(): some functions in p3d_pqp are needed to deal with non triangular faces.\n", __FILE__,__LINE__);
          #else
          triangles= pqp_triangulate_face(body->pol[j]->poly, k, &nb_triangles);
          for(it=0; it<nb_triangles; it++)
          {
            fprintf(file, "f %d %d %d\n", triangles[it][0]+1+shift, triangles[it][1]+1+shift, triangles[it][2]+1+shift);
          }
          free(triangles);
          #endif
        }
      }
      shift+= body->pol[j]->poly->nb_points;
    }

    fclose(file);
    file= NULL;
  }

  return 1;
}


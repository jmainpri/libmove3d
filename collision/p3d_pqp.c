#ifdef PQP
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"


#include <../collision/PQP/include/PQP.h>
#include <../collision/PQP/include/p3d_pqp.h>


// Define to enable/disable some debug checkings
#ifndef PQP_DEBUG
#define PQP_DEBUG
#endif

//! Boolean to display a message if a collision is detected in any of the collision test function:
static unsigned int pqp_COLLISION_MESSAGE= 0;

static pqp_collision_grid pqp_COLLISION_PAIRS= { 0, 0, 0, NULL, NULL, NULL, NULL}; 


//! Enables/disables the display of messages about collisions.
//! \param set messages about collision will be displayed if set > 0, they will not if set = 0
void pqp_set_collision_message(unsigned int set)
{
  if(set > 0)
  {
    pqp_COLLISION_MESSAGE= 1;
  }
  else
  {
    pqp_COLLISION_MESSAGE= 0;
  }

}


//! Gets the pose of an object.
//! \param obj pointer to the object
//! \param pose a 4x4 matrix that will be filled with the current object pose
//! \return 1 in case of success, 0 otherwise 
int pqp_get_obj_pos(p3d_obj *obj, p3d_matrix4 pose)
{
   #ifdef PQP_DEBUG
    if(obj==NULL)
    {
      printf("%s: %d: pqp_get_obj_pos(): obj is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(obj->pol[0]==NULL)
    {
      printf("%s: %d: pqp_get_obj_pos(): obj->pol[0] is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(obj->pol[0]->poly==NULL)
    {
      printf("%s: %d: pqp_get_obj_pos(): obj->pol[0]->poly is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
   #endif

   if(obj->is_used_in_device_flag)
   {
     p3d_get_poly_pos(obj->pol[0]->poly, pose);
   }
   else
   {
      p3d_mat4Copy(p3d_mat4IDENTITY, pose);
   } 

  return 1;
}

//! Converts a p3d pose matrix to an OpenGL one.
//! \param T the input p3d_matrix4
//! \param mat a float array that will be filled 
void pqp_p3d_to_gl_matrix(p3d_matrix4 T, GLfloat mat[16])
{
   mat[0]= T[0][0];      mat[4]= T[0][1];      mat[8]=  T[0][2];      mat[12]= T[0][3];
   mat[1]= T[1][0];      mat[5]= T[1][1];      mat[9]=  T[1][2];      mat[13]= T[1][3];
   mat[2]= T[2][0];      mat[6]= T[2][1];      mat[10]= T[2][2];      mat[14]= T[2][3];
   mat[3]=       0;      mat[7]=       0;      mat[11]=       0;      mat[15]=       1;
}


//! Finds (if it exists) the previous body of the body given in argument, in the kinematic chain
//! it belongs to. This is used to skip the collision tests between two bodies that are linked
//! by a joint.
p3d_obj *pqp_get_previous_body(p3d_obj *body)
{
   #ifdef PQP_DEBUG
    if(body==NULL)
    {
      printf("%s: %d: pqp_get_previous_body(): body is NULL.\n",__FILE__,__LINE__);
      return NULL;
    }
   #endif

    p3d_jnt *joint= NULL;

    if(body->jnt==NULL)
        return NULL;

    if(body->jnt->prev_jnt==NULL)
        return NULL;

    joint= body->jnt->prev_jnt;

    while (joint->o==NULL)
    {
        if(joint->prev_jnt==NULL)
            return NULL;
        joint= joint->prev_jnt;
    }

    return joint->o;
}


//! This function must be called to create all the structures used by detection collision
//! when the activated collision checker is PQP.
//! A variable of type (class) PQP_Model is associated to each p3d_obj of the environment
//! that means obstacles as well as robot bodies.
//! PQP only works with triangles but the function triangulates all the faces
//! that need to be.
void p3d_start_pqp()
{
    static int firstTime= 1;

    if(firstTime==1)
    {
      firstTime= 0;
    }
    else
    {
      printf("%s: %d: The function p3d_start_pqp() must be called only one time.\n",__FILE__,__LINE__);
      return;
    }

    //set the global variable COLLISION_BY_OBJECT to true to indicate that collision tests
    //are performed between pairs of objects (p3d_obj) and not pairs of polyhedra (p3d_poly):
    set_collision_by_object(TRUE);

    //crucial setting for local path collision test:
    p3d_BB_set_selection_method(DEACTIVATE_BB);

    unsigned int i, j, k, ir, it;
    unsigned int nb_face_triangles, nb_triangles;
    unsigned int nb_pqpModels= 1;
    int has_degenerate_faces= 0;
    PQP_REAL a[3], b[3], c[3];
    PQP_REAL at[3], bt[3], ct[3];
    p3d_matrix4 T, Tinv;
    pqp_triangle *triangles= NULL;
    p3d_polyhedre *polyh= NULL;
    p3d_obj *object= NULL;
    p3d_rob *robot= NULL;

    //First, the obstacles:
    for (i=0; i<(unsigned int) XYZ_ENV->no; i++)
    {
        object= XYZ_ENV->o[i];
        object->pqpModel= NULL;

        //Test if the object has non graphic polyhedra:
    	if(pqp_is_pure_graphic(object))
    	{
          continue;
    	}

        object->pqpModel= new PQP_Model;
        nb_triangles= 0;
        object->pqpModel->BeginModel();
        for (j=0; j<(unsigned int) object->np; j++)
        {
            if(object->pol[j]->TYPE==P3D_GRAPHIC || object->pol[j]->p3d_objPt!=object)
            {  continue;  }

            polyh= object->pol[j]->poly;

            p3d_mat4Copy(object->pol[j]->pos0, T);
            p3d_matInvertXform(T, Tinv);

            has_degenerate_faces= 0;
            for (k=0; k<polyh->nb_faces; k++)
            {
                if( pqp_is_face_degenerate(polyh, k) )
                {
                  has_degenerate_faces= 1;
                  continue;
                }  

                if(polyh->the_faces[k].nb_points==3) //triangular face
                {
                    a[0]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[0] - 1 ][0];
                    a[1]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[0] - 1 ][1];
                    a[2]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[0] - 1 ][2];

                    b[0]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[1] - 1 ][0];
                    b[1]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[1] - 1 ][1];
                    b[2]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[1] - 1 ][2];

                    c[0]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[2] - 1 ][0];
                    c[1]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[2] - 1 ][1];
                    c[2]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[2] - 1 ][2];

                    if(object->pol[j]->entity_type!=POLYHEDRON_ENTITY)
                    {             
                      at[0]= T[0][0]*a[0] + T[0][1]*a[1]  + T[0][2]*a[2]  + T[0][3];
                      at[1]= T[1][0]*a[0] + T[1][1]*a[1]  + T[1][2]*a[2]  + T[1][3];
                      at[2]= T[2][0]*a[0] + T[2][1]*a[1]  + T[2][2]*a[2]  + T[2][3];

                      bt[0]= T[0][0]*b[0] + T[0][1]*b[1]  + T[0][2]*b[2]  + T[0][3];
                      bt[1]= T[1][0]*b[0] + T[1][1]*b[1]  + T[1][2]*b[2]  + T[1][3];
                      bt[2]= T[2][0]*b[0] + T[2][1]*b[1]  + T[2][2]*b[2]  + T[2][3];

                      ct[0]= T[0][0]*c[0] + T[0][1]*c[1]  + T[0][2]*c[2]  + T[0][3];
                      ct[1]= T[1][0]*c[0] + T[1][1]*c[1]  + T[1][2]*c[2]  + T[1][3];
                      ct[2]= T[2][0]*c[0] + T[2][1]*c[1]  + T[2][2]*c[2]  + T[2][3];

                      object->pqpModel->AddTri(at, bt, ct, nb_triangles++);
                    }
                    else
                    {   object->pqpModel->AddTri(a, b, c, nb_triangles++);  }
                }
                else //non triangular face -> needs to be triangulated first
                { 
                    triangles=  pqp_triangulate_face(polyh, k, &nb_face_triangles);
                    for (it=0; it<(unsigned int) nb_face_triangles; it++)
                    {
                        a[0]= polyh->the_points[ triangles[it][0] ][0];
                        a[1]= polyh->the_points[ triangles[it][0] ][1];
                        a[2]= polyh->the_points[ triangles[it][0] ][2];

                        b[0]= polyh->the_points[ triangles[it][1] ][0];
                        b[1]= polyh->the_points[ triangles[it][1] ][1];
                        b[2]= polyh->the_points[ triangles[it][1] ][2];

                        c[0]= polyh->the_points[ triangles[it][2] ][0];
                        c[1]= polyh->the_points[ triangles[it][2] ][1];
                        c[2]= polyh->the_points[ triangles[it][2] ][2];

                        if(object->pol[j]->entity_type!=POLYHEDRON_ENTITY)
                        {              
                          at[0]= T[0][0]*a[0] + T[0][1]*a[1]  + T[0][2]*a[2]  + T[0][3];
                          at[1]= T[1][0]*a[0] + T[1][1]*a[1]  + T[1][2]*a[2]  + T[1][3];
                          at[2]= T[2][0]*a[0] + T[2][1]*a[1]  + T[2][2]*a[2]  + T[2][3];

                          bt[0]= T[0][0]*b[0] + T[0][1]*b[1]  + T[0][2]*b[2]  + T[0][3];
                          bt[1]= T[1][0]*b[0] + T[1][1]*b[1]  + T[1][2]*b[2]  + T[1][3];
                          bt[2]= T[2][0]*b[0] + T[2][1]*b[1]  + T[2][2]*b[2]  + T[2][3];
  
                          ct[0]= T[0][0]*c[0] + T[0][1]*c[1]  + T[0][2]*c[2]  + T[0][3];
                          ct[1]= T[1][0]*c[0] + T[1][1]*c[1]  + T[1][2]*c[2]  + T[1][3];
                          ct[2]= T[2][0]*c[0] + T[2][1]*c[1]  + T[2][2]*c[2]  + T[2][3];

                          object->pqpModel->AddTri(at, bt, ct, nb_triangles++);
                        }
                        else
                        {  object->pqpModel->AddTri(a, b, c, nb_triangles++);  }
                    }
                    free(triangles);
                    triangles= NULL;
                }

            }
            if(has_degenerate_faces==1)
            {
              printf("%s: %d: p3d_start_pqp(): some faces of %s are degenerate.\n",__FILE__,__LINE__,polyh->name);
            }
        }
        object->pqpModel->EndModel();
        object->pqpID= nb_pqpModels++;
    }


    //Now, the robots:
    for (ir=0; ir<(unsigned int) XYZ_ENV->nr; ir++)
    {
        robot= XYZ_ENV->robot[ir];
        for (i=0; i<(unsigned int) robot->no; i++)
        {
            object= robot->o[i];
            object->pqpModel= NULL;

            //Test if the object has non graphic polyhedra:
            if(pqp_is_pure_graphic(object))
            {
              continue;
            }

            object->pqpPreviousBody= pqp_get_previous_body(object);

            nb_triangles= 0;
            object->pqpModel= new PQP_Model;
            object->pqpModel->BeginModel();
            for (j=0; j<(unsigned int) object->np; j++)
            { 
                if(object->pol[j]->TYPE==P3D_GRAPHIC || object->pol[j]->p3d_objPt!=object)
                {  continue;  }

                polyh= object->pol[j]->poly;
                p3d_mat4Copy(object->pol[j]->pos0, T);

                p3d_matInvertXform(T, Tinv);

                for (k=0; k<(unsigned int) polyh->nb_faces; k++)
                {
                    if( pqp_is_face_degenerate(polyh, k) )
                    {
                      has_degenerate_faces= 1;
                      continue;
                    }  
 
                    if(polyh->the_faces[k].nb_points==3) //triangular face
                    {
                        a[0]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[0] - 1 ][0];
                        a[1]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[0] - 1 ][1];
                        a[2]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[0] - 1 ][2];

                        b[0]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[1] - 1 ][0];
                        b[1]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[1] - 1 ][1];
                        b[2]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[1] - 1 ][2];

                        c[0]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[2] - 1 ][0];
                        c[1]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[2] - 1 ][1];
                        c[2]= polyh->the_points[ polyh->the_faces[k].the_indexs_points[2] - 1 ][2];

                        if(object->pol[j]->entity_type!=POLYHEDRON_ENTITY)
                        {             
                          at[0]= T[0][0]*a[0] + T[0][1]*a[1]  + T[0][2]*a[2]  + T[0][3];
                          at[1]= T[1][0]*a[0] + T[1][1]*a[1]  + T[1][2]*a[2]  + T[1][3];
                          at[2]= T[2][0]*a[0] + T[2][1]*a[1]  + T[2][2]*a[2]  + T[2][3];

                          bt[0]= T[0][0]*b[0] + T[0][1]*b[1]  + T[0][2]*b[2]  + T[0][3];
                          bt[1]= T[1][0]*b[0] + T[1][1]*b[1]  + T[1][2]*b[2]  + T[1][3];
                          bt[2]= T[2][0]*b[0] + T[2][1]*b[1]  + T[2][2]*b[2]  + T[2][3];

                          ct[0]= T[0][0]*c[0] + T[0][1]*c[1]  + T[0][2]*c[2]  + T[0][3];
                          ct[1]= T[1][0]*c[0] + T[1][1]*c[1]  + T[1][2]*c[2]  + T[1][3];
                          ct[2]= T[2][0]*c[0] + T[2][1]*c[1]  + T[2][2]*c[2]  + T[2][3];

                          object->pqpModel->AddTri(at, bt, ct, nb_triangles++);
                        }
                        else
                        { object->pqpModel->AddTri(a, b, c, nb_triangles++); }

                    }
                    else //non triangular face -> needs to be triangulated first
                    {
                        triangles=  pqp_triangulate_face(polyh, k, &nb_face_triangles);
                        for (it=0; it<(unsigned int) nb_face_triangles; it++)
                        {
                            a[0]= polyh->the_points[ triangles[it][0] ][0];
                            a[1]= polyh->the_points[ triangles[it][0] ][1];
                            a[2]= polyh->the_points[ triangles[it][0] ][2];

                            b[0]= polyh->the_points[ triangles[it][1] ][0];
                            b[1]= polyh->the_points[ triangles[it][1] ][1];
                            b[2]= polyh->the_points[ triangles[it][1] ][2];

                            c[0]= polyh->the_points[ triangles[it][2] ][0];
                            c[1]= polyh->the_points[ triangles[it][2] ][1];
                            c[2]= polyh->the_points[ triangles[it][2] ][2];

                            if(object->pol[j]->entity_type!=POLYHEDRON_ENTITY)
                            {             
                               at[0]= T[0][0]*a[0] + T[0][1]*a[1]  + T[0][2]*a[2]  + T[0][3];
                               at[1]= T[1][0]*a[0] + T[1][1]*a[1]  + T[1][2]*a[2]  + T[1][3];
                               at[2]= T[2][0]*a[0] + T[2][1]*a[1]  + T[2][2]*a[2]  + T[2][3];

                               bt[0]= T[0][0]*b[0] + T[0][1]*b[1]  + T[0][2]*b[2]  + T[0][3];
                               bt[1]= T[1][0]*b[0] + T[1][1]*b[1]  + T[1][2]*b[2]  + T[1][3];
                               bt[2]= T[2][0]*b[0] + T[2][1]*b[1]  + T[2][2]*b[2]  + T[2][3];

                               ct[0]= T[0][0]*c[0] + T[0][1]*c[1]  + T[0][2]*c[2]  + T[0][3];
                               ct[1]= T[1][0]*c[0] + T[1][1]*c[1]  + T[1][2]*c[2]  + T[1][3];
                               ct[2]= T[2][0]*c[0] + T[2][1]*c[1]  + T[2][2]*c[2]  + T[2][3];

                               object->pqpModel->AddTri(at, bt, ct, nb_triangles++);
                            }
                            else
                            {   object->pqpModel->AddTri(a, b, c, nb_triangles++);  }
                        }
                        free(triangles);
                        triangles= NULL;
                    }

                }
                if(has_degenerate_faces==1)
                {
                  printf("%s: %d: p3d_start_pqp(): some faces of %s are degenerate.\n",__FILE__,__LINE__,polyh->name);
                }
            }

            object->pqpModel->EndModel();
            object->pqpID= nb_pqpModels++;
        }
    }


    //Create and activate all obstacle-robot and robot-robot pairs:
    pqp_create_collision_pairs();

//     pqp_print_collision_pairs();

}

//! Checks if the number of bodies used in the pqp_collision_grid is valid.
//! \return 1 if the number of bodies used in the pqp_collision_grid is valid, 0 otherwise
int pqp_check_collision_pair_validity()
{
  unsigned int count_obj= 0;
  int i;

  count_obj+= XYZ_ENV->no;
  for(i=0; i<XYZ_ENV->nr; i++)
  {
    count_obj+= XYZ_ENV->robot[i]->no;
  }
  
  if(count_obj!=pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_check_collision_pair_validity(): the number of p3d_obj has been changed since the last call to p3d_start_pqp().\n",__FILE__,__LINE__);
     return 0;
  }

  return 1;
}

//! Allocates the array used to know which collisions have to be tested.
//! All the collision tests are enabled, except for the ones between two bodies linked
//! by a joint, that are all disabled.
void pqp_create_collision_pairs()
{
  unsigned int i, j, count, nb_rob, nb_obst;
  p3d_obj *obj1, *obj2;

  nb_rob = XYZ_ENV->nr;
  nb_obst= XYZ_ENV->no;

  pqp_COLLISION_PAIRS.nb_robots   = nb_rob;
  pqp_COLLISION_PAIRS.nb_obstacles= nb_obst;
  pqp_COLLISION_PAIRS.nb_objs= 0;
  

  // first, count all the environment obstacles and robot bodies:
  for(i=0; i<nb_obst; i++)
  {

    //Test if the object has non graphic polyhedra:
    if(pqp_is_pure_graphic(XYZ_ENV->o[i]))
    {
      continue;
    }

    XYZ_ENV->o[i]->pqpID= pqp_COLLISION_PAIRS.nb_objs;
    pqp_COLLISION_PAIRS.nb_objs++;
  }

  for(i=0; i<nb_rob; i++)
  {
    for(j=0; j<(unsigned int) XYZ_ENV->robot[i]->no; j++)
    { 
      //Test if the object has non graphic polyhedra:
      if(pqp_is_pure_graphic(XYZ_ENV->robot[i]->o[j]))
      {
        continue;
      }

      //printf("name= %s\n", XYZ_ENV->robot[i]->o[j]->name);
      XYZ_ENV->robot[i]->o[j]->pqpID= pqp_COLLISION_PAIRS.nb_objs;
      pqp_COLLISION_PAIRS.nb_objs++;
    }    
  }


  if(pqp_COLLISION_PAIRS.obj_obj!=NULL) 
  { 
    free(pqp_COLLISION_PAIRS.obj_obj);
    pqp_COLLISION_PAIRS.obj_obj= NULL;
  }
  pqp_COLLISION_PAIRS.obj_obj= (unsigned int **) malloc(pqp_COLLISION_PAIRS.nb_objs*sizeof(unsigned int *));
  for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
  {
    pqp_COLLISION_PAIRS.obj_obj[i]= NULL;
    pqp_COLLISION_PAIRS.obj_obj[i]= (unsigned int *) malloc(pqp_COLLISION_PAIRS.nb_objs*sizeof(unsigned int));
  }


  if(pqp_COLLISION_PAIRS.obj_from_pqpID!=NULL) 
  { 
    free(pqp_COLLISION_PAIRS.obj_from_pqpID);
    pqp_COLLISION_PAIRS.obj_from_pqpID= NULL;
  }
  pqp_COLLISION_PAIRS.obj_from_pqpID= (p3d_obj **) malloc(pqp_COLLISION_PAIRS.nb_objs*sizeof(p3d_obj *));


  // fill the p3d_obj* from pqpID array:
  count= 0;
  for(i=0; i<nb_obst; i++)
  {
    //Test if the object has non graphic polyhedra:
    if(pqp_is_pure_graphic(XYZ_ENV->o[i]))
    {
      continue;
    }

    pqp_COLLISION_PAIRS.obj_from_pqpID[count]= XYZ_ENV->o[i];
    count++;
  }

  for(i=0; i<nb_rob; i++)
  {
    for(j=0; j<(unsigned int) XYZ_ENV->robot[i]->no; j++)
    { 
      //Test if the object has non graphic polyhedra:
      if(pqp_is_pure_graphic(XYZ_ENV->robot[i]->o[j]))
      {
        continue;
      }

      pqp_COLLISION_PAIRS.obj_from_pqpID[count]= XYZ_ENV->robot[i]->o[j];
      count++;
    }    
  }


  //fill the collision pair array:
  for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
  {
    pqp_COLLISION_PAIRS.obj_obj[i][i]= 0;

    for(j=i+1; j<pqp_COLLISION_PAIRS.nb_objs; j++)
    { 
      obj1= pqp_COLLISION_PAIRS.obj_from_pqpID[i];
      obj2= pqp_COLLISION_PAIRS.obj_from_pqpID[j];
      

      if( obj1->pqpPreviousBody==obj2 || obj2->pqpPreviousBody==obj1 )
      {
         pqp_COLLISION_PAIRS.obj_obj[i][j]= 0;
         pqp_COLLISION_PAIRS.obj_obj[j][i]= 0;
      }
      else
      {
         pqp_COLLISION_PAIRS.obj_obj[i][j]= 1;
         pqp_COLLISION_PAIRS.obj_obj[j][i]= 1;
      }
    }
  }

  pqp_COLLISION_PAIRS.colliding_body1= NULL;
  pqp_COLLISION_PAIRS.colliding_body2= NULL;
}

//! Test if the object has non graphic polyhedra:
//! \return 1 in case of success, 0 otherwise
int pqp_is_pure_graphic(p3d_obj* obj)
{
    for (int k=0; k<obj->np;k++)
    {
      if(obj->pol[k]->TYPE!=P3D_GRAPHIC)
      {
        return 0;
      }
    }
    return 1;
}

//! Displays, for each pair of bodies, if the collision between the two bodies will be tested.
//! \return 1 in case of success, 0 otherwise
int pqp_print_collision_pairs()
{
  unsigned int i, j, nb_rob, nb_obst;
  p3d_obj *body1, *body2;
  nb_rob = XYZ_ENV->nr;
  nb_obst= XYZ_ENV->no;


  printf("Active collision pairs are:\n");
  for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
  {
    for(j=0; j<i; j++)
    {
      body1= pqp_COLLISION_PAIRS.obj_from_pqpID[i];
      body2= pqp_COLLISION_PAIRS.obj_from_pqpID[j];

      if(body1==NULL || body2==NULL)
      {
        printf("%s: %d: pqp_print_collision_pairs(): the \"obj_from_pqpID\" array of the \"pqp_collision_grid\" structure is corrupted.\n",__FILE__,__LINE__);
        return 0;
      }

      if(pqp_COLLISION_PAIRS.obj_obj[i][j]==1)
      {
        printf("\t [ \"%s\",\n", body1->name);
        printf("\t   \"%s\" ]\n\n", body2->name);
      }
    }
  }
  
  printf("\n-----------------------------------\n");

  printf("Inactive collision pairs are:\n");
  for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
  {
    for(j=0; j<i; j++)
    {
      body1= pqp_COLLISION_PAIRS.obj_from_pqpID[i];
      body2= pqp_COLLISION_PAIRS.obj_from_pqpID[j];

      if(pqp_COLLISION_PAIRS.obj_obj[i][j]==0)
      {
        printf("\t [ \"%s\",\n", body1->name);
        printf("\t   \"%s\" ]\n\n", body2->name);
      }
    }
  }

  return 1;
}

//! Returns wether or not the collision will be tested between two objects.
//! \return 1 if the collision test between the two objects is activated, 0 otherwise
int pqp_is_collision_pair_activated(p3d_obj *o1, p3d_obj *o2)
{
 #ifdef PQP_DEBUG
  if(o1==NULL)
  {
     printf("%s: %d: pqp_is_collision_pair_activated(): o1 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
   if(o2==NULL)
  {
     printf("%s: %d: pqp_is_collision_pair_activated(): o2 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }

  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_is_collision_pair_activated(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(o1->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_is_collision_pair_activated(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,o1->name);
     return 0;
  }
  if(o2->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_is_collision_pair_activated(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,o2->name);
     return 0;
  }
 #endif  

 return pqp_COLLISION_PAIRS.obj_obj[o1->pqpID][o2->pqpID];
}


//! Deallocates all the PQP_Models.
void p3d_end_pqp()
{
  unsigned int i, j;
  p3d_obj *object= NULL;
  p3d_rob *robot= NULL;

  for (i=0; i<(unsigned int) XYZ_ENV->no; i++)
  {
    object= XYZ_ENV->o[i];
    if(object->pqpModel!=NULL)
    {  delete object->pqpModel;  }
  }

  for (j=0; j<(unsigned int) XYZ_ENV->nr; j++)
  {
    robot= XYZ_ENV->robot[j];
    for (i=0; i<(unsigned int) robot->no; i++)
    {
     object= robot->o[i];
     if(object->pqpModel!=NULL)
     {  delete object->pqpModel; }
    }
  }

  if(pqp_COLLISION_PAIRS.obj_obj!=NULL) 
  { 
    for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
    {
      if(pqp_COLLISION_PAIRS.obj_obj[i]!=NULL)
      {
        free(pqp_COLLISION_PAIRS.obj_obj[i]);
        pqp_COLLISION_PAIRS.obj_obj[i]= NULL;
      } 
    }
    free(pqp_COLLISION_PAIRS.obj_obj);
    pqp_COLLISION_PAIRS.obj_obj= NULL;
  }

  if(pqp_COLLISION_PAIRS.obj_from_pqpID!=NULL) 
  { 
    free(pqp_COLLISION_PAIRS.obj_from_pqpID);
    pqp_COLLISION_PAIRS.obj_from_pqpID= NULL;
  }

  pqp_COLLISION_PAIRS.nb_robots   = 0;
  pqp_COLLISION_PAIRS.nb_obstacles= 0;
  pqp_COLLISION_PAIRS.nb_objs     = 0;
}



//! Activates the collision tests between a given object and any other object.
//! If the object is linked to another object by a joint, the collision test will not be activated.
//! \param obj object to activate collision test with
//! \return 1 in case of success, 0 otherwise
int pqp_activate_object_collision(p3d_obj *object)
{
 #ifdef PQP_DEBUG
   if(object==NULL)
  {
     printf("%s: %d: pqp_activate__object_collision(): object is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate__object_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(object->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_activate__object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,object->name);
     return 0;
  }
 #endif

 unsigned int i, j;
 p3d_obj *object2;
 
 pqp_COLLISION_PAIRS.obj_obj[object->pqpID][object->pqpID]= 0;

 for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
 {
   for(j=i+1; j<pqp_COLLISION_PAIRS.nb_objs; j++)
   {
     if(pqp_COLLISION_PAIRS.obj_from_pqpID[i]==object)
     {  
       object2= pqp_COLLISION_PAIRS.obj_from_pqpID[j];
     }
     else
     {
       if(pqp_COLLISION_PAIRS.obj_from_pqpID[j]==object)
       {  
         object2= pqp_COLLISION_PAIRS.obj_from_pqpID[i];
       }
       else
       {
         continue;
       }
     }

     if(object->pqpPreviousBody==object2 || object2->pqpPreviousBody==object)
     {
       pqp_COLLISION_PAIRS.obj_obj[i][j]= 0;
       pqp_COLLISION_PAIRS.obj_obj[j][i]= 0;
     }
     else
     {
       pqp_COLLISION_PAIRS.obj_obj[i][j]= 1;
       pqp_COLLISION_PAIRS.obj_obj[j][i]= 1;
     }
   }
 }

 return 1;
}

//! Deactivates the collision tests between the given object and any other object.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_object_collision(p3d_obj *obj)
{
 #ifdef PQP_DEBUG
   if(obj==NULL)
  {
     printf("%s: %d: pqp_deactivate__object_collision(): the input is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate__object_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_deactivate__object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
     return 0;
  }
 #endif

 unsigned int i;
 for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
 {
   pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][i]= 0;
   pqp_COLLISION_PAIRS.obj_obj[i][obj->pqpID]= 0;
 }

 return 1;
}


//! Activates the collision test between the two given objects.
//! If the two objects are linked by a joint, the collision test will not be activated.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_object_object_collision(p3d_obj *o1, p3d_obj *o2)
{
 #ifdef PQP_DEBUG
   if(o1==NULL || o2==NULL)
  {
     printf("%s: %d: pqp_activate_object_object_collision(): one input or more is NULL (%p %p).\n",__FILE__,__LINE__,o1,o2);
     return 0;
  }
  if(o1==o2)
  {
     printf("%s: %d: pqp_activate_object_object_collision(): the inputs are identical.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate_object_object_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(o1->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_activate_object_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,o1->name);
     return 0;
  }
  if(o2->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_activate_object_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,o2->name);
     return 0;
  }
 #endif

 if(o1->pqpPreviousBody==o2 || o2->pqpPreviousBody==o1)
 {
   pqp_COLLISION_PAIRS.obj_obj[o1->pqpID][o2->pqpID]= 0; 
   pqp_COLLISION_PAIRS.obj_obj[o2->pqpID][o1->pqpID]= 0;
 }
 else
 {
   pqp_COLLISION_PAIRS.obj_obj[o1->pqpID][o2->pqpID]= 1; 
   pqp_COLLISION_PAIRS.obj_obj[o2->pqpID][o1->pqpID]= 1;
 }


 return 1;
}

//! Deactivates the collision tests between the two given objects.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_object_object_collision(p3d_obj *obj1, p3d_obj *obj2)
{
 #ifdef PQP_DEBUG
   if(obj1==NULL)
  {
     printf("%s: %d: pqp_deactivate_object_object_collision(): obj1 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
   if(obj2==NULL)
  {
     printf("%s: %d: pqp_deactivate_object_object_collision(): obj2 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }

  if(obj1==obj2)
  {
     printf("%s: %d: pqp_deactivate_object_object_collision(): the inputs are identical.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_object_object_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(obj1->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_deactivate_object_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj1->name);
     return 0;
  }
  if(obj2->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_deactivate_object_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj2->name);
     return 0;
  }
 #endif

 pqp_COLLISION_PAIRS.obj_obj[obj1->pqpID][obj2->pqpID]= 0; 
 pqp_COLLISION_PAIRS.obj_obj[obj2->pqpID][obj1->pqpID]= 0;

 return 1;
}

//! Activates the collision tests between the two given robots.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_robot_robot_collision(p3d_rob *robot1, p3d_rob *robot2)
{
  #ifdef PQP_DEBUG
   if(robot1==NULL)
  {
     printf("%s: %d: pqp_activate_robot_robot_collision(): robot1 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
   if(robot2==NULL)
  {
     printf("%s: %d: pqp_activate_robot_robot_collision(): robot2 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(robot1==robot2)
  {
     printf("%s: %d: pqp_activate_robot_robot_collision(): the inputs are identical.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate_robot_robot_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  #endif

  int i, j;
  for(i=0; i<robot1->no; i++)
  {
    for(j=0; j<robot2->no; j++)
    {
      #ifdef PQP_DEBUG
         if(robot1->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
         {
            printf("%s: %d: pqp_activate_robot_robot_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot1->o[i]->name);
            return 0;
         }
         if(robot2->o[j]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
         {
            printf("%s: %d: pqp_activate_robot_robot_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot2->o[j]->name);
            return 0;
         }
      #endif
      pqp_COLLISION_PAIRS.obj_obj[robot1->o[i]->pqpID][robot2->o[j]->pqpID]= 1;
      pqp_COLLISION_PAIRS.obj_obj[robot2->o[j]->pqpID][robot1->o[i]->pqpID]= 1;
    }
  }

  return 1;
}

//! Deactivates the collision tests between the two given robots.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_robot_robot_collision(p3d_rob *robot1, p3d_rob *robot2)
{
 #ifdef PQP_DEBUG
  if(robot1==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_robot_collision(): robot1 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(robot2==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_robot_collision(): robot2 is NULL.\n",__FILE__,__LINE__);
     return 0;
  }

  if(robot1==robot2)
  {
     printf("%s: %d: pqp_deactivate_robot_robot_collision(): the inputs are identical.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_robot_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
 #endif

  int i, j;
  for(i=0; i<robot1->no; i++)
  {
    for(j=0; j<robot2->no; j++)
    {
      #ifdef PQP_DEBUG
         if(robot1->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
         {
            printf("%s: %d: pqp_activate_robot_robot_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot1->o[i]->name);
            return 0;
         }
         if(robot2->o[j]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
         {
            printf("%s: %d: pqp_activate_robot_robot_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot2->o[j]->name);
            return 0;
         }
      #endif
      pqp_COLLISION_PAIRS.obj_obj[robot1->o[i]->pqpID][robot2->o[j]->pqpID]= 0;
      pqp_COLLISION_PAIRS.obj_obj[robot2->o[j]->pqpID][robot1->o[i]->pqpID]= 0;
    }
  }

  return 1;
}

//! Activates all the collision tests between the given robot and the environment obstacles.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_robot_environment_collision(p3d_rob *robot)
{
  #ifdef PQP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: pqp_activate_robot_environment_collision(): the input is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate_robot_environment_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  #endif

  int i, j;
  for(i=0; i<robot->no; i++)
  {
    #ifdef PQP_DEBUG
    if(robot->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {
      printf("%s: %d: pqp_activate_robot_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot->o[i]->name);
      return 0;
    }
    #endif   
    for(j=0; j<XYZ_ENV->no; j++)
    {
      #ifdef PQP_DEBUG
      if(XYZ_ENV->o[j]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
      {
        printf("%s: %d: pqp_activate_robot_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,XYZ_ENV->o[j]->name);
        return 0;
      }
      #endif
      pqp_COLLISION_PAIRS.obj_obj[robot->o[i]->pqpID][XYZ_ENV->o[j]->pqpID]= 1;
      pqp_COLLISION_PAIRS.obj_obj[XYZ_ENV->o[j]->pqpID][robot->o[i]->pqpID]= 1;
    }
  }


  return 0;
}


//! Deactivates all the collision tests between the given robot and the environment obstacles.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_robot_environment_collision(p3d_rob *robot)
{

 #ifdef PQP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_environment_collision(): the input is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_environment_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
 #endif


  int i, j;
  for(i=0; i<robot->no; i++)
  {
    #ifdef PQP_DEBUG
    if(robot->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {
      printf("%s: %d: pqp_deactivate_robot_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot->o[i]->name);
      return 0;
    }
    #endif   
    for(j=0; j<XYZ_ENV->no; j++)
    {
      #ifdef PQP_DEBUG
      if(XYZ_ENV->o[j]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
      {
        printf("%s: %d: pqp_deactivate_robot_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,XYZ_ENV->o[j]->name);
        return 0;
      }
      #endif
      pqp_COLLISION_PAIRS.obj_obj[robot->o[i]->pqpID][XYZ_ENV->o[j]->pqpID]= 0;
      pqp_COLLISION_PAIRS.obj_obj[XYZ_ENV->o[j]->pqpID][robot->o[i]->pqpID]= 0;
    }
  }


  return 0;
}

//! Activates the collision test between the given robot and object.
//! If the object is a body of the robot and is linked  by a joint to another body of the robot,
//! the collision test between these two bodies will not be activated.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_robot_object_collision(p3d_rob *robot, p3d_obj *obj)
{
  #ifdef PQP_DEBUG
  if(robot==NULL || obj==NULL)
  {
     printf("%s: %d: pqp_activate_robot_object_collision(): one input or more is NULL (%p %p).\n",__FILE__,__LINE__,robot,obj);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate_robot_object_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
   printf("%s: %d: pqp_activate_robot_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
    return 0;
  }
  #endif

  int i;
  for(i=0; i<robot->no; i++)
  {
    #ifdef PQP_DEBUG
    if(robot->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {
        printf("%s: %d: pqp_activate_robot_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot->o[i]->name);
        return 0;
    }
    #endif

    if(robot->o[i]->pqpPreviousBody==obj || obj->pqpPreviousBody==robot->o[i])
    {
      pqp_COLLISION_PAIRS.obj_obj[robot->o[i]->pqpID][obj->pqpID]= 0;
      pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][robot->o[i]->pqpID]= 0;
    }
    else
    {
      pqp_COLLISION_PAIRS.obj_obj[robot->o[i]->pqpID][obj->pqpID]= 1;
      pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][robot->o[i]->pqpID]= 1;
    }
  }

  return 1;
}

//! Deactivates the collision test between the given robot and the obstacle.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_robot_object_collision(p3d_rob *robot, p3d_obj *obj)
{
  #ifdef PQP_DEBUG
  if(robot==NULL || obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_object_collision(): one input or more is NULL (%p %p).\n",__FILE__,__LINE__,robot,obj);
     return 0;
  } 
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_object_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
     printf("%s: %d: pqp_deactivate_robot_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
     return 0;
  }
  #endif

  int i;
  for(i=0; i<robot->no; i++)
  {
    #ifdef PQP_DEBUG
    if(robot->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {
      printf("%s: %d: pqp_deactivate_robot_object_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot->o[i]->name);
      return 0;
    }
    #endif
    pqp_COLLISION_PAIRS.obj_obj[robot->o[i]->pqpID][obj->pqpID]= 0;
    pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][robot->o[i]->pqpID]= 0;
  }

  return 1;
}


//! Activates the selfcollision tests of the given robot.
//! The collision tests between two bodies that are linked by a joint are not activated.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_robot_selfcollision(p3d_rob *robot)
{
  #ifdef PQP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: pqp_activate_robot_selfcollision():robot is NULL.\n",__FILE__,__LINE__);
     return 0;
  } 
  #endif

  int i, j;
  p3d_obj *body1, *body2;

  for (i=0; i<robot->no; i++)
  {
      body1= robot->o[i];

      if(body1->pqpModel==NULL)
      {  continue;  }

      for (j=i+1; j<robot->no; j++)
      {
          body2= robot->o[j];

          if(body2->pqpModel==NULL)
          {  continue;  }

          #ifdef PQP_DEBUG
          if(body1->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
          {               
            printf("%s: %d: pqp_activate_robot_selfcollision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body1->name);
            return 0;
          }
          if(body2->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
          {               
            printf("%s: %d: pqp_activate_robot_selfcollision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body2->name);
            return 0;
          }
          #endif

          if(body1->pqpPreviousBody==body2 || body2->pqpPreviousBody==body1)
          {   
            pqp_COLLISION_PAIRS.obj_obj[body1->pqpID][body2->pqpID]= 0;
            pqp_COLLISION_PAIRS.obj_obj[body2->pqpID][body1->pqpID]= 0;
          }
          else
          {
            pqp_COLLISION_PAIRS.obj_obj[body1->pqpID][body2->pqpID]= 1;
            pqp_COLLISION_PAIRS.obj_obj[body2->pqpID][body1->pqpID]= 1;
          }

      }
  }

  return 1;
}

//! Deactivates the selfcollision tests of the given robot.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_robot_selfcollision(p3d_rob *robot)
{
  #ifdef PQP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_selfcollision():robot is NULL.\n",__FILE__,__LINE__);
     return 0;
  } 
  #endif

  int i, j;
  p3d_obj *body1, *body2;

  for (i=0; i<robot->no; i++)
  {
      body1= robot->o[i];

      if(body1->pqpModel==NULL)
      {  continue;  }

      for (j=i+1; j<robot->no; j++)
      {
          body2= robot->o[j];

          if(body2->pqpModel==NULL)
          {  continue;  }

          if(body2==body1->pqpPreviousBody || body1==body2->pqpPreviousBody)
          {   continue;  }

          #ifdef PQP_DEBUG
          if(body1->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
          {               
            printf("%s: %d: pqp_deactivate_robot_selfcollision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body1->name);
            return 0;
          }
          if(body2->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
          {               
            printf("%s: %d: pqp_deactivate_robot_selfcollision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body2->name);
            return 0;
          }
          #endif

          pqp_COLLISION_PAIRS.obj_obj[body1->pqpID][body2->pqpID]= 0;
          pqp_COLLISION_PAIRS.obj_obj[body2->pqpID][body1->pqpID]= 0;
      }
  }

  return 1;
}


//! Activates all the collision tests of the given robot.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_robot_collision(p3d_rob *robot)
{
  int i;
  #ifdef PQP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: pqp_activate_robot_collision():robot is NULL.\n",__FILE__,__LINE__);
     return 0;
  } 
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate_robot_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  #endif


  pqp_activate_robot_environment_collision(robot);

  for(i=0; i<XYZ_ENV->nr; i++)
  {
    if(robot==XYZ_ENV->robot[i])
    {
      continue;
    }
    pqp_activate_robot_robot_collision(robot, XYZ_ENV->robot[i]);
  }

  pqp_activate_robot_selfcollision(robot);  

  return 1;
}


//! Deactivates all the collision tests of the given robot.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_robot_collision(p3d_rob *robot)
{
  int i;
  #ifdef PQP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_collision():robot is NULL.\n",__FILE__,__LINE__);
     return 0;
  } 
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_robot_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  #endif


  pqp_deactivate_robot_environment_collision(robot);

  for(i=0; i<XYZ_ENV->nr; i++)
  {
    if(robot==XYZ_ENV->robot[i])
    {
      continue;
    }
    pqp_deactivate_robot_robot_collision(robot, XYZ_ENV->robot[i]);
  }

  pqp_deactivate_robot_selfcollision(robot);  

  return 1;
}


//! Activates the collision test between the given object and the environment.
//! The object can be a robot body or an environment obstacle.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_object_environment_collision(p3d_obj *obj)
{
  #ifdef PQP_DEBUG
  if(obj==NULL)
  {
     printf("%s: %d: pqp_activate_object_environment_collision(): the input is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_activate_object_environment_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
   printf("%s: %d: pqp_activate_object_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
    return 0;
  }
  #endif

  int i;
  p3d_obj *obst= NULL;
  for(i=0; i<XYZ_ENV->no; i++)
  {
    obst= XYZ_ENV->o[i];
    #ifdef PQP_DEBUG
    if(obst->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {
        printf("%s: %d: pqp_activate_object_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obst->name);
        return 0;
    }
    #endif
    pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][obst->pqpID]= 1;
    pqp_COLLISION_PAIRS.obj_obj[obst->pqpID][obj->pqpID]= 1;
  }

  return 1;
}



//! Deactivates the collision test between the given object and the environment.
//! The object can be a robot body or an environment obstacle.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_object_environment_collision(p3d_obj *obj)
{
  #ifdef PQP_DEBUG
  if(obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_object_environment_collision(): the input is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_deactivate_object_environment_collision(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
  {
   printf("%s: %d: pqp_deactivate_object_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
    return 0;
  }
  #endif

  int i;
  p3d_obj *obst= NULL;
  for(i=0; i<XYZ_ENV->no; i++)
  {
    obst= XYZ_ENV->o[i];
    #ifdef PQP_DEBUG
    if(obst->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {
        printf("%s: %d: pqp_deactivate_object_environment_collision(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obst->name);
        return 0;
    }
    #endif
    pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][obst->pqpID]= 0;
    pqp_COLLISION_PAIRS.obj_obj[obst->pqpID][obj->pqpID]= 0;
  }

  return 1;
}


//! Activates the collisions between all the body pairs.
//! If two bodies are linked by a joint, the collision test between this two bodies will not be activated.
//! \return 1 in case of success, 0 otherwise
int pqp_activate_all_collisions()
{
  #ifdef PQP_DEBUG
   if(pqp_COLLISION_PAIRS.obj_obj==NULL)
   {
     printf("%s: %d: pqp_activate_all_collisions(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
     return 0;    
   }
  #endif

  unsigned int i, j, nb_rob, nb_obst;
  p3d_obj *obj1, *obj2;

  nb_rob = XYZ_ENV->nr;
  nb_obst= XYZ_ENV->no;

  for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
  {
    pqp_COLLISION_PAIRS.obj_obj[i][i]= 0;
    for(j=i+1; j<pqp_COLLISION_PAIRS.nb_objs; j++)
    {
      obj1= pqp_COLLISION_PAIRS.obj_from_pqpID[i];
      obj2= pqp_COLLISION_PAIRS.obj_from_pqpID[j];

      if(obj1->pqpPreviousBody==obj2 || obj2->pqpPreviousBody==obj1)
      {
        pqp_COLLISION_PAIRS.obj_obj[i][j]= 0;
        pqp_COLLISION_PAIRS.obj_obj[j][i]= 0;
      }
      else
      {
        pqp_COLLISION_PAIRS.obj_obj[i][j]= 1;
        pqp_COLLISION_PAIRS.obj_obj[j][i]= 1;
      }
    }  
  }  
/*
  //Deactivate all collisions for the P3D_GRAPHIC_OBJECT bodies:
  for(i=0; i<nb_obst; i++)
  {  
    if(XYZ_ENV->o[i]->GRAPHIC_TYPE==P3D_GRAPHIC_OBJECT)
    {
      pqp_deactivate_object_collision(XYZ_ENV->o[i]);
    }   
  }

  for(i=0; i<nb_rob; i++)
  {
    for(j=0; j<XYZ_ENV->robot[i]->no; j++)
    {
      if(XYZ_ENV->robot[i]->o[j]->GRAPHIC_TYPE==P3D_GRAPHIC_OBJECT)
      {
        pqp_deactivate_object_collision(XYZ_ENV->robot[i]->o[j]);
      }  
    }    
  }*/


  return 1;
}

//! Deactivates the collisions between all the body pairs.
//! \return 1 in case of success, 0 otherwise
int pqp_deactivate_all_collisions()
{
  #ifdef PQP_DEBUG
   if(pqp_COLLISION_PAIRS.obj_obj==NULL)
   {
     printf("%s: %d: pqp_deactivate_all_collisions(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
   }
  #endif

  unsigned int i, j;
  for(i=0; i<pqp_COLLISION_PAIRS.nb_objs; i++)
  {
    for(j=0; j<pqp_COLLISION_PAIRS.nb_objs; j++)
    {
      pqp_COLLISION_PAIRS.obj_obj[i][j]= 0;
    }  
  }  

  return 1;
}


//! Computes and returns the area of the triangle (p1p2p3).
//! It uses Hero of Alexandria formula.
//! \param p1 coordinates of triangle first vertex
//! \param p2 coordinates of triangle second vertex
//! \param p3 coordinates of triangle third vertex
//! \return the computed area
double pqp_triangle_area(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3)
{
    double a, b, c, s;
    p3d_vector3 p1p2, p2p3, p3p1;

    p3d_vectSub(p2, p1, p1p2);
    p3d_vectSub(p3, p2, p2p3);
    p3d_vectSub(p1, p3, p3p1);

    a= p3d_vectNorm(p1p2);
    b= p3d_vectNorm(p2p3);
    c= p3d_vectNorm(p3p1);

  
    s= ( a + b + c )/2;

    return sqrt( s*(s-a)*(s-b)*(s-c) );  
}


//! A face is degenerate if it has duplicate vertices.
//! Another case is when a face is not planar.
//! NB: The 'return' corresponding to the case where a face is non planar has been commented because
//! some modellers (like Blender) can create models with non planar faces.
//! This should not be an issue for the triangulation function.
//! \return 1 if the face is degenerate, 0 otherwise
int pqp_is_face_degenerate(p3d_polyhedre *polyhedron, unsigned int face_index)
{
    #ifdef PQP_DEBUG
    if(polyhedron==NULL)
    {
        printf("%s: %d: pqp_is_face_degenerate(): input is NULL.\n",__FILE__,__LINE__);
        return 0;
    }
    if(face_index >= polyhedron->nb_faces)
    {
        printf("%s: %d: pqp_is_face_degenerate(): index of face exceeds p3d_polyhedre face array dimension (%d vs %d).\n",__FILE__,__LINE__, face_index, polyhedron->nb_faces);
        return 0;
    }
    #endif

    unsigned int i, j;
    unsigned int ind1, ind2, ind3, tmp;
    double d, dmax;
    p3d_vector3 e1, e2, normal;
    p3d_vector3 *points= polyhedron->the_points;
    p3d_face *face= &polyhedron->the_faces[face_index];


    //Check if there is no duplicate vertex:
    for(i=0; i<face->nb_points; i++)
    {
      for(j=i+1; j<face->nb_points; j++)
      {
        p3d_vectSub(points[face->the_indexs_points[i]-1] , points[face->the_indexs_points[j]-1], e1);
        if(p3d_vectNorm(e1)<1e-7)
        { 
          return 1; 
        }
      }
    }

    //find the farthest vextex from the first one (vertex1) -> vertex2
    dmax= 0;
    ind1= 0;
    for(i=1; i<face->nb_points; i++)
    {
      p3d_vectSub(points[face->the_indexs_points[i]-1], points[face->the_indexs_points[ind1]-1], e1);
      d= p3d_vectNorm(e1);
      if(d>dmax)
      { 
        dmax= d;
        ind2= i;
      }
    }

    //find the vextex that maximizes the area of the triangle (vertex1,vertex2,vertex3):
    dmax= 0;
    for(i=1; i<face->nb_points; i++)
    {
      if(i==ind2)
        continue;

      d= pqp_triangle_area(points[face->the_indexs_points[ind1]-1], points[face->the_indexs_points[ind2]-1], points[face->the_indexs_points[i]-1]);

      if(d>dmax)
      { 
        dmax= d;
        ind3= i;
      }
    }

    //the three vertices must be in the same order than in the face:
    if(ind3<ind2)
    {
      tmp= ind2;
      ind2= ind3;
      ind3= tmp;
    }

    //compute the face normal:
    p3d_vectSub(points[face->the_indexs_points[ind2]-1] , points[face->the_indexs_points[ind1]-1], e1 );
    p3d_vectSub(points[face->the_indexs_points[ind3]-1] , points[face->the_indexs_points[ind1]-1], e2 );

    p3d_vectNormalize(e1, e1);
    p3d_vectNormalize(e2, e2);
    p3d_vectXprod(e1, e2, normal);
    p3d_vectNormalize(normal, normal);
    d= -p3d_vectDotProd(normal, points[face->the_indexs_points[ind1]-1]);

    //check if the face is planar:
    for(i=1; i<face->nb_points; i++)
    {
      if( fabs( p3d_vectDotProd(normal, points[face->the_indexs_points[i]-1]) + d )> 1e-4)
      {  
        printf("%s: %d: pqp_is_face_degenerate(): %s has a non planar face.\n",__FILE__,__LINE__,polyhedron->name);
        //return 1; //see above to know why this return is commented
      }
    }


    return 0;
}




//! Triangulates the face of index k in the face array of a p3d_polyhedre.
//! The computed triangles (indices of vertices in the vertex array of the polyhedron) are returned
//! while the number of computed triangles (that must be the vertex number of the face minus 2) is copied in nb_triangles.
pqp_triangle* pqp_triangulate_face(p3d_polyhedre *polyhedron, unsigned int face_index, unsigned int *nb_triangles)
{
    #ifdef PQP_DEBUG
    if(polyhedron==NULL)
    {
        printf("%s: %d: pqp_triangulate_face(): input is NULL.\n",__FILE__,__LINE__);
        return NULL;
    }
    if(face_index >= polyhedron->nb_faces)
    {
        printf("%s: %d: pqp_triangulate_face(): index of face exceeds p3d_polyhedre face array dimension (%d vs %d).\n",__FILE__,__LINE__, face_index, polyhedron->nb_faces);
        return NULL;
    }
    #endif

    unsigned int i, nb_triangles2= 0;
    p3d_vector3 e1, e2, normal;

    p3d_vector2 *vertices= NULL;
    p3d_vector3 *points= polyhedron->the_points;
    p3d_face *face= &polyhedron->the_faces[face_index];
    pqp_triangle *triangles= NULL;
    pqp_triangle *triangles2= NULL;

    if(face->nb_points < 4)
        return NULL;
 

    // compute face normal:
    p3d_vectSub(points[face->the_indexs_points[1]-1] , points[face->the_indexs_points[0]-1], e1 );
    p3d_vectNormalize(e1, e1);

    for(i=2; i<face->nb_points; i++)
    {
        p3d_vectSub(points[face->the_indexs_points[i]-1] , points[face->the_indexs_points[0]-1], e2 );
        p3d_vectNormalize(e2, e2);
        p3d_vectXprod(e1, e2, normal);
        if(p3d_vectNorm(normal)>1e-7)
        {
          p3d_vectNormalize(normal, normal);
          break;
        }
    }

    if(isnan(normal[0]) || isnan(normal[1]) || isnan(normal[2]))
    {
      printf("%s: %d: pqp_triangulate_face(): face normal computation error.\n",__FILE__,__LINE__);
      return NULL;
    }


    // reduce all the face points to 2D vectors:
    vertices= (p3d_vector2 *) malloc(face->nb_points*sizeof(p3d_vector2));

    pqp_orthonormal_basis(normal, e1, e2);

    for (i=0; i<face->nb_points; i++)
    {
      vertices[i][0]= p3d_vectDotProd(points[face->the_indexs_points[i]-1], e1);
      vertices[i][1]= p3d_vectDotProd(points[face->the_indexs_points[i]-1], e2);

      if(isnan(vertices[i][0]) || isnan(vertices[i][1]))
      {
        printf("%s: %d: pqp_triangulate_face(): vertex coordinate is NaN.\n",__FILE__,__LINE__);
        return NULL;
      }
    }


    triangles= pqp_triangulate_polygon(vertices, face->nb_points, &nb_triangles2);

    triangles2= (pqp_triangle *) malloc(nb_triangles2*sizeof(pqp_triangle));

    for (i=0; i<nb_triangles2; i++)
    {
      triangles2[i][0]= face->the_indexs_points[triangles[i][0]] - 1;
      triangles2[i][1]= face->the_indexs_points[triangles[i][1]] - 1;
      triangles2[i][2]= face->the_indexs_points[triangles[i][2]] - 1;
    }

    free(triangles);
    *nb_triangles= nb_triangles2;

    return triangles2;
}


//! Tests if a 2D-point is inside a triangle or not.
//! \param p the coordinates of the point
//! \param a the coordinates of the first vertex of the triangle
//! \param b the coordinates of the second vertex of the triangle
//! \param c the coordinates of the third vertex of the triangle
//! \return 1 if the point is inside the triangle, 0 otherwise
int pqp_is_point_in_triangle(p3d_vector2 p, p3d_vector2 a, p3d_vector2 b, p3d_vector2 c)
{
    int i;
    p3d_vector2 u, v, w;

    #ifdef PQP_DEBUG
    if(p==NULL || a==NULL || b==NULL || c==NULL)
    {
        printf("isPointInTriangle(): one or more inputs is NULL (%p %p %p %p)\n", p, a, b, c);
    }
    #endif

    for (i=0; i<2; i++)
    {
        u[i]= b[i] - a[i];
        v[i]= p[i] - a[i];
        w[i]= c[i] - a[i];
    }

    if( ( u[1]*v[0] - u[0]*v[1] > 0 )  !=  ( u[1]*w[0] - u[0]*w[1] > 0 ) )
    {
        return 0;
    }

    for (i=0; i<2; i++)
    {
        u[i]= c[i] - b[i];
        v[i]= p[i] - b[i];
        w[i]= a[i] - b[i];
    }

    if( ( u[1]*v[0] - u[0]*v[1] > 0 )  !=  ( u[1]*w[0] - u[0]*w[1] > 0 ) )
    {
        return 0;
    }

    for (i=0; i<2; i++)
    {
        u[i]= a[i] - c[i];
        v[i]= p[i] - c[i];
        w[i]= b[i] - c[i];
    }

    if( ( u[1]*v[0] - u[0]*v[1] > 0 )  !=  ( u[1]*w[0] - u[0]*w[1] > 0 ) )
    {
        return 0;
    }

    return 1;
}


//! Triangulates the polygon whose vertices are given in the corresponding array. The number of vertices
//! is given in nb_vertices.
//! Returns a pointer to an array of triangles (indices, in the array of vertices, of the triangle vertices).
//! The number of triangles is written in nb_triangles and must be (nb_vertices-2).
//! If the triangulation fails, the function returns NULL and nb_triangles is set to 0.
//! The used algorithm is the "ear-cut algorithm".
//! \param vertices the array of polygon vertex coordinates
//! \param nb_vertices the number of vertices of the polygon (size of the vertex array)
//! \param nb_triangles a pointer to an integer that will be filled with the number of triangles of the triangulation
//! \return pointer to an array of pqp_triangle that are the result of the triangulation
pqp_triangle* pqp_triangulate_polygon(p3d_vector2 *vertices, int nb_vertices, unsigned int *nb_triangles)
{
    #ifdef PQP_DEBUG
    if(vertices==NULL || nb_triangles==NULL)
    {
        printf("%s: %d: pqp_triangulate_polygon(): one or more inputs is NULL (%p %p).\n", __FILE__, __LINE__,vertices,nb_triangles);
        return NULL;
    }
    if(nb_vertices<=3)
    {
        printf("%s: %d: pqp_triangulate_polygon(): at least 4 vertices are needed.\n", __FILE__, __LINE__);
        return NULL;
    }
    #endif

    int i, j, k, n, count, nb_iters= 0, nb_triangles2= 0;
    int previous, current, next;
    double norm1, norm2;
    p3d_vector2 p1, p2;

    int *isVertexConvex= NULL; //used to mark each vertex convexity
    int *polygon= NULL; //used to store the indices of the vertices that remain to be treated
    //(the polygon that is incrementally triangulated)
    int *polygon_bis= NULL; //used to modify the order of the array polygon
    int *tmp= NULL;
    pqp_triangle *triangles= NULL;

    isVertexConvex= (int *) malloc(nb_vertices*sizeof(int));
    polygon= (int *) malloc(nb_vertices*sizeof(int));
    polygon_bis= (int *) malloc(nb_vertices*sizeof(int));
    triangles= (pqp_triangle *) malloc((nb_vertices-2)*sizeof(pqp_triangle));

    n= nb_vertices;
    nb_triangles2= 0;

    for (i=0; i<nb_vertices; i++)
    {
      polygon[i]= i;
      #ifdef PQP_DEBUG
        if(isnan(vertices[i][0]) ||  isnan(vertices[i][1]) )
        {
          printf("%s: %d: pqp_triangulate_polygon(): One of the input vertex coordinates is NaN.\n", __FILE__, __LINE__);
        }
      #endif
    }  

    while(1)
    {
        if(nb_iters>300)
        {
            printf("%s: %d: pqp_triangulate_polygon(): the number of iterations is abnormally high. Check the consistency of input data.\n", __FILE__, __LINE__);

            *nb_triangles= 0;
            free(isVertexConvex);
            free(polygon);
            free(triangles);
            return NULL;
        }
        nb_iters++;

        if(n==3)
        {
            triangles[nb_triangles2][0] =  polygon[0];
            triangles[nb_triangles2][1] =  polygon[1];
            triangles[nb_triangles2][2] =  polygon[2];
            nb_triangles2++;
            break;
        }

        //compute vertex convexities:
        for (i=0; i<n; i++)
        {
            current= polygon[i];
            if(i==0)
            {
                previous= polygon[n-1];
            }
            else
            {
                previous= polygon[i-1];
            }

            if(i==n-1)
            {
                next= polygon[0];
            }
            else
            {
                next= polygon[i+1];
            }
            for (j=0; j<2; j++)
            {
                p1[j]=  vertices[current][j] - vertices[previous][j];
                p2[j]=  vertices[next][j]    - vertices[current][j];
            }
            norm1= sqrt( p1[0]*p1[0] + p1[1]*p1[1] );
            norm2= sqrt( p2[0]*p2[0] + p2[1]*p2[1] );
            for (j=0; j<2; j++)
            {
                p1[j]/=  norm1;
                p2[j]/=  norm2;
            }
            if( p1[1]*p2[0] - p1[0]*p2[1] > EPSILON )
                isVertexConvex[current]= FALSE;
            else
                isVertexConvex[current]= TRUE;
        }

        //find "ear" to cut:
        for (i=0; i<n; i++)
        {
            current= polygon[i];

            if( isVertexConvex[current]==FALSE )
                continue;

            if(i==0)
                previous= polygon[n-1];
            else
                previous= polygon[i-1];

            if(i==n-1)
                next= polygon[0];
            else
                next= polygon[i+1];

            for (j=0; j<n; j++)
            {
                if( polygon[j]==previous || polygon[j]==current || polygon[j]==next )
                {
                    continue;
                }
                else
                {
                    if( pqp_is_point_in_triangle(vertices[polygon[j]], vertices[previous], vertices[current], vertices[next])==TRUE )
                        break;
                }
            }

            //ear found
            if(j==n)
            {
                //add the new triangle:
                triangles[nb_triangles2][0] =  previous;
                triangles[nb_triangles2][1] =  current;
                triangles[nb_triangles2][2] =  next;
                nb_triangles2++;
                count= 0;

                //remove the ear vertex from the vertices to treat:
                for (k=0; k<n; k++)
                {
                    if(polygon[k]!=current)
                    {
                        polygon_bis[count]= polygon[k];
                        count++;
                    }
                }
                n--;
                tmp= polygon_bis;
                polygon_bis= polygon;
                polygon= tmp;
                break;
            }
        }

    }

    free(isVertexConvex);
    free(polygon);
    free(polygon_bis);

    *nb_triangles= nb_triangles2;

    return triangles;
}

//! Draws the PQP model (triangles) of the given object.
//! Use this function in an OpenGL display function.
void pqp_draw_model(p3d_obj *object, double red, double green, double blue)
{
    #ifdef PQP_DEBUG
    if(object==NULL)
    {
      printf("%s: %d: pqp_draw_models(): argument 1 (p3d_obj *) is NULL.\n", __FILE__, __LINE__);
      return;
    }
    if(object->pqpModel==NULL)
    {
      printf("%s: %d: pqp_draw_models(): the p3d_obj variable (%s) has no PQP model.\n", __FILE__, __LINE__, object->name);
      return;
    }
    #endif

    int i;
    PQP_Model *pqpModel= (PQP_Model *) object->pqpModel;

    if(pqpModel==NULL)
    {   return;  }

    double color_vect[4]= {red, green, blue, 1.0};

    g3d_set_color_mat(Any, color_vect);

    float u[3], v[3], n[3];
    float norm;
    for (i=0; i<pqpModel->num_tris; i++)
    {
        v[0]= pqpModel->tris[i].p2[0] - pqpModel->tris[i].p3[0];
        v[1]= pqpModel->tris[i].p2[1] - pqpModel->tris[i].p3[1];
        v[2]= pqpModel->tris[i].p2[2] - pqpModel->tris[i].p3[2];
        u[0]= pqpModel->tris[i].p1[0] - pqpModel->tris[i].p3[0];
        u[1]= pqpModel->tris[i].p1[1] - pqpModel->tris[i].p3[1];
        u[2]= pqpModel->tris[i].p1[2] - pqpModel->tris[i].p3[2];
        n[0]= u[1]*v[2] - u[2]*v[1];
        n[1]= u[2]*v[0] - u[0]*v[2];
        n[2]= u[0]*v[1] - u[1]*v[0];
        norm= sqrt(pow(n[0],2) + pow(n[1],2) + pow(n[2],2));
        glNormal3f(n[0]/norm, n[1]/norm, n[2]/norm);
        glBegin(GL_TRIANGLES);
        glVertex3f(pqpModel->tris[i].p1[0], pqpModel->tris[i].p1[1], pqpModel->tris[i].p1[2]);
        glVertex3f(pqpModel->tris[i].p2[0], pqpModel->tris[i].p2[1], pqpModel->tris[i].p2[2]);
        glVertex3f(pqpModel->tris[i].p3[0], pqpModel->tris[i].p3[1], pqpModel->tris[i].p3[2]);
        glEnd(); /*
    glNormal3f(-n[0]/norm, -n[1]/norm, -n[2]/norm);
    glBegin(GL_TRIANGLES);
       glVertex3f(pqpModel->tris[i].p1[0], pqpModel->tris[i].p1[1], pqpModel->tris[i].p1[2]);
       glVertex3f(pqpModel->tris[i].p3[0], pqpModel->tris[i].p3[1], pqpModel->tris[i].p3[2]);
       glVertex3f(pqpModel->tris[i].p2[0], pqpModel->tris[i].p2[1], pqpModel->tris[i].p2[2]);
    glEnd();  */
    }


    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);
    for (i=0; i<pqpModel->num_tris; i++)
    {
        glBegin(GL_LINES);
        glVertex3f(pqpModel->tris[i].p1[0],pqpModel->tris[i].p1[1],pqpModel->tris[i].p1[2]);
        glVertex3f(pqpModel->tris[i].p2[0],pqpModel->tris[i].p2[1],pqpModel->tris[i].p2[2]);
        glVertex3f(pqpModel->tris[i].p1[0],pqpModel->tris[i].p1[1],pqpModel->tris[i].p1[2]);
        glVertex3f(pqpModel->tris[i].p3[0],pqpModel->tris[i].p3[1],pqpModel->tris[i].p3[2]);
        glVertex3f(pqpModel->tris[i].p3[0],pqpModel->tris[i].p3[1],pqpModel->tris[i].p3[2]);
        glVertex3f(pqpModel->tris[i].p2[0],pqpModel->tris[i].p2[1],pqpModel->tris[i].p2[2]);
        glEnd();
    }
    glEnable(GL_LIGHTING);

}

//! Draws the PQP models (triangles) of the all the environment objects (obstacles and robot bodies).
//! Use this function in an OpenGL display function.
void pqp_draw_all_models()
{
    int i, j;
    p3d_matrix4 pose;
    GLfloat mat[16];
    p3d_obj *object= NULL;
    p3d_rob *robot= NULL;

    for (i=0; i<XYZ_ENV->no; i++)
    {  
      glPushMatrix();
      object= XYZ_ENV->o[i];
 
      if(object->pqpModel==NULL)
      { continue; }

      pqp_get_obj_pos(object, pose);
      pqp_p3d_to_gl_matrix(pose, mat);
      glMultMatrixf(mat);
      //glTranslatef(0, 0, d); 
      pqp_draw_model(object, 0.0, 1.0, 1.0);
      glPopMatrix();
    }


    for (i=0; i<XYZ_ENV->nr; i++)
    {
        robot= XYZ_ENV->robot[i];
        for (j=0; j<robot->no; j++)
        {
          object= robot->o[j];

          if(object->pqpModel==NULL)
          { continue; }

          pqp_get_obj_pos(object, pose);
          pqp_p3d_to_gl_matrix(pose, mat);
          glPushMatrix();
          glMultMatrixf(mat);
          pqp_draw_model(object, 1.0, 0.0, 1.0);
          glPopMatrix();
        }
    }
}

//! Recursive function used by the function pqp_draw_OBBs (see below).
void pqp_draw_OBBs_recursive(p3d_obj *object, double M[4][4], int bn, int currentLevel, int level)
{
    int i, j;
    double M2[4][4], M3[4][4];
    double p1[8][3], p2[8][3];

    #ifdef PQP_DEBUG
    if(object==NULL)
    {
        printf("%s: %d: pqp_draw_OBB_recursive(): argument 1 (p3d_obj *) is NULL.\n", __FILE__, __LINE__);
        return;
    }
    if(object->pqpModel==NULL)
    {
        printf("%s: %d: pqp_draw_OBB_recursive(): the p3d_obj variable (%s) has no PQP model.\n", __FILE__, __LINE__, object->name);
        return;
    }
    #endif

    if(currentLevel + 1 > level)
    {
       return;
    }

    PQP_Model *pqpModel= (PQP_Model *) object->pqpModel;


    BV* bv= pqpModel->child(bn);
    if(bv==NULL)
       return;

    M2[0][0]= bv->R[0][0];
    M2[0][1]= bv->R[0][1];
    M2[0][2]= bv->R[0][2];
    M2[0][3]= bv->To[0];
    M2[1][0]= bv->R[1][0];
    M2[1][1]= bv->R[1][1];
    M2[1][2]= bv->R[1][2];
    M2[1][3]= bv->To[1];
    M2[2][0]= bv->R[2][0];
    M2[2][1]= bv->R[2][1];
    M2[2][2]= bv->R[2][2];
    M2[2][3]= bv->To[2];
    M2[3][0]= 0;
    M2[3][1]= 0;
    M2[3][2]= 0;
    M2[3][3]= 1;

    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
        {
            M3[i][j] = M[i][0]*M2[0][j] + M[i][1]*M2[1][j] + M[i][2]*M2[2][j] + M[i][3]*M2[3][j];
        }
    }

    if(bv->Leaf())
    {
        i= -bv->first_child-1;
        g3d_set_color_mat(Black, NULL);
        glBegin(GL_TRIANGLES);
        glVertex3f(pqpModel->tris[i].p1[0], pqpModel->tris[i].p1[1], pqpModel->tris[i].p1[2]);
        glVertex3f(pqpModel->tris[i].p2[0], pqpModel->tris[i].p2[1], pqpModel->tris[i].p2[2]);
        glVertex3f(pqpModel->tris[i].p3[0], pqpModel->tris[i].p3[1], pqpModel->tris[i].p3[2]);
        glEnd();
        return;
    }
    else
    {
        if(currentLevel + 1 == level)
        {
            p1[0][0]=  bv->d[0];
            p1[0][1]=  bv->d[1];
            p1[0][2]=  bv->d[2];
            p1[1][0]=  bv->d[0];
            p1[1][1]=  bv->d[1];
            p1[1][2]= -bv->d[2];
            p1[2][0]=  bv->d[0];
            p1[2][1]= -bv->d[1];
            p1[2][2]=  bv->d[2];
            p1[3][0]=  bv->d[0];
            p1[3][1]= -bv->d[1];
            p1[3][2]= -bv->d[2];
            p1[4][0]= -bv->d[0];
            p1[4][1]=  bv->d[1];
            p1[4][2]=  bv->d[2];
            p1[5][0]= -bv->d[0];
            p1[5][1]=  bv->d[1];
            p1[5][2]= -bv->d[2];
            p1[6][0]= -bv->d[0];
            p1[6][1]= -bv->d[1];
            p1[6][2]=  bv->d[2];
            p1[7][0]= -bv->d[0];
            p1[7][1]= -bv->d[1];
            p1[7][2]= -bv->d[2];

            for (i=0; i<8; i++)
            {
                for (j=0; j<3; j++)
                {
                    p2[i][j]= M3[j][0]*p1[i][0] + M3[j][1]*p1[i][1] + M3[j][2]*p1[i][2] + M3[j][3];
                }
            }
            glDisable(GL_LIGHTING);
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            glVertex3dv(p2[0]);
            glVertex3dv(p2[1]);
            glVertex3dv(p2[0]);
            glVertex3dv(p2[2]);
            glVertex3dv(p2[0]);
            glVertex3dv(p2[4]);
            glVertex3dv(p2[1]);
            glVertex3dv(p2[3]);
            glVertex3dv(p2[1]);
            glVertex3dv(p2[5]);

            glVertex3dv(p2[2]);
            glVertex3dv(p2[3]);
            glVertex3dv(p2[2]);
            glVertex3dv(p2[6]);
            glVertex3dv(p2[3]);
            glVertex3dv(p2[7]);
            glVertex3dv(p2[4]);
            glVertex3dv(p2[5]);
            glVertex3dv(p2[4]);
            glVertex3dv(p2[6]);

            glVertex3dv(p2[3]);
            glVertex3dv(p2[7]);
            glVertex3dv(p2[4]);
            glVertex3dv(p2[5]);
            glVertex3dv(p2[4]);
            glVertex3dv(p2[6]);
            glVertex3dv(p2[5]);
            glVertex3dv(p2[7]);
            glVertex3dv(p2[6]);
            glVertex3dv(p2[7]);
            glEnd();
            glEnable(GL_LIGHTING);
        }
        pqp_draw_OBBs_recursive(object, M3, bv->first_child, currentLevel+1, level);
        pqp_draw_OBBs_recursive(object, M3, bv->first_child+1, currentLevel+1, level);
    }
}

//! Draws the OBBs (Oriented Bounding Boxes), computed by PQP, of the given object at the given level
//! in the OBB tree hierarchy.
//! Use this function in an OpenGL display function.
void pqp_draw_OBBs(p3d_obj *object, int level)
{
    double theta;
    double M[4][4];
    p3d_vector3 axis;
    p3d_matrix4 pose;

    #ifdef PQP_DEBUG
    if(object==NULL)
    {
        printf("%s: %d: pqp_draw_OBBs(): argument 1 (p3d_obj *) is NULL.\n", __FILE__, __LINE__);
        return;
    }
    if(object->pqpModel==NULL)
    {
        printf("%s: %d: pqp_draw_OBBs(): the p3d_obj variable (%s) has no PQP model.\n", __FILE__, __LINE__, object->name);
        return;
    }
    #endif

    M[0][0]= 1;
    M[0][1]= 0;
    M[0][2]= 0;
    M[0][3]= 0;
    M[1][0]= 0;
    M[1][1]= 1;
    M[1][2]= 0;
    M[1][3]= 0;
    M[2][0]= 0;
    M[2][1]= 0;
    M[2][2]= 1;
    M[2][3]= 0;
    M[3][0]= 0;
    M[3][1]= 0;
    M[3][2]= 0;
    M[3][3]= 1;

    pqp_get_obj_pos(object, pose);
    p3d_mat4ExtractRot(pose, axis, &theta);

    glPushMatrix();
    glTranslatef(pose[0][3], pose[1][3], pose[2][3]);
    glRotatef(theta*(180.0/M_PI),axis[0],axis[1],axis[2]);
    pqp_draw_OBBs_recursive(object, M, 0, 0, level);
    glPopMatrix();

}

//! Draws the OBBs (Oriented Bounding Boxes), computed by PQP,
//! of all the environment objects (obstacles and robot bodies) at the given level
//! in the OBB tree hierarchy.
//! Use this function in an OpenGL display function.
void pqp_draw_all_OBBs(int level)
{
    int i, ir;
    p3d_obj *object= NULL;
    p3d_rob *robot= NULL;

    for (i=0; i<XYZ_ENV->no; i++)
    {
        object= XYZ_ENV->o[i];
        pqp_draw_OBBs(object, level);
    }

    for (ir=0; ir<XYZ_ENV->nr; ir++)
    {
        robot= XYZ_ENV->robot[ir];
        for (i=0; i<robot->no; i++)
        {
            object= robot->o[i];
            pqp_draw_OBBs(object, level);
        }

    }
}


//! Computes a vector that is orthogonal to the vector v and normalizes it.
//! The function returns an arbitrary choice for the orthogonal vector.
void pqp_orthogonal_vector(p3d_vector3 v, p3d_vector3 result)
{
    #ifdef PQP_DEBUG
    if( p3d_vectNorm(v) < EPSILON )
    {
        printf("%s: %d: pqp_orthogonal_vector(): bad input (vector norm is null).\n",__FILE__,__LINE__);
        result[0]= 1;
        result[1]= 0;
        result[2]= 0;
        return;
    }
    #endif

    if( fabs(v[2]) <= EPSILON )
    {
        result[0]= 0;
        result[1]= 0;
        result[2]= 1;
        return;
    }
    else
    {
        result[0]= 0;
        result[1]= 1;
        result[2]= -v[1]/v[2];
        p3d_vectNormalize(result, result);
        return;
    }
}

//! Computes the vectors v and w such as (u,v,w) is a direct orthonormal base.
//! The function returns an arbitrary choice.
void pqp_orthonormal_basis(p3d_vector3 u, p3d_vector3 v, p3d_vector3 w)
{
    pqp_orthogonal_vector(u, v);
    p3d_vectXprod(u, v, w);
    p3d_vectNormalize(w, w);
}

//! Checks the collision between two p3d_obj variables.
//! Returns 1 if there is a collision, 0 otherwise.
//! This is the direct interface to PQP collision test function.
int pqp_collision_test(p3d_obj *o1, p3d_obj *o2)
{
    #ifdef PQP_DEBUG
    if(o1==NULL || o2==NULL)
    {
        printf("%s: %d: pqp_collision_test(): NULL input(s) (%p %p).\n",__FILE__,__LINE__,o1,o2);
        return 0;
    }
    if(o1->pqpModel==NULL)
    {
        printf("%s: %d: pqp_collision_test(): an input has no PQP model (%s).\n",__FILE__,__LINE__,o1->name);
        return 0;
    }
    if(o2->pqpModel==NULL)
    {
        printf("%s: %d: pqp_collision_test(): an input has no PQP model (%s).\n",__FILE__,__LINE__,o2->name);
        return 0;
    }
    #endif

    PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
    p3d_matrix4 pose1, pose2;
    PQP_CollideResult cres;

    pqp_get_obj_pos( o1, pose1 );
    pqp_get_obj_pos( o2, pose2 );

    R1[0][0]= pose1[0][0];
    R1[0][1]= pose1[0][1];
    R1[0][2]= pose1[0][2];
    R1[1][0]= pose1[1][0];
    R1[1][1]= pose1[1][1];
    R1[1][2]= pose1[1][2];
    R1[2][0]= pose1[2][0];
    R1[2][1]= pose1[2][1];
    R1[2][2]= pose1[2][2];

    T1[0] = pose1[0][3];
    T1[1] = pose1[1][3];
    T1[2] = pose1[2][3];

    R2[0][0]= pose2[0][0];
    R2[0][1]= pose2[0][1];
    R2[0][2]= pose2[0][2];
    R2[1][0]= pose2[1][0];
    R2[1][1]= pose2[1][1];
    R2[1][2]= pose2[1][2];
    R2[2][0]= pose2[2][0];
    R2[2][1]= pose2[2][1];
    R2[2][2]= pose2[2][2];

    T2[0] = pose2[0][3];
    T2[1] = pose2[1][3];
    T2[2] = pose2[2][3];

    PQP_Collide(&cres, R1, T1, o1->pqpModel, R2, T2, o2->pqpModel, PQP_FIRST_CONTACT);

   if(cres.NumPairs()!=0)
   {
     pqp_COLLISION_PAIRS.colliding_body1= o1;
     pqp_COLLISION_PAIRS.colliding_body2= o2;
   }

    return cres.NumPairs();
}


//! This function computes the distance between two p3d_obj variable.
//! In case there is collision between the bodies, it returns 0.
//! It copies in closest_point1 and closest_point2, the positions of the closest points
//! on body 1 and body 2 respectively.
//! This is the direct interface to PQP distance computation function.
double pqp_distance(p3d_obj *o1, p3d_obj *o2, p3d_vector3 closest_point1, p3d_vector3 closest_point2)
{
    #ifdef PQP_DEBUG
    if(o1==NULL || o2==NULL)
    {
        printf("%s: %d: pqp_distance(): one or more input is NULL (%p %p).\n",__FILE__,__LINE__,o1,o2);
        return 0;
    }
    if(o1->pqpModel==NULL)
    {
        printf("%s: %d: pqp_distance(): an input has no PQP model (%s).\n",__FILE__,__LINE__,o1->name);
        return 0;
    }
    if(o2->pqpModel==NULL)
    {
        printf("%s: %d: pqp_collision_test(): an input has no PQP model (%s).\n",__FILE__,__LINE__,o2->name);
        return 0;
    }
    #endif

    PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
    p3d_vector3 p1_rel, p2_rel;
    p3d_matrix4 pose1, pose2;
    PQP_DistanceResult dres;

    pqp_get_obj_pos( o1, pose1 );
    pqp_get_obj_pos( o2, pose2 );

    R1[0][0]= pose1[0][0];
    R1[0][1]= pose1[0][1];
    R1[0][2]= pose1[0][2];
    R1[1][0]= pose1[1][0];
    R1[1][1]= pose1[1][1];
    R1[1][2]= pose1[1][2];
    R1[2][0]= pose1[2][0];
    R1[2][1]= pose1[2][1];
    R1[2][2]= pose1[2][2];

    T1[0] = pose1[0][3];
    T1[1] = pose1[1][3];
    T1[2] = pose1[2][3];

    R2[0][0]= pose2[0][0];
    R2[0][1]= pose2[0][1];
    R2[0][2]= pose2[0][2];
    R2[1][0]= pose2[1][0];
    R2[1][1]= pose2[1][1];
    R2[1][2]= pose2[1][2];
    R2[2][0]= pose2[2][0];
    R2[2][1]= pose2[2][1];
    R2[2][2]= pose2[2][2];

    T2[0] = pose2[0][3];
    T2[1] = pose2[1][3];
    T2[2] = pose2[2][3];

    PQP_Distance(&dres, R1, T1, o1->pqpModel, R2, T2, o2->pqpModel, 0.0, 0.0);

    //These coordinates are given in the local frame of each body:
    p1_rel[0]= dres.p1[0];
    p1_rel[1]= dres.p1[1];
    p1_rel[2]= dres.p1[2];
    p2_rel[0]= dres.p2[0];
    p2_rel[1]= dres.p2[1];
    p2_rel[2]= dres.p2[2];


    //Compute the global coordinates of the closest points:
    closest_point1[0]= R1[0][0]*p1_rel[0] + R1[0][1]*p1_rel[1] + R1[0][2]*p1_rel[2] + T1[0];
    closest_point1[1]= R1[1][0]*p1_rel[0] + R1[1][1]*p1_rel[1] + R1[1][2]*p1_rel[2] + T1[1];
    closest_point1[2]= R1[2][0]*p1_rel[0] + R1[2][1]*p1_rel[1] + R1[2][2]*p1_rel[2] + T1[2];

    closest_point2[0]= R2[0][0]*p2_rel[0] + R2[0][1]*p2_rel[1] + R2[0][2]*p2_rel[2] + T2[0];
    closest_point2[1]= R2[1][0]*p2_rel[0] + R2[1][1]*p2_rel[1] + R2[1][2]*p2_rel[2] + T2[1];
    closest_point2[2]= R2[2][0]*p2_rel[0] + R2[2][1]*p2_rel[1] + R2[2][2]*p2_rel[2] + T2[2];


    return ((double) dres.distance);
}


//! Tests all the self-collisions of the given robot.
//! Return 1 in case of collision, 0 otherwise.
int pqp_robot_selfcollision_test(p3d_rob *robot)
{
    #ifdef PQP_DEBUG
    if(robot==NULL)
    {
        printf("%s: %d: pqp_robot_selfcollision_test(): input is NULL.\n",__FILE__,__LINE__);
        return 0;
    }
    if(pqp_COLLISION_PAIRS.obj_obj==NULL)
    {
     printf("%s: %d: pqp_robot_selfcollision_test(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
    }
    #endif

    int i, j, nb_cols;
    p3d_obj *body1, *body2;

    for (i=0; i<robot->no; i++)
    {
        body1= robot->o[i];

        if(body1->pqpModel==NULL)
        {  continue;  }

        for (j=i+1; j<robot->no; j++)
        {
            body2= robot->o[j];

            if(body2->pqpModel==NULL)
            {  continue;  }

            #ifdef PQP_DEBUG
            if(body1->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
            {               
              printf("%s: %d: pqp_robot_selfcollision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body1->name);
              return 0;
            }
            if(body2->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
            {               
              printf("%s: %d: pqp_robot_selfcollision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body2->name);
              return 0;
            }
            #endif



            if(pqp_COLLISION_PAIRS.obj_obj[body1->pqpID][body2->pqpID]==0)
            {  continue;  }

            // in theory, it should not have to be tested
            if(body1->pqpPreviousBody==body2 || body2->pqpPreviousBody==body1)
            {  continue;  }


            nb_cols= pqp_collision_test(body1, body2);

            if(nb_cols!=0)
            {
               if(pqp_COLLISION_MESSAGE)
               {
                 printf("pqp_robot_selfcollision_test(): robot %s: self-collision between %s and %s\n",robot->name,body1->name, body2->name);
               }
               return 1;
            }
        }
    }

    return 0;
}


//! Tests all the collisions between the given robot and the environment obstacles.
//! Returns 1 in case of collision, 0 otherwise.
int pqp_robot_environment_collision_test(p3d_rob *robot)
{
  #ifdef PQP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: pqp_robot_environment_collision_test(): input is NULL.\n",__FILE__,__LINE__);
    return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
    printf("%s: %d: pqp_robot_environment_collision_test(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
    return 0;    
  }
  #endif

  int i, j, nb_cols;

  for(i=0; i<robot->no; i++)
  {
     if(robot->o[i]->pqpModel==NULL)
     {  continue;  }

     #ifdef PQP_DEBUG
     if(robot->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
     {               
       printf("%s: %d: pqp_robot_environment_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot->o[i]->name);
       return 0;
     }
     #endif     

     for(j=0; j<XYZ_ENV->no; j++)
     {
       if(XYZ_ENV->o[j]->pqpModel==NULL)
       {  continue;  }

       #ifdef PQP_DEBUG
       if(XYZ_ENV->o[j]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
       {               
         printf("%s: %d: pqp_robot_environment_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,XYZ_ENV->o[j]->name);
         return 0;
       }
       #endif  

       if(pqp_COLLISION_PAIRS.obj_obj[robot->o[i]->pqpID][XYZ_ENV->o[j]->pqpID]==0)
       {  continue;  }

       nb_cols= pqp_collision_test(robot->o[i], XYZ_ENV->o[j]);
       if(nb_cols!=0)
       {
         if(pqp_COLLISION_MESSAGE)
         {
           printf("pqp_robot_environment_collision_test(): collision between \"%s\" and \"%s\"\n",robot->o[i]->name, XYZ_ENV->o[j]->name);
         }
         return 1;
       }

     }
  }

  return 0;
}

//! Tests all the collisions between the bodies of the two given robots.
//! Returns 1 in case of collision, 0 otherwise.
int pqp_robot_robot_collision_test(p3d_rob *robot1, p3d_rob *robot2)
{
  #ifdef PQP_DEBUG
  if(robot1==NULL || robot2==NULL)
  {
     printf("%s: %d: pqp_robot_robot_collision_test(): one input or more is NULL (%p %p).\n",__FILE__,__LINE__,robot1,robot2);
     return 0;
  }
  if(pqp_COLLISION_PAIRS.obj_obj==NULL)
  {
     printf("%s: %d: pqp_robot_robot_collision_test(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
  }
  #endif

  int i, j, nb_cols;

  for(i=0; i<robot1->no; i++)
  {
    if(robot1->o[i]->pqpModel==NULL)
    {   continue;   }

    for(j=0; j<robot2->no; j++)
    {
       if(robot2->o[j]->pqpModel==NULL)
       {   continue;   }

       #ifdef PQP_DEBUG
       if(robot1->o[i]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
       {               
         printf("%s: %d: pqp_robot_robot_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot1->o[i]->name);
         return 0;
       }
       if(robot2->o[j]->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
       {               
         printf("%s: %d: pqp_robot_robot_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,robot2->o[j]->name);
         return 0;
       }
       #endif  

       if(pqp_COLLISION_PAIRS.obj_obj[robot1->o[i]->pqpID][robot2->o[j]->pqpID]==0)
         continue;

       nb_cols= pqp_collision_test(robot1->o[i], robot2->o[j]);
       if(nb_cols!=0)
       {
         if(pqp_COLLISION_MESSAGE)
         {
           printf("pqp_robot_robot_collision_test(): collision between robots \"%s\" and \"%s\" (bodies \"%s\" and \"%s\")\n", robot1->name, robot2->name, robot1->o[i]->name, robot2->o[j]->name);
         }
         return 1;
       }
    }
  }

  return 0;
}


//! Tests the collisions between the given robot and the given object (that can be a body of the robot).
//! Returns 1 in case of collision, 0 otherwise.
int pqp_robot_obj_collision_test(p3d_rob *robot, p3d_obj *obj)
{
    #ifdef PQP_DEBUG
    if(robot==NULL)
    {
      printf("%s: %d: pqp_robot_obj_collision_test(): argument 1 is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(obj==NULL)
    {
      printf("%s: %d: pqp_robot_obj_collision_test(): argument 2 is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(pqp_COLLISION_PAIRS.obj_obj==NULL)
    {
      printf("%s: %d: pqp_robot_obj_collision_test(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
    }
    if(obj->pqpModel==NULL)
    {               
      printf("%s: %d: pqp_robot_obj_collision_test(): objetc %s has no PQP model.\n",__FILE__,__LINE__,obj->name);
      return 0;
    }
    if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {               
      printf("%s: %d: pqp_robot_obj_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
      return 0;
    }
    #endif

    int i, nb_cols;
    p3d_obj *body= NULL;

    for(i=0; i<robot->no; i++)
    {
       body= robot->o[i];
       if(body->pqpModel==NULL)
       {  continue;  }

       if(body==obj)
       {  continue;  }

       #ifdef PQP_DEBUG
       if(body->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
       {               
         printf("%s: %d: pqp_robot_obj_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,body->name);
         return 0;
       }
       #endif

       if(pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][body->pqpID]==0)
       {  continue;  }

       nb_cols= pqp_collision_test(body, obj);
       if(nb_cols!=0)
        {
         if(pqp_COLLISION_MESSAGE)
         {
           printf("pqp_robot_obj_collision_test(): collision between %s and %s\n", body->name, obj->name);
         }
         return 1;
       }

    }

    return 0;
}


//! Tests all the collisions between the given object and the environment obstacles.
//! Returns 1 in case of collision, 0 otherwise.
int pqp_obj_environment_collision_test(p3d_obj *obj)
{
    #ifdef PQP_DEBUG
    if(obj==NULL)
    {
      printf("%s: %d: pqp_obj_environment_collision_test(): the input is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(obj->pqpModel==NULL)
    {
      printf("%s: %d: pqp_obj_environment_collision_test(): object %s has no PQP model.\n",__FILE__,__LINE__,obj->name);
      return 0;
    }
    if(pqp_COLLISION_PAIRS.obj_obj==NULL)
    {
      printf("%s: %d: pqp_obj_environment_collision_test(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
    }
    if(obj->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
    {               
      printf("%s: %d: pqp_obj_environment_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obj->name);
      return 0;
    }
    #endif

    int i, nb_cols;
    p3d_obj *obst= NULL;

    for (i=0; i<XYZ_ENV->no; i++)
    {
        obst= XYZ_ENV->o[i];
        if(obst==obj)
          continue;

        #ifdef PQP_DEBUG
        if(obst->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
        {               
          printf("%s: %d: pqp_obj_environment_collision_test(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obst->name);
          return 0;
        }
        #endif

        if(pqp_COLLISION_PAIRS.obj_obj[obj->pqpID][obst->pqpID]==0)
           continue;

        nb_cols= pqp_collision_test(obj, obst);
        if(nb_cols!=0)
        {
          if(pqp_COLLISION_MESSAGE)
          {
            printf("pqp_obj_environment_collision_test(): collision between %s and %s\n", obj->name, obst->name);
          }
          return 1;
        }

    }

    return 0;
}

//! Performs all the collision tests (robot-robot, robot-environment and robot self-collisions)
//! for the given robot only.
//! Returns 1 in case of collision, 0 otherwise.
int pqp_robot_all_collision_test(p3d_rob *robot)
{
 #ifdef PQP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: pqp_robot_all_collision_test(): the input is NULL.\n",__FILE__,__LINE__);
    return 0;
  }
 #endif

  int i, j, nb_cols;

  //robot-robot collisions:
  for(i=0; i<XYZ_ENV->nr; i++)
  {
    for(j=i+1; j<XYZ_ENV->nr; j++)
    {  
      nb_cols= pqp_robot_robot_collision_test(XYZ_ENV->robot[i], XYZ_ENV->robot[j]);
      if(nb_cols!=0)
        return 1;
    }
  }

  //robot-environment collisions:
  nb_cols= pqp_robot_environment_collision_test(robot);
  if(nb_cols!=0)
    return 1;

  //robot self collisions:
  nb_cols= pqp_robot_selfcollision_test(robot);
  if(nb_cols!=0)
     return 1;
  
  return 0;
}


//! Performs all the collision tests (robot-robot, robot-environment and robot self-collisions).
//! Returns 1 in case of collision, 0 otherwise.
int pqp_all_collision_test()
{
  int ir1, ir2, nb_cols;
  p3d_rob *robot= NULL;

  for(ir1=0; ir1<XYZ_ENV->nr; ir1++)
  {
    robot= XYZ_ENV->robot[ir1];

    //robot-robot collisions:
    for(ir2=ir1+1; ir2<XYZ_ENV->nr; ir2++)
    {
      nb_cols= pqp_robot_robot_collision_test(robot, XYZ_ENV->robot[ir2]);
      if(nb_cols!=0)
        return 1;
    }

    //robot-environment collisions:
    nb_cols= pqp_robot_environment_collision_test(robot);
    if(nb_cols!=0)
      return 1;

    //robot self collisions:
    nb_cols= pqp_robot_selfcollision_test(robot);
    if(nb_cols!=0)
      return 1;
  }

  return 0;
}

//! This function computes the minimal distance between the robot and the environment obstacles.
//! The closest points on the robot surface and the closest obstacle surface are copied in
//! closest_point_rob and closest_point_obst.
//! In case of collision, it returns 0.
double pqp_robot_environment_distance(p3d_rob *robot, p3d_vector3 closest_point_rob, p3d_vector3 closest_point_obst)
{
    #ifdef PQP_DEBUG
    if(robot==NULL)
    {
      printf("%s: %d: pqp_robot_environment_distance(): input is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(pqp_COLLISION_PAIRS.obj_obj==NULL)
    {
      printf("%s: %d: pqp_robot_environment_distance(): the function pqp_create_collision_pairs() has not been called.\n",__FILE__,__LINE__);
      return 0;    
    }
    #endif


    int i, j;
    double distance, distance_min= 1e9;
    p3d_vector3 p1, p2;
    p3d_obj *body, *obst;

    for (i=0; i<robot->no; i++)
    {
        body= robot->o[i];
        #ifdef PQP_DEBUG
        if(body->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
        {               
          printf("%s: %d: pqp_robot_environment_distance(): the pqpID of \"%s\" is inconsistent with the    pqp_collision_grid.\n",__FILE__,__LINE__,body->name);
          return 0;
        }
        #endif

        for (j=0; j<XYZ_ENV->no; j++)
        {
            #ifdef PQP_DEBUG
            if(obst->pqpID >= pqp_COLLISION_PAIRS.nb_objs)
            {               
             printf("%s: %d: pqp_robot_environment_distance(): the pqpID of \"%s\" is inconsistent with the pqp_collision_grid.\n",__FILE__,__LINE__,obst->name);
             return 0;
            }
            #endif

            if(pqp_COLLISION_PAIRS.obj_obj[body->pqpID][obst->pqpID]==0)
              continue;

            obst= XYZ_ENV->o[j];
            distance= pqp_distance(body, obst, p1, p2);

            if(distance<=0)
            {
                return 0;
            }

            if(distance < distance_min)
            {
                distance_min= distance;
                p3d_vectCopy(p1, closest_point_rob);
                p3d_vectCopy(p2, closest_point_obst);
            }
        }
    }

    return distance_min;
}


//! This function returns 1 if the distance between the two bodies is <= tolerance,
//! 0 otherwise.
//! In most query calling this function is faster than calling pqp_distance() and then comparing the result
//! to tolerance. 
int pqp_tolerance(p3d_obj *o1, p3d_obj *o2, double tolerance)
{
    #ifdef PQP_DEBUG
    if(o1==NULL || o2==NULL)
    {
      printf("%s: %d: pqp_tolerance(): NULL input(s) (%p %p).\n",__FILE__,__LINE__,o1,o2);
      return 0;
    }
    if(o1->pqpModel==NULL)
    {
      printf("%s: %d: pqp_tolerance(): an input has no PQP model (%s).\n",__FILE__,__LINE__,o1->name);
      return 0;
    }
    if(o2->pqpModel==NULL)
    {
      printf("%s: %d: pqp_tolerance(): an input has no PQP model (%s).\n",__FILE__,__LINE__,o2->name);
      return 0;
    }
    #endif


    PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
    PQP_REAL tol= tolerance;
    p3d_matrix4 pose1, pose2;
    PQP_ToleranceResult tres;

    pqp_get_obj_pos( o1, pose1 );
    pqp_get_obj_pos( o2, pose2 );

    R1[0][0]= pose1[0][0];
    R1[0][1]= pose1[0][1];
    R1[0][2]= pose1[0][2];
    R1[1][0]= pose1[1][0];
    R1[1][1]= pose1[1][1];
    R1[1][2]= pose1[1][2];
    R1[2][0]= pose1[2][0];
    R1[2][1]= pose1[2][1];
    R1[2][2]= pose1[2][2];

    T1[0] = pose1[0][3];
    T1[1] = pose1[1][3];
    T1[2] = pose1[2][3];

    R2[0][0]= pose2[0][0];
    R2[0][1]= pose2[0][1];
    R2[0][2]= pose2[0][2];
    R2[1][0]= pose2[1][0];
    R2[1][1]= pose2[1][1];
    R2[1][2]= pose2[1][2];
    R2[2][0]= pose2[2][0];
    R2[2][1]= pose2[2][1];
    R2[2][2]= pose2[2][2];

    T2[0] = pose2[0][3];
    T2[1] = pose2[1][3];
    T2[2] = pose2[2][3];


    PQP_Tolerance(&tres, R1, T1, o1->pqpModel, R2, T2, o2->pqpModel, tol);

    return tres.CloserThanTolerance();
}

//! Copies in o1 and o2 the addresses of the two objects that were reported as colliding during
//! the last collision test. This function must be called after a positive collision test.
int pqp_colliding_pair(p3d_obj **o1, p3d_obj **o2)
{
   #ifdef PQP_DEBUG
   if(o1==NULL || o2==NULL)
   {
     printf("%s: %d: pqp_colliding_pair(): NULL input(s) (%p %p).\n",__FILE__,__LINE__,o1, o2);
     return 0;
   }
   #endif

   if(pqp_COLLISION_PAIRS.colliding_body1==NULL || pqp_COLLISION_PAIRS.colliding_body2==NULL)
   {
     return 0;
   }

   *o1= pqp_COLLISION_PAIRS.colliding_body1;
   *o2= pqp_COLLISION_PAIRS.colliding_body2;

   return 1;
}
#endif


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#include "proto/hri_gik_proto.h"

hri_gik * HRI_GIK = NULL;
int HRI_GIK_CONTINUOUS = TRUE;
int GIK_VIS = 100;

/*******************************************************************/
/****************** GIK FUCTIONS ***********************************/
/*******************************************************************/
#ifndef HRI_PLANNER
double p3d_psp_pointtolinedist(p3d_vector3 p, p3d_vector3 l1, p3d_vector3 l2)
{
	p3d_vector3 l1mp;
	p3d_vector3 l2ml1;
	p3d_vector3 crossprod;
	
	p3d_vectSub(l1,p,l1mp);
	p3d_vectSub(l2,l1,l2ml1);
	p3d_vectXprod(l1mp,l2ml1,crossprod);
	
	return (p3d_vectNorm(crossprod)/p3d_vectNorm(l2ml1));
}
#endif
/****************************************************************/
/*!
 * \brief Creates a Generelized Inverse Kinematics Structure
 *
 * !

 * \return NULL in case of a problem
 */
/****************************************************************/
hri_gik * hri_gik_create_gik()
{
  hri_gik * gik;

  gik = MY_ALLOC(hri_gik, 1);
  if(gik == NULL)
    return NULL;
  gik->task_no = 0;
  gik->task = NULL;
  gik->robot = NULL;
  gik->GIKInitialized = FALSE;

  return gik;
}

/****************************************************************/
/*!
 * \brief Creates a GIK Task
 *
 * !

 * \return NULL in case of a problem
 */
/****************************************************************/
hri_gik_task * hri_gik_create_task()
{
  hri_gik_task * task;

  task = MY_ALLOC(hri_gik_task, 1);

  if(task == NULL)
    return NULL;
  task->m = 0;
  task->n = 0;
  task->jnt_no = 0;
  task->Jacobian = NULL;
  task->PsInvJacobianWoS = NULL;
  task->PsInvJacobianWS = NULL;
  task->PJ = NULL;
  task->deltaTheta = NULL;
  task->deltaX = NULL;
  task->jnt = NULL;
  task->initialized = FALSE;
  task->priority = 99;

  return task;
}

/****************************************************************/
/*!
 * \brief Destroys the data in a GIK structure
 *
 * \param gik   gik structure
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_destroy_gik_data(hri_gik * gik)
{
  int i;

  if(gik == NULL){
    PrintError(("Cant destroy GIK: gik=null"));
    return FALSE;
  }
  if( !gik->GIKInitialized )
    return TRUE;

  for(i=0; i<gik->task_no; i++){
    hri_gik_destroy_task(gik->task[i]);
  }
  gik->task = NULL;
  gik->task_no = 0;
  gik->GIKInitialized = FALSE;

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Initialize a GIK structure
 *
 * \param gik             gik structure
 * \param robot           the robot
 * \param total_joint_no  number of joints involved in gik
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_initialize_gik(hri_gik * gik, p3d_rob * robot, int total_joint_no)
{
  if(gik == NULL || robot == NULL){
    PrintError(("Cant initialize GIK: gik or robot =null"));
    return FALSE;
  }

  gik->robot = robot;
  gik->joints = MY_ALLOC(p3d_jnt *, total_joint_no);
  gik->free_joints = MY_ALLOC(int, total_joint_no);
  gik->joint_no = total_joint_no;

  gik->GIKInitialized = TRUE;

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Removes the inside of a GIK structure
 *
 * \param gik             gik structure
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_uninitialize_gik(hri_gik * gik)
{
  if(gik == NULL){
    PrintError(("GIK is already destroyed\n"));
    return TRUE;
  }

  MY_FREE(gik->joints,p3d_jnt *,gik->joint_no);
  MY_FREE(gik->free_joints,int ,gik->joint_no);
  gik->joint_no = 0;

  gik->GIKInitialized = FALSE;

  return TRUE;
}



/****************************************************************/
/*!
 * \brief Adds a task to GIK structure
 *
 * \param gik        gik structure
 * \param m          number of constraints
 * \param n          number of controls
 * \param jindexes   array of involved joint numbers
 * \param eef_no     target joint no
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_add_task(hri_gik * gik, int m, int n, int priority, int * jindexes, int eef_no)
{
  hri_gik_task ** temp;
  int temp_length;
  int i=0;

  if(gik == NULL || !gik->GIKInitialized){
    PrintError(("Cant add task: gik =null"));
    return FALSE;
  }
  if(jindexes == NULL){
    PrintWarning(("No joints in task: gik =null, ignoring..."));
    return TRUE;
  }

  if(gik->task_no == 0 || gik->task == NULL){
    gik->task = MY_ALLOC(hri_gik_task*,1);
    gik->task[0] = hri_gik_create_task();
    gik->task_no = 1;
    hri_gik_initialize_task(gik,gik->task[0],m,n,priority,jindexes,eef_no);
  }
  else{
    temp = gik->task;
    temp_length = gik->task_no;

    gik->task = MY_ALLOC(hri_gik_task*, gik->task_no+1);
    gik->task_no++;

    while(i<temp_length){
      if(temp[i]->priority < priority){
        gik->task[i] = temp[i];
      }
      else
        break;
      i++;
    }
    gik->task[i] = hri_gik_create_task();
    hri_gik_initialize_task(gik,gik->task[i],m,n,priority,jindexes,eef_no);
    i++;
    while(i<gik->task_no){
      gik->task[i] = temp[i-1];
      i++;
    }

    MY_FREE(temp,hri_gik_task*,temp_length);
  }
  return TRUE;

}

/****************************************************************/
/*!
 * \brief Initialize a task
 * \param gik        gik structure
 * \param task       task structure
 * \param m          number of constraints
 * \param n          number of controls
 * \param priority   priority of the task, lower is prior
 * \param jindexes   array of involved joint numbers
 * \param eef_no     target joint no
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_initialize_task(hri_gik * gik, hri_gik_task * task , int m, int n, int priority, int * jindexes, int eef_no)
{
  int i,j;

  if(gik == NULL){
    PrintError(("Cant initialize GIK: gik=null"));
    return FALSE;
  }

  if(task->initialized){
    PrintWarning(("Cant Initialize a task already initialized, ignoring..."));
    return TRUE;
  }
  task->jnt_no = 0;
  for(i=0; i<n; i++)
    if(jindexes[i] != 0)
      task->jnt_no++;
  task->n = n;
  task->m = m;
  task->Jacobian = gsl_matrix_alloc(m,n);
  task->PsInvJacobianWoS = gsl_matrix_alloc(n,m);
  task->PsInvJacobianWS = gsl_matrix_alloc(n,m);
  task->PJ = gsl_matrix_alloc(n,n);
  task->deltaTheta = gsl_vector_alloc(n);
  task->deltaX = gsl_vector_alloc(m);
  task->goal = gsl_vector_alloc(m);
  task->jnt = MY_ALLOC(hri_gik_joint_info*, task->jnt_no );
  for(i=0,j=0; i<n; i++){
    if(jindexes[i] == 0)
      continue;
    task->jnt[j] = MY_ALLOC(hri_gik_joint_info, 1 );
    task->jnt[j]->joint = gik->robot->joints[jindexes[i]];
    task->jnt[j]->no_dof = gik->robot->joints[jindexes[i]]->dof_equiv_nbr;
    task->jnt[j]->index = i;
    gik->joints[i] = task->jnt[j]->joint;
    gik->free_joints[i] = TRUE;
    j++;
  }
  task->eef_no = eef_no;
  task->initialized = TRUE;
  task->priority = priority;

  return TRUE;

}

/****************************************************************/
/*!
 * \brief Destroys a task structure
 *
 * \param task   task structure
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_destroy_task(hri_gik_task * task)
{
  if(task != NULL){
    gsl_matrix_free(task->Jacobian);
    gsl_matrix_free(task->PsInvJacobianWoS);
    gsl_matrix_free(task->PsInvJacobianWS);
    gsl_matrix_free(task->PJ);
    gsl_vector_free(task->deltaTheta);
    gsl_vector_free(task->deltaX);
    MY_FREE(task->jnt, hri_gik_joint_info*,task->jnt_no );
    task->jnt_no = 0;
  }

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a GIK structure
 *
 * \param gik   gik structure
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_destroy_gik(hri_gik * gik)
{
  if( hri_gik_destroy_gik_data(gik) == FALSE )
    return FALSE;

  MY_FREE( gik->task,hri_gik_task*,gik->task_no);
  MY_FREE(gik,hri_gik,1);
  return TRUE;
}


/****************************************************************/
/*!
 * \brief Compute the Jacobian matrix of the task and writes to gik->Jacobian
 *
 * \param gik      gik structure
 * \param task_no  task number
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_computeJacobian(hri_gik * gik, int task_no, int rotation)
{
  int i,j,k=0;
  double J[gik->task[task_no]->m][gik->task[task_no]->n], p[3],z[3];
  hri_gik_task * task;

  if(gik == NULL || !gik->GIKInitialized ||
     gik->task[task_no] == NULL || !gik->task[task_no]->initialized){
    PrintError(("Cant compute jacobian : gik=null"));
    return FALSE;
  }
  task = gik->task[task_no];

  for(i=0; i<task->m; i++)
    for(j=0; j<task->n; j++)
      J[i][j] = 0;

  for(j=0; j<task->jnt_no; j++){

    for(i=0; i<3; i++)
      p[i] = gik->robot->joints[task->eef_no]->abs_pos[i][3] - task->jnt[j]->joint->abs_pos[i][3];

    for(i=0; i<3; i++)
      z[i] = task->jnt[j]->joint->abs_pos[i][2];

    if(task->jnt[j]->joint->type == P3D_ROTATE){
      J[0][task->jnt[j]->index] = z[1]*p[2]-z[2]*p[1];
      J[1][task->jnt[j]->index] = z[2]*p[0]-z[0]*p[2];
      J[2][task->jnt[j]->index] = z[0]*p[1]-z[1]*p[0];
      if(isnan(J[0][task->jnt[j]->index])){
        printf("NAN VALUE\n");
      }
      if(rotation){
        J[3][k]=z[0];
        J[4][k]=z[1];  /* Rotation */
        J[5][k]=z[2];
      }
    }
    if(task->jnt[j]->joint->type == P3D_TRANSLATE){
      J[0][task->jnt[j]->index] = z[0];
      J[1][task->jnt[j]->index] = z[1];
      J[2][task->jnt[j]->index] = z[2];
    }
    if(task->jnt[j]->joint->type == P3D_PLAN){
      J[0][task->jnt[j]->index] = 1;
      J[1][task->jnt[j]->index] = 0;
      J[2][task->jnt[j]->index] = 0;
      if(rotation){
        J[3][k]=0;
        J[4][k]=0;  /* Rotation */
        J[5][k]=0;
        k++;
      }
      J[0][task->jnt[j]->index+1] = 0;
      J[1][task->jnt[j]->index+1] = 1;
      J[2][task->jnt[j]->index+1] = 0;
      if(rotation){
        J[3][k]=0;
        J[4][k]=0;  /* Rotation */
        J[5][k]=0;
        k++;
      }

      for(i=0; i<3; i++)
        z[i] =task->jnt[j]->joint->abs_pos[i][2];

      J[0][task->jnt[j]->index+2]=z[1]*p[2]-z[2]*p[1];
      J[1][task->jnt[j]->index+2]=z[2]*p[0]-z[0]*p[2];
      J[2][task->jnt[j]->index+2]=z[0]*p[1]-z[1]*p[0];
      if(rotation){
        J[3][k]=z[0];
        J[4][k]=z[1];   /* Rotation */
        J[5][k]=z[2];
      }
    }
    k++;

  }

  for(i=0; i<gik->task[task_no]->m; i++)
    for(j=0; j<gik->task[task_no]->n; j++)
      gsl_matrix_set(gik->task[task_no]->Jacobian, i, j, J[i][j]);

  //  hri_gik_ShowTheMatrix(gik->task[task_no]->Jacobian);

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Computes PseudeInvere of Jacobian matrix and writes to gik->PsInvJacobianWoS
 *        wich avoids singularities and gik->PsInvJacobianWS which contains singularities
 *
 * \param gik     gik structure
 * \param task_no task_number
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_computePsInvJacobian(hri_gik * gik, int task_no)
{
  unsigned int i;
  double delta;
  gsl_vector * SWoS;
  gsl_vector * SWS;
  gsl_matrix * V;
  gsl_matrix * S_sqr;
  gsl_matrix * Temp;
  gsl_matrix * JackT;
  gsl_matrix * PsiJackT;
  gsl_vector * workSpace;
  gsl_matrix * U_T;

  if(gik == NULL){
    PrintError(("Cant compute PsInv Jacobian: gik=null"));
    return FALSE;
  }

  if(gik->task[task_no]->Jacobian == NULL){
    PrintError(("GIK not initialized"));
    return FALSE;
  }

  JackT = gsl_matrix_alloc(gik->task[task_no]->n, gik->task[task_no]->m);
  V = gsl_matrix_alloc(gik->task[task_no]->m, gik->task[task_no]->m);
  SWS = gsl_vector_alloc(gik->task[task_no]->m);      /* vector S With Singularities */
  SWoS = gsl_vector_alloc(gik->task[task_no]->m);     /* vector S Without Singularities */
  workSpace = gsl_vector_alloc(gik->task[task_no]->m);
  U_T = gsl_matrix_alloc(gik->task[task_no]->m, gik->task[task_no]->n);
  S_sqr =  gsl_matrix_alloc(gik->task[task_no]->m, gik->task[task_no]->m);
  PsiJackT = gsl_matrix_alloc(gik->task[task_no]->m, gik->task[task_no]->n);
  Temp = gsl_matrix_alloc(gik->task[task_no]->m, gik->task[task_no]->n);

  gsl_matrix_transpose_memcpy(JackT, gik->task[task_no]->Jacobian);

  //printf("\nJacobian T\n"); hri_gik_ShowTheMatrix(JackT);

  gsl_linalg_SV_decomp(JackT, V, SWS, workSpace); /* JackT is replaced by U matrix */

  //printf("\nSWS\n"); hri_gik_ShowTheVector(SWS);

  /*** V must be transposed to reform the Jacobian ***/

  for(i=0; i<SWS->size; i++){
    delta = gsl_vector_get(SWS, i);
    gsl_vector_set(SWS, i, 1/delta);
    delta = delta / (delta*delta + EPS2*EPS2); /* to avoid the singularities */
    gsl_vector_set(SWoS, i, delta);
  }

  //printf("\nS\n"); hri_gik_ShowTheVector(S);

  gsl_matrix_transpose_memcpy(U_T, JackT); /* JackT is the transpose of the U_T matrix */

  /************ Calculating PsInvJacobian avoiding singular values ************/

  gsl_matrix_set_zero(S_sqr);
  for(i=0; i<SWoS->size; i++)
    gsl_matrix_set(S_sqr, i, i, gsl_vector_get(SWoS,i)); /* SWoS became a square matrix */

  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, V, U_T, 0.0, Temp); /* Temp = V * U_T */
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_sqr, Temp, 0.0, PsiJackT); /* PsiJackT = S_sqr * Temp */

  gsl_matrix_transpose_memcpy(gik->task[task_no]->PsInvJacobianWoS, PsiJackT);

  //printf("\nPsInvJacobianWoS T\n"); hri_gik_ShowTheMatrix(gik->PsInvJacobianWoS);

  /******************* Calculating PsInvJacobian keeping singular values **********/

  for(i=0; i<SWS->size; i++)
    gsl_matrix_set(S_sqr, i, i, gsl_vector_get(SWS,i)); /* SWS became a square matrix */

  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, S_sqr, Temp, 0.0, PsiJackT); /* PsiJackT = S_sqr * Temp */

  gsl_matrix_transpose_memcpy(gik->task[task_no]->PsInvJacobianWS, PsiJackT);

  //printf("\nPsInvJacobianWS T\n"); hri_gik_ShowTheMatrix(gik->PsInvJacobianWS);

  gsl_matrix_free(JackT);
  gsl_matrix_free(V);
  gsl_vector_free(SWoS);
  gsl_vector_free(SWS);
  gsl_vector_free(workSpace);
  gsl_matrix_free(U_T);
  gsl_matrix_free(S_sqr);
  gsl_matrix_free(PsiJackT);
  gsl_matrix_free(Temp);

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Prints the matrix
 *
 * \param M     matrix
 * !

 */
/****************************************************************/
void hri_gik_ShowTheMatrix(gsl_matrix * M)
{
  unsigned int i,j;

  for(i=0; i<M->size1; i++){
    for(j=0; j<M->size2; j++)
      printf("%10lf ", gsl_matrix_get(M,i,j));
    printf("\n");
  }
}

/****************************************************************/
/*!
 * \brief Prints the vector
 *
 * \param V   vector
 * !

 */
/****************************************************************/
void hri_gik_ShowTheVector(gsl_vector * V)
{
  unsigned int i;

  for(i=0; i<V->size; i++){
    printf("%3f\n", gsl_vector_get(V,i));
  }
}

p3d_vector4 center;
//p3d_rob * goal = NULL;

/****************************************************************/
/*!
 * \brief Takes the goal coordinate
 *
 * \param gik      gik structure
 * \param task_no  task number
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_take_goal(hri_gik * gik, int task_no)
{
  double  coord[6];
  int i;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob * goal = NULL;

  if(gik == NULL){
    PrintError(("Cant compute the goal point: gik=null"));
    return FALSE;
  }

  if(!gik->GIKInitialized || !gik->task[task_no]->initialized){
    PrintError(("GIK/task not initialized"));
    return FALSE;
  }

  if(goal == NULL){
    for(i=0; i<env->nr; i++){
      if( strcasestr(env->robot[i]->name,"BOTTLE") ){
        goal = env->robot[i];
        break;
      }
    }
  }
  p3d_mat4ExtractPosReverseOrder(goal->joints[1]->abs_pos,
                                 coord, coord+1, coord+2, coord+3, coord+4, coord+5);

  /* if((center[0]==coord[0]) && (center[1]==coord[1]) && (center[2]==coord[2])){ */
  /*     return TRUE; */
  /*   } */
  for(i=0; i<gik->task[task_no]->m; i++)
    center[i] = coord[i];

  //p3d_psp_gen_rand_3Dpoint(result,center,0.1,0.2);

  for(i=0; i<gik->task[task_no]->m; i++){
    gsl_vector_set(gik->task[task_no]->goal, i, center[i]);
  }

  return TRUE;
}


/****************************************************************/
/*!
 * \brief Updates robot config according to posture variation vector
 *
 * \param gik    gik structure
 * \param DT     posture variation vector
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_updaterobot(hri_gik * gik, gsl_vector * DT)
{
  int i;
  configPt newconfig;

  if(gik == NULL){
    PrintError(("Cant update robot: gik=null"));
    return FALSE;
  }

  if(!gik->GIKInitialized){
    PrintError(("GIK not initialized"));
    return FALSE;
  }

  newconfig = p3d_get_robot_config(gik->robot);

  for(i=0;i<gik->joint_no; i++)
    newconfig[gik->joints[i]->index_dof] = newconfig[gik->joints[i]->index_dof] + gsl_vector_get(DT,i);

  p3d_set_and_update_this_robot_conf(gik->robot,newconfig);

  p3d_destroy_config(gik->robot,newconfig);

  return TRUE;
}


/****************************************************************/
/*!
 * \brief Computes the remaining distance vector
 *
 * \param gik      gik structure
 * \param task_no  task no
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_compute_DX(hri_gik * gik, int task_no)
{
  double Ccoord[6];
  int i;

  if(gik == NULL || !gik->GIKInitialized)
    return FALSE;

  /* hri_gik_take_goal(gik,task_no);  the goal point is already in goal structure */

  p3d_mat4ExtractPosReverseOrder(gik->robot->joints[gik->task[task_no]->eef_no]->abs_pos, Ccoord, Ccoord+1, Ccoord+2,
                                 Ccoord+3, Ccoord+4, Ccoord+5);

  for(i=0; i<gik->task[task_no]->m; i++){
    // if(i==5 || i==4 || i==3)
    //   gsl_vector_set(gik->task[task_no]->deltaX,i,(gsl_vector_get(gik->task[task_no]->goal,i)-Ccoord[i])/10);
    //else
    gsl_vector_set(gik->task[task_no]->deltaX,i,gsl_vector_get(gik->task[task_no]->goal,i)-Ccoord[i]);
  }
  
  // TEST CODE
  // Clamping target vector to reduce oscillations
  //double goal_i;
//  double Dmax = 0.2;
//  
//  for(i=0; i<gik->task[task_no]->m; i++){
//    goal_i = gsl_vector_get(gik->task[task_no]->goal,i);
//    
//    if(ABS(goal_i-Ccoord[i]) > Dmax) {
//      gsl_vector_set(gik->task[task_no]->deltaX,i,Dmax*(goal_i-Ccoord[i])/ABS(goal_i-Ccoord[i]));
//    }
//    else {
//      gsl_vector_set(gik->task[task_no]->deltaX,i,goal_i-Ccoord[i]);
//    }
//  }
//  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
//  for(i=0; i<env->nr; i++){
//    if( strcasestr(env->robot[i]->name,"VISBALL") )
//      break;
//  }
//  configPt q;
//  q = p3d_get_robot_config(env->robot[i]);
//  q[6] = gsl_vector_get(gik->task[task_no]->deltaX,0)+gik->robot->joints[37]->abs_pos[0][3];
//  q[7] = gsl_vector_get(gik->task[task_no]->deltaX,1)+gik->robot->joints[37]->abs_pos[1][3];
//  q[8] = gsl_vector_get(gik->task[task_no]->deltaX,2)+gik->robot->joints[37]->abs_pos[2][3];
//  
//  p3d_set_and_update_this_robot_conf(env->robot[i], q);
  
  //hri_gik_ShowTheVector(gik->task[task_no]->deltaX);
  //printf("\n");

  /* This works only if you reach x,y,z; orientations are not taken into account */

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Computes the inside of convergence loop
 *
 * \param gik      gik structure
 * \param force    perturbation force
 * \param DT_final resulting posture variation vector
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_compute_core(hri_gik * gik, gsl_vector * DT_final)
{
  int i;
  double thetadiff;
  gsl_matrix * P_initial;
  gsl_vector * DT_initial;
  gsl_vector * tempVm;
  gsl_matrix * tempMmn;
  int clamping_detected;
  double norm=0;

  P_initial = gsl_matrix_alloc(gik->task[0]->n,gik->task[0]->n);
  DT_initial = gsl_vector_alloc(gik->task[0]->n);
  tempVm = gsl_vector_alloc(gik->task[0]->m);
  tempMmn = gsl_matrix_alloc(gik->task[0]->m,gik->task[0]->n);

  gsl_matrix_set_identity(P_initial); /* PNJ0 = I */
  gsl_vector_set_zero(DT_initial);    /* Dtheta0 = 0 */
  hri_gik_free_joints(gik);

  do{  /* clamping loop */
    i=0;
    do{ /* pritority loop */

      hri_gik_compute_DX(gik,i);

      if(i==0)
        gsl_blas_dgemv(CblasNoTrans, 1.0, gik->task[i]->Jacobian, DT_initial,
                       0.0, tempVm); /* tempVm = J0 * DT_init */
      else
        gsl_blas_dgemv(CblasNoTrans, 1.0, gik->task[i]->Jacobian, gik->task[i-1]->deltaTheta,
                       0.0, tempVm); /* tempVm = J0 * DT_i-1 */

      gsl_vector_sub(gik->task[i]->deltaX, tempVm); /* DeltaXi <--- DeltaXi-tempVm */

      if(i==0)
        gsl_blas_dgemm(CblasNoTrans,CblasNoTrans, 1.0, gik->task[i]->Jacobian, P_initial,
                       0.0,tempMmn); /* tempMmn = J0 * PJ_init */
      else
        gsl_blas_dgemm(CblasNoTrans,CblasNoTrans, 1.0, gik->task[i]->Jacobian, gik->task[i-1]->PJ,
                       0.0,tempMmn); /* tempMmn = J0 * PJ_i-1 */

      gsl_matrix_memcpy(gik->task[i]->Jacobian, tempMmn); /* J = tempMmn */

      hri_gik_computePsInvJacobian(gik,i);

      gsl_blas_dgemv(CblasNoTrans, 1.0, gik->task[i]->PsInvJacobianWoS,
                     gik->task[i]->deltaX,
                     0.0, gik->task[i]->deltaTheta); /* DT_i = PsInvJ * DX_i */

      if(i==0)
        gsl_vector_add(gik->task[i]->deltaTheta, DT_initial); /* DT_i <--- DT_initial + DT_i */
      else
        gsl_vector_add(gik->task[i]->deltaTheta, gik->task[i-1]->deltaTheta); /* DT_i <--- DT_i-1 + DT_i */

      gsl_blas_dgemm(CblasNoTrans,CblasNoTrans, 1.0, gik->task[i]->PsInvJacobianWoS, gik->task[i]->Jacobian,
                     0.0,gik->task[i]->PJ); /* PJ_i = PsInvJ_i * J_i */

      gsl_matrix_scale(gik->task[i]->PJ, -1); /* PJ_i = -PJ_i */

      if(i==0)
        gsl_matrix_add(gik->task[i]->PJ, P_initial); /* PJ_i <--- PJ_initial + PJ_i  */
      else
        gsl_matrix_add(gik->task[i]->PJ, gik->task[i-1]->PJ); /* PJ_i <--- PJ_i-1 + PJ_i */

      i++;
    } while(i<gik->task_no); /* End of pritority loop */

    if(DT_final->size != gik->task[gik->task_no-1]->deltaTheta->size){
      PrintError(("Can't copy DT, vector size don't match"));
      return FALSE;
    }
    gsl_vector_memcpy(DT_final,gik->task[gik->task_no-1]->deltaTheta ); /* DT_final = DT */
    /* CALCULATE DALPHA and multiply it with PN_task_no-1  then add is to DT_final */

    for(i=0; i<(signed int)(DT_final->size); i++){
      norm += SQR(gsl_vector_get(DT_final,i));
    }

    norm = sqrt(norm)*20;
    /* printf("D theta:\n");hri_gik_ShowTheVector(DT_final); printf("\n"); */
    //norm =1;
    if(norm!=0){
      for(i=0; i<(signed int)DT_final->size; i++){
        gsl_vector_set(DT_final,i,gsl_vector_get(DT_final,i)/norm);
      }
    }
    clamping_detected = FALSE;

    for(i=0; i<gik->joint_no; i++){
      if(gik->free_joints[i]){

        if(gsl_vector_get(DT_final,i) + gik->joints[i]->dof_data->v > gik->joints[i]->dof_data->vmax ||
           gsl_vector_get(DT_final,i) + gik->joints[i]->dof_data->v < gik->joints[i]->dof_data->vmin){

          clamping_detected = TRUE;
          /* printf("\nClamping occured in joint %d\n",i); */ /*, joint limit min %f max %f\n value: %f",i,
                                                               gik->joints[i]->dof_data->vmin,gik->joints[i]->dof_data->vmax,
                                                               gsl_vector_get(DT_final,i) + gik->joints[i]->dof_data->v); */
          if(gsl_vector_get(DT_final,i) + gik->joints[i]->dof_data->v > gik->joints[i]->dof_data->vmax){
            thetadiff = gsl_vector_get(DT_final,i) + gik->joints[i]->dof_data->v - gik->joints[i]->dof_data->vmax;
            p3d_set_robot_dof(gik->robot, gik->joints[i]->index_dof,gik->joints[i]->dof_data->vmax);
          }
          else{
            thetadiff = gsl_vector_get(DT_final,i) + gik->joints[i]->dof_data->v - gik->joints[i]->dof_data->vmin;
            p3d_set_robot_dof(gik->robot, gik->joints[i]->index_dof,gik->joints[i]->dof_data->vmin);
          }
          gsl_vector_set(DT_initial, i, thetadiff);

          gsl_matrix_set(P_initial,i,i,0);

          gik->free_joints[i] = FALSE;

        }
      }
    }

  }while(clamping_detected); /* End of Clamping loop */

  gsl_matrix_free(P_initial);
  gsl_vector_free(DT_initial);
  gsl_vector_free(tempVm);
  gsl_matrix_free(tempMmn);

  return TRUE;

}

/****************************************************************/
/*!
 * \brief Freedom for joints!
 *
 * \param gik  gik structure
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_free_joints(hri_gik * gik)
{
  int i;

  if(gik==NULL || gik->task[0]==NULL)
    return FALSE;

  for(i=0; i<gik->joint_no; i++){
    gik->free_joints[i]=TRUE;
  }
  return TRUE;
}


/****************************************************************/
/*!
 * \brief Starts the GIK method for robot
 *
 * \param robot    robot
 * \param gik      gik structure
 * \param step     limit of convergence loop
 * \param reach    the tolenrance to decide if it has reached or not
 * \param goalCoord Coordinates of the goal point
 * \param goalOrient Orientation of the goal point
 * \param qresult  resulting configuration
 * \param fct      draw function
 * !

 * \return TRUE   if the goal is reached
 */
/****************************************************************/

int hri_gik_compute(p3d_rob * robot, hri_gik * gik, int step, double reach,
                    p3d_vector3* goal, configPt * qresult, int (*fct)(void))
{
  int count = 0, viscount=0;
  double remainingdist, maxdistance=0;
  int i;
  gsl_vector * DT;
  configPt qsaved=NULL;

  if(gik == NULL || !gik->GIKInitialized){
    PrintError(("Cant compute GIK: gik is not initialized"));
    return FALSE;
  }

  qsaved = p3d_get_robot_config(robot);

  if(goal != NULL){
    for(i=0; i<gik->task_no; i++){
      gsl_vector_set(gik->task[i]->goal, 0, goal[i][0]);
      gsl_vector_set(gik->task[i]->goal, 1, goal[i][1]);
      gsl_vector_set(gik->task[i]->goal, 2, goal[i][2]);
    }
  }
  else{
    PrintError(("GoalCoord = NULL"));
    return FALSE;
  }
  DT = gsl_vector_alloc(gik->joint_no);

  do{  /* Convergence loop */

    for(i=0; i<gik->task_no; i++){
      hri_gik_computeJacobian(gik,i,FALSE);
    }

    hri_gik_compute_core(gik, DT);

    hri_gik_updaterobot(gik, DT);
    
#ifdef HRI_PLANNER
    /* printf("\n update vector is :\n"); hri_gik_ShowTheVector(DT); */
    if(viscount == GIK_VIS){
      //g3d_draw_allwin_active();
      g3d_refresh_allwin_active();
      viscount=0;
    }
    else
      viscount++;
#endif

    if(fct) if(((*fct)()) == FALSE) return(count);

    maxdistance = 0;
    for(i=0; i<gik->task_no; i++){
      remainingdist = hri_gik_remainingdistance(gik,i);
      if(remainingdist > maxdistance)
        maxdistance = remainingdist;
      /*  printf("Remaining distance for task %d: %f\n",i, remainingdist);   */
    }
    /*  printf("\n");  */

    count++;
  }while(count < step && maxdistance > reach);

  gsl_vector_free(DT);

  //  configPt qtmpo = p3d_get_robot_config(robot);
  p3d_get_robot_config_into(robot,qresult);
  p3d_set_and_update_this_robot_conf(robot,qsaved);
  p3d_destroy_config(robot,qsaved);

  //g3d_draw_allwin_active();
  //  res = p3d_col_test_robot(gik->robot,0);
  //  if(res) return FALSE;
  /* printf("Remaining distance for first task: %f\n", maxdistance); */
  if(maxdistance <= reach)
    return TRUE;
  else
    return FALSE;

}

/****************************************************************/
/*!
 * \brief Remaining distance to task's goal
 *
 * \param gik      gik structure
 * \param task_no  task number
 * !

 * \return double  the distance
 */
/****************************************************************/
double hri_gik_remainingdistance(hri_gik * gik, int task_no)
{
  double Ccoord[6];
  double dist=0;
  int i;

  p3d_mat4ExtractPosReverseOrder(gik->robot->joints[gik->task[task_no]->eef_no]->abs_pos, Ccoord, Ccoord+1, Ccoord+2,
                                 Ccoord+3, Ccoord+4, Ccoord+5);

  for(i=0; i<3; i++){
    dist +=  SQR(gsl_vector_get(gik->task[task_no]->goal,i)-Ccoord[i]);
  }
  dist = sqrt(dist);
  return dist;
}

/****************************************************************/
/*!
 * \brief Remaining distance to task's goal
 *
 * \param gik      gik structure
 * \param task_no  task number
 * !
 
 * \return double  the distance
 */
/****************************************************************/
int hri_gik_compute_sdls(hri_gik * gik, gsl_vector * DT_final)
{
  double lambdasq = SQR(0.01);
  double val;
  int i, s;
  gsl_matrix *U, *Jt;
  gsl_vector *Dt;
  
  hri_gik_compute_DX(gik,0);
  
  Jt = gsl_matrix_alloc(gik->task[0]->n,gik->task[0]->m);
  U =  gsl_matrix_alloc(gik->task[0]->m,gik->task[0]->m);
  Dt = gsl_vector_alloc(gik->task[0]->m);
  
  gsl_matrix_transpose_memcpy(Jt, gik->task[0]->Jacobian);
 
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, gik->task[0]->Jacobian, Jt,
                 0.0, U); /* U = J * Jt */
  
  // Add lambdasq to the diagonal of U
  for (i=0; i<(signed int)U->size1; i++) {
    val = gsl_matrix_get(U, i, i) + lambdasq;
    gsl_matrix_set(U, i, i, val);
  }
  
  // Solve the equation Dt = U * DX
  
  gsl_permutation *p = gsl_permutation_alloc(gik->task[0]->m);
  
  gsl_linalg_LU_decomp(U, p, &s);
  
  gsl_linalg_LU_solve (U, p, Dt, gik->task[0]->deltaX);
  
  gsl_blas_dgemv(CblasNoTrans, 1.0, Jt, Dt,
                 0.0, DT_final); /* DTheta = Jt * Dt */
  
  return TRUE;
}
  
int hri_gik_sdls(p3d_rob * robot, hri_gik * gik, int step, double reach,
                 p3d_vector3* goal, configPt * qresult, int (*fct)(void))
{
  double remainingdist, maxdistance=0;
  int i;
  int viscount = 0;
  gsl_vector * DT;
  configPt qsaved=NULL;
  
  if(gik == NULL || !gik->GIKInitialized){
    PrintError(("Cant compute GIK: gik is not initialized"));
    return FALSE;
  }
  
  qsaved = p3d_get_robot_config(robot);
  
  if(goal != NULL){
    for(i=0; i<gik->task_no; i++){
      gsl_vector_set(gik->task[i]->goal, 0, goal[i][0]);
      gsl_vector_set(gik->task[i]->goal, 1, goal[i][1]);
      gsl_vector_set(gik->task[i]->goal, 2, goal[i][2]);
    }
  }
  else{
    PrintError(("GoalCoord = NULL"));
    return FALSE;
  }
  DT = gsl_vector_alloc(gik->joint_no);
      
  hri_gik_computeJacobian(gik,0,FALSE);
  hri_gik_ShowTheMatrix(gik->task[0]->Jacobian);
  hri_gik_compute_sdls(gik, DT);
  hri_gik_ShowTheVector(DT);
  hri_gik_updaterobot(gik, DT);
  
#ifdef HRI_PLANNER
  /* printf("\n update vector is :\n"); hri_gik_ShowTheVector(DT); */
  if(viscount == GIK_VIS){
    //g3d_draw_allwin_active();
    g3d_refresh_allwin_active();
    viscount=0;
  }
  else {
    viscount++;
  }
#endif
    
  maxdistance = 0;
  for(i=0; i<gik->task_no; i++){
    remainingdist = hri_gik_remainingdistance(gik,i);
    if(remainingdist > maxdistance)
      maxdistance = remainingdist;
    /*  printf("Remaining distance for task %d: %f\n",i, remainingdist);   */
  }
  /*  printf("\n");  */
  
  gsl_vector_free(DT);
  
  //  configPt qtmpo = p3d_get_robot_config(robot);
  p3d_get_robot_config_into(robot,qresult);
  p3d_set_and_update_this_robot_conf(robot,qsaved);
  p3d_destroy_config(robot,qsaved);
  
  //g3d_draw_allwin_active();
  //  res = p3d_col_test_robot(gik->robot,0);
  //  if(res) return FALSE;
  /* printf("Remaining distance for first task: %f\n", maxdistance); */
  if(maxdistance <= reach)
    return TRUE;
  else
    return FALSE;
  
}


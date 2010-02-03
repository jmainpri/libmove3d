#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

hri_gik * HRI_GIK = NULL;
int HRI_GIK_CONTINUOUS = TRUE;

/*******************************************************************/
/****************** GIK FUCTIONS ***********************************/
/*******************************************************************/

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
  gik->direct = FALSE;
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
  gik->direct = FALSE;
  gik->GIKInitialized = FALSE;

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Initialize a GIK structure
 *
 * \param gik             gik structure
 * \param robot           the robot
 * \param direct          true if you don't use perturbation forces
 * \param total_joint_no  number of joints involved in gik
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_gik_initialize_gik(hri_gik * gik, p3d_rob * robot, int direct, int total_joint_no)
{
  if(gik == NULL || robot == NULL){
    PrintError(("Cant initialize GIK: gik or robot =null"));
    return FALSE;
  }

  gik->robot = robot;
  gik->direct = direct;
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

/*************** OBSOLETE ****************/
int hri_gik_computePJ(hri_gik * gik, int task_no)
{
  gsl_matrix * Temp;

  if(gik == NULL){
    PrintError(("Cant compute PJ: gik=null"));
    return FALSE;
  }

  if(!gik->GIKInitialized || !gik->task[task_no]->initialized ||
     gik->task[task_no]->Jacobian == NULL ||
     gik->task[task_no]->PsInvJacobianWoS == NULL){
    PrintError(("GIK not initialized"));
    return FALSE;
  }

  Temp = gsl_matrix_alloc(gik->task[task_no]->n, gik->task[task_no]->n);

  //gsl_matrix_memcpy(gik->task[task_no]->PJ, gik->task[task_no]->PJ0);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                 gik->task[task_no]->PsInvJacobianWS,
                 gik->task[task_no]->Jacobian, 0.0, Temp); /* Temp = PsiJack * Jack */
  gsl_matrix_sub(gik->task[task_no]->PJ, Temp);

  gsl_matrix_free(Temp);

  return TRUE;
  //printf("\n PJ \n"); ShowTheMatrix(PJ);
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
      if( strstr(env->robot[i]->name,"BOTTLE") ){
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


p3d_rob * obstacle = NULL;

/*************** OBSOLETE ****************/
int hri_gik_computeDeltaAlpha(hri_gik * gik, double force, int task_no)
{
  int i,l=0,j;
  double visdistance, disturbance;
  p3d_vector3 bodypoint, camerapoint, destpoint, obstaclecenter;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector3 y_vector,f,proj;
  double fnorm = 0;

  if(gik == NULL){
    PrintError(("Cant compute dAlpha: gik=null"));
    return FALSE;
  }

  if(!gik->GIKInitialized || !gik->task[task_no]->initialized){
    PrintError(("GIK not initialized"));
    return FALSE;
  }

  /*  for(i=0; i<gik->task[task_no]->deltaAlpha->size; i++) */
  /*     gsl_vector_set(gik->task[task_no]->deltaAlpha,i,0); */

  //return TRUE; /* we ignore  Dalpha */

  if(gik->direct)
    return TRUE;

  /***** TEST FOR NON-BLOCKING OF THE CAMERA VIEW *******/
  if(FALSE){
    camerapoint[0] = gik->robot->joints[ROBOTj_PAN]->abs_pos[0][3];
    camerapoint[1] = gik->robot->joints[ROBOTj_PAN]->abs_pos[1][3];
    camerapoint[2] = gik->robot->joints[ROBOTj_PAN]->abs_pos[2][3];

    destpoint[0] = BTSET->visball->joints[1]->abs_pos[0][3];
    destpoint[1] = BTSET->visball->joints[1]->abs_pos[1][3];
    destpoint[2] = BTSET->visball->joints[1]->abs_pos[2][3];

    g3d_drawOneLine(camerapoint[0],camerapoint[1],camerapoint[2],
                    destpoint[0],destpoint[1],destpoint[2], 1, NULL);

    for(i=0, l=0; i<gik->task[task_no]->jnt_no; i++){
      if(i==0 || i==5 || i==4 || i==3){
        l++;
        continue;
      }
      bodypoint[0] = (gik->task[task_no]->jnt[i]->joint->o->BB.xmax +
                      gik->task[task_no]->jnt[i]->joint->o->BB.xmin)/2;
      bodypoint[1] = (gik->task[task_no]->jnt[i]->joint->o->BB.ymax +
                      gik->task[task_no]->jnt[i]->joint->o->BB.ymin)/2;
      bodypoint[2] = (gik->task[task_no]->jnt[i]->joint->o->BB.zmax +
                      gik->task[task_no]->jnt[i]->joint->o->BB.zmin)/2;

      visdistance = p3d_psp_pointtolinedist(bodypoint, camerapoint, destpoint);

      disturbance = pow(1/(force*visdistance+1),8);
      printf("joint %d, distance: %f, perturbation: %f\n",i,visdistance,disturbance);
      /*   gsl_vector_set(gik->task[task_no]->deltaAlpha,l,disturbance); */
      /*       gsl_vector_set(gik->task[task_no]->deltaAlpha,0, */
      /* 		     gsl_vector_get(gik->task[task_no]->deltaAlpha,0)+disturbance); */
      /*       gsl_vector_set(gik->task[task_no]->deltaAlpha,3, */
      /* 		     gsl_vector_get(gik->task[task_no]->deltaAlpha,3)+disturbance); */

      /*       l++; */

    }
  }
  if(obstacle == NULL){
    for(i=0; i<env->nr; i++){
      if( !strcmp("OBSTACLE",env->robot[i]->name) ){
        obstacle = env->robot[i];
        break;
      }
    }
  }

  /********* TEST OF NON COLLISION *******/
  for(i=2; i<gik->task[task_no]->jnt_no; i++){
    bodypoint[0] = (gik->task[task_no]->jnt[i]->joint->o->BB.xmax +
                    gik->task[task_no]->jnt[i]->joint->o->BB.xmin)/2;
    bodypoint[1] = (gik->task[task_no]->jnt[i]->joint->o->BB.ymax +
                    gik->task[task_no]->jnt[i]->joint->o->BB.ymin)/2;
    bodypoint[2] = (gik->task[task_no]->jnt[i]->joint->o->BB.zmax +
                    gik->task[task_no]->jnt[i]->joint->o->BB.zmin)/2;

    obstaclecenter[0] = (obstacle->joints[1]->o->BB.xmax + obstacle->joints[1]->o->BB.xmin)/2;
    obstaclecenter[1] = (obstacle->joints[1]->o->BB.ymax + obstacle->joints[1]->o->BB.ymin)/2;
    obstaclecenter[2] = (obstacle->joints[1]->o->BB.zmax + obstacle->joints[1]->o->BB.zmin)/2;

    if( DISTANCE3D(bodypoint[0],bodypoint[1],bodypoint[2],
                   obstaclecenter[0],obstaclecenter[1], obstaclecenter[2])>0.3 )
      continue;

    printf("CHECKPOINT\n");

    for(j=0; j<3; j++)
      y_vector[j] = gik->task[task_no]->jnt[i]->joint->abs_pos[j][0];


    // if(ABS(obstaclecenter[0]-bodypoint[0])>0.2 || ( ABS(obstaclecenter[1]-bodypoint[1])>0.3 &&
    //					    ABS(obstaclecenter[2]-bodypoint[2])>0.3))
    //continue;

    for(j=0; j<3; j++)
      //f[j] = force * (bodypoint[j] - obstaclecenter[j]);
      f[j] = -1/(10*force*(bodypoint[j] - obstaclecenter[j])+1);

    for(j=0; j<3; j++)
      proj[j] = (p3d_vectDotProd(y_vector,f)/p3d_square_of_vectNorm(y_vector)) * y_vector[j];

    if(p3d_vectNorm(proj)==0)
      continue;

    if(proj[0]==0)
      proj[0]=proj[1];
    if(proj[0]==0)
      proj[0]=proj[2];
    if(proj[0]==0)
      continue;

    fnorm =  p3d_vectNorm(f);

    /*  if(y_vector[0]/proj[0] < 0) */
    /* 	gsl_vector_set( gik->task[task_no]->deltaAlpha, gik->task[task_no]->joints[i]->jacob_index, */
    /* 			(-1*fnorm)+gsl_vector_get(gik->task[task_no]->deltaAlpha,gik->task[task_no]->joints[i]->jacob_index) ); */
    /*       else */
    /* 	gsl_vector_set(gik->task[task_no]->deltaAlpha,gik->task[task_no]->joints[i]->jacob_index, */
    /* 		       (fnorm)+gsl_vector_get(gik->task[task_no]->deltaAlpha,gik->task[task_no]->joints[i]->jacob_index)); */


    /*  visdistance = DISTANCE3D(bodypoint[0], bodypoint[1],bodypoint[2], */
    /* 			     obstaclecenter[0],obstaclecenter[1],obstaclecenter[2]); */

    /*  disturbance = pow(-1/(force*visdistance+0.2+1),8); */
    /*       printf("joint %d, distance: %f, perturbation: %f\n",i,visdistance,disturbance); */
    /*       gsl_vector_set(gik->task[task_no]->deltaAlpha, */
    /* 		     gik->task[task_no]->joints[i]->jacob_index, */
    /* 		   disturbance); */

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

/*************** OBSOLETE ****************/
configPt hri_gik_update_core_result(hri_gik * gik)
{
  int i,j;
  configPt robotnewconfig;

  if(gik == NULL){
    PrintError(("Cant update robot: gik=null"));
    return NULL;
  }

  if(!gik->GIKInitialized){
    PrintError(("GIK not initialized"));
    return NULL;
  }

  robotnewconfig = p3d_get_robot_config(gik->robot);

  for(i=0; i<gik->task_no; i++)
    for(j=0; j<gik->task[i]->jnt_no; j++){
      robotnewconfig[gik->task[i]->jnt[j]->joint->index_dof] =
      robotnewconfig[gik->task[i]->jnt[j]->joint->index_dof] + gsl_vector_get(gik->task[i]->deltaTheta,j);
    }


  p3d_set_and_update_this_robot_conf(gik->robot,robotnewconfig);

  return robotnewconfig;
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
int hri_gik_compute_core(hri_gik * gik, double force, gsl_vector * DT_final)
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
 * \param direct   perturbation or not
 * \param force    intensity of perturbation
 * \param fct      draw function
 * !

 * \return FALSE in case of a problem
 */
/****************************************************************/

int hri_gik_compute(p3d_rob * robot, hri_gik * gik, int step, double reach, int direct, double force, p3d_vector3* goalCoord, p3d_vector3* goalOrient, configPt * qresult, int (*fct)(void))
{
  int count = 0, viscount=0;
  double remainingdist, maxdistance=0;
  int jointindexesJido[] = {5,6,7,8,9,10}; /* for jido  */
  //int jointindexesHrp2[3][20]= { {14,15,16,17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,44,45,46, 0},
  //  {14,15, 0, 0,18,19,20,21,22,23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //  {14,15, 0, 0, 0, 0, 0, 0, 0, 0,31,32,33,34,35,36, 0, 0, 0,48} }; /* for hrp2*/
  int jointindexesHrp2a[2][13]={ {14,15,16,17, 0, 0, 0, 0, 0, 0,45,46,47},
    {14,15, 0, 0,19,20,21,22,23,24, 0, 0, 0} };
  //int jointindexesHrp2head[1][5]={ {14,15,16,17,18} };
  /* int jointindexesBH[3][19]=  { {2,3,4,5,6,0,0, 0, 0, 0, 0, 0, 0, 0, 0,30,31,32, 0},
   {2,3,0,0,0,8,9,10,11,12, 0, 0, 0, 0, 0, 0, 0, 0, 0},
   {0,0,0,0,0,0,0, 0, 0, 0,19,20,21,22,23, 0, 0, 0,34} };  for blueHumanoid */

  int jointindexesBH[2][13]=  { {2,3,5,6,7,0, 0, 0, 0,0,30,31,32},
    {2,3,0,0,0,8,9,10,11,12, 0, 0, 0} }; /* for blueHumanoid */
  //int jointindexesJido[] = {5,6,7,8,9,10,15}; /* for jido  */
  int i;
  gsl_vector * DT;
  configPt qsaved=NULL;
  //int res;

  if(gik == NULL){
    PrintError(("Cant compute GIK: gik=null"));
    return FALSE;
  }

  qsaved = p3d_get_robot_config(robot);

  if(!gik->GIKInitialized){
#ifdef HRI_JIDO
    /***** FOR JIDO *****/
    hri_gik_initialize_gik(gik,robot,direct,6);
    hri_gik_add_task(gik, 6, 6, 1, jointindexesJido,ROBOTj_GRIP);  /* Gripper */

    //	hri_gik_initialize_gik(gik,robot,direct,7);
    //  hri_gik_add_task(gik, 3, 7, 1, jointindexesJido,ROBOTj_POINT);  /* Pointing */
#elif defined HRI_HRP2
    /***** FOR HRP2 *****/
    hri_gik_initialize_gik(gik,robot,direct,13); /* Attention to joint number */
    hri_gik_add_task(gik, 3, 13, 2, jointindexesHrp2a[0],ROBOTj_LOOK);  /* HEAD */
    hri_gik_add_task(gik, 3, 13, 1, jointindexesHrp2a[1],ROBOTj_OBJECT); /* RIGHT ARM */
    //   hri_gik_add_task(gik, 3, 20, 3, jointindexesHrp2[2],ROBOTj_LHAND); /* LEFT ARM */
#elif defined HRI_TUM_BH
    /***** FOR BH *****/
    hri_gik_initialize_gik(gik,robot,direct,13); /* Attention to joint number */
    hri_gik_add_task(gik, 3, 13, 2, jointindexesBH[0],ROBOTj_LOOK);  /* HEAD */
    hri_gik_add_task(gik, 3, 13, 1, jointindexesBH[1],ROBOTj_OBJECT); /* RIGHT ARM */
    /* hri_gik_add_task(gik, 3, 19, 3, jointindexesBH[2],ROBOTj_LHAND); */ /* LEFT ARM */
#else
#error "No robot defined in Hri-Planner-pkg"
#endif
  }

  if(goalCoord != NULL){
    for(i=0; i<gik->task_no; i++){
      gsl_vector_set(gik->task[i]->goal, 0, goalCoord[i][0]);
      gsl_vector_set(gik->task[i]->goal, 1, goalCoord[i][1]);
      gsl_vector_set(gik->task[i]->goal, 2, goalCoord[i][2]);
    }
  }
  else{
    PrintError(("GoalCoord = NULL"));
    return FALSE;
  }
  if(goalOrient != NULL){
    for(i=0; i<gik->task_no; i++){
      gsl_vector_set(gik->task[i]->goal, 3, goalOrient[i][0]);
      gsl_vector_set(gik->task[i]->goal, 4, goalOrient[i][1]);
      gsl_vector_set(gik->task[i]->goal, 5, goalOrient[i][2]);
    }
  }

  DT = gsl_vector_alloc(gik->joint_no);
  gik->direct = direct;

  do{  /* Convergence loop */

    for(i=0; i<gik->task_no; i++){
      if(goalOrient == NULL)
        hri_gik_computeJacobian(gik,i,0);
      else
        hri_gik_computeJacobian(gik,i,1);
    }

    hri_gik_free_joints(gik);

    hri_gik_compute_core(gik, force, DT);

    hri_gik_updaterobot(gik, DT);

    /* printf("\n update vector is :\n"); hri_gik_ShowTheVector(DT); */
    if(viscount == GIK_VIS){
      g3d_draw_allwin_active();
      viscount=0;
    }
    else
      viscount++;

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

/*************** OBSOLETE ****************/
int hri_gik_computestep(p3d_rob * robot, hri_gik * gik, int step, double force)
{
  /*  int count = 0, viscount=0; */
  /*   double remainingdist; */
  /*   int jointindexes[] = {4,5,6,7,8,9};  */

  /*   if(gik == NULL){ */
  /*     PrintError(("Cant update robot: gik=null")); */
  /*     return FALSE; */
  /*   }  */

  /*   if(!gik->GIKInitialized){ */
  /*     hri_gik_initialize_gik(robot, gik, 3, robot->nb_user_dof-2-2-3, jointindexes, 6  ); /\* no base *\/  */
  /*     free_joints = MY_ALLOC(int, gik->n);  */
  /*   } */
  /*   firstrunned=FALSE; */
  /*   do{ */
  /*     hri_gik_takeEndeffector(gik); */
  /*     hri_gik_compute_core(gik,force);  */

  /*     hri_gik_updaterobot(gik); */
  /*     if(viscount == 0){ */
  /*       g3d_draw_allwin_active(); */
  /*       viscount=0; */
  /*     } */
  /*     else */
  /*       viscount++; */
  /*     remainingdist = hri_gik_remainingdistance(gik); */
  /*     printf("Remaining distance: %f\n",remainingdist); */
  /*     count++;     */
  /*   }while(count < step); */

  return TRUE;
}

/*************** OBSOLETE ****************/
int hri_gik_computestepWoP(p3d_rob * robot, hri_gik * gik, int step, int direct, double force)
{
  /*  int count = 0, viscount=0; */
  /*   double remainingdist; */
  /*   int jointindexes[] = {4,5,6,7,8,9};  */

  /*   if(gik == NULL){ */
  /*     PrintError(("Cant update robot: gik=null")); */
  /*     return FALSE; */
  /*   }  */

  /*   if(!gik->GIKInitialized){ */
  /*     hri_gik_initialize_gik(robot, gik, 3, robot->nb_user_dof-2-2-3, jointindexes, 6  );  /\* no base *\/ */
  /*     free_joints = MY_ALLOC(int, gik->n);  */
  /*   } */
  /*   gik->direct = direct; */
  /*   if(qsaved == NULL) */
  /*     qsaved = p3d_get_robot_config(gik->robot); */
  /*   else */
  /*     p3d_set_and_update_this_robot_conf(gik->robot, qsaved); */

  /*   firstrunned=FALSE; */

  /*   do{ */
  /*     hri_gik_takeEndeffector(gik); */
  /*     hri_gik_compute_core(gik, force);  */

  /*     hri_gik_updaterobot(gik); */
  /*     if(viscount == hri_gik_VIS){ */
  /*       g3d_draw_allwin_active(); */
  /*       viscount=0; */
  /*     } */
  /*     else */
  /*       viscount++; */
  /*     remainingdist = hri_gik_remainingdistance(gik); */
  /*     printf("Remaining distance: %f\n",remainingdist); */
  /*     count++;     */
  /*   }while(count < step); */

  return TRUE;
}

/*************** OBSOLETE ****************/
int hri_gik_computestepPer(p3d_rob * robot, hri_gik * gik, int step, double force)
{
  /*  int count = 0, viscount=0; */
  /*   double remainingdist; */
  /*   int i=0,k=0; */
  /*   configPt q, qback; */
  /*   int jointindexes[] = {4,5,6,7,8,9};  */

  /*   if(gik == NULL){ */
  /*     PrintError(("Cant update robot: gik=null")); */
  /*     return FALSE; */
  /*   }  */

  /*   if(!gik->GIKInitialized){ */
  /*     hri_gik_initialize_gik(robot, gik, 3, robot->nb_user_dof-2-2-3, jointindexes, 6  );   /\* no base *\/ */
  /*     free_joints = MY_ALLOC(int, gik->n);  */
  /*   } */
  /*   gik->direct = FALSE; */

  /*   if(qsaved == NULL) */
  /*     qsaved = p3d_get_robot_config(gik->robot); */
  /*   else */
  /*     p3d_set_and_update_this_robot_conf(gik->robot, qsaved); */

  /*   firstrunned=FALSE; */

  /*   q = p3d_get_robot_config(gik->robot); */


  /*   do{ */
  /*     hri_gik_takeEndeffector(gik); */
  /*     hri_gik_compute_core(gik,force);  */

  /*     hri_gik_updaterobot(gik); */
  /*     printf("dAlpha:"); */
  /*     hri_gik_ShowTheVector(gik->deltaAlpha); */
  /*     printf("dAlphaproj:"); */
  /*     hri_gik_ShowTheVector(AlphaProj); */
  /*     printf("dTheta:"); */
  /*     hri_gik_ShowTheVector(gik->deltaTheta); */
  /*     for(i=ROBOTq_PAN, k=0; i<ROBOTq_EEF-1; i++){ /\*  base change point *\/  */
  /*       if(i == ROBOTq_PAN || i == ROBOTq_TILT) /\* camera joints *\/ */
  /* 	continue; */

  /*       q[i] = q[i] + gsl_vector_get(AlphaProj,k); */
  /*       k++; */
  /*     } */


  /*     if(viscount == hri_gik_VIS){ */
  /*       qback = p3d_get_robot_config(gik->robot); */
  /*       p3d_set_and_update_this_robot_conf(gik->robot, q); */
  /*       g3d_draw_allwin_active(); */
  /*       p3d_set_and_update_this_robot_conf(gik->robot, qback); */
  /*       viscount=0; */
  /*     } */
  /*     else */
  /*       viscount++; */
  /*     remainingdist = hri_gik_remainingdistance(gik); */
  /*     printf("Remaining distance: %f\n",remainingdist); */
  /*     count++;     */
  /*   }while(count < step); */

  return TRUE;
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
 * \brief Draws a line between goal and robot's camera
 *
 * !

 */
/****************************************************************/
void g3d_draw_cameratargetline()
{
  p3d_vector3 camerapoint, destpoint;

  if(HRI_GIK==NULL || HRI_GIK->robot==NULL)
    return;

  camerapoint[0] = HRI_GIK->robot->joints[ROBOTj_PAN]->abs_pos[0][3];
  camerapoint[1] = HRI_GIK->robot->joints[ROBOTj_PAN]->abs_pos[1][3];
  camerapoint[2] = HRI_GIK->robot->joints[ROBOTj_PAN]->abs_pos[2][3];

  destpoint[0] = BTSET->visball->joints[1]->abs_pos[0][3];
  destpoint[1] = BTSET->visball->joints[1]->abs_pos[1][3];
  destpoint[2] = BTSET->visball->joints[1]->abs_pos[2][3];

  g3d_drawOneLine(camerapoint[0],camerapoint[1],camerapoint[2],
                  destpoint[0],destpoint[1],destpoint[2], 1, NULL);

}

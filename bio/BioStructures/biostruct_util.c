#include "P3d-pkg.h"
#include "Bio-pkg.h"

static int bio_set_bio_jnt_type(p3d_jnt *jntPt)
{
  char *jnt_type_name;
  int indexletter=0;

  jnt_type_name = givemeword(jntPt->name, '.', &indexletter);
  if(strcmp(jnt_type_name,"omega") == 0) {
    jntPt->bio_jnt_type = BIO_OMEGA_JNT;
  }
  else if(strcmp(jnt_type_name,"phi") == 0) {
    jntPt->bio_jnt_type = BIO_PHI_JNT;
  }
  else if(strcmp(jnt_type_name,"psi") == 0) {
    jntPt->bio_jnt_type = BIO_PSI_JNT;
  }
  else if(strncmp(jnt_type_name,"gamma",5*sizeof(char)) == 0) {
    jntPt->bio_jnt_type = BIO_GAMMA_JNT;
  }
  else {
    jntPt->bio_jnt_type = BIO_OTHER_JNT;
  }
  return 1;
}

int bio_set_bio_jnt_types(void)
{
  pp3d_env envPt;
  p3d_rob *robotPt;
  p3d_rob **robots;
  int nrobots;
  int ir, ij;

  envPt = (p3d_env*) p3d_get_desc_curid(P3D_ENV);
  robots = envPt->robot;
  nrobots = p3d_get_desc_number(P3D_ROBOT);
  
  for (ir=0; ir < nrobots; ir++){
    robotPt = robots[ir];
    for (ij=0; ij <= robotPt->njoints; ij++){
      if(!bio_set_bio_jnt_type(robotPt->joints[ij])) {
	return 0;
      }
    }
  }
  return 1;
}


static int bio_set_bio_jnt_AAnumber(p3d_jnt *jntPt)
{
  char* name = jntPt->name; 
  jntPt->bio_AAnumber = get_AAnumber_from_name(name);
  return 1;
}


int bio_set_bio_jnt_AAnumbers(void)
{
  pp3d_env envPt;
  p3d_rob *robotPt;
  p3d_rob **robots;
  int nrobots;
  int ir, ij;

  envPt = (p3d_env*) p3d_get_desc_curid(P3D_ENV);
  robots = envPt->robot;
  nrobots = p3d_get_desc_number(P3D_ROBOT);
  
  for (ir=0; ir < nrobots; ir++){
    robotPt = robots[ir];
    for (ij=0; ij <= robotPt->njoints; ij++){
      if(!bio_set_bio_jnt_AAnumber(robotPt->joints[ij])) {
	return 0;
      }
    }
  }
  return 1;
}

/************************************************************/
/* \brief:  set the total number of aminoacid of the loaded 
 *          file in the robot structure                                            
************************************************************/
void bio_set_AAtotal_number(void)
{
  int i = 0;
  int nbAA= 0;
  char AAtype[4];
  int numAA = 0;
  int ancienNumAA = 0;
  for(i=0;i <XYZ_ENV->cur_robot->no;i++)
    {
      get_AAtype_from_name(XYZ_ENV->cur_robot->o[i]->name,AAtype); 
      if( !(strcmp(AAtype,"LIG")==0) )
	{
	  numAA = get_AAnumber_from_name(XYZ_ENV->cur_robot->o[i]->name);
	}
      if((ancienNumAA != numAA )  && !(strcmp(AAtype,"LIG")==0))
	{
	  nbAA++;
	  ancienNumAA = numAA;
	}
    }
  XYZ_ROBOT->nbAA = nbAA;
}

/************************************************************
 * \brief: compute the number of flexible side chains of    
 *         the robot and set it in the robot structure                                                                         
 * \note: Warning, the search of flexible sidechains is     
 *        on the existance of psi joint. Maybe for some     
 *        cinematic structures the function doesnot work    
************************************************************/
void bio_set_nb_flexible_sc(void) {
  p3d_rob* robotPt = XYZ_ROBOT;
  int cur_jnt_number = 1;
  int nb_flexible_jnts = 0;
  p3d_jnt *cur_jnt, *prev_jnt;

  while (cur_jnt_number <robotPt->njoints) {
    cur_jnt = robotPt->joints[cur_jnt_number];
    prev_jnt = robotPt->joints[cur_jnt_number-1];
/*     if((cur_jnt->bio_jnt_type == BIO_PSI_JNT)  && */
/*        (robotPt->joints[cur_jnt->prev_jnt->num+1] != cur_jnt)){ */
    if((cur_jnt->bio_jnt_type == BIO_GAMMA_JNT)  &&
       (prev_jnt->bio_jnt_type != BIO_GAMMA_JNT)  &&
       (cur_jnt->dof_data[0].vmin != cur_jnt->dof_data[0].vmax)  &&
       (p3d_jnt_get_dof_is_user(cur_jnt, 0))) {
      nb_flexible_jnts++;
    }
    cur_jnt_number++;
  }
  robotPt->nb_flexible_sc = nb_flexible_jnts;

  printf("\n\nNUM FLEX SCH = %d\n\n",nb_flexible_jnts);
}


/************************************************************/
/* \brief:  compute a list of first joints of all flexible  */
/*          side chains and set it in the robot structure   */
/* \note: Warning, the search of flexible sidechains is     */
/*        on the existance of psi joint. Maybe for some     */
/*        cinematic structures the function doesnot work !  */
/************************************************************/
void bio_set_list_firstjnts_flexible_sc(void) {
  p3d_rob* robotPt = XYZ_ROBOT;
  p3d_jnt** list_jnt = NULL, *cur_jnt, *prev_jnt;
  int cur_jnt_number = 0;
  int nb_flexible_jnts = 0;

  list_jnt = MY_ALLOC(p3d_jnt*, robotPt->nb_flexible_sc);
  cur_jnt_number = 1;
  nb_flexible_jnts = 0;
  while (cur_jnt_number <robotPt->njoints) {
    cur_jnt = robotPt->joints[cur_jnt_number];
    prev_jnt = robotPt->joints[cur_jnt_number-1];
/*     if((cur_jnt->bio_jnt_type == BIO_PSI_JNT)  && */
/*        (robotPt->joints[cur_jnt->prev_jnt->num+1] != cur_jnt)){ */
    if((cur_jnt->bio_jnt_type == BIO_GAMMA_JNT)  &&
       (prev_jnt->bio_jnt_type != BIO_GAMMA_JNT)  &&
       (cur_jnt->dof_data[0].vmin != cur_jnt->dof_data[0].vmax)  &&
       (p3d_jnt_get_dof_is_user(cur_jnt, 0))) {
      list_jnt[nb_flexible_jnts] = cur_jnt->prev_jnt;
      nb_flexible_jnts++;
    }
    cur_jnt_number++;
  }
  robotPt->list_firstjnts_flexible_sc = list_jnt;

  printf("\n\nNUM FLEX SCH = %d\n\n",nb_flexible_jnts);

}


void bio_set_nb_dof_ligand(void)
{
  p3d_rob* robotPt = XYZ_ROBOT;
  int num_jnt = robotPt->joints[0]->next_jnt[1]->num;
  int nb_dof = 0;

  while(num_jnt < XYZ_ROBOT->njoints +1)
    {
      nb_dof = nb_dof + XYZ_ROBOT->joints[num_jnt]->user_dof_equiv_nbr;
      num_jnt++;
    }
   robotPt->nb_ligand_dof = nb_dof;
}



void bio_set_num_subrobot_ligand(void)
{
  int num_jnt = XYZ_ROBOT->joints[0]->n_next_jnt;  // nb of joints under the P3D_BASE

  if(num_jnt < 2) {
      XYZ_ROBOT->num_subrobot_ligand = -1;    // case where there is no ligand
  }
  else {
    num_jnt = XYZ_ROBOT->joints[0]->next_jnt[0]->n_next_jnt; 
    XYZ_ROBOT->num_subrobot_ligand = num_jnt;
  }
}


static int bio_set_num_subrobot_AA_from_AAjnt(p3d_rob* robotPt, 
						  p3d_jnt* jntPt) {
 int joint_number=0, num_subrobot =-1;
  p3d_jnt* cur_jnt;
  char AAtype[4]; 
 
  while(joint_number<robotPt->njoints) {
    cur_jnt = robotPt->joints[joint_number];
    if(cur_jnt == jntPt) {
      jntPt->num_subrobot = num_subrobot;
      return 1;
    }
    get_AAtype_from_name(cur_jnt->name,AAtype);     
    if(strcmp(AAtype,"cha")==0) {//the joint is a chain_base
      num_subrobot++;
    }
    joint_number++;
  }
  return 0;
}


int bio_set_num_subrobot_AA(void)
{
  pp3d_env envPt;
  p3d_rob *robotPt;
  p3d_rob **robots;
  int nrobots;
  int ir, ij;

  envPt = (p3d_env*) p3d_get_desc_curid(P3D_ENV);
  robots = envPt->robot;
  nrobots = p3d_get_desc_number(P3D_ROBOT);
  
  for (ir=0; ir < nrobots; ir++){
    robotPt = robots[ir];
    for (ij=0; ij <= robotPt->njoints; ij++){
      if(!bio_set_num_subrobot_AA_from_AAjnt(robotPt, robotPt->joints[ij])) {
	return 0;
      }
    }
  }
  return 1;
}

void bio_set_list_AA_first_jnt(void) {
  int cur_AAnumber, max_AAnum = -1, i;
  p3d_rob* robotPt = XYZ_ROBOT;
  p3d_jnt* cur_jnt;
  int joint_number=0;

  while(joint_number<robotPt->njoints) {
    cur_jnt = robotPt->joints[joint_number];
    cur_AAnumber =  get_AAnumber_from_jnt(cur_jnt);
    if(cur_AAnumber>max_AAnum){
      max_AAnum = cur_AAnumber;
    }
    joint_number++;
  }
  robotPt->list_AA_firstjnt = MY_ALLOC(p3d_jnt*, max_AAnum);
  for(i = 0; i< max_AAnum; i++) {
    robotPt->list_AA_firstjnt[i] =NULL;
  }
  joint_number=0;
  while(joint_number<robotPt->njoints) {
    cur_jnt = robotPt->joints[joint_number];
    cur_AAnumber =  get_AAnumber_from_jnt(cur_jnt);
    if(robotPt->list_AA_firstjnt[cur_AAnumber] == NULL) {
    robotPt->list_AA_firstjnt[cur_AAnumber] = cur_jnt;
    }
    joint_number++;
  }
}

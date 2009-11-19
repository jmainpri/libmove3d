#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"


hri_bitmapset * INTERPOINT = NULL;
hri_bitmapset * OBJSET = NULL;
int * orderedpointsx = NULL;
int * orderedpointsy = NULL;
int * orderedpointsz = NULL;
int orderedlength = 0;
int ordereddrawno = 0;
double HRI_WEIGHTS[5] =  {1,1,1,1,1};
int exP[3];

static double combine_costs(double* Costs, double* w, int l, int combine_type);
static void rp_on_halfdisc(double rmax, double rmin, p3d_matrix4 Tbase, double *x, double *y, double * angle);
static void rp_on_disc(double rmax, double rmin, p3d_matrix4 Tbase, double *x, double *y, double * angle);

static int step = 1;
static int jump = 1;

int hri_exp_get_robot_joint_object()
{
    return ROBOTj_OBJECT;
}

void hri_exp_set_exp_from_config(hri_bitmapset* btset, configPt q)
{
    double W_space_point[3];

    configPt q_saved;
    q_saved = p3d_get_robot_config(btset->robot);
    p3d_set_and_update_this_robot_conf(btset->robot,q);

    W_space_point[0] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[0][3];
    W_space_point[1] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[1][3];
    W_space_point[2] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[2][3];

    exP[0] = (int)((W_space_point[0]-btset->realx)/btset->pace);
    exP[1] = (int)((W_space_point[1]-btset->realy)/btset->pace);
    exP[2] = (int)((W_space_point[2]-btset->realz)/btset->pace);

    p3d_set_and_update_this_robot_conf(btset->robot,q_saved);
    p3d_destroy_config(btset->robot,q_saved);
}

double hri_exp_combined_val(hri_bitmapset* btset, int x, int y, int z)
{
  double Costs[3],result;

  if(hri_exp_obstacle_val(btset,x,y,z) == -2)
    return -1;

  Costs[0] = hri_exp_distance_val(btset,x,y,z);
  Costs[1] = hri_exp_vision_val(btset,x,y,z);
  Costs[2] = hri_exp_hcomfort_val(btset,x,y,z);
  // Costs[3] = hri_exp_rreach_val(btset,x,y,z);

  result = combine_costs(Costs,HRI_WEIGHTS,3,CMB_HRI_SUM);

  return result;
}

double hri_exp_path_val(hri_bitmapset* btset, int x, int y, int z)
{
  double Costs[2],result;

  Costs[0] = hri_exp_distance_val(btset,x,y,z);
  Costs[1] = hri_exp_vision_val(btset,x,y,z);

  result = combine_costs(Costs,HRI_WEIGHTS,2,CMB_HRI_SUM);

  return result;
}


static double combine_costs(double* Costs, double* w, int l, int combine_type)
{
  int i;
  double result=0;

  if(combine_type == CMB_HRI_SUM){
    for(i=0; i<l; i++){
      result += w[i] * Costs[i];
    }
  }
  else{
    PrintError(("combine type not defined"));
  }

  return result;
}

static double human_max_reach_length = 0;

double hri_exp_distance_val(hri_bitmapset * btset, int x, int y, int z)
{
  double hbody[3], hneck[3];
  double pointbodydist, pointneckdist;
  double point[3];
  double mindist[3];
  int i;

  if(human_max_reach_length == 0){
    human_max_reach_length = DISTANCE3D(btset->human[btset->actual_human]->HumanPt->joints[17]->abs_pos[0][3],
				  btset->human[btset->actual_human]->HumanPt->joints[17]->abs_pos[1][3],
				  btset->human[btset->actual_human]->HumanPt->joints[17]->abs_pos[2][3],
				  btset->human[btset->actual_human]->HumanPt->joints[25]->abs_pos[0][3],
				  btset->human[btset->actual_human]->HumanPt->joints[25]->abs_pos[1][3],
				  btset->human[btset->actual_human]->HumanPt->joints[25]->abs_pos[2][3])+
      DISTANCE3D(btset->human[btset->actual_human]->HumanPt->joints[25]->abs_pos[0][3],
		 btset->human[btset->actual_human]->HumanPt->joints[25]->abs_pos[1][3],
		 btset->human[btset->actual_human]->HumanPt->joints[25]->abs_pos[2][3],
		 btset->human[btset->actual_human]->HumanPt->joints[29]->abs_pos[0][3],
		 btset->human[btset->actual_human]->HumanPt->joints[29]->abs_pos[1][3],
		 btset->human[btset->actual_human]->HumanPt->joints[29]->abs_pos[2][3])+
      DISTANCE3D(btset->human[btset->actual_human]->HumanPt->joints[17]->abs_pos[0][3],
		 btset->human[btset->actual_human]->HumanPt->joints[17]->abs_pos[1][3],
		 btset->human[btset->actual_human]->HumanPt->joints[17]->abs_pos[2][3],
		 btset->human[btset->actual_human]->HumanPt->joints[7]->abs_pos[0][3],
		 btset->human[btset->actual_human]->HumanPt->joints[7]->abs_pos[1][3],
		 btset->human[btset->actual_human]->HumanPt->joints[7]->abs_pos[2][3])*cos(DTOR(30));

    printf("Human reach lengh: %f\n",human_max_reach_length);
  }

  point[0] = x * btset->pace + btset->realx;
  point[1] = y * btset->pace + btset->realy;
  point[2] = z * btset->pace + btset->realz;

  hbody[0] = btset->human[btset->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[0][3];
  hbody[1] = btset->human[btset->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[1][3];
  hbody[2] = btset->human[btset->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[2][3];

  pointbodydist = DISTANCE3D(point[0], point[1],point[2],hbody[0],hbody[1],hbody[2]);

  hneck[0] = btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos[0][3];
  hneck[1] = btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos[1][3];
  hneck[2] = btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos[2][3];

  pointneckdist = DISTANCE3D(point[0],point[1],point[2],hneck[0],hneck[1],hneck[2]);

  // Warning here
  human_max_reach_length = 1.5;
  if(pointneckdist > human_max_reach_length)
    return 0;


  if(pointneckdist < pointbodydist)
    for(i=0; i<3; i++)
      mindist[i] = ABS(point[i] - hneck[i]);
  else
    for(i=0; i<3; i++)
      mindist[i] = ABS(point[i] - hbody[i]);

  /* printf("mindistance values : %f %f %f\n",mindist[0],mindist[1],mindist[2]); */

  /* if(mindist[0]*4 > M_2PI || mindist[1]*4 > M_2PI || mindist[2]*4 > M_2PI || */
  /*      mindist[0]*4 < -M_2PI || mindist[1]*4 < -M_2PI || mindist[2]*4 < -M_2PI) */
  /*     return 0; */
  //return (human_max_reach_length - pointneckdist)/human_max_reach_length;
  return  cos((mindist[0]/human_max_reach_length)*M_PI_2)*
	  cos((mindist[1]/human_max_reach_length)*M_PI_2)*
	  cos((mindist[2]/human_max_reach_length)*M_PI_2);
  //return (cos(pointneckdist))*(cos(pointneckdist))*(cos(pointneckdist));

}


double hri_exp_vision_val(hri_bitmapset * btset, int x, int y, int z)
{
  double phi,theta;
  double Dphi, Dtheta;
  double Ccoord[6];
  p3d_vector4 realcoord,newcoord;
  p3d_matrix4 inv;

  realcoord[0] = x*btset->pace+btset->realx;
  realcoord[1] = y*btset->pace+btset->realy;
  realcoord[2] = z*btset->pace+btset->realz;
  realcoord[3] = 1;

  p3d_mat4ExtractPosReverseOrder(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos,
					 Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);

  p3d_matInvertXform(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos, inv);

  p3d_matvec4Mult(inv, realcoord, newcoord);

  p3d_psp_cartesian2spherical(newcoord[0],newcoord[1],newcoord[2],
							  0,0,0,&phi,&theta);

  phi = ABS(phi);
  theta = ABS(theta - M_PI_2);

  if(phi < HRI_EYE_TOLERANCE_PAN/2.)
    Dphi = 0;
  else
    Dphi = phi - HRI_EYE_TOLERANCE_PAN/2.;

  if(theta < HRI_EYE_TOLERANCE_TILT/2.)
    Dtheta = 0;
  else
    Dtheta = theta - HRI_EYE_TOLERANCE_TILT/2.;

  return 10*((Dtheta+Dphi)/(M_2PI-(HRI_EYE_TOLERANCE_TILT/2.)-(HRI_EYE_TOLERANCE_PAN/2.)))/0.65;
}

double hri_exp_rreach_val(hri_bitmapset * btset, int x, int y, int z)
{
  p3d_vector3 realcoord;
  p3d_vector3 objectcoord;

  realcoord[0] = x*btset->pace+btset->realx;
  realcoord[1] = y*btset->pace+btset->realy;
  realcoord[2] = z*btset->pace+btset->realz;

  objectcoord[0] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[0][3];
  objectcoord[1] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[1][3];
  objectcoord[2] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[2][3];

  return 10*(cos(objectcoord[0]-realcoord[0]*2)+1)*
    (cos(objectcoord[1]-realcoord[1]*2)+1)*
    (cos(objectcoord[2]-realcoord[2]*2)+1)/8;

}

double hri_exp_hcomfort_val(hri_bitmapset * btset, int x, int y, int z)
{
  double val;

  val = MIN(hri_exp_hlreach_val(btset,x,y,z),
			hri_exp_hrreach_val(btset,x,y,z)+0.2); /* penalty for the right arm as the human is left handed */
  return 10*val;

}


double hri_exp_hlreach_val(hri_bitmapset * btset, int x, int y, int z)
{
  configPt humanC,savedConf;
  double cost=0, potential = 0;
  double restq1 = DTOR(-60),restq2 = DTOR(-19),restq3 = DTOR(-10),restq4 = DTOR(40);
  p3d_rob *r;
  double min,max;
  int i;
  p3d_matrix4 Tgrip,Tinv,Tdiff,TrotBase;
  double q[7];
  p3d_jnt * pasjnts[7];
  p3d_vector4 realcoord,newcoord;
  p3d_matrix4 inv;
  double dis1,dis2;

  p3d_matrix4 Tatt = {{ 0, 0, 1, 0},
  {-1, 0, 0, 0},
  { 0, 1, 0, 0},
  { 0, 0, 0, 1}};

  p3d_matrix4 Tatt2 = {{1, 0, 0, 0},
  {0,-1, 0, 0},
  {0, 0,-1, 0},
  {0, 0, 0, 1}};

  p3d_matrix4 Tejemp = {{ 0, 0, 1, 0},
  { 0, 1, 0, 0},
  {-1, 0, 0, 0},
  { 0, 0, 0, 1}};

  p3d_matrix4 Tbase = {{ 1, 0, 0, 0},
  { 0, 1, 0, 0},
  { 0, 0, 1, 0},
  { 0, 0, 0, 1}};

  realcoord[0] = x*btset->pace+btset->realx;
  realcoord[1] = y*btset->pace+btset->realy;
  realcoord[2] = z*btset->pace+btset->realz;
  realcoord[3] = 1;

  if(realcoord[0]==0.3){
	printf("crap");
	}

  p3d_matInvertXform(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos, inv);
  inv[2][3] = 0;
  p3d_matvec4Mult(inv, realcoord, newcoord);

  r = btset->human[btset->actual_human]->HumanPt;

  pasjnts[0] = r->joints[14];
  pasjnts[1] = r->joints[15];
  pasjnts[2] = r->joints[16];
  pasjnts[3] = r->joints[22];
  pasjnts[4] = r->joints[26];
  pasjnts[5] = r->joints[27];
  pasjnts[6] = r->joints[28];

  // shoulder to elbow distance
  dis1 = sqrt(SQR(pasjnts[3]->p0.x - pasjnts[0]->p0.x)+SQR(pasjnts[3]->p0.y - pasjnts[0]->p0.y)+SQR(pasjnts[3]->p0.z - pasjnts[0]->p0.z));

  // elbow to wrist distance
  dis2 = sqrt(SQR(pasjnts[4]->p0.x - pasjnts[3]->p0.x)+SQR(pasjnts[4]->p0.y - pasjnts[3]->p0.y)+SQR(pasjnts[4]->p0.z - pasjnts[3]->p0.z));

  humanC = p3d_get_robot_config(r);
  savedConf = p3d_copy_config(r, humanC);

  for(i=0; i<r->nb_dof; i++)
	humanC[i] = 0;

  humanC[8] = 0.86;

  humanC[66] = newcoord[0];
  humanC[67] = newcoord[1];
  humanC[68] = newcoord[2];
  humanC[71] = humanC[11];

  p3d_set_robot_config(r, humanC);
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);

  p3d_mat4Mult(r->joints[57]->abs_pos,Tatt,Tgrip);

  Tbase[0][3] = r->joints[16]->abs_pos[0][3];
  Tbase[1][3] = r->joints[16]->abs_pos[1][3];
  Tbase[2][3] = r->joints[16]->abs_pos[2][3];
  p3d_mat4Mult(Tbase,Tatt2,TrotBase);

  p3d_matInvertXform(r->joints[14]->abs_pos_before_jnt,Tinv);
  p3d_mat4Mult(Tejemp,Tinv,Tdiff);

  p3d_set_and_update_this_robot_conf(r, savedConf);

  if(compute_inverse_kinematics_R7_human_arm (q,Tgrip,TrotBase,Tdiff,
											  0, dis1,dis2,0)){
    for(i=0; i<4; i++) {
      p3d_jnt_get_dof_bounds(pasjnts[i],0, &min, &max);
      if((q[i] <= max)&&(q[i] >= min)) {
		p3d_jnt_set_dof(pasjnts[i],0, q[i]);
      }
      else {
		p3d_set_and_update_this_robot_conf(r, savedConf);
		p3d_destroy_config(r,humanC);
		p3d_destroy_config(r,savedConf);
		return 1;
      }
    }

    cost = (SQR(q[1]-restq1) + SQR(q[2]-restq2) +
			SQR(q[3]-restq3) + SQR(q[4]-restq4)-0.76)/23.4;

    humanC[pasjnts[0]->index_dof] = q[0];
    humanC[pasjnts[1]->index_dof] = q[1];
    humanC[pasjnts[2]->index_dof] = q[2];
    humanC[pasjnts[3]->index_dof] = q[3];

    p3d_set_and_update_this_robot_conf(r, humanC);

    potential = (btset->human[btset->actual_human]->HumanPt->joints[16]->abs_pos[2][3]+
				 btset->human[btset->actual_human]->HumanPt->joints[22]->abs_pos[2][3]-2.43)/0.67;

    p3d_set_and_update_this_robot_conf(r, savedConf);
    p3d_destroy_config(r,humanC);
    p3d_destroy_config(r,savedConf);
    return ABS(cost+potential)/2;
  }
  else{
    p3d_destroy_config(r,humanC);
    p3d_destroy_config(r,savedConf);
    return 1;
  }

}

double hri_exp_hrreach_val(hri_bitmapset * btset, int x, int y, int z)
{
  configPt humanC,savedConf;
  double cost=0,potential=0;
  double restq1 = DTOR(-60),restq2 = DTOR(-19),restq3 = DTOR(-10),restq4 = DTOR(40);
  p3d_rob *r;
  double min,max;
  int i;
  p3d_matrix4 Tgrip,Tinv,Tdiff,TrotBase;
  double q[7];
  p3d_jnt * pasjnts[7];
  p3d_vector4 realcoord,newcoord;
  p3d_matrix4 inv;
  double dis1,dis2;

  p3d_matrix4 Tatt = {{ 0, 0, 1, 0},
  { 1, 0, 0, 0},
  { 0, 1, 0, 0},
  { 0, 0, 0, 1}};

  p3d_matrix4 Tatt2 = {{1, 0, 0, 0},
  {0,-1, 0, 0},
  {0, 0,-1, 0},
  {0, 0, 0, 1}};

  p3d_matrix4 Tejemp = {{ 0, 0, 1, 0},
  { 0, 1, 0, 0},
  {-1, 0, 0, 0},
  { 0, 0, 0, 1}};

  p3d_matrix4 Tbase = {{ 1, 0, 0, 0},
  { 0, 1, 0, 0},
  { 0, 0, 1, 0},
  { 0, 0, 0, 1}};

  r = btset->human[btset->actual_human]->HumanPt;

  realcoord[0] = x*btset->pace+btset->realx;
  realcoord[1] = y*btset->pace+btset->realy;
  realcoord[2] = z*btset->pace+btset->realz;
  realcoord[3] = 1;

  p3d_matInvertXform(r->joints[HUMANj_BODY]->abs_pos, inv);
  /* inv[2][3]-= btset->human[btset->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[2][3]- */
  /* btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos[2][3];  */
  inv[2][3] = 0;
  p3d_matvec4Mult(inv, realcoord, newcoord);

  newcoord[1] = -newcoord[1];

  pasjnts[0] = r->joints[14];
  pasjnts[1] = r->joints[15];
  pasjnts[2] = r->joints[16];
  pasjnts[3] = r->joints[22];
  pasjnts[4] = r->joints[26];
  pasjnts[5] = r->joints[27];
  pasjnts[6] = r->joints[28];

  // shoulder to elbow distance
  dis1 = sqrt(SQR(pasjnts[3]->p0.x - pasjnts[0]->p0.x)+
			  SQR(pasjnts[3]->p0.y - pasjnts[0]->p0.y)+
			  SQR(pasjnts[3]->p0.z - pasjnts[0]->p0.z));

  // elbow to wrist distance
  dis2 = sqrt(SQR(pasjnts[4]->p0.x - pasjnts[3]->p0.x)+
			  SQR(pasjnts[4]->p0.y - pasjnts[3]->p0.y)+
			  SQR(pasjnts[4]->p0.z - pasjnts[3]->p0.z));


  humanC = p3d_get_robot_config(r);
  savedConf = p3d_copy_config(r, humanC);

  for(i=0; i<r->nb_dof; i++)
    humanC[i] = 0;
  humanC[8] = r->joints[1]->abs_pos[2][3];

  humanC[66] = newcoord[0];
  humanC[67] = newcoord[1];
  humanC[68] = newcoord[2];
  humanC[71] = humanC[11];

  p3d_set_and_update_this_robot_conf(r, humanC);
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);

  p3d_mat4Mult(r->joints[57]->abs_pos,Tatt,Tgrip);

  Tbase[0][3] = r->joints[16]->abs_pos[0][3];
  Tbase[1][3] = r->joints[16]->abs_pos[1][3];
  Tbase[2][3] = r->joints[16]->abs_pos[2][3];
  p3d_mat4Mult(Tbase,Tatt2,TrotBase);

  p3d_matInvertXform(r->joints[17]->abs_pos_before_jnt,Tinv);
  p3d_mat4Mult(Tejemp,Tinv,Tdiff);

  p3d_set_and_update_this_robot_conf(r, savedConf);

  if(compute_inverse_kinematics_R7_human_arm (q,Tgrip,TrotBase,Tdiff,
											  0, dis1, dis2,0)){
    for(i=0; i<4; i++) {
      p3d_jnt_get_dof_bounds(pasjnts[i],0, &min, &max);
      if((q[i] <= max)&&(q[i] >= min)) {
		p3d_jnt_set_dof(pasjnts[i],0, q[i]);
      }
      else {
		p3d_set_and_update_this_robot_conf(r, savedConf);
		p3d_destroy_config(r,humanC);
		p3d_destroy_config(r,savedConf);
		return 1;
      }
    }

    cost = (SQR(q[1]-restq1) + SQR(q[2]-restq2) +
			SQR(q[3]-restq3) + SQR(q[4]-restq4)-0.76)/23.4;

    humanC[pasjnts[0]->index_dof] = q[0];
    humanC[pasjnts[1]->index_dof] = q[1];
    humanC[pasjnts[2]->index_dof] = q[2];
    humanC[pasjnts[3]->index_dof] = q[3];

    p3d_set_and_update_this_robot_conf(r, humanC);

    potential = (btset->human[btset->actual_human]->HumanPt->joints[16]->abs_pos[2][3]+
				 btset->human[btset->actual_human]->HumanPt->joints[22]->abs_pos[2][3]-2.43)/0.67;

    p3d_set_and_update_this_robot_conf(r, savedConf);
    p3d_destroy_config(r,humanC);
    p3d_destroy_config(r,savedConf);
    return ABS(cost+potential)/2;

  }
  else{
    p3d_set_and_update_this_robot_conf(r, savedConf);
    p3d_destroy_config(r,humanC);
    p3d_destroy_config(r,savedConf);
    return 1;
  }

}

double hri_exp_obstacle_val(hri_bitmapset * btset, int x, int y, int z)
{
  if(btset == NULL){
    PrintWarning(("Cant get obstacle value: btset=null"));
    return -1;
  }
  if(btset->bitmap[BT_3D_OBSTACLES] == NULL){
    PrintWarning(("Cant get obstacle value: bitmap=null"));
    return -1;
  }

  if(btset->bitmap[BT_3D_OBSTACLES]->data[x][y][z].val == -1)
    return -1;
  else
    return 0;

}


int hri_exp_fill_obstacles(hri_bitmapset * btset)
{
  p3d_env* env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;

  if(btset == NULL || btset->bitmap[BT_3D_OBSTACLES] == NULL){
    PrintWarning(("Obstacle bitmap is null"));
    return FALSE;
  }

  hri_bt_reset_bitmap_data(btset->bitmap[BT_3D_OBSTACLES]);

  for(i=0; i<env->no ; i++){
    hri_bt_insert_obs(btset,btset->bitmap[BT_3D_OBSTACLES], env->o[i], env, 0, -1,1);
  }

  return TRUE;

}


double NEXT_POINT[3];

int hri_exp_find_manip_path(hri_bitmapset * btset)
{
  int x,y,z;
  double start[3], goal[3];

  x = exP[0];
  y = exP[1];
  z = exP[2];

  //hri_bt_min_cell_limited(btset,btset->bitmap[BT_3D_COMBINED],&x,&y,&z,1,1,1);
  start[0] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[0][3];
  start[1] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[1][3];
  start[2] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[2][3];

  goal[0] = x * btset->pace + btset->realx;
  goal[1] = y * btset->pace + btset->realy;
  goal[2] = z * btset->pace + btset->realz;


  if(hri_bt_calculate_bitmap_pathwGIK(btset,start,goal, TRUE)) {
    btset->bitmap[BT_3D_PATH]->active=TRUE;
    return TRUE;
  } else {
    return FALSE;
  }
}

p3d_rob * bottle[] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};

int * opx=NULL, *opy=NULL,* opz=NULL;
double *costt =NULL;
int ol=0;

int hri_exp_find_10_exchange_point(hri_bitmapset * btset)
{
  int x,y,z;
  configPt visballConf;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i,j;

  if(bottle[0] == NULL){
	j=0;
	for(i=0; i<env->nr; i++){
	  if( !strncmp("bottle",env->robot[i]->name,6) ){
		bottle[j] = env->robot[i];
		j++;
		if(j==10)
		  break;

	  }
	}
  }

  costt = MY_ALLOC(double,10);

  if(opx == NULL)
	opx =  MY_ALLOC(int,10);
  if(opy == NULL)
	opy =  MY_ALLOC(int,10);
  if(opz == NULL)
	opz =  MY_ALLOC(int,10);
  ol = 10;

  hri_bt_min_cell_n(btset,btset->bitmap[BT_3D_COMBINED],
					opx,opy,opz,costt,ol);

  for(i=0; i<10; i++){
	visballConf = p3d_get_robot_config(bottle[i]);

	visballConf[6] = opx[i]*btset->pace+btset->realx;
	visballConf[7] = opy[i]*btset->pace+btset->realy;
	visballConf[8] = opz[i]*btset->pace+btset->realz;

	p3d_set_and_update_this_robot_conf(bottle[i],visballConf);
  }

  return TRUE;

}


int hri_exp_find_exchange_point()
{
  int x,y,z;
  configPt visballConf;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;
  p3d_rob * goal = NULL;


  for(i=0; i<env->nr; i++){
    if( !strcmp("bottle",env->robot[i]->name) ){
      goal = env->robot[i];
      break;
    }
  }

  hri_bt_min_cell_limited(INTERPOINT,INTERPOINT->bitmap[BT_3D_COMBINED],&x,&y,&z,1,1,1);

  visballConf = p3d_get_robot_config(goal);

  exP[0] = x; visballConf[6] = x*INTERPOINT->pace+INTERPOINT->realx;
  exP[1] = y; visballConf[7] = y*INTERPOINT->pace+INTERPOINT->realy;
  exP[2] = z; visballConf[8] = z*INTERPOINT->pace+INTERPOINT->realz;

  printf("Bottle Coords: %f %f %f\n",visballConf[6],visballConf[7],visballConf[8]);

  p3d_set_and_update_this_robot_conf(goal,visballConf);


  return TRUE;

}


hri_bitmapset* hri_exp_init()
{
  int dimx,dimy, dimz;
  configPt humanConf;
  double hx,hy,hz;
  double Ccoord[6];
  hri_bitmapset * btset;
  double xsize=6,ysize=6,zsize=3;


  btset = hri_bt_create_bitmaps();

  btset->pace = BT_3D_SAMPLING;

  dimx = (int)(xsize/BT_3D_SAMPLING);
  dimy = (int)(ysize/BT_3D_SAMPLING);
  dimz = (int)(zsize/BT_3D_SAMPLING);

  humanConf = p3d_get_robot_config(btset->human[btset->actual_human]->HumanPt);

  hx = humanConf[6] - (xsize/2.);
  hy = humanConf[7] - (ysize/2.);

  p3d_destroy_config(btset->human[btset->actual_human]->HumanPt,humanConf);

  p3d_mat4ExtractPosReverseOrder(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos,
					 Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);

  hz = Ccoord[2] - (zsize/2.);

  btset->bitmap = MY_ALLOC(hri_bitmap*,7);
  btset->n = 7;

  btset->bitmap[BT_3D_VISIBILITY] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_VISIBILITY,
						 hri_exp_vision_val);
  btset->bitmap[BT_3D_DISTANCE] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_DISTANCE,
						 hri_exp_distance_val);
  btset->bitmap[BT_3D_HCOMFORT] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_HCOMFORT,
						 hri_exp_hcomfort_val);
  btset->bitmap[BT_3D_RREACH] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_RREACH,
						 hri_exp_rreach_val);
  btset->bitmap[BT_3D_OBSTACLES] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_OBSTACLES,
						 hri_exp_obstacle_val);
  btset->bitmap[BT_3D_COMBINED] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_COMBINED,
						 hri_exp_combined_val);
  btset->bitmap[BT_3D_PATH] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_PATH,
						 hri_exp_path_val);

  btset->path = NULL;
  btset->pathexist = FALSE;
  btset->combine_type = BT_COMBINE_SUM; /* default value */
  btset->changed = FALSE;

  hri_bt_change_bitmap_position(btset,hx,hy,hz);

  //hri_exp_fill_obstacles(btset);

  return btset;
}


void hri_exp_save(hri_bitmapset* btset, hri_bitmap * bitmap, char * name, double excld)
{
  int i,j,k;
  FILE * f;

  f = fopen(name,"w");

  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){
				if(bitmap->data[i][j][k].val == excld)
					continue;
				fprintf(f,"%f %f %f %f\n",
								i*btset->pace+btset->realx,
								j*btset->pace+btset->realy,
								k*btset->pace+btset->realz,
								bitmap->data[i][j][k].val);
      }
      fprintf(f,"\n");
    }
    fprintf(f,"\n");
  }
  fclose(f);
}



void hri_exp_save_npoint(hri_bitmapset* btset,hri_bitmap * bitmap, char * name,int incl_zeros,int n)
{
  int i;
  FILE * f;
  int *x,*y,*z;
  double * cost;

  f = fopen(name,"w");

  cost = MY_ALLOC(double,n);
  x =  MY_ALLOC(int,n);
  y =  MY_ALLOC(int,n);
  z =  MY_ALLOC(int,n);

  hri_bt_min_cell_n(btset, bitmap,x, y, z,cost, n);

  for(i=0; i<n; i++){
    fprintf(f,"%f %f %f %f\n",
			x[i]*btset->pace+btset->realx,
			y[i]*btset->pace+btset->realy,
			z[i]*btset->pace+btset->realz,
			cost[i]);
    fprintf(f,"\n");
  }
  fclose(f);
}

void hri_exp_save_table(char * name, double * val, int n)
{
  int i;
  FILE * f;

  f = fopen(name,"w");

  for(i=0; i<n; i++){
    fprintf(f,"%d %f \n",i,val[i]);
  }
  fclose(f);
}

void hri_exp_save_4tables(char * name, int *x, int *y, int *z, double * val, int n)
{
  int i;
  FILE * f;

  f = fopen(name,"w");

  for(i=0; i<n; i++){
    fprintf(f,"%d %d %d %f\n\n",x[i],y[i],z[i],val[i]);
  }
  fclose(f);
}

void hri_exp_draw_ordered_points()
{
  int i;

  if(orderedlength == 0 || orderedpointsz == NULL || orderedpointsy == NULL || orderedpointsx == NULL)
    return;

  for(i=0; i<ordereddrawno; i++){

	g3d_drawOneLine(orderedpointsx[i]*INTERPOINT->pace+INTERPOINT->realx-0.01,orderedpointsy[i]*INTERPOINT->pace+INTERPOINT->realy,orderedpointsz[i]*INTERPOINT->pace+INTERPOINT->realz,
					orderedpointsx[i]*INTERPOINT->pace+INTERPOINT->realx+0.01,orderedpointsy[i]*INTERPOINT->pace+INTERPOINT->realy,orderedpointsz[i]*INTERPOINT->pace+INTERPOINT->realz,
					Blue,NULL);
    g3d_drawOneLine(orderedpointsx[i]*INTERPOINT->pace+INTERPOINT->realx,orderedpointsy[i]*INTERPOINT->pace+INTERPOINT->realy-0.01,orderedpointsz[i]*INTERPOINT->pace+INTERPOINT->realz,
					orderedpointsx[i]*INTERPOINT->pace+INTERPOINT->realx,orderedpointsy[i]*INTERPOINT->pace+INTERPOINT->realy+0.01,orderedpointsz[i]*INTERPOINT->pace+INTERPOINT->realz,
					Blue,NULL);
    g3d_drawOneLine(orderedpointsx[i]*INTERPOINT->pace+INTERPOINT->realx,orderedpointsy[i]*INTERPOINT->pace+INTERPOINT->realy,orderedpointsz[i]*INTERPOINT->pace+INTERPOINT->realz-0.01,
					orderedpointsx[i]*INTERPOINT->pace+INTERPOINT->realx,orderedpointsy[i]*INTERPOINT->pace+INTERPOINT->realy,orderedpointsz[i]*INTERPOINT->pace+INTERPOINT->realz+0.01,
					Blue,NULL);
  }

}

/***********************************************************************************/
/************************************ HRI RRT **************************************/
/***********************************************************************************/


int hri_exp_rrt_path(double *qs, int *iksols, int *iksolg, int (*fct_stop)(void),void (*fct_draw)(void))
{
  p3d_rob   *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph *G;
  p3d_node  *Ns=NULL,*Ng=NULL,*added_node=NULL;
  p3d_list_node *graph_node;
  configPt  qtest, q_s=NULL, q_g=NULL;
  int       inode=1,fail=1,ADDED,REACHED;
  double    tu,ts;
  int       *iksols_s=NULL,*iksolg_s=NULL;
  int x,y,z;
  double start[3], goal[3];
  double orient[3];
  int exp =0;

  if(INTERPOINT!=NULL){
    hri_bt_reset_path(INTERPOINT);
  }
  else{
    INTERPOINT = hri_exp_init();
  }
  if(BTGRAPH!=NULL){
    p3d_del_graph(BTGRAPH);
    BTGRAPH = NULL;
  }

  hri_bt_min_cell_limited(INTERPOINT,INTERPOINT->bitmap[BT_3D_COMBINED],&x,&y,&z,2,2,2);

  p3d_mat4ExtractPosReverseOrder(INTERPOINT->robot->joints[ROBOTj_OBJECT]->abs_pos,
		     &start[0], &start[1], &start[2], orient, orient+1, orient+2);

  /*   start[0] = INTERPOINT->robot->joints[ROBOTj_OBJECT]->abs_pos[0][3]; */
  /*   start[1] = INTERPOINT->robot->joints[ROBOTj_OBJECT]->abs_pos[1][3]; */
  /*   start[2] = INTERPOINT->robot->joints[ROBOTj_OBJECT]->abs_pos[2][3]; */

  goal[0] = x * INTERPOINT->pace + INTERPOINT->realx;
  goal[1] = y * INTERPOINT->pace + INTERPOINT->realy;
  goal[2] = z * INTERPOINT->pace + INTERPOINT->realz;

  if(!INTERPOINT->pathexist){
    if(hri_bt_start_search(start, goal, INTERPOINT, TRUE) == FALSE){ /* here we find the path */
      PrintInfo(("Object path not found! We stop.\n"));
      return(FALSE);
    }
    else{
      INTERPOINT->bitmap[BT_3D_PATH]->active=TRUE;
    }
  }

#ifndef JIDO
  if(!hri_bt_write_TRAJ(INTERPOINT,INTERPOINT->robot->joints[ROBOTj_RHAND])){
    PrintError(("Can't create the path structure! We stop.\n"));
    return FALSE;
  }
#endif

  /* if(!BTGRAPH)     G = hri_bt_create_graph(btset->robot); */
  /*   else             G = BTGRAPH; */
  /*   btset->robot->GRAPH = G; */
  /*   graph_node = G->nodes; */


  /* Avoid some stupid errors */
  if(qs == NULL){
    PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration initiale\n"));
    return(FALSE);
  }

  ChronoOn();

  if(!XYZ_GRAPH) { G = p3d_create_graph(); XYZ_GRAPH = G; }
  else            G = XYZ_GRAPH;
  XYZ_ROBOT = robotPt;
  /* If not, create them */
  if(Ns == NULL) {
    q_s = p3d_copy_config(robotPt, qs);
    p3d_copy_iksol(robotPt->cntrt_manager, robotPt->ikSolPos, &iksolg_s);
    Ns  = p3d_APInode_make_multisol(G,q_s,iksols_s);
  }

  p3d_insert_node(G,Ns);
  p3d_create_compco(G,Ns);
  Ns->type = ISOLATED;
  // Ns->attached_bt_path_node = 0; CBIO commented?

  if(p3d_link_node_graph(Ns,G)){
    PrintInfo(("qs reliee au graphe\n"));
  }
  else{
    p3d_APInode_expand(G,Ns,fct_stop,fct_draw);
  }
  printf("first node created\n");
  /* Initialize some data in the graph for the A* graph search */
  G->search_start = Ns;
  G->search_done = FALSE;

  ADDED = FALSE;
  REACHED = FALSE;

  /* While solution does not exists, insert new nodes with basic PRM or Visibility or RRT */
  while(!REACHED){

    /* SAMPLE THE PATH */
    /* for now we consider the path is sampled into bitmap cells */

    /* THIS IS WHERE WE STAND */
    while(exp < 1){
      printf("%d\n",exp);
      //RRT ADDED = hri_expand_start_hri_rrt(G,INTERPOINT,inode,&added_node,&REACHED,fct_stop) | ADDED ;
      ADDED = hri_expand_prm(G,INTERPOINT,inode,&added_node,&REACHED,fct_stop) | ADDED ;
      exp++;
    }
    //  ADDED = hri_expand_start_hri_rrt(G,INTERPOINT,inode,&added_node,&REACHED,fct_stop);
    exp = 0;
    if(ADDED) {
      inode = inode + 1;
      if (fct_draw) (*fct_draw)();
      printf("node added\n");
    }
    else {
      printf("node added\n");
      PrintInfo(("p3d_specific_learn : ECHEC a l'insertion d'un noeud\n"));
      break;
    }
  }
  G->search_goal = added_node;
  if(ADDED){
    p3d_copy_config_into(robotPt,added_node->q, &robotPt->ROBOT_GOTO);
  }
  PrintInfo(("Pour la creation de %d noeuds : ",inode));
  ChronoPrint("");
  ChronoTimes(&tu,&ts);
  G->time = G->time + tu;
  ChronoOff();
  p3d_print_info_graph(G);
  MY_ALLOC_INFO("After p3d_specific_learn");

  return(ADDED);
}

/****************************************************************/
/*!
* \brief expand the connected component of the start configuration as a RRT
 *
 * \param G the graph
 * \param rob the robot
 *
 * \return FALSE if the component has more than p3d_get_COMP_NODES() nodes
 */
/****************************************************************/
int hri_expand_start_hri_rrt(p3d_graph *G,hri_bitmapset * btset,
							 int inode,p3d_node ** added_node,
							 int *reached,int (*fct_stop)(void))
{
  configPt q, q_prev;
  int nnodemax = p3d_get_COMP_NODES();
  int Added = FALSE;
  int Stop = FALSE;
  int fail;
  q = p3d_alloc_config(G->rob);

	while(!Stop){
		if (fct_stop)
			if (!(*fct_stop)()){
				PrintInfo(("RRT building canceled\n"));
				p3d_destroy_config(G->rob, q);
				return FALSE;
			}
		Added = FALSE;
		Stop = TRUE;

		if(!hri_shoot_with_btset(G->rob,btset,inode,q)){
			PrintError(("HRI Shooting error!\n"));
			return FALSE;
		}

		if (G->search_start->comp->nnode < nnodemax){
		 	Added = hri_expand_one_hri_rrt(G, &(G->search_start->comp),added_node,inode,q);
	//	Added = hri_add_basic_node(G,btset,inode,&fail,q)	;
			Stop = Added;
		}
	}

  PrintInfo(("  %4d   \r",G->search_start->comp->nnode));
  p3d_destroy_config(G->rob, q);



  if(Added && inode == btset->path->length-2)
    *reached = TRUE;

  return Added;
}


int hri_shoot_with_btset(p3d_rob *robotPt, hri_bitmapset * btset, int inode, configPt q)
{
  configPt *Result;
  p3d_matrix4 BaseMatrix;
  int i=0;
  int shoot_success = FALSE, shoot_counter = 0;
  p3d_vector3 next_target[3];
	int report;

  if(btset->path == NULL)
    return FALSE;

  if(btset->path->length-1 < inode || inode < 0)
    return FALSE;

  p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &q);

  BaseMatrix[0][3] = btset->path->xcoord[inode];
  BaseMatrix[1][3] = btset->path->ycoord[inode];

  /* if(btset->path->length-1 == inode){
    p3d_mat4PosReverseOrder(BaseMatrix,btset->path->xcoord[inode],btset->path->ycoord[inode],0,
							0,0,0);
  rp_on_disc(1,0,BaseMatrix,&q[6],&q[7],&q[11]);
  }else{ */
  p3d_mat4PosReverseOrder(BaseMatrix,btset->path->xcoord[inode],btset->path->ycoord[inode],0,
						  0,0,atan2(btset->path->ycoord[inode]-btset->path->ycoord[inode-1],
									btset->path->xcoord[inode]-btset->path->xcoord[inode-1]));

  while(!shoot_success && shoot_counter < 10){

	rp_on_halfdisc(1,0,BaseMatrix,&q[6],&q[7],&q[11]);
	q[11] = atan2(btset->realy+btset->pace*btset->bitmap[0]->ny/2-q[7],btset->realx+btset->pace*btset->bitmap[0]->nx/2-q[6])
	  +p3d_random(-M_PI/6,M_PI/6);

	/* } */

	p3d_set_and_update_this_robot_conf(robotPt,q);

	for(i=0; i<3; i++){
	  next_target[i][0] = btset->path->xcoord[inode];
	  next_target[i][1] = btset->path->ycoord[inode];
	  next_target[i][2] = btset->path->zcoord[inode];
	}

		if(hri_gik_compute(robotPt, HRI_GIK, 200, 0.05, 1, 0, next_target,NULL,&q, NULL)){
			p3d_set_and_update_this_robot_conf(robotPt, q);
			if(p3d_col_test_robot(robotPt, 0))
				shoot_success = TRUE;
		}
		shoot_counter++;
  }

  if(jump==20){
		jump = 0;
		step++;
  }

  jump++;
  return TRUE;

}

int hri_expand_one_hri_rrt(p3d_graph *G, p3d_compco **CompPt, p3d_node **node, int inode, configPt q)
{
  int Added = FALSE;
  p3d_node *Nnear, *NewNode;
  p3d_compco *CompConnect, *NextCompConnect;
  int jcomp;
  double dist = 0;
  p3d_rob *rob = G->rob;


  /* THIS IS RRT */
  Nnear = hri_nearest_neighbor(rob,q,*CompPt);
  //FIXIT NewNode = hrm_extend_on_surface(rob,G,Nnear,q);
  //NewNode = hrm_extend(rob,G,0.005,Nnear,q);
  if (NewNode)
  {
	*CompPt = NewNode->comp;
	Added = TRUE;
	CompConnect = G->comp;
	for(jcomp=0;(jcomp<G->ncomp)&&(CompConnect->num<=G->ncomp);jcomp++)
	{
	  NextCompConnect = CompConnect->suiv;
	  if (CompConnect->num!=(*CompPt)->num)
	  {
	Nnear = hri_nearest_neighbor(rob,NewNode->q,CompConnect);
		if (p3d_APInode_linked(G,NewNode,Nnear,&dist))
		{
		  PrintInfo(("RRT linking\n"));
		  if((*CompPt)->num<CompConnect->num)
		    p3d_merge_comp(G,*CompPt,&CompConnect);
		  else
		  {
			p3d_merge_comp(G,CompConnect,CompPt);
			*CompPt = CompConnect;
		  }

		  p3d_create_edges(G,Nnear,NewNode,dist);
		}
	  }
	  CompConnect = NextCompConnect;
	}
  }
  *node = NewNode;

  /* THIS IS PRM */


  return Added;
}

int hri_shoot(p3d_rob *robotPt, configPt q)
{
  configPt *Result;
  double dist = sqrt(SQR(robotPt->ROBOT_GOTO[7]-robotPt->ROBOT_POS[7])+
					 SQR(robotPt->ROBOT_GOTO[6]-robotPt->ROBOT_POS[6]));
  p3d_matrix4 BaseMatrix;

  p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &q);

  BaseMatrix[0][3] = robotPt->ROBOT_POS[6]+
    ((robotPt->ROBOT_GOTO[6]-robotPt->ROBOT_POS[6])*step*0.1);
  BaseMatrix[1][3] = robotPt->ROBOT_POS[7]+
    ((robotPt->ROBOT_GOTO[7]-robotPt->ROBOT_POS[7])*step*0.1);


  rp_on_disc(0.5,0,BaseMatrix,&q[6],&q[7],&q[11]);

  q[6] = BaseMatrix[0][3];
  q[7] = BaseMatrix[1][3];

  if(jump==20){
    jump = 0;
    step++;
  }

  jump++;
  return TRUE;

}

static void rp_on_halfdisc(double rmax, double rmin, p3d_matrix4 Tbase, double *x, double *y, double *angle)
{
  double x_b,y_b,z_b;
  double orient[3];
  double r,theta;

  p3d_mat4ExtractPosReverseOrder(Tbase,&x_b, &y_b, &z_b, orient, orient+1, orient+2);
  r = p3d_random(rmin,rmax);
  theta = p3d_random(-M_PI/2,M_PI/2);
  theta += orient[2];
  *angle = p3d_random(-M_PI/6,M_PI/6);

  *x = r * cos(theta) + x_b;
  *y = r * sin(theta) + y_b;

  *angle = 0 + atan2(y_b-(*y),x_b-(*x));
  *angle = /**angle +*/ orient[2];
}

static void rp_on_disc(double rmax, double rmin, p3d_matrix4 Tbase, double *x, double *y, double *angle)
{
  double x_b,y_b;
  double r,theta;

  x_b = Tbase[0][3];
  y_b = Tbase[1][3];
  r = p3d_random(rmin,rmax);
  theta = p3d_random(-M_PI,M_PI);
  /*  theta_z = atan2(z_y,z_x) - M_PI; */
  /*  theta += theta_z; */
  *angle = p3d_random(-M_PI,M_PI);
  *x = r * cos(theta) + x_b;
  *y = r * sin(theta) + y_b;
}

p3d_node *hri_nearest_neighbor(p3d_rob *rob, configPt q, p3d_compco *comp)
{
  p3d_list_node *ListNode = comp->dist_nodes;
  p3d_node *Nmin = NULL;
  double dmin = P3D_HUGE, d;

  while(ListNode != NULL) {
    //FIXIT  d = hrm_dist_config(rob, q,ListNode->N->q);
    if (d<dmin) {
      dmin = d;
      Nmin = ListNode->N;
    }
    ListNode = ListNode->next;
  }

  return Nmin;
}


int hri_expand_prm(p3d_graph *G,hri_bitmapset * btset,int inode,p3d_node ** added_node,int *reached,int (*fct_stop)(void))
{
  configPt q, q_prev;
  int nnodemax = p3d_get_COMP_NODES();
  int Added = FALSE;
  int Stop = FALSE;
  int fail;
  q = p3d_alloc_config(G->rob);

  while(!Stop){
    if (fct_stop)
      if (!(*fct_stop)()){
	PrintInfo(("PRM building canceled\n"));
	p3d_destroy_config(G->rob, q);
	return FALSE;
      }
    Added = FALSE;
    Stop = TRUE;

    if(!hri_shoot_with_btset(G->rob,btset,inode,q)){
      PrintError(("HRI Shooting error!\n"));
      return FALSE;
    }
    /* We have a good confguration q */
    if (G->search_start->comp->nnode < nnodemax){
      Added = hri_add_basic_node(G,btset,inode,added_node,&fail,q);
      Stop = Added;
    }
  }

  PrintInfo(("  %4d   \r",G->search_start->comp->nnode));
  p3d_destroy_config(G->rob, q);

  if(Added && inode == btset->path->length-2)
    *reached = TRUE;

  return Added;
}

int hri_add_basic_node(p3d_graph *G,  hri_bitmapset * btset, int inode, p3d_node ** added_node, int * fail, configPt q)
{
  p3d_node *N=NULL;
  p3d_list_edge * EdgeScan;
  p3d_list_compco * CompcoScan;
  int *iksol = NULL;

  /* if (p3d_local_is_oriented(G->rob->lpl_type)) */
/*     { G->oriented = TRUE; } */

  /* Create a node */
  N = p3d_APInode_make_multisol(G,q,iksol);

  p3d_insert_node(G,N);
  //N->attached_bt_path_node = inode; CBIO Commentd?

  /* Try connections with others compcos */
  hri_link_node_graph(N,G);

  if (N->numcomp == -1) {
    /* Node have not been included in a compco, create one for it */
    p3d_create_compco(G,N);
    if (G->oriented) {
      /* In the oriented case, some arcs may have been created, so update the lists of successors */
      EdgeScan = G->edges;
      while (EdgeScan) {
	if (EdgeScan->E->Ni == N) {
	  p3d_add_compco_to_reachable_list_and_update_predecessors(G, N->comp, EdgeScan->E->Nf->comp);
	  CompcoScan = EdgeScan->E->Nf->comp->canreach;
	  while (CompcoScan != NULL) {
	    p3d_add_compco_to_reachable_list_and_update_predecessors(G, N->comp, CompcoScan->comp);
	    CompcoScan = CompcoScan->next;
	  }
	}
	if (EdgeScan->E->Nf == N) {
	  p3d_add_compco_to_reachable_list_and_update_predecessors(G, EdgeScan->E->Ni->comp, N->comp );
	  CompcoScan = N->comp->canreach;
	  while (CompcoScan != NULL) {
	    p3d_add_compco_to_reachable_list_and_update_predecessors(G, EdgeScan->E->Ni->comp, CompcoScan->comp);
	    CompcoScan = CompcoScan->next;
	  }
	}
	EdgeScan = EdgeScan->next;
      }
    }
    p3d_merge_check(G);
  }
  *added_node = N;
  return(TRUE);
}

/***************************************************************/
/*!\fn int p3d_link_node_graph(p3d_node* Node, p3d_graph* Graph)
 * \brief try to link a node to the other connected component
 *
 * \param Node  the node to link
 * \param Graph the graph
 * \return number of linked components
 */
/***************************************************************/

int hri_link_node_graph(p3d_node* Node, p3d_graph* Graph)
{
  p3d_compco * Comp = Graph->comp;
  int nof_link = 0;
  /* For each compco of the graph */
  while(Comp) {
    if (Node->numcomp != Comp->num) {
      /* Try to connect the new node to the already existing compcos */
      if (hri_link_node_comp(Graph, Node, &Comp)) {
				nof_link++;
      }
    }
    if (Comp == NULL) Comp = Graph->comp;
    Comp = Comp->suiv;
  }
  return nof_link;
}

/*********************************************/
/* Fonction qui essaye de connecter un noeud */
/* a une composante connexe                  */
/* In : le graphe, le noeud, la composante   */
/* connexe                                   */
/* Out : relies ou non ?                     */
/*********************************************/
int hri_link_node_comp(p3d_graph *G, p3d_node *N, p3d_compco **compPt)
{
  p3d_compco * TargetComp = *compPt;
  double dist=0.;
  p3d_node * Nc = NULL;
  p3d_edge *e;
  p3d_list_node *list_node;
  int ValidForward, ValidBackward;

  /* If the criteria for choosing the best node in the target compco is */
  /* the distance, then node lists must be ordered */
  if (p3d_get_SORTING() == P3D_DIST_NODE) {
    list_node = TargetComp->dist_nodes;
    while (list_node != NULL) {
      list_node->N->dist_Nnew = p3d_APInode_dist(G,N,list_node->N);
      list_node = list_node->next;
    }
    p3d_order_node_list(TargetComp->dist_nodes);
  }

  /* Test the existence of a valid forward and backward path */
  list_node = TargetComp->dist_nodes;

  ValidBackward = ValidForward = FALSE;

  while (list_node != NULL) {

    Nc = list_node->N;

    if (p3d_get_SORTING() == P3D_DIST_NODE) {
      if ((Nc->dist_Nnew > p3d_get_DMAX()) && (p3d_get_DMAX() > 0.)) {
				return (FALSE);
      }
    }
    /* Oriented case, forward and backward paths must be separately tested */
    if (G->oriented) {
      if (ValidForward == FALSE) {
				if(p3d_APInode_linked(G,N,Nc,&dist)) {
					/* A forward path is found */
					p3d_create_one_edge(G,N,Nc,dist);
					ValidForward = TRUE;
				}
      }

      if (ValidBackward == FALSE) {
				if (p3d_APInode_linked(G,Nc,N,&dist)) {
					/* A bacward path is found */
					p3d_create_one_edge(G,Nc,N,dist);
					ValidBackward = TRUE;
				}
      }

      if (ValidBackward && ValidForward) {
				if (!N->comp) {
					/* A valid forward and backward path exist, and the node is still in none compco */
					/* so the tested compco will now include the new node */
					p3d_add_node_compco(N,TargetComp, TRUE);
				}
				else {
					/* A valid forward and backward path exist, and the node is already included in a compco */
					/* so the tested compco and the compco of the new node must merge */
					if(TargetComp->num > N->numcomp) {
						p3d_merge_comp(G, N->comp, compPt);
						*compPt = NULL;
					}
					else {
						p3d_merge_comp(G, TargetComp, &(N->comp));
					}
				}
				return TRUE;
      }
    }
    /* Non - oriented case, If the forward path is valid, the backward one is also valid. */
    else{
      if(hri_APInode_linked(G,N,Nc,&dist)) {
				p3d_create_edges(G,N,Nc,dist);
				/* If the node is still not included in a compco, it will be absorbed in the tested compco*/
				if (N->comp == NULL) {
					p3d_add_node_compco(N,TargetComp, TRUE);
				}
				/* Otherwise compcos merge */
				else {
					if(TargetComp->num > N->numcomp) {
						p3d_merge_comp(G, N->comp, compPt);
						*compPt = NULL;
					}
					else {
						p3d_merge_comp(G, TargetComp, &(N->comp));
					}
				}
				return(TRUE);
      }
    }
    list_node = list_node->next;
  }
  /* Non connexion has been found (in oriented case, arcs may have been created) */
  return(FALSE);
}

/**********************************************************
 * Function testant la connection entre deux noeuds
 * (remplace la fonction link())
 * Input:  the graph, the two nodes.
 * Output: whether connected or not, distance between nodes.
 ***********************************************************/

int hri_APInode_linked(p3d_graph *graphPt, p3d_node *N1,  p3d_node *N2, double *dist)
{
  p3d_rob *robotPt = graphPt->rob;
  p3d_localpath *localpathPt;
  int ntest=0,col;
  configPt qsave;

  /* current position of robot is saved */
  qsave = p3d_get_robot_config(robotPt);

  /* compute the local path using the local method associated to
	 the robot */
  localpathPt = p3d_local_planner(robotPt,N1->q,N2->q);

  if (localpathPt == NULL) { // Not valid localpath
    p3d_destroy_config(robotPt, qsave);
    return(FALSE);
  }

  if((p3d_get_SORTING()==P3D_NB_CONNECT)&&
     (p3d_get_MOTION_PLANNER()==P3D_BASIC)){
    if (localpathPt->length != NULL)
      *dist = localpathPt->length(robotPt,localpathPt);
    else{
      PrintInfo(("linked: no distance function specified\n"));
      *dist = 0;
    }

    if((*dist > p3d_get_DMAX())&&(LEQ(0.,p3d_get_DMAX()))){ /* ecremage deja fait dans le cas tri par distance... */
      /* the local path is destroyed */
      localpathPt->destroy(robotPt, localpathPt);
      localpathPt = NULL;

      /* The initial position of the robot is recovered */
      p3d_set_robot_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      return(FALSE);
    }
  }

  col = !p3d_unvalid_localpath_test(robotPt, localpathPt, &ntest);   // <- modif Juan
  localpathPt->destroy(robotPt, localpathPt);

  graphPt->nb_local_call = graphPt->nb_local_call +1;
  graphPt->nb_test_coll = graphPt->nb_test_coll + ntest;

  /* The initial position of the robot is recovered */
  p3d_set_robot_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  return(col);
}

/****** TEST FUNCTIONS ******/


hri_bitmapset* hri_object_reach_init(double objx, double objy, double objz)
{
  int dimx,dimy, dimz;
  configPt objConf;
  double hx,hy,hz;
  double Ccoord[6], Ccoord2[6];
  hri_bitmapset * btset;
  double xsize=6, ysize=6, zsize=3;


  btset = hri_bt_create_bitmaps();

  btset->pace = BT_3DR_SAMPLING;

  p3d_mat4ExtractPosReverseOrder(btset->robot->joints[ROBOTj_GRIP]->abs_pos,
		     Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);

  p3d_mat4ExtractPosReverseOrder(btset->robot->joints[5]->abs_pos,
		     Ccoord2, Ccoord2+1, Ccoord2+2,Ccoord2+3, Ccoord2+4, Ccoord2+5);

  xsize = (2*ABS(Ccoord[0]-objx) > 2*ABS(Ccoord2[0]-objx))?2*ABS(Ccoord[0]-objx):2*ABS(Ccoord2[0]-objx);
  ysize = (2*ABS(Ccoord[1]-objy) > 2*ABS(Ccoord2[1]-objy))?2*ABS(Ccoord[1]-objy):2*ABS(Ccoord2[1]-objy);
  zsize = ABS(0.40-1.90); /* Jido min et max reach height */

  btset->manip = BT_MANIP_REACH;
  dimx = (int)(xsize/BT_3DR_SAMPLING)+1;
  dimy = (int)(ysize/BT_3DR_SAMPLING)+1;
  dimz = (int)(zsize/BT_3DR_SAMPLING)+1;

  hx = objx - xsize/2;
  hy = objy - ysize/2;
  hz = 0.40;

  btset->bitmap = MY_ALLOC(hri_bitmap*,7);
  btset->n = 7;

  btset->bitmap[BT_3D_VISIBILITY] = NULL;
  btset->bitmap[BT_3D_DISTANCE] =  NULL;
  btset->bitmap[BT_3D_HCOMFORT] = NULL;
  btset->bitmap[BT_3D_RREACH] = NULL;
  btset->bitmap[BT_3D_OBSTACLES] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_OBSTACLES,
			 hri_exp_obstacle_val);
  btset->bitmap[BT_3D_COMBINED] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_COMBINED,
			 hri_exp_combined_val);
  btset->bitmap[BT_3D_PATH] =
    hri_bt_create_bitmap(dimx,dimy,dimz,BT_3D_SAMPLING,BT_3D_PATH,
			 hri_obj_reach_path_val);

  btset->path = NULL;
  btset->pathexist = FALSE;
  btset->combine_type = BT_COMBINE_SUM; /* default value */
  btset->changed = FALSE;

  hri_bt_change_bitmap_position(btset,hx,hy,hz);
  hri_bt_create_precise_obstacles(btset);
  //hri_exp_fill_obstacles(btset);

  return btset;
}

int hri_exp_find_obj_reach_path(hri_bitmapset * btset)
{
  double start[3], goal[3];
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);


  start[0] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[0][3];
  start[1] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[1][3];
  start[2] = btset->robot->joints[ROBOTj_OBJECT]->abs_pos[2][3];

  goal[0] = btset->object->joints[1]->abs_pos[0][3];
  goal[1] = btset->object->joints[1]->abs_pos[1][3];
  goal[2] = btset->object->joints[1]->abs_pos[2][3];

  if(hri_bt_calculate_bitmap_pathwR6IK(btset,start,goal, TRUE))
    btset->bitmap[BT_3D_PATH]->active=TRUE;

  return TRUE;
}


int hri_compute_R6IK(p3d_rob * robotPt, p3d_rob * objectPt, configPt  q )
{
  p3d_matrix4 invers,mat, ref, objrot,rotated;
  configPt qPt,qo;
  double qo10saved;
  Gb_6rParameters * bras;
  Gb_th * eth;
  Gb_q6 * old_q, * qarm2;
  Gb_dataMGD * d;
  int res, i;
  double r7 = 0.28;
  p3d_vector3 rot;
  int sign = 1;

  bras = MY_ALLOC(Gb_6rParameters,1);
  eth = MY_ALLOC(Gb_th,1);
  old_q = MY_ALLOC(Gb_q6,1);
  qarm2 = MY_ALLOC(Gb_q6,1);
  d = MY_ALLOC(Gb_dataMGD,1);

  p3d_mat4Copy(robotPt->joints[1]->abs_pos,ref);

  ref[0][3] = robotPt->joints[6]->abs_pos[0][3];
  ref[1][3] = robotPt->joints[6]->abs_pos[1][3];
  ref[2][3] = robotPt->joints[6]->abs_pos[2][3];

  bras->a2 = 0.45;
  bras->r4 = 0.48;
  bras->epsilon = 0.01;
  bras->of1 = M_PI;
  bras->of2 = -M_PI/2;
  bras->of3 = -M_PI/2;
  bras->of4 = 0;
  bras->of5 = 0;
  bras->of6 = 0;

  qPt = p3d_get_robot_config(robotPt);
  old_q->q1 = qPt[12];
  old_q->q2 = qPt[13];
  old_q->q3 = qPt[14];
  old_q->q4 = qPt[15];
  old_q->q5 = qPt[16];
  old_q->q6 = qPt[17];

  p3d_matInvertXform(ref, invers);

  qo = p3d_get_robot_config(objectPt);
  qo10saved = qo[10];

  for(i=0; i<10; i++){
    sign*=-1;

    qo[10] = qo[10]+i*sign*0.3;

    p3d_set_and_update_this_robot_conf(objectPt,qo);


    p3d_mat4Mult(invers,objectPt->joints[1]->abs_pos,mat);

    eth->vx.x = mat[0][0];
    eth->vx.y = mat[1][0];
    eth->vx.z = mat[2][0];

    eth->vy.x = mat[0][1];
    eth->vy.y = mat[1][1];
    eth->vy.z = mat[2][1];

    eth->vz.x = mat[0][2];
    eth->vz.y = mat[1][2];
    eth->vz.z = mat[2][2];

    eth->vp.x = mat[0][3] - r7 * mat[0][2];
    eth->vp.y = mat[1][3] - r7 * mat[1][2];
    eth->vp.z = mat[2][3] - r7 * mat[2][2];

    res = Gb_MGI6rTh(bras, eth, -1,  -1, -1, old_q, d, qarm2);


    q[12] = qarm2->q1;
    q[13] = qarm2->q2;
    q[14] = qarm2->q3;
    q[15] = qarm2->q4;
    q[16] = qarm2->q5;
    q[17] = qarm2->q6;

    p3d_set_and_update_this_robot_conf(robotPt,q);
    if(!p3d_col_test_robot_statics(robotPt,FALSE)){
      if(!p3d_col_test_self_collision(robotPt,FALSE))
	break;
      else
	continue;
    }
    else{
      continue;
    }
  }


  qo[10] = qo10saved;

  p3d_set_and_update_this_robot_conf(objectPt,qo);

  MY_FREE(bras, Gb_6rParameters,1);
  MY_FREE(eth,Gb_th ,1);
  MY_FREE(old_q,Gb_q6 ,1);
  MY_FREE(qarm2,Gb_q6 ,1);
  MY_FREE(d,Gb_dataMGD ,1);

  p3d_destroy_config(robotPt,qPt);
  p3d_destroy_config(objectPt,qo);

  if(i<10)
    return TRUE;
  else
    return FALSE;


}

double hri_obj_reach_path_val(hri_bitmapset* btset, int x, int y, int z)
{
  return 1;
}

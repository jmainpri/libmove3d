#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Bio-pkg.h"

static int INDEX_REF_FRAME = -1;
static int INDEX_MOB_FRAME = -1;

static p3d_matrix4 MOB_FRAME0 = {{1, 0, 0, 0},  {0, 1, 0, 0}, 
				 {0, 0, 1, 0}, {0, 0, 0, 1}};
static double WEIGHT_ROTA_DIST_FRAME = -1.;

/* p3d_SetMobFrame0
 * Set the value of the matrix of the mobile
 * frame MOB_FRAME0
 * @param[In]: MobileFrame0: the value of MOB_FRAME0
 */
void p3d_SetMobFrame0(p3d_matrix4 MobileFrame0)
{
  p3d_mat4Copy(MobileFrame0,MOB_FRAME0); 
}

/* p3d_GetMobFramePt
 * Get  the matrix of the mobile
 * frame MOB_FRAME0
 * @return: A pointer to the MOB_FRAME0
 */
void p3d_GetMobFramePt0(p3d_matrix4 **MobileFrame0Pt)
{
  *MobileFrame0Pt = &MOB_FRAME0; 
}

/* p3d_SetRefAndMobFrames
 * Set the index of the joints to which the reference
 * and the mobile frames are attached
 * @param[In]: IndexRefFrame: 
 * @param[In]: IndexMobFrame:
 */
void p3d_SetRefAndMobFrames(int IndexRefFrame, int IndexMobFrame) {
  INDEX_REF_FRAME = IndexRefFrame;
  INDEX_MOB_FRAME = IndexMobFrame;
}

/* p3d_GetRefAndMobFrames
 * Get pointers on the reference and mobile frames
 * @param[In]: robotPt: a pointer to the robot 
 * @param[Out]: RefFramePPt a pointer of pointer 
 * for the reference frame returned.
 * @param[Out]: MobFramePPt a pointer of pointer 
 * for the mobile frame returned.
 * @return: TRUE if at least the mobile 
 * frame has been setted. FALSE otherwise: 
 */
int p3d_GetRefAndMobFrames(p3d_rob *robotPt, p3d_matrix4** RefFramePPt, 
			      p3d_matrix4** MobFramePPt) {

  if((robotPt == NULL) ||
     ( (INDEX_REF_FRAME == -1) && (INDEX_MOB_FRAME == -1) ))
    return(FALSE);
  
  if(INDEX_REF_FRAME == -1) {
    (*RefFramePPt) = NULL;
  }
  else{
    if(INDEX_REF_FRAME < robotPt->njoints) {
      *RefFramePPt = &(robotPt->joints[INDEX_REF_FRAME]->abs_pos);
    }
    else {
      printf("ERROR: p3d_GetRefAndMobFrames: wrong INDEX_REF_FRAME \n");
      return(FALSE);
    }
  }
  if(INDEX_MOB_FRAME < robotPt->njoints) {
    (*MobFramePPt) = &(robotPt->joints[INDEX_MOB_FRAME]->abs_pos);
  }
  else {
    printf("ERROR: p3d_GetRefAndMobFrames: wrong INDEX_MOB_FRAME\n");
    return(FALSE);
  }
  
  return(TRUE);
}

/* p3d_SetWeightRotaFrameMetric
 * Set the WEIGHT_ROTA_DIST_FRAME to the originr value of the euclidian distance
 * between the origins of the mobile and reference frames
 * this distance is then used to weight the rotations in 
 * a specific metric between  frames.
 * @param[In]: robotPt: a pointer to the robot 
 */
void p3d_SetWeightRotaFrameMetric(p3d_rob * robotPt) {
  p3d_matrix4 *RefFramePt = NULL;
  p3d_matrix4 *MobFramePt = NULL;
  p3d_vector3 XyzRefFrame,XyzMobFrame,DeltaXyzFrames;

  if (p3d_GetRefAndMobFrames(robotPt, &RefFramePt, 
			     &MobFramePt) == FALSE ) {
    printf("WARNING: p3d_GetWeightRotaFrameMetric: to make the function  work\
 the mobile frame has at least to been defined\n");
    WEIGHT_ROTA_DIST_FRAME = -1.;
    return;

  }
  if(RefFramePt == NULL) {
    DeltaXyzFrames[0] = (*MobFramePt)[0][3];
    DeltaXyzFrames[1] = (*MobFramePt)[1][3];
    DeltaXyzFrames[2] = (*MobFramePt)[2][3];
  }  else {
    XyzRefFrame[0] = (*RefFramePt)[0][3];
    XyzRefFrame[1] = (*RefFramePt)[1][3];
    XyzRefFrame[2] = (*RefFramePt)[2][3];
    XyzMobFrame[0] = (*MobFramePt)[0][3];
    XyzMobFrame[1] = (*MobFramePt)[1][3];
    XyzMobFrame[2] = (*MobFramePt)[2][3];
    p3d_vectSub(XyzRefFrame,XyzMobFrame,DeltaXyzFrames);
  }
  WEIGHT_ROTA_DIST_FRAME = (double) p3d_vectNorm(DeltaXyzFrames);
  return;
}

/* p3d_GetWeightRotaFrameMetric
 * Get the value of the WEIGHT_ROTA_DIST_FRAME corresponding to the origine 
 * value of the euclidian distance
 * between the origins of the mobile and reference frames
 * this distance is then used to weight the rotations in 
 * a specific metric between  frames.
 * @param[In]: robotPt: a pointer to the robot 
 */
double p3d_GetWeightRotaFrameMetric(void) {
  return WEIGHT_ROTA_DIST_FRAME;
}

/* p3d_GetSe3DistanceFrames
 * Get the Se3 distance between two frames. It uses a specific 
 * metric combining the positions and the orientation of the frames.
 * the weight of the rotations is pondered thanks to the 
 * p3d_GetWeightRotaFrameMetric function.
 * @return: the Se3 distance between the two frames.
 */
double p3d_GetSe3DistanceFrames(p3d_rob * robotPt, p3d_matrix4 Frame1, 
				p3d_matrix4 Frame2) {
  p3d_vector3 XyzFrame1,XyzFrame2,ThetaXyzFrame1,ThetaXyzFrame2;
  p3d_vector3 posdiff,DeltaThetaXyz;
  double DeltaTransla,DeltaRota;
  double WeigtRota;
  double Se3Dist;
  int i;

  // NOTE : The distance is computed using a weighted metric 
  //        in rotation and translation.
  //        Consecutive rotations Rx,Ry,Rz are considered for the
  //        orientation.
  //        This probably not the best distance metric.
  //        Use quatenions !!??

  // Note2: It seems better to use dist_circle in state of the actual implementation
  // the implementation is slightly different than taking the sqrt of the sum of the squared
  // distances coordinate by coordinate, as it is done in the p3d_jnt_calc_dof_dist function

  p3d_mat4ExtractPosReverseOrder(Frame1,
				 &(XyzFrame1[0]),&(XyzFrame1[1]),&(XyzFrame1[2]),
				 &(ThetaXyzFrame1[0]),&(ThetaXyzFrame1[1]),&(ThetaXyzFrame1[2]));
  p3d_mat4ExtractPosReverseOrder(Frame2,
				 &(XyzFrame2[0]),&(XyzFrame2[1]),&(XyzFrame2[2]),
				 &(ThetaXyzFrame2[0]),&(ThetaXyzFrame2[1]),&(ThetaXyzFrame2[2]));

  p3d_vectSub(XyzFrame1,XyzFrame2,posdiff);
  DeltaTransla = (double) p3d_vectNorm(posdiff);
  p3d_vectSub(ThetaXyzFrame1,ThetaXyzFrame2,DeltaThetaXyz);
  for(i=0; i<3; i++) {
    if(DeltaThetaXyz[i] > M_PI)
      DeltaThetaXyz[i] -= (2.0*M_PI);
    else if(DeltaThetaXyz[i] < -M_PI)
      DeltaThetaXyz[i] += (2.0*M_PI);
  }    
  DeltaRota = (double) p3d_vectNorm(DeltaThetaXyz);

  WeigtRota = p3d_GetWeightRotaFrameMetric();  

  Se3Dist = DeltaTransla + (WeigtRota * DeltaRota);
  return(Se3Dist);
}

/* p3d_SetMobFrameToNode
 * Set the value of RelMobFrame to a node
 * TODO: a better explanation of what it is
 * @param[In]: graphPt: a pointer to the robot graph
 * @param[In]: nodePt: a pointer to the node for which 
 * the MobileFrame is set 
 */ 
void p3d_SetMobFrameToNode(p3d_graph *graphPt, p3d_node *nodePt) {
  p3d_matrix4* RefFramePt=NULL, *MobFramePt=NULL;
  p3d_matrix4 MobFrameRef;
  p3d_matrix4 InversedMatRefFrame;

  // NOTE: The mobile frame is refered to its initial location 
  //       The inital location corresponts to the root node
  //       (Currently implemented only for Ns !!!)

  if(p3d_GetRefAndMobFrames(graphPt->rob, &RefFramePt,&MobFramePt)) {
    if(RefFramePt == NULL) {
      p3d_mat4Copy(*MobFramePt,nodePt->RelMobFrame);
    }
    else {
      p3d_matInvertXform(*RefFramePt, InversedMatRefFrame);
      p3d_matMultXform(InversedMatRefFrame,*MobFramePt,MobFrameRef);      
      p3d_mat4Copy(MobFrameRef,nodePt->RelMobFrame);
    }
 }
}

// NOTE: this function returns the maximum difference
// between the orientation value (rx,ry,rz)
//       of both frames
// OTHER MEASURES ARE POSSIBLE !!!


/* p3d_MaxDiffOrientFrames
 * Compute the circle distance for all differences of orientation angles
 * of the 2 frames. 
 * Note: This function should probably use the dist_circle function
 * @param[In]: Frame1: the first frame to consider
 * @param[In]: Frame2: the second frame to consider
 * return: the maximal theta value between the frames (in radians)
 */ 
double p3d_MaxDiffOrientFrames(p3d_matrix4 Frame1, p3d_matrix4 Frame2)
{
  p3d_vector3 XyzPosFrame1,XyzPosFrame2;
  p3d_vector3 ThetaXyzFrame1, ThetaXyzFrame2;
  p3d_vector3 DeltaThetaXyzFrame;
  double DeltaThetaFrameMax = -P3D_HUGE;
  int i;

  p3d_mat4ExtractPosReverseOrder(Frame1,
				 &(XyzPosFrame1[0]),&(XyzPosFrame1[1]),&(XyzPosFrame1[2]),
				 &(ThetaXyzFrame1[0]),&(ThetaXyzFrame1[1]),&(ThetaXyzFrame1[2]));
  p3d_mat4ExtractPosReverseOrder(Frame2,
				 &(XyzPosFrame2[0]),&(XyzPosFrame2[1]),&(XyzPosFrame2[2]),
				 &(ThetaXyzFrame2[0]),&(ThetaXyzFrame2[1]),&(ThetaXyzFrame2[2]));

  p3d_vectSub(ThetaXyzFrame1,ThetaXyzFrame2,DeltaThetaXyzFrame);
  for(i=0; i<3; i++) {
    if(DeltaThetaXyzFrame[i] > M_PI)
      DeltaThetaXyzFrame[i] -= (2.0*M_PI);
    else if(DeltaThetaXyzFrame[i] < -M_PI)
      DeltaThetaXyzFrame[i] += (2.0*M_PI);
  }    
  for(i=0; i<3; i++) {
    if(fabs(DeltaThetaXyzFrame[i])>DeltaThetaFrameMax)
      DeltaThetaFrameMax = fabs(DeltaThetaXyzFrame[i]);
  }

  return(DeltaThetaFrameMax);
}

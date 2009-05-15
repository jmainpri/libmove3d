#ifndef __CEXTRACT__
/* p3d_SetMobFrame0
 * Set the value of the matrix of the mobile
 * frame MOB_FRAME0
 * @param[In]: MobileFrame0: the value of MOB_FRAME0
 */
void p3d_SetMobFrame0(p3d_matrix4 MobileFrame0);

/* p3d_GetMobFramePt
 * Get  the matrix of the mobile
 * frame MOB_FRAME0
 * @return: A pointer to the MOB_FRAME0
 */
void p3d_GetMobFramePt0(p3d_matrix4 **MobileFrame0Pt);

/* p3d_SetRefAndMobFrames
 * Set the index of the joints to which the reference
 * and the mobile frames are attached
 * @param[In]: IndexRefFrame: 
 * @param[In]: IndexMobFrame:
 */
void p3d_SetRefAndMobFrames(int IndexRefFrame, int IndexMobFrame);

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
			   p3d_matrix4** MobFramePPt);

/* p3d_GetWeightRotaFrameMetric
 * Get the value of the WEIGHT_ROTA_DIST_FRAME corresponding to the origine 
 * value of the euclidian distance
 * between the origins of the mobile and reference frames
 * this distance is then used to weight the rotations in 
 * a specific metric between  frames.
 * @param[In]: robotPt: a pointer to the robot 
 */
double p3d_GetWeightRotaFrameMetric(void);

/* p3d_SetWeightRotaFrameMetric
 * Set the WEIGHT_ROTA_DIST_FRAME to the originr value 
 *of the euclidian distance
 * between the origins of the mobile and reference frames
 * this distance is then used to weight the rotations in 
 * a specific metric between  frames.
 * @param[In]: robotPt: a pointer to the robot 
 */
void p3d_SetWeightRotaFrameMetric(p3d_rob * robotPt);

/* p3d_GetSe3DistanceFrames
 * Get the Se3 distance between two frames. It uses a specific 
 * metric combining the positions and the orientation of the frames.
 * the weight of the rotations is pondered thanks to the 
 * p3d_GetOriginDistFramesForMetric function.
 * @return: the Se3 distance between the two frames.
 */
double p3d_GetSe3DistanceFrames(p3d_rob * robotPt, 
				p3d_matrix4 Frame1, p3d_matrix4 Frame2);

/* p3d_SetMobFrameToNode
 * Set the value of RelMobFrame to a node
 * TODO: a better explanation of what it is
 * @param[In]: graphPt: a pointer to the robot graph
 * @param[In]: nodePt: a pointer to the node for which 
 * the MobileFrame is set 
 */ 
void p3d_SetMobFrameToNode(p3d_graph *graphPt, p3d_node *nodePt);


/* p3d_MaxDiffOrientFrames
 * Compute the circle distance for all differences of orientation angles
 * of the 2 frames. 
 * Note: This function should probably use the dist_circle function
 * @param[In]: Frame1: the first frame to consider
 * @param[In]: Frame2: the second frame to consider
 * return: the maximal theta value between the frames (in radians)
 */ 
double p3d_MaxDiffOrientFrames(p3d_matrix4 Frame1, p3d_matrix4 Frame2);



#endif

  #ifdef MULTILOCALPATH

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

/* TUNNEL_DIM defines the radius of the pipe within motion must be around the point to point localpath */
#define TUNNEL_DIM 0.05
/* VELOCITY_STEP_PERCENT defines the percentage to decrease module of velocity when there is a collision or when motion path over TUNNEL_DIM */
#define VELOCITY_STEP_PERCENT 0.1

static int factorToViewTrajectory = 1;
// variable to set the stay_within_dist value ; if 1 the step is 10ms else factorToViewTrajectory*10ms

// Functions specific to group
char * array_group_name[] =
{
 "base",
 "freeflyer",
 "joint"
};

int P3D_NB_GROUP = 3;

ptr_to_softMotion_groupplanner array_softMotion_groupplanner[]=
{
	(NULL),
	(int (*)(p3d_rob*, int, p3d_group_type, p3d_softMotion_data* ))(p3d_softMotion_localplanner_FREEFLYER),
	(int (*)(p3d_rob*, int, p3d_group_type, p3d_softMotion_data* ))(p3d_softMotion_localplanner_JOINT)
};


p3d_group_type p3d_group_getid_group(const char * name)
{
	int i;
	for(i=0; i<P3D_NB_GROUP; i++) {
		if (strcmp(name, array_group_name[i]) == 0)
		{ return (p3d_group_type)i; }
	}
	return (p3d_group_type)P3D_NULL_OBJ;
}

p3d_group_type p3d_group_getType_group(int nblpGp)
{
	p3d_rob * robotPt = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);

	return robotPt->mlp->mlpJoints[nblpGp]->gpType;
}

int p3d_group_planner(p3d_rob* robotPt, int nblpGp, p3d_group_type gpType, p3d_softMotion_data* softMotion_data)
{
	if(gpType >= P3D_NB_GROUP) {
		return FALSE;
	}
	return array_softMotion_groupplanner[gpType](robotPt, nblpGp, gpType, softMotion_data);
}


// End functions specific to group

p3d_softMotion_data * p3d_create_softMotion_data_multigraph(p3d_rob* robotPt, p3d_group_type gpType, int nbJoints, int mgID)
{
	p3d_softMotion_data * softMotion_data = NULL;
	psoftMotion_str softMotion_params = NULL;

	if ((softMotion_data = MY_ALLOC(p3d_softMotion_data,1)) == NULL) {
     return NULL;
	}
	softMotion_data->isPlanned = FALSE;
	softMotion_data->isPTP     = TRUE;
	softMotion_data->nbJoints  = nbJoints;
	softMotion_data->gpType    = gpType;
	softMotion_data->freeflyer   = NULL;
	softMotion_data->joint     = NULL;

	softMotion_data->q_init  = NULL;
	softMotion_data->q_end   = NULL;
	softMotion_data->q_endp1 = NULL;

	if(gpType == FREEFLYER) {
		if ((softMotion_data->freeflyer = MY_ALLOC(p3d_softMotion_data_FREEFLYER, 1)) == NULL) {
			return NULL;
		}

		Gb_v3_set(&softMotion_data->freeflyer->velLinInit, 0.0, 0.0, 0.0);
		Gb_v3_set(&softMotion_data->freeflyer->velAngInit, 0.0, 0.0, 0.0);
		Gb_v3_set(&softMotion_data->freeflyer->velLinEnd, 0.0, 0.0, 0.0);
		Gb_v3_set(&softMotion_data->freeflyer->velAngEnd, 0.0, 0.0, 0.0);

		softMotion_params = lm_get_softMotion_lm_param_multilocalpath(robotPt, mgID);

		/* Copy softMotion_params to Motion */
		softMotion_data->freeflyer->J_max_lin = softMotion_params->freeflyer->J_max_lin;
		softMotion_data->freeflyer->A_max_lin = softMotion_params->freeflyer->A_max_lin;
		softMotion_data->freeflyer->V_max_lin = softMotion_params->freeflyer->V_max_lin;
		softMotion_data->freeflyer->J_max_ang = softMotion_params->freeflyer->J_max_ang;
		softMotion_data->freeflyer->A_max_ang = softMotion_params->freeflyer->A_max_ang;
		softMotion_data->freeflyer->V_max_ang = softMotion_params->freeflyer->V_max_ang;
	}

	if(gpType == JOINT) {
		if ((softMotion_data->joint = MY_ALLOC(p3d_softMotion_data_joint, 1)) == NULL) {
			return NULL;
		}
		if ((softMotion_data->joint->J_max = MY_ALLOC(double, nbJoints)) == NULL) {
			PrintWarning(("  lm_create_softMotion: allocation failed\n"));
			return (NULL);
		}
		if ((softMotion_data->joint->A_max = MY_ALLOC(double, nbJoints)) == NULL) {
			PrintWarning(("  lm_create_softMotion: allocation failed\n"));
			return (NULL);
		}
		if ((softMotion_data->joint->V_max = MY_ALLOC(double, nbJoints)) == NULL) {
			PrintWarning(("  lm_create_softMotion: allocation failed\n"));
			return (NULL);
		}
		if ((softMotion_data->joint->motion = MY_ALLOC(SM_MOTION_MONO, nbJoints)) == NULL) {
			PrintWarning(("  lm_create_softMotion: allocation failed\n"));
			return (NULL);
		}
	}
	return softMotion_data;
}

/* allocation of local path of type softMotion */
p3d_localpath * p3d_alloc_softMotion_localpath(p3d_rob *robotPt,
																							 p3d_softMotion_data * sm_data,
																								int lp_id,
																								int is_valid)
{
  p3d_localpath * localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

	localpathPt->specific.softMotion_data = sm_data;

  /* Initialization of the generic part */
  /* fields */
  localpathPt->type_lp = SOFT_MOTION;
  localpathPt->valid = is_valid;
  localpathPt->lp_id = lp_id;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;

#ifdef MULTILOCALPATH
	localpathPt->mlpID = -1;

	for(int j=0; j< MAX_MULTILOCALPATH_NB ; j++) {
		localpathPt->mlpLocalpath[j] = NULL;
	}
#endif

	/* methods associated to the local path */
	/* compute the length of the local path */
	//localpathPt->length =
	//		(double (*)(p3d_rob*, p3d_localpath*))(p3d_lin_dist);
  /* extract from a local path a sub local path starting from length
	l1 and ending at length l2 */
	localpathPt->extract_sub_localpath =
			(p3d_localpath* (*)(p3d_rob*, p3d_localpath*, double, double))(p3d_extract_softMotion);
  /* extract from a local path a sub local path starting from parameter
	u1 and ending at parameter u2 */
	localpathPt->extract_by_param =
			(p3d_localpath* (*)(p3d_rob*, p3d_localpath*, double, double))(p3d_extract_softMotion);

	/* destroy the localpath */
	localpathPt->destroy =
			(void (*)(p3d_rob*, p3d_localpath*))(p3d_softMotion_destroy);
	/*copy the local path */
	localpathPt->copy =
			(p3d_localpath* (*)(p3d_rob*, p3d_localpath*))(p3d_copy_softMotion_localpath);
	/* computes the configuration at given distance along the path */
	localpathPt->config_at_distance =
			(configPt (*)(p3d_rob*, p3d_localpath*, double))(p3d_softMotion_config_at_param);
	/* computes the configuration at given parameter along the path */
	localpathPt->config_at_param =
			(configPt (*)(p3d_rob*, p3d_localpath*, double))(p3d_softMotion_config_at_param);
	  /* This function return the step in tick for range_param (1 tick here = 10 ms) */
	localpathPt->stay_within_dist =
			(double (*)(p3d_rob*, p3d_localpath*, double, whichway, double*))(p3d_softMotion_stay_within_dist);
	/* compute the cost of a local path */
	localpathPt->cost =
			(double (*)(p3d_rob*, p3d_localpath*))(p3d_softMotion_cost);
	  /* function that simplifies the sequence of two local paths: valid
	only for RS curves */
	localpathPt->simplify =
			(p3d_localpath* (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_softMotion);
	/* write the local path in a file */
	//localpathPt->write =
	//		(int (*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_lin);

	/* the length of the localpath is the time duration */

	switch (sm_data->gpType) {
		case FREEFLYER:
			localpathPt->length_lp = sm_data->freeflyer->motionTime;
			localpathPt->range_param = sm_data->freeflyer->motionTime;
			break;
		case JOINT:
			localpathPt->length_lp = sm_data->joint->motionTime;
			localpathPt->range_param = sm_data->joint->motionTime;
			break;
		default:
			localpathPt->length_lp = 0.0;
			localpathPt->range_param = 0.0;
			break;
	}
  localpathPt->ikSol = NULL;
  localpathPt->nbActiveCntrts = 0;
  localpathPt->activeCntrts = NULL;
	return localpathPt;
}

void p3d_softMotion_compute_tangent_velocity_values(configPt q1, configPt q2, configPt q3, Gb_v3* tangent, double* vel)
{
	/* unused for the moment */
	return;
}

/*
 * SoftMotion local planner
 *
 * Input:  the robot, the softMotion_data and three configurations
 *
 * Output: a local path.
 *
 * Allocation: the initial, goal and next goal config are copied
 */
p3d_localpath *p3d_softMotion_localplanner(p3d_rob *robotPt, int multiLocalpathID, p3d_softMotion_data* softMotion_data, configPt qi, configPt qf, configPt qfp1, int* ikSol)
{
	p3d_localpath *localpathPt;
//	psoftMotion_str softMotion_params;
	p3d_group_type gpType;

	/* on verifie que les configurations de depart et d'arrivee existent */
	if(qi == NULL){
		PrintInfo((("MP: p3d_softMotion_localplanner: no start configuration\n")));
		p3d_set_search_status(P3D_ILLEGAL_START);
		return(NULL);
	}
	if(qf == NULL){
		PrintInfo((("MP: p3d_softMotion_localplanner: no goal configuration\n")));
		p3d_set_search_status(P3D_ILLEGAL_GOAL);
		return(NULL);
	}
	if(qfp1 == NULL){
		PrintInfo((("MP: p3d_softMotion_localplanner: no goalp1 configuration\n")));
		p3d_set_search_status(P3D_ILLEGAL_GOAL);
		return(NULL);
	}

	if(p3d_get_search_verbose()){
		PrintInfo(("MP: p3d_softMotion_localplanner : "));
		PrintInfo(("qi=("));
		print_config(robotPt, qi);
		PrintInfo((") ; "));
		PrintInfo(("qf=("));
		print_config(robotPt, qf);
		PrintInfo(("=("));
		PrintInfo((")\n"));
	}

	softMotion_data->q_init  = p3d_copy_config(robotPt, qi);
	softMotion_data->q_end   = p3d_copy_config(robotPt, qf);
	softMotion_data->q_endp1 = p3d_copy_config(robotPt, qfp1);
	softMotion_data->isPlanned = FALSE;



	if (multiLocalpathID>=0 && multiLocalpathID<=robotPt->mlp->nblpGp) {

		gpType = p3d_group_getType_group(multiLocalpathID);
		softMotion_data->isPlanned = TRUE;

		if(p3d_group_planner(robotPt, multiLocalpathID, gpType, softMotion_data)!= TRUE) {
			return NULL;
		}

	} else {
		PrintError(("p3d_softMotion_lp: Wrong multiLocalpathID\n"));
		return NULL;
	}

	/* Allocation of the localpath */
	localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_data, 0, TRUE);

	/* Here localpathPt can't be NULL but just in case ... */
	if (localpathPt == NULL) {
		PrintError(("p3d softMotion localpath return NULL at the end\n"));
	}

	p3d_set_search_status(P3D_SUCCESS);
  localpathPt->ikSol = ikSol;
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
	return(localpathPt);
}

void lm_get_paramDiff_for_param(p3d_softMotion_data* softMotion_data, SM_SEGMENT* seg, int segId, int index, int param, int* paramDiff)
{
	if (segId == 0) {
		*paramDiff = param;
		return;
	}

	*paramDiff = param - softMotion_data->freeflyer->motion.TimeCumulM[index][segId];
	return;
}

void lm_compute_error_dist(p3d_softMotion_data* softMotion_data, double *errorDist)
{
	int i=0;
	int paramDiff = 0;
	int paramLocal = 0;
	int segId = 0;
	SM_SEGMENT segment[SM_NB_DIM];
	int step = 5;
	double dTmp= 0.0;
	double param = 0.0;
	Gb_v3 p1, p2, p3;
	SM_COND cond[SM_NB_DIM];
	*errorDist = 0.0;
	Gb_v3_set(&p1, softMotion_data->freeflyer->motion.IC[0].x, softMotion_data->freeflyer->motion.IC[1].x, softMotion_data->freeflyer->motion.IC[2].x);
	Gb_v3_set(&p2, softMotion_data->freeflyer->motion.FC[0].x, softMotion_data->freeflyer->motion.FC[1].x, softMotion_data->freeflyer->motion.FC[2].x);

	for(param=0; param<=softMotion_data->freeflyer->motionTime; param=(param+step)) {
		/*compute pose at param */
		for (i=0;i<6;i++) {
			lm_get_softMotion_segment_params_FREEFLYER( softMotion_data, param, &segment[i], &segId, i);
			if (param >= softMotion_data->freeflyer->motion.MotionDurationM[i]) {
				paramLocal = (int)softMotion_data->freeflyer->motion.MotionDurationM[i];
			}
			else {
				paramLocal = (int)param;
			}
			lm_get_paramDiff_for_param( softMotion_data, &segment[i], segId, i, paramLocal, &paramDiff);
			sm_CalculOfAccVelPosAtTime(paramDiff, &segment[i], &cond[i]);
		}

		Gb_v3_set( &p3, cond[0].x,  cond[1].x, cond[2].x);
		dTmp =  Gb_v3_dist_droite(&p1, &p2, &p3);
		if (dTmp > *errorDist) {
			*errorDist = dTmp;
		}
	}
	return;
}



void p3d_copy_softMotion_data_into(p3d_rob* robotPt, p3d_softMotion_data* sm_data1, p3d_softMotion_data* sm_data2)
{

	return;
}


void lm_destroy_softMotion_params(p3d_rob * robotPt, void *local_method_params)
{
	if (local_method_params != NULL){
		softMotion_str * paramPt = (softMotion_str *)local_method_params;
		if (paramPt->freeflyer != NULL){
			MY_FREE(paramPt->freeflyer, gp_freeflyer_params_str, 1);
		}
		if (paramPt->joint != NULL){
			if(paramPt->joint->J_max != NULL) {
				MY_FREE(paramPt->joint->J_max, double,  paramPt->nbJoints);
			}
			if(paramPt->joint->A_max != NULL) {
				MY_FREE(paramPt->joint->A_max, double,  paramPt->nbJoints);
			}
			if(paramPt->joint->V_max != NULL) {
				MY_FREE(paramPt->joint->V_max, double, paramPt->nbJoints);
			}
			MY_FREE(paramPt->joint, gp_joint_str, 1);
		}
		MY_FREE(paramPt, softMotion_str, 1);
	}
}


/*
 * destroys a structure of type p3d_softMotion_data
 */
void p3d_destroy_softMotion_data(p3d_rob* robotPt, p3d_softMotion_data* softMotion_dataPt)
{
	if (softMotion_dataPt != NULL){
		if (softMotion_dataPt->q_init != NULL){
			p3d_destroy_config(robotPt, softMotion_dataPt->q_init);
			p3d_destroy_config(robotPt, softMotion_dataPt->q_end);
			p3d_destroy_config(robotPt, softMotion_dataPt->q_endp1);
		}
		if(softMotion_dataPt->freeflyer != NULL){
			MY_FREE(softMotion_dataPt->freeflyer, p3d_softMotion_data_FREEFLYER, 1);
		}
		if(softMotion_dataPt->joint != NULL){
			if(softMotion_dataPt->joint->J_max != NULL) {
				MY_FREE(softMotion_dataPt->joint->J_max, double,  softMotion_dataPt->nbJoints);
			}
			if(softMotion_dataPt->joint->A_max != NULL) {
				MY_FREE(softMotion_dataPt->joint->A_max, double,  softMotion_dataPt->nbJoints);
			}
			if(softMotion_dataPt->joint->V_max != NULL) {
				MY_FREE(softMotion_dataPt->joint->V_max, double, softMotion_dataPt->nbJoints);
			}
			if(softMotion_dataPt->joint->V_max != NULL) {
				MY_FREE(softMotion_dataPt->joint->motion, SM_MOTION_MONO, softMotion_dataPt->nbJoints);
			}
			MY_FREE(softMotion_dataPt->joint, p3d_softMotion_data_joint, 1);
		}

		MY_FREE(softMotion_dataPt, p3d_softMotion_data, 1);
		softMotion_dataPt = NULL;
	}
}

/*
 * Destroy a softMotion local path
 */
void p3d_softMotion_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt)
{
	if (localpathPt != NULL){

		/* test whether the type of local path is the expected one */
		if (localpathPt->type_lp != SOFT_MOTION){
			PrintError(("p3d_softMotion_destroy: softMotion local path expected\n"));
		}
		/* destroy the specific part */
		if (localpathPt->specific.softMotion_data != NULL){
			p3d_destroy_softMotion_data(robotPt, localpathPt->specific.softMotion_data);
		}

		localpathPt->next_lp = NULL;
		localpathPt->prev_lp = NULL;
    MY_FREE(localpathPt->activeCntrts, int, localpathPt->nbActiveCntrts);
		MY_FREE(localpathPt, p3d_localpath, 1);
	}
}

/*
 *  Copy one local path.
 *  Input:  the robot, the local path.
 *  Output: the copied local path
 */
p3d_localpath *p3d_copy_softMotion_localpath(p3d_rob* robotPt, p3d_localpath* localpathPt)
{
	p3d_localpath *softMotion_localpathPt;
	p3d_softMotion_data*  softMotion_dataPt = NULL;
	p3d_group_type gpType;
	int lp_id = localpathPt->lp_id;
	int is_valid = localpathPt->valid;
	int nbJoints = 0;


	gpType = robotPt->mlp->mlpJoints[localpathPt->mlpID]->gpType;
	nbJoints = robotPt->mlp->mlpJoints[localpathPt->mlpID]->nbJoints;

	softMotion_dataPt = p3d_create_softMotion_data_multigraph(robotPt, gpType, nbJoints, localpathPt->mlpID);

		softMotion_dataPt->isPlanned = localpathPt->specific.softMotion_data->isPlanned;
		softMotion_dataPt->isPTP     = localpathPt->specific.softMotion_data->isPTP;
		softMotion_dataPt->nbJoints  = localpathPt->specific.softMotion_data->nbJoints;

		softMotion_dataPt->q_init  = p3d_copy_config(robotPt, localpathPt->specific.softMotion_data->q_init);
		softMotion_dataPt->q_end   = p3d_copy_config(robotPt, localpathPt->specific.softMotion_data->q_end);
		softMotion_dataPt->q_endp1 = p3d_copy_config(robotPt, localpathPt->specific.softMotion_data->q_endp1);

		if(localpathPt->specific.softMotion_data->freeflyer != NULL) {
			softMotion_dataPt->freeflyer->J_max_lin = localpathPt->specific.softMotion_data->freeflyer->J_max_lin;
			softMotion_dataPt->freeflyer->A_max_lin = localpathPt->specific.softMotion_data->freeflyer->A_max_lin;
			softMotion_dataPt->freeflyer->V_max_lin = localpathPt->specific.softMotion_data->freeflyer->V_max_lin;
			softMotion_dataPt->freeflyer->J_max_ang = localpathPt->specific.softMotion_data->freeflyer->J_max_ang;
			softMotion_dataPt->freeflyer->A_max_ang = localpathPt->specific.softMotion_data->freeflyer->A_max_ang;
			softMotion_dataPt->freeflyer->V_max_ang = localpathPt->specific.softMotion_data->freeflyer->V_max_ang;
			softMotion_dataPt->freeflyer->poseLinInit = localpathPt->specific.softMotion_data->freeflyer->poseLinInit;
			softMotion_dataPt->freeflyer->poseLinEnd = localpathPt->specific.softMotion_data->freeflyer->poseLinEnd;
			softMotion_dataPt->freeflyer->velLinInit = localpathPt->specific.softMotion_data->freeflyer->velLinInit;
			softMotion_dataPt->freeflyer->velLinEnd = localpathPt->specific.softMotion_data->freeflyer->velLinEnd;
			softMotion_dataPt->freeflyer->poseAngInit = localpathPt->specific.softMotion_data->freeflyer->poseAngInit;
			softMotion_dataPt->freeflyer->poseAngEnd = localpathPt->specific.softMotion_data->freeflyer->poseAngEnd;
			softMotion_dataPt->freeflyer->velAngInit = localpathPt->specific.softMotion_data->freeflyer->velAngInit;
			softMotion_dataPt->freeflyer->velAngEnd = localpathPt->specific.softMotion_data->freeflyer->velAngEnd;
			softMotion_dataPt->freeflyer->motion = localpathPt->specific.softMotion_data->freeflyer->motion;
			softMotion_dataPt->freeflyer->motionTime = localpathPt->specific.softMotion_data->freeflyer->motionTime;
		}

		if(localpathPt->specific.softMotion_data->joint != NULL) {

		}



		softMotion_localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_dataPt, lp_id, is_valid);

	/* update length and range of parameter */
	softMotion_localpathPt->length_lp   = localpathPt->length_lp;
	softMotion_localpathPt->range_param = localpathPt->range_param;
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(softMotion_localpathPt->ikSol));
  softMotion_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  softMotion_localpathPt->activeCntrts = MY_ALLOC(int, softMotion_localpathPt->nbActiveCntrts);
  for(int i = 0; i < softMotion_localpathPt->nbActiveCntrts; i++){
    softMotion_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
  }
	return softMotion_localpathPt;
}

void softMotion_data_copy_into(p3d_rob *robotPt, const p3d_softMotion_data * sm_data, p3d_softMotion_data * softMotion_data)
{

	if(softMotion_data->freeflyer != NULL) {
		softMotion_data->q_init = p3d_copy_config(robotPt, sm_data->q_init);
		softMotion_data->q_end  = p3d_copy_config(robotPt, sm_data->q_end);
		softMotion_data->q_endp1 = p3d_copy_config(robotPt, sm_data->q_endp1);
		softMotion_data->freeflyer->J_max_lin = sm_data->freeflyer->J_max_lin;
		softMotion_data->freeflyer->A_max_lin = sm_data->freeflyer->A_max_lin;
		softMotion_data->freeflyer->V_max_lin = sm_data->freeflyer->V_max_lin;
		softMotion_data->freeflyer->J_max_ang = sm_data->freeflyer->J_max_ang;
		softMotion_data->freeflyer->A_max_ang = sm_data->freeflyer->A_max_ang;
		softMotion_data->freeflyer->V_max_ang = sm_data->freeflyer->V_max_ang;

		Gb_v3_set( &(softMotion_data->freeflyer->poseLinInit), sm_data->freeflyer->poseLinInit.x, sm_data->freeflyer->poseLinInit.y, sm_data->freeflyer->poseLinInit.z);
		Gb_v3_set( &(softMotion_data->freeflyer->poseLinEnd), sm_data->freeflyer->poseLinEnd.x, sm_data->freeflyer->poseLinEnd.y, sm_data->freeflyer->poseLinEnd.z);
		Gb_v3_set( &(softMotion_data->freeflyer->velLinInit), sm_data->freeflyer->velLinInit.x, sm_data->freeflyer->velLinInit.y, sm_data->freeflyer->velLinInit.z);
		Gb_v3_set( &(softMotion_data->freeflyer->velLinEnd), sm_data->freeflyer->velLinEnd.x, sm_data->freeflyer->velLinEnd.y, sm_data->freeflyer->velLinEnd.z);
		Gb_v3_set( &(softMotion_data->freeflyer->poseAngInit), sm_data->freeflyer->poseAngInit.x, sm_data->freeflyer->poseAngInit.y, sm_data->freeflyer->poseAngInit.z);
		Gb_v3_set( &(softMotion_data->freeflyer->poseAngEnd), sm_data->freeflyer->poseAngEnd.x, sm_data->freeflyer->poseAngEnd.y, sm_data->freeflyer->poseAngEnd.z);
		Gb_v3_set( &(softMotion_data->freeflyer->velAngInit), sm_data->freeflyer->velAngInit.x, sm_data->freeflyer->velAngInit.y, sm_data->freeflyer->velAngInit.z);
		Gb_v3_set( &(softMotion_data->freeflyer->velAngEnd), sm_data->freeflyer->velAngEnd.x, sm_data->freeflyer->velAngEnd.y, sm_data->freeflyer->velAngEnd.z);

		sm_copy_SM_MOTION_into(&(sm_data->freeflyer->motion), &(softMotion_data->freeflyer->motion));
		softMotion_data->freeflyer->motionTime = sm_data->freeflyer->motionTime;
	}
	return;
}

/*
 *  lm_create_softMotion only for PA10
 */
psoftMotion_str lm_create_softMotion(p3d_rob *robotPt, p3d_group_type gpType, int nbJoints, double *dtab)
{
	psoftMotion_str softMotion_params = NULL;

	switch (gpType) {
		case FREEFLYER:

			if ((softMotion_params = MY_ALLOC(softMotion_str, 1)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			softMotion_params->joint = NULL;
			if ((softMotion_params->freeflyer = MY_ALLOC(gp_freeflyer_params_str, 1)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			softMotion_params->freeflyer->J_max_lin = dtab[0];
			softMotion_params->freeflyer->A_max_lin = dtab[1];
			softMotion_params->freeflyer->V_max_lin = dtab[2];
			softMotion_params->freeflyer->J_max_ang = dtab[3];
			softMotion_params->freeflyer->A_max_ang = dtab[4];
			softMotion_params->freeflyer->V_max_ang = dtab[5];

			break;
		case JOINT:
		{
			if ((softMotion_params = MY_ALLOC(softMotion_str, 1)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			softMotion_params->freeflyer = NULL;
			if ((softMotion_params->joint = MY_ALLOC(gp_joint_str, 1)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			if ((softMotion_params->joint->J_max = MY_ALLOC(double, nbJoints)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			if ((softMotion_params->joint->A_max = MY_ALLOC(double, nbJoints)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			if ((softMotion_params->joint->V_max = MY_ALLOC(double, nbJoints)) == NULL) {
				PrintWarning(("  lm_create_softMotion: allocation failed\n"));
				return (NULL);
			}
			int j=0;
			for(int i=0; i<nbJoints; i=i+3) {
				softMotion_params->joint->J_max[j] = dtab[i];
				softMotion_params->joint->A_max[j] = dtab[i+1];
				softMotion_params->joint->V_max[j] = dtab[i+2];
				j++;
			}
		}
			break;
		default:
			printf("unknow multiLocalpath %d\n",gpType);
			break;
	}
	return(softMotion_params);
}

/*
 *  lm_get_softMotion_lm_param --
 *
 *  find the first occurence of softMotion local method parameters.
 */
psoftMotion_str lm_get_softMotion_lm_param(p3d_rob *robotPt)
{
	lm_list_param_str *list_paramPt = robotPt->local_method_params;
	psoftMotion_str resultPt=NULL;

	while (list_paramPt) {
		if (list_paramPt->lpl_type != P3D_SOFT_MOTION_PLANNER) {
			list_paramPt = list_paramPt->next;
		}
		else {
			resultPt = (psoftMotion_str)(list_paramPt->lm_param);
			list_paramPt = NULL;
		}
	}
	return resultPt;
}

/*
 *  lm_get_softMotion_lm_param_multilocapath --
 *
 *  find the first occurence of softMotion local method parameters.
 */
psoftMotion_str lm_get_softMotion_lm_param_multilocalpath(p3d_rob *robotPt, int nblpGp)
{
	lm_list_param_str *list_paramPt = (robotPt->mlp->mlpJoints[nblpGp])->local_method_params;
	psoftMotion_str resultPt=NULL;

	while (list_paramPt) {
		if (list_paramPt->lpl_type != P3D_SOFT_MOTION_PLANNER) {
			list_paramPt = list_paramPt->next;
		}
		else {
			resultPt = (psoftMotion_str)(list_paramPt->lm_param);
			list_paramPt = NULL;
		}
	}
	return resultPt;
}

/*
		*  lm_get_q6_from_configPt --
		*
	*/
Gb_q6 lm_get_q6_from_configPt(configPt conf, int index)
{
	Gb_q6 resultQ6;
	resultQ6.q1 = conf[index];
	resultQ6.q2 = conf[index+1];
	resultQ6.q3 = conf[index+2];
	resultQ6.q4 = conf[index+3];
	resultQ6.q5 = conf[index+4];
	resultQ6.q6 = conf[index+5];

	if(SOFT_MOTION_PRINT_DATA) {
		printf("\n");
		printf("Gb_q6.q1 %f\n",resultQ6.q1 );
		printf("Gb_q6.q2 %f\n",resultQ6.q2 );
		printf("Gb_q6.q3 %f\n",resultQ6.q3 );
		printf("Gb_q6.q4 %f\n",resultQ6.q4 );
		printf("Gb_q6.q5 %f\n",resultQ6.q5 );
		printf("Gb_q6.q6 %f\n",resultQ6.q6 );
	}
	return resultQ6;
}

void lm_convert_p3dMatrix_To_GbTh(const p3d_matrix4 M ,Gb_th* th) {

th->vx.x = M[0][0];
th->vx.y = M[1][0];
th->vx.z = M[2][0];
th->vy.x = M[0][1];
th->vy.y = M[1][1];
th->vy.z = M[2][1];
th->vz.x = M[0][2];
th->vz.y = M[1][2];
th->vz.z = M[2][2];
th->vp.x = M[0][3];
th->vp.y = M[1][3];
th->vp.z = M[2][3];
return;
}

void lm_convert_GbTh_To_p3dMatrix(const Gb_th* th, p3d_matrix4 M) {

	M[0][0] = th->vx.x;
	M[1][0] = th->vx.y;
	M[2][0] = th->vx.z;
	M[3][0] = 0.0;
	M[0][1] = th->vy.x;
	M[1][1] = th->vy.y;
	M[2][1] = th->vy.z;
	M[3][1] = 0.0;
	M[0][2] = th->vz.x;
	M[1][2] = th->vz.y;
	M[2][2] = th->vz.z;
	M[3][2] = 0.0;
	M[0][3] = th->vp.x;
	M[1][3] = th->vp.y;
	M[2][3] = th->vp.z;
	M[3][3] = 1.0;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			if(M[i][j] < -1.0) {
				M[i][j] = -1.0;
			}
			if(M[i][j] > 1.0) {
				M[i][j] = 1.0;
			}
		}
	}

	return;
}

/*
		*   lm_set_Gb_v3_for_translation_and_rotation--
		*   input Gb_th : homogenous matrix
		*   output Gb_v3 pose : x, y and z cartesian position
		*          Gb_v3 rot  : rotation represented by vector with angle (angle = norm(vector))
	*/
void lm_set_Gb_v3_for_translation_and_rotation(const Gb_th* th, Gb_v3* pose, Gb_v3* rot)
{
	Gb_dep dep;
	Gb_v3 tmp;

	/* Set cartesian position */
	pose->x = th->vp.x;
	pose->y = th->vp.y;
	pose->z = th->vp.z;

	/* compute dep from homogenous matrix */
	Gb_th_dep( th, &dep);

	tmp.x = dep.rx;
	tmp.y = dep.ry;
	tmp.z = dep.rz;

	Gb_v3_norme( &tmp, rot);

	rot->x = rot->x * dep.a;
	rot->y = rot->y * dep.a;
	rot->z = rot->z * dep.a;
	return;
}

/*
		*   lm_compute tangent at middle point--
		*   input: 3* Gb_v3 (3 points)
		*   output: Gb_v3 tangent vector normalized
	*/
void lm_softMotion_compute_tangent(const Gb_v3 *p1, const Gb_v3 *p2, const Gb_v3 *p3, Gb_v3 *tangent)
{
	Gb_v3 v1, v1n, v2, v2n, v3, v3n, v4, v4n, v5, vTmp, tangentn;
	double angle, ratio;

	/* Fisrtly we compute the tangent normalized */
	Gb_v3_moins( p2, p3, &vTmp);
	if((vTmp.x == 0.0) && (vTmp.y == 0.0) && (vTmp.z == 0.0)) {
		tangent->x = 0.0;
		tangent->y = 0.0;
		tangent->z = 0.0;
		return;
	}
	Gb_v3_moins( p1, p2, &v1);
	Gb_v3_moins( p3, p2, &v2);
	Gb_v3_norme(&v1, &v1n);
	Gb_v3_norme(&v2, &v2n);

	Gb_v3_cross_product(&v1n, &v2n, &v3);
	Gb_v3_norme(&v3, &v3n);

	Gb_v3_plus(&v1, &v2, &v4);
	Gb_v3_norme(&v4, &v4n);

	Gb_v3_cross_product(&v3n, &v4n, &v5);
	Gb_v3_norme(&v5, &tangentn);

	/* Then when compute the angle between the both vectors */
	angle = Gb_v3_angle(&v1n, &v2n);
	ratio = ABS(angle) / M_PI;
	if(SOFT_MOTION_PRINT_DATA) {
		printf("angle %f    ratio %f\n",angle, ratio);
	}
	ratio = 1;
	Gb_v3_product_r(&tangentn, ratio, tangent);

	return;
}

void lm_set_cond_softMotion_data_FREEFLYER(Gb_v3 poseLinInit, Gb_v3 poseLinEnd, Gb_v3 poseAngInit,
														Gb_v3 poseAngEnd, Gb_v3 velLinInit, Gb_v3 velAngInit,
														Gb_v3 velLinEnd, Gb_v3 velAngEnd, p3d_softMotion_data* softMotion_data)
{
	softMotion_data->freeflyer->poseLinInit = poseLinInit;
	softMotion_data->freeflyer->poseLinEnd  = poseLinEnd;
	softMotion_data->freeflyer->poseAngInit = poseAngInit;
	softMotion_data->freeflyer->poseAngEnd  = poseAngEnd;
	softMotion_data->freeflyer->velLinEnd   = velLinEnd;
	softMotion_data->freeflyer->velAngEnd   = velAngEnd;
	softMotion_data->freeflyer->velLinInit  = velLinInit;
	softMotion_data->freeflyer->velAngInit  = velAngInit;

	softMotion_data->freeflyer->motion.IC[0].a = 0.0;
	softMotion_data->freeflyer->motion.IC[0].v = velLinInit.x;
	softMotion_data->freeflyer->motion.IC[0].x = poseLinInit.x;
	softMotion_data->freeflyer->motion.IC[1].a = 0.0;
	softMotion_data->freeflyer->motion.IC[1].v = velLinInit.y;
	softMotion_data->freeflyer->motion.IC[1].x = poseLinInit.y;
	softMotion_data->freeflyer->motion.IC[2].a = 0.0;
	softMotion_data->freeflyer->motion.IC[2].v = velLinInit.z;
	softMotion_data->freeflyer->motion.IC[2].x = poseLinInit.z;
	softMotion_data->freeflyer->motion.IC[3].a = 0.0;
	softMotion_data->freeflyer->motion.IC[3].v = velAngInit.x;
	softMotion_data->freeflyer->motion.IC[3].x = poseAngInit.x;
	softMotion_data->freeflyer->motion.IC[4].a = 0.0;
	softMotion_data->freeflyer->motion.IC[4].v = velAngInit.y;
	softMotion_data->freeflyer->motion.IC[4].x = poseAngInit.y;
	softMotion_data->freeflyer->motion.IC[5].a = 0.0;
	softMotion_data->freeflyer->motion.IC[5].v = velAngInit.z;
	softMotion_data->freeflyer->motion.IC[5].x = poseAngInit.z;

	softMotion_data->freeflyer->motion.FC[0].a = 0.0;
	softMotion_data->freeflyer->motion.FC[0].v = velLinEnd.x;
	softMotion_data->freeflyer->motion.FC[0].x = poseLinEnd.x;
	softMotion_data->freeflyer->motion.FC[1].a = 0.0;
	softMotion_data->freeflyer->motion.FC[1].v = velLinEnd.y;
	softMotion_data->freeflyer->motion.FC[1].x = poseLinEnd.y;
	softMotion_data->freeflyer->motion.FC[2].a = 0.0;
	softMotion_data->freeflyer->motion.FC[2].v = velLinEnd.z;
	softMotion_data->freeflyer->motion.FC[2].x = poseLinEnd.z;
	softMotion_data->freeflyer->motion.FC[3].a = 0.0;
	softMotion_data->freeflyer->motion.FC[3].v = velAngEnd.x;
	softMotion_data->freeflyer->motion.FC[3].x = poseAngEnd.x;
	softMotion_data->freeflyer->motion.FC[4].a = 0.0;
	softMotion_data->freeflyer->motion.FC[4].v = velAngEnd.y;
	softMotion_data->freeflyer->motion.FC[4].x = poseAngEnd.y;
	softMotion_data->freeflyer->motion.FC[5].a = 0.0;
	softMotion_data->freeflyer->motion.FC[5].v = velAngEnd.z;
	softMotion_data->freeflyer->motion.FC[5].x = poseAngEnd.z;
	return;
}

void 	lm_set_motion_softMotion_data_FREEFLYER(SM_TIMES localtimes[], double jerk[], int DirTransition_a[],
																		int DirTransition_b[], p3d_softMotion_data* softMotion_data)
{
	int i;
	for (i=0; i<SM_NB_DIM; i++) {
	softMotion_data->freeflyer->motion.jerk[i].J1 = jerk[i];
	softMotion_data->freeflyer->motion.jerk[i].J2 = jerk[i];
	softMotion_data->freeflyer->motion.jerk[i].J3 = jerk[i];
	softMotion_data->freeflyer->motion.jerk[i].J4 = jerk[i];
	sm_SM_TIMES_copy_into(&localtimes[i], &softMotion_data->freeflyer->motion.Times[i]);
	softMotion_data->freeflyer->motion.Dir_a[i] =  DirTransition_a[i];
	softMotion_data->freeflyer->motion.Dir_b[i] =  DirTransition_b[i];
	}
	return;
}

void lm_get_softMotion_segment_params_FREEFLYER(p3d_softMotion_data* softMotion_data, int param, SM_SEGMENT * segment, int * segId, int index)
{
	if (param>=softMotion_data->freeflyer->motion.MotionDurationM[index]) {
		param = softMotion_data->freeflyer->motion.MotionDurationM[index];
	}

	if(softMotion_data->freeflyer->motion.motionIsAdjusted == 0) {

		if(param >= softMotion_data->freeflyer->motion.TimeCumulM[index][6]) {
			segment->type = 7;
			*segId = 6;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjpb;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tacb;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tacb;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tacb;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];

		} else if (param >= softMotion_data->freeflyer->motion.TimeCumulM[index][5]) {
			segment->type = 6;
			*segId = 5;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tacb;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tjnb;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tjnb;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tjnb;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];

		} else if (param >= softMotion_data->freeflyer->motion.TimeCumulM[index][4]) {
			segment->type = 5;
			*segId = 4;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjnb;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tvc;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tvc;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tvc;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];

		} else if (param >= softMotion_data->freeflyer->motion.TimeCumulM[index][3]) {
			segment->type = 4;
			*segId = 3;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tvc;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tjna;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tjna;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tjna;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];

		} else if (param >= softMotion_data->freeflyer->motion.TimeCumulM[index][2]) {
			segment->type = 3;
			*segId = 2;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjna;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Taca;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Taca;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Taca;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];

		} else if (param >= softMotion_data->freeflyer->motion.TimeCumulM[index][1]) {
			segment->type = 2;
			*segId = 1;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Taca;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tjpa;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tjpa;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tjpa;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];

		} else {
			segment->type = 1;
			*segId = 0;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjpa;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.IC[index].a;
			segment->V0   = softMotion_data->freeflyer->motion.IC[index].v;
			segment->X0   = softMotion_data->freeflyer->motion.IC[index].x;
			segment->dir  = softMotion_data->freeflyer->motion.Dir[index];
		}

	} else {  // motionIsAdjusted == 1

		if(param > softMotion_data->freeflyer->motion.TimeCumulM[index][6]) {
			segment->type = 3;
			*segId = 6;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjpb;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tacb;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tacb;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tacb;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_b[index];

		} else if (param > softMotion_data->freeflyer->motion.TimeCumulM[index][5]) {
			segment->type = 2;
			*segId = 5;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tacb;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tjnb;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tjnb;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tjnb;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_b[index];

		} else if (param > softMotion_data->freeflyer->motion.TimeCumulM[index][4]) {
			segment->type = 1;
			*segId = 4;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjnb;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tvc;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tvc;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tvc;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_b[index];

		} else if (param > softMotion_data->freeflyer->motion.TimeCumulM[index][3]) {
			segment->type = 4;
			*segId = 3;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tvc;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tjna;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tjna;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tjna;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_a[index];

		} else if (param > softMotion_data->freeflyer->motion.TimeCumulM[index][2]) {
			segment->type = 3;
			*segId = 2;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjna;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Taca;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Taca;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Taca;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_a[index];

		} else if (param > softMotion_data->freeflyer->motion.TimeCumulM[index][1]) {
			segment->type = 2;
			*segId = 1;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Taca;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.Acc[index].Tjpa;
			segment->V0   = softMotion_data->freeflyer->motion.Vel[index].Tjpa;
			segment->X0   = softMotion_data->freeflyer->motion.Pos[index].Tjpa;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_a[index];

		} else {
			segment->type = 1;
			*segId = 0;
			segment->time = softMotion_data->freeflyer->motion.TimesM[index].Tjpa;
			segment->J    = softMotion_data->freeflyer->motion.jerk[index].J1;
			segment->A0   = softMotion_data->freeflyer->motion.IC[index].a;
			segment->V0   = softMotion_data->freeflyer->motion.IC[index].v;
			segment->X0   = softMotion_data->freeflyer->motion.IC[index].x;
			segment->dir  = softMotion_data->freeflyer->motion.Dir_a[index];
		}
	}
	return;
}

void lm_set_and_get_motionTimes_FREEFLYER(p3d_softMotion_data* softMotion_data, int* timeMotionMax, int* axisMotionMax)
{
	int i=0;
	int NOE;
	*timeMotionMax = 0;
	* axisMotionMax = 0;
	SM_TIMES smTimesTmp;

	for(i=0; i<SM_NB_DIM; i++) {
		sm_GetMonotonicTimes(softMotion_data->freeflyer->motion.Times[i], &smTimesTmp, &NOE);
		sm_GetNumberOfElement(&smTimesTmp, &softMotion_data->freeflyer->motion.TimesM[i]);

		sm_sum_motionTimes(&(softMotion_data->freeflyer->motion.Times[i]), &(softMotion_data->freeflyer->motion.MotionDuration[i]));
		sm_sum_motionTimes(&(softMotion_data->freeflyer->motion.TimesM[i]), &(softMotion_data->freeflyer->motion.MotionDurationM[i]));

		if(SOFT_MOTION_PRINT_DATA) {
			printf("motionDuration[%d] = %f\n",i,softMotion_data->freeflyer->motion.MotionDurationM[i]);
		}
		softMotion_data->freeflyer->motion.TimeCumulM[i][0] = 0;
		softMotion_data->freeflyer->motion.TimeCumulM[i][1] = softMotion_data->freeflyer->motion.TimesM[i].Tjpa;
		softMotion_data->freeflyer->motion.TimeCumulM[i][2] = softMotion_data->freeflyer->motion.TimeCumulM[i][1] \
				+ softMotion_data->freeflyer->motion.TimesM[i].Taca;
		softMotion_data->freeflyer->motion.TimeCumulM[i][3] = softMotion_data->freeflyer->motion.TimeCumulM[i][2] \
				+ softMotion_data->freeflyer->motion.TimesM[i].Tjna;
		softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3] \
				+ softMotion_data->freeflyer->motion.TimesM[i].Tvc;
		softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4] \
				+ softMotion_data->freeflyer->motion.TimesM[i].Tjnb;
		softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5] \
				+ softMotion_data->freeflyer->motion.TimesM[i].Tacb;
		//softMotion_data->freeflyer->motion.TimeCumulM[i][7] = softMotion_data->freeflyer->motion.TimeCumulM[i][6] + softMotion_data->freeflyer->motion.TimesM[i].Tjpb;

		if (softMotion_data->freeflyer->motion.MotionDurationM[i] > *timeMotionMax) {
			*timeMotionMax = softMotion_data->freeflyer->motion.MotionDurationM[i];
			*axisMotionMax = i;
		}
	}
	return;
}

/*
		*   compute softMotion for the localpath
		*
*/
void lm_compute_softMotion_for_freeflyer( p3d_softMotion_data* softMotion_data)
{
	SM_LIMITS auxLimits;
	SM_POSELIMITS poseLimits;
	SM_COND IC, FC;
	int timeMotionMaxM = 0;
	int axisMotionMax = 0;
	double distanceTolerance = 0.0, GD = 0.0;
	int i = 0;
	double Jerk[SM_NB_DIM];
	int DirTransition_a[6], DirTransition_b[6];
	SM_TIMES localTimes[6];
	double timeMotionMax = 0.0;
	int adjustTimeError = 0;

	/* Set kinematic limits */
	poseLimits.linear.maxJerk = softMotion_data->freeflyer->J_max_lin;
	poseLimits.linear.maxAcc  = softMotion_data->freeflyer->A_max_lin;
	poseLimits.linear.maxVel  = softMotion_data->freeflyer->V_max_lin;
	poseLimits.angular.maxJerk = softMotion_data->freeflyer->J_max_ang;
	poseLimits.angular.maxAcc  = softMotion_data->freeflyer->A_max_ang;
	poseLimits.angular.maxVel  = softMotion_data->freeflyer->V_max_ang;

	for (i=0; i < 6; i++) {

		softMotion_data->freeflyer->motion.motionIsAdjusted[i] = 0;
		if (i<3) {
			auxLimits.maxJerk = softMotion_data->freeflyer->J_max_lin;
			auxLimits.maxAcc  = softMotion_data->freeflyer->A_max_lin;
			auxLimits.maxVel  = softMotion_data->freeflyer->V_max_lin;
			distanceTolerance = SM_DISTANCE_TOLERANCE_LINEAR;
			softMotion_data->freeflyer->motion.jerk[i].J1 = softMotion_data->freeflyer->J_max_lin;
			softMotion_data->freeflyer->motion.jerk[i].sel = 1;

		} else {
			auxLimits.maxJerk = softMotion_data->freeflyer->J_max_ang;
			auxLimits.maxAcc  = softMotion_data->freeflyer->A_max_ang;
			auxLimits.maxVel  = softMotion_data->freeflyer->V_max_ang;
			distanceTolerance = SM_DISTANCE_TOLERANCE_ANGULAR;
			softMotion_data->freeflyer->motion.jerk[i].J1 = softMotion_data->freeflyer->J_max_ang;
			softMotion_data->freeflyer->motion.jerk[i].sel = 1;
		}

		IC.a =  softMotion_data->freeflyer->motion.IC[i].a;
		IC.v =  softMotion_data->freeflyer->motion.IC[i].v;
		IC.x =  0.0;

		FC.a = softMotion_data->freeflyer->motion.FC[i].a;
		FC.v = softMotion_data->freeflyer->motion.FC[i].v;
		FC.x = (softMotion_data->freeflyer->motion.FC[i].x - softMotion_data->freeflyer->motion.IC[i].x);

		if (sm_ComputeSoftMotion( IC, FC, auxLimits, &(softMotion_data->freeflyer->motion.Times[i]), &(softMotion_data->freeflyer->motion.Dir[i]))!=0) {
			printf("ERROR Jerk Profile on dim %d\n",i);
			return;
		}

		/* Get initial conditions for each vectors Acc Vel and Pos */
		GD =  FC.x * softMotion_data->freeflyer->motion.Dir[i];

		if (sm_VerifyTimes( distanceTolerance, GD, softMotion_data->freeflyer->motion.jerk[i], softMotion_data->freeflyer->motion.IC[i], softMotion_data->freeflyer->motion.Dir[i], softMotion_data->freeflyer->motion.Times[i], &FC, &(softMotion_data->freeflyer->motion.Acc[i]), &(softMotion_data->freeflyer->motion.Vel[i]), &(softMotion_data->freeflyer->motion.Pos[i]), SM_ON)!=0) {
			printf(" Verify Times on dim %d\n",i);

			return;
		} else {
			if(SOFT_MOTION_PRINT_DATA) {
				printf(" Verify Times on dim %d is OK\n",i);
			}
		}
	}

	lm_set_and_get_motionTimes_FREEFLYER(softMotion_data, &timeMotionMaxM, &axisMotionMax);
	softMotion_data->freeflyer->motionTime = timeMotionMaxM;
	timeMotionMax = (double)timeMotionMaxM / (double)SM_NB_TICK_SEC;

	/* Adjust Motion Times */
	/* Compute interval time where slowing velocity works */
//  	if (sm_FindTransitionTime(poseLimits, &localMotion,  &impTimeM) != 0) {
//  		printf("lm_compute_softMotion_for_r6Arm CANNOT adjust transition times\n");
//  	}
	// Adjust times like in xarmPlanificationCodels.c
	adjustTimeError = 0;

	for (i=0; i < 6; i++) {
		if (i != axisMotionMax) {
			if (i<3) {
				auxLimits.maxJerk = softMotion_data->freeflyer->J_max_lin;
				auxLimits.maxAcc  = softMotion_data->freeflyer->A_max_lin;
				auxLimits.maxVel  = softMotion_data->freeflyer->V_max_lin;
				distanceTolerance = SM_DISTANCE_TOLERANCE_LINEAR;
				softMotion_data->freeflyer->motion.jerk[i].J1 = softMotion_data->freeflyer->J_max_lin;
				softMotion_data->freeflyer->motion.jerk[i].sel = 1;

			} else {
				auxLimits.maxJerk = softMotion_data->freeflyer->J_max_ang;
				auxLimits.maxAcc  = softMotion_data->freeflyer->A_max_ang;
				auxLimits.maxVel  = softMotion_data->freeflyer->V_max_ang;
				distanceTolerance = SM_DISTANCE_TOLERANCE_ANGULAR;
				softMotion_data->freeflyer->motion.jerk[i].J1 = softMotion_data->freeflyer->J_max_ang;
				softMotion_data->freeflyer->motion.jerk[i].sel = 1;
			}

			IC.a =  softMotion_data->freeflyer->motion.IC[i].a;
			IC.v =  softMotion_data->freeflyer->motion.IC[i].v;
			IC.x =  0.0;

			FC.a = softMotion_data->freeflyer->motion.FC[i].a;
			FC.v = softMotion_data->freeflyer->motion.FC[i].v;
			FC.x = (softMotion_data->freeflyer->motion.FC[i].x - softMotion_data->freeflyer->motion.IC[i].x);


			if(sm_AdjustTime(IC, FC, timeMotionMax, auxLimits, &localTimes[i], &Jerk[i], &DirTransition_a[i], &DirTransition_b[i])!= 0) {
				printf("sm_AdjustTime ERROR at axis %d\n",i);
				adjustTimeError ++;
			}
	// 		lm_sum_motionTimes(&localTimes[i], &sum);
	// 		printf("sum[%d] = %f\n",i,sum);
		} else {  // i == axisMotionMax

			sm_SM_TIMES_copy_into(&(softMotion_data->freeflyer->motion.Times[i]), &localTimes[i]);
			DirTransition_a[i] = softMotion_data->freeflyer->motion.Dir[i];
			DirTransition_b[i] = -softMotion_data->freeflyer->motion.Dir[i];
			Jerk[i] = softMotion_data->freeflyer->motion.jerk[i].J1;
		}

	}

	if (adjustTimeError > 0) {
		// TODO
		printf("lm_compute_softMotion_for_r6Arm can't adjust time motion \n motion must be stopped at next configuration\n");
		return;
	}

	/* Replace old motion by adjusted motion */
	lm_set_motion_softMotion_data_FREEFLYER(localTimes, Jerk, DirTransition_a, DirTransition_b, softMotion_data);

	for (i=0; i < SM_NB_DIM; i++) {
			if (i<3) {
				distanceTolerance = SM_DISTANCE_TOLERANCE_LINEAR;
			} else {
				distanceTolerance = SM_DISTANCE_TOLERANCE_ANGULAR;
			}

			softMotion_data->freeflyer->motion.jerk[i].J1 = Jerk[i];
			softMotion_data->freeflyer->motion.jerk[i].J2 = Jerk[i];
			softMotion_data->freeflyer->motion.jerk[i].J3 = Jerk[i];
			softMotion_data->freeflyer->motion.jerk[i].J4 = Jerk[i];
			softMotion_data->freeflyer->motion.jerk[i].sel = 1;

			IC.a =  softMotion_data->freeflyer->motion.IC[i].a;
			IC.v =  softMotion_data->freeflyer->motion.IC[i].v;
			IC.x =  softMotion_data->freeflyer->motion.IC[i].x;
			FC.a = softMotion_data->freeflyer->motion.FC[i].a;
			FC.v = softMotion_data->freeflyer->motion.FC[i].v;
			FC.x = (softMotion_data->freeflyer->motion.FC[i].x - softMotion_data->freeflyer->motion.IC[i].x);

			/* Verify Times */
			if (sm_VerifyTimes_Dir_ab(distanceTolerance, FC.x, softMotion_data->freeflyer->motion.jerk[i], IC,
																softMotion_data->freeflyer->motion.Dir_a[i], softMotion_data->freeflyer->motion.Dir_b[i],
																softMotion_data->freeflyer->motion.Times[i], &FC, &(softMotion_data->freeflyer->motion.Acc[i]),
																&(softMotion_data->freeflyer->motion.Vel[i]), &(softMotion_data->freeflyer->motion.Pos[i])) != 0) {
				printf("lm_compute_softMotion_for_r6Arm ERROR Verify Times on axis [%d] \n",i);
				return;
			}
			softMotion_data->freeflyer->motion.motionIsAdjusted[i] = 1;
	}

	lm_set_and_get_motionTimes_FREEFLYER(softMotion_data, &timeMotionMaxM, &axisMotionMax);
	softMotion_data->freeflyer->motionTime = timeMotionMaxM;
	return;
}

/*
 *  Compute the configuration situated at given param (time) on the local path.
 *
 *  Input:  the robot, the param.
 *
 *  Output: the configuration
 */
configPt p3d_softMotion_config_at_param(p3d_rob *robotPt, p3d_localpath *localpathPt, double param)
{
	p3d_softMotion_data *softMotion_specificPt;
	configPt q;
	int i, j;
	int paramDiff = 0;
	int paramLocal = 0;
	int segId = 0;
	psoftMotion_str softMotion_params;
	int index_dof = robotPt->joints[robotPt->mlp->mlpJoints[localpathPt->mlpID]->joints[0]]->index_dof;

	/* For FREEFLYER_GROUP */
	double q_init[6];
	double q_end[6];
	SM_SEGMENT segment[6];
	SM_COND condEnd[6];
	double angle = 0.0;
	Gb_th thMat , thInit, thMats;

	p3d_matrix4 p3dMat, freeflyerPose_init;
	Gb_dep gbDep;
	Gb_v3 gbV3Rot, gbV3Rotn;
	double Tx, Ty, Tz, Rx, Ry, Rz;
	if (localpathPt == NULL)
		return NULL;

	if (localpathPt->type_lp != SOFT_MOTION){
		PrintError(("p3d_softMotion_config_at_param: local path must be softMotion\n"));
		return NULL;
	}
	softMotion_specificPt = localpathPt->specific.softMotion_data;
	q = p3d_alloc_config(robotPt);

	softMotion_params = lm_get_softMotion_lm_param_multilocalpath(robotPt, localpathPt->mlpID );

	/* SWITCH WRT group */
	switch (robotPt->mlp->mlpJoints[localpathPt->mlpID]->gpType) {

		case FREEFLYER:
			j = 0;
			for(i=index_dof; i<(index_dof+6); i++) {
				q_init[j] = softMotion_specificPt->q_init[i];
				if(SOFT_MOTION_PRINT_DATA) {
					printf("q_init[%d]= %f\n",i,q_init[i]);
				}
				q_end[j] = softMotion_specificPt->q_end[i];
				j++;
			}
			if(SOFT_MOTION_PRINT_DATA) {
				printf("param %f \n",param);
			}

			if (param < 0) {
				param = 0.0;
			}
			if (param > localpathPt->range_param){
				param = localpathPt->range_param;
			}

			//not needed for at this time
			//alpha = param / localpathPt->range_param;

			for (i=0;i<6;i++) {
				lm_get_softMotion_segment_params_FREEFLYER( softMotion_specificPt, param, &segment[i], &segId, i);
				//compute pose at param
				if (param >= softMotion_specificPt->freeflyer->motion.MotionDurationM[i]) {
					paramLocal = (int)softMotion_specificPt->freeflyer->motion.MotionDurationM[i];
				}
				else {
					paramLocal = (int)param;
				}
				lm_get_paramDiff_for_param( softMotion_specificPt, &segment[i], segId, i, paramLocal, &paramDiff);

				sm_CalculOfAccVelPosAtTime(paramDiff, &segment[i], &condEnd[i]);
			}
		//lm_Get_TimeCumul_for_param(softMotion_data, segment[i])
			if (SM_NB_DIM != 6) {
				printf("ATTENTION ERROR le calcul se fait avec des Gb_dep et SM_NB_DIM != 6\n");
			}


			p3d_mat4PosReverseOrder(freeflyerPose_init, q_init[0], q_init[1],q_init[2],q_init[3],q_init[4],q_init[5]);
			lm_convert_p3dMatrix_To_GbTh(freeflyerPose_init ,&thInit);

			Gb_v3_set(&gbV3Rot,condEnd[3].x, condEnd[4].x,condEnd[5].x);
			angle = Gb_v3_norme(&gbV3Rot, &gbV3Rotn);
			Gb_dep_set(&gbDep, condEnd[0].x, condEnd[1].x, condEnd[2].x, gbV3Rotn.x, gbV3Rotn.y, gbV3Rotn.z, angle);
			Gb_dep_th(&gbDep, &thMat);

			Gb_th_produit(&thInit, &thMat, &thMats);

			lm_convert_GbTh_To_p3dMatrix(&thMats,p3dMat);

			p3d_mat4ExtractPosReverseOrder(p3dMat,&Tx, &Ty, &Tz, &Rx, &Ry, &Rz);

			p3d_copy_config_into(robotPt, softMotion_specificPt->q_init, &q);
			q[index_dof]   = Tx;
			q[index_dof+1] = Ty;
			q[index_dof+2] = Tz;
			q[index_dof+3] = Rx;
			q[index_dof+4] = Ry;
			q[index_dof+5] = Rz;
			if(isnan(Rx)) {
printf("isnan Rx\n");

			}
			if(isnan(Ry)) {
				printf("isnan Rx\n");

			}
			if(isnan(Rz)) {
				printf("isnan Rx\n");

			}
			break;

		case JOINT :
			break;

		default:

			break;
	}
	if(SOFT_MOTION_PRINT_DATA) {
		PrintInfo(("qlocal=("));
		print_config(robotPt, softMotion_specificPt->q_init);
		PrintInfo(("qlocal=("));
		print_config(robotPt, q);
	}
	return q;
}


/*  p3d_softMotion_stay_within_dist
 *
 *  Input:  the robot,
 *          the local path,
 *          the parameter along the curve,
 *          the direction of motion
 *          the maximal distance moved by all the points of the
 *          robot
 *
 *  Output: length of the interval of parameter the robot can
 *          stay on without any body moving by more than the input distance
 *
 *  Description:
 *          From a configuration on a local path, this function
 *          computes an interval of parameter on the local path on
 *          which all the points of the robot move by less than the
 *          distance given as input.  The interval is centered on the
 *          configuration given as input. The function returns the
 *          half length of the interval
 */
double p3d_softMotion_stay_within_dist(p3d_rob* robotPt,
																p3d_localpath* localpathPt,
								double parameter, whichway dir,
				double *distances)
{
// 	p3d_softMotion_data *softMotion_data = NULL;
// //	p3d_lin_data *lin_localpathPt=NULL;
// 	int i, j, njnt = robotPt->njoints;
// 	p3d_jnt *cur_jntPt, *prev_jntPt;
// 	configPt q_max_param, q_param;
// 	double max_param, min_param;
// 	double parameterLength= 0.0;
// 	double range_param = p3d_dist_config(robotPt, localpathPt->specific.softMotion_data->q_init, localpathPt->specific.softMotion_data->q_end);
// 	p3d_stay_within_dist_data * stay_within_dist_data;
//
// 	/* local path has to be of type linear */
// 	if (localpathPt->type_lp != SOFT_MOTION){
// 		PrintError(("p3d_lin_stay_within_dist: linear local path expected\n"));
// 		return 0;
// 	}
//
// 	parameterLength  = (parameter / localpathPt->length_lp) * range_param;
//
//   /* store the data to compute the maximal velocities at the
// 	joint for each body of the robot */
// 	stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2);
// 	p3d_init_stay_within_dist_data(stay_within_dist_data);
//
// 	if (dir == FORWARD) {
// 		q_max_param = localpathPt->specific.softMotion_data->q_end;
// 		min_param = max_param = range_param - parameter;
// 	} else {
// 		q_max_param = localpathPt->specific.softMotion_data->q_init;
// 		min_param = max_param = parameter;
// 	}
// 	/* Get the current config to have the modifications of the constraints */
// 	q_param = p3d_get_robot_config(robotPt);
//
//   /* computation of the bounds for the linear and angular
// 	velocities of each body */
//
// 	int minJnt = 0;
// 	for(i=0; i<=njnt; i++) {
// 		cur_jntPt = robotPt->joints[i];
// 		prev_jntPt = cur_jntPt->prev_jnt;
//
// 		/* j = index of the joint to which the current joint is attached */
// 		if (prev_jntPt==NULL)
// 		{ j = -1; } /* environment */
// 		else
// 		{ j = prev_jntPt->num; }
// 		double bakMinParam = min_param;
// 		p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
// 															 &(stay_within_dist_data[i+1]), &(distances[i]),
// 																 q_param, q_max_param, max_param, &min_param);
// 		if (min_param < bakMinParam){
// 			minJnt = cur_jntPt->num;
// 		}
// 		/* Rem: stay_within_dist_data[0] is bound to the environment */
// 	}
//
// 	MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);
// 	p3d_destroy_config(robotPt, q_param);




//////////////////////////////////////////////////////////////////
	if(localpathPt != NULL) {
	if (localpathPt->length_lp > 0.0) {
		//return (min_param*localpathPt->length_lp / range_param);
		return 1;
	} else {return 0;}

	} else {return 0;}


}


void p3d_softMotion_set_stay_within_dist(int value) {
	if(value < 1) {
		factorToViewTrajectory = 1;
	} else {
		factorToViewTrajectory = (int)value;
	}

}


/*
 *  Cost of a local path
 *
 *  Input:  the local path
 *
 *  Output: the cost
 */
double p3d_softMotion_cost(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
	return localpathPt->length_lp;
}

/*
 *  Extract from a linear local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */
p3d_localpath *p3d_extract_softMotion(p3d_rob *robotPt, p3d_localpath *localpathPt,	double l1, double l2)
{
	configPt q1, q2;
	p3d_localpath *sub_localpathPt;
	p3d_softMotion_data *softMotion_data = NULL;

	if ((softMotion_data = MY_ALLOC(p3d_softMotion_data,1)) == NULL)
		return NULL;

		if(localpathPt->specific.softMotion_data->freeflyer != NULL) {
			if ((softMotion_data->freeflyer = MY_ALLOC(p3d_softMotion_data_FREEFLYER,1)) == NULL)
				return NULL;
		}
		if(localpathPt->specific.softMotion_data->joint != NULL) {
// 		if ((softMotion_data->joint = MY_ALLOC(p3d_softMotion_data_joint,1)) == NULL)
// 			return NULL;
		}
		softMotion_data->joint = NULL;



	if (l1 == l2) {
		printf("p3d_extract_softMotion return NULL l1==l2\n");
		return NULL;
	}

	q1 = p3d_softMotion_config_at_param(robotPt, localpathPt, l1);
	if (robotPt->cntrt_manager->cntrts != NULL) {
		p3d_set_and_update_this_robot_conf(robotPt, q1);
		p3d_get_robot_config_into(robotPt, &q1);
	}
	q2 = p3d_softMotion_config_at_param(robotPt, localpathPt, l2);
	if (robotPt->cntrt_manager->cntrts != NULL) {
		p3d_set_and_update_this_robot_conf(robotPt, q2);
		p3d_get_robot_config_into(robotPt, &q2);
	}

	Gb_v3_set(&softMotion_data->freeflyer->velLinInit, 0.0, 0.0, 0.0);
	Gb_v3_set(&softMotion_data->freeflyer->velAngInit, 0.0, 0.0, 0.0);
	Gb_v3_set(&softMotion_data->freeflyer->velLinEnd, 0.0, 0.0, 0.0);
	Gb_v3_set(&softMotion_data->freeflyer->velAngEnd, 0.0, 0.0, 0.0);

	softMotion_data->freeflyer->J_max_lin = localpathPt->specific.softMotion_data->freeflyer->J_max_lin;
	softMotion_data->freeflyer->A_max_lin = localpathPt->specific.softMotion_data->freeflyer->A_max_lin;
	softMotion_data->freeflyer->V_max_lin = localpathPt->specific.softMotion_data->freeflyer->V_max_lin;
	softMotion_data->freeflyer->J_max_ang = localpathPt->specific.softMotion_data->freeflyer->J_max_ang;
	softMotion_data->freeflyer->A_max_ang = localpathPt->specific.softMotion_data->freeflyer->A_max_ang;
	softMotion_data->freeflyer->V_max_ang = localpathPt->specific.softMotion_data->freeflyer->V_max_ang;
	softMotion_data->gpType = localpathPt->specific.softMotion_data->gpType;
	softMotion_data->nbJoints = localpathPt->specific.softMotion_data->nbJoints;
	softMotion_data->isPTP = localpathPt->specific.softMotion_data->isPTP ;


	sub_localpathPt = p3d_softMotion_localplanner(robotPt, localpathPt->mlpID, softMotion_data, q1, q2, q2, NULL);
	if(sub_localpathPt != NULL) {
	 sub_localpathPt->mlpID = localpathPt->mlpID;
	 p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
	 sub_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
	 sub_localpathPt->activeCntrts = MY_ALLOC(int, sub_localpathPt->nbActiveCntrts);
	 for(int i = 0; i < sub_localpathPt->nbActiveCntrts; i++){
		 sub_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
	 }


	}
//   p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
//   sub_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
//   sub_localpathPt->activeCntrts = MY_ALLOC(int, sub_localpathPt->nbActiveCntrts);
//   for(int i = 0; i < sub_localpathPt->nbActiveCntrts; i++){
//     sub_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
//   }
	return sub_localpathPt;
}

/*
 *  Extract from a linear local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */
p3d_localpath *p3d_extract_softMotion_with_velocities(p3d_rob *robotPt, p3d_localpath *localpathPt, double l1, double l2)
{
	configPt q1, q2;
	p3d_localpath* sub_localpathPt = NULL;
	p3d_softMotion_data* softMotion_data_l1 = NULL;
	p3d_softMotion_data* softMotion_data = NULL;
	p3d_softMotion_data* softMotion_data_In = NULL;
	double ltmp=0.0, sum=0.0;
	int j=0, i=0;
	int paramDiffl1 = 0;
	int paramLocal = 0;
	int segIdl1 = 0;
	SM_SEGMENT segmentl1[SM_NB_DIM];

	SM_COND condl1[SM_NB_DIM];
	int paramDiffl2 = 0;
	int segIdl2 = 0;
	SM_SEGMENT segmentl2[SM_NB_DIM];
	SM_COND condl2[SM_NB_DIM];

	if (l1 == l2) {
		printf("p3d_extract_softMotion return NULL l1==l2\n");
		return NULL;
	}

	if (l1 > l2) {
		ltmp = l1;
		l1 = l2;
		l2 = l1;
	}

	q1 = p3d_softMotion_config_at_param(robotPt, localpathPt, l1);
	if (robotPt->cntrt_manager->cntrts != NULL) {
		p3d_set_and_update_this_robot_conf(robotPt, q1);
		p3d_get_robot_config_into(robotPt, &q1);
	}
	q2 = p3d_softMotion_config_at_param(robotPt, localpathPt, l2);
	if (robotPt->cntrt_manager->cntrts != NULL) {
		p3d_set_and_update_this_robot_conf(robotPt, q2);
		p3d_get_robot_config_into(robotPt, &q2);
	}
	softMotion_data_In = (localpathPt->specific.softMotion_data);

	if(softMotion_data == NULL) {
		softMotion_data = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, localpathPt->mlpID);
	}

	softMotion_data->freeflyer->J_max_lin = softMotion_data_In->freeflyer->J_max_lin;
	softMotion_data->freeflyer->A_max_lin = softMotion_data_In->freeflyer->A_max_lin;
	softMotion_data->freeflyer->V_max_lin = softMotion_data_In->freeflyer->V_max_lin;
	softMotion_data->freeflyer->J_max_ang = softMotion_data_In->freeflyer->J_max_ang;
	softMotion_data->freeflyer->A_max_ang = softMotion_data_In->freeflyer->A_max_ang;
	softMotion_data->freeflyer->V_max_ang = softMotion_data_In->freeflyer->V_max_ang;


	for (i=0;i<SM_NB_DIM;i++) {
		lm_get_softMotion_segment_params_FREEFLYER( softMotion_data_In, (int)l1, &segmentl1[i], &segIdl1, i);
		//compute pose at param
		if (l1 >= softMotion_data_In->freeflyer->motion.MotionDurationM[i]) {
			paramLocal = (int)softMotion_data_In->freeflyer->motion.MotionDurationM[i];
		}
		else {
			paramLocal = (int)l1;
		}
		lm_get_paramDiff_for_param( softMotion_data_In, &segmentl1[i], segIdl1, i, paramLocal, &paramDiffl1);

		sm_CalculOfAccVelPosAtTime(paramDiffl1, &segmentl1[i], &condl1[i]);
	}

	for (i=0;i<SM_NB_DIM;i++) {
		lm_get_softMotion_segment_params_FREEFLYER( softMotion_data_In, (int)l2, &segmentl2[i], &segIdl2, i);
		//compute pose at param
		if (l2 >= softMotion_data_In->freeflyer->motion.MotionDurationM[i]) {
			paramLocal = (int)softMotion_data_In->freeflyer->motion.MotionDurationM[i];
		}
		else {
			paramLocal = (int)l2;
		}

		lm_get_paramDiff_for_param( softMotion_data_In, &segmentl2[i], segIdl2, i, paramLocal, &paramDiffl2);

		sm_CalculOfAccVelPosAtTime(paramDiffl2, &segmentl2[i], &condl2[i]);
	}

	/* Set sub Motion */


	if(softMotion_data_l1 == NULL) {
		softMotion_data_l1 = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, localpathPt->mlpID);
	}
	softMotion_data_copy_into(robotPt, softMotion_data_In, softMotion_data_l1);


	Gb_v3_set( &(softMotion_data_l1->freeflyer->poseLinInit), condl1[0].x, condl1[1].x, condl1[2].x);
	Gb_v3_set( &(softMotion_data_l1->freeflyer->poseLinEnd), condl2[0].x, condl2[1].x, condl2[2].x);
	Gb_v3_set( &(softMotion_data_l1->freeflyer->velLinInit), condl1[0].v, condl1[1].v, condl1[2].v);
	Gb_v3_set( &(softMotion_data_l1->freeflyer->velLinEnd), condl2[0].v, condl2[1].v, condl2[2].v);

	Gb_v3_set( &(softMotion_data_l1->freeflyer->poseAngInit), condl1[3].x, condl1[4].x, condl1[5].x);
	Gb_v3_set( &(softMotion_data_l1->freeflyer->poseAngEnd),  condl2[3].x, condl2[4].x, condl2[5].x);
	Gb_v3_set( &(softMotion_data_l1->freeflyer->velAngInit),  condl1[3].v, condl1[4].v, condl1[5].v);
	Gb_v3_set( &(softMotion_data_l1->freeflyer->velAngEnd),   condl2[3].v, condl2[4].v, condl2[5].v);

	for(i=0;i<SM_NB_DIM;i++) {
		softMotion_data_l1->freeflyer->motion.IC[i] = condl1[i];
		softMotion_data_l1->freeflyer->motion.FC[i] = condl2[i];
	}

	for(i=0;i<SM_NB_DIM;i++) {

			if(segIdl1==0) {
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = softMotion_data_In->freeflyer->motion.Times[i].Tjpa - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = softMotion_data_In->freeflyer->motion.TimesM[i].Tjpa - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = softMotion_data_In->freeflyer->motion.TNE.Tjpa - paramDiffl1;
				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1;
				for(j=1;j<SM_NB_SEG;j++) {
					softMotion_data_l1->freeflyer->motion.TimeCumulM[i][j] = softMotion_data_In->freeflyer->motion.TimeCumulM[i][j] - paramDiffl1;
				}
			}

			if(segIdl1==1) {
				softMotion_data_l1->freeflyer->motion.Times[i].Taca = softMotion_data_In->freeflyer->motion.Times[i].Taca - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Taca = softMotion_data_In->freeflyer->motion.TimesM[i].Taca - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Taca = softMotion_data_In->freeflyer->motion.TNE.Taca - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = 0;

				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1 /100.0
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][1]/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1
																										- softMotion_data_In->freeflyer->motion.TimeCumulM[i][1];;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][1] = 0;
				for(j=2;j<SM_NB_SEG;j++) {
					softMotion_data_l1->freeflyer->motion.TimeCumulM[i][j] = softMotion_data_In->freeflyer->motion.TimeCumulM[i][j] - paramDiffl1
							-softMotion_data_In->freeflyer->motion.TimeCumulM[i][1];;
				}
			}

			if(segIdl1==2) {
				softMotion_data_l1->freeflyer->motion.Times[i].Tjna = softMotion_data_In->freeflyer->motion.Times[i].Tjna - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjna = softMotion_data_In->freeflyer->motion.TimesM[i].Tjna - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Tjna = softMotion_data_In->freeflyer->motion.TNE.Tjna - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Taca = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Taca = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Taca = 0;

				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1 /100.0
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][2]/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][2];;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][1] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][2] = 0;
				for(j=3;j<SM_NB_SEG;j++) {
					softMotion_data_l1->freeflyer->motion.TimeCumulM[i][j] = softMotion_data_In->freeflyer->motion.TimeCumulM[i][j] - paramDiffl1
							-softMotion_data_In->freeflyer->motion.TimeCumulM[i][2];
				}
			}

			if(segIdl1==3) {
				softMotion_data_l1->freeflyer->motion.Times[i].Tvc = softMotion_data_In->freeflyer->motion.Times[i].Tvc - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tvc = softMotion_data_In->freeflyer->motion.TimesM[i].Tvc - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Tvc = softMotion_data_In->freeflyer->motion.TNE.Tvc - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Taca = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Taca = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Taca = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjna = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjna = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjna = 0;

				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1 /100.0
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][3]/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][3];;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][1] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][2] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][3] = 0;
				for(j=4;j<SM_NB_SEG;j++) {
					softMotion_data_l1->freeflyer->motion.TimeCumulM[i][j] = softMotion_data_In->freeflyer->motion.TimeCumulM[i][j] - paramDiffl1
							-softMotion_data_In->freeflyer->motion.TimeCumulM[i][3];
				}
			}

			if(segIdl1==4) {
				softMotion_data_l1->freeflyer->motion.Times[i].Tjnb = softMotion_data_In->freeflyer->motion.Times[i].Tjnb - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjnb = softMotion_data_In->freeflyer->motion.TimesM[i].Tjnb - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Tjnb = softMotion_data_In->freeflyer->motion.TNE.Tjnb - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Taca = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Taca = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Taca = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjna = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjna = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjna = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tvc = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tvc = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tvc = 0;

				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1/100.0
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][4]/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][4];;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][1] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][2] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][3] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][4] = 0;
				for(j=5;j<SM_NB_SEG;j++) {
					softMotion_data_l1->freeflyer->motion.TimeCumulM[i][j] = softMotion_data_In->freeflyer->motion.TimeCumulM[i][j] - paramDiffl1
							-softMotion_data_In->freeflyer->motion.TimeCumulM[i][4];
				}
			}

			if(segIdl1==5) {
				softMotion_data_l1->freeflyer->motion.Times[i].Tacb = softMotion_data_In->freeflyer->motion.Times[i].Tacb - paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tacb = softMotion_data_In->freeflyer->motion.TimesM[i].Tacb - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Tacb = softMotion_data_In->freeflyer->motion.TNE.Tacb - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Taca = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Taca = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Taca = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjna = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjna = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjna = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tvc = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tvc = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tvc = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjnb = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjnb = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjnb = 0;

				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1 /100.0
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][5]/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][5];;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][1] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][2] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][3] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][4] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][5] = 0;
				for(j=6;j<SM_NB_SEG;j++) {
					softMotion_data_l1->freeflyer->motion.TimeCumulM[i][j] = softMotion_data_In->freeflyer->motion.TimeCumulM[i][j] - paramDiffl1
							-softMotion_data_In->freeflyer->motion.TimeCumulM[i][5];
				}
			}

			if(segIdl1==6) {
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpb = softMotion_data_In->freeflyer->motion.Times[i].Tjpb- paramDiffl1/100.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpb = softMotion_data_In->freeflyer->motion.TimesM[i].Tjpb - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpb = softMotion_data_In->freeflyer->motion.TNE.Tjpb - paramDiffl1;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjpa = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjpa = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Taca = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Taca = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Taca = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjna = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjna = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjna = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tvc = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tvc = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tvc = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tjnb = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tjnb = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tjnb = 0;
				softMotion_data_l1->freeflyer->motion.Times[i].Tacb = 0.0;
				softMotion_data_l1->freeflyer->motion.TimesM[i].Tacb = 0;
				softMotion_data_l1->freeflyer->motion.TNE.Tacb = 0;

				// Acc , Vel and Pos don't change
				softMotion_data_l1->freeflyer->motion.MotionDuration[i] = softMotion_data_In->freeflyer->motion.MotionDuration[i] - paramDiffl1 /100.0
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][6]/100.0;
				softMotion_data_l1->freeflyer->motion.MotionDurationM[i] = softMotion_data_In->freeflyer->motion.MotionDurationM[i] - paramDiffl1
						- softMotion_data_In->freeflyer->motion.TimeCumulM[i][6];;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][1] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][2] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][3] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][4] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][5] = 0;
				softMotion_data_l1->freeflyer->motion.TimeCumulM[i][6] = 0;

			}
	}

	/*
	*
	* Take into account l2
	*
	*/

	softMotion_data_copy_into(robotPt, softMotion_data_l1, softMotion_data);

	for(i=0;i<SM_NB_DIM;i++) {

		if(segIdl2==6) {
			if(segIdl1!=6) {
				softMotion_data->freeflyer->motion.Times[i].Tjpb  = paramDiffl2/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjpb = paramDiffl2;
				softMotion_data->freeflyer->motion.TNE.Tjpb       = paramDiffl2;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i]  - (softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0  - paramDiffl2/100.0);
				softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] -(softMotion_data_l1->freeflyer->motion.TNE.Tjpb - paramDiffl2);
			} else {
				softMotion_data->freeflyer->motion.Times[i].Tjpb  = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjpb = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TNE.Tjpb       = paramDiffl2 - paramDiffl1;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.MotionDurationM[i] = paramDiffl2 - paramDiffl1;
			}
		}

		if(segIdl2==5) {
			if(segIdl1!=5) {
				softMotion_data->freeflyer->motion.Times[i].Tacb  = paramDiffl2/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tacb = paramDiffl2;
				softMotion_data->freeflyer->motion.TNE.Tacb       = paramDiffl2;
				softMotion_data->freeflyer->motion.Times[i].Tjpb  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjpb = 0;
				softMotion_data->freeflyer->motion.TNE.Tjpb       = 0;

					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0
						- (softMotion_data_l1->freeflyer->motion.TNE.Tacb/100.0  - paramDiffl2/100.0);
				softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb
																									-(softMotion_data_l1->freeflyer->motion.TNE.Tjpb - paramDiffl2);

				softMotion_data->freeflyer->motion.TimeCumulM[i][6] =  softMotion_data_l1->freeflyer->motion.TimeCumulM[i][5] + paramDiffl2;

			} else {
				softMotion_data->freeflyer->motion.Times[i].Tacb  = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tacb = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TNE.Tacb       = paramDiffl2 - paramDiffl1;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.MotionDurationM[i] = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] =  paramDiffl2 - paramDiffl1;
			}
		}

		if(segIdl2==4) {
			if(segIdl1!=4) {
				softMotion_data->freeflyer->motion.Times[i].Tjnb  = paramDiffl2/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjnb = paramDiffl2;
				softMotion_data->freeflyer->motion.TNE.Tjnb       = paramDiffl2;
				softMotion_data->freeflyer->motion.Times[i].Tacb  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tacb = 0;
				softMotion_data->freeflyer->motion.TNE.Tacb       = 0;
				softMotion_data->freeflyer->motion.Times[i].Tjpb  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjpb = 0;
				softMotion_data->freeflyer->motion.TNE.Tjpb       = 0;

					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0
						- softMotion_data_l1->freeflyer->motion.TNE.Tacb/100.0
						- (softMotion_data_l1->freeflyer->motion.TNE.Tjnb/100.0  - paramDiffl2/100.0);
				softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb
						- softMotion_data_l1->freeflyer->motion.TNE.Tacb
						-(softMotion_data_l1->freeflyer->motion.TNE.Tjnb - paramDiffl2);

				softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4] + paramDiffl2;
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			} else {
				softMotion_data->freeflyer->motion.Times[i].Tjnb  = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjnb = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TNE.Tjnb       = paramDiffl2 - paramDiffl1;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.MotionDurationM[i] = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][5] =  paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			}

		}

		if(segIdl2==3) {
			if(segIdl1!=3) {
			softMotion_data->freeflyer->motion.Times[i].Tvc  = paramDiffl2/100.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tvc = paramDiffl2;
			softMotion_data->freeflyer->motion.TNE.Tvc       = paramDiffl2;
			softMotion_data->freeflyer->motion.Times[i].Tjnb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjnb = 0;
			softMotion_data->freeflyer->motion.TNE.Tjnb       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tacb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tacb = 0;
			softMotion_data->freeflyer->motion.TNE.Tacb       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tjpb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjpb = 0;
			softMotion_data->freeflyer->motion.TNE.Tjpb       = 0;

				// Acc , Vel and Pos don't change
			softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tacb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tjnb/100.0
					- (softMotion_data_l1->freeflyer->motion.TNE.Tvc/100.0 - paramDiffl2/100.0);
			softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb
					- softMotion_data_l1->freeflyer->motion.TNE.Tacb
					- softMotion_data_l1->freeflyer->motion.TNE.Tjnb
					-(softMotion_data_l1->freeflyer->motion.TNE.Tvc - paramDiffl2);

			softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3] + paramDiffl2;
			softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
			softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			} else {
				softMotion_data->freeflyer->motion.Times[i].Tvc  = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tvc = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TNE.Tvc       = paramDiffl2 - paramDiffl1;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.MotionDurationM[i] = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][4] =  paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			}
		}

		if(segIdl2==2) {
			if(segIdl1!=2) {
				softMotion_data->freeflyer->motion.Times[i].Tjna  = paramDiffl2/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjna = paramDiffl2;
				softMotion_data->freeflyer->motion.TNE.Tjna       = paramDiffl2;
				softMotion_data->freeflyer->motion.Times[i].Tvc  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tvc = 0;
				softMotion_data->freeflyer->motion.TNE.Tvc       = 0;
				softMotion_data->freeflyer->motion.Times[i].Tjnb  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjnb = 0;
				softMotion_data->freeflyer->motion.TNE.Tjnb       = 0;
				softMotion_data->freeflyer->motion.Times[i].Tacb  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tacb = 0;
				softMotion_data->freeflyer->motion.TNE.Tacb       = 0;
				softMotion_data->freeflyer->motion.Times[i].Tjpb  = 0.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjpb = 0;
				softMotion_data->freeflyer->motion.TNE.Tjpb       = 0;

					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0
						- softMotion_data_l1->freeflyer->motion.TNE.Tacb/100.0
						- softMotion_data_l1->freeflyer->motion.TNE.Tjnb/100.0
						- softMotion_data_l1->freeflyer->motion.TNE.Tvc/100.0
						- (softMotion_data_l1->freeflyer->motion.TNE.Tjna/100.0 - paramDiffl2/100.0);
				softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb
						- softMotion_data_l1->freeflyer->motion.TNE.Tacb
						- softMotion_data_l1->freeflyer->motion.TNE.Tjnb
						- softMotion_data_l1->freeflyer->motion.TNE.Tvc
						-(softMotion_data_l1->freeflyer->motion.TNE.Tjna - paramDiffl2);

				softMotion_data->freeflyer->motion.TimeCumulM[i][3] = softMotion_data->freeflyer->motion.TimeCumulM[i][2] + paramDiffl2;
				softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3];
				softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			} else {
				softMotion_data->freeflyer->motion.Times[i].Tjna  = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Tjna = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TNE.Tjna       = paramDiffl2 - paramDiffl1;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.MotionDurationM[i] = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][3] =  paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3];
				softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			}

		}

		if(segIdl2==1) {
			if(segIdl1!=1) {
			softMotion_data->freeflyer->motion.Times[i].Taca  = paramDiffl2/100.0;
			softMotion_data->freeflyer->motion.TimesM[i].Taca = paramDiffl2;
			softMotion_data->freeflyer->motion.TNE.Taca       = paramDiffl2;
			softMotion_data->freeflyer->motion.Times[i].Tjna  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjna = 0;
			softMotion_data->freeflyer->motion.TNE.Tjna       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tvc  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tvc = 0;
			softMotion_data->freeflyer->motion.TNE.Tvc       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tjnb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjnb = 0;
			softMotion_data->freeflyer->motion.TNE.Tjnb       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tacb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tacb = 0;
			softMotion_data->freeflyer->motion.TNE.Tacb       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tjpb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjpb = 0;
			softMotion_data->freeflyer->motion.TNE.Tjpb       = 0;

				// Acc , Vel and Pos don't change
			softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tacb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tjnb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tvc/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tjna/100.0
					- (softMotion_data_l1->freeflyer->motion.TNE.Taca/100.0 - paramDiffl2/100.0);
			softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb
					- softMotion_data_l1->freeflyer->motion.TNE.Tacb
					- softMotion_data_l1->freeflyer->motion.TNE.Tjnb
					- softMotion_data_l1->freeflyer->motion.TNE.Tvc
					- softMotion_data_l1->freeflyer->motion.TNE.Tjna
					-(softMotion_data_l1->freeflyer->motion.TNE.Taca - paramDiffl2);

			softMotion_data->freeflyer->motion.TimeCumulM[i][2] = softMotion_data->freeflyer->motion.TimeCumulM[i][1] + paramDiffl2;
			softMotion_data->freeflyer->motion.TimeCumulM[i][3] = softMotion_data->freeflyer->motion.TimeCumulM[i][2];
			softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3];
			softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
			softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			} else {
				softMotion_data->freeflyer->motion.Times[i].Taca  = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.TimesM[i].Taca = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TNE.Taca       = paramDiffl2 - paramDiffl1;
					// Acc , Vel and Pos don't change
				softMotion_data->freeflyer->motion.MotionDuration[i] = paramDiffl2/100.0- paramDiffl1/100.0;
				softMotion_data->freeflyer->motion.MotionDurationM[i] = paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][2] =  paramDiffl2 - paramDiffl1;
				softMotion_data->freeflyer->motion.TimeCumulM[i][3] = softMotion_data->freeflyer->motion.TimeCumulM[i][2];
				softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3];
				softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
				softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
			}

		}

		if(segIdl2==0) {
			softMotion_data->freeflyer->motion.Times[i].Tjpa  = paramDiffl2/100.0 - paramDiffl1/100.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjpa = paramDiffl2 - paramDiffl1;
			softMotion_data->freeflyer->motion.TNE.Tjpa       = paramDiffl2 - paramDiffl1;
			softMotion_data->freeflyer->motion.Times[i].Taca  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Taca = 0;
			softMotion_data->freeflyer->motion.TNE.Taca       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tjna  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjna = 0;
			softMotion_data->freeflyer->motion.TNE.Tjna       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tvc  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tvc = 0;
			softMotion_data->freeflyer->motion.TNE.Tvc       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tjnb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjnb = 0;
			softMotion_data->freeflyer->motion.TNE.Tjnb       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tacb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tacb = 0;
			softMotion_data->freeflyer->motion.TNE.Tacb       = 0;
			softMotion_data->freeflyer->motion.Times[i].Tjpb  = 0.0;
			softMotion_data->freeflyer->motion.TimesM[i].Tjpb = 0;
			softMotion_data->freeflyer->motion.TNE.Tjpb       = 0;

				// Acc , Vel and Pos don't change
			softMotion_data->freeflyer->motion.MotionDuration[i] = softMotion_data_l1->freeflyer->motion.MotionDuration[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tacb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tjnb/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tvc/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Tjna/100.0
					- softMotion_data_l1->freeflyer->motion.TNE.Taca/100.0
					- (softMotion_data_l1->freeflyer->motion.TNE.Tjpa/100.0 - paramDiffl2/100.0 - paramDiffl1/100.0);
			softMotion_data->freeflyer->motion.MotionDurationM[i] = softMotion_data_l1->freeflyer->motion.MotionDurationM[i] - softMotion_data_l1->freeflyer->motion.TNE.Tjpb
					- softMotion_data_l1->freeflyer->motion.TNE.Tacb
					- softMotion_data_l1->freeflyer->motion.TNE.Tjnb
					- softMotion_data_l1->freeflyer->motion.TNE.Tvc
					- softMotion_data_l1->freeflyer->motion.TNE.Tjna
					- softMotion_data_l1->freeflyer->motion.TNE.Taca
					-(softMotion_data_l1->freeflyer->motion.TNE.Tjpa - paramDiffl2 - paramDiffl1);


			softMotion_data->freeflyer->motion.TimeCumulM[i][1] =  paramDiffl2 - paramDiffl1;
			softMotion_data->freeflyer->motion.TimeCumulM[i][2] = softMotion_data->freeflyer->motion.TimeCumulM[i][1];
			softMotion_data->freeflyer->motion.TimeCumulM[i][3] = softMotion_data->freeflyer->motion.TimeCumulM[i][2];
			softMotion_data->freeflyer->motion.TimeCumulM[i][4] = softMotion_data->freeflyer->motion.TimeCumulM[i][3];
			softMotion_data->freeflyer->motion.TimeCumulM[i][5] = softMotion_data->freeflyer->motion.TimeCumulM[i][4];
			softMotion_data->freeflyer->motion.TimeCumulM[i][6] = softMotion_data->freeflyer->motion.TimeCumulM[i][5];
		}
	}

	/* We assume that this a ptp motion thus all axis share the same motion time */
	sm_sum_motionTimes(&softMotion_data->freeflyer->motion.TimesM[0], &sum);
	softMotion_data->freeflyer->motionTime = (int)sum;

	sub_localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_data, 0, TRUE);

	p3d_destroy_softMotion_data(robotPt, softMotion_data_l1);

	sub_localpathPt->length_lp = softMotion_data->freeflyer->motionTime;
	sub_localpathPt->range_param = softMotion_data->freeflyer->motionTime;

	return sub_localpathPt;
}

/*
 *  does nothing
 */
p3d_localpath *p3d_simplify_softMotion(p3d_rob *robotPt, p3d_localpath *localpathPt, int *need_colcheck)
{
	return localpathPt;
}

void p3d_softMotion_write_curve_for_bltplot(p3d_localpath* lp, FILE* fileptr, int* index)
{
	int i=0;
	int paramDiff = 0;
	int paramLocal = 0;
	int segId = 0;
	SM_SEGMENT segment[SM_NB_DIM];
	int param = 0;
	SM_COND cond[SM_NB_DIM];
	p3d_softMotion_data *specificPt = lp->specific.softMotion_data;;

	for(param=0; param<= specificPt->freeflyer->motionTime; param++) {
			for (i=0;i<SM_NB_DIM;i++) {
				(*index) = *index + 1;
				lm_get_softMotion_segment_params_FREEFLYER( specificPt, (int)param, &segment[i], &segId, i);
		//compute pose at param
				if (param >= specificPt->freeflyer->motion.MotionDurationM[i]) {
					paramLocal = (int)specificPt->freeflyer->motion.MotionDurationM[i];
				}
				else {
					paramLocal = (int)param;
				}
				lm_get_paramDiff_for_param( specificPt, &segment[i], segId, i, paramLocal, &paramDiff);

				sm_CalculOfAccVelPosAtTime(paramDiff, &segment[i], &cond[i]);
			}
			fprintf(fileptr,"%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ;\n", *index, cond[0].a, cond[0].v, cond[0].x, cond[1].a, cond[1].v, cond[1].x,cond[2].a, cond[2].v, cond[2].x,cond[3].a, cond[3].v, cond[3].x, cond[4].a, cond[4].v, cond[4].x, cond[5].a, cond[5].v, cond[5].x);
			//printf("index %d \n",*index);
		}
		return;
}


int p3d_softMotion_localplanner_FREEFLYER(p3d_rob* robotPt, int mlpId, p3d_group_type gpType, p3d_softMotion_data* softMotion_data) {
	p3d_localpath *localpathPt = NULL;
	int equal = 0;
	int index_dof = robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[0]]->index_dof;
	int nb_dof = 0;
	int nbJoints = robotPt->mlp->mlpJoints[mlpId]->nbJoints;
	psoftMotion_str softMotion_params = NULL;

	SM_POSELIMITS poseLimits;
	double errorDist = 0.0, errorDistMax=0.0;
	double norVelLin = 0.0;
	int cptAppel = 0;
	int i=0, ntest = 0, collision = 0;
	Gb_v3 velLinEndTmp, velAngEndTmp, poselinEnd, poseAngInit, poseLinInit;
	Gb_th thInit, thDelta, thInvInit, thEnd, thEndp1;

	p3d_matrix4 freeflyerPose_init, freeflyerPose_end, freeflyerPose_endp1;



	/* If initconfPt == goalconfPt, free initconfPt and goalconfPt and return NULL */
	for(int v=0; v<nbJoints; v++) {

		nb_dof += robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[v]]->user_dof_equiv_nbr;
	}
	equal = p3d_equal_config_n_offset(nb_dof, index_dof, softMotion_data->q_init, softMotion_data->q_end);

	if(equal && softMotion_data->isPTP == TRUE) {
// 		PrintInfo((("MP: p3d_softMotion_localplanner PA10: q_init = q_goal! \n")));
		p3d_set_search_status(P3D_CONFIG_EQUAL);
		return FALSE;
	}

	if(softMotion_data->isPTP == TRUE) {
		// It's a point to point motion

		poseLimits.linear.maxJerk = softMotion_data->freeflyer->J_max_lin;
		poseLimits.linear.maxAcc  = softMotion_data->freeflyer->A_max_lin;
		poseLimits.linear.maxVel  = softMotion_data->freeflyer->V_max_lin;
		poseLimits.angular.maxJerk = softMotion_data->freeflyer->J_max_ang;
		poseLimits.angular.maxAcc  = softMotion_data->freeflyer->A_max_ang;
		poseLimits.angular.maxVel  = softMotion_data->freeflyer->V_max_ang;

		p3d_mat4PosReverseOrder(freeflyerPose_init, softMotion_data->q_init[index_dof],
														softMotion_data->q_init[index_dof+1],
							softMotion_data->q_init[index_dof+2],
							softMotion_data->q_init[index_dof+3],
			 softMotion_data->q_init[index_dof+4],
		softMotion_data->q_init[index_dof+5]);

		p3d_mat4PosReverseOrder(freeflyerPose_end, softMotion_data->q_end[index_dof],
														softMotion_data->q_end[index_dof+1],
							softMotion_data->q_end[index_dof+2],
			 softMotion_data->q_end[index_dof+3],
		softMotion_data->q_end[index_dof+4],
	softMotion_data->q_end[index_dof+5]);

		p3d_mat4PosReverseOrder(freeflyerPose_endp1, softMotion_data->q_endp1[index_dof],
														softMotion_data->q_endp1[index_dof+1],
							softMotion_data->q_endp1[index_dof+2],
			 softMotion_data->q_endp1[index_dof+3],
		softMotion_data->q_endp1[index_dof+4],
	softMotion_data->q_endp1[index_dof+5]);


		lm_convert_p3dMatrix_To_GbTh(freeflyerPose_init ,&thInit);
		lm_convert_p3dMatrix_To_GbTh(freeflyerPose_end,&thEnd);
		lm_convert_p3dMatrix_To_GbTh(freeflyerPose_endp1,&thEndp1);

		Gb_th_inverse(&thInit, &thInvInit);
		Gb_th_produit(&thInvInit, &thEnd, &thDelta);

 		if(SOFT_MOTION_PRINT_DATA) {
			PrintInfo(("first point x=%f y=%f z=%f\n",softMotion_data->q_init[index_dof], softMotion_data->q_init[index_dof+1], softMotion_data->q_init[index_dof+2]));
			PrintInfo(("middle point x=%f y=%f z=%f\n",softMotion_data->q_end[index_dof], softMotion_data->q_end[index_dof+1], softMotion_data->q_end[index_dof+2]));
			PrintInfo(("final point x=%f y=%f z=%f\n",softMotion_data->q_endp1[index_dof], softMotion_data->q_endp1[index_dof+1], softMotion_data->q_endp1[index_dof+2]));
		}

 		lm_set_Gb_v3_for_translation_and_rotation(&thInit, &softMotion_data->freeflyer->poseLinInit, &softMotion_data->freeflyer->poseAngInit);
 		lm_set_Gb_v3_for_translation_and_rotation(&thEnd, &softMotion_data->freeflyer->poseLinEnd, &softMotion_data->freeflyer->poseAngEnd);

		lm_set_Gb_v3_for_translation_and_rotation(&thDelta,  &softMotion_data->freeflyer->poseLinEnd, &softMotion_data->freeflyer->poseAngEnd);
		Gb_v3_set(&poseAngInit, 0.0, 0.0, 0.0);
		Gb_v3_set(&poseLinInit, 0.0, 0.0, 0.0);
//============================================================================


		/* Set initial and final conditions (SM_COND IC and SM_COND FC structures needed by the planner) in softMotion_data */
		lm_set_cond_softMotion_data_FREEFLYER(poseLinInit,
																			softMotion_data->freeflyer->poseLinEnd,
																			poseAngInit,
																			softMotion_data->freeflyer->poseAngEnd,
																			softMotion_data->freeflyer->velLinInit,
																			softMotion_data->freeflyer->velAngInit,
																			softMotion_data->freeflyer->velLinEnd,
																			softMotion_data->freeflyer->velAngEnd, softMotion_data);

		/* Compute the point to point motion */
		if(sm_ComputeSoftMotionPointToPoint(softMotion_data->freeflyer->motion.IC, softMotion_data->freeflyer->motion.FC, poseLimits,
			 &softMotion_data->freeflyer->motion)!=0) {
				 PrintError(("p3d softMotion localpath CANNOT compute point to point motion on group FREEFLYER\n"));
				 return FALSE;
		}

		/* Determine the motion duration */
		softMotion_data->freeflyer->motionTime = 0;
		for(i=0;i<SM_NB_DIM;i++) {
			if(softMotion_data->freeflyer->motion.MotionDurationM[i] > softMotion_data->freeflyer->motionTime) {
				softMotion_data->freeflyer->motionTime = softMotion_data->freeflyer->motion.MotionDurationM[i];
			}
		}
		return TRUE;

	}
	else {
		/* PTP motion is already computed */

		errorDist = 1.0; /* Set arbitrary to a lare value */
		cptAppel = 0; /* Backup the number of times the planner was called */
		errorDistMax = 0.06; /* set the maximum distance between the path followed by the tool and the point to point motion */
		collision = 1; /* Initialize flag collision */

		/* Set initial and final conditions (SM_COND IC and SM_COND FC structures needed by the planner) in softMotion_data */
		lm_set_cond_softMotion_data_FREEFLYER( softMotion_data->freeflyer->poseLinInit,
																			softMotion_data->freeflyer->poseLinEnd,
																			softMotion_data->freeflyer->poseAngInit,
																			softMotion_data->freeflyer->poseAngEnd,
																			softMotion_data->freeflyer->velLinInit,
																			softMotion_data->freeflyer->velAngInit,
																			softMotion_data->freeflyer->velLinEnd,
																			softMotion_data->freeflyer->velAngEnd, softMotion_data);

		while((errorDist > errorDistMax) || (collision==1)) {

			/* Compute the planner and the maximal distance between the new path and the straight line motion */
			lm_compute_softMotion_for_freeflyer(softMotion_data);
			lm_compute_error_dist(softMotion_data, &errorDist);

			/* Determine the motion duration */
			softMotion_data->freeflyer->motionTime = 0;
			for(i=0;i<SM_NB_DIM;i++) {
				if(softMotion_data->freeflyer->motion.MotionDurationM[i] > softMotion_data->freeflyer->motionTime) {
					softMotion_data->freeflyer->motionTime = softMotion_data->freeflyer->motion.MotionDurationM[i];
				}
			}

			if( (ABS(softMotion_data->freeflyer->velLinEnd.x) > EPS4)
							&& (ABS(softMotion_data->freeflyer->velLinEnd.y) > EPS4)
							&& (ABS(softMotion_data->freeflyer->velLinEnd.z) > EPS4)
							&& (ABS(softMotion_data->freeflyer->velAngEnd.x) > EPS4)
							&& (ABS(softMotion_data->freeflyer->velAngEnd.y) > EPS4)
							&& (ABS(softMotion_data->freeflyer->velAngEnd.z) > EPS4)) {

				localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_data, 0, TRUE);

				if (p3d_col_test_localpath(robotPt, localpathPt, &ntest)){
					collision = 1;
					localpathPt->destroy(robotPt, localpathPt);
					localpathPt = NULL;
				} else {
					collision = 0;
				}

							} else {

								localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_data, 0, TRUE);

								/* End Velocity is smaller than EPS4 so if there is a collision Init velocity may be too high */
								if (p3d_col_test_localpath(robotPt, localpathPt, &ntest)){
									collision = 1;
									localpathPt->destroy(robotPt, localpathPt);
									localpathPt = NULL;
									return FALSE;
								} else {
									collision = 0;
									localpathPt->destroy(robotPt, localpathPt);
									localpathPt = NULL;
									return TRUE;
								}
							}

							if ((errorDist > errorDistMax) || (collision==1)) {
								/* Decrease end velocity */
								Gb_v3_product_r(&(softMotion_data->freeflyer->velLinEnd), VELOCITY_STEP_PERCENT, &(velLinEndTmp));
								Gb_v3_product_r(&(softMotion_data->freeflyer->velAngEnd), VELOCITY_STEP_PERCENT, &(velAngEndTmp));
								Gb_v3_moins(&(softMotion_data->freeflyer->velLinEnd), &(velLinEndTmp), &(softMotion_data->freeflyer->velLinEnd));
								Gb_v3_moins(&(softMotion_data->freeflyer->velAngEnd), &(velAngEndTmp), &(softMotion_data->freeflyer->velAngEnd));

								/* Update initial and final conditions in softMotion_data */
								lm_set_cond_softMotion_data_FREEFLYER( softMotion_data->freeflyer->poseLinInit,
																		softMotion_data->freeflyer->poseLinEnd,
																		softMotion_data->freeflyer->poseAngInit,
																		softMotion_data->freeflyer->poseAngEnd,
																		softMotion_data->freeflyer->velLinInit,
																		softMotion_data->freeflyer->velAngInit,
																		softMotion_data->freeflyer->velLinEnd,
																		softMotion_data->freeflyer->velAngEnd, softMotion_data);
							}
							cptAppel ++;
		}
		/* Not needed explicitely but it for you °_~ */
		norVelLin = Gb_v3_module(&(softMotion_data->freeflyer->velLinEnd));
		PrintInfo(("PA10 Velocity norVelLin %f cptAppel %d \n", norVelLin, cptAppel));
		return TRUE;
		}
}

int p3d_softMotion_localplanner_JOINT(p3d_rob* robotPt, int mlpId, p3d_group_type gpType, p3d_softMotion_data* sm_data) {

	sm_data->isPlanned = FALSE;
	return TRUE;
}
 #endif

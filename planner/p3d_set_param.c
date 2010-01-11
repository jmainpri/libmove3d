//#include "Util-pkg.h"
#include "P3d-pkg.h"
//#include "Planner-pkg.h"
//#include "Localpath-pkg.h"
//#include "Collision-pkg.h"

/***************************************************************/
/* Fichier contenant les fonctions permettant de fixer les     */
/* parametres des algorithmes de planification                 */
/***************************************************************/


/* parametres pour les algorithmes de planification */

static double DMAX = -66.6;  /* modif. Carl :  mieux vaut valeur initial */

static int NB_NODES = 10000;
static int COMP_NODES = 10000;

static int EXPAND_CHOICE = 1;
static int SAMPLING_CHOICE = P3D_UNIFORM_SAMPLING;
static int RANDOM_CHOICE = P3D_RANDOM_SAMPLING;
static int MOTION_PLANNER = P3D_ISOLATE_LINKING;

static int PLANNER_CHOICE = 1;

static int SORTING = P3D_DIST_NODE;
static int COST = FALSE;
static int ORIENTED = 0;
static int EXPANDING = 0;
static int DB_RRT_MODE = 0;

static int PASSIVE_PARAMS = 0;

static int NBNODE = 10;
static int NBWALK = 5;
static int NB_OPTIM = 30;
static int NB_TRY = 1000;
static int NTRY_DD = 15;
static int NB_specific = 1;

double *EXPAND_DQ_EXP;    /* angles expressed in radian */
static double *EXPAND_DELTA_EXP; /* angles expressed in radian */
static double T_MAX = 1800; // temps max de calcul en secondes.
static double OPTIMIZATION_TIME = 1; //Temps de calcul max d'une optimisation en secondes
static int USE_OPTIMIZATION_TIME = 0; //Temps de calcul max d'une optimisation en secondes
static int IK_CHOICE = IK_NORMAL;
static int IK_DRAW_CURRENT[10]; //UGLY
static int PLANNING_TYPE = P3D_NONE;
static int SAVEINFOINFILE = 0;

//start path deform
static int DOF1 = 0;
static int DOF2 = 0;
static int CYCLES = FALSE;
static int TEST_REDUC = FALSE;
static int IS_VISIBILITY_DISCREET = FALSE;
static int NRETRACT = 1;
static int NUM_DRAW_PROJ = 1;
static int Nstep = 40;
static int SHOW_CYCLE = 0;
static int RETRACT_SEARCH_TYPE = ALL_RETRACT_SEARCH;
static int NRETRACT_MAX = 5;
static int IS_GEN_PROJ = FALSE;
static int MAX_CONNECT = 1000000;
static int GVISPLOT = FALSE;
//end path deform

static int STOP = FALSE;
static double IS_WEIGHTED_ROTATIONS;
static p3d_node* CURRENT_NEAR_NODE;
#ifdef MULTIGRAPH
static int ISMULTIGRAPH = 0;
#endif

/**
 * p3d_SetCurrentNearNode
 * Set which is the near/best node found in
 * the roadmap.
 * @param[In]: the near/best node found in
 * the roadmap.
 */
void p3d_SetCurrentNearNode(p3d_node *NearNodePt) {
  CURRENT_NEAR_NODE = NearNodePt;
}

/**
 * p3d_GetCurrentNearNode
 * Get  which is the near/best node found in
 * the roadmap.
 * @return: the near/best node found in
 * the roadmap.
 */
p3d_node* p3d_GetCurrentNearNode(void) {
  return CURRENT_NEAR_NODE;
}

/* seuil de distance au dela duquel on ne cherche */
/* plus les voisins d un noeud                    */
void p3d_set_DMAX(double D)
{
	ENV.setDouble(Env::dist,D);
}

double p3d_get_DMAX(void)
{
  return(ENV.getDouble(Env::dist));
}

/***************************/
/* Fonction calculant DMAX */
/* In : le robot           */
/* Out :                   */
/***************************/
double p3d_calc_DMAX(p3d_rob *robotPt)
{double x1,x2,y1,y2,z1,z2;
 double min,max,D=0.; 
 int njnt=robotPt->njoints, i, j;
 p3d_jnt * jntPt;

  if(p3d_get_desc_number(P3D_ENV)){
    p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
    D = SQR(x2-x1)+SQR(y2-y1)+SQR(z2-z1);
    for(i=0; i<=njnt; i++){
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	p3d_jnt_get_dof_bounds(jntPt, j, &min, &max);
	D = D + SQR(max-min);
      }
    }
    D = sqrt(D)/3.;
    p3d_set_DMAX(D);
  }
  return(D);
}


int p3d_GetStopValue() {
  return STOP;
}
void p3d_SetStopValue(int Stop) {
  STOP = Stop;
}


/* nombre maximum de noeuds a examiner dans une  */
/* composante connexe                            */


void p3d_set_COMP_NODES(int N)
{
	ENV.setInt(Env::maxNodeCompco,N);
}

int p3d_get_COMP_NODES(void)
{
	return ENV.getInt(Env::maxNodeCompco);
}

void p3d_set_NB_NODES(int N)
{
  NB_NODES = N;
}

int p3d_get_NB_NODES(void)
{
  return(NB_NODES);
}

/* methode d'expansion des noeuds  */
void p3d_set_EXPAND_CHOICE(int N)
{
  EXPAND_CHOICE = N;
}

int p3d_get_EXPAND_CHOICE(void)
{
  return(EXPAND_CHOICE);
}
/* random number (Halton set/random) */
void p3d_set_RANDOM_CHOICE(int N)
{
  RANDOM_CHOICE = N;
}

int p3d_get_RANDOM_CHOICE(void)
{
  return(RANDOM_CHOICE);
}


/* generation aleatoire des noeuds (uniforme/gaussian) */
void p3d_set_SAMPLING_CHOICE(int N)
{
  SAMPLING_CHOICE = N;
}

int p3d_get_SAMPLING_CHOICE(void)
{
  return(SAMPLING_CHOICE);
}

void p3d_set_EXPAND_NB_NODES(int N)
{
  NBNODE = N;
}

int p3d_get_EXPAND_NB_NODES(void)
{
  return(NBNODE);
}

void p3d_set_EXPAND_NB_WALK(int N)
{
  NBWALK = N;
}

int p3d_get_EXPAND_NB_WALK(void)
{
  return(NBWALK);
}


/* option expansion dans specific  */

void p3d_set_EXPAND_OPTION(int N)
{
  EXPANDING = N;
}
  
int p3d_get_EXPAND_OPTION(void)
{
  return(EXPANDING);
}

void p3d_set_EXPAND_DQ(double *dq)
{
  EXPAND_DQ_EXP = dq;
}

double *p3d_get_EXPAND_DQ(void)
{
  return(EXPAND_DQ_EXP);
}

void p3d_set_EXPAND_DELTA(double *dq)
{
  EXPAND_DELTA_EXP = dq;
}

double *p3d_get_EXPAND_DELTA(void)
{
  return(EXPAND_DELTA_EXP);
}


/* methode locale    */
void p3d_set_PLANNER_CHOICE(int N)
{
  PLANNER_CHOICE = N;
}

int p3d_get_PLANNER_CHOICE(void)
{
  return(PLANNER_CHOICE);
}


/* seuil de distance au dela duquel on ne cherche */
/* plus les voisins d un noeud                    */
void p3d_set_MOTION_PLANNER(int N)
{
  MOTION_PLANNER = N;

}

int p3d_get_MOTION_PLANNER(void)
{
  return(MOTION_PLANNER);
}

void p3d_set_SORTING(int N)
{
  SORTING = N;
}

int p3d_get_SORTING(void)
{
  return(SORTING);
}

// modif Juan
void p3d_set_rrt_pb_mode(int N)
{
  DB_RRT_MODE = N;
}

int p3d_get_rrt_pb_mode(void)
{
  return(DB_RRT_MODE);
}


void p3d_set_flag_passive_parameters_for_planner(int val)
{
  PASSIVE_PARAMS = val;
}

int p3d_get_flag_passive_parameters_for_planner(void)
{
  return(PASSIVE_PARAMS);
}
// fmodif Juan

void p3d_set_ORIENTED(int N)
{
  ORIENTED = N;
}

int p3d_get_ORIENTED(void)
{
  return(ORIENTED);
}

void p3d_set_nbtry_DD(int N)
{
   NTRY_DD = N;
}

int p3d_get_nbtry_DD(void)
{
  return(NTRY_DD);
}


/* nombre d'iterations pour l'optimisation */
void p3d_set_NB_OPTIM(int N)
{
   NB_OPTIM = N;
}

int p3d_get_NB_OPTIM(void)
{
  return(NB_OPTIM);
}

void p3d_set_NB_specific(int N)
{
   NB_specific = N;
}

int p3d_get_NB_specific(void)
{
  return(NB_specific);
}

void p3d_set_tmax(double tmax){
  T_MAX = tmax;
}

double p3d_get_tmax(void){
  return(T_MAX);
}

void p3d_set_optimization_time(double optTime){
  OPTIMIZATION_TIME = optTime;
}

double p3d_get_optimization_time(void){
  return(OPTIMIZATION_TIME);
}

void p3d_set_use_optimization_time(int useOptTime){
  USE_OPTIMIZATION_TIME = useOptTime;
}

double p3d_get_use_optimization_time(void){
  return(USE_OPTIMIZATION_TIME);
}


void p3d_set_ik_choice(int ik_choice){
  IK_CHOICE = ik_choice;
}

int p3d_get_ik_choice(void){
  return IK_CHOICE;
}

void p3d_init_ik_draw_current(){
  int i = 0;
  for(i = 0; i < 10; i++){
    IK_DRAW_CURRENT[i] = 0;
  }
}

void p3d_set_ik_draw(int ik_draw, int value){
  IK_DRAW_CURRENT[ik_draw] = value;
}

int* p3d_get_ik_draw(){
  return IK_DRAW_CURRENT;
}

int p3d_get_planning_type(){
  return PLANNING_TYPE;
}

void p3d_set_planning_type(int planning_type){
  PLANNING_TYPE = planning_type;
}

int p3d_get_saveInfoInFile(){
  return SAVEINFOINFILE;
}

void p3d_set_saveInfoInFile(int saveInfoInFile){
  SAVEINFOINFILE = saveInfoInFile;
}

#ifdef MULTIGRAPH
void p3d_set_multiGraph(int multiGraph){
  ISMULTIGRAPH = multiGraph;
}

int p3d_get_multiGraph(void){
  return(ISMULTIGRAPH);
}
#endif

void p3d_set_costComputation(int cost){
  COST = cost;
}

int p3d_get_costComputation(void){
  return(COST);
}

//start path deform
void p3d_set_dof1(int dof1){
  DOF1 = dof1;
}
int p3d_get_dof1(void){
  return(DOF1);
}

void p3d_set_dof2(int dof2){
  DOF2 = dof2;
}
int p3d_get_dof2(void){
  return(DOF2);
}

void p3d_set_cycles(int cycles){
  CYCLES = cycles;
}
int p3d_get_cycles(void){
  return(CYCLES);
}

void p3d_set_test_reductib(int test_reduc){
 TEST_REDUC = test_reduc ;
}
int p3d_get_test_reductib(void){
  return(TEST_REDUC);
}

void p3d_set_is_visibility_discreet(int vis){
  IS_VISIBILITY_DISCREET = vis;
}
int p3d_get_is_visibility_discreet(void){
  return(IS_VISIBILITY_DISCREET);
}

void p3d_set_nretract(int nretract){
  NRETRACT = nretract;
}
int p3d_get_nretract(void){
  return(NRETRACT);
}

void p3d_set_num_draw_proj(int num){
  NUM_DRAW_PROJ = num;
}
int p3d_get_num_draw_proj(void){
  return(NUM_DRAW_PROJ);
}

void p3d_set_Nstep(int nStep){
  Nstep = nStep;
}
int p3d_get_Nstep(void){
  return(MIN(Nstep,100));
}

void p3d_set_show_cycle_only(int cycle){
  SHOW_CYCLE = cycle;
}
int p3d_get_show_cycle_only(void){
  return(SHOW_CYCLE);
}

void p3d_set_retract_search_type(int type){
  RETRACT_SEARCH_TYPE = type;
}
int p3d_get_retract_search_type(void){
  return(RETRACT_SEARCH_TYPE);
}

void p3d_set_nretract_max(int nretract){
  NRETRACT_MAX = nretract;
}
int p3d_get_nretract_max(void){
  return(NRETRACT_MAX);
}

void p3d_set_is_general_proj(int gen_proj){
  IS_GEN_PROJ = gen_proj;
}
int p3d_get_is_general_proj(void){
  return(IS_GEN_PROJ);
}

void p3d_set_max_connect(int n)
{
  MAX_CONNECT = n;
}

int p3d_get_max_connect(void)
{
  return(MAX_CONNECT);
}

void p3d_set_gvis(int n)
{
  GVISPLOT = n;
}
int p3d_get_gvis(void)
{
  return(GVISPLOT);
}

/**
 * p3d_GetIsWeightedRotations
 * Get if rotations are weighted
 * @return: TRUE if the rotations are weighted
 */
int p3d_GetIsWeightedRotations(void) {
  return IS_WEIGHTED_ROTATIONS;
}

/**
 * p3d_SetIsWeightedRotations
 * Set if rotations are weighted
 * @param[In] IsWeightedRota: TRUE if the rotations are weighted
 */
void p3d_SetIsWeightedRotations(int IsWeightedRota) {
 IS_WEIGHTED_ROTATIONS = IsWeightedRota;
}



//end path deform


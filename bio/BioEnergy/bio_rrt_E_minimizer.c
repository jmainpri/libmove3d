#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "UserAppli-pkg.h"  
#include "Localpath-pkg.h"
#include "Planner-pkg.h"
#include "Util-pkg.h"
#include "Collision-pkg.h"
#include "Bio-pkg.h"
#include "include/Energy-pkg.h"

#include "include/bioenergy_common.h"

 

#define NUMBERPAIRES 10
#define MAXN_FAILS_E 10
#define MAXN_EXTEND_E 30
#define DEBUGRRT_E 1
//static double moyen_cal = 0;
//static double min_dist_cal = 0;
//static int num_col_cal = 0;

static double E_THRESHOLD;
static double AVRD_THRESHOLD;
static double MIND_THRESHOLD;


static double ene_energy_calcul(p3d_rob *robot)
{
  double totE;

  CalculateEnergy(robot,&totE);
  return totE;
}


/********************************************************
 * la fonction qui le poids en ft de la dist etl'energy *
 *param dist: la distance entre deux configurations     *
 * param energy: l'energie calcul� pour conf q          *
 *return  weight = A*dist + norme_energy                *
 ********************************************************/
//static double surface_d;

//static double bio_coll_step_length = 0.2;  // default (max. penetration = 0.1 A)
//static double bio_dis = 0.1;




//#ifdef HYDROGEN_BOND

//#define SQR_SUM_DIST (f= p1->radii[p2->type] + surface_d) * f

//#else

//#define SQR_SUM_DIST (f= p1->r + p2->r + surface_d) * f

//#endif

//#define SQR_DIST ((f= *(v1) - *(v2)) * f) + ((f= *(++v1) - *(++v2)) * f) + ((f= *(++v1) - *(++v2)) * f)


//double dist_poly(p3d_poly *p1, p3d_poly *p2)
//{
  // register double *v1= p1->pos;
  // register double *v2= p2->pos;
// register double f;
// return  SQR_SUM_DIST;
//}

static double 
ene_weight_dist_energy(double dist, double energy)
{
  double energy_normalise = 0;
  double weight = 0.0;

  energy_normalise = (E_THRESHOLD - energy) / E_THRESHOLD ; 
  // NOTE JUAN : le choix d'alpha est delicat et (je pense) dependant de la molecule
  //weight = dist + ALPHA*energy_normalise;  // ALPHA IS NOT DEFINED !!!
  weight = dist + 1.0*energy_normalise;
  //printf("--------- dist = %f --------- energy_normalise = %f ------- weight = %f\n",dist,energy_normalise,weight);
  return weight;
}


/**********************************************************
 *la fonction qui retourne le voisin plus proche          *
 * param rob: le robot                                    * 
 * param q:  configuration                                *
 * param comp: component connect�                         *
 * return le noeud plus proche et avec energie minimal    *
 **********************************************************/

static p3d_node *hrm_nearest_neighbor_energy(p3d_rob *rob, 
				      configPt q, 
				      p3d_compco *comp){
  p3d_list_node *ListNode  = comp->dist_nodes;
  p3d_node *Nmin_energy = NULL;
  double weight_min, weight;
  double  E, d;

  printf("******************************** Num.nodes = %d\n",comp->nnode);
  weight_min = P3D_HUGE;
  while(ListNode != NULL){    
    if((comp->nnode == 1)||
       ((ListNode->N->n_fail_extend < MAXN_FAILS_E)&&(ListNode->N->n_extend < MAXN_EXTEND_E))) {
      //      d = hrm_dist_config(rob, q,ListNode->N->q);
      d = p3d_dist_config(rob, q,ListNode->N->q);
      // correct. Juan : E est deja calcule !!! (verifier pour "root")
      // ListNode->N->E = ene_energy_calcul(rob);
      E = ListNode->N->E;
      weight = ene_weight_dist_energy(d , E); 
      // Juan : ponderation
      weight *= ((double)MAXN_EXTEND_E)/((double)(MAXN_EXTEND_E - ListNode->N->n_extend));
      //printf("weight =  %f\n",weight);
      if (weight < weight_min) {
	weight_min = weight ;                   
	Nmin_energy = ListNode->N;
      }
    }
    ListNode = ListNode->next;
  }
  return Nmin_energy;
}
 
/* static void swap_mod(double a[], int i, int j) { */
/*     double tmp = a[i]; */
/*     a[i] = a[j]; */
/*     a[j] = tmp; */
/* } */

/* static int Random_mod(int i, int j) { */
/*     return i + rand() % (j-i+1); */
/* } */

/* void  */
/* quicksort_mod(double a[], int left, int right) { */
/*     int last = left, i; */

/*     if (left >= right) return; */

/*     swap_mod(a,left,Random_mod(left,right)); */
/*     for (i = left + 1; i <= right; i++) */
/*         if (a[i] < a[left]) */
/*             swap_mod(a,++last,i); */
/*     swap_mod(a,left,last); */
/*     quicksort_mod(a,left,last-1); */
/*     quicksort_mod(a,last+1,right); */
/* } */


/***************************************************************
* la fonction qui calcule le nombre de collision, le nombre de *
*paires d'atones qui ont une distance inferieure a une distance*
*donn�, ordone cette distance et en fin calcule sa moyenne     * 
****************************************************************/

// correct. Juan 
static int ene_paire_proche_ou_collision(int *n_pairs, double *min_dist, double *avr_dist) 
{	
  p3d_poly **p1,**p2;
  double *distances;
  int i;
  int n_dist_pairs;
  double sum = 0;
  //int in_collision;
  
  /////////////////  FOR NEW BIOCD /////////////////////////////
  // JUAN : AVEC NOUVEAU MODE BIOCD
  //bio_set_col_mode(SURFDIST_COL_BIOCOL_MODE);
  //bio_set_surface_d(1.0);   // WARNING : SURFACE DIST. CODE EN DUR !!! 
  //bio_all_molecules_col();
  //in_collision = biocol_distance_report(&n_dist_pairs, &p1, &p2, &distances);
  //bio_set_col_mode(NORMAL_BIOCOL_MODE);

  //if(in_collision)
  //  return 1;
  //////////////////////////////////////////////////////////////

  ///// WITH OLD BIOCD ////  
  if (bio_all_molecules_col() > 0){ 
    return 1;
  }

  bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);
  bio_set_surface_d(1.0); 
  bio_all_molecules_col();
  biocol_report_dist(&n_dist_pairs, &p1, &p2, &distances);
  bio_set_col_mode(NORMAL_BIOCOL_MODE);
  /////////////////////////

  *n_pairs = n_dist_pairs;
  
  //for(i = 0; i <n_dist_pairs ; i++){  // correct. Juan (c'etatit n_coll)
  //  printf("la paire  %s   %s est a distance : %f � \n", p1[i]->poly->name, p2[i]->poly->name, distances[i] );
  //}

  *min_dist = distances[0];
  for(i = 1; i <n_dist_pairs; i++){
    sum += distances[i];
    if(distances[i] < *min_dist)
      *min_dist = distances[i];
  }
  *avr_dist = sum / n_dist_pairs;

  //printf("la plus petite distance est %f\n",*min_dist);
  //printf("la distance moyenne est %f\n",*avr_dist);  

  return 0;
}


/*****************************************
 * fonction qui la validit� de localpath  *
 *parm robot: le robot                    *
 *parm localpath: le localpath a test�    *
 *parm ntest : le nomber le test          *
 *parm kpath :                            *
 *parm q_atkpath:                         *
 *return un boolean(localpath valid ou non*
 ******************************************/  
#define MAXCONF 1000
//#define MAXNODE 10000
//#define NODE_PATH 4
//#define NLPATHS_RECOMP_STEP 100
//static int      n_steps_rrt_N = 10;
//static int      min_n_steps_rrt_N = 3;
//static int      wanted_n_inter_rrt_N = 3;
//static int      count_lpaths = 0;
//static int      array_n_inter_rrt_N[NLPATHS_RECOMP_STEP];

//static double   config_min_dist[MAXCONF];
//static double   config_avr_dist[MAXCONF];
//static double   path_dist[MAXCONF];
//static configPt   config_save[MAXCONF];
//static double   array_energy_node[MAXCONF];

// modif Juan
typedef struct s_ene_inter_conf_data {
  int      index;
  double   mind;
  double   avrd;
  double   path_dist;
  configPt q;
  double   E;
} ene_inter_conf_data;

static ene_inter_conf_data interconf_data[MAXCONF];

// fmodif Juan

static double energy_count[MAXCONF];
static double min_dist_count[MAXCONF] ;
static double avr_dist_count[MAXCONF];

// JUAN : begin new function 
// WARNING : cette fonction utilisse des vaiables globales !!!
// garde 1 ou 2 confs en fonction de la taille du path !!!      
static void ene_generate_intermediate_nodes(p3d_rob *robotPt, int n_confs_path)
{
  int *index_min_avrd;
  //int *index_min_mind;
  int i,j;
  double max_avrd,pmax_avrd;
  double max_avrd_diff;
  int index_best;
  p3d_node *NewNode = NULL, *BaseNode = NULL;
  int      *iksol = NULL;
  configPt q;

  printf("------------------------------------------- n_confs_path = %d\n",n_confs_path);

  // returns in there are no interediate confs
  if(n_confs_path == 0)
    return;

  BaseNode = p3d_GetCurrentNearNode();
  
  // NOTE : currently, heuristic is only based on avrd 
  index_min_avrd = MY_ALLOC(int, n_confs_path);
  //index_min_mind = MY_ALLOC(int, n_confs_path);

  // simple (but inefficient) sorting
  pmax_avrd = P3D_HUGE;
  for(i=0;i<n_confs_path;i++) {
    max_avrd = -P3D_HUGE;
    for(j=0;j<n_confs_path;j++) {
      if((interconf_data[j].avrd > max_avrd)&&(interconf_data[j].avrd < pmax_avrd)) {
	index_min_avrd[i] = j;
	max_avrd = interconf_data[j].avrd;
      }      
    }
    pmax_avrd = interconf_data[index_min_avrd[i]].avrd;
  }
  max_avrd_diff = interconf_data[index_min_avrd[0]].avrd - 
                  interconf_data[index_min_avrd[n_confs_path - 1]].avrd;
  
  // identify "best" conf
  i = 0;
  //if(n_confs_path > 1) { // PROBLEM WITH 1 !!!???
  if(n_confs_path > 2) {
    // DEBUG
/*     if(n_confs_path == 2) */
/*       printf("OJOORRR!!!\n"); */
    ///////
    while(index_min_avrd[i] <= (int) floor((double)n_confs_path / 2.0))
      i++; 
  }
  if(interconf_data[index_min_avrd[i]].avrd > 
     (interconf_data[index_min_avrd[0]].avrd - max_avrd_diff/2.0))
    index_best = index_min_avrd[i];
  else
    index_best = index_min_avrd[0];


  // generate node for "best" conf
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,interconf_data[index_best].q);
  p3d_get_iksol_vector(robotPt->cntrt_manager,iksol);
  q = p3d_alloc_config(robotPt);
  p3d_copy_config_into(robotPt,interconf_data[index_best].q,&q); 
  NewNode = p3d_APInode_make_multisol(robotPt->GRAPH,q,iksol);
  NewNode->type = LINKING;
  NewNode->avr_dist = interconf_data[index_best].avrd;
  NewNode->min_dist = interconf_data[index_best].mind;
  if(interconf_data[index_best].E > P3D_HUGE)
    NewNode->E = ene_energy_calcul(robotPt);
  else
    NewNode->E = interconf_data[index_best].E;
  p3d_insert_node(robotPt->GRAPH,NewNode);
  p3d_create_edges(robotPt->GRAPH,BaseNode,NewNode,interconf_data[index_best].path_dist);
  p3d_add_node_compco(NewNode, BaseNode->comp, TRUE);	

  // generate another node : only for long paths
  //if(n_confs_path > 5) {
      
  // aqui !!!

  //}

  // free qs in interconf_data
  for(i=0;i<n_confs_path;i++)
    p3d_destroy_config(robotPt, interconf_data[i].q);

}
// end new function

static int bio_energy_test_localpath_step(p3d_rob *robotPt,
					  p3d_localpath *localpathPt,
					  int *ntest_energy,int *ntest_coll, double *Kpath,
					  configPt *q_atKpath)
{
  double   u, du, umax; 
  configPt qsave, qp;
  int      i,j, njnt = robotPt->njoints;
  double   *distances;
  int      end_localpath = 0;
  double   step_dep;
  int      counter_steps_rrt_N = 0;
  int      AVRDCONTROL  =  0;
  int      MINDCONTROL =  0;
  int      n_pairs =0; 
  double   min_dist=0;
  double   avr_dist =0 ;
  double   E_node = 0;
  int n_inter_confs = 0;

  u = 0.0;
  *Kpath = 0.0;
  bio_col_get_step_deplacement(&step_dep);
  if (localpathPt == NULL)
    { return FALSE; }

  // JUAN : INIT ARRAY E
  for(i=0;i<MAXCONF;i++)
    interconf_data[i].E = P3D_HUGE + 1.0;
  
  // current position of robot is saved
  qsave = p3d_get_robot_config(robotPt);
  qp = p3d_alloc_config(robotPt);
  p3d_copy_config_into(robotPt, localpathPt->specific.lin_data->q_init, &qp);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
  umax = localpathPt->range_param;
  distances = MY_ALLOC(double, njnt+1);

  for (j=0; j<=njnt; j++)
    { distances[j] = step_dep; }  
  du = localpathPt->stay_within_dist(robotPt, localpathPt,
				     u, FORWARD, distances);
  u+=du;
  if (u > umax - EPS6){
    u = umax;
    end_localpath = 1;
  }

  while(!end_localpath) {  
    if (change_position_robot_multisol(robotPt, localpathPt, u, du, qp)) {
      // JUAN : WARNING : - fuction qui marche avec variables globales
      //                  - doit faire free de q !!! 
      ene_generate_intermediate_nodes(robotPt,n_inter_confs);
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return TRUE;
    }
    p3d_get_robot_config_into(robotPt, &qp);
    
    /* collision checking  and  test of energy *****/
    // JUAN : CETTE FONCTION VERIFIE COLLISIONS !!!!
    if(ene_paire_proche_ou_collision(&n_pairs, &min_dist, &avr_dist)) {
      // JUAN : WARNING : - fuction qui marche avec variables globales
      //                  - doit faire free de q !!!
      ene_generate_intermediate_nodes(robotPt,n_inter_confs);
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return TRUE;
    }
    *ntest_coll = *ntest_coll + 1;
    
    interconf_data[n_inter_confs].mind = min_dist;
    interconf_data[n_inter_confs].avrd = avr_dist;
    // JUAN : CETTE FONCTION ALOUE ET COPIE !!!
    // NOTE : q is necessary instead of simply saving Kpath for cases with loops
    interconf_data[n_inter_confs].q = p3d_copy_config(robotPt,qp);
    interconf_data[n_inter_confs].path_dist = *Kpath * localpathPt->length_lp;
    interconf_data[n_inter_confs].index = n_inter_confs;
    n_inter_confs++;

    // JUAN : AU K OU !!!
    //        les arrais de taille constante doivent etre remplaces par variables !!!
    if(n_inter_confs> MAXCONF) {
      // JUAN : WARNING : - fuction qui marche avec variables globales
      //                  - doit faire free de q !!!
      ene_generate_intermediate_nodes(robotPt,n_inter_confs);
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return TRUE;
    }

    // correct. Juan
    // NOTE : on commence a regarder E a partir d''un certain num. de pas des l'origine
    if(n_inter_confs > 3) {  
      //MINDCONTROL = (interconf_data[n_inter_confs - 1].mind > interconf_data[n_inter_confs - 2].mind);
      //AVRDCONTROL = (interconf_data[n_inter_confs - 1].avrd > interconf_data[n_inter_confs - 2].avrd);
      MINDCONTROL = (interconf_data[n_inter_confs - 1].mind > AVRD_THRESHOLD);
      AVRDCONTROL = (interconf_data[n_inter_confs - 1].avrd > MIND_THRESHOLD);
      //printf(" MINDCONTROL %d AVRDCONTROL %d \n", MINDCONTROL, AVRDCONTROL);

      //  la configuration actuel se trouve dans une distance min et moyenne plus petite que la conf precedente
      if( !MINDCONTROL && !AVRDCONTROL ){
	E_node = ene_energy_calcul(robotPt);
	interconf_data[n_inter_confs - 1].E =  E_node ;
	*ntest_energy = *ntest_energy + 1;
	if(E_node > E_THRESHOLD){
	  // JUAN : WARNING : - fuction qui marche avec variables globales
	  //                  - doit faire free de q !!!
	  ene_generate_intermediate_nodes(robotPt,n_inter_confs);
	  /* The initial position of the robot is recovered */
	  p3d_set_and_update_this_robot_conf(robotPt, qsave);
	  p3d_destroy_config(robotPt, qsave);
	  p3d_destroy_config(robotPt, qp);
	  MY_FREE(distances, double, njnt+1);
	  return TRUE; 
	}
      }
    }
    
    *Kpath = u / localpathPt->range_param;
    if(q_atKpath != NULL)
      p3d_get_robot_config_into(robotPt, q_atKpath);        
    for (j=0; j<=njnt; j++)
      { distances[j] = step_dep; }  
    counter_steps_rrt_N ++;
    du = localpathPt->stay_within_dist(robotPt, localpathPt, u, FORWARD, distances);
    u+=du;
    if (u > umax - EPS6){
      u = umax;
      end_localpath = 1;
    }
  }    

  *Kpath = 1.0;
  if(q_atKpath != NULL)
    p3d_copy_config_into(robotPt, localpathPt->specific.lin_data->q_end, q_atKpath);
  

  // JUAN : WARNING : - fuction qui marche avec variables globales
  //                  - doit faire free de q !!!
  ene_generate_intermediate_nodes(robotPt,n_inter_confs);
  /* The initial position of the robot is recovered */
  p3d_set_and_update_this_robot_conf(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qp);
  MY_FREE(distances, double, njnt+1);
  return FALSE; 
}

static void p3d_print_min_energy_and_set_conf(p3d_graph *G){
  //int i;
  double energy_min = P3D_HUGE, energy;
  p3d_list_node *ListNode  = G->nodes;
  p3d_node *NminE = NULL;

  while(ListNode != NULL){
    energy = ListNode->N->E;
    if(energy < energy_min ){
      energy_min = energy; 
      NminE = ListNode->N;
    }
    ListNode = ListNode->next;
  }
  printf("Minumum Energy  : %f\n", energy_min);
  p3d_set_and_update_this_robot_conf_without_cntrt(G->rob,NminE->q);
  g3d_draw_allwin_active();
  // for(i=0;i<MAXCONF;i++){
  // printf("l'energy est %f\n",energy_count[i]);
  // }
}

static void p3d_print_dist_avrg_energy_evaluation(p3d_graph *G){
  p3d_list_node *ListNode  = G->nodes;
  double energy = 0;
  double avr_dist =0;
  double min_dist = 0;
  configPt  qp;
  int count = 0;
  printf("energy\t\t\tmoyen\t\t\tdistance_min\n");
  while(ListNode != NULL && count < 400){
    count++;
    energy = ListNode->N->E;
    avr_dist =  ListNode->N->avr_dist;
    min_dist = ListNode->N->min_dist;
    qp= ListNode->N->q;
    printf("%f\t\t%f\t\t\t%f\t\n",energy,avr_dist,min_dist);
  }
}

static void p3d_print_dist_avrg_energy(void){
  int i;
  
  for(i = 0; i < MAXCONF; i++){
    printf("la distance %f moyen %f energy %f\n",
	   min_dist_count[i] ,avr_dist_count[i], energy_count[i]);
  }
}

/*****************************************
* fonction qui la validit� de localpath  *
*parm robot: le robot                    *
*parm localpath: le localpath a test�    *
*parm ntest : le nomber le test          *
*parm kpath :                            *
*parm q_atkpath:                         *
*return un boolean(localpath valid ou non*
******************************************/
static int  p3d_unvalid_localpath_energy_test(p3d_rob *robotPt,
					      p3d_localpath *localpathPt,
					      int *ntest_energy,int *ntest_coll, double *Kpath,
					      configPt *q_atKpat)
{
  int unvalid;  
  unvalid = bio_energy_test_localpath_step(robotPt,localpathPt,ntest_energy,ntest_coll,Kpath,q_atKpat);
  return (unvalid);  
}

 /*******************************************************************
 *la fonction qui extende rrt_energy de noeud voisin vers une q_rand*                                                         
 *param rob:  le robot                                              *
 * param G :  le graphe                                             *
 * param K : le pas (read the documention)                          *
 * param Nnear:  plus proche voisin de la configuration qnew        *            
 * param qnew : la configuration random                             *
 * le N_new viene d'etre ajouter � Nnear->q + K*(qnew - Nnear->q)   *
 * si il n'y a pas de collision et l'enrgie est petit au seuil      *
 * return pointer � N_new or NULL                                   *
 ********************************************************************/  
static int ngoods = 0;
static int ntries = 0;

static int hrm_extend_with_energy(p3d_rob *rob, 
			      p3d_graph *G, 
			      p3d_node *Nnear,
			      configPt qnew)
{
  //p3d_node      *NewNode = NULL;
  configPt      q;
  p3d_localpath *path;
  double        Kpath  = 0;
  //double avr_dist = 0;
  //double dist_min =0;
  //int num_col = 0;    
  
  if(DEBUGRRT_E)
    printf("Nnear = %d -- n_fail_extend = %d\n", Nnear->num,Nnear->n_fail_extend);
  if (p3d_equal_config(rob, qnew, Nnear->q))
    {
      if((rob->cntrt_manager == NULL)||(rob->cntrt_manager->cntrts == NULL))  
	PrintInfo(("qnew invalide\n"));
      return 0;
    }
  (G->nb_q_closed)++;

  if (!(path = p3d_local_planner(rob, Nnear->q, qnew)))
    {
      PrintInfo(("impossible de planifier\n"));
      return 0;
    }
  (G->nb_local_call)++;
  q = p3d_alloc_config(rob);
  p3d_unvalid_localpath_energy_test(rob, path, &(G->nb_test_energy),&(G->nb_test_coll),&Kpath, &q);
  printf("Kpath = %f\n",Kpath);
  if (Kpath==0)
    {
      ntries++;
      ngoods = 0;
      PrintInfo(("noeud trop proche\n"));
      path->destroy(rob,path);
      p3d_destroy_config(rob,q);
      return 0;
    }
  printf("RRT tries: %d\n",ntries+1);
  ntries = 0;
  ngoods ++;

  //dist = Kpath * path->length_lp;
  path->destroy(rob,path);

  // JUAN : ON NE GARDE PAS LA DERNIERE CONF (NOEUDS GARDES AVANT) !!!
  //p3d_get_iksol_vector(rob->cntrt_manager,iksol);
  //NewNode = p3d_APInode_make_multisol(G, q, iksol);
  //ene_paire_proche_ou_collision(&num_col, &dist_min, &avr_dist);
  //NewNode->type = LINKING;
  //NewNode->E = ene_energy_calcul(rob);
  //NewNode->avr_dist =avr_dist;
  //NewNode->min_dist =dist_min;
  // NewNode->q=q;
  //array_n_inter_rrt_energy_N[count_lpaths-1]= NewNode->E ;
  //p3d_insert_node(G, NewNode);
  //p3d_create_edges(G,Nnear,NewNode,dist);
  //p3d_add_node_compco(NewNode, Nnear->comp);

  return 1;
}


//++++++++++++++++++++++++

static int hrm_expand_one_rrt_energy(p3d_graph *G, p3d_compco **CompPt, configPt q)
{
  int Added = FALSE;
  p3d_node *Nnear;
  p3d_rob *rob = G->rob;

  Nnear = hrm_nearest_neighbor_energy(rob,q,*CompPt);
  p3d_SetCurrentNearNode(Nnear);
  Added = hrm_extend_with_energy(rob,G,Nnear,q);

  Nnear->n_extend++;
  if (!Added)
    Nnear->n_fail_extend++;     

 return Added;
}


//++++++++++++++++++++


static int p3d_expand_start_rrt_energy(p3d_graph *G,int (*fct_stop)(void)) 
{
  configPt q;
  // int nnodemax =  100; // p3d_get_COMP_NODES();
  int Added = FALSE;
  int Stop = FALSE;
  int actRLG;

  q = p3d_alloc_config(G->rob);
  while(!Stop)
    {
      if (fct_stop)
	if (!(*fct_stop)())
	  {
	    PrintInfo(("RRT_ENERGY building canceled\n"));
	    p3d_destroy_config(G->rob, q);
	    return FALSE;
	  }
      Added = FALSE;
      Stop = TRUE;
      actRLG = p3d_get_RLG();
      p3d_set_RLG(0);
      p3d_shoot(G->rob,q,1);
      p3d_set_RLG(actRLG);
      Added = hrm_expand_one_rrt_energy(G, &(G->search_start->comp), q); 
      Stop = Added;	
    }
  PrintInfo(("  %4d   \r",G->search_start->comp->nnode));
  p3d_destroy_config(G->rob, q);
  return Added;
}


// +++++++++++++++
// MAIN EXTERNAL FUNCTION 
int bio_rrt_E_minimize(double *qs, int *iksols, 
		       int (*fct_stop)(void),void (*fct_draw)(void))
{
  p3d_rob   *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph *G;
  p3d_node  *Ns=NULL; //*Ng=NULL;
  p3d_list_node *graph_node;
  configPt  qtest, q_s=NULL; // q_g=NULL; 
  int       inode=0 ,ADDED;
  double    tu,ts;
  //int nnodemax;
  int       *iksols_s=NULL,*iksolg_s=NULL;
  //int i;
  double curr_E = 0;
  int i;
  int n_dist_pairs;
  p3d_poly **p1; 
  p3d_poly **p2;
  double *distances;
  double sum = 0;
  double avr_dist = 0;
  double min_dist = 0;
  //int in_collision;


  if(qs == NULL){
    PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration initiale\n"));
    return(FALSE);
  }

  
  ChronoOn();

  if(!XYZ_GRAPH)  G = p3d_create_graph();
  else            G = XYZ_GRAPH;

  /* Nodes QS and QG exist ?*/
  graph_node = G->nodes;

  while(graph_node) {
    qtest = graph_node->N->q;
    if(p3d_equal_config(robotPt, qs, qtest)) 	Ns = graph_node->N;
    graph_node = graph_node->next;    
  }

  /* If not, create them */
  if(Ns == NULL) {
    q_s = p3d_copy_config(robotPt, qs);
    p3d_copy_iksol(robotPt->cntrt_manager, iksols, iksolg_s);
    Ns  = p3d_APInode_make_multisol(G,q_s,iksols_s);
    p3d_insert_node(G,Ns);
    p3d_create_compco(G,Ns);
    Ns->type = ISOLATED;

    // set robot and compute E (correct. Juan)
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qs);
    curr_E = ene_energy_calcul(G->rob);
    
    
    /////////////////  FOR NEW BIOCD /////////////////////////////
    // JUAN : AVEC NOUVEAU MODE BIOCD
    //bio_set_col_mode(SURFDIST_COL_BIOCOL_MODE);
    //bio_set_surface_d(1.0);   // WARNING : SURFACE DIST. CODE EN DUR !!! 
    //bio_all_molecules_col();
    //in_collision = biocol_distance_report(&n_dist_pairs, &p1, &p2, &distances);
    //bio_set_col_mode(NORMAL_BIOCOL_MODE);

    //if(in_collision)
    //  return 1;
    //////////////////////////////////////////////////////////////

    ///// WITH OLD BIOCD ////  
    if (bio_all_molecules_col() > 0){ 
      printf("COLLISION !!!\n");
    }

    bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);
    bio_set_surface_d(1.0); 
    bio_all_molecules_col();
    biocol_report_dist(&n_dist_pairs, &p1, &p2, &distances);
    bio_set_col_mode(NORMAL_BIOCOL_MODE);
    /////////////////////////

    //for(i = 0; i <n_dist_pairs ; i++){  // correct. Juan (c'etatit n_coll)
    //printf("la paire  %s   %s est a distance : %f � \n", p1[i]->poly->name, p2[i]->poly->name, distances[i] );
    //}
    
    min_dist = distances[0];
    for(i = 1; i <n_dist_pairs; i++){
      sum += distances[i];
      if(distances[i] < min_dist)
	min_dist = distances[i];
    }
    avr_dist = sum / n_dist_pairs;

    Ns->E = curr_E;
    Ns->avr_dist = avr_dist;
    Ns->min_dist = min_dist;
    
    // set thresholds (modif. Juan)
    E_THRESHOLD = curr_E + fabs(curr_E*0.3);   // correct. Juan (fabs)
    AVRD_THRESHOLD = avr_dist - fabs(avr_dist*0.3);   // correct. Juan (fabs)
    MIND_THRESHOLD = min_dist - fabs(min_dist*0.3);   // correct. Juan (fabs)
    
    if(p3d_link_node_graph(Ns,G)) 
      PrintInfo(("qs reliee au graphe\n"));
    else 
      p3d_APInode_expand(G,Ns,fct_stop,fct_draw);
  }

  /* Initialize some data in the graph for the A* graph search */
  G->search_start = Ns;
  G->search_done = FALSE;
  //FAIRE UN BOTON POUR FAIRE VARIER LE NOMBER DE NOEUD
  // nnodemax =  p3d_get_COMP_NODES();

  ADDED = TRUE;
  /* While solution does not exists, insert new nodes with RRT_ENERGY */
  //  while(G->search_start->comp->nnode < p3d_get_COMP_NODES()){   
  while(G->search_start->comp->nnode < 50){   
    ADDED = p3d_expand_start_rrt_energy(G, fct_stop);
    } 
    if(ADDED) {
      inode = inode + 1;
      if (fct_draw) (*fct_draw)();
    }
    else {
      PrintInfo(("bio_rrt_E_minimize : ECHEC a l'insertion d'un noeud\n"));
      //  break;
    }
  
  
  PrintInfo(("Pour la creation de %d noeuds : ",inode));
  ChronoPrint("");
  ChronoTimes(&tu,&ts);
  G->time = G->time + tu;
  ChronoOff();
  p3d_print_info_graph(G);
  p3d_print_min_energy_and_set_conf(G);
  // p3d_print_dist_avrg_energy_evaluation(G);
  // p3d_print_dist_avrg_energy();
  // MY_ALLOC_INFO("After p3d_specific_learn");


 return(ADDED);
}

  
static void print_les_paires_proches(void){
  int i;
  int n_dist_pairs;
  p3d_poly **p1; 
  p3d_poly **p2;
  double *distances;
  double sum = 0;
  double avr_dist = 0;
  double min_dist = 0;
  //int in_collision;


  /////////////////  FOR NEW BIOCD /////////////////////////////
  // JUAN : AVEC NOUVEAU MODE BIOCD
  //bio_set_col_mode(SURFDIST_COL_BIOCOL_MODE);
  //bio_set_surface_d(1.0);   // WARNING : SURFACE DIST. CODE EN DUR !!! 
  //bio_all_molecules_col();
  //in_collision = biocol_distance_report(&n_dist_pairs, &p1, &p2, &distances);
  //bio_set_col_mode(NORMAL_BIOCOL_MODE);

  //if(in_collision)
  //  return 1;
  //////////////////////////////////////////////////////////////

  ///// WITH OLD BIOCD ////  
  if (bio_all_molecules_col() > 0){ 
    printf("COLLISION !!!\n");
  }

  bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);
  bio_set_surface_d(1.0); 
  bio_all_molecules_col();
  biocol_report_dist(&n_dist_pairs, &p1, &p2, &distances);
  bio_set_col_mode(NORMAL_BIOCOL_MODE);
  /////////////////////////

  for(i = 0; i <n_dist_pairs ; i++){  // correct. Juan (c'etatit n_coll)
  printf("la paire  %s   %s est a distance : %f � \n", p1[i]->poly->name, p2[i]->poly->name, distances[i] );
  }
  
  min_dist = distances[0];
  for(i = 1; i <n_dist_pairs; i++){
    sum += distances[i];
    if(distances[i] < min_dist)
      min_dist = distances[i];
  }
  avr_dist = sum / n_dist_pairs;

  printf("la plus petite distance est %f\n",min_dist);
  printf("la distance moyenne est %f\n",avr_dist);  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY EVALUATION
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void ene_evaluate_traj(void)
{
  pp3d_rob robotPt = (pp3d_rob) p3d_get_desc_curid(P3D_ROBOT);
  pp3d_localpath localpathPt;
  int i;
  int n_dist_pairs;
  p3d_poly **p1; 
  p3d_poly **p2;
  double *distances;
  double sum = 0;
  double avr_dist;
  double min_dist;
  double E;
  configPt q;
  int nconfs;

  if(robotPt->tcur == NULL)
    {
      PrintInfo(("ene_evaluate_traj : no current trajectory\n"));
      return;
    }
  
  // set BioCD mode
  bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);  // OLD BIOCD
  //bio_set_col_mode(SURFDIST_COL_BIOCOL_MODE);    // NEW BIOCD
  bio_set_surface_d(1.0);   // WARNING : SURFACE DIST. CODE EN DUR !!! 

 
  localpathPt = robotPt->tcur->courbePt;
  nconfs = 0;
  while (localpathPt != NULL){
    nconfs ++;
    // place "robot" at intitial conf. of the local path
    q = localpathPt->config_at_param(robotPt, localpathPt, 0);
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, q);
    p3d_destroy_config(robotPt, q);
    
    // call bio_coll
    //biocol_distance_report(&n_dist_pairs, &p1, &p2, &distances);  // OLD BIOCD
    bio_all_molecules_col();                                        // NEW BIOCD
    biocol_report_dist(&n_dist_pairs, &p1, &p2, &distances);           // NEW BIOCD
    
    // evaluate distances
    min_dist = distances[0];
    sum = 0;
    for(i = 1; i <n_dist_pairs; i++){
	sum += distances[i];
	if(distances[i] < min_dist)
	  min_dist = distances[i];
    }
    avr_dist = sum / n_dist_pairs;
    
    // evaluate E
    E = ene_energy_calcul(robotPt);    

    //print
    printf("CONF %d : avr_dist = %f  min_dist = %f  E = %f\n ",nconfs,avr_dist,min_dist,E);

    localpathPt = localpathPt->next_lp;  
  }
 
  // reset BioCD mode
  //bio_set_surface_d(dist_surf);
  bio_set_col_mode(NORMAL_BIOCOL_MODE);
  
}

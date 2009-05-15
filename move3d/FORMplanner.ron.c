#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Bio-pkg.h"
#include "sys/stat.h"

#define MAX_NB_TRY_OPTIM  20

int G3D_DRAW_TRAJ = FALSE;
int G3D_SAVE_MULT = FALSE;
int G3D_DRAW_GRAPH = FALSE;
//extern int test_brunet; // KINEO-DEV :doit �re d�lar�dans un .h !!
extern FL_OBJECT *planner_obj;  // KINEO-DEV: doit �re d�lar�dans un .h !!
extern FL_FORM *OPTIM_FORM;

FL_FORM  *PLANNER_FORM = NULL;

extern MENU_ROBOT *ROBOTS_FORM;

static void g3d_create_local_search_obj(void);
static void g3d_create_random_confs_obj(void);   // modif Juan
static void g3d_create_global_search_obj(void);
static void g3d_create_specific_search_obj(void);
static void g3d_create_N_specific_obj(void);
static void g3d_create_optim_obj(void);
static void g3d_create_drawnjnt_obj(void);    // modif Juan
static void g3d_create_global_input_obj(void);
static void g3d_create_global_param_obj(void);
static void g3d_create_compco_input_obj(void);
static void g3d_create_compco_param_obj(void);
static void g3d_create_del_param_obj(void);
static void g3d_create_DMAX_param_obj(void);
static void g3d_create_print_obj(void);
static void g3d_create_draw_obj(void);
static void g3d_create_draw_optim_obj(void);
static void g3d_create_save_mult_obj(void);
static void g3d_create_load_graph_obj(void);
static void g3d_create_save_graph_obj(void);
static void g3d_create_stop_obj(void);
static void g3d_create_pinpoint_node_obj(void);
static void g3d_create_expand_obj(void);
static void g3d_create_isolate_obj(void);
static void g3d_create_linking_obj(void);
static void g3d_create_test_obj(void);
static void g3d_create_del_node_obj(void);
static void g3d_create_plan_strategy_obj(void);
static void g3d_create_sorting_obj(void);
static void g3d_create_sampling_obj(void);
static void g3d_create_W_obj(void);  // modif Juan
static void g3d_create_nbtry_input_obj(void);
static void g3d_create_nbtry_param_obj(void);
static void g3d_create_ntryDD_param_obj(void);
static void g3d_create_lambda_param_obj(void);
static void g3d_create_add_obj(void);
static void g3d_create_frame_obj(void);
static void g3d_create_oriented_obj(void);

static void g3d_delete_local_search_obj(void);
static void g3d_delete_random_confs_obj(void);   // modif Juan
static void g3d_delete_drawnjnt_obj(void);   // modif Juan
static void g3d_delete_global_search_obj(void);
static void g3d_delete_specific_search_obj(void);
static void g3d_delete_N_specific_obj(void);
static void g3d_delete_optim_obj(void);
static void g3d_delete_global_input_obj(void);
static void g3d_delete_global_param_obj(void);
static void g3d_delete_compco_input_obj(void);
static void g3d_delete_compco_param_obj(void);
static void g3d_delete_del_param_obj(void);
static void g3d_delete_DMAX_param_obj(void);
static void g3d_delete_print_obj(void);
static void g3d_delete_draw_obj(void);
static void g3d_delete_draw_optim_search_obj(void);
static void g3d_delete_load_graph_obj(void);
static void g3d_delete_save_graph_obj(void);
static void g3d_delete_stop_obj(void);
static void g3d_delete_pinpoint_node_obj(void);
static void g3d_delete_expand_obj(void);
static void g3d_delete_isolate_obj(void);
static void g3d_delete_linking_obj(void);
static void g3d_delete_test_obj(void);
static void g3d_delete_del_node_obj(void);
static void g3d_delete_plan_strategy_obj(void);
static void g3d_delete_sorting_obj(void);
static void g3d_delete_sampling_obj(void);
static void g3d_delete_W_obj(void);    // modif Juan
static void g3d_delete_nbtry_input_obj(void);
static void g3d_delete_nbtry_param_obj(void);
static void g3d_delete_ntryDD_param_obj(void);
static void g3d_delete_lambda_param_obj(void);
static void g3d_delete_add_obj(void);
static void g3d_delete_frame_obj(void);
static void g3d_delete_oriented_obj(void);

/* booleen d'arret */
static int STOP = FALSE;

/* variable pour guarder la derniere conf. valide d'un robot */
static configPt last_p_deg;


/************************/
/* Option form creation */
/************************/
void g3d_create_planner_form(void)
{
  
  PLANNER_FORM = fl_bgn_form(FL_UP_BOX,370.0,435.0);
  g3d_create_local_search_obj(); 
  g3d_create_random_confs_obj();   // modif Juan
  g3d_create_global_search_obj();
  g3d_create_specific_search_obj();
  g3d_create_N_specific_obj();
  g3d_create_optim_obj();
  g3d_create_drawnjnt_obj();      // modif Juan 
  g3d_create_global_input_obj();
  g3d_create_global_param_obj();
  g3d_create_compco_input_obj();
  g3d_create_compco_param_obj();
  g3d_create_del_param_obj();
  g3d_create_DMAX_param_obj();
  g3d_create_print_obj();
  g3d_create_draw_obj();
  g3d_create_draw_optim_obj();
  g3d_create_save_mult_obj();
  g3d_create_save_graph_obj();
  g3d_create_load_graph_obj();
  g3d_create_stop_obj();
  g3d_create_pinpoint_node_obj();
  g3d_create_expand_obj();
  g3d_create_isolate_obj();
  g3d_create_linking_obj();
  g3d_create_test_obj();
  g3d_create_del_node_obj();  
  g3d_create_plan_strategy_obj();
  g3d_create_sorting_obj();
  g3d_create_sampling_obj();
  g3d_create_W_obj();  // modif Juan
  g3d_create_oriented_obj();
  g3d_create_nbtry_input_obj();
  g3d_create_nbtry_param_obj();
  g3d_create_ntryDD_param_obj();
  g3d_create_lambda_param_obj();
  g3d_create_add_obj();
  g3d_create_frame_obj();

  fl_end_form();

  g3d_create_optim_form();
}

/**********************************************************************/
static FL_OBJECT  *SEARCH_OBJ;
static FL_OBJECT  *RANDCONFS_OBJ;   // modif Juan
static FL_OBJECT  *SEARCH_GLOBAL_OBJ;
static FL_OBJECT  *SEARCH_SPECIFIC_OBJ;
static FL_OBJECT  *N_SPECIFIC_OBJ;

       FL_OBJECT  *SEARCH_LINEAR_OBJ;
       FL_OBJECT  *SEARCH_PLATEFORM_OBJ;
       FL_OBJECT  *SEARCH_ARM_OBJ;
       FL_OBJECT  *SEARCH_LOCAL_OBJ;
       FL_OBJECT  *SEARCH_MANHAT_OBJ;

       FL_OBJECT  *SEARCH_GLOBAL_PARAM_OBJ;
static FL_OBJECT  *SEARCH_GLOBAL_INPUT_OBJ;
static FL_OBJECT  *SEARCH_DEL_PARAM_OBJ;

       FL_OBJECT  *SEARCH_DMAX_PARAM_OBJ; 

       FL_OBJECT  *DRAWNJNT_OBJ;   // modif Juan

       FL_OBJECT  *SEARCH_DRAW_OBJ; 
       FL_OBJECT  *SEARCH_PRINT_OBJ; 

       FL_OBJECT  *SEARCH_COMPCO_PARAM_OBJ;
static FL_OBJECT  *SEARCH_COMPCO_INPUT_OBJ;

       FL_OBJECT *OPTIM_OBJ;

       FL_OBJECT  *SEARCH_DRAW_OPTIM_OBJ;

static FL_OBJECT  *SAVE_GRAPH_OBJ;
static FL_OBJECT  *LOAD_GRAPH_OBJ; 

static FL_OBJECT  *STOP_OBJ; 

static FL_OBJECT  *EXPAND_OBJ; 
static FL_OBJECT  *PINPOINT_NODE_OBJ; 
static FL_OBJECT  *ISOLATE_OBJ; 
static FL_OBJECT  *LINKING_OBJ; 

static FL_OBJECT  *DEL_NODE_OBJ; 

static FL_OBJECT  *TEST_OBJ; 

       FL_OBJECT  *STRAT1_OBJ; 
       FL_OBJECT  *STRAT2_OBJ;
       FL_OBJECT  *STRAT3_OBJ;
       FL_OBJECT  *STRAT4_OBJ;
       FL_OBJECT  *STRAT5_OBJ;
       FL_OBJECT  *STRAT6_OBJ;

       FL_OBJECT  *SORT1_OBJ; 
       FL_OBJECT  *SORT2_OBJ;

       FL_OBJECT  *SAMPL1_OBJ; 
       FL_OBJECT  *SAMPL2_OBJ;
       FL_OBJECT  *SAMPL3_OBJ; 
       FL_OBJECT  *SAMPL4_OBJ;


       FL_OBJECT  *ORIENTED_OBJ;

       FL_OBJECT  *W_OBJ;    // modif Juan

static FL_OBJECT  *GROUP2; 
static FL_OBJECT  *GROUP3;
static FL_OBJECT  *GROUP4;
 
       FL_OBJECT  *SEARCH_NBTRY_PARAM_OBJ;
static FL_OBJECT  *NTRYDD_PARAM_OBJ;
       FL_OBJECT  *LAMBDA_PARAM_OBJ;
static FL_OBJECT  *SEARCH_NBTRY_INPUT_OBJ; 

static FL_OBJECT  *ADD_OBJ;

static FL_OBJECT  *FRAME_OBJ;


/**********************************************************************/
static int  s_default_drawtraj_fct(void)
{
  fl_check_forms();
  return(!STOP);
}

/* planification local + test de validite */
static void CB_local_search_obj(FL_OBJECT *ob, long arg)
{
 p3d_rob *robotPt; 
 char      str[50],sti[20];
 p3d_localpath *localpathPt;
 int ntest,ntrj,ir;
 p3d_localplanner_type lpl_type;

 STOP = FALSE;

 ir = p3d_get_desc_curnum(P3D_ROBOT);
 robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 strcpy(str, "loctrj."); 
 sprintf(sti,"%s",robotPt->name); strcat(str,sti);
 sprintf(sti,"%s","."); strcat(str,sti);
 ntrj = p3d_get_desc_number(P3D_TRAJ)+1;
 sprintf(sti,"%d",ntrj); strcat(str,sti);

 lpl_type = robotPt->lpl_type;
 
 localpathPt = p3d_local_planner(robotPt,
         robotPt->ROBOT_POS,robotPt->ROBOT_GOTO);
 if(localpathPt != NULL){
   p3d_beg_desc(P3D_TRAJ,str);
   p3d_add_desc_courbe(localpathPt);
   p3d_end_desc();
   g3d_add_traj("Localsearch",p3d_get_desc_number(P3D_TRAJ));
 }
    
 PrintInfo(("MP : robot %d : p3d_localplanner : ",ir));
 switch(p3d_search_status()) {
 case P3D_SUCCESS: PrintInfo(("SUCCESS"));break;
 case P3D_FAILURE: PrintInfo(("FAILURE"));break;
 case P3D_CONFIG_EQUAL: PrintInfo(("CONFIG_EQUAL"));break;
 case P3D_ILLEGAL_START: PrintInfo(("ILLEGAL_START"));break;
 case P3D_ILLEGAL_GOAL: PrintInfo(("ILLEGAL_GOAL"));break;
 case P3D_WRONG_CALL: PrintInfo(("WRONG_CALL"));break;
 default: PrintInfo(("???"));
 }
 PrintInfo(("\n"));
    
 if(localpathPt != NULL){
   ntest=0;
   if(p3d_unvalid_localpath_test(robotPt, robotPt->tcur->courbePt, &ntest)){   // <- modif Juan
     PrintInfo(("p3d_col_test_localpath: PATH NOT VALID ! nbtest : %d\n",ntest));
   }
   else{
     PrintInfo(("p3d_col_test_localpath: PATH VALID; nbtest : %d\n",ntest));
   }
 }

 g3d_show_tcur_rob(robotPt,s_default_drawtraj_fct);
 fl_set_button(SEARCH_OBJ,0);
}



static void g3d_create_local_search_obj(void)
{ 
  SEARCH_OBJ = fl_add_button(FL_PUSH_BUTTON,
          20.0,20.0,50.0,30.0,
            "Local\nPlanner");     // modif Juan
  fl_set_call_back(SEARCH_OBJ,CB_local_search_obj,0);
}


/***********************************************************/
/****************** planificateur global *******************/
/***********************************************************/

int fct_stop(void)
{
  static double ti = 0;
  double ts, tu;
  ChronoTimes(&tu, &ts);
  if (tu>ti)
    {
      fl_check_forms();
      ti = tu + 0.5;
    }
  if(STOP)
    {
      STOP = FALSE; 
      ti = 0;
      return(FALSE);
    }
  return(TRUE);
}

void fct_draw(void)
{
  if(G3D_DRAW_GRAPH) 
    g3d_draw_allwin_active();
} 


/* cree NB_NODES noeuds dans le graphe courant */
static void CB_global_search_obj(FL_OBJECT *ob, long arg)
{
  STOP = FALSE;
  
  MY_ALLOC_INFO("Avant la creation du graphe");
 
  p3d_learn(p3d_get_NB_NODES(),fct_stop, fct_draw);

  fl_ringbell(0);
  fl_set_button(SEARCH_GLOBAL_OBJ,0);
}


static void g3d_create_global_search_obj(void)
{ 
  SEARCH_GLOBAL_OBJ = fl_add_button(FL_PUSH_BUTTON,
          20.0,55.0,100.0,30.0,
            "Global Planner");
  fl_set_call_back(SEARCH_GLOBAL_OBJ,CB_global_search_obj,0);

}


// modif Juan
/***********************************************************/
/****************** random configurations ******************/
/***********************************************************/

// as CB_global_search_obj but noder are not connected
static void CB_random_confs_obj(FL_OBJECT *ob, long arg)
{
  STOP = FALSE;
  
  MY_ALLOC_INFO("Avant la creation du graphe");
  
  p3d_randconfs(p3d_get_NB_NODES(),fct_stop, fct_draw);

  fl_ringbell(0);
  fl_set_button(RANDCONFS_OBJ,0);
}

static void g3d_create_random_confs_obj(void)   // modif Juan
{ 
  RANDCONFS_OBJ = fl_add_button(FL_PUSH_BUTTON,
          70.0,20.0,50.0,30.0,
            "Random\nConfs.");
  fl_set_call_back(RANDCONFS_OBJ,CB_random_confs_obj,0);
}



/***********************************************************/
/***********************************************************/

static void CB_N_specific_input_obj(FL_OBJECT *ob, long arg)
{
  p3d_set_NB_specific(atoi(fl_get_input(N_SPECIFIC_OBJ)));
}

static void g3d_create_N_specific_obj(void)
{ 
  N_SPECIFIC_OBJ = fl_add_input(FL_NORMAL_INPUT,
          100.0,90.0,25.0,30.0,
            "");

  fl_set_call_back(N_SPECIFIC_OBJ,CB_N_specific_input_obj,0);
}

/***********************************************************/

/* relie deux configuration par une recherche aleatoire */
void CB_specific_search_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt qs, qg;
  int res = 0;
  int *iksols=NULL, *iksolg=NULL;  
  int i,j,imin=0,imax=0;
  double *arraytimes;
  int sumnnodes=0,sumnsamples=0,sumncallsCD=0,sumncallsLP=0;
  double mintime,maxtime,ttime,ttime2,avtime,coravtime;
  double std_dev,corstd_dev,add_diff;
  int nfail = 0;
  int count_cor;
  char *filePrefix;
  char c_dir_name[300];
  char fileGraph[300];
  char fileTraj[300];
  // For trajectory generation - copied from CB_test_lig_btn_obj
  int       it      = p3d_get_desc_number(P3D_TRAJ);
  int       ir      = p3d_get_desc_curnum(P3D_ROBOT); 
  // For writing PDB's
  double dep_dist;

  
  if( G3D_SAVE_MULT && arg != -999 ) {

  p3d_get_directory(c_dir_name);
  
  filePrefix = fl_show_simple_input("Directory to save files",c_dir_name);
  if( filePrefix == NULL ) {
    printf("Error: Directory to save trajectory files not specified.");
    filePrefix = "";
  }
  }


  /* test_brunet=-2; */
  
 STOP = FALSE;
   
 qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
 qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
 


 arraytimes = MY_ALLOC(double,p3d_get_NB_specific()); 

 /* on construit un graph ou qs et qg seront dans la meme composante connexe */
 MY_ALLOC_INFO("Avant la creation du graphe");
 if(p3d_get_NB_specific() > 1) {
   for(i=0; i<p3d_get_NB_specific(); i++) {
     p3d_set_rrt_stopped_by_weight(0);
     res = p3d_specific_learn(qs,qg,iksols,iksolg,fct_stop,fct_draw);
     if(!res) {
       if(p3d_get_rrt_stopped_by_weight()) {
   arraytimes[i] = robotPt->GRAPH->time;   
       }
       else {
   nfail++;
   arraytimes[i] = - (robotPt->GRAPH->time);
   //break;
       }
     }
     else {
       // keep time     
       arraytimes[i] = robotPt->GRAPH->time;
       // resets
       if(p3d_graph_to_traj(robotPt)) {
   //g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
   ////////// BIO /////////
#ifdef BIO
   if(bio_get_flag_evaluate_traj() == 1)
     bio_evaluate_traj();
#endif
   //////////
       }
     }
     sumnnodes += robotPt->GRAPH->nnode;
     sumnsamples += robotPt->GRAPH->nb_q;
     sumncallsCD += robotPt->GRAPH->nb_test_coll;
     sumncallsLP += robotPt->GRAPH->nb_local_call;   

     if( G3D_SAVE_MULT && arg != -999 ) {

  if(p3d_get_rrt_stopped_by_weight()) {
    //bio_loop_graph_statistics();
    bio_search_max_weight_in_curr_rrt();
    // To prevent infinite recursion, call this function using arg = -999 and check for this argument value before entering a G3D_SAVE_MULT code segment
    CB_specific_search_obj(ob,-999); 
  }
  if(p3d_get_desc_number(P3D_TRAJ) > it) {
    printf("\nSMOOTHING PATH ...\n");
    CB_start_optim_obj(ob,0);
    fl_set_button(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,1);
    CB_showtraj_obj(ROBOTS_FORM[ir].SHOWTRAJ_OBJ,0);
    // write a pdbfile to display the positions of the ligand's barycenter for all the nodes in the tree
    //bio_write_baryplot();
    // identify the exhausted nodes
    p3d_identify_exhausted_nodes();

    sprintf(fileTraj,"%s/%d.trj", filePrefix, i);
    printf("Saving .trj file <%s>.\n", fileTraj);
    void *traj = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
    printf(".\n", fileTraj);
    p3d_save_traj(fileTraj, traj);
    printf(".\n", fileTraj);
    
    sprintf(fileGraph,"%s/%d.graph", filePrefix, i);
    printf("Saving .graph file <%s>...\n", fileGraph);
    p3d_write_graph(XYZ_GRAPH,(char *)fileGraph);

    // Write PDB's
    #ifdef BIO
    dep_dist = 1.0;
    sprintf(fileGraph,"%s/%d/", filePrefix, i);
    printf("Saving .pdb files in directory <%s>...\n", fileGraph);
    mkdir(fileGraph, S_IRUSR | S_IWUSR | S_IXUSR);
    bio_write_pdbs_unifom_dep(robotPt, fileGraph,dep_dist, trajExpMinNONE);
    #endif

  }
  else{
    printf("Problem during trajectory extraction, not writing trajectory files.\n");
  }

     }

     if(p3d_get_NB_specific()>1)
       printf("\n#### END OF TEST NUM.%d ####\n\n",i+1);

     // prueba KK (Juan)
     //bio_loop_graph_statistics();
     ///////////////////
     if(i < (p3d_get_NB_specific() - 1)) {
       p3d_del_graph(robotPt->GRAPH); 
       if(p3d_get_RANDOM_CHOICE() == P3D_HALTON_SAMPLING) {
   p3d_init_random_seed(i);
       }
     }
   }
   // print average
   printf("\n## Computing time of %d tests ##\n", i);
   //if(i<30) {
   //  printf("Too small number of tests for making average\n");
   //}
   //else {
   mintime = P3D_HUGE;
   maxtime = -P3D_HUGE;
   ttime = 0;
   for(j=0; j<i; j++) {
     //if(arraytimes[j]>0) {
     if(fabs(arraytimes[j]) < mintime) {
   imin = j;
   mintime = fabs(arraytimes[j]);
     }
     if(fabs(arraytimes[j]) > maxtime) {
   imax = j;
   maxtime = fabs(arraytimes[j]);
     }    
     //}
   }
   for(j=0; j<i; j++) {
     if(arraytimes[j] > 0)
       printf("Time %d = %f s.\n",j+1,arraytimes[j]);
     else
       printf("Time %d = %f s. --  FAILED\n",j+1,-arraytimes[j]);
     //if(arraytimes[j]>0) {
     ttime += fabs(arraytimes[j]);
     //}
   }
   //avtime = ttime/((double) (i-nfail));
   avtime = ttime/((double) i);
   add_diff = 0.0;
   for(j=0; j<i; j++) {
     //if(arraytimes[j]>0)
     add_diff += SQR(fabs(arraytimes[j]) - avtime);
   }     
   //std_dev = sqrt((1.0/(((double)(i-nfail)) - 1.0)) * add_diff);
   //std_dev = sqrt((1.0/(((double)i) - 1.0)) * add_diff);
   std_dev = sqrt((1.0/((double)i)) * add_diff);

   // corrrected average
   count_cor = 0;
   ttime2 = 0.0;
   for(j=0; j<i; j++) {
     //if((arraytimes[j] > 0) && (arraytimes[j] > (avtime-std_dev)) && (arraytimes[j] < (avtime+std_dev))) {
     if((fabs(arraytimes[j]) > (avtime-std_dev)) && (fabs(arraytimes[j]) < (avtime+std_dev))) {
       ttime2 += fabs(arraytimes[j]);
       count_cor++;   
     }
   }
   coravtime = ttime2/((double) (count_cor));

   add_diff = 0.0;
   for(j=0; j<i; j++) {
     if((fabs(arraytimes[j]) > (avtime-std_dev)) && (fabs(arraytimes[j]) < (avtime+std_dev))) {
       add_diff += SQR(fabs(arraytimes[j]) - coravtime);
     }
   }     
   //corstd_dev = sqrt((1.0/(((double)(i-nfail)) - 1.0)) * add_diff);
   //corstd_dev = sqrt((1.0/(((double)i) - 1.0)) * add_diff);
   corstd_dev = sqrt((1.0/((double)i)) * add_diff);
   
   printf("\nAverage time      = %f s.\n",avtime);
   printf("Minimum time      = %f s.\n",mintime);
   printf("Maximum time      = %f s.\n",maxtime);     
   printf("Std. deviation    = %f\n",std_dev);
   printf("Corrected Average = %f s.\n",coravtime);
   printf("Corrected Std.Dev = %f\n",corstd_dev);
   printf("Num. fails        = %d\n",nfail);
   printf("Average Nnodes    = %f\n",(double)sumnnodes/(double)i);
   printf("Average Nsamples  = %f\n",(double)sumnsamples/(double)i);
   printf("Average NcallsCD  = %f\n",(double)sumncallsCD/(double)i);
   printf("Average NcallsLP  = %f\n",(double)sumncallsLP/(double)i);
   //}
 }
 else {
   res = p3d_specific_learn(qs,qg,iksols,iksolg,fct_stop,fct_draw);
 }

 p3d_destroy_config(robotPt, qs);
 p3d_destroy_config(robotPt, qg);


 g3d_draw_allwin_active();

 if(!res){
   printf("p3d_specific_planner : ECHEC : il n'existe pas de chemin\n");
 }
 else{
     /* on construit la trajectoire entre les points etapes */

   if(p3d_graph_to_traj(robotPt)) {
     g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
     

     ////////// BIO /////////
#ifdef BIO
     if(bio_get_flag_evaluate_traj() == 1)
       bio_evaluate_traj();
#endif
     ////////////////////////
   }
   else{
     printf("Problem during trajectory extraction\n");
     g3d_draw_allwin_active();
     MY_FREE(arraytimes,double,p3d_get_NB_specific());
     return;
   }
 }

 MY_FREE(arraytimes,double,p3d_get_NB_specific());
 fl_ringbell(0);
 fl_set_button(SEARCH_SPECIFIC_OBJ,0);
}


/* relie deux configuration par une recherche aleatoire */
void CB_specific_search_writing_path_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt qs, qg;
  int res = 0;
  int *iksols=NULL, *iksolg=NULL;  
  int i,j,imin=0,imax=0;
  double *arraytimes;
  double mintime,maxtime,ttime,avtime;
  double std_dev, add_diff;
  int nfail = 0;
  FILE *paths_file;
  
  /* test_brunet=-2; */
  
 STOP = FALSE;
   
 qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
 qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
 

 arraytimes = MY_ALLOC(double,p3d_get_NB_specific()); 

 if(!(paths_file=fopen("./test_paths.trjs","w"))) {
   printf("ERROR: can't open %s\n","./test_paths.trjs");
   return;
 }

 /* on construit un graph ou qs et qg seront dans la meme composante connexe */
 MY_ALLOC_INFO("Avant la creation du graphe");
 for(i=0; i<p3d_get_NB_specific(); i++) {
   if(i > 0) {
     p3d_del_graph(robotPt->GRAPH); 
     if(p3d_get_RANDOM_CHOICE() == P3D_HALTON_SAMPLING) {
       p3d_init_random_seed(i);
     }
   }

   res = p3d_specific_learn(qs,qg,iksols,iksolg,fct_stop,fct_draw);
   if(!res) {
     nfail++;
     arraytimes[i] = -1;
     printf("p3d_specific_planner : ECHEC : il n'existe pas de chemin\n");
     //break;
   }
   else {
     // keep time     
     arraytimes[i] = robotPt->GRAPH->time;
     // resets
     if(p3d_graph_to_traj(robotPt)) {
       g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
       g3d_draw_allwin_active();
       //save path
       fprintf(paths_file, "\n\nPATH NUM.%d\n",i);
       save_trajectory_without_header(paths_file,p3d_get_desc_curid(P3D_TRAJ));       
       ////////// BIO /////////
#ifdef BIO
       if(bio_get_flag_evaluate_traj() == 1)
   bio_evaluate_traj();
#endif
       //////////
     }
     else{
       printf("Problem during trajectory extraction\n");
       g3d_draw_allwin_active();
       MY_FREE(arraytimes,double,p3d_get_NB_specific());
       fclose(paths_file);
       return;
     }
   }
 }

 // print average
 printf("\n## Computing time of %d tests ##\n", i);
 //if(i<30) {
 //  printf("Too small number of tests for making average\n");
 //}
 //else {
 mintime = P3D_HUGE;
 maxtime = -P3D_HUGE;
 ttime = 0;
 for(j=0; j<i; j++) {
   if(arraytimes[j]>0) {
     if(arraytimes[j] < mintime) {
       imin = j;
       mintime = arraytimes[j];
     }
     if(arraytimes[j] > maxtime) {
       imax = j;
       maxtime = arraytimes[j];
     }    
   }
 }
 for(j=0; j<i; j++) {
   printf("Time %d = %f s.\n",j+1,arraytimes[j]);
   if(arraytimes[j]>0) {
     ttime += arraytimes[j];
   }
 }
 //avtime = ttime/((double) i - 2.0);
 avtime = ttime/((double) (i-nfail));
 add_diff = 0.0;
 for(j=0; j<i; j++) {
     if(arraytimes[j]>0)
       add_diff += SQR(arraytimes[j] - avtime);
 }     
 std_dev = sqrt((1.0/(((double)(i-nfail)) - 1.0)) * add_diff);
 
 printf("\nAverage time   = %f s.\n",avtime);
 printf("Minimum time   = %f s.\n",mintime);
 printf("Maximum time   = %f s.\n",maxtime);     
 printf("Std. deviation = %f\n",std_dev);
 printf("Num. fails     = %d\n",nfail);


 p3d_destroy_config(robotPt, qs);
 p3d_destroy_config(robotPt, qg);



 fclose(paths_file);
 MY_FREE(arraytimes,double,p3d_get_NB_specific());
 fl_ringbell(0);
 fl_set_button(SEARCH_SPECIFIC_OBJ,0);
}



static void g3d_create_specific_search_obj(void)
{ 
  SEARCH_SPECIFIC_OBJ = fl_add_button(FL_PUSH_BUTTON,
          20.0,90.0,80.0,30.0,
            "Specific Planner");
  fl_set_call_back(SEARCH_SPECIFIC_OBJ,CB_specific_search_obj,0);

}

static void CB_optim_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);
  if(val) 
    fl_show_form(OPTIM_FORM,FL_PLACE_SIZE,TRUE,"Optimization");
  else    
    fl_hide_form(OPTIM_FORM);
}

static void g3d_create_optim_obj(void)
{
  OPTIM_OBJ = fl_add_button(FL_PUSH_BUTTON, 250.0,20.0,100.0,30.0, "Optimize");
  fl_set_call_back(OPTIM_OBJ,CB_optim_obj,0);
}

static FL_OBJECT  *INPUT_OBJ;
static FL_FORM *INPUT_FORM;
static void CB_global_input_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_NB_NODES(atoi(fl_get_input(INPUT_OBJ)));
   fl_hide_form(INPUT_FORM);
   fl_freeze_form(PLANNER_FORM);
   fl_set_slider_value(SEARCH_GLOBAL_PARAM_OBJ,p3d_get_NB_NODES());
   fl_unfreeze_form(PLANNER_FORM);
   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}


static void CB_cancel(FL_OBJECT *ob, long arg)
{
   fl_hide_form(INPUT_FORM);
   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}  
  
static void CB_create_global_input_obj(FL_OBJECT *ob, long arg)
{FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Nodes");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Annuler");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_global_input_obj,0);
  
  fl_set_button(SEARCH_GLOBAL_INPUT_OBJ,0);
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
}


static void g3d_create_global_input_obj(void)
{
  SEARCH_GLOBAL_INPUT_OBJ = fl_add_button(FL_PUSH_BUTTON,65.0,285.0,50.0,30.0,"Nb\nnode");
  fl_set_object_callback(SEARCH_GLOBAL_INPUT_OBJ,CB_create_global_input_obj,0);
}  

static void CB_global_param_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_NB_NODES((int)floor(fl_get_slider_value(SEARCH_GLOBAL_PARAM_OBJ)));
}
  
  
static void g3d_create_global_param_obj(void)
{
  SEARCH_GLOBAL_PARAM_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,285.0,245.0,30.0,"");
  fl_set_slider_step(SEARCH_GLOBAL_PARAM_OBJ,1.0);
  fl_set_slider_bounds(SEARCH_GLOBAL_PARAM_OBJ,1,100000);   // modif Juan  
  fl_set_slider_value(SEARCH_GLOBAL_PARAM_OBJ,p3d_get_NB_NODES());
  fl_set_object_callback(SEARCH_GLOBAL_PARAM_OBJ,CB_global_param_obj,0);
}

/*******************************************************************/

/* detruit le graphe courant */
void CB_del_param_obj(FL_OBJECT *ob, long arg)
{
  p3d_del_graph(XYZ_GRAPH);
  p3d_reinit_array_exhausted_nodes();
  p3d_reinit_array_farther_nodes();
  //  XYZ_GRAPH = NULL; Modif Fabien
  MY_ALLOC_INFO("Apres destruction du graphe");
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DEL_PARAM_OBJ,0);   
}


static void g3d_create_del_param_obj(void)
{
  SEARCH_DEL_PARAM_OBJ = fl_add_button(FL_PUSH_BUTTON,
          10.0,320.0,50.0,30.0,
            "Reset\ngraph");
  fl_set_call_back(SEARCH_DEL_PARAM_OBJ,CB_del_param_obj,0);
}

/*************************************************************/

/* permet de modifier le parametre DMAX */
void  CB_DMAX_param_obj(FL_OBJECT *ob, long arg)
{double val = fl_get_slider_value(ob);

 p3d_set_DMAX(val); 
}

static void g3d_create_DMAX_param_obj(void)
{void  CB_DMAX_param_obj(FL_OBJECT *ob, long arg);
 p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 double vmin,vmax;
 

 p3d_set_DMAX(p3d_calc_DMAX(r));
 vmin = p3d_get_DMAX()/1000.;
 vmax = p3d_get_DMAX()*5.;
 fl_add_box(FL_UP_BOX,65.0,250.0,50.0,30.0,"Distance\nmax.");
 SEARCH_DMAX_PARAM_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,250.0,245.0,30.0,"");
 fl_set_slider_bounds(SEARCH_DMAX_PARAM_OBJ,vmin,vmax);
 fl_set_slider_value(SEARCH_DMAX_PARAM_OBJ,p3d_get_DMAX());
 fl_set_call_back(SEARCH_DMAX_PARAM_OBJ,CB_DMAX_param_obj,0);
}

/***********************************************************/

/* permet de tracer ou non le graphe courant */
static void CB_draw_obj(FL_OBJECT *ob, long arg)
{

  G3D_DRAW_GRAPH = !G3D_DRAW_GRAPH;
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DRAW_OBJ,G3D_DRAW_GRAPH);   
}

// modif Juan
static void g3d_create_draw_obj(void)
{
  SEARCH_DRAW_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,135,30,20,30,"");
  fl_set_object_color(SEARCH_DRAW_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_DRAW_OBJ,CB_draw_obj,0);
}

/***********************************************************/
// allows to choice the jnt which frame is drawn in the graph

static int user_drawnjnt = -1;

int p3d_get_user_drawnjnt(void)
{
  return(user_drawnjnt);
}

static void CB_drawnjnt_obj(FL_OBJECT *ob, long arg)
{
   user_drawnjnt = (atoi(fl_get_input(INPUT_OBJ)));

   fl_hide_form(INPUT_FORM);
   
   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}

void CB_create_drawnjnt_obj(FL_OBJECT *ob, long arg)
{FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Index\nJNT");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Annuler");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_drawnjnt_obj,0);
  
  fl_set_button(DRAWNJNT_OBJ,0);
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
}


static void g3d_create_drawnjnt_obj(void)
{
  DRAWNJNT_OBJ = fl_add_button(FL_PUSH_BUTTON,165,32,55,25,"Draw graph");
  fl_set_object_callback(DRAWNJNT_OBJ,CB_create_drawnjnt_obj,0); 
}

/***********************************************************/
// fmodif Juan


/* permet d'imprimer ou non des infos sur le graphe courant */
static void CB_print_obj(FL_OBJECT *ob, long arg)
{p3d_graph *G = XYZ_GRAPH;
 p3d_compco *comp;
 p3d_node *Ncomp=NULL;
 p3d_list_edge *edge;
 p3d_list_node *list_node;
 

  if(XYZ_GRAPH) {
    printf("Composantes connexes :\n");
    comp = G->comp;
    while(comp != NULL) {
      list_node = comp->dist_nodes;
      printf("%d ; nombre de noeuds  %d : ",comp->num,comp->nnode);
      while(list_node != NULL) {
  Ncomp = list_node->N;
  printf(" %d(%d",Ncomp->num,Ncomp->nedge);
  if(Ncomp->nedge>0){
    printf(" :");
    edge = Ncomp->edges;
    while(edge != NULL) {
      printf(" [%d %d]",edge->E->Ni->num,edge->E->Nf->num); 
      edge = edge->next;
    }
  }
  printf(")");
  list_node = list_node->next;
      }
      printf("\n");
      comp=comp->suiv;
    }
  }
 
  printf("on met le bouton a 0\n");
  fl_set_button(SEARCH_PRINT_OBJ,0);   
}


static void g3d_create_print_obj(void)
{
  SEARCH_PRINT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,135,55,55,30,"Print graph"); 
  fl_set_object_color(SEARCH_PRINT_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_PRINT_OBJ,CB_print_obj,0);
}

/************************************************************/

/* permet de modifier le parametre COMP_NODES */
static void CB_compco_input_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_COMP_NODES(atoi(fl_get_input(INPUT_OBJ)));
   fl_hide_form(INPUT_FORM);
   
   fl_freeze_form(PLANNER_FORM);
   fl_set_slider_value(SEARCH_COMPCO_PARAM_OBJ,p3d_get_COMP_NODES());
   fl_unfreeze_form(PLANNER_FORM);

   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}

static void CB_create_compco_input_obj(FL_OBJECT *ob, long arg)
{FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Nodes");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Annuler");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_compco_input_obj,0);
  
  fl_set_button(SEARCH_COMPCO_INPUT_OBJ,0);
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_compco_input_obj(void)
{
  SEARCH_COMPCO_INPUT_OBJ = fl_add_button(FL_PUSH_BUTTON,65.0,320.0,50.0,30.0,"Nb node\n/Comp.co.");
  fl_set_object_callback(SEARCH_COMPCO_INPUT_OBJ,CB_create_compco_input_obj,0);
}  

static void CB_compco_param_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_COMP_NODES(floor(fl_get_slider_value(SEARCH_COMPCO_PARAM_OBJ)));
}
  
  
static void g3d_create_compco_param_obj(void)
{
  SEARCH_COMPCO_PARAM_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,320.0,245.0,30.0,"");
  fl_set_slider_step(SEARCH_COMPCO_PARAM_OBJ,1.0);
  fl_set_slider_bounds(SEARCH_COMPCO_PARAM_OBJ,1,100000);   // modif Juan
  fl_set_slider_value(SEARCH_COMPCO_PARAM_OBJ,p3d_get_COMP_NODES());
  fl_set_object_callback(SEARCH_COMPCO_PARAM_OBJ,CB_compco_param_obj,0);
}

/************************************************************/


/* permet de tracer la courbe a optimiser */
static void CB_draw_optim_obj(FL_OBJECT *ob, long arg)
{
  G3D_DRAW_TRAJ = !G3D_DRAW_TRAJ;
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DRAW_OPTIM_OBJ,G3D_DRAW_TRAJ);  
}


static void g3d_create_draw_optim_obj(void)
{

  SEARCH_DRAW_OPTIM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,135,70,55,30,"Draw trajectory"); 
  fl_set_object_color(SEARCH_DRAW_OPTIM_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_DRAW_OPTIM_OBJ,CB_draw_optim_obj,0);
}

/***************************************************************/

static void CB_save_mult_obj(FL_OBJECT *ob, long arg)
{
  G3D_SAVE_MULT = !G3D_SAVE_MULT;
  g3d_draw_allwin_active();
  fl_set_button(SEARCH_DRAW_OPTIM_OBJ,G3D_SAVE_MULT);  
}


static void g3d_create_save_mult_obj(void)
{

  SEARCH_DRAW_OPTIM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,135,85,55,30,"Save Traj."); 
  fl_set_object_color(SEARCH_DRAW_OPTIM_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SEARCH_DRAW_OPTIM_OBJ,CB_save_mult_obj,0);
}

/***************************************************************/

/* permet de sauvegarder le graphe courant */
static void CB_save_graph_obj(FL_OBJECT *ob, long arg)
{char TERM[20],DEF_FILE[200],c_dir_name[300];
 char *env_name; 
 const char *file;
 int c_sz=0, i;
 pp3d_rob robotPt;
  
 if(!XYZ_GRAPH)
   { printf("save_graph : ERREUR : no current graph...\n"); }
 else { /* D�ut modification Fabien */
   if ((XYZ_GRAPH == NULL) || (XYZ_GRAPH->file == NULL)) { 
     env_name = p3d_get_desc_curname(P3D_ENV);
     robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
     c_sz = (int) strlen(c_dir_name); 
     if(c_dir_name[c_sz-1] != '/'){strcat(c_dir_name,"/");} 
     strcat(c_dir_name,"GRAPH");
   } else { 
     strcpy(c_dir_name, XYZ_GRAPH->file);
     c_sz = (int) strlen(c_dir_name); 
     /* On s�are le nom du fichier de celui du r�ertoire  */
     for(i=c_sz-1; (i>=0) && (c_dir_name[i]!='/'); i--);
     strcpy(DEF_FILE, c_dir_name + i + 1);
     c_dir_name[i+1] = '\0';
   }
   /* Fin modification Fabien */
   file =fl_show_fselector("P3D_GRAPH filename",c_dir_name,"*.graph",DEF_FILE);

   if(file){p3d_write_graph(XYZ_GRAPH,(char *)file);}
 }
 fl_set_button(SAVE_GRAPH_OBJ,0);
}

static void g3d_create_save_graph_obj(void)
{
  SAVE_GRAPH_OBJ = fl_add_button(FL_PUSH_BUTTON,10.0,285.0,50.0,30.0,"Save\nGraph");
  fl_set_object_callback(SAVE_GRAPH_OBJ,CB_save_graph_obj,0);
}  

/* permet de charger un grave sauve dans le graphe courant */
static void CB_load_graph_obj(FL_OBJECT *ob, long arg)
{char TERM[20],DEF_FILE[200],c_dir_name[200];
 char *env_name; 
 const char *file;
 int  c_sz=0, i; 
 pp3d_rob robotPt;

 if(XYZ_GRAPH){printf("load_graph : ATTENTION : le graphe courant sera ecrase...\n");}

 /* D�ut modification Fabien */
 // p3d_del_graph(XYZ_GRAPH); 
 // XYZ_GRAPH = NULL;         

 if ((XYZ_GRAPH == NULL) || (XYZ_GRAPH->file == NULL)) { 
   env_name = p3d_get_desc_curname(P3D_ENV);
   robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
   strcpy(DEF_FILE,env_name);
   strcat(DEF_FILE, "."); 
   strcat(DEF_FILE, robotPt->name);
   strcpy(TERM,".graph");
   strcat(DEF_FILE,TERM); 
   
   p3d_get_directory(c_dir_name);
   c_sz = (int) strlen(c_dir_name); 
   if(c_dir_name[c_sz-1] != '/'){strcat(c_dir_name,"/");} 
   strcat(c_dir_name,"GRAPH");
 } else { 
   strcpy(c_dir_name, XYZ_GRAPH->file);
   c_sz = (int) strlen(c_dir_name); 
   /* On s�are le nom du fichier de celui du r�ertoire  */
   for(i=c_sz-1; (i>=0) && (c_dir_name[i]!='/'); i--);
   strcpy(DEF_FILE, c_dir_name + i + 1);
   c_dir_name[i+1] = '\0';
 }
 /* Fin modification Fabien */

  file = fl_show_fselector("P3D_GRAPH filename",c_dir_name,"*.graph",DEF_FILE);

  if(file){p3d_read_graph((char *) file);} // Modif Fabien

  g3d_draw_allwin_active();

   MY_ALLOC_INFO("Apres lecture du graphe");

  fl_set_button(LOAD_GRAPH_OBJ,0);
}

static void g3d_create_load_graph_obj(void)
{
  LOAD_GRAPH_OBJ = fl_add_button(FL_PUSH_BUTTON,10.0,250.0,50.0,30.0,"Load\nGraph");
  fl_set_object_callback(LOAD_GRAPH_OBJ,CB_load_graph_obj,0);
} 

/******************************************************************/

/* permet de stoper la creation de noeuds dans le noeud courant */
void CB_stop_obj(FL_OBJECT *ob, long arg)
{
  STOP = TRUE;
  fl_set_button(STOP_OBJ,0);
}

static void g3d_create_stop_obj(void)
{
  STOP_OBJ = fl_add_button(FL_PUSH_BUTTON,315.0,395.0,45.0,30.0,"Stop");
  fl_set_object_callback(STOP_OBJ,CB_stop_obj,0);
}  

/*********************************************************/

/* cree le menu d'expansion des noeuds */
static void CB_expand_obj(FL_OBJECT *ob, long arg)
{
  FL_FORM *EXPAND_MENU;
  FL_OBJECT *obj, *ok, *cancel, *choice1, *choice2, *node, 
    *nbnode, *glob, *nbwalk;
  int NNODE=0,i;
  p3d_list_node *list_node;
  p3d_node *Nex=NULL;
  
  STOP = FALSE;
  
  EXPAND_MENU = fl_bgn_form(FL_FLAT_BOX,230.0,145.0);
  
  obj = fl_add_box(FL_UP_BOX,0.0,0.0,230.0,145.0,"");
  
  obj = fl_add_frame(FL_ENGRAVED_FRAME,110,20,110,60,""); 
  obj = fl_add_box(FL_FLAT_BOX,125,15,80,10,"Expand type");

      choice1 = fl_add_checkbutton(FL_RADIO_BUTTON,120,30,55,30,"Centered box");
      fl_set_object_color(choice1,FL_MCOL,FL_GREEN);
      fl_set_button(choice1,1);
      choice2 = fl_add_checkbutton(FL_RADIO_BUTTON,120,50,55,30,"Random walk");
      fl_set_object_color(choice2,FL_MCOL,FL_GREEN);

      node = fl_add_input(FL_NORMAL_INPUT,50.0,10.0,40.0,20.0,
                                "No node");
      fl_set_input_color(node, 0, 0);
      
      nbnode = fl_add_input(FL_NORMAL_INPUT,50.0,40.0,40.0,20.0,
                                "Nb node");
      fl_set_input_color(nbnode, 0, 0);

      nbwalk = fl_add_input(FL_NORMAL_INPUT,50.0,70.0,40.0,20.0,
                                "Nb walk");
      fl_set_input_color(nbnode, 0, 0);


      ok = fl_add_button(FL_PUSH_BUTTON, 10,110,40,20,"OK");
      cancel = fl_add_button(FL_PUSH_BUTTON, 60,110,40,20,"Annuler");

      glob = fl_add_button(FL_PUSH_BUTTON, 140,110,80,20,"Global expand");

      fl_end_form();

      fl_show_form(EXPAND_MENU,FL_PLACE_SIZE,TRUE,"Expand");
      while (1){
       obj = fl_do_forms();
       if (obj == ok){
   
   fl_hide_form(EXPAND_MENU);

   NNODE = atoi(fl_get_input(node));
   p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));

   if(NNODE>XYZ_GRAPH->nnode){printf("PAS ASSEZ DE NOEUDS !\n");return;}
   list_node = XYZ_GRAPH->nodes;
   for(i=1;i<=XYZ_GRAPH->nnode;i++){
     if(list_node->N->num == NNODE){Nex = list_node->N;break;}
     else{list_node = list_node->next;}
   }
   
   p3d_APInode_expand(XYZ_GRAPH,Nex,fct_stop,fct_draw);
       
   fl_set_button(EXPAND_OBJ,0);
   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(choice1);
   fl_free_object(choice2);
   fl_free_object(node);
   fl_free_object(nbnode);
   fl_free_object(nbwalk);
   fl_free_object(glob);
   fl_free_form(EXPAND_MENU);
   return;
       }
       if (obj == glob){
   
   fl_hide_form(EXPAND_MENU);

   p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));

   p3d_expand_graph(XYZ_GRAPH,0.1,fct_stop,fct_draw);
       
   fl_set_button(EXPAND_OBJ,0);
   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(choice1);
   fl_free_object(choice2);
   fl_free_object(node);
   fl_free_object(nbnode);
   fl_free_object(nbwalk);
   fl_free_object(glob);
   fl_free_form(EXPAND_MENU);
   return;
       }

       if (obj == cancel){
   fl_hide_form(EXPAND_MENU);
   fl_set_button(EXPAND_OBJ,0);
   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(choice1);
   fl_free_object(choice2);
   fl_free_object(nbnode);
   fl_free_object(nbwalk);
   fl_free_object(node);
   fl_free_object(glob);
   fl_free_form(EXPAND_MENU);
   return;
       }
       if (obj == choice1){
   p3d_set_EXPAND_CHOICE(P3D_EXPAND_BOX);
       }
       if (obj == choice2){
   p3d_set_EXPAND_CHOICE(P3D_EXPAND_RAND_WALK);
       }
     }
}

static void g3d_create_expand_obj(void)
{
  EXPAND_OBJ = fl_add_button(FL_PUSH_BUTTON,
          65.0,395.0,45.0,30.0,
            "Expand\nnode");
  fl_set_call_back(EXPAND_OBJ,CB_expand_obj,0);
}

/*****************************************************************/

/* lance la creation de NB_NODES noeuds isoles */
static void CB_isolate_obj(FL_OBJECT *ob, long arg)
{
  STOP = FALSE;
  
  p3d_create_orphans(p3d_get_NB_NODES(),fct_stop,fct_draw);

  fl_set_button(ISOLATE_OBJ,0);
}


static void g3d_create_isolate_obj(void)
{ 
  ISOLATE_OBJ = fl_add_button(FL_PUSH_BUTTON,
          210.0,395.0,45.0,30.0,
            "Create\nisolated");
  fl_set_call_back(ISOLATE_OBJ,CB_isolate_obj,0);

}

/* lance la creation de NB_NODES noeuds "liants" */
static void CB_linking_obj(FL_OBJECT *ob, long arg)
{
  STOP = FALSE;
  
  p3d_create_linking(p3d_get_NB_NODES(),fct_stop,fct_draw);

  fl_set_button(LINKING_OBJ,0);
}

static void g3d_create_linking_obj(void)
{ 
  LINKING_OBJ = fl_add_button(FL_PUSH_BUTTON,
          260.0,395.0,45.0,30.0,
            "Create\nlinking");
  fl_set_call_back(LINKING_OBJ,CB_linking_obj,0);

}



/* lance le test d'un graphe */
static void CB_test_obj(FL_OBJECT *ob, long arg)
{FL_FORM *TEST_MENU;
 FL_OBJECT *obj, *ok, *cancel, *choice1, *choice2, *nbtest;
 int NBTEST,TEST_CHOICE=1;
 double success=0.;

     STOP = FALSE;

     TEST_MENU = fl_bgn_form(FL_FLAT_BOX,230.0,110.0);
     
     obj = fl_add_box(FL_UP_BOX,0.0,0.0,230.0,110.0,"");

      obj = fl_add_frame(FL_ENGRAVED_FRAME,110,20,110,80,""); 
      obj = fl_add_box(FL_FLAT_BOX,125,15,80,10,"Test type");

      choice1 = fl_add_checkbutton(FL_RADIO_BUTTON,120,30,55,30,"Test connexity");
      fl_set_object_color(choice1,FL_MCOL,FL_GREEN);
      fl_set_button(choice1,1);
      choice2 = fl_add_checkbutton(FL_RADIO_BUTTON,120,50,55,30,"Test covering");
      fl_set_object_color(choice2,FL_MCOL,FL_GREEN);

      nbtest = fl_add_input(FL_NORMAL_INPUT,50.0,20.0,40.0,20.0,
                                "Nb test");
      fl_set_input_color(nbtest, 0, 0);

      ok = fl_add_button(FL_PUSH_BUTTON, 10,80,40,20,"OK");
      cancel = fl_add_button(FL_PUSH_BUTTON, 60,80,40,20,"Annuler");

      fl_end_form();

      fl_show_form(TEST_MENU,FL_PLACE_SIZE,TRUE,"Test");
      while (1){
       obj = fl_do_forms();
       if (obj == ok){

   if(!XYZ_GRAPH){
     printf("test : ERREUR : pas de graphe courant\n"); 
     fl_hide_form(TEST_MENU);
     fl_set_button(TEST_OBJ,0);
     fl_free_object(cancel);
     fl_free_object(ok);
     fl_free_object(choice1);
     fl_free_object(choice2);
     fl_free_object(nbtest);
     fl_free_form(TEST_MENU);
     return;
   }

   fl_hide_form(TEST_MENU);

   NBTEST = atoi(fl_get_input(nbtest));

   switch(TEST_CHOICE){
   case 1: 
     success = p3d_test_quality_graph(XYZ_GRAPH,NBTEST);
     break;

   case 2:
     success = p3d_test_good_graph(XYZ_GRAPH,NBTEST);
     break;

   default:
     printf("WRONG TYPE OF TEST\n");
     break;
   }

   printf("Taux de reussite : %f\n",100*success);
       
   fl_set_button(TEST_OBJ,0);
   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(choice1);
   fl_free_object(choice2);
   fl_free_object(nbtest);
   fl_free_form(TEST_MENU);
   return;
       }

       if (obj == cancel){
   fl_hide_form(TEST_MENU);
   fl_set_button(TEST_OBJ,0);
   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(choice1);
   fl_free_object(choice2);
   fl_free_object(nbtest);
   fl_free_form(TEST_MENU);
   return;
       }
       if (obj == choice1){
   TEST_CHOICE = 1;
       }
       if (obj == choice2){
   TEST_CHOICE = 2;
       }

     }
}

static void g3d_create_test_obj(void)
{ 
  TEST_OBJ = fl_add_button(FL_PUSH_BUTTON,
          10.0,355.0,50.0,30.0,
            "Test\ngraph");

  fl_set_call_back(TEST_OBJ,CB_test_obj,0);
}


/***************************************************************************************/

static p3d_node *lastpinpointedN = NULL;

static void p3d_pinpoint_node(p3d_node *N, p3d_graph *G)
{
  if(lastpinpointedN != NULL) {
    lastpinpointedN->pinpointed = 0;
  }
  N->pinpointed = 1;
  lastpinpointedN = N;

  if(!G3D_DRAW_GRAPH) {
    p3d_set_and_update_this_robot_conf_without_cntrt(G->rob,N->q);
    p3d_copy_config_into(G->rob, N->q, &(G->rob->ROBOT_GOTO));  
    printf("\nROBOT_GOTO has been updated with the configuration of the pinpointed node\n");
  }
}


/* cree le menu de selection d'un noeuds */
static void CB_pinpoint_node_obj(FL_OBJECT *ob, long arg)
{FL_FORM *PINPOINT_NODE_MENU;
 FL_OBJECT *obj, *ok, *cancel,*node;
 int NNODE=0,i;
 p3d_list_node *list_node;
 p3d_node *Nex=NULL;


 PINPOINT_NODE_MENU = fl_bgn_form(FL_FLAT_BOX,110.0,70.0);
     
 obj = fl_add_box(FL_UP_BOX,0.0,0.0,230.0,145.0,"");

 node = fl_add_input(FL_NORMAL_INPUT,50.0,10.0,40.0,20.0,
                                "No node");
 fl_set_input_color(node, 0, 0);
      
 ok = fl_add_button(FL_PUSH_BUTTON, 10,40,40,20,"OK");
 cancel = fl_add_button(FL_PUSH_BUTTON, 60,40,40,20,"Annuler");

 fl_end_form();

 fl_show_form(PINPOINT_NODE_MENU,FL_PLACE_SIZE,TRUE,"Pinpoint Node");
 while (1){
   obj = fl_do_forms();
   if (obj == ok){
     
     fl_hide_form(PINPOINT_NODE_MENU);

     NNODE = atoi(fl_get_input(node));
     
     if(NNODE>XYZ_GRAPH->nnode){printf("Wrong node number !\n");return;}
     Nex=NULL;
     list_node = XYZ_GRAPH->nodes;
     for(i=1;i<=XYZ_GRAPH->nnode;i++){
       if(list_node->N->num == NNODE){Nex = list_node->N;break;}
       else{list_node = list_node->next;}
     }
     if(Nex == NULL){printf("ERROR : no node corresponding to number\n");}
     else{p3d_pinpoint_node(Nex,XYZ_GRAPH);}
   
     fl_set_button(PINPOINT_NODE_OBJ,0);
   
     g3d_draw_allwin_active();
     
     fl_free_object(cancel);
     fl_free_object(ok);
     fl_free_object(node);
     
     fl_free_form(PINPOINT_NODE_MENU);
     return;
   }
   if (obj == cancel){
     fl_hide_form(PINPOINT_NODE_MENU);
     fl_set_button(PINPOINT_NODE_OBJ,0);
     fl_free_object(cancel);
     fl_free_object(ok);
     fl_free_object(node);
     fl_free_form(PINPOINT_NODE_MENU);
     return;
   }
 }
}

static void g3d_create_pinpoint_node_obj(void)
{
  PINPOINT_NODE_OBJ = fl_add_button(FL_PUSH_BUTTON,
            10.0,395.0,45.0,30.0,
            "Pinpoint\nnode");
  fl_set_call_back(PINPOINT_NODE_OBJ,CB_pinpoint_node_obj,0);
}

/***************************************************************************************/

/* cree le menu de destruction d'un noeuds */
static void CB_del_node_obj(FL_OBJECT *ob, long arg)
{FL_FORM *DEL_NODE_MENU;
 FL_OBJECT *obj, *ok, *cancel,*node;
 int NNODE=0,i;
 p3d_list_node *list_node;
 p3d_node *Nex=NULL;


     DEL_NODE_MENU = fl_bgn_form(FL_FLAT_BOX,110.0,70.0);
     
     obj = fl_add_box(FL_UP_BOX,0.0,0.0,230.0,145.0,"");

      node = fl_add_input(FL_NORMAL_INPUT,50.0,10.0,40.0,20.0,
                                "No node");
      fl_set_input_color(node, 0, 0);
      
      ok = fl_add_button(FL_PUSH_BUTTON, 10,40,40,20,"OK");
      cancel = fl_add_button(FL_PUSH_BUTTON, 60,40,40,20,"Annuler");

      fl_end_form();

      fl_show_form(DEL_NODE_MENU,FL_PLACE_SIZE,TRUE,"Delete Node");
      while (1){
       obj = fl_do_forms();
       if (obj == ok){
   
   fl_hide_form(DEL_NODE_MENU);

   NNODE = atoi(fl_get_input(node));

   if(NNODE>XYZ_GRAPH->nnode){printf("PAS ASSEZ DE NOEUDS !\n");return;}
   Nex=NULL;
   list_node = XYZ_GRAPH->nodes;
   for(i=1;i<=XYZ_GRAPH->nnode;i++){
     if(list_node->N->num == NNODE){Nex = list_node->N;break;}
     else{list_node = list_node->next;}
   }
   if(Nex == NULL){printf("ERREUR : pas de noeud\n");}
   else{p3d_del_node(Nex,XYZ_GRAPH);}
   
   fl_set_button(DEL_NODE_OBJ,0);
   
   g3d_draw_allwin_active();

   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(node);
   
   fl_free_form(DEL_NODE_MENU);
   return;
       }
       if (obj == cancel){
   fl_hide_form(DEL_NODE_MENU);
   fl_set_button(DEL_NODE_OBJ,0);
   fl_free_object(cancel);
   fl_free_object(ok);
   fl_free_object(node);
   fl_free_form(DEL_NODE_MENU);
   return;
       }
     }
}

static void g3d_create_del_node_obj(void)
{
  DEL_NODE_OBJ = fl_add_button(FL_PUSH_BUTTON,
          110.0,395.0,45.0,30.0,
            "Delete\nnode");
  fl_set_call_back(DEL_NODE_OBJ,CB_del_node_obj,0);
}



/* choix de la strategie globale */
static void CB_plan_strategy_obj(FL_OBJECT *ob, long arg)
{
  if(arg == 0){p3d_set_MOTION_PLANNER(P3D_BASIC);}
  if(arg == 1){p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);}
  if(arg == 2){p3d_set_MOTION_PLANNER(P3D_RRT);}
  if(arg == 3){p3d_set_MOTION_PLANNER(P3D_RRT2);}
  if(arg == 4){p3d_set_MOTION_PLANNER(P3D_LIGAND_RRT);}
  if(arg == 5){p3d_set_MOTION_PLANNER(P3D_FILTER_RRT);}
}  

static void g3d_create_plan_strategy_obj(void)
{FL_OBJECT *obj;
 
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,130,330,30,""); 
  obj = fl_add_box(FL_FLAT_BOX,145,125,80,10,"Planning Strategy");

  GROUP2 = fl_bgn_group();
  STRAT1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,17,130,50,30,"PRM");
  fl_set_object_color(STRAT1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STRAT1_OBJ,CB_plan_strategy_obj,0);
  STRAT2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,65,130,50,30,"VisPRM");
  fl_set_object_color(STRAT2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STRAT2_OBJ,CB_plan_strategy_obj,1);
  STRAT3_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,121,130,50,30,"RRT");
  fl_set_object_color(STRAT3_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STRAT3_OBJ,CB_plan_strategy_obj,2);
  STRAT4_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,169,130,50,30,"bi-RRT");
  fl_set_object_color(STRAT4_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STRAT4_OBJ,CB_plan_strategy_obj,3);
  STRAT5_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,225,130,50,30,"ligRRT");
  fl_set_object_color(STRAT5_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STRAT5_OBJ,CB_plan_strategy_obj,4);
  STRAT6_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,285,130,50,30,"filtRRT");
  fl_set_object_color(STRAT6_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STRAT6_OBJ,CB_plan_strategy_obj,5);
  GROUP2 = fl_end_group();
  fl_set_button(STRAT3_OBJ,1);
}

/* choix de la strategie globale */
static void CB_sorting_obj(FL_OBJECT *ob, long arg)
{
  if(arg == 0) {
    p3d_set_SORTING(P3D_NB_CONNECT);
    p3d_order_list_node_nofconnex();
  }
  if(arg == 1) {
    p3d_set_SORTING(P3D_DIST_NODE);
  }
}  
  





static void g3d_create_sorting_obj(void)
{FL_OBJECT *obj;
 
  obj = fl_add_frame(FL_ENGRAVED_FRAME,105,170,150,30,""); 
  obj = fl_add_box(FL_FLAT_BOX,140,165,90,10,"Connexion Strategy");

  GROUP3 = fl_bgn_group();
  SORT1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,105,167,50,35,"Nb edges");
  fl_set_object_color(SORT1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SORT1_OBJ,CB_sorting_obj,0);
  SORT2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,180,167,50,35,"Distance");
  fl_set_object_color(SORT2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SORT2_OBJ,CB_sorting_obj,1);
  GROUP3 = fl_end_group();
  fl_set_button(SORT2_OBJ,1);
}
static void  CB_sampling_obj(FL_OBJECT *ob, long arg) {
  switch(arg) {  
  case 0:
    p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
    p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
    break;
  case 1:
   p3d_set_RANDOM_CHOICE(P3D_HALTON_SAMPLING);
   p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
   break;
  case 2:
   p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
   p3d_set_SAMPLING_CHOICE(P3D_DYNAMIC_DOMAIN_SAMPLING);
   break;
  case 3:
   p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
   p3d_set_SAMPLING_CHOICE(P3D_GAUSSIAN_SAMPLING);
   break;
  default:
    PrintInfo(("Error: Sampling strategy not defined \n"));
  }

}

static void g3d_create_sampling_obj(void)
{FL_OBJECT *obj;
 
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,210,330,30,""); 
  obj = fl_add_box(FL_FLAT_BOX,140,205,90,10,"Sampling Strategy");

  GROUP4 = fl_bgn_group();
  SAMPL1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,30,207,50,35,"Random");
  fl_set_object_color(SAMPL1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SAMPL1_OBJ,CB_sampling_obj,0);
  SAMPL2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,105,207,50,35,"Halton");
  fl_set_object_color(SAMPL2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SAMPL2_OBJ,CB_sampling_obj,1);
  SAMPL3_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,180,207,50,35,"Dyn. Domain");
  fl_set_object_color(SAMPL3_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SAMPL3_OBJ,CB_sampling_obj,2);
  SAMPL4_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,255,207,50,35,"Gaussian");
  fl_set_object_color(SAMPL4_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SAMPL4_OBJ,CB_sampling_obj,3);
  GROUP3 = fl_end_group();
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  fl_set_button(SAMPL1_OBJ,1);
}

/* choix de l'orientation du graph */
static void CB_oriented_obj(FL_OBJECT *ob, long arg)
{
  int val;
  
  val = fl_get_button(ob);

  p3d_set_ORIENTED(val);
  if (XYZ_GRAPH != NULL)
    { XYZ_GRAPH->oriented = val; }
}  

static void g3d_create_oriented_obj(void)
{FL_OBJECT *obj;
 
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,170,80,30,""); 

  ORIENTED_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,15,167,50,35,"Oriented");
  fl_set_object_color(ORIENTED_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(ORIENTED_OBJ,CB_oriented_obj,0);
  fl_set_button(ORIENTED_OBJ,0);
}

// modif Juan
static void CB_W_obj(FL_OBJECT *ob, long arg)
{
  int val;
  val = fl_get_button(ob);

  p3d_set_rrt_w_mode(val);
  //p3d_set_rrt_pb_mode(val);
}  

static void g3d_create_W_obj(void)
{FL_OBJECT *obj;
 
  obj = fl_add_frame(FL_ENGRAVED_FRAME,260,170,90,30,""); 

  W_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,260,167,50,35,"Weighted");
  fl_set_object_color(W_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(W_OBJ,CB_W_obj,0);
  fl_set_button(W_OBJ,p3d_get_rrt_w_mode());
}
// fmodif Juan

/* permet de modifier le parametre NB_TRY */
static void CB_nbtry_input_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_NB_TRY(atoi(fl_get_input(INPUT_OBJ)));

   fl_hide_form(INPUT_FORM);
   
   fl_freeze_form(PLANNER_FORM);
   fl_set_slider_value(SEARCH_NBTRY_PARAM_OBJ,p3d_get_NB_TRY());
   fl_unfreeze_form(PLANNER_FORM);

   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}

static void CB_create_nbtry_input_obj(FL_OBJECT *ob, long arg)
{FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Nodes");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Annuler");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_nbtry_input_obj,0);
  
  fl_set_button(SEARCH_NBTRY_INPUT_OBJ,0);
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
}

static void g3d_create_nbtry_input_obj(void)
{
  SEARCH_NBTRY_INPUT_OBJ = fl_add_button(FL_PUSH_BUTTON,65.0,355.0,50.0,30.0,"Nb try");
  fl_set_object_callback(SEARCH_NBTRY_INPUT_OBJ,CB_create_nbtry_input_obj,0);
}

static void CB_nbtry_param_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_NB_TRY(floor(fl_get_slider_value(SEARCH_NBTRY_PARAM_OBJ)));
}

  static void CB_ntryDD_param_obj(FL_OBJECT *ob, long arg)
{
   p3d_set_nbtry_DD(floor(fl_get_slider_value(NTRYDD_PARAM_OBJ)));
}
static void CB_lambda_param_obj(FL_OBJECT *ob, long arg)
{
  p3d_set_LAMBDA(floor(fl_get_slider_value(LAMBDA_PARAM_OBJ)));
}


static void g3d_create_nbtry_param_obj(void)
{
  SEARCH_NBTRY_PARAM_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,355.0,245.0,30.0,"");
  fl_set_slider_step(SEARCH_NBTRY_PARAM_OBJ,1.0);
  fl_set_slider_bounds(SEARCH_NBTRY_PARAM_OBJ,10,10000);
  fl_set_slider_value(SEARCH_NBTRY_PARAM_OBJ,p3d_get_NB_TRY());
  fl_set_object_callback(SEARCH_NBTRY_PARAM_OBJ,CB_nbtry_param_obj,0);
}

static void g3d_create_ntryDD_param_obj(void)
{
  NTRYDD_PARAM_OBJ= fl_add_valslider(FL_HOR_SLIDER,250.0,60.0,110.0,20.0,"ntry DD");
  fl_set_slider_step(NTRYDD_PARAM_OBJ,1.0);
  fl_set_slider_bounds(NTRYDD_PARAM_OBJ,1,200);
  fl_set_slider_value(NTRYDD_PARAM_OBJ,p3d_get_nbtry_DD());
  fl_set_object_callback(NTRYDD_PARAM_OBJ,CB_ntryDD_param_obj,0);
}
static void g3d_create_lambda_param_obj(void)
{
  LAMBDA_PARAM_OBJ = fl_add_valslider(FL_HOR_SLIDER,250.0,95.0,110.0,20.0,"DD radius");
  fl_set_slider_step(LAMBDA_PARAM_OBJ,1.0);
  fl_set_slider_bounds(LAMBDA_PARAM_OBJ,1,100.);
  fl_set_slider_value(LAMBDA_PARAM_OBJ,p3d_get_LAMBDA());
  fl_set_object_callback(LAMBDA_PARAM_OBJ,CB_lambda_param_obj,0);
}

static FL_FORM *ADD_FORM;
static FL_OBJECT **POSITION_OBJ,**POSITION_BOX_OBJ,**POSITION_WALK_OBJ;
static configPt qadd, qadd_deg; /* configuration of the robot expressed 
           in radian and in degree */
double *dq_exp,  /* angles expressed in radian */
  *delta_exp;           /* angles expressed in radian */
static FL_FORM *EXP_CHOICE_FORM,*EXP_BOX_FORM,*EXP_WALK_FORM;
static int EXPAND = FALSE;
static FL_OBJECT *ok,*cancel,*expand;
 
static void CB_exp_walk_form(FL_OBJECT *ob, long arg)
{
  double val = fl_get_slider_value(ob), f_min, f_max;
  p3d_rob * robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);

  p3d_get_robot_dof_bounds(robotPt,arg, &f_min, &f_max);
  delta_exp[arg] = val*(f_max-f_min)/100.;
}


static void g3d_create_exp_walk_form(void)
{
int njnt, nb_dof;
double s;
double f_min, f_max;  // angles expressed in radian 
int ord, i, j, k;
char  str[30];
p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
p3d_jnt * jntPt;
FL_OBJECT *obj,*ok,*nbnode,*nbwalk;


nb_dof = calc_real_dof();
s = nb_dof*20.0+60.0;

nb_dof = robotPt->nb_dof;
njnt = robotPt->njoints;
  
  POSITION_WALK_OBJ = MY_ALLOC(FL_OBJECT *,nb_dof);
  delta_exp= MY_ALLOC(double,nb_dof);
  p3d_set_EXPAND_DELTA(delta_exp);
  
  EXP_WALK_FORM = fl_bgn_form(FL_UP_BOX,500.0,s);
  ord = 0;
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
  p3d_jnt_get_dof_bounds(jntPt, j, &f_min, &f_max);
  POSITION_WALK_OBJ[k] = 
    fl_add_valslider(FL_HOR_SLIDER,110.0,10.+20.0*ord,280.0,20.0,"");
  fl_set_slider_bounds(POSITION_WALK_OBJ[k],0.,100.);
  strcpy(str, "Delta "); 
  strcat(str, p3d_jnt_get_dof_name(jntPt, j));
  fl_add_box(FL_UP_BOX,10.0,10.+20.0*ord,100.0,20.0,str);

  fl_set_slider_value(POSITION_WALK_OBJ[k],10.);
  fl_set_call_back(POSITION_WALK_OBJ[k],CB_exp_walk_form,k);
  delta_exp[k] = 0.1*(f_max-f_min);
  ord = ord+1;
      } else
  { POSITION_WALK_OBJ[k] = NULL; }
    }
  }
  
  ok = fl_add_button(FL_PUSH_BUTTON, 10,20.+20.0*ord,60,30,"OK");

  nbnode = fl_add_input(FL_NORMAL_INPUT,450.0,140.0,40.0,20.0,
           "Nb node");
  fl_set_input_color(nbnode, 0, 0);
 
  nbwalk = fl_add_input(FL_NORMAL_INPUT,450.0,170.0,40.0,20.0,
           "Nb walk");
  fl_set_input_color(nbnode, 0, 0);
 
  fl_end_form();
  fl_show_form(EXP_WALK_FORM,FL_PLACE_SIZE,TRUE,"Expand Walk Param.");
 
  while (1){
    obj = fl_do_forms();
    if (obj == ok){
      p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));
      p3d_set_EXPAND_NB_WALK(atoi(fl_get_input(nbwalk)));
      fl_hide_form(EXP_WALK_FORM);
      fl_free_object(ok);
      fl_free_object(nbnode);
      fl_free_object(nbwalk);

      for(i=0;i<nb_dof;i++) {
  if(POSITION_WALK_OBJ[i]!=NULL) {
    fl_free_object(POSITION_WALK_OBJ[i]);
  } 
      } 
      MY_FREE(POSITION_WALK_OBJ,FL_OBJECT *,nb_dof);
      fl_free_form(EXP_WALK_FORM); 
      return;
    }
  }
}


static void CB_exp_box_form(FL_OBJECT *ob, long arg)
{
  double val = fl_get_slider_value(ob),f_min,f_max;
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);  
 
  p3d_get_robot_dof_bounds(robotPt, arg, &f_min,&f_max);
  dq_exp[arg] = val*(f_max-f_min)/100.;
}


static void g3d_create_exp_box_form(void)
{
  int njnt, nb_dof;
  double s;
  double f_min, f_max;
  int ord, i, j, k;
  char      str[30];
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;
  FL_OBJECT *obj,*ok,*nbnode;


  nb_dof = calc_real_dof();
  s = nb_dof*20.0+60.0;
  
  nb_dof = robotPt->nb_dof;
  njnt = robotPt->njoints;

  POSITION_BOX_OBJ = MY_ALLOC(FL_OBJECT *,nb_dof);
  dq_exp = MY_ALLOC(double,nb_dof);
  p3d_set_EXPAND_DQ(dq_exp);

  EXP_BOX_FORM = fl_bgn_form(FL_UP_BOX,500.0,s);
  ord = 0;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
  p3d_jnt_get_dof_bounds(jntPt, j, &f_min, &f_max);
  POSITION_BOX_OBJ[k] = 
    fl_add_valslider(FL_HOR_SLIDER,110.0,10.+20.0*ord,280.0,20.0,"");
  fl_set_slider_bounds(POSITION_BOX_OBJ[k],0.,100.);
  strcpy(str, "Delta "); 
  strcat(str, p3d_jnt_get_dof_name(jntPt, j));
  fl_add_box(FL_UP_BOX,10.0,10.+20.0*ord,100.0,20.0,str);

  fl_set_slider_value(POSITION_BOX_OBJ[k],10.);
  fl_set_call_back(POSITION_BOX_OBJ[k],CB_exp_box_form,k);
  dq_exp[k] = 0.1*(f_max-f_min);
  ord = ord+1;
      } else
  { POSITION_BOX_OBJ[k] = NULL; }
    }
  }

  ok = fl_add_button(FL_PUSH_BUTTON, 10,20.+20.0*ord,60,30,"OK");
  nbnode = fl_add_input(FL_NORMAL_INPUT,450.0,140.0,40.0,20.0,
      "Nb node");
  fl_set_input_color(nbnode, 0, 0);

  fl_end_form();
  fl_show_form(EXP_BOX_FORM,FL_PLACE_SIZE,TRUE,"Expand Box Param.");

  while (1){
    obj = fl_do_forms();
    if (obj == ok){
      p3d_set_EXPAND_NB_NODES(atoi(fl_get_input(nbnode)));
      fl_hide_form(EXP_BOX_FORM);
      fl_free_object(ok);
      fl_free_object(nbnode);

      for(i=0;i<nb_dof;i++){
  if(POSITION_BOX_OBJ[i]!=NULL)
    { fl_free_object(POSITION_BOX_OBJ[i]); }
      } 
      MY_FREE(POSITION_BOX_OBJ,FL_OBJECT *,nb_dof);
      fl_free_form(EXP_BOX_FORM); 
      return;
    }
  }
}

static void g3d_create_expand_choice_form(void)
{
  FL_OBJECT *obj,*group,*choice1,*choice2;
  
  EXP_CHOICE_FORM = fl_bgn_form(FL_UP_BOX,115.0,105);
  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,20,95,70,""); 
  obj = fl_add_box(FL_FLAT_BOX,25,15,60,10,"Expand type");
  
  group = fl_bgn_group();
  choice1 = fl_add_checkbutton(FL_RADIO_BUTTON,10,25,60,30,"Centered box");
  fl_set_object_color(choice1,FL_MCOL,FL_GREEN);
  p3d_set_EXPAND_CHOICE(P3D_EXPAND_BOX);
  choice2 = fl_add_checkbutton(FL_RADIO_BUTTON,10,55,60,30,"Random walk");
  fl_set_object_color(choice2,FL_MCOL,FL_GREEN);
  group = fl_end_group();
  
  fl_end_form();
  fl_show_form(EXP_CHOICE_FORM,FL_PLACE_SIZE,TRUE,"Expansion Mode");
  
  while (1){
    obj = fl_do_forms();
    if (obj == choice1){
      fl_hide_form(EXP_CHOICE_FORM);
      p3d_set_EXPAND_CHOICE(P3D_EXPAND_BOX);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(group);
      fl_free_form(EXP_CHOICE_FORM);
      g3d_create_exp_box_form();
      
    }
    if (obj == choice2){
      fl_hide_form(EXP_CHOICE_FORM); 
      p3d_set_EXPAND_CHOICE(P3D_EXPAND_RAND_WALK);
      fl_free_object(choice1);
      fl_free_object(choice2);
      fl_free_object(group);
      fl_free_form(EXP_CHOICE_FORM);
      g3d_create_exp_walk_form();
    }
  }     
}


static void CB_add_form(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;
  double val = fl_get_slider_value(ob);
  int i, i_dof;
  int nb_dof;
  int I_can;
  
  qadd_deg[arg] = val;
  I_can = p3d_set_and_update_robot_conf_deg(qadd_deg);

  nb_dof = p3d_get_robot_ndof();
 
  if (robotPt->cntrt_manager->cntrts != NULL) {  
    if(I_can) {
      p3d_get_robot_config_deg_into(robotPt, &qadd_deg);
      for(i=0; i<nb_dof; i++) {
  fl_set_slider_value(POSITION_OBJ[i], qadd_deg[i]);
      }
      p3d_copy_config_into(robotPt, qadd_deg, &last_p_deg);
    } else {
      p3d_copy_config_into(robotPt, last_p_deg, &qadd_deg);
      for(i=0; i<nb_dof; i++) {
  fl_set_slider_value(POSITION_OBJ[i], qadd_deg[i]);
      }
      jntPt = p3d_robot_dof_to_jnt(robotPt, arg, &i_dof);
      p3d_jnt_set_dof_deg(jntPt, i_dof, qadd_deg[arg]);
      p3d_update_this_robot_pos_without_cntrt(robotPt);
    }
  }
  
  p3d_convert_config_deg_to_rad(robotPt, qadd_deg, &qadd);
  
  if(G3D_ACTIVE_CC){
    p3d_col_test_all();
  }
  
  g3d_draw_allwin_active();
}


static void CB_add_exp_button_obj(FL_OBJECT *ob, long arg)
{
  EXPAND = TRUE;
  g3d_create_expand_choice_form();
}

static void CB_add_ok_button_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  int i, nb_dof = robotPt->nb_dof;
  p3d_node *N,*Nc;
  int nb_node_sorted;
  p3d_compco *comp;
  int LINK_LINK,LINK_ORPH,NB_LINK_ORPH=0,NB_LINK=0,GRAPH_LINKED=FALSE;
  double dist;
  p3d_list_node *node, *linked_nodes=NULL, *last_node=NULL,
    *destr_node,*list_node;
  int *iksol=NULL;   // <- modif Juan



 fl_hide_form(ADD_FORM);

 p3d_set_and_update_robot_conf(qadd);

 // modif Juan
 p3d_get_iksol_vector(robotPt->cntrt_manager,iksol);
 N = p3d_APInode_make_multisol(XYZ_GRAPH,qadd,iksol);
 // fmodif Juan

 switch(p3d_get_MOTION_PLANNER()){
   case 1: 
     p3d_insert_node(XYZ_GRAPH,N);
     if(XYZ_GRAPH->nnode>1){
       comp = XYZ_GRAPH->comp;
       while(comp != NULL) {
   if(N->numcomp != comp->num){
     if(p3d_link_node_comp(XYZ_GRAPH, N, &comp)){
       GRAPH_LINKED = TRUE;
     }
   }
   comp = comp->suiv;
       }
     }
     break;
   case 2:
     
     if(XYZ_GRAPH->nnode>=1){
       comp = XYZ_GRAPH->comp;
       /* on parcourt toutes les composantes connexes */
       while(comp != NULL) {
   LINK_LINK = FALSE;
   LINK_ORPH = FALSE;
   PrintInfo(("On essaye de relier le noeud %d a la comp %d\n",
        N->num,comp->num));
   list_node = comp->dist_nodes;
   nb_node_sorted = 0;
   while(list_node != NULL) {
     Nc = list_node->N;
     /* si le noeud est reliable, on arrete */
     if(p3d_APInode_linked(XYZ_GRAPH,N,Nc,&dist)){
       PrintInfo(("elles sont reliees\n"));
       if(Nc->type == ISOLATED){
         if(!LINK_ORPH) 
     { NB_LINK_ORPH = NB_LINK_ORPH + 1; }
         LINK_ORPH = TRUE;
       }
       if(!LINK_LINK)
         { NB_LINK = NB_LINK + 1; }
       LINK_LINK = TRUE;
       p3d_add_node_compco(Nc, comp);
       node = MY_ALLOC(p3d_list_node,1);
       node->N = Nc;
       node->next = NULL;
       if(NB_LINK==1) {
         linked_nodes = node; last_node = node;
       } else if(NB_LINK>1) {
         last_node->next = node; 
         last_node = last_node->next;
       }
     }
     list_node = list_node->next;
   }
   comp = comp->suiv;
       }
     }
     
     /* quand toutes les composantes ont ete parcourues */
     if(NB_LINK_ORPH == 0 && NB_LINK == 0){
       N->type = ISOLATED;
       p3d_insert_node(XYZ_GRAPH,N);
       
       GRAPH_LINKED = FALSE;
     }
     else{     
       if(NB_LINK_ORPH >= 0 && NB_LINK>1){
   N->type = LINKING;
       }
       else{
   N->type = ISOLATED;
       }
       p3d_insert_node(XYZ_GRAPH,N);
       node = linked_nodes;
       while(node != NULL) {
   if(N->numcomp<node->N->numcomp){
     p3d_merge_comp(XYZ_GRAPH,N->comp,&(node->N->comp));
   } else {
     p3d_merge_comp(XYZ_GRAPH,node->N->comp,&(N->comp));
   }
   p3d_create_edges(XYZ_GRAPH,N,node->N,dist);  
   destr_node = node;
   node = node->next;
   MY_FREE(destr_node,p3d_list_node,1);
       }
       GRAPH_LINKED = TRUE;
     }
     break;
 default: 
   p3d_insert_node(XYZ_GRAPH,N);
   if(XYZ_GRAPH->nnode>1){
     comp = XYZ_GRAPH->comp;
     while(comp != NULL) {
       if(N->numcomp != comp->num){
   if(p3d_link_node_comp(XYZ_GRAPH, N, &comp)){
     GRAPH_LINKED = TRUE;
   }
       }
       comp = comp->suiv;
     }
   }
 }
 
 PrintInfo(("GRAPH_LINKED : %d\n",GRAPH_LINKED));
 
 if(!(GRAPH_LINKED) && EXPAND)
   p3d_APInode_expand(XYZ_GRAPH,N,fct_stop,fct_draw);

 fl_set_button(ADD_OBJ,0);
 fl_free_object(cancel);
 fl_free_object(ok);
 fl_free_object(expand);

 for(i=0;i<nb_dof;i++){
   if (POSITION_OBJ[i] != NULL)
     { fl_free_object(POSITION_OBJ[i]); }
 }
 MY_FREE(POSITION_OBJ,FL_OBJECT *,nb_dof);
 fl_free_form(ADD_FORM);
 p3d_destroy_config(robotPt, last_p_deg);
 return;
}

static void CB_add_cancel_button_obj(FL_OBJECT *ob, long arg)
{p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 int nb_dof = robotPt->nb_dof;
 int i;

 fl_hide_form(ADD_FORM);
  
 p3d_destroy_config(robotPt, qadd_deg);
 p3d_destroy_config(robotPt, qadd);

 fl_set_button(ADD_OBJ,0);
 fl_free_object(cancel);
 fl_free_object(ok);
 fl_free_object(expand);

 for(i=0;i<nb_dof;i++) {
   if (POSITION_OBJ[i] != NULL)
     { fl_free_object(POSITION_OBJ[i]); }
 } 
 MY_FREE(POSITION_OBJ,FL_OBJECT *,nb_dof);
 fl_free_form(ADD_FORM); 
 return; 
}


static void CB_add_obj(FL_OBJECT *ob, long arg)
{
  int njnt, nb_dof;
  double s;
  double f_min ,f_max;
  int ord, i, j, k;
  char      str[30];
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_jnt * jntPt;

  /* copy the position of the current robot into qadd */
  qadd_deg = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_POS);
  qadd = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  
  last_p_deg = p3d_alloc_config(robotPt);
  p3d_copy_config_into(robotPt, qadd_deg, &last_p_deg);
  
  if(!XYZ_GRAPH){printf("ERREUR : pas de graphe courant\n");return;}
  
  nb_dof = calc_real_dof();
  s = nb_dof*20.0+60.0;
  
  nb_dof = robotPt->nb_dof;
  njnt = robotPt->njoints;
  
  POSITION_OBJ = MY_ALLOC(FL_OBJECT *,nb_dof);
  
  ADD_FORM = fl_bgn_form(FL_UP_BOX,500.0,s);
  ord = 0;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
  p3d_jnt_get_dof_bounds_deg(jntPt, j, &f_min, &f_max);
  POSITION_OBJ[k] = 
    fl_add_valslider(FL_HOR_SLIDER,110.0,10.+20.0*ord,280.0,20.0,"");
  fl_set_slider_bounds(POSITION_OBJ[k], f_max, f_min);
  strcpy(str, "Pos "); 
  strcat(str, p3d_jnt_get_dof_name(jntPt, j));
  fl_add_box(FL_UP_BOX,10.0,10.+20.0*ord,100.0,20.0,str);

  fl_set_slider_value(POSITION_OBJ[k], qadd_deg[k]);
  fl_set_call_back(POSITION_OBJ[k],CB_add_form,k);
  if(robotPt->cntrt_manager->in_cntrt[k] == 2) {
    fl_set_slider_size(POSITION_OBJ[k],1.0);
  }
  ord = ord+1;
      } else
  { POSITION_OBJ[k] = NULL; }
    }
  }
  
  ok = fl_add_button(FL_PUSH_BUTTON, 10,20.+20.0*ord,60,30,"OK");
  fl_set_call_back(ok,CB_add_ok_button_obj,arg);
  cancel = fl_add_button(FL_PUSH_BUTTON, 330,20.+20.0*ord,60,30,"cancel");
  fl_set_call_back(cancel,CB_add_cancel_button_obj,arg);
  expand = fl_add_checkbutton(FL_RADIO_BUTTON,410,20,55,30,"Expand \nif linking \nfails");
  fl_set_call_back(expand,CB_add_exp_button_obj,arg);
  fl_set_object_color(expand,FL_MCOL,FL_GREEN);
  
  fl_end_form();
  fl_show_form(ADD_FORM,FL_PLACE_SIZE,TRUE,"Add Node");
}
  

static void g3d_create_add_obj(void)
{
  ADD_OBJ = fl_add_button(FL_PUSH_BUTTON, 155.0,395.0,45.0,30.0,
        "Add\nnode");
  fl_set_call_back(ADD_OBJ,CB_add_obj,0);
}




static void g3d_create_frame_obj(void)
{
  
  FRAME_OBJ = fl_add_frame(FL_ENGRAVED_FRAME,130,30,110,80,""); 
}

/*****************************************************************/

/* fonctions de destruction des objets xforms */
void g3d_delete_planner_form(void)
{
  if(fl_get_button(planner_obj)){fl_hide_form(PLANNER_FORM);}
  
  g3d_delete_local_search_obj(); 
  g3d_delete_random_confs_obj();   // modif Juan
  g3d_delete_drawnjnt_obj();       // modif Juan
  g3d_delete_global_search_obj();
  g3d_delete_specific_search_obj();
  g3d_delete_N_specific_obj();
  g3d_delete_optim_form();
  g3d_delete_optim_obj();


  g3d_delete_global_input_obj();
  g3d_delete_global_param_obj();

  g3d_delete_compco_input_obj();
  g3d_delete_compco_param_obj();  

  g3d_delete_del_param_obj();

  g3d_delete_DMAX_param_obj();

  g3d_delete_draw_optim_search_obj();

  g3d_delete_print_obj();
  g3d_delete_draw_obj();

  g3d_delete_load_graph_obj();
  g3d_delete_save_graph_obj();

  g3d_delete_stop_obj();

  g3d_delete_pinpoint_node_obj();
  g3d_delete_expand_obj();
  g3d_delete_isolate_obj();
  g3d_delete_linking_obj();
  g3d_delete_test_obj();

  g3d_delete_del_node_obj();
  g3d_delete_plan_strategy_obj();
  g3d_delete_sorting_obj();
  g3d_delete_sampling_obj();
  g3d_delete_W_obj();
  g3d_delete_oriented_obj();
  g3d_delete_nbtry_input_obj();
  g3d_delete_nbtry_param_obj();
  g3d_delete_ntryDD_param_obj();
  g3d_delete_lambda_param_obj();
  g3d_delete_add_obj();

  g3d_delete_frame_obj();
    
  fl_free_form(PLANNER_FORM);
}


static void g3d_delete_local_search_obj(void)
{ 
  fl_free_object(SEARCH_OBJ);
}

// modif Juan
static void g3d_delete_random_confs_obj(void)
{ 
  fl_free_object(RANDCONFS_OBJ);
}

static void g3d_delete_drawnjnt_obj(void)
{ 
  fl_free_object(DRAWNJNT_OBJ);
}
// fmodif Juan

static void g3d_delete_global_search_obj(void)
{ 
  fl_free_object(SEARCH_GLOBAL_OBJ);
}


static void g3d_delete_specific_search_obj(void)
{ 
  fl_free_object(SEARCH_SPECIFIC_OBJ);
}

static void g3d_delete_N_specific_obj(void)
{ 
  fl_free_object(N_SPECIFIC_OBJ);
}

static void g3d_delete_optim_obj(void)
{
  fl_free_object(OPTIM_OBJ);
}

static void g3d_delete_global_input_obj(void)
{
  fl_free_object(SEARCH_GLOBAL_INPUT_OBJ);
}  

static void g3d_delete_global_param_obj(void)
{
  fl_free_object(SEARCH_GLOBAL_PARAM_OBJ);
}

static void g3d_delete_del_param_obj(void)
{
  fl_free_object(SEARCH_DEL_PARAM_OBJ);
}

static void g3d_delete_DMAX_param_obj(void)
{
  fl_free_object(SEARCH_DMAX_PARAM_OBJ);
}

static void g3d_delete_draw_obj(void)
{
  fl_free_object(SEARCH_DRAW_OBJ);
}

static void g3d_delete_print_obj(void)
{
  fl_free_object(SEARCH_PRINT_OBJ);
}

static void g3d_delete_compco_param_obj(void)
{
  fl_free_object(SEARCH_COMPCO_PARAM_OBJ);
}

static void g3d_delete_compco_input_obj(void)
{
  fl_free_object(SEARCH_COMPCO_INPUT_OBJ);
}  

static void g3d_delete_draw_optim_search_obj(void)
{ 
  fl_free_object(SEARCH_DRAW_OPTIM_OBJ);
}

static void g3d_delete_load_graph_obj(void)
{
  fl_free_object(LOAD_GRAPH_OBJ);
} 

static void g3d_delete_save_graph_obj(void)
{
  fl_free_object(SAVE_GRAPH_OBJ);
}

static void g3d_delete_stop_obj(void)
{
  fl_free_object(STOP_OBJ);
}

static void g3d_delete_pinpoint_node_obj(void)
{
  fl_free_object(PINPOINT_NODE_OBJ);
}

static void g3d_delete_expand_obj(void)
{
  fl_free_object(EXPAND_OBJ);
}

static void g3d_delete_isolate_obj(void)
{
  fl_free_object(ISOLATE_OBJ);
}

static void g3d_delete_linking_obj(void)
{
  fl_free_object(LINKING_OBJ);
}

static void g3d_delete_test_obj(void)
{
  fl_free_object(TEST_OBJ);
}

static void g3d_delete_del_node_obj(void)
{
  fl_free_object(DEL_NODE_OBJ);
}

static void g3d_delete_plan_strategy_obj(void)
{
  fl_free_object(STRAT1_OBJ);
  fl_free_object(STRAT2_OBJ);
  fl_free_object(STRAT3_OBJ);
  fl_free_object(STRAT4_OBJ);
  fl_free_object(STRAT5_OBJ);
  fl_free_object(STRAT6_OBJ);
}

static void g3d_delete_sorting_obj(void)
{
  fl_free_object(SORT1_OBJ);
  fl_free_object(SORT2_OBJ); 
} 

static void g3d_delete_sampling_obj(void)
{
  fl_free_object(SAMPL1_OBJ);
  fl_free_object(SAMPL2_OBJ);
  fl_free_object(SAMPL3_OBJ);
  fl_free_object(SAMPL4_OBJ);
}
// modif juan
static void g3d_delete_W_obj(void)
{
  fl_free_object(W_OBJ);
}
// fmodif Juan

static void g3d_delete_oriented_obj(void)
{
  fl_free_object(ORIENTED_OBJ);
}

static void g3d_delete_nbtry_param_obj(void)
{
  fl_free_object(SEARCH_NBTRY_PARAM_OBJ);
}

static void g3d_delete_ntryDD_param_obj(void)
{
  fl_free_object(NTRYDD_PARAM_OBJ);
}



static void g3d_delete_nbtry_input_obj(void)
{
  fl_free_object(SEARCH_NBTRY_INPUT_OBJ);
}  
static void g3d_delete_lambda_param_obj(void)
{
  fl_free_object(LAMBDA_PARAM_OBJ);
}

static void g3d_delete_add_obj(void)
{
  fl_free_object(ADD_OBJ);
}

static void g3d_delete_frame_obj(void)
{
  fl_free_object(FRAME_OBJ);
}

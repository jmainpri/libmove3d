#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Move3d-pkg.h"


#define DEFAULT_SEED 1
#define DEFAULT_CALL 100
#define USER_DEFINED 0
#define TO_BE_CHOSEN 1

static configPt *confi = NULL;
static int n_tests;

static int SEED = -5;
static int REPETE = 0;
static int CALL = -5;
/* extern int PLANNER_CHOICE; */
extern void p3d_col_get_dmax(double *val);
/* choose configs */
void init_confi(int nrjt, int nr_tests)
{
  confi = MY_ALLOC(configPt, nr_tests);
  n_tests = nr_tests;
}


void clean_up_confi(int nrjt, int nr_tests)
{int i;
 p3d_rob *r = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);

  if(confi!=NULL)
    {
      for(i=0;i<nr_tests;i++)
	{
	  p3d_destroy_config(r,confi[i]);
	}
      MY_FREE(confi, configPt, n_tests);
      confi = NULL;
    }
}

void choose_confi(int nrjt, int nr_tests)
{
  int i;
  p3d_rob *r = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);

  if(confi == NULL)
    {    
      init_confi(nrjt, nr_tests);
      for(i=0;i<nr_tests;i++)
	{
	  confi[i] = p3d_alloc_config(r);
#ifdef P3D_PLANNER
	  p3d_shoot(r,confi[i],1);
#else
		printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	  /* report on configuration */
	}
    }
}




void init_seed(void)
{
  srand(SEED);
}

void set_repete(int seed_val)
{
  REPETE = seed_val;
}

void set_seed(int seed_val)
{
  SEED = seed_val;
}

void unset_seed(void)
{
  SEED = -5;
}

void set_call(int call_val)
{
  CALL = call_val;
}

void unset_call(void)
{
  CALL = -5;
}

void shoot_n_hit(int nr_tests)
{
  int k, i, hit=0, nr_hits=0;
  configPt q = NULL;

  /* q = MY_ALLOC(double,njnt+4); */
  if (p3d_col_get_mode() == p3d_col_mode_v_collide)
    { PrintInfo(("V_COLLIDE\n")); }
  else if(p3d_col_get_mode() == p3d_col_mode_kcd)
    PrintInfo(("KCD\n"));
  else
    PrintInfo(("error: no active collision detector\n"));

  if(g3d_get_KCD_CHOICE_IS_ACTIVE())
    {
      PrintInfo(("shoot_n_hit for kcd choice\n"));
      ChronoOn();           
      for(i=0;i<nr_tests;i++)
	{	
	  q = confi[i];
	  for(k=0;k<=REPETE;k++)
	    {	    
	      p3d_set_and_update_robot_conf(q);
	      hit = p3d_col_test_choice();	    
	      if(hit)
		{		 
		  nr_hits++;	 
		}
	    }
	} 
    }
  else
    {
      /* start timing */
      ChronoOn();
      /* do tests */
      
      for(i=0;i<nr_tests;i++)
	{
	  /* chose random configuration */
	  /* p3d_shoot(r,q,1); */
	  q = confi[i];
	  /* report on configuration */
	  /* print_config(XYZ_ENV->cur_robot,q); */

	  /* we repete the test REPETE times */
	  for(k=0;k<=REPETE;k++)
	    {
	      /* put robot in place */
	      p3d_set_and_update_robot_conf(q);
	      
	      /* test for collision */	 
	      hit = p3d_col_test();
	      /* report on collision test */
	      if(hit)
		{
		  /* PrintInfo(("hit \n")); */
		  nr_hits++;
		}
	      /* 	  else */
	      /* 	    PrintInfo(("missed \n")); */	 
	    }
	} 
    }
  ChronoPrint("Shoot-n-Hit");
  ChronoOff();
  PrintInfo(("number of hits = %i out of %i\n",nr_hits,CALL*(REPETE+1)));
}

void test_current_coll_test(int nr_tests)
{
  int default_seed = FALSE;
  int clean_up_later = FALSE;
  p3d_rob *r = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
  int nrjt = r->njoints;

  if(confi == NULL)
    {
      clean_up_later = TRUE;
      choose_confi(nrjt,nr_tests);
    }

  if(SEED == -5)
    {
      default_seed = TRUE;
      set_seed(DEFAULT_SEED);
    }
  init_seed();
PrintInfo(("test_current_coll_test(): SEED = %i, DEFAULT_SEED = %i \n", 
SEED, DEFAULT_SEED));
PrintInfo(("test_current_coll_test(): CALL = %i, DEFAULT_CALL = %i \n", 
CALL, DEFAULT_CALL));
PrintInfo(("test_current_coll_test(): REPETE = %i \n", REPETE));
  shoot_n_hit(nr_tests);

  if(default_seed)
    unset_seed();

  if(clean_up_later)
      clean_up_confi(nrjt,nr_tests);

}

void compare_I_V_collide(int nr_tests)
{
  p3d_rob *r = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
  int nrjt = r->njoints;

  PrintInfo(("--\n"));

  choose_confi(nrjt,nr_tests);

  /* VCOLLIDE */
  /* p3d_BB_set_mode_close(); */
  p3d_col_start(p3d_col_mode_v_collide);
  test_current_coll_test(nr_tests);

  clean_up_confi(nrjt,nr_tests);
}

void plan_n_hit(int what_path)
{
  p3d_rob          *robotPt;
  int               njnt, ntest, ntrj;
  configPt          q_init = NULL;
  configPt          q_goal = NULL;
  p3d_localpath     *localpathPt=NULL;
  char              str[50],sti[20];
  double dmax;
	p3d_col_get_dmax(&dmax);
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  njnt = robotPt->njoints;
  PrintInfo(("plan_n_hit(): njnt = %i, tol + %f \n",njnt,dmax));
  /* start timing */
  ChronoOn();

  if(what_path == TO_BE_CHOSEN)
    {
      /* random initial configuration */
      q_init = p3d_alloc_config(robotPt);
			
#ifdef P3D_PLANNER
      p3d_shoot(robotPt, q_init, 1);
#else
			printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif

      print_config(robotPt, q_init);

      /* random goal configuration */
      q_goal = p3d_alloc_config(robotPt);
			
#ifdef P3D_PLANNER
      p3d_shoot(robotPt, q_goal, 1);
#else
			printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif

      print_config(robotPt, q_goal);

    }
  else 
    {
      /* what_path == USER_DEFINED */
      q_init = robotPt->ROBOT_POS;
      q_goal = robotPt->ROBOT_GOTO;
    }

  /* make the local path */
  
  localpathPt = p3d_local_planner(robotPt,q_init,q_goal);

  /* report path found or not */
  switch(p3d_search_status()){
    case P3D_SUCCESS: PrintInfo(("SUCCESS"));break;
    case P3D_FAILURE: PrintInfo(("FAILURE"));break;
    case P3D_CONFIG_EQUAL: PrintInfo(("CONFIG_EQUAL"));break;
    case P3D_ILLEGAL_START: PrintInfo(("ILLEGAL_START"));break;
    case P3D_ILLEGAL_GOAL: PrintInfo(("ILLEGAL_GOAL"));break;
    case P3D_WRONG_CALL: PrintInfo(("WRONG_CALL"));break;
    default: PrintInfo(("???"));
  }
  PrintInfo(("\n"));
  /* test for collision along path */
  if(localpathPt != NULL){
    /* make  a name for the path */
    strcpy(str, "loctrj."); 
    sprintf(sti,"%s",robotPt->name); strcat(str,sti);
    sprintf(sti,"%s","."); strcat(str,sti);
    ntrj = p3d_get_desc_number(P3D_TRAJ)+1;
    sprintf(sti,"%d",ntrj); strcat(str,sti);
    /* recuperate the description of the path */
    p3d_beg_desc(P3D_TRAJ,str);
    p3d_add_desc_courbe(localpathPt);
    p3d_end_desc();
    g3d_add_traj((char*)"Localsearch",p3d_get_desc_number(P3D_TRAJ));
    /* do collision test */
    ntest=0;
    ChronoPrint("planned");
    if(p3d_col_test_localpath(robotPt, localpathPt, &ntest))
      PrintInfo(("p3d_col_test_localpath: IN COLLISION ! nbtest : %d\n",
	     ntest));
    else
      PrintInfo(("p3d_col_test_localpath: collision-free ; nbtest : %d\n",
	     ntest));
    
  }
  /* report timing and stop timing */
  ChronoPrint("Plan-n-Hit");
  ChronoOff();
  
  PrintInfo(("\n"));
  if(what_path == TO_BE_CHOSEN){
    p3d_destroy_config(robotPt, q_init);
    p3d_destroy_config(robotPt, q_goal);
  }
}

void test_path_coll_test()
{
  int default_seed = FALSE;
  
  if(SEED == -5)
    {
      default_seed = TRUE;
      set_seed(DEFAULT_SEED);
    }
  init_seed();
  PrintInfo(("test_path_coll_test(): SEED = %i, DEFAULT_SEED = %i \n", 
	 SEED, DEFAULT_SEED));
  plan_n_hit(TO_BE_CHOSEN);
  if(default_seed)
    unset_seed();
}

void compare_I_V_path_collide()
{
  PrintInfo(("--\n"));
  /* VCOLLIDE */
  /* p3d_BB_set_mode_close(); */
  p3d_col_start(p3d_col_mode_v_collide);
  test_path_coll_test();
}

void compare_I_V_collide_call_times()
{
  int default_call = FALSE;

  if(CALL == -5)
    {
      default_call = TRUE;
      set_call(DEFAULT_CALL);
    }
  compare_I_V_collide(CALL);
  if(default_call)
    unset_call();
}
void test_current_coll_call_times()
{
  int default_call = FALSE;

  if(CALL == -5)
    {
      default_call = TRUE;
      set_call(DEFAULT_CALL);
    }
  test_current_coll_test(CALL);
  if(default_call)
    unset_call();
}

void test_upath_coll_test(void)
{
  plan_n_hit(USER_DEFINED);
}

void compare_I_V_upath_collide(void)
{
  PrintInfo(("--\n"));
  /* VCOLLIDE */
  /* p3d_BB_set_mode_close(); */
  p3d_col_start(p3d_col_mode_v_collide);
  test_upath_coll_test();
}

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
//#include "Collision-pkg.h"

static void save_scenario_name(const char * file);
static int read_scenario(FILE * f);
static int save_scenario(FILE * f);

/* Missing prototypes */
//extern int	fclose(FILE *);
//extern int	fscanf(FILE *, const char *, ...);

/* Local variables */
static char * scenario_name = NULL; // Scenario file name
static char * scenario_path = NULL; // Scenario file path

/***************************************************************/

/* NOTE (Juan) :
   RLG and parallel are not already considered !
*/

/***************************************************************/
/***************************************************************/
/* Public functions */

/*------------------------------------------------------------*/
/* Scenario file name management */

/* Clear the current file name */
void p3d_rw_scenario_delete_name(void)
{
  if (scenario_name != NULL) {
    MY_STRFREE(scenario_name);
    scenario_name = NULL;
  }
  if (scenario_path != NULL) {
    MY_STRFREE(scenario_path);
    scenario_path = NULL;
  }
}

/* Give a default file name if it is not initialized */
void p3d_rw_scenario_init_name(void)
{
  char c_dir_name[300];
  int c_sz;

  if (scenario_name == NULL) {  /* Default name */
    strcpy(c_dir_name, p3d_get_desc_curname(P3D_ENV));
    strcat(c_dir_name, ".sce");
    scenario_name = MY_STRDUP(c_dir_name);
  }
  if (scenario_path == NULL) { /* Default path */
    p3d_get_directory(c_dir_name);
    c_sz = (int) strlen(c_dir_name); 
    if(c_dir_name[c_sz-1] != '/')
      { strcat(c_dir_name,"/SCENARIO"); }
    else
      { strcat(c_dir_name,"SCENARIO"); }
    scenario_path = MY_STRDUP(c_dir_name);
  }
}

/* Return the current scenario file name (without the path)  */
const char * p3d_rw_scenario_get_name(void)
{ return scenario_name; }

/* Return the current scenario file path */
const char * p3d_rw_scenario_get_path(void)
{ return scenario_path; }


/*------------------------------------------------------------*/
/* Read a scenario file,
   return FALSE in case of failure */
int p3d_read_scenario(const char *file)
{
  FILE *fdc;
  int ret; 

  if(!(fdc=fopen(file,"r"))) {
    PrintError(("p3d_rw_scenario_read: can't open %s\n",file));
    return(FALSE);
  }
  save_scenario_name(file);
  fseek(fdc,0,0);
  
  ret = read_scenario(fdc);

  fclose(fdc);
  return(ret);
}


/*------------------------------------------------------------*/
/* Save a scenario file,
   return FALSE in case of failure */
int p3d_save_scenario(const char *file)
{
  FILE *fdc;
  int ret; 

  if(!(fdc=fopen(file,"w"))) {
    PrintError(("p3d_rw_scenario_save: can't write %s\n",file));
    return(FALSE);
  }
  save_scenario_name(file);
  
  ret = save_scenario(fdc); 

  fclose(fdc);
  return(ret);
}


/***************************************************************/
/***************************************************************/
/* Local functions */

/* Save the name and the path of the scenario file "file" */
static void save_scenario_name(const char * file)
{
  int c_sz, i;
  char * tmp_str;

  p3d_rw_scenario_delete_name();
  tmp_str = MY_STRDUP(file);
  c_sz = strlen(tmp_str);
  for(i=c_sz-1; (i>=0) && (file[i]!='/'); i--);
  if (i==0) {
    scenario_name = tmp_str;
    scenario_path = MY_STRDUP("");
  } else {
    tmp_str[i] = '\0';
    scenario_name = MY_STRDUP(tmp_str+i+1);
    scenario_path = MY_STRDUP(tmp_str);
    tmp_str[i] = '/';
    MY_STRFREE(tmp_str);
  }
}


/*------------------------------------------------------------*/
/* Return a type of curpos, and change curpos to point on the
   next name.
   Return FALSE if curpos is a void string. */
static int read_string_type(char ** curpos, int *type)
{
  char * name, *pos;

  pos = * curpos;
  if (!p3d_read_string_name(&pos, &name))
    { return FALSE; }

  if((strcmp(name,"P3D_ROBOT")== 0) ||
     (strcmp(name,"M3D_ROBOT")== 0))    *type = P3D_ROBOT;
  else return(FALSE);

  *curpos = pos;
  return(TRUE);
}


/*------------------------------------------------------------*/
/* Check if the environnement of the scenario file is the
   current environnement. Return FALSE in case of failure. */
static int check_current_environnement(FILE * f, int * num_line) 
{
  char * pos, * name, * line = NULL;  
  int size_max_line = 0;  /* dynamic management of line lenght */

  size_max_line = p3d_read_line_next_function(f, &line, 
					      size_max_line, num_line);
  if (size_max_line == 0) {
    PrintError(("p3d_rw_scenario: check_current_environnement: no function in the file !\n"));
    return FALSE;
  }
  pos = line;
  p3d_read_string_name(&pos, &name); /* with read_line_next_fonction before,
				     p3d_read_string_name returns TRUE */
  if ((strcmp(name,"p3d_sel_desc_name") != 0) &&
      (strcmp(name,"M3D_sel_desc_name") != 0)) {
    PrintError(("p3d_rw_scenario: check_current_environnement: the first function of the file must be p3d_sel_desc_name P3D_ENV name !\n"));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  if((!p3d_read_string_name(&pos, &name)) || ((strcmp(name,"P3D_ENV")!= 0) && 
     (strcmp(name,"M3D_ENV")!= 0))) {
    PrintError(("p3d_rw_scenario: check_current_environnement: the first function of the file must be p3d_sel_desc_name P3D_ENV name !\n"));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  if((!p3d_read_string_name(&pos, &name)) ||
     (strcmp(name, p3d_get_desc_curname(P3D_ENV))!= 0)) {
    PrintError(("p3d_rw_scenario: check_current_environnement: current environnement no corresponding: %s != %s !\n", p3d_get_desc_curname(P3D_ENV), name));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  MY_FREE(line, char, size_max_line);
  return TRUE;
}

/*------------------------------------------------------------*/
/* Initialize data before loading a scenario */
static void init_variables_scenario()
{
  int ir, nrob, njnt;
  pp3d_rob robotPt;
  p3d_env * envPt;

  nrob = p3d_get_desc_number(P3D_ROBOT);
  for(ir=0; ir<nrob; ir++) {
    p3d_sel_desc_num(P3D_ROBOT, ir);
    robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
    njnt = robotPt->njoints;
    while(robotPt->nconf>0)
      { p3d_del_config(robotPt, robotPt->conf[0]); }
    while(robotPt->nt>0)
      { p3d_del_traj(robotPt->t[0]); }
    p3d_clear_cntrt_manager(robotPt->cntrt_manager);
  }
  envPt = (p3d_env*)p3d_get_desc_curid(P3D_ENV);
}

/*------------------------------------------------------------*/
  /* static function to return errors for read_scenario()*/
  
static int read_error(char *fct, int num_line, int size_max_line, 
		      char **line, int size_max_dtab, double **dtab, int rcur)
{
  PrintWarning(("p3d_rw_scenario: read_scenario: invalid function %s\n",fct));
  PrintWarning(("  In line %i:%s\n", num_line, *line));
  if (size_max_line>0) {
    MY_FREE(*line, char, size_max_line); 
    *line = NULL;
  }
  if (size_max_dtab>0) {
    MY_FREE(*dtab, double, size_max_dtab);
    *dtab = NULL;
  }
  p3d_sel_desc_num(P3D_ROBOT, rcur);
  return FALSE;
}


/*------------------------------------------------------------*/

/*------------------------------------------------------------*/
/* Read a scenario file,
   return FALSE in case of failure */
static int read_scenario(FILE *f)
{
  char * pos, * fct, * name, * line = NULL; 
  int size_max_line = 0;  /* dynamic management of line lenght */
  double dval, *dtab = NULL;
  int *itab1 = NULL, *itab2 = NULL, *itab3 = NULL;
  int *itab4 = NULL, *itab5 = NULL;
  int size_max_dtab = 0;  /* dynamic management of tab lenght  */
  int size_max_itab1 = 0, size_max_itab2 = 0, size_max_itab3 = 0;
  int size_max_itab4 = 0, size_max_itab5 = 0;
  int type, n, nb_dof, num_line = 0;
  int rcur, i;
  int rwc = 0;
  int argnum[5];

  /* Function to return errors */
#define READ_ERROR() (read_error(fct, num_line, size_max_line, &line, size_max_dtab, &dtab, rcur))
  
  /* Check the environnement */
  if (!check_current_environnement(f, &num_line))
    { return FALSE; }
  rcur = p3d_get_desc_curnum(P3D_ROBOT);

  init_variables_scenario();

  while((size_max_line = p3d_read_line_next_function(f, &line, size_max_line, 
						 &num_line)) != 0) {
    pos = line;
    p3d_read_string_name(&pos, &fct); /* with p3d_read_line_next_function before,
				    p3d_read_string_name returns TRUE */

    if((strcmp(fct,"p3d_sel_desc_name")==0) ||
       (strcmp(fct,"M3D_sel_desc_name")==0)) {
      if(!read_string_type(&pos,&type))   return(READ_ERROR());
      if(!p3d_read_string_name(&pos,&name))   return(READ_ERROR());
      if(!p3d_sel_desc_name(type, name)) return(READ_ERROR());
      continue;
    }
    if((strcmp(fct,"p3d_set_robot_radius")==0) ||
       (strcmp(fct,"M3D_set_robot_radius")==0)) {
      if(!p3d_read_string_double(&pos,1,&dval)) return(READ_ERROR());
      p3d_set_robot_radius(dval);
      PrintInfo(("  Robot %s: radius %f loaded\n", 
	     p3d_get_desc_curname(P3D_ROBOT), dval));
      continue;
    }

    if((strcmp(fct,"p3d_set_robot_goto")==0) ||
       (strcmp(fct,"M3D_set_robot_goto")==0)) {
      if(!p3d_read_string_line_double(&pos,&n,&dtab,&size_max_dtab))
	{ return(READ_ERROR()); }
      nb_dof = p3d_get_robot_ndof();
      /* check the number of values specified */
      if(n>nb_dof){
        PrintWarning(("Function: %s, in line %i",fct,num_line));
	PrintWarning((": too many values specified, last ones are ignored\n"));
	n = nb_dof;
      }
      if (n<nb_dof){
        PrintWarning(("Function: %s, in line %i",fct,num_line));
	PrintWarning((": too few values specified, last ones are set to 0\n"));
	if (size_max_dtab<nb_dof)
	  { size_max_dtab=p3d_increase_size_double(&dtab,size_max_dtab,nb_dof); }
	if (size_max_dtab<nb_dof) {
	  PrintError(("p3d_rw_scenario: read_scenario: can't allocate memory !\n"));
	  return(READ_ERROR());
	}
	for (i=n; i<nb_dof; i++)
	  { dtab[i]=0; }
	n = nb_dof;
      }
      p3d_set_ROBOT_GOTO_deg_to_rad(dtab);
      PrintInfo(("  Robot %s: Goto position loaded\n",
	     p3d_get_desc_curname(P3D_ROBOT)));
      continue;
    }

    if((strcmp(fct,"p3d_set_robot_current")==0) ||
       (strcmp(fct,"M3D_set_robot_current")==0)) {
      if(!p3d_read_string_line_double(&pos,&n,&dtab,&size_max_dtab))
	{ return(READ_ERROR()); }
      nb_dof = p3d_get_robot_ndof();
      /* check the number of values specified */
      if(n>nb_dof){
        PrintWarning(("Function: %s, in line %i",fct,num_line));
	PrintWarning((": too many values specified, last ones are ignored\n"));
	n = nb_dof;
      }
      if (n<nb_dof){
        PrintWarning(("Function: %s, in line %i",fct,num_line));
	PrintWarning((": too few values specified, last ones are set to 0\n"));
	if (size_max_dtab<nb_dof)
	  { size_max_dtab=p3d_increase_size_double(&dtab,size_max_dtab,nb_dof); }
	if (size_max_dtab<nb_dof) {
	  PrintError(("p3d_rw_scenario: read_scenario: can't allocate memory !\n"));
	  return(READ_ERROR());
	}
	for (i=n; i<nb_dof; i++)
	  { dtab[i]=0; }
	n = nb_dof;
      }      
      p3d_set_ROBOT_START_deg_to_rad(dtab);
      PrintInfo(("  Robot %s: Current position loaded\n",
	     p3d_get_desc_curname(P3D_ROBOT)));
      continue;
    }

    if((strcmp(fct,"p3d_set_robot_config")==0) ||
       (strcmp(fct,"M3D_set_robot_config")==0)) {
      if(!p3d_read_string_name(&pos,&name))   return(READ_ERROR());
      if(!p3d_read_string_line_double(&pos,&n,&dtab,&size_max_dtab))
	{ return(READ_ERROR()); }
      nb_dof = p3d_get_robot_ndof();
      /* check the number of values specified */
      if(n>nb_dof){
        PrintWarning(("Function: %s, in line %i",fct,num_line));
	PrintWarning((": too many values specified, last ones are ignored\n"));
	n = nb_dof;
      }
      if (n<nb_dof){
        PrintWarning(("Function: %s, in line %i",fct,num_line));
	PrintWarning((": too few values specified, last ones are set to 0\n"));
	if (size_max_dtab<nb_dof)
	  { size_max_dtab=p3d_increase_size_double(&dtab,size_max_dtab,nb_dof); }
	if (size_max_dtab<nb_dof) {
	  PrintError(("p3d_rw_scenario: read_scenario: can't allocate memory !\n"));
	  return(READ_ERROR());
	}
	for (i=n; i<nb_dof; i++)
	  { dtab[i]=0; }
	n = nb_dof;
      }
      p3d_set_robot_config_deg_to_rad(name, dtab);
      PrintInfo(("  Robot %s: Configuration %s loaded\n", 
	     p3d_get_desc_curname(P3D_ROBOT), name));
      continue;
    }

    if((strcmp(fct,"p3d_set_robot_steering_method")==0) ||
       (strcmp(fct,"M3D_set_robot_steering_method")==0)) {
      if(!p3d_read_string_name(&pos,&name))  return(READ_ERROR());
      p3d_set_robot_steering_method(name);
      PrintInfo(("  Robot %s: Steering method %s\n",
	     p3d_get_desc_curname(P3D_ROBOT), name));
      continue;
    }

    if((strcmp(fct,"p3d_set_robot_graph")==0) ||
       (strcmp(fct,"M3D_set_robot_graph")==0)) {
      if(!p3d_read_string_name(&pos,&name))  return(READ_ERROR());
      if (p3d_read_graph(name)) {
	PrintInfo(("  Robot %s: Graph %s loaded\n", 
	       p3d_get_desc_curname(P3D_ROBOT), name)); 
      }
      continue;
    }

    if((strcmp(fct,"p3d_set_robot_traj")==0) ||
       (strcmp(fct,"M3D_set_robot_traj")==0)) {
      if(!p3d_read_string_name(&pos,&name))  return(READ_ERROR());
      if (p3d_read_traj(name)) {
	PrintInfo(("  Robot %s: Trajectory %s loaded\n", 
	       p3d_get_desc_curname(P3D_ROBOT), name));
      }
      continue;
    }

    if(strcmp(fct,"p3d_constraint")==0) { 
      rwc = 1;
      if(!p3d_read_string_name(&pos,&name))  return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum))  return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[0],&itab1,&size_max_itab1))
	return(READ_ERROR());      
      if(!p3d_read_string_int(&pos,1,argnum+1))  return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[1],&itab2,&size_max_itab2))
	return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum+2))  return(READ_ERROR());
      if(!p3d_read_string_n_double(&pos,argnum[2],&dtab,&size_max_dtab))
	return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum+3))  return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[3],&itab3,&size_max_itab3))
	return(READ_ERROR());
      if(!p3d_constraint(name, argnum[0], itab1, argnum[1], itab2,
		     argnum[2], dtab, argnum[3], itab3, -1, 1)) return(READ_ERROR());
      continue;
    }

    if(strcmp(fct,"p3d_constraint_dof")==0) { 
      rwc = 1;
      if(!p3d_read_string_name(&pos,&name))  return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum))  return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[0],&itab1,&size_max_itab1))
	return(READ_ERROR());      
      if(!p3d_read_string_n_int(&pos,argnum[0],&itab4,&size_max_itab4))
	return(READ_ERROR());      
      if(!p3d_read_string_int(&pos,1,argnum+1))  return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[1],&itab2,&size_max_itab2))
	return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[1],&itab5,&size_max_itab5))
	return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum+2))  return(READ_ERROR());
      if(!p3d_read_string_n_double(&pos,argnum[2],&dtab,&size_max_dtab))
	return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum+3))  return(READ_ERROR());
      if(!p3d_read_string_n_int(&pos,argnum[3],&itab3,&size_max_itab3))
	return(READ_ERROR());
      if(!p3d_read_string_int(&pos,1,argnum+4))  return(READ_ERROR());
      p3d_constraint_dof(name, argnum[0], itab1, itab4,
			 argnum[1], itab2, itab5,
			 argnum[2], dtab, argnum[3], itab3, -1, argnum[4]);
      continue;
    }
  if (strcmp(fct, "p3d_set_cntrt_Tatt") == 0) {
      p3d_rob *robot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
      if (!p3d_read_string_int(&pos, 1, argnum)) return(read_desc_error(fct));
      if (!p3d_read_string_n_double(&pos, 12, &dtab, &size_max_dtab)) return(read_desc_error(fct));
      if (robot->cntrt_manager->ncntrts < argnum[0] ) return(read_desc_error(fct));
      p3d_set_cntrt_Tatt(argnum[0], dtab);
      continue;
    }
#ifdef LIGHT_PLANNER
    if (strcmp(fct, "p3d_set_object_base_and_arm_constraints") == 0) {
      p3d_rob *robot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
      robot = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
      if (!robot) return(read_desc_error(fct));
      if (!p3d_read_string_int(&pos, 4, argnum)) return(read_desc_error(fct)); //joints for the object, the base and the closedChain constraints
      robot->objectJnt = robot->joints[argnum[0]];
      robot->baseJnt = robot->joints[argnum[1]];
      robot->relativeZRotationBaseObject = DTOR(argnum[2]);
      if(robot->nbCcCntrts != 0){
        MY_FREE(robot->ccCntrts, p3d_cntrt* ,robot->nbCcCntrts);
      }
      robot->nbCcCntrts = argnum[3];
      if (!p3d_read_string_int(&pos, robot->nbCcCntrts, argnum)) return(read_desc_error(fct)); //closedChain contraint ids
      robot->ccCntrts = MY_ALLOC(p3d_cntrt*, robot->nbCcCntrts);
      for(int i = 0; i < robot->nbCcCntrts; i++){
        if(argnum[i] < robot->cntrt_manager->ncntrts){
          robot->ccCntrts[i] = robot->cntrt_manager->cntrts[argnum[i]];
        }else{
          read_desc_error(fct);
        }
      }
      continue;
    }
#endif
  }

  if (size_max_dtab>0) {
    MY_FREE(dtab, double, size_max_dtab);
    dtab = NULL;
  }
  if (size_max_itab1>0) {
    MY_FREE(itab1, int, size_max_itab1);
    itab1 = NULL;
  }
  if (size_max_itab2>0) {
    MY_FREE(itab2, int, size_max_itab2);
    itab2 = NULL;
  }
  if (size_max_itab3>0) {
    MY_FREE(itab3, int, size_max_itab3);
    itab3 = NULL;
  }
  if (size_max_itab4>0) {
    MY_FREE(itab4, int, size_max_itab4);
    itab4 = NULL;
  }
  if (size_max_itab5>0) {
    MY_FREE(itab5, int, size_max_itab5);
    itab5 = NULL;
  }
  p3d_sel_desc_num(P3D_ROBOT, rcur);
  if(rwc) {
    p3d_col_deactivate_cntrt_pairs();
    p3d_update_all_jnts_state(1);    
  }
  return(TRUE);
}

/***************************************************************/
/* Add roboPt information to fdest  */
static void save_robot_data(FILE * fdest, pp3d_rob robotPt){
  int i, j, nb_dof;
  configPt q;
  lm_reeds_shepp_str *rs_paramPt=NULL;
  p3d_cntrt * cntrtPt;

  nb_dof = robotPt->nb_dof;
  fprintf(fdest, "\n\np3d_sel_desc_name P3D_ROBOT %s\n\n", robotPt->name);
  fprintf(fdest, "p3d_set_robot_steering_method %s\n", 
	  p3d_local_getname_planner(robotPt->lpl_type));

  rs_paramPt = lm_get_reeds_shepp_lm_param(robotPt);
  if (rs_paramPt != NULL){
    fprintf(fdest, "p3d_set_robot_radius %f\n", rs_paramPt->radius);
  }
  if ((robotPt->GRAPH!=NULL) && (robotPt->GRAPH->file!=NULL))
    { fprintf(fdest, "p3d_set_robot_graph %s\n", robotPt->GRAPH->file); }

  fprintf(fdest, "\np3d_set_robot_current");
  q = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_POS);
  for(j=0; j<nb_dof; j++)
    { fprintf(fdest, " %f", q[j]); }
  fprintf(fdest, "\n");
  p3d_destroy_config(robotPt, q);
  fprintf(fdest, "p3d_set_robot_goto");
  q = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO);
  for(j=0; j<nb_dof; j++)
    { fprintf(fdest, " %f", q[j]); }
  fprintf(fdest, "\n");
  p3d_destroy_config(robotPt, q);
  for(i=0; i<robotPt->nconf; i++) {
    fprintf(fdest, "p3d_set_robot_config %s", robotPt->conf[i]->name);
    q = p3d_copy_config_rad_to_deg(robotPt, robotPt->conf[i]->q);
    for(j=0; j<nb_dof; j++)
      { fprintf(fdest, " %f", q[j]); }
    fprintf(fdest, "\n");
    p3d_destroy_config(robotPt, q);
  }
  fprintf(fdest,"\n");
  for(i=0; i<robotPt->nt; i++) {
    if (robotPt->t[i]->file != NULL) 
      { fprintf(fdest, "p3d_set_robot_traj %s\n", robotPt->t[i]->file); }
  }
  fprintf(fdest,"\n");
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for(i=0;i<robotPt->cntrt_manager->ncntrts;i++) {
      cntrtPt = robotPt->cntrt_manager->cntrts[i];
      fprintf(fdest, "p3d_constraint %s", cntrtPt->namecntrt);
      fprintf(fdest, " %d", cntrtPt->npasjnts);
      for(j = 0; j < cntrtPt->npasjnts; j++){
        fprintf(fdest, " %d", cntrtPt->pasjnts[j]->num);
      }
      fprintf(fdest, " %d", cntrtPt->nactjnts);
      for(j=0; j<(cntrtPt->nactjnts); j++){
        fprintf(fdest, " %d", cntrtPt->actjnts[j]->num);
      }
      fprintf(fdest, " %d", cntrtPt->ndval);
      for(j=0; j<(cntrtPt->ndval); j++){
        fprintf(fdest, " %f", cntrtPt->argu_d[j]);
      }
      fprintf(fdest, " %d", cntrtPt->nival);
      for(j=0; j<(cntrtPt->nival); j++){
        fprintf(fdest, " %d", cntrtPt->argu_i[j]);
      }
      if(!p3d_mat4IsEqual(cntrtPt->Tatt, p3d_mat4IDENTITY)){
        fprintf(fdest, "\np3d_set_cntrt_Tatt %d", cntrtPt->num);
        for(j=0; j < 3; j++){
          for(int k = 0; k < 4; k++){
            fprintf(fdest, " %f", cntrtPt->Tatt[j][k]);
          }
        }
      }
      fprintf(fdest,"\n");
    }
  }
#ifdef LIGHT_PLANNER
  if(robotPt->baseJnt && robotPt->objectJnt) {
    fprintf(fdest, "p3d_set_object_base_and_arm_constraints %d %d %d %d", robotPt->objectJnt->num, robotPt->baseJnt->num, (int)RTOD(robotPt->relativeZRotationBaseObject), robotPt->nbCcCntrts);
    for(j = 0; j < robotPt->nbCcCntrts; j++){
      fprintf(fdest, " %d", robotPt->ccCntrts[j]->num);
    }
    fprintf(fdest,"\n");
  }
#endif
}


/***************************************************************/
/* Write some robot data in file */     
static int save_scenario(FILE * f){
  int ir, nrob, rcur;
  pp3d_rob robotPt;
  time_t t = time(NULL);

  /* Header */
  fprintf(f,"#************************************************************\n");
  fprintf(f,"# Scenario of %s\n", p3d_get_desc_curname(P3D_ENV));
  fprintf(f,"#\n# date : %s", ctime(&t));
  fprintf(f,"#************************************************************\n");

  fprintf(f,"\np3d_sel_desc_name P3D_ENV %s\n\n", 
	  p3d_get_desc_curname(P3D_ENV));

  rcur = p3d_get_desc_curnum(P3D_ROBOT);
  nrob = p3d_get_desc_number(P3D_ROBOT);
  for(ir=0; ir<nrob; ir++) {
    p3d_sel_desc_num(P3D_ROBOT, ir);
    robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
    save_robot_data(f, robotPt);
  }
  p3d_sel_desc_num(P3D_ROBOT, rcur);
  return(TRUE);
}

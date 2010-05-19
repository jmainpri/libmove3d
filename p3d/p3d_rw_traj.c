#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif


/***************************************************************/
/***************************************************************/
/* Public functions of rw trajectory access */

static int read_trajectory(FILE * f);
static int save_trajectory(FILE * f, pp3d_traj trajPt);


/*------------------------------------------------------------*/
/* Read a trajectory file to the current robot,
   return FALSE in case of failure */
int p3d_read_traj(const char *file) {
  FILE *fdc;
  int ret;
  p3d_traj * traj;

  if (!(fdc = fopen(file, "r"))) {
    PrintError(("p3d_trajectory_read: can't open %s\n", file));
    return(FALSE);
  }
  fseek(fdc, 0, 0);

  ret = read_trajectory(fdc);
  if (ret) {
    traj = (p3d_traj*)p3d_get_desc_curid(P3D_TRAJ);
    traj->file = strdup(file);
  }
  fclose(fdc);
  return(ret);
}


/*------------------------------------------------------------*/
/* Save a trajectory file,
   return FALSE in case of failure */
int p3d_save_traj(const char *file, p3d_traj * traj) {
  FILE *fdc;
  int ret;

  if (!(fdc = fopen(file, "w"))) {
    PrintError(("p3d_trajectory_save: can't write %s\n", file));
    return(FALSE);
  }

  ret = save_trajectory(fdc, traj);
  if (ret) {
    if (traj->file != NULL) {
      free(traj->file);
    }
    traj->file = strdup(file);
  }
  fclose(fdc);
  return(ret);
}


/*------------------------------------------------------------*/
/* Return a type of curpos, and change curpos to point on the
   next name.
   Return FALSE if curpos is a void string. */
static int read_string_type(char ** curpos, p3d_localplanner_type *type) {
  char * name, *pos;

  pos = * curpos;
  if (!p3d_read_string_name(&pos, &name)) {
    return FALSE;
  }
#ifdef P3D_LOCALPATH
  *type = p3d_local_getid_planner(name);
  if (*type == P3D_NULL_OBJ) {
    return(FALSE);
  }
#endif
  *curpos = pos;
  return(TRUE);
}


/*------------------------------------------------------------*/
/* Check if the environment and the robot of the trajectory
   file are the current environment and the current robot.
   Find the name of the trajectory and begin its description.
   Return FALSE in case of failure. */
static int check_header(FILE * f, int * num_line) {
  char * pos, * name, * line = NULL;
  int size_max_line = 0;  /* dynamic management of line lenght */

  /*-------------------- Check the environment --------------------*/
  if ((size_max_line = p3d_read_line_next_function(f, &line, size_max_line,
                       num_line)) == 0) {
    PrintError(("p3d_trajectory: check_header: no function in the file !"));
    return FALSE;
  }
  pos = line;
  p3d_read_string_name(&pos, &name); /* with read_line_next_fonction before,
         p3d_read_string_name returns TRUE */
  if (strcmp(name, "p3d_env") != 0) {
    PrintError(("p3d_trajectory: check_header: the first function of the file must be p3d_env name !"));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  if ((!p3d_read_string_name(&pos, &name)) ||
      (strcmp(name, p3d_get_desc_curname(P3D_ENV)) != 0)) {
    PrintError(("p3d_trajectory: check_header: current environment no corresponding: %s != %s !", p3d_get_desc_curname(P3D_ENV), name));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  /*-------------------- Check the robot --------------------*/
  if ((size_max_line = p3d_read_line_next_function(f, &line, size_max_line,
                       num_line)) == 0) {
    PrintError(("p3d_trajectory: check_header: no function in the file !"));
    return FALSE;
  }
  pos = line;
  p3d_read_string_name(&pos, &name); /* with read_line_next_fonction before,
         p3d_read_string_name returns TRUE */
  if (strcmp(name, "p3d_rob") != 0) {
    PrintError(("p3d_trajectory: check_header: the second function of the file must be p3d_rob name !"));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  if ((!p3d_read_string_name(&pos, &name)) ||
      (strcmp(name, p3d_get_desc_curname(P3D_ROBOT)) != 0)) {
    PrintError(("p3d_trajectory: check_header: current robot no corresponding: %s != %s !", p3d_get_desc_curname(P3D_ROBOT), name));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }
  /*-------------------- Check the trajectory name --------------------*/
  if ((size_max_line = p3d_read_line_next_function(f, &line, size_max_line,
                       num_line)) == 0) {
    PrintError(("p3d_trajectory: check_header: no function in the file !"));
    return FALSE;
  }
  pos = line;
  p3d_read_string_name(&pos, &name); /* with read_line_next_fonction before,
         p3d_read_string_name returns TRUE */
  if ((strcmp(name, "p3d_traj") != 0) && (pos[0] != '\0')) {
    PrintError(("p3d_trajectory: check_header: the third function of the file must be p3d_traj name !"));
    MY_FREE(line, char, size_max_line);
    return FALSE;
  }

  p3d_beg_desc(P3D_TRAJ, pos);
  MY_FREE(line, char, size_max_line);
  return TRUE;
}


/*------------------------------------------------------------*/
/* static function to return errors for read_trajectory()*/

static int read_error(char*fct,
                      int num_line,
                      int size_max_line,
                      char **line,
                      int size_max_dtab,
                      double **dtab,
                      pp3d_rob robotPt,
                      configPt *qi,
                      configPt *qg,
                      int tcur,
                      p3d_localplanner_type lpl_cur) {
  PrintError(("p3d_trajectory: read_error: invalid function %s\n", fct));
  PrintError(("  In line %i:%s\n", num_line, *line));
  if (size_max_line > 0) {
    MY_FREE(*line, char, size_max_line);
    *line = NULL;
  }
  if (size_max_dtab > 0) {
    MY_FREE(*dtab, double, size_max_dtab);
    *dtab = NULL;
  }
  p3d_destroy_config(robotPt, *qi);
  p3d_destroy_config(robotPt, *qg);
  p3d_end_desc();
  p3d_del_traj((p3d_traj*)p3d_get_desc_curid(P3D_TRAJ));
  if (tcur != P3D_NULL_OBJ) {
    p3d_sel_desc_num(P3D_TRAJ, tcur);
  }
#ifdef P3D_LOCALPATH
  p3d_local_set_planner(lpl_cur);
#endif
  return FALSE;
}
/*------------------------------------------------------------*/

/*------------------------------------------------------------*/
/* Read a trajectory file to the current robot,
   return FALSE in case of failure */
static int read_trajectory(FILE *f) {
  char * pos, * fct, * line = NULL, *tmpLine = NULL;
  int size_max_line = 0;  /* dynamic management of line lenght */
  double *dtab = NULL;
  configPt qi, qg, qi_deg = NULL, qg_deg = NULL;
  int size_max_dtab = 0; /* dynamic management of tab lenght  */
  int num_line = 0;
  int tcur, *ikSol = NULL, ikSolFlag = TRUE;
  p3d_localplanner_type type, lpl_cur;
  pp3d_rob robotPt;
  p3d_localpath *localpathPt;

  /* Function to return errors */
#define READ_ERROR() (read_error(fct, num_line, size_max_line, &line, size_max_dtab, &dtab, robotPt, &qi, &qg, tcur, lpl_cur))


  /* Check the environment and the robot. Begin the description. */
  tcur = p3d_get_desc_curnum(P3D_TRAJ);
#ifdef P3D_LOCALPATH
  lpl_cur = p3d_local_get_planner();
#endif
  if (!check_header(f, &num_line)) {
    return FALSE;
  }
  robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  qi = p3d_alloc_config(robotPt);
  qg = p3d_alloc_config(robotPt);

  while (1){
    if(ikSolFlag){
      if((size_max_line = p3d_read_line_next_function(f, &line, size_max_line,&num_line)) == 0){
        break;
      }
    }
    pos = line;
    p3d_read_string_name(&pos, &fct); /* with p3d_read_line_next_function before,
        p3d_read_string_name returns TRUE */

    if (strcmp(fct, "p3d_add_localpath") == 0) {
      if (!read_string_type(&pos, &type))  return(READ_ERROR());

      /*---------- Initial configuration ----------*/
      if ((size_max_line =
            p3d_read_line_next_function(f, &line, size_max_line,
                                        &num_line)) == 0) {
        return(READ_ERROR());
      }
      if ((qi_deg =
             p3d_read_word_and_config(robotPt, line, (char*)"q_init", 1)) == NULL) {
        return(READ_ERROR());
      }
      p3d_convert_config_deg_to_rad(robotPt, qi_deg, &qi);

      /*---------- Final configuration ----------*/
      if ((size_max_line = p3d_read_line_next_function(f, &line,
                           size_max_line,
                           &num_line)) == 0) {
        return(READ_ERROR());
      }

      if ((qg_deg =
             p3d_read_word_and_config(robotPt, line, (char*)"q_end", 1)) == NULL) {
        return(READ_ERROR());
      }

      p3d_convert_config_deg_to_rad(robotPt, qg_deg, &qg);
      /*---------- ikSol ----------*/
      if ((size_max_line = p3d_read_line_next_function(f, &line,
                           size_max_line,
                           &num_line)) == 0) {
#ifdef P3D_LOCALPATH
        p3d_local_set_planner(type);
        localpathPt = p3d_local_planner_multisol(robotPt, qi, qg, NULL);
#endif
        if (localpathPt == NULL)
          return(READ_ERROR());
        p3d_add_desc_courbe(localpathPt);
        break;
      }
      tmpLine = strdup(line);
      if (!p3d_read_word_and_n_int(robotPt, line, (char*)"ikSol", &ikSol, robotPt->cntrt_manager->ncntrts)) {
        ikSolFlag = FALSE;
        strcpy(line, tmpLine);
        free(tmpLine);
        tmpLine = NULL;
        ikSol = NULL;
      }
#ifdef P3D_LOCALPATH
      p3d_local_set_planner(type);
      localpathPt = p3d_local_planner_multisol(robotPt, qi, qg, ikSol);
#endif
      if (localpathPt == NULL)
        return(READ_ERROR());
      p3d_add_desc_courbe(localpathPt);
    }
  }

  if (size_max_dtab > 0) {
    MY_FREE(dtab, double, size_max_dtab);
    dtab = NULL;
  }
  p3d_destroy_config(robotPt, qi);
  p3d_destroy_config(robotPt, qg);
  p3d_end_desc();
#ifdef P3D_LOCALPATH	
  p3d_local_set_planner(lpl_cur);
#endif
  return(TRUE);
}

//modif Mokhtar Not used
// static int read_trajectory_from_localpath_data(FILE *f)
// {
//   char * pos, * fct, * line = NULL;
//   int size_max_line = 0;  /* dynamic management of line lenght */
//   double *dtab = NULL;
//   configPt qi, qg;
//   int size_max_dtab = 0; /* dynamic management of tab lenght  */
//   int num_line = 0;
//   int tcur;
//   char *type, lpl_cur=0;
//   pp3d_rob robotPt;
//   p3d_localpath *localpathPt;
//
//   /* Function to return errors */
//   #define READ_ERROR() (read_error(fct, num_line, size_max_line, &line, size_max_dtab, &dtab, robotPt, &qi, &qg, tcur, lpl_cur))
//
//
//   /* Check the environment and the robot. Begin the description. */
//   tcur = p3d_get_desc_curnum(P3D_TRAJ);
//   if (!check_header(f, &num_line))
//     { return FALSE; }
//   robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
//
//   while((size_max_line = p3d_read_line_next_function(f, &line, size_max_line,
//            &num_line)) != 0) {
//     pos = line;
//     p3d_read_string_name(&pos, &fct); /* with p3d_read_line_next_function before,
//         p3d_read_string_name returns TRUE */
//
//     if(strcmp(fct,"p3d_add_localpath")==0) {
//       if(!p3d_read_string_name(&pos,&type))  return(READ_ERROR());
//
//       localpathPt = p3d_read_localpath(robotPt, f, type, p3d_version);
//       if (localpathPt == NULL)
//  return(READ_ERROR());
//       p3d_add_desc_courbe(localpathPt);
//       continue;
//     }
//   }
//
//   if (size_max_dtab>0) {
//     MY_FREE(dtab, double, size_max_dtab);
//     dtab = NULL;
//   }
//   p3d_end_desc();
//   return(TRUE);
// }

/***************************************************************/
/* Add a Localpath information to fdest  */
static void save_localpath_data(FILE * fdest, pp3d_rob robotPt,
                                pp3d_localpath localpathPt) {
  configPt q, q_deg;
	
#ifdef P3D_LOCALPATH
  fprintf(fdest, "\n\np3d_add_localpath %s\n",
          p3d_local_getname_planner(robotPt->lpl_type));
#endif
	
  /* write initial configuration */
  q = localpathPt->config_at_distance(robotPt, localpathPt, 0);
  q_deg = p3d_copy_config_rad_to_deg(robotPt, q);

  fprintf(fdest, "    ");
  p3d_write_word_and_config(robotPt, fdest, (char*)"q_init", q_deg);

  p3d_destroy_config(robotPt, q);
  p3d_destroy_config(robotPt, q_deg);

  /* write end configuration */
  q = localpathPt->config_at_distance(robotPt, localpathPt,
                                      localpathPt->length_lp);
  q_deg = p3d_copy_config_rad_to_deg(robotPt, q);

  fprintf(fdest, "    ");
  p3d_write_word_and_config(robotPt, fdest, (char*)"q_end", q_deg);

  p3d_destroy_config(robotPt, q);
  p3d_destroy_config(robotPt, q_deg);
  if (localpathPt->ikSol) {
    fprintf(fdest, "    ");
    p3d_write_word_and_n_int(fdest, (char*)"ikSol", localpathPt->ikSol, robotPt->cntrt_manager->ncntrts);
  }
}

/***************************************************************/
/* Write some robot data in file */
static int save_trajectory(FILE * f, pp3d_traj trajPt) {
  pp3d_rob robotPt;
  pp3d_localpath localpathPt;
  time_t t = time(NULL);

  robotPt = trajPt->rob;
  localpathPt = trajPt->courbePt;

  /* Header */
  fprintf(f, "#************************************************************\n");
  fprintf(f, "# Trajectory %s of %s\n", trajPt->name, robotPt->name);
  fprintf(f, "#\n# date : %s", ctime(&t));
  fprintf(f, "#************************************************************\n");

  fprintf(f, "\np3d_env %s\n", p3d_get_desc_curname(P3D_ENV));
  fprintf(f, "p3d_rob %s\n", robotPt->name);
  fprintf(f, "p3d_traj %s\n", trajPt->name);

  while (localpathPt != NULL) {
    save_localpath_data(f, robotPt, localpathPt);
    localpathPt = localpathPt->next_lp;
  }
  return(TRUE);
}

/***************************************************************/
int save_trajectory_without_header(FILE * f, pp3d_traj trajPt) {
  pp3d_rob robotPt;
  pp3d_localpath localpathPt;

  robotPt = trajPt->rob;
  localpathPt = trajPt->courbePt;

  while (localpathPt != NULL) {
    save_localpath_data(f, robotPt, localpathPt);
    localpathPt = localpathPt->next_lp;
  }
  return(TRUE);
}


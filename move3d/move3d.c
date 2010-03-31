#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif
#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#include "Move3d-pkg.h"
#include "Bio-pkg.h"
#include "UserAppli-pkg.h"
#include "UdpClient.h"

#include "locale.h"
#include "molecule.xpm"

#ifdef QT_GL
#include "qtWindow/cppToQt.hpp"
#endif
#ifdef LIGHT_PLANNER
#include "../lightPlanner/proto/DlrPlanner.h"
#include "../lightPlanner/proto/DlrParser.h"
#endif


static int FILTER_TO_BE_SET_ACTIVE = FALSE;
#ifdef WITH_XFORMS
Pixmap ApplicationIcon = 0;
#endif

//extern void g3d_create_main_form(void);
//extern void g3d_loop(void);
//extern void p3d_col_set_mode(int);
//extern void kcd_set_user_defined_small_volume(double);
//extern double p3d_get_env_dmax(void);
static void use(void);
#ifdef WITH_XFORMS
Pixmap GetApplicationIcon();
#endif

#if defined QT_LIBRARY || BioMove3D_EXPORTS
int mainMhp(int argc, char ** argv) {
#else
int main(int argc, char ** argv) {
#endif

  // modif Pepijn apropos dmax and tol
  int user_dmax_to_be_set = FALSE;  /* Modif. Pepijn on dmax */
  double user_dmax = 0.0;     /* Modif. Pepijn on dmax */
  int user_obj_tol_to_be_set = FALSE;  /* Modif. Carl on tolerance */
  double user_obj_tol = 0.0;
  // end modif Pepijn

  double user_volume, user_size;   /* Modif. Carl on volume */
#ifndef BIO
  const char *file;
#endif
  char file_directory[200];
  char filename[200];
  char scenario[200];
	char dlrReadFile[200];
	char dlrSaveFile[200];
  int i = 0;
  /* carl: */
  int seed_set = FALSE;
  int dir_set = FALSE;
  int file_set = FALSE;
  int scenario_set = FALSE;
  int col_det_set = FALSE;
  int col_mode_to_be_set = p3d_col_mode_kcd; /* Nic p3d_col_mode_v_collide;*/
  // init English C
  if (! setlocale(LC_ALL, "C"))
    fprintf(stderr, "There was an error while setting the locale to \"C\"\n");

  // modif Brice SALVA

#ifdef BIO
  int usrAnswer = FALSE;
  file_name_list* file_list = NULL;
  char name[PSF_MAX_NAME_LENGTH];
  int is_p3d = FALSE;
#endif
  if (! setlocale(LC_ALL, "C")) fprintf(stderr, "There was an error while setting the locale to \"C\"\n");

  // End modif Brice SALVA

  /* lecture des arguments */
  /* carl: */
  i = 1;
  while (i < argc) {
    if (strcmp(argv[i], "-debug") == 0) {
      basic_alloc_debugon();
      i++;
    } else if (strcmp(argv[i], "-d") == 0) {
      ++i;
      if ((i < argc)) {
        strcpy(file_directory, argv[i]);
        dir_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-f") == 0) {
      ++i;
      if ((i < argc)) {
        strcpy(filename, argv[i]);
        file_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-sc") == 0) {
      ++i;
      if ((i < argc)) {
        strcpy(scenario, argv[i]);
        scenario_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    }
#ifdef P3D_COLLISION_CHECKING
	else if (strcmp(argv[i], "-o") == 0) {
      set_collision_by_object(FALSE);
      ++i;
    } else if (strcmp(argv[i], "-x") == 0) {
      FILTER_TO_BE_SET_ACTIVE = TRUE;
      ++i;
    } else if (strcmp(argv[i], "-nkcdd") == 0) {
      set_return_kcd_distance_estimate(FALSE);
      ++i;
    }
#endif
	else if (strcmp(argv[i], "-s") == 0) {
      ++i;
      if ((i < argc)) {
#ifdef P3D_PLANNER
        p3d_init_random_seed(atoi(argv[i]));
        seed_set = TRUE;
#endif
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-v") == 0) {
      ++i;
      if (i < argc) {

        user_size = atof(argv[i]);
        user_volume = user_size * user_size * user_size;
#ifdef P3D_COLLISION_CHECKING
        kcd_set_user_defined_small_volume(user_volume);
#endif
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-vol") == 0) {
      ++i;
      if (i < argc) {
#ifdef P3D_COLLISION_CHECKING
        kcd_set_user_defined_small_volume(atof(argv[i]));
#endif
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-tol") == 0) {
      ++i;
      if (i < argc) {
        user_obj_tol = atof(argv[i]) ;
        user_obj_tol_to_be_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-dmax") == 0) {
      ++i;
      if (i < argc) {
        user_dmax = atof(argv[i]) ;
        user_dmax_to_be_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-stat") == 0) {
      ++i;
#ifdef P3D_PLANNER
      enableStats();
#endif
    } else if (strcmp(argv[i], "-udp") == 0) {
      std::string serverIp(argv[i+1]);
      int port = 0;
      sscanf(argv[i+2], "%d", &port);
      globalUdpClient = new UdpClient(serverIp, port);
      i += 3;
    } else if (strcmp(argv[i], "-dlr") == 0) {
			strcpy(dlrReadFile, argv[i + 1]);
			strcpy(dlrSaveFile, argv[i + 2]);
      i += 3;
    }	else if (strcmp(argv[i], "-c") == 0) {
      ++i;
      if (strcmp(argv[i], "vcollide") == 0) {
        col_mode_to_be_set = p3d_col_mode_v_collide;
        col_det_set = TRUE;
        ++i;
      }
#ifdef P3D_COLLISION_CHECKING
	  else if (strcmp(argv[i], "kcd") == 0) {
        col_mode_to_be_set = p3d_col_mode_kcd;
        set_DO_KCD_GJK(TRUE);
        col_det_set = TRUE;
        ++i;
      }
#ifdef PQP
	else if (strcmp(argv[i], "pqp") == 0) {
    	printf("Colmod pqp");
    	col_mode_to_be_set= p3d_col_mode_pqp;
    	col_det_set = TRUE;
	++i;
      }
#endif
#endif
 else if (strcmp(argv[i], "bio") == 0) {
        col_mode_to_be_set = p3d_col_mode_bio;
        col_det_set = TRUE;
        ++i;
      } else if (strcmp(argv[i], "kng") == 0) {
        col_mode_to_be_set = p3d_col_mode_kcd;
#ifdef P3D_COLLISION_CHECKING
        set_DO_KCD_GJK(FALSE);
#endif
        col_det_set = TRUE;
        ++i;
      } else if (strcmp(argv[i], "gjk") == 0) {
        col_mode_to_be_set = p3d_col_mode_gjk;
        col_det_set = TRUE;
        ++i;
      } else if (strcmp(argv[i], "none") == 0) {
        col_mode_to_be_set = p3d_col_mode_none;
        col_det_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else {
      use();
      return 0;
    }
  }

  if (!dir_set)
    strcpy(file_directory, "../../demo");
#ifdef P3D_PLANNER
  if (!seed_set)
    p3d_init_random();
#endif
  if (!col_det_set){
    // modif Juan

  //check that the HOME_MOVE3D environment variable exists:
  if(getenv("HOME_MOVE3D")==NULL) {
   printf("%s: %d: main(): The environment variable \"HOME_MOVE3D\" is not defined. This might cause some problems or crashes (e.g. with video capture).\n", __FILE__,__LINE__);
  }


#ifdef GRASP_PLANNING
  col_mode_to_be_set= p3d_col_mode_pqp;
  #ifndef PQP
  printf("%s: %d: main(): GRASP_PLANNING must be compiled with PQP.\n", __FILE__,__LINE__);
  printf("Program must quit.\n");
  return 1;
  #endif
#else
    col_mode_to_be_set = p3d_col_mode_kcd;
#endif
  }
#ifdef P3D_COLLISION_CHECKING
  if (col_mode_to_be_set != p3d_col_mode_v_collide)
    set_collision_by_object(FALSE);
  /* : carl */
  if (col_mode_to_be_set == p3d_col_mode_v_collide) {
    p3d_filter_switch_filter_mechanism(FILTER_TO_BE_SET_ACTIVE);
  }
  /* begin added KCD FILTER */
  else if (col_mode_to_be_set == p3d_col_mode_kcd) {
    p3d_filter_switch_filter_mechanism(FILTER_TO_BE_SET_ACTIVE);
  }
#endif
  /*  end  added KCD FILTER */


  /* lecture du fichier environnement */
  p3d_set_directory(file_directory);
#ifdef WITH_XFORMS
  fl_initialize(&argc, argv, "FormDemo", 0, 0);
  fl_set_border_width(1);
#endif
  // init English C
  if (! setlocale(LC_ALL, "C"))
    fprintf(stderr, "There was an error while setting the locale to \"C\"\n");


  while (!p3d_get_desc_number(P3D_ENV)) {

#ifdef BIO
    if (file_set == TRUE) {
//       if (!filename) {
//         exit(0);
//       }

#ifndef PQP
      p3d_col_set_mode(p3d_col_mode_none);
#else
      p3d_col_set_mode(col_mode_to_be_set);
#endif
      p3d_BB_set_mode_close();
      if (!p3d_read_desc(filename)) {
        if (fl_show_question("ENV file not found! Exit?\n", 1)) {
          exit(0);
        } else {
          file_set = FALSE;
        }
      }
    }
    if (file_set == FALSE) {
      // Modif Brice SALVA
      file_list = init_file_name_list();
      create_file_selector_Form();
      usrAnswer = do_file_selector_Form(file_directory, file_list, name, PSF_MAX_NAME_LENGTH - 1, &is_p3d);
      if (usrAnswer) {
        if (is_p3d) {
          // only one file
          p3d_read_desc((char*)file_list->name_list[0]);
        } else {
          if (!psf_make_p3d_from_multiple_pdb(file_list, name)) {
            fl_show_alert("Can't perform PDB to P3D traduction", "", "", 1);
            free_file_name_list(file_list);
            file_list = NULL;
            exit(0);
          }
        }
      } else {
        free_file_name_list(file_list);
        file_list = NULL;
        exit(0);
      }

      free_file_name_list(file_list);
      file_list = NULL;
    }
#else

    if (file_set == TRUE) {
      file = filename;
    } else {
#ifdef WITH_XFORMS
      file = fl_show_fselector("P3D_ENV filename", file_directory, "*.p3d", "");
#endif
    }
    if (!file) {
      exit(0);
    }
#ifdef P3D_COLLISION_CHECKING
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_BB_set_mode_close();
#endif
	//printf("Reading desc %s",file);
    p3d_read_desc((char *) file);
#endif

    if (!p3d_get_desc_number(P3D_ENV)) {
	printf("loading done...\n");
#ifdef WITH_XFORMS
      if (fl_show_question("Can't read a P3D_ENV from this file! Exit?\n", 1)) {
        exit(0);
      }
#endif
    }
  }

  MY_ALLOC_INFO("After p3d_read_desc");

  printf("Nb poly : %d\n", p3d_poly_get_nb());

  /* for start-up with currently chosen collision detector: */
  /* MY_ALLOC_INFO("Before initialization of a collision detector"); */
#ifdef P3D_COLLISION_CHECKING
  p3d_col_set_mode(col_mode_to_be_set);
  p3d_col_start(col_mode_to_be_set);
#endif

  /* modif Pepijn april 2001
    * this changes have to be made after the initialistion of the collision checker
    * because in p3d_col_start KCD migth be initialised, and during this initialisation
    * there is automaticly calculated a dmax
    * So if the user wants to set his own dmax this most be done after this initialisation
    * INTERNAL NOTE: in this case the users are the developpers of MOVE3D, normally the clients
    * who purchase Move3d don't know that this option exists
    */
  if (user_dmax_to_be_set) {
    if (user_dmax < EPS4) {
      printf("WARNING: User chose dmax too small --> new value set to 0.0001 (EPS4)\n");
      user_dmax = EPS4;
    }
    p3d_set_env_dmax(user_dmax);
  }
  if (user_obj_tol_to_be_set) {
    if (user_obj_tol < 0.0) {
      printf("WARNING: Negative tolerance, tolerance is set to 0.0\n");
      user_obj_tol = 0.0;
    }
    p3d_set_env_object_tolerance(user_obj_tol);
  }

  printf("Env dmax = %f\n",p3d_get_env_dmax());
  printf("Env Object tol = %f\n",p3d_get_env_object_tolerance());
  /* always set tolerance even if the user didn't specify any options
   * it's possible that Kcd has calculated automaticly a dmax
   */
  /* Normally  p3d_col_set_tolerance(); is called when initialising
   * the sliders, in case the sliders are not used we have to use
   * p3d_col_set_tolerance()
   */

  printf("MAX_DDLS  %d\n", MAX_DDLS);




  // modif Juan
#ifdef BIO
  if (col_mode_to_be_set == p3d_col_mode_bio) {
    bio_set_num_subrobot_AA();
    bio_set_num_subrobot_ligand();
    bio_set_bio_jnt_types();
    bio_set_bio_jnt_AAnumbers();
    bio_set_list_AA_first_jnt();
    bio_set_AAtotal_number();
    bio_set_nb_flexible_sc();
    bio_set_list_firstjnts_flexible_sc();
    if (XYZ_ROBOT->num_subrobot_ligand != -1)
      bio_set_nb_dof_ligand();
  }
#endif
  // fmodif Juan

  #if defined(LIGHT_PLANNER) && defined(FK_CNTRT)
   for(int i=0; i<XYZ_ENV->nr; i++)
    {  p3d_create_FK_cntrts(XYZ_ENV->robot[i]);  }
  #endif

#ifdef P3D_CONSTRAINTS
  // Modif Mokhtar Initialisation For Multisolutions constraints
  p3d_init_iksol(XYZ_ROBOT->cntrt_manager);
#endif




  /* creation du FORM main */
#ifdef WITH_XFORMS
  g3d_create_main_form();
#endif
  /*
   * needs to be run after main form has been created
   */
  if (scenario_set == TRUE) {
#ifdef WITH_XFORMS
    read_scenario_by_name(scenario);
#endif
  }

  //Set the robots to initial Pos if defined
  for(i = 0; i < XYZ_ENV->nr; i++){
    if(!p3d_isNullConfig(XYZ_ENV->robot[i], XYZ_ENV->robot[i]->ROBOT_POS)){
      p3d_set_and_update_this_robot_conf(XYZ_ENV->robot[i], XYZ_ENV->robot[i]->ROBOT_POS);
    }
  }
	//Exection Of Dlr Planner
//	do{
//		DlrPlanner* planner = new DlrPlanner(dlrSaveFile);
//		DlrParser parser(dlrReadFile, planner);
//		if(parser.parse()){
//			planner->process();
//		}else{
//			sleep(2);
//		}
//		free(planner);
//	}while(1);
  /* go into loop */
#ifdef QT_GL
  sem->release();
#endif

//  double c, color[4];
//  srand(time(NULL));
//  c= rand()/((double)RAND_MAX+1);
//  g3d_rgb_from_hue(c, color);
//  g3d_set_win_floor_color(g3d_get_cur_win(), color[0], color[1], color[2]);

#ifdef WITH_XFORMS
 g3d_set_win_floor_color(g3d_get_cur_win(), 0.5, 1.0, 1.0);
//  g3d_set_win_bgcolor(g3d_get_cur_win(), 0.5, 0.6, 1.0);
 g3d_set_win_wall_color(g3d_get_cur_win(), 0.4, 0.45, 0.5);
 g3d_set_win_bgcolor(g3d_get_cur_win(), XYZ_ENV->background_color[0], XYZ_ENV->background_color[1], XYZ_ENV->background_color[2]);
  //p3d_print_env_info();

  g3d_loop();
#endif
  return 0;
}

#ifdef WITH_XFORMS
Pixmap GetApplicationIcon() {
  static unsigned int width, height;
  if (ApplicationIcon == 0)
    ApplicationIcon = fl_create_from_pixmapdata(fl_root, molecule_xpm, &width, &height, 0, 0, 0, 0);
  /*   Pixmap fl_create_from_pixmapdata(Window win, char **data,
      unsigned *width, unsigned *height,
      Pixmap *shape_mask,
      int *hotx, int *hoty, FL_COLOR tran) */
  return ApplicationIcon;
}
#endif

/* fonction de rappel des formats des arguments */
static void use(void) {

  printf("main : ERROR : wrong arguments !\n move3d -d data_directory -f file -sc scenario -s random_seed -c collision_detector -tol tolerance -dmax dist [-v size | -vol volume] -o -x -nkcdd\n");
  printf("collision_detector may be: icollide, vcollide, solid, kcd, kng, none\n");
  printf("(kng == kcd without gjk)\n");
  printf("-o: consider polyhedrons as objects (only for vcollide)\n");
  printf("-x: do filtering by robot body (default: don't filter)\n");
  printf("-v: (only with kcd) minimal relevant size (in 1 dimension) \n");
  printf("-dmax: for Vcollide & nkcdd  => maximum penetration distance \n");
  printf("           KCD               => artificial enlargement of the obstacles \n");
  printf("-tol: (only with kcd) object tolerance (enlarge objects for the collision checker) \n");
  printf("-vol: (only with kcd) minimal relevant size (volume) \n");
  printf("(internal note, kcd: volume of a box around a facet == (surface of box)^(3/2) )\n");
  printf("-nkcdd: (only with -c kcd) return only boolean reply, don't use distance \n");
}


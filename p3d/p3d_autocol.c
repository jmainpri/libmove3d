#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

typedef struct col_act {
  int    *** body_links;
  int      * described_robots;
  int        cur_rob_id;
  int        cur_env_id;
  int        nof_robot_for_struct;
  int       *nof_obj_for_robots;
} p3d_rob_col_activ, *pp3d_rob_col_activ;

static p3d_rob_col_activ * ROB_AUTOCOL = NULL;
static p3d_rob_col_activ ** ROB_AUTOCOL_ENVS = NULL;
static int NB_ROB_AUTOCOL_ENVS = 0;
static int DEBUG = 0;
static int DEEP2 = 0; //Flag to desactivate the grand father body in addition of the father

/*--------------------------------------------------------------------------*/
/*! \brief Create a structure to store the data of autocollisison.
 *
 * \note Modify ::ROB_AUTOCOL and ::ROB_AUTOCOL_ENVS.
 *
 * \internal
 */
static void s_p3d_autocol_create(void) {
  int i, nof_robots;

  nof_robots = XYZ_ENV->nr;
  if (DEBUG) {
    PrintInfo(("Memory allocation for ROB_AUTOCOL structure (%d robots) \n",
               nof_robots));
  }
  ROB_AUTOCOL = MY_ALLOC(p3d_rob_col_activ, 1);

  ROB_AUTOCOL->cur_env_id = XYZ_ENV->num;
  ROB_AUTOCOL->nof_robot_for_struct = nof_robots;

  ROB_AUTOCOL->described_robots   = MY_ALLOC(int,   nof_robots);
  ROB_AUTOCOL->body_links         = MY_ALLOC(int**, nof_robots);
  ROB_AUTOCOL->nof_obj_for_robots = MY_ALLOC(int,   nof_robots);

  for (i = 0; i < nof_robots; i++) {
    ROB_AUTOCOL->described_robots[i] = -1;
    ROB_AUTOCOL->nof_obj_for_robots[i] = XYZ_ENV->robot[i]->no;
    ROB_AUTOCOL->body_links[i] = NULL;
  }

  ROB_AUTOCOL_ENVS = MY_REALLOC(ROB_AUTOCOL_ENVS, pp3d_rob_col_activ,
                                NB_ROB_AUTOCOL_ENVS, NB_ROB_AUTOCOL_ENVS + 1);
  ROB_AUTOCOL_ENVS[NB_ROB_AUTOCOL_ENVS] = ROB_AUTOCOL;
  NB_ROB_AUTOCOL_ENVS ++;
}

void p3d_autocol_check_integrity(void) {
  int i;

  /*** ALLOCATION DE LA STRUCTURE AU PREMIER APPEL OU APRES DESALLOCATION ***/
  if (ROB_AUTOCOL == NULL) {
    s_p3d_autocol_create();
  }

  if (ROB_AUTOCOL->cur_env_id != XYZ_ENV->num) {
    /*** TEST SI L'ENVIRONMENT EST DEJA SAUVE ***/
    for (i = 0; i < NB_ROB_AUTOCOL_ENVS; i++) {
      if (ROB_AUTOCOL_ENVS[i]->cur_env_id == XYZ_ENV->num) {
        ROB_AUTOCOL = ROB_AUTOCOL_ENVS[i];
        return;
      }
    }
    /*** NOUVELLE ALLOCATION DE LA STRUCTURE ***/
    s_p3d_autocol_create();
  }
}

/*******************************************************/
/* Fonction d'initialisation de la structure d'activa- */
/* tion du collision checker des corps du robot        */
/* In : le nom du robot                                */
/* Out :                                               */
/* J. PETTRE 30/05/01                                  */
/* Resume :                                            */
/* Les listes des bodies a activer sont construites    */
/* comme ceci : (exemple robot 5 bodies) sur ce tableau*/
/* on visualise les deux entrees du tableau ( la toute */
/* premiere etant l'index du robot concerne ici )      */
/* ROB_AUTOCOL->active links, le body 5 n'est    */
/* pas verifie en collision avec 2, 4 avec 1, 3 est    */
/* tjrs detecte et 2 jamais, etc ... Pour eviter la    */
/* redondance d'information chaque body n'est renseigne*/
/* que sur les bodies de rang inferieur                */
/* X | 4  3  2  1  0                                   */
/* --+--------------                                   */
/* 5 | 1  1 -1  1  1                                   */
/* 4 |    1  1 -1  1                                   */
/* 3 |       1  1  1                                   */
/* 2 |         -1 -1                                   */
/* 1 |             1                                   */
/* 0 |                                                 */
/*******************************************************/
int p3d_init_rob_col_activ(char *name) {
  int i, j, nof_obj_for_rob;
  int to_init = TRUE;  /* added by Carl */

  if (DEBUG) basic_alloc_debugon();
  /*** VERIFIER SI STRUCTURE CORRECTEMENT ALLOUEE ***/
  p3d_autocol_check_integrity();

  /*** RECHERCHER LE NUM ID DU ROBOT DECRIT ***/
  ROB_AUTOCOL->cur_rob_id = p3d_get_rob_nid_by_name(name);
  if (ROB_AUTOCOL->cur_rob_id == -1) {
    PrintInfo(("ERROR IN INTER BODY COLLISION DESACTIVATION - Robot name doesn't exist\n"));
    return (-1);
  }
  if (DEBUG) PrintInfo(("Working with robot %s whose identifier is %d \n", name, ROB_AUTOCOL->cur_rob_id));

  /*** VERIFIER QU'IL N'EST PAS DEJA DECRIT ***/
  for (i = 0; i < ROB_AUTOCOL->nof_robot_for_struct; i++)
    if (ROB_AUTOCOL->described_robots[i] == ROB_AUTOCOL->cur_rob_id) {
      /* modified by Carl: */
      PrintWarning(("WARNING IN INTER BODY COLLISION DESACTIVATION - Robot redefinition\n"));
      to_init = FALSE;
      /* return (-2); */
      /* end modif. */
    }

  /* moved by Carl: */
  /*** MAKE THE ROBOT DESCRIBED THE CURRENT ROBOT IN STRUCT ENV ***/
  nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[ROB_AUTOCOL->cur_rob_id];

  if (to_init) { /* added by Carl */
    /*** PAS ENCORE DECRIT, ON NOTE QU'IL L'EST A PRESENT ***/
    i = 0;
    while (ROB_AUTOCOL->described_robots[i] != -1) {
      i++;
    }
    ROB_AUTOCOL->described_robots[i] = ROB_AUTOCOL->cur_rob_id;


    /*** ALLOCATION DES LISTES DES BODIES ACTIFS POUR CHAQUE BODY DU ROBOT ***/
    ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id]
    = MY_ALLOC(int*, nof_obj_for_rob);
    for (i = nof_obj_for_rob - 1; i > 0; i--)
      ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i] = MY_ALLOC(int, i);
    if (DEBUG) PrintInfo(("Allocation for activation lists (%d objects)\n", nof_obj_for_rob));
  }
  
  /*** INITIALISATION DES LISTES ***/
  for (i = nof_obj_for_rob - 1; i > 0; i--)
    for (j = 0; j < i; j++)
      ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][j] = 1;
  if (DEBUG) PrintInfo(("p3d_init_rob_col_activ ended \n"));
  if (DEBUG) basic_alloc_debugoff();
  /*** ET VOILA ***/

  return 0;
}

/*******************************************************/
/* Fonction de desactivation du collision checker sur  */
/* deux bodies d'un meme robot                         */
/* In : les noms des 2 bodies                          */
/* Out :                                               */
/* J. PETTRE 30/05/01                                  */
/*******************************************************/

int p3d_desactivate_col_check(char *name_body1, char *name_body2) {
  int body_index1, body_index2, anint;
  char buffer[255];

  /*** MAKE THE ROBOT DESCRIBED THE CURRENT ROBOT IN STRUCT ENV ***/

  /*** On cherche les indexes des deux bodies dans la structure robot ***/
  strcpy(buffer, XYZ_ENV->robot[ROB_AUTOCOL->cur_rob_id]->name);
  strcat(buffer, ".");
  strcat(buffer, name_body1);
  body_index1 = p3d_get_body_nid_by_name(buffer);
  strcpy(buffer, XYZ_ENV->robot[ROB_AUTOCOL->cur_rob_id]->name);
  strcat(buffer, ".");
  strcat(buffer, name_body2);
  body_index2 = p3d_get_body_nid_by_name(buffer);
  if (DEBUG) PrintInfo(("Desactivating the pair of bodies %s and %s whose identifiers are %d and %d \n", name_body1, name_body2, body_index1, body_index2));
  if (body_index2 == -1 || body_index1 == -1) {
    PrintError(("ERROR IN INTER BODY COLLISION DESACTIVATION - Body name not found\n"));
    return (-2);
  }

  /*** BODY_INDEX1 DOIT ETRE SUPERIEUR POUR ACCES CORRECT DANS LA LISTE ***/
  if (body_index1 < body_index2) {
    anint = body_index2;
    body_index2 = body_index1;
    body_index1 = anint;
  }

  /*** ON DESACTIVE LE LIEN DANS LA LISTE ***/
  ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][body_index1][body_index2] = -1;
  if(DEBUG){
    int nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[ROB_AUTOCOL->cur_rob_id];
    printf("   ");
    for (int i = 0; i < nof_obj_for_rob - 1 ; i++){
      printf("%d ", i%10);
    }
    int nbPair = 0;
    for (int i = nof_obj_for_rob - 1; i > 0; i--){
      printf("\n%d ", i);
      for (int j = 0; j < i; j++){
        if((((ROB_AUTOCOL->body_links)[ROB_AUTOCOL->cur_rob_id])[i])[j] > 0){
          printf("+ ");
          nbPair++;
        }else{
          printf("- ");
        }
      }
    }
    printf("\nnbPair = %d\n", nbPair);
  }
  return 0;
}

/*******************************************************/
/* Fonction de desactivation du collision checker      */
/* automatique                                         */
/* In : Desactiver les body grand pere egalement ou non*/
/* Out :                                               */
/* J. PETTRE 30/05/01                                  */
/*******************************************************/

int p3d_desactivate_col_check_automatic() {
  int i, j, father_body_found, grand_father_body_found = 0, body_index, nof_obj_for_rob;
  int *hierarchical_father, *graphicObjects = NULL;
	int deep2 = DEEP2;
  float epsilon = 0.1;
  p3d_jnt * ajntPt, *father_jnt = NULL;

  /*** MAKE THE ROBOT DESCRIBED THE CURRENT ROBOT IN STRUCT ENV ***/
  if (ROB_AUTOCOL->cur_rob_id < 0) {
    PrintInfo(("Aucun robot selectionne ! appelez p3d_add_desc_rob_col_init # NOM DU ROBOT # auparavant \n"));
    return (-3);
  }
  XYZ_ENV->cur_robot = XYZ_ENV->robot[ROB_AUTOCOL->cur_rob_id];
  /*** DETERMINATION DES COUPLES CANDIDATS A LA DESACTIVATION ***/
  nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[ROB_AUTOCOL->cur_rob_id];
  graphicObjects = MY_ALLOC(int, nof_obj_for_rob);
  /*** REGLE N*1 : DESACTIVER LES BODIES VIDES (SANS POLYHEDRE) et les P3D_GRAPHIC AVEC TOUS LES AUTRES BODIES. ***/
  for (i = 0;i < nof_obj_for_rob ; i++){
    if (XYZ_ENV->cur_robot->o[i]->np == 0) {
      /*** HERE WE KNOW THE BODY IS EMPTY ***/
      /*** DESACTIVATION DES PREDECESSEURS ***/
      for (j = 0; j < i; j++)
        ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][j] = -1;
      /*** DESACTIVATION DES SUCCESSEURS ***/
      for (j = nof_obj_for_rob - 1; j > i; j--)
        ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][j][i] = -1;
    }else{
      bool isPureGraphic = true;
      for(j = 0; j < XYZ_ENV->cur_robot->o[i]->np; j++){
        if(XYZ_ENV->cur_robot->o[i]->pol[j]->TYPE != P3D_GRAPHIC){
          isPureGraphic = false;
          break;
        }
      }
      if(isPureGraphic){
        graphicObjects[i] = 1;
        /*** DESACTIVATION DES PREDECESSEURS ***/
        for (j = 0; j < i; j++)
          ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][j] = -1;
        /*** DESACTIVATION DES SUCCESSEURS ***/
        for (j = nof_obj_for_rob - 1; j > i; j--)
          ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][j][i] = -1;
      }else{
        graphicObjects[i] = 0;
      }
    }
  }
  /*** REGLE N*2 : DESACTIVATION DE TOUS LES BODIES AVEC LEUR PREDECESSEUR DIRECT ***/
  /*** ALLOCATION  DU TABLEAU HIERARCHIQUE ***/
  hierarchical_father = MY_ALLOC(int, nof_obj_for_rob);
  /*** INITIALISATION ****/
  for (i = 0; i < nof_obj_for_rob; i++)
    hierarchical_father[i] = -1;
  for (i = 0; i < nof_obj_for_rob; i++) {
    /*** ON EVITE LE CAS DES CORPS VIDES DEJA TRAITE OU GRAPHIQUES PURE***/
    if (XYZ_ENV->cur_robot->o[i]->np != 0 && !graphicObjects[i]) {
      /*** RECHERCHE DU CORPS PREDECESSEUR NON VIDE ***/
      father_body_found = 0;
      grand_father_body_found = 0;
      ajntPt = XYZ_ENV->cur_robot->o[i]->jnt;
      /*** TANT QUE AUCUNE REGLE DE SORTIE N'EST ATTEINTE ***/
      while (father_body_found == 0 || (deep2 && grand_father_body_found == 0)) {
        /*** ON REMONTE D'UN CRAN DANS LA CHAINE CINEMATIQUE SI POSSIBLE***/
        if (ajntPt->prev_jnt != NULL) {
          ajntPt = ajntPt->prev_jnt;
          /*** SI ON TROUVE UN CORPS NON VIDE ***/
          if (ajntPt->o != NULL)
            if (ajntPt->o->np > 0){
              if(father_body_found == 0){
                father_body_found = 1;
                father_jnt = ajntPt;
              }else if (deep2){//Desactivation du joint grand pere si le pere a deja ete trouve
                grand_father_body_found = 1;
              }
            }
          /*** SI ON NE PEUT PLUS REMONTER PLUS LOIN ET QUE LE CORPS N'EST PAS CANDIDAT***/
          if (father_body_found == 0 && ajntPt->prev_jnt == NULL) father_body_found = -1;
          if (deep2 && grand_father_body_found == 0 && ajntPt->prev_jnt == NULL) grand_father_body_found = -1;
          /*** SI ON BOUCLE ***/
          if (ajntPt->o == XYZ_ENV->cur_robot->o[i]) father_body_found = -1;
          if (deep2 && ajntPt->o == XYZ_ENV->cur_robot->o[i] && father_body_found == 1) grand_father_body_found = -1;
        }
        /*** ON ETAIT SUR LE PREMIER JOINT DES LE DEPART ***/
        else{
          father_body_found = -1;
          grand_father_body_found = -1;
        }
      }
      /*** ON DESACTIVE EVENTUELLEMENT LE CANDIDAT RETENU ***/
      if (father_body_found == 1) {
        body_index = p3d_get_body_nid_by_name(father_jnt->o->name);
        if (i > body_index)
          ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][body_index] = -1;
        else if (body_index > i)
          ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][body_index][i] = -1;
        /*** ON MEMORISE LES LIENS POUR LA REGLE N*3 ***/
        hierarchical_father[i] = body_index;
      } else hierarchical_father[i] = -1;

      /*** ON DESACTIVE EVENTUELLEMENT LE CANDIDAT RETENU ***/
      if (deep2 && grand_father_body_found == 1) {
        body_index = p3d_get_body_nid_by_name(ajntPt->o->name);
        if (i > body_index)
          ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][body_index] = -1;
        else if (body_index > i)
          ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][body_index][i] = -1;
      }
    }
  }
	if(DEBUG){
		printf("   ");
		for (i = 0; i < nof_obj_for_rob - 1 ; i++){
			printf("%d ", i%10);
		}
    int nbPair = 0;
		for (i = nof_obj_for_rob - 1; i > 0; i--){
			printf("\n%d ", i);
			for (j = 0; j < i; j++){
				if((((ROB_AUTOCOL->body_links)[ROB_AUTOCOL->cur_rob_id])[i])[j] > 0){
					printf("+ ");
          nbPair++;
				}else{
					printf("- ");
				}
			}
		}
		printf("\nnbPair = %d\n", nbPair);
	}
  /*** REGLE N*3 : DESACTIVATION DES CORPS AYANT UN PARENT IDENTIQUE, PARTANT D'UN POINT COMMUN OU DONT LES JNT SONT FIXES***/
  for (i = 0; i < nof_obj_for_rob; i++) {
    /*** EXISTE T IL UN PERE IDENTIQUE POUR DEUX BODIES NON VIDES ET NON GRAPHICS PURE POSSEDANT EUX MEME UN PERE ? ***/
    if (hierarchical_father[i] > -1 && XYZ_ENV->cur_robot->o[i]->np > 0 && !graphicObjects[i]) {
      for (j = i + 1; j < nof_obj_for_rob; j++) {
        /*** SI LE CAS EST VERIFIE ***/
        if (hierarchical_father[i] == hierarchical_father[j])
          /*** ON VERIFIE QUE LES POINTS DE DEPART SONT PROCHES ***/
          if ((XYZ_ENV->cur_robot->o[i]->jnt->p0.x - XYZ_ENV->cur_robot->o[j]->jnt->p0.x < epsilon
              && XYZ_ENV->cur_robot->o[i]->jnt->p0.y - XYZ_ENV->cur_robot->o[j]->jnt->p0.y < epsilon
              && XYZ_ENV->cur_robot->o[i]->jnt->p0.z - XYZ_ENV->cur_robot->o[j]->jnt->p0.z < epsilon) ||
							XYZ_ENV->cur_robot->o[i]->jnt->type == P3D_FIXED && XYZ_ENV->cur_robot->o[j]->jnt->type == P3D_FIXED)
            /*** AUQUEL CAS ON DESACTIVE ***/
            ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][j][i] = -1;
      }
    }
  }
	/*** REGLE N*4 : DESACTIVATION DES CORPS AYANT LE MEME JOINT ***/
	//Ce code n'est pas optimal
	for (i = 0; i < nof_obj_for_rob; i++) {
		if (XYZ_ENV->cur_robot->o[i]->jnt != NULL) {
			if (XYZ_ENV->cur_robot->o[i]->jnt->o != XYZ_ENV->cur_robot->o[i]){
				for(j = 0; j < nof_obj_for_rob; j++){
					if (i > j)
						ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][j] = -1;
					else if (j > i)
						ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][j][i] = -1;
				}
			}
		}
	}
	if(DEBUG){
		printf("   ");
		for (i = 0; i < nof_obj_for_rob - 1 ; i++){
			printf("%d ", i%10);
		}
    int nbPair = 1;
		for (i = nof_obj_for_rob - 1; i > 0; i--){
			printf("\n%d ", i);
			for (j = 0; j < i; j++){
				if((((ROB_AUTOCOL->body_links)[ROB_AUTOCOL->cur_rob_id])[i])[j] > 0){
					printf("+ ");
          nbPair++;
				}else{
					printf("- ");
				}
			}
		}	
    printf("\nnbPair = %d\n", nbPair);
	}
  MY_FREE(hierarchical_father, int, nof_obj_for_rob);
  return 0;
}


/* modif Juan Cortes */
/*******************************************************/
/* Fonction de desactivation du collision checker      */
/* de toutes les auto-collisions d'un robot            */
/* In :                                                */
/* Out :                                               */
/*******************************************************/

int p3d_desactivate_col_check_all(void) {
  int i, j, nof_obj_for_rob;

  /*** MAKE THE ROBOT DESCRIBED THE CURRENT ROBOT IN STRUCT ENV ***/
  if (ROB_AUTOCOL->cur_rob_id < 0) {
    PrintInfo(("Aucun robot selectionne ! appelez p3d_add_desc_rob_col_init # NOM DU ROBOT # auparavant \n"));
    return (-3);
  }

  nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[ROB_AUTOCOL->cur_rob_id];

  for (i = nof_obj_for_rob - 1; i > 0; i--)
    for (j = 0; j < i; j++)
      ROB_AUTOCOL->body_links[ROB_AUTOCOL->cur_rob_id][i][j] = -1;

  return 0;
}
/* finmodif Juan Cortes */


int p3d_autocol_activate_rob(p3d_rob *rob) {
  int nof_robots = 0, nof_obj_for_rob;
  int retval = 0;

  int rob_index, i, j, rob_found_in_lists;

  /*** CHECK INTEGRITY OF THE STRUCTURE ***/
  p3d_autocol_check_integrity();
  nof_robots = ROB_AUTOCOL->nof_robot_for_struct;
  /*** FIND THE ROBOT INDEX IN XYZ_ENV STRUCTURE ***/
  rob_index = -1;
  for (i = 0; i < nof_robots; i++)
    if (XYZ_ENV->robot[i] == rob) rob_index = i;

  /*** IS THE ROBOT DESCRIBED IN ROB_AUTOCOL LISTS ? ***/
  rob_found_in_lists = 0;
  for (i = 0; i < nof_robots; i++)
    if (ROB_AUTOCOL->described_robots[i] == rob_index) rob_found_in_lists = 1;

  /*** YES IT IS ***/
  if (rob_found_in_lists) {
    if (DEBUG) PrintInfo(("Robot %s has been defined in p3d file... taking in account activation lists\n", XYZ_ENV->robot[rob_index]->name));
    nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[rob_index];
    for (i = nof_obj_for_rob - 1; i > 0; i--)
      for (j = 0; j < i; j++)
        p3d_autocol_activate_body_pair(rob_index, i, j);
  }

  /*** NO IT'S NOT ***/
  else {
    retval = p3d_init_rob_col_activ(rob->name);
    if (retval == 0) {
      retval = p3d_desactivate_col_check_automatic();
      if (retval == 0) {
        nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[rob_index];
        for (i = nof_obj_for_rob - 1;i > 0;i--)
          for (j = 0; j < i; j++)
            p3d_autocol_activate_body_pair(rob_index, i, j);
      }
    }
  }
  /*** DESALLOCATION ***/
  /* p3d_autocol_destroy_datas_for_robot(rob_index); */
  /*** END ***/
  return retval;
}

/*! \brief Function to activate the collision between two robot bodies.
 *
 *  \param  rob_index: The robot index
 *  \param  body1: The first body index
 *  \param  body2: The second body index
 */
void p3d_autocol_activate_body_pair(int rob_index, int body1, int body2) {
  if (ROB_AUTOCOL->body_links[rob_index][body1][body2] == 1) {
    p3d_col_activate_obj_obj(XYZ_ENV->robot[rob_index]->o[body1], XYZ_ENV->robot[rob_index]->o[body2]);
    /* added by Carl: */
  } else {
    p3d_col_deactivate_obj_obj(XYZ_ENV->robot[rob_index]->o[body1], XYZ_ENV->robot[rob_index]->o[body2]);
    /* end addition */
  }
}

/*! \brief Function to check if the collision between two bodies is checked or not.
 *
 *  \param  rob_index: The robot index
 *  \param  body1: The first body index
 *  \param  body2: The second body index
 *  \return 1 if the collision is checked, -1 otherwise.
 */
int p3d_isMarkedForautocol(int rob_index, int body1, int body2) {
  if (body2 < body1){
    return ROB_AUTOCOL->body_links[rob_index][body1][body2];
  }else{
    return ROB_AUTOCOL->body_links[rob_index][body2][body1];
  }
}

/*--------------------------------------------------------------------------*/
/*! \brief Destroy a structure to store the data of autocollisison.
 *
 * \note Modify ::ROB_AUTOCOL but not ::ROB_AUTOCOL_ENVS (pointer nod valid).
 *
 * \internal
 */
static void s_p3d_autocol_destroy_data(void) {
  int i_rob, i_obj, i, is_described, nof_robots, nof_obj_for_rob;

  nof_robots = ROB_AUTOCOL->nof_robot_for_struct;
  for (i_rob = 0; i_rob < nof_robots; i_rob++) {
    /*** CHANGE THE CURRENT ROBOT ***/
    XYZ_ENV->cur_robot = XYZ_ENV->robot[i_rob];
    /*** DESALLOCATION OF THE STRUCTURE IF THE ROBOT HAS BEEN DESCRIBED AND NOT YET FREE ***/
    is_described = 0;
    for (i = 0; i < nof_robots; i++) {
      if (ROB_AUTOCOL->described_robots[i] == i_rob) {
        is_described = 1;
      }
    }
    if (is_described == 1) {
      nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[i_rob];
      for (i_obj = nof_obj_for_rob - 1; i_obj > 0; i_obj--) {
        MY_FREE(ROB_AUTOCOL->body_links[i_rob][i_obj], int, i_obj);
      }
      MY_FREE(ROB_AUTOCOL->body_links[i_rob], int*, nof_obj_for_rob);
    }
  }
  /*** DESALLOCATION OF THE REST ***/
  MY_FREE(ROB_AUTOCOL->body_links,         int**, nof_robots);
  MY_FREE(ROB_AUTOCOL->described_robots,   int,   nof_robots);
  MY_FREE(ROB_AUTOCOL->nof_obj_for_robots, int,   nof_robots);
  MY_FREE(ROB_AUTOCOL, p3d_rob_col_activ, 1);
  ROB_AUTOCOL = NULL;
}

void p3d_autocol_destroy_datas(void) {
  int i;

  if (DEBUG) PrintInfo(("Structure destruction ..."));

  for (i = 0; i < NB_ROB_AUTOCOL_ENVS; i++) {
    ROB_AUTOCOL = ROB_AUTOCOL_ENVS[i];
    s_p3d_autocol_destroy_data();
  }
  MY_FREE(ROB_AUTOCOL_ENVS, pp3d_rob_col_activ, NB_ROB_AUTOCOL_ENVS);
  NB_ROB_AUTOCOL_ENVS = 0;
  ROB_AUTOCOL_ENVS = NULL;
  if (DEBUG) PrintInfo((" ENDED\n"));
}

void p3d_autocol_destroy_datas_for_robot(int index_rob_to_delete) {
  int i_obj, is_described, i, nof_obj_for_rob;
  if (DEBUG) basic_alloc_debugon();
  /*** DESALLOCATION OF THE STRUCTURE IF THE ROBOT HAS BEEN DESCRIBED ***/
  is_described = 0;
  for (i = 0;i < ROB_AUTOCOL->nof_robot_for_struct;i++)
    if (ROB_AUTOCOL->described_robots[i] == index_rob_to_delete) {
      ROB_AUTOCOL->described_robots[i]  = -1;
      is_described = 1;
    }
  if (is_described == 1) {
    nof_obj_for_rob = ROB_AUTOCOL->nof_obj_for_robots[index_rob_to_delete];
    for (i_obj = nof_obj_for_rob - 1; i_obj > 0; i_obj--)
      MY_FREE(ROB_AUTOCOL->body_links[index_rob_to_delete][i_obj], int, i_obj);
    MY_FREE(ROB_AUTOCOL->body_links[index_rob_to_delete], int*, nof_obj_for_rob);
  }
  if (DEBUG) basic_alloc_debugoff();
}

void p3d_setAutocolDeep2(int deep2){
	DEEP2 = deep2;
}

/*** ANCIENNE FONCTION D'INITIALISATION p3d_col_env.c ***/
/*********************************************************/
/* Fonction d'activation de toutes les paires d'un robot */
/* In : le robot                                         */
/* Out :                                                 */
/*********************************************************/
/* void p3d_col_activate_rob(p3d_rob *rob) */
/* {int ib,jb,nb,margin = 0,base; */
/*  p3d_obj *bod,*bodcur; */
/*  int rob_index,i,j,rob_found_in_lists; */
/*   DEBUT Modification Thibaut */
/*  nb= rob->no; */
/*   Pour tout corps */
/*  for(ib=0;ib<nb;ib++) {  */
/*    bodcur = rob->o[ib]; */
/*     Desactivation des body extremes uniquement dans le cas         */
/*     des articulations a plusieurs ddl (empilement de "corps pipo") */
/*     */
/*      #### cas particulier de la chaine ramifiee sans corps de base #### */
/*         Si la marge est nulle */
/*        if(!margin) { */
/*         Mettre a jour la base avec le no du joint precedent */
/*        base = (bodcur->jnt->num) ? bodcur->jnt->prev_jnt->num : 0; */
/*        } */
/*        ####----------------------------------------------------------#### */
/*    */

/*     Si le corps est pipo */
/*    if(!(bodcur->np)) { */
/*        Incrementer la marge si la chaine cinematique n'est pas rompue*/
/*      (!(bodcur->jnt->num) */
/*       || (bodcur->jnt->num == bodcur->jnt->prev_jnt->num+1) */
/*       || (bodcur->jnt->prev_jnt->num == base)) ? margin++ : (margin = 0); */
/*       Prochain corps ^ */
/*      continue; */
/*    } */

/*     Si le corps n'est pas pipo */
/*     Pour tous les corps suivants du robot */
/*    for(jb=ib+1;jb<nb;jb++) { */
/*      bod = rob->o[jb]; */
/*       Si */
/*  no du joint du corps courant - marge <= no du joint precedent du corps suivant < no du joint du corps courant */
/*  ( le corps suivant s'appuie sur un corps pipo de la chaine cinematique du corps courant ) */
/*  | */
/*  no du joint precedent du corps suivant = no du joint du corps courant */
/*  ( le corps suivant continue la chaine cinematique du corps courant )  */
/*  #### cas particulier de la chaine ramifiee sans corps de base #### */
/*  ## |    */
/*  ## no du joint precedent du corps suivant = base */
/*  ## ( le corps suivant s'appuie sur le meme joint que le corps courant ) */
/*  ####----------------------------------------------------------#### */
/*      */
/*      if((bodcur->jnt->num-margin <= bod->jnt->prev_jnt->num */
/*   && bod->jnt->prev_jnt->num <= bodcur->jnt->num) */
/*   */
/*    #### cas particulier de la chaine ramifiee sans corps de base #### */
/*    || (bod->jnt->prev_jnt->num == base) */
/*    ####----------------------------------------------------------#### */
/*  */
/*  ){ */

/*        Le corps suivant devient le premier corps non pipo dans les corps suivants du robot */
/*        while(!(bod->np) && jb<nb) { */
/*   bod = rob->o[++jb]; */
/*        } */
/*      } else { */
/*         Si le corps suivant n'est pas pipo */
/*          et les 2 corps n'ont pas le meme repere et partent du meme point */
/*        if((bod->np)&&!((bod->jnt->prev_jnt == bodcur->jnt->prev_jnt)&&((bod->jnt->p0.x == bodcur->jnt->p0.x)&&(bod->jnt->p0.y == bodcur->jnt->p0.y)&&(bod->jnt->p0.z == bodcur->jnt->p0.z)))) { */
/*    Activer les corps suivant et courant*/
/*   p3d_col_activate_obj_obj(bodcur,bod); */
/*        } */
/*      } */
/*    } */
/*     Reinitialiser la marge */
/*    margin = 0; */
/*     FIN Modification Thibaut */
/*  } */


/** FIN Modif J. PETTRE 30/05/01-- ajout de la fonction permettant
    la desactivation a la main de la detection de collision pour les
    bodies des robots **/

/* On desactive les paires de corps affectes par contraintes de fermeture */
/*  p3d_col_deactivate_cntrt_pairs(); */
/* }    */

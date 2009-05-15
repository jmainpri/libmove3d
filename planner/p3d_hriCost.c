#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"
#include "Util-pkg.h"
#include "Graphic-pkg.h"

/**********************************************************************************
 * New Code for Hri Transition-RRT
 * *******************************************************************************/

/**
 * Flag to set if the configuration space
 * has got associated a safe zone
 */
static int IS_HRI_FUCNTION_SPACE = FALSE;

/**
 * Number of HRI zones
 */
static int NB_HRI_ZONES = 0;

/**
 * Size of the zone to be cared as safe
 *
 */
static double HRI_ZONE_SIZE = 11.0;

/**
 * Vector to be shown  in the environement
 *
 */
static double vect_jim[6];


/**
 * p3d_SetIsHriFuncSpace
 */
void p3d_SetIsHriFuncSpace(int IsHriFuncSpace) {
  IS_HRI_FUCNTION_SPACE = IsHriFuncSpace;
  printf("IsHriFuncSpace\n");
}

/**
 * p3d_GetIsHriFuncSpace
 */
int p3d_GetIsHriFuncSpace() {
  return IS_HRI_FUCNTION_SPACE;
}

/**
 * p3d_SetHriZoneSize
 */
void p3d_SetHriZoneSize(double size) {
  HRI_ZONE_SIZE = size;
}

/**
 * p3d_AddHriZone
 */
void p3d_AddHriZone() {
  NB_HRI_ZONES++;
}

/**
 * p3d_GetHriZoneSize
 */
double p3d_GetHriZoneSize() {
  return HRI_ZONE_SIZE;
}

/**
 * p3d_GetVectJim
 */
double* p3d_GetVectJim() {
  return vect_jim;
}

/**
 * p3d_DeactivateAllButHri
 */
void p3d_DeactivateAllButHri(int disp) {

  for (int i = 0; i < XYZ_ENV->no; i++) {
    if (strcmp(XYZ_ENV->o[i]->name, "zone_hri") != 0) {
      p3d_kcd_deactivate_obstacle(XYZ_ENV->o[i]);

      if (disp)
        PrintInfo(("Desactive l'objet %s\n", XYZ_ENV->o[i]->name));
    } else {
      p3d_kcd_activate_obstacle(XYZ_ENV->o[i]);

      if (disp)
        PrintInfo(("Active l'objet %s\n", XYZ_ENV->o[i]->name));
    }
  }
}

void p3d_ActivateAll(int disp) {

  for (int i = 0; i < XYZ_ENV->no; i++) {
    p3d_kcd_activate_obstacle(XYZ_ENV->o[i]);

    if (disp)
      PrintInfo(("Active l'objet %s\n", XYZ_ENV->o[i]->name));
  }
}

/**
 * p3d_GetMinDistCost
 *
 * Get the cost of a current configuration based
 * on its minimal distance to the HRI zone.
 */
double p3d_GetHriDistCost(p3d_rob* robotPt, int disp) {

  // Lancer un test avant...
  // IMPORTANT
  // ????????????? ... 2 Fois

  for (int i = 0;i < 2;i++) {
    p3d_col_test_all();
    if (disp)
      PrintInfo(("   p3d_col_test_all\n"));

    p3d_DeactivateAllButHri(FALSE);
    if (disp)
      PrintInfo(("   p3d_DeactivateAllButHri\n"));
  }

  int nof_bodies = robotPt->no;

  // Distance pour chaque body
  double* distances = MY_ALLOC(double, nof_bodies);

  //p3d_vector3 *points_mos; // Movable
  p3d_vector3 *body = MY_ALLOC(p3d_vector3, nof_bodies);
  //p3d_vector3 *points_sos; // Statical
  p3d_vector3 *other = MY_ALLOC(p3d_vector3, nof_bodies);

  // Resultat
  p3d_vector3 zpa, zpb;

  // Distance Estimée vers le plus proche static
  double distance_estimate = P3D_HUGE;

  int k = 0;

  for (int j = 0;j < nof_bodies;j++) {

    kcd_get_points_closest_pair(robotPt->num, j, zpa, zpb);

    for (int i = 0 ; i < 3 ; i++) {
      body[j][i]   = zpa[i];
      other[j][i]  = zpb[i];
    }
    kcd_get_dist_grp_mo_so(robotPt->num, j, distances + j);

    if (distances[j] < distance_estimate) {
      distance_estimate = distances[j];
      k = j;
    }
  }

  vect_jim[0] = body[k][0];
  vect_jim[1] = body[k][1];
  vect_jim[2] = body[k][2];
  vect_jim[3] = other[k][0];
  vect_jim[4] = other[k][1];
  vect_jim[5] = other[k][2];

//    g3d_drawOneLine(
//      body[k][0], body[k][1], body[k][2],
//      other[k][0], other[k][1], other[k][2],
//      Red,NULL);

  double dist_penetration =  HRI_ZONE_SIZE - distance_estimate;

  // A voir pour seuillage etc. Definition de la fonction de cout
  double cost = dist_penetration < 0 ? 0 : dist_penetration;
  

  if (disp) {
    PrintInfo(("body = (%3.2f,%3.2f,%3.2f)\n", body[k][0], body[k][1], body[k][2]));
    PrintInfo(("other = (%3.2f,%3.2f,%3.2f)\n", other[k][0], other[k][1], other[k][2]));
    PrintInfo(("Cost = \t%3.2f, and distance_estimate = %3.2f \n", cost, distance_estimate));
  }

  p3d_ActivateAll(TRUE);

  if (disp)
    PrintInfo(("-----------------------------------------------------------\n"));

  return cost;
}

/**
 * p3d_SaveGPlotCostTraj
 *
 * Save the cost of configuration along the trajectory
 */
void p3d_SaveGPlotCostTraj(void) {

  FILE* traj_file = fopen("cost_traj.txt", "w");

  p3d_rob* robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  for (double current_param = 0.;
       current_param < robotPt->tcur->range_param;
       current_param += p3d_get_env_dmax()) {
    configPt q = p3d_config_at_param_along_traj(robotPt->tcur,
                 current_param);
    fprintf(traj_file, "%f %f\n", current_param, p3d_GetConfigCost(robotPt,
            q));
  }

  fclose(traj_file);
  PrintInfo(("'cost_traj.txt' creation\n"));
}

/**
 * p3d_SaveGPlotCostNodeChro
 *
 * Save the costs of all nodes in the graph
 * with an chronological order of apearance
 */
void p3d_SaveGPlotCostNodeChro(void) {
  // Save cost to file
  int i = 0;
  int j = 0;

  FILE* traj_file = fopen("cost_chro.txt", "w");

  for (p3d_compco *comp = XYZ_GRAPH->comp;
       comp != NULL && j < 10;
       comp = comp->suiv, j++) {

    for (p3d_list_node *n = comp->dist_nodes;
         n != NULL && i < 10000;
         n = n->next, i++) {
      fprintf(traj_file, "%d %f %f\n", n->N->num, n->N->cost, n->N->temp);
    }
  }

  fclose(traj_file);
  PrintInfo(("'cost_chro.txt' creation\n"));
}

double p3d_GetLpHriDistCost(p3d_rob* robotPt, p3d_localpath * lp){
  double dist = 0.0, cost = 0.0;
  configPt q;


  for(int i = 0; dist < lp->length_lp; i++, dist=i*lp->length_lp/20){
    q = lp->config_at_distance(robotPt, lp, dist);
    p3d_set_robot_config(robotPt, q);
    cost += p3d_GetHriDistCost(robotPt, 0);
  }
  return cost;
}



/**
 * p3d_SaveGPlotCostNodeChro
 *
 * Save the of the given 2D cost Space into a
 * Energetic surface type structure in an array
 */
// void p3d_SetCostToTab(p3d_rob *robotPt,  conf_cost * tab, int nbPart) {
// 
//   int i, j, k, count = 0;
// 
//   double ZminEnv = HUGE_VAL;
//   double ZmaxEnv = 0;
//   double vMinDof1, vMaxDof1, vMinDof2, vMaxDof2, currentCost;
// 
//   configPt q, QSaved;
//   p3d_jnt * jntPt;
// 
// 
//   q  = p3d_alloc_config(robotPt);
// 
//   jntPt = robotPt->joints[0];
//   for (i = 0 ; i < 6; i++) {
//     double vmin = 0;
//     double vmax = 0;
//     p3d_jnt_get_dof_bounds(jntPt, i, &vmin, &vmax);
//     q[i] = vmax;
//     PrintInfo(("Joint[0][%d] (vmax) = %f\n", i, vmax));
//   }
// 
//   if (robotPt->joints[1]->type == P3D_PLAN) {
// 
//     jntPt = robotPt->joints[1];
//     p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof1, &vMaxDof1);
//     p3d_jnt_get_dof_rand_bounds(jntPt, 1, &vMinDof2, &vMaxDof2);
// 
//   } else if (robotPt->joints[1]->type == P3D_ROTATE) {
// 
//     jntPt = robotPt->joints[1];
//     p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof1, &vMaxDof1);
// 
//     jntPt = robotPt->joints[2];
//     p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof2, &vMaxDof2);
// 
//   } else {
// 
//     PrintInfo(("Error in set_cost_to_tab : Joint type not taken into accout\n"));
//     return;
// 
//   }
// 
// 
// 
//   PrintInfo(("vMinDof1 = %f\n", vMinDof1));
//   PrintInfo(("vMaxDof1 = %f\n", vMaxDof1));
// 
//   PrintInfo(("vMinDof2 = %f\n", vMinDof2));
//   PrintInfo(("vMaxDof2 = %f\n", vMaxDof2));
// 
//   PrintInfo(("nb_dof = %d\n", robotPt->nb_dof));
//   PrintInfo(("nb_user_dof = %d\n", robotPt->nb_user_dof));
// 
//   for (i = 0; i < nbPart; i++) {
//     for (j = 0; j < nbPart; j++) {
//       q[6] = vMinDof1 + i * (vMaxDof1 - vMinDof1) / (nbPart - 1);
//       q[7] = vMinDof2 + j * (vMaxDof2 - vMinDof2) / (nbPart - 1);
// 
//       QSaved = p3d_get_robot_config(robotPt);
// //   p3d_set_and_update_robot_conf(q);
//       p3d_set_and_update_this_robot_conf(robotPt, q);
// 
//       /* Jim modif hri */
//       tab[j*nbPart+i].cost = p3d_GetHriDistCost(robotPt, FALSE);
//       tab[j*nbPart+i].q = MY_ALLOC(double , robotPt->nb_dof);
// 
//       p3d_set_and_update_robot_conf(QSaved);
//       p3d_destroy_config(robotPt, QSaved);
// 
//       for (int id = 0; id < robotPt->nb_dof; id++)
//         tab[j*nbPart+i].q[id] = q[id];
//     }
//   }
// }

/**
 * p3d_WriteConfCostToCSV
 */
// void p3d_WriteConfCostToCSV(FILE *fd, conf_cost *tab, int size) {
//   conf_cost cc;
//   fd = fopen("tab.cost" , "w");
// 
//   for (int a = 0;a < size;a++) {
//     for (int b = 0;b < size;b++) {
// 
//       cc = tab[b*size+a];
// 
//       fprintf(fd, "%.10f,%.10f,%.10f\n", cc.q[6], cc.q[7], cc.cost);
// 
//     }
//   }
// 
//   fclose(fd);
// }


/**
 * p3d_WriteConfCostToOBPlane
 */
// void p3d_WriteConfCostToOBPlane(FILE *fd, conf_cost *tab, int size) {
//   conf_cost cc;
// 
//   fd = fopen("Ob_Plane.macro" , "w");
// 
//   fprintf(fd, "p3d_beg_desc P3D_OBSTACLE\n\n");
//   fprintf(fd, "\tp3d_add_desc_poly polyhedre1\n");
// 
//   for (int a = 0;a < size;a++) {
//     for (int b = 0;b < size;b++) {
//       cc = tab[b*size+a];
//       fprintf(fd, "\t\tp3d_add_desc_vert %.10f,%.10f,%.10f\n", cc.q[6], cc.q[7], cc.cost);
//     }
//   }
// 
//   fprintf(fd, "\n\n");
// 
//   for (int a = 0;a < size - 1;a++) {
//     for (int b = 0;b < size - 1;b++) {
//       fprintf(fd, "\t\tp3d_add_desc_face %d %d %d %d \n",
//               b*size + a , (b++)*size + a , b*size + (a++) , (b++)*size + (a++));
//     }
//   }
// 
//   fprintf(fd, "\n\tp3d_end_desc_poly/n");
//   fprintf(fd, "\tp3d_set_prim_pos_by_mat polyhedre1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/n");
//   fprintf(fd, "3d_end_desc/n/n");
//   fprintf(fd, "p3d_set_obst_poly_color 1 Any 0.8 0.8 0.8/n");
// 
//   fclose(fd);
// }

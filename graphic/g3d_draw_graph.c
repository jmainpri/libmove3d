#include "Util-pkg.h"
#include "P3d-pkg.h"

#include "env.hpp"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#include "Graphic-pkg.h"
#include "GroundHeight-pkg.h"

#include "move3d-headless.h"  // <- modif Juan

extern double ZminEnv;
extern double ZmaxEnv;
static void draw_joints(p3d_rob *r, double *q, int num);
static void draw_joint_j(pp3d_jnt j, double *xi, double *yi, double *zi, double ray, int color);
extern void* GroundCostObj;
extern int IsGraphMovie;

/*******************************************************************************/

static
int mod(int a, int b) {
  int res;

  res = a - b * floor(a / b);
  return(res);
}

/*******************************************************************************/

/*
 * draw current graph
 */

// modif Juan
void g3d_draw_graph(void) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  double *q, ray, normalray;
  configPt qsave;
  int nedge = 0;
#ifdef P3D_PLANNER
  p3d_graph *G = XYZ_GRAPH;
#endif
  p3d_list_edge *list_edge;
  p3d_list_node *list_node;
  p3d_compco *comp;
  p3d_node *N = NULL;
  double x1, x2, y1, y2, z1, z2;
  p3d_obj *body = robotPt->o[robotPt->no-1];
  p3d_matrix_type *pi,*pf;
  int color;
  //lm_reeds_shepp_str *rs_paramPt= lm_get_reeds_shepp_lm_param(robotPt);
  p3d_jnt *drawnjnt;
  //p3d_cntrt *ctPt;
  int i;
  p3d_obj *oray;
  int indexjnt = 0;
  int MaxFailExtendNode;
  int val1, val2;
  double Cost1, Cost2;
  int* ikSol = NULL, validIkSol = 1;

  /*   if (rs_paramPt != NULL) */
  /*     ray=rs_paramPt->radius/5.; */
  /*   else  */
  /*     ray = -.2; */

  if (!body)
    return;

  indexjnt = p3d_get_user_drawnjnt();
  if (indexjnt != -1 && indexjnt <= robotPt->njoints ) {
    drawnjnt = XYZ_ROBOT->joints[indexjnt];
  } else {
#ifdef P3D_COLLISION_CHECKING
    if (p3d_col_get_mode() == p3d_col_mode_bio) {
      drawnjnt = XYZ_ROBOT->joints[0]->next_jnt[XYZ_ROBOT->joints[0]->n_next_jnt - 1];
    } else {
      drawnjnt = body->jnt;
    }
#endif
  }

  // if the joint to be drawn has changed,
  // recompute the positions of the nodes in the 3d view
  bool compute_positions = false;

#ifdef P3D_PLANNER
  if(G->g3d_drawnjnt != drawnjnt) {
    G->g3d_drawnjnt = drawnjnt;
    compute_positions = true;
  }
#endif

  qsave = p3d_get_robot_config(robotPt);

  if (body->np == 0) {//take a non empty body
    i = 0;
    oray = robotPt->o[robotPt->no-2-i];
    while ((oray != NULL) && (oray->np == 0)) {
      oray = robotPt->o[robotPt->no-2-(--i)];
    }
  } else {
    oray = body;
  }

  p3d_get_box_obj(oray, &x1, &x2, &y1, &y2, &z1, &z2); //get the object bounding box
  normalray = sqrt(SQR(x2 - x1) + SQR(y2 - y1) + SQR(z2 - z1)) / 100.;
	
#ifdef P3D_PLANNER
  comp = G->comp;

  /* for each connected component */
  while (comp) {
    list_node = comp->dist_nodes;
    /* for each node */
		//printf("--------------------------------\n");
    while (list_node) {
      N = list_node->N;
      q = N->q;
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, q);
//			printf("Draw node %d\n",N->num);
//			printf("next node %d\n",list_node->next);
			
      ikSol = p3d_get_ik_draw();// draw a specific solution class

      for (i = 0, validIkSol = 1; N->iksol && i < robotPt->cntrt_manager->ncntrts; i++) {
        if ((ikSol[i] > 0 && N->iksol[i] > 0 && N->iksol[i] != ikSol[i]) ||
            (ikSol[i] < 0 && N->iksol[i] < 0 && N->iksol[i] != ikSol[i]) ||
            (ikSol[i] < 0 && N->iksol[i] > 0) || (ikSol[i] > 0 && N->iksol[i] < 0)) {
          validIkSol = 0;
        }

      }
      /* draw configuration */
//    if(N->n_fail_extend < 10) {   // modif juan (test)
//    g3d_draw_rep_obj(drawnjnt,ray,N->numcomp);
      //start path defrom
      if (((p3d_get_show_cycle_only() == FALSE) || ((G->start_nodePt != NULL) && (N->comp == G->start_nodePt->comp))) && (validIkSol)) {
        //end path deform
        if (N->pinpointed == 1) {
          ray = normalray * 5;
        } else {
          ray = normalray;
        }
        //start Mokhtar for picking
        glLoadName(N->num);
        //end Mokhtar
        if (!ENV.getBool(Env::isCostSpace)) {
          if ((N->only_for_cycle == FALSE) && (!N->isSingularity) && ((p3d_get_MOTION_PLANNER() == P3D_BASIC) || (p3d_get_MOTION_PLANNER() == P3D_ISOLATE_LINKING))) {
            g3d_draw_rep_obj(drawnjnt, ray, N->numcomp);
          } else if (N->pinpointed == 1) {
            g3d_draw_rep_obj(drawnjnt, ray, 5); // Violet
          } else if (N->isSingularity == 1) {
            g3d_draw_rep_obj(drawnjnt, ray, 6); // Black
          } else {
            MaxFailExtendNode = ENV.getBool(Env::discardNodes);
            if (N->num == 1) {
              g3d_draw_rep_obj(drawnjnt, ray, 2); // Blue
            } else if (N->IsDiscarded == TRUE) {
              g3d_draw_rep_obj(drawnjnt, ray, 0);
            } else if (N->n_fail_extend < MaxFailExtendNode / 2) {
              g3d_draw_rep_obj(drawnjnt, ray, 1); // Green
            } else if (N->n_fail_extend < MaxFailExtendNode) {
              g3d_draw_rep_obj(drawnjnt, ray, 3); // Yellow
              //start path deform
            } else if (N->only_for_cycle == TRUE) {
              g3d_draw_rep_obj(drawnjnt, ray, Blue2); //Blue
              //end path deform
            } else {
              g3d_draw_rep_obj(drawnjnt, ray, 0); // Red
            }
          }
        }        /* draw edges */
        list_edge = N->edges;
        while (list_edge) {
          color = Black;
          if (!G->oriented && (list_edge->E->Ni->numcomp != list_edge->E->Nf->numcomp) && list_edge->E->unvalid == FALSE) {
            PrintInfo(("draw : ERREUR : arete entre %d de la comp %d et %d de la comp %d\n",
                       list_edge->E->Ni->num, list_edge->E->Ni->numcomp,
                       list_edge->E->Nf->num, list_edge->E->Nf->numcomp));
            color = Red;
          } else if(list_edge->E->unvalid == TRUE){
            color = Green;
          }
	  // compute node positions on an as needed basis
	  if(compute_positions || !list_edge->E->Ni->g3d_position_flag) {
	    p3d_set_and_update_this_robot_conf(robotPt,list_edge->E->Ni->q);
	    p3d_jnt_get_cur_vect_point(drawnjnt, list_edge->E->Ni->g3d_position);
	    list_edge->E->Ni->g3d_position_flag = true;
	  }
	  if(compute_positions || !list_edge->E->Nf->g3d_position_flag) {
	    p3d_set_and_update_this_robot_conf(robotPt,list_edge->E->Nf->q);
	    p3d_jnt_get_cur_vect_point(drawnjnt, list_edge->E->Nf->g3d_position);
	    list_edge->E->Nf->g3d_position_flag = true;
	  }
	  pi = list_edge->E->Ni->g3d_position;
	  pf = list_edge->E->Nf->g3d_position;
          //start path deform
          if (list_edge->E->for_cycle == TRUE) {
            color = Red;
          }
          //end path deform
          if ((G->oriented) && (list_edge->E->Ni->numcomp != list_edge->E->Nf->numcomp)) {
            color = Blue;
            g3d_drawOneLine(pi[0], pi[1], pi[2], (pf[0] + pi[0]) / 2., (pf[1] + pi[1]) / 2., (pf[2] + pi[2]) / 2., color, NULL);
            color = Red;
            g3d_drawOneLine((pf[0] + pi[0]) / 2., (pf[1] + pi[1]) / 2., (pf[2] + pi[2]) / 2., pf[0], pf[1], pf[2], color, NULL);
          } else {
            if ((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL)) {
              g3d_drawOneLine(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2], color, NULL);
            } else {
            	color = Red;
              val1 = GHintersectionVerticalLineWithGround(GroundCostObj, pi[0], pi[1], &Cost1);
              val2 = GHintersectionVerticalLineWithGround(GroundCostObj, pf[0], pf[1], &Cost2);
              g3d_drawOneLine(pi[0], pi[1], Cost1 + (ZmaxEnv - ZminEnv)*0.02, pf[0], pf[1], Cost2 + (ZmaxEnv - ZminEnv)*0.02, color, NULL);
              glLineWidth(2.);
              //       }
            }
          }
          list_edge = list_edge->next;
        }
      }
      nedge = nedge + N->nedge;
      list_node = list_node->next;
    }
    comp = comp->suiv;
  }
#endif
	
  p3d_set_and_update_robot_conf(qsave);
  p3d_destroy_config(robotPt,qsave);
#ifdef WITH_XFORMS
  if(IsGraphMovie == TRUE) {
    MovieDrawGraph();
  }
#endif
}
// fmodif Juan

/*******************************************************/
/* Fonction tracant le graphe courant en materialisant */
/* les noeuds sous forme de chaines cinematiques       */
/*******************************************************/
void g3d_draw_graph_detailed(void) {
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt q, qsave;
  double min[NDOF_BASE], max[NDOF_BASE], pi[3], pf[3];
  int in, ie, icomp, nedge = 0;
#ifdef P3D_PLANNER
  p3d_graph *G = XYZ_GRAPH;
#endif
  p3d_list_edge *list_edge;
  p3d_list_node *list_node;
  p3d_compco *comp;
  p3d_node *N = NULL;
  double x1, x2, y1, y2, z1, z2;
  int nb = p3d_get_desc_number(P3D_BODY);
  p3d_obj *o;
  int color;


  qsave = p3d_get_robot_config(robotPt);
#ifdef P3D_PLANNER
  comp = G->comp;
#endif
	
#ifdef P3D_PLANNER
  /* for each connected component */
  for (icomp = 1;icomp <= G->ncomp;icomp++) {

    list_node = comp->dist_nodes;

    /* for each node */
    for (in = 1;in <= comp->nnode;in++) {

      N = list_node->N;
      q = N->q;
      p3d_set_and_update_robot_conf(q);

      /* draw configuration */
      draw_joints(robotPt, q, N->numcomp);

      /* draw edge */
      list_edge = N->edges;
      p3d_get_robot_box(&min[0], &max[0], &min[1], &max[1],
                        &min[2], &max[2], &min[3], &max[3],
                        &min[4], &max[4], &min[5], &max[5]);

      for (ie = 0;ie <= N->nedge - 1;ie++) {

        color = Black;

        if (list_edge->E->Ni->numcomp != list_edge->E->Nf->numcomp  && list_edge->E->unvalid == FALSE) {
          PrintInfo(("draw : ERREUR : arete entre %d de la comp %d et %d de la comp %d\n",
                     list_edge->E->Ni->num, list_edge->E->Ni->numcomp,
                     list_edge->E->Nf->num, list_edge->E->Nf->numcomp));
          color = Red;
          exit(0);
        }

        if ((max[0] - min[0] != 0.) && (max[1] - min[1] != 0.) && (robotPt->o[0]->np != 0)) {
          g3d_drawOneLine(list_edge->E->Ni->q[0], list_edge->E->Ni->q[1], list_edge->E->Ni->q[2],
                          list_edge->E->Nf->q[0], list_edge->E->Nf->q[1], list_edge->E->Nf->q[2], color, NULL);
        } else {
          p3d_sel_desc_num(P3D_BODY, nb - 1);
          o = (p3d_obj *) p3d_get_desc_curid(P3D_BODY);

          p3d_set_and_update_robot_conf(list_edge->E->Ni->q);
          p3d_get_BB_obj(o, &x1, &x2, &y1, &y2, &z1, &z2);
          pi[0] = x1 + (x2 - x1) / 2.;
          pi[1] = y1 + (y2 - y1) / 2.;
          pi[2] = z1 + (z2 - z1) / 2.;

          p3d_set_and_update_robot_conf(list_edge->E->Nf->q);
          p3d_get_BB_obj(o, &x1, &x2, &y1, &y2, &z1, &z2);
          pf[0] = x1 + (x2 - x1) / 2.;
          pf[1] = y1 + (y2 - y1) / 2.;
          pf[2] = z1 + (z2 - z1) / 2.;

          g3d_drawOneLine(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2], color, NULL);
        }

        list_edge = list_edge->next;
      }
      nedge = nedge + N->nedge;

      list_node = list_node->next;
    }
    comp = comp->suiv;
  }
#endif

  p3d_set_and_update_robot_conf(qsave);
  p3d_destroy_config(robotPt, qsave);
}


/*************************************************/
/* Fonction materialisant un noeud sous forme de */
/* chaine cinematique                            */
/*************************************************/
static
void draw_joints(p3d_rob *r, double *q, int num) {
  pp3d_jnt j = r->joints[0];
  double ray, x1, x2, y1, y2, z1, z2;
  double min[NDOF_BASE], max[NDOF_BASE], xi, yi, zi;
  int i, color = 0, numcol;
  GLfloat mat_ambient_diffuse[4] = { .0, .0, .0, 1.0 };

  p3d_get_robot_box(&min[0], &max[0], &min[1], &max[1],
                    &min[2], &max[2], &min[3], &max[3],
                    &min[4], &max[4], &min[5], &max[5]);

  p3d_get_BB_rob(r, &x1, &x2, &y1, &y2, &z1, &z2);
  ray = sqrt(SQR(x2 - x1) + SQR(y2 - y1) + SQR(z2 - z1)) / 20.;


  mat_ambient_diffuse[0] = 0.;
  mat_ambient_diffuse[1] = 0.;
  mat_ambient_diffuse[2] = 0.;

  numcol = mod(num, 6);
  switch (numcol) {
    case 0:
      color = Red;
      break;
    case 1:
      color = Green;
      break;
    case 2:
      color = Blue;
      break;
    case 3:
      color = Yellow;
      break;
    case 4:
      color = Blue2;
      break;
    case 5:
      color = Violet;
      break;
  }

  if (r->o[0] && r->o[0]->np != 0) {
    xi = q[0];
    yi = q[1];
    zi = q[2];
    /* PrintInfo(("objet : %s %f %f %f %f %f %f\n",o->name,x1,x2,y1,y2,z1,z2)); */
    if (max[0] - min[0] != 0. && max[1] - min[1] != 0.) {
      if (max[2] - min[2] == 0.) {
        g3d_drawDisc(xi, yi, zi, ray, color, NULL);
      } else {
        g3d_drawColorSphere(xi, yi, zi, ray, color, NULL);
      }
    }
  } else {
    xi = P3D_HUGE;
    yi = P3D_HUGE;
    zi = P3D_HUGE;
  }
  ray = ray / 1.5;
  for (i = 0;i < j->n_next_jnt;i++) {
    draw_joint_j(j->next_jnt[i], &xi, &yi, &zi, ray, num);
  }
}

static
void draw_joint_j(pp3d_jnt j, double *xi, double *yi, double *zi, double ray, int color) {
  int i;
  p3d_obj *o = j->o;
  double xf, yf, zf, x1, x2, y1, y2, z1, z2;

  if (o) {
    p3d_get_BB_obj(o, &x1, &x2, &y1, &y2, &z1, &z2);
    xf = x1 + (x2 - x1) / 2.;
    yf = y1 + (y2 - y1) / 2.;
    zf = z1 + (z2 - z1) / 2.;
    g3d_drawColorSphere(xf, yf, zf, ray, color, NULL);
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    if (*xi != P3D_HUGE && *yi != P3D_HUGE && *zi != P3D_HUGE) {
      g3d_drawOneLine(*xi, *yi, *zi, xf, yf, zf, color, NULL);
    }
  } else {
    xf = *xi;
    yf = *yi;
    zf = *zi;
  }

  ray = ray / 1.5;
  for (i = 0;i < j->n_next_jnt;i++) {
    draw_joint_j(j->next_jnt[i], &xf, &yf, &zf, ray, color);
  }
}

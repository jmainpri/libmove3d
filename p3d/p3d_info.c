#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
//#include "Bio-pkg.h"
//#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "P3d-pkg.h"

#define rob_space() PrintInfo(("             "))
#define obs_space() PrintInfo(("             "))
#define bod_space() PrintInfo(("       "))



double resultArray[9];

/***************************************************************/


/***************************************************************/
/* Fonction affichant la matrice position d'un polyhedre donne */
/* In : le pointeur sur le polyhedre                           */
/* Out :                                                       */
/***************************************************************/
void p3d_get_info_pos_poly(p3d_poly *p) {
  p3d_matrix4 pos;

  p3d_get_poly_pos(p->poly, pos);
  /*tsiano p3d_mat4Print(pos,((Polyhedre *)(p3d_i_collide_get_ptr(p)))->name); */
  p3d_mat4Print(pos, p3d_get_name(p->poly));
}

/**************************************************/
/* Fonction affichant les informations concernant */
/* l'environnement courant                        */
/* In :                                           */
/* Out :                                          */
/**************************************************/
void p3d_env_info(void) {
  int no = p3d_get_desc_number(P3D_OBSTACLE);
  int nr = p3d_get_desc_number(P3D_ROBOT);
  int rob, obs, i;

  PrintInfo(("-------------------------------------------------------------\n"));
  PrintInfo(("ENVIRONMENT: name=%s num=%d id=%0lx no=%d nr=%d\n",
             p3d_get_desc_curname(P3D_ENV),
             p3d_get_desc_curnum(P3D_ENV),
             (unsigned long)p3d_get_desc_curid(P3D_ENV),
             no,
             nr));

  obs = p3d_get_desc_curnum(P3D_OBSTACLE);
  rob = p3d_get_desc_curnum(P3D_ROBOT);

  for (i = 0;i < no;i++) {
    p3d_sel_desc_num(P3D_OBSTACLE, i);
    p3d_obstacle_info();
  }
  p3d_sel_desc_num(P3D_OBSTACLE, obs);

  for (i = 0;i < nr;i++) {
    p3d_sel_desc_num(P3D_ROBOT, i);
    p3d_robot_info();
  }
  p3d_sel_desc_num(P3D_ROBOT, rob);
  PrintInfo(("-------------------------------------------------------------\n"));
}

/**************************************************/
/* Fonction affichant les informations concernant */
/* l'obstacle courant                             */
/* In :                                           */
/* Out :                                          */
/**************************************************/
void p3d_obstacle_info(void) {
  int   i, npt, n;
  double x1, x2, y1, y2, z1, z2;

  p3d_get_obstacle_box(&x1, &x2, &y1, &y2, &z1, &z2);

  PrintInfo(("OBSTACLE   : name=%s num=%d id=%0lx\n",
             p3d_get_desc_curname(P3D_OBSTACLE),
             p3d_get_desc_curnum(P3D_OBSTACLE),
             (unsigned long)p3d_get_desc_curid(P3D_OBSTACLE)));

  obs_space();
  PrintInfo(("box:  x1=%f x2=%f y1=%f y2=%f z1=%f z2=%f\n", x1, x2, y1, y2, z1, z2));

  obs_space();

  for (n = 0;n < p3d_get_obstacle_npoly();n++) {
    npt = p3d_get_obstacle_npt(n);
    PrintInfo(("poly: "));
    for (i = 0;i < npt;i++) {
      p3d_get_obstacle_pt(n, i, &x1, &y1, &z1);
      p3d_get_obstacle_pt(n, i + 1, &x2, &y2, &z2);
      if (i != 0) {
        obs_space();
        PrintInfo(("      "));
      }
      PrintInfo(("(%f %f %f) > (%f %f %f)\n", x1, y1, z1, x2, y2, z1));
    }
  }
}


/**************************************************/
/* Fonction affichant les informations concernant */
/* le robot courant                               */
/* In :                                           */
/* Out :                                          */
/**************************************************/
void p3d_robot_info(void) {
  p3d_rob* robotPt = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
  int   i, n, body;
  double x1, x2, y1, y2, z1, z2, t1, t2, u1, u2, v1, v2;
  static configPt q = NULL;

  if (q == NULL) {
    q = p3d_alloc_config(robotPt);
  }

  n    = p3d_get_desc_number(P3D_BODY);
  body = p3d_get_desc_curnum(P3D_BODY);

  p3d_get_robot_box_deg(&x1, &x2, &y1, &y2, &z1, &z2,
                        &t1, &t2, &u1, &u2, &v1, &v2);

  PrintInfo(("ROBOT      : name=%s num=%d id=%0lx nb=%d nj=%d\n",
             p3d_get_desc_curname(P3D_ROBOT),
             p3d_get_desc_curnum(P3D_ROBOT),
             (unsigned long)p3d_get_desc_curid(P3D_ROBOT),
             n, p3d_get_robot_njnt()));

  rob_space();
  PrintInfo(("box:  x1=%f x2=%f y1=%f y2=%f z1=%f z2=%f t1=%f t2=%f u1=%f u2=%f v1=%f v2=%f\n", x1, x2, y1, y2, z1, z2, t1, t2, u1, u2, v1, v2));

  rob_space();
  p3d_get_robot_pos_deg(q);
  PrintInfo(("pos:  x=%f y=%f z=%f rx=%f ry=%f rz=%fdeg\n", q[0], q[1], q[2], q[3], q[4], q[5]));

  for (i = 0;i < n;i++) {
    p3d_sel_desc_num(P3D_BODY, i);
    p3d_body_info();
  }
  p3d_sel_desc_num(P3D_BODY, body);
}

/**************************************************/
/* Fonction affichant les informations concernant */
/* le body courant                                */
/* In :                                           */
/* Out :                                          */
/**************************************************/
void p3d_body_info(void) {
  int   i, npt, n;
  double x1, x2, y1, y2, z1, z2;

  p3d_get_body_box(&x1, &x2, &y1, &y2, &z1, &z2);

  {
    p3d_obj *o = (pp3d_obj)p3d_get_desc_curid(P3D_BODY);
    p3d_jnt *j = o->jnt;


    if (j->num != 0) {
      rob_space();
      PrintInfo(("JOINT: num = %d ", j->num));
      if (j->type == P3D_ROTATE) PrintInfo(("type=R "));
      else                      PrintInfo(("type=P "));

      if (!j->prev_jnt) PrintInfo(("after ROOT"));
      else             PrintInfo(("after JNT%d", j->prev_jnt->num));
      if (j->next_jnt) {
        PrintInfo(("before"));
        for (i = 0;i < j->n_next_jnt;i++)
          PrintInfo(("JNT%d ", j->next_jnt[i]->num));
      }
      PrintInfo(("\n"));

      PrintInfo(("      x0 =%f    y0=  %f z0  =%f\n",
                 j->pos0[0][3], j->pos0[1][3], j->pos0[2][3]));
      //      if (j->type == P3D_ROTATE)
      //        PrintInfo(("      val=%fdeg vmin=%f vmax=%f\n",
      //                   RTOD(j->v), RTOD(j->vmin), RTOD(j->vmax)));
      //      else
      //        PrintInfo(("      val=%f    vmin=%f vmax=%f\n", j->v, j->vmin, j->vmax));

    }
  }


  rob_space();
  PrintInfo(("BODY:  name=%s num=%d id=%0lx\n",
             p3d_get_desc_curname(P3D_BODY),
             p3d_get_desc_curnum(P3D_BODY),
             (unsigned long)p3d_get_desc_curid(P3D_BODY)));

  rob_space();
  bod_space();
  PrintInfo(("box:  x1=%f x2=%f y1=%f y2=%f z1=%f z2=%f\n", x1, x2, y1, y2, z1, z2));

  rob_space();
  bod_space();
  for (n = 0;n < p3d_get_body_npoly();n++) {
    npt = p3d_get_body_npt(n);
    PrintInfo(("poly: "));
    for (i = 0;i < npt;i++) {
      p3d_get_body_pt(n, i, &x1, &y1, &z1);
      p3d_get_body_pt(n, i + 1, &x2, &y2, &z2);

      if (i != 0) {
        rob_space();
        bod_space();
        PrintInfo(("      "));
      }
      PrintInfo(("(%f %f %f) > (%f %f %f)\n", x1, y1, z1, x2, y2, z2));
    }
  }
  rob_space();

}

void p3d_print_graph(p3d_graph *G) {
  p3d_compco *comp;
  int icomp;
  p3d_node *Ncomp;
  p3d_list_node *list_node;

  if (G) {
    PrintInfo(("Graphe courant : %d noeuds\n", G->nnode));

    PrintInfo(("Composantes connexes :\n"));
    comp = G->comp;
    for (icomp = 1;icomp <= G->ncomp;icomp++) {

      PrintInfo(("Comp. %d ; nombre de noeuds  %d : ", comp->num, comp->nnode));
      /* Ncomp = comp->nodes; */
      list_node = comp->dist_nodes;
      while (list_node != NULL) {
        Ncomp = list_node->N;
        PrintInfo((" %d(%d) ", Ncomp->num, Ncomp->nedge));
        list_node = list_node->next;
      }
      PrintInfo(("\n"));
      comp = comp->suiv;
    }
  }
}

void p3d_print_graph_light(p3d_graph *G) {
  p3d_compco *comp;
  int icomp;

  PrintInfo(("Graphe courant : %d noeuds\n", G->nnode));

  PrintInfo(("Composantes connexes :\n"));
  comp = G->comp;
  for (icomp = 1;icomp <= G->ncomp;icomp++) {
    PrintInfo(("Comp. %d ; nombre de noeuds  %d : ", comp->num, comp->nnode));
    comp = comp->suiv;
  }
}

void p3d_print_info_graph(p3d_graph *G) {
  p3d_list_node *tmp;
  PrintInfo(("Nombre de noeuds du graphe : %d\n", G->nnode));
  PrintInfo(("Nombre d'aretes du graphe : %d\n", G->nedge / 2));
  PrintInfo(("Nombre total de configurations generees : %d\n", G->nb_q));
  if (p3d_random_loop_generator_ok()) {
    PrintInfo(("Nombre de configurations fermees: %d\n", G->nb_q_closed));
    if (p3d_col_get_mode() != p3d_col_mode_bio) {   // IF MODE BIO !!!
      PrintInfo(("Nombre de configurations du backbone sans collision: %d\n", G->nb_bkb_q_free));
    }
  }
  PrintInfo(("Nombre de configurations generees dans l'espace libre: %d\n", G->nb_q_free));
  PrintInfo(("Temps pour construire le graphe : %f\n", G->time));
#ifdef MULTIGRAPH
  if (p3d_get_multiGraph()) {
    PrintInfo(("Temps pour construire les Graphs : %f\n", G->time - G->mgTime));
    PrintInfo(("Temps pour construire le super Graph : %f\n", G->mgTime));
  }
#endif
  PrintInfo(("Appels au test des BB : %d\n", G->nb_test_BB));
  PrintInfo(("Appels au test de collision : %d\n", G->nb_test_coll));
  PrintInfo(("Appels a la methode locale : %d\n", G->nb_local_call));
  //mokhtar
  resultArray[0] = G->nnode;
  resultArray[1] = G->nedge / 2;
  if (!p3d_get_cycles()) {
    resultArray[2] = G->time;//si on est pas en mode création de cycles "non PDR"
  } else {
    resultArray[6] = G->time - resultArray[2]; //PDR
  }
  tmp = G->nodes;
  resultArray[3] = G->nedge / 2 - G->nnode + G->ncomp;
  resultArray[4] = G->nb_test_coll;
  resultArray[5] = G->nb_local_call;
#ifdef MULTIGRAPH
  if (p3d_get_multiGraph()) {
    resultArray[7] = G->mgTime;
    resultArray[8] = G->time - G->mgTime;
  }
#endif
  //mokhtar
}

/*******************************************************/
/********************************************************/
/* Fonction permettant d'afficher l'etat general        */
/* I_collide dans l'environnement s'y rapportant        */
/* In : l'environnement qu'on veut afficher             */
/* Out :                                                */
/********************************************************/
/* void p3d_get_info_state(pp3d_env e) */
/* {int i; */

/*   PrintInfo(("\n\nEtat de l'environnement : %s\n",e->name)); */
/*   PrintInfo(("  Nombre de polyhedres : %d\n",e->state->max_polytopes)); */

/*   PrintInfo(("  Liste des polyhedres : \n")); */
/*   for(i=0;i<e->state->max_polytopes;i++){ */
/*     PrintInfo(("  %s :\n",((Polyhedre *) (e->p3d_lib[i]->polyhedre))->name)); */
/*     p3d_mat4Print(((Polyhedre *) (e->p3d_lib[i]->polyhedre))->pose,"  matrice de position"); */
/*   } */

/*   PrintInfo(("  Liste des polytopes :\n")); */
/*   for(i=0;i<e->state->max_polytopes;i++){ */

/*     PrintInfo(("  id : %d\n",e->state->polytopes[i].id)); */
/*     PrintInfo(("  %s :\n",e->state->polytopes[i].polytope->name)); */
/*     p3d_mat4Print(e->state->polytopes[i].polytope->pose,"  matrice de position"); */

/*   } */

/* } */

/* void p3d_get_info_global_state(void) */
/* {int i; */
/*  col_PairNode *pair; */
/*  p3d_poly *p1,*p2; */

/*   PrintInfo(("\n\nEtat de l'environnement courant\n")); */
/*   PrintInfo(("  Nombre de polyhedres : %d\n",state.max_polytopes)); */

/*   PrintInfo(("  Liste des polyhedres : \n")); */
/*   for(i=0;i<state.max_polytopes;i++){ */
/*     PrintInfo(("  %s :\n",((Polyhedre *) (p3d_lib[i]->polyhedre))->name)); */
/*     p3d_mat4Print(((Polyhedre *) (p3d_lib[i]->polyhedre))->pose,"  matrice de position"); */
/*   } */

/*   PrintInfo(("  Liste des polytopes :\n")); */
/*   for(i=0;i<state.max_polytopes;i++){ */
/*     PrintInfo(("  id : %d\n",state.polytopes[i].id)); */
/*     PrintInfo(("  %s :\n",state.polytopes[i].polytope->name)); */
/*     p3d_mat4Print(state.polytopes[i].polytope->pose,"  matrice de position"); */

/*   } */

/*   pair=state.active_list; */
/*   while(pair != NULL){ */
/*     p1=p3d_get_poly_by_id(pair->ids[0]); */
/*     p2=p3d_get_poly_by_id(pair->ids[1]); */
/*     PrintInfo(("PAIRE dans STATE: [%s %s]\n",((Polyhedre *) (p1->polyhedre))->name,((Polyhedre *) (p2->polyhedre))->name)); */
/*     pair = pair->next_active; */
/*   } */

/* } */



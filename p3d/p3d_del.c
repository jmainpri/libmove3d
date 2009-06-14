#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

static void add_comp(p3d_graph *G, p3d_node *Ns, p3d_node *N);
/* static */ void destroy_length_array(pp3d_rob r);
/* static */ void delete_config(pp3d_rob r, config_namePt config);
#ifdef MULTIGRAPH
static void p3d_del_multiGraphJoint(p3d_multiGraphJoint * mgJoint);
#endif

/*************************************************************************************/

/****************************************/
/* Fonction detruisant une description  */
/* In : le type d'objet a detruire      */
/* Out :                                */
/****************************************/
int p3d_del_desc(int type)
{void *id;
     
    if(p3d_inside_desc()) {
	PrintWarning(("MP: p3d_del_desc:  can't delete in DEF mode\n"));
	return(FALSE);
    }

    id = p3d_get_desc_curid(type);
    if(id == NULL) {
	PrintWarning(("MP: p3d_del_desc:  nothing to delete! \n"));
	return(FALSE);
    }
    
    switch(type) {
	
      case P3D_ENV:
	return(p3d_del_env((pp3d_env) id));

      case P3D_OBSTACLE:
	return(p3d_del_obst((pp3d_obj) id));

      case P3D_ROBOT:

	
	return(p3d_del_rob((pp3d_rob) id));

      /* case P3D_TRAJ: */
	/* return(p3d_del_traj((pp3d_traj) id)); */

      default:
	PrintWarning(("MP: p3d_del_desc: wrong type\n"));
	return(FALSE);
    }
}


/*
 *  Destroy trajectory
 *
 *  Input:  the trajectory
 *
 *  Description: destroy a trajectory by destroying the local paths
 *          composing this trajectory.
 */

int p3d_del_traj(pp3d_traj trajPt)
{
  pp3d_rob robotPt  = trajPt->rob;
  pp3d_traj *newt;
  int      nt,ntcur;
  int      i, j;
  p3d_localpath *destr_lpPt, *localpathPt = NULL ;
  
  /* If trajectory is empty, return NULL */
  if(trajPt == NULL) 
    return FALSE;

  localpathPt = trajPt->courbePt;
  
  /* desallocation of the name (allocated by strdup in p3d_beg_traj) */
  if (trajPt->name != NULL) {
    free(trajPt->name);
    trajPt->name = NULL;
  }
  /* desallocation of the file (allocated by strdup) */
  if (trajPt->file != NULL) {
    free(trajPt->file);
    trajPt->file = NULL;
  }
  /* liberation du tableau de portions de courbes de la trajectoire */
  while(localpathPt != NULL){
    destr_lpPt = localpathPt;
    localpathPt = localpathPt->next_lp;
    destr_lpPt->destroy(robotPt, destr_lpPt);
    }
  trajPt->courbePt = NULL;
        
  /* actualisation du tableau des trajectoires du robot */
  nt = robotPt->nt;
  if(nt == 1)	
    newt = NULL;
  else       
    newt = MY_ALLOC(p3d_traj *, nt-1);
  ntcur = -1;
  j = 0;  /* Modif Fabien pour �viter un d�passement m�moire */
  for(i=0;i<nt;i++) {
    if(trajPt != robotPt->t[i])
      newt[i-j] = robotPt->t[i];
    else {
      ntcur = i; 
      j = 1;  /* Modif Fabien pour �viter un d�passement m�moire */
    }
  }
  if (ntcur == -1) {
    PrintError(("MP: p3d_del_traj: trajectoire n'appartient pas au robot\n"));
    return(FALSE);
  }
  /* Modif Fabien pour �viter un d�passement m�moire
  for(i=ntcur+1;i<nt;i++) {
    newt[i-1]      = robotPt->t[i];
    robotPt->t[i]->num = i-1;
  }
  */
  MY_FREE(robotPt->t,p3d_traj *, nt);
  robotPt->t = newt;
  robotPt->nt= nt-1;
  /* modification de la trajectoire courante du robot */
  if(robotPt->tcur == trajPt) {
    if(robotPt->nt == 0)          
      robotPt->tcur = NULL;
    else if(ntcur == robotPt->nt) 
      robotPt->tcur = newt[ntcur-1];
    else                      
      robotPt->tcur = newt[ntcur];
  }
  /* liberation de la trajectoire */
  trajPt->rob = NULL;
  MY_FREE(trajPt, p3d_traj, 1);
  return(TRUE);
}



/**********************************************/
/* Fonction detruisant un obstacle            */
/* In : le pointeur sur l'obstacle a detruire */
/* Out :                                      */
/**********************************************/
int p3d_del_obst(pp3d_obj o)
{pp3d_env env = o->env;
 pp3d_obj *newo;
 int no,nocur;
 int i;

 if(o) {

/*  PrintInfo(("r : %s\n",o->name)); */
/*  MY_ALLOC_INFO("Avant la destruction d'un obstacle");  */

    /* on verifie que l'objet est bien un obstacle */
    if(o->type != P3D_OBSTACLE) return(FALSE);
    /* liberation du nom (alloue par strdup dans p3d_beg_obj) Rachid */
    if (o->name != NULL) {
      free(o->name);
      o->name = NULL;
    }

    /* liberation du tableau de polyhedres */
    if (o->np>0) { // Modification Fabien
      for(i=0;i<o->np;i++){
	p3d_poly_del_poly(o->pol[i]);
      }
      MY_FREE(o->pol,p3d_poly *,o->np);
    }
    /* actualisation du tableau des obstacles de l'environnement */
    no = env->no;
    if(no == 1)	newo = NULL;
    else       newo =  MY_ALLOC(p3d_obj *, no-1);

    nocur = -1;

    for(i=0;i<no;i++) {
	if(o != env->o[i]) newo[i] = env->o[i];
	else              {nocur = i; break;}
    }

    if (nocur == -1) {
	PrintError(("MP: p3d_del_obst: obstacle n'appartient pas au robot\n"));
	return(FALSE);
    }

    for(i=nocur+1;i<no;i++) {
	newo[i-1]      = env->o[i];
	env->o[i]->num = i-1;
    }

    MY_FREE(env->o,p3d_obj *, no);
    env->o = newo;
    env->no= no-1;

    /* modification de l'obstacle courant de l'environnement */
    if(env->ocur == o) {
	if(env->no == 0)          env->ocur = NULL;
	else if(nocur == env->no) env->ocur = newo[nocur-1];
	else                      env->ocur = newo[nocur];
    }

    /* liberation de l'obstacle */
    o->type = 0;
    o->env  = NULL;
    MY_FREE(o,p3d_obj,1);
 }
    return(TRUE);
}

/* D�but modification Fabien */

/* Destruction d'une configuration */
/* Carl 20062001: used also in UI now => not static */
/* static */ void delete_config(pp3d_rob r, config_namePt config)
{
  if (config != NULL) {
    free(config->name);
    p3d_destroy_config(r, config->q);
    p3d_destroy_specific_iksol(r->cntrt_manager, config->ikSol);
    MY_FREE(config, config_name, 1);
  }
}

/* destruction d'une configuration et mise � jour de la liste des 
   configurations du robot */
int p3d_del_config(pp3d_rob robotPt, config_namePt config)
{
  int i, num;
  config_namePt * new_conf;

  num = -1;
  for(i=0; i<robotPt->nconf; i++) {
    if (robotPt->conf[i] == config) {
      num = i;
      break;
    }
  }
  if (num == -1) {
    PrintInfo(("Configuration non trouvee !\n"));
    return FALSE;
  }
  new_conf = MY_ALLOC(config_namePt, robotPt->nconf);
  if (new_conf == NULL) {
    fprintf(stderr, "Erreur d'allocation memoire dans p3d_del_config !\n");
    return FALSE;
  }

  /* configuration courante */
  if (robotPt->confcur == config) {
    if (robotPt->nconf<=1)
      { robotPt->confcur = NULL; }
    else {
      if (num<robotPt->nconf-1)
	{ robotPt->confcur = robotPt->conf[num+1]; }
      else
	{ robotPt->confcur = robotPt->conf[num-1]; }
    }
  }

  /* Destruction de la config */
  delete_config(robotPt, config);

  /* Mise � jour de la liste des configurations du robot */
  robotPt->nconf --;
  for(i=0; i<num; i++) 
    { new_conf[i] = robotPt->conf[i]; }
  for(i=num; i<robotPt->nconf; i++) 
    { new_conf[i] = robotPt->conf[i+1]; }
  MY_FREE(robotPt->conf, config_namePt, robotPt->nconf+1);
  robotPt->conf = new_conf;
  return TRUE;
}

/* Fin modification Fabien */
  
  
/**********************************************/
/* Fonction detruisant un robot               */
/* In : le pointeur sur le robot a detruire   */
/* Out :                                      */
/**********************************************/
int p3d_del_rob(pp3d_rob r)
{pp3d_env env = r->env;
 pp3d_rob *newr;
 pp3d_jnt jnt;
 int      nr,nrcur;
 int      i,j;
 int traj_id;
 
    /* on verifie que le robot n'a pas deja ete libere */
    if (env == NULL){return(FALSE);}

    /* deleting the prejacobian matrix */
    p3d_jacDelete(r);

     /* liberation du nom (alloue par strdup dans p3d_beg_rob) Rachid */
    if (r->name != NULL) {
      free(r->name);
      r->name = NULL;
    }

    /* D�but modification Fabien */
    /* Lib�ration du GRAPH */
    if (r->GRAPH != NULL)            
      { p3d_del_graph(r->GRAPH); }
    /* Fin modification Fabien */

    /* liberation du tableau des joints */
    for(i=0;i<=r->njoints;i++) {
 	jnt = r->joints[i];
	p3d_jnt_destroy(jnt);
    }    
    MY_FREE(r->joints,p3d_jnt *,r->njoints+1);

    if(r->ikSolPos){
      MY_FREE(r->ikSolPos, int, r->cntrt_manager->ncntrts);
      r->ikSolPos = NULL;
    }
    if(r->ikSolGoto){
      MY_FREE(r->ikSolGoto, int, r->cntrt_manager->ncntrts);
      r->ikSolGoto = NULL;
    }
    if(r->ikSol){
      MY_FREE(r->ikSol, int, r->cntrt_manager->ncntrts);
      r->ikSol = NULL;
    }
    for(int i = 0; i < MAX_TRANSITION; i++){
      if(r->ikSolTransition[i]){
        MY_FREE(r->ikSolTransition[i], int, r->cntrt_manager->ncntrts);
        r->ikSolTransition[i] = NULL;
      }
    }
    /* liberation du tableau des trajectoires */
    while(r->nt > 0) {
      traj_id = r->t[0]->id;  /* added Carl 28032001 */
      p3d_del_traj(r->t[0]);
      trj_set_null(traj_id);  /* added Carl 28032001 */
    }
    /* liberation du tableau des contraintes */
    /* nouveau treatement (jcortes) */
    if(r->cntrt_manager != NULL) {
      p3d_destroy_cntrt_manager(r->cntrt_manager);
      r->cntrt_manager = NULL;
    }

    /* liberation du tableau des body */
    for(i=0;i<r->no;i++) {
      if(r->o[i]->np>0){
	for(j=0;j<r->o[i]->np;j++){
	  p3d_poly_del_poly(r->o[i]->pol[j]);
    (r->o[i]->pol[j])->color = NULL;
    (r->o[i]->pol[j])->primitive_data = NULL;
    (r->o[i]->pol[j])->poly = NULL;
    r->o[i]->pol[j] = NULL;
	}
	MY_FREE(r->o[i]->pol,p3d_poly *,r->o[i]->np);
      }
      if (r->o[i]->name != NULL) {
	free(r->o[i]->name);
	r->o[i]->name = NULL;
      }
      MY_FREE(r->o[i],p3d_obj,1);
    }

    MY_FREE(r->o,p3d_obj *,r->no);
    


    /*liberation des positions initiales et finale */

    /* liberation du tableau des coefficients de longueur pour la distance */
    destroy_length_array(r);

    p3d_destroy_config(r, r->ROBOT_POS);
    p3d_destroy_config(r, r->ROBOT_GOTO);
    for(int i = 0; i < MAX_TRANSITION; i++){
      p3d_destroy_config(r, r->transitionConfigs[i]);
    }

    /* free data relative to local method */
    if (r->local_method_params){
      lm_destroy_params(r, r->local_method_params);
      r->local_method_params = NULL;
    }


    /* Debut modification fabien : liberation des configurations */
    for(i=0; i<r->nconf; i++) 
      { delete_config(r, r->conf[i]); }
    if (r->nconf>0)
      { MY_FREE(r->conf, config_namePt, r->nconf); }
    /* Fin modification fabien */
	
#ifdef MULTIGRAPH
  p3d_del_multiGraph(r, r->mg);
#endif
#ifdef LIGHT_MODE
  MY_FREE(r->isUserDof, int, XYZ_ROBOT->nb_dof);
  p3d_destroy_config(r, r->openChainConf);
	p3d_destroy_config(r, r->closedChainConf);
#endif
    /* actualisation du tableau des robots de l'environnement */
    nr = env->nr;
    if(nr == 1){
      newr = NULL;
    }
    else{
      newr =  MY_ALLOC(p3d_rob *, nr-1);
    }

    nrcur = -1;

    for(i=0;i<nr;i++) {
      if(r != env->robot[i]){
	newr[i] = env->robot[i];
      }
      else{
	nrcur = i; 
	break;
      }
    }
    
    if (nrcur == -1) {
	PrintError(("MP: p3d_del_rob: robot n'appartient pas a env\n"));
	return(FALSE);
    }

    for(i=nrcur+1;i<nr;i++) {
	newr[i-1]      = env->robot[i];
	env->robot[i]->num = i-1;
    }

    MY_FREE(env->robot,p3d_rob *, nr);
    env->robot = newr;
    env->nr= nr-1;

    /* modification du robot courant de l'environnement */
    if(env->cur_robot == r) {
	if(env->nr == 0)          env->cur_robot = NULL;
	else if(nrcur == env->nr) env->cur_robot = newr[nrcur-1];
	else                      env->cur_robot = newr[nrcur];
    }
    /* liberation du robot */
    r->env  = NULL;
    MY_FREE(r,p3d_rob,1);
	
    return(TRUE);
}

/***************************************************/
/* Fonction detruisant un environnement            */
/* In : le pointeur sur l'environnement a detruire */
/* Out :                                           */
/***************************************************/
int p3d_del_env(pp3d_env e)
{int necur,i,no,no2,nr,nr2;
   

    /* on verifie que l'env n'a pas deja ete libere */   
    if(e->num < 0) return(FALSE);

    /* liberation du nom (alloue par strdup dans p3d_beg_env) Rachid */
    if (e->name != NULL){
      free(e->name);
      e->name = NULL;
    }

    /* liberation de tous les obstacles */
    no = e->no;
    no2 = e->no;
    while(e->no > 0){p3d_del_obst(e->o[0]);}

    /* liberation de tous les robots */  
    nr = e->nr;
    nr2 = e->nr;
    while(e->nr > 0){p3d_del_rob(e->robot[0]);}

    if(e->stat){
      destroyStat(&(e->stat));
    }
    /* actualisation du tableau des environnement */
    necur = -1;

    for(i=0;i<XYZ_NUM_ENV;i++){
      if(e == XYZ_TAB_ENV[i]){necur = i; break;}
    }

    if(XYZ_NUM_ENV>1){
      for(i=necur+1;i<XYZ_NUM_ENV;i++) {
	XYZ_TAB_ENV[i-1]      = XYZ_TAB_ENV[i];
	XYZ_TAB_ENV[i]->num   = i-1;
      }
    }

    XYZ_NUM_ENV--;
    /* modification de l'environnement courant */
    if(XYZ_ENV == e) {
	if(XYZ_NUM_ENV == 0)          XYZ_ENV = NULL;
	else if(necur == XYZ_NUM_ENV) XYZ_ENV = XYZ_TAB_ENV[necur-1];
	else                    XYZ_ENV = XYZ_TAB_ENV[necur];
    }

    /* liberation de l'environnement */
    e->num = -1;
    MY_FREE(e,p3d_env,1);
    
    return(TRUE);
}



/*************************************************************************************/
static
void add_comp(p3d_graph *G, p3d_node *Ns, p3d_node *N)
{
 p3d_list_node *neighb;
 int nneighb, in;
 p3d_node *Nv;

 p3d_add_node_compco(N, Ns->comp);

 neighb = N->neighb;
 nneighb = N->nneighb;
 if(nneighb>0){
   for(in=1;in<=nneighb;in++){
     Nv = neighb->N;
     /* PrintInfo(("add_comp : voisin %d dans la composante %d\n", Nv->num, N->comp->num)); */
     if(Nv->num != Ns->num){add_comp(G,N,Nv);}
     neighb = neighb->next;
   }
 }
}

void create_comp(p3d_node *Ns, p3d_node *N, p3d_graph *G)
{p3d_list_node *neighb;
 int nneighb=N->nneighb, in;
 p3d_node *Nv;

  p3d_create_compco(G,N);
  
  neighb = N->neighb;
  nneighb = N->nneighb;
  if(nneighb>0){
    for(in=1;in<=nneighb;in++){
      Nv = neighb->N;
      if(Nv->num != Ns->num){
	/* PrintInfo(("create_comp %d va dans la comp %d\n",Nv->num,N->numcomp)); */
	add_comp(G,N,Nv);
      }
      neighb = neighb->next;
    }
  }
}

void recreate_comp(p3d_node *Ns,p3d_node *N, p3d_compco *comp, p3d_graph *G)
  {p3d_list_node *neighb;
   int nneighb=N->nneighb, in;
   p3d_node *Nv;

   /* PrintInfo(("on insere %d dans la composante %d\n",N->num,comp->num)); */

   p3d_add_node_compco(N, comp);

   neighb = N->neighb;
   nneighb = N->nneighb;
   if(nneighb>0){
     for(in=1;in<=nneighb;in++){
       Nv = neighb->N;
       /* si le voisin n'est pas le noeud dont on est parti */
       if(Nv->num != Ns->num){
	 /* PrintInfo(("%d reste aussi dans la comp %d\n",Nv->num,comp->num)); */
	 add_comp(G,N,Nv);
       }
       neighb = neighb->next;
     }
   }
  }


void split_comp(p3d_graph *G, p3d_compco *comp, p3d_node *N)
  {
   p3d_node *Ntemp;
   p3d_list_node *neighb = N->neighb,*list_node,*destr_node;
   int in,nneighb = N->nneighb;
   
   /* on vide la composante connexe de ses noeuds */

   list_node = comp->dist_nodes;
   while(list_node != NULL) {
     destr_node = list_node;
     list_node = list_node->next;
     MY_FREE(destr_node,p3d_list_node,1);
   }
   comp->nnode = 0;
  
   Ntemp = neighb->N;
   /* pour le premier voisin de N : on remplit la composanet avec lui et ses voisins */
   /* PrintInfo(("le noeud %d reste dans la composante %d\n",Ntemp->num,comp->num)); */
   recreate_comp(N,Ntemp,comp,G);   
   neighb = neighb->next;

   /* pour les autres : on cree une composante pour chacun d'eux et leurs voisins */
   if(nneighb>1){
     for(in=1;in<nneighb;in++){
       Ntemp = neighb->N;
       /* PrintInfo(("on cree une composante pour le voisin %d\n",Ntemp->num)); */
       create_comp(N,Ntemp,G);
       neighb = neighb->next;
     }
   }
   
  }

/***************************************************/
/* Fonction detruisant un noeud d'un graphe        */
/* In : le pointeur sur le noeud a detruire et le  */
/* graphe le contenant                             */
/* Out :                                           */
/***************************************************/
void p3d_del_node(p3d_node *N, p3d_graph *G)
{int nneighb = N->nneighb, nneighbv,nedgev, nnode,in, i;
 p3d_list_node *neighb, *neighbv, *destr_node;
 p3d_list_edge *neighb_edge,*destr_edge;
 p3d_compco *comp;
 p3d_node *Nv;

  if(N->num > G->nnode){PrintInfo(("p3d_del_node : ERREUR : le noeud n'appartient pas au graphe\n"));}


  if(N->nneighb>0){
    neighb = N->neighb;
    nneighb = N->nneighb;
    /* pour tous les voisins de N : */
    for(in=1;in<=nneighb;in++){
      Nv = neighb->N;
      /*   - suppression de N dans leur liste de voisins */
      neighbv = Nv->neighb;
      nneighbv = Nv->nneighb;
      if(nneighbv>1){
	for(i=1;i<=nneighbv;i++){
	  /* PrintInfo(("voisin du voisin : %d\n",neighbv->N->num)); */
	  if(neighbv->N->num == N->num){
	    if(neighbv->prev != NULL){
	      neighbv->prev->next = neighbv->next; 
	    }
	    else{Nv->neighb = neighbv->next;}
	    if(neighbv->next != NULL){
	      neighbv->next->prev = neighbv->prev; 
	    }	      
	    Nv->nneighb = Nv->nneighb-1;
	    MY_FREE(neighbv, p3d_list_node,1);
	    break;
	  }
	  neighbv = neighbv->next;
	}
/* 	neighbv = Nv->neighb; */
/* 	while(neighbv != NULL){ */
/* 	  PrintInfo(("nouveau voisin du voisin : %d\n",neighbv->N->num)); */
/* 	  neighbv = neighbv->next; */
/* 	} */
      }
      else{
	MY_FREE(neighbv,p3d_list_node,1);
	Nv->neighb = NULL;
	Nv->nneighb = 0;
      }

      neighb_edge = Nv->edges;
      nedgev = Nv->nedge;
      /* PrintInfo(("nombre d'aretes de %d : %d\n",Nv->num,nedgev)); */
      /*   - destruction des aretes Nv->N et N->Nv dans les listes */
      /*     d'aretes des noeuds*/
      if(nedgev>1){
	for(i=1;i<=nedgev;i++){
	  /* PrintInfo(("arete : [%d %d]\n",neighb_edge->E->Ni->num,neighb_edge->E->Nf->num)); */
	  if((neighb_edge->E->Nf->num == N->num) || (neighb_edge->E->Ni->num == N->num)){
	    /* PrintInfo(("arete a detruire : [%d %d]\n",neighb_edge->E->Ni->num,neighb_edge->E->Nf->num)); */
	    if(neighb_edge->prev != NULL){
	      /* PrintInfo(("pas le premier maillon de la chaine...\n")); */
	      neighb_edge->prev->next = neighb_edge->next; 
	    }
	    else{Nv->edges = neighb_edge->next;}
	    if(neighb_edge->next != NULL){
	      /* PrintInfo(("pas le dernier maillon de la chaine...\n")); */
	      neighb_edge->next->prev = neighb_edge->prev;
	    }
	    Nv->nedge = Nv->nedge-1;
	    destr_edge = neighb_edge;
	    neighb_edge = neighb_edge->next;
	    MY_FREE(destr_edge,p3d_list_edge,1);
	  }
	  else{
	    neighb_edge = neighb_edge->next;
	  }
	}
      }
      else{
	MY_FREE(neighb_edge,p3d_list_edge,1);
	Nv->edges = NULL;
	Nv->nedge = 0;
      }
      neighb = neighb->next;
    }
  }


  /* pour la composante connexe de N : */
  comp = N->comp;
  nnode = comp->nnode;
  if(nnode>1){
    /*   - scinder la composante si necessaire (N a plusieurs conections) */
    /* on laisse une partie des noeuds dans la composante de N */
    /* et on met les autres dans d'autres composantes */
      split_comp(G,comp,N);
  } else
    { p3d_remove_compco(G, comp); }
    
  /* detruire les aretes partant et venant a N dans la liste des aretes du graphe */
  neighb_edge = G->edges;
  nedgev = G->nedge;
  for(i=1;i<=nedgev;i++){
    if((neighb_edge->E->Ni->num == N->num)||(neighb_edge->E->Nf->num == N->num)){
      if(neighb_edge->next != NULL){
	neighb_edge->next->prev = neighb_edge->prev;
      }
      if(neighb_edge->prev != NULL){
	neighb_edge->prev->next = neighb_edge->next;
      }
      else{G->edges = neighb_edge->next;}
      G->nedge = G->nedge-1;
      destr_edge = neighb_edge;
      neighb_edge = neighb_edge->next;
      MY_FREE(destr_edge->E,p3d_edge,1);
      MY_FREE(destr_edge,p3d_list_edge,1);
    }
    else{
      neighb_edge = neighb_edge->next;
    }
  }
  /* detruire N dans la liste des noeuds du graphe */
  neighb = G->nodes;
  nneighb = G->nnode;
  for(i=1;i<=nneighb;i++){
    /* PrintInfo(("noeud examine : %d\n",neighb->N->num)); */
    if(neighb->N->num == N->num){
      /* PrintInfo(("noeud a detruire : %d\n",neighb->N->num)); */
      if(neighb->next != NULL){
	neighb->next->prev = neighb->prev;
      }
      if(neighb->prev != NULL){
	neighb->prev->next = neighb->next;
      }
      else{/* PrintInfo(("neighb->prev = NULL\n")); */ G->nodes = neighb->next; G->nodes->prev = NULL;}
      G->nnode = G->nnode-1;
      destr_node = neighb;
      MY_FREE(destr_node,p3d_list_node,1);
      break;
    }
    neighb = neighb->next;
  }
  
  neighb = G->nodes;
  nneighb = G->nnode;
  for(i=1;i<=nneighb;i++){
    if(neighb->N->num > N->num){neighb->N->num = neighb->N->num - 1;}
    neighb = neighb->next;
  }


  neighb = N->neighb;
  while(neighb != NULL){
    destr_node = neighb;
    neighb = neighb->next;
    MY_FREE(destr_node,p3d_list_node,1);
  }
  neighb_edge = N->edges;
   while(neighb_edge != NULL){
    destr_edge = neighb_edge;
    neighb_edge = neighb_edge->next;
    MY_FREE(destr_edge,p3d_list_edge,1);
  }

  p3d_APInode_desalloc(G, N);

}

/* Carl 20062001: used also in UI now => not static */
/* static */ void destroy_length_array(pp3d_rob r)
{int njnt = r->njoints;

 MY_FREE(r->length_array,double, njnt+1);
}

#ifdef MULTIGRAPH
void p3d_del_multiGraph(p3d_rob* robot, p3d_multiGraph * mg){
  /*destruction de la partie  multigraph*/
  for(int k = 0; k < mg->nbGraphs; k++){
    if(mg->graphs[k] != NULL){
      p3d_del_graph(mg->graphs[k]);
      mg->graphs[k] = NULL;
    }
    if(mg->mgJoints[k] != NULL){
      p3d_del_multiGraphJoint(mg->mgJoints[k]);
      mg->mgJoints[k] = NULL;
    }
  }
  if(mg->graphs){
    MY_FREE(mg->graphs, p3d_graph*, mg->nbGraphs);
    mg->graphs = NULL;
  }
  if(mg->mgJoints){
    MY_FREE(mg->mgJoints, p3d_multiGraphJoint *, mg->nbGraphs);
    mg->mgJoints = NULL;
  }
  MY_FREE(mg->usedJoint, int, robot->njoints);
  mg->usedJoint = NULL;
  p3d_delFlatSuperGraph(robot, mg->fsg);
  mg->fsg = NULL;
  MY_FREE(mg, p3d_multiGraph, 1);
}

/** \brief Delete a p3d_multiGraphJoint structure
    \param mgJoint the p3d_multiGraphJoint* to free
*/
void p3d_del_multiGraphJoint(p3d_multiGraphJoint * mgJoint){
  if(mgJoint){
    if(mgJoint->joints){
      MY_FREE(mgJoint->joints, int, mgJoint->nbJoints);
    }
    if(mgJoint->cntrts){
      MY_FREE(mgJoint->cntrts, p3d_cntrt, mgJoint->nbJoints);
    }
    MY_FREE(mgJoint, p3d_multiGraphJoint, 1);
  }
}
#endif

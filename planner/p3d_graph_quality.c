#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

/*********************************************/
/* Fonction testant la 'qualite' d'un graphe */
/* (la chance qu'ont deux configurations     */
/* quelconques d etre reliees par le graphe) */
/* In : le graphe, le nombre de tests        */
/* Out : le taux de reussite                 */
/*********************************************/
double p3d_test_quality_graph(p3d_graph *G, int NBTEST)
{
  p3d_node *Ns=NULL,*Ng=NULL,*Nc;
  p3d_compco *comp;
  p3d_list_compco *list_compco=NULL,*last_compco=NULL,*compco, *deb_comp;
  p3d_list_node *list_node;
 int itest,icomp;
 int LINK_S=FALSE,LINK_G=FALSE,ncomp_s=0;
 double dist=0.;
 double NB_SUCCESS=0.,NB_TRY = ((double)(NBTEST));

  if(!G)
  {
    PrintInfo(("p3d_test_quality_graph : ERREUR : G NULL\n"));
    return(0.);
  }

  for(itest=1;itest<=NBTEST;itest++){

    ncomp_s = 0;

    /* on cree deux noeuds pipeau au hasard */
    Ns = p3d_APInode_shoot(G);      
    Ng = p3d_APInode_shoot(G);      

    /* on construit la liste des composantes connexes auxquelles Ns est relie */

    /* si on a plus d'un noeud */
    if(G->nnode>=1){
      comp = G->comp;
      for(icomp=1;icomp<=G->ncomp;icomp++){
	/* PrintInfo(("comp : %d\n",comp->num)); */
	LINK_S = FALSE;

	list_node = comp->dist_nodes;
	/* on parcourt tous les noeuds de la composante connexe */
	while(list_node != NULL) {
	  dist = 0.;
	  Nc = list_node->N;
	  /* PrintInfo(("noeud examine : %d\n",Nc->num)); */
	  /* si le noeud est reliable, on arrete */
	  if(p3d_APInode_linked(G,Ns,Nc,&dist)){
	    /* PrintInfo(("LINK_S !!\n")); */
	    LINK_S = TRUE;
	    ncomp_s = ncomp_s + 1;
	    compco = MY_ALLOC(p3d_list_compco,1);
	    compco->comp = comp;
	    compco->next = NULL;
	    if(ncomp_s==1) {
	      list_compco = compco; 
	      last_compco = compco;
	    } else {
	      last_compco->next = compco; 
	      last_compco = last_compco->next;
	    }
	    break;
	  }
	  list_node = list_node->next;
	}
	comp = comp->suiv;
      }
    }


    LINK_G = FALSE;
    if(ncomp_s>0){
      LINK_S = TRUE;
      deb_comp = list_compco;
      for(icomp=1;icomp<=ncomp_s;icomp++){
	comp = deb_comp->comp;
	/* PrintInfo(("on examine la composante %d\n",comp->num)); */
	list_node = comp->dist_nodes;
	while(list_node != NULL) {
	  dist = 0.;
	  Nc = list_node->N;
	  if(p3d_APInode_linked(G,Ng,Nc,&dist)){
	    /* PrintInfo(("LINK_G !!\n")); */
	    LINK_G = TRUE;
	    break;
	  }
	  list_node = list_node->next;
	}
	if(!LINK_G)
	  { deb_comp = deb_comp->next; }
	else
	  { break; }
      }
    }
    else{LINK_S = FALSE;}


  
    if(LINK_S && LINK_G){NB_SUCCESS = NB_SUCCESS + 1;}
/*     PrintInfo(("test %d effectue : ",itest)); */
/*     if(LINK_S && LINK_G){PrintInfo(("SUCCESS\n"));} */
/*     else{PrintInfo(("FAILURE\n"));} */
    
    p3d_APInode_desalloc(G,Ns);
    p3d_APInode_desalloc(G,Ng);

    if(ncomp_s>0){
      compco = list_compco;
      while(compco != NULL) {
	last_compco = compco;
	compco = compco->next;
	MY_FREE(last_compco,p3d_list_compco,1);
      }
    }
  }
  return(NB_SUCCESS/NB_TRY);
}


/***********************************************/
/* Fonction testant la 'qualite' d'un graphe,  */
/* la chance qu'a une configuration quelconque */
/* d etre reliee au graphe                     */
/* In : le graphe, le nombre de tests          */
/* Out : le taux de reussite                   */
/***********************************************/
double p3d_test_good_graph(p3d_graph *G, int NBTEST)
{
 p3d_node *Ns=NULL,*Nc;
 p3d_compco *comp;
 p3d_list_node *list_node;
 int itest,icomp;
 int LINK_S=FALSE;
 double dist=0.;
 double NB_SUCCESS=0.,NB_TRY = ((double)(NBTEST));


  if(!G){PrintInfo(("p3d_test_good_graph : ERREUR : G NULL\n");return(0.));}

  for(itest=1;itest<=NBTEST;itest++){

    /* on cree une conf pipeau au hasard */
    Ns = p3d_APInode_shoot(G);

    LINK_S=FALSE;

    /* on essaye de la relier au graphe */

    /* si on a plus d'un noeud */
    if(G->nnode>=1){
      comp = G->comp;
      for(icomp=1;icomp<=G->ncomp;icomp++){
	/* PrintInfo(("comp : %d\n",comp->num)); */
	
	list_node = comp->dist_nodes;
	/* on parcourt tous les noeuds de la composante connexe */
	while(list_node != NULL) {
	  Nc = list_node->N;
	  /* PrintInfo(("noeud examine : %d\n",Nc->num)); */
	  /* si le noeud est reliable, on arrete */
	  if(p3d_APInode_linked(G,Ns,Nc,&dist)){
	    /* PrintInfo(("LINK_S !!\n")); */
	    LINK_S = TRUE;
	    break;
	  }
	  list_node = list_node->next;
	}
	if (LINK_S)
	  { break; }
	else
	  { comp = comp->suiv; }
      }
    }

    if(LINK_S){NB_SUCCESS = NB_SUCCESS + 1;}
    p3d_APInode_desalloc(G,Ns);
  }

  return(NB_SUCCESS/NB_TRY);
}

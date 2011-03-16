#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
//#include "Bio-pkg.h"
#include "Collision-pkg.h"

#define KD_GRAPH_BUFF_BLOCSIZE	1024
#define KD_NCS 256


static void write_graph(p3d_graph *G, FILE *fd);
static void write_comp(p3d_graph *G, p3d_compco *comp, FILE *fd);
static void write_node(p3d_graph *G, p3d_node *N, FILE *fd);

static int write_graph_buff(p3d_graph *G,size_t* buffer, size_t *bsize, size_t *MaxAlloc);
static int write_comp_buff(p3d_graph *G, p3d_compco *comp, size_t* buffer, size_t *bsize, size_t *MaxAlloc);
static int write_node_buff(p3d_graph *G, p3d_node *N, size_t* buffer, size_t *bsize, size_t *MaxAlloc);
static int graph_realloc_buff(void** buffer, size_t *MaxAlloc);

static int read_graph_buff(size_t buffer, double *version);


/***************************************************************/
/* Fonction ecrivant un graphe sur un fichier                  */
/***************************************************************/
int p3d_write_graph(p3d_graph *G, const char *file)
{
  FILE *fd;
  
  if(!(fd = fopen(file,"w"))){
    PrintError(("MP: p3d_read_desc: can't open %s\n",file));
    return(FALSE);
  }
  write_graph(G,fd);
  if (G->file != NULL)
    { MY_STRFREE(G->file); }
  G->file = MY_STRDUP(file);
  fclose(fd);
  return(TRUE);
}
      
/***************************************************************/
/* Fonction ecrivant un graphe dans un buffer                  */
/***************************************************************/
int p3d_write_graph_buff(p3d_graph *G, void** buffer, size_t *bsize)
// buffer is allocated and must be freed
{
	size_t MaxAlloc = 0;
	double version;

	// verif
	if (*buffer != NULL)
	{
		PrintError(("MP: p3d_graph_write_buff: buffer must be == NULL\n"));
		return FALSE;
	}

	// alloc
	if (graph_realloc_buff(buffer, &MaxAlloc) == FALSE) return FALSE;
	*bsize = 0;

	// write version
	version = 1.000;  // devrait correspondre � un #define global � l'application

	memcpy((void*)((size_t)*buffer + *bsize), (void*)&version, sizeof(double));
	*bsize += sizeof(double);

	// write graph
	if (write_graph_buff(G, (size_t*)buffer, bsize, &MaxAlloc) == FALSE)
	{
		PrintError(("MP: graph_write_buff: failed\n"));
		return FALSE;
	}

	return(TRUE);
}
      
/***************************************************************/
/* Fonction reconstruisant un graphe a partir d'un fichier     */
/***************************************************************/
int p3d_read_graph(char *file/*, pp3d_graph *pG*/) // Modification Fabien
{FILE *fd;
 p3d_graph *G;

 /* D�but modification Fabien */
  if(!(fd=fopen(file,"r"))) {
    PrintError(("MP: p3d_read_graph: can't open %s\n",file));
    return(FALSE);
  }
 
  /* On garde le graph courant en m�moire en cas d'�chec de lecture */
  G = XYZ_GRAPH;
  XYZ_GRAPH = NULL;
  if (G!=NULL)
    { G->rob->GRAPH = NULL; }

  if (!p3d_read_this_graph(fd,&XYZ_GRAPH,(p3d_env*)p3d_get_desc_curid(P3D_ENV),
			   (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT))) {
    printf("llego despues de p3d_read_this_graph FALSE\n");
    if (G!=NULL) {  /* Restauration de l'ancien graphe */
      XYZ_GRAPH = G;
      G->rob->GRAPH = G;
    }
    fclose(fd);
    return(FALSE);
  }
  
  printf("llego despues de p3d_read_this_graph TRUE\n");

  if (G!=NULL)    /* Effacement de l'ancien graphe */
    { p3d_del_graph(G); }
  XYZ_GRAPH->rob->GRAPH = XYZ_GRAPH;
  XYZ_GRAPH->file = MY_STRDUP(file);
  /* Fin modification Fabien */
  fclose(fd);
  return(TRUE);
}

/***************************************************************/
/* Fonction reconstruisant un graphe a partir d'un buffer     */
/***************************************************************/
int p3d_read_graph_buff(void* buffer, double *version)
{
	p3d_graph *G;
	size_t	ptr;

	ptr = (size_t)buffer; // keep buffer at start value

	/* On garde le graphe courant en m�moire en cas d'�chec de lecture */
	G = XYZ_GRAPH;
	XYZ_GRAPH = NULL;
	if (G!=NULL)
	{
		G->rob->GRAPH = NULL;
	}

	if (!read_graph_buff(ptr, version))
	{
		if (G!=NULL)	/* Restauration de l'ancien graphe */
		{  
			  XYZ_GRAPH = G;
			  G->rob->GRAPH = G;
		}
		return(FALSE);
	}
	if (G!=NULL)	/* Effacement de l'ancien graphe */
	{    
		p3d_del_graph(G); 
		XYZ_GRAPH->rob->GRAPH = XYZ_GRAPH;
	}

	if (XYZ_GRAPH->file != NULL)
	{
		basic_free(XYZ_GRAPH->file, 1,1);
		XYZ_GRAPH->file = NULL;
	}

	return(TRUE);
}

/***************************************************************/


static void write_graph(p3d_graph *G, FILE *fd)
{int icomp;
 p3d_compco *comp; 

   fprintf(fd,"p3d_env %s \n",G->env->name);

   fprintf(fd,"p3d_robot %s \n\n",G->rob->name);

   fprintf(fd,"p3d_nnodes %d \n\n",G->nnode);

   fprintf(fd,"p3d_nbq %d \n",G->nb_q);
    
   if(p3d_random_loop_generator_ok()) {
     fprintf(fd,"p3d_nbq_closed %d \n",G->nb_q_closed);
     if(p3d_col_get_mode()!=p3d_col_mode_bio) {   // IF MODE BIO !!!
       fprintf(fd,"p3d_nbq_bkb_free %d \n",G->nb_bkb_q_free);
     }
   }

   fprintf(fd,"p3d_nbq_free %d \n",G->nb_q_free);

   fprintf(fd,"p3d_time %f \n",G->time);

   fprintf(fd,"p3d_local_call %d \n",G->nb_local_call);

   fprintf(fd,"p3d_test_coll %d \n",G->nb_test_coll);

   fprintf(fd,"p3d_HHCount %d \n\n\n",(int)G->hhCount);

   comp = G->comp;
   for(icomp=1;icomp<=G->ncomp;icomp++){
     write_comp(G,comp,fd);
     comp=comp->suiv;
   }
}
    
// buffer version
static int write_graph_buff(p3d_graph *G,size_t* buffer, size_t *bsize, size_t *MaxAlloc)
{
	int icomp;
	p3d_compco *comp;
	char maxstr[KD_NCS];
	size_t local_size;

	// global memory size
	local_size = 2 * KD_NCS * sizeof(char) + sizeof(double) + 5 * sizeof(int);

	while (*bsize + local_size	> *MaxAlloc)
	{
		if (graph_realloc_buff((void**)buffer, MaxAlloc) == FALSE) return FALSE;
	}


	// fprintf(fd,"p3d_env %s \n",G->env->name);
	strncpy(maxstr, G->env->name, KD_NCS * sizeof(char));
	memcpy((void*)(/*(size_t)*/*buffer + *bsize),
		(void*)maxstr, 
		KD_NCS * sizeof(char));
	*bsize += KD_NCS * sizeof(char);

	// fprintf(fd,"p3d_robot %s \n\n",G->rob->name);
	strncpy(maxstr, G->rob->name, KD_NCS * sizeof(char));
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)maxstr, KD_NCS * sizeof(char));
	*bsize += KD_NCS * sizeof(char);

	// fprintf(fd,"p3d_nnodes %d \n\n",G->nnode);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->nnode, sizeof(int));
	*bsize += sizeof(int);

	// fprintf(fd,"p3d_nbq %d \n",G->nb_q);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->nb_q, sizeof(int));
	*bsize += sizeof(int);

	// fprintf(fd,"p3d_nbq_free %d \n",G->nb_q_free);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->nb_q_free, sizeof(int));
	*bsize += sizeof(int);

	// fprintf(fd,"p3d_time %f \n",G->time);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->time, sizeof(double));
	*bsize += sizeof(double);

	// fprintf(fd,"p3d_local_call %d \n",G->nb_local_call);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->nb_local_call, sizeof(int));
	*bsize += sizeof(int);

	// fprintf(fd,"p3d_test_coll %d \n\n",G->nb_test_coll);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->nb_test_coll, sizeof(int));
	*bsize += sizeof(int);



   	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&G->ncomp, sizeof(int));
	*bsize += sizeof(int);

  comp = G->comp;

  for(icomp=1;icomp<=G->ncomp;icomp++){
     if (write_comp_buff(G,comp, buffer, bsize, MaxAlloc) == FALSE) return FALSE;
     comp=comp->suiv;
   }

   return TRUE;
}
    
// reallocate for buffer version
static int graph_realloc_buff(void** buffer, size_t *MaxAlloc)
{

	if (*buffer == NULL)
	{	// alloc
		*buffer = (void*)basic_alloc(1, KD_GRAPH_BUFF_BLOCSIZE);
		if (*buffer == NULL)
			return FALSE;

		*MaxAlloc = KD_GRAPH_BUFF_BLOCSIZE;
	}
	else
	{	// realloc
		*buffer = basic_realloc(*buffer, *MaxAlloc, *MaxAlloc+KD_GRAPH_BUFF_BLOCSIZE, 1);
		if (*buffer == NULL)
			return FALSE;

		*MaxAlloc += KD_GRAPH_BUFF_BLOCSIZE;
	}

	return TRUE;
}

/***************************************************************/

static void write_comp(p3d_graph *G, p3d_compco *comp, FILE *fd)
{int num;
 p3d_node *Ncomp;
 p3d_list_node *list_node;

 num = comp->num; 
 fprintf(fd,"p3d_comp %d \n\n",num);


 list_node = comp->dist_nodes;
 while(list_node != NULL) {
   Ncomp = list_node->N;
   write_node(G,Ncomp,fd);
   list_node = list_node->next;
 }

 fprintf(fd,"p3d_end_comp \n\n");
}


//
// version avec buffer
static int write_comp_buff(p3d_graph *G, p3d_compco *comp, size_t* buffer, size_t *bsize, size_t *MaxAlloc)
{
	int num;
	p3d_node *Ncomp;
	p3d_list_node *list_node;

	if (*bsize + 2 * sizeof(int) > *MaxAlloc)
		if (graph_realloc_buff((void**)buffer, MaxAlloc) == FALSE) return FALSE;

	num = comp->num;

	// fprintf(fd,"p3d_comp %d \n\n",num);
	memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&num, sizeof(int));
	*bsize += sizeof(int);


	memcpy((void*)(*buffer + *bsize), (void*)&comp->nnode, sizeof(int));
	*bsize += sizeof(int);

	list_node = comp->dist_nodes;
	while(list_node != NULL) {
	  Ncomp = list_node->N;
	  write_node_buff(G,Ncomp,buffer, bsize, MaxAlloc);
	  list_node = list_node->next;
	}

	return TRUE;
}

/***************************************************************/

static void write_node(p3d_graph *G, p3d_node *N, FILE *fd)
{
  int ndof,ij,num,nneighb,ineighb,nedge,iedge;
  p3d_list_node *neighb;
  p3d_list_edge *edges;
  double *qdeg, radius;
  plm_reeds_shepp_str rs_paramPt=NULL;

  num = N->num; 
  fprintf(fd,"  p3d_node %d \n",num);

  fprintf(fd,"    n_fail_extend %d\n",N->n_fail_extend);
  fprintf(fd,"    weight %f\n",N->weight);
  if(N->iksol){
    fprintf(fd,"    ikSol");
    for(int i = 0; i < G->rob->cntrt_manager->ncntrts; i++){
      fprintf(fd, " %d", N->iksol[i]);
    }
  fprintf(fd,"\n");
  }  fprintf(fd,"    p3d_q");
  qdeg = p3d_copy_config_rad_to_deg(G->rob,N->q);
  ndof = G->rob->nb_dof;
  for(ij=0; ij<ndof; ij++){
    fprintf(fd," %f ",qdeg[ij]);
  }
  fprintf(fd,"\n");
  p3d_destroy_config(G->rob, qdeg);


  nneighb = N->nneighb;
  fprintf(fd,"    p3d_neighb %d ",nneighb);
  
  neighb = N->neighb;
  for(ineighb=1;ineighb<=nneighb;ineighb++){
    fprintf(fd," %d ",neighb->N->num);
    neighb = neighb->next;
  }
  fprintf(fd,"\n");

  nedge = N->nedge;
  fprintf(fd,"    p3d_nedge %d \n",nedge);

  edges = N->edges;
  for(iedge=1;iedge<=nedge;iedge++){
    fprintf(fd,"      p3d_edge %d %d %s %f",edges->E->Ni->num,edges->E->Nf->num,
	    p3d_local_getname_planner(edges->E->planner),edges->E->longueur);

    if ((edges->E->planner == REEDS_SHEPP) || 
	(edges->E->planner == DUBINS)) {
      rs_paramPt = lm_get_reeds_shepp_lm_param(G->rob);
      if (rs_paramPt == NULL){
	radius = -1;
      }
      else {
	radius = rs_paramPt->radius;
      }
       
      fprintf(fd," p3d_radius %f",radius);
    }
    fprintf(fd,"\n");
    edges = edges->next;
  }
  fprintf(fd,"  p3d_end_node \n\n");
    
}

/***************************************************************/
// modif Juan
// write_node on shell
void p3d_print_node(p3d_graph *G, p3d_node *N)
{
  int ndof,ij,num,nneighb,ineighb,nedge,iedge;
  p3d_list_node *neighb;
  p3d_list_edge *edges;
  double *qdeg, radius;
  plm_reeds_shepp_str rs_paramPt=NULL;
 
  num = N->num; 
  printf("  p3d_node %d \n",num);
  printf("  compco %d\n", N->numcomp);
  
  printf("    p3d_q :deg\t\trad\n");
  qdeg = p3d_copy_config_rad_to_deg(G->rob,N->q);
  ndof = G->rob->nb_dof;
  for(ij=0; ij<ndof; ij++){
    printf("            q[%d] = %f\t\t%f\n",ij,qdeg[ij],N->q[ij]);
  }
  printf("\n");
  p3d_destroy_config(G->rob, qdeg);

  nneighb = N->nneighb;
  printf("    p3d_neighb %d ",nneighb);
  
  neighb = N->neighb;
  for(ineighb=1;ineighb<=nneighb;ineighb++){
    printf(" %d ",neighb->N->num);
    neighb = neighb->next;
  }
  printf("\n");

  nedge = N->nedge;
  printf("    p3d_nedge %d \n",nedge);

  edges = N->edges;
  for(iedge=1;iedge<=nedge;iedge++){
    printf("      p3d_edge %d %d %s %f Cost %f",edges->E->Ni->num,edges->E->Nf->num,
	    p3d_local_getname_planner(edges->E->planner),edges->E->longueur, edges->E->cost);

    if ((edges->E->planner == REEDS_SHEPP) || 
	(edges->E->planner == DUBINS)) {
      rs_paramPt = lm_get_reeds_shepp_lm_param(G->rob);
      if (rs_paramPt == NULL){
	radius = -1;
      }
      else {
	radius = rs_paramPt->radius;
      }
       
      printf(" p3d_radius %f",radius);
    }
    printf("\n");
    edges = edges->next;
  }
  printf(" Is singularity %d\n",N->isSingularity);
  printf(" Cost %f\n",N->cost);
  if (N->iksol){
    printf("    ikSol :\n");
    for(ij = 0; ij < G->rob->cntrt_manager->ncntrts; ij++){
      printf("            %d\n", N->iksol[ij]);
    }
  }
  
  printf("  p3d_end_node \n\n");
    
}

/***************************************************************/

// buffer version
static int write_node_buff(p3d_graph *G, p3d_node *N, size_t* buffer, 
			   size_t *bsize, size_t *MaxAlloc)
{
  int ndof,ij,num,nneighb,ineighb,nedge,iedge;
  p3d_list_node *neighb;
  p3d_list_edge *edges;
  double *qdeg;
  size_t local_size;
  double radius;
  plm_reeds_shepp_str rs_paramPt=NULL;

  ndof = G->rob->nb_dof;
  // global memory size for this function
  // i + i + ndof*f + i + N->nneighb * i + i + N->nedge *( i i i f + i)
  local_size = (3 + N->nneighb + 4 * N->nedge) * sizeof(int) + (ndof + N->nedge) * sizeof(double);
  while (*bsize + local_size	> *MaxAlloc)
    {
      if (graph_realloc_buff((void**)buffer, MaxAlloc) == FALSE) return FALSE;
    }



  num = N->num; 
  // fprintf(fd,"  p3d_node %d \n",num);
  memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&num, sizeof(int));  // num: int
  *bsize += sizeof(int);

  // fprintf(fd,"    p3d_q");
  qdeg = p3d_copy_config_rad_to_deg(G->rob,N->q);

  memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&ndof, sizeof(int));	// ndof: int
  *bsize += sizeof(int);

  for(ij=0;ij<ndof;ij++)
    {
      // fprintf(fd," %f ",qdeg[ij]);
      memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&qdeg[ij], sizeof(double));	// qdeg[]: double (* ndof)
      *bsize += sizeof(double);
    }
  // fprintf(fd,"\n");
  p3d_destroy_config(G->rob, qdeg);

  nneighb = N->nneighb;
	
	// fprintf(fd,"    p3d_neighb %d ",nneighb);
  memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&nneighb, sizeof(int));	// nneighb: int
  *bsize += sizeof(int);

  neighb = N->neighb;
  for(ineighb=1;ineighb<=nneighb;ineighb++)
    {
      // fprintf(fd," %d ",neighb->N->num);
      memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&neighb->N->num, sizeof(int));	// neighb->N->num: int (* nneighb)
      *bsize += sizeof(int);

      neighb = neighb->next;
    }
  // fprintf(fd,"\n");

  nedge = N->nedge;
  // fprintf(fd,"    p3d_nedge %d \n",nedge);
  memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&nedge, sizeof(int));	// nedge: int
  *bsize += sizeof(int);

  edges = N->edges;
  for(iedge=1;iedge<=nedge;iedge++)
    {
      /* fprintf(fd,"      p3d_edge %d %d %s %f",edges->E->Ni->num,edges->E->Nf->num, 
	 p3d_local_getname_planner(edges->E->planner),edges->E->longueur); */

      memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&edges->E->Ni->num, sizeof(int));	// edges->E->Ni->num: int (* nedge)
      *bsize += sizeof(int);
      memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&edges->E->Nf->num, sizeof(int));	// edges->E->Nf->num: int (* nedge)
      *bsize += sizeof(int);
      memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&edges->E->planner, sizeof(int));	// edges->E->planner: int (* nedge)
      *bsize += sizeof(int);
      memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&edges->E->longueur, sizeof(double));	// edges->E->longueur: double (* nedge)
      *bsize += sizeof(double);

      if((edges->E->planner == REEDS_SHEPP) ||
	 (edges->E->planner == DUBINS))
	{
	  rs_paramPt = lm_get_reeds_shepp_lm_param(G->rob);
	  if (rs_paramPt == NULL){
	    radius = -1;
	  }
	  else {
	    radius = rs_paramPt->radius;
	  }
	  // fprintf(fd," p3d_radius %f",G->rob->radius);
	  memcpy((void*)(/*(size_t)*/*buffer + *bsize), (void*)&radius, sizeof(double));	// G->rob->radius: double (* nedge)
	  *bsize += sizeof(double);
	}

      // fprintf(fd,"\n");
      edges = edges->next;
    }
  // fprintf(fd,"  p3d_end_node \n\n");

  return TRUE;

}


/*--------------------------------------------------------------------------*/
/*! \brief Funtion to create the canreach list of component
 *
 *  \param graphPt: the graph to update
 *
 *  Test all the nodes of each component. If one node is linked
 *  to an other component, then add it to the canreach list.
 *
 *  \internal
 */
static void set_canreach_compco(p3d_graph * graphPt)
{
  p3d_compco * compcoPt;
  p3d_list_node * list_nodePt, * list_neighbPt;

  compcoPt = graphPt->comp;
  while(compcoPt != NULL) {
    list_nodePt = compcoPt->dist_nodes;

    while(list_nodePt != NULL) {
      list_neighbPt = list_nodePt->N->neighb;

      while(list_neighbPt != NULL) {
	if (list_neighbPt->N->comp != compcoPt) {
	  p3d_add_compco_to_reachable_list(compcoPt, 
					   list_neighbPt->N->comp);
	}
	list_neighbPt = list_neighbPt->next;
      }
      list_nodePt = list_nodePt->next;
    }
    compcoPt = compcoPt->suiv;
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Funtion to create one edge
 *
 *  \param graphPt:  the graph to update
 *  \param node_iPt: the initial node
 *  \param node_fPt: the final node
 *  \param planner:  the planner type
 *  \param dist:     the distance of the edge
 *
 *  \return the edge created (NULL if error)
 *
 *  Do not use component, then the canreach list is not updated.
 *  Need the call of set_canreach_compco() 
 *
 *  \internal
 */
static p3d_edge * set_one_edge(p3d_graph * graphPt, p3d_node * node_iPt,
			       p3d_node * node_fPt,
			       p3d_localpath_type type, double dist)
{
  p3d_edge * edgePt;
  p3d_list_edge * list_edge1Pt, * list_edge2Pt;

  edgePt = MY_ALLOC(p3d_edge, 1);
  list_edge1Pt = MY_ALLOC(p3d_list_edge, 1);
  list_edge2Pt = MY_ALLOC(p3d_list_edge, 1);
  if ((edgePt == NULL) || (list_edge1Pt == NULL) || (list_edge2Pt == NULL)) {
    PrintError(("Not enough memory !!!\n"));
    return NULL; 
  }
  memset(edgePt, 0, sizeof(p3d_edge));
  memset(list_edge1Pt, 0, sizeof(p3d_list_edge));
  memset(list_edge2Pt, 0, sizeof(p3d_list_edge));

  edgePt->Ni = node_iPt;
  edgePt->Nf = node_fPt;
  edgePt->planner = type;
  edgePt->longueur = dist;

  /* Put the list in the current node */
  node_iPt->nedge ++;
  list_edge1Pt->E = edgePt;
  if (node_iPt->edges == NULL) 
    { node_iPt->edges = list_edge1Pt; }
  else
    { node_iPt->last_edge->next = list_edge1Pt; }
  list_edge1Pt->prev = node_iPt->last_edge;
  node_iPt->last_edge = list_edge1Pt;

  /* Put the list in the graph */
  graphPt->nedge ++;
  list_edge2Pt->E = edgePt;
  if (graphPt->edges == NULL) 
    { graphPt->edges = list_edge2Pt; }
  else
    { graphPt->last_edge->next = list_edge2Pt; }
  list_edge2Pt->prev = graphPt->last_edge;
  graphPt->last_edge = list_edge2Pt;

  return edgePt;
}      


/////////////////////////
// version buffer
static int read_graph_buff(size_t buffer, double *version)
{
	int				itab[2000],n,nnode=0,inode;
	int				i, j, k, /*ij,iloc,*/ buf_i;
	int				local_nedge, local_g_ncomp, local_comp_nnode;
//	char			fct[100];
	char			name[256];
	char			buf_s[KD_NCS];
//	double			dtab[50];
	double			*qdeg, buf_d;
	p3d_rob			*robbie = NULL;
	p3d_env			*env;
	p3d_edge		*e;
	p3d_node		*N= NULL, **tab_node= NULL;
	p3d_graph		*G = NULL;
	p3d_compco		*comp= NULL;
	p3d_list_node	*list_node;


	G = p3d_create_graph();

	// get version	
	memcpy((void*)version, (void*)buffer, sizeof(double));
	/*(size_t)*/buffer += sizeof(double);

	// get p3d_env name
	memcpy((void*)buf_s, (void*)buffer, KD_NCS * sizeof(char));
	/*(size_t)*/buffer += KD_NCS * sizeof(char);
	strcpy(name, buf_s);

	 // on verifie que le graphe correspond a l'environnement courant */
	env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	if(strcmp(env->name,name) != 0)
	{
		PrintInfo(("read_graph_buff : environnement courant : %s ; environnement lu : %s\n",env->name,name));
		goto error;
	}


	// get p3d_robot name
	memcpy((void*)buf_s, (void*)buffer, KD_NCS * sizeof(char));
	/*(size_t)*/buffer += KD_NCS * sizeof(char);
	strcpy(name, buf_s);

	// on verifie que le robot correspond au robot courant
	robbie = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	if(strcmp(robbie->name,name)!= 0)
	{
		PrintInfo(("read_graph_buff : robot courant : %s ; robot lu : %s\n", robbie->name,name));	
		goto error;
	}

	// get node number
	memcpy((void*)&nnode, (void*)buffer, sizeof(int));
	/*(size_t)*/buffer += sizeof(int);

	G->nnode = nnode;
	/* on alloue les noeuds */
	tab_node = MY_ALLOC(pp3d_node,nnode);
	/* on cree la liste des noeuds du graphe */
	list_node = MY_ALLOC(p3d_list_node,1);
	tab_node[0] = MY_ALLOC(p3d_node,1);
	tab_node[0]->num = 1;
	tab_node[0]->numcomp = 0;
	tab_node[0]->comp = NULL;
	tab_node[0]->q = NULL;
	tab_node[0]->dq = 0.;
	tab_node[0]->dqmin = 0.;
	tab_node[0]->nneighb = 0;
	tab_node[0]->neighb = NULL;
	tab_node[0]->last_neighb = NULL;
	tab_node[0]->nedge = 0;
	tab_node[0]->edges = NULL;
	tab_node[0]->last_edge = NULL;

	list_node->N = tab_node[0]; 
	list_node->next = NULL; 
	list_node->prev = NULL; 
	G->nodes = list_node;
	G->last_node = list_node;

	for (inode = 1; inode < nnode; inode++)
	{
		list_node = MY_ALLOC(p3d_list_node,1);
		tab_node[inode] = MY_ALLOC(p3d_node,1);
		tab_node[inode]->num = inode+1;
		tab_node[inode]->numcomp = 0;
		tab_node[inode]->comp = NULL;
		tab_node[inode]->q = NULL;
		tab_node[inode]->dq = 0.;
		tab_node[inode]->dqmin = 0.;
		tab_node[inode]->nneighb = 0;
		tab_node[inode]->neighb = NULL;
		tab_node[inode]->last_neighb = NULL;
		tab_node[inode]->nedge = 0;
		tab_node[inode]->edges = NULL;
		tab_node[inode]->last_edge = NULL;

		list_node->N = tab_node[inode]; 
		list_node->next = NULL; 
		G->last_node->next = list_node;
		list_node->prev = G->last_node;
		G->last_node = G->last_node->next;
	}

	// get p3d_nbq
	memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
	/*(size_t)*/buffer += sizeof(int);
	G->nb_q = buf_i;

	// get p3d_nbq_free
	memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
	/*(size_t)*/buffer += sizeof(int);
	G->nb_q_free = buf_i;

	// get p3d_time
	memcpy((void*)&buf_d, (void*)buffer, sizeof(double));
	/*(size_t)*/buffer += sizeof(double);
	G->time = buf_d;

	// get p3d_local_call
	memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
	/*(size_t)*/buffer += sizeof(int);
	G->nb_local_call = buf_i;

	// get p3d_test_coll
	memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
	/*(size_t)*/buffer += sizeof(int);
	G->nb_test_coll = buf_i;

	// get composante connexe number
	memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
	/*(size_t)*/buffer += sizeof(int);
	local_g_ncomp = buf_i;
	G->ncomp = 0;

	for (i = 0; i < local_g_ncomp; i++)
	{
		// get p3d_comp number
		memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
		/*(size_t)*/buffer += sizeof(int);

		/* on alloue la composante connexe courante */
		comp = p3d_create_void_compco(G);
		comp->num = buf_i;    

		// get node number in comp
		memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
		/*(size_t)*/buffer += sizeof(int);
		local_comp_nnode = buf_i;
		comp->nnode = 0;

		for (j = 0; j < local_comp_nnode; j++)
		{
			// get p3d_node
			memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
			/*(size_t)*/buffer += sizeof(int);

			// noeud courant
			N = tab_node[buf_i - 1];
			N->num = buf_i;
			N->numcomp = comp->num;
			N->comp = comp;

			// get n_fail_extend
			memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
			/*(size_t)*/buffer += sizeof(int);
			N->n_fail_extend = buf_i;

			// get weight
			memcpy((void*)&buf_d, (void*)buffer, sizeof(double));
			/*(size_t)*/buffer += sizeof(double);
			N->weight = buf_d;

			// get p3d_q
			memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
			/*(size_t)*/buffer += sizeof(int);
			n = buf_i;
			if (n!=G->rob->nb_dof) goto error; // verif
			qdeg = MY_ALLOC(double,n);

			memcpy((void*)qdeg, (void*)buffer, n * sizeof(double));
			/*(size_t)*/buffer += n * sizeof(double);

			N->q = p3d_copy_config_deg_to_rad(G->rob,qdeg);
			MY_FREE(qdeg,double,n);


			// get p3d_neighb
			memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
			/*(size_t)*/buffer += sizeof(int);
			N->nneighb = buf_i;
			n = buf_i;
			if (n != 0)
			{
				memcpy((void*)itab, (void*)buffer, n * sizeof(int));
				/*(size_t)*/buffer += n * sizeof(int);

				// on cree la liste des noeuds voisins du noeud courant
				list_node = MY_ALLOC(p3d_list_node,1);
				list_node->N = tab_node[itab[0]-1]; 
				list_node->next = NULL; 
				N->neighb = list_node;
				N->last_neighb = list_node;
				for(inode=1;inode<n;inode++)
				{
					list_node = MY_ALLOC(p3d_list_node,1);
					list_node->N = tab_node[itab[inode]-1]; 
					list_node->next = NULL; 
					N->last_neighb->next = list_node; list_node->prev = N->last_neighb; 
					N->last_neighb = N->last_neighb->next;
				}

			}

			// get p3d_nedge
			memcpy((void*)&buf_i, (void*)buffer, sizeof(int));
			/*(size_t)*/buffer += sizeof(int);
			local_nedge = buf_i;
			N->nedge = 0 /*buf_i*/;	// on n'utilise pas l'information nombre de noeud pour pouvoir creer la liste plus facilement */
			for (k = 0; k < local_nedge; k++)
			{
			  memcpy((void*)itab, (void*)buffer, 3*sizeof(int));
			  /*(size_t)*/buffer += 3*sizeof(int);

			  memcpy((void*)&buf_d, (void*)buffer, sizeof(double));
			  /*(size_t)*/buffer += sizeof(double);

			  e = set_one_edge(G, tab_node[itab[0]-1], 
				       tab_node[itab[1]-1],
				       (p3d_localpath_type)itab[2], buf_d);
			  
			  if((e->planner == REEDS_SHEPP) ||
			     (e->planner == DUBINS)) {
					memcpy((void*)&buf_d, (void*)buffer, sizeof(double));
					/*(size_t)*/buffer += sizeof(double);
					/* this is the robot radius for a platform, to be ignored as long as Move3D
					does not allow different radii in one graph (currently storage of the 
					radius for a particular edge is missing at the level of p3d_edge data) */
				}
				// p3d_stepsize : not save in buffer - ignore: the planner software of University Utrecht may generate this keyword for its own purposes
			}
			// p3d_end_node

			/* on insere le noeud courant dans la liste ebt de la composante connexe */
			list_node = MY_ALLOC(p3d_list_node, 1);
			list_node->N = N;
			list_node->prev = comp->last_node;
			list_node->next = NULL;
			if (comp->last_node == NULL)
			  { comp->dist_nodes = list_node; }
			else
			  { comp->last_node->next = list_node; }
			comp->last_node  = list_node;
			comp->nnode ++;
		}

		// p3d_end_comp 
		comp = NULL;
	}

	set_canreach_compco(G);

	// free & ok
	MY_FREE(tab_node,pp3d_node,nnode);
	return(TRUE);



	// free & error
error:
	// Proc�dure de fin de fonction avec affichage d'erreurs et lib�ration de la m�moire
	PrintError(("MP: read_graph_buff: return error\n"));
	while(p3d_inside_desc()) p3d_end_desc();
	p3d_del_graph(G);
	if (tab_node!= NULL)
	{ 
		MY_FREE(tab_node,pp3d_node,nnode);
	}
	return(FALSE);

}

  
/*--------------------------------------------------------------------------*/
/*! \brief Read a graph from a stream 
 *
 *  Initialises a graph, reads it from a file and returns a  
 *  pointer to the graph  (no validation or evaluation of    
 *  graph is done whatsoever).
 *
 *  \param fd:      the file stream
 *  \param envPt:   a pointer to the environment in which the graph is loaded
 *                  if it is NULL, the function do not check if the graph 
 *                  is in the right environment
 *  \param robotPt: a pointer to the robot used in this graph, if it is NULL,
 *                  the function do not check the validity of the robot.
 *
 *  \retval pG: the adresse of the new graph. If there is any error 
 *              its value does not change.
 *
 *  \return TRUE if the graph has been succesfully read.
 *
 *  \warning If both envPt and robotPt are NULL there is no check for the
 *           robot. If angle are in degree they will not be converted.
 */  
int p3d_read_this_graph(FILE *fd, p3d_graph **pG, 
			p3d_env * envPt, p3d_rob * robotPt)
{char  fct[100];
 p3d_graph *G = NULL;
 int   itab[2000],n,nnode=0,inode;
 double dtab[1000];
 char  name[256];
 p3d_node *N= NULL, **tab_node= NULL;
 p3d_list_node *list_node;
 p3d_compco *comp= NULL;
 int ij,iloc,check_q;
 double *qdeg;

 G = p3d_allocinit_graph();
 G->env = envPt;
 G->rob = robotPt;
 check_q = FALSE;

 while(fscanf(fd,"%s", fct) != EOF) {

    if(strcmp(fct,"p3d_env")== 0) {
      if(!read_desc_name(fd,name))  goto error;
      if (envPt != NULL) {
	if(strcmp(envPt->name,name)!= 0){
	  PrintInfo(("read_graph : environment needed : %s ; "
		     "environment read : %s\n", envPt->name, name));
	  p3d_del_graph(G);
	  if (tab_node!= NULL)
	    { MY_FREE(tab_node,pp3d_node,nnode); }
	  return FALSE;
	}
	G->env = envPt;
      }
      continue;
    }

    if(strcmp(fct,"p3d_robot")== 0) {
      if(!read_desc_name(fd,name))  goto error;
      check_q = TRUE;
      if(robotPt != NULL) {
	if(strcmp(robotPt->name,name) == 0) {
	  G->rob = robotPt;
	  continue;
	}
      } else {
	if (envPt != NULL) {
	  for(ij=0; ij<envPt->nr; ij++) {
	    if(strcmp(envPt->robot[ij]->name, name) == 0) {
	      G->rob = envPt->robot[ij];
	      continue;
	    }
	  }
	} else {/* Do not check the robot */
	  check_q = FALSE;
	  continue; 
	}
      }
      PrintInfo(("read_graph : the robot does not match: %s\n", name));
      p3d_del_graph(G);
      if (tab_node!= NULL)
	{ MY_FREE(tab_node,pp3d_node,nnode); }
      return FALSE;
    }

    if(strcmp(fct,"p3d_nnodes")== 0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      nnode = itab[0];
      /* on alloue les noeuds */
      tab_node = MY_ALLOC(pp3d_node,nnode);
      /* on cree la liste des noeuds du graphe */

      for(inode=0;inode<nnode;inode++){
	tab_node[inode] = p3d_allocinit_node();
	tab_node[inode]->num = inode+1;	
	p3d_insert_node_in_graph(G, tab_node[inode]);
      }
      continue;
    } 

    if(strcmp(fct,"p3d_nbq")== 0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      G->nb_q = itab[0];
      continue;
    } 

    if(strcmp(fct,"p3d_nbq_free")== 0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      G->nb_q_free = itab[0];
      continue;
    } 


    if(strcmp(fct,"p3d_time")== 0) {
      if(!read_desc_double(fd,1,dtab)) goto error;
      G->time = dtab[0];
      continue;
    } 

    if(strcmp(fct,"p3d_local_call")== 0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      G->nb_local_call = itab[0];
      continue;
    } 

    if(strcmp(fct,"p3d_test_coll")== 0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      G->nb_test_coll = itab[0];
      continue;
    } 

    if(strcmp(fct,"p3d_comp")== 0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      /* on alloue la composante connexe courante */
      comp = p3d_create_void_compco(G);
      comp->num = itab[0];    
      continue;
    }

   
    if(strcmp(fct,"p3d_node")==0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      /* on charge le noeud courant pour pouvoir le remplir */
      N = tab_node[itab[0]-1];
      N->num = itab[0];
      N->numcomp = comp->num;
      N->comp = comp;
      continue;      
    }

    if(strcmp(fct,"n_fail_extend")==0) {
      if(!read_desc_int(fd,1,itab)) goto error;
      /* on charge le noeud courant pour pouvoir le remplir */
      N->n_fail_extend = itab[0];
      continue;      
    }

    if(strcmp(fct,"weight")== 0) {
      if(!read_desc_double(fd,1,dtab)) goto error;
      N->weight = dtab[0];
      continue;
    }

    if(strcmp(fct,"ikSol")== 0) {
      if(!read_desc_int(fd,G->rob->cntrt_manager->ncntrts,itab)) goto error;
      p3d_copy_iksol(G->rob->cntrt_manager,itab, &(N->iksol));
      continue;
    }

    if(strcmp(fct,"p3d_q")==0) {
      if(!read_desc_line_double(fd,&n,dtab)) goto error;
      /* D�but modification Fabien */
      if (check_q) {
	if (n == G->rob->nb_dof) {
	  qdeg = p3d_alloc_config(G->rob);
	  for(ij=0;ij<n;ij++)
	    { qdeg[ij] = dtab[ij]; }
	} else if (n == G->rob->nb_dof - 2) {
	  /* old style config with only 4 degrees of freedom for main body
	     set Rx and Ry to 0 */
	  PrintInfo(("Old style of configuration, resave the graph\n"));
	  qdeg = p3d_alloc_config(G->rob);
	  for (ij=0; ij<3; ij++)
	    { qdeg[ij] = dtab[ij]; }
	  qdeg[3] = 0;
	  qdeg[4] = 0;
	  for (ij=5; ij<n+2; ij++)
	    { qdeg[ij] = dtab[ij-2]; }
	} else {
	  PrintWarning(("read_graph: wrong configuration size\n"));
	  goto error;
	}
	N->q = p3d_copy_config_deg_to_rad(G->rob,qdeg);
	p3d_destroy_config(G->rob, qdeg);
      } else { /* Take not in account the robot */
	qdeg = p3d_alloc_config_n(n);
	for(ij=0;ij<n;ij++)
	  { qdeg[ij] = dtab[ij]; }
	N->q = qdeg;
      }
      continue;
    }
    
    if(strcmp(fct,"p3d_neighb")==0) {    
      if(!read_desc_int(fd,1,itab)) goto error;
      N->nneighb = itab[0];
      if(itab[0] != 0){
	if(!read_desc_line_int(fd,&n,itab)) goto error;
	/* on cree la liste des noeuds voisins du noeud courant */
	list_node = MY_ALLOC(p3d_list_node,1);
	list_node->N = tab_node[itab[0]-1]; 
	list_node->next = NULL; 
	N->neighb = list_node;
	N->last_neighb = list_node;
	for(inode=1;inode<n;inode++){
	  list_node = MY_ALLOC(p3d_list_node,1);
	  list_node->N = tab_node[itab[inode]-1]; 
	  list_node->next = NULL; 
	  N->last_neighb->next = list_node; list_node->prev = N->last_neighb; 
	  N->last_neighb = N->last_neighb->next;
	}
      }
      continue;
    }

    if(strcmp(fct,"p3d_nedge")==0) {
     if(!read_desc_int(fd,1,itab)) goto error; 
     /* on n'utilise pas l'information nombre de noeud */
     /* pour pouvoir creer la liste plus facilement */
     N->nedge = 0;
     continue;
    }

    if(strcmp(fct,"p3d_edge")==0) {
      if(!read_desc_int(fd,2,itab)) goto error;
      if(!read_desc_name(fd,name)) goto error;
      if(!read_desc_double(fd,1,dtab)) goto error;

      for(iloc=0;iloc<P3D_NB_LOCAL_PLANNER;iloc++){
	if(strcmp(name,p3d_local_getname_planner(
				 (p3d_localpath_type)iloc)) == 0)
	  { break; }
      }
      set_one_edge(G, tab_node[itab[0]-1], tab_node[itab[1]-1],
		   (p3d_localpath_type)iloc, dtab[0]);
      continue;
    }

    if(strcmp(fct,"p3d_radius")==0) {
      if(!read_desc_double(fd,1,dtab)) goto error;
      /* this is the robot radius for a platform, to be ignored as long as Move3D
	 does not allow different radii in one graph (currently storage of the 
	 radius for a particular edge is missing at the level of p3d_edge data) */
      continue;
    }

    if(strcmp(fct,"p3d_stepsize")==0) {
      if(!read_desc_double(fd,1,dtab)) goto error;
      /* ignore: the planner software of University Utrecht may generate 
	 this keyword for its own purposes */
      continue;
    }

    if(strcmp(fct,"p3d_end_node")==0) {
      /* on insere le noeud courant dans la liste ebt de la composante connexe */
      list_node = MY_ALLOC(p3d_list_node, 1);
      list_node->N = N;
      list_node->prev = comp->last_node;
      list_node->next = NULL;
      if (comp->last_node == NULL)
	{ comp->dist_nodes = list_node; }
      else
	{ comp->last_node->next = list_node; }
      comp->last_node  = list_node;
      comp->nnode ++;
    }

    if(strcmp(fct,"p3d_end_comp")==0) {
      comp = NULL;
      continue;      
    }
  }

 set_canreach_compco(G);

 // free & ok
 *pG = G; 
 /* Carl 27062001 */
 MY_FREE(tab_node,pp3d_node,nnode);

 printf("llego despues de free\n");

 return(TRUE);

 // free & error
error:

 // Proc�dure de fin de fonction avec affichage d'erreurs et lib�ration de la m�moire
	PrintError(("MP: p3d_read_desc: unknow function %s\n",fct));
	while(p3d_inside_desc()) p3d_end_desc();
	p3d_del_graph(G);
	if (tab_node!= NULL)
	{ 
		MY_FREE(tab_node,pp3d_node,nnode);
	}
	return(FALSE);

}

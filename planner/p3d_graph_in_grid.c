#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Bio-pkg.h"
#include "Graphic-pkg.h" //for g3d_draw_allwin_active();
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif

/************************************************************/
/* Constants                                                */
/************************************************************/

// size of the cell edge (in angstroms)
#define BIO_CELL_EDGE_SIZE 0.5   
//#define BIO_CELL_FRAME_ORI_THRESHOLD 0.785 // ~ 45 deg.  
#define BIO_CELL_FRAME_ORI_THRESHOLD 1.571 // ~ 90 deg.  





/************************************************************/
/* Local Structures (TO MOVE TO A .h FILE ?)                */
/************************************************************/

typedef struct cell_3d {
  p3d_vector3 center;
  p3d_vector3 size;
  int index[3];
  int n_nodes;
  p3d_node **array_nodesPt;
  int n_edges;
  p3d_edge **array_edgesPt;
  int n_interm_nodes;
  p3d_node **array_intermediate_nodesPt;
  double average_value;  
  // list of nodes with significantly different orientation (one representative node per class)
  int n_diffori_nodes;
  p3d_node **array_diffori_nodesPt;
} p3d_cell_3d;


typedef struct grid_3d {
  int ncells[3];
  double cell_size[3];
  int origin[3];
  int n_noempty_cells;
  struct cell_3d ****arraycell3dPt;
} p3d_grid_3d;


/************************************************************/
/* Static Global Variables                                  */
/************************************************************/

// pointer the current grid
static p3d_grid_3d *curr_grid3dPt = NULL;
static int p3d_insert_node_in_grid_3d(p3d_grid_3d *grid3dPt,
				      p3d_node *N, double *min_N_rmf_pos, 
				      int *cell_number, double *cell_size);  
static int p3d_insert_edge_in_grid_3d(p3d_grid_3d *grid3dPt, p3d_graph *graphPt, 
				      p3d_edge *E, double *min_N_rmf_pos, int *cell_number,
				      double *cell_size) ;


/************************************************************/
/* Memory Allocation and Initialization                     */
/************************************************************/

static void p3d_init_cell_3d(p3d_cell_3d *cell3dPt)
{
  int i;

  for (i=0; i<3; i++) {
    cell3dPt->center[i] = 0.0;
    cell3dPt->size[i] = 0.0;
  }
  cell3dPt->n_nodes = 0;
  cell3dPt->array_nodesPt = NULL;
  cell3dPt->n_edges = 0;
  cell3dPt->array_edgesPt = NULL;
  cell3dPt->n_interm_nodes = 0;
  cell3dPt->array_intermediate_nodesPt = NULL;
  cell3dPt->average_value = 0.0;
  cell3dPt->n_diffori_nodes = 0;
  cell3dPt->array_diffori_nodesPt = NULL;
}


// grid initialization with NULL elemets
static void p3d_init_grid_3d(p3d_grid_3d *grid3dPt, double nxcells, double nycells, double nzcells)
{
  int i,j,k;
  p3d_cell_3d ****arraycell3dPt;

  grid3dPt->ncells[0] = nxcells;
  grid3dPt->ncells[1] = nycells;
  grid3dPt->ncells[2] = nzcells;

  // by default the "origin" is cell 0,0,0
  grid3dPt->origin[0] = 0;
  grid3dPt->origin[1] = 0;
  grid3dPt->origin[2] = 0;

  grid3dPt->n_noempty_cells = 0;

  arraycell3dPt = MY_ALLOC(p3d_cell_3d ***, nxcells);

  for (i=0; i<nxcells; i++) {
    arraycell3dPt[i] = MY_ALLOC(p3d_cell_3d **, nycells);
    
    for (j=0; j<nycells; j++) {
      arraycell3dPt[i][j] = MY_ALLOC(p3d_cell_3d *, nzcells);
      
      for (k=0; k<nzcells; k++) {
	arraycell3dPt[i][j][k] = NULL;
      }
    }
  }  
  grid3dPt->arraycell3dPt = arraycell3dPt;

}

/************************************************************/
/* Memory Free                                              */
/************************************************************/

static void p3d_delete_cell_3d(p3d_graph *graphPt, p3d_cell_3d *cell3dPt)
{
  int i;
  
  MY_FREE(cell3dPt->array_nodesPt,p3d_node *,cell3dPt->n_nodes);
  MY_FREE(cell3dPt->array_edgesPt,p3d_edge *,cell3dPt->n_edges);
  // NOTE : THE NODES IN array_intermediate_nodesPt HAVE TO BE DELETED
  for (i=0; i<cell3dPt->n_interm_nodes; i++) {
    p3d_APInode_desalloc(graphPt,cell3dPt->array_intermediate_nodesPt[i]);
  }
  MY_FREE(cell3dPt->array_intermediate_nodesPt,p3d_node *,cell3dPt->n_interm_nodes);
		       
  MY_FREE(cell3dPt,p3d_cell_3d,1);
}


static void p3d_delete_grid_3d(p3d_graph *graphPt, p3d_grid_3d *grid3dPt)
{
  int i,j,k;
  double nxcells, nycells, nzcells;
  p3d_cell_3d *cell3dPt;

  nxcells = grid3dPt->ncells[0];
  nycells = grid3dPt->ncells[1];
  nzcells = grid3dPt->ncells[2];

  for (i=0; i<nxcells; i++) {
    for (j=0; j<nycells; j++) {
      for (k=0; k<nzcells; k++) {
	cell3dPt = grid3dPt->arraycell3dPt[i][j][k];
	if(cell3dPt != NULL)
	  p3d_delete_cell_3d(graphPt,cell3dPt);
      }
    }
  }  
  MY_FREE(grid3dPt,p3d_grid_3d,1);
}

/************************************************************/
/* Internal Functions                                       */
/************************************************************/

/* This function returns the index of the cell containing the node N.
   The grid is constructed taking as reference the minimum values. */
static int p3d_identify_cell_3d_index_for_node(p3d_node *N,int *cell_index, 
					       double *cell_size, double *min_N_rmf_pos, int *cell_number)
{
  int i;

  for(i=0; i<3; i++) {
    cell_index[i] = (int) floor((N->RelMobFrame[i][3] - min_N_rmf_pos[i])/((double)cell_size[i]));
    if(cell_index[i] >= cell_number[i]) {
      printf("ERROR : p3d_identify_cell_3d_index_for_node : node out of grid\n");
      return 0;      
    }
  }

  return 1;
}

/* Idem than preceding function, but for a conf. */
/* This function also returns the RelMobFrame computed for the conf */
static int p3d_identify_cell_3d_index_for_conf(p3d_rob *robotPt, configPt q, int *cell_index, p3d_matrix4 RelMobFrame,
					       double *cell_size, double *min_N_rmf_pos, int *cell_number)
{
  int i;
  p3d_matrix4 *RefFramePt=NULL, *MobFramePt=NULL;
  p3d_matrix4 invT;

  // NOTE : suppose that q has ben updated

  // compute conf. RelMobFrame
  if(p3d_GetRefAndMobFrames(robotPt,&RefFramePt,&MobFramePt)) {
    if(RefFramePt == NULL) {
      p3d_mat4Copy(*MobFramePt,RelMobFrame);
    }
    else {
      p3d_matInvertXform(*RefFramePt,invT);
      p3d_matMultXform(invT,*MobFramePt,RelMobFrame);      
    }
  }

  for(i=0; i<3; i++) {
    cell_index[i] = (int) floor((RelMobFrame[i][3] - min_N_rmf_pos[i])/((double)cell_size[i]));
    if(cell_index[i] >= cell_number[i]) {
      printf("ERROR : p3d_identify_cell_3d_index_for_node : node out of grid\n");
      return 0;      
    }
  }

  return 1;
}


/************************************************************/

static int p3d_different_cell_indices(int *ci1, int *ci2, int dimension)
{
  int i;

  for(i=0; i<dimension; i++) {
    if(ci1[i] != ci2[i])
      return 1;
  }
  
  return 0;
}


/************************************************************/


static int p3d_cell_3d_is_in_grid(p3d_grid_3d *grid3dPt, int *cell_index)
{
  int i;

  for(i=0; i<3; i++) {
    if((cell_index[i] >= grid3dPt->ncells[i]) ||
       (cell_index[i] < 0))
      return 0;
  }
  
  return 1;
}

/************************************************************/

static int p3d_point_is_in_cell_3d(p3d_cell_3d *cell3dPt, double *p)
{
  int i;

  for(i=0; i<3; i++) {
    if(fabs(cell3dPt->center[i] - p[i]) > (cell3dPt->size[i] / 2.0))
      return 0;
  }
  
  return 1;
}



/************************************************************/
/* Main External Functions                                  */
/************************************************************/

void p3d_put_graph_in_grid_3d(p3d_graph *graphPt)
{
  p3d_rob *robotPt;
  p3d_compco* compPt;
  p3d_list_node *ListNode;
  p3d_node *N=NULL, *refN;
  p3d_list_edge *ListEdge;
  p3d_edge *E=NULL;
  p3d_grid_3d *grid3dPt;
  p3d_cell_3d *cell3dPt;
  p3d_matrix4 *ref_frame=NULL, *mob_frame=NULL;
  //p3d_matrix4 mob_frame_ref,invT;
  double max_N_rmf_pos[3],min_N_rmf_pos[3];
  double cell_size[3];
  int cell_number[3]; 
  int cell_index[3];
  int i,j,k;
  double nxcells, nycells, nzcells;
  //int ne = 0, nbe = 0;
  int nn,cn,nn0=0,nin0=0;
  double ori_dist;
  int inserted;
  int incluster;
  double /*totE,*/ minE;
#ifdef ENERGY  
  //double AmberE[ NB_OF_ENERGY_TERMS ];
#endif
  double goal_cell_center[3] = {5.9, 3.0, 16.8};
  //int ndof;

  // WARNING: THIS FUNCTION ONLY WORKS WITH MONO-DIRECTIONAL RRT SEARCH !!!
  //          IT NEEDS TO BE EXTENDED TO SEVERAN CONNECTED COMPONENTS
  robotPt = graphPt->rob;
  compPt = graphPt->search_start->comp;
  ListNode = compPt->dist_nodes;
  refN = graphPt->search_start;

  if(!p3d_GetRefAndMobFrames(robotPt,&ref_frame,&mob_frame)) {
    printf("ERROR : p3d_put_graph_in_grid_3d : this function needs frames_for_metric be set\n");
    return;
  }  

  // compute grid bounds
  for(i=0; i<3; i++) {
    max_N_rmf_pos[i] = -P3D_HUGE;
    min_N_rmf_pos[i] =  P3D_HUGE;
  }

  while(ListNode != NULL) {
    N = ListNode->N;      
    for(i=0; i<3; i++) {
      if(N->RelMobFrame[i][3] > max_N_rmf_pos[i]) {
	max_N_rmf_pos[i] = N->RelMobFrame[i][3];
      }
      if(N->RelMobFrame[i][3] < min_N_rmf_pos[i]) {
	min_N_rmf_pos[i] = N->RelMobFrame[i][3];
      }
    }
    ListNode = ListNode->next;
  }
  
  // define cells size 
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    for(i=0; i<3; i++) {
      cell_size[i] = BIO_CELL_EDGE_SIZE;
    }
  }
  else {
    printf("ERROR : p3d_put_graph_in_grid_3d : this function is currently implemented only for BIO\n");
    return;
  }

  // compute cell number (by axis)
  for(i=0; i<3; i++) {
    cell_number[i] = (int) ceil((max_N_rmf_pos[i] - min_N_rmf_pos[i])/((double)cell_size[i]));
  }

  // create grid
  grid3dPt = MY_ALLOC(p3d_grid_3d, 1);
  // clean and update curr_grid3dPt
  if(curr_grid3dPt != NULL) {
    p3d_delete_grid_3d(graphPt,curr_grid3dPt);
  }
  curr_grid3dPt = grid3dPt;

  // init grid
  p3d_init_grid_3d(grid3dPt,cell_number[0],cell_number[1],cell_number[2]);
  for(i=0; i<3; i++) {
    grid3dPt->cell_size[i] = cell_size[i];
  }

  // identify origin
  if(!p3d_identify_cell_3d_index_for_node(refN,cell_index,cell_size,min_N_rmf_pos,cell_number)) {
    p3d_delete_grid_3d(graphPt,grid3dPt);
    return;
  }
  for (i=0; i<3; i++) {
    grid3dPt->origin[i] = cell_index[i];
  }

  // insert all the nodes (OF THE CONNECTED COMPONENT) in the grid
  ListNode = compPt->dist_nodes;
  while(ListNode != NULL) {
    N = ListNode->N;      
    if(! p3d_insert_node_in_grid_3d(grid3dPt,N,min_N_rmf_pos,cell_number,cell_size)) {
      p3d_delete_grid_3d(graphPt,grid3dPt);
      return;
    }
    ListNode = ListNode->next;
  }

  // insert all the edges (OF THE CONNECTED COMPONENT) in the grid
  ListNode = compPt->dist_nodes;
  while(ListNode != NULL) {
    N = ListNode->N;      
    if(!p3d_identify_cell_3d_index_for_node(N,cell_index,cell_size,min_N_rmf_pos,cell_number)) {
      p3d_delete_grid_3d(graphPt,grid3dPt);
      return;
    }
    ListEdge = N->edges;
    while(ListEdge != NULL) {
      // only for out edges
      if (ListEdge->E->Ni != N) {
	printf("N = Nf\n");
      }
      if (ListEdge->E->Ni == N) {
	if (ListEdge->E->Nf == N) {
	  printf("N = Ni = Nf = PROBLEMA !!!\n");
	}
	//ne++;	
	E = ListEdge->E;	
	if(! p3d_insert_edge_in_grid_3d(grid3dPt,graphPt,E,min_N_rmf_pos,cell_number,cell_size)) {
	  p3d_delete_grid_3d(graphPt,grid3dPt);
	  return;
	}
      }
      ListEdge = ListEdge->next;
    }
    ListNode = ListNode->next;
  }
  
  //printf("NE : %d - %d NBE : %d\n",ne,graphPt->nedge,nbe);


  // CELL LABELING ///////////////
  
  // identify nodes in cell with different orientations of the ref_frame
  nxcells = grid3dPt->ncells[0];
  nycells = grid3dPt->ncells[1];
  nzcells = grid3dPt->ncells[2];
  
  for (i=0; i<nxcells; i++) {
    for (j=0; j<nycells; j++) {
      for (k=0; k<nzcells; k++) {
	cell3dPt = grid3dPt->arraycell3dPt[i][j][k];
	if(cell3dPt != NULL) {
	  // init cell3dPt->array_diffori_nodesPt
	  inserted = 0;
	  if(cell3dPt->n_nodes != 0) {
	    for(nn0=0; (nn0<cell3dPt->n_nodes) && !inserted; nn0++) {
	      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,cell3dPt->array_nodesPt[nn0]->q);
	      if (p3d_col_test()){
		printf("WARARNIG : p3d_put_graph_in_grid_3d : node in collision\n");
	      }
	      else {
		insert_pointer_in_list((void*) cell3dPt->array_nodesPt[0],(void ***)&(cell3dPt->array_diffori_nodesPt),&(cell3dPt->n_diffori_nodes));
		inserted = 1;
		nin0 = -1;
	      }
	    }
	  }
	  if(!inserted) {
	    if(cell3dPt->n_interm_nodes != 0) {
	      for(nin0=0; (nin0<cell3dPt->n_interm_nodes) && !inserted; nin0++) {
		p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,cell3dPt->array_intermediate_nodesPt[nin0]->q);
		if (p3d_col_test()){
		  printf("WARARNIG : p3d_put_graph_in_grid_3d : node in collision\n");
		}
		else {
		  insert_pointer_in_list((void*) cell3dPt->array_intermediate_nodesPt[0],(void ***)&(cell3dPt->array_diffori_nodesPt),&(cell3dPt->n_diffori_nodes));
		  inserted = 1;
		}
	      }
	    }
	  }

	  if(inserted) {
	    for(nn=nn0+1; nn < cell3dPt->n_nodes; nn++) {
	      incluster = 0;
	      for(cn=0; (cn < cell3dPt->n_diffori_nodes) && (incluster == 0); cn++) {
		ori_dist = p3d_MaxDiffOrientFrames(cell3dPt->array_nodesPt[nn]->RelMobFrame,
						      cell3dPt->array_diffori_nodesPt[cn]->RelMobFrame);
		if(ori_dist < BIO_CELL_FRAME_ORI_THRESHOLD) {
		  incluster = 1;
		}
	      }
	      if(incluster == 0) {
		insert_pointer_in_list((void*) cell3dPt->array_nodesPt[nn],(void ***)&(cell3dPt->array_diffori_nodesPt),&(cell3dPt->n_diffori_nodes));
	      }
	    }
	    
	    for(nn=nin0+1; nn < cell3dPt->n_interm_nodes; nn++) {
	      incluster = 0;
	      for(cn=0; (cn < cell3dPt->n_diffori_nodes) && (incluster == 0); cn++) {
		ori_dist = p3d_MaxDiffOrientFrames(cell3dPt->array_intermediate_nodesPt[nn]->RelMobFrame,
						      cell3dPt->array_diffori_nodesPt[cn]->RelMobFrame);
		if(ori_dist < BIO_CELL_FRAME_ORI_THRESHOLD) {
		  incluster = 1;
		}
	      }
	      if(incluster == 0) {
		insert_pointer_in_list((void*) cell3dPt->array_intermediate_nodesPt[nn],(void ***)&(cell3dPt->array_diffori_nodesPt),&(cell3dPt->n_diffori_nodes));
	      }
	    }

	    // VALUE = N_DIFFORI_NODES
	    //cell3dPt->average_value = (double) cell3dPt->n_diffori_nodes;
	    // VALUE = ENERGY
	    if(p3d_col_get_mode() == p3d_col_mode_bio) {
	      
	      if (0) { // NO E !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	      
#ifdef ENERGY
	      minE = P3D_HUGE;
	      for(cn=0; cn < cell3dPt->n_diffori_nodes; cn++) {
		p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, cell3dPt->array_diffori_nodesPt[cn]->q);
		// view conf
		g3d_draw_allwin_active();	      
		// print config
/* 		printf("q = "); */
/* 		for(ndof=0; ndof<robotPt->nb_dof;ndof++){ */
/* 		  printf("%f ", cell3dPt->array_diffori_nodesPt[cn]->q[ndof]); */
/* 		}	       */
/* 		printf("\n"); */
		//CalculateEnergy(robotPt,&totE);
		//EnergyMinimization(robotPt, NULL, AmberE);	
		//totE = AmberE[0];
		EnergyMinimizationUsingAmbmov(robotPt,&totE);
		if(totE < minE)
		  minE = totE;
	      }
#endif
	      cell3dPt->average_value = - minE;
	      
	      }  // END NO E !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	      
	    }  
	    else {
	      // TO DO
	    }
	  }
	  // if not inserted
	  else {
	    printf("WARARNIG : p3d_put_graph_in_grid_3d : all nodes into cell are in collision\n");
	    cell3dPt->average_value = - P3D_HUGE;	    
	  }

	  // FOR DEBUGGING
/* 	  if(cell3dPt->n_diffori_nodes > 1) { */
/* 	    printf("MORE THAN ONE ORIENTATION\n"); */
/* 	    printf("CELL DATA [%d][%d][%d]: nn = %d, nin = %d, ne = %d, value = %f \n", */
/* 		   i,j,k,cell3dPt->n_nodes,cell3dPt->n_interm_nodes,cell3dPt->n_edges,cell3dPt->average_value); */
/* 	    printf(" NODES : ");		 */
/* 	    for(nn=0; nn < cell3dPt->n_diffori_nodes; nn++) { */
/* 	      printf("%d ",cell3dPt->array_diffori_nodesPt[nn]->num); */
/* 	    } */
/* 	    printf("\n");		 */
/* 	  } */
	  //
	    
	  // SEARCH GOAL CELL
	  if (p3d_point_is_in_cell_3d(cell3dPt,goal_cell_center)){
	    printf("CELL CONTAINING GOAL\n");
	    printf("CELL DATA [%d][%d][%d]: nn = %d, nin = %d, ne = %d, value = %f \n",
		   i,j,k,cell3dPt->n_nodes,cell3dPt->n_interm_nodes,cell3dPt->n_edges,cell3dPt->average_value);
	    printf(" NODES : ");		
	    for(nn=0; nn < cell3dPt->n_diffori_nodes; nn++) {
	      printf("%d ",cell3dPt->array_diffori_nodesPt[nn]->num);
	    }
	    printf("\n");			    
	  }
	  
	}
      }
    }
  }
}

/************************************************************/
/* Main Internal Functions                                  */
/* NOTE: some of them can became external                   */
/************************************************************/

static int p3d_insert_node_in_grid_3d(p3d_grid_3d *grid3dPt, p3d_node *N, 
				      double *min_N_rmf_pos, int *cell_number, double *cell_size)  
// NOTE: the three last arguments should be data in the p3d_grid_3d structure 
{
  p3d_cell_3d *cell3dPt;
  int cell_index[3];
  int i;

  if(!p3d_identify_cell_3d_index_for_node(N,cell_index,cell_size,min_N_rmf_pos,cell_number)) {
    return 0;
  }
  
  cell3dPt = grid3dPt->arraycell3dPt[cell_index[0]][cell_index[1]][cell_index[2]];
  
  // cell creation and initialization
  if(cell3dPt == NULL) {
    cell3dPt = MY_ALLOC(p3d_cell_3d, 1);
    p3d_init_cell_3d(cell3dPt);
    for (i=0; i<3; i++) {
      cell3dPt->size[i] = cell_size[i];
      cell3dPt->center[i] = min_N_rmf_pos[i] + cell_size[i] * (((double)cell_index[i]) + 0.5);
      cell3dPt->index[i] = cell_index[i];
    }
    grid3dPt->arraycell3dPt[cell_index[0]][cell_index[1]][cell_index[2]] = cell3dPt;
    grid3dPt->n_noempty_cells++;
  }
  
  // insert node pointer
  insert_pointer_in_list((void*)N,(void ***)&(cell3dPt->array_nodesPt),&(cell3dPt->n_nodes));
  
  return 1;
}

/************************************************************/

static int p3d_insert_edge_in_grid_3d(p3d_grid_3d *grid3dPt, p3d_graph *graphPt, p3d_edge *E, 
				      double *min_N_rmf_pos, int *cell_number, double *cell_size)  
// NOTE: the three last arguments should be data in the p3d_grid_3d structure 
{
  p3d_localpath *localpathPt;
  p3d_rob *robotPt;
  int njnt;
  double *distances;
  configPt q,qp;
  int end_localpath = 0;
  double step_dep;
  double u, du, umax; /* parameters along the local path */
  int **iksol = NULL;
  p3d_cell_3d *cell3dPt;
  int cell_index[3],cell_index_conf[3],cell_index_Nf[3];
  p3d_matrix4 conf_rel_mob_frame;
  int i;
  p3d_node *intnodePt;

  robotPt = graphPt->rob;
  njnt = robotPt->njoints;
  qp = p3d_alloc_config(robotPt);
  distances = MY_ALLOC(double, njnt+1);

  // localpathPt = E->path; // NOTE : E DOES NOT CONTAIN THE LOCAL PATH !!!!   
  localpathPt = p3d_local_planner(robotPt, E->Ni->q, E->Nf->q);
  
  if(!p3d_identify_cell_3d_index_for_node(E->Nf,cell_index_Nf,cell_size,min_N_rmf_pos,cell_number)) {
    p3d_destroy_config(robotPt, qp);
    MY_FREE(distances, double, njnt+1);
    return 0;
  }
  
  p3d_copy_config_into(robotPt,E->Ni->q, &qp);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
  
  // FOR DEBUG
  //g3d_draw_allwin_active();
  
  umax = localpathPt->range_param;
  
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    //bio_col_get_step_deplacement(&step_dep);
    step_dep = BIO_CELL_EDGE_SIZE / 5.0;
    du = bio_compute_localpath_validation_const_step(robotPt,localpathPt,step_dep);  
  }
  else {
    /* 	  step_dep = p3d_get_env_dmax();  */
    /* 	  for (i=0; i<=njnt; i++){ */
    /* 	    distances[i] = step_dep*5.0; */
    /* 	  } */
    /* 	  du = localpathPt->stay_within_dist(robotPt, localpathPt, */
    /* 					     u, FORWARD, distances); */
    // TO DO : IMPLEMENTATION FOR NON-BIO
  }
  
  end_localpath = 0;
  u = 0.0;
  u+=du;
  if (u > umax - EPS6){
    u = umax;
    end_localpath = 1;
  }
  
  while(!end_localpath) {
    if (change_position_robot_multisol(robotPt, localpathPt, u, du, qp)) {
      printf("ERROR : p3d_put_graph_in_grid_3d : intermediate conf in localpath cannot be updated\n");
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return 0;
    }
    p3d_get_robot_config_into(robotPt, &qp);
    
    // FOR DEBUG
    //g3d_draw_allwin_active();
    
    // identify cell corresponding to the current conf in the localpath
    if(!p3d_identify_cell_3d_index_for_conf(robotPt,qp,cell_index_conf,conf_rel_mob_frame,
					    cell_size,min_N_rmf_pos,cell_number)) {
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return 0;
    }
    
    // check if cell is in grid 
    if(!p3d_cell_3d_is_in_grid(grid3dPt,cell_index_conf)) {	  
      printf("WARNING : p3d_put_graph_in_grid_3d : localpath cell out of grid\n");
    }
    // if cell in grid
    else {
      // if localpath reaches the last cell
      if(!p3d_different_cell_indices(cell_index_conf,cell_index_Nf,3)) {	  
	end_localpath = 1;
      }
      // if localpath exits the current cell
      else if(p3d_different_cell_indices(cell_index,cell_index_conf,3)) {
	
	// update cell_index
	for (i=0; i<3; i++){
	  cell_index[i] = cell_index_conf[i];
	}
	
	cell3dPt = grid3dPt->arraycell3dPt[cell_index[0]][cell_index[1]][cell_index[2]];
	
	// cell creation and initialization
	if(cell3dPt == NULL) {
	  cell3dPt = MY_ALLOC(p3d_cell_3d, 1);
	  p3d_init_cell_3d(cell3dPt);
	  for (i=0; i<3; i++) {
	    cell3dPt->size[i] = cell_size[i];
	    cell3dPt->center[i] = min_N_rmf_pos[i] + cell_size[i] * (((double)cell_index[i]) + 0.5);
	    cell3dPt->index[i] = cell_index[i];
	  }
	  grid3dPt->arraycell3dPt[cell_index[0]][cell_index[1]][cell_index[2]] = cell3dPt;
	  (grid3dPt->n_noempty_cells)++;
	}
	
	// insert edge pointer
	insert_pointer_in_list((void*) E,(void ***)&(cell3dPt->array_edgesPt),&(cell3dPt->n_edges));
	
	// create intermediate node and insert it in list
	p3d_get_iksol_vector(robotPt->cntrt_manager,&iksol);
	q = p3d_copy_config(robotPt,qp);
	// WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
	intnodePt = p3d_APInode_make_multisol(graphPt, q, NULL);
	p3d_mat4Copy(conf_rel_mob_frame,intnodePt->RelMobFrame);
	insert_pointer_in_list((void*) intnodePt,(void ***)&(cell3dPt->array_intermediate_nodesPt),&(cell3dPt->n_interm_nodes));	    
      }
    }	    
    if(p3d_col_get_mode() != p3d_col_mode_bio) {
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
    }
    u+=du;
    if (u > umax - EPS6){
      u = umax;
      end_localpath = 1;
    }	  
  }
  // check
  //p3d_identify_cell_3d_index_for_node(E->Nf,cell_index,cell_size,min_N_rmf_pos,cell_number);
  //if(p3d_different_cell_indices(cell_index_Nf,cell_index_conf,3)) {
  if((abs(cell_index_Nf[0] - cell_index_conf[0]) > 1) ||
     (abs(cell_index_Nf[1] - cell_index_conf[1]) > 1) ||
     (abs(cell_index_Nf[2] - cell_index_conf[2]) > 1)) {
    /* 	  printf("OJOOOO!!!!! : last cell localpath != cell Nf\n"); */
    /* 	  printf("              last cell %d %d %d\n",cell_index_conf[0],cell_index_conf[1],cell_index_conf[2]); */
    /* 	  printf("              Nf   cell %d %d %d\n",cell_index_Nf[0],cell_index_Nf[1],cell_index_Nf[2]); */
    //nbe++;
  }

  p3d_destroy_config(robotPt, qp);
  MY_FREE(distances, double, njnt+1);
  return 1;
}      



/************************************************************/

void p3d_write_curr_grid3d_in_geomview_file(void)
{
  FILE *fp;
  int i,j,k;
  double nxcells, nycells, nzcells;
  p3d_grid_3d *grid3dPt;
  p3d_cell_3d *cell3dPt;
  int n_noemptycells,ccount;
  double max_value = -P3D_HUGE, min_value = P3D_HUGE, val_range;
  p3d_cell_3d *maxvalcellPt;
  double cellcolor[3];
  double celltransparency = 0.4; // FLAG

  if(curr_grid3dPt == NULL) {
    printf("ERROR : p3d_write_curr_grid3d_in_geomview_file : NO curr_grid3d\n");
    return;
  }

  grid3dPt = curr_grid3dPt;

  nxcells = grid3dPt->ncells[0];
  nycells = grid3dPt->ncells[1];
  nzcells = grid3dPt->ncells[2];

  n_noemptycells = grid3dPt->n_noempty_cells;

  // open w file
  fp= fopen("graph_in_grid3d.off","w");

  // write header
  fprintf(fp,"OFF\n");         // geomview input file format
  fprintf(fp,"%d %d %d\n\n",8 * n_noemptycells,6 * n_noemptycells,12 * n_noemptycells);   // n vertices, n faces, n edges
  
  // write vertices
  for (i=0; i<nxcells; i++) {
    for (j=0; j<nycells; j++) {
      for (k=0; k<nzcells; k++) {
	cell3dPt = grid3dPt->arraycell3dPt[i][j][k];
	if(cell3dPt != NULL) {
	  // write the 8 cell vertices
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] - cell3dPt->size[0]/2.0, cell3dPt->center[1] - cell3dPt->size[1]/2.0, cell3dPt->center[2] + cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] + cell3dPt->size[0]/2.0, cell3dPt->center[1] - cell3dPt->size[1]/2.0, cell3dPt->center[2] + cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] + cell3dPt->size[0]/2.0, cell3dPt->center[1] + cell3dPt->size[1]/2.0, cell3dPt->center[2] + cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] - cell3dPt->size[0]/2.0, cell3dPt->center[1] + cell3dPt->size[1]/2.0, cell3dPt->center[2] + cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] - cell3dPt->size[0]/2.0, cell3dPt->center[1] - cell3dPt->size[1]/2.0, cell3dPt->center[2] - cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] + cell3dPt->size[0]/2.0, cell3dPt->center[1] - cell3dPt->size[1]/2.0, cell3dPt->center[2] - cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] + cell3dPt->size[0]/2.0, cell3dPt->center[1] + cell3dPt->size[1]/2.0, cell3dPt->center[2] - cell3dPt->size[2]/2.0); 
	  fprintf(fp,"   %f %f %f\n", 
		  cell3dPt->center[0] - cell3dPt->size[0]/2.0, cell3dPt->center[1] + cell3dPt->size[1]/2.0, cell3dPt->center[2] - cell3dPt->size[2]/2.0); 
	  // identify max cell value
	  if(cell3dPt->average_value > max_value) {
	    max_value = cell3dPt->average_value;
	    maxvalcellPt = cell3dPt;
	  }
	  else if((cell3dPt->average_value > -P3D_HUGE) && (cell3dPt->average_value < min_value)) {
	    min_value = cell3dPt->average_value;
	  }
	}
      }
    }
  }
  printf("max_value = %f - for cell [%d][%d][%d] - center {%f,%f,%f}\n",
	 max_value,maxvalcellPt->index[0],maxvalcellPt->index[1],maxvalcellPt->index[2],
	 maxvalcellPt->center[0],maxvalcellPt->center[1],maxvalcellPt->center[2]);

  printf("ORIG_value = %f - for cell [%d][%d][%d]  \n",
	 grid3dPt->arraycell3dPt[grid3dPt->origin[0]][grid3dPt->origin[1]][grid3dPt->origin[2]]->average_value,
	 grid3dPt->origin[0],grid3dPt->origin[1],grid3dPt->origin[2]);
  
  val_range = max_value - min_value;

  // write faces
  ccount = 0;
  for (i=0; i<nxcells; i++) {
    for (j=0; j<nycells; j++) {
      for (k=0; k<nzcells; k++) {
	cell3dPt = grid3dPt->arraycell3dPt[i][j][k];
	if(cell3dPt != NULL) {	  
	  // cell color 
	  // NOTE : all the cells are blue : darker cells have higher value
	  //        the "origin" cell is red 
	  if(p3d_different_cell_indices(cell3dPt->index,grid3dPt->origin,3)) {
	    if(cell3dPt->average_value > (max_value - val_range/10.0)) {
	      cellcolor[0] = 1.0;
	      cellcolor[1] = 0.0;
	      cellcolor[2] = 0.0;
	    }
	    else if (cell3dPt->average_value > (max_value - val_range/4.0)) {
	      cellcolor[0] = 1.0;
	      cellcolor[1] = 0.5;
	      cellcolor[2] = 0.0;
	    }
	    else if (cell3dPt->average_value > (max_value - val_range/2.0)) {
	      cellcolor[0] = 1.0;
	      cellcolor[1] = 1.0;
	      cellcolor[2] = 0.0;
	    }
	    else if (cell3dPt->average_value > (max_value - val_range*3.0/4.0)) {
	      cellcolor[0] = 0.0;
	      cellcolor[1] = 1.0;
	      cellcolor[2] = 0.0;
	    }
	    else {
	      cellcolor[0] = 0.0;
	      cellcolor[1] = 0.0;
	      cellcolor[2] = 1.0;
	    }

	  }
	  else {
	    cellcolor[0] = 0.0;
	    cellcolor[1] = 0.0;
	    cellcolor[2] = 0.0;
	  }
	  // write the 6 cell faces	  
	  fprintf(fp,"4   %d %d %d %d  %f %f %f  %f\n", 
		  8*ccount + 0, 8*ccount + 1, 8*ccount + 2, 8*ccount + 3, cellcolor[0], cellcolor[1], cellcolor[2], celltransparency);
	  fprintf(fp,"4   %d %d %d %d  %f %f %f  %f\n", 
		  8*ccount + 4, 8*ccount + 7, 8*ccount + 6, 8*ccount + 5, cellcolor[0], cellcolor[1], cellcolor[2], celltransparency);
	  fprintf(fp,"4   %d %d %d %d  %f %f %f  %f\n", 
		  8*ccount + 0, 8*ccount + 4, 8*ccount + 5, 8*ccount + 1, cellcolor[0], cellcolor[1], cellcolor[2], celltransparency);
	  fprintf(fp,"4   %d %d %d %d  %f %f %f  %f\n", 
		  8*ccount + 1, 8*ccount + 5, 8*ccount + 6, 8*ccount + 2, cellcolor[0], cellcolor[1], cellcolor[2], celltransparency);
	  fprintf(fp,"4   %d %d %d %d  %f %f %f  %f\n", 
		  8*ccount + 2, 8*ccount + 6, 8*ccount + 7, 8*ccount + 3, cellcolor[0], cellcolor[1], cellcolor[2], celltransparency);
	  fprintf(fp,"4   %d %d %d %d  %f %f %f  %f\n", 
		  8*ccount + 3, 8*ccount + 7, 8*ccount + 4, 8*ccount + 0, cellcolor[0], cellcolor[1], cellcolor[2], celltransparency);

	  ccount++;
	}
      }
    }
  }

  fclose(fp);	

  printf("DONE : written file graph_in_grid3d.off\n");

}


void test_grid(void)
{
  int i,j,k;
  double nxcells, nycells, nzcells;
  p3d_grid_3d *grid3dPt;
  p3d_cell_3d *cell3dPt;
  
  grid3dPt = curr_grid3dPt;
  
  nxcells = grid3dPt->ncells[0];
  nycells = grid3dPt->ncells[1];
  nzcells = grid3dPt->ncells[2];
  
  for (i=0; i<nxcells; i++) {
    for (j=0; j<nycells; j++) {
      for (k=0; k<nzcells; k++) {
	cell3dPt = grid3dPt->arraycell3dPt[i][j][k];
	if(cell3dPt == NULL) {
	  printf("Cell index [%d][%d][%d] : NULL\n",i,j,k);
	}
	else {
	  printf("Cell index [%d][%d][%d] : n_nodes = %d :n_edges = %d :n_interm_nodes = %d : center = (%f, %f, %f) : value = %f \n",
		 i,j,k,cell3dPt->n_nodes,cell3dPt->n_edges,cell3dPt->n_interm_nodes,
		 cell3dPt->center[0],cell3dPt->center[1],cell3dPt->center[2],cell3dPt->average_value);
	}
      }
    }
  }
}
 


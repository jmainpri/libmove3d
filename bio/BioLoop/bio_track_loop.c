/* modif Juan (for BioMove3d) */
/* WARNING :
   This file is already in testing phase.
   Some functions work well only in particular cases.
*/

#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Util-pkg.h"
#include "Bio-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"


/************************************************************/
/* constants */

#define DEBUG_LOOP_TRACKING 1
#define DEBUG_LOOP_TRACKING_COLL 1

#define MAX_N_FRAMES 100
#define MAX_LOOP_LENGTH 100

#define MIN_INC_ANG 0.005 // ~  deg
#define MAX_INC_ANG 3.1416 // ~ 180 deg

/************************************************************/
/* structures */

typedef struct s_track_atom_data {
  int serial;
  char name[5];
  atomTypes type;
  int resSeq;
  char resName[4];
  char chainID[2];
  p3d_vector3 pos;
} track_atom_data;


typedef struct s_three_atom_pos {
  p3d_vector3 Npos,CApos,Cpos;
  int okN,okCA,okC; // for checking
} three_atom_pos;


typedef struct s_frame_data {
  struct s_three_atom_pos forFb, forFe; 
  int okFb, okFe; // for checking
} frame_data;


/************************************************************/
/* external global variables */

int TRACK_LOOP_FLAG = FALSE;

/************************************************************/
/* static global variables */

static p3d_jnt *loopJbase, *loopJend;
static int nframes = 0;
static p3d_matrix4 **listFb, **listFe;
static p3d_matrix4 FbtoJbase;

/************************************************************/
/* static function declarations */

static void alloc_and_init_frame_data(frame_data **framedataPtPt);
static int read_file_line(FILE *framestraj_file, track_atom_data *adataPt);
static int bio_compute_Fb_from_3atom_pos(p3d_matrix4 F, three_atom_pos *forF);
static int bio_compute_Fe_from_3atom_pos(p3d_matrix4 F, three_atom_pos *forF);
static int bio_read_and_set_FbFe_sequence_from_file(char *framestraj_filename);
static double bio_compute_maximum_allowed_increment_jnt_angle(p3d_matrix4 pF, p3d_matrix4 F);

/************************************************************/
/************************************************************/
/* static functions */
/************************************************************/
/************************************************************/

/************************************************************/
/* reading input file and computing reference frames        */
/************************************************************/

static void alloc_and_init_frame_data(frame_data **framedataPtPt)
{
  frame_data *fdPt;
  
  fdPt = (frame_data *) malloc(sizeof(frame_data)); 
  fdPt->okFb = 0;
  fdPt->okFe = 0;
  fdPt->forFb.okN = 0;
  fdPt->forFb.okCA = 0;
  fdPt->forFb.okC = 0;
  fdPt->forFe.okN = 0;
  fdPt->forFe.okCA = 0;
  fdPt->forFe.okC = 0;
  
  *framedataPtPt = fdPt;
}

/************************************************************/

/* function reading lines of the input file
   returns :
   - 1: empty line
   - 2: new FRAME
   - 3: start BASE - Fb
   - 4: start END  - Fe
   - 5: ATOM line
*/
static int read_file_line(FILE *framestraj_file, track_atom_data *adataPt)
{
  char *pdbline;
  char piece[10];
  char rec[10];
  char returned_s[100];

  pdbline = fgets(returned_s,100,framestraj_file);
  if(pdbline == NULL) {
     return 0;
  }

  strcpy(piece,"         ");
  strncpy(piece,pdbline,6);
  if(sscanf(piece,"%s",rec) < 0) return 1;
  if(strcmp(rec,"ATOM") == 0) {    
    // extract only used data
    strcpy(piece,"         ");
    strncpy(piece,pdbline+6,5);
    if(sscanf(piece,"%d",&adataPt->serial) < 0) return -1;
    strcpy(piece,"         ");
    strncpy(piece,pdbline+12,4);
    if(sscanf(piece,"%s",adataPt->name) < 0) return -1;
    strcpy(piece,"         ");
    strncpy(piece,pdbline+21,1);
    sscanf(piece,"%s",adataPt->chainID);
    // when no chainID
    if(strcmp(adataPt->chainID,"") == 0)
      strcpy(adataPt->chainID,"0");
    strcpy(piece,"         ");
    strncpy(piece,pdbline+17,3);
    if(sscanf(piece,"%s",adataPt->resName) < 0) return -1;
    strcpy(piece,"         ");
    strncpy(piece,pdbline+22,4);
    if(sscanf(piece,"%d",&adataPt->resSeq) < 0) return -1;
    strcpy(piece,"         ");
    strncpy(piece,pdbline+30,8);
    if(sscanf(piece,"%lf",&adataPt->pos[0]) < 0) return -1;
    strcpy(piece,"         ");
    strncpy(piece,pdbline+38,8);
    if(sscanf(piece,"%lf",&adataPt->pos[1]) < 0) return -1;
    strcpy(piece,"         ");
    strncpy(piece,pdbline+46,8);
    if(sscanf(piece,"%lf",&adataPt->pos[2]) < 0) return -1; 
    return 5;
  }
  else if(strcmp(rec,"FRAME:") == 0) {
    return 2;
  }
  else if(strcmp(rec,"BASE:") == 0) {
    return 3;
  }
  else if(strcmp(rec,"END:") == 0) {
    return 4;
  }
  
  return 1;
}


/************************************************************/

static int bio_compute_Fb_from_3atom_pos(p3d_matrix4 F, three_atom_pos *forF)
{
  p3d_vector3 posdiff,prev_zaxis;
  p3d_vector3 xaxis,yaxis,zaxis;

  p3d_vectSub(forF->Npos,forF->Cpos,posdiff);
  p3d_vectNormalize(posdiff,prev_zaxis);
  p3d_vectSub(forF->CApos,forF->Npos,posdiff);
  p3d_vectNormalize(posdiff,zaxis);
  p3d_vectXprod(prev_zaxis,zaxis,posdiff);
  p3d_vectNormalize(posdiff,xaxis);
  p3d_vectXprod(zaxis,xaxis,posdiff);
  p3d_vectNormalize(posdiff,yaxis);
    
  F[0][0] = xaxis[0]; F[0][1] = yaxis[0]; F[0][2] = zaxis[0]; F[0][3] = forF->CApos[0]; 
  F[1][0] = xaxis[1]; F[1][1] = yaxis[1]; F[1][2] = zaxis[1]; F[1][3] = forF->CApos[1]; 
  F[2][0] = xaxis[2]; F[2][1] = yaxis[2]; F[2][2] = zaxis[2]; F[2][3] = forF->CApos[2]; 
  F[3][0] = 0.0;      F[3][1] = 0.0;      F[3][2] = 0.0     ; F[3][3] = 1.0; 

  return 1;
}

static int bio_compute_Fe_from_3atom_pos(p3d_matrix4 F, three_atom_pos *forF)
{
  p3d_vector3 posdiff,prev_zaxis;
  p3d_vector3 xaxis,yaxis,zaxis;

  p3d_vectSub(forF->Cpos,forF->CApos,posdiff);
  p3d_vectNormalize(posdiff,prev_zaxis);
  p3d_vectSub(forF->Npos,forF->Cpos,posdiff);
  p3d_vectNormalize(posdiff,zaxis);
  p3d_vectXprod(prev_zaxis,zaxis,posdiff);
  p3d_vectNormalize(posdiff,xaxis);
  p3d_vectXprod(zaxis,xaxis,posdiff);
  p3d_vectNormalize(posdiff,yaxis);
    
  F[0][0] = xaxis[0]; F[0][1] = yaxis[0]; F[0][2] = zaxis[0]; F[0][3] = forF->Npos[0]; 
  F[1][0] = xaxis[1]; F[1][1] = yaxis[1]; F[1][2] = zaxis[1]; F[1][3] = forF->Npos[1]; 
  F[2][0] = xaxis[2]; F[2][1] = yaxis[2]; F[2][2] = zaxis[2]; F[2][3] = forF->Npos[2]; 
  F[3][0] = 0.0;      F[3][1] = 0.0;      F[3][2] = 0.0     ; F[3][3] = 1.0; 

  return 1;
}

/************************************************************/

// this function reads information from file and generates arrays of frames (static global variables)
static int bio_read_and_set_FbFe_sequence_from_file(char *framestraj_filename)
{
  FILE *framestraj_file;
  int state;
  track_atom_data adata;
  frame_data **array_frame_data;
  frame_data *curframePt = NULL;
  three_atom_pos *cur3atomposPt = NULL;
  int i;

  // open file
  framestraj_file = fopen(framestraj_filename, "r");
  if (framestraj_file == NULL) {
    printf("file containing frames trajectory cannot be open\n");
    return 0;
  }

  // alloc
  array_frame_data = (frame_data**) malloc(sizeof(frame_data *) * MAX_N_FRAMES);
  for(i=0; i<MAX_N_FRAMES; i++) 
    array_frame_data[i] = NULL;

  // extract information
  state = 1;
  nframes = 0;
  while(state > 0) {
    state = read_file_line(framestraj_file,&adata);
    switch(state) {
    case 1: // empty line
      break;
    case 2: // new FRAME
      if(curframePt != NULL) {
	// check that contains all the information
	// TO DO !!!
      }
      alloc_and_init_frame_data(&(array_frame_data[nframes])); 
      curframePt = array_frame_data[nframes];
      nframes++;
      if(nframes > MAX_N_FRAMES) {
	printf("ERROR : Maximum number of frames reached\n");
	for(i=0; i<nframes; i++) 
	  free(array_frame_data[i]);
	free(array_frame_data);
	fclose(framestraj_file);
	return 0;
      }
      break;
    case 3: // start BASE - Fb
      cur3atomposPt = &(curframePt->forFb);
      break;
    case 4: // start END  - Fe
      cur3atomposPt = &(curframePt->forFe);
      break;
    case 5: // ATOM line
      if(strcmp(adata.name,"N") == 0) {
	p3d_vectCopy(adata.pos,cur3atomposPt->Npos);
	cur3atomposPt->okN = 1;
      }
      else if(strcmp(adata.name,"CA") == 0) {
	p3d_vectCopy(adata.pos,cur3atomposPt->CApos);
	cur3atomposPt->okCA = 1;
      }
      else if(strcmp(adata.name,"C") == 0) {
	p3d_vectCopy(adata.pos,cur3atomposPt->Cpos);
	cur3atomposPt->okC = 1;
      }
      break;      
    }
  }   

  // alloc reference frames
  // WARNING : THIS MEMORY SHOULD BE FREE AT THE END OF THE PROGRAM !!!
  listFb = (p3d_matrix4**) malloc(sizeof(p3d_matrix4 *) * nframes);
  listFe = (p3d_matrix4**) malloc(sizeof(p3d_matrix4 *) * nframes);
  for(i=0; i<nframes; i++) {
    listFb[i] = (p3d_matrix4 *) malloc(sizeof(p3d_matrix4));
    listFe[i] = (p3d_matrix4 *) malloc(sizeof(p3d_matrix4));
  }
  
  // compute reference frames
  for(i=0; i<nframes; i++) {
    bio_compute_Fb_from_3atom_pos(*(listFb[i]),&(array_frame_data[i]->forFb));
    bio_compute_Fe_from_3atom_pos(*(listFe[i]),&(array_frame_data[i]->forFe));
  }

  // draw trace of Fb and Fe origins
/*   for(i=0; i<(nframes +1); i++) {   */
/*     g3d_drawOneLine((*(listFb[i]))[0][3],(*(listFb[i]))[1][3],(*(listFb[i]))[2][3], */
/* 		    (*(listFb[i+1]))[0][3],(*(listFb[i+1]))[1][3],(*(listFb[i+1]))[2][3], */
/* 		    14,NULL);  // DGreen line */
/*     g3d_drawOneLine((*(listFe[i]))[0][3],(*(listFe[i]))[1][3],(*(listFe[i]))[2][3], */
/* 		    (*(listFe[i+1]))[0][3],(*(listFe[i+1]))[1][3],(*(listFe[i+1]))[2][3], */
/* 		    16,NULL);  // Violet line */
/*   } */

  // free
  for(i=0; i<nframes; i++) 
    free(array_frame_data[i]);
  free(array_frame_data);

  fclose(framestraj_file);

  return 1;
}


/************************************************************/
/* functions for tracking loop conformation                 */
/************************************************************/


static int bio_recursive_perturb_jnt_val_and_set_qi(p3d_jnt *jntPt, configPt q, configPt qp, double inc_ang, 
						    p3d_jnt *last_jntPt, int onlylast)
{
  double vmin,vmax;
  int i,j,k;
  double vari=0.0;
  p3d_jnt *next_jntPt;
  double this_inc_ang;

  if(jntPt != last_jntPt) {
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
	p3d_jnt_get_dof_bounds(jntPt, j, &vmin, &vmax);
	// adapt inc_ang to jnt value range
	this_inc_ang = inc_ang * (vmax - vmin) / (2*M_PI);
	vari = p3d_random(-this_inc_ang, this_inc_ang);
	q[k] = qp[k] + vari;
	// punt into jnt range
	if(EQ(vmin,-M_PI) && EQ(vmax,M_PI)) {
	  // put into {-pi,pi} range
	  if(q[k] > M_PI)
	    q[k] -= 2.0*M_PI;
	  if(q[k] < -M_PI)
	    q[k] += 2.0*M_PI;
	}
	else {
	  if(q[k] > vmax)
	    q[k] = vmax;
	  if(q[k] < vmin)
	    q[k] = vmin;
	}
	p3d_jnt_set_dof(jntPt,j,q[k]);
      } 
      else {
	q[k] = qp[k];
      }
/*       if(DEBUG_LOOP_TRACKING) */
/* 	printf("J %d = %f -- max_vari = %f\n",jntPt->num,q[k],vari);	 */
    }

    if(onlylast) {
      next_jntPt = jntPt->next_jnt[jntPt->n_next_jnt - 1];
      if(!bio_recursive_perturb_jnt_val_and_set_qi(next_jntPt,q,qp,inc_ang,last_jntPt,onlylast)) {
	return 0;
      }
    }
    else {
      for(i=0; i<jntPt->n_next_jnt; i++) {
	next_jntPt = jntPt->next_jnt[i];
	if(!bio_recursive_perturb_jnt_val_and_set_qi(next_jntPt,q,qp,inc_ang,last_jntPt,onlylast)) {
	  return 0;
	}
      }
    }
  }
  return 1;
}


/* this function computes the maximum variation allowed for a dihedral algle in the loop
   as a function of the deplacement of the origin of the end frame */
static double bio_compute_maximum_allowed_increment_jnt_angle(p3d_matrix4 pF, p3d_matrix4 F)
{
  double radius, inc_ang, depl;
  p3d_vector3 p1, p2, pos_diff;
  
  // NOTA : suppose that, in the "worst case", only a residue moves
  radius = 3.8; // the lenght of a residue ???

  p1[0] = pF[0][3]; p1[1] = pF[1][3]; p1[2] = pF[2][3]; 
  p2[0] =  F[0][3]; p2[1] =  F[1][3]; p2[2] =  F[2][3]; 
  p3d_vectSub(p1,p2,pos_diff);
  depl = (double) p3d_vectNorm(pos_diff);

  inc_ang = depl / radius;

  return inc_ang;  // in radians
}

/* NOTE :
   There are two possibilities for generating intermediate conformations:
   1- There is not a defined final loop conformation:
      In this case, conformations are progressively perturbed
   2- There is a given final loop conformation:
      We can make two things :
      A- Guiding the perturbation considering a linear interpolation
      B- Generating several random trajectories and comparing the last 
         conformation with the sought goal
*/

static configPt bio_generate_nearby_random_loop_conf_old(p3d_rob *robotPt, p3d_cntrt *ctPt, 
							 p3d_matrix4 Fb, p3d_matrix4 Fe, 
							 configPt qp, double max_inc_ang)
{
  int procOK;
  int n_whole_proc,n_bkb_test,n_sch_test;
  int MAX_ITER_WHOLE_PROC;
  int MAX_NUM_BKB_TEST;
  int MAX_NUM_SCH_TEST;
  configPt q,q_save,q_inter;
  double dl = 0.0;     // WARNING : unused by current cntrt functions
  p3d_matrix4 FJb0,FJbi,Frel,invT;
  p3d_jnt *jntPt,*base_jntPt,*bkb_jntPt;
  double FFdofs[6];
  double vmin,vmax;
  int i;
  double cur_inc_ang;
  p3d_poly **listpol1, **listpol2;
  int col_number,j;

  // init parameters
  MAX_ITER_WHOLE_PROC = 100;
  MAX_NUM_BKB_TEST = 1000;
  MAX_NUM_SCH_TEST = 100;

  // WARNING : suppose that loop is currently "set and updated" at conf. qp 

  // alloc and set q
  q = p3d_copy_config(robotPt,qp);

  // generate base-jnt parameters
  base_jntPt = ctPt->actjnts[0];
  p3d_mat4Copy(base_jntPt->pos0,FJb0);
  p3d_mat4Mult(Fb,FbtoJbase,FJbi);
  p3d_matInvertXform(FJb0,invT); 
  p3d_mat4Mult(invT,FJbi,Frel);
  p3d_mat4ExtractPosReverseOrder(Frel,&(FFdofs[0]),&(FFdofs[1]),&(FFdofs[2]),&(FFdofs[3]),&(FFdofs[4]),&(FFdofs[5]));
  for(i=0; i<6; i++) {
    p3d_jnt_get_dof_bounds(base_jntPt,i,&vmin,&vmax);
    if((vmax < FFdofs[i]) || (vmin > FFdofs[i])) {
      printf("ERROR : bio_generate_nearby_random_loop_conf : exceeded jnt limints : J%d\n",base_jntPt->num);
      p3d_destroy_config(robotPt,q);
      return NULL;
    }
    q[base_jntPt->index_dof + i] = FFdofs[i];
    p3d_jnt_set_dof(base_jntPt,i,FFdofs[i]);
  }

  // QUITAR !!! ///////
  //p3d_update_this_robot_pos_without_cntrt(robotPt);	
  //g3d_draw_allwin_active();
  /////////////////////

  // set end-frame for loop-closure constraint
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
  p3d_matInvertXform(base_jntPt->abs_pos,invT); 
  p3d_mat4Mult(invT,Fe,Frel);
  ctPt->argu_d[0]  = Frel[0][0]; ctPt->argu_d[1]  = Frel[1][0]; ctPt->argu_d[2]  = Frel[2][0]; 
  ctPt->argu_d[3]  = Frel[0][1]; ctPt->argu_d[4]  = Frel[1][1]; ctPt->argu_d[5]  = Frel[2][1]; 
  ctPt->argu_d[6]  = Frel[0][2]; ctPt->argu_d[7]  = Frel[1][2]; ctPt->argu_d[8]  = Frel[2][2]; 
  ctPt->argu_d[9]  = Frel[0][3]; ctPt->argu_d[10] = Frel[1][3]; ctPt->argu_d[11] = Frel[2][3]; 

  // alloc and set q_save
  q_save = p3d_copy_config(robotPt,q);
  // alloc q_inter
  q_inter = p3d_alloc_config(robotPt);

  // main iteration : iterate the whole process
  procOK = 0;
  for(n_whole_proc=0; (n_whole_proc < MAX_ITER_WHOLE_PROC) && !procOK ; n_whole_proc++) {
    // NOTE : collisions are checked for the whole bkb+sch confromation
    //        -> OTHER POSSIBILITY : check progressively like in 'bio_shoot_loop'

    // NOTE : values are uniformly sampled in a reduced interval (that increase iteratively)
    //        -> OTHER POSSIBILITY : using normal distributions (iteratively increasing sigma)

    // compute bkb loop conformation
    for(n_bkb_test=0; (n_bkb_test < MAX_NUM_BKB_TEST) && !procOK ; n_bkb_test++) {
      // reset q
      p3d_copy_config_into(robotPt,q_save,&q);

      // adapt sampling interval
      if(n_bkb_test < (int) ceil ((double)MAX_NUM_BKB_TEST/2.0))
	 cur_inc_ang = max_inc_ang / 10.0;
      else
	 cur_inc_ang = max_inc_ang;

      // perturb conf. of active sub-chain
      jntPt = base_jntPt->next_jnt[0]; // WARNING : suppose that 1st bkb jnt is the next to the base-jnt
      if(!bio_recursive_perturb_jnt_val_and_set_qi(jntPt,q,qp,cur_inc_ang,ctPt->pasjnts[0],1)) {
	p3d_destroy_config(robotPt,q);
	p3d_destroy_config(robotPt,q_save);
	p3d_destroy_config(robotPt,q_inter);
	return NULL;
      }

      // QUITAR !!! ///////
      //p3d_update_this_robot_pos_without_cntrt(robotPt);	
      //g3d_draw_allwin_active();
      /////////////////////
      
      // solve for passive sub-chain
      if(p3d_update_this_robot_pos_multisol(robotPt,qp,dl,NULL)) {
	procOK = 1;
	if(DEBUG_LOOP_TRACKING) 
	  printf("### LOOP CLOSURE OK at iter. %d\n",n_bkb_test + 1);
      }
      else {
/* 	if(DEBUG_LOOP_TRACKING)  */
/* 	  printf("### LOOP CLOSURE FAILS at iter. %d\n",n_bkb_test + 1); */
      }
    } // end bkb iteration 

    // get q modified by cntrt
    p3d_get_robot_config_into(robotPt,&q);
    // set q_inter
    p3d_copy_config_into(robotPt,q,&q_inter);

    if(procOK) {    
      procOK = 0;
      // compute sch conformation
      for(n_sch_test=0; (n_sch_test < MAX_NUM_SCH_TEST) && !procOK ; n_sch_test++) {
	// reset q
	p3d_copy_config_into(robotPt,q_inter,&q);

	// adapt sampling interval
	if(n_sch_test < (int) ceil ((double)MAX_NUM_SCH_TEST/2.0))
	  cur_inc_ang = max_inc_ang / 10.0;
	else
	  cur_inc_ang = max_inc_ang;

	cur_inc_ang = max_inc_ang;
	bkb_jntPt = base_jntPt->next_jnt[0]; // WARNING : suppose that 1st bkb jnt is the next to the base-jnt
	while(bkb_jntPt != NULL) {
	  // WARNING : at most 2 next jnts !!! : 1st is the sch-jnt and 2nd is the bkb-jnt
	  if(bkb_jntPt->n_next_jnt > 1) {
	    jntPt = bkb_jntPt->next_jnt[0];
	    if(!bio_recursive_perturb_jnt_val_and_set_qi(jntPt,q,qp,cur_inc_ang,NULL,0)) {
	      p3d_destroy_config(robotPt,q);
	      p3d_destroy_config(robotPt,q_save);
	      p3d_destroy_config(robotPt,q_inter);
	      return NULL;
	    }
	  }
	  if(bkb_jntPt->next_jnt != NULL){
	    bkb_jntPt = bkb_jntPt->next_jnt[bkb_jntPt->n_next_jnt - 1];
	  }
	  else {
	    bkb_jntPt = NULL;
	  }
	}
	
	// collision checking
	// WARNING : WITH THE CURRENT IMPLEMENTATION, THE PROCEDURE IS INEFFICIENT 
	//           IF THERE IS COLLISION BETWEEN BKB ATOMS !!!!!!!!!!!!!!!!!!!!!

	// IDEA : CHECK COLLISIONS FOR SIDECHAINS IN RANDOM ORDER AND 
	//        CHANGE CONFORMOTION ONLY IF IT COLLIDES

	p3d_update_this_robot_pos_without_cntrt(robotPt);	
	if(DEBUG_LOOP_TRACKING_COLL)
	  bio_set_col_mode(NORMAL_BIOCOL_MODE);   // ?????	  
	if(bio_all_molecules_col() > 0) {
	  if(DEBUG_LOOP_TRACKING_COLL) {
	    biocol_report(&col_number,&listpol1,&listpol2);
	    printf("### Num. heavy atoms in contact %d ###\n",col_number);
	    for(j=0;j<col_number;j++) {
	      printf("%s  -  %s\n",listpol1[j]->poly->name,listpol2[j]->poly->name);
	    }    
	    bio_set_col_mode(NORMAL_BIOCOL_MODE);
	  }
	}
	else {
	  procOK = 1;
	}
      } // end sch iteration
    }      
  } // end main iteration

  // free q_save and q_inter
  p3d_destroy_config(robotPt,q_save);
  p3d_destroy_config(robotPt,q_inter);

  if(!procOK) {
    p3d_destroy_config(robotPt,q);
    return NULL;
  }

  // display
  g3d_draw_allwin_active();

  return q;
}

// same than _old but with incremental collision checking
configPt bio_generate_nearby_random_loop_conf(p3d_rob *robotPt, p3d_cntrt *ctPt, 
					      p3d_matrix4 Fb, p3d_matrix4 Fe, 
					      configPt qp, double max_inc_ang)
{
  int procOK, procOK2=0;
  int n_whole_proc,n_bkb_test,n_sch_test,n_onesh_test;
  int MAX_ITER_WHOLE_PROC;
  int MAX_NUM_BKB_TEST;
  int MAX_NUM_SCH_TEST;
  int MAX_NUM_ONESCH_TEST;
  configPt q,q_save,q_inter;
  double dl = 0.0;     // WARNING : unused by current cntrt functions
  p3d_matrix4 FJb0,FJbi,Frel,invT;
  p3d_jnt *jntPt,*base_jntPt;
  double FFdofs[6];
  double vmin,vmax;
  int i;
  double cur_inc_ang, inc_factor;
  int fres, lres;
  int array_sch[MAX_LOOP_LENGTH]; // WARNING : max. loop size limited at 50 residues !
  int ordered_array_sch[MAX_LOOP_LENGTH];
  int size_arr = MAX_LOOP_LENGTH;
  int index_arr, index_res;
  int num_sch;
  Joint_tablespt *jnt_table;

  // init parameters
  MAX_ITER_WHOLE_PROC = 500;
  MAX_NUM_BKB_TEST = 300;
  MAX_NUM_SCH_TEST = 10;
  MAX_NUM_ONESCH_TEST = 100;

  // WARNING : suppose that loop is currently "set and updated" at conf. qp 

  // alloc and set q
  q = p3d_copy_config(robotPt,qp);

  // identify base-jnt
  //base_jntPt = ctPt->actjnts[0];
  if((ctPt->rlgPt == NULL) || (ctPt->rlgPt->rlgchPt == NULL)) {
    printf("ERROR : bio_generate_nearby_random_loop_conf :  RLG is not set : required for information\n");
    p3d_destroy_config(robotPt,q);
    return NULL;
  }
  base_jntPt = ctPt->rlgPt->rlgchPt->rlg_data[0]->jnt;

  // generate base-jnt parameters
  if(Fb != NULL) {
    p3d_mat4Copy(base_jntPt->pos0,FJb0);
    p3d_mat4Mult(Fb,FbtoJbase,FJbi);
    p3d_matInvertXform(FJb0,invT); 
    p3d_mat4Mult(invT,FJbi,Frel);
    p3d_mat4ExtractPosReverseOrder(Frel,&(FFdofs[0]),&(FFdofs[1]),&(FFdofs[2]),&(FFdofs[3]),&(FFdofs[4]),&(FFdofs[5]));
    for(i=0; i<6; i++) {
      p3d_jnt_get_dof_bounds(base_jntPt,i,&vmin,&vmax);
      if((vmax < FFdofs[i]) || (vmin > FFdofs[i])) {
	printf("ERROR : bio_generate_nearby_random_loop_conf : exceeded jnt limints : J%d\n",base_jntPt->num);
	p3d_destroy_config(robotPt,q);
	return NULL;
      }
      q[base_jntPt->index_dof + i] = FFdofs[i];
      p3d_jnt_set_dof(base_jntPt,i,FFdofs[i]);
    }
  }

  // QUITAR !!! ///////
  //p3d_update_this_robot_pos_without_cntrt(robotPt);	
  //g3d_draw_allwin_active();
  /////////////////////

  // set end-frame for loop-closure constraint
  if(Fe != NULL) {
    p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
    p3d_matInvertXform(base_jntPt->abs_pos,invT); 
    p3d_mat4Mult(invT,Fe,Frel);
    ctPt->argu_d[0]  = Frel[0][0]; ctPt->argu_d[1]  = Frel[1][0]; ctPt->argu_d[2]  = Frel[2][0]; 
    ctPt->argu_d[3]  = Frel[0][1]; ctPt->argu_d[4]  = Frel[1][1]; ctPt->argu_d[5]  = Frel[2][1]; 
    ctPt->argu_d[6]  = Frel[0][2]; ctPt->argu_d[7]  = Frel[1][2]; ctPt->argu_d[8]  = Frel[2][2]; 
    ctPt->argu_d[9]  = Frel[0][3]; ctPt->argu_d[10] = Frel[1][3]; ctPt->argu_d[11] = Frel[2][3]; 
  }

  // alloc and set q_save
  q_save = p3d_copy_config(robotPt,q);
  // alloc q_inter
  q_inter = p3d_alloc_config(robotPt);

  /********* set side-chain array and desactivate collisions *******/
  // first and last loop residues
  fres = get_AAnumber_from_jnt(base_jntPt->next_jnt[0]); // WARNING : suppose that 1st bkb jnt is the next to the base-jnt
  lres = get_AAnumber_from_jnt(ctPt->pasjnts[7]);
  
  // init array_sch at 0
  for(i=0; i < size_arr; i++) {
    array_sch[i] = 0;
  }

  // ordered array of indices of resudues with side-chain  
  jnt_table = give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,fres));
  i = 0;
  num_sch = 0;
  while((fres + i) <= lres) {
    if(jnt_table[fres + i]->n_sc_joints > 0) {
      ordered_array_sch[num_sch] = jnt_table[fres + i]->namino;  // must be ==  fres + i
      num_sch ++;
    }
    i ++;
  }      

  // deactivate collisions of (mobile) side-chains
  supress_sc_rigid_sequence(robotPt->num, fres, lres);
  
  /******************/

  // main iteration : iterate the whole process
  procOK = 0;
  for(n_whole_proc=0; (n_whole_proc < MAX_ITER_WHOLE_PROC) && !procOK ; n_whole_proc++) {
    // NOTE : collisions are checked for the whole bkb+sch confromation
    //        -> OTHER POSSIBILITY : check progressively like in 'bio_shoot_loop'

    // NOTE : values are uniformly sampled in a reduced interval (that increase iteratively)
    //        -> OTHER POSSIBILITY : using normal distributions (iteratively increasing sigma)

    // compute bkb loop conformation
    for(n_bkb_test=0; (n_bkb_test < MAX_NUM_BKB_TEST) && !procOK ; n_bkb_test++) {
      // reset q
      p3d_copy_config_into(robotPt,q_save,&q);

      // adapt sampling interval
      inc_factor = (double)(n_bkb_test+1) / (double)MAX_NUM_BKB_TEST;
      cur_inc_ang = max_inc_ang * inc_factor;
      if(cur_inc_ang < MIN_INC_ANG)
	cur_inc_ang = MIN_INC_ANG;

      // perturb conf. of active sub-chain
      jntPt = base_jntPt->next_jnt[0]; // WARNING : suppose that 1st bkb jnt is the next to the base-jnt
      if(!bio_recursive_perturb_jnt_val_and_set_qi(jntPt,q,qp,cur_inc_ang,ctPt->pasjnts[0],1)) {
	p3d_destroy_config(robotPt,q);
	p3d_destroy_config(robotPt,q_save);
	p3d_destroy_config(robotPt,q_inter);
	return NULL;
      }

      // QUITAR !!! ///////
      //p3d_update_this_robot_pos_without_cntrt(robotPt);	
      //g3d_draw_allwin_active();
      /////////////////////
      
      // solve for passive sub-chain
      if(p3d_update_this_robot_pos_multisol(robotPt,qp,dl,NULL)) {
	if(DEBUG_LOOP_TRACKING) 
	  printf("### LOOP CLOSURE OK at iter. %d - inc_ang = %f\n",n_bkb_test + 1,cur_inc_ang);
	// check bkb collision
	if(bio_all_molecules_col() > 0) {
	  if(DEBUG_LOOP_TRACKING_COLL) {
	    printf("--- BKB COLLIDES ---\n");
	    afficher_lescollisions();		
	  }
	}
	else {
	  procOK = 1;
	  printf("### FREE LOOP BKB at iter. %d\n",n_bkb_test + 1);
	}	    
      }
      else {
/* 	if(DEBUG_LOOP_TRACKING)  */
/* 	  printf("### LOOP CLOSURE FAILS at iter. %d\n",n_bkb_test + 1); */
      }
    } // end bkb iteration 

    if(!procOK) {
      printf("Can't generate BKB\n");
    }

    // get q modified by cntrt
    p3d_get_robot_config_into(robotPt,&q);
    // set q_inter
    p3d_copy_config_into(robotPt,q,&q_inter);

    if(procOK) {    
      procOK = 0;
      // compute sch conformation
      for(n_sch_test=0; (n_sch_test < MAX_NUM_SCH_TEST) && !procOK ; n_sch_test++) {
	// reset q
	p3d_copy_config_into(robotPt,q_inter,&q);
	p3d_update_this_robot_pos_without_cntrt(robotPt);	

	// init array of side-chains with random order
	bio_init_array_sch(array_sch, ordered_array_sch, num_sch);
	index_arr = 0;
	procOK2 = 1;
	while(procOK2 && array_sch[index_arr] != 0) {
	  index_res = array_sch[index_arr];
	  // activate collision of this side-chain
	  activate_sc_rigid(robotPt->num, index_res);
	  // change conformation only if collides
	  //if(bio_sc_col(robotPt->num, index_res) > 0) {
	  if(bio_sc_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0) {
	    n_onesh_test = 0;
	    procOK2 = 0;
	    while(!procOK2 && (n_onesh_test < MAX_NUM_ONESCH_TEST)) {    
	      n_onesh_test ++;  
	      // adapt sampling interval
	      inc_factor = (double)n_onesh_test / (double)MAX_NUM_ONESCH_TEST;
	      cur_inc_ang = MAX_INC_ANG * inc_factor;
	      if(cur_inc_ang < max_inc_ang)
		cur_inc_ang = max_inc_ang;
	      // generate this side-chain conformation 
	      jntPt = jnt_table[index_res]->sc_joints[0];
	      if(!bio_recursive_perturb_jnt_val_and_set_qi(jntPt,q,qp,cur_inc_ang,NULL,0)) {
		p3d_destroy_config(robotPt,q);
		p3d_destroy_config(robotPt,q_save);
		p3d_destroy_config(robotPt,q_inter);
		return NULL;
	      }
	      // check collision
	      p3d_update_this_robot_pos_without_cntrt(robotPt);	
	      // QUITAR !!! ///////
	      g3d_draw_allwin_active();
	      //if(!bio_sc_col(robotPt->num, index_res) > 0) {
	      if(!bio_sc_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0) {
		procOK2 = 1;
	      }
	      else {
		if(DEBUG_LOOP_TRACKING_COLL) {
		  printf("--- SCH of res num.%d COLLIDES ---\n",index_res);
		  afficher_lescollisions();		
		}	
	      }
	    }
	  }
	  index_arr ++;
	}
	procOK = procOK2 && (array_sch[index_arr] == 0);
	if(!procOK) {
	  // deactivate collisions of (mobile) side-chains
	  supress_sc_rigid_sequence(robotPt->num, fres, lres);
	}
      } // end sch iteration
    }      
  } // end main iteration

  // free q_save and q_inter
  p3d_destroy_config(robotPt,q_save);
  p3d_destroy_config(robotPt,q_inter);

  if(!procOK) {
    p3d_destroy_config(robotPt,q);
    printf("Can't generate SCH\n");
    return NULL;
  }

  // display
  g3d_draw_allwin_active();

  return q;
}


/************************************************************/
/************************************************************/
/* external functions */
/************************************************************/
/************************************************************/

/************************************************************/
// SETTING function
// WARNING : setting arguments are stored in static global variables
int bio_set_track_loop_from_FbFe_sequence(int indJbase, int indJend, char *framestraj_filename )
{
  p3d_rob *robotPt;
  p3d_matrix4 Frel,invT;
  double Dval[12];
  configPt curq;
  p3d_jnt *J=NULL;
  double vmin,vmax;
  int i,ifpj,irj;

  /*** setting joints ***/
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  // loopJbase must be the FF joint placed at the same point that the first bkb rotation
  loopJbase = robotPt->joints[indJbase];
  // loopJbase must be the last joint in the loop
  loopJend = robotPt->joints[indJend];

  // reading Fb and Fe sequence
  if(!bio_read_and_set_FbFe_sequence_from_file(framestraj_filename)) 
    return 0;
  else 
    printf("Reading Fb-Fe sequence : OK\n");

  /*** setting loop IK  ***/
  // NOTE : the passive variables are at the C-terminal part of the loop
  //        the passive chain is the same for all the path (is should be changed)

  // compute transformation Fb->Fe
  p3d_matInvertXform(loopJbase->abs_pos,invT); 
  p3d_mat4Mult(invT,*(listFe[0]),Frel);    

  Dval[0]  = Frel[0][0]; Dval[1]  = Frel[1][0]; Dval[2]  = Frel[2][0]; 
  Dval[3]  = Frel[0][1]; Dval[4]  = Frel[1][1]; Dval[5]  = Frel[2][1]; 
  Dval[6]  = Frel[0][2]; Dval[7]  = Frel[1][2]; Dval[8]  = Frel[2][2]; 
  Dval[9]  = Frel[0][3]; Dval[10] = Frel[1][3]; Dval[11] = Frel[2][3]; 

  // put on peptide planes
  curq = p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt,&curq);

  // WARNING : suppose that the first loop bkb joint is the next to loopJbase
  J = loopJbase->next_jnt[loopJbase->n_next_jnt - 1];
  while(J != loopJend) {
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if(vmax < M_PI)
      p3d_jnt_set_dof(J,0,-M_PI);
    else
      p3d_jnt_set_dof(J,0,M_PI);
    J = J->next_jnt[J->n_next_jnt - 1];
  }
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);

  // last passive joint == loopJend
  // base-joint  == loopJbase
  // consider that end-frame is defined by a trasnfromation (Frel) from the subchain base-jnt
  irj = loopJbase->num;
  // first passive joint
  J = loopJend;
  for(i=0; i<8; i++) {
    J = J->prev_jnt;
  }
  ifpj = J->num;
  /*   if(!p3d_constraint("p3d_6R_bio_ik",1,&ifpj,1,&irj,0,NULL,0,NULL,-1,1)) */
  if(!p3d_constraint("p3d_6R_bio_ik_nopep_new",1,&ifpj,1,&irj,12,Dval,0,NULL,-1,1)) {
    p3d_set_robot_config(robotPt,curq);
    p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
    p3d_destroy_config(robotPt,curq);
    return 0;
  }

  // restore config
  p3d_set_robot_config(robotPt,curq);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
  p3d_destroy_config(robotPt,curq);

  // compute Frel between Fb[0] and loopJbase : FbtoJbase
  p3d_matInvertXform(*(listFb[0]),invT); 
  p3d_mat4Mult(invT,loopJbase->abs_pos,FbtoJbase);    

  // set global flag
  TRACK_LOOP_FLAG = TRUE;

  return 1;
}


/************************************************************/
// MAIN function
int bio_track_loop_from_FbFe_sequence( void )
{
  p3d_rob *robotPt;
  p3d_cntrt *ctPt;
  configPt qs, q, qp;
  int *iksols=NULL, **iksol=NULL;
  p3d_graph *G;
  p3d_node  *N=NULL,*Np=NULL,*Ns=NULL;
  double tu,ts; 
  int i;
  double max_inc_ang;
  double dist;

  //  WARNING : suppose ONE ONLY loop !!!
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ctPt = robotPt->cntrt_manager->cntrts[0];

  ChronoOn();

  /* NOTE :
     The solution will be stored in a GRAPH with only a sequence of NODES
     (without edges) that contain the sequence of loop confromations
  */

  // create graph
  if(!XYZ_GRAPH)  G = p3d_create_graph();
  else            G = XYZ_GRAPH;

  // put loop (robot) in initial conformation and generate 1st node
  // WARNING : suppose that initial conformation is in INIT
  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  p3d_copy_iksol(robotPt->cntrt_manager, NULL, &iksols);
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  Ns  = p3d_APInode_make_multisol(G,qs,NULL);
  p3d_insert_node(G,Ns);
  p3d_create_compco(G,Ns);
  Ns->type = ISOLATED;  

  p3d_set_robot_config(robotPt,qs);
  p3d_update_this_robot_pos_without_cntrt(robotPt);

  // computing loop path
  qp = qs;
  Np = Ns;
  for(i=1; i<nframes; i++) {
    max_inc_ang = bio_compute_maximum_allowed_increment_jnt_angle(*(listFe[i-1]),*(listFe[i]));
    q = bio_generate_nearby_random_loop_conf(robotPt,ctPt,*(listFb[i]),*(listFe[i]),qp,max_inc_ang);
    if(q != NULL) {
      printf("\n### FRAME %d from %d reached\n\n",i,nframes);
      p3d_get_iksol_vector(robotPt->cntrt_manager,&iksol);
      // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
      N = p3d_APInode_make_multisol(G, q, NULL);
      N->type = LINKING;
      p3d_insert_node(G,N);
      // NOTE : 'dist' is the distance in the configuration space
      dist = p3d_dist_config(robotPt,N->q,Np->q);
      p3d_create_edges(G,Np,N,dist);
      p3d_add_node_compco(N,Np->comp, TRUE);
      qp = q;
      Np = N;
    }
    else {
      printf("bio_track_loop_from_FbFe_sequence : FAILURE : can't reach frame num.: %d\n",i);
      return 0;
    }
  }

  PrintInfo(("\n\n### For the generation of the loop path ###\n"));
  ChronoTimes(&tu,&ts);
  G->time = G->time + tu;
  ChronoPrint("");
  ChronoOff();
  p3d_print_info_graph(G);

  // set GOAL conf  
  p3d_copy_config_into(robotPt,N->q,&(robotPt->ROBOT_GOTO));
  /* Initialize some data in the graph for the A* graph search */
  G->search_start = Ns;
  G->search_goal = N; 
  G->search_done = FALSE;
  // generate a path 
  if(p3d_graph_to_traj(robotPt)) {
    g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));    
    printf("G to path OK\n");
  }

  return 1;
}


/************************************************************/



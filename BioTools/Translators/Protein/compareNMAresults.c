/**********************************************************************/
// INCLUDES

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "defs.h"
#include "atoms.h"
#include "protein.h"
#include "atomhandling.h"
#include "mathfcts.h"
#include "pdbFormat.h"

/**********************************************************************/
// EXTERN FUNCTIONS

extern int fill_protein_struct(FILE *pdbfile, char *pdbfilename, protein *protPt);
extern void free_protein(protein *protPt);
extern int number_bkb_atoms(residueTypes resType);
extern int number_sch_atoms(residueTypes resType);

/**********************************************************************/
// GLOBAL VARIABLES

// call arguments
//  - files
#define ALL_ATOMS 1
#define ALL_HEAVY_ATOMS 2
#define ALL_BKB_HEAVY_ATOMS 3
#define CA_ATOMS 4

static char str_which_atoms[10];
static char INITpdbfilename[255],GOALpdbfilename[255],NMApdbfilename[255];
static FILE *INITpdbfile,*GOALpdbfile,*NMApdbfile;
//  - options
//     NONE

/**********************************************************************/
// GENERAL FUNCTIONS

static int read_call_arguments(int argc, char **argv)
{
   if(argc < 4)
   {
      printf("Not enought arguments\n");
      return -1;
   }
   
   if((!strcpy(str_which_atoms,argv[1])) ||
      (!strcpy(INITpdbfilename,argv[2])) ||
      (!strcpy(GOALpdbfilename,argv[3])) ||
      (!strcpy(NMApdbfilename,argv[4])))
     return -1;

   return 1;
}

/**********************************************************************/
/**********************************************************************/

static int number_of_res_in_prot(protein *protPt)
{
  int i;
  int nres=0;
  
  for(i=0;i<protPt->nchains;i++) {
    nres += protPt->chainlist[i]->nresidues;
  }

  return (nres);
}

static int number_of_atoms_in_prot(protein *protPt, int which_atoms)
{
  int i,j,k;
  residue *resPt;
  int nha = 0;
  atom *aPt;
  int nmax_bkb_atoms;
  int nmax_sch_atoms;
  
  for(i=0;i<protPt->nchains;i++) {
    for(j=0;j<protPt->chainlist[i]->nresidues;j++) {
      resPt = protPt->chainlist[i]->reslist[j];
      nmax_bkb_atoms = number_bkb_atoms(resPt->resType);
      nmax_sch_atoms = number_sch_atoms(resPt->resType);
      for(k=0;k<nmax_bkb_atoms;k++) {
	aPt = resPt->bkbAlist[k];
	if(aPt != NULL) {
	  if((aPt->atomType != HYDROGEN) && (aPt->atomType != HYDROGEN_P)) {
	    nha++;
	  }
	}
      }
      if((which_atoms == ALL_ATOMS)||(which_atoms == ALL_HEAVY_ATOMS)) {
	for(k=0;k<nmax_sch_atoms;k++) {
	  aPt = resPt->schAlist[k];
	  if(aPt != NULL) {
	    if((which_atoms == ALL_ATOMS) || ((aPt->atomType != HYDROGEN) && (aPt->atomType != HYDROGEN_P))) {
	      nha++;
	    }
	  }
	}
      }
    }
  }
  
  return (nha);
}


static int copy_prot_CApos_into_array(protein *protPt,double **arrayCApos)
{
  int i,j;
  AAchain *AAchainPt;
  residue *resPt;
  int nres,totnres=0;
  double *apos;
  
  for(i=0;i<protPt->nchains;i++) {
    AAchainPt = protPt->chainlist[i];
    nres = AAchainPt->nresidues;
    for(j=0;j<nres;j++) {
      resPt = AAchainPt->reslist[j];
      get_CA_pos(resPt,&apos);
      arrayCApos[totnres] = apos;
      totnres++;
    }
  }

  return 1;
}


static int copy_prot_heavy_atom_pos_into_array(protein *protPt,double **arrayHApos, int which_atoms)
{
  int i,j,k;
  AAchain *AAchainPt;
  residue *resPt;
  int nres;
  int totnha=0;
  atom *aPt;
  double *apos;
  int nmax_bkb_atoms;
  int nmax_sch_atoms;
  
  // NOTE: normally, atoms are stored always in the same order in de psf lists
  for(i=0;i<protPt->nchains;i++) {
    AAchainPt = protPt->chainlist[i];
    nres = AAchainPt->nresidues;
    for(j=0;j<nres;j++) {
      resPt = AAchainPt->reslist[j];
      nmax_bkb_atoms = number_bkb_atoms(resPt->resType);
      nmax_sch_atoms = number_sch_atoms(resPt->resType);
      for(k=0;k<nmax_bkb_atoms;k++) {
	aPt = resPt->bkbAlist[k];
	if(aPt != NULL) {
	  if((aPt->atomType != HYDROGEN) && (aPt->atomType != HYDROGEN_P)) {
	    apos = aPt->pos;
	    arrayHApos[totnha] = apos;
	    totnha++;
	  }
	}
      }
      if((which_atoms == ALL_ATOMS)||(which_atoms == ALL_HEAVY_ATOMS)) {
	for(k=0;k<nmax_sch_atoms;k++) {
	  aPt = resPt->schAlist[k];
	  if(aPt != NULL) {
	    if((which_atoms == ALL_ATOMS) || ((aPt->atomType != HYDROGEN) && (aPt->atomType != HYDROGEN_P))) {
	      apos = aPt->pos;
	      arrayHApos[totnha] = apos;
	      totnha++;
	    }
	  }
	}
      }
    }
  }
  
  return 1;
}


/**********************************************************************/
/**********************************************************************/

int bio_compare_3_structures(protein *INITprotPt, protein* GOALprotPt, protein *NMAprotPt, int which_atoms)
{
  int nselectedatoms;
  int nres,nres2,nres3;
  int nha,nha2,nha3;
  double **INIT_Apos,**GOAL_Apos,**NMA_Apos;
  double Ai[3],Ri[3],Ni[3];
  double Ai_norm,Ri_norm,Ni_norm;
  double sum_Ai_norm,sum_Ri_norm;
  double sum_Ai_norm2,sum_Ri_norm2,sum_Ni_norm2;
  double avr_Ai_norm,avr_Ri_norm;
  double sum_Ai2,sum_Ri2;
  double sigma_Ai,sigma_Ri,sigma_Ni;
  double sum_Ai_prod_Ri;
  double sum_dev_Ai_Ri;
  double overlap,correlation;
  int i;

  // get and compare number of residues
  nres = number_of_res_in_prot(INITprotPt);
  nres2 = number_of_res_in_prot(GOALprotPt);
  nres3 = number_of_res_in_prot(NMAprotPt);
  if((nres != nres2) || (nres != nres2)) {
    printf("ERROR: proteins have different number of residues\n");
    return -1;
  }

  // atoms selected = CA
  if(which_atoms == CA_ATOMS) {
    nselectedatoms = nres;
    // create arrays with CA positions
    INIT_Apos = (double**)malloc(sizeof(double*)*nres);
    GOAL_Apos = (double**)malloc(sizeof(double*)*nres);
    NMA_Apos = (double**)malloc(sizeof(double*)*nres);
    
    copy_prot_CApos_into_array(INITprotPt,INIT_Apos);
    copy_prot_CApos_into_array(GOALprotPt,GOAL_Apos);
    copy_prot_CApos_into_array(NMAprotPt,NMA_Apos);
  }    
  // atoms selected = all heavy atoms (with or without side-chain)
  else {
    nha = number_of_atoms_in_prot(INITprotPt,which_atoms);
    nha2 = number_of_atoms_in_prot(INITprotPt,which_atoms);
    nha3 = number_of_atoms_in_prot(INITprotPt,which_atoms);
    
    if((nha != nha2) || (nha != nha2)) {
      printf("ERROR: proteins have different number of atoms\n");
      return -1;
    }
    
    nselectedatoms = nha;
    
    INIT_Apos = (double**)malloc(sizeof(double*)*nha);
    GOAL_Apos = (double**)malloc(sizeof(double*)*nha);
    NMA_Apos = (double**)malloc(sizeof(double*)*nha);
    
    copy_prot_heavy_atom_pos_into_array(INITprotPt,INIT_Apos,which_atoms);
    copy_prot_heavy_atom_pos_into_array(GOALprotPt,GOAL_Apos,which_atoms);
    copy_prot_heavy_atom_pos_into_array(NMAprotPt,NMA_Apos,which_atoms);    
  }

  // compute OVERLAP and CORRELATION
  sum_Ai_norm = 0.0;
  sum_Ri_norm = 0.0;
  sum_Ai_norm2 = 0.0;
  sum_Ri_norm2 = 0.0;
  sum_Ai2 = 0.0;
  sum_Ri2 = 0.0;
  for(i=0; i<nselectedatoms; i++) {
    vectSub(INIT_Apos[i],NMA_Apos[i],Ai);
    vectSub(INIT_Apos[i],GOAL_Apos[i],Ri);
    vectSub(NMA_Apos[i],GOAL_Apos[i],Ni);
    Ai_norm = vectNorm(Ai);
    Ri_norm = vectNorm(Ri);
    Ni_norm = vectNorm(Ni);
    sum_Ai_norm += Ai_norm;
    sum_Ri_norm += Ri_norm;
    sum_Ai_norm2 += SQR(Ai_norm);
    sum_Ri_norm2 += SQR(Ri_norm);
    sum_Ni_norm2 += SQR(Ni_norm);
    sum_Ai2 += vectDotProd(Ai,Ai);
    sum_Ri2 += vectDotProd(Ri,Ri);
  }

  sigma_Ai = sqrt(sum_Ai_norm2/((double)nselectedatoms));
  sigma_Ri = sqrt(sum_Ri_norm2/((double)nselectedatoms));
  sigma_Ni = sqrt(sum_Ni_norm2/((double)nselectedatoms));
  avr_Ai_norm = sum_Ai_norm/((double)nselectedatoms);
  avr_Ri_norm = sum_Ri_norm/((double)nselectedatoms);

  sum_Ai_prod_Ri = 0.0;
  sum_dev_Ai_Ri =0.0;
  for(i=0; i<nselectedatoms; i++) {
    vectSub(INIT_Apos[i],NMA_Apos[i],Ai);
    vectSub(INIT_Apos[i],GOAL_Apos[i],Ri);
    Ai_norm = vectNorm(Ai);
    Ri_norm = vectNorm(Ri);
    sum_Ai_prod_Ri += vectDotProd(Ai,Ri);    
    sum_dev_Ai_Ri += (Ai_norm-avr_Ai_norm)*(Ri_norm-avr_Ri_norm);
  }

  overlap = sum_Ai_prod_Ri/sqrt(sum_Ai2*sum_Ri2);
  correlation = (1.0/((double)nselectedatoms))*(sum_dev_Ai_Ri/(sigma_Ai*sigma_Ri));
  
  printf("\nOVERLAP = %f\n",overlap);
  printf("CORRELATION = %f\n",correlation);
  printf("RMS INIT-GOAL = %f\n",sigma_Ri);
  printf("RMS INIT-NMA  = %f\n",sigma_Ai);
  printf("RMS NMA-GOAL  = %f\n",sigma_Ni);

  // free memory
  free(INIT_Apos);
  free(GOAL_Apos);
  free(NMA_Apos);

  return 1;
}

/**********************************************************************/
/**********************************************************************/

int bio_compute_strain_energy_CA(protein *INITprotPt, protein *NMAprotPt)
{
  int nres,nres2;
  double **INIT_Apos,**NMA_Apos;
  int i,j;
  double posdiff[3];
  double di,di0;
  double cutoff = 8.0; // same than Karplus !
  //double C = 10.0;     // same than Karplus !
  double C = 0.02;     // same than Karplus !
  double sum_norm_dif_sqr;
  double strainE;

  // get and compare number of residues
  nres = number_of_res_in_prot(INITprotPt);
  nres2 = number_of_res_in_prot(NMAprotPt);
  if(nres != nres2) {
    printf("ERROR: proteins have different number of residues\n");
    return -1;
  }

  INIT_Apos = (double**)malloc(sizeof(double*)*nres);
  NMA_Apos = (double**)malloc(sizeof(double*)*nres);
  
  copy_prot_CApos_into_array(INITprotPt,INIT_Apos);
  copy_prot_CApos_into_array(NMAprotPt,NMA_Apos);
  
  sum_norm_dif_sqr = 0.0;
  for(i=0; i<(nres-1); i++) {    
    for(j=i+1; j<nres; j++) {
      vectSub(NMA_Apos[i],NMA_Apos[j],posdiff);
      di = vectNorm(posdiff);
      if(di < cutoff) {
	vectSub(INIT_Apos[i],INIT_Apos[j],posdiff);
	di0 = vectNorm(posdiff);
	sum_norm_dif_sqr += SQR(di - di0);
      }
    }
  }
  strainE = sum_norm_dif_sqr * C / 2.0;

  printf("Strain Energy (from CA pos) = %f\n",strainE);

  return 1;
}

/**********************************************************************/
/**********************************************************************/
// MAIN
/**********************************************************************/
/**********************************************************************/

// operations with only CA
int main(int argc, char **argv)
{
  protein *INITprotPt,*GOALprotPt,*NMAprotPt;
  int which_atoms;

  if(read_call_arguments(argc, argv) < 0) {
    printf("Usage: compareNMAresults <-all OR -ca OR -bkb> <INIT_pdbfile> <GOAL_pdbfile> <NMA_pdbfile>\n");
    return -1;
  }

  // open files
  INITpdbfile = fopen(INITpdbfilename, "r");
  if (INITpdbfile == NULL) {
    printf("INIT pdb file cannot be open\n");
    return -1;
  }
  GOALpdbfile = fopen(GOALpdbfilename, "r");
  if (GOALpdbfile == NULL) {
    printf("GOAL pdb file cannot be open\n");
    fclose(INITpdbfile);
    return -1;
  }
  NMApdbfile = fopen(NMApdbfilename, "r");
  if (NMApdbfile == NULL) {
    printf("NMA pdb file cannot be open\n");
    fclose(INITpdbfile);
    fclose(GOALpdbfile);
    return -1;
  }

  // alloc protein structures
  INITprotPt = (protein *) malloc(sizeof(protein));
  GOALprotPt = (protein *) malloc(sizeof(protein));
  NMAprotPt = (protein *) malloc(sizeof(protein));


  // read INIT PDB and write protein data structure
  if(fill_protein_struct(INITpdbfile,INITpdbfilename,INITprotPt) < 0) {
    printf("ERROR while writing protein data structure from INIT PDB\n");
    free_protein(INITprotPt);  
    free_protein(GOALprotPt);  
    free_protein(NMAprotPt);  
    fclose(INITpdbfile);
    fclose(GOALpdbfile);
    fclose(NMApdbfile);
    return -1;
  }
  // read GOAL PDB and write protein data structure
  if(fill_protein_struct(GOALpdbfile,GOALpdbfilename,GOALprotPt) < 0) {
    printf("ERROR while writing protein data structure from GOAL PDB\n");
    free_protein(INITprotPt);  
    free_protein(GOALprotPt);  
    free_protein(NMAprotPt);  
    fclose(INITpdbfile);
    fclose(GOALpdbfile);
    fclose(NMApdbfile);
    return -1;
  }
  // read NMA PDB and write protein data structure
  if(fill_protein_struct(NMApdbfile,NMApdbfilename,NMAprotPt) < 0) {
    printf("ERROR while writing protein data structure from NMA PDB\n");
    free_protein(INITprotPt);  
    free_protein(GOALprotPt);  
    free_protein(NMAprotPt);  
    fclose(INITpdbfile);
    fclose(GOALpdbfile);
    fclose(NMApdbfile);
    return -1;
  }
  
  // which atoms are selected for operations
  if(strcmp(str_which_atoms,"-all") == 0) {
    which_atoms = ALL_ATOMS;
    printf("\nComputing values for all heavy atons\n");
  }
  else if(strcmp(str_which_atoms,"-allnoH") == 0) {
    which_atoms = ALL_HEAVY_ATOMS;
    printf("\nComputing values for all heavy atons\n");
  }
  else if (strcmp(str_which_atoms,"-bkb") == 0) {
    which_atoms = ALL_BKB_HEAVY_ATOMS;
    printf("\nComputing values for backbone heavy atons\n");
  }
  else if (strcmp(str_which_atoms,"-ca") == 0) {
    which_atoms = CA_ATOMS;  
    printf("\nComputing values for CA atons\n");
  }
  else {
    printf("ERROR : wrong atom selection : argument must be -all, -bkb or -ca\n");
    free_protein(INITprotPt);  
    free_protein(GOALprotPt);  
    free_protein(NMAprotPt);  
    fclose(INITpdbfile);
    fclose(GOALpdbfile);
    fclose(NMApdbfile);
    return -1;
  }
    
  bio_compare_3_structures(INITprotPt,GOALprotPt,NMAprotPt,which_atoms);

  bio_compute_strain_energy_CA(INITprotPt,NMAprotPt);

  // free memory
  free_protein(INITprotPt);  
  free_protein(GOALprotPt);  
  free_protein(NMAprotPt);  
  fclose(INITpdbfile);
  fclose(GOALpdbfile);
  fclose(NMApdbfile);

  return 1;
}


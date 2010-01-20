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

/**********************************************************************/
// GLOBAL VARIABLES


/**********************************************************************/
// GENERAL FUNCTIONS


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

/**********************************************************************/
/**********************************************************************/

static int compute_pseudodihedrals_array(double **list_CApos, int nres, 
					 double *list_pseudodihedrals)
{
  int i;
  double posdiff[3],axis1[3],axis2[3],axis3[3];
  
  for(i=0; i<nres-3; i++) {
    vectSub(list_CApos[i+1],list_CApos[i],posdiff);
    vectNormalize(posdiff,axis1);
    vectSub(list_CApos[i+2],list_CApos[i+1],posdiff);
    vectNormalize(posdiff,axis2);
    vectSub(list_CApos[i+3],list_CApos[i+2],posdiff);
    vectNormalize(posdiff,axis3);

    list_pseudodihedrals[i] = compute_dihedang(axis3,axis2,axis1)*(180/PI);
  }
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
  protein **list_protPt;
  FILE **list_pdbfiles;
  FILE *outputfile;
  char outputfilename[256];
  int n_models;
  int j,i;
  int refmodel_nres,imodel_nres;
  double **refmodel_CApos,**imodel_CApos;
  double *refmodel_pseudodihedrals,*imodel_pseudodihedrals;
  double pseudodihedral_diff;

  n_models = argc - 1;

  if(n_models < 2 ) {
    printf("Usage: pseudodihedral_diff <pdbfile0 pdbfile1 pdbfile2 ...>\n");
    return -1;
  }

  // alloc lists
  list_pdbfiles = (FILE**)malloc(sizeof(FILE*)*n_models);
  list_protPt = (protein**)malloc(sizeof(protein*)*n_models);

  for(i=0; i<n_models; i++) {
    // open files
    list_pdbfiles[i] = fopen(argv[i+1], "r");
    if (list_pdbfiles[i] == NULL) {
      printf("pdb file cannot be open\n");
      for(j=0;j<i;j++) {
	fclose(list_pdbfiles[j]);
      }
      free(list_pdbfiles);
      free(list_protPt);
      return -1;
    }
    
    // alloc protein structure
    list_protPt[i] = (protein *) malloc(sizeof(protein));

    // write protein data structure
    if(fill_protein_struct(list_pdbfiles[i],argv[i+1],list_protPt[i]) < 0) {
      printf("ERROR while writing protein data structure from PDB\n");
      for(j=0;j<=i;j++) {
	free_protein(list_protPt[j]);	
	fclose(list_pdbfiles[j]);
      }
      free(list_pdbfiles);
      free(list_protPt);
      return -1;
    }
  }

  // reference model = list_protPt[0]
  refmodel_nres = number_of_res_in_prot(list_protPt[0]);
  refmodel_CApos = (double**)malloc(sizeof(double*)*refmodel_nres);
  copy_prot_CApos_into_array(list_protPt[0],refmodel_CApos);
  refmodel_pseudodihedrals = (double*)malloc(sizeof(double)*(refmodel_nres-3));
  compute_pseudodihedrals_array(refmodel_CApos,refmodel_nres,refmodel_pseudodihedrals);

  // open output file
  strcpy(outputfilename,"pseudodihedral_diff.output");
  outputfile = fopen(outputfilename, "w");
  printf("OUTPUT written in %s\n",outputfilename);

  // treat all the models
  imodel_CApos = (double**)malloc(sizeof(double*)*refmodel_nres);
  imodel_pseudodihedrals = (double*)malloc(sizeof(double)*(refmodel_nres-3));
  for(i=1; i<n_models; i++) {
    imodel_nres = number_of_res_in_prot(list_protPt[i]);
    if(imodel_nres != refmodel_nres) {
      printf("ERROR : all the models must have the same number of residues\n");
      printf("WARNING : incomplete output file");
      // free memory
      for(i=0;i<n_models;i++) {
	free_protein(list_protPt[i]);	
	fclose(list_pdbfiles[i]);
      }
      free(list_pdbfiles);
      free(list_protPt);
      free(refmodel_CApos);
      free(refmodel_pseudodihedrals);
      free(imodel_CApos);
      free(imodel_pseudodihedrals);
      fclose(outputfile);
      return 0;
    }
    copy_prot_CApos_into_array(list_protPt[i],imodel_CApos);
    compute_pseudodihedrals_array(imodel_CApos,imodel_nres,imodel_pseudodihedrals);
    for(j=0; j<(imodel_nres-3); j++) {
      pseudodihedral_diff = fabs(imodel_pseudodihedrals[j]-refmodel_pseudodihedrals[j]);
      if(pseudodihedral_diff > 180.0)
	pseudodihedral_diff = 360.0 - pseudodihedral_diff;
      if(pseudodihedral_diff > 20.0)
	pseudodihedral_diff = 20;
      fprintf(outputfile,"%d %d %f\n",i,j+1,pseudodihedral_diff);     
    }    
    fprintf(outputfile,"\n");     
    // MODIF FOR COMPARING WITH THE PREVIOUS CONF INSTEAD OF THE STARTING POINT
    //memcpy((void*)refmodel_pseudodihedrals, (void*)imodel_pseudodihedrals,sizeof(double)*(imodel_nres-3));  
  }    
  
  // free memory
  for(i=0;i<n_models;i++) {
    free_protein(list_protPt[i]);	
    fclose(list_pdbfiles[i]);
  }
  free(list_pdbfiles);
  free(list_protPt);
  free(refmodel_CApos);
  free(refmodel_pseudodihedrals);
  free(imodel_CApos);
  free(imodel_pseudodihedrals);
  fclose(outputfile);

  return 1;
}


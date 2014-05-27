/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
/**********************************************************************/
// INCLUDES

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**********************************************************************/
// GENERAL STRUCTURES

typedef struct s_atom_data {
  int resSeq;
  char resName[3];
} atom_data;

/**********************************************************************/
// GLOBAL VARIABLES

static char pdbfilename[255],aaafilename[255];
static FILE *pdbfile, *aaafile;

/**********************************************************************/
// FCTS DECLARATION
static int read_call_arguments(int argc, char **argv);
static int read_atom_from_pdbline(char *pdbline, atom_data *adata);

/**********************************************************************/
/**********************************************************************/
// GENERAL READING FUNCTIONS

static int read_call_arguments(int argc, char **argv)
{
   if(argc < 2)
   {
      printf("Not enought arguments\n");
      return 0;
   }

   if((!strcpy(pdbfilename,argv[1])) ||
      (!strcpy(aaafilename,argv[2])))
     return 0;

   return 1;
}


static int read_atom_from_pdbline(char *pdbline, atom_data *adata)
{
  char piece[10];
  char rec[10];

  strcpy(piece,"         ");
  strncpy(piece,pdbline,6);
  sscanf(piece,"%s",rec);
  if(strcmp(rec,"ATOM") != 0) return 0;

  // extract only used data
  strcpy(piece,"         ");
  strncpy(piece,pdbline+17,3);
  if(!sscanf(piece,"%s",adata->resName)) return 0;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+22,4);
  if(!sscanf(piece,"%d",&adata->resSeq)) return 0;

  return 1;
}


/**********************************************************************/
// MAIN

int main(int argc, char **argv)
{
  char pdbline[100];
  atom_data atom;
  int cur_resSeq;

  if(read_call_arguments(argc, argv) == -1)
   {
     printf("Usage: gen_aaafile <pdbfile> <aaafile>\n");
     return 0;
   }

  // open files
  pdbfile = fopen(pdbfilename, "r");
  if (pdbfile == NULL) {
    printf("pdb file cannot be open\n");
    return 0;
  }
  aaafile = fopen(aaafilename, "w");
  if (aaafile == NULL) {
    printf("aaa file cannot be open\n");
    fclose(pdbfile);
    return 0;
  }

  // write aaa head
  fprintf(aaafile,"/*\n# ARTICULATED AMINO-ACIDS INPUT FILE\n\n# LINE FORMAT :\nresName resSeq <rigid-bkb OR Lflex-bkb OR Mflex-bkb OR Fflex-bkb> <rigid-sch OR Lflex-sch OR Mflex-sch OR Fflex-sch>\n\n- bkb = backbone\n- sch = side-chain\n\n- Fflex = full rotation\n- Mflex = medium flexibility (+-30 deg)\n- Lflex = low flexibility (+-10 deg)\n\n////////////////////////////\n         1         2\n1234567890123456789012345678\n////////////////////////////\n*/\n\n");
  fprintf(aaafile,"SUB\n");

  cur_resSeq = 0;
  while(fgets(pdbline,100,pdbfile)) {
    if(read_atom_from_pdbline(pdbline,&atom)) {
      if(atom.resSeq != cur_resSeq) {
	if((cur_resSeq != 0) && (atom.resSeq != cur_resSeq + 1))
	  fprintf(aaafile,"SUB\n");
	cur_resSeq = atom.resSeq;
	if(strcmp(atom.resName,"HIE") == 0) {
	  fprintf(aaafile,"%s","HIS");
	  printf("WARNING : residue %d name HIE replaced by HIS\n",atom.resSeq);
	}
	else
	  fprintf(aaafile,"%s",atom.resName);
	if(atom.resSeq < 10)
	  fprintf(aaafile,"    ");
	else if(atom.resSeq < 100)
	  fprintf(aaafile,"   ");
	else if(atom.resSeq < 1000)
	  fprintf(aaafile,"  ");
	else
	  fprintf(aaafile," ");
	fprintf(aaafile,"%d rigid-bkb rigid-sch\n",atom.resSeq);
      }
    }
  }
  
  // close files
  fclose(pdbfile);
  fclose(aaafile);
 
  return 1;
}


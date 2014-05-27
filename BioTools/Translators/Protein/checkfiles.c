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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define TRUE 1
#define FALSE 0

typedef struct s_pdb_file_data {
  int serial;
  char name[5];
  int resSeq;
  char resName[4];
/*   char chainID[2]; */
  double pos[3];
} pdb_file_data;


/**********************************************************************/
// GLOBAL VARIABLES

static char pdbfilename1[255];
static char pdbfilename2[255];
static FILE *pdbfile1 = NULL;
static FILE *pdbfile2 = NULL;


/**********************************************************************/

static int read_atom(FILE *pdbfile, pdb_file_data *file_dataPt, int* npdfline)
{
  char *pdbline;
  char piece[10];
  char rec[10];
  char returned_s[100];
  

  pdbline = fgets(returned_s,100,pdbfile);
  if(pdbline == NULL) {
     return 0;
  }
  (*npdfline)++;
    
  strcpy(piece,"         ");
  strncpy(piece,pdbline,6);
  if(sscanf(piece,"%s",rec) < 0) return -2;
  if(strcmp(rec,"ATOM") != 0) return -2;

  // extract only used data
  strcpy(piece,"         ");
  strncpy(piece,pdbline+6,5);
  if(sscanf(piece,"%d",&file_dataPt->serial) < 0) return -1;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbline+12,4);
  if(sscanf(piece,"%s",file_dataPt->name) < 0) return -1;

/*   strcpy(piece,"         "); */
/*   strncpy(piece,pdbline+21,1); */
/*   sscanf(piece,"%s",file_dataPt->chainID); */
/*   // when no chainID */
/*   if(strcmp(file_dataPt->chainID,"") == 0) */
/*     strcpy(file_dataPt->chainID,"0"); */
  
  strcpy(piece,"         ");
  strncpy(piece,pdbline+17,3);
  if(sscanf(piece,"%s",file_dataPt->resName) < 0) return -1;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbline+22,4);
  if(sscanf(piece,"%d",&file_dataPt->resSeq) < 0) return -1;

  strcpy(piece,"         ");
  strncpy(piece,pdbline+30,8);
  if(sscanf(piece,"%lf",&file_dataPt->pos[0]) < 0) return -1;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+38,8);
  if(sscanf(piece,"%lf",&file_dataPt->pos[1]) < 0) return -1;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+46,8);
  if(sscanf(piece,"%lf",&file_dataPt->pos[2]) < 0) return -1; 

  return 1;
}


static int cmp_file_data(pdb_file_data *file_dataPt_1, pdb_file_data *file_dataPt_2) {

 int i = 0;
 int error = FALSE;

 if (file_dataPt_1->serial!=file_dataPt_2->serial) {
   printf("different serials 1: %d - 2: %d",file_dataPt_1->serial,file_dataPt_2->serial);
   return FALSE;
 }
 if (strncmp(file_dataPt_1->name, file_dataPt_2->name,4)!=0) {
   printf("different names 1: %s - 2: %s",file_dataPt_1->name,file_dataPt_2->name);
   return FALSE;
 }
 if (file_dataPt_1->resSeq!=file_dataPt_2->resSeq) {
   printf("different resSeqs 1: %d - 2: %d",file_dataPt_1->resSeq,file_dataPt_2->resSeq);
   return FALSE;
 }
 if (strncmp(file_dataPt_1->resName, file_dataPt_2->resName,3)!=0) {
   printf("different resNames 1: %s - 2: %s",file_dataPt_1->resName,file_dataPt_2->resName);
   return FALSE;
 }
/*  if (strncmp(file_dataPt_1->chainID, file_dataPt_2->chainID,1)!=0) { */
/*    printf("different chainIDs 1: %s - 2: %s",file_dataPt_1->chainID,file_dataPt_2->chainID); */
/*    return FALSE; */
/*  } */
 while(i<3 && !error) {
   error = (file_dataPt_1->pos[i]!=file_dataPt_2->pos[i]);
   if(!error) {
     i++;
   }
   else {
     printf("different pos[%d]) 1: %f - 2: %f",i,file_dataPt_1->pos[i],file_dataPt_2->pos[i]);  
   }
 }

 return !error;
 
}


static int read_call_arguments(int argc, char **argv)
{
  
  if(argc < 3)
   {
      printf("Not enought arguments\n");
      return FALSE;
   }
  
  if((!strcpy(pdbfilename1,argv[1])) ||
     (!strcpy(pdbfilename2,argv[2])))
    return FALSE;
  
  return TRUE;
}


/**********************************************************************/
/**********************************************************************/
// MAIN
/**********************************************************************/
/**********************************************************************/

int main(int argc, char **argv)
{
  int status1;
  int status2;
  int cmpOK = TRUE;
  int line1 = 0;
  int line2 = 0;
  pdb_file_data data1;
  pdb_file_data data2;

  // get arguments
  if(read_call_arguments(argc, argv) < 0) {
    printf("Usage: checkfiles <pdbfile1> <pdbfile2>\n");
    return FALSE;
  }

  // open files
  pdbfile1 = fopen(pdbfilename1, "r");
  if (pdbfile1 == NULL) {
    printf("first pdb file cannot be open\n");
    return FALSE;
  }
  pdbfile2 = fopen(pdbfilename2, "r");
  if (pdbfile2 == NULL) {
    printf("second pdb file cannot be open\n");
    return FALSE;
  }


  do {

    status1 = read_atom(pdbfile1, &data1, &line1);
    while (status1==-2){
      status1 = read_atom(pdbfile1, &data1, &line1);
    }
    status2 = read_atom(pdbfile2, &data2, &line2);
    while (status2==-2){
      status2 = read_atom(pdbfile2, &data2, &line2);
    }

    if (status1>0 && status2>0) {
      cmpOK = cmp_file_data(&data1, &data2);
      if(!cmpOK) {
	printf("(1:line %d - 2:line %d)\n",line1,line2);
      }
    }
    
  } while (status1>0 && status2>0);

  if (status1<0 && status2<0) {
    printf("ERROR while reading files\n");
  }
  
  fclose(pdbfile1);
  fclose(pdbfile1);
  return TRUE;
}

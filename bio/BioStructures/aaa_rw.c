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

#include "P3d-pkg.h"
#include "Bio-pkg.h"

// PRIVATE FUNCTIONS ///////////////////////////////////////////////////////////

static void aaa_insert_pointer_in_list(void *thePt, void ***listPt, int *nelems);
static void aaa_clear_pointer_list(void ***listPt, int *size);
static void aaa_free_pointer_list(void ***listPt, int *size);

static int read_aaa_line(char *aaaLine, AAA_residue_data** aaa_residue, int* other_line);
static void write_aaa_in_file(FILE* aaaFile, AAA_residue_data* aaa_residue);



// LIST MANAGEMENT /////////////////////////////////////////////////////////////

static void aaa_insert_pointer_in_list(void *thePt, void ***listPt, int *nelems)
{
  if(*nelems == 0) {
    *listPt = (void **) malloc(sizeof(void *));
  }
  else {
    *listPt = (void **) realloc(*listPt,sizeof(void *) * (*nelems + 1));
  }
  (*listPt)[*nelems] = thePt;
  (*nelems)++;
}

/**********************************************************************/

static void aaa_clear_pointer_list(void ***listPt, int *size) {
  
  int i;
  for (i=0; i<*size; i++) {
    free((*listPt)[i]);
    (*listPt)[i]=NULL;
  }

  *size=0;
}

/**********************************************************************/

static void aaa_free_pointer_list(void ***listPt, int *size) {
  
  if (*size!=0)
    aaa_clear_pointer_list(listPt, size);
  
  free(*listPt);
  *listPt = NULL;
}

// FUNCTIONS ON AAA STRUCT /////////////////////////////////////////////////////

static void add_AAA_Residue(AAA_residue_data_list* AAA_List, AAA_residue_data* AAA_Residue) {
  aaa_insert_pointer_in_list(AAA_Residue,(void ***)&(AAA_List->AAA_residueList),&(AAA_List->nb_AAA_residues));
}

/**********************************************************************/

AAA_residue_data* create_AAA_Residue(int resSeq, const char *resName, int art_bkb, int art_sch) {
  
  AAA_residue_data* AAA_Residue = (AAA_residue_data*)malloc(sizeof(AAA_residue_data));
  AAA_Residue->resSeq = resSeq;
  strncpy(AAA_Residue->resName, resName, 4) ;
  AAA_Residue->art_bkb = art_bkb;
  AAA_Residue->art_sch = art_sch;

  return AAA_Residue;
}

/**********************************************************************/

void set_Bkb_Art(AAA_residue_data* AAA_Residue, int art) {
  AAA_Residue->art_bkb = art;
}

/**********************************************************************/

void set_Sch_Art(AAA_residue_data* AAA_Residue, int art) {
  AAA_Residue->art_sch = art;
}

/**********************************************************************/

void aaa_toggle_Bkb_Art(AAA_residue_data *AAA_Residue) {
 
  if (AAA_Residue->art_bkb == RIGID)
    set_Bkb_Art(AAA_Residue, MOBILE);
  else if (AAA_Residue->art_bkb == MOBILE)
    set_Bkb_Art(AAA_Residue, RIGID);
}

/**********************************************************************/

void aaa_toggle_Sch_Art(AAA_residue_data *AAA_Residue) {

  if (AAA_Residue->art_sch == RIGID)
    set_Sch_Art(AAA_Residue, MOBILE);
  else if (AAA_Residue->art_sch == MOBILE)
    set_Sch_Art(AAA_Residue, RIGID);

}

/**********************************************************************/

AAA_residue_data_list* copy_AAA_residue_data_list(AAA_residue_data_list* src_AAA_List) {

  int res;
  
  AAA_residue_data* AAA_Residue;
  AAA_residue_data* copied_AAA_Residue;

  AAA_residue_data_list* copied_AAA_List;

  copied_AAA_List = (AAA_residue_data_list*)malloc(sizeof(AAA_residue_data_list));
  copied_AAA_List->nb_AAA_residues = 0;

  for(res=0; res<src_AAA_List->nb_AAA_residues; res++) {

    AAA_Residue = src_AAA_List->AAA_residueList[res];
    copied_AAA_Residue = create_AAA_Residue(AAA_Residue->resSeq,
					    AAA_Residue->resName,
					    AAA_Residue->art_bkb,
					    AAA_Residue->art_sch);

    add_AAA_Residue(copied_AAA_List, copied_AAA_Residue);
  }
   
  return copied_AAA_List;

}

/**********************************************************************/

int update_AAA_residue_data_list(AAA_residue_data_list* src_AAA_List, AAA_residue_data_list* AAA_List_to_update) {
  
  int res = 0;
  
  AAA_residue_data* src_AAA_Residue;
  AAA_residue_data* AAA_Residue_to_update;

  int error = FALSE;

  if (src_AAA_List->nb_AAA_residues!=AAA_List_to_update->nb_AAA_residues)
    error = TRUE;

  while (res<src_AAA_List->nb_AAA_residues && !error) {

    src_AAA_Residue = src_AAA_List->AAA_residueList[res];
    AAA_Residue_to_update = AAA_List_to_update->AAA_residueList[res]; 
 
    if ((src_AAA_Residue->resSeq!=AAA_Residue_to_update->resSeq)
	||(strncmp(src_AAA_Residue->resName,AAA_Residue_to_update->resName, 3)!=0)) {
      error = TRUE;
    }
    else {
      if (src_AAA_Residue->art_bkb!=AAA_Residue_to_update->art_bkb)
	set_Bkb_Art(AAA_Residue_to_update, src_AAA_Residue->art_bkb);
      
      if (src_AAA_Residue->art_sch!=AAA_Residue_to_update->art_sch)
	set_Sch_Art(AAA_Residue_to_update, src_AAA_Residue->art_sch);
    
      res++;
    }
  
  }

  return !error;

}

int read_Art(AAA_residue_data_list* AAA_List, int num, int* bkb, int* sch) {

  AAA_residue_data* AAA_Residue = AAA_List->AAA_residueList[num];
  if (AAA_Residue == NULL)
    return FALSE;

  *bkb = AAA_Residue->art_bkb;
  *sch = AAA_Residue->art_sch;
  
  return TRUE;
  
}

// PSF TO AAA //////////////////////////////////////////////////////////////////

AAA_residue_data_list* AAA_read_psf_protein_struct(psf_protein *proteinPt){

  int chain;
  int res;

  psf_AAchain* AAchainPt;
  psf_residue* residuePt;
  
  AAA_residue_data* AAA_Residue;
  AAA_residue_data_list* AAA_List;

  AAA_List = (AAA_residue_data_list*)malloc(sizeof(AAA_residue_data_list));
  AAA_List->nb_AAA_residues = 0;
  snprintf(AAA_List->proteinName, MAX_PROTEIN_NAME_LENGTH-1, "%s", proteinPt->name);

  for(chain=0; chain<proteinPt->nchains; chain++) {
    AAchainPt = proteinPt->chainList[chain];

    for(res=0; res<AAchainPt->nresidues; res++) {
      residuePt = AAchainPt->resList[res];

      AAA_Residue = create_AAA_Residue(residuePt->resSeq,
				       residuePt->resName,
				       RIGID,
				       RIGID);

      add_AAA_Residue(AAA_List, AAA_Residue);
    }
   
  }

  return AAA_List;
}

/**********************************************************************/

void free_AAA_List(AAA_residue_data_list *AAA_List) {
  aaa_free_pointer_list((void***)&(AAA_List->AAA_residueList), &(AAA_List->nb_AAA_residues));
}

/**********************************************************************/

AAA_protein_data_list* AAA_read_psf_struct(psf_protein** list, int nproteins){

  int protein;
  psf_protein* proteinPt;
  AAA_protein_data_list* AAA_protein_list;
  AAA_residue_data_list* AAA_residue_list;

  AAA_protein_list = (AAA_protein_data_list*)malloc(sizeof(AAA_protein_data_list));
  AAA_protein_list->nb_AAA_protein = 0;

  for(protein=0; protein<nproteins; protein++) {
    proteinPt = list[protein];
    AAA_residue_list = AAA_read_psf_protein_struct(proteinPt);
    aaa_insert_pointer_in_list((void*)AAA_residue_list,
			       (void***)&(AAA_protein_list->AAA_proteinList),
			       &(AAA_protein_list->nb_AAA_protein));
  }

  return AAA_protein_list;
}

/**********************************************************************/

void free_AAA_protein_List(AAA_protein_data_list *AAA_protein_list) {
  
  int i;
  for(i=0; i<AAA_protein_list->nb_AAA_protein; i++) {
    aaa_free_pointer_list((void***)&(AAA_protein_list->AAA_proteinList[i]->AAA_residueList),
			  &(AAA_protein_list->AAA_proteinList[i]->nb_AAA_residues));
  }
  free(AAA_protein_list);
}

/**********************************************************************/

int get_AAA_index_by_protein_Name(AAA_protein_data_list* AAA_protein_list, const char* name, int* index) {
  
  int found = FALSE;
  int i = 0;

  while (i<AAA_protein_list->nb_AAA_protein && (!found)) {
    if (strncmp(AAA_protein_list->AAA_proteinList[i]->proteinName,
		name, MAX_PROTEIN_NAME_LENGTH) == 0) {
      found = TRUE;
    }
    else
      i++;
  }

  if (found)
    *index = i;
  
  return found;

}

/**********************************************************************/

AAA_protein_data_list* copy_AAA_protein_data_list(AAA_protein_data_list* src_AAA_List) {
  
  int i;
  AAA_residue_data_list* AAA_residue_list;
  AAA_protein_data_list* protein_AAA_list = (AAA_protein_data_list*)malloc(sizeof(AAA_protein_data_list));
  protein_AAA_list->nb_AAA_protein = 0;

  for (i=0; i<src_AAA_List->nb_AAA_protein; i++) {
    AAA_residue_list = copy_AAA_residue_data_list(src_AAA_List->AAA_proteinList[i]);
    aaa_insert_pointer_in_list((void*)AAA_residue_list, (void***)&(protein_AAA_list->AAA_proteinList), &(protein_AAA_list->nb_AAA_protein));
  }

  return protein_AAA_list;
}

/**********************************************************************/

int update_AAA_protein_data_list(AAA_protein_data_list* src_AAA_protein_list, AAA_protein_data_list* AAA_protein_list_to_update) {

  int error = FALSE;
  int prot = 0;

  if (src_AAA_protein_list->nb_AAA_protein!=AAA_protein_list_to_update->nb_AAA_protein)
    error = TRUE;
  
  while (prot<src_AAA_protein_list->nb_AAA_protein && !error) {
    error = !(update_AAA_residue_data_list(src_AAA_protein_list->AAA_proteinList[prot], AAA_protein_list_to_update->AAA_proteinList[prot]));
    if (!error)
      prot++;
  }
  
  return !error;

}

/**********************************************************************/

static int read_aaa_line(char *aaaLine, AAA_residue_data** aaa_residue, int* other_line) {

  char piece1[5];
  char piece2[3];
  char name[3];
  
  char resName[4];
  int resSeq;
  int art_bkb = FALSE;
  int art_sch = FALSE;
  residueTypes type;
  int read_OK = FALSE;

  sscanf(aaaLine,"%s",resName); 
  read_OK = resName_to_resType(resName, &type);
  if (read_OK) {

    strcpy(piece1,"    ");
    strncpy(piece1,aaaLine+4,4);

    if(!sscanf(piece1,"%d",&resSeq))
      return FALSE;

    strcpy(piece2,"  ");
    strncpy(piece2,aaaLine+9,2);
    if(!sscanf(piece2,"%s",name))
      return FALSE;
    
    if(strcmp(name,"rb") == 0)
      art_bkb = FALSE;
    else if(strcmp(name,"mb") == 0) 
      art_bkb = TRUE;
    else
      return FALSE;
    
    strcpy(piece2,"  ");
    strncpy(piece2,aaaLine+12,2);
    if(!sscanf(piece2,"%s",name))
      return FALSE;
    
    if(strcmp(name,"rr") == 0)
      art_sch = FALSE;
    else if(strcmp(name,"mr") == 0) 
      art_sch = TRUE;
    else
      return FALSE;

    *aaa_residue = create_AAA_Residue(resSeq, resName, art_bkb, art_sch);
  }
  else {
    *other_line = TRUE;
    *aaa_residue = NULL;
  }

  return TRUE;
}

/**********************************************************************/

int read_aaa_desc_from_file(const char* fullname, AAA_residue_data_list* list) {

  FILE* aaaFile;
  char *read_line;
  char returned_s[50];
  int read_OK = TRUE;
  int end = FALSE;
  int other_line;
  AAA_residue_data* new_aaa_data = NULL;

  aaaFile = fopen(fullname, "r");
  if (aaaFile == NULL) {
    printf("aaa file cannot be open\n");
    return FALSE;
  }

  do {

    other_line = FALSE;    
    read_line = fgets(returned_s,50,aaaFile);
    
    if(read_line == NULL) {
      end = TRUE;
    }
    else {
      
      read_OK = read_aaa_line(read_line, &new_aaa_data, &other_line);

      if (read_OK) {
	if (!other_line)
	  add_AAA_Residue(list, new_aaa_data);
      }
      else {
	free_AAA_List(list);
	list = NULL;
      }
    }
  
  } while (read_OK && !end);

  fclose(aaaFile);
    
  return read_OK;
}

/**********************************************************************/

static void write_aaa_in_file(FILE* aaaFile, AAA_residue_data* aaa_residue) {

  fprintf(aaaFile,"%-3s",aaa_residue->resName);

  fprintf(aaaFile,"%5d",aaa_residue->resSeq);
  
  if (aaa_residue->art_bkb)
    fprintf(aaaFile,"%3s","mb");
  else 
  fprintf(aaaFile,"%3s","rb");
  
  if (aaa_residue->art_sch)
    fprintf(aaaFile,"%3s","mr");
  else 
    fprintf(aaaFile,"%3s","rr");
  
  fprintf(aaaFile,"\n");

}

/**********************************************************************/

int save_aaa_desc_in_file(const char* fullname, AAA_residue_data_list* aaa_list) {

  FILE* aaaFile;
  int aaaIndex;

  aaaFile = fopen(fullname, "w");
  if (aaaFile == NULL) {
    printf("aaa file cannot be open\n");
    return FALSE;
  }

  // write aaa head
  fprintf(aaaFile,"/*\n# ARTICULATED AMINO-ACIDS INPUT FILE\n\n# LINE FORMAT :\nresName resSeq <rb OR mb> <rr or mr>\n\n- rb = rigid backbone\n- mb = mobile backbone\n- rr = rigid radical\n- mr = mobile radical\n\n//////////////\n0        1\n12345678901234\n//////////////\n*/\n\n");
  
  for (aaaIndex=0; aaaIndex<aaa_list->nb_AAA_residues; aaaIndex++) {
    write_aaa_in_file(aaaFile, aaa_list->AAA_residueList[aaaIndex]);
  }

  fclose(aaaFile);
  return TRUE;

}


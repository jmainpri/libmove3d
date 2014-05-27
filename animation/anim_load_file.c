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
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"

#define ANIM_MANAGER_DEBUG 0 
 
/* Ce fichier contient toutes les fonctions relatives a la */
/* lecture des fichiers de captures de mouvements associes */
/* au personnage. Lors de cette etape, le conteneur des captures, */
/* soit la bibliotheque de mouvements est initialisee */
/* Cette structure est commune est unique : XYZ_ANIM et est definie */
/* dans animation.h */

static int XYZ_ANIM_initialized = 0;

static void anim_initialize_XYZ_ANIM_structure (void) {
  if (XYZ_ANIM_initialized != 1) {
    XYZ_ANIM.nof_animation = 0;
    XYZ_ANIM_initialized = 1;
    if (ANIM_MANAGER_DEBUG)
      PrintInfo (("anim_load_file.c -- structure XYZ_ANIM initialisee \n"));
  }
  else {
    if (ANIM_MANAGER_DEBUG)
      PrintInfo (("anim_load_file.c -- structure XYZ_ANIM a ete precedemment initialisee \n"));
  }
}

static p3d_rob * anim_find_robot_pointer_from_name (const char RobotName[255]) 
{
  p3d_rob * RobotPt;
  int RobotIndex;
  
  RobotIndex = p3d_get_rob_nid_by_name((char *)RobotName);
  if (RobotIndex <0) {
    PrintWarning (("anim_load_file -- Le robot %s est associe au
                    fichier de capture de mouvement en cours de lecture. 
                    Ce robot n'existe pas. Le programme est interrompu.
                    \n",
		   RobotName));
    exit(101);
  }
  RobotPt    = XYZ_ENV->robot[RobotIndex];
  if (ANIM_MANAGER_DEBUG)
    PrintInfo (("anim_load_file.c -- %s a ete associe a la lecture courante du fichier\n",RobotPt->name));
  return RobotPt;
}

static FILE * anim_open_mcap_file (const char * FileName) {
  FILE * FilePt;
    if ((FilePt = fopen(FileName,"r"))==NULL) {
      PrintInfo(("anim_load_file.c -- Le fichier %s n'a pu etre
 ouvert. Le programme est interrompu \n",FileName));
      exit (102);
    }
  fseek (FilePt,0,0);
  if (ANIM_MANAGER_DEBUG)
    PrintInfo (("anim_load_file.c -- Fichier %s ouvert\n",FileName));
  return FilePt;
}

static void anim_alloc_data (const p3d_rob * RobotPt,
			     p3d_animation * anim)
{
  int lframe, ldof;
  anim->data = MY_ALLOC(configPt, anim->nof_frames);
  for (lframe=0; lframe < anim->nof_frames; lframe++) {
    anim->data[lframe] = p3d_alloc_config((p3d_rob *)RobotPt);
    for (ldof=0; ldof < RobotPt->nb_dof; ldof++) {
      anim->data[lframe][ldof] = 0.;
    }
  }
}


/* EXTERN FUNCTION */
void anim_load_mcap_file (const char * FileName, 
			  const char RobotName[255])
{
  p3d_rob * RobotPt;
  p3d_animation * AnimDestPt;
  FILE          * FilePt;
  float  FloatRead;
  char   EOFReached, CharRead, WordRead[255];
  int    NofFrames, LFrame, IndexAnim, IntRead, NumJoint, NumDof,
    IndexConfig;

  anim_initialize_XYZ_ANIM_structure();
  RobotPt = anim_find_robot_pointer_from_name(RobotName);

  /* INITIALISATIONS & ALLOCATIONS*/
  IndexAnim  = XYZ_ANIM.nof_animation;

  XYZ_ANIM.animation[IndexAnim] = MY_ALLOC(p3d_animation,1);
  XYZ_ANIM.nof_animation++;
  
  AnimDestPt = XYZ_ANIM.animation[IndexAnim];
  AnimDestPt->aim_robot = RobotPt;
  AnimDestPt->id = IndexAnim;
  AnimDestPt->active = 1;
  AnimDestPt->degree = 0;
  AnimDestPt->straight = 0;
  AnimDestPt->processed = 0;
  strcpy(AnimDestPt->name, FileName);

  FilePt = anim_open_mcap_file(FileName);

  EOFReached = 0;
  NumJoint = -1;

  while (EOFReached != 1) {
    CharRead = '#';		
    while (CharRead != ':' && EOFReached != 1) {
      CharRead = fgetc(FilePt);	
      if (CharRead == EOF) EOFReached = 1;
    }
    WordRead[0] = '\0';
    if (EOFReached != 1) fscanf (FilePt, "%s", WordRead);
    
    if (strcmp (WordRead, "DEGREE") == 0) {
      AnimDestPt->degree = 1;
    }
    else if (strcmp (WordRead, "STRAIGHT") == 0) {
      AnimDestPt->straight = 1;
    }
    else if (strcmp (WordRead, "TURNING") == 0) {
      AnimDestPt->straight = 0;
    }
    else if (strcmp (WordRead, "TYPE") == 0) {
      fscanf (FilePt, "%s", WordRead);
      strcpy (AnimDestPt->type, WordRead);
    }
    else if (strcmp (WordRead, "WHEN") == 0) {
      fscanf (FilePt, "%s", WordRead);
      strcpy (AnimDestPt->when, WordRead);
    }
    else if (strcmp (WordRead, "FRAMES") == 0) {
      fscanf(FilePt, "%d", &NofFrames);
      AnimDestPt->nof_frames = NofFrames;
      anim_alloc_data(RobotPt, AnimDestPt);
    }
    
    else if (strcmp (WordRead, "JOINT") == 0) {
      fscanf (FilePt, "%d", &IntRead);
      NumJoint = IntRead;
    }

    else if (strcmp (WordRead, "INDEXTHETA") == 0) {
      fscanf (FilePt, "%d", &IntRead);
      AnimDestPt->index_theta = IntRead;
    }    

    else if (strcmp (WordRead, "FRAMERATE") == 0) {
      fscanf (FilePt, "%d", &IntRead);
      AnimDestPt->framerate = IntRead;
    }

    else if (strcmp (WordRead, "NOFCOEF") == 0) {
      fscanf (FilePt, "%d", &IntRead);
      AnimDestPt->nof_coef = IntRead;
    }
    
    else if (strcmp (WordRead, "DOF") == 0) {
      fscanf (FilePt, "%d", &IntRead);
      NumDof      = IntRead;
      IndexConfig = RobotPt->joints[NumJoint]->index_dof + NumDof;
      for (LFrame = 0; LFrame < NofFrames; LFrame++) {
	fscanf (FilePt, "%f", &FloatRead);
	AnimDestPt->data[LFrame][IndexConfig] = (double)FloatRead;
      }
    }
  }
  fclose(FilePt);
}

int anim_load_mldef(const char * FileName,
		    const char * RobotName) {
  FILE * FilePt;
  char FileToLoad[1024];
  int FileCount = 0, ScanResult;

  FilePt = anim_open_mcap_file(FileName);
  ScanResult = fscanf (FilePt,"%s", FileToLoad);
  while (ScanResult == 1) {
    anim_load_mcap_file(FileToLoad, RobotName);
    FileCount++;
    ScanResult = fscanf (FilePt,"%s", FileToLoad);
  }
  fclose(FilePt);
  return FileCount;
}

void anim_unload_file(int AnimId) {
  p3d_animation  * AnimPt = XYZ_ANIM.animation[AnimId];
  int LFrame, LAnim;

  for (LFrame = 0; LFrame < AnimPt->nof_frames; LFrame ++) {
    p3d_destroy_config(AnimPt->aim_robot,AnimPt->data[LFrame]);
  }
  MY_FREE(AnimPt->data, configPt*, AnimPt->nof_frames);

  if (AnimPt->processed) {
    for (LFrame = 0; LFrame < AnimPt->nof_coef; LFrame ++) {
      MY_FREE(AnimPt->fourier_real[LFrame], double, AnimPt->aim_robot->nb_dof);
      MY_FREE(AnimPt->fourier_imag[LFrame], double, AnimPt->aim_robot->nb_dof);
    }
    MY_FREE(AnimPt->fourier_real, double*, AnimPt->nof_coef);
    MY_FREE(AnimPt->fourier_imag, double*, AnimPt->nof_coef);
  }
  MY_FREE(AnimPt, p3d_animation, 1);
  for (LAnim = AnimId ; LAnim < XYZ_ANIM.nof_animation -1; LAnim++) {
    XYZ_ANIM.animation[LAnim] = XYZ_ANIM.animation[LAnim +1];
  }
  XYZ_ANIM.nof_animation--;
}

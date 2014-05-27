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

#define ANIM_REACTIVE_DEBUG 0

p3d_joint_list *  Chain[5] = {NULL,NULL,NULL,NULL,NULL};

static void anim_add_reactive_dof(int NumJoint, int NumChain, p3d_rob * RobotPt) 
{
  p3d_joint_list * JointList;
  JointList = MY_ALLOC(p3d_joint_list, 1);
  JointList->NumJoint = NumJoint;
  JointList->Joint    = RobotPt->joints[NumJoint];
  JointList->next     = Chain[NumChain];
  Chain[NumChain]     = JointList;
}

static FILE * anim_open_definition_file (const char * FileName) {
  FILE * FilePt;
  if ((FilePt = fopen(FileName,"r"))==NULL) {
    PrintInfo(("anim_reactive.c -- Le fichier %s n'a pu etre
 ouvert. Le programme est interrompu \n",FileName));
    exit (112);
  }
  fseek (FilePt,0,0);
  if (ANIM_REACTIVE_DEBUG)
    PrintInfo (("anim_load_file.c -- Fichier %s ouvert\n",FileName));
  return FilePt;
}

int anim_joint_is_reactive(int NumJoint) {
  int LChain;
  p3d_joint_list * JointListScan;

  for (LChain = 0; LChain < 5; LChain ++) {
    JointListScan = Chain[LChain];
    while (JointListScan != NULL) {
      if (JointListScan->NumJoint == NumJoint) return LChain;
      JointListScan = JointListScan->next;
    }
  }
  return -1;
}

int anim_reactive_is_initialized(void) {
  if (Chain[0] == NULL) return 0;
  else return 1;
}

p3d_joint_list * anim_reactive_get_pointer_on_reactive_chain(int NumChain) {
  p3d_joint_list * Result;
  Result = Chain[NumChain];
  return Result;
}

void anim_reactive_definition_file(const char * FileName, p3d_rob * RobotPt)
{
  FILE * FilePt;
  char   EOFReached, CharRead, WordRead[255];
  int IntRead;
  int NumChain, NofElement, LElement;
  
  FilePt = anim_open_definition_file(FileName);

  EOFReached = 0;

  while (EOFReached != 1) {
    CharRead = '#';		
    while (CharRead != ':' && EOFReached != 1) {
      CharRead = fgetc(FilePt);	
      if (CharRead == EOF) EOFReached = 1;
    }
    WordRead[0] = '\0';
    if (EOFReached != 1) fscanf (FilePt, "%s", WordRead);

    if (strcmp (WordRead, "CHAIN") == 0) {
      fscanf(FilePt,"%d",&IntRead);
      NumChain = IntRead;
      fscanf(FilePt,"%d",&IntRead);
      NofElement = IntRead;
      for (LElement = 0; LElement< NofElement; LElement++) {
	fscanf(FilePt,"%d",&IntRead);
	anim_add_reactive_dof(IntRead,NumChain,RobotPt);
      }
    }
  }
  fclose(FilePt);
}

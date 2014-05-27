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
#include "Graphic-pkg.h"
#include "Collision-pkg.h"
#include "Move3d-pkg.h"

#define COLDEBUG 0

static double my_p3d_random(double a, double b)
{
  double v;
  v =  rand()/((double)RAND_MAX+1); /* nombre aleatoire [0.,1.] */
  v = (b-a)*v + a;
  return(v);
}

static int anim_col_test_joint (p3d_rob * robot, configPt ConfPt, double * distances)
{
  int LJoint = 0, LPoly;
  int Consistent = TRUE;
  
  p3d_set_and_update_this_robot_conf (robot, ConfPt);

  if (COLDEBUG) {
    fl_check_forms();
    g3d_draw_allwin_active();
  }

  p3d_col_env_switch_robot(robot, P3D_COL_ROBOT_ENV);
  if (p3d_col_test()) {

    p3d_col_report_distance(robot,distances);
    Consistent = FALSE;
    while (Consistent == FALSE) {
      LJoint++;
      Consistent = FALSE;
      if (robot->joints[LJoint]->o != NULL)
	if (!(distances[LJoint] < P3D_HUGE))
	  for (LPoly = 0; LPoly < robot->joints[LJoint]->o->np ; LPoly ++)
	    if (robot->joints[LJoint]->o->pol[LPoly]->TYPE != P3D_GRAPHIC)
	      Consistent = TRUE;
      
      if (LJoint > robot->njoints) {
	p3d_col_env_restore();
	return FRAME_INVAL;
      }
    }
    p3d_col_env_restore();
    return LJoint;
  }

  p3d_col_env_restore();
  p3d_col_env_switch_robot(robot, P3D_COL_ROBOT_AUTOCOL);
  if (p3d_col_test()) {
    p3d_col_env_restore();
    return FRAME_AUTOCOL;
  }
  p3d_col_env_restore();
  return FRAME_NOCOL;
}

static int anim_col_test (p3d_rob * robot, configPt ConfPt, double * distances) {
  int ColTestResult, ReactiveResult;
  
  ColTestResult = anim_col_test_joint(robot, ConfPt, distances);
  if (ColTestResult < 0) return ColTestResult;
  ReactiveResult = anim_joint_is_reactive(ColTestResult);
  if (ReactiveResult < 0) {
    return FRAME_INVAL;
  }
  else {
    return ReactiveResult;
  }
}

static p3d_frame_list * anim_col_exhaustive_test (AnimProb * ZANIM) {
  int CollisionDetected, lframe;
  double *distances;
  p3d_rob * robot = ZANIM->Robot;
  p3d_frame_list * FrameList = NULL;
  p3d_frame_list * NewFrameInList = NULL;
  
  distances = MY_ALLOC(double, robot->njoints+1);
  for (lframe = ZANIM->WalkContRes.NofFrames -1; lframe >= 0; lframe --) {
    CollisionDetected = anim_col_test(robot, ZANIM->WalkContRes.AnimBuffer[lframe], distances);
    if (CollisionDetected == FRAME_INVAL) {
      PrintWarning (("AN UNSOLVABLE COLLISION HAS BEEN FOUND. THIS MOTION PLANNER IS STUPID\n"));
    }
    else if (CollisionDetected > 0) {
      if (COLDEBUG)      printf ("Collision on frame %d\n",lframe);
      if (COLDEBUG)      printf ("Chain %d invalid\n",CollisionDetected);
      NewFrameInList = MY_ALLOC(p3d_frame_list, 1);
      NewFrameInList->next = FrameList;
      NewFrameInList->NumFrame = lframe;
      NewFrameInList->NumChain = CollisionDetected;
      FrameList = NewFrameInList;
    }
  }
  MY_FREE(distances, double, robot->njoints+1);
  return FrameList;
}

static int anim_col_chain_listed (anim_colblock Block, int Chain) 
{
  int LChain;
  for (LChain = 0; LChain <Block.NofChain; LChain ++) {
    if (Chain == Block.ColChain[LChain]) return LChain;
  }
  return -1;
}

static void anim_col_constitute_blocks(AnimProb * ZANIM, p3d_frame_list * FrameList) {
  p3d_frame_list * FrameScan;
  anim_colblock * CurrentBlock;
  int ExtZone = (int)(ZANIM->ColOptions.ColBlockExtTime * ZANIM->SampleOptions.FramePerSecond);
  int NofFrames = ZANIM->WalkContRes.NofFrames;
  
  FrameScan = FrameList;
  ZANIM->ColOptions.NofBlocks = 1;
  CurrentBlock = &(ZANIM->ColOptions.Blocks[0]);
  
  CurrentBlock->InitFrame = FrameScan->NumFrame - ExtZone;
  CurrentBlock->EndFrame  = FrameScan->NumFrame + ExtZone;
  CurrentBlock->ColChain[0]  = FrameScan->NumChain;
  CurrentBlock->NofChain     = 1;

  if (CurrentBlock->InitFrame < 0)
    CurrentBlock->InitFrame = 0;
  if (CurrentBlock->EndFrame > NofFrames) 
    CurrentBlock->EndFrame  = NofFrames;
  
  while (FrameScan) {
    if (FrameScan->NumFrame < CurrentBlock->EndFrame) {
      CurrentBlock->EndFrame = FrameScan->NumFrame + ExtZone;
      if (CurrentBlock->EndFrame > NofFrames) CurrentBlock->EndFrame = NofFrames;
      if (anim_col_chain_listed(*CurrentBlock, FrameScan->NumChain) == -1) {
	CurrentBlock->ColChain[CurrentBlock->NofChain] = FrameScan->NumChain;
	CurrentBlock->NofChain ++;
      }
    }
    else {
      CurrentBlock = &(ZANIM->ColOptions.Blocks[ZANIM->ColOptions.NofBlocks]);
      ZANIM->ColOptions.NofBlocks ++;
      CurrentBlock->InitFrame = FrameScan->NumFrame -ExtZone;
      CurrentBlock->EndFrame  = FrameScan->NumFrame +ExtZone;
      CurrentBlock->ColChain[0]  = FrameScan->NumChain;
      CurrentBlock->NofChain     = 1;
      if (CurrentBlock->InitFrame < 0) CurrentBlock->InitFrame = 0;
      if (CurrentBlock->EndFrame > ZANIM->WalkContRes.NofFrames) CurrentBlock->EndFrame  = ZANIM->WalkContRes.NofFrames;
    }
    FrameScan = FrameScan->next;
  }
}

static double anim_col_compute_influence (AnimProb * ZANIM, anim_colblock Block, int Frame, int ExtZone)
{
  double Influence;
  Influence = 1.;
  if ((Frame < (Block.InitFrame + ExtZone)) && (Block.InitFrame > 0))
    Influence = (double)(Frame - Block.InitFrame) / (double)ExtZone;
  if ((Frame > (Block.EndFrame - ExtZone)) && (Block.EndFrame < ZANIM->WalkContRes.NofFrames))
    Influence = (double)(Block.EndFrame - Frame) / (double)ExtZone;
  return Influence;
}

static configPt anim_col_solution(AnimProb * ZANIM, int NumBlock) {
  p3d_joint_list * JointScan;
  p3d_rob * robot = ZANIM->Robot;
  int 
    StillInCollision, 
    CollisionDetected,
    ChSel, 
    NofAttempt, 
    lframe, 
    lchain, 
    lchain2,
    when,
    countframe,
    ExtZone = (int)(ZANIM->ColOptions.ColBlockExtTime * ZANIM->SampleOptions.FramePerSecond);
  double 
    vmin, 
    vmax, 
    * distances, 
    Influence;
  p3d_jnt * JntPt;
  configPt 
    NewSol, 
    solution;
  anim_colblock Block = ZANIM->ColOptions.Blocks[NumBlock];

  distances = MY_ALLOC(double, robot->njoints+1);
  solution  = p3d_alloc_config(robot);
  
  for (lchain = 0; lchain < Block.NofChain; lchain ++) {
    if (COLDEBUG)    printf ("Searching solution for chain %d\n",Block.ColChain[lchain]);
    
    StillInCollision = TRUE;
    NofAttempt = 0;
    
    while (StillInCollision && NofAttempt < ZANIM->ColOptions.NofAttempts) {
      NofAttempt ++;
      StillInCollision = FALSE;
      
      /* FIND A SOLUTION FOR EACH BLOCK */
      
      ChSel = Block.ColChain[lchain];
      JointScan = anim_reactive_get_pointer_on_reactive_chain(ChSel);
      /* Propose another solution for this chain */
      while (JointScan) {
	JntPt = JointScan->Joint;
	p3d_jnt_get_dof_rand_bounds(JntPt, 0, &vmin, &vmax);
	solution[JntPt->index_dof] = my_p3d_random(vmin, vmax);
	JointScan = JointScan->next;
      }
      
      /* TEST SOLUTION */
      
      lframe = (Block.InitFrame+Block.EndFrame)/2. -1;

      for (countframe = 0; countframe < (Block.EndFrame - Block.InitFrame); countframe ++) {
	lframe ++;
	if (lframe >= Block.EndFrame) lframe = Block.InitFrame;
	NewSol = p3d_copy_config(robot, ZANIM->WalkContRes.AnimBuffer[lframe]);
	Influence = anim_col_compute_influence(ZANIM, Block, lframe, ExtZone);
	for (lchain2 = 0; lchain2 <= lchain; lchain2 ++) {
	  ChSel = Block.ColChain[lchain2];
	  JointScan = anim_reactive_get_pointer_on_reactive_chain(ChSel);
	  /* Propose another solution for this chain */
	  while (JointScan) {
	    JntPt = JointScan->Joint;
	    NewSol[JntPt->index_dof] = NewSol[JntPt->index_dof] + Influence * diff_angle(NewSol[JntPt->index_dof], solution[JntPt->index_dof]);
	    JointScan = JointScan->next;
	  }
	}
	CollisionDetected = anim_col_test(robot,NewSol,distances);
	if (COLDEBUG)	printf ("Testing solution for frame %d -> COL = %d\n",lframe, CollisionDetected);
	if (CollisionDetected == FRAME_INVAL) {
	  if (COLDEBUG)	  printf ("This case should not happen \n");
	  StillInCollision = TRUE;
	  break;
	}

	if (CollisionDetected == Block.ColChain[lchain]) {
	  StillInCollision = TRUE;
	  break;
	}
	
	else if (CollisionDetected == FRAME_AUTOCOL) {
	  if (COLDEBUG)	  printf ("Autocollision for frame %d \n", lframe);
	  StillInCollision = TRUE;
	  break;
	}
	
	else if (CollisionDetected > 0) {
	  if (COLDEBUG)	  printf ("Solution Invalid for frame %d -> %d\n", lframe, CollisionDetected);
	  if (anim_col_chain_listed(Block, CollisionDetected)==-1) {
	    if (COLDEBUG)	    printf ("Add %d To chain list\n", CollisionDetected);
	    Block.ColChain[Block.NofChain] = CollisionDetected;
	    Block.NofChain ++;
	    ZANIM->ColOptions.Blocks[NumBlock] = Block;
	  }
	  else {
	    when = anim_col_chain_listed(Block, CollisionDetected);
	    if (when < lchain) {
	      if (COLDEBUG)	      printf ("Chain %d Need to be retested\n", CollisionDetected);
	      Block.ColChain[Block.NofChain] = CollisionDetected;
	      Block.NofChain ++;
	      ZANIM->ColOptions.Blocks[NumBlock] = Block;
	    }
	  }
	}
	p3d_destroy_config(robot, NewSol);
      }
    }
    if (COLDEBUG)    printf ("Solution found in %d attempts\n ",NofAttempt);
    if (NofAttempt >= ZANIM->ColOptions.NofAttempts) break;    
  }
  
  MY_FREE(distances, double, robot->njoints+1);
  
  if (NofAttempt < ZANIM->ColOptions.NofAttempts) {
    return solution;
  }

  else {
  if (COLDEBUG)    printf ("No solution for the block %d \n", NumBlock);
    p3d_destroy_config(robot,solution);
    anim_utils_destroy_buffer(robot, &(ZANIM->ColAvoidRes));
    return NULL;
  }
}

void anim_col_copy_solution(AnimProb * ZANIM, configPt solution, int NumBlock)
{
  int lframe, lchain, ChSel, 
    ExtZone = (int)(ZANIM->ColOptions.ColBlockExtTime * ZANIM->SampleOptions.FramePerSecond);
  double Influence;
  p3d_joint_list * JointScan;
  p3d_jnt * JntPt;
  anim_colblock Block = ZANIM->ColOptions.Blocks[NumBlock];
    
  for (lframe=Block.InitFrame; lframe<Block.EndFrame; lframe++) {
    
    Influence = anim_col_compute_influence(ZANIM, Block, lframe, ExtZone);
    
    if (COLDEBUG) printf ("frame %d, Influence = %3.3f \n",lframe, Influence);
    
    for (lchain = 0; lchain < Block.NofChain; lchain ++) {
      ChSel = Block.ColChain[lchain];
      JointScan = anim_reactive_get_pointer_on_reactive_chain(ChSel);
      while (JointScan) {
	JntPt = JointScan->Joint;
	ZANIM->ColAvoidRes.AnimBuffer[lframe][JntPt->index_dof] = 
	  ZANIM->WalkContRes.AnimBuffer[lframe][JntPt->index_dof] 
	  + Influence * diff_angle(ZANIM->WalkContRes.AnimBuffer[lframe][JntPt->index_dof], solution[JntPt->index_dof]);
	JointScan = JointScan->next;
      }
    }
  }
}

static double anim_col_optim_solution1(AnimProb * ZANIM, anim_colblock Block)
{
  double Ratio, * distances, Step = 0.1;
  int ChSel, lchain, lframe, Collides;
  p3d_joint_list * JointScan;
  p3d_jnt * JntPt;
  p3d_rob * Robot = ZANIM->Robot;
  configPt CurrentConfig;

  Collides = TRUE;
  Ratio = Step;
  
  distances = MY_ALLOC(double, Robot->njoints +1);
  
  while (Collides && Ratio < 1.) {
    Collides = FALSE;
    
    for (lframe = Block.InitFrame; lframe<Block.EndFrame; lframe++) {
      p3d_destroy_config (Robot, ZANIM->ColOpt1Res.AnimBuffer[lframe]);
      ZANIM->ColOpt1Res.AnimBuffer[lframe] = p3d_copy_config(Robot,ZANIM->WalkContRes.AnimBuffer[lframe]);
      CurrentConfig = ZANIM->ColOpt1Res.AnimBuffer[lframe];
      
      for (lchain = 0; lchain < Block.NofChain; lchain ++) {	
	ChSel = Block.ColChain[lchain];
	JointScan = anim_reactive_get_pointer_on_reactive_chain(ChSel);
	while (JointScan) {
	  JntPt = JointScan->Joint;
	  CurrentConfig[JntPt->index_dof] = 
	    CurrentConfig[JntPt->index_dof]
	    + Ratio * diff_angle(CurrentConfig[JntPt->index_dof], 
				 ZANIM->ColAvoidRes.AnimBuffer[lframe][JntPt->index_dof]);
	  JointScan = JointScan->next;
	}
      }
      if (anim_col_test(Robot, CurrentConfig, distances) > 0) {
	Collides = TRUE;
	break;
      }
    }
    Ratio += Step;
    if (COLDEBUG)    printf ("Ratio = %3.3f\n",Ratio);
  }
  
  MY_FREE(distances, double, Robot->njoints+1);
  if (Ratio > 1.) Ratio = 1.;
  return Ratio;
}

static void anim_col_optim_solution3(AnimProb * ZANIM, anim_colblock Block) {
  
  int FrameChanged, ntest, lframe;
  p3d_localpath * LinLPPt;
  double ecart, EcartMax = 1. / ZANIM->SampleOptions.FramePerSecond, param1;
  configPt CurrentConfig;
  p3d_rob * Robot = ZANIM->Robot;
  double Params[MAX_NOF_FRAMES];
  configPt q; // Modif NIC pour cvs juan

  q = p3d_alloc_config(Robot); // Modif NIC pour cvs juan

  p3d_col_env_switch_robot(Robot, P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL);
  for (lframe = Block.InitFrame; lframe < Block.EndFrame; lframe ++) {
    LinLPPt = p3d_linear_localplanner (Robot, ZANIM->ColOpt1Res.AnimBuffer[lframe], ZANIM->WalkContRes.AnimBuffer[lframe]);
    p3d_col_test_localpath_classic(Robot, LinLPPt, &ntest, &param1,&q); // Modif NIC pour cvs juan
    if (LinLPPt)
      Params[lframe] = param1;
    else
      Params[lframe] = 1.0;
    
    if (LinLPPt) {
      if (COLDEBUG)      printf ("param1 : %3.3f - range_param : %3.3f \n",param1, LinLPPt->range_param);
    }
    else 
      if (COLDEBUG)      printf ("No LPPT\n");

    p3d_lin_destroy(Robot, LinLPPt);
  }
  
  FrameChanged = TRUE;
  while (FrameChanged) {
    FrameChanged = FALSE;
    for (lframe=Block.InitFrame; lframe<(Block.EndFrame-1); lframe++) {  
      ecart = fabs(Params[lframe+1] -  Params[lframe]);
      if (ecart > (EcartMax + EPS)) {
	FrameChanged = TRUE;
	if (Params[lframe+1] > Params[lframe])
	  Params[lframe+1] = Params[lframe] + EcartMax;
	else
	  Params[lframe] = Params[lframe+1] + EcartMax;
      }
    }
  }

  p3d_col_env_restore();

  for (lframe = Block.InitFrame; lframe < Block.EndFrame; lframe ++) {
    LinLPPt = p3d_linear_localplanner (Robot, ZANIM->ColOpt1Res.AnimBuffer[lframe], ZANIM->WalkContRes.AnimBuffer[lframe]);
    if (LinLPPt) {
      p3d_destroy_config (Robot, ZANIM->ColOpt2Res.AnimBuffer[lframe]);
      param1 = Params[lframe] * LinLPPt->range_param;
      ZANIM->ColOpt2Res.AnimBuffer[lframe] = LinLPPt->config_at_param(Robot, LinLPPt, param1);
      p3d_lin_destroy(Robot,LinLPPt);
    }
    CurrentConfig = ZANIM->ColOpt1Res.AnimBuffer[lframe];
  }
}

void anim_col_avoidance_step1(AnimProb * ZANIM) {
  p3d_frame_list * FrameList, * NewFrameInList;

  FrameList = anim_col_exhaustive_test(ZANIM);

  if (FrameList == NULL) {
    return;
  }
  
  anim_col_constitute_blocks(ZANIM, FrameList);

  NewFrameInList = FrameList;
  while (NewFrameInList) {
    FrameList = NewFrameInList;
    NewFrameInList = NewFrameInList->next;
    MY_FREE (FrameList, p3d_frame_list, 1);
  }
}

void anim_col_avoidance_step2(AnimProb * ZANIM) {
  int LBlock;
  configPt solution;

  anim_utils_copy_buffer (ZANIM->Robot, &(ZANIM->WalkContRes), &(ZANIM->ColAvoidRes));
    
  for (LBlock = 0; LBlock < ZANIM->ColOptions.NofBlocks; LBlock ++) {
    solution = anim_col_solution(ZANIM, LBlock);
    if (solution) {
      anim_col_copy_solution(ZANIM, solution, LBlock);
      p3d_destroy_config(ZANIM->Robot, solution);
    }
    else {
      if (COLDEBUG) printf ("No solution for block %d \n",LBlock);
      anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes));
    }
  }
}

void anim_col_avoidance_step3(AnimProb * ZANIM) {
  int LBlock;
  
  anim_utils_copy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes), &(ZANIM->ColOpt1Res));
  anim_utils_copy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes), &(ZANIM->ColOpt2Res));
  for (LBlock = 0; LBlock < ZANIM->ColOptions.NofBlocks; LBlock++) {
    anim_col_optim_solution1(ZANIM, ZANIM->ColOptions.Blocks[LBlock]);
  }
  for (LBlock = 0; LBlock < ZANIM->ColOptions.NofBlocks; LBlock++) {
    anim_col_optim_solution3(ZANIM, ZANIM->ColOptions.Blocks[LBlock]);
  }
}

void anim_col_initialize(AnimProb * ZANIM) {
  ZANIM->ColOptions.NofBlocks = 0;
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt1Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt2Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->PosPlanRes));
}

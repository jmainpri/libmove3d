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

#define CHARAC_DEBUG 1

void anim_charac_traj(AnimProb * ZANIM)
{
  int lframe, nofframes;
  double OldX, OldY, OldTheta, X, Y, Theta;
  double SupposedX, SupposedY, frameduration = ZANIM->FrameDuration;
  p3d_jnt * Jnt1Pt;
  p3d_rob * robot;

  if (ZANIM->SamplingRes.NofFrames < 1) {
    PrintWarning(("No Sampling Result Found\n"));
    return;
  }
  
  robot = ZANIM->Robot;
  nofframes = ZANIM->SamplingRes.NofFrames;
  /* -- STRUCTURE ALLOCATION */
  
  Jnt1Pt = robot->joints[1];
  /* -- FILL IN THE STRUCTURE, COMPUTE ALSO MAX DISTANCE COVERED IN ONE FRAME */

  ZANIM->CharacRes.TotalDist[0]    = 0.;
  ZANIM->CharacRes.FrameDist[0]    = 0.;
  ZANIM->CharacRes.WalkRelSpeed[0] = 0.;
  ZANIM->CharacRes.LinSpeed[0]     = 0.;
  ZANIM->CharacRes.RotSpeed[0]     = 0.;

  OldX     = ZANIM->Qi[MI_X];
  OldY     = ZANIM->Qi[MI_Y];
  OldTheta = ZANIM->Qi[MI_GRZ];
  
  for (lframe = 1; lframe < nofframes; lframe ++) {
    
    X = ZANIM->SamplingRes.AnimBuffer[lframe][MI_X];
    Y = ZANIM->SamplingRes.AnimBuffer[lframe][MI_Y];
    Theta = ZANIM->SamplingRes.AnimBuffer[lframe][MI_GRZ];

    ZANIM->CharacRes.FrameDist[lframe]    = euclidian_dist(X, Y, 0., OldX, OldY, 0.);
    ZANIM->CharacRes.TotalDist[lframe]    = ZANIM->CharacRes.TotalDist[lframe-1] + ZANIM->CharacRes.FrameDist[lframe];
    ZANIM->CharacRes.LinSpeed[lframe]     = ZANIM->CharacRes.FrameDist[lframe] / frameduration;
    ZANIM->CharacRes.RotSpeed[lframe]     = diff_angle(OldTheta, Theta) / frameduration;

    /* Sens */
    SupposedX = OldX + ZANIM->CharacRes.FrameDist[lframe] * cos(Theta);
    SupposedY = OldY + ZANIM->CharacRes.FrameDist[lframe] * sin(Theta);
    if (euclidian_dist(X, Y, 0., SupposedX, SupposedY, 0.) > ZANIM->CharacRes.FrameDist[lframe]) 
      ZANIM->CharacRes.Sens[lframe] = -1.;
    else
      ZANIM->CharacRes.Sens[lframe] = 1.;

    OldX = X;
    OldY = Y;
    OldTheta = Theta;  
    if (CHARAC_DEBUG) {
      if (lframe > 0) ;
      printf ("%d X %3.3f Y %3.3f LS %3.3f RS %3.3f Acc %3.3f\n", 
	      lframe,
	      X, 
	      Y,
	      ZANIM->CharacRes.LinSpeed[lframe],
	      ZANIM->CharacRes.RotSpeed[lframe],
	      ( ZANIM->CharacRes.LinSpeed[lframe] - ZANIM->CharacRes.LinSpeed[lframe-1] )/ ZANIM->FrameDuration);
      
    }
  }
  
  ZANIM->CharacRes.TotalDist[0]    = 0.;
  ZANIM->CharacRes.FrameDist[0]    = ZANIM->CharacRes.FrameDist[1];
  ZANIM->CharacRes.LinSpeed[0]     = ZANIM->CharacRes.LinSpeed[1];
  ZANIM->CharacRes.RotSpeed[0]     = ZANIM->CharacRes.RotSpeed[1];
  ZANIM->TrajCharacterized = 1;
}

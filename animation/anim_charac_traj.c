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

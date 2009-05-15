#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Planner-pkg.h"

#define MAX_NOF_LOC 100

#define SAMPLE_DEBUG 0

#define SAMPLE_PARAM_OK 0
#define SAMPLE_PARAM_OVER -1
#define SAMPLE_PARAM_UNDER -2
#define SAMPLE_PARAM_REVERSED -3

double ANIM_SAMPLE[MAX_NOF_FRAMES];

static int Variation(p3d_traj * traj, double Param1, double Param2, double * dL, double *dTheta) {
  configPt Q;
  double 
    X, Y, Theta, 
    OldX, OldY, OldTheta,
    IntX, IntY, IntTheta;
  int Result;

  Result = SAMPLE_PARAM_OK;

  if (Param1 > Param2) Result = SAMPLE_PARAM_REVERSED;
  
  if (Param2 > traj->range_param - EPS) {
    Result = SAMPLE_PARAM_OVER;
  }
  
  if (Param2 < EPS) {
    Result =  SAMPLE_PARAM_UNDER;
  }
  
  Q  = p3d_config_at_param_along_traj(traj, Param1);
  OldX     = Q[MI_X];
  OldY     = Q[MI_Y];
  OldTheta = Q[MI_GRZ];
  p3d_destroy_config(traj->rob, Q);

  Q  = p3d_config_at_param_along_traj(traj, (Param1 + Param2) / 2.);
  IntX     = Q[MI_X];
  IntY     = Q[MI_Y];
  IntTheta = Q[MI_GRZ];
  p3d_destroy_config(traj->rob, Q);

  Q = p3d_config_at_param_along_traj(traj, Param2);
  X     = Q[MI_X];
  Y     = Q[MI_Y];
  Theta = Q[MI_GRZ];
  p3d_destroy_config(traj->rob, Q);
  
  *dL = euclidian_dist(IntX, IntY, 0., OldX, OldY, 0.) + euclidian_dist(IntX, IntY, 0., X, Y, 0.);
  *dTheta = fabs(diff_angle(IntTheta, OldTheta)) + fabs(diff_angle(IntTheta, Theta));

  return Result;
}

static void anim_sample_param_for_vmax (p3d_traj * traj, int Frame, double FrameDuration, double MaxLinSpeed, double MaxAcc) {
  double MaxSpeed, Speed;
  double Ecart;
  double Tolerance = 0.001;
  double dL, dTheta;
  int VariationResult;
  int DeadLockProtection = 0;
  
  /* Special case, sample has reached the end of the path */
  if (ANIM_SAMPLE[Frame] > traj->range_param - EPS) {
    ANIM_SAMPLE[Frame] = traj->range_param;
    return;
  }
  
  /* Compute Previous Speed */
  if (Frame < 2) {
    /* If at the beginning previous speed is zero */
    dL = 0.;
  }

  else {
    VariationResult = Variation(traj, ANIM_SAMPLE[Frame-2], ANIM_SAMPLE[Frame-1], &dL, &dTheta);
    if (VariationResult != SAMPLE_PARAM_OK) printf ("problem while sampling \n");
  }
  
  /* Compute Max Speed to adopt */
  MaxSpeed = dL / FrameDuration + MaxAcc * FrameDuration;

  if (MaxSpeed > MaxLinSpeed)
    MaxSpeed = MaxLinSpeed;

  /* Verify that problem is feasible */
  VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], traj->range_param, &dL, &dTheta);
  Speed = dL/FrameDuration;
  if (Speed < MaxSpeed) {
    ANIM_SAMPLE[Frame] = traj->range_param;
    return;
  }
  
  /* Compute Actual Speed */
  VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], ANIM_SAMPLE[Frame], &dL, &dTheta);
  Speed = dL/FrameDuration;
  
  Ecart = (ANIM_SAMPLE[Frame] - ANIM_SAMPLE[Frame -1]) / 2.;
  
  while ((fabs(MaxSpeed - Speed) / MaxSpeed) > Tolerance) {
    
    DeadLockProtection ++;
    
    if (VariationResult == SAMPLE_PARAM_OVER) /* One of the param is < 0. */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] - Ecart;
    else if (VariationResult == SAMPLE_PARAM_UNDER) /* One of the param is > range_param */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    else if (VariationResult == SAMPLE_PARAM_REVERSED) /* There is a backward walk */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    else if (Speed > MaxSpeed)	/* Speed is over the limit */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] - Ecart;
    else			/* Speed is under the limit */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    
    VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], ANIM_SAMPLE[Frame], &dL, &dTheta);
    Speed = dL/FrameDuration;	/* New speed */
    Ecart = Ecart/2.;
    
    if (DeadLockProtection > 50) return;
  }
  
  if (SAMPLE_DEBUG) printf("VITESSE : redefinition de %d \n",Frame);
}

static void anim_sample_param_for_specified_vmax (p3d_traj * traj, int Frame, double FrameDuration, double SpecifiedSpeed) {
  double MaxSpeed, Speed;
  double Ecart;
  double Tolerance = 0.001;
  double dL, dTheta;
  int VariationResult, DeadLockProtection = 0;
  
  MaxSpeed = SpecifiedSpeed;
  if (MaxSpeed < EPS) {
    ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame -1];
    return;
  }

  /* Verify that problem is feasible */
  VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], traj->range_param, &dL, &dTheta);
  Speed = dL/FrameDuration;
  
  if (Speed < MaxSpeed) {
    ANIM_SAMPLE[Frame] = traj->range_param;
    return;
  }

  VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], ANIM_SAMPLE[Frame], &dL, &dTheta);
  Speed = dL/FrameDuration;
  
  Ecart = (ANIM_SAMPLE[Frame] - ANIM_SAMPLE[Frame -1]);
  
  while ((fabs(MaxSpeed - Speed) / MaxSpeed) > Tolerance) {
    DeadLockProtection ++;
    
    if (VariationResult == SAMPLE_PARAM_OVER) /* One of the param is < 0. */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] - Ecart;
    else if (VariationResult == SAMPLE_PARAM_UNDER) /* One of the param is > range_param */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    else if (VariationResult == SAMPLE_PARAM_REVERSED) /* There is a backward walk */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    else if (Speed > MaxSpeed)	/* Speed is over the limit */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] - Ecart;
    else			/* Speed is under the limit */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;

    VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], ANIM_SAMPLE[Frame], &dL, &dTheta);
    Speed = dL/FrameDuration;
    Ecart = Ecart/2.;
    
    if (DeadLockProtection > 50) return;
    
  }
  if (SAMPLE_DEBUG) printf("SPECIFIED : redefinition de %d\n",Frame);
}

static void anim_sample_param_for_rmax (p3d_traj * traj, int Frame, double FrameDuration, double MaxAngSpeed) {
  double MaxSpeed, Speed;
  double Ecart;
  double Tolerance = 0.001;
  double dL, dTheta;
  int VariationResult, DeadLockProtection = 0;
  
  if (Frame < 1) dTheta = 0.;
  
  else 
    VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], ANIM_SAMPLE[Frame], &dL, &dTheta);
  
  Speed = dTheta / FrameDuration;
  
  MaxSpeed = MaxAngSpeed;
  
  /* Verify that problem is feasible */
  if (Speed < MaxSpeed) return;

  Ecart = (ANIM_SAMPLE[Frame] - ANIM_SAMPLE[Frame -1]);
  
  while ((fabs(MaxSpeed - Speed) / MaxSpeed) > Tolerance && (fabs(Ecart) > EPS6 )) {
    DeadLockProtection ++;
    
    if (VariationResult == SAMPLE_PARAM_OVER) /* One of the param is < 0. */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] - Ecart;
    else if (VariationResult == SAMPLE_PARAM_UNDER) /* One of the param is > range_param */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    else if (VariationResult == SAMPLE_PARAM_REVERSED) /* There is a backward walk */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;
    else if (Speed > MaxSpeed)	/* Speed is over the limit */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] - Ecart;
    else			/* Speed is under the limit */
      ANIM_SAMPLE[Frame] = ANIM_SAMPLE[Frame] + Ecart;

    VariationResult = Variation(traj, ANIM_SAMPLE[Frame-1], ANIM_SAMPLE[Frame], &dL, &dTheta);
    Speed = dTheta/FrameDuration;
    Ecart = Ecart/2.;

    if (DeadLockProtection> 50) return;
    
  }
  if (SAMPLE_DEBUG) printf("ROTATION : redefinition de %d\n", Frame);
}

static void anim_sample_param_for_amax (p3d_traj * traj, int * Frame, double FrameDuration, double AccMax) {
  double Speed, PreviousSpeed;
  double dL, dTheta;
  double Acc;
  double BrakeTime, BrakeDist;
  int NofBack, LBack, MoreFrames1, MoreFrames2, MoreFrames, StartBrakeFrame, LCount;
  int CriteriaRespected, AccSmallEnough;
  double SpeedToBrake;
  double Ecart, Tolerance = 0.001;
  int DeadLockProtection = 0, VariationResult;
  double ParamToReach, SpeedToReach;
  int EndIsReached = FALSE;

  if (ANIM_SAMPLE[*Frame] > traj->range_param - EPS) EndIsReached = TRUE;

  Variation(traj, ANIM_SAMPLE[*Frame-2], ANIM_SAMPLE[*Frame-1], &dL, &dTheta);
  PreviousSpeed = dL/FrameDuration;

  VariationResult = Variation(traj, ANIM_SAMPLE[*Frame-1], ANIM_SAMPLE[*Frame], &dL, &dTheta);
  Speed = dL/FrameDuration;

  if (VariationResult == SAMPLE_PARAM_OVER)
    Speed = 0.;
  
  Acc = (Speed - PreviousSpeed) / FrameDuration;
  
  SpeedToBrake = Speed;
  
  if (Acc < -AccMax) {
    CriteriaRespected = FALSE;
    NofBack = 2;
    /* Rollback */
    while (CriteriaRespected == FALSE) {
      Variation(traj, ANIM_SAMPLE[*Frame -NofBack -1], ANIM_SAMPLE[*Frame -NofBack], &dL, &dTheta);
      PreviousSpeed = dL/FrameDuration;
      BrakeDist = 0.;
      for (LBack = 0; LBack < NofBack; LBack ++) {
	Variation(traj, ANIM_SAMPLE[*Frame -LBack -1], ANIM_SAMPLE[*Frame -LBack], &dL, &dTheta);
	BrakeDist = BrakeDist + dL;
      }
      /* Apply Full Brakes for this distance */
      
      Acc = (SpeedToBrake*SpeedToBrake - PreviousSpeed*PreviousSpeed) / (2*BrakeDist);
      
      /* Compute necessary deceleration */
      if (Acc > -AccMax) {
	CriteriaRespected = TRUE;
	StartBrakeFrame = *Frame - NofBack;
      }
      else {
	NofBack ++;
      }
    }
    
    BrakeTime = (Speed - PreviousSpeed) / Acc;
    
    MoreFrames1 = (int)(BrakeTime / FrameDuration);
    MoreFrames2 = NofBack;
    MoreFrames = MoreFrames1  - MoreFrames2;
    
    if (MoreFrames < 0) MoreFrames = 0;

    for (LCount = 0; LCount < MoreFrames; LCount++) {
      *Frame = *Frame +1;
      ANIM_SAMPLE[*Frame] = ANIM_SAMPLE[*Frame -1];
    }

    BrakeTime = (NofBack + MoreFrames) * FrameDuration;
    
    Acc = (Speed - PreviousSpeed) / BrakeTime;

    for (LCount = StartBrakeFrame +1; LCount < (*Frame) +1; LCount++) {
      if (SAMPLE_DEBUG) printf ("ACCELERATION redefinition de %d  \n",LCount);
      ANIM_SAMPLE[LCount] = traj->range_param;
      Variation(traj, ANIM_SAMPLE[LCount-2], ANIM_SAMPLE[LCount-1], &dL, &dTheta);
      Speed = dL / FrameDuration + Acc * FrameDuration;
      anim_sample_param_for_specified_vmax(traj, LCount, FrameDuration, Speed);
    }

    if (EndIsReached == TRUE)
      ANIM_SAMPLE[*Frame] = traj->range_param;

    if (SAMPLE_DEBUG) printf ("last speed is %3.3f and should be %3.3f \n", Speed, SpeedToBrake);
  }
}

double anim_sample_path(AnimProb * ZANIM)
{
  double FrameDuration;
  double RangeParam;
  int LFrame, LDof;
  int NofFrames;
  int CriteriaRespected = FALSE;
  configPt Q;
  int CurrentFrame;
  double TrucBizarre;  
  p3d_traj * traj;
  p3d_rob * RobotPt;
  p3d_traj * TrajPt;

  RobotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  TrajPt =  p3d_get_desc_curid(P3D_TRAJ);

  if (ZANIM->OptimizedResult) traj = ZANIM->OptimizedResult;
  else traj = ZANIM->FindTrajResult;
  if (!traj) {
    PrintWarning(("No traj found to sample\n"));
    return;
  }

  p3d_sel_desc_id(P3D_ROBOT, ZANIM->RobotMin);

  p3d_sel_desc_id(P3D_TRAJ, traj);

  RangeParam = traj->range_param;
  
  /* Initialization */
  FrameDuration = 1. / ZANIM->SampleOptions.FramePerSecond;
  
  ZANIM->FrameDuration = FrameDuration;

  NofFrames = 1;
  ANIM_SAMPLE[0] = 0.;
  CurrentFrame = 0;
  
  while (ANIM_SAMPLE[CurrentFrame] < (RangeParam - EPS)) {

    ANIM_SAMPLE[NofFrames] = (traj->range_param + ANIM_SAMPLE[NofFrames -1]) / 2.;
    CurrentFrame = NofFrames;
    NofFrames ++;
    
    /* Place sample in order to respect VLMAX */
    anim_sample_param_for_vmax (traj, CurrentFrame, FrameDuration, ZANIM->SampleOptions.MaxLinearSpeed, ZANIM->SampleOptions.MaxAcceleration);
    if (ZANIM->SampleOptions.EnableMaxAngularSpeed) {
      anim_sample_param_for_rmax (traj, CurrentFrame, FrameDuration, ZANIM->SampleOptions.MaxAngularSpeed);
      if (CurrentFrame > 2 && ZANIM->SampleOptions.EnableMaxAcceleration) {
	anim_sample_param_for_amax (traj, &CurrentFrame, FrameDuration, ZANIM->SampleOptions.MaxAcceleration);
	NofFrames = CurrentFrame +1;
      }
    }
  }
  
  /* WOOPS TRUC */
  TrucBizarre = ANIM_SAMPLE[NofFrames -1] - ANIM_SAMPLE[NofFrames -2];
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    ANIM_SAMPLE[LFrame] += TrucBizarre * (LFrame) / (NofFrames -1);
  }
  
  /* ALLOC */
  ZANIM->SamplingRes.AnimBuffer = MY_ALLOC(configPt, NofFrames);

  /* SAMPLING */
  for (LFrame = 0 ; LFrame < NofFrames; LFrame ++) {
    
    ZANIM->SamplingRes.AnimBuffer[LFrame] = p3d_alloc_config(ZANIM->Robot);
    Q = p3d_config_at_param_along_traj(traj, ANIM_SAMPLE[LFrame]);
    for (LDof = 0; LDof < ZANIM->Robot->nb_dof; LDof ++) {
      if (LDof < ZANIM->RobotMin->nb_dof) 
	ZANIM->SamplingRes.AnimBuffer[LFrame][LDof] = Q[LDof];
      else 
	ZANIM->SamplingRes.AnimBuffer[LFrame][LDof] = 0.;
    }
    p3d_destroy_config(ZANIM->RobotMin, Q);
  }
  ZANIM->SamplingRes.NofFrames = NofFrames;

  p3d_sel_desc_id(P3D_ROBOT, RobotPt);  
  p3d_sel_desc_id(P3D_TRAJ, TrajPt);
}

void anim_sample_path_initialize(AnimProb * ZANIM) {
  ZANIM->TrajCharacterized = 0;
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->SamplingRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->WalkContRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt1Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt2Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->PosPlanRes));
}

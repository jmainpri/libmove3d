#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#define ANIM_DEBUG 0
#define WALK_DEBUG 0
#define EXTRAPOLATION_FORBIDDEN 1

static void anim_walk_destroy_weights(const int NofFrames, 
				      const p3d_animlib * AnimLib,
				      double ** Weights) 
{
  int LFrame;
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) 
    MY_FREE(Weights[LFrame], double, AnimLib->nof_animation);
  MY_FREE(Weights, double *, NofFrames);
  Weights = NULL;
}

static double ** anim_walk_create_weights(const int NofFrames, 
					  const p3d_animlib * AnimLib) 
{
  double ** Weights;
  int LFrame, LAnim;
  Weights = MY_ALLOC(double *, NofFrames);
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    Weights[LFrame] = MY_ALLOC(double, AnimLib->nof_animation);
  }
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    for (LAnim = 0; LAnim < AnimLib->nof_animation; LAnim ++) {
      Weights[LFrame][LAnim] = 0.;
    } 
  }
  return Weights;
}

static void anim_walk_solve_weights_one_anim(const int NofFrames, 
					     double ** Weights) 
{
  int LFrame;
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    Weights[LFrame][0] = 1.;
  }
}

static void anim_walk_solve_weights_two_anim(const int NofFrames, 
					     const double * LinSpeed, 
					     const double * RotSpeed, 
					     const p3d_animlib * AnimLib, 
					     double ** Weights) 
{
  int LFrame;
  double 
    R1 = AnimLib->animation[0]->rotation_speed,
    R2 = AnimLib->animation[1]->rotation_speed,
    L1 = AnimLib->animation[0]->linear_speed,
    L2 = AnimLib->animation[1]->linear_speed;

  if (fabs (R1 - R2) > fabs (L1 - L2)) {
    for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
      Weights[LFrame][0] = (RotSpeed[LFrame] - R2) / (R1 - R2);
      Weights[LFrame][1] = 1. - Weights[LFrame][0];
    }
  }
  else {
    for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
      Weights[LFrame][0] = (LinSpeed[LFrame] - L2) / (L1 - L2);
      Weights[LFrame][1] = 1. - Weights[LFrame][0];
    }
  }
}

static void anim_walk_solve_weights_three_anim_oldversion(const int NofFrames, 
							  const double * LinSpeed, 
							  const double * RotSpeed, 
							  const p3d_animlib * AnimLib, 
							  double ** Weights) 
{
  double Result, BestResult;
  double W1, W2, W3, W1B, W2B, W3B;
  int BestTrio[3], OldBestTrio[3] = {-1, -1, -1}, LAnim1, LAnim2, LAnim3, LFrame;
  double l, r, l1, l2, l3, r1, r2, r3;
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    l = LinSpeed[LFrame];
    r = RotSpeed[LFrame];
    BestResult = P3D_HUGE;
    for (LAnim1 = 0; LAnim1 < (AnimLib->nof_animation -2); LAnim1 ++) {
      for (LAnim2 = (LAnim1  +1); LAnim2 < (AnimLib->nof_animation -1); LAnim2 ++) {
	for (LAnim3 = (LAnim2 +1); LAnim3 < AnimLib->nof_animation; LAnim3 ++) {
	  l1 = AnimLib->animation[LAnim1]->linear_speed;
	  l2 = AnimLib->animation[LAnim2]->linear_speed;
	  l3 = AnimLib->animation[LAnim3]->linear_speed;
	  r1 = AnimLib->animation[LAnim1]->rotation_speed;
	  r2 = AnimLib->animation[LAnim2]->rotation_speed;
	  r3 = AnimLib->animation[LAnim3]->rotation_speed;
	  
	  if (fabs((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1) > EPS) {
	    W1 =  ((l2-l)*r3+(l-l3)*r2+(l3-l2)*r)/((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1);
	    W2 = -((l1-l)*r3+(l-l3)*r1+(l3-l1)*r)/((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1);
	    W3 =  ((l1-l)*r2+(l-l2)*r1+(l2-l1)*r)/((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1);
	  }
	  else {
	    W1 = P3D_HUGE;
	    W2 = P3D_HUGE;
	    W3 = P3D_HUGE;
	  }
	  
	  Result = fabs(W1) + fabs(W2) + fabs(W3);
	  if (Result < BestResult) {
	    BestTrio[0] = LAnim1;
	    BestTrio[1] = LAnim2;
	    BestTrio[2] = LAnim3;
	    W1B = W1;
	    W2B = W2;
	    W3B = W3;
	    BestResult = Result;
	  }
	  if (LAnim1 == OldBestTrio[0] && LAnim2 == OldBestTrio[1] && LAnim3 == OldBestTrio[2]) {
	    if (WALK_DEBUG) printf ("Result for old trio : \t %d %d %d %3.3f %3.3f %3.3f\n", LAnim1, LAnim2, LAnim3, W1, W2, W3);
	    if ((W1>0. && W1<1.) && (W2>0. && W2<1.) && (W3>0. && W3<1.)) {
	      BestTrio[0] = LAnim1;
	      BestTrio[1] = LAnim2;
	      BestTrio[2] = LAnim3;
	      W1B = W1;
	      W2B = W2;
	      W3B = W3;
	      BestResult = 0.;
	    }
	  }
	}
      }
    }
    Weights[LFrame][BestTrio[0]] = W1B;
    Weights[LFrame][BestTrio[1]] = W2B;
    Weights[LFrame][BestTrio[2]] = W3B;
    OldBestTrio[0] = BestTrio[0];
    OldBestTrio[1] = BestTrio[1];
    OldBestTrio[2] = BestTrio[2];
    if (WALK_DEBUG) printf ("Best Frame %d - \t %d %d %d %3.3f %3.3f %3.3f\n",LFrame, BestTrio[0], BestTrio[1], BestTrio[2], W1B, W2B, W3B);
  }
}

static void anim_walk_solve_weights_three_anim(const int NofFrames, 
					       const double * LinSpeed, 
					       const double * RotSpeed, 
					       const p3d_animlib * AnimLib, 
					       double ** Weights) 
{
  double ResultInter, BestResultInter;
  double ResultExtra, BestResultExtra;
  double W1, W2, W3, W1BI, W2BI, W3BI, W1BE, W2BE, W3BE;
  int BestTrioInter[3], BestTrioExtra[3];
  int LAnim1, LAnim2, LAnim3, LFrame;
  double l, r, l1, l2, l3, r1, r2, r3;
  double rdl1,rdr1,rdl2,rdr2,rdl3,rdr3;
  double maxlinspeed, minlinspeed, maxrotspeed, minrotspeed;
  
  maxrotspeed = maxlinspeed = -P3D_HUGE;
  minrotspeed = minlinspeed =  P3D_HUGE;

  for (LAnim1 = 0; LAnim1 < AnimLib->nof_animation; LAnim1 ++) {
    if (AnimLib->animation[LAnim1]->linear_speed > maxlinspeed) 
      maxlinspeed = AnimLib->animation[LAnim1]->linear_speed;
    if (AnimLib->animation[LAnim1]->rotation_speed > maxrotspeed)
      maxrotspeed = AnimLib->animation[LAnim1]->rotation_speed;
    if (AnimLib->animation[LAnim1]->linear_speed < minlinspeed) 
      minlinspeed = AnimLib->animation[LAnim1]->linear_speed;
    if (AnimLib->animation[LAnim1]->rotation_speed < minrotspeed)
      minrotspeed = AnimLib->animation[LAnim1]->rotation_speed;
  }
  
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    l = LinSpeed[LFrame];
    r = RotSpeed[LFrame];
    BestResultInter = P3D_HUGE;
    BestResultExtra = P3D_HUGE;

    for (LAnim1 = 0; LAnim1 < (AnimLib->nof_animation -2); LAnim1 ++) {
      for (LAnim2 = (LAnim1  +1); LAnim2 < (AnimLib->nof_animation -1); LAnim2 ++) {
	for (LAnim3 = (LAnim2 +1); LAnim3 < AnimLib->nof_animation; LAnim3 ++) {
	  
	  l1 = AnimLib->animation[LAnim1]->linear_speed;
	  l2 = AnimLib->animation[LAnim2]->linear_speed;
	  l3 = AnimLib->animation[LAnim3]->linear_speed;
	  r1 = AnimLib->animation[LAnim1]->rotation_speed;
	  r2 = AnimLib->animation[LAnim2]->rotation_speed;
	  r3 = AnimLib->animation[LAnim3]->rotation_speed;
	  
	  if (fabs((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1) > EPS) {
	    W1 =  ((l2-l)*r3+(l-l3)*r2+(l3-l2)*r)/((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1);
	    W2 = -((l1-l)*r3+(l-l3)*r1+(l3-l1)*r)/((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1);
	    W3 =  ((l1-l)*r2+(l-l2)*r1+(l2-l1)*r)/((l2-l1)*r3+(l1-l3)*r2+(l3-l2)*r1);
	  }
	  else {
	    W1 = P3D_HUGE;
	    W2 = P3D_HUGE;
	    W3 = P3D_HUGE;
	  }

	  if (W1>=0. && W1 <=1. && W2 >=0. && W2 <= 1. && W3>=0. && W3 <= 1.) {
	    rdl1 = (l1 - l) / (maxlinspeed - minlinspeed);
	    rdr1 = (r1 - r) / (maxrotspeed - minrotspeed);
	    rdl2 = (l2 - l) / (maxlinspeed - minlinspeed);
	    rdr2 = (r2 - r) / (maxrotspeed - minrotspeed);
	    rdl3 = (l3 - l) / (maxlinspeed - minlinspeed);
	    rdr3 = (r3 - r) / (maxrotspeed - minrotspeed);
	    ResultInter = (rdl1*rdl1) + (rdl2*rdl2) + (rdl3*rdl3) + (rdr1*rdr1) + (rdr2*rdr2) + (rdr3*rdr3);
	    if (ResultInter < BestResultInter) {
	      BestResultInter = ResultInter;
	      BestTrioInter[0] = LAnim1;
	      BestTrioInter[1] = LAnim2;
	      BestTrioInter[2] = LAnim3;
	      W1BI = W1;
	      W2BI = W2;
	      W3BI = W3;
	    }
	  }

	  else {
	    rdl1 = (l1 - l) / (maxlinspeed - minlinspeed);
	    rdr1 = (r1 - r) / (maxrotspeed - minrotspeed);
	    rdl2 = (l2 - l) / (maxlinspeed - minlinspeed);
	    rdr2 = (r2 - r) / (maxrotspeed - minrotspeed);
	    rdl3 = (l3 - l) / (maxlinspeed - minlinspeed);
	    rdr3 = (r3 - r) / (maxrotspeed - minrotspeed);
	    ResultExtra = (rdl1*rdl1) + (rdl2*rdl2) + (rdl3*rdl3) + (rdr1*rdr1) + (rdr2*rdr2) + (rdr3*rdr3);
	    if (ResultExtra < BestResultExtra) {
	      BestResultExtra = ResultExtra;
	      BestTrioExtra[0] = LAnim1;
	      BestTrioExtra[1] = LAnim2;
	      BestTrioExtra[2] = LAnim3;
	      W1BE = W1;
	      W2BE = W2;
	      W3BE = W3;
	    }
	  }
	    
	}
      }
    }
    if (BestResultInter < (P3D_HUGE / 2.)) {
      Weights[LFrame][BestTrioInter[0]] = W1BI;
      Weights[LFrame][BestTrioInter[1]] = W2BI;
      Weights[LFrame][BestTrioInter[2]] = W3BI;
      if (WALK_DEBUG) printf ("INTERPOLATION Frame %d - \t %d %d %d %3.3f %3.3f %3.3f\n",LFrame, BestTrioInter[0], BestTrioInter[1], BestTrioInter[2], W1BI, W2BI, W3BI);
    }
    else {
      Weights[LFrame][BestTrioExtra[0]] = W1BE;
      Weights[LFrame][BestTrioExtra[1]] = W2BE;
      Weights[LFrame][BestTrioExtra[2]] = W3BE;
      if (WALK_DEBUG) printf ("EXTRAPOLATION Frame %d - \t %d %d %d %3.3f %3.3f %3.3f\n",LFrame, BestTrioExtra[0], BestTrioExtra[1], BestTrioExtra[2], W1BE, W2BE, W3BE);
    }
  }
}

static double ** anim_walk_solve_weights(const int NofFrames, 
					 const double * LinSpeed, 
					 const double * RotSpeed, 
					 const p3d_animlib * AnimLib) 
{
  double ** Weights;
  int LAnim, LFrame;
  double Min,Max, PositiveSum;
  Weights = anim_walk_create_weights(NofFrames, AnimLib);
  if (AnimLib->nof_animation == 1) {
    anim_walk_solve_weights_one_anim(NofFrames, Weights);
    return Weights;
  }
  if (AnimLib->nof_animation == 2) {
    anim_walk_solve_weights_two_anim(NofFrames, LinSpeed, RotSpeed, AnimLib, Weights);
    return Weights;
  }
  anim_walk_solve_weights_three_anim(NofFrames, LinSpeed, RotSpeed, AnimLib, Weights);

  if (EXTRAPOLATION_FORBIDDEN) {
    for (LFrame = 0; LFrame < NofFrames; LFrame++) {
      Min = 0. + EPS6;
      Max = 1. - EPS6;
      PositiveSum = 0.;
      for (LAnim = 0; LAnim < AnimLib->nof_animation; LAnim++) {
	if (Weights[LFrame][LAnim] > Max) Max = Weights[LFrame][LAnim];
	if (Weights[LFrame][LAnim] < Min) Min = Weights[LFrame][LAnim];
	if (Weights[LFrame][LAnim] > 0.) PositiveSum += Weights[LFrame][LAnim];
      }
      if (Min < 0. || Max > 1.) {
	
	  
	/*	MEthode 1 : projection sur l'arete de l'enveloppe du domaine de vitesses */
	for (LAnim = 0; LAnim < AnimLib->nof_animation; LAnim++) {
	  if (Weights[LFrame][LAnim] < 0.) 
	    Weights[LFrame][LAnim] = 0.;
	  else 
	    Weights[LFrame][LAnim] = Weights[LFrame][LAnim] / PositiveSum;

	}
      }
    }
  }

  return Weights;
}

void anim_walk (AnimProb * ZANIM)
{
  int LFrame, LAnim, LDof, LCoef, LastFrame, NofFrames;
  double param, Oldparam, *RealIn[MAX_NOF_DOFS], *ImagIn[MAX_NOF_DOFS];
  double LocalCycleLength;
  double InfluenceCoef;
  double Orientation;
  configPt QResult;
  p3d_animation * AnimPt;
  double FrameDuration = ZANIM->FrameDuration;
  p3d_rob * robot = ZANIM->Robot;
  Charac AnimCharac;
  p3d_animlib AnimLib;
  double ** AnimLibWeights;
  int NofCoef;

  LastFrame = ZANIM->SamplingRes.NofFrames -1;
  NofFrames = ZANIM->SamplingRes.NofFrames;
  
  anim_selec_robot(&XYZ_ANIM, &AnimLib, ZANIM->Robot);

  if (AnimLib.nof_animation < 1) return;

  AnimLibWeights = anim_walk_solve_weights(NofFrames, ZANIM->CharacRes.LinSpeed, ZANIM->CharacRes.RotSpeed, &AnimLib);
  AnimCharac = ZANIM->CharacRes;
  

  NofCoef = AnimLib.animation[0]->nof_coef;

  for (LDof = 0; LDof < MAX_NOF_DOFS; LDof++) {
    RealIn[LDof] = MY_ALLOC(double, NofCoef);
    ImagIn[LDof] = MY_ALLOC(double, NofCoef);
  }
  
  QResult  = p3d_alloc_config(robot);
  
  Oldparam = 0.;
  
  ZANIM->WalkContRes.NofFrames = NofFrames;
  ZANIM->WalkContRes.AnimBuffer = MY_ALLOC(configPt, NofFrames);
  for (LFrame = 0; LFrame < NofFrames; LFrame++) {
    ZANIM->WalkContRes.AnimBuffer[LFrame] = p3d_copy_config(robot, ZANIM->SamplingRes.AnimBuffer[LFrame]);
  }

  for (LFrame = 0; LFrame < NofFrames; LFrame++) {
    
    for (LDof = 0; LDof < MAX_NOF_DOFS; LDof++) {
      for (LCoef = 0; LCoef < NofCoef; LCoef++) {
	RealIn[LDof][LCoef] = 0.;
	ImagIn[LDof][LCoef] = 0.;
      }
    }
    /* compute local cycle length and param progression */
    LocalCycleLength = 0.0;
    for (LAnim = 0 ; LAnim < AnimLib.nof_animation; LAnim++) {
      LocalCycleLength+=AnimLib.animation[LAnim]->length * AnimLibWeights[LFrame][LAnim];
    }
    if (LocalCycleLength > EPS6) 
      param = Oldparam + AnimCharac.Sens[LFrame] * AnimCharac.FrameDist[LFrame] / LocalCycleLength;
    else
      param = Oldparam;
    
    Oldparam = param;
    
    for (LAnim = 0 ; LAnim < AnimLib.nof_animation; LAnim++) {
      if (fabs(AnimLibWeights[LFrame][LAnim]) > EPS3) {
	for (LDof = 0; LDof < robot->nb_dof; LDof ++) {
	  for (LCoef = 0; LCoef < NofCoef; LCoef ++) {
	    RealIn[LDof][LCoef] += AnimLib.animation[LAnim]->fourier_real[LCoef][LDof] * AnimLibWeights[LFrame][LAnim];
	    ImagIn[LDof][LCoef] += AnimLib.animation[LAnim]->fourier_imag[LCoef][LDof] * AnimLibWeights[LFrame][LAnim];
	  }
	}
      }
    }

    InfluenceCoef = 1.;
    
    if (LFrame * FrameDuration < ZANIM->WalkOptions.RespectTime && ZANIM->WalkOptions.RespectInit) {
      InfluenceCoef = LFrame * FrameDuration / ZANIM->WalkOptions.RespectTime;
      for (LDof = robot->joints[2]->index_dof; LDof < robot->nb_dof; LDof++) {
	RealIn[LDof][0] = RealIn[LDof][0] * InfluenceCoef + (1. - InfluenceCoef) * ZANIM->Qi[LDof] * (double)NofCoef;
      }
      
      
      for (LDof = robot->joints[1]->index_dof; LDof < robot->nb_dof; LDof++) {
	for (LCoef = 1; LCoef < NofCoef; LCoef ++) {
	  RealIn[LDof][LCoef] *= InfluenceCoef;
	  ImagIn[LDof][LCoef] *= InfluenceCoef;
	}
      }
    }
    
    if ((LastFrame - LFrame)* FrameDuration < ZANIM->WalkOptions.RespectTime && ZANIM->WalkOptions.RespectEnd) {
	  
      InfluenceCoef = (LastFrame - LFrame)* FrameDuration / ZANIM->WalkOptions.RespectTime;
      
      for (LDof = robot->joints[2]->index_dof; LDof < robot->nb_dof; LDof++) {
	RealIn[LDof][0] = RealIn[LDof][0] * InfluenceCoef + (1. - InfluenceCoef) * ZANIM->Qf[LDof] * (double)NofCoef;
      }
	  
      for (LCoef = 1; LCoef < NofCoef; LCoef ++) {
	for (LDof = robot->joints[1]->index_dof; LDof < robot->nb_dof; LDof++) {
	  RealIn[LDof][LCoef] *= InfluenceCoef;
	  ImagIn[LDof][LCoef] *= InfluenceCoef;
	}
      }
    }
    
    for (LDof = 0; LDof < robot->nb_dof; LDof++) {
      QResult[LDof] = p3d_fourier_inverse_transform(ZANIM->WalkOptions.Filter,NofCoef, RealIn[LDof], ImagIn[LDof], param);
    }

    Orientation = ZANIM->SamplingRes.AnimBuffer[LFrame][MI_GRZ];
    
    ZANIM->WalkContRes.AnimBuffer[LFrame][MI_X] += (QResult[MI_X] * cos(Orientation) + QResult[MI_Y] * sin(Orientation)); 
    ZANIM->WalkContRes.AnimBuffer[LFrame][MI_Y] += (QResult[MI_Y] * cos(Orientation) - QResult[MI_X] * sin(Orientation)); 
    ZANIM->WalkContRes.AnimBuffer[LFrame][MI_Z] += QResult[MI_Z]; 
    ZANIM->WalkContRes.AnimBuffer[LFrame][MI_GRZ] += QResult[MI_GRZ];
    
    for (LDof = robot->joints[3]->index_dof; LDof<robot->nb_dof; LDof++) {
      ZANIM->WalkContRes.AnimBuffer[LFrame][LDof] = QResult[LDof];
    }
  }
  
  for (LDof = 0; LDof < MAX_NOF_DOFS; LDof++) {
    MY_FREE(RealIn[LDof],double, NofCoef);
    MY_FREE(ImagIn[LDof],double, NofCoef);
  }
  p3d_destroy_config(robot, QResult);
  
  anim_walk_destroy_weights(NofFrames, &AnimLib, AnimLibWeights);
  
}

void anim_walk_initialize(AnimProb * ZANIM) {
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->WalkContRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt1Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt2Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->PosPlanRes));
}

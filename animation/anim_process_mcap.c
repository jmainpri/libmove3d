#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"

#define ANIM_PROCESS_DEBUG 0

static FILE * DebugFile;

static void anim_debug_open_file (void) {
  int fd;

  if (ANIM_PROCESS_DEBUG) {
    fd = creat("DEBUG_anim_process_mcap",0666);
    DebugFile = fopen("DEBUG_anim_process_mcap","w");
  }
}

static void anim_deg_to_rad(p3d_animation * AnimPt) 
{
  int ldof, lframe;
  p3d_rob * RobotPt = AnimPt->aim_robot;

  for (lframe=0; lframe<AnimPt->nof_frames; lframe++) {
    for (ldof=RobotPt->joints[1]->index_dof+3; ldof<RobotPt->nb_dof; ldof++) {
      AnimPt->data[lframe][ldof] = angle_limit_PI(AnimPt->data[lframe][ldof]*M_PI/180.);
    }
  }
}

static double anim_comp_circle_error(p3d_animation * anim, 
				     double A, 
				     double B, 
				     double R) 
{  
  int lframe;
  double error;
  
  error = 0;
  for (lframe = 0; lframe < anim->nof_frames; lframe ++){
    error += ((anim->data[lframe][6] - A)*(anim->data[lframe][6] - A)
	      + (anim->data[lframe][7] - B)*(anim->data[lframe][7] - B)
	      - R*R) 
      * ((anim->data[lframe][6] - A)*(anim->data[lframe][6] - A)
	 + (anim->data[lframe][7] - B)*(anim->data[lframe][7] - B)
	 - R*R)
      ;
  }
  return error;
}

static void anim_comp_average(const p3d_animation * anim,
			      double * MX, 
			      double * MY, 
			      double * MZ, 
			      double * MT)
{
  int lframe;

  *MX = 0.;
  *MZ = 0.;
  *MY = 0.;
  *MT = 0.;
  for (lframe = 0; lframe < anim->nof_frames; lframe ++) {
    *MX += anim->data[lframe][6];
    *MY += anim->data[lframe][7];
    *MZ += anim->data[lframe][8];
    *MT += lframe;
  }
  *MZ /= anim->nof_frames;
  *MY /= anim->nof_frames;
  *MX /= anim->nof_frames;
  *MT /= anim->nof_frames;
}

static void anim_comp_covariance(const p3d_animation * anim,
				 double * CX, 
				 double * CY, 
				 double * CZ, 
				 double * CT,
				 const double MX, 
				 const double MY, 
				 const double MZ, 
				 const double MT) 
{
  int lframe;

  *CX = 0.;
  *CZ = 0.;
  *CY = 0.;
  *CT = 0.;
  for (lframe = 0; lframe < anim->nof_frames; lframe++) {
    *CZ += (anim->data[lframe][8] - MZ)*(lframe - MT);
    *CY += (anim->data[lframe][7] - MY)*(lframe - MT);
    *CX += (anim->data[lframe][6] - MX)*(lframe - MT);
    *CT += (lframe - MT)*(lframe - MT);
  }
}

static void p3d_anim_change_rotation_data (p3d_animation * AnimPt)
{
  p3d_rob * RobotPt = AnimPt->aim_robot;
  int LJoint, LFrame, LDof, DofIndex;
  p3d_jnt * JntPt;
  for (LJoint = 0; LJoint < RobotPt->njoints; LJoint ++) {
    JntPt = RobotPt->joints[LJoint];
    switch (JntPt->type) {
    case P3D_PLAN :
    case P3D_TRANSLATE : break;
    case P3D_ROTATE : 
      DofIndex = JntPt->index_dof;
      for (LFrame = 1; LFrame < AnimPt->nof_frames; LFrame++) {
	AnimPt->data[LFrame][DofIndex] = AnimPt->data[LFrame-1][DofIndex] 
	  + diff_angle (AnimPt->data[LFrame -1][DofIndex],
			AnimPt->data[LFrame][DofIndex]) ;
      }
      break;
    case P3D_BASE : 
    case P3D_FREEFLYER :
      DofIndex = JntPt->index_dof;
      for (LDof = DofIndex + 3; LDof < DofIndex + 6; LDof ++) {
	for (LFrame = 1; LFrame < AnimPt->nof_frames; LFrame++) {
	  AnimPt->data[LFrame][LDof] = AnimPt->data[LFrame-1][LDof] 
	    + diff_angle (AnimPt->data[LFrame -1][LDof],
			  AnimPt->data[LFrame][LDof]) ;
	}
      }
      break;
    default : 
      PrintError(("p3d_anifourcomp.c : JOINT TYPE UNKNOWN \n"));
      break;
    }
  }
}

static void anim_process_straight_mcap (p3d_animation * anim)
				  
{
  double MZ, MY, MX, MT;
  double CZ, CY, CX, CT;
  double AX, BX, AY, BY, AZ, BZ;

  anim_comp_average (anim, &MX, &MY, &MZ, &MT);
  anim_comp_covariance (anim, &CX, &CY, &CZ, &CT, MX, MY, MZ, MT);

  AX = CX / CT;
  BX = MX - AX * MT;
  AY = CY / CT;
  BY = MY - AY * MT;
  AZ = CZ / CT;
  BZ = MZ - AZ * MT;

  anim->tcharac.AX = AX;
  anim->tcharac.BX = BX;
  anim->tcharac.AY = AY;
  anim->tcharac.BY = BY;
  anim->tcharac.AZ = AZ;
  anim->tcharac.BZ = BZ;
  
  if (ANIM_PROCESS_DEBUG) {
    fprintf(DebugFile,"Straight motion capture chgaracteristics : 
AX = %f
BX = %f
AY = %f
BY = %f
AZ = %f
BZ = %f \n", AX,BX,AY,BY,AZ,BZ);
  }
}

static void anim_process_turning_mcap (p3d_animation * anim)
{
  double MZ, MY, MX, MT;
  double CX, CY, CZ, CT;
  double AZ, BZ, A, B, R, AngleDepart, AngleArrivee, OldA, OldB, OldR;
  double OldError, ErrorPlus, ErrorLess;
  double Step, FinalStep;
  
  anim_comp_average (anim, &MX, &MY, &MZ, &MT);
  anim_comp_covariance (anim, &CX, &CY, &CZ, &CT, MX, MY, MZ, MT);
  AZ = CZ / CT;
  BZ = MZ - AZ * MT;
  /* ADAPTATION CERCLE */
  A = 0; OldA = -1;
  B = 0; OldB = -1;
  R = 100; OldR = -1;
  Step = fabs(anim->data[anim->nof_frames-1][6] - anim->data[0][6]) / 2.;
  FinalStep = Step / 100000.;
  while (Step > FinalStep) {
    OldError  = anim_comp_circle_error (anim, A, B, R);
    ErrorPlus = anim_comp_circle_error (anim, A+Step, B, R);
    ErrorLess = anim_comp_circle_error (anim, A-Step, B, R);
    if (OldError < ErrorLess && OldError < ErrorPlus);
    else if (ErrorPlus < ErrorLess) A += Step;
    else A-=Step;
    OldError  = anim_comp_circle_error (anim, A, B, R);
    ErrorPlus = anim_comp_circle_error (anim, A, B+Step, R);
    ErrorLess = anim_comp_circle_error (anim, A, B-Step, R);
    if (OldError < ErrorLess && OldError < ErrorPlus);
    else if (ErrorPlus < ErrorLess) B += Step;
    else B-=Step;
    OldError  = anim_comp_circle_error (anim, A, B, R);
    ErrorPlus = anim_comp_circle_error (anim, A, B, R+Step);
    ErrorLess = anim_comp_circle_error (anim, A, B, R-Step);
    if (OldError < ErrorLess && OldError < ErrorPlus);
    else if (ErrorPlus < ErrorLess) R += Step;
    else R-=Step;
    if (OldA == A && OldR == R && OldB == B) Step /= 2.;
    OldA = A;
    OldB = B;
    OldR = R;
  }
  R = fabs(R);
  AngleDepart = atan ((anim->data[0][7] - B) / (anim->data[0][6] - A));
  if (anim->data[0][6] < A) AngleDepart = AngleDepart + M_PI;
  AngleDepart = angle_limit_PI(AngleDepart);
  AngleArrivee = atan ((anim->data[anim->nof_frames -1][7] - B) / (anim->data[anim->nof_frames -1][6] - A));
  if (anim->data[anim->nof_frames -1][6] < A) AngleArrivee = AngleArrivee + M_PI;
  AngleArrivee = angle_limit_PI(AngleArrivee);

  anim->tcharac.AZ = AZ;
  anim->tcharac.BZ = BZ;
  anim->tcharac.A = A;
  anim->tcharac.B = B;
  anim->tcharac.R = R;
  anim->tcharac.AngleDepart = AngleDepart;
  anim->tcharac.AngleArrivee = AngleArrivee;
  
  if (ANIM_PROCESS_DEBUG) {
    fprintf(DebugFile,"Straight motion capture chgaracteristics : 
A = %f
B = %f
R = %f
AngleDepart  = %f
AngleArrivee = %f \n", A,B,R,AngleDepart,AngleArrivee);
  }
}

static void anim_extract_periodic (p3d_animation * AnimPt, 
				   double ** TempResizedAnimation,
				   const int NofCoef) { 
  
  double *EstX, *EstY, *EstZ, *EstTheta, *EstAngle;
  double *ErrX, *ErrY, *ErrZ, *ErrTheta;
  double AX = AnimPt->tcharac.AX;
  double BX = AnimPt->tcharac.BX;
  double AY = AnimPt->tcharac.AY;
  double BY = AnimPt->tcharac.BY;
  double AZ = AnimPt->tcharac.AZ;
  double BZ = AnimPt->tcharac.BZ;
  double A = AnimPt->tcharac.A;
  double B = AnimPt->tcharac.B;
  double R = AnimPt->tcharac.R;
  double AngleDepart = AnimPt->tcharac.AngleDepart;
  double AngleArrivee = AnimPt->tcharac.AngleArrivee;
  double *RealIn, *ImagIn;
  double *RealOut, *ImagOut;
  double Param, DeltaX, DeltaY;
  int LFrame;

  RealIn  = MY_ALLOC(double,NofCoef);
  RealOut = MY_ALLOC(double,NofCoef);
  ImagIn  = MY_ALLOC(double,NofCoef);
  ImagOut = MY_ALLOC(double,NofCoef);
  EstX    = MY_ALLOC(double,NofCoef);
  EstY    = MY_ALLOC(double,NofCoef);
  EstZ    = MY_ALLOC(double,NofCoef);
  EstTheta= MY_ALLOC(double,NofCoef);
  EstAngle= MY_ALLOC(double,NofCoef);
  ErrX    = MY_ALLOC(double,NofCoef);
  ErrY    = MY_ALLOC(double,NofCoef);
  ErrZ    = MY_ALLOC(double,NofCoef);
  ErrTheta= MY_ALLOC(double,NofCoef);
  
  for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
    Param = (double)LFrame * (double)(AnimPt->nof_frames)  / (double)NofCoef;
    if (AnimPt->straight == 1) {
      EstX[LFrame] = Param * AX + BX;
      EstY[LFrame] = Param * AY + BY;
      EstZ[LFrame] = Param * AZ + BZ;
      EstTheta[LFrame] = atan(AY / AX);
      if (AX < 0.) EstTheta[LFrame] = EstTheta[LFrame] + M_PI;
    }
    else {
      EstAngle[LFrame] = AngleDepart + diff_angle (AngleDepart, AngleArrivee) * Param / (NofCoef -1);
      EstAngle[LFrame] = angle_limit_PI(EstAngle[LFrame]);
      
      EstX[LFrame] = A + R * cos (EstAngle[LFrame]);
      EstY[LFrame] = B + R * sin (EstAngle[LFrame]);
      EstZ[LFrame] = Param * AZ + BZ;
      if (angle_limit_PI(diff_angle (AngleDepart, AngleArrivee)) < 0) {
	EstTheta[LFrame] = EstAngle[LFrame] - M_PI / 2.; 
      }
      else {
	EstTheta[LFrame] = EstAngle[LFrame] + M_PI / 2.;
      }
      EstTheta[LFrame] = angle_limit_PI(EstTheta[LFrame]);
    }
    if (ANIM_PROCESS_DEBUG) {
      fprintf (DebugFile, "Estimated values (X,Y,Z,T) : %f %f %f %f \n",EstX[LFrame],EstY[LFrame],EstZ[LFrame],EstTheta[LFrame]);
      fprintf (DebugFile, "Real      values (X,Y,Z,T) : %f %f %f %f \n",TempResizedAnimation[LFrame][6],TempResizedAnimation[LFrame][7],TempResizedAnimation[LFrame][8],TempResizedAnimation[LFrame][AnimPt->index_theta]);
    }
  }

  for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
    DeltaX = TempResizedAnimation[LFrame][6] - EstX[LFrame];
    DeltaY = TempResizedAnimation[LFrame][7] - EstY[LFrame];
    ErrX[LFrame]     = DeltaX * cos(EstTheta[LFrame]) - DeltaY * sin(EstTheta[LFrame]);
    ErrY[LFrame]     = DeltaX * sin(EstTheta[LFrame]) + DeltaY * cos(EstTheta[LFrame]);
    ErrZ[LFrame]     = TempResizedAnimation[LFrame][8] - EstZ[LFrame];
    ErrTheta[LFrame] = TempResizedAnimation[LFrame][AnimPt->index_theta] - EstTheta[LFrame];
    ImagIn[LFrame] = 0.;
    if (ANIM_PROCESS_DEBUG) {
      fprintf (DebugFile, "Errx, y, z, theta : %f %f %f %f \n",ErrX[LFrame],ErrY[LFrame],ErrZ[LFrame],ErrTheta[LFrame]);
      fprintf (DebugFile, "Deltax, y : %f %f \n",DeltaX, DeltaY);
    }
  }

  anim_fft_double (NofCoef, 0, ErrX, ImagIn, RealOut, ImagOut);
  for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
    AnimPt->fourier_real[LFrame][6] = RealOut[LFrame];
    AnimPt->fourier_imag[LFrame][6] = ImagOut[LFrame];
  }
  anim_fft_double (NofCoef, 0, ErrY, ImagIn, RealOut, ImagOut);
  for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
    AnimPt->fourier_real[LFrame][7] = RealOut[LFrame];
    AnimPt->fourier_imag[LFrame][7] = ImagOut[LFrame];
  }
  anim_fft_double (NofCoef, 0, ErrZ, ImagIn, RealOut, ImagOut);
  for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
    AnimPt->fourier_real[LFrame][8] = RealOut[LFrame];
    AnimPt->fourier_imag[LFrame][8] = ImagOut[LFrame];
  }
  anim_fft_double (NofCoef, 0, ErrTheta, ImagIn, RealOut, ImagOut);
  for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
    AnimPt->fourier_real[LFrame][AnimPt->index_theta] = RealOut[LFrame];
    AnimPt->fourier_imag[LFrame][AnimPt->index_theta] = ImagOut[LFrame];
  }

  MY_FREE(RealIn, double,NofCoef);
  MY_FREE(RealOut,double,NofCoef);
  MY_FREE(ImagIn, double,NofCoef);
  MY_FREE(ImagOut,double,NofCoef);
  MY_FREE(EstX, double,NofCoef);
  MY_FREE(EstY,double,NofCoef);
  MY_FREE(EstZ, double,NofCoef);
  MY_FREE(EstTheta,double,NofCoef);
  MY_FREE(ErrX, double,NofCoef);
  MY_FREE(ErrY,double,NofCoef);
  MY_FREE(ErrZ, double,NofCoef);
  MY_FREE(ErrTheta,double,NofCoef);

}

static void anim_freq_trans(p3d_animation * AnimPt) {
  double ** TempResizedAnimation;
  double *RealIn, *ImagIn;
  double *RealOut, *ImagOut;
  double FCoef;
  int LFrame, LDof, IndexEquivFrame, NofCoef;
  p3d_rob * RobotPt = AnimPt->aim_robot;

  NofCoef = AnimPt->nof_coef;
  
  TempResizedAnimation = MY_ALLOC(double *,NofCoef);
  RealIn  = MY_ALLOC(double,NofCoef);
  RealOut = MY_ALLOC(double,NofCoef);
  ImagIn  = MY_ALLOC(double,NofCoef);
  ImagOut = MY_ALLOC(double,NofCoef);

  p3d_anim_change_rotation_data(AnimPt);

  for (LFrame = 0; LFrame < NofCoef; LFrame++) {
    TempResizedAnimation[LFrame] = MY_ALLOC(double, RobotPt->nb_dof);
    FCoef = ((double)LFrame / (double)(NofCoef-1))*(double)(AnimPt->nof_frames-1);
    IndexEquivFrame = (int)FCoef;
    FCoef -= IndexEquivFrame;
    for (LDof = 0; LDof < RobotPt->nb_dof; LDof ++) {
      if (LFrame != (NofCoef -1))
	TempResizedAnimation[LFrame][LDof] = AnimPt->data[IndexEquivFrame][LDof] * (1. - FCoef) + AnimPt->data[IndexEquivFrame + 1][LDof] * FCoef;
      else
	TempResizedAnimation[LFrame][LDof] = AnimPt->data[IndexEquivFrame][LDof];
    }
  }
  
  AnimPt->fourier_real = MY_ALLOC (double *, NofCoef);
  AnimPt->fourier_imag = MY_ALLOC (double *, NofCoef);

  for (LFrame = 0; LFrame < NofCoef; LFrame++) {
    AnimPt->fourier_real[LFrame] = MY_ALLOC (double, RobotPt->nb_dof);
    AnimPt->fourier_imag[LFrame] = MY_ALLOC (double, RobotPt->nb_dof);
  }

  for (LDof = 0; LDof < RobotPt->nb_dof; LDof ++) {
    for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
      RealIn[LFrame] = TempResizedAnimation[LFrame][LDof];
      ImagIn[LFrame] = 0.0;
    }

    anim_fft_double (NofCoef, 0, RealIn, ImagIn, RealOut, ImagOut);

    for (LFrame = 0; LFrame < NofCoef; LFrame ++) {
      AnimPt->fourier_real[LFrame][LDof] = RealOut[LFrame];
      AnimPt->fourier_imag[LFrame][LDof] = ImagOut[LFrame];
    }
  }

  MY_FREE(RealIn, double,NofCoef);
  MY_FREE(RealOut,double,NofCoef);
  MY_FREE(ImagIn, double,NofCoef);
  MY_FREE(ImagOut,double,NofCoef);

  anim_extract_periodic(AnimPt, TempResizedAnimation, NofCoef);

  for (LFrame = 0; LFrame < NofCoef; LFrame++) {
    MY_FREE(TempResizedAnimation[LFrame], double, RobotPt->nb_dof);
  }

  MY_FREE(TempResizedAnimation, double*, NofCoef);

}

void anim_speed_characterization(p3d_animation * AnimPt) {
  double Param;
  int Framerate;
  double AX = AnimPt->tcharac.AX;
  double BX = AnimPt->tcharac.BX;
  double AY = AnimPt->tcharac.AY;
  double BY = AnimPt->tcharac.BY;
  double AZ = AnimPt->tcharac.AZ;
  double BZ = AnimPt->tcharac.BZ;
  double R = AnimPt->tcharac.R;
  double AngleDepart = AnimPt->tcharac.AngleDepart;
  double AngleArrivee = AnimPt->tcharac.AngleArrivee;

  Param = AnimPt->nof_frames -1;
  Framerate = AnimPt->framerate;
  if (AnimPt->straight == 1) {
    AnimPt->length         = euclidian_dist(BX, BY, BZ, AX*Param+BX, AY*Param+BY, AZ*Param+BZ);
    AnimPt->linear_speed   = AnimPt->length / (double) AnimPt->nof_frames * Framerate;
    AnimPt->rotation_speed = 0.0;
  }
  else {
    AnimPt->length         = fabs(R * diff_angle (AngleArrivee, AngleDepart));
    AnimPt->linear_speed   = AnimPt->length / (double)AnimPt->nof_frames * Framerate;
    AnimPt->rotation_speed = diff_angle(AngleDepart, AngleArrivee) / (double)AnimPt->nof_frames * Framerate;
  }
}

void anim_process_mcap(p3d_animation * AnimPt) {
  anim_debug_open_file();

  if (AnimPt->degree == 1) 
    anim_deg_to_rad(AnimPt);
  if (AnimPt->straight == 1) 
    anim_process_straight_mcap(AnimPt);
  else 
    anim_process_turning_mcap(AnimPt);

  anim_freq_trans(AnimPt);
  anim_speed_characterization(AnimPt);
  AnimPt->processed = 1;
  if (ANIM_PROCESS_DEBUG) fclose(DebugFile);
}

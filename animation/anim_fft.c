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

#define CHECKPOINTER(p)  CheckPointer(p,#p)



void CheckPointer (const void * Pointer,
			  const char * Name )
{
  if (Pointer == NULL ) {
    PrintError(("Error in fft_double():  %s == NULL\n", Name));
    exit(1);
  }
}

int IsPowerOfTwo (const unsigned x )
{
  if ( (x < 2) || (x & (x-1)) )
    return FALSE;
  return TRUE;
}

unsigned NumberOfBitsNeeded ( unsigned PowerOfTwo )
{
  unsigned Result;
  if ( PowerOfTwo < 2 ) {
    PrintError(("p3d_anifourcomp.c : argument %d to NumberOfBitsNeeded is too small.\n", PowerOfTwo));
    exit(1);
  }
  for ( Result=0; ; Result++ ) {
    if ( PowerOfTwo & (1 << Result) )
      return Result;
  }
}

unsigned ReverseBits ( unsigned Index, unsigned NumBits )
{
  unsigned LBit, Result;
  for ( LBit=Result=0; LBit < NumBits; LBit++ ) {
    Result = (Result << 1) | (Index & 1);
    Index >>= 1;
  }
  return Result;
}

double Index_to_frequency ( unsigned NumSamples, unsigned Index )
{
  if ( Index >= NumSamples )
    return 0.0;
  else if ( Index <= NumSamples/2 )
    return (double)Index / (double)NumSamples;
  return -(double)(NumSamples-Index) / (double)NumSamples;
}

void anim_fft_double (unsigned  NofSamples,
		      int       InverseTransform,
		      double   *RealIn,
		      double   *ImagIn,
		      double   *RealOut,
		      double   *ImagOut )
{
  unsigned NofBits;    /* Number of bits needed to store indices */
  unsigned LSample, j, k, n;
  unsigned BlockSize, BlockEnd;
  double AngleNumerator = 2.0 * M_PI;
  double TempReal, TempImagin;     /* temp real, temp imaginary */
  
  if ( !IsPowerOfTwo(NofSamples) ) {
    PrintError(("Error in fft():  NofSamples=%u is not power of two\n", NofSamples) );
    exit(1);
  }
  if ( InverseTransform ) AngleNumerator = -AngleNumerator;
  
  CHECKPOINTER ( RealIn );
  CHECKPOINTER ( RealOut );
  CHECKPOINTER ( ImagOut );

  NofBits = NumberOfBitsNeeded ( NofSamples );

  /*
  **   Do simultaneous data copy and bit-reversal ordering into outputs...
  */
  
  for ( LSample=0; LSample < NofSamples; LSample++ ) {
    j = ReverseBits ( LSample, NofBits );
    RealOut[j] = RealIn[LSample];
    ImagOut[j] = (ImagIn == NULL) ? 0.0 : ImagIn[LSample];
  }
  
  /*
  **   Do the FFT itself...
  */
  
  BlockEnd = 1;
  for ( BlockSize = 2; BlockSize <= NofSamples; BlockSize <<= 1 ) {
    double delta_angle = AngleNumerator / (double)BlockSize;
    double sm2 = sin ( -2 * delta_angle );
    double sm1 = sin ( -delta_angle );
    double cm2 = cos ( -2 * delta_angle );
    double cm1 = cos ( -delta_angle );
    double w = 2 * cm1;
    double ar[3], ai[3];
    
    for ( LSample=0; LSample < NofSamples; LSample += BlockSize ) {
      ar[2] = cm2;
      ar[1] = cm1;
      
      ai[2] = sm2;
      ai[1] = sm1;
      
      for ( j=LSample, n=0; n < BlockEnd; j++, n++ ) {
	ar[0] = w*ar[1] - ar[2];
	ar[2] = ar[1];
	ar[1] = ar[0];
	  
	ai[0] = w*ai[1] - ai[2];
	ai[2] = ai[1];
	ai[1] = ai[0];
	
	k = j + BlockEnd;
	TempReal = ar[0]*RealOut[k] - ai[0]*ImagOut[k];
	TempImagin = ar[0]*ImagOut[k] + ai[0]*RealOut[k];
	
	RealOut[k] = RealOut[j] - TempReal;
	ImagOut[k] = ImagOut[j] - TempImagin;
	
	RealOut[j] += TempReal;
	ImagOut[j] += TempImagin;
      }
    }
    BlockEnd = BlockSize;
  }
  
  /*
  **   Need to normalize if inverse transform...
  */
    
  if ( InverseTransform ) {
    double denom = (double)NofSamples;
    
    for ( LSample=0; LSample < NofSamples; LSample++ ) {
      RealOut[LSample] /= denom;
      ImagOut[LSample] /= denom;
    }
  }
}



double p3d_fourier_inverse_transform(int Filter, double NofSamples, double * RealIn, double * ImagIn, double param)
{
  int LoopSample;
  double Result;
  double t;
  Result = *RealIn / 2.;
  for (LoopSample = 1; LoopSample < Filter; LoopSample ++) {
    RealIn++;ImagIn++;
    t = param * 2 * M_PI * LoopSample;
    Result += *RealIn * cos (t);
    Result += *ImagIn * sin (t);

  }
  Result = Result * 2. / (double)NofSamples;
  return Result;
}

double modulo (double Input, double Mod) 
{
  while (Input > Mod) {
    Input -= Mod;
  }
  return Input;
}

void anim_initialize_anim_options_default (void) 
{
  AnimOptions.AnimationActive    = 1;
  AnimOptions.SpeedProfiledSampling = 1;
  AnimOptions.InitPosRespect     = 1;
  AnimOptions.FinalPosRespect    = 1;
  AnimOptions.LinkLocalPath      = 0;
  AnimOptions.FourierNofCoefs    = 6;
  AnimOptions.FourierInitNofCoefs = 32;
  AnimOptions.RespectSpeedProfile = 1;
  AnimOptions.Extrapolation      = 0;
  AnimOptions.HumanApproxSpeed   = 1300;
  AnimOptions.AccelerationLength = 200.;
  AnimOptions.Reactif = 0;
  AnimOptions.ArmController = 0;

  AnimOptions.FramePerSecond = 25;
  AnimOptions.MaximumLinearSpeed = 1300.;
  AnimOptions.MaximumRotationSpeed = 1.;
  AnimOptions.MaximumAcceleration = 3000.;  
  AnimOptions.InfluenceDuration = 0.5;
  AnimOptions.NofOptims = 5;
  AnimOptions.ColBlockExtTime = 0.4;
  AnimOptions.ColAvNofAttemps = 100;
  AnimOptions.Optim1 = TRUE;
  AnimOptions.Optim2 = TRUE;
  AnimOptions.PosPlanParam = 100.;
}

void anim_initialize_anim_options (int a, int b, int c, int d, 
				   int e, int f, int g, int h,
				   double i, double j, int k, int l) 
{
  AnimOptions.AnimationActive    = a;
  AnimOptions.SpeedProfiledSampling = b;
  AnimOptions.InitPosRespect     = c;
  AnimOptions.FinalPosRespect    = d;
  AnimOptions.LinkLocalPath      = e;
  AnimOptions.FourierNofCoefs    = f;
  AnimOptions.RespectSpeedProfile = g;
  AnimOptions.Extrapolation      = h;
  AnimOptions.HumanApproxSpeed   = i;
  AnimOptions.AccelerationLength = j;
  AnimOptions.Reactif = k;
  AnimOptions.ArmController = l;
}



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
/***************************************************/
/*DECLARATION PROVISOIRE DES STRUCTURES ANIMATIONS */
/***************************************************/
/*TYPE*/

#define MAX_NOF_FRAMES 10000
#define MAX_NOF_DOFS 100
#define MAX_NOF_ANIM 10

#define MI_RZ 13
#define MI_RY 14
#define MI_RX 12
#define MI_Z 8
#define MI_Y 7
#define MI_X 6
#define MI_GRZ 11
#define MI_GRY 10
#define MI_GRX 9

#define FRAME_NOCOL -1
#define FRAME_AUTOCOL -2
#define FRAME_INVAL -3

typedef struct trajectory_characteristics {
  double AX, BX, AY, BY, AZ, BZ;
  double A, B, R, AngleDepart, AngleArrivee;
} anim_traj_charac;

typedef struct animation {
  char     name[255];
  char     type[255];
  char     when[255];
  anim_traj_charac tcharac;
  p3d_rob  * aim_robot;
  configPt * data;
  double ** fourier_real;
  double ** fourier_imag;
  double linear_speed;
  double rotation_speed;
  double length;
  int id;
  int nof_frames;
  int nof_coef;
  int index_theta;
  int framerate;
  char degree;
  char straight;
  char processed;
  char active;
} p3d_animation;

typedef struct {
  int AnimationActive;
  int SpeedProfiledSampling;
  int InitPosRespect;
  int FinalPosRespect;
  int LinkLocalPath;
  int FourierNofCoefs;
  int FourierInitNofCoefs;
  int RespectSpeedProfile;
  int Extrapolation;
  int ArmController;
  int Reactif;
  double HumanApproxSpeed;
  double AccelerationLength;

  int FramePerSecond;
  double MaximumLinearSpeed;
  double MaximumRotationSpeed;
  double MaximumAcceleration;  
  double InfluenceDuration;
  int NofOptims;
  double ColBlockExtTime;
  int ColAvNofAttemps;
  int Optim1;
  int Optim2;
  double PosPlanParam;
}AnimOptionsStruct;

AnimOptionsStruct AnimOptions;

typedef struct characteristics{
  double * TotalDist;
  double * FrameDist;
  double * AnimRelSpeed;
  double * WalkRelSpeed;
  double * LinSpeed;
  double * RotSpeed;
}p3d_localpath_characteristics;

#define NOFANIM 30
#define FRAMERATE 60.
#define HUMANAPPROXSPEED 1300.	/* m/s */

typedef struct bibliotheque {
  p3d_animation  * animation[NOFANIM];	/* PROVISOIRE . ALLOCATION DYNAMIQUE NECESSAIRE */
  int            nof_animation;
}p3d_animlib;

typedef struct frame_strip {
  struct frame_strip * next;
  int NumFrame;
  int NumChain;
}p3d_frame_list;

typedef struct joint_strip {
  struct joint_strip * next;
  int NumJoint;
  p3d_jnt * Joint;
}p3d_joint_list;

typedef struct AnimBuffer {
  configPt * AnimBuffer;
  int NofFrames;
}anim_buffer;

typedef struct charac{
  double TotalDist[MAX_NOF_FRAMES];
  double FrameDist[MAX_NOF_FRAMES];
  double AnimRelSpeed[MAX_NOF_FRAMES];
  double WalkRelSpeed[MAX_NOF_FRAMES];
  double LinSpeed[MAX_NOF_FRAMES];
  double RotSpeed[MAX_NOF_FRAMES];
  double Sens[MAX_NOF_FRAMES];
}Charac;

typedef struct SampleOptions {
  int EnableMaxAngularSpeed;
  int EnableMaxAcceleration;
  int FramePerSecond;
  double MaxAcceleration;
  double MaxAngularSpeed;
  double MaxLinearSpeed; 
}anim_sample_options;

typedef struct WalkOptions {
  int RespectEnd;
  int RespectInit;
  double RespectTime;
  int Filter;
}anim_walk_controller_options;

typedef struct ColBlock{
  int InitFrame;
  int EndFrame;
  int ColChain[100];
  int NofChain;
}anim_colblock;

typedef struct ColOptions {
  anim_colblock Blocks[50];
  int NofBlocks;
  double ColBlockExtTime;
  int NofAttempts;
}anim_reac_col_options;

typedef struct animprob {
  configPt Qi;
  configPt Qf;

  p3d_rob * Robot;
  p3d_rob * RobotMin;

  p3d_traj * FindTrajResult;

  p3d_traj * OptimizedResult;

  anim_sample_options SampleOptions;
  anim_buffer SamplingRes;

  Charac CharacRes;
  char TrajCharacterized;

  anim_walk_controller_options WalkOptions;
  anim_buffer WalkContRes;

  anim_reac_col_options ColOptions;
  anim_buffer ColAvoidRes;
  anim_buffer ColOpt1Res;
  anim_buffer ColOpt2Res;
  anim_buffer PosPlanRes;
  double FrameDuration;
  p3d_graph * InitRRTGraph;
  p3d_graph * FinalRRTGraph;
}AnimProb;

p3d_animlib XYZ_ANIM;

/***************************************************/
/***************************************************/

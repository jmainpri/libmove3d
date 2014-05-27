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

double euclidian_dist (const double x1, const double y1, const double z1, 
			      const double x2, const double y2, const double z2) {
  double Result;
  Result = sqrt ( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1) );
  return Result;
}

void anim_utils_destroy_buffer (p3d_rob * Robot, anim_buffer * Buffer) {
  int LFrame;
  for (LFrame = 0; LFrame < Buffer->NofFrames; LFrame ++) {
    p3d_destroy_config(Robot, Buffer->AnimBuffer[LFrame]);
  }
  if (Buffer->NofFrames > 0) {
    MY_FREE(Buffer->AnimBuffer, configPt *, Buffer->NofFrames);
  }
  Buffer->NofFrames = 0;
}

void anim_utils_copy_buffer (p3d_rob * Robot, anim_buffer * BufferSource, anim_buffer * BufferCible) {
  int LFrame;
  int NofFrames;
  if (BufferCible) {
    if (BufferCible->NofFrames > 0) {
      anim_utils_destroy_buffer (Robot, BufferCible);
    }
  }
  NofFrames = BufferSource->NofFrames;
  BufferCible->AnimBuffer = MY_ALLOC(configPt, NofFrames);
  for (LFrame = 0; LFrame < NofFrames; LFrame ++) {
    BufferCible->AnimBuffer[LFrame] = p3d_copy_config(Robot, BufferSource->AnimBuffer[LFrame]);
  }
  BufferCible->NofFrames = BufferSource->NofFrames;
}

void anim_selec_robot (p3d_animlib * FromLib, p3d_animlib * WhereLib, p3d_rob * RobotPt) {
  int LAnim;
  WhereLib->nof_animation = 0;
  for (LAnim = 0; LAnim < FromLib->nof_animation; LAnim ++) {
    if (FromLib->animation[LAnim]->aim_robot == RobotPt && FromLib->animation[LAnim]->active) {
      WhereLib->animation[WhereLib->nof_animation] = FromLib->animation[LAnim];
      WhereLib->nof_animation++;
    }
  }
}

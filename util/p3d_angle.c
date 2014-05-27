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

/*
 *  set an angle in degree between -180 and 180
 */

double angle_limit_180(double angle){

  while (angle < -180){
    angle += 360;
  }
  while (angle > 180){
    angle -= 360;
  }
  return angle;
}

/*
 *  set an angle in degree between 0 and 360
 */

double angle_limit_360(double angle){

  while (angle < 0){
    angle += 360;
  }
  while (angle > 360){
    angle -= 360;
  }
  return angle;
}

/*
 *  set an angle in radian between -PI and PI
 */

double angle_limit_PI(double angle){

  while (angle < -M_PI){
    angle += 2*M_PI;
  }
  while (angle > M_PI){
    angle -= 2*M_PI;
  }
  return angle;
}

/*
 *  set an angle in radian between 0 and 2 PI
 */

double angle_limit_2PI(double angle){

  while (angle < 0){
    angle += 2*M_PI;
  }
  while (angle > 2*M_PI){
    angle -= 2*M_PI;
  }
  return angle;
}

/* 
 *  Compute the shortest curvilinear distance between two points on
 *  the unit circle.
 */

double dist_circle(double theta1, double theta2)
{
  double dtheta = fabs(angle_limit_PI(theta2 - theta1));
  return dtheta;
}

/* 
 *  Compute the difference between two angles between -180 and 180
 */

double diff_angle(double theta1, double theta2)
{
  double dtheta = angle_limit_PI(theta2 - theta1);
  return dtheta;
}

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

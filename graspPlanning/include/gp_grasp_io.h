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
#ifndef GP_GRASP_IO_H
#define GP_GRASP_IO_H

#include <string>

//! @defgroup graspIO
//! @ingroup graspPlanning 
//! This module is used to read and write files (in XML format) containing a list of grasps computed
//! for a given object with a given hand.

//! @ingroup graspIO 
typedef struct gpElementParserData
{
  std::string object_name, hand_type, version, autoGen;
  int ID, fingerID, handID, body_index, face;
  double stability, quality, friction_coefficient, curvature;
  double position[3], normal[3], force_direction[3], baryCoords[3];
  double frame[4][4];
  std::list<double> configuration, open_configuration;
} gpElementParserData;


//! @ingroup graspIO 
typedef struct gpContactParserData
{
  double friction_coefficient, curvature;
  double position[3], normal[3], force_direction[3], baryCoords[3];
  int fingerID, face;
} gpContactParserData;

//! @ingroup graspIO 
typedef struct gpGraspParserData
{
  bool autoGen;
  std::string object_name;
  gpHand_type hand_type;
  int body_index;
  int ID, handID;
  double stability, quality;
  double frame[4][4];
  std::list<gpContact> contacts;
  std::list<double> configuration, open_configuration;
} gpGraspParserData;


extern int gpSave_grasp_list(std::list<class gpGrasp> &graspList, std::string filename);

extern int gpLoad_grasp_list(std::string filename, std::list<class gpGrasp> &graspList);

extern int gpCheck_grasp_list_validity(std::list<class gpGrasp> &graspList, std::string objectName);

extern int gpInvert_axis(std::string inputFile, std::string outputFile, p3d_matrix4 T);

extern int gpMirror_robot(p3d_rob *robot, int axis);

extern int gpMirror_robot_bodies(p3d_rob *robot, std::string path, int axis);

#endif


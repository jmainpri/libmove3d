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

#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

#include "p3d.h"
#include "device.h"

/* Structure d'environnement */
typedef struct env {
  char      *name;
  char      *p3d_file_path; /* path to the loaded p3d file */
  char      *sce_file_path; /* path to the loaded sce file */
  int       num;
  int       nof_objs; /* total number of objects, both obstacles and robot bodies */
  int       no; /* number of obstacles */
  int       nr; /* number of robots */
  p3d_obj   **o,*ocur;  /* the obstacles*/
  p3d_rob   **robot,    /* array of robots for multi-robot */
    *cur_robot,         /* pointer to current robot */
    *active_robot;      /* pointer the active robot (this should not change durring env life)  */
  p3d_box   box;
  int INIT; /* environnement initialise graphiquement ou pas */
  double dmax; /* pas de discretisation maximal */
  double object_tolerance; /* real object tolerance (to enlarge objects)*/
  double graphic_dmin; /* pas de discretisation graphique */
  /*  struct graph *graphPt; */
  int      modif; /* TRUE if environment recently modified, FALSE otherwise */
  int   envBoxIsUpToDate; /* SDK: TRUE if the environment box (box field in
			     this struct) was computed after last modification
			     of environment, FALSE otherwise */
  int   ccIsUpToDate;     /* SDK: TRUE if a collision checker was initialized
			     after last modification of environment,
			     FALSE otherwise */
  int nof_new_obsts;      /* number of added obstacles after latest
			     (re)initialization or update of collision checker */
  int *new_obsts;         /* array of length nof_new_obsts, for each new obstacle
			     since latest (re)initialization or update of collision
			     checker we store the ranknumber in list p3d_obj **o
			     for that obstacle */
  struct p3d_statistics * stat;  /* Global Environment statistics ; Commit Jim ; date: 01/10/2008 */

  //! color that will be used by glClearColor function
  double background_color[3];

  //! a point cloud (e.g. the output of a 3D scanner), that can be used to test collision with the robots
  p3d_vector3 *collisionCloud;
  int cloudSize;
} p3d_env,*pp3d_env;

#endif /* #ifndef _ENVIRONMENT_H */



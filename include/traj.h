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
#ifndef _TRAJ_H
#define _TRAJ_H
/* Structure de trajectoire */
typedef struct traj {
  char       *name;       /* nom de la courbe */
  char       *file;       /* nom du fichier de la trajectoire */
  int        id;          /* identificateur unique de la trajectoire, 
                             set by p3d_end_traj() */ /* Carl */
  int        sdk_id;      /* identifier of trajectory in sdk. Might be the
			     same as id ... */
  int        num;         /* numero de trajectoire */
  struct rob *rob;        /* robot */
  int        nlp;         /* number of local paths */
  double     range_param; /* range of parameter along the trajectory */
  struct localpath *courbePt;
#ifdef DPG
  bool isOptimized;
  int savelpNum;
  struct localpath *trajInGraph;
#endif
} p3d_traj,*pp3d_traj;


#endif /* ifndef _TRAJ_H */

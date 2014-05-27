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
/****************************************************************************/
/*!
 *  \file collision_context.h
 *
 *    Collision-context structure definition.
 *
 *    This structure must be able to save and change quickly the 
 * selection of collision detection that must be done.
 */
/****************************************************************************/

#ifndef collision_context_h
#define collision_context_h

/*---------------------------------------------------------------------------
 * Collision pair structures.
 */

/*!
 * \brief Collision-pair structure definition.
 *
 * This structure is able to save the selection of objects which
 * are tested for collision.
 *
 * \note This structure affect kcd_table.c and p3d_BB.c
 *
 * \warning VCollide is not taken into account.
 */
typedef struct p3d_collision_pair {
  /*! \brief The Kcd table for the  selection of objects which
   *         are tested for collision 
   */
  struct kcd_col_handle * kcd_handlePt;

  /*! \brief The list of the selection of objects pair which
   *         are tested for the distances 
   */
  struct BB_handle * BB_handlePt;

} p3d_collision_pair;


/*!
 * \brief Collision-contexts.
 *
 * This structure is list of collision pair with parameters. It allows
 * opperations on list of parametrized collision pair.
 */
typedef struct p3d_collision_context {
  
  /*! \brief The collision pair for the parameter */
  p3d_collision_pair * collision_pair;
 
  /*! \brief The pointer to the parameter.
   *
   * Note: The parameter must be unique, for this we use pointer to 
   *       structure that qualify the object. For example, we could use
   *       \a config_namePt for robot or multi-robot. But after we have 
   *       to not destroy this config.
   */
  void * param;

  /*! \brief Next element of the list */
  struct p3d_collision_context * next;

} p3d_collision_context;

#endif

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

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
/* ---------------------------------------------------------------------*/
/*! \file elastic.h
 * \brief structure for elastic band
 * 
 * \author E.Ferre
 * \date   Aug 2001
 */

#ifndef _ELASTIC_H
#define _ELASTIC_H



/*! \struct StripPoint
\brief cell of an elastic strip */

typedef struct StripPoint{
  double*                 q; /*!< control configuration */
  struct StripPoint   *prev; /*!< pointer to the previous strippoint */
  struct StripPoint   *next; /*!< pointer to the next strippoint */
  struct localpath    *path; /*!< local path which link the strippoint to the previous one */
  double                Kes; /*!< scaling factor for internal force */
}p3d_strippoint;

/*! \struct Strip
\brief list of strippoints */

typedef struct Strip{
  struct rob        *rob; /*!< the robot */
  struct StripPoint  *sp; /*!< pointer to the first strippoint */
} p3d_strip;

#endif

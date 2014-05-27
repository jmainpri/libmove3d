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
#ifndef FORM_JOINT_LIG_H
#define FORM_JOINT_LIG_H

#ifndef PROTO
#include "forms.h"
#endif

typedef struct {
  FL_FORM *CURRENT_FORM;
  FL_OBJECT *CHOICE;
  FL_OBJECT *BROWSER_ATOMS;
  FL_OBJECT *BROWSER_DIHEDRE;
  FL_OBJECT *FRAME;
  FL_OBJECT *ADD_ATOM[4];
  FL_OBJECT *INPUT_ATOM[4];
  FL_OBJECT *INPUT_VVALUES[2];
  FL_OBJECT *ADD;
  FL_OBJECT *REMOVE;
  FL_OBJECT *FRAME_SAVE_LOAD;
  FL_OBJECT *CHECK_SAVE;
  FL_OBJECT *CHECK_LOAD;
  FL_OBJECT *INPUT_SAVE;
  FL_OBJECT *INPUT_LOAD;
  FL_OBJECT *BROWSE_SAVE;
  FL_OBJECT *BROWSE_LOAD;
  FL_OBJECT *VALIDATE;
  FL_OBJECT *CANCEL;
} JOINT_LIG_FORM;

#endif

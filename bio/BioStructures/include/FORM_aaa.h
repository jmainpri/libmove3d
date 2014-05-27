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
#ifndef FORM_AAA_H
#define FORM_AAA_H

#ifndef PROTO
#include "forms.h"
#endif

#define RIGID_COLOR "@C0" // Black
#define MOBILE_COLOR "@C4" // Blue 

#define MAX_LINE_LENGTH 80

typedef struct {
  FL_FORM *CURRENT_FORM;
  FL_OBJECT *CHOICE;
  FL_OBJECT *BROWSER;
  FL_OBJECT *FRAME;
  FL_OBJECT *TOGGLE_BKB;
  FL_OBJECT *TOGGLE_SCH;
  FL_OBJECT *TOGGLE_ALL_BKB;
  FL_OBJECT *TOGGLE_ALL_SCH;
  FL_OBJECT *FRAME_SAVE_LOAD;
  FL_OBJECT *CHECK_SAVE;
  FL_OBJECT *CHECK_LOAD;
  FL_OBJECT *INPUT_SAVE;
  FL_OBJECT *INPUT_LOAD;
  FL_OBJECT *BROWSE_SAVE;
  FL_OBJECT *BROWSE_LOAD;
  FL_OBJECT *VALIDATE;
  FL_OBJECT *CANCEL;
} AAA_FORM;

#endif

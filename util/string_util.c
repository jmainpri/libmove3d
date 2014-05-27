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
 *  \file string_util.c
 *
 *  \brief String utilities
 *
 *  These functions are used to manage strings.
 */
/****************************************************************************/

#include "Util-pkg.h"

// strdup is not a standard option, it can be avalaible or not depending on compilers/options
#ifndef __cplusplus
char *strdup(const char *str) {
  char *dup = MY_ALLOC(char, strlen(str) + 1);
  if(dup)
  { strcpy(dup, str); }
  return dup;
}
#endif

/****************************************************************************/
/****************************************************************************/
/* Fixed length strings functions
 */


/*--------------------------------------------------------------------------*/
/*! \brief Concat a string to a fixed sized string
 *
 * \param dest:   the string to copy into
 * \param src:    string that must be copied
 * \param indice: where the string must be copied
 * \param size:   the size of \a dest
 * 
 * \retval indice: the lenght of the string
 *
 * \return FALSE if \a src could not be copied
 *
 * \note if (* \a indice) == 0 then this function is similar to
 *       strncpy
 * \note if (* \a indice) == length of \a dest then this function is
 *       similar to strlcat
 */
int fixed_str_copy(char * dest, const char * src, int * indice, int size)
{
  int i;

  i = 0;
  while(((*indice)+1<size) && (src[i] != '\0')) {
    dest[(*indice)] = src[i];
    (*indice) ++;
    i ++;
  }
  dest[(*indice)] = '\0';
  if (src[i] != '\0') {
    PrintInfo(("!!! WARNING fixed_str_copy: line too long !!!\n"));
    return FALSE;
  }
  return TRUE;
}


/****************************************************************************/
/****************************************************************************/
/* Dynamic length strings functions
 */

/*--------------------------------------------------------------------------*/
/*! \brief Copy and allocate the memory for a string.
 *
 *  \param  data: the string that must be copied
 *
 *  \return The new string.
 */
char * dyna_str_dup(const char * data)
{
  char * dest;
  int   size;

  if (data==NULL) {
    PrintWarning(("!!! WARNING: dyna_str_dup: NULL pointer ...\n"));
    return(NULL);
  }

  size = strlen(data)+1;

  dest = MY_ALLOC(char, size);
  
  if (dest == NULL) 
    { return NULL; }

  memcpy(dest, data, size);

  return dest;
}


/*--------------------------------------------------------------------------*/
/*! \brief Release the memory owned by a dynamic string.
 *
 *  \param  data: the string.
 *
 *  \warning This function assumes that the memory allocated is given 
 *           by the size of the string.
 */
void dyna_str_free(char * data)
{
  int s = 0;

  if (data != NULL) {
    s  = strlen(data)+1;
    MY_FREE(data, char, s);
  }
}

/****************************************************************************/

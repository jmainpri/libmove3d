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

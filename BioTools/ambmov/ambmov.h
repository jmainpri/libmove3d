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
#include "basics.h"
#include "vector.h"
#include "matrix.h"
#include "classes.h"
#include "dictionary.h"
#include "database.h"
#include "library.h"
#include "parmLib.h"
#include "pdbFile.h"
#include "help.h"
#include "parser.h"
#include "tools.h"
#include "amber.h"
#include "commands.h"
#include "defaults.h"
#include "leap.h"
#include "octree.h"
#include "tripos.h"
#include "block.h"
#include "leap.h"
#include "getline.h" 
#include "stringExtra.h"
#define NB_OF_CHAR 200 
#define NB_OF_LIB  4

typedef struct s_PARAMET {

 double *px;
 int *pix;
 STRING *pih;
 int *pipairs;
 double *pr_stack;
 int *pi_stack;

} PARAMET;

extern struct{

  int nrp,nspm,ig;
  int ntx,ntcx,ntxo,ntt,ntp,ntr,init,ntcm,nscm;
  int isolvp,nsolut,ntc,ntcc,ntf,ntid,ntn,ntnb,nsnb,ndfmin;
  int nstlim,nrc,ntrx,npscal,imin,maxcyc,ncyc,ntmin;
  int irest,jfastw,ibgwat,ienwat,iorwat;

} mdi_;


extern struct{

  double t,dt,temp0,tautp,pres0,comp,taup,temp,tempi;
  double tol,taur,dx0,drms,timlim,timtot,timdel;
  double timrun,timsts,vlimit,rbtarg,tmass,tmassinv;
  double kappa,offset,surften,gamma_ln,extdiel,intdiel,rdt;
  double gbalpha,gbbeta,gbgamma,cut_inner,clambda,saltcon;
  double solvph,rgbmax,fsmax,restraint_wt;

} mdr_;


extern struct{

  char *restraintmask;
  char *bellymask;
  char *tgtfitmask;
  char *tgtrmsmask;
  char *iwtnm;
  char *iowtnm;
  char *ihwtnm[2];

} mds_;

/*
extern struct{

  STRING restraintmask;
  STRING bellymask;
  STRING tgtfitmask;
  STRING tgtrmsmask;
  STRING iwtnm;
  STRING iowtnm;
  STRING ihwtnm[2];

} mds_;
*/

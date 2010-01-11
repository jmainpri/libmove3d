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

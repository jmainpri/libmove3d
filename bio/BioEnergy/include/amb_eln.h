// Description: 
//    This file contains the methods declaration for all the external methods (from Amber and ElNemo)
//    External here may mean (until further notice): a) defined in our Fortran code or b) taken from an external library


#include <stringExtra.h>
#include <basics.h>


// read_inp_, read_parm_, min_energy_ , diagrtb_ -- FORTRAN function
/*
 *     read_inp.f 
 */

extern void readmemorysizesfromparmfile_( int *lastr,int *lasti,int *lasth,int *lastpr, int *lastrst,int *lastist, char *name_first, int *len_name, int *iconst);
extern   void writecoord_(double *x_coo, int *number_att); 


/*
 *     read_parm.f 
 */
extern void readfromparmfile_(double *px, int *pix, STRING *pih, int *pipairs,double *pr_stack,int *pi_stack);
/*
 *     mini.f   
 */
extern void fortrancodeenergyminimisation_(double *px_coo, double *px, int *pix, STRING *pih, int *pipairs,
			double *pr_stack,int *pi_stack,double *penergy, double *pm_coo, int *pnumber_at);

/*
 *     diagrtb.f
 */
extern void  diagrtb_(char *nompdb, int *lnompdb, double *coo_x, int *pnumber_at,
		      double*vec_x,int*pnvec);




// ParseInit, Basics, Destroy  -- functions of AMBER in AMBER_SRC/leap/src/leap
extern void    ParseInit( RESULTt *rPResult );
extern void    BasicsInitialize();
extern void Destroy( OBJEKT  *oPObject);



// extern structures from AMBER correspond to FORTRAN common 
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


/* structures */

typedef struct s_PARAMET {

 double *px;
 int *pix;
 STRING *pih;
 int *pipairs;
 double *pr_stack;
 int *pi_stack;

} PARAMET;



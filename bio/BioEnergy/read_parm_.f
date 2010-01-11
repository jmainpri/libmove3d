


!++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+ Subroutine READ_PARM 
!+         reads molecular topology and force field parameters 
!+         after memory allocation 
!  
!+   It's small part of subroutine sander modified by S.Kirillova
!+   SANDER in AMBER_SRC/sander/sander.f  --  
!+          call subroutines to run MD or minimization calculations.
!+                                       
!++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

subroutine readfromparmfile (x,ix,ih,ipairs,r_stack,i_stack)

!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+  x, ix, ih , ipairs, r_stack, i_stack -- variables for determination of 
!+                                          molecular topology and force field
!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   implicit none

   logical belly, erstop
   integer ier,ifind,jn,ncalls
   character(len=4) itest
   logical ok
!+ Specification and control of Amber's Input/Output


! File names
character(len=512) groupbuffer
character(len=256) mdin, mdout, inpcrd, parm, restrt, &
      refc, mdvel, mden, mdcrd, mdinfo, nmr, mincor, &
      vecs, radii, freqe,redir(8), &
      rstdip,mddip,inpdip,groups,gpes, &
      cpin, cpout, cprestrt &
!cnt N.TAKADA: For MDM
!cnt N.TAKADA: For MDM
; ! line terminator for free-form version

character owrite
common /files/ groupbuffer, mdin, mdout, inpcrd, parm, restrt, &
      refc, mdvel, mden, mdcrd, mdinfo, nmr, mincor, &
      vecs, radii, freqe, owrite, &
      rstdip,mddip,inpdip,groups,gpes, &
      cpin, cpout, cprestrt &
!cnt N.TAKADA: For MDM
!cnt N.TAKADA: For MDM
; ! line terminator for free-form version

! put this in a seperate common block to stop the compiler from
! complaining about misalignment
integer numgroup
common/nmgrp/ numgroup


! File units
! An I/O Unit resource manager does not exist.
integer     MDCRD_UNIT
integer     MDEN_UNIT
integer     MDINFO_UNIT
integer     MDVEL_UNIT
parameter ( MDINFO_UNIT =  7 )
parameter ( MDCRD_UNIT  = 12 )
parameter ( MDEN_UNIT   = 15 )
parameter ( MDVEL_UNIT  = 13 )
integer, parameter :: CNSTPH_UNIT = 18, CPOUT_UNIT = 19

! 18 was picked because CNSTPH uses it; conflicts are not expected.
integer     MMTSB_UNIT
parameter ( MMTSB_UNIT = 18 )


! File related controls and options
character(len=80) title,title1
common/runhed/ title, title1

logical mdin_ewald,mdin_pb &


; ! line terminator for free-form version

common/mdin_flags/mdin_ewald,mdin_pb &






; ! line terminator for free-form version

integer BC_HULP  ! size in integers of common HULP
parameter ( BC_HULP = 9 )

integer     ntpr,ntwr,ntwx,ntwv,ntwe,ntpp,ioutfm,ntwprt,ntave &



; ! line terminator for free-form version
common/hulp/ntpr,ntwr,ntwx,ntwv,ntwe,ntpp,ioutfm,ntwprt,ntave &




; ! line terminator for free-form version

!      NMRRDR : Contains information about input/output file redirection
!               REDIR and IREDIR contain information regarding
!               LISTIN, LISTOUT, READNMR, NOESY, SHIFTS, DUMPAVE,
!               PCSHIFT and DIPOLE respectively. If IREDIR(I) > 0,
!               then that input/output has been redirected.

integer iredir(8)
common/nmrrdr/redir,iredir

!  --- following should not need to be modified unless you are adding
!      more variables to the "locmem" style of memory management

!       BC_MEMORY is the size of the MEMORY common block:
! MFC changing indices into IX and X to more rational names
!       I12 = Iibh
!       I14 = Ijbh
!       I16 = Iicbh
!       I18 = Iiba
!       I20 = Ijba
!       I22 = Iicba

integer       natom,nres,nbonh,nbona,ntheth,ntheta,nphih, &
      nphia,nnb,ntypes,nconp,maxmem,nwdvar,maxnb,nparm, &
      natc,nattgtfit,nattgtrms,ibelly,natbel,ishake,nmxrs, &
      mxsub,natyp,npdec,i02,i04,i06,i08,i10, &
      iibh,ijbh,iicbh,iiba,ijba,iicba, &
      i24,i26,i28,i30,i32,i34,i36,i38,i40, &
      i42,i44,i46,i48,i50,i52,i54,i56,i58,ibellygp, &
      icnstrgp,itgtfitgp,itgtrmsgp,i64,i65,i66,i68, &
      i70,i72,i74,i76,i78,i79,i80,i82,i84,i86, &
      icpstinf,icpresst,icptrsct, icpptcnt, &
      l05,l10,l15,lwinv,lpol,lcrd,lforce,l36,lvel,lvel2,l45,l50, &
      lcrdr,l60,l65,lmass,l75,l80,l85,l90,l95,l96,l97,l98,lfrctmp, &
      l105,l110,l115,l120,l125,l130,l135,l140,l145,l150, &
      l165,l170,l175,l180,l185,l186,l187,l188,l189,l190, &
      lcpcrg,lcpene, &
      m02,m04,m06,m08,m10,m12,m14,m16,m18,i01, &
      iifstwt,iifstwr,nrealb,nintb,nholb,npairb,lastr,lasti,lasth, &
      lastpr,lastrst,lastist,nbper,ngper,ndper,ifpert,lpolp, ncopy, &
      imask1,imask2,numadjst,mxadjmsk

! 1     2      3      4      5      6      7      8      9      10
common/memory/ &
 natom ,nres  ,nbonh ,nbona ,ntheth,ntheta,nphih ,                       & ! 7
 nphia ,nnb   ,ntypes,nconp ,maxmem,nwdvar,maxnb ,nparm ,                & !15
 natc  ,nattgtfit,nattgtrms ,ibelly,natbel,ishake,nmxrs ,                & !22
 mxsub ,natyp ,npdec ,i02   ,i04   ,i06   ,i08   ,i10,                   & !30
 iibh  ,ijbh  ,iicbh ,iiba  ,ijba  ,iicba ,                              & !36
 i24   ,i26   ,i28   ,i30   ,i32   ,i34   ,i36   ,i38   ,i40   ,         & !45
 i42   ,i44   ,i46   ,i48   ,i50   ,i52   ,i54   ,i56   ,i58   ,ibellygp,& !55
 icnstrgp,itgtfitgp,itgtrmsgp,i64  ,i65   ,i66   ,i68   ,                & !62
 i70   ,i72   ,i74   ,i76   ,i78   ,i79   ,i80   ,i82   ,                & !70
 i84   ,i86   ,                                                          & !72
 icpstinf,icpresst,icptrsct, icpptcnt,                                   & !76
 l05   ,l10   ,l15   ,lwinv ,lpol  ,lcrd  ,lforce,l36   ,lvel  ,lvel2 ,  & !86
 l45   ,l50   ,                                                          & !88
 lcrdr ,l60   ,l65   ,lmass ,l75   ,l80   ,l85   ,l90   ,l95   ,l96   ,  & !98
 l97   ,l98   ,lfrctmp,                                                  & !101
 l105  ,l110  ,l115  ,l120  ,l125  ,l130  ,l135  ,l140  ,l145  ,l150  ,  & !111
 l165  ,l170  ,l175  ,l180  ,l185  ,l186  ,l187  ,l188  ,l189  ,l190  ,  & !121
 lcpcrg,lcpene,                                                          & !123
 m02   ,m04   ,m06   ,m08   ,m10   ,m12   ,m14   ,m16   ,m18   ,i01   ,  & !133
 iifstwt,iifstwr,nrealb,nintb,nholb,npairb,lastr ,lasti ,lasth ,         & !142
 lastpr ,lastrst,lastist,nbper,ngper,ndper,ifpert,lpolp ,ncopy,          & !151
 imask1 ,imask2 ,numadjst,mxadjmsk                                         !155
!+ Specification and control of Amber's working precision


! Description:
! Preprocessor directives that characterize the floating-point
! working precision as single or double precision.
! The current scheme guarantees internal consistency at the expense
! of flexibility.  A need for flexibility has yet to appear.
! The preprocessor guard  prevents multiple, and thus
! inconsistent, definitions.
! The default working precision is double precision.
! User control of the working precision at build time should be
! exercised via the preprocessor name _REAL_.
! To build a single precision Amber use
!     make -e AMBERBUILDFLAGS=' -D_REAL_ '
! The preprocessor names that characterize the precision are

!   _REAL_     precision type specifier.
!              Use  _REAL_ foo  as the precision independent
!              notation for  double precision foo  and  real foo.

!   AMBER_MPI_REAL
!              MPI precision type specifier.
!              Use AMBER_MPI_REAL as the precision independent
!              notation for MPI_DOUBLE_PRECISION and MPI_REAL.

!   D_OR_S()   precision prefix for the BLAS and LAPACK Library routines.
!              Use, e.g.,  D_OR_S()axpy(...)  as the precision independent
!              notation for daxpy(...) and saxpy(...).

!   DPREC      defined when the working precision is double;
!              undefined when the working precision is single.

!   VD_OR_VS() precision prefix for the Intel Vector Math Library routines.
!              Use, e.g.,  VD_OR_VS()exp(...)  as the precision independent
!              notation for vdexp(...) and vsexp(...).

!   WIDE_REAL  defined when a platform's single precision is wider than the
!              IEEE 754 single precision (32-bit).  This is the case on Cray
!              machines.  Note that on these machines _REAL_ is defined to
!              double precision, and the compiler flag -dp is used to disable
!              double precision.  The result is consistent 64-bit working
!              precision.

! History:
! $Id: dprec.h,v 7.6 2003/06/14 00:47:00 sbrozell Exp $

! Code Description:
!   Languages:          C Preprocessor and Fortran 90.
!   Software Standards: "European Standards for Writing and
!     Documenting Exchangeable Fortran 90 Code":
!     http://nsipp.gsfc.nasa.gov/infra/eurorules.html

! References:
!   IEEE 754: Standard for Binary Floating-Point Arithmetic.
!     http://grouper.ieee.org/groups/754/

!   The unused Fortran 90 module numerics.f90.





! 


!-------------BEGIN    nmr.h  ------------------------------------------------

!  ---Header file for the chemical shifts and NOESY intensity

!     Because of the complexity of the storage requirements for
!     these calculations, (and because one of the authors is very
!     lazy,) storage allocation for these is not done in the
!     LOCMEM routine, but rather arranged at compile time through
!     the information given below.

!     If you do not plan to make use of this section of the code,
!     set the parameters below to small numbers to avoid allocating
!     space unnecessarily.  When/if you change your mind, reset the
!     parameters and re-compile.

!     Input parameters (things you have to set):

!     MATOM = max # of atoms in the system
!     MXR = max # residues
!     MA = max # of protons in any sub-molecule
!     MXTAU = max number of mixing times
!     MXP = max number of input intensities (peaks) per mixing time
!     MTOT = max. # total peaks, all mixing times, all sub-molecules
!     MXVAR = max. # of "extra" dynamic variables
!     MRING = max. # of rings for chemical shift calculation
!     MSHF = max # of protons whose shifts are to be calculated
!     MAXDIP = max # of dipolar interactions

integer mring,mshf,mxvar,matom,ma,ma2,lst,mxr,mxtau,mxp, &
      mtot,maxdip,maxdipsets


!   --- here are some "standard" suggestions for NMR properties:

! parameter (mring=40)
! parameter (mshf=800)
! parameter (mxvar=36)
! parameter (matom=1000)
! parameter (ma=500)
! parameter (ma2=ma*ma)
! parameter (lst=(ma2+ma)/2)
! parameter (mxr=100)
! parameter (mxtau=2)
! parameter (mxp=1500)
! parameter (mtot=3000)
! parameter (maxdip=2000)
! parameter (maxdipsets=5)

!  --- "standard" parameters for jobs *not* doing NOESY, alignment, or
!          chemical shift-based refinements:

parameter (mring=50)
parameter (mshf=500)
parameter (mxvar=10)
parameter (matom=5000)
parameter (ma=1)
parameter (ma2=ma*ma)
parameter (lst=(ma2+ma)/2)
parameter (mxr=300)
parameter (mxtau=1)
parameter (mxp=1)
parameter (mtot=1)
parameter (maxdip=2000)
parameter (maxdipsets=2)

integer isubi,isubr
parameter (isubi=8 + 3*ma + 2*matom + mxtau + 2*mxtau*mxp + mxp)
parameter (isubr=3*ma + mxtau + 3*mxtau*mxp + 4)


integer peakid(mxp)
double precision tau(ma),pop(ma),popn(ma),emix(mxtau), &
      aexp(mxtau,mxp),awt(mxtau,mxp),arange(mxtau,mxp),oscale, &
      omega,taumet,taurot,invwt1,invwt2
integer nath,natmet,nummt,id2o,iroesy,ihet,nvect,ihsful,m2(ma), &
      inn(ma),ihyp(ma),ihyd(matom),inatm(matom),npeak(mxtau), &
      ihp(mxtau,mxp),jhp(mxtau,mxp)
common /methylr/ tau,pop,popn,emix, &
      aexp,awt,arange,oscale, &
      omega,taumet,taurot,invwt1,invwt2
common /methyli/ &
      nath,natmet,nummt,id2o,iroesy,ihet,nvect,ihsful,m2, &
      inn,ihyp,ihyd,inatm,npeak, &
      ihp,jhp,peakid

!    Parameters for parallel broadcast

integer BC_METHYLR,BC_METHYLI,BC_ALIGNR,BC_ALIGNI
parameter(BC_METHYLR=3*ma+mxtau+3*mxtau*mxp+6)
parameter(BC_METHYLI=8 + 3*ma + 2*matom + mxtau + 2*mxtau*mxp)
parameter(BC_ALIGNR=5*maxdip + 1 + 6*maxdipsets)
parameter(BC_ALIGNI=3*maxdip + 3)

integer nmropt,iprint,noeskp,iscale,ipnlty,iuse,maxsub
double precision scalm,pencut,ensave,tausw,ebdev,eadev
common/nmr1/scalm,pencut,ensave,tausw,ebdev,eadev, &
      nmropt,iprint,noeskp,iscale,ipnlty,iuse,maxsub

character(len=14) resat(matom)
common/nmr2/ resat

double precision dobsu(maxdip),dobsl(maxdip),dcut,gigj(maxdip), &
      dij(maxdip),dwt(maxdip), &
      s11(maxdipsets),s12(maxdipsets),s13(maxdipsets), &
      s22(maxdipsets),s23(maxdipsets),s33(maxdipsets)
integer ndip,num_datasets,id(maxdip),jd(maxdip),dataset(maxdip), &
      ifreeze
common/align/dobsu,dobsl,dcut,gigj,dij,dwt, &
      s11,s12,s13,s22,s23,s33, &
      ndip,id,jd,dataset,num_datasets,ifreeze


integer mxvect
parameter (mxvect=1)


! Common block containing variables relating to nmr restraints.

integer       intreq,irlreq,lnmr01,inmr02,iprr,iprw
common/nmrstf/intreq,irlreq,lnmr01,inmr02,iprr,iprw

integer ntot,ntota,ipmix(mtot),ntotb
double precision calc(mtot),exper(mtot),calca(mtot), &
      expera(mtot),calcb(mtot),experb(mtot)
common/correl/ntot,ntota,calc,exper,calca, &
      expera,calcb,experb,ipmix,ntotb

double precision        wnoesy,wshift,enoe,eshf,epcshf,ealign
common/wremar/wnoesy,wshift,enoe,eshf,epcshf,ealign

!-------------END      nmr.h  ------------------------------------------------

!     common block sizes:

integer bc_boxi,bc_boxr
parameter(bc_boxi=7)
parameter(bc_boxr=613)

! ... floats:

!+ Specification and control of Amber's working precision


double precision box,cut,scnb,scee,dielc,rad,wel,radhb,welhb, &
      cutcap,xcap,ycap,zcap,fcap,rwell
common/boxr/box(3),cut,scnb,scee,dielc, &
      cutcap,xcap,ycap,zcap,fcap,rwell, &
      rad(100),wel(100),radhb(200),welhb(200)

! ... integers:

integer ntb,ifbox,numpk,nbit,ifcap,natcap,isftrp

common/boxi/ntb,ifbox,numpk,nbit,ifcap,natcap,isftrp
!+ Specification and control of Amber's working precision


!-------------BEGIN    md.h  ------------------------------------------------
integer numsts,bc_mdi,bc_mdr
parameter (numsts=30)
parameter (BC_MDI=54)
parameter (BC_MDR=45+numsts)

! ... integer variables:

integer nrp,nspm,ig,ntx,ntcx,           &!5
      ntxo,ntt,ntp,ntr,init,             &!10
      ntcm,nscm,isolvp,nsolut,klambda,   &!15
      ntc,ntcc,ntf,ntid,ntn,             &!20
      ntnb,nsnb,ndfmin,nstlim,nrc,       &!25
      ntrx,npscal,imin,maxcyc,ncyc,      &!30
      ntmin,irest,jfastw,                &!33
      ibgwat,ienwat,iorwat,              &!36
      iwatpr,nsolw,igb,iyammp,           &!40
      gbsa,vrand,iwrap,nrespa,irespa,nrespai,icfe,  &!47
      rbornstat,ivcap,iconstreff,        &!50
      idecomp,icnstph,ntcnstph,maxdup     !54

common/mdi/nrp,nspm,ig, &
      ntx,ntcx,ntxo,ntt,ntp,ntr,init,ntcm,nscm, &
      isolvp,nsolut,ntc,ntcc,ntf,ntid,ntn,ntnb,nsnb,ndfmin, &
      nstlim,nrc,ntrx,npscal,imin,maxcyc,ncyc,ntmin, &
      irest,jfastw,ibgwat,ienwat,iorwat, &
      iwatpr,nsolw,igb,iyammp,gbsa,vrand, &
      iwrap,nrespa,irespa,nrespai,icfe,rbornstat, &
      ivcap,iconstreff,idecomp,klambda,icnstph,ntcnstph,maxdup

! ... floats:

double precision t,dt,temp0,tautp,pres0,comp,taup,temp,tempi, & !9
      tol,taur,dx0,drms,timlim,timtot,timdel, & !16
      timrun,timsts(numsts),vlimit,rbtarg(8),tmass,tmassinv, & !28 +numsts
      kappa,offset,surften,gamma_ln,extdiel,intdiel,rdt, & !35 +numsts
      gbalpha,gbbeta,gbgamma,cut_inner,clambda,saltcon, & !41 +numsts
      solvph,rgbmax,fsmax,restraint_wt                   !45 +numsts
common/mdr/t,dt,temp0,tautp,pres0,comp,taup,temp,tempi, &
      tol,taur,dx0,drms,timlim,timtot,timdel, &
      timrun,timsts,vlimit,rbtarg,tmass,tmassinv, &
      kappa,offset,surften,gamma_ln,extdiel,intdiel,rdt, &
      gbalpha,gbbeta,gbgamma,cut_inner,clambda,saltcon, &
      solvph,rgbmax,fsmax,restraint_wt 


! ... strings:

character(len=4) iwtnm,iowtnm,ihwtnm
character(len=256) restraintmask,bellymask,tgtfitmask,tgtrmsmask
common/mds/ restraintmask,bellymask,tgtfitmask,tgtrmsmask,  &
            iwtnm,iowtnm,ihwtnm(2)

!-------------END    md.h  ------------------------------------------------


!    BC_PARMR is the number of reals in common RPARMS; BC_PARMI is the
!    number of ints in IPARMS.  (Change these if you change the sizes below).


double precision rk(5000),req(5000),tk(900),teq(900),pk(1200), &
      pn(1200),phase(1200),cn1(1830),cn2(1830),solty(60), &
      gamc(1200),gams(1200),fmn(1200), &
      asol(200),bsol(200),hbcut(200)
common/rparms/rk,req,tk,teq,pk, &
      pn,phase,cn1,cn2,solty, &
      gamc,gams,fmn, &
      asol,bsol,hbcut

integer ipn(1200)
common/iparms/ipn


! NPHB is the number of h-bond parameters. NIMPRP is the number of
! improper torsional parameters (NPTRA-NIMPRP is the number of regular
! torsional parameters).

integer       numbnd,numang,nptra,nphb,nimprp
common/prmlim/numbnd,numang,nptra,nphb,nimprp
!+ Specification and control of Amber's working precision


!  dec cpp requires this comment line to avoid losing beginning spaces

integer ilbopt, ilbnob, lbverb, lbfreq, dozero
common/extra_int/ilbopt, ilbnob, lbverb, lbfreq, dozero

double precision lbwght
common/extra_real/lbwght

logical master
common/extra_logical/master

!     note: if this common block if changed, dont forget to
!     update the initial broadcast in subroutine startup()
!     in parallel.f

!     some ntr stuff is broadcast as part of memory.h in NATOM block
!     ntr itself is in md.h in the NRP block

!       itgtmd    0 is default
!                 1 implies targeted md is to be used
!                  xc() will be the refc for RMSD calculation

!       dotgtmd    like "konst" logical for restrained (ntr=1) md

!       tgtrmsd    target rmsd value

!       tgtmdfrc    force constant for tmd


!        cannot be used with ntr=1 since refc shared
!        some info shared with variables normally used with ntr=1


integer itgtmd
common/tmd_int/ itgtmd

logical dotgtmd,rmsok

double precision tgtrmsd,tgtmdfrc
common/tmd_real/ tgtrmsd,tgtmdfrc


! this does not need to be broadcast by parallel.f, it is done in runmd
! will need to be used in ene.f

double precision rmsdvalue
common/tmd_real2/ rmsdvalue



!+ Specification and control of Amber's working precision



!-------------BEGIN    pol.h  ------------------------------------------------

integer    MAGIC_NUMBER_5
parameter (MAGIC_NUMBER_5 = 5)
! what does 5 represent ?

integer          iat1(MAGIC_NUMBER_5)
character(len=4) iat2(MAGIC_NUMBER_5)
integer          i3b(MAGIC_NUMBER_5)
integer          n3b
integer          nion
double precision acon(MAGIC_NUMBER_5), beta3b(MAGIC_NUMBER_5), gama3b(MAGIC_NUMBER_5)

common /thrbod/ iat1, iat2, acon, beta3b, gama3b, i3b, n3b, nion

integer      ipol, iesp
common /pol/ ipol, iesp

!-------------END      pol.h  ------------------------------------------------

!---------------------- les.h --------------------
!+ Specification and control of Amber's working precision



!  parameters for LES:

integer maxles,maxlestyp,maxlesadj
parameter (maxles=50000)
parameter (maxlestyp=100)
parameter (maxlesadj=100000)

integer bc_lesr,bc_lesi
parameter( bc_lesi=1+maxles*3+maxlesadj*2+1)
parameter (bc_lesr=maxlestyp*maxlestyp+1)

double precision lesfac(maxlestyp*maxlestyp),lfac

! for separate LES and non-LES temperature coupling

double precision ekinles0,temp0les,rndfles,sdfacles
double precision scaltles,tempsules,ekeles,rsdles
double precision ekmhles,ekphles

common/lesr/lesfac,temp0les


! ileslst, jleslst and nlesadj are for PME and should maybe only be defined
! if the PME compile option is on

integer ileslst(maxlesadj),jleslst(maxlesadj),nlesadj
integer lestyp(maxles),nlesty,lestmp,cnum(maxles),subsp(maxles)

! this one is 1+maxles*3+maxlesadj*2+1

common/lesi/nlesty,lestyp,cnum,subsp,ileslst,jleslst,nlesadj

! some PME variables
! these are communicated in places other than parallel.f! but the sizes should
! not change

double precision eeles,les_vir(3,3)
common/ewlescomm/eeles,les_vir

! some partial REM variables
double precision elesa,elesb,elesd,elesp
common/eprem/elesa,elesb,elesd,elesp
!---------------------- END les.h --------------------





!+ Specification and control of Amber's working precision



! SIZES

integer sizfftab,sizffwrk,siztheta,siz_q,sizheap,sizstack,sizscr
common/pme_size/ sizfftab,sizffwrk,siztheta,siz_q,sizheap,sizstack,sizscr

! PME HEAP STORAGE: ALL REAL (permanent tabled values)

integer lfftable,lprefac1,lprefac2,lprefac3
common/pme_heap/lfftable,lprefac1,lprefac2,lprefac3

! PARAMETERS

double precision dsum_tol,rsum_tol,maxexp,ew_coeff
common/pme_pars_real/dsum_tol,rsum_tol,maxexp,ew_coeff

integer order,nfft1,nfft2,nfft3,mlimit(3)
common/pme_pars_int/order,nfft1,nfft2,nfft3,mlimit
!---------------------- ew_frc.h --------------------
!+ Specification and control of Amber's working precision



double precision eer,eed,evdw,evdwr,ehb, &
      eedvir,eea,ees,epold,epola,epols
double precision dipself,dipkine,diprms,dipndf
double precision rec_vir(3,3),dir_vir(3,3), &
      adj_vir(3,3),rec_vird(3,3),self_vir(3,3)
double precision frcx,frcy,frcz
double precision atvir(3,3),molvir(3,3),subvir(3,3)
double precision c1,c2,c3,xr1,xr2,xr3
double precision ee14,enb14,epol14
double precision e14vir(3,3),framevir(3,3)

common/ew_comm3/eer,eed,evdw,evdwr,ehb,eedvir,eea,ees, &
      epold,epola,epols, &
      dipself,dipkine,diprms,dipndf, &
      ee14,enb14,epol14, &
      e14vir,framevir, &
      rec_vir,dir_vir,adj_vir,rec_vird,self_vir, &
      c1,c2,c3,xr1,xr2,xr3, &
      atvir,molvir,subvir, &
      frcx,frcy,frcz
!-------------------END ew_frc.h --------------------

! DIR_SUM STORAGE
double precision eedtbdns,dxdr
! real part
integer leed_cub,leed_lin,ltau,mxeedtab
integer eedmeth,ee_type
common/eedctab/leed_cub,leed_lin,ltau,mxeedtab, &
      eedmeth,ee_type
common/tabpars/eedtbdns,dxdr
integer num_recip,num_direct
common/taskcnt/num_recip,num_direct
!-------------  ew_unitcell.h --------------------------

! DEFINING QUANTITIES FOR UNIT CELL
!   ucell is the 3x3 of direct lattice vectors
!   dirlng are their lengths
!   recip is the 3x3 of reciprocal lattice vectors
!   reclng are their lengths
!---------------------------------------------------------


double precision a,b,c,alpha,beta,gamma,volume
double precision ucell,recip,dirlng,reclng,sphere,cutoffnb
double precision olducell,oldrecip
double precision skinnb,cutlist,nbfilter
common/unitcell/ &
      ucell(3,3),recip(3,3),dirlng(3),reclng(3),   &! 24
      olducell(3,3),oldrecip(3,3),                 &! 42
      a,b,c,alpha,beta,gamma, volume,              &! 49
      sphere,cutoffnb,skinnb,cutlist,nbfilter     ! 54

integer nbflag,nbtell,steps_since_list_build
common/ew_upd/nbflag,nbtell,steps_since_list_build


!END----------  ew_unitcell.h --------------------------
!---------------- nonper.h -----------------
!    parameters for nonperiodic box specs
double precision extraboxdim
parameter (extraboxdim=30.d0)
!   Center of box coords used for centering imagcrds in ew_direct.f
double precision xbox0,ybox0,zbox0
common /nonper_real/xbox0,ybox0,zbox0
!----------------end of nonper.h ---------
!------------------ ew_localnb.h --------------------------------

! UNIT CELL  GRID of mapped unit cell coords for preimaging
! NUCGRD1 is number of cells along 1st direction
! NUCGRD2 is number of cells along 2nd direction
! NUCGRD3 is number of cells along 3rd direction
! The subcell neighborhood is obtained by considering
! cells within +- NGHB1 in the first direction
! cells within +- NGHB2 in the second direction, and
! cells within +- NGHB3 in the third direction

! The distance between parallel faces of a subcell is then
!  reclng(1)/NUCGRD1, reclng(2)/NUCGRD2 or
!  reclng(3)/NUCGRD3
! Thus the short range cutoff is the minimum of
!  NGHB1*reclng(1)/NUCGRD1,NGHB2*reclng(2)/NUCGRD2 and
!  NGHB3*reclng(3)/NUCGRD3
! MXATCELL is the maximum number of atoms per subcell
! NOTE!!!!!! YOU MUST HAVE
!           NGHB1 < NUCGRD1, NGHB2 < NUCGRD2, and NGHB3 < NUCGRD3
! imagptr(i) is the subcell number in the imaged grid corresponding to
! subcell i in the unit cell grid
! nghbptr is array from which the neighbor atoms
! in the image grid can be rapidly retrieved.


!----------------------------------------------------------
!  neighbor cell #
integer nghb
parameter (nghb = 3)
!ccccccccccccccccccccccccccccccccccccccccccccccccc
integer nucgrd1,nucgrd2,nucgrd3,nucgmax
integer nghb1,nghb2,nghb3
integer numnptrs
integer numimage

common/dirpars/ &
      numnptrs,nucgrd1,nucgrd2, &
      nucgrd3,nghb1,nghb2,nghb3,numimage,nucgmax

integer nucgrd1_0,nucgrd2_0,nucgrd3_0
common/dirpars_sav/nucgrd1_0,nucgrd2_0,nucgrd3_0

integer mempage
parameter (mempage=512)
! mempage is a page of doubles used for proper offset of scratch
!ccccccccccccccccccccccccccccccccccccccccccccccccc
integer maxnblst,maxnptrs,maximage,mxlstmsk
common/nb_bound/maxnblst,maxnptrs,maximage,mxlstmsk
!ccccccccccccccccccccccccccccccccccccccccccccccccc
! REAL STORAGE
integer limgcrds,lfrction,lsavfrac,ldfrac,lsavcrd
common/nb_float/ &
      limgcrds,lfrction,lsavfrac,ldfrac,lsavcrd
!ccccccccccccccccccccccccccccccccccccccccccccccccc
! INTEGER STORAGE
integer &
      inumatg, iindatg, iatmcell,  iindoff,   iimagptr, &
      inghbptr,ibckptr, iucptr,    inlogrid,  inhigrid, &
      inumimg, inummask,imaskptr,  imask,     iatmlist, &
      ilist,   iscratch,iiwa,      iiwh,               &
      inumvdw, inumhbnd,iipack,    iitran,    iktran, &
      ixtran,  inghtran,imygrdflag,imygrdlist,imy_grids, &
      invdwcls,iindexlo,  iindexhi,  myindexlo, &
      myindexhi,inddelta,invdwcls2

common/nb_integ/ &
      inumatg, iindatg, iatmcell,  iindoff,   iimagptr, &
      inghbptr,ibckptr, iucptr,    inlogrid,  inhigrid, &
      inumimg, inummask,imaskptr,  imask,     iatmlist, &
      ilist,   iscratch,iiwa,      iiwh,               &
      inumvdw, inumhbnd,iipack,    iitran,    iktran, &
      ixtran,  inghtran,imygrdflag,imygrdlist,imy_grids, &
      invdwcls,iindexlo,  iindexhi,  myindexlo, &
      myindexhi,inddelta,invdwcls2

integer max_dcmp_partners
parameter (max_dcmp_partners=32)

integer &
      iib2h,ijb2h,iicb2h,nb2h, &
      iib2a,ijb2a,iicb2a,nb2a
integer iit2h,ijt2h,ikt2h,iict2h,nt2h
integer iit2a,ijt2a,ikt2a,iict2a,nt2a
integer iip2h,ijp2h,ikp2h,ilp2h,iicp2h,nphi2h
integer iip2a,ijp2a,ikp2a,ilp2a,iicp2a,nphi2a
integer inb_14m,numnb14m
integer ishkh,nshkh
integer ineed_atmlist,nneedatm
integer ispmylist,nmyatm
integer iifstwr2,nfstwat2
integer idadjmsk,ndadjmsk
integer ineedatms,ifrcatms,nfrcatm
integer iownedatms,indfrcatms
integer iknown_atm,inew_atm,ifrc_atmlist
integer ifrecvlist,icrecvlist
integer ifsendlist,icsendlist
integer isgrpa0,isgrpa,isgrpb0,isgrpb,nshkgrp,ishkatmgrp
integer iclose_pe,iupdate_pes
integer icellowner
!
!                                            Subtotal of items to broadcast
common /spatial_lists/ &
      iib2h,ijb2h,iicb2h,                   &!3
      iib2a,ijb2a,iicb2a,                   &!6
      iit2h,ijt2h,ikt2h,iict2h,             &!10
      iit2a,ijt2a,ikt2a,iict2a,             &!14
      iip2a,ijp2a,ikp2a,ilp2h,iicp2a,       &!19
      iip2h,ijp2h,ikp2h,ilp2a,iicp2h,       &!24
      inb_14m,                              &!25
      ishkh,iifstwr2,ineed_atmlist,ispmylist,  &!29
      idadjmsk,                             &!30
      ineedatms,                            &!31
      ifrecvlist,icrecvlist,                &!33
      ifsendlist,icsendlist,                &!35
      isgrpa0,isgrpa,isgrpb0,isgrpb,        &!39
      ishkatmgrp,                           &!40
      iclose_pe,                            &!41
      iupdate_pes,                          &!42
      ifrcatms,                             &!43
      iknown_atm,inew_atm,ifrc_atmlist,     &!46
      icellowner,                           &!47
      iownedatms,indfrcatms,                &!49
      nfrcatm, &
      nb2h,nb2a,nt2h,nt2a,nphi2h,nphi2a, &
      numnb14m, &
      nshkh,nfstwat2,nneedatm,ndadjmsk,nmyatm, &
      nshkgrp
integer &
      ineed_mask,ifrc_mask,sizmask, &
      ineed_masksv,ifrc_masksv, &
      ifrc_send,ifrc_recv,icrd_send,icrd_recv, &
      ispneedfrc, &
      ntskfrcsnd,numfrcsnd, &
      ntskfrcrcv,numfrcrcv, &
      ntskcrdsnd,numcrdsnd, &
      ntskcrdrcv,numcrdrcv
common /spatial_masks/ &
      ineed_mask,ifrc_mask,sizmask, &
      ineed_masksv,ifrc_masksv, &
      ifrc_send,ifrc_recv,icrd_send,icrd_recv, &
      ispneedfrc, &
      ntskfrcsnd,numfrcsnd, &
      ntskfrcrcv,numfrcrcv, &
      ntskcrdsnd,numcrdsnd, &
      ntskcrdrcv,numcrdrcv
!ccccccccccccccccccccccccccccccccccccccccccccccccc
integer gindexlo,gindexhi
common/nb_gindex/gindexlo,gindexhi

!---------------END ew_localnb.h --------------------------------

!-----------  begin ew_mpole.h --------------------------------------------
!     indmeth......<&ewald variable> order of extrapolation in 1st estimate
!                   of the iterative process. DEFAULT = 3
!                   Doc says [0,1,2] for 1st,2nd,3rd order, mfc does not
!                       know yet what 3 is....





!   ***********************************************************
!   BC_MULTPOLE needs to be set to the size of the common block:
!   BC_INDDIPR
!   BC_INDDIPI

integer &
      ifirst,  imiddle, ithird,  lfixdip,  linddip, &
      ldipole, lquad,   lfield,  ltorque,  leold1, &
      leold2,  leold3,  ldipvel
common/multpole/ &
      ifirst,  imiddle, ithird,  lfixdip,  linddip, &
      ldipole, lquad,   lfield,  ltorque,  leold1, &
      leold2,  leold3,  ldipvel

double precision diptol,dipmass, diptau,  diptemp
common/inddipr/ &
      diptol,dipmass, diptau,  diptemp

integer maxiter, indmeth, irstdip, iquench, nquench, &
      nttdip
common/inddipi/ &
      maxiter, indmeth, irstdip, iquench, nquench, &
      nttdip

!-----------  END   ew_mpole.h --------------------------------------------
!  ------------ begin ew_cntrl.h -----------------------------------------
!  control parameters:
!   verbose controls level of output. look in force_info
!   checkacc allows rigouous RMS force error checks
!   netfrc       = 1 if remove average force (due to analytic forces in pme)

!   ew_type      = 0 for pme,
!                  1 for reg ewald
!   vdwmeth      = 0 for cutoff of vdw,
!                  1 for analytic fix,
!                  2 for pme with geo mean mixing,
!                  3 for pme lorentz-berthelot mixing
!   use_pme      = 0 to skip reciprocal part of PME,
!                  1 to include it (default)

!   induced      = ipol (&cntrl var) see ipol in pol.h  can be 0,1,2
!   mpoltype     = 1 if induced > 0  (1=dipoles, 0=no dipoles)



!   ***********************************************************
!   BC_EWCTNRL needs to be set to the size of the common block:


!   ***********************************************************
integer verbose,netfrc,     ew_type,    vdwmeth, &
      periodic,  use_pme,    opt_infl,   ischrgd, fix_dip, &
      fix_quad,  mpoltype,   induced,    frameon, chngmask, &
      scaldip

common/ewcntrl/ &
      verbose,   netfrc,     ew_type,    vdwmeth,    &! 4
      periodic,  use_pme,    opt_infl,   ischrgd, fix_dip,    &!9
      fix_quad,  mpoltype,   induced,    frameon, chngmask,   &!14
      scaldip

logical nogrdptrs,nocutoff,boxbad
common/nogrd_flags/nogrdptrs,nocutoff,boxbad
integer inogrdptrs,inocutoff
common/nogrd_flags/inogrdptrs,inocutoff
!  ------------ end   ew_cntrl.h -----------------------------------------
integer bspl_maxorder
parameter( bspl_maxorder=20)
integer maxnfft
parameter( maxnfft=500)
integer nbsplist(50000),nbspstrt(200)
integer nstart,nend,nremain,ndelt,nst
common/ew_bspline_mfc/ nbsplist,nbspstrt, &
      ndelt,nst,nstart,nend,nremain

integer kbot,ktop,igood
integer my_ks(50000)
integer nxtra,pe_xtra(200),lget(200)
common /mfcfcg4/my_ks,igood,kbot,ktop &
      ,lget,nxtra,pe_xtra





   logical do_list_update, qsetup
   data do_list_update / .true. /

   double precision ene(51)
   integer native,nr3,nr

   ! nmrcal vars
   double precision f,enmr,devdis,devang,devtor,ag,bg,cg
   integer numphi,nttyp,nhb

   ! runmin/trajene var
   double precision carrms

   ! dipole momemt stuff
   integer ngrp

   character(len=8) initial_date, setup_end_date, final_date
   character(len=10) initial_time, setup_end_time, final_time
   double precision time0, time1





 

   double precision x (*), r_stack (*)
   integer  ix(*), ipairs (*), i_stack(*)
   character(len=4)  ih (*)


   !     ---- HERE BEGIN THE EXECUTABLE STATEMENTS ----


      !        --- finish reading the prmtop file and other user input:
      call rdparm2(x,ix,ih,ipairs,8,i_stack)
      call mdread2(x,ix,ih,ipairs,r_stack,i_stack)


                                !add comments for read_parm version

      !        --- alloc memory for decomp module that needs info from mdread2
!      if( idecomp == 1 .or. idecomp == 2 ) then
!         call allocate_real_decomp(nres)
!      else if( idecomp == 3 .or. idecomp == 4 ) then
!         call allocate_real_decomp(npdec*npdec)
!     end if

end subroutine readfromparmfile 

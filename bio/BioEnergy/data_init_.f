

!***************************************************************
!*** version of subroutine mdread1 with input default  *********
!**** without read_lmod_list  **********************************
!***************************************************************


!************************************************************************
!                              AMBER                                   **
!                                                                      **
!                        Copyright (c) 2003                            **
!                Regents of the University of California               **
!                       All Rights Reserved.                           **
!                                                                      **
!  This software provided pursuant to a license agreement containing   **
!  restrictions on its disclosure, duplication, and use. This software **
!  contains confidential and proprietary information, and may not be   **
!  extracted or distributed, in whole or in part, for any purpose      **
!  whatsoever, without the express written permission of the authors.  **
!  This notice, and the associated author list, must be attached to    **
!  all copies, or extracts, of this software. Any additional           **
!  restrictions set forth in the license agreement also apply to this  **
!  software.                                                           **
!************************************************************************

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




subroutine data_init(iconst)
   
!   use lmod_driver, only : read_lmod_namelist
   
   implicit none
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
!---------------------- constants.h --------------------

!-------------------------------------------------------
! Physical Constants

! This is based on old values of the electric constants.
double precision      AMBER_ELECTROSTATIC
parameter ( AMBER_ELECTROSTATIC = 18.2223d0 )

! 1998 value of the Bohr radius, physics.nist.gov/constants.
! a0 is the standard abbreviation, but perhaps that is too cryptic here.
! The previous value was 0.5291771 in Amber 7.
double precision      BOHR_RADIUS
parameter ( BOHR_RADIUS = 0.5291772083d0 )


!-------------------------------------------------------
! Numeric Constants

double precision      PI
parameter ( PI = 3.1415926535897932384626433832795d0 )

double precision      SQRT2
parameter ( SQRT2 = 1.4142135623730950488016887242097d0 )


!-------------------------------------------------------
! Unusual Constants

integer     RETIRED_INPUT_OPTION
parameter ( RETIRED_INPUT_OPTION = -10301 ) ! first 5 digit palindromic prime


!-------------------------------------------------------
! Generic Floating Point Constants

double precision      TEN_TO_MINUS2
parameter ( TEN_TO_MINUS2  = 1.0d-2 )
double precision      TEN_TO_MINUS3
parameter ( TEN_TO_MINUS3  = 1.0d-3 )
double precision      TEN_TO_MINUS4
parameter ( TEN_TO_MINUS4  = 1.0d-4 )
double precision      TEN_TO_MINUS5
parameter ( TEN_TO_MINUS5  = 1.0d-5 )
double precision      TEN_TO_MINUS6  
parameter ( TEN_TO_MINUS6  = 1.0d-6 )
double precision      TEN_TO_MINUS10
parameter ( TEN_TO_MINUS10 = 1.0d-10 )
double precision      TEN_TO_PLUS3
parameter ( TEN_TO_PLUS3  = 1.0d+3 )
double precision      TENTOPLUS10
parameter ( TENTOPLUS10 = 1.0d+10 )

double precision      zero
parameter ( zero   = 0.0d0 )
double precision      one
parameter ( one    = 1.0d0 )
double precision      two
parameter ( two    = 2.0d0 )
double precision      three
parameter ( three  = 3.0d0 )
double precision      four
parameter ( four   = 4.0d0 )
double precision      five
parameter ( five   = 5.0d0 )
double precision      six
parameter ( six    = 6.0d0 )
double precision      seven
parameter ( seven  = 7.0d0 )
double precision      eight
parameter ( eight  = 8.0d0 )
double precision      nine
parameter ( nine   = 9.0d0 )
double precision      ten
parameter ( ten    = 10.0d0 )
double precision      eleven
parameter ( eleven = 11.0d0 )
double precision      twelve
parameter ( twelve = 12.0d0 )
double precision      half
parameter ( half        = one/two )
double precision      third
parameter ( third       = one/three )
double precision      fourth
parameter ( fourth      = one/four )
double precision      fifth
parameter ( fifth       = one/five )
double precision      sixth
parameter ( sixth       = one/six )
double precision      seventh
parameter ( seventh     = one/seven )
double precision      eighth
parameter ( eighth      = one/eight )
double precision      ninth
parameter ( ninth       = one/nine )
double precision      tenth
parameter ( tenth       = one/ten )
double precision      eleventh
parameter ( eleventh    = one/eleven )
double precision      twelfth
parameter ( twelfth     = one/twelve )

!------------------ End constants.h --------------------
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
!+ Specification and control of Amber's working precision


!  dec cpp requires this comment line to avoid losing beginning spaces

integer ilbopt, ilbnob, lbverb, lbfreq, dozero
common/extra_int/ilbopt, ilbnob, lbverb, lbfreq, dozero

double precision lbwght
common/extra_real/lbwght

logical master
common/extra_logical/master

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
!+ MMTSB Replica Exchange


! public members

integer mmtsb_switch       ! Replica Exchange Control:
integer mmtsb_off          ! No Replica Exchange
parameter ( mmtsb_off = 0 )
integer mmtsb_temp_rex     ! Temperature Replica Exchange
parameter ( mmtsb_temp_rex = 1 )
integer mmtsb_lambda_rex   ! Lambda Replica Exchange
parameter ( mmtsb_lambda_rex = 2 )
integer mmtsb_iterations   ! Replica Exchange Frequency in Iterations
logical mmtsb_is_exchanged ! replica exchange occurred this step

common /mmtsb_public/ mmtsb_switch, mmtsb_iterations, &
      mmtsb_is_exchanged

! These may cause compilation errors in the subroutines themselves.
!external Mmtsb_init
!external Mmtsb_newtemp
!external Mmtsb_print_banner


!+ Specification and control of Amber's working precision



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



   double precision      temp0les
   character(len=4) watdef(4),watnam,owtnm,hwtnm1,hwtnm2

   double precision      at1
   double precision      at2
   double precision      beta3
   double precision      dele
   double precision      gamma3
   integer     ierr
   integer     ifind
   integer     imcdo
   integer     itotst
   integer     jn
   logical     mdin_cntrl, mdin_lmod  ! true if these namelists exist in mdin
   integer     mxgrp
   character(len=8) date
   character(len=10) time
   double precision      dtemp  ! retired 
   double precision      dxm  ! retired 
   double precision      heat  ! retired 

   integer len_name, iconst

   
namelist /cntrl/ timlim,irest,ibelly, &
         ntx,ntxo,ntcx,ig,tempi, &
         ntb,ntt,temp0,tautp, &
         ntp,pres0,comp,taup,npscal, &
         nscm,nstlim,t,dt, &
         ntc,ntcc,nconp,tol,ntf,ntn,nsnb, &
         cut,scnb,scee,dielc, &
         ntpr,ntwx,ntwv,ntwe,ntave,ntpp,ioutfm, &
         ntr,nrc,ntrx,taur,nmropt, &
         ivcap,fcap,imin,drms,dele,dx0, &
         pencut,ipnlty,iscale,scalm,noeskp, &
         maxcyc,ncyc,ntmin,vlimit, &
         mxsub,ipol,jfastw,watnam,owtnm,hwtnm1,hwtnm2, iesp, &
         ntwprt,n3b,nion,at1,at2,acon,beta3,gamma3,tausw, &
         ntwr,iyammp,imcdo, &
         igb,rgbmax,saltcon,offset,gbsa,vrand, &
         surften,iwrap,nrespa,nrespai,gamma_ln,extdiel,intdiel, &
         cut_inner,icfe,clambda,klambda, rbornstat,lastrst,lastist,  &
         itgtmd,tgtrmsd,tgtmdfrc,tgtfitmask,tgtrmsmask, &
         idecomp,temp0les,restraintmask,restraint_wt,bellymask, &
         mmtsb_switch, mmtsb_iterations,rdt,icnstph,solvph,ntcnstph &
         ,dtemp &  ! retired 
         ,dxm &  ! retired 
         ,heat &  ! retired 
         ; ! line terminator for free-form version

   ! Define default water residue name and the names of water oxygen & hydrogens
   
   data watdef/'WAT ','O   ','H1  ','H2  '/
   
   !     ----- READ THE CONTROL DATA AND OPEN DIFFERENT FILES -----

  
   
   if (mdout /= "stdout" ) &
         call amopen(6,mdout,owrite,'F','W')

    call amopen(5,mdin,'U','F','U')
    
      write (5, 1111) 
 1111 format (///,15X,'INPUT FILE FOR FLAGS ') 

   write(6,9308)
   call date_and_time( DATE=date, TIME=time )
   write(6,'(12(a))') '| Run on ', date(5:6), '/', date(7:8), '/',  &
        date(1:4), ' at ', time(1:2), ':', time(3:4), ':', time(5:6)
   if (owrite /= 'N') write(6, '(2x,a)') '[-O]verwriting output'
   
   ! Echo the file assignments to the user:
   
   write(6,9700) 'MDIN'   ,mdin(1:70)  , 'MDOUT' ,mdout(1:70) , &
         'INPCRD' ,inpcrd(1:70), 'PARM'  ,parm(1:70)  , &
         'RESTRT',restrt(1:70) , 'REFC'  ,refc(1:70)  , &
         'MDVEL' ,mdvel(1:70)  , 'MDEN'   ,mden(1:70) , &
         'MDCRD' ,mdcrd(1:70)  , 'MDINFO' ,mdinfo(1:70), &
         'INPDIP', inpdip(1:70), 'RSTDIP', rstdip(1:70)
   
   ! Echo the input file to the user:
   
!   call echoin(5,6)
   
   
   !     ----- READ DATA CHARACTERIZING THE MD-RUN -----
   
!   read(5,'(20a4)') title
   
   !       ----read input in namelist format, first setting up defaults
   
   dtemp = RETIRED_INPUT_OPTION
   dxm   = RETIRED_INPUT_OPTION
   heat  = RETIRED_INPUT_OPTION
   timlim = 999999.d0
   irest = 0
   ibelly = 0
   ipol = 0
   iesp = 0
   ntx = 1
   ntxo = 1
   ig = 71277
   tempi = ZERO
   ntb = 0
   ntt = 0
   temp0 = 300.0d0

   tautp = ONE
   ntp = 0
   pres0 = ONE
   comp = 44.6d0
   taup = ONE
   npscal = 1
   nscm = 1000
   nstlim = 1
   t = ZERO
   dt = 0.001d0
   ntc = 1
   tol = 0.00001
   ntf = 1
   nsnb = 25
   cut =  1.5d01
   scnb = TWO
   scee = 1.2d0
   dielc = ONE
   ntpr = 10
   ntwr = 500
   ntwx = 0
   ntwv = 0
   ntwe = 0
   ntave = 0
   ioutfm = 0
   ntr = 0
   ntrx = 1
   ivcap = 0
   fcap = 1.5d0
   
   ! carlos targeted MD, like ntr
   
   itgtmd=0
   tgtrmsd=0.
   tgtmdfrc=0.
   tgtfitmask=''
   tgtrmsmask=''

   pencut = 0.1d0
   taumet = 0.0001d0
   omega = 500.0d0
   ipnlty = 1
   scalm = 100.0d0
   iscale = 0
   noeskp = 1
   nmropt = 0
   tausw = 0.1d0
   imin = 1
   isftrp = 0
   rwell = ONE
   maxcyc = 100
   ncyc = 10
   ntmin = 1
   dx0 = 0.01d0
   drms = 1.0d-4
   vlimit = 20.0d0
   mxsub = 1
   jfastw = 0
   watnam = '    '
   owtnm =  '    '
   hwtnm1 = '    '
   hwtnm2 = '    '
   ntwprt = 0
   igb = 0
   rgbmax = 25.d0
   saltcon = ZERO
   offset = 0.09d0
   iyammp = 0
   imcdo = -1
   gbsa = 0
   vrand=1000
   surften = 0.005d0
   iwrap = 0
   nrespa = 1
   nrespai = 1
   irespa = 1
   gamma_ln = ZERO
   extdiel = 78.5d0
   intdiel = ONE
   gbgamma = ZERO
   gbbeta = ZERO
   gbalpha = ONE
   iconstreff = 0
   cut_inner = EIGHT
   icfe = 0
   clambda = ZERO
   klambda = 1
   rbornstat = 0
   idecomp = 0
   lastrst = 2000000
   lastist = 2000000
   restraintmask=''
   restraint_wt = ZERO
   bellymask=''
   mmtsb_switch = mmtsb_off ! MMTSB Replica Exchange Off by Default
   mmtsb_iterations = 100   ! MMTSB Replica Exchange Frequency in Iterations
   
   if (iconst == 1) then
   ntr = 1
   restraintmask=':FRA'
   restraint_wt = 1.0

!      ibelly = 1
!      write(AA(1:3),'(A3)') ':1-'
!      write(AA(4:8),'(I5)') (nres-1) 
!      bellymask = AA (1:8)
   endif
   
   icnstph = 0
   solvph = SEVEN
   ntcnstph = 10
   
   !     Check to see if "cntrl" namelist has been defined.
   
   mdin_cntrl=.false.
   mdin_ewald=.false.
   mdin_pb=.false.
   mdin_lmod=.false.
!   call nmlsrc('cntrl',5,ifind)
!   if (ifind /= 0) mdin_cntrl=.true.
!   call nmlsrc('ewald',5,ifind)
!   if (ifind /= 0) mdin_ewald=.true.
!   call nmlsrc('pb',5,ifind)
!   if (ifind /= 0) mdin_pb=.true.
!   call nmlsrc('lmod',5,ifind)
!   if (ifind /= 0) mdin_lmod=.true.
!#ifdef MDM_MD
!   call nmlsrc('mdm',5,ifind)
!   if (ifind /= 0) mdin_mdm=.true.                                                       
!#endif                                                                                      
!#ifdef MDM_PDB                                                                              
!   call nmlsrc('pdb',5,ifind)                                                            
!   if (ifind /= 0) mdin_pdb=.true.                                                       
!#endif                                                                                      
  mdin_cntrl=.true.  
 
!   rewind 5
!   if ( mdin_cntrl ) then
!      read(5,nml=cntrl)
!   else
!      write(6, '(1x,a,/)') 'Could not find cntrl namelist'
!      call mexit(6,1)
!   end if
   if ( mdin_lmod ) then
!      rewind 5
!      call read_lmod_namelist()
   end if
   
   !--------------------------------------------------------------------
   !     --- vars have been read ---
   !--------------------------------------------------------------------
   
   write(6,9309)
   
   ! emit warnings for retired cntrl namelist variables

   if ( dtemp /= RETIRED_INPUT_OPTION ) then
      write(6,'(/,a,/,a,/,a)') 'Warning: dtemp has been retired.', &
            '  Check the Retired Namelist Variables Appendix in the manual.'
   end if
   if ( dxm /= RETIRED_INPUT_OPTION ) then
      write(6,'(/,a,/,a,/,a)') 'Warning: dxm has been retired.', &
            '  Check the Retired Namelist Variables Appendix in the manual.'
            ! '  The step length will be unlimited.'
   end if
   if ( heat /= RETIRED_INPUT_OPTION ) then
      write(6,'(/,a,/,a,/,a)') 'Warning: heat has been retired.', &
            '  Check the Retired Namelist Variables Appendix in the manual.'
   end if

   call printflags()

   ! -------------------------------------------------------------------
   ! If the user has requested a 3-body polar calc, do some extra reads
   ! -------------------------------------------------------------------
   
   if (ipol > 1) then
!      read(5,'(2I5)',end=1155) n3b,nion
      if (nion == 0 ) then
         write(6,'(t2,''%SANDER-W-NO_ION, nion = 0'')')
      end if
      write(6,'(/,t2,a,i5,a,i5)') &
            'Number of triplets = ',n3b, &
            ' nion = ',nion
      write(6,'(t2,a)') &
            'sing pair    acons     beta     gamma'
      do 55 jn = 1,n3b
!         read(5,'(a4,a4,2x,3e10.3)',end=1156) &
!               iat1(jn),iat2(jn),acon(jn),beta3b(jn),gama3b(jn)
         write(6,'(t2,a4,1x,a4,1x,3e12.4)') &
               iat1(jn),iat2(jn),acon(jn),beta3b(jn),gama3b(jn)
      55 continue
   end if
   
   !--------------------------------------------------------------------
   ! If user has requested ewald electrostatics, read some more input
   !--------------------------------------------------------------------
   
   if( igb == 0 ) call load_ewald_info_modif(parm,inpcrd,ntp,ipol,len_name)
   
   ishake = 0
   if (ntc > 1) ishake = 1
   
   !--------------------------------------------------------------------
   ! Set up some parameters for GB simulations:
   !--------------------------------------------------------------------
   
   if( igb == 2 ) then
      
      !       --- use our best guesses for Onufriev/Case GB  (GB^OBC I)
      
      gbgamma = 2.909125d0
      gbbeta = ZERO
      gbalpha = 0.8d0
   end if

   if( igb == 5 ) then
      
      !       --- use our second best guesses for Onufriev/Case GB (GB^OBC II)
      
      gbgamma = 4.851d0
      gbbeta = 0.8d0
      gbalpha = ONE
   end if
   
   !--------------------------------------------------------------------
   ! If user has requested PB electrostatics, read some more input
   !--------------------------------------------------------------------

   if ( igb == 10 ) then
      call pb_read
   end if
   
!cnt N.TAKADA: For MDM_start

!cnt N.TAKADA: For MDM_end
   
   ! -------------------------------------------------------------------
   ! If the user has requested NMR restraints, do a cursory read of the
   ! restraints file(s) now to determine the amount of memory necessary
   ! for these restraints:
   ! -------------------------------------------------------------------
   
   intreq = 0
   irlreq = 0
   if (nmropt > 0) then
      mxgrp = 0
      itotst = 1
      
      ! Set ITOTST to 0 if IMIN equals 1 (i.e. if minimization, not dynamics)
      ! This will cause any "time-averaged" requests to be over-ridden.
      
      if (imin == 1) then 
         itotst = 0
      end if 
      !         CALL AMOPEN(31,NMR,'O','F','R')
      call restlx(5,itotst,mxgrp,dt,6,ierr)
      !         CLOSE(31)
   end if
   
   ! Set the definition of the water molecule. The default definition is in
   ! WATDEF(4).
   
   read(watdef(1),'(A4)') iwtnm
   read(watdef(2),'(A4)') iowtnm
   read(watdef(3),'(A4)') ihwtnm(1)
   read(watdef(4),'(A4)') ihwtnm(2)
   if (watnam /= '    ') read(watnam,'(A4)') iwtnm
   if (owtnm /= '    ') read(owtnm, '(A4)') iowtnm
   if (hwtnm1 /= '    ') read(hwtnm1,'(A4)') ihwtnm(1)
   if (hwtnm2 /= '    ') read(hwtnm2,'(A4)') ihwtnm(2)
   
   return
   
   ! --- input file polar opts read err trapping:
   
   1155 write(6,*) ' ** EOF reading N3B,NION for 3-body option'
   call mexit(6,1)
   1156 write(6,*) ' ** EOF reading triplets for 3-body option'
   call mexit(6,1)
   
   9308 format(/10x,55('-'),/10x, &
         'Amber 8  SANDER                 Scripps/UCSF 2004', &
         /10x,55('-')/)
   9309 format(/80('-')/'   1.  RESOURCE   USE: ',/80('-')/)
   9700 format(/,'File Assignments:',/,12('|',a6,': ',a,/))
end subroutine data_init

subroutine load_ewald_info_modif(parm,inpcrd,ntp,ipol,len_name)
   implicit none
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
! DIR_SUM STORAGE
double precision eedtbdns,dxdr
! real part
integer leed_cub,leed_lin,ltau,mxeedtab
integer eedmeth,ee_type
common/eedctab/leed_cub,leed_lin,ltau,mxeedtab, &
      eedmeth,ee_type
common/tabpars/eedtbdns,dxdr
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
   double precision ax,bx,cx,alphax,betax,gammax
   integer ipol,ntp
   integer len_name
   character(len=(len_name+5)) parm
   character(len=(len_name+6)) inpcrd

   
   induced = ipol
   
   ! get the values for ucell:
   
   if( ntb > 0 ) then
      write(6,*)'getting new box info from bottom of inpcrd'
      call peek_ewald_inpcrd(inpcrd,ax,bx,cx,alphax,betax,gammax)
   end if
   call read_ewald(ax,bx,cx,alphax,betax,gammax)
   
   !     ---- also set the older parameters in box.h to these values:
   
   box(1) = a
   box(2) = b
   box(3) = c
   
   !  Check if non-isotropic scaling is requested with non-orthorhombic cell:
   
   if ( ntp > 1 ) then
      if ( abs(alpha-90.0d0 ) > 1.d-5 .or. &
            abs(beta - 90.0d0) > 1.d-5 .or. &
            abs(gamma - 90.0d0) > 1.d-5 ) then
         call sander_bomb('read_ewald', &
               'Cannot do non-isotropic scaling unless orthorhombic' &
               ,'use ntp=1 if angles are not 90 degrees. ')
      end if
   end if

   ! implement extended list
   
   if ( nbflag == 1 .or. skinnb > 1.d-10 ) then
      nbfilter = cutoffnb
      cutlist = cutoffnb + skinnb
   else
      
      ! don't exceed the erfc table maximum
      
      cutlist = cutoffnb
      nbfilter = 1.4 * cutoffnb
   end if
   
   ! setup dxdr map from r to x in table lookup of eed
   
   if ( ee_type == 1 )then
      dxdr = ew_coeff
   else if ( ee_type == 2 )then
      dxdr = 1.d0/cutoffnb
   end if
   
   ! setup point multipoles
   
   mpoltype = 0
   if ( induced > 0 )mpoltype = 1
   if ( induced == 0 )then
      irstdip =0
      indmeth = 0
   end if
   
   return
end subroutine load_ewald_info_modif 

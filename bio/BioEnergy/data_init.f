!***************************************************************
!*** version of subroutine mdread1 with input default  *********
!**** without read_lmod_list  **********************************
!***************************************************************

#include "copyright.h"
#include "dprec.h"


#ifdef QMMM
subroutine data_init(ifqnt,nquant,labels,idc,len_name,iconst)
#else
subroutine data_init(iconst)
#endif
   
#ifndef QMMM
!   use lmod_driver, only : read_lmod_namelist
#endif
   
   implicit none
#  include "box.h"
#  include "constants.h"
#  include "def_time.h"
#  include "ew_cntrl.h"
#  include "extra.h"
#  include "files.h"
#  include "md.h"
#  include "memory.h"
#  include "mmtsb.h"
#  include "nmr.h"
#  include "parms.h"
#  include "pol.h"
#  include "tgtmd.h"
#ifdef LES
#  include "les.h"
#else
   _REAL_      temp0les
#endif
#ifdef QMMM
#  include "cp.h"
   integer idochg,idovdw,idopmf,ifqt,ifpme,i,nqt,modchg,kmax
   _REAL_  alpha
#endif
   character(len=4) watdef(4),watnam,owtnm,hwtnm1,hwtnm2

   _REAL_      at1
   _REAL_      at2
   _REAL_      beta3
   _REAL_      dele
   _REAL_      gamma3
   integer     ierr
   integer     ifind
   integer     imcdo
   integer     itotst
   integer     jn
   logical     mdin_cntrl, mdin_lmod  ! true if these namelists exist in mdin
   integer     mxgrp
   character(len=8) date
   character(len=10) time
   _REAL_      dtemp  ! retired 
   _REAL_      dxm  ! retired 
   _REAL_      heat  ! retired 

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
#ifdef REM
         numexchg, repcrd, &
#endif
         ntwprt,n3b,nion,at1,at2,acon,beta3,gamma3,tausw, &
         ntwr,iyammp,imcdo, &
         igb,rgbmax,saltcon,offset,gbsa,vrand, &
         surften,iwrap,nrespa,nrespai,gamma_ln,extdiel,intdiel, &
         cut_inner,icfe,clambda,klambda, rbornstat,lastrst,lastist,  &
         itgtmd,tgtrmsd,tgtmdfrc,tgtfitmask,tgtrmsmask, &
         idecomp,temp0les,restraintmask,restraint_wt,bellymask, &
         mmtsb_switch, mmtsb_iterations,rdt,icnstph,solvph,ntcnstph &
#ifdef QMMM
         ,IDOCHG,IDOVDW,IDOPMF,CHGLAM,DCHG,VLAMBI,VLAMBF &
         ,NPERT,NEQUIL,NSAMPL,IFQT,NQT,MODCHG,kmax,idc &
#endif
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
#ifdef LES
   
   ! alternate temp for LES copies, if negative then use single bath
   ! single bath not the same as 2 baths with same target T
   
   temp0les = -ONE
   rdt = 0.0
   
#endif

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
#ifdef REM
   numexchg = 0
   repcrd   = 0
#endif
   
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
#ifdef QMMM
   alpha = ZERO
   IFPME = 0
   kmax = 0
   nqt = 0
   NRESPA = 1
#endif
   
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
#ifdef MDM_MD
   mdin_mdm=.false.
#endif
#ifdef MDM_PDB
   mdin_pdb=.false.
#endif
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
#ifdef QMMM
      do_scf = .true.
      ifqnt=ifqt
      call int_legal_range('QMMM: (number of quantum atoms) ', &
            nqt, 0, mxqmatm )
      nquant=nqt
      modchg=0
      if(ifqnt.eq.1) then
!        read(5,*) (labels(i),i=1,nquant)
        call qmsort(nquant,labels)
!       if(modchg.ne.0)then
!         do 120 i=1,modchg
!           read(5,*) ichng(i),tmpchg(i)
!120      continue
!       endif
      endif
#else
   if ( mdin_lmod ) then
!      rewind 5
!      call read_lmod_namelist()
   end if
#endif
   
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
   
#ifndef QMMM
   if( igb == 0 ) call load_ewald_info_modif(parm,inpcrd,ntp,ipol,len_name)
#endif
   
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
#ifdef MDM_MD
!cnt -------------------------------------------------------------------
!cnt  If user has requested MDM system, read some more input variables:
!cnt -------------------------------------------------------------------
   call LoadMDMInfo
#endif

#ifdef MDM_PDB
!cnt -------------------------------------------------------------------
!cnt  If user has requested PDB files, read some more input variables:
!cnt -------------------------------------------------------------------
   call LoadPDBInfo
#endif
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
#  include "ew_cntrl.h"
#  include "ew_unitcell.h"
#  include "ew_pme_recip.h"
#  include "ew_erfc_spline.h"
#  include "box.h"
#  include "ew_mpole.h"
#ifdef MPI
#  include "ew_parallel.h"
#endif
   _REAL_ ax,bx,cx,alphax,betax,gammax
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

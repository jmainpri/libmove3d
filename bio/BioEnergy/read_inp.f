!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+   Subroutine READ_INP reads first part of parm file
!+                       and determines memory size;
!+                       no memory allocation here.
!  
!+   It's first part of subroutine sander modified by S.Kirillova.
!+   SANDER in AMBER_SRC/sander/sander.f;   
!+          call subroutines to run MD or minimization calculations.
!+                                       
!++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

subroutine readmemorysizesfromparmfile(lstr, lsti, lsth, lstpr, lstrst, lstist, name_first, len_name, iconst )

!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+ lstr, lsti, lsth, lstpr, lstrst, lstist -- variables for determination  
!+                                            of memory size
!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  !no no 3-point solvent molecules  (fastwat) ;
  !no LMOD methods (lmod_driver);
  !no computation of nonbonded interactions with a generalized Born model,
  !getting the "effective" Born radii via the approximate pairwise method
  !Use Eqs 9-11 of Hawkins, Cramer, Truhlar, J. Phys. Chem. 100:19824 (1996) (getborn).
                                
                               !add comments for read_inp version
!#ifndef QMMM
!   use lmod_driver
!   use genborn
!#endif
!   use decomp, only : allocate_int_decomp, allocate_real_decomp, &
!                      deallocate_int_decomp, deallocate_real_decomp
!   use fastwt

   implicit none

#  include "files.h"
#  include "memory.h"
#  include "nmr.h"
#  include "box.h"
#  include "md.h"
#  include "parms.h"
#  include "extra.h"
#  include "tgtmd.h"
#  include "pol.h"
#  include "les.h"
#ifdef QMMM
#  include "cp.h"
#endif
#ifdef MPI
   !     =========================== AMBER/MPI ===========================
#  include "parallel.h"
#  include "ew_parallel.h"
#  ifdef MPI_DOUBLE_PRECISION
#    undef MPI_DOUBLE_PRECISION
#  endif
#  include "mpif.h"
#  ifdef CRAY_PVP
#    define MPI_DOUBLE_PRECISION MPI_REAL8
#  endif
#  ifdef MPI_BUFFER_SIZE
   integer*4 mpibuf(mpi_buffer_size)
#  endif
#ifdef REM
#  include "rem.h"
   integer ii
#endif
   _REAL_ ener(30),vir(4)
   !     ========================= END AMBER/MPI =========================
#endif
#  include "ew_pme_recip.h"
#  include "ew_frc.h"
#  include "ew_erfc_spline.h"
#  include "ew_numtasks.h"
#  include "ew_unitcell.h"
#  include "nonper.h"
#  include "ew_localnb.h"
#  include "ew_mpole.h"
#  include "ew_cntrl.h"
#  include "ew_bspline.h"
#  include "def_time.h"

  logical do_list_update, qsetup
  data do_list_update / .true. /

   _REAL_ ene(51)
   integer native,nr3,nr

   ! nmrcal vars
   _REAL_ f,enmr,devdis,devang,devtor,ag,bg,cg
   integer numphi,nttyp,nhb

   ! runmin/trajene var
   _REAL_ carrms

   ! dipole momemt stuff
   integer ngrp

   character(len=8) initial_date, setup_end_date, final_date
   character(len=10) initial_time, setup_end_time, final_time
   _REAL_ time0, time1

#ifdef QMMM
   integer idiff,i,j,klink,istop,index,ierror,itemp
   _REAL_  dummy
#endif
                                !add comments for read_inp version

!   _REAL_,  dimension(:), allocatable :: x, r_stack
!   integer, dimension(:), allocatable :: ix, ipairs, i_stack
!   character(len=4), dimension(:), allocatable :: ih

   logical belly, erstop
   integer ier,ifind,jn,ncalls
   character(len=4) itest
   logical ok

   integer len_name
   character(len=len_name) name_first
   integer lstr, lsti, lsth, lstpr, lstrst, lstist;
   integer i, iconst  

   !     ---- HERE BEGIN THE EXECUTABLE STATEMENTS ----

  !     --- default file names from mdfil.f ---
 
      owrite = 'U'

   inpcrd = name_first//'.coord'
   parm = name_first//'.parm'

   mdin = name_first// '.inp'   ! input; information for PB (Poisson-Boltzman) method, Ewald method,
                                ! and for debugging the forces;
                                ! if these methods are not used no information in min.in file
   mdout = 'stdout'             ! output to the terminal 
   restrt = name_first// '.restrt' ! final coordinates
   refc   = name_first// '.refc'   ! input; reference coords for position restraints; also used for targeted MD 
   mdvel  = name_first// '.mdvel'  ! output; velocity sets saved over MD trajectory
   mden   = name_first// '.mden'   ! output; extensive energy data over MD trajectory 
   mdcrd  = name_first// '.mdcrd'  ! output; coordinate sets saved over MD trajectory
   mdinfo = name_first// '.mdinfo' ! output; latest mdout-format energy info for MD
   vecs   = name_first// '.vecs'   ! input; NOE restraints for MD; used in subroutine noeread 
   freqe  = name_first// '.dummy'  ! input; NOE restraints for MD; used in subroutine noeread
   rstdip = name_first//'.rstdip'  ! the restart file for dipoles in subroutine get_dips
   inpdip = name_first//'.inpdip'  ! input file for dipoles in subroutine get_dips
   mddip  = name_first// '.mddip'  ! ???
   radii  = name_first//'.radii'   ! ???
   cpin   = name_first//'.cpin'    ! Constant pH state information
   cpout  = name_first//'.cpout'   ! Constant pH protonation output
   cprestrt = name_first//'.cprest' ! Constant pH state restart information

   ! Initialize the cpu timer. Needed for machines where returned cpu times
   ! are relative.
                                !add comments for read_inp version
                                !no timer here

!   call date_and_time( initial_date, initial_time )
!   call wallclock( time0 )
!   call init_timers()

#ifdef MPI
   !     =========================== AMBER/MPI ===========================

   !     Parallel initialization (setup is now down outside of sander)

   !     Make PE 0 the master
   master = mytaskid == 0

#  ifndef noBTREE
   !  BTREE is selected by default if noBTREE is not specified
   !     The number of processes is required to be a power of two.
   !
   if ( master .and. numtasks > 1 ) then
      if ( numtasks > MPI_MAX_PROCESSORS .or. &
            logtwo(numtasks) <= 0 ) then  ! assume short-circut logical or
         write(0,*) 'The number of processors must be a power of 2', &
               ' and no greater than ', MPI_MAX_PROCESSORS, &
               ', but is ', numtasks
!         call mexit(6,1)
      end if
   end if
#  endif /* BTREE */
#  ifdef MPI_BUFFER_SIZE
   call mpi_buffer_attach(mpibuf, mpi_buffer_size*4, ierr)
#  endif

   !     ========================= END AMBER/MPI =========================


#else   /* not MPI follows */

   !     in the single-threaded version, the one process is master
   master = .true.
#endif  /* MPI */
   erstop = .false.
   qsetup = .true.
   !     --- generic packing scheme ---

   nwdvar = 1
   native = 32

#ifdef ISTAR2

   !     --- Int*2 packing scheme ---

   nwdvar = 2
#endif  /*ISTAR2*/


   numpk = nwdvar
   nbit = native/numpk

   !     ----- Only the master node (only node when single-process)
   !           performs the initial setup and reading/writing -----

!   call timer_start(TIME_TOTAL)
!   if (master) then

      !        --- initialize stack ---

      call rstack_setup()
      call istack_setup()

      !        ---- first, initial reads to determine memry sizes:

                                ! here subroutine mdread1 replaced by
                                ! subroutine data_init 
#ifdef QMMM

      call data_init(ifqnt,nquant,labels,idc,len_name,iconst)
#else
      call data_init(iconst)
#endif

      call amopen(8,parm,'O','F','R')
      call rdparm1(8)
      
      call locmem

      lstr = lastr
      lsti = lasti
      lsth = lasth
      lstpr = lastpr
                                !add comments for read_inp version

      !     --- dynamic memory allocation:
!      allocate( x(lastr), ix(lasti), ipairs(lastpr), ih(lasth), stat = ier )
!      REQUIRE( ier == 0 )

      if( igb == 0 ) then
#ifndef PSANDER
         lastrst = sizffwrk + siz_q + natom*(4 + 6*order)
         if( mpoltype == 1 ) lastrst = lastrst + 3*order*natom
         if( ntb == 0 ) then
            ! 1000 is too small; test/dmp has natom=13 and lastrst=130000
            ! 100000 is too big; test/nonper has natom=991 and allocation fails.
            lastrst = 10*natom + 50000
         end if
#endif
      else
         lastrst = 1
#ifndef QMMM
!         if( igb /= 10 ) call allocate_gb( natom )
#endif
#ifdef LES
         lastrst = lastrst + 4*natom*ncopy
#endif
      end if
      if( icfe > 0 )  lastrst = lastrst + 3*natom
!      allocate( r_stack(1:lastrst), stat = ier )
!      REQUIRE( ier == 0 )

        lstrst = lastrst

      if( igb == 0 ) then
#ifndef PSANDER
         lastist = 5*natom
#endif
      else
         lastist = 1
      end if

                                !add comments for read_inp version
!      allocate( i_stack(1:lastist), stat = ier )
!      REQUIRE( ier == 0 )

      lstist = lastist
!      if( idecomp > 0 ) then
!         call allocate_int_decomp(idecomp, natom, nres)
!      endif

      write(6,'(/,a,5x,a)') '|','Memory Use     Allocated'
      write(6,'(a,5x,a,i14)') '|', 'Real      ', lastr
      write(6,'(a,5x,a,i14)') '|', 'Hollerith ', lasth
      write(6,'(a,5x,a,i14)') '|', 'Integer   ', lasti
      write(6,'(a,5x,a,i14)') '|', 'Max Pairs ', lastpr
      write(6,'(a,5x,a,i14)') '|', 'Max Rstack', lastrst
      write(6,'(a,5x,a,i14)') '|', 'Max Istack', lastist
      write(6,'(a,5x,a,i14,a)') '|', '  Total   ', &
           (8*(lastr+lastrst) + 4*(lasth+lasti+lastpr+lastist))/1024, ' kbytes'


                                !add comments for read_inp version

      !        --- alloc memory for decomp module that needs info from mdread2
      if( idecomp == 1 .or. idecomp == 2 ) then
!         call allocate_real_decomp(nres)
      else if( idecomp == 3 .or. idecomp == 4 ) then
!         call allocate_real_decomp(npdec*npdec)
      end if

      !        ----- EVALUATE SOME CONSTANTS FROM MDREAD SETTINGS -----

      nr = nrp
      nr3 = 3*nr
      belly = ibelly > 0

      !        --- seed the random number generator ---

#if defined (MPI) && defined (REM)
      ! if not REM, call amrset
      if(rem <= 0) call amrset(ig)
#else
      call amrset(ig)
#endif

      if (nbit < 32 .and. nr > 32767) then
         write(6, *) '  Too many atoms for 16 bit pairlist -'
         write(6, *) '    Recompile without ISTAR2'
         call mexit(6, 1)
      end if

      if (ntp > 0.and.iabs(ntb) /= 2) then
         write(6,*) 'Input of NTP/NTB inconsistent'
         call mexit(6, 1)
      end if


end subroutine readmemorysizesfromparmfile 


subroutine  writecoord (x_coo, number_at) 

  integer  number_at, i
  _REAL_ , dimension (0:number_at*3):: x_coo 

      open (unit = 44, file = 'test.coord')

      write (44,2222) number_at
 2222 format (/I6)
         write (44, 1111) (x_coo ((i-1)*3), x_coo((i-1)*3+1), &
         x_coo((i-1)*3+2), i = 1,number_at) 
 1111    format (6f12.7)

      close (44)

end subroutine  writecoord

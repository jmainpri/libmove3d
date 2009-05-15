
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
 

   _REAL_ x (*), r_stack (*)
   integer  ix(*), ipairs (*), i_stack(*)
   character(len=4)  ih (*)


   !     ---- HERE BEGIN THE EXECUTABLE STATEMENTS ----


      !        --- finish reading the prmtop file and other user input:
#ifdef QMMM
      call rdparm2(x,ix,ih,ipairs,8,i_stack, &
                   ifqnt,nquant,labels,mlabel)
      call mdread2 (x,ix,ih,ipairs,r_stack,i_stack)
#else
      call rdparm2(x,ix,ih,ipairs,8,i_stack)
      call mdread2(x,ix,ih,ipairs,r_stack,i_stack)
#endif


                                !add comments for read_parm version

      !        --- alloc memory for decomp module that needs info from mdread2
!      if( idecomp == 1 .or. idecomp == 2 ) then
!         call allocate_real_decomp(nres)
!      else if( idecomp == 3 .or. idecomp == 4 ) then
!         call allocate_real_decomp(npdec*npdec)
!     end if

end subroutine readfromparmfile 

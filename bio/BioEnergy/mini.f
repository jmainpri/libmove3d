!++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+   Version of subroutine sander modified by S. Kirillova
!+
!+   SANDER  in AMBER_SRC/sander/sander.f  --  
!+           call subroutines to run MD or minimization calculations.
!+                                       
!++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

subroutine fortrancodeenergyminimisation (x_coo,x,ix,ih,ipairs,r_stack,i_stack,&
      ene, m_coo, number_at)
     
!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+
!+     x_coo -- coordinates of atoms
!+     x, ix, ih , ipairs, r_stack, i_stack -- variables for determination of 
!+                                             molecular topology and force field
!+     energy -- calculated value of energy
!+     m_coo  -- coordinates of atoms after minimization
!+     number_at -- number of atoms in molecule
!+
!++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!   implicit none
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
#  include "assert.h"

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
   _REAL_ ener(0:30),vir(4)
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

                               !  passed in variables 
                               
! integer mlastr
! integer mlasti
! integer mlasth
! integer mlastpr
! integer mlastrst
! integer mlastist
!   _REAL_ x (0:mlastr), r_stack (0:mlastrst)
!   integer  ix(0:mlasti), ipairs (0:mlastpr), i_stack(0:mlastist)
!   character(len=4)  ih (0:mlasth)


   _REAL_ x (*), r_stack (*)
   integer  ix(*), ipairs (*), i_stack(*)
   character(len=4)  ih (*)

   integer number_at 

   double precision x_coo(0:number_at*3)
   double precision m_coo(0:number_at*3)
   integer i
   integer init_min


   !     ---- HERE BEGIN THE EXECUTABLE STATEMENTS ----


                                ! determination of nr, erstop and qsetup  
                                ! for min_energy version
   nr = natom 
   erstop = .false.
   qsetup = .true.


!        ----- READ COORDINATES AND VELOCITIES -----
!               isn't need in this version, 
!                     add  comments     

!    call timer_start(TIME_RDCRD)
!#ifdef QMMM
!      nlink = 0
!      call getcor(nr,x(lcrd),x(lvel),x(lforce),ntx,box,irest,t,temp0, &
!                 nlink)
!#else
!#ifdef LES
!      call getcor(nr,x(lcrd),x(lvel),x(lforce),ntx,box,irest,t,temp0les)
!#else
!     call getcor(nr,x(lcrd),x(lvel),x(lforce),ntx,box,irest,t,temp0)
!#endif
!#endif
#ifdef MPI
#ifdef REM
      ! REM: if attempt is accepted, sette the target temperature to the
      ! new one and rescale velocities
      ! newTargetTemp and myScaling should always be negative if not REM
      if(rem == 1) then  
        if(newTargetTemp > 0.0) temp0 = newTargetTemp
        if(myScaling > 0.0) then
           do ii = 1, nr3
              x(lvel+ii-1) = x(lvel+ii-1) * myScaling
           end do
        end if
      else if(rem == 2) then
#ifdef LES
        if(igb /= 1) then
          write(6,*) ' partial REM (rem=2) only works with igb=1'
          call mexit(6, 1)
     endif
        if(newTargetTemp > 0.0) temp0les = newTargetTemp
        if(myScaling > 0.0) then
           do ii = 1, nr
             if(cnum(ii) > 0) then
              x(lvel+3*ii-3) = x(lvel+3*ii-3) * myScaling
              x(lvel+3*ii-2) = x(lvel+3*ii-2) * myScaling
              x(lvel+3*ii-1) = x(lvel+3*ii-1) * myScaling
             end if
           end do
        end if
#else
        write(6,*) '*******   For rem == 2, partial REM'
        write(6,*) 'use sander.LES with topology created by addles'
        call mexit(6, 1)
#endif      
      end if

#endif
#endif /* MPI */

#ifndef QMMM
      if( igb == 0 .and. induced == 1 ) call get_dips(x,nr)
#endif
!
      !        ----- SET THE INITIAL VELOCITIES -----

      if (ntx <= 3) call setvel(nr,x(lvel),x(lwinv),tempi,init,  &
                                iscale,scalm)

      if (belly) call bellyf(natom,ix(ibellygp),x(lvel))
!   call timer_stop(TIME_RDCRD)

      !        --- If we are reading NMR restraints/weight changes,
      !            read them now:

      if (nmropt >= 1) then
         call nmrcal(x(lcrd),f,ih(m04),ih(m02),ix(i02),x(lwinv),enmr, &
               devdis,devang,devtor,temp0,tautp,cut,ntb,x(lnmr01), &
               ix(inmr02),x(l95),5,6,rk,tk,pk,cn1,cn2, &
               ag,bg,cg,numbnd,numang,numphi,nimprp, &
               nttyp,nhb,natom,natom,ntypes,nres,rad,wel,radhb, &
               welhb,rwell,isftrp,tgtrmsd,temp0les,-1,'READ')

         !           --- Determine how many of the torsional parameters
         !               are impropers

         call impnum(ix(i46),ix(i56),ix(i48),ix(i58),nphih,nphia, &
               0,nptra,nimprp)
      end if

      !        --set up info related to weight changes for the non-bonds:


                                ! determination of coordinates for min_energy version
      nr3 = natom*3
      do i = 0, nr3-1
         x(lcrd+i) = x_coo(i)
      end do


      call nmrrad(rad,wel,cn1,cn2,ntypes,0,0.0d0)
      call decnvh(asol,bsol,nphb,radhb,welhb)

      if (iredir(4) > 0) call noeread(x,ix,ih)
      if (iredir(8) > 0) call alignread(natom, x(lcrd))

      !-----------------------------------------------------------------------
      !        --- Call FASTWAT, which will tag those bonds which are part
      !            of 3-point water molecules. Constraints will be effected
      !            for these waters using a fast analytic routine -- dap.


!     call timer_start(TIME_FASTWT )

                                !no 3-point solvent molecules in this version (fastwat and getwds) 
                                !add comments for min_energy version
  

!     call fastwat(ih(m04),nres,ix(i02),ih(m02), &
!           nbonh,nbona,ix(iibh),ix(ijbh),ibelly,ix(ibellygp), &
!           iwtnm,iowtnm,ihwtnm,jfastw,ix(iifstwt), &
!           ix(iifstwr),ibgwat,ienwat,iorwat, &
!           6,natom)
!     call timer_stop(TIME_FASTWT)
!
!     call getwds(ih(m04)   ,nres      ,ix(i02)   ,ih(m02)   , &
!           nbonh     ,nbona     ,0         ,ix(iibh)  ,ix(ijbh)  , &
!           iwtnm     ,iowtnm    ,ihwtnm    ,jfastw    ,ix(iicbh) , &
!           req       ,x(lwinv)  ,rbtarg    ,ibelly  ,ix(ibellygp), &
!           6)
#ifdef QMMM
      if(ifqnt.eq.0) then
         if(ntc.eq.3) then
            iqmshk(1:nbonh+nbona) = 1
         else if(ntc.eq.2) then
            iqmshk(1:nbonh)       = 1
            iqmshk(nbonh+1:nbona) = 0
         else if(ntc.eq.1) then
            iqmshk(1:nbonh+nbona) = 0
         else
              write(6,*)'NTC assigned impossible value:', ntc
              call mexit(6,1)
         end if
      end if

!     assign link atoms between quantum mechanical and molecular mechanical
!     atoms if quantum atoms are present
!
!     after assigning the link atoms delete all connectivity between the
!     QM atoms
!
      if(ifqnt.eq.1) then
!
          ipert = 0  ! dac trial
          npert = 0  ! dac trial
          klink = nlink
          call link_atoms(nbona,x(Lcrd),nlink,npert,ix(iiba), &
             ix(ijba),x(Lcrd+3*natom),nquant, &
             labels,natom,x(Lwinv+natom),mmqmbo(1),mmqmbo(2), &
             ix(icnstrgp+nr))
          if((imin.eq.0).and.(nlink.ne.0).and.(klink.eq.0)) then
              write(6,*) 'FATAL ERROR'
              write(6,*) 'Link atoms MUST be optimized before MD run'
              call mexit(6,1)
          end if

          if(nbonh.gt.0) then
              call del_bond(nbonh,ix(iibh),ix(ijbh),ix(iicbh), &
                  nquant,labels)
          end if

          if(nbona.gt.0) then
              itemp = nbona
              call del_bond2(nbona,ix(iibh+nbonh),ix(ijbh+nbonh), &
                  ix(iicbh+nbonh),ix(iiba),ix(ijba),ix(iicba),  &
                  nquant,labels)
              idiff = itemp - nbona
              nbona = nbona - idiff

!     adjust memory location pointers to reflect changes in bonding

              Iiba = Iibh + nbonh
              Ijba = Ijbh + nbonh
              Iicba = Iicbh + nbonh
          end if

!         now that all of the "qm bonds" have been deleted from the bond lists
!         need to reconstruct the SHAKE bond list

          dummy = 0.0d0
!         call bshake(nbonh,nbona,0,ix(i16),x(l50),req,dummy)

          if(ntheth.gt.0) then
              call del_angl(ntheth,ix(i24),ix(i26),ix(i28),ix(i30), &
                                                   nquant,labels)
          end if

          if(ntheta.gt.0) then
              call del_angl(ntheta,ix(i32),ix(i34),ix(i36),ix(i38), &
                                                   nquant,labels)
          end if

          if(nphih.gt.0) then
              call del_dihed(nphih,ix(i40),ix(i42),ix(i44),ix(i46), &
                                           ix(i48),nquant,labels)
          end if

          if(nphia.gt.0) then
              call del_dihed(nphia,ix(i50),ix(i52),ix(i54),ix(i56), &
                                           ix(i58),nquant,labels)
              mphia = nphia
          end if
!
!     set flags to run shake for bonds between mm atoms but not for
!     bonds between qm and mm atoms
!
         if(ntc.eq.3) then
            iqmshk(1:nbonh+nbona) = 1
         else if(ntc.eq.2) then
            iqmshk(1:nbonh)       = 1
            iqmshk(nbonh+1:nbona) = 0
         else if(ntc.eq.1) then
            iqmshk(1:nbonh+nbona) = 0
         else
              write(6,*)'NTC assigned impossible value:', ntc
              call mexit(6,1)
         end if
         if((nbonh.gt.0).and.(ntc.gt.1))then
            call ifshk(nbonh,ix(iibh),ix(ijbh),nquant,labels,iqmshk)
         end if
!
          if((nbona.gt.0).and.(ntc.gt.2))then
              call ifshk(nbona,ix(iiba),ix(ijba),nquant,labels, &
                                                iqmshk(nbonh+1))
          end if
!
!         Output any PMF information.
!
          if(ipert.ne.0) then
            if(npert.ne.0)then
              write(6,'(/" IN MINMD4, ATOMS INVOLVED IN THE PMF:")')
              do 10 i=1,npert
                write(6,'(/" GROUP ",i2,":")') i
                write(6,'(16i5)') (iatms(j),j=istrt(i),iendl(i))
                if(lnkend(i).gt.iendl(i))then
                  write(6,'(" LINK ATOMS:")')
                  write(6,'(16i5)') (iatms(j),j=iendl(i)+1,lnkend(i))
                endif
 10           continue
!
!             Compute and output initial center of mass or bonded atom
!             coordinates.  If user has specified all zeros for sxcm,
!             sycm, and szcm, then stop execution.
!
              call cmwrit(x(Lcrd),x(Lwinv),istop)
              if (istop.ne.0) then
                 write(6,*)'Error in cmwrit called from sander.F'
                 call mexit(6,1)
              endif
            endif
          endif
!
!         Assign any QM constraints.
!
          if(nquant.gt.1)then
            if(.not.do_scf)then
!
!             A slow growth FEP calculation to zero out QM van der
!             Waals parameters has been requested, and there is no
!             electrostatic coupling of the MM and QM systems.  In
!             this case, no QM scf calculations will be done, so the
!             QM system (i.e., solute) needs to be locked in a rigid
!             conformation to keep it from falling apart.  Generate
!             the appropriate constraints.
!
!             call rigid(x(Lcrd),nquant,labels)
            else
!
!             See if there is a user-defined list of constraints
!             in the file constraint.dat.
!
              call rdcnst(x(Lcrd),ierror)
              if (ierror.ne.0) then
                 write(6,*)'Error in rdcnst called from sander.F'
                 call mexit(6,1)
              endif

            endif
          endif
!
!         See if the user has constrained any atoms to lie in a plane.
!
          call rdpln(ierror)
          if (ierror.ne.0) then
             write(6,*)'Error in rdpln called from sander.F'
             call mexit(6,1)
          endif
!
!         See if the user has constrained sets of atoms to have the
!         same bond length.
!
          call rdsym(ierror)
          if (ierror.ne.0) then
             write(6,*)'Error in cmwrit called from sander.F'
             call mexit(6,1)
          endif
!
!
!     zero out the charges on the quantum mechanical atoms
!
          do 235 i=1,nquant
              index = L15 + labels(i) - 1
              x(index) = 0.0d0
 235      continue
      endif
!
#endif

      !        --- OPEN THE DATA DUMPING FILES AND POSITION IT DEPENDING
      !            ON THE TYPE OF RUN -----

      call open_dump_files
      if (master) then 
      call amflsh(6)
      !        --- end of master process setup ---
  end if  ! (master)


!cnt N.TAKADA:For MDM_START
#ifdef MDM_MD
   if (mdin_mdm) then
   !cnt   =========================== Open MDM ============================
                             !add comments for min_energy version
                             !OpenMDM uses min.in file, no min.in file here

!      call OpenMDM
   !cnt   =================================================================
   end if
#endif
!cnt N.TAKADA:For MDM_END


#ifdef MPI
   !     =========================== AMBER/MPI ===========================

   !     NOTE: in the current AMBER/MPI implementation, two means of
   !     running in parallel within sander are supported. The value
   !     of mpi_orig determines which approach is used.
   !     This is turned on when minimization (imin .ne. 0) is requested,
   !     and is otherwise off.

   !     When running the mpi_orig case, a variable notdone is now
   !     set by the master and determines when to exit the force()
   !     loop.  When the master has finished calling force, the
   !     master changes notdone to 0 and broadcasts the data one more
   !     time to signal end of the loop.  force() is modified so that
   !     in the mpi_orig case, an initial broadcast is done to receive
   !     the value from the master to decide whether to do the work or
   !     simply exit.

   !     ...set up initial data and send all needed data to other nodes,
   !     now that the master has it

   nr = nrp
   nr3 = 3*nr
   belly = ibelly > 0

   !     First, broadcast parameters in memory.h, so that all processors
   !     will know how much memory to allocate:

   call mpi_bcast(natom,BC_MEMORY,mpi_integer,0,commsander,ierr)
   call mpi_barrier(commsander,ierr)

   !     ---allocate memory on the non-master nodes:


                                ! add comments  for min_energy version

!   if( .not.master ) then
!     allocate( x(1:lastr), stat = ier )
!     REQUIRE( ier == 0 )
!
!     allocate( ix(1:lasti), stat = ier )
!     REQUIRE( ier == 0 )
!
!     allocate( ipairs(1:lastpr), stat = ier )
!     REQUIRE( ier == 0 )
!
!     allocate( ih(1:lasth), stat = ier )
!     REQUIRE( ier == 0 )
!
!     allocate( r_stack(1:lastrst), stat = ier )
!     REQUIRE( ier == 0 )
!
!     allocate( i_stack(1:lastist), stat = ier )
!     REQUIRE( ier == 0 )
!
!   end if  ! ( .not.master )

#ifndef QMMM
   call startup_groups(ierr)
#endif
   call startup(x,ix,ih)

                                ! add comments  for min_energy version

   !     ---allocate memory for GB on the non-master nodes:

!   if( .not.master ) then
!#ifndef QMMM
!     if(igb /= 0 .and. igb /= 10) call allocate_gb( natom )
!#endif
!   end if  ! ( .not.master )

#ifdef REM
   ! REM: call amrset if not REM. there is no need to do this everytime
   if(rem <= 0) call amrset(ig) 
#else
   call amrset(ig) 
#endif

   if (nmropt >= 1) &
         call nmrcal(x(lcrd),f,ih(m04),ih(m02),ix(i02),x(lwinv),enmr, &
         devdis,devang,devtor,temp0,tautp,cut,ntb,x(lnmr01), &
         ix(inmr02),x(l95),5,6,rk,tk,pk,cn1,cn2, &
         ag,bg,cg,numbnd,numang,numphi,nimprp, &
         nttyp,nhb,natom,natom,ntypes,nres,rad,wel,radhb, &
         welhb,rwell,isftrp,tgtrmsd,temp0les,-1,'MPI ')


   !    ---------------- Old parallel for minimization ----------------------

   if (imin /= 0) then
      mpi_orig = .true.
      notdone = 1
   else
      mpi_orig = .false.
   end if
   if (mpi_orig .and. .not.master) then

      !        ...all nodes only do the force calculations (JV)

#ifndef QMMM
      do while( notdone == 1 )
         call force(x,ix,ih,ipairs,x(lcrd),x(lforce),ener,vir, &
               r_stack,i_stack, x(l96), x(l97), x(l98), &
               do_list_update,qsetup)
      end do
#endif

      goto 999  ! deallocate and return
   end if
   !    ----------------------------------------------------------------------

   if (master) write(6, '(a,i4,a,/)') &
         '|  Running AMBER/MPI version on ',numtasks, ' nodes'
   if (master .and. numgroup > 1) write(6, '(a,i4,a,i4,a,i4,a)') &
         '|  MULTISANDER: ', numgroup, 'groups. ', &
         numtasks, ' processors out of ', worldsize, ' total.'
   if(master)call amflsh(6)

   !     ========================= END AMBER/MPI =========================
#endif /* MPI */

!  call date_and_time( setup_end_date, setup_end_time )


   ! ----------------------------------------------------------------------
   ! Now do the dynamics or minimization.
   ! ----------------------------------------------------------------------

   !     ---tad  pulled check neutral out of ew_setup
   !     debug needs to copy charges at start and they can't change later

#ifndef QMMM
   if( igb == 0 .and. iyammp == 0 ) &
         call check_neutral(x(l15),natom)

   ! use the debugf namelist to activate
   call debug_frc(x,ix,ih,ipairs,x(lcrd),x(lforce), &
         r_stack,i_stack,cn1,cn2)
#endif

   if( master ) write(6,'(/80(1H-)/''   4.  RESULTS'',/80(1H-)/)')

   ! Input flag imin determines the type of calculation: MD, minimization, ...

                                ! set cut for min_energy version
   cut = cut*cut

   select case ( imin )
   case ( 0 )
      !        --- Dynamics:

!      call timer_start(TIME_RUNMD)
#ifdef QMMM
      call runmd(x,ix,ih,ipairs, &
            x(lcrd),x(lwinv),x(lmass),x(lforce), &
            x(lvel),x(lvel2),x(l45),x(lcrdr), &
            x(l50),x(l95),ix(i70),x(l75),erstop,r_stack,i_stack, &
            ifqnt,nquant,labels, &
            mlabel,nlink,mmqmbo,iqmshk,iqmres,idc,qsetup)
#else
      call runmd(x,ix,ih,ipairs, &
            x(lcrd),x(lwinv),x(lmass),x(lforce), &
            x(lvel),x(lvel2),x(l45),x(lcrdr), &
            x(l50),x(l95),ix(i70),x(l75),erstop, &
            r_stack,i_stack,qsetup)
#endif
!      call timer_stop(TIME_RUNMD)

      if (master) call amflsh(6)

      if (erstop) then
         ! This error condition stems from subroutine shake;
         ! furthermore, it seems that erstop can never be true since shake
         ! can never return with its third last argument, niter, equal to 0.
         ! SRB, Sep 24, 2003
         if (master) then
            write(6, *) 'FATAL ERROR'
         end if
         call mexit(6,1)
      end if

   case ( 1 )

      !        --- Minimization:

      ! Input flag ntmin determines the method of minimization
      select case ( ntmin )
      case ( 0, 1, 2 )
#ifdef QMMM
         call runmin(x,ix,ih,ipairs,x(lcrd),x(lforce),x(lvel), &
               ix(iibh),ix(ijbh),x(l50),x(lwinv),ix(ibellygp), &
               x(l95),ene,r_stack,i_stack, carrms, &
               ifqnt,nquant,labels,mlabel,nlink,mmqmbo,iqmshk, &
               iqmres,idc,qsetup)
#else
                call runmin(x,ix,ih,ipairs,x(lcrd),x(lforce),x(lvel), &
               ix(iibh),ix(ijbh),x(l50),x(lwinv),ix(ibellygp), &
               x(l95),ene,r_stack,i_stack, carrms,qsetup)



                               ! determination of min_coordinates 
                               ! and energy for min_energy version            
 
      do i = 0, nr3-1
         m_coo(i) =  x(lcrd+i)
      end do

                                ! no XMIN and LMOD methods in this version
                                ! add comments  for min_energy 

!     case ( LMOD_NTMIN_XMIN )
!        write(6,'(a,i4)') '  LMOD XMIN Minimization.'
!        call run_xmin( x, ix, ih, ipairs, r_stack, i_stack, &
!              x(lcrd), x(lforce), ene, qsetup )
!     case ( LMOD_NTMIN_LMOD )
!        write(6,'(a,i4)') '  LMOD LMOD Minimization.'
!        call run_lmod( x, ix, ih, ipairs, r_stack, i_stack, &
!              x(lcrd), x(lforce), ene, qsetup )
#endif
      case default
         ! invalid ntmin
         ! ntmin input validation occurs in mdread.f
         ASSERT( .false. )
      end select

                                ! add comments  for min_energy 
#ifdef QMMM
      if (master) call minrit(x(lcrd),nlink)  ! Write the restart file
#else
      if (master) then
	call minrit(x(lcrd))  ! Write the restart file
      end if
#endif

   case ( 5 )
      !       ---carlos modified for reading trajectories (trajene option)

      write (6,*) "POST-PROCESSING OF TRAJECTORY ENERGIES"

      !       ---read trajectories and calculate energies for each frame

      call trajectoryenergy(x,ix,ih,ipairs, ene,ok,r_stack,i_stack,qsetup)

      if (.not.ok) then
         write (6,*) 'error in trajectoryenergy()'
         call mexit(6,1)
      end if

   case default
      ! invalid imin
      ! imin input validation should be transferred to mdread.f
      write(6,'(/2x,a,i3,a)') 'Error: Invalid IMIN (',imin,').'
      ASSERT( .false. )
   end select

   !     -- calc time spent running vs setup

!   call timer_stop(TIME_TOTAL)
!   call wallclock( time1 )
!   call date_and_time( final_date, final_time )
!#ifdef NO_DETAILED_TIMINGS
!#else
!  call profile_time( time1 - time0 )
!#endif

#ifdef MPI
   !     =========================== AMBER/MPI ===========================

   !     Set and broadcast notdone in mpi_orig case to inform
   !     other nodes that we are finished calling force(). (tec3)

   if ( mpi_orig ) then
      notdone = 0
      call mpi_bcast(notdone,1,mpi_integer,0, commsander,ierr)
   end if

   !     ========================= END AMBER/MPI =========================
#endif

   if( master ) then
      call close_dump_files

      !     --- write out final times

    !write(6,'(12(a))') '|           Job began  at ', initial_time(1:2), &
    !       ':', initial_time(3:4), ':', initial_time(5:10), '  on ',&
    !       initial_date(5:6), '/', initial_date(7:8), '/', initial_date(1:4)
    !  write(6,'(12(a))') '|           Setup done at ', setup_end_time(1:2),  &
    !       ':', setup_end_time(3:4), ':', setup_end_time(5:10), '  on ', &
    !       setup_end_date(5:6), '/',setup_end_date(7:8),'/',setup_end_date(1:4)
    ! write(6,'(12(a))') '|           Run   done at ', final_time(1:2),  &
    !       ':', final_time(3:4), ':', final_time(5:10), '  on ', &
    !       final_date(5:6), '/', final_date(7:8), '/', final_date(1:4)
    !  call nwallclock( ncalls )
    ! write(6, '(''|'',5x,''wallclock() was called'',I8,'' times'')') ncalls

                               
      if(iesp > 0) then
         call calcul_esp(natom,x(lcrd),x(linddip))
      end if
   end if

999 continue  !     --- dynamic memory deallocation:

#ifdef QMMM
   ! hack to bypass a deallocation error in sander.QMMM
   call mexit(6,0)
#endif

                                ! add comments  for min_energy version
!  if (master) then  
!     if( idecomp > 0 ) then
!        call deallocate_real_decomp()
!        call deallocate_int_decomp(idecomp)
!     endif
!  endif
!  deallocate( i_stack, stat = ier )
!  REQUIRE( ier == 0 )
!  deallocate( r_stack, stat = ier )
!  REQUIRE( ier == 0 )
!  if( igb /= 0 .and. igb /= 10 ) then
!ifndef QMMM
!     call deallocate_gb( )
!endif
!  end if
!  deallocate( ih, stat = ier )
!  REQUIRE( ier == 0 )
!  deallocate( ipairs, stat = ier )
!  REQUIRE( ier == 0 )
!  deallocate( ix, stat = ier )
!  REQUIRE( ier == 0 )
!  deallocate( x, stat = ier )
!  REQUIRE( ier == 0 )

   return

end subroutine fortrancodeenergyminimisation

!*********************************************************************
!               SUBROUTINE TRAJENE
!          in AMBER_SRC/sander/sander.f 
!*********************************************************************
! carlos add trajene routine for processing trajectory energies

!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!+ [Enter a one-line description of subroutine trajene here]
subroutine trajectoryenergy(x,ix,ih,ipairs,ene,ok, r_stack,i_stack,qsetup)

   implicit none
   integer ipairs(*),i_stack(*)
   _REAL_ r_stack(*),carrms

   _REAL_ x(*),ene(*)
   integer ix(*)
   character(len=4) ih(*)
   logical ok,qsetup
   integer member,j

#  include "memory.h"
#  include "tgtmd.h"
#  include "extra.h"
#  include "box.h"

   member=0

   !     loop over trajectory file, exiting only on error or end of file

   do while ( .true. )

      !       --- read next coordinate set from trajectory

      read(12,110,end=1000,err=1010) (x(j),j=lcrd,lcrd+natom*3-1)

      if (ifbox > 0) read(12,110,end=1000,err=1010)

      !       --- uncomment this to force box read if prmtop doesn't have it
      !           but traj does:
      !        READ(12,110,END=1000,ERR=1010)
      110 format(10f8.3)

      member=member+1

      write (6,'(a,i6)') 'minimizing coord set #',member

      call runmin(x,ix,ih,ipairs,x(lcrd),x(lforce),x(lvel), &
            ix(iibh),ix(ijbh),x(l50),x(lwinv),ix(ibellygp), &
            x(l95),ene,r_stack,i_stack,carrms,qsetup)

      write (6,364) ene(1),carrms
      364 format ('minimization completed, ENE=',1x,e12.6, &
            1x,'RMS=',1x,e12.6)

      if (master .and. itgtmd == 1) then
         write (6,'(a,f8.3)') "Final RMSD from reference: ",rmsdvalue
      end if

      !       ---loop for next coordinate set

   end do

   !     ---end of trajectory file

   1000 write (6,*) "Trajectory file ended"
   ok=.true.
   return

   1010 write (6,*) "Error in trajectory file"
   ok=.false.
   return

end subroutine trajectoryenergy
!-----------------------------------------------------------------


subroutine calcul_esp(natom,x,mom_ind)

   ! routine to calculate the ESP due to the induced moments (only)
   ! at the same spatial points as the reference QM.

   implicit none
   integer natom
   _REAL_  x(3,*)
   _REAL_  mom_ind(3,*)

#  include "files.h"
#  include "ew_mpole.h"
#  include "constants.h"

   integer dat_unit, new_unit, minus_new_unit
   parameter(dat_unit=30, new_unit=31, minus_new_unit=33)


   integer inat, nesp, idum
   _REAL_  xin, yin, zin
   integer jn, kn
   _REAL_  esp_qm, xb_esp, yb_esp, zb_esp
   _REAL_  x_esp, y_esp, z_esp
   _REAL_  e_x, e_y, e_z, e_q, esp_new
   _REAL_  dist, dist3
   integer iptr

   call amopen(dat_unit,"esp.dat",'O','F','R')
   call amopen(new_unit,"esp.induced",owrite,'F','W')
   call amopen(minus_new_unit,"esp.qm-induced",owrite,'F','W')
   read (dat_unit,'(3i5)')inat,nesp,idum
   write(6,'(t2,''inat = '',i5)')inat
   write(6,'(t2,''nesp = '',i5)')nesp

   write(new_unit,'(2i5)')inat,nesp
   write(minus_new_unit,'(2i5)')inat,nesp

   if (inat /= natom) then
      write(6,'(t2,''natom mismatch with esp file'')')
      call mexit(6,1)
   end if
   do jn = 1,inat
      read (dat_unit,'(17x,3e16.0)')xin,yin,zin
      write(new_unit,'(17x,3e16.7)')xin,yin,zin
      write(minus_new_unit,'(17x,3e16.7)')xin,yin,zin
   end do
   do jn = 1,nesp
      e_x = 0.0d0
      e_y = 0.0d0
      e_z = 0.0d0
      e_q = 0.0d0
      read(dat_unit,'(1x,4e16.0)')esp_qm,xb_esp,yb_esp,zb_esp
      x_esp = xb_esp * bohr_radius
      y_esp = yb_esp * bohr_radius
      z_esp = zb_esp * bohr_radius
      do kn = 1,natom
         dist = (sqrt((x(1,kn)-x_esp)**2 + &
               (x(2,kn)-y_esp)**2 + &
               (x(3,kn)-z_esp)**2))
         dist3 = dist**3
         e_x = e_x - mom_ind(1,kn   )*(x(1,kn)-x_esp)/dist3
         e_y = e_y - mom_ind(2,kn   )*(x(2,kn)-y_esp)/dist3
         e_z = e_z - mom_ind(3,kn   )*(x(3,kn)-z_esp)/dist3
      end do
      e_x = e_x * bohr_radius/18.2223d0
      e_y = e_y * bohr_radius/18.2223d0
      e_z = e_z * bohr_radius/18.2223d0
      e_q = e_q * bohr_radius/18.2223d0
      esp_new = e_x + e_y + e_z

      write(new_unit,      '(1x,4e16.7)')esp_new &
            ,xb_esp,yb_esp,zb_esp
      write(minus_new_unit,'(1x,4e16.7)')esp_qm-esp_new &
            ,xb_esp,yb_esp,zb_esp
   end do

   close(dat_unit)
   close(new_unit)
   close(minus_new_unit)
   return
end subroutine calcul_esp

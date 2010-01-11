      subroutine diagrtb (nompdb,lnompdb,coo_x,natom,vec_x,nvec)
!
!     DIAGonalisation of a matrix with the RTB method.
!     ************************************************
!     Required eigenvectors are supposed to be well
!     described by rigid body motions (Rotations-Translations)
!     of blocks of atoms buildt with sets of consecutive
!     residues, as defined in the corresponding PDB file.
!
!     Output: 
!     *******
!     matrix.eigenrtb, with the eigenvalues and
!     eigenvectors, in x,y,z "CERFACS" format.
!
!     Note that the input matrix is assumed to be:
!     a) real, square and symmetrical.
!     b) in the following format: i, j, i-j-matrix-element
!
!     In the present state of the program, the matrix
!     is first re-written in a block-by-block form.
!     Temporary (small size, and sometimes now useless) 
!     rtb.* files are also created during each program run.
!     
!     References: 
!     ***********
!     1) P. Durand, G. Trinquier, Y.H. Sanejouand (1994):
!    "A new approach for determining low-frequency
!     normal modes in macromolecules", 
!     Biopolymers vol.34, p759-771.
!
!     2) F. Tama, F.X. Gadea, O. Marques, Y.H. Sanejouand (2000):
!    "Building-block approach for determining 
!     low-frequency normal modes of macromolecules", 
!     Proteins: Structure, Function, and Genetics vol.41(1), p1-7.
!
!     Version 2000 by Florence Tama (ftama@scripps.edu).
!
!     Modified by Osni Marques and Yves-Henri Sanejouand.
!     Last modifications: September 2002.
!
!     In case of problem, feel free to contact: 
!     sanejouand@crpp.u-bordeaux.fr
!     =======================================================
!
!     LIMITS for the PDB file:
!    (little computer memory required)
!
!     natmax    = maximum number of atoms in the system.
!     natresmax = maximum number of atoms in each block.
!     nresmax   = maximum number of residues and of blocks.
!     nbrt      = number of degrees of freedom of each block.
!
!     LIMITS for the matrix:
!    (computer memory required can be large,
!     when the size of the largest block is large)
!    
!     llwork    = size of the LOGICAL working array.
!     liwork    = size of the INTEGER working array.
!     lrwork    = size of the DOUBLE PRECISION working array.
 
      implicit         none
      integer          natmax, natresmax, nbrt, nresmax 
      integer          maxsize
 
!     ===================================
      parameter        (natmax    =50000)
      parameter        (natresmax =  500)
      parameter        (nbrt      =    6)
      parameter        (nresmax   =50000)
      parameter        (maxsize=50000000)
!     ===================================
 
      integer          liwork, llwork, lrwork
 
!     ====================================
      parameter        (llwork = 3*natmax)
      parameter        (liwork = 1000000)
      parameter        (lrwork =20000000)
!     parameter        (lrwork =43600000) 
!     ====================================
 
      logical          LWORK(llwork)
      integer          IWORK(liwork)
      double precision RWORK(lrwork)
 
      character        clean*4, eige*4, field*64, fmtx*256, fpdb*256, &
                       macro*4, mass*4, program*9, progrer*12, &
                       progrwn*12, string*64, version*30 
      logical          alive, qexist, qexistm, qexistp
      integer          i, iiwkm1, iiwkm2, &
                       iiwkp1, iiwkp2, iiwkp3, &
                       iiwrk1, iiwrk2, iiwrk3, &
                       irwkd1, irwkd2, irwkd3, irwkd4, &
                       irwkm1, irwkm2, irwkm3, irwkm4, irwkm5, iiwkmx, &
                       irwkp1, irwkp2, irwkmx, &
                       irwrk1, irwrk2, irwrk3, irwrk4, irwrk5, irwrk6, &
                       irwrk7, irwrk8, irwrk9, &
                       natblocs, natom, nb, &
                       nddres, nddres2, ndim, nrbl, nvec

!----------------------------------------------------------------------
      integer lnompdb,k
      character (len=lnompdb) nompdb
      double precision coo_x(0:3*natom)
      double precision  vec_x (0:nvec*3*natom)
!----------------------------------------------------------------------

!
!-----------------------------------------------------------------------
      version=' Version 2.11, September 2002.'
 
      program=' Diagrtb>'
      progrwn='%Diagrtb-Wn>'
      progrer='%Diagrtb-Er>'
      write(6,'(2A)') program, version
 
!.... defaults .........................................................
!     These values can be modified in 'diagrtb.dat'.
 
!     Number of residues per block:
      nrbl = 1
      
!     PDB : Look for masses in the B-factors column of the pdb file.
!     CONS: Set them to a constant value (mass=1).
      mass = 'CONS'
!
!     Number of eigenvectors to compute:
!---------------------------------------------------
!      in move3d + elnemo version 'nvec' is argument, 
!      add comments
!---------------------------------------------------
!      nvec = 56
      
!     LOWE: The lowest-eigenvalues ones.
!     HIGH: The highest-eigenvalues ones.
      eige = 'LOWE'
 
!     ALL: Work files are deleted at the end of the job.
!     NO : They are kept.
      clean= 'ALL'
 
!     PDB default filename with the coordinates of the system.
!      fpdb = 'structure.pdb'

!     Matrix default filename with the non-zero elements.
      fmtx = nompdb//'.sdijf'
 
!.... read options from file diagrtb.dat ..............................
 
      inquire(file='diagrtb.dat',exist=qexist)
      if (.not.qexist) then
          write(6,'(2A)') progrwn, &
        ' No diagrtb.dat file. Defaults assumed for all options.'
          goto 15
      endif
 
      open (10,file='diagrtb.dat')
 5    continue 
 
      read (10,end=10,err=20,fmt='(a)') string
      call parser (string,field)
      if (field(1:1).eq.'!'.or.field(1:1).eq.'*'.or. &
             field(1:1).eq.'#') goto 5

      read (field,'(a)') macro
      if      ( macro.eq.'NRBL' .or. macro.eq.'nrbl' ) then
                call parser (string,field)
                read (field,'(i16)') nrbl 
      else if ( macro.eq.'NVEC' .or. macro.eq.'nvec' ) then
                call parser (string,field)
                read (field,'(i16)') nvec 
      else if ( macro.eq.'FPDB' .or. macro.eq.'fpdb' ) then
                call parser (string,field)
                read (field,'(a)') fpdb 
      else if ( macro.eq.'FMTX' .or. macro.eq.'fmtx' ) then
                call parser (string,field)
                read (field,'(a)') fmtx 
      else if ( macro.eq.'MASS' .or. macro.eq.'mass' ) then
                call parser (string,field)
                read (field,'(a)') mass 
                if (mass.eq.'pdb') mass='PDB'
                if (mass.eq.'cons') mass='CONS'
                if (mass.ne.'PDB'.and.mass.ne.'CONS') then
                 write(6,'(3A)') program,mass, &
               ' is not a known MASS option (try PDB or CONS).'
                 stop
                endif
      else if ( macro.eq.'EIGE' .or. macro.eq.'eige' ) then
                call parser (string,field)
                read (field,'(a)') eige 
                if (eige.eq.'lowe') eige='LOWE'
                if (eige.eq.'high') eige='HIGH'
                if (eige.ne.'LOWE'.and.eige.ne.'HIGH') then
                 write(6,'(A,1X,A)') program,eige, &
               ' is not a known EIGE option (try HIGH or LOWE).'
                 stop
                endif
      else if ( macro.eq.'CLEA' .or. macro.eq.'clea' ) then
                call parser (string,field)
                read (field,'(a)') clean
                if (clean.eq.'all') clean='ALL'
                if (clean(1:2).eq.'no'.or.clean(1:2).eq.'NO')  &
                    clean='NO'
                if (clean.ne.'ALL'.and.clean.ne.'NO') then
                 write(6,'(3A)') program,clean, &
               ' is not a known clean option (try ALL or NO).'
                 stop
                endif
      else    
                write(6,'(3A)') program,macro, &
              ' is not a known option.'
                stop
      end if
!
      goto 5
   10 close (10)
   15 continue
!

      call pdbmat (nompdb,lnompdb, coo_x, natom)
! by AS. Under Linux we receive an error at runtime that the file "nompdb" is already opened, so we make sure that it is closed
!   after having executed "call pdbmat "
      inquire (file=nompdb//'.pdb',opened=alive,number=i)
      if ( alive ) then
         close (i,status='keep')
      end if

      fpdb = nompdb//'.pdb'

      write(6,'(/2A)') program,' Options to be taken into account:'
      write(6,'(A,I6,A)') &
          ' NRBL= ',nrbl,' residue(s) per bloc.', &
          ' NVEC= ',nvec,' eigenvectors required, those with the:'
      write(6,'(A,2X,2A)') &
          ' EIGE= ',eige,'-est eigenvalues.'
      write(6,'(A,2X,2A)') &
          ' MASS= ',mass
      write(6,'(A,2X,2A)') & 
          ' CLEA= ',clean
      write(6,'(A/(2A))') &
          ' For the following structure and the corresponding matrix:', &
          ' FPDB= ',fpdb, &
          ' FMTX= ',fmtx
!
      inquire(file=fpdb,exist=qexistp)
      inquire(file=fmtx,exist=qexistm)
!
      if (.not.qexistp) &
      write(6,'(2A)') progrer,' PDB file not found. Required !' 
      if (.not.qexistm) &
      write(6,'(2A)') progrer,' Matrix not found. Required !'
      if (.not.qexistp.or..not.qexistm) stop
!
!.... pointers for BLOCPDB .............................................
!
      write(6,'(/2A)') program,' Memory allocation for Blocpdb:'
      iiwrk1 = 1
      iiwrk2 = iiwrk1 + nresmax
      iiwrk3 = iiwrk2 + nresmax
      iiwkmx = iiwrk3 + natmax
      write(6,'(A,I12)') ' IIWKMX= ',IIWKMX
      if ( iiwkmx.gt.liwork ) STOP 'Quitting: IIWKMX > LIWORK'
!
      irwrk1 = 1
      irwrk2 = irwrk1 + 3*natmax
      irwrk3 = irwrk2 + 3*natmax
      irwrk4 = irwrk3 + natmax
      irwrk5 = irwrk4 + natmax
      irwrk6 = irwrk5 + natmax
      irwkmx = irwrk6 + natmax
      write(6,'(A,I12)') ' IRWKMX= ',IRWKMX
      if ( irwkmx.gt.lrwork ) STOP 'Quitting: IRWKMX > LRWORK'

!       do k=1,natom
!        write (6, *) k,coo_x((k-1)*3),coo_x((k-1)*3+1),coo_x((k-1)*3+2)
!        enddo


!
      CALL BLOCPDB ( fpdb, mass, natmax, natresmax, nrbl, nresmax, &
                     IWORK(iiwrk2), IWORK(iiwrk3), IWORK(iiwrk1), &
                     RWORK(irwrk1), RWORK(irwrk2), RWORK(irwrk3), &
                     RWORK(irwrk4), RWORK(irwrk5), RWORK(irwrk6), &
                     natblocs, natom, nb, coo_x )
!
!.... pointers for PREPMAT .............................................
!
!     Note: The first nresmax entries of IWORK (on output from BLOCPDB)
!           are required in PREPMAT, so we don't change iiwrk2.
!
      write(6,'(/2A)') program,' Memory allocation for Prepmat:'
      iiwkp1 = iiwrk1
      iiwkp2 = iiwrk2
      iiwkp3 = iiwkp2 + 9*natblocs**2
      iiwkmx = iiwkp3 + 9*natblocs**2
      write(6,'(A,I12)') ' IIWKMX= ',IIWKMX
      if ( iiwkmx.gt.liwork ) STOP 'Quitting: IIWKMX > LIWORK'
 
      irwkp1 = 1
      irwkp2 = irwkp1 + 9*natblocs*natom
      irwkmx = irwkp2 + 9*natblocs*natblocs
!     irwkmx = irwkp2 + 9*natblocs*natom      -trop ?
      write(6,'(A,I12)') ' IRWKMX= ',IRWKMX
      if ( irwkmx.gt.lrwork ) STOP 'Quitting: IRWKMX > LRWORK'
 
!.... pointers for RTB .................................................
 
      write(6,'(2A)') program,' Memory allocation for RTB:'
      nddres = 3*natblocs
      nddres2 = nddres**2
 
      iiwrk1 = 1
      iiwrk2 = iiwrk1 + nddres2
      iiwrk3 = iiwrk2 + nddres2 
      iiwkmx = iiwrk3 + nb
      write(6,'(A,I12)') ' IIWKMX= ',IIWKMX
      if ( iiwkmx.gt.liwork ) STOP 'Quitting: IIWKMX > LIWORK'
 
      irwrk1 = 1
      irwrk2 = irwrk1 + 3*natom
      irwrk3 = irwrk2 + 3*natom
      irwrk4 = irwrk3 + nbrt*nddres
      irwrk5 = irwrk4 + nddres2
      irwrk6 = irwrk5 + nddres*nbrt 
      irwrk7 = irwrk6 + nddres2
      irwrk8 = irwrk7 + 36*nb**2 
      irwrk9 = irwrk8 + nbrt*nddres*nb
      irwkmx = irwrk9 + nbrt*nbrt
      write(6,'(A,I12)') ' IRWKMX= ',IRWKMX
      if ( irwkmx.gt.lrwork ) STOP 'Quitting: IRWKMX > LRWORK'
!
!.... pointers for DIAGSTD .............................................
 
      write(6,'(2A)') program,' Memory allocation for Diagstd:'
      ndim = 6*nb
 
      iiwkmx = ndim
      write(6,'(A,I12)') ' IIWKMX= ',IIWKMX
      if ( iiwkmx.gt.liwork ) STOP 'Quitting: IIWKMX > LIWORK'

      irwkd1 = 1
      irwkd2 = irwkd1 + ndim*ndim
      irwkd3 = irwkd2 + ndim
      irwkd4 = irwkd3 + ndim
      irwkmx = irwkd4 + ndim
      write(6,'(A,I12)') ' IRWKMX= ',IRWKMX
      if ( irwkmx.gt.lrwork ) STOP 'Quitting: IRWKMX > LRWORK'
 
      if (ndim.lt.nvec) then
          nvec = ndim
          write(6,'(A,I9,A)') progrwn, &
          nvec,' vectors, only, can be determined.'
      endif
 
!.... pointers for RTBTOMODES ..........................................
!
      write(6,'(2A)') program,' Memory allocation for RTB_to_modes:'
      iiwkm1 = 1
      iiwkm2 = iiwkm1 + nb
      iiwkmx = iiwkm2 + nvec
      if ( iiwkmx.gt.liwork ) STOP 'Quitting: IIWKMX > LIWORK'
 
      irwkm1 = 1
      irwkm2 = irwkm1 + 3*natom*nvec
      irwkm3 = irwkm2 + nvec
      irwkm4 = irwkm3 + 6*nb*nvec
      irwkm5 = irwkm4 + nvec
      irwkmx = irwkm5 + nbrt*nddres*nb
      write(6,'(A,I12)') ' IRWKMX= ',IRWKMX
      if ( irwkmx.gt.lrwork ) STOP 'Quitting: IRWKMX > LRWORK'

  
! by AS. Under Linux we receive an error at runtime (in "CALL PREPMAT" below) that the file "fmtx" is already opened , so we make sure that it is closed
      inquire (file=fmtx,opened=alive,number=i)
      if ( alive ) then
         close (i,status='keep')
      end if

      CALL PREPMAT ( fmtx, natblocs, natom, nb, LWORK, IWORK(iiwkp2), &
                     IWORK(iiwkp3), IWORK(iiwkp1), RWORK(irwkp1), &
                     RWORK(irwkp2))
 
      CALL RTB     ( natom, nb, nbrt, nddres, nddres2, IWORK(iiwrk1), &
                     IWORK(iiwrk2), IWORK(iiwrk3), RWORK(irwrk1),&
                     RWORK(irwrk2), RWORK(irwrk3), RWORK(irwrk4),&
                     RWORK(irwrk5), RWORK(irwrk6), RWORK(irwrk7),&
                     RWORK(irwrk8), RWORK(irwrk9) )
 
      CALL DIAGSTD ( nompdb,lnompdb, ndim, nvec, eige, IWORK, &
                     RWORK(irwkd1), RWORK(irwkd2),&
                     RWORK(irwkd3), RWORK(irwkd4) )
      


      CALL RTBTOMODES (nompdb,lnompdb, natom, nb, nbrt, nddres, nvec,  &
                        IWORK(iiwkm1), &
                        IWORK(iiwkm2), RWORK(irwkm1), RWORK(irwkm2), &
                        RWORK(irwkm3), RWORK(irwkm4), RWORK(irwkm5), &
                        vec_x )
 
!.... clean up .........................................................
 
      if (clean.eq.'ALL') then
      inquire (file='rtb.pdb',exist=alive)
      if ( alive ) then
         open  (10,file='rtb.pdb')
         close (10,status='delete')
      end if
      inquire (file='pdbmat.xyzm',exist=alive)
      if ( alive ) then
         open  (10,file='pdbmat.xyzm')
         close (10,status='delete')
      end if
      inquire (file='rtb.blocs',exist=alive)
      if ( alive ) then
         open  (10,file='rtb.blocs')
         close (10,status='delete')
      end if
      inquire (file='rtb.matblocs',exist=alive)
      if ( alive ) then
         open  (10,file='rtb.matblocs')
         close (10,status='delete')
      end if
      inquire (file='rtb.sdijb',exist=alive)
      if ( alive ) then
         open  (10,file='rtb.sdijb')
         close (10,status='delete')
      end if
     inquire (file='rtb.eigenfacs',exist=alive)
      if ( alive ) then
         open  (10,file='rtb.eigenfacs')
         close (10,status='delete')
      end if
      end if
!
!.... End of diagrtb ..................................................
!
      write(6,'(/2A)') program,' Normal end.'
      return
   20 write(6,'(/2A)') program,'* I/O error, check file diagrtb.dat'
	return -1
      end

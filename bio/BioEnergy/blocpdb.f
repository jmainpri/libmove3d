      SUBROUTINE BLOCPDB ( fpdb, mass, natmax, natresmax, nbb, nresmax, &
                           ntab, &
                           resi, tabb, amass, corlin, massat, xat, yat, &
                           zat, natblocs, natom, nb,coo_x )
 
!     Scalar Arguments
!
      character        fpdb*(*), mass*(*)
      integer          natblocs, natmax, natom, natresmax, nb, nresmax 
 
!     Array Arguments
!
      integer          ntab(nresmax), resi(natmax), tabb(nresmax)
      double precision amass(3*natmax), corlin(3*natmax), &
                       massat(natmax), xat(natmax), &
                       yat(natmax), zat(natmax)
 
!     Purpose:
!     =======
!
!     -------------------------------------------------------------
!    Atomic coordinates are read and block lengths are determined,
!    as a consequence of the number of residues per block.
!    -------------------------------------------------------------
!   (The peptidic bond between residues is simply split)

!     Arguments
!     =========
!
!     natmax    : maximum number of atoms in the system.
!     natresmax : maximum number of atoms in each block.
!     nresmax   : maximum number of residues and of blocks.
!     nbb       : number of residues per block.
!     ntab      : working array.
!     resi      : residue number each atom belongs to.
!     tabb      : block lengths.
!     amass     : coordinate masses.
!     corlin    : coordinates.
!     massat    : atomic masses.
!     xat       : atomic x-coordinate.
!     yat       : atomic y-coordinate.
!     zat       : atomic z-coordinate.
!     natblocs  : maximum number of atoms found in blocks.
!     natom     : number of atoms in the system.
!     nb        : number of blocks the system is split into.
! 
!     Version 2000 by Florence Tama (ftama@scripps.edu).
!
!     Modified by Osni Marques and Yves-Henri Sanejouand.
!     Last modifications: March 2001.
!-----------------------------------------------------------------------
!
!     Local variables
!
      character*4      atname, resname, sid
      character*9      program
      character*12     progrwn, progrer
      character*80     lign80
      logical          qmass
      integer          at, i, ii, j, k, lgbloc, nbb, nbbcur, nbblast, &
                       nn, no, restst, natbmin
      double precision tot
!
!-----------------------------------------------------------------------

      double precision coo_x(0:3*natom)
 
      write(6,*) '==================== Begin of Blocpdb 2.08.'
      program=' Blocpdb>'
      progrwn='%Blocpdb-Wn>'
      progrer='%Blocpdb-Er>'
!
!     Fichiers d'entree 
!     -----------------
      open(unit=72,file=fpdb,status='old',form='formatted')
      rewind(72)
      
!     Fichiers de sortie
!     ------------------
      open(unit=74,file='rtb.pdb',status='unknown',form='unformatted')

      if (nbb.le.0) then
          write(6,'(A,I6,A)') progrer,nbb, &
        ' residues per bloc required. Not a positive number !'
          stop
      endif
!
!   ----------------------------------
!   Pdb file with coordinates is read:
!   ----------------------------------
      i=0
      nb=1
      qmass=.true.
      if (mass.eq.'CONS') qmass=.false.
 
 25   continue
      read(72,'(A)',end=30) lign80
      if (lign80(1:4).ne.'ATOM') goto 25
      i=i+1
      read(lign80, '(6X,I5,1X,A4,1X,A4,1X,I4)') at,atname,resname,resi(i)
      read(lign80, '(30X,F8.3,F8.3,F8.3,1X,F5.2,F6.2,6X,A4)') xat(i),yat(i),zat(i)
      read(lign80, '(55X,F5.2,F6.2,6X,A4)') tot,massat(i),sid  
      if (i.gt.1.and.resi(i).ne.resi(i-1)) nb=nb+1
      if (massat(i).le.0.and.qmass) then
          write(6,'(2A,I6,A,F8.2)') progrwn,' Atom ',i, &
              ' Mass read in B-factor column= ',massat(i)
          qmass=.false.
      endif
      goto 25
 30   continue
 
      natom=i
      write(6,'(I6,A)') natom,' atoms in pdb file.'
      if (natom.le.0) stop
      if (natom.gt.natmax) then
          write(6,'(/A,I6,A)') progrer,natmax, &
              ' is the maximum allowed. Sorry.'
          stop
      endif

      write(6,'(I6,A)')  nb,' residues.'
      if (nb.le.0) stop
      if (nb.gt.nresmax) then
          write(6,'(/A,I6,A)') progrer,nresmax, &
              ' is the maximum allowed. Sorry.'
          stop
      endif

      no=nb/nbb
      nbblast=nbb
      if (mod(nb,nbb).ne.0) then 
          no=no+1
          nbblast=mod(nb,nbb)
      endif

      write(6,'(I6,A)') nbb,' residue(s) per block.'

      if (no.le.0) then
          write(6,'(2A)') progrer,' Not enough ...'
!         stop
      endif

      if (nbblast.ne.nbb) then
      write(6,'(I6,A)') nbblast,' residues in last block.'
      endif

      if (.not.qmass.or.mass.eq.'CONS') then
         do i=1,natom
            massat(i)=1.d0
         enddo
         if (mass.eq.'PDB') then
             write(6,'(2A)') progrwn,' All masses set to unity,' // &
          ' non-positive one(s) being found in pdb file.'
         else
             write(6,'(A)') ' All masses set to unity.'
         endif
      else 
         write(6,'(A)') ' Masses were read in pdb file.'
      endif

!------------------------------------------------------
!      in move3d - elnemo version 
!      use coo_x instead of coordinates of pdb file
!------------------------------------------------------
           do k=1,natom
              xat(k) = coo_x((k-1)*3) 
              yat(k) = coo_x((k-1)*3+1)
              zat(k) = coo_x((k-1)*3+2)
!          write (6,*) k, coo_x((k-1)*3),coo_x((k-1)*3+1),coo_x((k-1)*3+2) 
             enddo
!
!   ---------------------------------------------------------
!   Creation du fichier rtb.pdb
!   Ce fichier contient:
!          1- les coordonnes 
!          2- les masses
!   ---------------------------------------------------------

      do i=1,natom
	 ii=3*i-2
	   corlin(ii)=xat(i)
	   corlin(ii+1)=yat(i)
	   corlin(ii+2)=zat(i)
	   amass(ii)=massat(i)
	   amass(ii+1)=massat(i)
	   amass(ii+2)=massat(i)
      enddo

      write(6,'(A)') ' Pdb file is rewritten.'
      write(74) natom
      write(74) (corlin(ii),ii=1,3*natom)
      write(74) (amass(ii),ii=1,3*natom)
!
!    ------------------------
!    Longueur de chaque bloc:
!    ------------------------

      restst=resi(1)
      lgbloc=0
      nn=0
      do i=1,nb
         TABB(i)=0
      enddo

      write(6,'(A)') ' Residue sizes are stored.'
      do i=1,natom
	 if (resi(i).eq.restst) then
	 lgbloc=lgbloc+1
	 endif
	 if (resi(i).ne.restst) then
	 lgbloc=3*lgbloc
	 nn=nn+1
         TABB(nn)=lgbloc
	 lgbloc=1
	 endif
	 restst=resi(i)
      enddo

       lgbloc=3*lgbloc
       nn=nn+1
       TABB(nn)=lgbloc

      if (nn.ne.nb) then
      write(6,'(I6,A)')  nn,' residues now: big problem !'
      stop 
      endif
!
!     --------------------------------------------------
!     Placement de plusieur residus dans un meme bloc
!     --------------------------------------------------

      if (nbb.gt.1) then

        do k=1,no
	ntab(k)=0
	enddo

	do k=1,no
	i=nbb*k
!       Dernier bloc:
        if (i.gt.nb) then
            i=nb
            nbbcur=nbblast
        else
            nbbcur=nbb
        endif
	do j=0,nbbcur-1
	ntab(k)=ntab(k)+tabb(i-j)
	enddo
	enddo
	 
        do k=1,no
	TABB(k)=NTAB(k)
	enddo
 
        NB=NO

      endif

!     -----------------------------
!     fin du placement & tests:
!     -----------------------------
!     Le dernier bloc est "soigne" s'il est trop petit:
!     il est concatene au precedent.
! 
      if (TABB(nb).lt.9) then
         nb=nb-1
         TABB(nb)=TABB(nb)+TABB(nb+1) 
      endif
!
      write(6,'(A,I6,A)') ' System is split into ',nb,' blocks.'
!
      nn=0
      natblocs=0
      natbmin=natmax
!
      do k=1,nb
       if (TABB(k).gt.3*natresmax) then
         write(6,'(A,I6,A,I4,A,I6,A)') progrer, &
         TABB(k),' d.o.f. in block ',k, &
       ' that is, more than the ',3*natresmax,' allowed. Sorry.'
         stop
       else if (TABB(k).gt.natblocs) then
         natblocs=TABB(k) 
       endif
       if (TABB(k).lt.9) then
         write(6,'(A,I2,A,I4,A/A)') progrer, &
         TABB(k),' d.o.f. in block ',k, &
       ' i.e., less than what is required for a rigid body.', &
       ' Try with more residues per bloc.'// &
       ' Or change some residue numbers. '
         stop
       endif
       if (TABB(k).lt.natbmin) natbmin=TABB(k)
       nn=nn+TABB(k)
      enddo
!
      natblocs=natblocs/3
      natbmin=natbmin/3
!
      write(6,'(A,I6,A)') ' At most, ', &
         natblocs,' atoms in each of them.'
      write(6,'(A,I6,A)') ' At least,', &
         natbmin,' atoms in each of them.'

      if (nn.ne.3*natom) then
         write(6,'(A,I6,A,I6,A)') & 
       ' Sum of block lengths: ',nn,', instead of ', &
           3*natom,': Big problem !'
         stop
      endif

      close(72)
      close(74)
      write(6,'(A)') ' Normal end of Blocpdb.'
      RETURN
      END


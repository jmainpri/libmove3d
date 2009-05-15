

      SUBROUTINE RTBTOMODES (nompdb,lnompdb,natom, nb, nbrt, nddres, nvec, n, numvec1, &
                              covec, freq1, matvec1, norm, rt, vec_x )
!
!     Scalar Arguments
!
      integer          natom, nb, nbrt, nddres, nvec
!
!     Array Arguments
!
      integer          n(*), numvec1(*)
      double precision covec(3*natom,nvec), freq1(nvec), &
                       matvec1(6*nb,nvec), norm(nvec), &
                       rt(nbrt,nddres,nb)
!----------------------------------------------------------------------
      integer lnompdb
      character (len=lnompdb) nompdb
!----------------------------------------------------------------------

!
!     Purpose:
!     =======
!
!     -----------------------------------------------------------
!     From eigenvectors in block-rotation-translation coordinates
!     to eigenvectors in cartesian coordinates.
!     -----------------------------------------------------------
!
!     Input : matrix file 'rtb.eigenfacs'
!     Output: matrix file 'matrix.eigenrtb'
!
!     Arguments
!     =========
!
!     natom     : maximum number of atoms in the system.
!     natresmax : maximum number of atoms in each block.
!     nresmax   : maximum number of residues and of blocks.
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
!
!     In case of problem, feel free to contact: 
!     sanejouand@crpp.u-bordeaux.fr
!-----------------------------------------------------------------------
!
!     Local variables
!
      character*80     cformat, cstatus
      character*256     nomeig1, nomeig2
      logical          qinter, qexist
      integer          i, ibloc, ii, ivec, j, k, natcur, nbcur, &
                       nbread, nddl1, &
                       ntot, nunit, uneig1, uneig2
      double precision dvec
      character        program*14, progrer*17, progrwn*17
!
!----------------------------------------------------------------------
      double precision  vec_x (0:nvec*3*natom)
!----------------------------------------------------------------------


      write (6,*) '==================== Begin of RTB_TO_MODES 2.04.'
!
      program=' Rtb_to_modes>'
      progrer='%Rtb_to_modes-Er>'
      progrer='%Rtb_to_modes-Wn>'

!     Fichier en lecture
!      -------------------

      nunit=10
      open (unit=35,file='rtb.blocs',status='old',form='unformatted')
!
      natcur=natom
      nbcur=nb
      read(35) natom,nb,(n(i),i=1,nb)

      write(6,*) 'Number of atoms in block-file= ',natom
      if (natom.ne.natcur) then
          write(6,'(A,I6,A)') progrer,natcur,' atoms... up to now.'
          stop
      endif

      write(6,*) 'Number of blocs= ',nb
      if (nb.ne.nbcur) then
          write(6,'(A,I6,A)') progrer,nbcur,' blocks... up to now.'
          stop
      endif

!     Matrice-input:

      nomeig1=nompdb//'.eigenfacs'
      write (6, *)  nomeig1
      uneig1=nunit
      nunit=nunit+1
      cformat="FORMATTED"
      cstatus="old"
      call openam(nomeig1,cformat,cstatus,uneig1,.true., &
                  qinter,qexist)
      if (qinter.or..not.qexist) stop
 
!     Matrice-output:

      nomeig2=nompdb//'.eigenrtb'
      uneig2=nunit
      nunit=nunit+1
      cformat="FORMATTED"
      cstatus="unknown"
      call openam(nomeig2,cformat,cstatus,uneig2,.true., &
                  qinter,qexist)
      if (qinter) stop

!     Lecture des fichiers:
!     --------------------

      call rdmodfacs(uneig1,6*nb,nvec,numvec1,freq1, &
                     matvec1,nddl1)
!
      write(6,'(/I5,A,I6,A)') &
      nvec,' vectors, with ',nddl1,' coordinates in vector file.'

      if (nddl1.ne.6*nb) then
          write(6,'(/A,I6,A)') ' Vectors of length: ',6*nb, &
        ' were expected. Block and vector files are not consistent.'
          stop
      endif
!
      write(6,'(/A)') &
      ' Norm of eigenvectors in projected coordinates (one expected):'
      do ivec=1,nvec
         norm(ivec)=0.d0
         do i=1,nddl1
         dvec=matvec1(i,ivec)**2
         norm(ivec)=norm(ivec)+dvec
         enddo
         norm(ivec)=dsqrt(norm(ivec))
       enddo
       write(6,'(5F8.5)') (norm(ivec),ivec=1,nvec)
!
      write(6,'(/A)') ' RTB block-file is being read.'
!
      nbread=0
      do i=1,nbrt
       do j=1,nddres
         do k=1,nb
           rt(i,j,k)=0.d0
         enddo
       enddo
      enddo
!
   90 continue
      read(35,end=99) i,j,ibloc,rt(i,j,ibloc)
      nbread=nbread+1
      if (i.gt.nbrt.or.i.le.0) then
          write(6,'(A,I6,A,I1,A,I4)') ' Error: ',i, &
        ' rigid body ddl instead of ',nbrt,' for block ',ibloc
          stop
      endif
      if (j.gt.nddres.or.j.le.0) then
          write(6,'(A,I6,A,I6,A,I6)') ' Error: ',j, &
        ' ddl for block ',ibloc,'. Maximum allowed is: ',nddres
          stop
      endif
      if (ibloc.gt.nb.or.ibloc.le.0) then
          write(6,'(A,I4,A,I4)') ' Error: ',ibloc, &
        ' blocks at least. Maximum allowed is: ',nb
          stop
      endif
      goto 90
   99 continue
!
      write(6,'(I5,A)') nbread,' lines found in RTB file.'
!
      do ivec=1,nvec
	 do i=1,3*natom
	    covec(i,ivec)=0.d0
         enddo
      enddo
 
      do ivec=1,nvec
       ntot=0
       ii=0
       do ibloc=1,nb
	 do j=1,n(ibloc)
	   covec(j+ntot,ivec)=0.d0
	   do i=1,6
	   covec(j+ntot,ivec)=covec(j+ntot,ivec)+ &
           rt(i,j,ibloc)*matvec1(i+ii,ivec)
	   enddo
          enddo
	   ii=ii+6
           ntot=ntot+n(ibloc)
       enddo
       enddo
!
      write(6,'(/A)')  &
      ' Norm of eigenvectors in cartesian coordinates (one expected):'
      do ivec=1,nvec
         norm(ivec)=0.d0
         do i=1,natom
         ii=3*i-2
         dvec=covec(ii,ivec)**2+covec(ii+1,ivec)**2 &
             +covec(ii+2,ivec)**2
         norm(ivec)=norm(ivec)+dvec
         enddo
         norm(ivec)=dsqrt(norm(ivec))
       enddo
       write(6,'(5F8.5)') (norm(ivec),ivec=1,nvec)
!
      write(6,'(/A)') &
      ' Orthogonality of first eigenvectors (zero expected):'
      do i=2,min(10,nvec)
         do j=1,i-1
            norm(j)=0.d0
            do k=1,natom
               ii=3*k-2
               norm(j)=norm(j)+ &
               covec(ii,i)*covec(ii,j)+covec(ii+1,i)*covec(ii+1,j)+ &
               covec(ii+2,i)*covec(ii+2,j)
            enddo
         enddo
         write(6,'(A,I3,A,10F6.3)') &
             ' Vector ',i,':',(norm(j),j=1,i-1)
      enddo
 
      write (6,'(A, i10)') 'number of vectors',nvec
      do i=1,nvec
         write(uneig2,'(A,I5,7X,A,1PG12.4)') &
             ' VECTOR',i,'VALUE',freq1(i)
!         write(uneig2,'(1X,35(1H-))')
         write(uneig2,'(3(F17.8))') &
        (covec(ii,i),covec(ii+1,i),covec(ii+2,i),ii=1,3*natom,3)
      enddo
 
      close(35)
      close(uneig1)
      close(uneig2)
!-------------------------------------------------------------
!     vec_x is one-dimension array for move3d + elnemo version
!-------------------------------------------------------------
      do i=1,nvec
         do j=1,3*natom
            vec_x ((i-1)*3*natom+j)= covec(j,i)
         enddo
      enddo
!------------------------------------------------------------
      write(6,'(/I5,A)') nvec,' eigenvectors saved on output.'
      write(6,'(/A)') ' Normal end of RTB_TO_MODES.'
      return
      end
!
      subroutine rdmodfacs(uneig,nddlmax,nvec,numvec,freq, &
                 matvec,nddl)
!
!     Lecture de modes au format "CERFACS".
!
!     Premieres versions (rdcerfacs):
!     YHS-Nov-1996.
!     Dernieres modifications:
!     YHS-Fev-2001.
!I/O:
      integer numvec(*), nddlmax, nvec, nddl, uneig
      double precision freq(*), matvec(nddlmax,*)
!Local:
      integer nmotsmax
      parameter(nmotsmax=100)
      integer nerr, ivec, indnm_cfacs, nmots,  &
              i, ii, k
      logical qfound, qold, qfirst
      character*1 carnum
      character*132 lign132, mots(nmotsmax)
!Defaut:
      nerr=0
      qfirst=.true.
      qold=.false.
      qfound=.false.
 100  continue
      read (uneig,'(A)',end=300,err=110) lign132
      goto 120
 110  continue
      nerr=nerr+1
 120  continue
!
      qfound=qfound.or. &
            (index(lign132,' value ').gt.0.and. &
             index(lign132,' vector ').gt.0.and. &
             index(lign132,' residual ').le.0)
      qold=qold.or. &
            (index(lign132,' VALUE ').gt.0.and. &
             index(lign132,' VECTOR ').gt.0)
!
      if (.not.qfound.and..not.qold) goto 100
!________________________________________
!
!     Lecture des frequences des modes :
!________________________________________
!
      if (qfirst) then
          if (qold) then
          write(6,'(/A)') &
        ' Rdmodfacs> Old Blzpack file format detected.'
          else
          write(6,'(/A)') &
        ' Rdmodfacs> Blzpack file format detected.'
          endif
          qfirst=.false.
      endif
!
      ivec=0
 250  continue
      ivec=ivec+1
      if (ivec.gt.nvec) then
          write(6,'(/A,I5,A)') &
        '%Rdmodfacs-Wn> More than ',nvec,' vectors in file.'
          return
      endif
!
      read(lign132,'(7X,I5,12X,G12.4)',end=240,err=240) &
           numvec(ivec), freq(ivec)
 
      goto 255
 240  continue
      write(6,'(/3A)') &
          '%Rdmodfacs-W> Pb with ligne: ',lign132(1:36),'...'
 255  continue
!
      write(6,'(/A,I6)') &
      ' Rdmodfacs> Eigenvector number:', &
        numvec(ivec)
      write(6,'(A,1PG12.4)') &
      ' Rdmodfacs> Corresponding eigenvalue:', &
        freq(ivec)
!
      if (numvec(ivec).le.0) &
          write(6,'(/A/A)') &
          '%Rdmodfacs-W> Vector number was expected in:', &
          lign132
!
      read(uneig,'(A)',end=230,err=230) lign132
 230  continue
      read(lign132,'(1X,A1)',end=232,err=232) carnum
 232  continue
      if ((qfound.and.carnum.ne.'=').or. &
          (qold.and.carnum.ne.'-')) then
          write(6,'(2A/A)') &
             ' %Rdmodfacs-Warning> Unexpected character ', &
             ' in second column of line:', &
          lign132
      endif
!____________________________________________________
!
!     2) Lecture des coordonnees des modes:
!        Format libre.
!____________________________________________________
!
      k=0
 257  continue
      if (k.gt.nddlmax) then
          write(6,'(/A,I6,A,I5)')  &
        '%Rdmodfacs-Err> More than ',nddlmax, &
        ' coordinates for vector ',ivec
          return
      endif
!
      read(uneig,'(A)',end=300,err=270) lign132
!
!     Nombre de coordonnees par ligne:
      call string_split(lign132,132,' ', &
                        mots,nmotsmax,nmots)
!
      if (lign132.eq.' ') then
          read(uneig,'(A)',end=300,err=260) lign132
      else if (.not.qold.or.index(lign132,' VALUE ').le.0) then
          read(lign132,*,end=258) &
         (matvec(k+ii,ivec),ii=1,nmots)
          k=k+nmots
          goto 257
 258      continue
      endif
      nddl=k
!
 260  continue
      indnm_cfacs=index(lign132,'       VALUE')
      if (indnm_cfacs.le.0) &
          indnm_cfacs=index(lign132,'       value')
      if (indnm_cfacs.gt.0) then
          goto 250
      else
          write(6,'(A,A/A/A)') &
        ' Rdmodfacs: Lecture des modes terminee.', &
        ' Item VALUE non trouve dans la ligne:',lign132
          goto 300
      endif
!
 270  continue
      write(6,'(A,I6,A)') &
            ' %Rdmodfacs-Error: durant la lecture de la coordonnee ', &
            i,' du mode.'
      stop
!
 220  continue
!*****Ligne suivante de la lecture du fichier des modes en cours :
!
      goto 100
!
!     Fin de la lecture du fichier des modes :
!
 300  continue
      return
      end
!
      subroutine string_split(chaine,taille,delimiteur, &
                              souschaine,nbremax,nbre)
!
!     "Chaine" est coupee en "nbre" "souschaine" de part et d'autre du
!     "delimiteur"
!      YHS-Sep-93, Uppsala
! I/O:
      integer taille, nbremax, nbre
      character*(*) chaine, souschaine(*), delimiteur
! Local:
      integer icar, iprev
!
      nbre=1
      iprev=1
      souschaine(1)=chaine
      do icar=1,taille
         if (chaine(icar:icar).eq.delimiteur) then
            if (icar-1.ge.iprev) then
               souschaine(nbre)=chaine(iprev:icar-1)
               nbre=nbre+1
               if (nbre.le.nbremax) then
                  if (icar+1.le.taille.and. &
                     chaine(icar+1:taille).ne.' ') then
                     souschaine(nbre)=chaine(icar+1:taille) 
                  else
                     nbre=nbre-1
                     return
                  endif
               else
                  write(6,'(A,I6,A/A)') &
                     ' %String_split-Err: more than ',nbremax, &
                     ' substrings in : ',chaine
                  return
               endif
            endif
            iprev=icar+1
         endif
      enddo
!
      return
      end
!
      subroutine stringcl(chaine,nonblancs)
!
!     Les caracteres "blancs" de la CHAINE sont retires (a gauche et au milieu).
!     L'entier NONBLANCS donne la position du dernier caractere.
!
!     YHS-Jan-95, Toulouse.
! I/O:
      integer nonblancs
      character*(*) chaine
! Local:
      integer icar, ncar, taille
! Begin:
      taille=len(chaine)
!
      if (index(chaine(1:taille),' ').le.0) then
          nonblancs=taille
          return
      endif
!
!*****Nettoyage des blancs a gauche.
!     Premier non-blanc:
!
      do icar=1,taille
         if (chaine(icar:icar).ne.' ') goto 150
      enddo
 150  continue
      chaine=chaine(icar:taille)
!
!*****Nettoyage des blancs au milieu.
!
          icar=1
          ncar=1
 170      continue
          icar=icar+1
          ncar=ncar+1
          if (chaine(icar:icar).eq.' ') then
              chaine=chaine(1:icar-1)//chaine(icar+1:taille) 
              icar=icar-1
          endif
          if (ncar.lt.taille-1) goto 170
!
      nonblancs=index(chaine,' ')-1
!
      return
      end

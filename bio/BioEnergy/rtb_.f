

      SUBROUTINE RTB ( natom, nb, nbrt, nddres, nddres2, indexi, indexj, &
                       n, amass, corlin, d, h, hd, hh, mat, rt, s )
!
!     Scalar Arguments
      integer          natom, nb, nbrt, nddres, nddres2
!
!     Array Arguments
      integer          indexi(nddres2), indexj(nddres2), n(nb)
      double precision amass(3*natom), corlin(3*natom), &
                       d(nbrt,nddres), h(nddres2), hd(nddres,nbrt), &
                       hh(nddres,nddres), mat(6*nb,6*nb), &
                       rt(nbrt,nddres,nb), s(nbrt,nbrt) 
!
!     Purpose:
!     =======
!
!     ------------------
!     Matrix projection.
!     ------------------
!
!     Lit les fichiers produits par "prepmat", c'est-a-dire:
!     Les coordonnes pdb ("rtb.pdb")
!     La matrice re-ecrite bloc par bloc ("rtb.matblocs")
!
!     Ecrit la matrice projetee ("rtb.sdijb")
!    -format i,j, element-non-nul.
!
!     Decrit les rotation-translations de
!     chaque element de chaque bloc ("rtb.blocs")
!
!     A faire: taux de remplissage de la matrice projetee.
!
!     Arguments
!     =========
!
!     natom   : number of atoms in the system.
!     nb      : number of blocks the system is split into.
!     nbrt    : number of degrees of freedom of each block.
!     nddres  : size of largest block.
!     nddres2 : nddres**2
!     indexi  : i-index of non-zero elements in block hw.
!     indexj  : j-index of non-zero elements in block hw.
!     n       : block lengths.
!     amass   : masses.
!     corlin  : coordinates.
!     d       : tranlation and rotation vectors of each block.
!     h       : block being considered.
!     hd      : working matrix for projection of a given block.
!     hh      : working matrix for projection of a given block.
!     mat     : projected matrix.
!     rt      : tranlation and rotation vectors of all blocks.
!     s       : working matrix for projection of a given block.
!
!     Version 2000 by Florence Tama (ftama@scripps.edu).
!
!     Modified by Osni Marques and Yves-Henri Sanejouand.
!     Last modifications: March 2001.
!
!     In case of problem, feel free to contact: 
!     sanejouand@crpp.u-bordeaux.fr
!
!-----------------------------------------------------------------------
!
!     Local variables
!
      logical          qok
      integer          i, ibloc, ii, indxi, indxj, j, jbloc, k, kk, kr, &
                       mmm, natcur, nbcur, nbnnul, nempty, ni, nj, &
                       nlign, nnskip, nnuls, nskip, ntot, ntoti, ntotj
      double precision amasstot, trace, xg, yg, zg
      character        program*5, progrer*8, progrwn*8
!
!     ------------------------- CHANGEMENT D'ORIGINE DES COORDONNEES CARTESIENNES
 
      write(6,*) '==================== Begin of RTB 2.04'

      program=' RTB>'
      progrwn='%RTB-Wn>'
      progrer='%RTB-Er>'

!     Fichier d'entree
!     ----------------

      open(unit=74,file='rtb.pdb',status='unknown',form='unformatted')
      open(unit=51,file='rtb.matblocs',status='unknown',form='unformatted')
 
!     Fichier de sortie
!     -----------------

      open(unit=52,file='rtb.sdijb',status='unknown', &
           form='unformatted')
      open(unit=35,file='rtb.blocs',status='unknown',form='unformatted')

!     Lectures + Tests:
!     -----------------

      natcur=natom
      read(74) natom

      write(6,*) 'Number of atoms in coordinate file=',natom
      if (natom.ne.natcur) then
          write(6,'(A,I6,A)') progrer,natcur,' atoms... up to now.'
          stop
      endif
!      
      read(74) (corlin(ii),ii=1,3*natom)
      read(74) (amass(ii),ii=1,3*natom)
!
      qok=.true.
       AMASSTOT = 0.D0 
       DO I=1,3*natom
	 AMASSTOT = AMASSTOT + AMASS(I)
         if (amass(i).lt.0) qok=.false.
       ENDDO      
      AMASSTOT = AMASSTOT / 3.D0 
      if (.not.qok) then
          write(6,*) 'Big problem with masses (negative values read).'
          stop
      endif
      write(6,'(A,F15.4)') ' Total mass=',AMASSTOT
!
      natcur=natom
      nbcur=nb
      read(51) natom,NB,(N(k),k=1,NB)

      write(6,*) 'Number of atoms in matrix  =',natom
      if (natom.ne.natcur) then
          write(6,*) 'Pdb file and matrix are not consistent, now.'
          stop
      endif
 
      write(6,*) 'Number of blocks =',nbcur
      if (nb.ne.nbcur) then
          write(6,'(A,I6,A)') progrer,nbcur,' blocks... up to now.'
          stop
      endif
      write(35) natom,NB,(N(k),k=1,NB)

      natcur=0
      do i=1,nb
         natcur=natcur+n(i)
      enddo
      if (natcur.ne.3*natom) then
          write(6,*) 'Sum of block lengths: ',natcur, &
        ' instead of ',3*natom,': Big problem somewhere !'
          stop
      endif

      nlign=1
      nempty=0
      nskip=0

!     <------------------------------------------ DEBUT DE BOUCLE SUR LES BLOCS
!
      write(6,'(A)') ' Projection begins.'
      kk=0
      NTOT=0

      DO IBLOC=1,NB  
!
!     REMPLISSAGE DES BLOCS
!
      nbnnul=0

      read(51,end=895,err=900) &
           nbnnul,(indexi(ii),indexj(ii),H(ii),ii=1,nbnnul)
      nlign=nlign+1

      if (nbnnul.gt.nddres2) then
          write(6,*)  nbnnul,' elements in bloc ',ibloc
          write(6,*) ' Max= ',nddres2,'. Sorry.'
          stop
      endif
      if (nbnnul.le.0) then
          nempty=nempty+1
      else
      do i=1,nbnnul
         if (indexi(i).le.NTOT.or.indexj(i).le.NTOT) then
             write(6,*) 'Ntot=',ntot,' but i= ',indexi(i),' j= ', &
             indexj(i),' for element ',i,' of bloc ',ibloc 
             stop
         endif
      enddo
      endif
     
!     <------------------------------------------On se place au bloc suivant 

      kk=kk+1
      do kr=1,NB-kk
      read(51,end=905,err=905) nnskip
      if (nnskip.le.0) nempty=nempty+1
      nskip=nskip+1
      enddo
 
!     <------------------------------------------Initialisation du bloc

      if (n(ibloc).gt.nddres.or.n(ibloc).le.0) then
          write(6,*) ' N(ibloc)= ',n(ibloc),' for bloc ',ibloc
          write(6,*) ' Max= ',nddres
          stop
      endif

      do i=1,N(IBLOC)
         do j=1,N(IBLOC)
         HH(i,j)=0.d0
         enddo
      enddo

!     <------------------------------------------Remplissage du bloc

      do ii=1,nbnnul
      i=indexi(ii)-NTOT
      j=indexj(ii)-NTOT
      HH(i,j)=H(ii)
      enddo

! 
!     CHANGEMENT D'ORIGINE DANS CHAQUE BLOC (CDG DU BLOC)
!     ---------------------------------------------------------

        AMASSTOT = 0.D0
        DO I=1,N(IBLOC)
         AMASSTOT = AMASSTOT + AMASS(NTOT+I)
        ENDDO      
	AMASSTOT=AMASSTOT/3.d0

!
        XG = 0.D0
        YG = 0.D0  
        ZG = 0.D0

        DO I=0,( (N(IBLOC)/3)-1 )
         XG = XG + AMASS(NTOT+3*I+1)*corlin(NTOT+3*I+1)
         YG = YG + AMASS(NTOT+3*I+2)*corlin(NTOT+3*I+2)
         ZG = ZG + AMASS(NTOT+3*I+3)*corlin(NTOT+3*I+3)
        ENDDO

       XG = XG / AMASSTOT
       YG = YG / AMASSTOT
       ZG = ZG / AMASSTOT
!
       DO I=0,( (N(IBLOC)/3)-1)      
        corlin(NTOT+3*I+1)= corlin(NTOT+3*I+1) - XG
        corlin(NTOT+3*I+2)= corlin(NTOT+3*I+2) - YG
        corlin(NTOT+3*I+3)= corlin(NTOT+3*I+3) - ZG
       ENDDO

!
!     PROJECTION DES TRANSLATIONS ET ROTATIONS --------------------------
!

      DO I=1,6
       DO J=1,N(IBLOC)
        D(I,J) = 0.D0 
       ENDDO
      ENDDO
!
      DO I=1, N(IBLOC)/3
       II=NTOT 
       D(1,1+3*(I-1))= DSQRT(AMASS(II+1+3*(I-1)))
       D(2,2+3*(I-1))= DSQRT(AMASS(II+2+3*(I-1)))
       D(3,3+3*(I-1))= DSQRT(AMASS(II+3+3*(I-1)))
       D(4,2+3*(I-1))=-DSQRT(AMASS(II+2+3*(I-1)))*corlin(II+3+3*(I-1))
       D(4,3+3*(I-1))= DSQRT(AMASS(II+3+3*(I-1)))*corlin(II+2+3*(I-1))
       D(5,1+3*(I-1))= DSQRT(AMASS(II+1+3*(I-1)))*corlin(II+3+3*(I-1))
       D(5,3+3*(I-1))=-DSQRT(AMASS(II+3+3*(I-1)))*corlin(II+1+3*(I-1))
       D(6,1+3*(I-1))=-DSQRT(AMASS(II+1+3*(I-1)))*corlin(II+2+3*(I-1))
       D(6,2+3*(I-1))= DSQRT(AMASS(II+2+3*(I-1)))*corlin(II+1+3*(I-1))
      ENDDO
!

      MMM=6

      CALL SCHMIDT(MMM,N(IBLOC),D ) 
!
      do i=1,6
         do j=1,N(IBLOC)
            RT(i,j,ibloc)=D(i,j)
	    write(35) i,j,ibloc,rt(i,j,ibloc)
         enddo
      enddo

      NTOT=NTOT+N(IBLOC)
      ENDDO
!   <-------------------------------------------FIN DE BOUCLE SUR LES BLOCS

      close(51)

      open(unit=51,file='rtb.matblocs',status='unknown',form='unformatted')

      read(51) natom,NB,(N(k),k=1,NB)

      ni=0
      nj=0
      indxi=0
      indxj=0
      NTOTI=0
      NTOTJ=0

      do IBLOC=1,NB
         do JBLOC=IBLOC,NB

         read(51,end=910,err=910) &
         nbnnul,(indexi(ii),indexj(ii),H(ii),ii=1,nbnnul)


         do i=1,N(IBLOC)
              do j=1,N(JBLOC)
               HH(i,j)=0.d0
              enddo
         enddo

         do ii=1,nbnnul
              i=indexi(ii)-NTOTI
              j=indexj(ii)-NTOTJ
              HH(i,j)=H(ii)
         enddo


         do j=1,N(IBLOC)
              do i=1,6
                HD(j,i)=0.d0
                 do k=1,N(JBLOC)
                  HD(j,i)=HD(j,i)+HH(j,k)*RT(i,k,jbloc)   
                 enddo
              enddo
         enddo

         do i=1,6
             do j=1,6
	       s(i,j)=0.d0
               do k=1,N(IBLOC)
                  s(i,j)=s(i,j)+RT(i,k,ibloc)*HD(k,j)
               enddo
                  ni=i+indxi
                  nj=j+indxj
                  mat(ni,nj)=0.d0
		  mat(ni,nj)=s(i,j)
             enddo
         enddo


      NTOTJ=NTOTJ+N(JBLOC)

      indxj=indxj+6

      ENDDO

      indxi=indxi+6
      indxj=indxi 
      NTOTI=NTOTI+N(IBLOC)
      NTOTJ=NTOTI

      ENDDO

!     Ecriture de la matrice projetee:

      write(6,'(A)') ' Projected matrix is being saved.'
      trace=0.d0
      nnuls=0
      do i=1,6*nb
	 do j=i,6*nb
         if (mat(i,j).ne.0.d0) then
            nnuls=nnuls+1
            write(52) i,j,mat(i,j)
            if (i.eq.j) trace=trace+mat(i,j)
         endif
	 enddo
      enddo

      close (35)
      close (51)
      close (52)
      close (74)
      write(6,'(A,F12.4)') ' Matrix trace= ',trace
      write(6,'(I9,A)') nnuls,' non-zero elements.'
      write(6,'(A)') ' Normal end of RTB.'
      return
 895  continue
      write(6,'(A,I3,A,I3)') &
          ' End-of-file while reading I-bloc: ',ibloc, &
          ' nbnnul= ',nbnnul
      goto 990
 900  continue
      write(6,'(A,I3,A,I3)') &
          ' Error while reading I-bloc: ',ibloc, &
          ' nbnnul= ',nbnnul
      goto 990
 905  continue
      write(6,'(3(A,I3))') &
          ' Error after reading I-bloc: ',ibloc, &
          ' NB= ',NB,' kk= ',kk 
      goto 990
 910  continue 
      write(6,'(A,I3,A,I3)') &
          ' Error while reading J-bloc: ',jbloc, &
          ' for band: ',ibloc
 990  continue
      write(6,'(I6,A)') nlign,' lignes read.'
      write(6,'(I6,A)') nempty,' empty lignes found.'
      write(6,'(I6,A)') nskip,' lignes skipped.'
      RETURN
      END

! -----------------------------------------------
      SUBROUTINE SCHMIDT(M,N,C)
! ----------------------------------------------- 
!
!     ORTHOGONALISATION PAR GRAM-SCHMIDT
!
      integer   I,J,K,M,N
      double precision aaa, anorm, C(6,*), REC(6,6)
!
      ANORM = 0.D0
      DO I=1,N
       ANORM = ANORM + C(1,I)*C(1,I)
      ENDDO
      ANORM = 1.D0 / ( DSQRT(ANORM) )
!
      DO I=1,N
       C(1,I) = ANORM * C(1,I)
      ENDDO
! 
      DO  I=2,M      
       DO J=1,I-1
        REC(J,I) = 0.D0
        DO K=1,N
         REC(J,I) = REC(J,I) + C(J,K)*C(I,K)
        ENDDO
       ENDDO
       DO K=1,N
        AAA = 0.D0
        DO J=1,I-1
         AAA = AAA + C(J,K)*REC(J,I)
        ENDDO
        C(I,K) = C(I,K) - AAA
       ENDDO
!
       ANORM = 0.D0
       DO K=1,N
        ANORM = ANORM + C(I,K)*C(I,K)
       ENDDO 
       ANORM = 1.D0 / ( DSQRT(ANORM) )
!
       DO K=1,N
        C(I,K) = ANORM*C(I,K)
       ENDDO
!
      ENDDO
      RETURN
      END  

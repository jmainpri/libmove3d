

      SUBROUTINE PREPMAT ( fmtx, natblocs, natom, nb, qtrace, &
                           indexi, indexj, tabb, a, hw)

!     Scalar Arguments
!
      character        fmtx*(*)
      integer          natblocs, natom, nb 
!
!     Array Arguments
!
      logical          qtrace(3*natom)
      integer          indexi(9*natblocs*natblocs), &
                       indexj(9*natblocs*natblocs), &
                       tabb(nb)
      double precision a(3*natblocs,3*natom), hw(9*natblocs*natblocs)
!
!     Purpose:
!     =======
!
!     --------------------------------------------
!     Matrix preparation: it is split into blocks.
!     --------------------------------------------
!
!     Arguments
!     =========
!
!     natblocs  : maximum number of atoms found in a block.
!     natom     : number of atoms in the system.
!     nb        : number of blocks the system is split into.
!     qtrace    : flag to remember that a given diagonal element is known.
!     indexi    : i-index of non-zero elements in block hw.
!     indexj    : j-index of non-zero elements in block hw.
!     tabb      : block lengths.
!     a         : band-matrix corresponding to a given block.
!     hw        : block being filled.
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
      logical          qinit
      integer          bloc, i, ii, iii, imax, j, jbloc, k, kk, &
                       nblocs, nbnnul, nempty, &
                       ni1, ni2, nj1, nj2, nlign, nn, ntotn1, &
                       tbloc, tt
      double precision toto, trace
      character        program*9, progrer*12, progrwn*12
!
!-----------------------------------------------------------------------
      program=' Prepmat>'
      progrwn='%Prepmat-Wn:'
      progrer='%Prepmat-Er:'
!     
      write(6,*) '==================== Begin of Prepmat 2.03.'
      nblocs=nb
!
!     Fichiers d'entree 
!     -----------------
      open(unit=50,file=fmtx,status='old',form='formatted')
      
!     Fichiers de sortie
!     ------------------
      open(unit=51,file='rtb.matblocs',status='unknown', &
           form='unformatted')

!    --------------------------------------------------------------
!    Creation du fichier rtb.matblocs => matrice ecrite par bloc
!    Ce fichier contient
!       1- la longueur de cahque bloc
!       2- les elements de la matrice correspondant a chaque bloc 
!    --------------------------------------------------------------

      write(6,'(A)') ' Rewriting of the matrix begins.'
      write(51) natom,NB,(TABB(k),k=1,NB)
      nlign=1
      nempty=0
      
!     ----------------------------      
!     Initialisation compteur
!     ----------------------------

      NI1=1
      NTOTN1=0
      nn=1
      NI2=TABB(nn)
      qinit=.true.
      TBLOC=0
!
      imax=-1
      trace=0.d0
      do i=1,natom
         qtrace(i)=.false.
      enddo
 50   continue

!     NI1,NI2 bornes du residue vertical
!     ----------------------------------

!        Matrice-bande A(i,j)=0.d0:
!        -------------------------
	 if (qinit) then
         do i=1,TABB(nn)
            kk=NTOTN1+1
            do j=kk,3*natom
               A(i,j)=0.d0
            enddo
         enddo
         endif

         READ(50,*,end=99) i,j,toto

         if (i.gt.3*natom.or.j.gt.3*natom) then
             write(6,'(A,I6,A,I6,A)') ' Matrix element ',i,' j= ',j, &
           ' found, i.e. too many atoms in matrix file !'
             stop
         endif

         if (i.eq.j) then
         if (.not.qtrace(i)) then
             trace=trace+toto
             qtrace(i)=.true. 
         endif
         endif

         if (i.gt.imax) imax=i
         if (j.gt.imax) imax=j

         IF (i.le.NI2) then
            iii=i-NTOTN1
            A(iii,j)=toto
	    qinit=.false.
         endif
!
!        Decoupage de la matrice-bande lue,
!        et sortie des blocs, a raison de un bloc par ligne:
!
         if (i.gt.NI2) then
!           On a depasse NI2, on remontre d'un cran:
            backspace 50
	    NJ1=NI1
	    NJ2=NI2

!    NJ1,NJ2 borne du residue en horizontal
!    --------------------------------------
!    au depart NJ1 et NJ2 = NI1 et NI2
!    TBLOC pour TABB 
!    a chaque fois il faut decaler de un pour avoir
!    la longueur du bon bloc

	 tt=0
	 TBLOC=TBLOC+1
	 BLOC=TBLOC
!
!    Boucle sur les blocs pour une bande NI1-NI2
!    NI1 et NI2 constant dans la boucle
!    -------------------------------------------
         do JBLOC=1,NB
	    nbnnul=0
            ii=0
	    tt=tt+1
!
!    Si premier bloc, seulement une partie donc on
!    cree la partie inferieure
!    ici NJ1 et NJ2 sont egaux a NI1 et NI2
!    ---------------------------------------------

            if (tt.eq.1) then
               do i=1,TABB(nn)
                  do j=kk,NI2   
                     A(j-NTOTN1,i+NTOTN1)=A(i,j)
                  enddo
               enddo
               do i=1,TABB(nn)
                  do j=NJ1,NJ2
                     if (A(i,j).ne.0.d0) then
                        ii=ii+1
                        HW(ii)=0.d0
                        nbnnul=ii
                        HW(ii)=A(i,j)
                        indexi(ii)=i+NTOTN1
                        indexj(ii)=j
                     endif
                  enddo
              enddo
         if (nbnnul.gt.9*natblocs*natblocs) then
             write(6,*) 'Nbnnul= ',nbnnul
             write(6,*) 'Max= ',9*natblocs*natblocs
             stop
         endif
         if (nbnnul.le.0) nempty=nempty+1
         nlign=nlign+1
         write(51) nbnnul,(indexi(ii),indexj(ii),HW(ii),ii=1,nbnnul) 
      endif
!
!    Pour les autres blocs de la bande
!    NJ1 et NJ2 different de NI1 et NI2
!    on decale au bloc suivant
!    bloc suivant=> bloc+1 sachant que la valeur de depart
!    pour bloc est tbloc
!    ----------------------------------
            if (tt.gt.1) then
	       BLOC=BLOC+1
	       NJ1=NJ2+1
	       NJ2=NJ2+TABB(BLOC)
               do i=1,TABB(nn)
                  do j=NJ1,NJ2
                     if (A(i,j).ne.0.d0) then
                        ii=ii+1
                        HW(ii)=0.d0
                        nbnnul=ii
                        HW(ii)=A(i,j)
                        indexi(ii)=i+NTOTN1
                        indexj(ii)=j
                     endif
                  enddo
               enddo
         if (nbnnul.gt.9*natblocs*natblocs) then
             write(6,*) 'Nbnnul= ',nbnnul
             write(6,*) 'Max= ',9*natblocs*natblocs
             stop
         endif
         if (nbnnul.le.0) nempty=nempty+1
         nlign=nlign+1
          write(51) nbnnul,(indexi(ii),indexj(ii),HW(ii),ii=1,nbnnul)
	  endif
        enddo
!
!       fin boucle sur les blocs => initialisation de A
!       necessaire donc qinit=vrai
!       on aura un bloc en moins => nb=nb-1
!       changement des bornes verticales
!        ------------------------------------------------------
	qinit=.true.
        NTOTN1=NTOTN1+TABB(nn)
	NB=NB-1
	nn=nn+1
	NI1=NI2+1
	NI2=NI2+TABB(nn)
       endif

      goto 50
 99   continue
      write(6,'(A,I10)') ' Its size = ',imax
      if (imax/3.ne.natom) then
          write(6,'(2A,I6,A/A,I6,A)') progrer,' Only ',imax, &
        ' d.o.f. found in this matrix, ', &
        ' instead of ',natom,'*3, as expected. Not the right one ?'
          stop
      endif
      write(6,'(A,F15.4)') ' Its trace= ',trace
      write(6,'(A,2I6,F15.4)') ' Last element read: ',i,j,toto

            iii=i-NTOTN1
            A(iii,j)=toto
            NJ1=NI1
            NJ2=NI2
	    ii=0
               do i=1,TABB(nn)
                  do j=kk,NJ2
                     A(j-NTOTN1,i+NTOTN1)=A(i,j)
                  enddo
               enddo
               do i=1,TABB(nn)
                  do j=NJ1,NJ2
                     if (A(i,j).ne.0.d0) then
                        ii=ii+1
                        HW(ii)=0.d0
                        nbnnul=ii
                        HW(ii)=A(i,j)
                        indexi(ii)=i+NTOTN1
                        indexj(ii)=j
                     endif
                  enddo
               enddo
         if (nbnnul.gt.9*natblocs*natblocs) then
             write(6,*) 'Nbnnul= ',nbnnul
             write(6,*) 'Max= ',9*natblocs*natblocs
             stop
         endif
         if (nbnnul.le.0) nempty=nempty+1
         nlign=nlign+1
      write(51) nbnnul,(indexi(ii),indexj(ii),HW(ii),ii=1,nbnnul)
!
      nb=nblocs 
      write(6,'(I6,A)') nempty,' empty lines.'
      write(6,'(I6,A)') nlign,' lines saved.'
      if (nlign.ne.nb*(nb+1)/2+1) then
          write(6,'(A,I6,A)') ' Warning: ', &
          nb*(nb+1)/2+1,' lines expected on output.'
          stop
      else
          write(6,'(A)')' Number of lines on output is as expected.'
      endif
!
      close(50)
      close(51)
      write(6,'(A)') ' Normal end of Prepmat.'
      RETURN
      END

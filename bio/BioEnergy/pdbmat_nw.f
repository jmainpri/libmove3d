      subroutine  pdbmat (nompdb,lnompdb,mat,imatmax,index1,index2)
!     
!     Lecture d'un fichier pdb.
!     ______________________________
!
!     Calcul d'une matrice "Tirion".
!     ______________________________
!     Formattee  -> matrice.sdijf (qbinary=F)
!     ou Binaire -> matrice.sdijb (qbinary=T)
!     De la forme:
!     i, j, element-non-nul.
!
!     Principe du modele (Tirion, 1996): 
!     
!     Tous les atomes a moins de "cutoff" les uns des autres 
!     sont supposes lies par des ressorts, qui ont tous
!     la meme raideur.
!     Simplification supplementaire par rapport au modele initial: 
!     les atomes sont supposes avoir tous la meme taille
!    (le cutoff est le meme pour toutes les paires d'atomes).
!     On peut de plus poser qu'ils ont tous la meme masse
!    (qmass=F). Sinon, celles-ci sont lues dans la colonne des 
!     facteurs B du fichier pdb.
!
!     Principal resultat:
!
!     Les modes de vibration de basse frequence obtenus
!     a partir d'un tel modele sont tres voisins de ceux
!     obtenus avec un modele beaucoup plus detaille, tels
!     ceux utilises lors des etudes de Dynamique Moleculaire.
!
!     Principal avantage:
!
!     Pas besoin de minimisation d'energie, prealablement
!     au calcul des modes de vibration (E=0 par construction).
!   
!     References:
!
!     1) M.M. Tirion (1996):
!    "Large amplitude elastic motions in proteins from
!     a single-parameter, atomic analysis",
!     Phys. Rev. letters vol.77(9), p1905-1908.
!
!     2) F. Tama, Y.H. Sanejouand (2001):
!    "Conformational change of proteins arising 
!     from normal modes calculations"
!     Protein Engineering vol.14, p1-6.
! 
!     YHS-nov-1996: version initiale.
!     YHS-mar-2001: dernieres modifications.
 
      implicit none
      
      integer nresmx,natmax,ntopmax, nmotsmax,maxsize
      parameter(nresmx=50000,natmax=50000,ntopmax=10*natmax,&
        nmotsmax=100,maxsize=50000000 )
 
        integer lnompdb, nmots, lnom,&
        iresat(natmax), ires, nres, nvoisat(natmax),&
        natom, nbonds, nangles,&
        idres(nresmx), fatres(nresmx+1),&
        nbond(ntopmax), jbond(ntopmax), &
        nangle(ntopmax), jangle(ntopmax),&
        idmax, unpdb, unout, nunit, uninp,&
        nbig, nunknown, &
        nmin, nmax, imax, imin, nbmax, nbmin, namax, namin, &
        i, j, k, ii, jj, kk, ll, jat, irnd
      double precision &
            der2(3,3*natmax), massat(natmax), massatbig,&
            xat(natmax), yat(natmax), zat(natmax), &
            dist, dist2, rx, ry, rz, &
            cutoff, cutbnd, &
            knonb, kbond, kangle, kij, nmoy, nrms,&
            ddf, elemnt, trace, elmax, levelshft,&
            unknown, rbig, rinput
      logical qinter, qexist, qerror, qmasse, qbinary, qok
      parameter(unknown=9999.d9,rbig=1e10)
      character*1  ssusel
      character*4  resnam(natmax), atonam(natmax) 
      character*32 version, cformat, cstatus
      character*64 nomfich, residus_stshort
      character*80 lign80, motinp, typbond, typforce
      character*132 residus_standards, mots(nmotsmax)
      
      integer lnompdb,imat,imatmax
      integer index1(maxsize),index2(maxsize)
      character (len=lnompdb) nompdb
      double precision  mat(maxsize)

!Defaults:
      version=' Version 3.31, Bordeaux.'
 
      idmax=21
      residus_standards='   ILE PHE TRP LEU CYS VAL MET TYR ALA HIS ' // &
                          'GLY THR SER PRO ARG GLN ASN ASP GLU LYS '
      residus_stshort='IFWLCVMYAHGTSPRQNDEKX'
 
!     Les masses sont-elles attendues en lecture ?
      qmasse=.false.
!     Sortie de la matrice en binaire ?
      qbinary=.false.
 
!     Calcium mass (heaviest "usual" atom) 
      massatbig=40.1

      irnd=314116
      imat=0
!     call srand(irnd)
!Begin:
      write(6,*) 'Pdbmat>',version
!____________________________________________________
!
!     Ouverture et Lecture du fichier d'instructions:
!____________________________________________________
!
!     Valeurs par default:

!   no default pdb file in  move3d-elnemo version  
!      nompdb='pdbmat.ent'
      typbond='NONE'
      typforce='CSTE'
      kbond=1000.0d0
      cutbnd=2.d0
      kangle=100.0d0
      knonb=10.0d0
      cutoff=8.d0
      levelshft=1e-9
      ssusel=' '
      imat=0
!
       nunit=10
       uninp=nunit
       nunit=nunit+1
!
!      pdbmat.dat isn't used in move3d-elnemo version   
!      add commentaries
!-------------------------------------------------------
!
!      nomfich='pdbmat.dat'
!      cformat="FORMATTED"
!      cstatus="old"
!      call openam(nomfich,cformat,cstatus,uninp,.false.,&
!          qinter,qexist)
!      if (qinter.or..not.qexist) then 
!          write(6,*) &
!        'Pdbmat> No pdbmat.dat file found. Default input assumed.'
!          goto 100
!      else
!          write(6,*) &
!        'Pdbmat> Input data to be read in pdbmat.dat file.'
!      endif
!
! 50   continue
!      read(uninp,'(A)',end=100) lign80
!
!      k=index(lign80,'=') 
!      motinp=' '
!      if (k.gt.0) motinp=lign80(1:k)
!     
!      if (index(motinp,' filename').gt.0) then 
!          nompdb=lign80(k+1:80)
!      else if (index(motinp,' definition').gt.0) then
!          typbond=lign80(k+1:80)
!          call stringcl(typbond,lnom)
!          if (typbond(1:3).eq.'ALL') then 
!              typbond='ALL'
!          else if (typbond(1:4).eq.'NONE') then
!              typbond='NONE'
!          else if (typbond(1:3).eq.'CON') then
!              typbond='CONSECUTIF'
!         else if (typbond(1:4).eq.'RAND') then
!             provisoire...
!             typbond='NONE'
!             typforce='RANDOM'
!          else
!              write(6,'(/3A/)') &
!            '%Pdbmat-W> bond definition :', &
!              typbond(1:3),'... is unknown. Default assumed.' 
!              typbond='NONE'
!          endif
!      else 
!          read(lign80(k+1:80),*) rinput
!          if (index(motinp,'shift ').gt.0) then 
!               levelshft=rinput
!          else if (index(motinp,'Nonbond').gt.0) then
!               if (index(motinp,' force ').gt.0) knonb=rinput
!               if (index(motinp,' cutof').gt.0) cutoff=rinput
!          else if (index(motinp,'Bond').gt.0) then
!               if (index(motinp,' force ').gt.0) kbond=rinput
!          else if (index(motinp,' length').gt.0) then
!               cutbnd=rinput
!          else if (index(motinp,'Angle').gt.0) then
!               if (index(motinp,' force ').gt.0) kangle=rinput
!          endif
!      endif
!      goto 50
!
! 100  continue
!
!      write(6,'(/(A,F10.2))') &
!      ' Pdbmat> Interaction cutoff (A)  = ',cutoff, &
!      '         Non-bond force constant = ',knonb
!      if (typbond.ne.'NONE') then
!      write(6,'(2A)') &
!      '         Kind of bond definition = ',typbond(1:10)
!      write(6,'(A,F10.2)') &
!      '         Maximum bond length (A) = ',cutbnd,&
!      '         Bond force constant     = ',kbond,&
!      '         Angle force constant    = ',kangle
!      write(6,'(2A)') &
!      '         Kind of force constant  = ',typforce
!      endif
!      write(6,'(A,1PG10.1)') &
!      'Pdbmat> Levelshift = ',levelshft
!
!     On recherche l'information/sous-unite:
!
!      call stringcl(nompdb,lnompdb)
!      call string_split(nompdb,lnompdb,":",&
!                       mots,nmotsmax,nmots)
!      call stringcl(mots(1),lnom)
!
!      if (nmots.gt.1) then
!          call stringcl(mots(2),lnom)
!          ssusel=mots(nmots)
!          write(6,*) 'Pdbmat> Subunit to be selected: ',ssusel
!          if (nmots.gt.2) then
!              write(6,'(A)') &
!            '%Pdbmat-W> The end of pdb name, ',&
!              nompdb(1:lnompdb),', was not understood.'
!          endif
!      else
!          ssusel=' '
!      endif
!      nompdb=mots(1)
!      call stringcl(nompdb,lnompdb)
!________________________________
!                                
!     Lecture du fichier pdb:
!________________________________
      write(6,'(/(3A))') &
      ' Pdbmat> Pdb file ',nompdb(1:lnompdb),' to be opened.'
      unpdb=nunit
      nunit=nunit+1
      cformat="FORMATTED"
      cstatus="old"
      call openam(nompdb,cformat,cstatus,unpdb,.true.,&
           qinter,qexist)
      if (qinter) stop
!
      call rdatompdb(unpdb,ssusel,xat,yat,zat,massat,&
           atonam,iresat,resnam,natmax,natom,&
           fatres,nresmx,nres,qerror)
!
      if (qerror) stop
      if (natom.le.1) then
          write(6,'(A)') &
       '%Pdbmat-Err: not enough atoms in file. Nothing done.'
          stop
      endif
!
      qok=.true.
      if (qmasse) then
          write(6,'(A)') &
       ' Pdbmat> Masses to be taken in pdb file.'
          nbig=0
          do i=1,natom
             if (massat(i).le.0) then
                 qok=.false. 
                 write(6,'(A,I6)') & 
               '%Pdbmat-W> Non-positive mass found for atom ',i
                 goto 150
             else if (massat(i).gt.massatbig) then
                 nbig=nbig+1
             endif
          enddo
          if (nbig.gt.0) &
          write(6,'(A,I6,A,F6.2,A)') &
        '%Pdbmat-W> ',nbig,' atoms with mass larger than: ', &
          massatbig,' a.u.'
      endif
 150  continue

      if (.not.qmasse.or..not.qok) then
          write(6,'(A)') &
        ' Pdbmat> Masses are all set to one.'
          do i=1,natom
             massat(i)=1.d0
          enddo
      endif
!
!     Identification des residus.
!
      nunknown=0
      do i=1,nres
         ires=fatres(i)
         idres(i)=index(residus_standards,resnam(ires))/4
         if (idres(i).le.0) then
             nunknown=nunknown+1
             if (nunknown.lt.10) then
                 write(6,'(3A)') &
               "%Pdbmat-W> residue:'", &
                 resnam(ires),"' is not a well known amino-acid."
                 idres(i)=idmax
             else if (nunknown.eq.10) then
                 write(6,'(A)') &
               '%Pdbmat-W> ........'
                 idres(i)=idmax
             endif
         endif
      enddo
!
      if (nunknown.gt.0) then
          write(6,'(/A,I6,A)') &
        '%Pdbmat-W> ',nunknown,' residue(s) not known.'
      endif
!
!     Detection des liaisons covalentes:
!
      nbonds=0
!      if (typbond.ne.'ALL'.and.typbond.ne.'CONSECUTIF') goto 200
!
!      if (typbond.eq.'ALL') then
         nbmax=0
         nbmin=999
         imax=-1
         imin=-1
         k=1
         nbond(1)=1
         do i=1,natom
            do j=1,natom
               if (i.ne.j) then
                   rx=xat(i)-xat(j)
                   ry=yat(i)-yat(j)
                   rz=zat(i)-zat(j)
                   dist=dsqrt(rx*rx + ry*ry + rz*rz)
                   if (dist.le.cutbnd) then
                       jbond(k)=j
                       k=k+1
                   endif
               endif
            enddo
            nbond(i+1)=k
            if (nbond(i+1)-nbond(i).gt.nbmax) then
                nbmax=nbond(i+1)-nbond(i)
                imax=i
            endif
            if (nbond(i+1)-nbond(i).lt.nbmin) then
                nbmin=nbond(i+1)-nbond(i)
                imin=i
            endif
            if (k-1.gt.ntopmax) then
                write(6,'(/A,I12)') &
              '%Pdbmat-Err> Too many bonds. Maximum is: ',&
                ntopmax
                stop
            endif
         enddo
         nbonds=k-1
!      else if (typbond.eq.'CONSECUTIF') then
!
!        On fait attention aux distances...
!        Il peut y avoir plusieurs molecules,
!        plusieurs chaines, dans le systeme.
!       (A nettoyer: deux fois le meme calcul)
!
         nbond(1)=1
         jbond(1)=2
         k=2
         do i=2,natom-1
            nbond(i)=k
            j=i-1
            rx=xat(i)-xat(j)
            ry=yat(i)-yat(j)
            rz=zat(i)-zat(j)
            dist=dsqrt(rx*rx + ry*ry + rz*rz)
            if (dist.le.cutbnd) then
                jbond(k)=j
                k=k+1
            endif
            j=i+1
            rx=xat(i)-xat(j)
            ry=yat(i)-yat(j)
            rz=zat(i)-zat(j)
            dist=dsqrt(rx*rx + ry*ry + rz*rz)
            if (dist.le.cutbnd) then
                jbond(k)=j
                k=k+1
            endif
            if (k.gt.ntopmax) then
                write(6,'(/A,I12)') &
              '%Pdbmat-Err> Too many bonds. Maximum is: ', &
                ntopmax
                stop
            endif
         enddo
         nbond(natom)=k
         jbond(k)=natom-1
         nbond(natom+1)=k+1
         imax=2
         imin=1
         nbmin=1
         nbmax=2
         nbonds=k
!      endif
!
      if (nbonds.eq.0) then
          write(6,'(/A/)') &
        '%Pdbmat-W> No bond found.'
          goto 200
      endif
!
      write(6,'(A,I6,A,F5.2,A/(A,I3,A,I6))') &
      ' Pdbmat> ',nbonds/2,' covalent bonds, i.e.,', &
        float(nbonds)/float(2*natom),' per atom.',&
      '         Maximum number found =',nbmax,' for atom ',imax, &
      '         Minimum number found =',nbmin,' for atom ',imin
!
!     Angles de valence:
!
      namax=0
      namin=9999
      imax=-1
      imin=-1
      ii=1
      nangle(1)=1
      do i=1,natom
         if (nbond(i+1).gt.nbond(i)) then
         do jj=nbond(i),nbond(i+1)-1
            j=jbond(jj)
            if (nbond(j+1).gt.nbond(j)) then
            do kk=nbond(j),nbond(j+1)-1
               k=jbond(kk)
               if (k.ne.i) then
                   jangle(ii)=k
                   ii=ii+1
               endif
            enddo
            endif
         enddo
         endif
         nangle(i+1)=ii
         if (nangle(i+1)-nangle(i).gt.namax) then
             namax=nangle(i+1)-nangle(i)
             imax=i
         endif
         if (nangle(i+1)-nangle(i).lt.namin) then
             namin=nangle(i+1)-nangle(i)
             imin=i
         endif
         if (ii.gt.ntopmax) then
             write(6,'(/A,I12)') &
            '%Pdbmat-Err> Too many angles. Maximum is: ',&
             ntopmax
             stop
         endif
      enddo
      nangles = ii-1
      write (6, *) 'NUMBER OF ANGLES', nangles
!
      write(6,'(A,I6,A,F5.2,A/(A,I3,A,I6))') &
      ' Pdbmat> ',nangles/2,' valence angles, i.e.,', &
        float(nangles)/float(2*natom),' per atom.', &
      '         Maximum number found =',namax,' for atom ',imax, &
      '         Minimum number found =',namin,' for atom ',imin
!______________________________________________________________
!
!     Matrice des derivees secondes:
!______________________________________________________________
!
 200  continue
      unout=nunit
      nunit=nunit+1
      if (qbinary) then
      nomfich="matrix.sdijb"
      cformat="UNFORMATTED"
      else
      nomfich="matrix.sdijf"
      cformat="FORMATTED"
      endif
      cstatus="unknown"
      call openam(nomfich,cformat,cstatus,unout,.true.,&
           qinter,qexist)
 
!     Les atomes sont tous lies deux a deux,
!     par un potentiel "universel" (M.Tirion).
 
      elmax=0.d0
      trace=0.d0
      kk=0
      ll=0
      nbig=0
!____________________________________________________________________________
!
      do i=1,natom
         ii=3*i-2
         nvoisat(i)=0
                                !
                                !        On calcule trois lignes de la matrice a la fois:
!        -----------------------------------------------
         do j=1,3*natom
            der2(1,j)=0.d0
            der2(2,j)=0.d0
            der2(3,j)=0.d0
         enddo
                               !
         do j=1,natom
            if (i.ne.j) then
               jj=3*j-2
               kij=knonb
                                !           if (typforce.eq.'RANDOM') kij=knonb*rand()
                                !
               rx=xat(i)-xat(j)
               ry=yat(i)-yat(j)
               rz=zat(i)-zat(j)
               dist2=rx*rx + ry*ry + rz*rz
               dist=dsqrt(dist2)
                                !
                                !           Constante de force topologiques:
                                !
               if (nbonds.gt.0) then
                  if (nbond(i+1).gt.nbond(i)) then
                     do k=nbond(i),nbond(i+1)-1
                        if (jbond(k).eq.j) then
                           kij=kbond
                           goto 300
                        endif
                     enddo
                  else
                     goto 300
                  endif
                                !
                  if (nangle(i+1).gt.nangle(i)) then
                     do k=nangle(i),nangle(i+1)-1
                        if (jangle(k).eq.j) then
                           kij=kangle
                           goto 300
                        endif
                     enddo
                  endif
               endif
 300           continue
                                !
                                !           Calcul des elements: (potentiel harmonique)
                                !           -------------------------------------------
               if (dist.le.cutoff) then
                  ll=ll+1
                  nvoisat(i)=nvoisat(i)+1
                                !
                                !               Elements diagonaux des blocs i et j:
                                !               -----------------------------------
                  ddf=kij/dist2
                  elemnt=rx*rx*ddf
                  der2(1,ii)=der2(1,ii)+elemnt
                  der2(1,jj)=der2(1,jj)-elemnt
                  elemnt=ry*ry*ddf
                  der2(2,ii+1)=der2(2,ii+1)+elemnt
                  der2(2,jj+1)=der2(2,jj+1)-elemnt
                  elemnt=rz*rz*ddf
                  der2(3,ii+2)=der2(3,ii+2)+elemnt
                  der2(3,jj+2)=der2(3,jj+2)-elemnt
                                !
                                !               Elements extra-diagonaux des deux blocs:
                                !               ---------------------------------------
                  elemnt=rx*ry*ddf
                  der2(1,ii+1)=der2(1,ii+1)+elemnt
                  der2(2,ii)=der2(2,ii)+elemnt
                  der2(1,jj+1)=der2(1,jj+1)-elemnt
                  der2(2,jj)=der2(2,jj)-elemnt
                  elemnt=rx*rz*ddf
                  der2(1,ii+2)=der2(1,ii+2)+elemnt
                  der2(3,ii)=der2(3,ii)+elemnt
                  der2(1,jj+2)=der2(1,jj+2)-elemnt
                  der2(3,jj)=der2(3,jj)-elemnt
                  elemnt=ry*rz*ddf
                  der2(2,ii+2)=der2(2,ii+2)+elemnt
                  der2(3,ii+1)=der2(3,ii+1)+elemnt
                  der2(2,jj+2)=der2(2,jj+2)-elemnt
                  der2(3,jj+1)=der2(3,jj+1)-elemnt
               endif
            endif
         enddo
!
!        Sortie de la matrice-bande calculee:
!        -----------------------------------
                                !       (Pour BLZPACK: uniquement la demi-matrice superieure)
                                !
                                !        Level-shift, pour eviter les zeros numeriques:
                                !       (minimisation parfaite !!!)
                                !
         der2(1,ii)=der2(1,ii)+levelshft
         der2(2,ii+1)=der2(2,ii+1)+levelshft
         der2(3,ii+2)=der2(3,ii+2)+levelshft
                     !
         do j=ii,3*natom
            jat=(j-1)/3+1
            if (der2(1,j).ne.0.d0) then
               kk=kk+1
               imat=imat+1
               mat(imat) = der2(1,j)/dsqrt(massat(i)*massat(jat))
               index1(imat) = ii
               index2(imat) = j
               if (qbinary) then
                  write(unout) & 
                  ii,j,der2(1,j)/dsqrt(massat(i)*massat(jat))
               else
                  write(unout,'(2I6,1PG20.12)') &
                  ii,j,der2(1,j)/dsqrt(massat(i)*massat(jat))
                endif
               if (dabs(der2(1,j)).gt.rbig) nbig=nbig+1
               if (dabs(der2(1,j)).gt.elmax) elmax=dabs(der2(1,j))
            endif
         enddo
         do j=ii+1,3*natom
            jat=(j-1)/3+1
            if (der2(2,j).ne.0.d0) then
               kk=kk+1
               imat=imat+1
               mat(imat) = der2(2,j)/dsqrt(massat(i)*massat(jat))
               index1(imat) = ii+1
               index2(imat) = j
               if (qbinary) then
                  write(unout) &
                  ii+1,j,der2(2,j)/dsqrt(massat(i)*massat(jat))
               else
                  write(unout,'(2I6,1PG20.12)') & 
                  ii+1,j,der2(2,j)/dsqrt(massat(i)*massat(jat))
                endif
               if (dabs(der2(2,j)).gt.rbig) nbig=nbig+1
               if (dabs(der2(2,j)).gt.elmax) elmax=dabs(der2(2,j))
            endif
         enddo
         do j=ii+2,3*natom
            jat=(j-1)/3+1
            if (der2(3,j).ne.0.d0) then
               kk=kk+1
               imat=imat+1
               mat(imat) = der2(3,j)/dsqrt(massat(i)*massat(jat))
               index1(imat) = ii+2
               index2(imat) = j
               if (qbinary) then
                  write(unout) &
                  ii+2,j,der2(3,j)/dsqrt(massat(i)*massat(jat))
               else
                  write(unout,'(2I6,1PG20.12)') &
                  ii+2,j,der2(3,j)/dsqrt(massat(i)*massat(jat))
               endif
               if (dabs(der2(3,j)).gt.rbig) nbig=nbig+1
               if (dabs(der2(3,j)).gt.elmax) elmax=dabs(der2(3,j))
            endif
         enddo
         elemnt=(der2(1,ii)+der2(2,ii+1)+der2(3,ii+2))/massat(i)
         if (elemnt.eq.0.d0) then
            write(6,'(A,I6,A)') &
            '%Pdbmat-W> Atom ',i, &
            ' has null second derivatives...'
         endif
         trace=trace+elemnt
      enddo
      imatmax = imat
      write (6, *) 'max index =', imatmax
!____________________________________________________________________________
!
      nmoy=0.d0
      nrms=0.d0
      nmin=natom
      nmax=0
      do i=1,natom
         if (nvoisat(i).gt.nmax) nmax=nvoisat(i)
         if (nvoisat(i).lt.nmin) nmin=nvoisat(i)
         nmoy=nmoy+nvoisat(i)
         nrms=nrms+nvoisat(i)**2.d0
      enddo
      nmoy=nmoy/float(natom)
      nrms=nrms/float(natom)-nmoy**2.d0
      if (nrms.gt.0.d0) nrms=dsqrt(nrms)
                                !
      write(6,'(/A)') &
      ' Pdbmat> Matrix data.'
      write(6,'(A,I9,A)') &
      ' Pdbmat> ',kk,' non-zero elements.'
      write(6,'(A,I9,A)') &
      ' Pdbmat> ',ll/2,' atom-atom interactions.'
      write(6,'(A,F9.2,A,F9.2/(A,I6))') & 
      ' Pdbmat> Number per atom= ',nmoy,' +/- ',nrms, &
      '         Maximum number = ',nmax, &
      '         Minimum number = ',nmin
                                !
      write(6,'(A,1PG12.6)') &
      ' Pdbmat> Matrix trace   = ',trace
      write(6,'(A,1PG12.6)') &
      ' Pdbmat> Level shift    = ',levelshft
      write(6,'(A,I6,A,1PG8.1)') & 
      ' Pdbmat> ',nbig,' elements larger than +/- ',rbig
                                !
      write(6,'(A,F12.2)') ' Pdbmat> Larger element = ',elmax
                                !
      write(6,'(/A)') ' Pdbmat> Normal end.'
      return
      end
!-----------------------------------------------------------------------------


      subroutine rdatompdb(unpdb,ssusel,xat,yat,zat,binfo, &
                 atonam,iresat,resnam,natmax,natom, &
                 fatres,nresmx,nres,qerror)
!
!     Lecture ligne a ligne d'un fichier pdb.
!     Uniquement les lignes commencant par 'ATOM'.
!     Uniquement ceux de la sous-unite selectionnee.
!
!     fatres(i): numero du premier atome du residu i.
!     YHS-novembre-1996.
!
      implicit none
!I/O:
      integer unpdb, natmax, iresat(*), natom, &
              nresmx, nres, fatres(*)
      double precision xat(*), yat(*), zat(*), binfo(*)
      logical qerror
      character*4 atonam(*), resnam(*)
      character*1 ssusel
!Local:
      integer nerr, iatom, irs, irsprev, lmot, ntit
      double precision x, y, z, bfact
      character*1  ssu
      character*4  ren
      character*5  atncur
      character*80 lign80
!Begin:
      write(6,'(/A)') ' Rdatompdb> Reading pdb file.'
!
      qerror=.false.
      nerr=0
!
      irsprev=-1
      nres=0
      iatom=0
      ntit=0
 105  continue   
      read(unpdb,'(A)',end=200,err=110) lign80 
! 
      goto 120                                
 110  continue
      nerr=nerr+1  
!                                !
 120  continue                              
      if (lign80(1:4).eq.'ATOM') then
      read(lign80,'(12X,A4,1X,A4,A1,I4,4X,3F8.3,6X,F6.2)') & 
                  atncur, ren, ssu, irs, x, y, z, & 
                  bfact
      if (iatom.lt.natmax) then
         if (ssu.eq.ssusel.or.ssusel.eq.' ') then
            iatom=iatom+1
            xat(iatom)=x
            yat(iatom)=y
            zat(iatom)=z
            binfo(iatom)=bfact
!
            call stringcl(atncur,lmot)
            atonam(iatom)=atncur
            call stringcl(ren,lmot)
            resnam(iatom)=ren
            iresat(iatom)=irs
!
            if (irs.ne.irsprev) then
               nres=nres+1
               if (nres.gt.nresmx) then
                  write(6,'(A/A,I6)') &
                  '%Rdatompdb-Err> Too many residues in this file.', &
                  ' Maximum allowed is = ',nresmx
                  stop
               endif
               irsprev=irs
               fatres(nres)=iatom
            endif
         endif
      else
         write(6,'(A/A,I6)') &
         '%Rdatompdb-Err> Too many atoms in this file.',&
         ' Maximum allowed is = ',natmax
         stop
      endif
      else if (lign80(1:6).eq.'REMARK') then
         ntit=ntit+1
         if (ntit.le.10) then
            write(6,'(A)') lign80
         else if (ntit.eq.11) then
            write(6,'(A)') ' .... '
         endif
      endif
!
!     2) Ligne suivante du fichier pdb :
!
      goto 105
!
!     3) Fin de la lecture du fichier pdb :
!
 200  continue 
      write(6,*) 'Rdatompdb> End of file reached.'
      write(6,*) 'Rdatompdb> Number of I/O errors: ', &
                  nerr
!
      natom=iatom
      fatres(nres+1)=natom+1
      irs=0
      if (natom.gt.0) irs=iresat(natom)
!
      write(6,'(/(A,I6))') &
      ' Rdatompdb> Number of residues found = ',nres, &
      '            First residue number     = ',iresat(1),&
      '            Last  residue number     = ',irs, &
      '            Number of atoms found    = ',natom
      write(6,'(A,F8.1)') &
      '            Mean number per residue  = ',float(natom)/float(nres)
!
      if (natom.eq.0) then
          write(6,'(A)') &
        '%Rdatompdb-Err> No atom found in file.'
          qerror=.true.
      endif
      if (nres.eq.0) then
          write(6,'(A)') &
        '%Rdatompdb-Err> No residue found in file.'
          qerror=.true.
      endif
!
      return
      end

Here are some notes related to the interaction between BioMove3D and Amber.
They are in no particular order as they reflect the problems encountered with the passage of time.
There is another document which describes the functioning of the Amber related part of BioMove3D (it started out as EnergyModuleInBioMove3D.rtf).

(Note to contributors: Please update the summary as you add content)


Summary
--------------------------------
1. Compilation issues
1.1. Compilation of Fortran and C(++)
2. Use of Amber libraries
2.1. Creation of libSanderManualLibrary.a
2.2. Creation of libLeapManualLibrary.a
3. Programming issues
3.1 Starting index of vectors
4. AMBMOV
4.1 Problems with input files
5. Possible problems
A1. List of contributors




1. Compilation issues
----------------------------------
AS: The compilation under the sparc operating system is made using "f90".
The compilation under the LINUX operating system is currently accomplished with "g95" (see g95.sourceforge.net/).

1.1. Compilation of Fortran and C(++)
-------------------------
The first thing to know when using C and Fortran code together is that the Fortran compiler automatically adds an underscore "_" caracter at the end of the functions names and thus, in order to call these functions from C one needs to add this underscore.
Ex: in order to call the function 
   FirstFortranFunction(parm1)
one will have to declare it in C with:
   extern firstfortranfunction(int *parm1)
The second thing to be noted is that all letters are in lowercaps, so the best thing to do is to use lower caps for the name of the function in Fortran also in order to avoid later problems.

2. Use of Amber libraries
-------------------------------------------
AS: As I write this text I have not yet seen how the libraries for Sander and Leap are obtained on the Sparc system, but I did notice that they do not exist in the Linux compilattion of Amber. In order to avoid long linking commands I have decided to recreate the Amber libraries to my best of knowledge. This here means that after each compilation of Amber under Linux the commands in sections 2.1 and 2.2 have to be executed before compiling BioMove3D.


2.1. Creation of libSanderManualLibrary.a
-----------------------------------------------------------------
AS: Without further explication, here are the commands to be executed in the 
	$(AMBERHOME)/src/sander 
folder:
      ar rv libSanderManualLibrary.a trace.o lmod.o decomp.o icosasurf.o egb.o findmask.o pb_force.o sander.o cshf.o noecalc.o noeread.o caldis.o calrate.o dinten.o drates.o indexn.o kmat.o pearsn.o plane.o remarc.o nmrcal.o nmrred.o restal.o getnat.o nmrnrg.o modwt.o disnrg.o angnrg.o tornrg.o nmrprt.o nmrgrp.o nmrcms.o nmrcmf.o impnum.o nmrsht.o at2res.o chklin.o opnmrg.o printe.o runmin.o ndvprt.o force.o rdparm.o mdread.o locmem.o runmd.o getcor.o r6ave.o r6drv.o aveint.o degcnt.o corf.o threeb.o tripl.o nmrrad.o decnvh.o fastwt.o echoin.o parallel.o jnrg.o shake.o ene.o mdwrit.o minrit.o set.o setmm.o dynlib.o mdfil.o nmlsrc.o ew_force.o ew_setup.o ew_box.o ew_bspline.o ew_fft.o ew_direct.o ew_recip.o pcshift.o align.o rstack.o istack.o rfree.o rgroup.o random.o lsqfit.o amopen.o debug.o ew_recip_reg.o ew_handle_dips.o ew_dipole_recip.o mexit.o new_time.o extra_pts.o thermo_int.o matinv.o assert.o mmtsb.o mmtsb_client.o erfcfun.o veclib.o mdm.o pb_init.o constantph.o prn_dipoles.o

	ranlib libSanderManualLibrary.a

The list of ".o" objects has been obtained from the $(AMBERHOME)/src/sander/Makefile   section   "OBJ=". 

2.2. Creation of libLeapManualLibrary.a
------------------------------------------------------------
AS: In order to create the "libLeapManualLibrary.a" library, one should go to the folder:
	$(AMBERHOME)/src/leap/src/leap
and execute the following commands:
	ar rv libLeapManualLibrary.a basics.o sysdepend.o stringExtra.o varArray.o getline.o avl.o pdb_format.o pdb_read.o pdb_sprntf.o pdb_sscanf.o pdb_write.o vector.o zMatrix.o sort.o bag.o hash.o dictionary.o database.o nVector.o ring.o matrix.o fortran.o displayer.o assoc.o atom.o byteArray.o collection.o container.o internal.o list.o loop.o molecule.o oDouble.o oInteger.o oString.o objekt.o parmSet.o residue.o unit.o unitio.o tripos.o graphUtil.o select.o amber.o build.o elements.o library.o chirality.o minimizer.o model.o parmLib.o pdbFile.o tools.o variables.o parser.o help.o helptext.o octree.o commands.o mathop.o block.o restraint.o hybrid.o

	ranlib libLeapManualLibrary.a

I took the list of ".o" from the sparc library.


3. Programming issues
------------------------------------------------
3.1 Starting index of vectors
----------------------------------------------------
	This section refers to the BioEnergy module. There is a little thing about the starting indexes: Since Fortran uses 1 (ie vect[1]) as first vector element by default, the person that first programmed the BioMove3dD -> Amber interface, did the same thing for the C vectors also. This implies two things:
	- the allocation of vectors has to be done with one extra element: ie. malloc( MAX_SIZE + 1 * sizeof(element) );
	- extra care has to be taken when programming so as not to forget to start at 1 instead of 0.
	Of course this can be changed, but this implies some amount of work and someone has to asume it.


4. AMBMOV
----------------------------------------------------
4.1 Problems with input files
---------------------------------------------------
	While trying to transform a PDB file with our AmbMov, I have encountered some problems. Here is what I have done to solve them. It may help you also:
	a) The protein and the ligand have to be in separate files
	b) At the end of my protein file there was an CA atom which I had to remove
	c) In the ligand file the fifth column contained "1B" (without the quotes). Remove all the 1Bs, but REPLACE THEM with empty spaces, because AMBER counts the characters.


5. Possible problems
----------------------------------------------------
**** AS ****: 
If you encouter this error:
     "(no leaprc in search path)"
then the problem is that Amber does not know which force field to use. In order to specify one, we have to create a "leaprc" file in the $AMBERHOME/dat/leap/cmd folder. If you do not know much about Amber (like me) you can just do "cp leaprc.ff03 leaprc" and things will start to work.

**** AS ****: 
An error that is less obvious is encountered when working with long file names+paths. This is due to a limitation in the internal variables  of AMBER. I will write here the files that I have updated in order to avoid these problems. Here is the error that I get with the G95 fortran compiler when this problem occurs:
     At line 1602 of file _ew_box.f (Unit 9 "fort.9")
     Traceback: not available, compile with -ftrace=frame or -ftrace=full
     Fortran runtime error: End of file
The modification to be performed is: 
   change from:  character(len=80) 	into :		character(len=256) 
			character*80 		into :		character*256
As I said, here are some of the files that need to be modified.
 - amber/src/sander/files.h  :   line 6
 - amber/src/etc/ambpdb.f  :   line 6

Of course this means that Amber has to be recompiled :( and also the concerned libraries (libSanderManualLibrary.a, see above).



A1. List of contributors
---------------------------------
AS: Alin STEFANIU
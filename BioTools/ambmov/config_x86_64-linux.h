

AMBER_SRC=/usr/local/motion/amber_64/src
MOVE3D_BIO_HOME=../../bio
SOME_VAR_BY_AS=/usr/local/motion/g95_64linux/lib/gcc-lib/i686-pc-linux-gnu/4.0.1



AMBERBUILDFLAGS= -I$(AMBER_SRC)/leap/src/leap  -I$(AMBER_SRC)/sander -I$(MOVE3D_BIO_HOME)/BioEnergy/include/ 

#------------------------------------------------------------------------------
# LOCALFLAGS is intended for program specific modifications to the
# Fortran build process and may be modified by the program's local makefile
#------------------------------------------------------------------------------
LOCALFLAGS= -static

#------------------------------------------------------------------------------
# Availability and method of delivery of math and optional libraries
#------------------------------------------------------------------------------
USE_BLASLIB=$(SOURCE_COMPILED)
USE_LAPACKLIB=$(SOURCE_COMPILED)
USE_LMODLIB=$(LMOD_UNAVAILABLE)

#------------------------------------------------------------------------------
# C compiler
#------------------------------------------------------------------------------
CC= gcc
CPLUSPLUS=g++
ALTCC=gcc
CFLAGS=-O2 -g $(AMBERBUILDFLAGS)
ALTCFLAGS= $(AMBERBUILDFLAGS)
CPPFLAGS= $(AMBERBUILDFLAGS)

#------------------------------------------------------------------------------
# Fortran preprocessing and compiler.
# FPPFLAGS holds the main Fortran options, such as whether MPI is used.
#------------------------------------------------------------------------------
FPPFLAGS= -P -I$(AMBER_SRC)/include  $(AMBERBUILDFLAGS)
FPP= cpp -traditional $(FPPFLAGS)
FC= g95
FFLAGS= -g -B$(SOME_VAR_BY_AS) -I$(SOME_VAR_BY_AS) -L$(SOME_VAR_BY_AS) -O0 $(LOCALFLAGS) $(AMBERBUILDFLAGS)
FOPTFLAGS= -g -O3 $(LOCALFLAGS) $(AMBERBUILDFLAGS)
FPP_PREFIX= _
FREEFORMAT_FLAG= -ffree-form

#------------------------------------------------------------------------------
# Loader:
#------------------------------------------------------------------------------
LOAD= g95  $(LOCALFLAGS) $(AMBERBUILDFLAGS)  -B$(SOME_VAR_BY_AS) -I$(SOME_VAR_BY_AS) -L$(SOME_VAR_BY_AS)
LOADCC= gcc  $(LOCALFLAGS) $(AMBERBUILDFLAGS)
LOADLIB=
LM= -lm
LOADPTRAJ= g95  -B$(SOME_VAR_BY_AS) -I$(SOME_VAR_BY_AS) -L$(SOME_VAR_BY_AS)   $(LOCALFLAGS) $(AMBERBUILDFLAGS)
XHOME= /usr/X11R6

#------------------------------------------------------------------------------
#  Other stuff:
#------------------------------------------------------------------------------
SYSDIR=lib
AR=ar r
M4=m4
RANLIB=ranlib
SFX=
MAKEDEPEND=$(AMBER_SRC)/../bin/amber_makedepend


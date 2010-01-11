#==============================================================================
# AMBER Makefile configuration for compiler/architecture: SunOS
# Generated via command: /home/skirillo/amber/amber8/src/configure 
#
# Configuration script written mainly by Joe Krahn, Scott Brozell, and
# Dave Case, with contributions from lots of people.
#==============================================================================

#------------------------------------------------------------------------------
# Main AMBER source root directory
#------------------------------------------------------------------------------
#AMBER_SRC=/home/skirillo/prot12-01

#------------------------------------------------------------------------------
# AMBERBUILDFLAGS provides a hook into the build process for installers;
# for example, to build debug versions of the amber programs
# make -e AMBERBUILDFLAGS="-DDEBUG -g"
#------------------------------------------------------------------------------
AMBERBUILDFLAGS= -I/usr/local/motion/AMBER/amber8/src/sander/ -I/usr/local/motion/AMBER/amber8/src/leap/src/leap 
#AMBERBUILDFLAGS=-DDEBUG -g #stef

#------------------------------------------------------------------------------
# LOCALFLAGS is intended for program specific modifications to the
# Fortran build process and may be modified by the program's local makefile
#------------------------------------------------------------------------------
LOCALFLAGS=

#------------------------------------------------------------------------------
# Availability and method of delivery of math and optional libraries
#------------------------------------------------------------------------------
USE_BLASLIB=$(VENDOR_SUPPLIED)
USE_LAPACKLIB=$(VENDOR_SUPPLIED)
USE_LMODLIB=$(LMOD_UNAVAILABLE)

#------------------------------------------------------------------------------
# C compiler
#------------------------------------------------------------------------------
CC = gcc
#CC= gcc #stef 
CPLUSPLUS= g++
ALTCC=gcc
CFLAGS= -DSYSV  $(AMBERBUILDFLAGS)
ALTCFLAGS= $(AMBERBUILDFLAGS)
CPPFLAGS= $(AMBERBUILDFLAGS)

#------------------------------------------------------------------------------
# Fortran preprocessing and compiler.
# FPPFLAGS holds the main Fortran options, such as whether MPI is used.
#------------------------------------------------------------------------------
FPPFLAGS= -P -I$(AMBER_SRC)/include $(AMBERBUILDFLAGS)
FPP= /lib/cpp $(FPPFLAGS)
FC= f90
FFLAGS= -O1   $(LOCALFLAGS) $(AMBERBUILDFLAGS)
FOPTFLAGS= -O2   $(LOCALFLAGS) $(AMBERBUILDFLAGS)
FPP_PREFIX= _
FREEFORMAT_FLAG= -free

#------------------------------------------------------------------------------
# Loader:
#------------------------------------------------------------------------------
LOAD= f90   $(LOCALFLAGS) $(AMBERBUILDFLAGS)
LOADCC= cc  $(LOCALFLAGS) $(AMBERBUILDFLAGS) 
#LOADCC= gcc  $(LOCALFLAGS) $(AMBERBUILDFLAGS) #stef
LOADLIB=  -xlic_lib=sunperf -lm
LOADPTRAJ= f90   $(LOCALFLAGS) $(AMBERBUILDFLAGS)
XHOME= /usr/openwin

#------------------------------------------------------------------------------
#  Other stuff:
#------------------------------------------------------------------------------
.SUFFIXES:  .f90
SYSDIR=lib
AR=ar rv 
M4=m4
RANLIB=ranlib
SFX=
MAKEDEPEND=$(AMBER_SRC)/../bin/amber_makedepend
#MAKEDEPEND=/home/skirillo/amber/amber8/src/sander/makedepend

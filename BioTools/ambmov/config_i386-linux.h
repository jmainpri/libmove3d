/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */


AMBER_SRC=/usr/local/motion/amber/src
MOVE3D_BIO_HOME=../../bio
FORTRAN_G95_COMPILER_PATH=/usr/local/motion/g95/lib/gcc-lib/i686-pc-linux-gnu/4.0.1
#FORTRAN_G95_COMPILER_PATH=/home/sony/g95/lib

AMBERBUILDFLAGS= -I$(AMBER_SRC)/leap/src/leap  -I$(AMBER_SRC)/sander -I../../src/bio/BioEnergy/include/ 

#------------------------------------------------------------------------------
# LOCALFLAGS is intended for program specific modifications to the
# Fortran build process and may be modified by the program's local makefile
#------------------------------------------------------------------------------
LOCALFLAGS=

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
FFLAGS= -g -B$(FORTRAN_G95_COMPILER_PATH) -I$(FORTRAN_G95_COMPILER_PATH) -L$(FORTRAN_G95_COMPILER_PATH) -O0 $(LOCALFLAGS) $(AMBERBUILDFLAGS)
FOPTFLAGS= -g -O3 $(LOCALFLAGS) $(AMBERBUILDFLAGS)
FPP_PREFIX= _
FREEFORMAT_FLAG= -ffree-form

#------------------------------------------------------------------------------
# Loader:
#------------------------------------------------------------------------------
LOAD= g95  $(LOCALFLAGS) $(AMBERBUILDFLAGS)  -B$(FORTRAN_G95_COMPILER_PATH) -I$(FORTRAN_G95_COMPILER_PATH) -L$(FORTRAN_G95_COMPILER_PATH)
LOADCC= gcc  $(LOCALFLAGS) $(AMBERBUILDFLAGS)
LOADLIB=
LM= -lm
LOADPTRAJ= g95  -B$(FORTRAN_G95_COMPILER_PATH) -I$(FORTRAN_G95_COMPILER_PATH) -L$(FORTRAN_G95_COMPILER_PATH)   $(LOCALFLAGS) $(AMBERBUILDFLAGS)
XHOME= /usr/X11R6

#------------------------------------------------------------------------------
#  Other stuff:
#------------------------------------------------------------------------------
.SUFFIXES:  .f90 .f .c .o
SYSDIR=lib
AR=ar r
M4=m4
RANLIB=ranlib
SFX=
MAKEDEPEND=$(AMBER_SRC)/../bin/amber_makedepend


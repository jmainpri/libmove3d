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

# -*- Makefile -*-
# Init.make.  Generated from Init.make.in by configure.
#
# Copyright (C) 1996-2004 LAAS/CNRS 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
#    - Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    - Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
# $LAAS: Init.make.in,v 1.2 2007/03/07 17:00:40 daniel Exp $

prefix=		/home/xbroquer/BioMove3D/other_libraries/gbM/build/install
exec_prefix=	${prefix}
abs_srcdir=	/home/xbroquer/BioMove3D/other_libraries/gbM/build/..

BINDIR= 	${exec_prefix}/bin
LIBDIR= 	${exec_prefix}/lib
DATADIR=	${prefix}/share
INCLUDEDIR=	${prefix}/include/gbM
PKGCONFIGDIR=   ${exec_prefix}/lib/pkgconfig
MODULESDIR=	${exec_prefix}/lib/gdM
GBM_VERSION=	0.2
# installation directory for tcl files
TCL_FILES_DIR=	${prefix}/share/gbM

CC=		gcc
SWIG=		swig
SWIG_TYPEMAP=	-I/usr/share/swig/1.3.31/tcl/typemaps.i
MKDEP=		/usr/local/openrobots/bin/mkdep
MKINSTALLDIRS=	$(SHELL) $(abs_srcdir)/mkinstalldirs

#CPPFLAGS= 	-I$(GDHE_CLIENT_SRC) -I$(GDHE_TOGL_SRC)
CFLAGS=		-O2 -g -pipe -Wall -Wp,-D_FORTIFY_SOURCE=2 -fexceptions -fstack-protector --param=ssp-buffer-size=4 -m32 -march=i386 -mtune=generic -fasynchronous-unwind-tables -pipe 


LIBTOOL=	$(SHELL) $(top_builddir)/libtool
LTCC=		$(LIBTOOL) --mode=compile $(CC)
LTLD=		$(LIBTOOL) --mode=link $(CC)
LTLDPURE=	$(LIBTOOL) --mode=link purify $(CC)

# Tcl/Tk
TCL_CPPFLAGS=		 -I/usr/include/tk-private/generic
TCL_LDFLAGS=		 -Wl,--export-dynamic  
TCL_LIBS=		-L/usr/lib -ltcl8.4${TCL_DBGX} -L/usr/lib -ltk8.4
TCL_LIBDIR=		/usr/lib
TCL_LIB_FLAGS=		-ltcl8.4${TCL_DBGX} -ltk8.4
TCL_DBGX=		
TCL_SHLIB_CFLAGS=	-fPIC
TCL_SHLIB_LDFLAGS=	
TCL_SHLIB_SUFFIX=	.so
TCL_SHLIB_LD=		${CC} ${CFLAGS} -shared ${LDFLAGS} -Wl,-soname,${@}

# Other libs
LIBS=			

# Math lib
LIBM=			-lm

# GDHE lib
LIBGB=	 		-lgb

# Source directories
#GB_TCL_SRC=		$(top_srcdir)/srcTcl

MAJOR=0
MINOR=0
TINY=0

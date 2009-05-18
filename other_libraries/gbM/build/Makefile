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
# $LAAS: Makefile.in,v 1.4 2007/03/07 17:00:40 daniel Exp $
#
# Makefile pour la librairie gb
#
top_srcdir=	..
include		Init.make

DISTNAME= gbM-0.2

INSTALL=	/usr/bin/install -c

SUBDIRS= src tclsrc tcl

MAKE_SUBDIRS= @for d in $(SUBDIRS); do $(MAKE) -C $$d $@ || exit 2; done

all: Makefile
	$(MAKE_SUBDIRS)

depend:
	$(MAKE_SUBDIRS)

install:: gbM.pc
	@echo LIBTOOL= $(LIBTOOL)
	$(MKINSTALLDIRS) $(LIBDIR)/pkgconfig
	$(INSTALL) -m 664 gbM.pc $(PKGCONFIGDIR)
	$(MAKE_SUBDIRS)

distclean: clean
	rm -f  TAGS Makefile gbM.pc
	$(MAKE_SUBDIRS)

clean:
	$(MAKE_SUBDIRS)

reconf:
	cd $(top_srcdir); aclocal -I .; autoconf; libtoolize --force
	./config.status --recheck
	./config.status

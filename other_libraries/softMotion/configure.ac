dnl autoconf script for softMotion
dnl
dnl Copyright (c) 2001-2007 CNRS/LAAS
dnl
dnl GPL, since some parts were copied from other configure.in
dnl 
AC_INIT(softMotion,1.0)
AC_PREREQ(2.59)


AC_CANONICAL_HOST

AC_PROG_CC

PACKAGE=softMotion
VERSION=0.2
AC_SUBST(PACKAGE)
AC_SUBST(VERSION)
AC_REVISION($LAAS$)

dnl configure libtool
AC_DISABLE_STATIC
AC_LIBTOOL_DLOPEN
AC_PROG_LIBTOOL
AC_PROG_INSTALL

dnl check for LAAS mkdep
AC_PATH_PROG(MKDEP, mkdep, no, $prefix/bin:$exec_prefix/bin:$PATH)

AC_OUTPUT(Makefile 
	softMotion.pc
	Init.make
	src/Makefile
)

dnl autoconf script for gbM
dnl
dnl Copyright (c) 2001-2007,2010 CNRS/LAAS
dnl
dnl GPL, since some parts were copied from other configure.in
dnl 
AC_INIT(gbM,0.3)
AC_PREREQ(2.59)

m4_include(swigAc.m4)

AC_CANONICAL_HOST

AC_PROG_CC

PACKAGE=gbM
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


dnl --- Embedded display facility -----------------------------------
dnl
AC_ARG_ENABLE(gbtcl,
   AC_HELP_STRING([--disable-gbtcl], [disable compilation of the gb tcl module]),
   [
      if test "x$enableval" != "xno" -a "x$enableval" != "x"; then
         GB_TCL="yes"
      else
         GB_TCL="no"
      fi
   ], [GB_TCL=yes])
AC_SUBST(GB_TCL)



dnl----------------------------------------------------------------------
dnl
dnl Tcl/Tk Config
dnl 
AC_ARG_WITH(tcl,
    [  --with-tcl=DIR          directory containing tclConfig.sh],
    [tcl_prefix=$withval],
    [for ac_dir in \
	${exec_prefix}/lib 	\
	/usr/local/lib/tcl8.4	\
	/usr/local/${usrlibdir}	\
	/usr/pkg/lib		\
	/usr/lib/tcl8.4		\
	/usr/${usrlibdir}	\
	; \
    do
	if test -r "$ac_dir/tclConfig.sh"; then
	  tcl_prefix=$ac_dir
	  break
	fi
    done])
if test "$GB_TCL" = "yes"; then
file=${tcl_prefix}/tclConfig.sh
. $file
fi
dnl substitute variables in TCL_LIB_FILE
eval TCL_LIB_FILE=${TCL_LIB_FILE}

AC_MSG_CHECKING([for tcl headers])
test -z "$tcl_test_include" && tcl_test_include=tcl.h
for ac_dir in \
	$TCL_PREFIX/include/tcl$TCL_VERSION	\
	$TCL_PREFIX/include 			\
	/usr/local/include/tcl$TCL_VERSION 	\
	/usr/local/include 			\
	/usr/include 				\
	/Library/Frameworks/Tcl.framework/Headers \
	$extra_include 				\
	; \
  do
  if test -r "$ac_dir/$tcl_test_include"; then
    ac_tcl_includes=$ac_dir
    break
  fi
done
if test "x$ac_tcl_includes" != "x"; then
  AC_MSG_RESULT($ac_tcl_includes)
else
if test "$GB_TCL" = "yes"; then
  AC_MSG_RESULT([Not found (fatal)])
  exit 2;
fi
fi

AC_MSG_CHECKING([for tcl library])
test -z "$tcl_test_lib" && tcl_test_lib="${TCL_LIB_FILE}"
for ac_dir in \
	$TCL_PREFIX/lib				\
	/usr/local/${usrlibdir}			\
	/usr/${usrlibdir}			\
	/Library/Frameworks/Tcl.framework	\
	$extra_lib				\
	; \
  do
  if test -r "$ac_dir/$tcl_test_lib"; then
    ac_tcl_libs=$ac_dir
    break
  fi
done
if test "x$ac_tcl_libs" != "x"; then
  AC_MSG_RESULT($ac_tcl_libs/$TCL_LIB_FILE)
else
if test "$GB_TCL" = "yes"; then
  AC_MSG_RESULT([Not found (fatal)])
  exit 2;
fi
fi
LIB_RUNTIME_DIR=$ac_tcl_libs

AC_ARG_WITH(tk,
    [  --with-tk=DIR           directory containing tkConfig.sh],
    [tk_prefix=$withval],
    [for ac_dir in \
	${tcl_prefix}			\
	${exec_prefix}/lib 		\
	/usr/local/lib/tk$TCL_VERSION	\
	/usr/local/${usrlibdir}		\
	/usr/pkg/lib			\
	/usr/lib/tk$TCL_VERSION		\
	/usr/${usrlibdir}		\
	; \
    do
	if test -r "$ac_dir/tkConfig.sh"; then
	  tk_prefix=$ac_dir
	  break
	fi
    done])

if test "$GB_TCL" = "yes"; then
file=${tk_prefix}/tkConfig.sh
. $file
fi
dnl substitute variables in TK_LIB_FILE
eval TK_LIB_FILE=${TK_LIB_FILE}


AC_MSG_CHECKING([for tk headers])
test -z "$tk_test_include" && tk_test_include=tk.h
for ac_dir in \
	$TK_PREFIX/include/tk$TK_VERSION	\
	$TK_PREFIX/include 			\
	$ac_tcl_includes			\
	/usr/local/include/tk$TK_VERSION 	\
	/usr/local/include 			\
	/usr/include 				\
	/Library/Frameworks/Tk.framework/Headers \
	$extra_include 				\
	; \
  do
  if test -r "$ac_dir/$tk_test_include"; then
    ac_tk_includes=$ac_dir
    break
  fi
done
if test "x$ac_tk_includes" != "x"; then
  AC_MSG_RESULT($ac_tk_includes)
else
if test "$GB_TCL" = "yes"; then
  AC_MSG_RESULT([Not found (fatal)])
  exit 2;
fi
fi
dnl look for tkInt.h
AC_MSG_CHECKING([for TkInt.h])
for ac_dir in \
	$TK_SRC_DIR/generic				\
	$TK_PREFIX/include/generic			\
	$TK_PREFIX/include/tk$TK_VERSION/generic	\
	/Library/Frameworks/Tk.framework/PrivateHeaders \
	$srcdir/tk/$TK_VERSION				\
	; \
  do
  if test -r "$ac_dir/tkInt.h"; then
    ac_tk_int=$ac_dir
    break
  fi
done
if test "x$ac_tk_int" != "x"; then
  AC_MSG_RESULT($ac_tk_int)
else
if test "$GB_TCL" = "yes"; then
  AC_MSG_RESULT([Not found (fatal)])
  exit 2;
fi
fi

if test "$ac_tcl_includes" != "/usr/include"; then
  TCL_CPPFLAGS=-I$ac_tcl_includes
else 
  TCL_CPPFLAGS=""
fi
if test "$ac_tk_includes" != "$ac_tcl_includes"; then
	TCL_CPPFLAGS="$TCL_CPPFLAGS -I$ac_tk_includes"
fi
if test "$ac_dir" = "$srcdir/tk/$TK_VERSION"; then
	TCL_CPPFLAGS="$TCL_CPPFLAGS -I\${abs_srcdir}/tk/${TK_VERSION}"
else
	TCL_CPPFLAGS="$TCL_CPPFLAGS -I$ac_tk_int"
fi
AC_SUBST(TCL_CPPFLAGS)
AC_SUBST(TCL_EXTRA_CFLAGS)

eval TCL_LDFLAGS=\"${TCL_LD_FLAGS} ${TCL_LD_SEARCH_FLAGS}\"
eval TK_LDFLAGS=\"${TK_LD_SEARCH_FLAGS}\"
if test "$TK_LDFLAGS" != "$TCL_LDFLAGS"; then
  TCL_LDFLAGS="$TCL_LDFLAGS $TK_LDFLAGS"
fi

eval TK_SHLIB_LDFLAGS=\"${TK_LD_SEARCH_FLAGS}\"
TCL_SHLIB_LDFLAGS="$TK_SHLIB_LDFLAGS"
TCL_LIB_FLAGS="$TCL_LIB_FLAG $TK_LIB_FLAG"

AC_SUBST(TCL_CC)
AC_SUBST(LIB_RUNTIME_DIR)
AC_SUBST(TCL_LDFLAGS)
AC_SUBST(TCL_LIB_SPEC)
AC_SUBST(TK_LIB_SPEC)
AC_SUBST(TCL_SHLIB_CFLAGS)
AC_SUBST(TCL_SHLIB_LDFLAGS)
AC_SUBST(TCL_SHLIB_LD)
AC_SUBST(TCL_SHLIB_SUFFIX)
AC_SUBST(TCL_LIB_FLAGS)
AC_SUBST(TCL_DBGX)



dnl  swig
dnl include acincludeswig.m4
AC_PROG_SWIG

AC_OUTPUT(Makefile 
	gbM.pc
	Init.make
	tcl/Makefile
	tcl/mkIndex
	tclsrc/Makefile
	src/Makefile
)



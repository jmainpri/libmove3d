
all: ambmov 

include config_$(HOSTTYPE).h

CFLAGS += -I../../include

# Sander Fortran source files are free format
LOCALFLAGS = -g $(FREEFORMAT_FLAG)

SHELL=/bin/sh
VPATH=

BIOENERGY_FORTRAN_FILES_PATH=./../../bio/BioEnergy

SRC= 	             \
	$(BIOENERGY_FORTRAN_FILES_PATH)/data_init.f \
	$(BIOENERGY_FORTRAN_FILES_PATH)/read_inp.f \
	$(BIOENERGY_FORTRAN_FILES_PATH)/read_parm.f \
	$(BIOENERGY_FORTRAN_FILES_PATH)/mini.f\
	atomhandling.c \
	pdbFormat.c \
	removeH.c  \
	pdbtopsf.c \
	mathfcts.c 

LIB_AMBMOV = libambmov.a


MEMLIB_ENERGY = $(patsubst %.f, $(LIB_AMBMOV)(%.o), $(SRC))
MEMLIB_ENERGY += $(patsubst %.c, $(LIB_AMBMOV)(%.o), $(SRC))


$(LIB_AMBMOV) : $(MEMLIB_ENERGY)
	$(RANLIB) $(LIB_AMBMOV)


# TODO: Replace the code copied from Move3D avec the libraries from Move3D.
#   here below is a beginning with libbio and libp3d
#LIB_BIO=../../bin/$(HOSTTYPE)/libbio.a
#LIB_P3D=../../bin/$(HOSTTYPE)/libp3d.a
#$(LIB_P3D):
#$(LIB_BIO):
#	cd ../../bin/$(HOSTTYPE) \
#	$(MAKE)


#---------------------------------------------------------------------------
#ambmov: $(LIB_P3D) $(LIB_BIO) ambmov.o $(LIB_AMBMOV)
ambmov: ambmov.o $(LIB_AMBMOV)

ifeq ($(HOSTTYPE),i386-linux)
########   These are the compiling options for LINUX
	$(LOAD) -o ambmov ambmov.o $(LIB_P3D) $(LIB_BIO) $(LIB_AMBMOV) \
		$(AMBERHOME)/src/lib/sys.a \
		$(AMBERHOME)/src/leap/src/leap/libLeapManualLibrary.a\
		$(AMBERHOME)/src/sander/libSanderManualLibrary.a\
		$(AMBERHOME)/src/lmod/lmod.a \
		$(AMBERHOME)/src/lapack/lapack.a \
		$(AMBERHOME)/src/blas/blas.a \
		$(AMBERHOME)/src/lib/nxtsec.o 

else
  ifeq ($(HOSTTYPE),x86_64-linux)
  ########   These are the compiling options for LINUX 64
	$(LOAD) -o ambmov ambmov.o $(LIB_P3D) $(LIB_BIO) $(LIB_AMBMOV) \
		$(AMBERHOME)/src/lib/sys.a \
		$(AMBERHOME)/src/leap/src/leap/libLeapManualLibrary.a\
		$(AMBERHOME)/src/sander/libSanderManualLibrary.a\
		$(AMBERHOME)/src/lmod/lmod.a \
		$(AMBERHOME)/src/lapack/lapack.a \
		$(AMBERHOME)/src/blas/blas.a \
		$(AMBERHOME)/src/lib/nxtsec.o 

  else
  #######    Here below are the options for compiling under SOLARIS
	$(LOAD) -o ambmov  ambmov.o $(LIB_AMBMOV) $(LOADLIB) \
	/usr/local/motion/AMBER/lib/sparc/libAmbSander.a \
	/usr/local/motion/AMBER/lib/sparc/libAmbLeap.a \
	/home/skirillo/amber/amber8/src/lib/nxtsec.o \
	/home/skirillo/amber/amber8/src/lapack/lapack.a \
	/home/skirillo/amber/amber8/src/blas/blas.a \
	/home/skirillo/amber/amber8/src/lmod/lmod.a \
	/home/skirillo/amber/amber8/src/lib/sys.a \
	/home/skirillo/amber/amber8/src/divcon/libdivcon.a \
	/home/skirillo/amber/amber8/src/divcon/block.o

  endif
endif


clean:
	-/bin/rm -f ambmov ambmov.o $(LIB_AMBMOV)





#  default rules for Fortran and C compilation:
(%.o) : %.f
	$(FPP) $(FPPFLAGS)  $< > $(<:.f=_.f)
	$(FC) -c $(FFLAGS) -o $% $(<:.f=_.f)
	$(AR) $@ $%
#	rm -f $% $(<:.f=_.f)

(%.o) : %.f90
	$(FPP) $(FPPFLAGS)  $< > $(<:.f90=_.f90)
	$(FC) -c $(FFLAGS) -o $% $(<:.f90=_.f90)
	$(AR) $@ $%
	rm -f $% $(<:.f=_.f)

(%.o) : %.c
	$(CC) -c $(CFLAGS) $<
	ar r $@ $%
	rm -f $%

.cc.o:
	$(CC) -c $(CFLAGS) $<
	ar r $@ $%
	rm -f $%



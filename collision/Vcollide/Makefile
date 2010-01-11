############################################################################
#
# Top Level Makefile for V-Collide.
#
#
############################################################################

#Choose the C++ compiler you want
CC = CC

# If you want the demos to be compiled with a graphical display of 
# the simulation, then uncomment the following lines and provide
# the correct locations of the libraries and include files.

#GRAPHICS_CFLAGS   = -DOGL_GRAPHICS
#GRAPHICS_INCLUDES = -I/afs/unc/project/walk/include
#GRAPHICS_LDFLAGS  = -L/afs/unc/project/walk/lib/`hw_os`
#GRAPHICS_LIBS     = -lglut -lGLU -lGL -lX11 -lXmu

all : 
	cd RAPID; \
	$(MAKE) CC="$(CC)"
	cd src; \
	$(MAKE) CC="$(CC)" install
	cd demos; \
	$(MAKE) CC="$(CC)" GRAPHICS_CFLAGS="$(GRAPHICS_CFLAGS)" GRAPHICS_INCLUDES="$(GRAPHICS_INCLUDES)" GRAPHICS_LDFLAGS="$(GRAPHICS_LDFLAGS)" GRAPHICS_LIBS="$(GRAPHICS_LIBS)"

clean : 
	cd RAPID; \
	$(MAKE) clean
	cd src; \
	$(MAKE) clean
	cd lib; \
	rm -f libVCollide.a
	cd include; \
	rm -f VCollide.H VCol.h
	cd demos; \
	$(MAKE) clean


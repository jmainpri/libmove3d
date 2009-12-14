VERBOSE = -DPRINT_INFO -DPRINT_ERROR -DPRINT_WARNING
HALF_VERBOSE = -DPRINT_ERROR -DPRINT_WARNING
SILENT = -DPRINT_ERROR -DPRINT_INFO

include ../../Init.make.move3d
include ../../Init.make.$(HOSTTYPE)
-include ../../Init.make.custom
LOCALFLAGS = $(FREEFORMAT_FLAG)

#---
# Dependances
#---

#all: $(MOVE3D_LIBS) move3d

all: move3d

lib: move3dlib
dylib:move3dDylib

clean:
	rm -f $(MOVE3D_LIBS) move3d
	rm -f DEPENDANCES*
	rm -f *.o
	rm -f moc*
	rm -f *.moc

cleanlib:
	rm -f $(MOVE3D_LIBS) move3d
	rm -f DEPENDANCES*
	rm -f *.o
	rm -f libBiomove3d.a
	rm -f libBiomove3d.so

ifeq (1,$(COMPILE_AS_CXX))
CC = $(CXX)
CFLAGS = $(CXXFLAGS)
else
CXXSTDLIB = -lstdc++
endif

ifeq (1,$(QT_LIBRARY))
MAINFILE = move3d.o main.o
else
MAINFILE = move3d.o
endif

#main.o: main.moc

move3d: $(ALLOBJECTS) $(MAINFILE) 
	$(PURIFY) $(CC) $(CFLAGS) $(CPPFLAGS) $(CXXSTDLIB) $(ALLOBJECTS) $(MAINFILE) -o move3d \
	$(QT_LIB) \
	$(XFORMS_LIB) \
	$(OPENGL_LIB) \
	$(AMBER_LIB) \
	$(MATH_LIB) -lm \
	$(DMALLOC_LIB) \
	$(XML_LIB)

move3d_with_libs: $(MOVE3D_LIBS) $(MAINFILE)
	$(PURIFY) $(CC) $(CFLAGS) $(CPPFLAGS) $(CXXSTDLIB) -o move3d $(MAINFILE) \
	$(MOVE3D_LIBS) \
	$(XFORMS_LIB) \
	$(OPENGL_LIB) \
	$(AMBER_LIB) \
	$(MATH_LIB) -lm \
	$(QT_LIB) \
	$(DMALLOC_LIB) \
	$(XML_LIB)
	rm -f *.o

#move3dlib: $(MOVE3D_LIBS) move3dMhp.o
move3dlib: $(ALLOBJECTS) move3dMhp.o
	@echo "***************************************"
	@echo "* Making the library for genom module *"
	@echo "* bin/$(HOSTTYPE)/libBiomove3d.a      *"
	@echo "***************************************"
	ar r libBiomove3d.a $(ALLOBJECTS) move3dMhp.o
	rm -f __.*

move3dDylib: $(ALLOBJECTS) move3dMhp.o
	@echo "*****************************************"
	@echo "* Making the library for genom module   *"
	@echo "* bin/$(HOSTTYPE)/libBiomove3d.so *"
	@echo "*****************************************"
	gcc -shared -Wl,-soname,libBiomove3d.so.1 -o libBiomove3d.so $(ALLOBJECTS) move3dMhp.o
	ln -s libBiomove3d.so libBiomove3d.so.1

# Default rule for libraries
define compile_module
$(LIB_$(1)) : $(MEMLIB_$(1))
	$(RANLIB) $(LIB_$(1))
	ar x $(LIB_$(1))
endef
$(foreach MOD,$(MOVE3D_MODULES),$(eval $(call compile_module,$(MOD))))

# ================== Regles par defaut ====================
#-----------------------------------------------------------
ifeq (1,$(ENERGY))
(%.o) : %.f
	$(FPP) $(FPPFLAGS)  $< > $(<:.f=_.f)
	$(CF) -c $(FFLAGS) -o $% $(<:.f=_.f)
	ar r $@ $%
	rm -f $% $(<:.f=_.f)
#	rm -f  $(<:.f=_.f)
endif
#-----------------------------------------------------------
(%.o) : %.c
	$(PURIFY) $(CC) -c $(CFLAGS) $(CPPFLAGS) $<
	ar r $@ $%
	rm -f $%

(%.o) : %.C
	$(PURIFY) $(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $<
	ar r $@ $%
	rm -f $%
	
(%.o) : %.cpp moc_%.cpp
	$(PURIFY) $(CXX) -c -DDEBUG $(CXXFLAGS) $(CPPFLAGS)  $<
	ar r $@ $%
	rm -f $%

%.o : %.c
	$(PURIFY) $(CC) -c -fPIC $(CFLAGS) $(CPPFLAGS) $<

%.o : %.C
	$(PURIFY) $(CXX) -c -fPIC $(CXXFLAGS) $(CPPFLAGS) $<

mainwindow.o: mainwindow.ui
sidewindow.o: sidewindow.ui
plotwindow.o: plotwindow.ui
moverobot.o: moverobot.ui

ifeq (1,$(QT_LIBRARY))
%.o : %.cpp
ifeq (1,$(QT_UI_XML_FILES))
	$(MY_UIC) $(UIC) $<
endif
	$(MY_MOC) $(MOC) $< $(CPPFLAGS) 
	$(PURIFY) $(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $<
else
%.o : %.cpp
	$(PURIFY) $(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $<
endif

%.o : %.cc
	$(PURIFY) $(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $<



DEPENDANCES_ALL = $(foreach LIB, $(MOVE3D_LIBS), DEPENDANCES.$(LIB))

depend: $(DEPENDANCES_ALL)
	cat $^ > DEPENDANCES

DEPENDANCES:
	@cat > DEPENDANCES << EOF
	$(MAKE) depend

# Default rule for calculating dependencies
define module_depends
DEPENDANCES.$(LIB_$(1)): $(SRC_$(1))
	$(MKDEP) -c$(CC) -p$(VPATH) -l$(LIB_$(1)) $(CPPFLAGS) \
	$(SRC_$(1)) > DEPENDANCES.$(LIB_$(1))
endef

$(foreach MOD,$(MOVE3D_MODULES),$(eval $(call module_depends,$(MOD))))

include DEPENDANCES

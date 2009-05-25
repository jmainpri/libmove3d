
all: $(HOSTTYPE)

#----------------------------------------------------------------------
# POSSIBLE ACTIONS
#----------------------------------------------------------------------

ARCHS = sparc i386-linux iris4d powermac x86_64-linux macintel

$(ARCHS):
	@echo
	@echo "************* Compiling bin/$@"
	@echo

	@cd bin/$@; $(MAKE)

# clean

clean: clean-$(HOSTTYPE)

define clean-arch
clean-$(1) :
	@cd bin/$(1); $(MAKE) clean
endef

$(foreach ARCH,$(ARCHS),$(eval $(call clean-arch,$(ARCH))))

# cleanlib

cleanlib: cleanlib-$(HOSTTYPE)

define cleanlib-arch
cleanlib-$(1) :
	@cd bin/$(1); $(MAKE) cleanlib
endef

$(foreach ARCH,$(ARCHS),$(eval $(call cleanlib-arch,$(ARCH))))

# dependances

depend: depend-$(HOSTTYPE)

define depend-arch
depend-$(1):
	@cd bin/$(1); $(MAKE) depend
endef

$(foreach ARCH,$(ARCHS),$(eval $(call depend-arch,$(ARCH))))

# function prototypes

proto:
	@cd util; $(MAKE) proto
	@cd planner; $(MAKE) proto
	@cd localpath; $(MAKE) proto
	@cd collision; $(MAKE) proto
	@cd collision/Kcd; $(MAKE) proto
	@cd graphic; $(MAKE) proto
	@cd p3d; $(MAKE) proto
	@cd sdk; $(MAKE) proto
	@cd move3d; $(MAKE) proto
	@cd userappli; $(MAKE) proto
	@cd animation; $(MAKE) proto
	@cd bio; $(MAKE) proto

clean-proto:
	@cd util; $(MAKE) clean-proto
	@cd planner; $(MAKE) clean-proto
	@cd localpath; $(MAKE) clean-proto
	@cd collision; $(MAKE) clean-proto
	@cd collision/Kcd; $(MAKE) clean-proto
	@cd graphic; $(MAKE) clean-proto
	@cd p3d; $(MAKE) clean-proto
	@cd sdk; $(MAKE) clean-proto
	@cd move3d; $(MAKE) clean-proto
	@cd userappli; $(MAKE) clean-proto
	@cd animation; $(MAKE) clean-proto
	@cd bio; $(MAKE) clean-proto


#lib

lib: lib-$(HOSTTYPE)

define lib-arch
lib-$(1) :
	@cd bin/$(1); $(MAKE) lib
endef

$(foreach ARCH,$(ARCHS),$(eval $(call lib-arch,$(ARCH))))


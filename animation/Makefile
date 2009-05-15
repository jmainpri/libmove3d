include ../Init.make.move3d
include ../Init.make.$(HOSTTYPE)

PROTO_ANIM = $(patsubst %.c, ../animation/proto/%_proto.h, $(SRC_ANIM))

#
all:	sparc

#----------------------------------------------------------------------
# POSSIBLE ACTIONS
#----------------------------------------------------------------------

sparc i386-linux iris4d:
	@echo
	@echo "************* Compiling bin/$@"
	@echo

	@cd ../bin/$@; $(MAKE)

depend: 
	@cd ..; $(MAKE) depend

proto: $(PROTO_ANIM) $(SRC_ANIM)
	@echo "**  Generating protos"
	$(ALLPROTO) proto/anim_proto.h $(PROTO_ANIM)

../animation/proto/%_proto.h:%.c
	$(CEXTRACT) $(CPPFLAGS) -DPROTO -P +p -s=none -o aux $<
	- red -s aux <../bin/script/Proto.move3d >/dev/null
	mv aux $@


clean-sparc:
	@cd ../bin/sparc; $(MAKE) clean
clean-i386-linux:
	@cd ../bin/i386-linux; $(MAKE) clean
clean-iris4d:
	@cd ../bin/iris4d; $(MAKE) clean
clean-powermac:
	@cd ../bin/powermac; $(MAKE) clean

clean-proto:
	rm -f $(PROTO_ANIM) proto/anim_proto.h

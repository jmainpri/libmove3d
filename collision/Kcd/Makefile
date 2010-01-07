include ../../Init.make.move3d
include ../../Init.make.$(HOSTTYPE)

PROTO_DIR2 = ../collision/Kcd/proto
PROTO_DIR = ../$(PROTO_DIR2)
PROTO_KCD = $(patsubst %.c, $(PROTO_DIR)/%_proto.h, $(SRC_KCD))
PROTO2_KCD = $(patsubst %.c, $(PROTO_DIR2)/%_proto.h, $(SRC_KCD))

#
all:	sparc

#----------------------------------------------------------------------
# POSSIBLE ACTIONS
#----------------------------------------------------------------------

sparc i386-linux iris4d:
	@echo
	@echo "************* Compiling bin/$@"
	@echo

	@cd ../../bin/$@; $(MAKE)

depend: 
	@cd ../..; $(MAKE) depend

proto: $(PROTO_DIR) $(PROTO_KCD) $(SRC_KCD)
	@echo "**  Generating protos"
	$(KCD_ALLPROTO) proto/kcd_proto.h $(PROTO2_KCD)

$(PROTO_DIR):
	@echo "**  Creating proto directory"
	@mkdir $(PROTO_DIR)
	@cvs add $(PROTO_DIR)
	@cat > proto/kcd_proto.h << EOF

$(PROTO_DIR)/%_proto.h:%.c
	$(CEXTRACT) $(CPPFLAGS) -DPROTO -P +p -s=none -o aux $<
	- red -s aux <../../bin/script/Proto.move3d >/dev/null
	mv aux $@

clean-sparc:
	@cd ../../bin/sparc; $(MAKE) clean
clean-i386-linux:
	@cd ../../bin/i386-linux; $(MAKE) clean
clean-iris4d:
	@cd ../../bin/iris4d; $(MAKE) clean

clean-proto:
	rm -f $(PROTO_KCD)
	cat > proto/kcd_proto.h << EOF

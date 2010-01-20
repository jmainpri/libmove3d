include ../Init.make.move3d
include ../Init.make.$(HOSTTYPE)

PROTO_DIR = ../graphic/proto
PROTO_GRAPHIC = $(patsubst %.c, $(PROTO_DIR)/%_proto.h, $(SRC_GRAPHIC))

#
all:	$(HOSTTYPE)

#----------------------------------------------------------------------
# POSSIBLE ACTIONS
#----------------------------------------------------------------------

sparc i386-linux iris4d powermac macintel:
	@echo
	@echo "************* Compiling bin/$@"
	@echo

	@cd ../bin/$@; $(MAKE)

depend: 
	@cd ..; $(MAKE) depend

$(PROTO_DIR):
	@echo "**  Creating proto directory"
	@mkdir $(PROTO_DIR)
	@cvs add $(PROTO_DIR)
	@cat > proto/graphic_proto.h << EOF

proto: $(PROTO_DIR) $(PROTO_GRAPHIC) $(SRC_GRAPHIC)
	@echo "**  Generating protos"
	$(ALLPROTO) proto/graphic_proto.h $(PROTO_GRAPHIC)

$(PROTO_DIR)/%_proto.h:%.c
	cd proto; $(CEXTRACT) $(CPPFLAGS) \
	 -DPROTO -P +p -s=none -o ../aux ../$<
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
clean-macintel:
	@cd ../bin/macintel; $(MAKE) clean

clean-proto:
	rm -f $(PROTO_GRAPHIC)
	cat > proto/graphic_proto.h << EOF

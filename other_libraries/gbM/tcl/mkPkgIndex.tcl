
puts "Rebuilding package index in [pwd]"
#pkg_mkIndex . gb.tcl gbModeles.tcl -load ../tclsrc/gb.so
pkg_mkIndex -verbose . gb.tcl  pipo.tcl -load ../buildir/install/lib/gb.so
exit 0

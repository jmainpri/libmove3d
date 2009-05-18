#  testGb.tcl
# Copyright (c) 2002-2007 LAAS/CNRS -- RIA --
# Daniel SIDOBRE -- mars 2007
# 

package require gbM
package require tcltest


::tcltest::configure -verbose {pass start}

set doubleTestEpsilon 1e-10
proc listDoubleTest { lexpected lresult } {
    foreach el1 $lresult el2 $lexpected {
	set err [expr $el2 -$el1]
	if { [expr $err < -$::doubleTestEpsilon]
	     || [expr $::doubleTestEpsilon < $err] } {
	    return 0
	}
    }
    return 1
}
::tcltest::customMatch listDouble listDoubleTest 

proc testDefaultSymbol { symbol type value} {
    ::tcltest::test "symbol_[set type]_[set symbol]" \
	"test if [set symbol] of type [set type] exist and its value" \
	-match listDouble -body {
	    Gb_[set type]_get [set symbol]
	} -result $value
}

proc testCompare { symbol1 symbol2 type} {
    ::tcltest::test "compare_[set symbol1]_[set symbol2]" \
	"compare symbols [set symbol1] and [set symbol2]" \
	-match listDouble -body {
	    Gb_[set type]_get [set symbol1]
	} -result { 
	    Gb_[set type]_get [set symbol1]
	}
}

proc pirand { } {
    return [ expr (rand() - 0.5) * 2. * 3.14159265358979323846 ]
}

foreach { symbol type value } {
    V0 v3 "0 0 0"
    VX v3 "1 0 0"
    VY v3 "0 1 0"
    VZ v3 "0 0 1"
    ThId th "1 0 0  0 1 0  0 0 1  0 0 0"
} {
    testDefaultSymbol $symbol $type $value 
}

proc testCreate { symbol type } {
    ::tcltest::test "creating_[set type]_[set symbol]" \
	"test creating [set symbol] of type [set type]" \
	-body {
	    set [set symbol] [Gb_[set type] [set symbol]]
	} -match regexp -result "_\[0-9a-f\]+_p_Gb_[set type]"
}

testCreate v3a v3
testCreate v3b v3

proc testSet { symbol type args } {
    ::tcltest::test "testSet_[set type]_[set symbol]" \
	"test setting [set symbol] of type [set type], values are [set args]" \
	-body {
	    eval Gb_[set type]_set [set symbol] [set args]
	    Gb_[set type]_get [set symbol]
	} \
	-match listDouble \
	-result $args
}

testSet v3a v3 [pirand] [pirand] [pirand]

proc testObject { symbol args } {
    set liste ""
    foreach {el val} $args {
	lappend liste $val
    }
    set lresult ""
    ::tcltest::test "testObject_[set symbol]" \
	"testObject [set symbol], fields are [set args]" \
	-body {
	    foreach {el val} $args {
		[set symbol] configure $el $val
		lappend lresult [[set symbol] cget $el]
	    }
	    set lresult
	} \
	-match listDouble \
	-result $liste
}

testObject v3a -x [pirand] -y [pirand] -z [pirand]

#myTest unset v3a

#myTest Gb_v3_print VX
#myTest Gb_v6_print GbV6_0

testCreate v6a v6
testSet v6a v6 0.1 0.2 0.3  1.1 1.2 1.3
testObject v6a -x 12 -y 23 -z 34 -rx 43 -ry 32 -rz 21

testCreate f force
testSet f force 1 2 3  7 8 9
testObject f -x 17 -y 73 -z 34 -rx 43 -ry 37 -rz 71

testCreate vsix vitesse
testSet vsix vitesse 1 2 3  7 8 9
testObject vsix -x 17 -y 73 -z 34 -rx 43 -ry 37 -rz 71

testCreate th th
eval testSet th th [Gb_th_get ThId]

return

myTest Gb_th_get ThId

myTest set gbTh_th [Gb_th gbTh_th]
myTest eval Gb_th_set gbTh_th [Gb_th_get ThId]
myTest Gb_th_print gbTh_th
	
myTest Gb_th th1
myTest Gb_th th2
myTest Gb_th th3
myTest Gb_dep dep
myTest Gb_dep_set dep 1 0 0  1 0 0 1
myTest Gb_dep_th dep th1
myTest Gb_th_inverse th1 th2
myTest Gb_th_produit th1 th2 th3
myTest Gb_th_print th3

myTest eval Gb_th_set th2 [Gb_th_get th1]

myTest Gb_v3 v3
myTest Gb_v3 v3s
myTest Gb_v6 v6
myTest Gb_v6 v6s
myTest Gb_v3_set v3 0 0 1
myTest Gb_v6_set v6 0 1 0 1 0 0
myTest Gb_dep_set dep 10 20 30  1 0 0 $M_PI_4
myTest Gb_dep_th dep th1
myTest Gb_th_x_v3 th1 v3 v3s
myTest Gb_v3_get v3s
myTest Gb_thInv_x_v6 th1 v6 v6s
myTest Gb_v6_get v6s

myTest Gb_dep dep
myTest Gb_dep_set dep 0 0 0 0 0 1 1.57079632679489661923
myTest Gb_th th
myTest Gb_dep_th dep th
myTest Gb_th_print th

myTest set f1 [Gb_force f1]
myTest set f2 [Gb_force f2]
myTest f2 configure -x 1 -y 0 -z 0 -rx 0 -ry 0 -rz 0
myTest set v [Gb_v6 v]

myTest Gb_th_x_force th f2 f1
myTest Gb_v6_get $f1


myTest set dep0 [Gb_dep dep0]
myTest Gb_th_dep ThId dep0
myTest set quat_0 [Gb_quat quat_0]
myTest Gb_th_quat ThId quat_0


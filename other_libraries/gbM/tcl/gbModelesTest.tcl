#
# Copyright (c) 2003 LAAS/CNRS -- RIA --
# Daniel SIDOBRE -- fevrier 2003


source gb.tcl
source gbModeles.tcl

proc myTest { args } {
    uplevel 1  "puts \"\#\# $args\""
    puts [uplevel 1  eval $args ]
}


set mgdData [Gb_dataMGD mgdData]
Gb_atan2 -0.0001 -1

Gb_dataMGD_puts $mgdData

set gt6a [Gb_6rParameters gt6a]
Gb_6rParameters_a2_set gt6a 0.4
Gb_6rParameters_r4_set gt6a 0.4
Gb_6rParameters_epsilon_set gt6a 0.00001

set pa10 [Gb_6rParameters pa10]
Gb_6rParameters_a2_set pa10 45
Gb_6rParameters_r4_set pa10 48
Gb_6rParameters_epsilon_set pa10 0.01


set q6 [Gb_q6 q6]
Gb_q6_set q6 0 0 0  0 0 0
Gb_q6_get q6

set th06 [Gb_th th06]

Gb_MGD6rTh gt6a q6 mgdData th06


set q6Old [Gb_q6 q6Old]
Gb_q6_set q6Old [Gb_q6_get q6]
 

Gb_MGI6rTh gt6a th06 1 1 1 q6Old mgdData q6

set jac [Gb_jac jac]

proc Gb_jac_print { jac } {
    foreach ell { x y z rx ry rz } {
	foreach elc { c0 c1 c2 c3 c4 c5 } {
	    puts -nonewline "[[$jac cget -$elc] cget -$ell]\t"
	}
	puts ""
    }
}

Gb_MDD6r gt6a mgdData th06 jac

proc test_MGD_MGDI { q6 {epsilon 1e-5} } {
    set th06 [new_Gb_th]
    set q6_n [new_Gb_q6]
    set mgdData [new_Gb_dataMGD]
    Gb_MGD6rTh ::gt6a $q6 $mgdData $th06
    set e1 [Gb_dataMGD_e1_get $mgdData]
    set e2 [Gb_dataMGD_e2_get $mgdData]
    set e3 [Gb_dataMGD_e3_get $mgdData]
    Gb_MGI6rTh ::gt6a $th06 $e1 $e2 $e3 $q6 $mgdData $q6_n
    set qdiff [new_Gb_v6]
    set vq6 [new_Gb_v6]
    set vq6_n [new_Gb_v6]
    eval Gb_v6_set $vq6 [Gb_q6_get $q6]
    eval Gb_v6_set $vq6_n [Gb_q6_get $q6_n]
    Gb_v6_moins $vq6 $vq6_n $qdiff
    puts "[Gb_q6_get $q6_n] epsilon= [Gb_v6_module $qdiff]"
    if { [Gb_v6_module $qdiff] <= $epsilon } {
	return 1
    } else {
	return 0
    }
}

proc test_MGD_MGDI2 { args } {
    set th06 [new_Gb_th]
    set q6_n [new_Gb_q6]
    set q6 [new_Gb_q6]
    eval Gb_q6_set $q6 $args
    puts "q6= [Gb_q6_get $q6]"
    set mgdData [new_Gb_dataMGD]
    Gb_MGD6rTh ::gt6a $q6 $mgdData $th06
    Gb_th_print $th06 "th06= "
    set e1 [Gb_dataMGD_e1_get $mgdData]
    set e2 [Gb_dataMGD_e2_get $mgdData]
    set e3 [Gb_dataMGD_e3_get $mgdData]
    Gb_MGI6rTh ::gt6a $th06 $e1 $e2 $e3 $q6 $mgdData $q6_n
    set qdiff [new_Gb_v6]
    set vq6 [new_Gb_v6]
    set vq6_n [new_Gb_v6]
    eval Gb_v6_set $vq6 [Gb_q6_get $q6]
    eval Gb_v6_set $vq6_n [Gb_q6_get $q6_n]
    Gb_v6_moins $vq6 $vq6_n $qdiff
    puts "[Gb_q6_get $q6_n] epsilon= [Gb_v6_module $qdiff]"
    if { [Gb_v6_module $qdiff] <= $epsilon } {
	return 1
    } else {
	return 0
    }
}

Gb_q6_set q6 0.1 0.2 0.3 0.4 0.5 0.6

test_MGD_MGDI $q6



proc pirand { } {
    return [ expr (rand() - 0.5) * 2. * 3.14159265358979323846 ]
}

proc test_MG { n {epsilon 1e-5} } {
    set u $n
    set q6 [new_Gb_q6]
    Gb_q6_set $q6 [pirand] [pirand] [pirand] [pirand] [pirand] [pirand]

    while { $u > 0 } {
	set u [expr $u -1]
	set i [expr $u % 6 + 1]
	Gb_q6_q[set i]_set $q6 [pirand]
	if { [test_MGD_MGDI $q6 $epsilon] != 1} {
	    puts [Gb_q6_get $q6]
	}
    }
}

proc mgd_test { n } {
    set u $n
    set q6 [Gb_q6 q6]
    Gb_q6_set q6 [pirand] [pirand] [pirand] [pirand] [pirand] [pirand]

    while { $u > 0 } {
	incr u -1
	set i [expr $u % 6 + 1]
	Gb_q6_q[set i]_set q6 [pirand]
	set res [test_MGD_MGDI q6]
	if { $res != 1 } {
	    error "mgd_test: test_MGD_MGDI [gb_q6_get q6]"
	}
    }
}

mgd_test 5

proc allmgdi { q1 q2 q3 q4 q5 q6 } {
    set q [Gb_q6 q]
    Gb_q6_set q $q1 $q2 $q3 $q4 $q5 $q6
    puts "allmgdi q= [Gb_q6_get q]"
    set th [Gb_th th]
    Gb_MGD6rTh ::gt6a $q ::mgdData $th
    puts "allmgdi th= [Gb_th_get th]"
    set qi [Gb_q6 qi]
    foreach e1 { 1 -1 } {
	foreach e2 { 1 -1 } {
	    foreach e3 { 1 -1 } {
		Gb_MGI6rTh ::gt6a th $e1 $e2 $e3 q6 ::mgdData qi
		puts "$e1\t$e2\t$e3\t[Gb_q6_get $qi]"
	    }
	}
    }
}

allmgdi $M_PI_2 0.2 0.3 0.4 0.5 0.6

proc test_MGD_6th { q1 q2 q3 q4 q5 q6 } {
    set q [Gb_q6 q]
    Gb_q6_set q $q1 $q2 $q3 $q4 $q5 $q6
    set th01 [Gb_th th01]
    set th02 [Gb_th th02]
    set th03 [Gb_th th03]
    set th04 [Gb_th th04]
    set th05 [Gb_th th05]
    set th06 [Gb_th th06]
    Gb_MGD6r_6Th ::gt6a q ::mgdData th01 th02 th03 th04 th05 th06
}



set Q6 [Gb_q6 Q6]

proc pa10MGD { q1 q2 q3 q4 q5 q6 } {
    Gb_q6_set Q6 [expr $::M_PI + $q1] \
	[expr $q2 + $::M_PI_2] [expr $q3 + $::M_PI_2] $q4 $q5 $q6
    Gb_MGD6rTh pa10 Q6 mgdData th06
    Gb_th_print th06
}

myTest pa10MGD 0 0.01 0   0 0 0


set pa10 [Gb_6rParameters pa10]
Gb_6rParameters_a2_set pa10 0.45
Gb_6rParameters_r4_set pa10 0.48
Gb_6rParameters_epsilon_set pa10 0.01

set q [Gb_q6 q]
Gb_q6_set q 0 0 0  0 0 0

set th [Gb_th th]

Gb_MGD6rTh pa10 $q ::mgdData $th

set jac [Gb_jac jac]

Gb_MDD6r pa10  ::mgdData th jac


proc gbJacPrint { jac } {
    set c1 [$jac cget -c1]
    set c2 [$jac cget -c2]
    set c3 [$jac cget -c3]
    set c4 [$jac cget -c4]
    set c5 [$jac cget -c5]
    set c6 [$jac cget -c6]
    puts [format "%13g %13g %13g %13g %13g %13g\n" \
	      [$c1 cget -x] [$c2 cget -x] [$c3 cget -x] [$c4 cget -x] [$c5 cget -x] [$c6 cget -x]]
    puts [format "%13g %13g %13g %13g %13g %13g\n" \
	      [$c1 cget -y] [$c2 cget -y] [$c3 cget -y] [$c4 cget -y] [$c5 cget -y] [$c6 cget -y]]
    puts [format "%13g %13g %13g %13g %13g %13g\n" \
	      [$c1 cget -z] [$c2 cget -z] [$c3 cget -z] [$c4 cget -z] [$c5 cget -z] [$c6 cget -z]]
    puts [format "%13g %13g %13g %13g %13g %13g\n" \
	      [$c1 cget -rx] [$c2 cget -rx] [$c3 cget -rx] [$c4 cget -rx] [$c5 cget -rx] [$c6 cget -rx]]
    puts [format "%13g %13g %13g %13g %13g %13g\n" \
	      [$c1 cget -ry] [$c2 cget -ry] [$c3 cget -ry] [$c4 cget -ry] [$c5 cget -ry] [$c6 cget -ry]]
    puts [format "%13g %13g %13g %13g %13g %13g\n" \
	      [$c1 cget -rz] [$c2 cget -rz] [$c3 cget -rz] [$c4 cget -rz] [$c5 cget -rz] [$c6 cget -rz]]
}

gbJacPrint jac

puts

Gb_th_print th

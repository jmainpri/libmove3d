#
# Copyright (c) 2003 LAAS/CNRS -- RIA --
# Daniel SIDOBRE -- fevrier 2003

package require gbM
package provide gbmModeles 0.0.0

proc Gb_q6_get { Q } {
    return "[$Q cget -q1] [$Q cget -q2] [$Q cget -q3] [$Q cget -q4] [$Q cget -q5] [$Q cget -q6]"
}

proc Gb_q6_print { q6 { linePrefix ""} } {
    puts "[set linePrefix][Gb_q6_get $q6]"
}

# foncion utilitaire pour Gb_q6_set qui accepte 6 parametres ou une liste
proc Gb_q6_set_u { Q q1 q2 q3 q4 q5 q6 } {
    $Q configure -q1 $q1 -q2 $q2 -q3 $q3 -q4 $q4 -q5 $q5 -q6 $q6
}

proc Gb_q6_set { Q q1 args } {
    eval Gb_q6_set_u $Q $q1 $args
}

set mgdData [Gb_dataMGD mgdData]

set gt6a [Gb_6rParameters gt6a]
Gb_6rParameters_a2_set gt6a 0.4
Gb_6rParameters_r4_set gt6a 0.4
Gb_6rParameters_epsilon_set gt6a 0.00001

set pa10 [Gb_6rParameters gt6a]
Gb_6rParameters_a2_set gt6a 45
Gb_6rParameters_r4_set gt6a 48
Gb_6rParameters_epsilon_set gt6a 0.01


proc Gb_jac_print { jac } {
    foreach ell { x y z rx ry rz } {
#	foreach elc { c1 c2 c3 c4 c5 c6 } {
#	    puts -nonewline "[[$jac cget -$elc] cget -$ell]\t"
#	}
#	puts ""
	puts [format "%f\t%f\t%f\t%f\t%f\t%f" \
		  [[$jac cget -c1] cget -$ell] \
		  [[$jac cget -c2] cget -$ell] \
		  [[$jac cget -c3] cget -$ell] \
		  [[$jac cget -c4] cget -$ell] \
		  [[$jac cget -c5] cget -$ell] \
		  [[$jac cget -c6] cget -$ell] ]
    }
}

proc pirand { } {
    return [ expr (rand() - 0.5) * 2. * 3.14159265358979323846 ]
}

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


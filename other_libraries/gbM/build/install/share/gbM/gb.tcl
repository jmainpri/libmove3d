#  gb.tcl
# Copyright (c) 2002 LAAS/CNRS -- RIA --
# Daniel SIDOBRE -- mai 2002
# 
package require gb
package provide gbM 0.0.0

set gb_tcl_oldpwd [pwd]
cd [file dirname [info script]]



proc gbload { } {
    if { [info exist ::GB_DONT_RELOAD] } {
	unset ::GB_DONT_RELOAD
    }
    uplevel \#0 {
	source gb.tcl
    } 
}

#if { [info exist GB_DONT_RELOAD] == 0 } {
#    puts "Loading gb"
#    load gb.so Gb
#    set GB_DONT_RELOAD 1
#}


# Pi
set M_PI		3.14159265358979323846
# Pi/2
set M_PI_2		1.57079632679489661923
# Pi/4
set M_PI_4		0.78539816339744830962

Gb_v3 VX
VX configure -x 1 -y 0 -z 0
Gb_v3 VY
VY configure -x 0 -y 1 -z 0
Gb_v3 VZ
VZ configure -x 0 -y 0 -z 1
Gb_v3 V0
V0 configure -x 0 -y 0 -z 0

proc Gb_v3_get { v } {
    return "[$v cget -x] [$v cget -y] [$v cget -z]"
}

proc Gb_v3_set { v3 x y z } {
    $v3 configure -x $x -y $y -z $z
}

proc Gb_v3_new { x y z } {
    set a [new_Gb_v3]
    $a configure -x $x -y $y -z $z
    return $a
}

proc Gb_v3_print { v3 { linePrefix ""} } {
    puts "[set linePrefix][Gb_v3_get $v3]"
}


proc Gb_v6_get { v } {
    return "[$v cget -x] [$v cget -y] [$v cget -z]  [$v cget -rx] [$v cget -ry] [$v cget -rz]"
}

proc Gb_v6_set { v x y z rx ry rz } {
    $v configure -x  $x
    $v configure -y  $y
    $v configure -z  $z
    $v configure -rx $rx
    $v configure -ry $ry
    $v configure -rz $rz
}

proc Gb_force_get { v } {
    return [Gb_v6_get $v]
}

proc Gb_force_set { args } {
    eval Gb_v6_set $args
}

proc Gb_vitesse_get { v } {
    return [Gb_v6_get $v]
}

proc Gb_vitesse_set { args } {
    eval Gb_v6_set $args
}


proc Gb_v6_print { v6 { linePrefix ""} } {
    puts "[set linePrefix][Gb_v6_get $v6]"
}

set  GbV6_0 [Gb_v6 GbV6_0]
Gb_v6_set GbV6_0    0 0 0   0 0 0


set ThId [Gb_th ThId]
ThId configure -vx VX -vy VY -vz VZ -vp V0

proc Gb_th_get { th } {
    return "[Gb_v3_get [$th cget -vx]] [Gb_v3_get [$th cget -vy]] [Gb_v3_get [$th cget -vz]] [Gb_v3_get [$th cget -vp]]"
}

proc Gb_th_set { th args } {
    if { $th == "" } { set th [new_Gb_th] }
    set dep [new_Gb_dep]
    set n [llength $args]
# corriger : tester type de 1    puts "Gb_th_set n= $n"
    if { $n == 6 } {
	set v [new_Gb_v3]
	eval Gb_v3_set [lrange $args 3 end]
	eval Gb_dep_set dep $args [Gb_v3_module v]
	Gb_dep_th dep $th
    } elseif { $n == 7 } {
	eval Gb_dep_set dep $args
	Gb_dep_th dep $th
    } elseif { $n == 12 } {
	eval Gb_v3_set [$th cget -vx] [lrange $args 0 2]
	eval Gb_v3_set [$th cget -vy] [lrange $args 3 5]
	eval Gb_v3_set [$th cget -vz] [lrange $args 6 8]
	eval Gb_v3_set [$th cget -vp] [lrange $args 9 11]
    } else { 
	error "Pb Gb_dep_th 6, 7 ou 12 valeurs"
    }
    return $th
}

	
proc Gb_th_print { th { lp ""} {quatre ""} } {
    regsub -all {[^ ]} $lp { } ls
    set vx [$th cget -vx]
    set vy [$th cget -vy]
    set vz [$th cget -vz]
    set vp [$th cget -vp]
    puts "[set lp][$vx cget -x]\t[$vy cget -x]\t[$vz cget -x]\t  [$vp cget -x]"
    puts "[set ls][$vx cget -y]\t[$vy cget -y]\t[$vz cget -y]\t  [$vp cget -y]"
    puts "[set ls][$vx cget -z]\t[$vy cget -z]\t[$vz cget -z]\t  [$vp cget -z]"
    if {$quatre != ""} {
	puts "[set ls]0.0\t0.0\t0.0\t1.0"
    }
}

proc Gb_dep_print { dep { linePrefix ""} } {
    puts "[set linePrefix][Gb_dep_get $dep]"
}


proc Gb_dep_get { dep } {
    return "[Gb_dep_x_get $dep] [Gb_dep_y_get $dep] [Gb_dep_z_get $dep]  [Gb_dep_rx_get $dep] [Gb_dep_ry_get $dep] [Gb_dep_rz_get $dep]  [Gb_dep_a_get $dep]"
}

proc Gb_dep_set { dep x y z rx ry rz a } {
    if { $dep == "" } { set dep [new_Gb_dep] }
    Gb_dep_x_set  $dep $x
    Gb_dep_y_set  $dep $y
    Gb_dep_z_set  $dep $z
    Gb_dep_rx_set $dep $rx
    Gb_dep_ry_set $dep $ry
    Gb_dep_rz_set $dep $rz
    Gb_dep_a_set  $dep $a 
    return $dep
}

proc Gb_quat_get { quat } {
    return "[Gb_quat_x_get $quat] [Gb_quat_y_get $quat] [Gb_quat_z_get $quat] [Gb_quat_vx_get $quat] [Gb_quat_vy_get $quat] [Gb_quat_vz_get $quat] [Gb_quat_w_get $quat]"
}

proc Gb_quat_set { quat x y z vx vy vz w } {
    if { $quat == "" } { set quat [new_Gb_quat] }
    Gb_quat_x_set  $quat $x
    Gb_quat_y_set  $quat $y
    Gb_quat_z_set  $quat $z
    Gb_quat_vx_set $quat $vx
    Gb_quat_vy_set $quat $vy
    Gb_quat_vz_set $quat $vz
    Gb_quat_w_set  $quat $w
    return quat
}

cd $gb_tcl_oldpwd
unset gb_tcl_oldpwd

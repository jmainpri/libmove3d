#  gbTest.tcl
# Copyright (c) 2002 LAAS/CNRS -- RIA --
# Daniel SIDOBRE -- avril 2005
# 

source gb.tcl

proc myTest { args } {
    uplevel 1  "puts \"\#\# $args\""
    puts [uplevel 1  eval $args ]
}


myTest Gb_v3_get V0
myTest Gb_v3_get VX
myTest Gb_v3_get VY
myTest Gb_v3_get VZ

myTest set gbV3_v [Gb_v3 gbV3_v]
myTest Gb_v3_set gbV3_v 2 4.3 6
myTest Gb_v3_get gbV3_v

myTest set a [Gb_v3_new 1 2 3]
myTest Gb_v3_get $a
myTest unset a

myTest Gb_v3_print VX


myTest Gb_v6_print GbV6_0

myTest set gbV6_v [Gb_v6 gbV6_v]
myTest Gb_v6_set gbV6_v 0.1 0.2 0.3  1.1 1.2 1.3
myTest Gb_v6_get gbV6_v

myTest set gbForce_v [Gb_force gbForce_v]
myTest Gb_force_set gbForce_v 1.1 1.2 1.3  1.1 1.2 1.3
myTest Gb_force_get gbForce_v


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


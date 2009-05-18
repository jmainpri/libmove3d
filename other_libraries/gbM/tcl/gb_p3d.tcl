source gb.tcl

#if { [info exist ::gbCurrentTPT ] } { unset ::gbCurrentTPT }

proc p3d_beg_desc { arg1 } { 
    if { [info exist ::gbCurrentTPT] } {
	error {Pb p3d_beg_desc ::gbCurrentTPT exist}
    }
    set ::gbCurrentTPT [gbTreePolyTriangleNewRacine]
}
proc p3d_add_desc_poly { name } { }
proc p3d_add_desc_vert { x y z } {
    set tpt [gbTreePolyTriangle_vertexAdd $::gbCurrentTPT $x $y $z]
    if { $tpt == "NULL" } {error "Pb p3d_add_desc_vert tpt is NULL"}
}
proc p3d_add_desc_face  { s1 s2 s3 } {
    set p1 [gbTreePolyTriangleGetVertexP $::gbCurrentTPT $s1]
    if { $p1 == "NULL" } { error "Pb p3d_add_desc_face p1 is NULL" }
    set p2 [gbTreePolyTriangleGetVertexP $::gbCurrentTPT $s2]
    if { $p2 == "NULL" } { error "Pb p3d_add_desc_face p2 is NULL" }
    set p3 [gbTreePolyTriangleGetVertexP $::gbCurrentTPT $s3]
    if { $p3 == "NULL" } { error "Pb p3d_add_desc_face p3 is NULL" }
    set edge1 [gbTPTEledgeGetNewS1S2 $::gbCurrentTPT $p1 $p2]
    if { $edge1 == "NULL" } { error "Pb p3d_add_desc_face edge1 is NULL" }
    set edge2 [gbTPTEledgeGetNewS1S2 $::gbCurrentTPT $p2 $p3]
    if { $edge2 == "NULL" } { error "Pb p3d_add_desc_face edge2 is NULL" }
    set edge3 [gbTPTEledgeGetNewS1S2 $::gbCurrentTPT $p3 $p1]
    if { $edge3 == "NULL" } { error "Pb p3d_add_desc_face edge3 is NULL" }
    set t [gbTreePolyTriangle_triangleAdd $::gbCurrentTPT \
	       $p1 $p2 $p3  $edge1 $edge2 $edge3 ]
    if { $t == "NULL" } { 
	set suite "\n  s1= $s1 s2= $s2 s3= $s3"
	error "Pb p3d_add_desc_face creation triangle $suite"
    }
}
proc p3d_end_desc_poly { } { 

}
proc p3d_end_desc { } {
    
}

# rename p3d_beg_desc {}
# rename p3d_add_desc_poly {}
# rename p3d_add_desc_vert {}
# rename p3d_add_desc_face {}
# rename p3d_end_desc_poly {}
# rename p3d_end_desc {}

#   source /home/edamian/Move3D/vrml-models/fused.p3d
#source tetraedre.p3d

#gbTreePolyTriangleWriteP3d $::gbCurrentTPT

#gbTreePolyTrianglePrint1 $::gbCurrentTPT 

proc gbPlan_new { px py pz vx vy vz } {
    set p [Gb_v3_new $px $py $pz]
    set v [Gb_v3_new $vx $vy $vz]
    Gb_v3_norme $v $v
    set ret [new_gbPlan]
    $ret configure -p $p -v $v
    return $ret
}

proc gbPlan_get { plan } {
    set p [$plan cget -p]
    set v [$plan cget -v]
    return "[$p cget -x] [$p cget -y] [$p cget -z]  [$v cget -x] [$v cget -y] [$v cget -z]"
}

#set plan [gbPlan_new 0.5 0.5 0.5  0 1 0]
#set u [expr sqrt(2) / 2.]
#set plan [gbPlan_new 0.5 0 0  $u -$u 0]
#puts "plan= [gbPlan_get $plan]"
#
#  gbTreePolyTriangleCut $::gbCurrentTPT $plan 1e-8
#  
#  gbTreePolyTrianglePrint1 $::gbCurrentTPT 

#  puts "\n------------------ left -------------\n"
#  gbTreePolyTrianglePrint1 [$::gbCurrentTPT  cget -left]
#  puts "\n------------------ right -------------\n"
#  gbTreePolyTrianglePrint1 [$::gbCurrentTPT  cget -right]


proc tt { }  {
 gbTreePolyTriangleCut $::gbCurrentTPT $::plan 1e-8
} 


#gbTreePolyTrianglePrint1 $::gbCurrentTPT

#gbTreePolyTrianglePrint1 [$::gbCurrentTPT  cget -left]

#gbTreePolyTriangleWriteP3d $::gbCurrentTPT
#gbTreePolyTriangleWriteP3d [$::gbCurrentTPT  cget -left]
#gbTreePolyTriangleWriteP3d [$::gbCurrentTPT  cget -right]


proc tte { epsilon } {
    if { [info exist ::gbCurrentTPT] } {
	gbTreePolyTriangleDelete $::gbCurrentTPT
	unset ::gbCurrentTPT
    }
    source tetraedre.p3d
#    source tetraedre3.p3d
#    gbTreePolyTrianglePrint1 $::gbCurrentTPT

    set u [expr sqrt(2) / 2.]
#    set ::plan [gbPlan_new 0.5 0 0  $u -$u 0]
    set ::plan [gbPlan_new 4.1 4 4  -1 0 0.45  ]
    gbTreePolyTriangleCut $::gbCurrentTPT $::plan 1e-8
    gbTreePolyTriangleWriteP3d [$::gbCurrentTPT  cget -left]
    gbTreePolyTriangleWriteP3d [$::gbCurrentTPT  cget -right]
#    gbTreePolyTrianglePrint1 $::gbCurrentTPT
}

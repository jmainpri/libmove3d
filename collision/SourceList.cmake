IF(P3D_COLLISION_CHECKING)
SET(BM3D_MODULE_NAME collision)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(
coltestcomp.c 
p3d_col.c 
p3d_col_env.c 
p3d_collision_context.c 
p3d_col_traj.c 
p3d_filter.c 
p3d_kcd.c 
p3d_triangles.c 
p3d_v_collide.c
p3d_pqp.c
)


include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/Kcd/SourceList.cmake)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/Vcollide/SourceList.cmake)
ENDIF(P3D_COLLISION_CHECKING)


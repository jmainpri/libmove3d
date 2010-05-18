IF(P3D_LOCALPATH)
SET(BM3D_MODULE_NAME localpath)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/include)
BM3D_SRC_SUBDIR_PROCESS(
p3d_hilare_flat.c 
p3d_linear.c 
p3d_local.c 
p3d_manhattan.c 
p3d_reeds_shepp.c 
p3d_trailer.c 
rs_curve.c 
rs_dist.c
)
IF(MULTILOCALPATH)
BM3D_SRC_SUBDIR_PROCESS(p3d_softMotion.c p3d_multiLocalPath.c)
ENDIF(MULTILOCALPATH)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/flat/SourceList.cmake)
ENDIF(P3D_LOCALPATH)
SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Kcd)
BM3D_SRC_SUBDIR_PROCESS(
kcd_aabb.c kcd_aabb_polyh.c kcd_aabb_tree.c kcd_api.c kcd_api_fcts.c kcd_api_report.c kcd_bb.c kcd_bb_init.c kcd_dist.c kcd_gjk.c kcd_gjk_debug.c kcd_gjk_support.c kcd_matrix.c kcd_obb_api_gjk.c kcd_obb_bb.c kcd_obb.c kcd_obb_overlap.c kcd_obb_polyh.c kcd_tables.c kcd_triangles.c
)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/include)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

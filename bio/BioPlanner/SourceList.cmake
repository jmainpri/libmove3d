SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/BioPlanner)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(bio_expand_lig_sc.c bio_ligandbased_rrt.c bio_localpath.c bio_planner_util.c)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/BioUtil)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(bio_interface_utilities.c move3d_pdb.c)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

IF(ENERGY)
SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/BioEnergy)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/include)
BM3D_SRC_SUBDIR_PROCESS(bio_2amber.c bio_allocations.c bio_coldeg_planner.c bioenergy_common.c bio_minimization.c bio_nmode.c bio_rrt_E_minimizer.c FORM_bio_energy.c)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
add_definitions(-DENERGY)
ENDIF(ENERGY)

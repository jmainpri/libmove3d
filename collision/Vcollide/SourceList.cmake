IF(VCOLLIDE)
SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/src/SourceList.cmake)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/RAPID/SourceList.cmake)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/include)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
add_definitions(-DVCOLLIDE_ACT)
ENDIF(VCOLLIDE)

SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/CppApi)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(

testModel.cpp
SaveContext.cpp

)

IF(QT_LIBRARY)
BM3D_SRC_SUBDIR_PROCESS(

MultiRun.cpp

)
ENDIF(QT_LIBRARY)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtMainInterface)
BM3D_SRC_SUBDIR_PROCESS(

kcdpropertieswindow.cpp 
mainwindow.cpp 
qdebugstream.cpp

)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_QT_GENERATE_MOC(kcdpropertieswindow.hpp mainwindow.hpp)
BM3D_QT_GENERATE_UI_HEADERS(kcdpropertieswindow.ui mainwindow.ui sidewindow.ui)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
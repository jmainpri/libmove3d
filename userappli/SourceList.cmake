SET(BM3D_MODULE_NAME userappli)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(FORMuser_appli.c user_appli.c)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/CppApi/SourceList.cmake)

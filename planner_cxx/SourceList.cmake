IF(CXX_PLANNER)
SET(BM3D_MODULE_NAME planner_cxx)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
planner.cpp 
plannerFunctions.cpp 
untitled.cpp 
cost_space.cpp
)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/API/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Diffusion/SourceList.cmake)
#include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Greedy/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/PRM/SourceList.cmake)
IF(HRI_COSTSPACE)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/HRI_CostSpace/SourceList.cmake)
ENDIF(HRI_COSTSPACE)
ENDIF(CXX_PLANNER)

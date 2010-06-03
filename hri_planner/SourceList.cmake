IF(HRI_GENERALIZED_IK)
SET(BM3D_MODULE_NAME hri_planner)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/include)
BM3D_SRC_SUBDIR_PROCESS(
hri_agent.c 
hri_gik.c 
)
ENDIF(HRI_GENERALIZED_IK)

IF(HRI_PLANNER)
BM3D_SRC_SUBDIR_PROCESS(
FORMgikjointselection.c 
FORMhri_planner.c 
FORMpsp_parameters.c 
hri_bitmap.c 
hri_visibility.c 
hri_wave_exp.c 
p3d_perspective.c
hri_manip.c 
)
IF(USE_MIGHTABILITY_MAPS)
BM3D_SRC_SUBDIR_PROCESS(Mightability_Maps.c FORM_HRI_affordance.c)
ENDIF(USE_MIGHTABILITY_MAPS)
IF(USE_HRP2_GIK)
BM3D_SRC_SUBDIR_PROCESS(HRP2_gik.cpp)
ENDIF(USE_HRP2_GIK)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/graphic/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/hri_bitmap/SourceList.cmake)
ENDIF(HRI_PLANNER)

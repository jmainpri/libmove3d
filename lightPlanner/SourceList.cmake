IF(LIGHT_PLANNER)
SET(BM3D_MODULE_NAME lightPlanner)
BM3D_SRC_SUBDIR_PROCESS(
lightPlannerApi.c lightPlanner.c robotPos.c DlrObject.cpp DlrParser.cpp DlrPlan.cpp DlrPlanner.cpp)
IF(GRASP_PLANNING)
  BM3D_SRC_SUBDIR_PROCESS(Manipulation.cpp)
IF(CXX_PLANNER) 
IF(MULTILOCALPATH)
  BM3D_SRC_SUBDIR_PROCESS(Manipulation_JIDO.cpp)
  BM3D_SRC_SUBDIR_PROCESS(ManipulationPlanner.cpp)
ENDIF(MULTILOCALPATH)
ENDIF(CXX_PLANNER)
ENDIF(GRASP_PLANNING)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
ENDIF(LIGHT_PLANNER)

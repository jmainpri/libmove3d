IF(GRASP_PLANNING)
SET(BM3D_MODULE_NAME graspPlanning)
BM3D_SRC_SUBDIR_PROCESS(
FORMgraspPlanning.c gp_extensionsM3D.c gp_force_closure.c gp_inertia_axes.c gp_volInt.c
gpConvexHull.cpp gp_geometry.cpp gpGrasp.cpp gp_grasp_generation.cpp gp_grasping_utils.cpp gp_grasp_io.cpp gpKdTree.cpp gpPose.cpp gpWorkspace.cpp
)
include_directories (${BM3D_MODULE_NAME}/proto)
include_directories (${BM3D_MODULE_NAME}/include)
ENDIF(GRASP_PLANNING)

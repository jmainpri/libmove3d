IF(GRASP_PLANNING)
SET(BM3D_MODULE_NAME graspPlanning)
BM3D_SRC_SUBDIR_PROCESS(
gp_extensionsM3D.c 
gp_force_closure.c 
gp_inertia_axes.c 
gp_volInt.c
gpConvexHull.cpp 
gp_geometry.cpp 
gpContact.cpp
gpGrasp.cpp 
gp_grasp_generation.cpp 
gp_grasping_utils.cpp 
gp_grasp_io.cpp 
gpKdTree.cpp 
gpPlacement.cpp 
gpWorkspace.cpp
qhull/rboxlib.c
qhull/user.c
qhull/global.c
qhull/stat.c
qhull/io.c
qhull/geom2.c
qhull/poly2.c
qhull/merge.c
qhull/libqhull.c
qhull/geom.c
qhull/poly.c
qhull/qset.c
qhull/mem.c
qhull/usermem.c
qhull/userprintf.c
qhull/random.c
)

IF(WITH_XFORMS)
BM3D_SRC_SUBDIR_PROCESS(
FORMgraspPlanning.c 
)
ENDIF(WITH_XFORMS)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/include)
ENDIF(GRASP_PLANNING)

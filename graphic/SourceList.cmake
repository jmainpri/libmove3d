SET(BM3D_MODULE_NAME graphic)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(
g3d_draw.c 
g3d_draw_env.c 
g3d_draw_graph.c 
g3d_draw_traj.c 
g3d_states.c
g3d_draw_cost.cpp
)

IF(WITH_XFORMS)
BM3D_SRC_SUBDIR_PROCESS(
g3d_draw_ui.c 
g3d_window.c
)
ENDIF(WITH_XFORMS)

IF(P3D_COLLISION_CHECKING)
BM3D_SRC_SUBDIR_PROCESS(
g3d_kcd_draw.c 
)
ENDIF(P3D_COLLISION_CHECKING)

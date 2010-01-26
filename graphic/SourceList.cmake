SET(BM3D_MODULE_NAME graphic)
include_directories (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(g3d_draw.c g3d_draw_env.c g3d_draw_graph.c g3d_draw_traj.c g3d_draw_ui.c g3d_kcd_draw.c g3d_window.c)

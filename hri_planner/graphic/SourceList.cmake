SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/graphic)
BM3D_SRC_SUBDIR_PROCESS(g3d_draw_camera.c g3d_draw_navigation.c g3d_perspective_window.c g3d_position_area.c)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/hri_bitmap)
BM3D_SRC_SUBDIR_PROCESS(hri_bitmap_bin_heap.c hri_bitmap_cost.c hri_bitmap_draw.c hri_bitmap_util.c)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

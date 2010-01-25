SET(BM3D_MODULE_NAME util)
include_directories (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(basic_alloc.c dyna_list.c ebt.c gnuplot.c p3d_angle.c stat.c string_util.c time.c UdpClient.cpp)


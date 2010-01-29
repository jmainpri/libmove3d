SET(BM3D_MODULE_NAME util)
BM3D_INC_DIR_PROCESS(${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(

basic_alloc.c 
dyna_list.c 
ebt.c 
gnuplot.c 
p3d_angle.c 
stat.c 
string_util.c 
time.c 
UdpClient.cpp
dummyFunctions.cpp

)

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/CppApi/SourceList.cmake)
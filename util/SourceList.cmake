SET(BM3D_MODULE_NAME util)
BM3D_INC_DIR_PROCESS(${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(

basic_alloc.c 
dyna_list.c 
ebt.c 
gnuplot.c 
p3d_angle.c 
string_util.c 
time.c 
UdpClient.cpp

)

IF(P3D_PLANNER)
BM3D_SRC_SUBDIR_PROCESS(

stat.c 

)
ENDIF(P3D_PLANNER)

IF(NOT BIO)
BM3D_SRC_SUBDIR_PROCESS(

dummyFunctions.cpp

)
ENDIF(NOT BIO)

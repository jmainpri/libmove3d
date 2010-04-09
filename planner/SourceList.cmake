IF(P3D_PLANNER)
SET(BM3D_MODULE_NAME planner)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
p3d_elastic.c 
p3d_graph_api.c 
p3d_graph.c 
p3d_graph_in_grid.c 
p3d_graph_quality.c 
p3d_graph_utils.c 
p3d_hriCost.c 
p3d_NodeAndCompTools.c 
p3d_NodeWeight.c 
p3d_optim.c 
p3d_potential.c 
p3d_sample.c 
p3d_SelectedDistConfig.c 
p3d_set_param.c 
p3d_trajectory.c 
MTRand.cpp
)
IF(MULTIGRAPH)
BM3D_SRC_SUBDIR_PROCESS(p3d_multiGraph.c)
ENDIF(MULTIGRAPH)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/astar/SourceList.cmake)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/dfs/SourceList.cmake)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/Diffusion/SourceList.cmake)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/dpg/SourceList.cmake)
include(${BioMove3D_SOURCE_DIR}/${BM3D_MODULE_NAME}/rwGraph/SourceList.cmake)
ENDIF(P3D_PLANNER)


SET(BM3D_MODULE_NAME graphic)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(
g3d_draw.c 
g3d_draw_env.c 
g3d_draw_graph.c 
g3d_draw_traj.c 
g3d_states.c
uglyfont.cpp
)

IF(USE_SHADERS)
BM3D_SRC_SUBDIR_PROCESS(
g3d_extensions.c
)
ENDIF(USE_SHADERS)

IF (NOT WITH_XFORMS) #---------------------------------

BM3D_SRC_SUBDIR_PROCESS(
g3d_newWindow.cpp
)
IF(QT_GL_WIDGET)
  BM3D_SRC_SUBDIR_PROCESS(
  glwidget.cpp 
  )
  BM3D_QT_GENERATE_MOC(proto/glwidget.hpp)
ENDIF()

IF(USE_GLUT AND NOT QT_GL_WIDGET)
  BM3D_SRC_SUBDIR_PROCESS(
  g3d_glut.cpp 
  )
ENDIF()

ENDIF() 

IF(P3D_COLLISION_CHECKING)
BM3D_SRC_SUBDIR_PROCESS(
g3d_kcd_draw.c 
)
ENDIF(P3D_COLLISION_CHECKING)

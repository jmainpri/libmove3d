SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtMainInterface)

BM3D_SRC_SUBDIR_PROCESS(
kcdpropertieswindow.cpp 
mainwindow.cpp
mainwindowTestFunctions.cpp
qtMotionPlanner.cpp
qtCost.cpp
qtRobot.cpp
qtUtil.cpp
qdebugstream.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
kcdpropertieswindow.hpp 
mainwindow.hpp
mainwindowTestFunctions.hpp
qtMotionPlanner.hpp
qtCost.hpp
qtRobot.hpp
qtUtil.hpp
)

BM3D_QT_GENERATE_UI_HEADERS(
kcdpropertieswindow.ui 
mainwindow.ui 
qtMotionPlanner.ui
qtCost.ui
qtRobot.ui
qtUtil.ui
)

IF(HRI_COSTSPACE)
BM3D_QT_GENERATE_MOC(
qtHrics.hpp
)
BM3D_QT_GENERATE_UI_HEADERS(
qtHrics.ui
)
BM3D_SRC_SUBDIR_PROCESS(
qtHrics.cpp
)
ENDIF(HRI_COSTSPACE)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
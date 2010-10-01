SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/sideWidgets)

BM3D_SRC_SUBDIR_PROCESS(
qtCost.cpp
qtMotionPlanner.cpp
qtUtil.cpp
qtRobot.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
qtCost.hpp
qtMotionPlanner.hpp
qtUtil.hpp
qtRobot.hpp
)

BM3D_QT_GENERATE_UI_HEADERS(
qtCost.ui
qtMotionPlanner.ui
qtUtil.ui
qtRobot.ui
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

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})

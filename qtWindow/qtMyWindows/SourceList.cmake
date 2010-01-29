SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtMyWindows)
BM3D_SRC_SUBDIR_PROCESS(

qtMainWindow.cpp 
qtColisionTestWindow.cpp 
qtDiffusionWindow.cpp 
qtGreedyWindow.cpp 
qtHriWindow.cpp 
qtOptimWindow.cpp 
qtPRMWindow.cpp 
qtTestWindow.cpp 
qtVisuWindow.cpp

)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_QT_GENERATE_MOC(qtMainWindow.hpp qtColisionTestWindow.hpp qtDiffusionWindow.hpp qtGreedyWindow.hpp qtHriWindow.hpp qtOptimWindow.hpp qtPRMWindow.hpp qtTestWindow.hpp qtVisuWindow.hpp)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})

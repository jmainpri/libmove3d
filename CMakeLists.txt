CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (BioMove3D C CXX)

# -------------------------------------------------------------
# --- Default compilation option : ----------------------------
# --- DO NOT MODIFY!!! use ccmake -----------------------------
# -------------------------------------------------------------
SET(BM3D_CompilationFlags "")

MACRO(BM3D_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND BM3D_CompilationFlags ${flagName})
ENDMACRO(BM3D_flag)

MACRO(BM3D_CheckCompilationFlags)
  MESSAGE ("Active Flags :")
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/move3DFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/move3DFlags "MOVE3D_COMPILATION_FLAGS = ")
  FOREACH(flag ${BM3D_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/move3DFlags "-D${flag} ")
      MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
    ENDIF()
  ENDFOREACH(flag)
ENDMACRO(BM3D_CheckCompilationFlags)


BM3D_flag(PLANAR_SHADOWS "Compilation avec planar shadow (pour avoir les ombres)" ON)
BM3D_flag(ANIMATION "Compilation avec le module personnage" OFF)

BM3D_flag(WITH_XFORMS "Compile the Xform interface" ON)
BM3D_flag(BIO "Compilation avec le module Bio" ON)
BM3D_flag(ENERGY "Compilation avec le module Bio-Energy" OFF)

BM3D_flag(KCD_CP_IS_SOLID "Convex polyhedron as a solid instead of as a collection of convex facets in KCD" OFF)
BM3D_flag(USE_P3D_BODY_AABB "test with AABB around body of robot" ON)
BM3D_flag(KCD_APPROX_ROBOT "KCD with volume" OFF)
BM3D_flag(GJK "Use gjk (just for debugging" OFF)
BM3D_flag(PQP "To compile with PQP (Proximity Query Package) collision library" OFF)
BM3D_flag(OBB_EVAL "write a file with OBB tree information (just for debuggingn" OFF)
BM3D_flag(VCOLLIDE "Use vCollide Collision detector" OFF)

BM3D_flag(MULTIGRAPH "Compilation avec le module Multi-Graph" OFF)
BM3D_flag(LIGHT_PLANNER "Extern Mode (Use this mode to have a scriped functions for planning pick and place tasks)" OFF)
BM3D_flag(DPG "Dynamic planning using grids" OFF)

BM3D_flag(HRI_PLANNER "Human Robot Interaction planner module for Move3D" OFF)
BM3D_flag(HRI_COSTSPACE "Human Robot Interaction planner module for Move3D" OFF)

BM3D_flag(MULTILOCALPATH "Compilation avec le module Multi-Localpath" OFF)
BM3D_flag(GRASP_PLANNING "Module de planification de prises" OFF)
BM3D_flag(FK_CNTRT "to use the automatic generation of forward kinematics constraints (reciprocal of the closed chained constraints)" OFF)

BM3D_flag(CXX_PLANNER "Compiles the planning API and planners" OFF)
BM3D_flag(USE_CXX_PLANNER "Sets the planning API to be default in Move3D" OFF)
BM3D_flag(LIST_OF_PLANNERS "Sets the planning API to be default in Move3D" OFF)

BM3D_flag(QT_LIBRARY "Sets QtLibrary compiling mode" OFF)
BM3D_flag(QT_GL "Sets QtLibrary compiling mode" OFF)
BM3D_flag(QT_OPENGL_SIDE "Sets QtLibrary compiling mode" OFF)
BM3D_flag(QT_UI_XML_FILES "Sets QtLibrary compiling mode" OFF)
BM3D_flag(QWT "Sets QtLibrary compiling mode" OFF)

BM3D_flag(USE_GSL "Mathematic Library" OFF)
BM3D_flag(USE_CLAPACK "Mathematic Library" OFF)
BM3D_CheckCompilationFlags()

#################################################################
#### Add here a library to include (custom FindPackage.cmake ####
#### are in BioMove3D/.cmakeScripts. Use the template to     ####
#### create a custom one)                                    ####
#################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")
IF(QT_LIBRARY)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(MOVE3D_COMPILATION_INCLUDES ${MOVE3D_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})
  IF(QT_GL)
    SET(QT_USE_QTOPENGL TRUE)
  ENDIF(QT_GL)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})
ENDIF(QT_LIBRARY)
FIND_PACKAGE(X11 REQUIRED)
INCLUDE_directories(${X11_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${X11_INCLUDE_DIR})
SET(LIBS ${LIBS} ${X11_LIBRARIES})
FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})
IF(APPLE)
  FIND_PACKAGE(GLU REQUIRED)
  INCLUDE_directories(${GLU_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLU_LIBRARIES})
  FIND_PACKAGE(GL REQUIRED)
  INCLUDE_directories(${GL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GL_LIBRARIES})
ELSE(APPLE)
  FIND_PACKAGE(OpenGL REQUIRED)
  INCLUDE_directories(${OPENGL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
ENDIF(APPLE)

FIND_PACKAGE(XPM REQUIRED)
SET(LIBS ${LIBS} ${XPM_LIBRARIES})
FIND_PACKAGE(XFORMS REQUIRED)
INCLUDE_directories(${XFORMS_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${XFORMS_INCLUDE_DIR})
SET(LIBS ${LIBS} ${XFORMS_LIBRARIES})
FIND_PACKAGE(GBM REQUIRED)
INCLUDE_directories(${GBM_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
SET(LIBS ${LIBS} ${GBM_LIBRARIES})
IF(USE_GSL)
  FIND_PACKAGE(GSL REQUIRED)
  INCLUDE_directories(${GSL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GSL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GSL_LIBRARIES})
ENDIF(USE_GSL)
IF(GRASP_PLANNING)
  FIND_PACKAGE(GLPK REQUIRED)
  INCLUDE_directories(${GLPK_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLPK_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLPK_LIBRARIES})
  FIND_PACKAGE(QHULL REQUIRED)
  INCLUDE_directories(${QHULL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QHULL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QHULL_LIBRARIES})
ENDIF(GRASP_PLANNING)
IF(MULTILOCALPATH)
  FIND_PACKAGE(SOFTMOTION REQUIRED)
  INCLUDE_directories(${SOFTMOTION_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${SOFTMOTION_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${SOFTMOTION_LIBRARIES})
ENDIF(MULTILOCALPATH)
IF(QWT)
  FIND_PACKAGE(QWT REQUIRED)
  INCLUDE_directories(${QWT_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QWT_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QWT_LIBRARIES})
ENDIF(QWT)

#Template :
# FIND_PACKAGE(<Package> REQUIRED)
# INCLUDE_directories(${<PACKAGE>_INCLUDE_DIR})
# SET(LIBS ${LIBS} ${<PACKAGE>_LIBRARIES})

STRING(REPLACE ";" " -I" MOVE3D_COMPILATION_INCLUDES "${INCLUDE_DIRS}")
FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/move3DFlags "\nMOVE3D_COMPILATION_INCLUDES = -I${MOVE3D_COMPILATION_INCLUDES}")

# -------------------------------------------------------------
# --- SUBDIR PROCESS ------------------------------------------
# -------------------------------------------------------------
SET(BM3D_src "")
SET(BM3D_inc "")
SET(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

#####################################################################
#### Add here a src directory containing a SourceList.cmake file ####
#####################################################################
SET(BM3D_SOURCE_DIRECTORIES animation bio collision graphic graspPlanning groundHeight hri_planner lightPlanner localpath move3d p3d planner planner_cxx qtWindow userappli util)

MACRO(BM3D_SRC_SUBDIR_PROCESS)
  FOREACH(srcfile ${ARGV})
    LIST(APPEND BM3D_src "${BM3D_MODULE_NAME}/${srcfile}")
    SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${BM3D_MODULE_NAME}/${srcfile})
  ENDFOREACH(srcfile)
ENDMACRO(BM3D_SRC_SUBDIR_PROCESS)
MACRO(BM3D_INC_DIR_PROCESS directory)
  include_directories (${directory})
  file (GLOB incfiles "${directory}/*.h")
  LIST(APPEND BM3D_inc "${incfiles}")
  SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${incfiles})
  file (GLOB incfiles "${directory}/*.hpp")
  LIST(APPEND BM3D_inc "${incfiles}")
  SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${incfiles})
ENDMACRO(BM3D_INC_DIR_PROCESS)

# -------------------------------------------------------------
# --- QT4 SUBDIR PROCESS (same but for preprocessing qt4 files)
# -------------------------------------------------------------
SET(BM3D_QT_FILES_FOR_MOC "")
MACRO(BM3D_QT_GENERATE_MOC)
  FOREACH(file ${ARGV})
    LIST(APPEND BM3D_QT_FILES_FOR_MOC "${BM3D_MODULE_NAME}/${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(BM3D_QT_GENERATE_MOC)

SET(BM3D_QT_UI_FILES "")
MACRO(BM3D_QT_GENERATE_UI_HEADERS)
  FOREACH(file ${ARGV})
    LIST(APPEND BM3D_QT_UI_FILES "${BM3D_MODULE_NAME}/${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(BM3D_QT_GENERATE_UI_HEADERS)

FOREACH(srcDir ${BM3D_SOURCE_DIRECTORIES})
  INCLUDE(${BioMove3D_SOURCE_DIR}/${srcDir}/SourceList.cmake)
ENDFOREACH(srcDir)

#MESSAGE("bm3dsrc : ${BM3D_src}") #Uncomment to print the included file by Cmake

# -------------------------------------------------------------
# --- Directories to include ----------------------------------
# -------------------------------------------------------------
BM3D_INC_DIR_PROCESS(${BioMove3D_SOURCE_DIR}/include)
BM3D_INC_DIR_PROCESS(${BioMove3D_SOURCE_DIR})
IF(QT_LIBRARY)
  # necessary for the UI headers
  BM3D_INC_DIR_PROCESS(${CMAKE_CURRENT_BINARY_DIR})
  # generate the UI headers
  QT4_WRAP_UI(BM3D_QT_UI_HEADERS ${BM3D_QT_UI_FILES})
  # Qt4 files that need to be preprocessed
  QT4_WRAP_CPP(BM3D_QT_MOC ${BM3D_QT_FILES_FOR_MOC})
ENDIF(QT_LIBRARY)

# -------------------------------------------------------------
# --- Compiler and Compilation flags --------------------------
# -------------------------------------------------------------
SET(CMAKE_CXX_COMPILER g++)
SET(CMAKE_C_COMPILER g++)

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX -x c++)
    SET (CMAKE_SHARED_LINKER_FLAGS "-dynamiclib -Wl,-headerpad_max_install_names,-undefined,dynamic_lookup")
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(EXECUTABLE_OUTPUT_PATH "${BioMove3D_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin/$ENV{HOSTTYPE}")
SET(LIBRARY_OUTPUT_PATH "${BioMove3D_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/$ENV{HOSTTYPE}")

#Default Flags
ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")

ADD_LIBRARY(m3d EXCLUDE_FROM_ALL SHARED ${BM3D_src} ${BM3D_QT_MOC} ${BM3D_QT_UI_HEADERS} ${BM3D_inc})
TARGET_LINK_LIBRARIES(m3d ${LIBS} m)

ADD_EXECUTABLE(move3d ${BM3D_src} ${BM3D_QT_MOC} ${BM3D_QT_UI_HEADERS} ${BM3D_inc})
TARGET_LINK_LIBRARIES(move3d ${LIBS} m)
# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${BioMove3D_BINARY_DIR}/install" CACHE PATH "BioMove3D install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
INSTALL(TARGETS move3d
        RUNTIME DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})

IF(EXISTS ${BioMove3D_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/libm3d.so)
INSTALL(TARGETS m3d
        RUNTIME DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})
ENDIF(EXISTS ${BioMove3D_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/libm3d.so)
INSTALL(DIRECTORY ${BM3D_SOURCE_DIRECTORIES} ${BioMove3D_SOURCE_DIR}/include DESTINATION include/BioMove3D FILES_MATCHING REGEX "\\.[h|hpp]")

# -------------------------------------------------------------
# --- Uninstall -----------------------------------------------
# -------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/script/CMake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

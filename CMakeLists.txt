CMAKE_MINIMUM_REQUIRED (VERSION 2.6.2)

PROJECT (libmove3d C CXX)

MESSAGE("test git post-update")

#--------- Set the project version

if( NOT BIOMOVE3D_VERSION)

set( CPACK_PACKAGE_VERSION_MAJOR 3)
set( CPACK_PACKAGE_VERSION_MINOR 18)
set( CPACK_PACKAGE_VERSION_PATCH 1)

set( LIBMOVE3D_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")

endif( NOT BIOMOVE3D_VERSION)

message( STATUS "Current project version is ${BIOMOVE3D_VERSION}")

# Define the project version

if( NOT NON_NATIVE_TARGET)
add_definitions( -DBIOMOVE3D_VERSION="${BIOMOVE3D_VERSION}")
endif( NOT NON_NATIVE_TARGET)

#--------- 

# -------------------------------------------------------------
# --- Default compilation option : ----------------------------
# --- DO NOT MODIFY!!! use ccmake -----------------------------
# -------------------------------------------------------------
SET(BM3D_CompilationFlags "")
SET(BM3D_LinkedCompilationFlags "")
SET(BM3D_ActiveFlags "")

MACRO(BM3D_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND BM3D_CompilationFlags ${flagName})
  FOREACH(linkFlag ${ARGN})
    LIST(APPEND BM3D_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(BM3D_flag)

MACRO(BM3D_ProcessCompilationFlags)

  FOREACH(flag ${BM3D_CompilationFlags})
	IF(${flag})
		LIST(APPEND BM3D_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)
  
  SET(goToNextLevel FALSE) # when TRUE, means that we must recursively call BM3D_ProcessFlags to process flags dependencies 
  FOREACH(activeFlag ${BM3D_ActiveFlags})
    FOREACH(linkFlag ${BM3D_LinkedCompilationFlags})
		IF(${linkFlag} MATCHES "${activeFlag}:(.*)")
			STRING(LENGTH "${activeFlag}:" prefixLength)
			STRING(LENGTH ${linkFlag} linkLength)
			MATH(EXPR linkLength "${linkLength} - ${prefixLength}")
			STRING(SUBSTRING ${linkFlag} ${prefixLength} ${linkLength} linkFlagToActivate)

			SET(isNewFlag TRUE)

			FOREACH(activeFlag2 ${BM3D_ActiveFlags})
				IF(${activeFlag2} STREQUAL ${linkFlagToActivate})
					SET(isNewFlag FALSE)
				ENDIF()
			ENDFOREACH(activeFlag2)

			IF(isNewFlag)
				SET(goToNextLevel TRUE)
				SET(${linkFlagToActivate} ON CACHE BOOL "${linkFlagToActivate} value" FORCE) #Need to force cache update
				LIST(APPEND BM3D_ActiveFlags ${linkFlagToActivate})
				MESSAGE("Enabled ${activeFlag} dependency: ${linkFlagToActivate}")
			ENDIF()
        ENDIF()

    ENDFOREACH(linkFlag)
  ENDFOREACH(activeFlag)

  IF(goToNextLevel)
	  BM3D_ProcessCompilationFlags()
  ENDIF()

ENDMACRO(BM3D_ProcessCompilationFlags)

MACRO(BM3D_SetCompilationFlags)

  MESSAGE ("Active Flags :")

  LIST(REMOVE_DUPLICATES BM3D_ActiveFlags)

  FOREACH(flag ${BM3D_ActiveFlags})
    SET(MOVE3D_COMPILATION_FLAGS ${MOVE3D_COMPILATION_FLAGS} ${flag})
    MESSAGE("${flag}")

    IF( QT_LIBRARY )
	ADD_DEFINITIONS("-D${flag}")
    ELSE( QT_LIBRARY )
	ADD_DEFINITIONS(" -D${flag}")
    ENDIF( QT_LIBRARY )

  ENDFOREACH(flag)
 
ENDMACRO(BM3D_SetCompilationFlags)

#-------------- Qt Library -----------------------------------------------------
FIND_PACKAGE(Qt4) #Qt4 is not required. If not found, set QT_LIBRARY to OFF.

SET(GL_NATIVE TRUE)

IF(QT_INCLUDE_DIR)
  MESSAGE("I've found Qt4 libraries. I will compile libmove3d with it.")
#  SET(QT_LIBRARY ON CACHE BOOL "" FORCE)
#  SET(QT_GL ON CACHE BOOL "" FORCE)
#  SET(QT_GL_WIDGET OFF CACHE BOOL "" FORCE)

  BM3D_flag(QT_LIBRARY "Sets QtLibrary compiling mode" ON)
  BM3D_flag(QT_GL "Sets QtLibrary compiling mode" ON QT_LIBRARY)

  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS}${QT_INCLUDE_DIR}) 
  SET(MOVE3D_COMPILATION_INCLUDES ${MOVE3D_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR}) 
  SET(QT_USE_QTOPENGL TRUE)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})

ELSE()
  MESSAGE("I've not found Qt4 libraries. I won't compile libmove3d with support for Qt.")
  SET(QT_LIBRARY OFF CACHE BOOL "" FORCE)
  SET(QT_GL OFF CACHE BOOL "" FORCE)
  SET(QT_GL_WIDGET OFF CACHE BOOL "" FORCE)
ENDIF()

# P3D core ---
BM3D_flag(P3D_CONSTRAINTS "Compilation with module Constraints" ON)
BM3D_flag(P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
BM3D_flag(P3D_LOCALPATH "Compilation with module Localpath" ON)
BM3D_flag(P3D_PLANNER "Compilation with module Planner" ON)

# Collision checking --
BM3D_flag(KCD_CP_IS_SOLID "Convex polyhedron as a solid instead of as a collection of convex facets in KCD" OFF)
BM3D_flag(USE_P3D_BODY_AABB "test with AABB around body of robot" ON)
BM3D_flag(KCD_APPROX_ROBOT "KCD with volume" OFF)
BM3D_flag(GJK "Use gjk (just for debugging)" OFF)
BM3D_flag(OBB_EVAL "write a file with OBB tree information (just for debugging)" OFF)
BM3D_flag(VCOLLIDE "Use vCollide Collision detector" OFF)

# Bio-related ---
BM3D_flag(BIO "Compilation with module Bio" OFF)
BM3D_flag(ENERGY "Compilation with module Bio-Energy" OFF)

# Path planning --
BM3D_flag(MULTIGRAPH "Compilation with module Multi-Graph" OFF P3D_PLANNER)
BM3D_flag(LIGHT_PLANNER "Extern Mode (Use this mode to have a scriped functions for planning pick and place tasks)"  OFF P3D_PLANNER)
BM3D_flag(DPG "Dynamic planning using grids" OFF CXX_PLANNER)

BM3D_flag(MULTILOCALPATH "Compilation avec le module Multi-Localpath" OFF USE_GBM)
BM3D_flag(GRASP_PLANNING "Grasp planning module" OFF USE_GBM)

BM3D_flag(FK_CNTRT "to use the automatic generation of forward kinematics constraints (reciprocal of the closed chained constraints)" OFF LIGHT_PLANNER)

BM3D_flag(USE_CXX_PLANNER "Sets the planning API to be default in Move3D" OFF)
BM3D_flag(LIST_OF_PLANNERS "Sets the planning API to be default in Move3D" OFF)

## User Interfaces ---
#BM3D_flag(QT_LIBRARY "Sets QtLibrary compiling mode" OFF)
#BM3D_flag(QT_GL "Sets QtLibrary compiling mode" OFF QT_LIBRARY)
#BM3D_flag(QT_GL_WIDGET "Compile with the OpenGl widget" OFF QT_GL QT_LIBRARY)

BM3D_flag(USE_GLUT "Compilation with a glut window" ON)

BM3D_flag(WITH_XFORMS "Compile the Xform interface" OFF)

BM3D_flag(ENABLE_ANTIALIASING "Enables antialiasing to draw smooth lines" OFF)
BM3D_flag(USE_SHADERS "Enables use of OpenGL shaders" OFF)

BM3D_flag(MOVE3D_CORE "Enables compilation of Move3D-core" OFF)

# Utility libraries ---
BM3D_flag(USE_GSL "Mathematic Library" OFF)
BM3D_flag(USE_CLAPACK "Mathematic Library" OFF)

# Other ---
BM3D_flag(ANIMATION "Compilation with the module character" OFF)
BM3D_flag(USE_HRP2_GIK "HRP2 GIK Library" OFF)
BM3D_flag(USE_HATP_LIBS "For including HATP libraries" OFF)
BM3D_flag(USE_GBM "Compilation with the gbM package" OFF)

BM3D_flag(USE_ROBOTMODELPARSER "Compilation with RobotModelParser" OFF)


# META flags ------
#------------------
BM3D_flag(GENERATE_LIBCORE "Set required options to compile the core libmove3d, without user interface" OFF)


BM3D_flag(MOVE3D_ROBOTICS "Set all the standard options related to robotics" OFF GRASP_PLANNING LIGHT_PLANNER MULTIGRAPH MULTILOCALPATH USE_GSL )

# TODO : implement a proper search for the GSL and/or Lapack libraries, and add the relevant flags (USE_GSL and USE_CLAPACK)
# It is necessary to link against either GSL or Lapack for protein loop sampling.
BM3D_flag(MOVE3D_BIO "Set all the standard options related to bio motion planning" OFF BIO WITH_XFORMS)
BM3D_flag(DEBUG_STATUS "Writting for debugs" ON)

BM3D_flag(GENERATE_LIBMOVE3D "Set the required options to compile the stand-alone move3d viewer" ON MOVE3D_ROBOTICS MOVE3D_CORE USE_GLUT)

BM3D_ProcessCompilationFlags()
BM3D_SetCompilationFlags()

#################################################################
#### Add here a library to include (custom FindPackage.cmake ####
#### are in BioMove3D/.cmakeScripts. Use the template to     ####
#### create a custom one)                                    ####
#################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")

# Add a "doc" target that runs Doxygen and install the doc.
#INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

#-------------- Qt Library --------------------------------
#----------------------------------------------------------

FIND_PACKAGE(X11 REQUIRED)
INCLUDE_directories(${X11_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${X11_INCLUDE_DIR})
SET(LIBS ${LIBS} ${X11_LIBRARIES})

FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

IF(APPLE)
# FIND_PACKAGE(GLU REQUIRED)
#  INCLUDE_directories(${GLU_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GLU_LIBRARIES})
	IF(GL_NATIVE)
  		FIND_PACKAGE(OpenGL REQUIRED)
  		INCLUDE_directories("${OPENGL_INCLUDE_DIR}/Headers")
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} "${OPENGL_INCLUDE_DIR}/Headers")
  		SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
  		Message(${OPENGL_INCLUDE_DIR})
  		Message(${OPENGL_LIBRARIES})
  		IF(WITH_XFORMS)
  			FIND_PACKAGE(GLU REQUIRED)
  			INCLUDE_directories(${GLU_INCLUDE_DIR})
  			SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  			SET(LIBS ${LIBS} ${GLU_LIBRARIES})
  			FIND_PACKAGE(GL REQUIRED)
  			INCLUDE_directories(${GL_INCLUDE_DIR})
  			SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
  			SET(LIBS ${LIBS} ${GL_LIBRARIES})
  		ENDIF(WITH_XFORMS)
	ELSE(GL_NATIVE)
		FIND_PACKAGE(GLU REQUIRED)
  		INCLUDE_directories(${GLU_INCLUDE_DIR})
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  		SET(LIBS ${LIBS} ${GLU_LIBRARIES})
  		FIND_PACKAGE(GL REQUIRED)
  		INCLUDE_directories(${GL_INCLUDE_DIR})
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
  		SET(LIBS ${LIBS} ${GL_LIBRARIES})
	ENDIF(GL_NATIVE)
ELSE(APPLE)
  FIND_PACKAGE(OpenGL REQUIRED)
  INCLUDE_directories(${OPENGL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})

  FIND_PACKAGE(GLU REQUIRED)
  INCLUDE_directories(${GLU_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLU_LIBRARIES})
ENDIF(APPLE)
FIND_PACKAGE(XPM REQUIRED)
SET(LIBS ${LIBS} ${XPM_LIBRARIES})

#-------------- XFORMS ------------------------------------
#----------------------------------------------------------
IF(WITH_XFORMS)
FIND_PACKAGE(XFORMS REQUIRED)
INCLUDE_directories(${XFORMS_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${XFORMS_INCLUDE_DIR})
SET(LIBS ${LIBS} ${XFORMS_LIBRARIES})
ENDIF(WITH_XFORMS)

#-------------- GBM ---------------------------------------
#----------------------------------------------------------
IF(USE_GBM)
FIND_PACKAGE(GBM REQUIRED)
INCLUDE_directories(${GBM_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
SET(LIBS ${LIBS} ${GBM_LIBRARIES})
ENDIF(USE_GBM)

#-------------- GSL ---------------------------------------
#----------------------------------------------------------
IF(USE_GSL)
  FIND_PACKAGE(GSL REQUIRED)
  INCLUDE_directories(${GSL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GSL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GSL_LIBRARIES})
ENDIF(USE_GSL)

#-------------- GLEW ---------------------------------------
#----------------------------------------------------------
IF(USE_SHADERS)
  FIND_PACKAGE(GLEW REQUIRED)
  INCLUDE_directories(${GLEW_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLEW_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLEW_LIBRARIES})
ENDIF(USE_SHADERS)

#-------------- CXX_PLANNER -------------------------------
#----------------------------------------------------------
IF(QT_GL)
# boost is used e.g. for managing the cost functions (T-RRT).
  FIND_PACKAGE(Boost REQUIRED)
  INCLUDE_directories(${BOOST_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${BOOST_INCLUDE_DIR})

ENDIF(QT_GL)


#-------------- GRASP PLANNER -----------------------------
#----------------------------------------------------------
IF(GRASP_PLANNING)
  FIND_PACKAGE(GLPK REQUIRED)
  INCLUDE_directories(${GLPK_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLPK_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLPK_LIBRARIES})

# FIND_PACKAGE(QHULL REQUIRED)
#  INCLUDE_directories(${QHULL_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QHULL_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${QHULL_LIBRARIES})


  FIND_PACKAGE(PkgConfig)
  pkg_check_modules(GLIB REQUIRED glib-2.0)
  include_directories(${GLIB_INCLUDE_DIRS})
  link_directories(${GLIB_LIBRARY_DIRS})

  FIND_PACKAGE(GTS REQUIRED)
  INCLUDE_directories(${GTS_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GTS_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GTS_LIBRARIES})
ENDIF(GRASP_PLANNING)

#-------------- MULTILOCALPATH ----------------------------
#----------------------------------------------------------
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

#-------------- HRP2 --------------------------------------
#----------------------------------------------------------
IF(USE_HRP2_GIK)
  FIND_PACKAGE(HRP2_GIK REQUIRED)
  INCLUDE_directories(${HRP2_GIK_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${HRP2_GIK_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${HRP2_GIK_LIBRARIES})
ENDIF(USE_HRP2_GIK)

#-------------- Glut --------------------------------------

IF(USE_GLUT)
  FIND_PACKAGE (GLUT REQUIRED glut)
  INCLUDE_directories (${GLUT_INCLUDE_DIR})
  SET (LIBS ${LIBS} ${GLUT_glut_LIBRARY})
ENDIF()

#-------------- HATP LIBRARY ------------------------------
#----------------------------------------------------------
IF(USE_HATP_LIBS)
  FIND_PACKAGE(HATP_LIB REQUIRED)
  INCLUDE_directories(${HATP_LIB_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${HATP_LIB_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${HATP_LIB_LIBRARIES})
ENDIF(USE_HATP_LIBS)

#Template :
# FIND_PACKAGE(<Package> REQUIRED)
# INCLUDE_directories(${<PACKAGE>_INCLUDE_DIR})
# SET(LIBS ${LIBS} ${<PACKAGE>_LIBRARIES})

#-------------- ROBOTMODELPARSER ------------------------------
#----------------------------------------------------------
IF(USE_ROBOTMODELPARSER)
    FIND_PACKAGE(robotModelParser REQUIRED)
    INCLUDE_directories(${robotModelParser_INCLUDE_DIR})
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${robotModelParser_INCLUDE_DIR})
    SET(LIBS ${LIBS} ${robotModelParser_LIBRARIES})

#-------------------- ASSIMP ------------------------------
#----------------------------------------------------------
    FIND_PACKAGE(ASSIMP REQUIRED)
    INCLUDE_directories(${ASSIMP_INCLUDE_DIR}/assimp)
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIR}/assimp)
    SET(LIBS ${LIBS} ${ASSIMP_LIBRARIES})
ENDIF(USE_ROBOTMODELPARSER)

MESSAGE(${LIBS})

#------------------- PKG_CONFIG ---------------------------
#----------------------------------------------------------
STRING(REPLACE ";" " -D" MOVE3D_COMPILATION_FLAGS "-D${MOVE3D_COMPILATION_FLAGS}")
STRING(REPLACE ";" " -I" MOVE3D_COMPILATION_INCLUDES "-I${INCLUDE_DIRS}")

STRING(REGEX REPLACE "(debug;[^;]*;)|optimized" "" MOVE3D_COMPILATION_LIBS "${LIBS}")
STRING(REPLACE ";" " " MOVE3D_COMPILATION_LIBS "${MOVE3D_COMPILATION_LIBS}")
add_subdirectory(pkgconfig)

#message("PKG_CONFIG : " )
#message(${pkgconfig})
#----------------------------------------------------------


# -------------------------------------------------------------
# --- SUBDIR PROCESS ------------------------------------------
# -------------------------------------------------------------
SET(BM3D_src "")
SET(BM3D_inc "")
SET(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

#####################################################################
#### Add here a src directory containing a SourceList.cmake file ####
#####################################################################
SET(BM3D_SOURCE_DIRECTORIES 
animation 
bio 
collision 
graphic 
graspPlanning 
groundHeight 
lightPlanner 
localpath 
p3d 
planner 
util)

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

#MESSAGE("Debug: source path : ${BM3D_inc}")
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

MESSAGE(libmove3d_SOURCE_DIR : ${libmove3d_SOURCE_DIR})
FOREACH(srcDir ${BM3D_SOURCE_DIRECTORIES})
  INCLUDE(${libmove3d_SOURCE_DIR}/${srcDir}/SourceList.cmake)
ENDFOREACH(srcDir)

#MESSAGE("bm3dsrc : ${BM3D_src}") #Uncomment to print the included file by Cmake

# -------------------------------------------------------------
# --- Directories to include ----------------------------------
# -------------------------------------------------------------
BM3D_INC_DIR_PROCESS(${libmove3d_SOURCE_DIR}/include)
BM3D_INC_DIR_PROCESS(${libmove3d_SOURCE_DIR})
IF(QT_LIBRARY)
  # necessary for the UI headers
  BM3D_INC_DIR_PROCESS(${CMAKE_CURRENT_BINARY_DIR})
  # generate the UI headers
  QT4_WRAP_UI(BM3D_QT_UI_HEADERS ${BM3D_QT_UI_FILES})
  # Qt4 files that need to be preprocessed
  #BM3D_SetCompilationFlags()
  add_definitions(-DFOO -DBAR)
  get_directory_property(flags DEFINITIONS)
  message("${flags}")
  message("CMAKE_C_FLAGS = " ${CMAKE_C_FLAGS}) 
  message("CMAKE_CXX_FLAGS ="  ${CMAKE_CXX_FLAGS})
  QT4_WRAP_CPP(BM3D_QT_MOC ${BM3D_QT_FILES_FOR_MOC})
ENDIF(QT_LIBRARY)

# -------------------------------------------------------------
# --- Compiler and Compilation flags --------------------------
# -------------------------------------------------------------
set_source_files_properties(${BM3D_src} PROPERTIES LANGUAGE CXX)

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

SET(EXECUTABLE_OUTPUT_PATH "${libmove3d_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin/$ENV{HOSTTYPE}")
SET(LIBRARY_OUTPUT_PATH "${libmove3d_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/$ENV{HOSTTYPE}")

#Default Flags
ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

# -------------------------------------------------------------
# --- Compilation and Installation ----------------------------
# -------------------------------------------------------------
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  SET(CMAKE_INSTALL_PREFIX "${libmove3d_BINARY_DIR}/install" CACHE PATH "libmove3d install prefix" FORCE)
   SET(CMAKE_INSTALL_PREFIX "$ENV{ROBOTPKG_BASE}" CACHE PATH "libmove3d install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
		
ADD_LIBRARY(move3d SHARED ${BM3D_src} ${BM3D_QT_MOC} ${BM3D_QT_UI_HEADERS} ${BM3D_inc})
#SET_TARGET_PROPERTIES(move3d PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(move3d ${LIBS} m)

ADD_EXECUTABLE(move3dtest test/main.cpp)
SET_TARGET_PROPERTIES(move3dtest PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(move3dtest move3d)

INSTALL(TARGETS move3d
  LIBRARY DESTINATION lib
  CONFIGURATIONS ${CMAKE_BUILD_TYPE})

INSTALL(DIRECTORY ${BM3D_SOURCE_DIRECTORIES} ${libmove3d_SOURCE_DIR}/include 
  DESTINATION include/libmove3d
  FILES_MATCHING PATTERN "*.h")

INSTALL(DIRECTORY ${BM3D_SOURCE_DIRECTORIES} ${libmove3d_SOURCE_DIR}/include 
  DESTINATION include/libmove3d
  FILES_MATCHING PATTERN "*.hpp")

# -------------------------------------------------------------
# --- Uninstall -----------------------------------------------
# -------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/script/CMake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
  
  
# -------------------------------------------------------------
# --- Testing -------------------------------------------------
# -------------------------------------------------------------
ENABLE_TESTING()

MESSAGE("CMAKE_BINARY_DIR = " ${CMAKE_BINARY_DIR})

ADD_TEST(NAME basic COMMAND move3dtest)

#ADD_TEST(NAME ArmFreeTest
#        COMMAND move3d 	-f ../../libmove3dDemos-JidoSAHand/GS/gsJidoKukaSAHandSM.p3d -sc ../../libmove3dDemos-JidoSAHand/GS/SCENARIO/ManipulationTestSAHand.sce -test 1)
	
#ADD_TEST(NAME ArmGotoTest
#        COMMAND move3d 	-f
#        ../../libmove3dDemos-JidoSAHand/GS/gsJidoKukaSAHandSM.p3d -sc
#        ../../libmove3dDemos-JidoSAHand/GS/SCENARIO/ManipulationTestSAHand.sce -test 2)

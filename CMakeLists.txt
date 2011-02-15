CMAKE_MINIMUM_REQUIRED (VERSION 2.6.4)
PROJECT (BioMove3D C CXX)


#--------- Set the project version

if( NOT BIOMOVE3D_VERSION)

if( NOT CPACK_PACKAGE_VERSION_MAJOR)
set( CPACK_PACKAGE_VERSION_MAJOR 3)
endif( NOT CPACK_PACKAGE_VERSION_MAJOR)

if( NOT CPACK_PACKAGE_VERSION_MINOR)
set( CPACK_PACKAGE_VERSION_MINOR 9)
endif( NOT CPACK_PACKAGE_VERSION_MINOR)

if( NOT CPACK_PACKAGE_VERSION_PATCH)
set( CPACK_PACKAGE_VERSION_PATCH 1)
endif( NOT CPACK_PACKAGE_VERSION_PATCH)

set( BIOMOVE3D_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")

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

  IF( GENERATE_LIBMOVE3D )
	  MESSAGE("You are building the libmove3d library.")
  ELSEIF( GENERATE_STUDIO )
	  MESSAGE("You are building the stand-alone move3d-studio application.")
  ELSEIF( GENERATE_VIEWER_LIB )
	  MESSAGE("You are building the libmove3d library for stand-alone move3d viewer.")
  ELSE ()
	  MESSAGE("You did not select any target yet (choose one of the GENERATE_* options).")
  ENDIF()

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

# P3D core ---
BM3D_flag(P3D_CONSTRAINTS "Compilation with module Constraints" ON)
BM3D_flag(P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
BM3D_flag(P3D_LOCALPATH "Compilation with module Localpath" ON)
BM3D_flag(P3D_PLANNER "Compilation with module Planner" ON)

# Collision checking --
BM3D_flag(KCD_CP_IS_SOLID "Convex polyhedron as a solid instead of as a collection of convex facets in KCD" OFF P3D_COLLISION_CHECKING)
BM3D_flag(USE_P3D_BODY_AABB "test with AABB around body of robot" ON P3D_COLLISION_CHECKING)
BM3D_flag(KCD_APPROX_ROBOT "KCD with volume" OFF P3D_COLLISION_CHECKING)
BM3D_flag(GJK "Use gjk (just for debugging)" OFF P3D_COLLISION_CHECKING)
BM3D_flag(OBB_EVAL "write a file with OBB tree information (just for debugging)" OFF P3D_COLLISION_CHECKING)
BM3D_flag(VCOLLIDE "Use vCollide Collision detector" OFF)

# Bio-related ---
BM3D_flag(BIO "Compilation with module Bio" OFF)
BM3D_flag(ENERGY "Compilation with module Bio-Energy" OFF)

# Path planning --
BM3D_flag(MULTIGRAPH "Compilation with module Multi-Graph" OFF P3D_PLANNER)
BM3D_flag(LIGHT_PLANNER "Extern Mode (Use this mode to have a scriped functions for planning pick and place tasks)"  OFF P3D_PLANNER)
BM3D_flag(DPG "Dynamic planning using grids" OFF CXX_PLANNER)

BM3D_flag(HRI_PLANNER "Human Robot Interaction planner module" OFF USE_GSL)
BM3D_flag(USE_MIGHTABILITY_MAPS "Mightability Map Calculation and Planning" OFF)


BM3D_flag(MULTILOCALPATH "Compilation avec le module Multi-Localpath" OFF USE_GBM)
BM3D_flag(GRASP_PLANNING "Grasp planning module" OFF)

BM3D_flag(FK_CNTRT "to use the automatic generation of forward kinematics constraints (reciprocal of the closed chained constraints)" OFF LIGHT_PLANNER)

BM3D_flag(USE_CXX_PLANNER "Sets the planning API to be default in Move3D" OFF)
BM3D_flag(LIST_OF_PLANNERS "Sets the planning API to be default in Move3D" OFF)

## User Interfaces ---
BM3D_flag(QT_LIBRARY "Sets QtLibrary compiling mode" OFF)
BM3D_flag(QT_GL "Sets QtLibrary compiling mode" OFF QT_LIBRARY)
BM3D_flag(QT_GL_WIDGET "Compile with the OpenGl widget" OFF QT_GL QT_LIBRARY)

BM3D_flag(USE_GLUT "Compilation with a glut window" OFF)

BM3D_flag(WITH_XFORMS "Compile the Xform interface" OFF)

BM3D_flag(ENABLE_ANTIALIASING "Enables antialiasing to draw smooth lines" OFF)
BM3D_flag(USE_SHADERS "Enables use of OpenGL shaders" OFF)

BM3D_flag(MOVE3D_CORE "Enables compilation of Move3D-core" OFF)
BM3D_flag(HRI_PLANNER_GUI "Xforms user interface OFF Human Robot Interaction planner module" OFF HRI_PLANNER WITH_XFORMS)

# Utility libraries ---
BM3D_flag(USE_GSL "Mathematic Library" OFF)
BM3D_flag(USE_CLAPACK "Mathematic Library" OFF)

# Other ---
BM3D_flag(ANIMATION "Compilation with the module character" OFF)
BM3D_flag(USE_HRP2_GIK "HRP2 GIK Library" OFF)
BM3D_flag(USE_HATP_LIBS "For including HATP libraries" OFF)
BM3D_flag(USE_GBM "Compilation with the gbM package" OFF)


# META flags ------
#------------------
BM3D_flag(GENERATE_LIBMOVE3D "Set required options to compile the core libmove3d, without user interface" OFF MOVE3D_ROBOTICS USE_GLUT)

BM3D_flag(GENERATE_STUDIO "Set required options to compile mov3d-studio, the stand-alone, xforms-based, dev environment" OFF MOVE3D_ROBOTICS HRI_PLANNER_GUI)

BM3D_flag(GENERATE_VIEWER_LIB "Set the required options to compile the stand-alone move3d viewer (requires SPARK at run-time)" OFF MOVE3D_ROBOTICS MOVE3D_CORE QT_LIBRARY QT_GL)

BM3D_flag(MOVE3D_ROBOTICS "Set all the standard options related to robotics" OFF HRI_PLANNER GRASP_PLANNING LIGHT_PLANNER MULTIGRAPH MULTILOCALPATH USE_GSL )

# TODO : implement a proper search for the GSL and/or Lapack libraries, and add the relevant flags (USE_GSL and USE_CLAPACK)
# It is necessary to link against either GSL or Lapack for protein loop sampling.
BM3D_flag(MOVE3D_BIO "Set all the standard options related to bio motion planning" OFF BIO WITH_XFORMS)

BM3D_ProcessCompilationFlags()
BM3D_SetCompilationFlags()

#################################################################
#### Add here a library to include (custom FindPackage.cmake ####
#### are in BioMove3D/.cmakeScripts. Use the template to     ####
#### create a custom one)                                    ####
#################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")

# Add a "doc" target that runs Doxygen and install the doc.
INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)

#-------------- Qt Library --------------------------------
#----------------------------------------------------------
SET(GL_NATIVE FALSE)

IF((QT_LIBRARY AND QT_GL) OR USE_GLUT)
SET(GL_NATIVE TRUE)
ENDIF()

IF(QT_LIBRARY)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS}${QT_INCLUDE_DIR}) 
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
  FIND_PACKAGE(EIGEN REQUIRED)
  INCLUDE_directories(${Eigen_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Eigen_INCLUDE_DIR})
  
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

  FIND_PACKAGE(GLIBCONFIG REQUIRED)
  INCLUDE_directories(${GLIBCONFIG_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLIBCONFIG_INCLUDE_DIR})

  FIND_PACKAGE(GLIB REQUIRED)
  INCLUDE_directories(${GLIB_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLIB_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLIB_LIBRARIES})

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

#------------------- PKG_CONFIG ---------------------------
#----------------------------------------------------------
STRING(REPLACE ";" " -D" MOVE3D_COMPILATION_FLAGS "-D${MOVE3D_COMPILATION_FLAGS}")
STRING(REPLACE ";" " -I" MOVE3D_COMPILATION_INCLUDES "-I${INCLUDE_DIRS}")

STRING(REGEX REPLACE "(debug;[^;]*;)|optimized" "" MOVE3D_COMPILATION_LIBS "${LIBS}")
STRING(REPLACE ";" " " MOVE3D_COMPILATION_LIBS "${MOVE3D_COMPILATION_LIBS}")
add_subdirectory(pkgconfig)


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
hri_planner 
lightPlanner 
localpath 
move3d 
p3d 
planner 
userappli 
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

MESSAGE(BioMove3D_SOURCE_DIR : ${BioMove3D_SOURCE_DIR})
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

# -------------------------------------------------------------
# --- Compilation and Installation ----------------------------
# -------------------------------------------------------------
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${BioMove3D_BINARY_DIR}/install" CACHE PATH "BioMove3D install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

IF(GENERATE_VIEWER_LIB) #Install the move3d-viewer library + the headers
		
	ADD_LIBRARY(move3d-viewer SHARED ${BM3D_src} ${BM3D_QT_MOC} ${BM3D_QT_UI_HEADERS} ${BM3D_inc})
	TARGET_LINK_LIBRARIES(move3d-viewer ${LIBS} m)

	INSTALL(TARGETS move3d-viewer
        LIBRARY DESTINATION lib
		CONFIGURATIONS ${CMAKE_BUILD_TYPE})
	
	INSTALL(DIRECTORY ${BM3D_SOURCE_DIRECTORIES} ${BioMove3D_SOURCE_DIR}/include 
			DESTINATION include/libmove3d 
			FILES_MATCHING REGEX "\\.[h|hpp]")

ELSEIF(GENERATE_LIBMOVE3D) # Only install the library and the headers
	
	ADD_LIBRARY(move3d SHARED ${BM3D_src} ${BM3D_QT_MOC} ${BM3D_QT_UI_HEADERS} ${BM3D_inc})
	TARGET_LINK_LIBRARIES(move3d ${LIBS} m)

	INSTALL(TARGETS move3d 
        LIBRARY DESTINATION lib
		CONFIGURATIONS ${CMAKE_BUILD_TYPE})
	
	INSTALL(DIRECTORY ${BM3D_SOURCE_DIRECTORIES} ${BioMove3D_SOURCE_DIR}/include 
			DESTINATION include/libmove3d 
			FILES_MATCHING REGEX "\\.[h|hpp]")


ELSE() # move3d-studio: Compiles a static libmove3d and only install the binary.

	#MESSAGE("Debug: sources files:  ${BM3D_src}")

	ADD_LIBRARY(move3d STATIC ${BM3D_src} ${BM3D_QT_MOC} ${BM3D_QT_UI_HEADERS} ${BM3D_inc})
	TARGET_LINK_LIBRARIES(move3d ${LIBS} m)

	ADD_EXECUTABLE(move3d-studio ${BioMove3D_SOURCE_DIR}/move3d/move3d.c ${BM3D_inc})
	TARGET_LINK_LIBRARIES(move3d-studio move3d ${LIBS} m)

	INSTALL(TARGETS move3d-studio
        RUNTIME DESTINATION bin 
		CONFIGURATIONS ${CMAKE_BUILD_TYPE})

ENDIF()


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

ADD_TEST(NAME ArmFreeTest
        COMMAND move3d 	-f ../../BioMove3DDemos-JidoSAHand/GS/gsJidoKukaSAHandSM.p3d -sc ../../BioMove3DDemos-JidoSAHand/GS/SCENARIO/ManipulationTestSAHand.sce -test 1)
	
ADD_TEST(NAME ArmGotoTest
        COMMAND move3d 	-f ../../BioMove3DDemos-JidoSAHand/GS/gsJidoKukaSAHandSM.p3d -sc ../../BioMove3DDemos-JidoSAHand/GS/SCENARIO/ManipulationTestSAHand.sce -test 2)

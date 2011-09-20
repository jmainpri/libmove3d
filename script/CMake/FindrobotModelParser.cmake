# - Check for the presence of robotModelParser
#
# The following variables are set when robotModelParser is found:
#  HAVE_robotModelParser       = Set to true, if all components of robotModelParser
#                          have been found.
#  robotModelParser_INCLUDE_DIR   = Include path for the header files of robotModelParser
#  robotModelParser_LIBRARIES  = Link these to use robotModelParser

## -----------------------------------------------------------------------------
## Check for the header files

find_path (robotModelParser_INCLUDE_DIR robotModelParser/urdf_model.h
  PATHS ${robotModelParser_INC} /usr/local/include /usr/include /sw/include /opt/local/include $ENV{ROBOTPKG_BASE}/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (robotModelParser_LIBRARIES robotModelParser
  PATHS ${robotModelParser_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (robotModelParser_INCLUDE_DIR AND robotModelParser_LIBRARIES)
  set (HAVE_robotModelParser TRUE)
else (robotModelParser_INCLUDE_DIR AND robotModelParser_LIBRARIES)
  if (NOT robotModelParser_FIND_QUIETLY)
    if (NOT robotModelParser_INCLUDE_DIR)
      message (STATUS "Unable to find robotModelParser header files!")
    endif (NOT robotModelParser_INCLUDE_DIR)
    if (NOT robotModelParser_LIBRARIES)
      message (STATUS "Unable to find robotModelParser library files!")
    endif (NOT robotModelParser_LIBRARIES)
  endif (NOT robotModelParser_FIND_QUIETLY)
endif (robotModelParser_INCLUDE_DIR AND robotModelParser_LIBRARIES)

if (HAVE_robotModelParser)
  if (NOT robotModelParser_FIND_QUIETLY)
    message (STATUS "Found components for robotModelParser")
    message (STATUS "robotModelParser_INCLUDE_DIR = ${robotModelParser_INCLUDE_DIR}")
    message (STATUS "robotModelParser_LIBRARIES = ${robotModelParser_LIBRARIES}")
  endif (NOT robotModelParser_FIND_QUIETLY)
else (HAVE_robotModelParser)
  if (robotModelParser_FIND_REQUIRED)
    SET(robotModelParser_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(robotModelParser_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find robotModelParser!")
  endif (robotModelParser_FIND_REQUIRED)
endif (HAVE_robotModelParser)

mark_as_advanced (
  HAVE_robotModelParser
  robotModelParser_LIBRARIES
  robotModelParser_INCLUDE_DIR
  )

# - Check for the presence of ROBOTMODELPARSER
#
# The following variables are set when ROBOTMODELPARSER is found:
#  HAVE_ROBOTMODELPARSER       = Set to true, if all components of ROBOTMODELPARSER
#                          have been found.
#  ROBOTMODELPARSER_INCLUDE_DIR   = Include path for the header files of ROBOTMODELPARSER
#  ROBOTMODELPARSER_LIBRARIES  = Link these to use ROBOTMODELPARSER

## -----------------------------------------------------------------------------
## Check for the header files

find_path (ROBOTMODELPARSER_INCLUDE_DIR RobotModelParser/urdf_model.h
  PATHS ${ROBOTMODELPARSER_INC} /usr/local/include /usr/include /sw/include /opt/local/include $ENV{ROBOTPKG_BASE}/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (ROBOTMODELPARSER_LIBRARIES RobotModelParser
  PATHS ${ROBOTMODELPARSER_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (ROBOTMODELPARSER_INCLUDE_DIR AND ROBOTMODELPARSER_LIBRARIES)
  set (HAVE_ROBOTMODELPARSER TRUE)
else (ROBOTMODELPARSER_INCLUDE_DIR AND ROBOTMODELPARSER_LIBRARIES)
  if (NOT ROBOTMODELPARSER_FIND_QUIETLY)
    if (NOT ROBOTMODELPARSER_INCLUDE_DIR)
      message (STATUS "Unable to find ROBOTMODELPARSER header files!")
    endif (NOT ROBOTMODELPARSER_INCLUDE_DIR)
    if (NOT ROBOTMODELPARSER_LIBRARIES)
      message (STATUS "Unable to find ROBOTMODELPARSER library files!")
    endif (NOT ROBOTMODELPARSER_LIBRARIES)
  endif (NOT ROBOTMODELPARSER_FIND_QUIETLY)
endif (ROBOTMODELPARSER_INCLUDE_DIR AND ROBOTMODELPARSER_LIBRARIES)

if (HAVE_ROBOTMODELPARSER)
  if (NOT ROBOTMODELPARSER_FIND_QUIETLY)
    message (STATUS "Found components for ROBOTMODELPARSER")
    message (STATUS "ROBOTMODELPARSER_INCLUDE_DIR = ${ROBOTMODELPARSER_INCLUDE_DIR}")
    message (STATUS "ROBOTMODELPARSER_LIBRARIES = ${ROBOTMODELPARSER_LIBRARIES}")
  endif (NOT ROBOTMODELPARSER_FIND_QUIETLY)
else (HAVE_ROBOTMODELPARSER)
  if (ROBOTMODELPARSER_FIND_REQUIRED)
    SET(ROBOTMODELPARSER_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(ROBOTMODELPARSER_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find ROBOTMODELPARSER!")
  endif (ROBOTMODELPARSER_FIND_REQUIRED)
endif (HAVE_ROBOTMODELPARSER)

mark_as_advanced (
  HAVE_ROBOTMODELPARSER
  ROBOTMODELPARSER_LIBRARIES
  ROBOTMODELPARSER_INCLUDE_DIR
  )

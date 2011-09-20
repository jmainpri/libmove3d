# - Check for the presence of ASSIMP
#
# The following variables are set when ASSIMP is found:
#  HAVE_ASSIMP       = Set to true, if all components of ASSIMP
#                          have been found.
#  ASSIMP_INCLUDE_DIR   = Include path for the header files of ASSIMP
#  ASSIMP_LIBRARIES  = Link these to use ASSIMP

## -----------------------------------------------------------------------------
## Check for the header files

find_path (ASSIMP_INCLUDE_DIR assimp/assimp.h
  PATHS ${ASSIMP_INC} /usr/local/include /usr/include /sw/include /opt/local/include $ENV{ROBOTPKG_BASE}/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (ASSIMP_LIBRARIES assimp
  PATHS ${ASSIMP_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARIES)
  set (HAVE_ASSIMP TRUE)
else (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARIES)
  if (NOT ASSIMP_FIND_QUIETLY)
    if (NOT ASSIMP_INCLUDE_DIR)
      message (STATUS "Unable to find ASSIMP header files!")
    endif (NOT ASSIMP_INCLUDE_DIR)
    if (NOT ASSIMP_LIBRARIES)
      message (STATUS "Unable to find ASSIMP library files!")
    endif (NOT ASSIMP_LIBRARIES)
  endif (NOT ASSIMP_FIND_QUIETLY)
endif (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARIES)

if (HAVE_ASSIMP)
  if (NOT ASSIMP_FIND_QUIETLY)
    message (STATUS "Found components for ASSIMP")
    message (STATUS "ASSIMP_INCLUDE_DIR = ${ASSIMP_INCLUDE_DIR}")
    message (STATUS "ASSIMP_LIBRARIES = ${ASSIMP_LIBRARIES}")
  endif (NOT ASSIMP_FIND_QUIETLY)
else (HAVE_ASSIMP)
  if (ASSIMP_FIND_REQUIRED)
    SET(ASSIMP_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(ASSIMP_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find ASSIMP!")
  endif (ASSIMP_FIND_REQUIRED)
endif (HAVE_ASSIMP)

mark_as_advanced (
  HAVE_ASSIMP
  ASSIMP_LIBRARIES
  ASSIMP_INCLUDE_DIR
  )

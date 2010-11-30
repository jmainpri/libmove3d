# - Check for the presence of PQP
#
# The following variables are set when PQP is found:
#  HAVE_PQP       = Set to true, if all components of PQP
#                          have been found.
#  PQP_INCLUDE_DIR   = Include path for the header files of PQP
#  PQP_LIBRARIES  = Link these to use PQP

## -----------------------------------------------------------------------------
## Check for the header files

find_path (PQP_INCLUDE_DIR pqp/PQP.h
  PATHS ${PQP_INC} /usr/local/include /usr/include /sw/include /opt/local/include $ENV{ROBOTPKG_BASE}/include
  )


## -----------------------------------------------------------------------------
## Check for the library

find_library (PQP_LIBRARIES pqp
  PATHS ${PQP_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (PQP_INCLUDE_DIR AND PQP_LIBRARIES)
  set (HAVE_PQP TRUE)
else (PQP_INCLUDE_DIR AND PQP_LIBRARIES)
  if (NOT PQP_FIND_QUIETLY)
    if (NOT PQP_INCLUDE_DIR)
      message (STATUS "Unable to find PQP header files!")
    endif (NOT PQP_INCLUDE_DIR)
    if (NOT PQP_LIBRARIES)
      message (STATUS "Unable to find PQP library files!")
    endif (NOT PQP_LIBRARIES)
  endif (NOT PQP_FIND_QUIETLY)
endif (PQP_INCLUDE_DIR AND PQP_LIBRARIES)

if (HAVE_PQP)
  if (NOT PQP_FIND_QUIETLY)
    message (STATUS "Found components for PQP")
    message (STATUS "PQP_INCLUDE_DIR = ${PQP_INCLUDE_DIR}")
    message (STATUS "PQP_LIBRARIES = ${PQP_LIBRARIES}")
  endif (NOT PQP_FIND_QUIETLY)
else (HAVE_PQP)
  if (PQP_FIND_REQUIRED)
    SET(PQP_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(PQP_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find PQP!")
  endif (PQP_FIND_REQUIRED)
endif (HAVE_PQP)

mark_as_advanced (
  HAVE_PQP
  PQP_LIBRARIES
  PQP_INCLUDE_DIR
  )

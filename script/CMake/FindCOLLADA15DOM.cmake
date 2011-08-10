# - Check for the presence of COLLADA15DOM
#
# The following variables are set when COLLADA15DOM is found:
#  HAVE_COLLADA15DOM       = Set to true, if all components of COLLADA15DOM
#                          have been found.
#  COLLADA15DOM_INCLUDE_DIR   = Include path for the header files of COLLADA15DOM
#  COLLADA15DOM_LIBRARIES  = Link these to use COLLADA15DOM

## -----------------------------------------------------------------------------
## Check for the header files
  
find_path (COLLADA15DOM_INCLUDE_DIR dae.h
  PATHS ${COLLADA15DOM_INC} /usr/local/include/collada-dom /usr/include /sw/include /opt/local/include
  )
  
## -----------------------------------------------------------------------------
## Check for the library

find_library (COLLADA15DOM_LIBRARIES collada15dom
  PATHS ${COLLADA15DOM_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (COLLADA15DOM_INCLUDE_DIR AND COLLADA15DOM_LIBRARIES)
  set (HAVE_COLLADA15DOM TRUE)
else (COLLADA15DOM_INCLUDE_DIR AND COLLADA15DOM_LIBRARIES)
  if (NOT COLLADA15DOM_FIND_QUIETLY)
    if (NOT COLLADA15DOM_INCLUDE_DIR)
      message (STATUS "Unable to find COLLADA15DOM header files!")
    endif (NOT COLLADA15DOM_INCLUDE_DIR)
    if (NOT COLLADA15DOM_LIBRARIES)
      message (STATUS "Unable to find COLLADA15DOM library files!")
    endif (NOT COLLADA15DOM_LIBRARIES)
  endif (NOT COLLADA15DOM_FIND_QUIETLY)
endif (COLLADA15DOM_INCLUDE_DIR AND COLLADA15DOM_LIBRARIES)

if (HAVE_COLLADA15DOM)
  if (NOT COLLADA15DOM_FIND_QUIETLY)
    message (STATUS "Found components for COLLADA15DOM")
    message (STATUS "COLLADA15DOM_INCLUDE_DIR = ${COLLADA15DOM_INCLUDE_DIR}")
    message (STATUS "COLLADA15DOM_LIBRARIES = ${COLLADA15DOM_LIBRARIES}")
  endif (NOT COLLADA15DOM_FIND_QUIETLY)
else (HAVE_COLLADA15DOM)
  if (COLLADA15DOM_FIND_REQUIRED)
    SET(COLLADA15DOM_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(COLLADA15DOM_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find COLLADA15DOM!")
  endif (COLLADA15DOM_FIND_REQUIRED)
endif (HAVE_COLLADA15DOM)

mark_as_advanced (
  HAVE_COLLADA15DOM
  COLLADA15DOM_LIBRARIES
  COLLADA15DOM_INCLUDE_DIR
  )

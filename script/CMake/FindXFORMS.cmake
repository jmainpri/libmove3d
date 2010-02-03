# - Check for the presence of XFORMS
#
# The following variables are set when XFORMS is found:
#  HAVE_XFORMS       = Set to true, if all components of XFORMS
#                          have been found.
#  XFORMS_INCLUDE_DIR   = Include path for the header files of XFORMS
#  XFORMS_LIBRARIES  = Link these to use XFORMS

## -----------------------------------------------------------------------------
## Check for the header files

if( NOT XFORMS_INCLUDE_DIR )
  find_path (XFORMS_INCLUDE_DIR forms.h
    PATHS /usr/local/include /usr/include /sw/include /opt/local/include
    )
endif( NOT XFORMS_INCLUDE_DIR )

## -----------------------------------------------------------------------------
## Check for the library

# expose var to ccmake
SET(XFORMS_LIBRARY_ADD_PATH "" CACHE PATH "Pathes where to additionally look for xform libs")

find_library (XFORMS_LIBRARIES formsGL
  PATHS ${XFORMS_LIBRARY_ADD_PATH} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib 
  )
find_library (FORMS_LIBRARIES forms
  PATHS ${XFORMS_LIBRARY_ADD_PATH} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib 
  )

SET(XFORMS_LIBRARIES ${XFORMS_LIBRARIES} ${FORMS_LIBRARIES})

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (XFORMS_INCLUDE_DIR AND XFORMS_LIBRARIES)
  set (HAVE_XFORMS TRUE)
else (XFORMS_INCLUDE_DIR AND XFORMS_LIBRARIES)
  if (NOT XFORMS_FIND_QUIETLY)
    if (NOT XFORMS_INCLUDE_DIR)
      message (STATUS "Unable to find XFORMS header files!")
    endif (NOT XFORMS_INCLUDE_DIR)
    if (NOT XFORMS_LIBRARIES)
      message (STATUS "Unable to find XFORMS library files!")
    endif (NOT XFORMS_LIBRARIES)
  endif (NOT XFORMS_FIND_QUIETLY)
endif (XFORMS_INCLUDE_DIR AND XFORMS_LIBRARIES)

if (HAVE_XFORMS)
  if (NOT XFORMS_FIND_QUIETLY)
    message (STATUS "Found components for XFORMS")
    message (STATUS "XFORMS_INCLUDE_DIR = ${XFORMS_INCLUDE_DIR}")
    message (STATUS "XFORMS_LIBRARIES = ${XFORMS_LIBRARIES}")
  endif (NOT XFORMS_FIND_QUIETLY)
else (HAVE_XFORMS)
  if (XFORMS_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find XFORMS, install it or fix XFORMS_INCLUDE_DIR and XFORMS_LIBRARY_ADD_PATH!")
  endif (XFORMS_FIND_REQUIRED)
endif (HAVE_XFORMS)

mark_as_advanced (
  HAVE_XFORMS
  XFORMS_LIBRARIES
  XFORMS_INCLUDE_DIR
  )
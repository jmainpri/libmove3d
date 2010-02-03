# - Check for the presence of XML2
#
# The following variables are set when XML2 is found:
#  HAVE_XML2       = Set to true, if all components of XML2
#                          have been found.
#  XML2_INCLUDE_DIR   = Include path for the header files of XML2
#  XML2_LIBRARIES  = Link these to use XML2

## -----------------------------------------------------------------------------
## Check for the header files

find_path (XML2_INCLUDE_DIR "libxml/xmlmemory.h"
  PATHS /usr/local/include /usr/include /sw/include /opt/local/include ${XML2_INC}
  PATH_SUFFIXES /libxml2
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (XML2_LIBRARIES xml2
  PATHS /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib ${XML2_LIB}
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (XML2_INCLUDE_DIR AND XML2_LIBRARIES)
  set (HAVE_XML2 TRUE)
else (XML2_INCLUDE_DIR AND XML2_LIBRARIES)
  if (NOT XML2_FIND_QUIETLY)
    if (NOT XML2_INCLUDE_DIR)
      message (STATUS "Unable to find XML2 header files!")
    endif (NOT XML2_INCLUDE_DIR)
    if (NOT XML2_LIBRARIES)
      message (STATUS "Unable to find XML2 library files!")
    endif (NOT XML2_LIBRARIES)
  endif (NOT XML2_FIND_QUIETLY)
endif (XML2_INCLUDE_DIR AND XML2_LIBRARIES)

if (HAVE_XML2)
  if (NOT XML2_FIND_QUIETLY)
    message (STATUS "Found components for XML2")
    message (STATUS "XML2_INCLUDE_DIR = ${XML2_INCLUDE_DIR}")
    message (STATUS "XML2_LIBRARIES = ${XML2_LIBRARIES}")
  endif (NOT XML2_FIND_QUIETLY)
else (HAVE_XML2)
  if (XML2_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find XML2!")
  endif (XML2_FIND_REQUIRED)
endif (HAVE_XML2)

mark_as_advanced (
  HAVE_XML2
  XML2_LIBRARIES
  XML2_INCLUDE_DIR
  )
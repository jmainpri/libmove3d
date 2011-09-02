# - Check for the presence of BOOST_SYSTEM-MT
#
# The following variables are set when BOOST_SYSTEM-MT is found:
#  HAVE_BOOST_SYSTEM-MT       = Set to true, if all components of BOOST_SYSTEM-MT
#                          have been found.
#  BOOST_SYSTEM-MT_INCLUDE_DIR   = Include path for the header files of BOOST_SYSTEM-MT
#  BOOST_SYSTEM-MT_LIBRARIES  = Link these to use BOOST_SYSTEM-MT

## -----------------------------------------------------------------------------
## Check for the header files

#find_path (BOOST_SYSTEM-MT_INCLUDE_DIR <header file(s)>
 # PATHS ${BOOST_SYSTEM-MT_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  #PATH_SUFFIXES <optional path extension>
  #)

## -----------------------------------------------------------------------------
## Check for the library

find_library (BOOST_SYSTEM-MT_LIBRARIES boost_system-mt
  PATHS ${BOOST_SYSTEM-MT_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (BOOST_SYSTEM-MT_LIBRARIES)
  set (HAVE_BOOST_SYSTEM-MT TRUE)
else (BOOST_SYSTEM-MT_LIBRARIES)
  if (NOT BOOST_SYSTEM-MT_FIND_QUIETLY)
    if (NOT BOOST_SYSTEM-MT_LIBRARIES)
      message (STATUS "Unable to find BOOST_SYSTEM-MT library files!")
    endif (NOT BOOST_SYSTEM-MT_LIBRARIES)
  endif (NOT BOOST_SYSTEM-MT_FIND_QUIETLY)
endif (BOOST_SYSTEM-MT_LIBRARIES)

if (HAVE_BOOST_SYSTEM-MT)
  if (NOT BOOST_SYSTEM-MT_FIND_QUIETLY)
    message (STATUS "Found components for BOOST_SYSTEM-MT")
    message (STATUS "BOOST_SYSTEM-MT_LIBRARIES = ${BOOST_SYSTEM-MT_LIBRARIES}")
  endif (NOT BOOST_SYSTEM-MT_FIND_QUIETLY)
else (HAVE_BOOST_SYSTEM-MT)
  if (BOOST_SYSTEM-MT_FIND_REQUIRED)
    SET(BOOST_SYSTEM-MT_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    message (FATAL_ERROR "Could not find BOOST_SYSTEM-MT!")
  endif (BOOST_SYSTEM-MT_FIND_REQUIRED)
endif (HAVE_BOOST_SYSTEM-MT)

mark_as_advanced (
  HAVE_BOOST_SYSTEM-MT
  BOOST_SYSTEM-MT_LIBRARIES
  )

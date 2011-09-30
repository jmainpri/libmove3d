prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_PREFIX@/lib
includedir=@CMAKE_INSTALL_PREFIX@/include
BioMove3DFlags= @MOVE3D_COMPILATION_FLAGS@
BioMove3DIncludes=@MOVE3D_COMPILATION_INCLUDES@
BioMove3DLibs=@MOVE3D_COMPILATION_LIBS@
 
Name: libmove3d
Description: Motion Planning Platform - Headless library
Version: @LIBMOVE3D_VERSION@
Libs: ${BioMove3DLibs} -L${libdir} -lmove3d
Cflags: -I${includedir}/libmove3d -I${includedir}/libmove3d/include -I${includedir} ${BioMove3DIncludes} ${BioMove3DFlags} 

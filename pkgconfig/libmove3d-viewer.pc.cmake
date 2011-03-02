prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_PREFIX@/lib
includedir=@CMAKE_INSTALL_PREFIX@/include
BioMove3DFlags= @MOVE3D_COMPILATION_FLAGS@
BioMove3DIncludes=@MOVE3D_COMPILATION_INCLUDES@
BioMove3DLibs=@MOVE3D_COMPILATION_LIBS@
 
Name: libmove3d-viewer
Description: Motion Planning Platform - Viewer library (with Qt support)
Version: @BIOMOVE3D_VERSION@
Libs: ${BioMove3DLibs} -L${libdir} -lmove3d-viewer
Cflags: -I${includedir}/libmove3d/include ${BioMove3DIncludes} ${BioMove3DFlags} 

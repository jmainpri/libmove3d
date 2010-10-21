prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_PREFIX@/lib
includedir=@CMAKE_INSTALL_PREFIX@/include
BioMove3DFlags= @MOVE3D_COMPILATION_FLAGS@
BioMove3DIncludes=@MOVE3D_COMPILATION_INCLUDES@
BioMove3DLibs=@MOVE3D_COMPILATION_LIBS@
 
Name: BioMove3D
Description: Motion Planning Platform
Version: @BIOMOVE3D_VERSION@
Libs: ${BioMove3DLibs} -L${libdir} -lMove3D-core
Cflags: -I${includedir}/Move3D-core/include -I${BioMove3DIncludes} ${BioMove3DFlags} 

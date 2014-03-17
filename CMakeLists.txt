CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( 3D_MSP )

SET( CMAKE_BUILD_TYPE Debug)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

include_directories("./3rdparty/kshortestpaths/")
include_directories("${BASE_DIR}/3rdparty/octomap-1.6.2-flo/octomap/include/octomap/")
# "/home/florian/workspace/octomap-1.6.1/octomap/include/octomap/"

ADD_SUBDIRECTORY( 3rdparty )
ADD_SUBDIRECTORY( src )

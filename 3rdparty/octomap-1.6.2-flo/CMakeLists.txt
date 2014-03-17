CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap-distribution )

SET( CMAKE_BUILD_TYPE Debug)

ENABLE_TESTING()  # enable CTest environment of subprojects

ADD_SUBDIRECTORY( octomap )
ADD_SUBDIRECTORY( octovis )
ADD_SUBDIRECTORY( dynamicEDT3D )
	



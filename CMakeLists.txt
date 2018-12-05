cmake_minimum_required(VERSION 3.10)
project(Slam2D)

set(CMAKE_CXX_STANDARD 14)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
#find_package( Eigen3 )
#include_directories( ${EIGEN3_INCLUDE_DIRS} )

# Ceres
find_package( Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRES})

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        )
############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( demo )
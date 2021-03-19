SET (PROJECT_NAME simulation_line)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

include_directories(
${EIGEN3_INCLUDE_DIR}
${CERES_INCLUDE_DIRS} # for both sophus and geographiclib
${OPENCV_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/src
)

LIST(APPEND LINK_LIBS
${OpenCV_LIBS}
${CERES_LIBRARIES}
)

add_executable(line_optlimization

  app/line_optlimization.cpp
  src/utils/line_geometry.cpp
  src/utils/line_manager.cpp
  src/utils/utility.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/plucker_line_parameterization.cpp
  src/factor/plucker_projection_factor.cpp
  src/factor/general_line_factor.cpp
  )

target_link_libraries(
  line_optlimization
   ${LINK_LIBS})

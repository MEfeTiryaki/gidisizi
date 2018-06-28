cmake_minimum_required(VERSION 2.8.3)
project(gidisizi)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

# to use eigen library
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  	include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
  DEPENDS
)

include_directories(
  include
	${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
add_library(
  ${PROJECT_NAME}
  src/Node.cpp
  src/RRT.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(iyileme_test
  test/test.cpp
)
add_dependencies(iyileme_test
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(iyileme_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

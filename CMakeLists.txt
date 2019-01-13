cmake_minimum_required(VERSION 2.8.3)
project(gidisizi)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

# to use eigen library
find_package(Eigen3 REQUIRED)
find_package(OpenCV)


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
  ${OpenCV_INCLUDE_DIRS}
)

#add_library(
#  ${PROJECT_NAME}
#)

#add_dependencies(${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)


add_executable(gidisizi_test
  test/test.cpp
)
add_dependencies(gidisizi_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gidisizi_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(rrt
  test/testRRT.cpp
)
add_dependencies(rrt
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rrt
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(rrt_star
  test/testRRTStar.cpp
)
add_dependencies(rrt_star
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rrt_star
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(rrt_rewired
  test/testRRTRewired.cpp
)
add_dependencies(rrt_rewired
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rrt_rewired
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(rrt_ss
  test/testRRTSS.cpp
)
add_dependencies(rrt_ss
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rrt_ss
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(rrt_car
  test/testRRTNonHolonomic.cpp
)
add_dependencies(rrt_car
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rrt_car
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

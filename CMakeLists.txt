cmake_minimum_required(VERSION 3.0.2)
project(ground_segmenter)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")  

if (BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
else()
  add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ground_segmenter
 CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} 
  app/segmenter_test.cpp 
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)
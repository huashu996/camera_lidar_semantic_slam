cmake_minimum_required(VERSION 2.8.3)
project(kittivelo_cam)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    message_filters
    roscpp
    rospy
    std_msgs
    tf
    pcl_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colored_pointcloud
#  CATKIN_DEPENDS cv_bridge image_transport message_filters roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# opencv
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context



add_executable(kittivelo_cam src/kittivelo_cam.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(kittivelo_cam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})









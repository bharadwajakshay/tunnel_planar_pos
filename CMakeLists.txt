cmake_minimum_required(VERSION 2.8.3)
project(tunnel_planar_pos)


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
    message("The -std=c++11 flags have been set")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
    message("The -std=c++0x flags have been set")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  xsens_slim
)

catkin_package(
	CATKIN_DEPENDS 
	roscpp
	rospy
	std_msgs
	pcl_conversions
	pcl_ros
	custom_msgs
	xsens_slim
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
 # ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tunnel_planar_pos src/tunnel_planar_pos.cpp src/tunnel_planar_pos_utils.cpp src/image_processing_tools.cpp)
set_target_properties(tunnel_planar_pos PROPERTIES COMPILE_FLAGS "-O3")
target_link_libraries(tunnel_planar_pos ${catkin_LIBRARIES} ${PCL_LIBRARIES})

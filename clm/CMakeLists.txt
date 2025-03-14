cmake_minimum_required(VERSION 3.0.2)
project(clm)
add_definitions(-fpermissive -g)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -w ")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
  cmake_modules
  image_transport
  pcl_conversions  
)
find_package (PCL REQUIRED)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/Extrinsic.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS TinyXML
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${TinyXML_INCLUDE_DIRS}
)

#link_directories()

#add_definitions()

## Declare cpp executables
add_executable(clm src/clm.cpp)
## as an example, message headers may need to be generated before nodes

add_dependencies(clm
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries( clm
			${PCL_LIBRARIES} 
			${OpenCV_LIBS} 
			${catkin_LIBRARIES}
			${CERES_LIBRARIES})

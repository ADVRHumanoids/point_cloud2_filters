cmake_minimum_required(VERSION 3.7.2)
project(point_cloud2_filters)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    pluginlib
    filters
    tf2_ros 
    pcl_conversions
    pcl_ros
    sensor_msgs
)

## System dependencies are found with CMake's conventions
#find_package(PCL REQUIRED COMPONENTS common filters)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_pass_through_filter_point_cloud2
   CATKIN_DEPENDS roscpp pluginlib filters tf2_ros pcl_ros pcl_conversions sensor_msgs
   #DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pass_through_filter_point_cloud2
  src/PassThroughFilterPointCloud2.cpp
)

target_link_libraries(pass_through_filter_point_cloud2
    ${catkin_LIBRARIES}
    #${PCL_LIBRARIES}
)
add_dependencies(pass_through_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS pass_through_filter_point_cloud2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install plugins xml file
install(FILES point_cloud2_filters_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pass_through_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 3.10.2)
project(point_cloud2_filters)

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
    dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/FilterBasePointCloud2.cfg
  cfg/FilterIndicesPointCloud2.cfg
  cfg/PassThroughPointCloud2.cfg
  cfg/CropBoxPointCloud2.cfg
  cfg/VoxelGridPointCloud2.cfg
  cfg/SacSegmentationExtractPointCloud2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(filter_base_point_cloud2
  src/FilterBasePointCloud2.cpp
)
target_link_libraries(filter_base_point_cloud2
    ${catkin_LIBRARIES}
)
add_dependencies(filter_base_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(filter_base_point_cloud2)

add_library(filter_point_cloud2
  src/FilterPointCloud2.cpp
)
target_link_libraries(filter_point_cloud2
    ${catkin_LIBRARIES}
    filter_base_point_cloud2
)
add_dependencies(filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
class_loader_hide_library_symbols(filter_point_cloud2)

add_library(filter_indices_point_cloud2
  src/FilterIndicesPointCloud2.cpp
)
target_link_libraries(filter_indices_point_cloud2
    ${catkin_LIBRARIES}
    filter_point_cloud2
)
add_dependencies(filter_indices_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(filter_indices_point_cloud2)

add_library(pass_through_filter_point_cloud2
  src/PassThroughFilterPointCloud2.cpp
)
target_link_libraries(pass_through_filter_point_cloud2
    ${catkin_LIBRARIES}
)
add_dependencies(pass_through_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(pass_through_filter_point_cloud2)

add_library(crop_box_filter_point_cloud2
  src/CropBoxFilterPointCloud2.cpp
)

target_link_libraries(crop_box_filter_point_cloud2
    ${catkin_LIBRARIES}
    filter_point_cloud2
)
add_dependencies(crop_box_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(crop_box_filter_point_cloud2)

add_library(voxel_grid_filter_point_cloud2
  src/VoxelGridFilterPointCloud2.cpp
)

target_link_libraries(voxel_grid_filter_point_cloud2
    ${catkin_LIBRARIES}
    filter_point_cloud2
)
add_dependencies(voxel_grid_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(voxel_grid_filter_point_cloud2)

add_library(sac_segmentation_extract_filter_point_cloud2
  src/SacSegmentationExtractFilterPointCloud2.cpp
)

target_link_libraries(sac_segmentation_extract_filter_point_cloud2
    ${catkin_LIBRARIES}
    filter_base_point_cloud2
)
add_dependencies(sac_segmentation_extract_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(sac_segmentation_extract_filter_point_cloud2)

#############
## Install ##
#############

install(TARGETS 
  filter_base_point_cloud2 
  filter_point_cloud2 
  filter_indices_point_cloud2
  pass_through_filter_point_cloud2 
  crop_box_filter_point_cloud2 
  voxel_grid_filter_point_cloud2
  sac_segmentation_extract_filter_point_cloud2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install plugins xml file
install(FILES point_cloud2_filters_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
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

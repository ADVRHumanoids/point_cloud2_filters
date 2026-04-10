cmake_minimum_required(VERSION 3.10.2)
project(point_cloud2_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(pluginlib REQUIRED)
find_package(filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)

find_package(Eigen3 REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/FilterBasePointCloud2.cfg
#   cfg/FilterIndicesPointCloud2.cfg
#   cfg/PassThroughPointCloud2.cfg
#   cfg/CropBoxPointCloud2.cfg
#   cfg/VoxelGridPointCloud2.cfg
#   cfg/SacSegmentationExtractPointCloud2.cfg
# )

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_filter_base_point_cloud2
  src/FilterBasePointCloud2.cpp
)
ament_target_dependencies(${PROJECT_NAME}_filter_base_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
#add_dependencies(${PROJECT_NAME}_filter_base_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#class_loader_hide_library_symbols(${PROJECT_NAME}_filter_base_point_cloud2)

add_library(${PROJECT_NAME}_filter_point_cloud2
  src/FilterPointCloud2.cpp
)
ament_target_dependencies(${PROJECT_NAME}_filter_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_filter_point_cloud2
    ${PROJECT_NAME}_filter_base_point_cloud2
)
#add_dependencies(${PROJECT_NAME}_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#class_loader_hide_library_symbols(${PROJECT_NAME}_filter_point_cloud2)

add_library(${PROJECT_NAME}_filter_indices_point_cloud2
  src/FilterIndicesPointCloud2.cpp
)
ament_target_dependencies(${PROJECT_NAME}_filter_indices_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_filter_indices_point_cloud2
    ${PROJECT_NAME}_filter_base_point_cloud2
)
#add_dependencies(${PROJECT_NAME}_filter_indices_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#class_loader_hide_library_symbols(${PROJECT_NAME}_filter_indices_point_cloud2)

add_library(${PROJECT_NAME}_pass_through_filter_point_cloud2
  src/PassThroughFilterPointCloud2.cpp
)
ament_target_dependencies(${PROJECT_NAME}_pass_through_filter_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_pass_through_filter_point_cloud2
    ${PROJECT_NAME}_filter_point_cloud2
)
#add_dependencies(${PROJECT_NAME}_pass_through_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#class_loader_hide_library_symbols(${PROJECT_NAME}_pass_through_filter_point_cloud2)

add_library(${PROJECT_NAME}_crop_box_filter_point_cloud2
  src/CropBoxFilterPointCloud2.cpp
)

ament_target_dependencies(${PROJECT_NAME}_crop_box_filter_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_crop_box_filter_point_cloud2
    ${PROJECT_NAME}_filter_point_cloud2
)
#add_dependencies(${PROJECT_NAME}_crop_box_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#class_loader_hide_library_symbols(${PROJECT_NAME}_crop_box_filter_point_cloud2)

add_library(${PROJECT_NAME}_voxel_grid_filter_point_cloud2
  src/VoxelGridFilterPointCloud2.cpp
)

ament_target_dependencies(${PROJECT_NAME}_voxel_grid_filter_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_voxel_grid_filter_point_cloud2
    ${PROJECT_NAME}_filter_point_cloud2
)
#add_dependencies(${PROJECT_NAME}_voxel_grid_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#class_loader_hide_library_symbols(${PROJECT_NAME}_voxel_grid_filter_point_cloud2)

add_library(${PROJECT_NAME}_sac_segmentation_extract_filter_point_cloud2
  src/SacSegmentationExtractFilterPointCloud2.cpp
)

ament_target_dependencies(${PROJECT_NAME}_sac_segmentation_extract_filter_point_cloud2
    rclcpp
    filters
    tf2_ros
    pcl_conversions
    pcl_ros
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_sac_segmentation_extract_filter_point_cloud2
    ${PROJECT_NAME}_filter_indices_point_cloud2
)
#add_dependencies(${PROJECT_NAME}_sac_segmentation_extract_filter_point_cloud2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#class_loader_hide_library_symbols(${PROJECT_NAME}_sac_segmentation_extract_filter_point_cloud2)

#############
## Linters ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(TARGETS 
  ${PROJECT_NAME}_filter_base_point_cloud2 
  ${PROJECT_NAME}_filter_point_cloud2 
  ${PROJECT_NAME}_filter_indices_point_cloud2
  ${PROJECT_NAME}_pass_through_filter_point_cloud2 
  ${PROJECT_NAME}_crop_box_filter_point_cloud2 
  ${PROJECT_NAME}_voxel_grid_filter_point_cloud2
  ${PROJECT_NAME}_sac_segmentation_extract_filter_point_cloud2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugins xml file
install(FILES point_cloud2_filters_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

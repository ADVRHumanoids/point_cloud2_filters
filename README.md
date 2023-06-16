# point_cloud_2_filters

Wrappers of pcl filters for `sensor_msgs/PointCloud2` ROS messages. The implementation and usage is based on the [filter](https://wiki.ros.org/filters) and [sensor_filter](https://wiki.ros.org/sensor_filters) packages, so it is different from the PCL filters provided by package [pcl_ros](https://wiki.ros.org/pcl_ros/Tutorials/filters) 

No ROS2 version (yet).

## Filters list
### PassThrough
Wrapper for the [pcl::PassThrough](https://pointclouds.org/documentation/classpcl_1_1_pass_through_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html) filter 
#### Params
-   `keep_organized`*(bool, default: true)* Keep the point cloud organized ([`pcl::FilterIndices<PointT>::setKeepOrganized	(bool keep_organized)`](https://pointclouds.org/documentation/classpcl_1_1_filter_indices.html#a21eb00357056c0cc432cd03afa84d08c)
-   `filter_field_name`*(str, default: z)* The field to be used for filtering data
-   `filter_limit_min`*(double, default: 0)* The minimum allowed field value a point will be considered
-   `filter_limit_min`*(double, default: 1)* The maximum allowed field value a point will be considered

### CropBox
Wrapper for the [pcl::CropBox](https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html) filter.  
**Warning** `pcl::CrobBox` parameter `keep_organized` is broken on ROS melodic (on noetic it is ok).
#### Params
See [pcl_ros crob box](https://wiki.ros.org/pcl_ros/Tutorials/filters#CropBox) params as they are the same

### VoxelGrid
Wrapper for the [pcl::VoxelGrid](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html) filter.  
**Warning** `pcl::CrobBox` parameter `keep_organized` is broken on ROS melodic (on noetic it is ok).
#### Params
See [pcl_ros crob box](https://wiki.ros.org/pcl_ros/Tutorials/filters#CropBox) params as they are the same

## Usage example
See launch and config folders

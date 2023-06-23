# point_cloud_2_filters
Wrappers for some of the [pcl filters](https://pointclouds.org/documentation/group__filters.html) for `sensor_msgs/PointCloud2` ROS messages. The implementation and usage is based on the [filter](https://wiki.ros.org/filters) and [sensor_filter](https://wiki.ros.org/sensor_filters) packages, so it is different from the wrappers of the PCL filters provided by the package [pcl_ros](https://wiki.ros.org/pcl_ros/Tutorials/filters).

No ROS2 version (yet).

## Usage example
See launch and config folders

## Filters list
### PassThrough
Wrapper for the [pcl::PassThrough](https://pointclouds.org/documentation/classpcl_1_1_pass_through_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html) filter 
#### Params
-  `input_frame`*(str, default: "")* The input TF frame the data should be transformed into before processing
-  `output_frame`*(str, default: "")* The output TF frame the data should be transformed into after processing
-  `keep_organized`*(bool, default: true)* Keep the point cloud organized ([`pcl::FilterIndices<PointT>::setKeepOrganized	(bool keep_organized)`](https://pointclouds.org/documentation/classpcl_1_1_filter_indices.html#a21eb00357056c0cc432cd03afa84d08c)
-  `negative`*(bool, default: false)* Set to true to return the data outside the min max limits
-  `filter_field_name`*(str, default: z)* The field to be used for filtering data
-  `filter_limit_min`*(double, default: 0)* The minimum allowed field value a point will be considered
-  `filter_limit_min`*(double, default: 1)* The maximum allowed field value a point will be considered

### VoxelGrid
Wrapper for the [pcl::VoxelGrid](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html) filter.  
#### Params
-  `input_frame`*(str, default: "")* The input TF frame the data should be transformed into before processing
-  `output_frame`*(str, default: "")* The output TF frame the data should be transformed into after processing
-  `negative`*(bool, default: false)* Set to true to return the data outside the min max limits
-  `leaf_size`*(double[3], default: [0.01, 0.01, 0.01])* The size of a leaf (on x,y,z) used for downsampling. Range: 0.0 to 1.0
-  `min_points_per_voxel`*(int, default:0)* et the minimum number of points required for a voxel to be used
-  `downsample_all_data`*(int, default:0)* Set to true if all fields need to be downsampled, or false if just XYZ
-  `filter_field_name`*(str, default: z)* The field to be used for filtering data, acting like a passthrough
-  `filter_limit_min`*(double, default: 0)* The minimum allowed field value a point will be considered
-  `filter_limit_min`*(double, default: 1)* The maximum allowed field value a point will be considered

### CropBox
Wrapper for the [pcl::CropBox](https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html) filter.  
**Warning** `pcl::CrobBox` parameter `keep_organized` is broken on ROS melodic (on noetic it is ok).
#### Params
-   `keep_organized`*(bool, default: true)* Keep the point cloud organized ([`pcl::FilterIndices<PointT>::setKeepOrganized	(bool keep_organized)`](https://pointclouds.org/documentation/classpcl_1_1_filter_indices.html#a21eb00357056c0cc432cd03afa84d08c)
-  `input_frame`*(str, default: "")* The input TF frame the data should be transformed into before processing
-  `output_frame`*(str, default: "")* The output TF frame the data should be transformed into after processing
-   `keep_organized`*(bool, default: true)* Keep the point cloud organized ([`pcl::FilterIndices<PointT>::setKeepOrganized	(bool keep_organized)`](https://pointclouds.org/documentation/classpcl_1_1_filter_indices.html#a21eb00357056c0cc432cd03afa84d08c)
-  `negative`*(bool, default: false)* Set to true to return the data outside the min max limits
-  `min_x`*(double, default: -1.0)* The minimum allowed x value a point will be considered from. Range: -1000.0 to 1000.0
-  `max_x`*(double, default: -1.0)* The maximum allowed x value a point will be considered from. Range: -1000.0 to 1000.0
-  `min_y`*(double, default: -1.0)* The minimum allowed y value a point will be considered from. Range: -1000.0 to 1000.0
-  `max_y`*(double, default: -1.0)* The maximum allowed y value a point will be considered from. Range: -1000.0 to 1000.0
-  `min_z`*(double, default: -1.0)* The minimum allowed z value a point will be considered from. Range: -1000.0 to 1000.0
-  `max_z`*(double, default: -1.0)* The maximum allowed z value a point will be considered from. Range: -1000.0 to 1000.0



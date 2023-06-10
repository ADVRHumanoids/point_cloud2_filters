# point_cloud_2_filters

Wrappers of pcl filters for `sensor_msgs/PointCloud2` ROS messages. The implementation and usage is based on the [filter](https://wiki.ros.org/filters) and [sensor_filter](https://wiki.ros.org/sensor_filters) packages, so it is different from the PCL filters provided by package [pcl_ros](https://wiki.ros.org/pcl_ros/Tutorials/filters) 

Despite the name, only one filter is available now. And also no ROS2 version (yet).

## Filters list
### PassThroughFilter
Wrapper for the [pcl::PassThrough](https://pointclouds.org/documentation/classpcl_1_1_pass_through_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html) filter 
#### Params
-   `reference_frame` the frame the limits refer to. The filters will tranform the point cloud wrt to this frame before passthroughing it.
-   `x_limits`, `x_limits`, `x_limits` The limits passed to  pcl::PassThroughsetFilterLimits.
-   `final_reference_frame` after passthroughing, the point cloud can be transformed again wrt to this frame.

All parameters are optional

## Usage example
See launch and config folders

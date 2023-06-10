#include <point_cloud2_filters/PassThroughFilterPointCloud2.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(point_cloud2_filters::PassThroughFilterPointCloud2, filters::FilterBase<sensor_msgs::PointCloud2>)

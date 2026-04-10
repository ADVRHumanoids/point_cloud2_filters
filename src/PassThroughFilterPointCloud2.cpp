#include <point_cloud2_filters/PassThroughFilterPointCloud2.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(point_cloud2_filters::PassThroughFilterPointCloud2, filters::FilterBase<sensor_msgs::msg::PointCloud2>)

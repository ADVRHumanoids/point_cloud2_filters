#include <pass_through_filter/PassThroughFilter.h>

using pass_through_filter::PassThroughFilter;

PassThroughFilter::PassThroughFilter() : sensor_filters::FilterChainNodelet<PointCloud>("point_cloud_pass_through_filter") {}


void PassThroughFilter::onInit()
{
    sensor_filters::FilterChainNodelet<PointCloud>::onInit();
   // custom nodelet code
}

bool PassThroughFilter::filter(const PointCloud& msgIn, PointCloud& msgOut)
{
    
    const PointCloud::ConstPtr cloud = boost::make_shared<PointCloud>(msgIn);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
   //pass.setNegative (true);
    pass.filter (msgOut);
    
    return sensor_filters::FilterChainNodelet<PointCloud>::filter(msgIn, msgOut);
}


PLUGINLIB_EXPORT_CLASS(PassThroughFilter, nodelet::Nodelet)

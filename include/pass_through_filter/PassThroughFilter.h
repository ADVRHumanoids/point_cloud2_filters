#ifndef PASS_THROUGH_FILTER_H
#define PASS_THROUGH_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_filters/FilterChainNodelet.h>

#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>


namespace pass_through_filter {
    
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class PassThroughFilter : public sensor_filters::FilterChainNodelet<PointCloud>
{
public:
  PassThroughFilter();

protected:
  void onInit() override;

  bool filter(const PointCloud& msgIn, PointCloud& msgOut) override;
  
private:
    PointCloud::Ptr _cloud_out;
};

} //namespace


#endif //PASS_THROUGH_FILTER_H

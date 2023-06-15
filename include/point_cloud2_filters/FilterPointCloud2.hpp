#ifndef FILTER_POINT_CLOUD_HPP
#define FILTER_POINT_CLOUD_HPP

#include <memory>

#include <ros/ros.h>

#if ROS_VERSION_MINIMUM(1, 15, 0)
    #include <filters/filter_base.hpp>
#else
    #include <filters/filter_base.h>
#endif
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

namespace point_cloud2_filters {
    
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class FilterPointCloud2 : public filters::FilterBase<sensor_msgs::PointCloud2>
{
public:
    FilterPointCloud2(const std::string& name);
    ~FilterPointCloud2();

public:
    virtual bool configure() = 0;

    /** \brief Update the filter and return the data seperately
    * \param data_in T array with length width
    * \param data_out T array with length width
    */
    virtual bool update( const sensor_msgs::PointCloud2& data_in, sensor_msgs::PointCloud2& data_out) override;
    
protected:
    const std::string name_;
    std::shared_ptr<pcl::Filter<Point>> filter_;

private:
    PointCloud::Ptr cloud_out_;
    
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string input_frame_ = "";
    std::string output_frame_ = "";
};

FilterPointCloud2::FilterPointCloud2(const std::string& name) : name_(name) {
    
    cloud_out_ = boost::make_shared<PointCloud>();
    
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    
};

FilterPointCloud2::~FilterPointCloud2()
{
};

bool FilterPointCloud2::configure()
{
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("input_frame"), input_frame_))
    {
        ROS_INFO_NAMED(name_, "Using input_frame='%s'", input_frame_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("output_frame"), output_frame_))
    {
        ROS_INFO_NAMED(name_, "Using output_frame='%s'", output_frame_.c_str());
    }
    
    
    return true;
    
};

bool FilterPointCloud2::update( const sensor_msgs::PointCloud2& data_in, sensor_msgs::PointCloud2& data_out)
{
    
    pcl::fromROSMsg(data_in, *cloud_out_);
    
    if (input_frame_.length() > 0) {
        
        pcl_ros::transformPointCloud (input_frame_, *cloud_out_, *cloud_out_, tf_buffer_);
    } 
    
    filter_->setInputCloud (cloud_out_);
    filter_->filter (*cloud_out_);

    if (output_frame_.length() > 0) {
        
        pcl_ros::transformPointCloud (output_frame_, *cloud_out_, *cloud_out_, tf_buffer_);
    } 
    
    pcl::toROSMsg(*cloud_out_, data_out);
    

    return true;
    
};



} //namespace point_cloud2_filters


#endif //FILTER_POINT_CLOUD_HPP


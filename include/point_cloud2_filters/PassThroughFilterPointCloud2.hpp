#ifndef PASS_THROUGH_FILTER_HPP
#define PASS_THROUGH_FILTER_HPP

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
#include <pcl/filters/passthrough.h>


namespace point_cloud2_filters {
    
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class PassThroughFilterPointCloud2 : public filters::FilterBase<sensor_msgs::PointCloud2>
{
public:
    PassThroughFilterPointCloud2();
    ~PassThroughFilterPointCloud2();

public:
    virtual bool configure() override;

    /** \brief Update the filter and return the data seperately
    * \param data_in T array with length width
    * \param data_out T array with length width
    */
    virtual bool update( const sensor_msgs::PointCloud2& data_in, sensor_msgs::PointCloud2& data_out) override;
  
private:
    PointCloud::Ptr cloud_out_;
    pcl::PassThrough<Point> pass_through_;
    
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool keep_organized_ = true;
    std::string reference_frame_ = "";
    std::string final_reference_frame_ = "";
    std::vector<double> x_limits_;
    std::vector<double> y_limits_;
    std::vector<double> z_limits_;

    
};

PassThroughFilterPointCloud2::PassThroughFilterPointCloud2() {
    
    cloud_out_ = boost::make_shared<PointCloud>();
    
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    
};

PassThroughFilterPointCloud2::~PassThroughFilterPointCloud2()
{
};


bool PassThroughFilterPointCloud2::configure()
{
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("keep_organized"), keep_organized_);
    pass_through_.setKeepOrganized(keep_organized_);
    ROS_INFO("PassThroughFilterPointCloud2: Keep Organized='%d'", keep_organized_);
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("reference_frame"), reference_frame_))
    {
        ROS_INFO("PassThroughFilterPointCloud2: Using reference_frame='%s'", reference_frame_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("x_limits"), x_limits_))
    {
        if (x_limits_.size() != 2) {
            ROS_ERROR("PassThroughFilterPointCloud2: x_limits argument not valid, please pass a vector with two elements");
        }
        
        ROS_INFO("PassThroughFilterPointCloud2: Using x_limits_='[%f, %f]'", x_limits_.at(0), x_limits_.at(1));
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("y_limits"), y_limits_))
    {
        if (y_limits_.size() != 2) {
            ROS_ERROR("PassThroughFilterPointCloud2: y_limits argument not valid, please pass a vector with two elements");
        }
        
        ROS_INFO("PassThroughFilterPointCloud2: Using y_limits_='[%f, %f]'", y_limits_.at(0), y_limits_.at(1));
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("z_limits"), z_limits_))
    {
        if (z_limits_.size() != 2) {
            ROS_ERROR("PassThroughFilterPointCloud2: z_limits argument not valid, please pass a vector with two elements");
        }
        
        ROS_INFO("PassThroughFilterPointCloud2: Using z_limits_='[%f, %f]'", z_limits_.at(0), z_limits_.at(1));
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("final_reference_frame"), final_reference_frame_))
    {
        ROS_INFO("PassThroughFilterPointCloud2: Using final_reference_frame='%s'", final_reference_frame_.c_str());
    }
    
    return true;
    
};

bool PassThroughFilterPointCloud2::update( const sensor_msgs::PointCloud2& data_in, sensor_msgs::PointCloud2& data_out)
{
    
    pcl::fromROSMsg(data_in, *cloud_out_);
    
    if (reference_frame_.length() > 0) {
        
        pcl_ros::transformPointCloud (reference_frame_, *cloud_out_, *cloud_out_, tf_buffer_);
    } 
    
    if (x_limits_.size() > 0) {
        pass_through_.setInputCloud (cloud_out_);
        pass_through_.setFilterFieldName ("x");
        pass_through_.setFilterLimits (x_limits_.at(0), x_limits_.at(1));
        //pass_through_.setNegative (true);
        pass_through_.filter (*cloud_out_);
    }
    if (y_limits_.size() > 0) {
        pass_through_.setInputCloud (cloud_out_);
        pass_through_.setFilterFieldName ("y");
        pass_through_.setFilterLimits (y_limits_.at(0), y_limits_.at(1));
        //pass_through_.setNegative (true);
        pass_through_.filter (*cloud_out_);
    }
    if (z_limits_.size() > 0) {
        pass_through_.setInputCloud (cloud_out_);
        pass_through_.setFilterFieldName ("z");
        pass_through_.setFilterLimits (z_limits_.at(0), z_limits_.at(1));
        //pass_through_.setNegative (true);
        pass_through_.filter (*cloud_out_);
    }
    
    
    if (final_reference_frame_.length() > 0) {
        
        pcl_ros::transformPointCloud (final_reference_frame_, *cloud_out_, *cloud_out_, tf_buffer_);
    } 
    
    pcl::toROSMsg(*cloud_out_, data_out);

    
    return true;
    
};



} //namespace point_cloud2_filters


#endif //PASS_THROUGH_FILTER_HPP

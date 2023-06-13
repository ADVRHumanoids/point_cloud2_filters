#ifndef CROP_BOX_FILTER_HPP
#define CROP_BOX_FILTER_HPP

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
#include <pcl/filters/crop_box.h>


namespace point_cloud2_filters {
    
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class CropBoxFilterPointCloud2 : public filters::FilterBase<sensor_msgs::PointCloud2>
{
public:
    CropBoxFilterPointCloud2();
    ~CropBoxFilterPointCloud2();

public:
    virtual bool configure() override;

    /** \brief Update the filter and return the data seperately
    * \param data_in T array with length width
    * \param data_out T array with length width
    */
    virtual bool update( const sensor_msgs::PointCloud2& data_in, sensor_msgs::PointCloud2& data_out) override;
  
private:
    PointCloud::Ptr cloud_out_;
    pcl::CropBox<Point> crop_box_;
    
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool keep_organized_ = false;
    std::string input_frame_ = "";
    std::string output_frame_ = "";
    bool negative_ = false;
    
    double min_x_, min_y_, min_z_ = -1;
    double max_x_, max_y_, max_z_ = 1;
};

CropBoxFilterPointCloud2::CropBoxFilterPointCloud2() {
    
    cloud_out_ = boost::make_shared<PointCloud>();
    
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    
};

CropBoxFilterPointCloud2::~CropBoxFilterPointCloud2()
{
};


bool CropBoxFilterPointCloud2::configure()
{
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("keep_organized"), keep_organized_);
    ROS_INFO("PassThroughFilterPointCloud2: Keep Organized='%d'", keep_organized_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO("PassThroughFilterPointCloud2: Negative='%d'", negative_);
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("input_frame"), input_frame_))
    {
        ROS_INFO("PassThroughFilterPointCloud2: Using input_frame='%s'", input_frame_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("output_frame"), output_frame_))
    {
        ROS_INFO("PassThroughFilterPointCloud2: Using output_frame='%s'", output_frame_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_x"), min_x_))
    {
        
        ROS_INFO("PassThroughFilterPointCloud2: Using min_x=%f", min_x_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_x"), max_x_))
    {
        
        ROS_INFO("PassThroughFilterPointCloud2: Using max_x=%f", max_x_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_y"), min_y_))
    {
        
        ROS_INFO("PassThroughFilterPointCloud2: Using min_y=%f", min_y_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_y"), max_y_))
    {
        
        ROS_INFO("PassThroughFilterPointCloud2: Using max_y=%f", max_y_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_z"), min_z_))
    {
        
        ROS_INFO("PassThroughFilterPointCloud2: Using min_z=%f", min_z_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_z"), max_z_))
    {
        
        ROS_INFO("PassThroughFilterPointCloud2: Using max_z=%f", max_z_);
    }
    
    Eigen::Vector4f min_point, max_point;
    min_point << min_x_, min_y_, min_z_, 0;
    max_point << max_x_, max_y_, max_z_, 0;
    
    crop_box_.setKeepOrganized(keep_organized_);
    crop_box_.setNegative(negative_);

    crop_box_.setMin(min_point);
    crop_box_.setMax(max_point);
    

    return true;
    
};

bool CropBoxFilterPointCloud2::update( const sensor_msgs::PointCloud2& data_in, sensor_msgs::PointCloud2& data_out)
{
    
    pcl::fromROSMsg(data_in, *cloud_out_);
    
    if (input_frame_.length() > 0) {
        
        pcl_ros::transformPointCloud (input_frame_, *cloud_out_, *cloud_out_, tf_buffer_);
    } 
    
    crop_box_.setInputCloud (cloud_out_);
    crop_box_.filter (*cloud_out_);

    if (output_frame_.length() > 0) {
        
        pcl_ros::transformPointCloud (output_frame_, *cloud_out_, *cloud_out_, tf_buffer_);
    } 
    
    pcl::toROSMsg(*cloud_out_, data_out);
    
    Eigen::Vector4f min_point, max_point;
    max_point = crop_box_.getMax();
    min_point =  crop_box_.getMin();

    return true;
    
};



} //namespace point_cloud2_filters


#endif //CROP_BOX_FILTER_HPP

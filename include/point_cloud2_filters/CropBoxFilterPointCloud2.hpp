#ifndef CROP_BOX_FILTER_HPP
#define CROP_BOX_FILTER_HPP

#include <point_cloud2_filters/FilterPointCloud2.hpp>
#include <pcl/filters/crop_box.h>


namespace point_cloud2_filters {

class CropBoxFilterPointCloud2 : public FilterPointCloud2
{
public:
    CropBoxFilterPointCloud2();
    ~CropBoxFilterPointCloud2();

public:
    virtual bool configure() override;
  
private:
    
    std::shared_ptr<pcl::CropBox<Point>> crop_box_;
    
    bool negative_ = false;
    bool keep_organized_ = true;
    //Provide a value that the filtered points should be set to instead of removing them
    double user_filter_value_ = std::numeric_limits<double>::quiet_NaN ();
    
    double min_x_, min_y_, min_z_ = -1;
    double max_x_, max_y_, max_z_ = 1;
};

CropBoxFilterPointCloud2::CropBoxFilterPointCloud2() : FilterPointCloud2("CropBoxFilterPointCloud2") {
    
    filter_ = std::make_shared<pcl::CropBox<Point>>();
    crop_box_ = std::dynamic_pointer_cast<pcl::CropBox<Point>>(filter_);
};

CropBoxFilterPointCloud2::~CropBoxFilterPointCloud2()
{
};


bool CropBoxFilterPointCloud2::configure()
{
    
    FilterPointCloud2::configure();
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("keep_organized"), keep_organized_);
    ROS_INFO_NAMED(name_, "[%s] Using keep organized='%d'", name_.c_str(), keep_organized_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO_NAMED(name_, "[%s] Using negative='%d'", name_.c_str(), negative_);
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("user_filter_value"), user_filter_value_)) {
        ROS_INFO_NAMED(name_, "[%s] Using user_filter_value='%f'", name_.c_str(), user_filter_value_);

    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_x"), min_x_))
    {
        
        ROS_INFO_NAMED(name_, "[%s] Using min_x=%f", name_.c_str(), min_x_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_x"), max_x_))
    {
        
        ROS_INFO_NAMED(name_, "[%s] Using max_x=%f", name_.c_str(), max_x_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_y"), min_y_))
    {
        
        ROS_INFO_NAMED(name_, "[%s] Using min_y=%f", name_.c_str(), min_y_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_y"), max_y_))
    {
        
        ROS_INFO_NAMED(name_, "[%s] Using max_y=%f", name_.c_str(), max_y_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_z"), min_z_))
    {
        
        ROS_INFO_NAMED(name_, "[%s] Using min_z=%f", name_.c_str(), min_z_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_z"), max_z_))
    {
        
        ROS_INFO_NAMED(name_, "[%s] Using max_z=%f", name_.c_str(), max_z_);
    }
    
    crop_box_->setKeepOrganized(keep_organized_);
    crop_box_->setNegative(negative_);
    
    if (user_filter_value_ != std::numeric_limits<double>::quiet_NaN ()) {
        crop_box_->setUserFilterValue(user_filter_value_);
    }
    
    Eigen::Vector4f min_point, max_point;
    min_point << min_x_, min_y_, min_z_, 0;
    max_point << max_x_, max_y_, max_z_, 0;

    crop_box_->setMin(min_point);
    crop_box_->setMax(max_point);
    

    return true;
    
};


} //namespace point_cloud2_filters


#endif //CROP_BOX_FILTER_HPP

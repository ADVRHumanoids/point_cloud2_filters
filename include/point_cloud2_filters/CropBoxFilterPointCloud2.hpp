#ifndef CROP_BOX_FILTER_HPP
#define CROP_BOX_FILTER_HPP

#include <point_cloud2_filters/FilterIndicesPointCloud2.hpp>
#include <pcl/filters/crop_box.h>

#include <point_cloud2_filters/CropBoxPointCloud2Config.h>

namespace point_cloud2_filters {

class CropBoxFilterPointCloud2 : public FilterIndicesPointCloud2
{
public:
    CropBoxFilterPointCloud2();
    ~CropBoxFilterPointCloud2();

public:
    virtual bool configure() override;
  
private:
    
    std::shared_ptr<pcl::CropBox<Point>> crop_box_;
    
    double min_x_, min_y_, min_z_ = -1;
    double max_x_, max_y_, max_z_ = 1;
    
    /** \brief Pointer to a dynamic reconfigure service. */
    std::unique_ptr<dynamic_reconfigure::Server<point_cloud2_filters::CropBoxPointCloud2Config>> dynamic_reconfigure_srv_;
    dynamic_reconfigure::Server<point_cloud2_filters::CropBoxPointCloud2Config>::CallbackType dynamic_reconfigure_clbk_;
    void dynamicReconfigureClbk(point_cloud2_filters::CropBoxPointCloud2Config &config, uint32_t level);
    boost::recursive_mutex dynamic_reconfigure_mutex_;
};

CropBoxFilterPointCloud2::CropBoxFilterPointCloud2() : FilterIndicesPointCloud2() {
    
    filter_ = std::make_shared<pcl::CropBox<Point>>();
};

CropBoxFilterPointCloud2::~CropBoxFilterPointCloud2()
{
};


bool CropBoxFilterPointCloud2::configure()
{

    FilterIndicesPointCloud2::configure();
    
    crop_box_ = std::dynamic_pointer_cast<pcl::CropBox<Point>>(filter_);
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_x"), min_x_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using min_x=%f", getName().c_str(), min_x_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_x"), max_x_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using max_x=%f", getName().c_str(), max_x_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_y"), min_y_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using min_y=%f", getName().c_str(), min_y_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_y"), max_y_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using max_y=%f", getName().c_str(), max_y_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_z"), min_z_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using min_z=%f", getName().c_str(), min_z_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_z"), max_z_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using max_z=%f", getName().c_str(), max_z_);
    }
    
    Eigen::Vector4f min_point, max_point;
    min_point << min_x_, min_y_, min_z_, 0;
    max_point << max_x_, max_y_, max_z_, 0;

    crop_box_->setMin(min_point);
    crop_box_->setMax(max_point);
    
    //dynamic reconfigure
    dynamic_reconfigure_srv_ = std::make_unique<dynamic_reconfigure::Server<point_cloud2_filters::CropBoxPointCloud2Config>>(
        dynamic_reconfigure_mutex_,
        ros::NodeHandle( dynamic_reconfigure_namespace_root_ + "/" + getName()));
    
    dynamic_reconfigure_clbk_ = boost::bind(&CropBoxFilterPointCloud2::dynamicReconfigureClbk, this, _1, _2);

    point_cloud2_filters::CropBoxPointCloud2Config initial_config;
    initial_config.min_x = min_x_;
    initial_config.max_x = max_x_;
    initial_config.min_y = min_y_;
    initial_config.max_y = max_y_;
    initial_config.min_z = min_z_;
    initial_config.max_z = max_z_;

    dynamic_reconfigure_srv_->setConfigDefault(initial_config);
    dynamic_reconfigure_srv_->updateConfig(initial_config);
    
    //put this after updateConfig!
    dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_clbk_);

    return true;
    
};

void CropBoxFilterPointCloud2::dynamicReconfigureClbk (point_cloud2_filters::CropBoxPointCloud2Config &config, uint32_t /*level*/)
{

    boost::recursive_mutex::scoped_lock lock(dynamic_reconfigure_mutex_);
    bool max_to_update = false;
    bool min_to_update = false;
    
    if (min_x_ != config.min_x)
    {
        min_to_update = true;
        min_x_ = config.min_x;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting min_x to: %f.", getName().c_str(), min_x_);
    }
    if (min_y_ != config.min_y)
    {
        min_to_update = true;
        min_y_ = config.min_y;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting min_y to: %f.", getName().c_str(), min_y_);
    }
    if (min_z_ != config.min_z)
    {
        min_to_update = true;
        min_z_ = config.min_z;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting min_z to: %f.", getName().c_str(), min_z_);
    }
    
    if (max_x_ != config.max_x)
    {
        max_to_update = true;
        max_x_ = config.max_x;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting max_x to: %f.", getName().c_str(), max_x_);
    }
    if (max_y_ != config.max_y)
    {
        max_to_update = true;
        max_y_ = config.max_y;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting max_y to: %f.", getName().c_str(), max_y_);
    }
    if (max_z_ != config.max_z)
    {
        max_to_update = true;
        max_z_ = config.max_z;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting max_z to: %f.", getName().c_str(), max_z_);
    }
    
    if (min_to_update) {
        Eigen::Vector4f min_point;
        min_point << min_x_, min_y_, min_z_, 0;
        crop_box_->setMin(min_point);
    }
    if (max_to_update) {
        Eigen::Vector4f max_point;
        max_point << max_x_, max_y_, max_z_, 0;
        crop_box_->setMax(max_point);
    }
}


} //namespace point_cloud2_filters


#endif //CROP_BOX_FILTER_HPP

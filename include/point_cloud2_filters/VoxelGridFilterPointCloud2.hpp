#ifndef VOXEL_GRID_FILTER_HPP
#define VOXEL_GRID_FILTER_HPP

#include <point_cloud2_filters/FilterPointCloud2.hpp>
#include <pcl/filters/voxel_grid.h>

#include <point_cloud2_filters/VoxelGridPointCloud2Config.h>

namespace point_cloud2_filters {

class VoxelGridFilterPointCloud2 : public FilterPointCloud2
{
public:
    VoxelGridFilterPointCloud2();
    ~VoxelGridFilterPointCloud2();

public:
    virtual bool configure() override;

private:
    std::shared_ptr<pcl::VoxelGrid<Point>> voxel_grid_;

    double leaf_size_x_, leaf_size_y_, leaf_size_z_ = 0.01;
    unsigned int min_points_per_voxel_ = 0;
    bool downsample_all_data_ = true;
    std::string filter_field_name_ = "";
    double filter_limit_min_ = -1000;
    double filter_limit_max_ = 1000;
    bool negative_ = false;
    
    /** \brief Pointer to a dynamic reconfigure service. */
    std::unique_ptr<dynamic_reconfigure::Server<point_cloud2_filters::VoxelGridPointCloud2Config>> dynamic_reconfigure_srv_;
    dynamic_reconfigure::Server<point_cloud2_filters::VoxelGridPointCloud2Config>::CallbackType dynamic_reconfigure_clbk_;
    void dynamicReconfigureClbk(point_cloud2_filters::VoxelGridPointCloud2Config &config, uint32_t level);
    boost::recursive_mutex dynamic_reconfigure_mutex_;

};

VoxelGridFilterPointCloud2::VoxelGridFilterPointCloud2() : FilterPointCloud2() {
    
    filter_ = std::make_shared<pcl::VoxelGrid<Point>>();
    
};

VoxelGridFilterPointCloud2::~VoxelGridFilterPointCloud2()
{
};


bool VoxelGridFilterPointCloud2::configure()
{
    
    FilterPointCloud2::configure();
    
    voxel_grid_ = std::dynamic_pointer_cast<pcl::VoxelGrid<Point>>(filter_);

    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("leaf_size_x"), leaf_size_x_);
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("leaf_size_y"), leaf_size_y_);
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("leaf_size_z"), leaf_size_z_);

    ROS_INFO_NAMED(getName(), "[%s] Using leaf_size='[%f, %f, %f]'", getName().c_str(), leaf_size_x_, leaf_size_y_, leaf_size_z_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_points_per_voxel"), min_points_per_voxel_);
    ROS_INFO_NAMED(getName(), "[%s] Using min_points_per_voxel=%d", getName().c_str(), min_points_per_voxel_);

    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("downsample_all_data"), downsample_all_data_);
    ROS_INFO_NAMED(getName(), "[%s] Using downsample_all_data=%d", getName().c_str(), downsample_all_data_);

    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_field_name"), filter_field_name_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using filter_field_name=%s", getName().c_str(), filter_field_name_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_min"), filter_limit_min_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using filter_limit_min=%f", getName().c_str(), filter_limit_min_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_max"), filter_limit_max_))
    {
        
        ROS_INFO_NAMED(getName(), "[%s] Using filter_limit_max=%f", getName().c_str(), filter_limit_max_);
    }
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO_NAMED(getName(), "[%s] Using negative='%d'", getName().c_str(), negative_);
    
    voxel_grid_->setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    voxel_grid_->setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
    voxel_grid_->setDownsampleAllData(downsample_all_data_);
    
    if (filter_field_name_.length() > 0) {
        voxel_grid_->setFilterFieldName(filter_field_name_);
        voxel_grid_->setFilterLimits(filter_limit_min_, filter_limit_max_);
    }
    voxel_grid_->setFilterLimitsNegative(negative_);
    
    //dynamic reconfigure
    dynamic_reconfigure_srv_ = std::make_unique<dynamic_reconfigure::Server<point_cloud2_filters::VoxelGridPointCloud2Config>>(
        dynamic_reconfigure_mutex_,
        ros::NodeHandle( dynamic_reconfigure_namespace_root_ + "/" + getName()));
    
    dynamic_reconfigure_clbk_ = boost::bind(&VoxelGridFilterPointCloud2::dynamicReconfigureClbk, this, _1, _2);

    point_cloud2_filters::VoxelGridPointCloud2Config initial_config;
    initial_config.negative = negative_;
    initial_config.leaf_size_x = leaf_size_x_;
    initial_config.leaf_size_y = leaf_size_y_;
    initial_config.leaf_size_z = leaf_size_z_;
    initial_config.min_points_per_voxel = min_points_per_voxel_;
    initial_config.downsample_all_data = downsample_all_data_;
    initial_config.filter_field_name = filter_field_name_;
    initial_config.filter_limit_min = filter_limit_min_;
    initial_config.filter_limit_max = filter_limit_max_;

    dynamic_reconfigure_srv_->setConfigDefault(initial_config);
    dynamic_reconfigure_srv_->updateConfig(initial_config);
    
    //put this after updateConfig!
    dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_clbk_);

    return true;
    
};

void VoxelGridFilterPointCloud2::dynamicReconfigureClbk (point_cloud2_filters::VoxelGridPointCloud2Config &config, uint32_t /*level*/)
{

    boost::recursive_mutex::scoped_lock lock(dynamic_reconfigure_mutex_);
    bool to_update_limits = false;
    bool to_update_leaf_size = false;
    
    if (leaf_size_x_ != config.leaf_size_x)
    {
        leaf_size_x_ = config.leaf_size_x;
        to_update_leaf_size = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting leaf_size_x to: %f.", getName().c_str(), leaf_size_x_);
    }
    if (leaf_size_y_ != config.leaf_size_y)
    {
        leaf_size_y_ = config.leaf_size_y;
        to_update_leaf_size = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting leaf_size_y to: %f.", getName().c_str(), leaf_size_y_);
    }
    if (leaf_size_z_ != config.leaf_size_z)
    {
        leaf_size_z_ = config.leaf_size_z;
        to_update_leaf_size = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting leaf_size_z to: %f.", getName().c_str(), leaf_size_z_);
    }
    
    if (min_points_per_voxel_ != config.min_points_per_voxel)
    {
        min_points_per_voxel_ = config.min_points_per_voxel;
        voxel_grid_->setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting min_points_per_voxel to: %d.", getName().c_str(), min_points_per_voxel_);
    }
    
    if (downsample_all_data_ != config.downsample_all_data)
    {
        downsample_all_data_ = config.downsample_all_data;
        voxel_grid_->setDownsampleAllData(downsample_all_data_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting downsample_all_data to: %d.", getName().c_str(), downsample_all_data_);
    }
    if (filter_field_name_.compare(config.filter_field_name) != 0)
    {
        filter_field_name_ = config.filter_field_name;
        voxel_grid_->setFilterFieldName(filter_field_name_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting filter_field_name to: %s.", getName().c_str(), filter_field_name_.c_str());
    }
    if (filter_limit_min_ != config.filter_limit_min)
    {
        filter_limit_min_ = config.filter_limit_min;
        to_update_limits = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting filter_limit_min to: %f.", getName().c_str(), filter_limit_min_);
    }
    if (filter_limit_max_ != config.filter_limit_max)
    {
        filter_limit_max_ = config.filter_limit_max;
        to_update_limits = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting filter_limit_max to: %f.", getName().c_str(), filter_limit_max_);
    }
    if (negative_ != config.negative)
    {
        negative_ = config.negative;
        voxel_grid_->setFilterLimitsNegative(negative_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting negative to: %d.", getName().c_str(), negative_);
    }
    
    
    if (to_update_limits) {
        voxel_grid_->setFilterLimits(filter_limit_min_, filter_limit_max_);
    }
    if (to_update_leaf_size) {
        voxel_grid_->setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    }
    


}


} //namespace point_cloud2_filters


#endif //VOXEL_GRID_FILTER_HPP

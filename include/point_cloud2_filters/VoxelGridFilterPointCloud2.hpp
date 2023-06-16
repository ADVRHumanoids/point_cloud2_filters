#ifndef VOXEL_GRID_FILTER_HPP
#define VOXEL_GRID_FILTER_HPP

#include <point_cloud2_filters/FilterPointCloud2.hpp>
#include <pcl/filters/voxel_grid.h>


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

    bool negative_ = false;

    std::vector<double> leaf_size_ = {0.01, 0.01, 0.01};
    unsigned int min_points_per_voxel_ = 0;
    bool downsample_all_data_ = true;
    std::string filter_field_name_ = "";
    double filter_limit_min_ = -1000;
    double filter_limit_max_ = 1000;
};

VoxelGridFilterPointCloud2::VoxelGridFilterPointCloud2() : FilterPointCloud2("VoxelGridFilterPointCloud2") {
    
    filter_ = std::make_shared<pcl::VoxelGrid<Point>>();
    voxel_grid_ = std::dynamic_pointer_cast<pcl::VoxelGrid<Point>>(filter_);
    
};

VoxelGridFilterPointCloud2::~VoxelGridFilterPointCloud2()
{
};


bool VoxelGridFilterPointCloud2::configure()
{
    
    FilterPointCloud2::configure();

    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO_NAMED(name_, "Negative='%d'", negative_);

    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("leaf_size"), leaf_size_))
    {
        if (leaf_size_.size() != 3) {
            ROS_ERROR_NAMED(name_, "leaf_size argument not valid, please pass a vector with three elements");
            return false;
        }
    }
    ROS_INFO_NAMED(name_, "Using leaf_size='[%f, %f, %f]'", leaf_size_.at(0), leaf_size_.at(1), leaf_size_.at(2));
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_points_per_voxel"), min_points_per_voxel_))
    {
        
        ROS_INFO_NAMED(name_, "Using min_points_per_voxel=%d", min_points_per_voxel_);
    }

    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("downsample_all_data"), downsample_all_data_))
    {
        
        ROS_INFO_NAMED(name_, "Using downsample_all_data=%d", downsample_all_data_);
    }

    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_field_name"), filter_field_name_))
    {
        
        ROS_INFO_NAMED(name_, "Using filter_field_name=%s", filter_field_name_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_min"), filter_limit_min_))
    {
        
        ROS_INFO_NAMED(name_, "Using filter_limit_min=%f", filter_limit_min_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_max"), filter_limit_max_))
    {
        
        ROS_INFO_NAMED(name_, "Using filter_limit_max=%f", filter_limit_max_);
    }
    
    voxel_grid_->setLeafSize(leaf_size_.at(0), leaf_size_.at(1), leaf_size_.at(2));
    voxel_grid_->setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
    voxel_grid_->setDownsampleAllData(downsample_all_data_);
    
    if (filter_field_name_.length() > 0) {
        voxel_grid_->setFilterFieldName(filter_field_name_);
        voxel_grid_->setFilterLimits(filter_limit_min_, filter_limit_max_);
    }
    voxel_grid_->setFilterLimitsNegative(negative_);

    return true;
    
};


} //namespace point_cloud2_filters


#endif //VOXEL_GRID_FILTER_HPP

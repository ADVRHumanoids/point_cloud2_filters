#ifndef FILTER_INDICES_POINT_CLOUD_HPP
#define FILTER_INDICES_POINT_CLOUD_HPP

#include <point_cloud2_filters/FilterPointCloud2.hpp>
#include <pcl/filters/filter_indices.h>

namespace point_cloud2_filters {

class FilterIndicesPointCloud2 : public FilterPointCloud2
{
public:
    FilterIndicesPointCloud2(const std::string& name);
    ~FilterIndicesPointCloud2();

public:
    virtual bool configure();

protected:
    std::shared_ptr<pcl::FilterIndices<Point>> filter_indices_;

private:
    bool negative_ = false;
    bool keep_organized_ = true;
    //Provide a value that the filtered points should be set to instead of removing them
    double user_filter_value_ = std::numeric_limits<double>::quiet_NaN ();
};

FilterIndicesPointCloud2::FilterIndicesPointCloud2(const std::string& name) : FilterPointCloud2(name) {
    
    filter_indices_ = std::dynamic_pointer_cast<pcl::FilterIndices<Point>>(filter_);
    
};

FilterIndicesPointCloud2::~FilterIndicesPointCloud2()
{
};

bool FilterIndicesPointCloud2::configure()
{
    FilterPointCloud2::configure();

    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("keep_organized"), keep_organized_);
    ROS_INFO_NAMED(name_, "Usin keep organized='%d'", keep_organized_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO_NAMED(name_, "Using negative='%d'", negative_);
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("user_filter_value"), user_filter_value_)) {
        ROS_INFO_NAMED(name_, "Using user_filter_value='%f'", user_filter_value_);

    }
    
    filter_indices_->setKeepOrganized(keep_organized_);
    filter_indices_->setNegative(negative_);
    
    if (user_filter_value_ != std::numeric_limits<double>::quiet_NaN ()) {
        filter_indices_->setUserFilterValue(user_filter_value_);
    }
    
    return true;
    
};


} //namespace point_cloud2_filters


#endif //FILTER_INDICES_POINT_CLOUD_HPP


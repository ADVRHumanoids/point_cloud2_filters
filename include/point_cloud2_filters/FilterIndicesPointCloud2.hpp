#ifndef FILTER_INDICES_POINT_CLOUD_HPP
#define FILTER_INDICES_POINT_CLOUD_HPP

#include <point_cloud2_filters/FilterPointCloud2.hpp>
#include <pcl/filters/filter_indices.h>

#include <point_cloud2_filters/FilterIndicesPointCloud2Config.h>

namespace point_cloud2_filters {

class FilterIndicesPointCloud2 : public FilterPointCloud2
{
public:
    FilterIndicesPointCloud2();
    ~FilterIndicesPointCloud2();

public:
    virtual bool configure() override;

protected:
    std::shared_ptr<pcl::FilterIndices<Point>> filter_indices_;

private:
    bool negative_ = false;
    bool keep_organized_ = true;
    //Provide a value that the filtered points should be set to instead of removing them
    double user_filter_value_ = std::numeric_limits<double>::quiet_NaN ();
    
    /** \brief Pointer to a dynamic reconfigure service. */
    std::unique_ptr<dynamic_reconfigure::Server<point_cloud2_filters::FilterIndicesPointCloud2Config>> dynamic_reconfigure_srv_;
    dynamic_reconfigure::Server<point_cloud2_filters::FilterIndicesPointCloud2Config>::CallbackType dynamic_reconfigure_clbk_;
    void dynamicReconfigureClbk(point_cloud2_filters::FilterIndicesPointCloud2Config &config, uint32_t level);
    boost::recursive_mutex dynamic_reconfigure_mutex_;
};

FilterIndicesPointCloud2::FilterIndicesPointCloud2() : FilterPointCloud2() 
{
};

FilterIndicesPointCloud2::~FilterIndicesPointCloud2()
{
};

bool FilterIndicesPointCloud2::configure()
{
    FilterPointCloud2::configure();
    
    filter_indices_ = std::dynamic_pointer_cast<pcl::FilterIndices<Point>>(filter_);

    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("keep_organized"), keep_organized_);
    ROS_INFO_NAMED(getName(), "[%s] Using keep organized='%d'", getName().c_str(), keep_organized_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO_NAMED(getName(), "[%s] Using negative='%d'", getName().c_str(), negative_);
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("user_filter_value"), user_filter_value_)) {
        ROS_INFO_NAMED(getName(), "[%s] Using user_filter_value='%f'", getName().c_str(), user_filter_value_);

    }
    
    filter_indices_->setKeepOrganized(keep_organized_);
    filter_indices_->setNegative(negative_);
    
    if (user_filter_value_ != std::numeric_limits<double>::quiet_NaN ()) {
        filter_indices_->setUserFilterValue(user_filter_value_);
    }
    
    //dynamic reconfigure
    dynamic_reconfigure_srv_ = std::make_unique<dynamic_reconfigure::Server<point_cloud2_filters::FilterIndicesPointCloud2Config>>(
        dynamic_reconfigure_mutex_,
        ros::NodeHandle(dynamic_reconfigure_namespace_root_ + "/filter_indices"));
    
    dynamic_reconfigure_clbk_ = boost::bind(&FilterIndicesPointCloud2::dynamicReconfigureClbk, this, _1, _2);

    point_cloud2_filters::FilterIndicesPointCloud2Config initial_config;
    initial_config.keep_organized = keep_organized_;
    initial_config.negative = negative_;

    dynamic_reconfigure_srv_->setConfigDefault(initial_config);
    dynamic_reconfigure_srv_->updateConfig(initial_config);
    
    //put this after updateConfig!
    dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_clbk_);
    
    return true;
    
};

void FilterIndicesPointCloud2::dynamicReconfigureClbk (point_cloud2_filters::FilterIndicesPointCloud2Config &config, uint32_t /*level*/)
{

    boost::recursive_mutex::scoped_lock lock(dynamic_reconfigure_mutex_);

    
    if (keep_organized_ != config.keep_organized)
    {
        keep_organized_ = config.keep_organized;
        filter_indices_->setKeepOrganized(keep_organized_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting keep_organized to: %d.", getName().c_str(), keep_organized_);
    }
    
    if (negative_ != config.negative)
    {
        negative_ = config.negative;
        filter_indices_->setNegative(negative_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting negative to: %d.", getName().c_str(), negative_);
    }
    
}


} //namespace point_cloud2_filters


#endif //FILTER_INDICES_POINT_CLOUD_HPP


#ifndef PASS_THROUGH_FILTER_HPP
#define PASS_THROUGH_FILTER_HPP

#include <point_cloud2_filters/FilterIndicesPointCloud2.hpp>
#include <pcl/filters/passthrough.h>

#include <point_cloud2_filters/PassThroughPointCloud2Config.h>

namespace point_cloud2_filters {
    

class PassThroughFilterPointCloud2 : public FilterIndicesPointCloud2
{
public:
    PassThroughFilterPointCloud2();
    ~PassThroughFilterPointCloud2();

public:
    virtual bool configure() override;
  
private:
    std::shared_ptr<pcl::PassThrough<Point>> pass_through_;

    std::string filter_field_name_ = "z";
    double filter_limit_min_ = 0;
    double filter_limit_max_ = 1;
    
    /** \brief Pointer to a dynamic reconfigure service. */
    std::unique_ptr<dynamic_reconfigure::Server<point_cloud2_filters::PassThroughPointCloud2Config>> dynamic_reconfigure_srv_;
    dynamic_reconfigure::Server<point_cloud2_filters::PassThroughPointCloud2Config>::CallbackType dynamic_reconfigure_clbk_;
    void dynamicReconfigureClbk(point_cloud2_filters::PassThroughPointCloud2Config &config, uint32_t level);
    boost::recursive_mutex dynamic_reconfigure_mutex_;
    
};

PassThroughFilterPointCloud2::PassThroughFilterPointCloud2() : FilterIndicesPointCloud2() {
    
    filter_ = std::make_shared<pcl::PassThrough<Point>>();
    
};

PassThroughFilterPointCloud2::~PassThroughFilterPointCloud2()
{
};


bool PassThroughFilterPointCloud2::configure()
{
    
    FilterIndicesPointCloud2::configure();
    
    pass_through_ = std::dynamic_pointer_cast<pcl::PassThrough<Point>>(filter_);

    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_field_name"), filter_field_name_);
    ROS_INFO_NAMED(getName(),"[%s] Using field name='%s'", getName().c_str(), filter_field_name_.c_str());
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_min"), filter_limit_min_);
    ROS_INFO_NAMED(getName(),"[%s] Using limit min='%f'", getName().c_str(), filter_limit_min_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_max"), filter_limit_max_);
    ROS_INFO_NAMED(getName(),"[%s] Using limit max='%f'", getName().c_str(), filter_limit_max_);
    
    pass_through_->setFilterFieldName(filter_field_name_);
    pass_through_->setFilterLimits(filter_limit_min_, filter_limit_max_);
    
     //dynamic reconfigure
    dynamic_reconfigure_srv_ = std::make_unique<dynamic_reconfigure::Server<point_cloud2_filters::PassThroughPointCloud2Config>>(
        dynamic_reconfigure_mutex_,
        ros::NodeHandle( dynamic_reconfigure_namespace_root_ + "/" + getName()));
    
    dynamic_reconfigure_clbk_ = boost::bind(&PassThroughFilterPointCloud2::dynamicReconfigureClbk, this, _1, _2);

    point_cloud2_filters::PassThroughPointCloud2Config initial_config;
    initial_config.filter_field_name = filter_field_name_;
    initial_config.filter_limit_min = filter_limit_min_;
    initial_config.filter_limit_max = filter_limit_max_;

    dynamic_reconfigure_srv_->setConfigDefault(initial_config);
    dynamic_reconfigure_srv_->updateConfig(initial_config);
    
    //put this after updateConfig!
    dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_clbk_);
    
    return true;
    
};

void PassThroughFilterPointCloud2::dynamicReconfigureClbk (point_cloud2_filters::PassThroughPointCloud2Config &config, uint32_t /*level*/)
{

    boost::recursive_mutex::scoped_lock lock(dynamic_reconfigure_mutex_);
    bool to_update_limits = false;
    
    if (filter_field_name_.compare(config.filter_field_name) != 0)
    {
        filter_field_name_ = config.filter_field_name;
        pass_through_->setFilterFieldName(filter_field_name_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting filter_field_name to: %s.", getName().c_str(), filter_field_name_.c_str());
    }
    
    if (filter_limit_min_ != config.filter_limit_min)
    {
        to_update_limits = true;
        filter_limit_min_ = config.filter_limit_min;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting filter_limit_min to: %f.", getName().c_str(), filter_limit_min_);
    }
    
    if (filter_limit_max_ != config.filter_limit_max)
    {
        to_update_limits = true;
        filter_limit_max_ = config.filter_limit_max;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting filter_limit_max to: %f.", getName().c_str(), filter_limit_max_);
    }
    
    if (to_update_limits) {
        pass_through_->setFilterLimits(filter_limit_min_, filter_limit_max_);
    }
    
}


} //namespace point_cloud2_filters


#endif //PASS_THROUGH_FILTER_HPP

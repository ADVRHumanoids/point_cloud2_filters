#ifndef PASS_THROUGH_FILTER_HPP
#define PASS_THROUGH_FILTER_HPP

#include <point_cloud2_filters/FilterPointCloud2.hpp>
#include <pcl/filters/passthrough.h>


namespace point_cloud2_filters {
    

class PassThroughFilterPointCloud2 : public FilterPointCloud2
{
public:
    PassThroughFilterPointCloud2();
    ~PassThroughFilterPointCloud2();

public:
    virtual bool configure() override;
  
private:
    std::shared_ptr<pcl::PassThrough<Point>> pass_through_;
    
    bool negative_ = false;
    bool keep_organized_ = true;

    std::string filter_field_name_ = "z";
    double filter_limit_min_ = 0;
    double filter_limit_max_ = 1;
    
};

PassThroughFilterPointCloud2::PassThroughFilterPointCloud2() : FilterPointCloud2("PassThroughFilterPointCloud2") {
    
    filter_ = std::make_shared<pcl::PassThrough<Point>>();
    pass_through_ = std::dynamic_pointer_cast<pcl::PassThrough<Point>>(filter_);
    
};

PassThroughFilterPointCloud2::~PassThroughFilterPointCloud2()
{
};


bool PassThroughFilterPointCloud2::configure()
{
    
    FilterPointCloud2::configure();
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("keep_organized"), keep_organized_);
    ROS_INFO_NAMED(name_, "[%s] Using keep organized='%d'", name_.c_str(), keep_organized_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    ROS_INFO_NAMED(name_, "[%s] Using negative='%d'", name_.c_str(), negative_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_field_name"), filter_field_name_);
    ROS_INFO_NAMED(name_,"[%s] Using field name='%s'", name_.c_str(), filter_field_name_.c_str());
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_min"), filter_limit_min_);
    ROS_INFO_NAMED(name_,"[%s] Using limit min='%f'", name_.c_str(), filter_limit_min_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("filter_limit_max"), filter_limit_max_);
    ROS_INFO_NAMED(name_,"[%s] Using limit max='%f'", name_.c_str(), filter_limit_max_);
    
    pass_through_->setKeepOrganized(keep_organized_);
    pass_through_->setNegative(negative_);
    
    pass_through_->setFilterFieldName(filter_field_name_);
    pass_through_->setFilterLimits(filter_limit_min_, filter_limit_max_);
    
    return true;
    
};


} //namespace point_cloud2_filters


#endif //PASS_THROUGH_FILTER_HPP

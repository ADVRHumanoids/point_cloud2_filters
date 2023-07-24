#ifndef FILTER_POINT_CLOUD_HPP
#define FILTER_POINT_CLOUD_HPP

#include <point_cloud2_filters/FilterBasePointCloud2.hpp>
#include <pcl/filters/filter.h>

namespace point_cloud2_filters {

class FilterPointCloud2 : public FilterBasePointCloud2
{
public:
    FilterPointCloud2();
    ~FilterPointCloud2();

public:
    
protected:
    std::shared_ptr<pcl::Filter<Point>> filter_;

    virtual bool execute() override;

private:
};

FilterPointCloud2::FilterPointCloud2() {
    
};

FilterPointCloud2::~FilterPointCloud2()
{
};



bool FilterPointCloud2::execute()
{
    
    filter_->setInputCloud (cloud_out_);
    filter_->filter (*cloud_out_);

    return true;
    
};


} //namespace point_cloud2_filters


#endif //FILTER_POINT_CLOUD_HPP


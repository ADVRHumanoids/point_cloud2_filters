#ifndef SAC_SEGMENTATION_EXTRACT_FILTER_POINT_CLOUD_HPP
#define SAC_SEGMENTATION_EXTRACT_FILTER_POINT_CLOUD_HPP

#include <point_cloud2_filters/FilterBasePointCloud2.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <point_cloud2_filters/SacSegmentationExtractPointCloud2Config.h>

namespace point_cloud2_filters {

class SacSegmentationExtractFilterPointCloud2 : public FilterBasePointCloud2
{
public:
    SacSegmentationExtractFilterPointCloud2();
    ~SacSegmentationExtractFilterPointCloud2();

public:
    virtual bool configure() override;
    
protected:
    virtual bool execute() override;

private:
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointIndices::Ptr inliers_;
    pcl::SACSegmentation<Point> sac_segmentation_;
    pcl::ExtractIndices<Point> extract_indices_;
    
    double axis_x_ = 0;
    double axis_y_ = 0;
    double axis_z_ = 1;
    double eps_angle_ = 0.15;
    double distance_threshold_ = 0.01;

    bool optimize_coefficents_ = true;
    bool negative_ = true; //true: remove the model, false: remove the rest
    int max_iterations_ = 50;
    double probability_ = 0.99;
    double min_radius_ = -10000; //dyn param server wants -2147483648 to 2147483647 range, cant use std::numeric_limits::max and min
    double max_radius_ = 10000;
    
    //https://pointclouds.org/documentation/model__types_8h_source.html
    int model_type_ = pcl::SacModel::SACMODEL_PERPENDICULAR_PLANE;

    //https://pointclouds.org/documentation/method__types_8h_source.html
    int method_type_ = pcl::SAC_RANSAC;

    /** \brief Pointer to a dynamic reconfigure service. */
    std::unique_ptr<dynamic_reconfigure::Server<point_cloud2_filters::SacSegmentationExtractPointCloud2Config>> dynamic_reconfigure_srv_;
    dynamic_reconfigure::Server<point_cloud2_filters::SacSegmentationExtractPointCloud2Config>::CallbackType dynamic_reconfigure_clbk_;
    void dynamicReconfigureClbk(point_cloud2_filters::SacSegmentationExtractPointCloud2Config &config, uint32_t level);
    boost::recursive_mutex dynamic_reconfigure_mutex_;

};

SacSegmentationExtractFilterPointCloud2::SacSegmentationExtractFilterPointCloud2()
{
    
    coefficients_ = boost::make_shared<pcl::ModelCoefficients>();
    inliers_ =  boost::make_shared<pcl::PointIndices>();
    
};

SacSegmentationExtractFilterPointCloud2::~SacSegmentationExtractFilterPointCloud2()
{
};

bool SacSegmentationExtractFilterPointCloud2::configure()
{

    FilterBasePointCloud2::configure();
        
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("optimize_coefficents"), optimize_coefficents_))
    {
        sac_segmentation_.setOptimizeCoefficients (optimize_coefficents_);
        ROS_INFO_NAMED(getName(), "[%s] Using optimize_coefficents=%d", getName().c_str(), optimize_coefficents_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_iterations"), max_iterations_))
    {
        sac_segmentation_.setMaxIterations (max_iterations_);
        ROS_INFO_NAMED(getName(), "[%s] Using max_iterations=%d", getName().c_str(), max_iterations_);
    }
    
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("probability"), probability_))
    {
        sac_segmentation_.setProbability (probability_);
        ROS_INFO_NAMED(getName(), "[%s] Using probability=%f", getName().c_str(), probability_);
    }
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("axis_x"), axis_x_);
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("axis_y"), axis_y_);
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("axis_z"), axis_z_);
    sac_segmentation_.setAxis(Eigen::Vector3f(axis_x_, axis_y_, axis_z_));
    ROS_INFO_NAMED(getName(), "[%s] Using axis=[%f, %f, %f]", getName().c_str(), axis_x_, axis_y_, axis_z_);
    
    bool using_radius = false;
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("min_radius"), min_radius_)) {
        using_radius = true;
    } 
    if (filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("max_radius"), max_radius_)) {
        using_radius = true;
    }
    if (using_radius) {
        sac_segmentation_.setRadiusLimits(min_radius_, max_radius_);
        ROS_INFO_NAMED(getName(), "[%s] Using radius limits=[%f, %f]", getName().c_str(), min_radius_, max_radius_);
    }
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("eps_angle"), eps_angle_);
    sac_segmentation_.setEpsAngle(eps_angle_);
    ROS_INFO_NAMED(getName(), "[%s] Using eps_angle=%f", getName().c_str(), eps_angle_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("distance_threshold"), distance_threshold_);
    sac_segmentation_.setDistanceThreshold (distance_threshold_);
    ROS_INFO_NAMED(getName(), "[%s] Using distance_threshold=%f", getName().c_str(), distance_threshold_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("negative"), negative_);
    extract_indices_.setNegative (negative_);
    ROS_INFO_NAMED(getName(), "[%s] Using negative='%d'", getName().c_str(), negative_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("model_type"), model_type_);
    sac_segmentation_.setModelType (model_type_);
    ROS_INFO_NAMED(getName(), "[%s] Using model_type='%d'", getName().c_str(), model_type_);
    
    filters::FilterBase<sensor_msgs::PointCloud2>::getParam(std::string("method_type"), method_type_);
    sac_segmentation_.setMethodType (method_type_);
    ROS_INFO_NAMED(getName(), "[%s] Using method_type='%d'", getName().c_str(), method_type_);
    

    
    //dynamic reconfigure
    dynamic_reconfigure_srv_ = std::make_unique<dynamic_reconfigure::Server<point_cloud2_filters::SacSegmentationExtractPointCloud2Config>>(
        dynamic_reconfigure_mutex_,
        ros::NodeHandle( dynamic_reconfigure_namespace_root_ + "/" + getName()));
    
    dynamic_reconfigure_clbk_ = boost::bind(&SacSegmentationExtractFilterPointCloud2::dynamicReconfigureClbk, this, _1, _2);

    point_cloud2_filters::SacSegmentationExtractPointCloud2Config initial_config;
    initial_config.optimize_coefficents = optimize_coefficents_;
    initial_config.axis_x = axis_x_;
    initial_config.axis_y = axis_y_;
    initial_config.axis_z = axis_z_;
    initial_config.eps_angle = eps_angle_;
    initial_config.distance_threshold = distance_threshold_;
    initial_config.negative = negative_;
    initial_config.max_iterations = max_iterations_;
    initial_config.probability = probability_;
    initial_config.min_radius = min_radius_;
    initial_config.max_radius = max_radius_;
    initial_config.method_type = method_type_;
    initial_config.model_type = model_type_;
    
    dynamic_reconfigure_srv_->setConfigDefault(initial_config);
    dynamic_reconfigure_srv_->updateConfig(initial_config);
    
    //put this after updateConfig!
    dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_clbk_);
    
    return true;
};



bool SacSegmentationExtractFilterPointCloud2::execute()
{
    
    sac_segmentation_.setInputCloud (cloud_out_);
    sac_segmentation_.segment (*inliers_, *coefficients_);
    
    extract_indices_.setInputCloud (cloud_out_);
    extract_indices_.setIndices (inliers_);
    
    extract_indices_.filter (*cloud_out_);
    
    
    return true;
    
};

void SacSegmentationExtractFilterPointCloud2::dynamicReconfigureClbk (point_cloud2_filters::SacSegmentationExtractPointCloud2Config &config, uint32_t /*level*/)
{

    boost::recursive_mutex::scoped_lock lock(dynamic_reconfigure_mutex_);
    bool axis_to_update = false;
    bool radius_to_update = false;
    
    if (axis_x_ != config.axis_x)
    {
        axis_to_update = true;
        axis_x_ = config.axis_x;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting axis_x to: %f.", getName().c_str(), axis_x_);
    }
    
    if (axis_y_ != config.axis_y)
    {
        axis_to_update = true;
        axis_y_ = config.axis_y;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting axis_y to: %f.", getName().c_str(), axis_y_);
    }
    
    if (axis_z_ != config.axis_z)
    {
        axis_to_update = true;
        axis_z_ = config.axis_z;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting axis_z to: %f.", getName().c_str(), axis_z_);
    }
    
    if (eps_angle_ != config.eps_angle)
    {
        eps_angle_ = config.eps_angle;
        sac_segmentation_.setEpsAngle(eps_angle_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting eps_angle to: %f.", getName().c_str(), eps_angle_);
    }
    if (distance_threshold_ != config.distance_threshold)
    {
        distance_threshold_ = config.distance_threshold;
        sac_segmentation_.setDistanceThreshold (distance_threshold_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting distance_threshold to: %f.", getName().c_str(), distance_threshold_);
    }
    if (optimize_coefficents_ != config.optimize_coefficents)
    {
        optimize_coefficents_ = config.optimize_coefficents;
        sac_segmentation_.setOptimizeCoefficients (optimize_coefficents_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting optimize_coefficents to: %d.", getName().c_str(), optimize_coefficents_);
    }
    if (negative_ != config.negative)
    {
        negative_ = config.negative;
        extract_indices_.setNegative (negative_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting negative to: %d.", getName().c_str(), negative_);
    }
    
    if (max_iterations_ != config.max_iterations)
    {
        max_iterations_ = config.max_iterations;
        sac_segmentation_.setMaxIterations (max_iterations_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting max_iterations to: %d.", getName().c_str(), max_iterations_);
    }
    
    if (probability_ != config.probability)
    {
        probability_ = config.probability;
        sac_segmentation_.setProbability (probability_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting probability to: %f.", getName().c_str(), probability_);
    }
    
    if (min_radius_ != config.min_radius)
    {
        min_radius_ = config.min_radius;
        radius_to_update = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting min_radius to: %f.", getName().c_str(), min_radius_);
    }
    
    if (max_radius_ != config.max_radius)
    {
        max_radius_ = config.max_radius;
        radius_to_update = true;
        ROS_DEBUG_NAMED (getName(), "[%s] Setting max_radius to: %f.", getName().c_str(), max_radius_);
    }
    
    if (model_type_ != config.model_type)
    {
        model_type_ = config.model_type;
        sac_segmentation_.setModelType(model_type_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting model_type to: %d.", getName().c_str(), model_type_);
    }
    
    if (method_type_ != config.method_type)
    {
        method_type_ = config.method_type;
        sac_segmentation_.setMethodType(method_type_);
        ROS_DEBUG_NAMED (getName(), "[%s] Setting method_type to: %d.", getName().c_str(), method_type_);
    }
    
    if (axis_to_update) {
        
        sac_segmentation_.setAxis(Eigen::Vector3f(axis_x_, axis_y_, axis_z_));
    }
    
    if (radius_to_update) {
        
        sac_segmentation_.setRadiusLimits(min_radius_, max_radius_);
    }

}


} //namespace point_cloud2_filters


#endif //SAC_SEGMENTATION_EXTRACT_FILTER_POINT_CLOUD_HPP


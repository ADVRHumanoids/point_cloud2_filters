#ifndef FILTER_BASE_POINT_CLOUD_HPP
#define FILTER_BASE_POINT_CLOUD_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <pcl_ros/transforms.hpp>

// Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <boost/thread/recursive_mutex.hpp>
//#include <point_cloud2_filters/FilterBasePointCloud2Config.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
namespace point_cloud2_filters {
    
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class FilterBasePointCloud2 : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
{
public:
    FilterBasePointCloud2();
    ~FilterBasePointCloud2();

public:
    virtual bool configure() override;

    /** \brief Update the filter and return the data seperately
    * \param data_in T array with length width
    * \param data_out T array with length width
    */
    virtual bool update( const sensor_msgs::msg::PointCloud2& data_in, sensor_msgs::msg::PointCloud2& data_out) override final;
    
protected:
    std::string dynamic_reconfigure_namespace_root_;
    
    virtual bool execute() = 0;
    
    PointCloud::Ptr cloud_out_;
    
private:
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    //std::unique_ptr<ros::NodeHandle> nh_;
    //ros::Publisher pc_pub_;
    
    /** \brief Pointer to a dynamic reconfigure service. */
    // std::unique_ptr<dynamic_reconfigure::Server<point_cloud2_filters::FilterBasePointCloud2Config>> dynamic_reconfigure_srv_;
    // dynamic_reconfigure::Server<point_cloud2_filters::FilterBasePointCloud2Config>::CallbackType dynamic_reconfigure_clbk_;
    //void dynamicReconfigureClbk(point_cloud2_filters::FilterBasePointCloud2Config &config, uint32_t level);
    // boost::recursive_mutex dynamic_reconfigure_mutex_;
    
    bool active_ = true;
    std::string input_frame_ = "";
    std::string output_frame_ = "";
    bool pub_cloud_ = false;

    bool log_time_ = false;
    std::chrono::high_resolution_clock::time_point start_time_, stop_time_;
};

FilterBasePointCloud2::FilterBasePointCloud2() {
    
    cloud_out_ = std::make_shared<PointCloud>();
    
    //TODO get clock from the node...
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
};

FilterBasePointCloud2::~FilterBasePointCloud2()
{
};

bool FilterBasePointCloud2::configure()
{

    //nh_ = std::make_unique<ros::NodeHandle>("~");

    if (filters::FilterBase<sensor_msgs::msg::PointCloud2>::getParam(std::string("active"), active_))
    {
        RCLCPP_INFO(logging_interface_->get_logger(), "[%s] Using active='%d'", getName().c_str(), active_);
    }
    
    if (filters::FilterBase<sensor_msgs::msg::PointCloud2>::getParam(std::string("input_frame"), input_frame_))
    {
        RCLCPP_INFO(logging_interface_->get_logger(), "[%s] Using input_frame='%s'", getName().c_str(), input_frame_.c_str());
    }
    
    if (filters::FilterBase<sensor_msgs::msg::PointCloud2>::getParam(std::string("output_frame"), output_frame_))
    {
        RCLCPP_INFO(logging_interface_->get_logger(), "[%s] Using output_frame='%s'", getName().c_str(), output_frame_.c_str());
    }    
    
    if (filters::FilterBase<sensor_msgs::msg::PointCloud2>::getParam(std::string("pub_cloud"), pub_cloud_))
    {
        RCLCPP_INFO(logging_interface_->get_logger(), "[%s] Using pub_cloud='%d'", getName().c_str(), pub_cloud_);
        //pc_pub_ = nh_->advertise<sensor_msgs::msg::PointCloud2>(getName()+"/points", 10);
    }
    if (filters::FilterBase<sensor_msgs::msg::PointCloud2>::getParam(std::string("log_time"), log_time_))
    {
        RCLCPP_INFO(logging_interface_->get_logger(), "[%s] Using log_time_='%d'", getName().c_str(), log_time_);
    }
    
    //WARNING dynamic reconfigure, the base class one. Children can have their own server for their specific values, but
    //be sure to use another namespace to be passed to the dyn server constructor (eg ros::NodeHandle(dynamic_reconfigure_namespace_root_ + "/" + getName())
    // dynamic_reconfigure_namespace_root_ = "/filter/" + getName();
    // dynamic_reconfigure_srv_ = std::make_unique<dynamic_reconfigure::Server<point_cloud2_filters::FilterBasePointCloud2Config>>(
    //     dynamic_reconfigure_mutex_,
    //     ros::NodeHandle(dynamic_reconfigure_namespace_root_ + "/base"));
    
    // dynamic_reconfigure_clbk_ = boost::bind(&FilterBasePointCloud2::dynamicReconfigureClbk, this, _1, _2);
        
    //point_cloud2_filters::FilterBasePointCloud2Config initial_config;
    //initial_config.active = active_;
    //initial_config.input_frame = input_frame_;
    //initial_config.output_frame = output_frame_;
    //initial_config.pub_cloud = pub_cloud_;
    // dynamic_reconfigure_srv_->setConfigDefault(initial_config);
    // dynamic_reconfigure_srv_->updateConfig(initial_config);
    
    //put this after updateConfig!
    // dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_clbk_);

    return true;
    
};

bool FilterBasePointCloud2::update( const sensor_msgs::msg::PointCloud2& data_in, sensor_msgs::msg::PointCloud2& data_out)
{

    if (log_time_) {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    if (active_) {
        pcl::fromROSMsg(data_in, *cloud_out_);
        
        //TODO do not transform if already that frame
        if (input_frame_.length() > 0) {
            
            pcl_ros::transformPointCloud (input_frame_, *cloud_out_, *cloud_out_, *tf_buffer_);
        } 
        
        if (!execute()) {
            data_out = data_in;
            return false;
        }

        //TODO do not transform if already that frame
        if (output_frame_.length() > 0) {
            
            pcl_ros::transformPointCloud (output_frame_, *cloud_out_, *cloud_out_, *tf_buffer_);
        } 
        
        pcl::toROSMsg(*cloud_out_, data_out);
        
    } else {
        data_out = data_in;
    }

    if (pub_cloud_) {
        //pc_pub_.publish(data_out);
    }
    if (log_time_) {
        stop_time_ = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(logging_interface_->get_logger(), "[%s] took %ld ms", getName().c_str(), std::chrono::duration_cast<std::chrono::milliseconds>(stop_time_ - start_time_).count());
    }

    return true;
    
};

// void FilterBasePointCloud2::dynamicReconfigureClbk (point_cloud2_filters::FilterBasePointCloud2Config &config, uint32_t /*level*/)
// {

//     boost::recursive_mutex::scoped_lock lock(dynamic_reconfigure_mutex_);

//     if (active_ != config.active)
//     {
//         active_ = config.active;
//         RCLCPP_DEBUG(logging_interface_->get_logger(), "[%s] Setting active to: %d.", getName().c_str(), active_);
//     }
    
//     if (input_frame_.compare(config.input_frame) != 0 )
//     {
//         input_frame_ = config.input_frame;
//         RCLCPP_DEBUG(logging_interface_->get_logger(), "[%s] Setting the input TF frame to: %s.", getName().c_str(), input_frame_.c_str());
//     }
    
//     if (output_frame_.compare(config.output_frame) != 0 )
//     {
//         output_frame_ = config.output_frame;
//         RCLCPP_DEBUG(logging_interface_->get_logger(), "[%s] Setting the output TF frame to: %s.", getName().c_str(), output_frame_.c_str());
//     }   
    
//     if (pub_cloud_ != config.pub_cloud)
//     {
//         pub_cloud_ = config.pub_cloud;
//         if (pub_cloud_) {
//             //pc_pub_ = nh_->advertise<sensor_msgs::msg::PointCloud2>(getName()+"/points", 10);
//         } else {
//             //pc_pub_.shutdown();
//         }
//         RCLCPP_DEBUG(logging_interface_->get_logger(), "[%s] Setting pub_cloud to: %d.", getName().c_str(), pub_cloud_);
//     }
// }


} //namespace point_cloud2_filters


#endif //FILTER_BASE_POINT_CLOUD_HPP


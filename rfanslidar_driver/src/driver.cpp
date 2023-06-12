//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "rfanslidar_driver/driver.hpp"

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Const
//-----------------------------
constexpr double TO_RAD = M_PI / 180.0;

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project name
 * 
 */
namespace MiYALAB{
namespace ROS2{
/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
RFansLiDAR::RFansLiDAR(rclcpp::NodeOptions options) : rclcpp::Node("rfans_lidar", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    const std::string IP_ADDRESS = this->declare_parameter("rfans.device.ip_address", "192.168.0.3");
    const std::string MODEL_NAME = this->declare_parameter("rfans.device.model", "R-Fans-16");
    const int STATUS_PORT = this->declare_parameter("rfans.device.status_port", 2030);
    this->forceSet(&SCAN_RATE, this->declare_parameter("rfans.scan.rate", 20));
    this->forceSet(&SCAN_ANGLE_MIN, this->declare_parameter("rfans.scan.angle_min", -180) * TO_RAD);
    this->forceSet(&SCAN_ANGLE_MAX, this->declare_parameter("rfans.scan.angle_max",  180) * TO_RAD);
    const bool POINTS_PUBLISH = this->declare_parameter("rfans.points.publish", false);
    const bool POINTS_NEAR_PUBLISH = this->declare_parameter("rfans.points.near_publish" false);
    this->forceSet(&POINST_NEAR_RANGE, this->declare_parameter("rfans.points.near_range", 50.0));
    bool DEPTH_IMG_PUBLISH = this->declare_parameter("rfans.img.depth_publish", false);
    bool INTENSITY_IMG_PUBLISH = this->declare_parameter("rfans.img.intensity_publish", false);
    const double OFFSET_LINEAR_X = this->declare_parameter("rfans.offset.linear.x", 0.0);
    const double OFFSET_LINEAR_Y = this->declare_parameter("rfans.offset.linear.y", 0.0);
    const double OFFSET_LINEAR_Z = this->declare_parameter("rfans.offset.linear.z", 0.0);
    const double OFFSET_ANGULAR_X = this->declare_parameter("rfans.offset.angular.x", 0.0);
    const double OFFSET_ANGULAR_Y = this->declare_parameter("rfans.offset.angular.y", 0.0);
    const double OFFSET_ANGULAR_Z = this->declare_parameter("rfans.offset.angular.z", 0.0);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize lidar
    RCLCPP_INFO(this->get_logger(), "Initialize lidar...");
    this->rfans = std::make_shared<MiYALAB::Sensor::RFansDriver>(IP_ADDRESS, STATUS_PORT, MODEL_NAME);
    RCLCPP_INFO(this->get_logger(), "Complete! lidar was initialized.");

    // Initialize subscriber
    // RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    // RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&RFansLiDAR::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
RFansLiDAR::~RFansLiDAR()
{
    this->rfans->scanStop();
    this->rfans = nullptr;
    this->thread.release();
}

/**
 * @brief Execute method
 * 
 */
void RFansLiDAR::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());

    }
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::RFansLiDAR)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
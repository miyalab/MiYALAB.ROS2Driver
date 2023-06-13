#ifndef __MIYALAB_ROS2_DRIVER_RFANS_LIDAR_DRIVER_HPP__
#define __MIYALAB_ROS2_DRIVER_RFANS_LIDAR_DRIVER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>

// MiYALAB
#include <MiYALAB/Sensor/RFansLiDAR/driver.hpp>

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Class
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Component Definition
 * 
 */
class RFansLiDAR: public rclcpp::Node {
public:
    RFansLiDAR(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~RFansLiDAR();
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr points_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr points_near_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_img_publisher;

    const int SCAN_RATE = 20;
    const std::string FRAME_ID = "";
    const double SCAN_THETA_MIN = -M_PI;
    const double SCAN_THETA_MAX =  M_PI;
    const double SCAN_PHI_MIN   = -M_PI;
    const double SCAN_PHI_MAX   =  M_PI;
    const double POINTS_NEAR_RANGE = 50;
    const double OFFSET_LINEAR_X = 0.0;
    const double OFFSET_LINEAR_Y = 0.0;
    const double OFFSET_LINEAR_Z = 0.0;
    const double OFFSET_ANGULAR_X = 0.0;
    const double OFFSET_ANGULAR_Y = 0.0;
    const double OFFSET_ANGULAR_Z = 0.0;
    const double IMG_THETA_RESOLUTION = 0.36;
    const double IMG_PHI_RESOLUTION = 2.0; 
    template<typename T, typename U> static void forceSet(const T *value, const U &set_value){*((T*)value) = set_value;}

    std::shared_ptr<MiYALAB::Sensor::RFansDriver> rfans;
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}
#endif // __MIYALAB_ROS2_DRIVER_RFANS_LIDAR_DRIVER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
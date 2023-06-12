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
    const int SCAN_RATE = 20;
    const std::string FRAME_ID = "";
    const std::string IP_ADDRESS = "192.168.0.3";
    const int STATUS_PORT = 2030;
    const double SCAN_ANGLE_MIN = -M_PI;
    const double SCAN_ANGLE_MAX =  M_PI;
    const bool POINTS_PUBLISH = false;
    const bool POINT_NEAR_PUBLISH = false;
    const double RANGE_NEAR = 50;
    const bool DEPTH_IMG_PUBLISH = false;
    const bool INTENSITY_IMG_PUBLISH = false;
    const double OFFSET_LINEAR_X = 0.0;
    const double OFFSET_LINEAR_Y = 0.0;
    const double OFFSET_LINEAR_Z = 0.0;
    const double OFFSET_ANGULAR_X = 0.0;
    const double OFFSET_ANGULAR_Y = 0.0;
    const double OFFSET_ANGULAR_Z = 0.0;
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
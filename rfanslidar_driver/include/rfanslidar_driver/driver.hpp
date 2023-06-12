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

    std::unique_ptr<std::thread> thread;
    void run();
};
}
}
#endif // __MIYALAB_ROS2_DRIVER_RFANS_LIDAR_DRIVER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
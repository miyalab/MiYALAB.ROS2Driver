#ifndef __MIYALAB_ROS2_DRIVER_RPLIDAR_DRIVER_HPP__
#define __MIYALAB_ROS2_DRIVER_RPLIDAR_DRIVER_HPP__

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
#include <sensor_msgs/msg/laser_scan.hpp>

// RPLiDAR SDK
#include "sl_lidar.h"

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
class RPLiDAR: public rclcpp::Node {
public:
    RPLiDAR(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~RPLiDAR();
private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_active_service;
    void serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                            const std_srvs::srv::SetBool::Request::SharedPtr request,
                            const std_srvs::srv::SetBool::Response::SharedPtr response);

    const std::string FRAME_ID = "";
    const bool INVERTED = false;
    const bool ANGLE_COMPENSATE = false;
    const std::string SCAN_MODE = "";
    const double SCAN_FREQUENCY = 0.0;

    void toROS2LaserScan(sensor_msgs::msg::LaserScan *laser, 
        sl_lidar_response_measurement_node_hq_t *nodes, const size_t &node_count, 
        const rclcpp::Time &start,const double &scan_time,
        const float &angle_min, const float &angle_max,
        const float &distance_max
    );
    sl::ILidarDriver *driver;
    bool getRPLIDARDeviceInfo(sl::ILidarDriver *driver);
    bool checkRPLIDARHealth(sl::ILidarDriver *driver);
    static float getAngle(const sl_lidar_response_measurement_node_hq_t& node) {return node.angle_z_q14 * 90.f / 16384.f;}
    template<typename T, typename U>static void forceSet(const T *value, const U &set){*((T*)value)=set;}
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}
#endif // __MIYALAB_ROS2_DRIVER_RPLIDAR_DRIVER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
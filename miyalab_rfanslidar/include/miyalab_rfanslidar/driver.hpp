#ifndef __MIYALAB_ROS2_DRIVER_MIYALAB_RFANS_LIDAR_HPP__
#define __MIYALAB_ROS2_DRIVER_MIYALAB_RFANS_LIDAR_HPP__

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr points_publisher = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr points_near_publisher = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_publisher = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_img_publisher = nullptr;

    struct Parameter{
        int scan_rate = 20;
        std::string frame_id = "";
        double scan_theta_min = -M_PI;
        double scan_theta_max =  M_PI;
        double scan_phi_min = -M_PI;
        double scan_phi_max =  M_PI;
        double points_near_range = 50.0;
        double offset_linear_x = 0.0;
        double offset_linear_y = 0.0;
        double offset_linear_z = 0.0;
        double offset_angular_x = 0.0;
        double offset_angular_y = 0.0;
        double offset_angular_z = 0.0;
        double img_theta_resolution = 0.36;
        double img_phi_resolution = 2.0;
        cv::Size img_size;
    } m_param;

    void pointsPublish(const std_msgs::msg::Header &header, const MiYALAB::Sensor::PointCloudPolar &polars);
    void imagePublish(const std_msgs::msg::Header &header, const MiYALAB::Sensor::PointCloudPolar &polars);

    std::shared_ptr<MiYALAB::Sensor::RFansDriver> rfans = nullptr;
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}
#endif // __MIYALAB_ROS2_DRIVER_RFANS_LIDAR_DRIVER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
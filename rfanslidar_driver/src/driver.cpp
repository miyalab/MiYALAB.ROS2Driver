//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>

// MiYALAB
#include <MiYALAB/Sensor/PointCloud/PointCloudPolar.hpp>

#include "rfanslidar_driver/driver.hpp"

//-----------------------------
// Symbol
//-----------------------------
#define DEBUG

//-----------------------------
// Namespace & using
//-----------------------------
using sensor_msgs::msg::PointCloud;
using sensor_msgs::msg::Image;

//-----------------------------
// Const
//-----------------------------
constexpr double TO_RAD = M_PI / 180.0;
constexpr double TO_DEG = 180.0 / M_PI;

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
    this->forceSet(&this->SCAN_RATE, this->declare_parameter("rfans.scan.rate", 20));
    this->forceSet(&this->SCAN_THETA_MIN, this->declare_parameter("rfans.scan.theta.min", -180.0) * TO_RAD);
    this->forceSet(&this->SCAN_THETA_MAX, this->declare_parameter("rfans.scan.theta.max",  180.0) * TO_RAD);
    this->forceSet(&this->SCAN_PHI_MIN, this->declare_parameter("rfans.scan.phi.min", -15.0) * TO_RAD);
    this->forceSet(&this->SCAN_PHI_MAX, this->declare_parameter("rfans.scan.phi.max",  15.0) * TO_RAD);
    const bool POINTS_PUBLISH = this->declare_parameter("rfans.points.publish", false);
    const bool POINTS_NEAR_PUBLISH = this->declare_parameter("rfans.points.near_publish", false);
    this->forceSet(&POINTS_NEAR_RANGE, this->declare_parameter("rfans.points.near_range", 50.0));
    bool DEPTH_IMG_PUBLISH = this->declare_parameter("rfans.img.depth_publish", false);
    bool INTENSITY_IMG_PUBLISH = this->declare_parameter("rfans.img.intensity_publish", false);
    this->forceSet(&this->OFFSET_LINEAR_X, this->declare_parameter("rfans.offset.linear.x", 0.0));
    this->forceSet(&this->OFFSET_LINEAR_Y, this->declare_parameter("rfans.offset.linear.y", 0.0));
    this->forceSet(&this->OFFSET_LINEAR_Z, this->declare_parameter("rfans.offset.linear.z", 0.0));
    this->forceSet(&this->OFFSET_ANGULAR_X, this->declare_parameter("rfans.offset.angular.x", 0.0) * TO_RAD);
    this->forceSet(&this->OFFSET_ANGULAR_Y, this->declare_parameter("rfans.offset.angular.y", 0.0) * TO_RAD);
    this->forceSet(&this->OFFSET_ANGULAR_Z, this->declare_parameter("rfans.offset.angular.z", 0.0) * TO_RAD);
    this->forceSet(&this->IMG_THETA_RESOLUTION, this->declare_parameter("rfans.img.theta_resolution", 0.36) * TO_RAD);
    this->forceSet(&this->IMG_PHI_RESOLUTION, this->declare_parameter("rfans.img.phi_resolution", 2.0) * TO_RAD);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize lidar
    RCLCPP_INFO(this->get_logger(), "Initialize lidar...");
    this->rfans = std::make_shared<MiYALAB::Sensor::RFansDriver>(IP_ADDRESS, STATUS_PORT, MODEL_NAME);
    if(!this->rfans->scanStart(this->SCAN_RATE)){
        RCLCPP_ERROR(this->get_logger(), "Failure lidar scan start");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Complete! lidar was initialized.");

    // Initialize subscriber
    // RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    if(POINTS_PUBLISH)        this->points_publisher = this->create_publisher<PointCloud>("~/points", 10);
    if(POINTS_NEAR_PUBLISH)   this->points_near_publisher = this->create_publisher<PointCloud>("~/points_near", 10);
    if(DEPTH_IMG_PUBLISH)     this->depth_img_publisher = this->create_publisher<Image>("~/depth_img", 10);
    if(INTENSITY_IMG_PUBLISH) this->intensity_img_publisher = this->create_publisher<Image>("~/intensity_img", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    // RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

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
    
    cv::Size img_size(
        (this->SCAN_THETA_MAX - this->SCAN_THETA_MIN) / this->IMG_THETA_RESOLUTION,
        (this->SCAN_PHI_MAX - this->SCAN_PHI_MIN) / this->IMG_PHI_RESOLUTION
    );
    RCLCPP_INFO_STREAM(this->get_logger(), img_size);
    cv_bridge::CvImage depth_img;
    cv_bridge::CvImage intensity_img;
    PointCloud::UniquePtr points_msg;
    PointCloud::UniquePtr points_near_msg;;
    Image::UniquePtr depth_img_msg;
    Image::UniquePtr intensity_img_msg;
    const bool points_publish = (this->points_publisher.get() != nullptr) || (this->points_near_publisher.get() != nullptr);
    const bool img_publish    = (this->depth_img_publisher.get() != nullptr) || (this->intensity_img_publisher.get() != nullptr);

    // Main loop
    for(rclcpp::WallRate loop(this->SCAN_RATE); rclcpp::ok(); loop.sleep()){
        std_msgs::msg::Header header;
        MiYALAB::Sensor::PointCloudPolar polars;
        header.frame_id = this->FRAME_ID;
        header.stamp = this->now();
        this->rfans->getPoints(&polars);

        if(this->points_publisher.get()){
            points_msg = std::make_unique<PointCloud>();
            points_msg->header = header;
            points_msg->channels.resize(1);
            points_msg->channels[0].name = "intensity";
        }
        if(this->points_near_publisher.get()){
            points_near_msg = std::make_unique<PointCloud>();
            points_near_msg->header = header;
            points_near_msg->channels.resize(1);
            points_near_msg->channels[0].name = "intensity";
        }
        if(this->depth_img_publisher.get()){
            depth_img.header = header;
            depth_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            depth_img.image = cv::Mat(img_size, CV_32FC1, cv::Scalar(-1.0));
        }
        if(this->intensity_img_publisher.get()){
            intensity_img.header = header;
            intensity_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            intensity_img.image = cv::Mat(img_size, CV_32FC1, cv::Scalar(-1.0));
        }

        for(int i=0, size=polars.polars.size(); i<size; i++){
            if(polars.polars[i].range < 0.1) continue;
            if(polars.polars[i].theta < this->SCAN_THETA_MIN || this->SCAN_THETA_MAX < polars.polars[i].theta) continue;
            if(polars.polars[i].phi < this->SCAN_PHI_MIN || this->SCAN_PHI_MAX < polars.polars[i].phi) continue;

            if(points_publish){
                double phi = polars.polars[i].phi + this->OFFSET_ANGULAR_Y;
                double theta = polars.polars[i].theta + this->OFFSET_ANGULAR_Z;
                geometry_msgs::msg::Point32 point;
                point.x = polars.polars[i].range * std::cos(phi) * std::cos(theta);
                point.y = polars.polars[i].range * std::cos(phi) * std::sin(theta);
                point.z = polars.polars[i].range * std::sin(phi);

                if(this->points_publisher.get()){
                    points_msg->points.emplace_back(point);
                    points_msg->channels[0].values.emplace_back(polars.intensity[i]);
                }
                if(this->points_near_publisher.get() && polars.polars[i].range < this->POINTS_NEAR_RANGE){
                    points_near_msg->points.emplace_back(point);
                    points_near_msg->channels[0].values.emplace_back(polars.intensity[i]);
                }
            }
            if(img_publish){
                int px = img_size.width - (polars.polars[i].theta - this->SCAN_THETA_MIN) / this->IMG_THETA_RESOLUTION;
                int py = img_size.height - (polars.polars[i].phi - this->SCAN_PHI_MIN) / this->IMG_PHI_RESOLUTION;

                if(0<=px && px<img_size.width && 0<=py && py<img_size.height){
                    if(this->depth_img_publisher.get())     depth_img.image.at<float>(py, px) = polars.polars[i].range;
                    if(this->intensity_img_publisher.get()) intensity_img.image.at<float>(py, px) = polars.intensity[i];
                }
            }
        }
        

        if(this->points_publisher.get())      this->points_publisher->publish(std::move(points_msg));
        if(this->points_near_publisher.get()) this->points_near_publisher->publish(std::move(points_near_msg));
        if(this->depth_img_publisher.get()){
            depth_img_msg = std::make_unique<Image>();
            depth_img.toImageMsg(*depth_img_msg);
            this->depth_img_publisher->publish(std::move(depth_img_msg));
        }
        if(this->intensity_img_publisher.get()){
            intensity_img_msg = std::make_unique<Image>();
            intensity_img.toImageMsg(*intensity_img_msg);
            this->intensity_img_publisher->publish(std::move(intensity_img_msg));
        }
    }
    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::RFansLiDAR)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
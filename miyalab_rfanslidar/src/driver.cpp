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

#include "miyalab_rfanslidar/driver.hpp"

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
    const std::string IP_ADDRESS     = this->declare_parameter("rfans.device.ip_address", "192.168.0.3");
    const std::string MODEL_NAME     = this->declare_parameter("rfans.device.model", "R-Fans-16");
    const int STATUS_PORT            = this->declare_parameter("rfans.device.status_port", 2030);
    const bool POINTS_PUBLISH        = this->declare_parameter("rfans.points.publish", false);
    const bool POINTS_NEAR_PUBLISH   = this->declare_parameter("rfans.points.near_publish", false);
    const bool DEPTH_IMG_PUBLISH     = this->declare_parameter("rfans.img.depth_publish", false);
    const bool INTENSITY_IMG_PUBLISH = this->declare_parameter("rfans.img.intensity_publish", false);
    m_param.frame_id          = this->declare_parameter("rfans.frame_id", "rfans");
    m_param.scan_rate         = this->declare_parameter("rfans.scan.rate", 20);
    m_param.scan_theta_min    = this->declare_parameter("rfans.scan.theta.min", -180.0) * TO_RAD;
    m_param.scan_theta_max    = this->declare_parameter("rfans.scan.theta.max",  180.0) * TO_RAD;
    m_param.scan_phi_min      = this->declare_parameter("rfans.scan.phi.min", -15.0) * TO_RAD;
    m_param.scan_phi_max      = this->declare_parameter("rfans.scan.phi.max",  15.0) * TO_RAD;
    m_param.points_near_range = this->declare_parameter("rfans.points.near_range", 50.0);
    m_param.offset_linear_x   = this->declare_parameter("rfans.offset.linear.x", 0.0);
    m_param.offset_linear_y   = this->declare_parameter("rfans.offset.linear.y", 0.0);
    m_param.offset_linear_z   = this->declare_parameter("rfans.offset.linear.z", 0.0);
    m_param.offset_angular_x  = this->declare_parameter("rfans.offset.angular.x", 0.0) * TO_RAD;
    m_param.offset_angular_y  = this->declare_parameter("rfans.offset.angular.y", 0.0) * TO_RAD;
    m_param.offset_angular_z  = this->declare_parameter("rfans.offset.angular.z", 0.0) * TO_RAD;
    m_param.img_theta_resolution = this->declare_parameter("rfans.img.theta_resolution", 0.36) * TO_RAD;
    m_param.img_phi_resolution   = this->declare_parameter("rfans.img.phi_resolution", 2.0) * TO_RAD;
    m_param.img_size.width  = (m_param.scan_theta_max - m_param.scan_theta_min) / m_param.img_theta_resolution;
    m_param.img_size.height = (m_param.scan_phi_max - m_param.scan_phi_min) / m_param.img_phi_resolution;
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize lidar
    RCLCPP_INFO(this->get_logger(), "Initialize lidar...");
    this->rfans = std::make_shared<MiYALAB::Sensor::RFansDriver>(IP_ADDRESS, STATUS_PORT, MODEL_NAME);
    if(!this->rfans->scanStart(m_param.scan_rate)){
        RCLCPP_ERROR(this->get_logger(), "Failure lidar scan start");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Complete! lidar was initialized.");

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

void RFansLiDAR::pointsPublish(const std_msgs::msg::Header &header, const MiYALAB::Sensor::PointCloudPolar &polars)
{
    auto points_msg = std::make_unique<PointCloud>();
    auto points_near_msg = std::make_unique<PointCloud>();
    points_msg->channels.resize(2);
    points_near_msg->channels.resize(2);
    points_msg->header = points_near_msg->header = header;
    points_msg->channels[0].name = points_near_msg->channels[0].name = "range";
    points_msg->channels[1].name = points_near_msg->channels[1].name = "intensity";

    for(int i=0, size=polars.polars.size(); i<size; i++){
        const double phi = polars.polars[i].phi + m_param.offset_angular_y;
        const double theta = polars.polars[i].theta + m_param.offset_angular_z;
        geometry_msgs::msg::Point32 point;
        
        const double cos_phi = std::cos(phi);
        point.x = polars.polars[i].range * cos_phi * std::cos(theta) + m_param.offset_linear_x;
        const double y = polars.polars[i].range * cos_phi * std::sin(theta);
        const double z = polars.polars[i].range * std::sin(phi);
        const double cos_x = std::cos(m_param.offset_angular_x);
        const double sin_x = std::sin(m_param.offset_angular_x);
        point.y = y * cos_x - z * sin_x + m_param.offset_linear_y;
        point.z = y * sin_x + z * cos_x + m_param.offset_linear_y;

        if(this->points_publisher.get()){
            points_msg->points.emplace_back(point);
            points_msg->channels[0].values.emplace_back(polars.polars[i].range);
            points_msg->channels[1].values.emplace_back(polars.channels[0].values[i]);
        }
        if(this->points_near_publisher.get() && polars.polars[i].range < m_param.points_near_range){
            points_near_msg->points.emplace_back(point);
            points_near_msg->channels[0].values.emplace_back(polars.polars[i].range);
            points_near_msg->channels[1].values.emplace_back(polars.channels[0].values[i]);
        }
    }
    if(this->points_publisher.get())      this->points_publisher->publish(std::move(points_msg));
    if(this->points_near_publisher.get()) this->points_near_publisher->publish(std::move(points_near_msg));
}

void RFansLiDAR::imagePublish(const std_msgs::msg::Header &header, const MiYALAB::Sensor::PointCloudPolar &polars)
{
    cv_bridge::CvImage depth_img;
    cv_bridge::CvImage intensity_img;
    depth_img.header = intensity_img.header = header;
    depth_img.encoding = intensity_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_img.image     = cv::Mat(m_param.img_size, CV_32FC1, cv::Scalar(-1.0f));
    intensity_img.image = cv::Mat(m_param.img_size, CV_32FC1, cv::Scalar(-1.0f));

    std::vector<std::vector<float>> depth(m_param.img_size.height, std::vector<float>(m_param.img_size.width, -1.0f));
    std::vector<std::vector<float>> intensity(m_param.img_size.height, std::vector<float>(m_param.img_size.width, -1.0f));
    for(int i=0, size=polars.polars.size(); i<size; i++){
        double theta = polars.polars[i].theta + m_param.offset_angular_z;
        if(theta > M_PI) theta -= 2*M_PI;
        if(theta < M_PI) theta += 2*M_PI;
        const int px = m_param.img_size.width - (polars.polars[i].theta - m_param.scan_theta_min) / m_param.img_theta_resolution;
        const int py = m_param.img_size.height - (polars.polars[i].phi - m_param.scan_phi_min) / m_param.img_phi_resolution;
        if(0<=px && px<m_param.img_size.width && 0<=py && py<m_param.img_size.height){
            depth[py][px] = polars.polars[i].range;
            intensity[py][px] = polars.channels[0].values[i];
        }
    }

    // スムージング処理
    for(int y=0; y<m_param.img_size.height; y++){
        auto depth_img_ptr = &depth_img.image.at<float>(y,0);
        auto intensity_img_ptr = &intensity_img.image.at<float>(y,0);
        for(int x=0; x<m_param.img_size.width; x++){
            // データ欠如箇所は周辺画素の平均値
            if(depth[y][x] < 0){
                int cnt = 0;
                float depth_sum = 0;
                float intensity_sum = 0;
                for(int j=y-1; j<=y+1; j++){
                    if(j<0 || m_param.img_size.height<=j) continue;
                    for(int i=x-1; i<=x+1; i++){
                        if(i<0 || m_param.img_size.width<=i) continue; 
                        if(depth[j][i]>=0){
                            depth_sum += depth[j][i];
                            intensity_sum += intensity[j][i];
                            cnt++;
                        }
                    }
                }
                if(cnt){
                    depth_img_ptr[x] = depth_sum/cnt;
                    intensity_img_ptr[x] = intensity_sum/cnt;
                }
                else{
                    depth_img_ptr[x] = -1.0f;
                    intensity_img_ptr[x] = -1.0f;
                }
            }
            // データがあるときはそのままコピー
            else{
                depth_img_ptr[x] = depth[y][x];
                intensity_img_ptr[x] = intensity[y][x];
            }
        }
    }

    if(this->depth_img_publisher.get()){
        auto depth_img_msg = std::make_unique<Image>();
        depth_img.toImageMsg(*depth_img_msg);
        this->depth_img_publisher->publish(std::move(depth_img_msg));
    }
    if(this->intensity_img_publisher.get()){
        auto intensity_img_msg = std::make_unique<Image>();
        intensity_img.toImageMsg(*intensity_img_msg);
        this->intensity_img_publisher->publish(std::move(intensity_img_msg));
    }
}

/**
 * @brief Execute method
 * 
 */
void RFansLiDAR::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    const bool points_publish = (this->points_publisher.get() != nullptr) || (this->points_near_publisher.get() != nullptr);
    const bool image_publish  = (this->depth_img_publisher.get() != nullptr) || (this->intensity_img_publisher.get() != nullptr);

    // Main loop
    for(rclcpp::WallRate loop(m_param.scan_rate); rclcpp::ok(); loop.sleep()){
        std_msgs::msg::Header header;
        MiYALAB::Sensor::PointCloudPolar polars;
        header.frame_id = m_param.frame_id;
        header.stamp = this->now();
        this->rfans->getPoints(&polars);

        std::thread points_thread;
        std::thread image_thread;
        if(points_publish) points_thread = std::thread(&RFansLiDAR::pointsPublish, this, header, polars);
        if(image_publish)  image_thread = std::thread(&RFansLiDAR::imagePublish, this, header, polars);
        if(points_publish) points_thread.join();
        if(image_publish)  image_thread.join();
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
//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_rplidar/driver.hpp"

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// const 
//-----------------------------
constexpr double TO_RAD = M_PI / 180;

constexpr int LIDAR_A_SERIES_MINUM_MAJOR_ID = 0;
constexpr int LIDAR_S_SERIES_MINUM_MAJOR_ID = 5;
constexpr int LIDAR_T_SERIES_MINUM_MAJOR_ID = 8;

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
RPLiDAR::RPLiDAR(rclcpp::NodeOptions options) : rclcpp::Node("rplidar", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    const std::string CHANNEL_TYPE    = this->declare_parameter("rplidar.channel_type", "serial");
    const std::string TCP_IP          = this->declare_parameter("rplidar.tcp_ip", "192.168.0.7");
    const int         TCP_PORT        = this->declare_parameter("rplidar.tcp_port", 20108);
    const std::string UDP_IP          = this->declare_parameter("rplidar.udp_ip", "192.168.11.2");
    const int         UDP_PORT        = this->declare_parameter("rplidar.udp_port", 8089);
    const std::string SERIAL_PORT     = this->declare_parameter("rplidar.serial.port", "/dev/ttyUSB0");
    const int         SERIAL_BAUDRATE = this->declare_parameter("rplidar.serial.baudrate", 115200);
    m_param.frame_id         = this->declare_parameter("rplidar.frame_id", "rplidar");
    m_param.inverted         = this->declare_parameter("rplidar.inverted", false);
    m_param.angle_compensate = this->declare_parameter("rplidar.angle_compensate", false);
    m_param.scan_mode        = this->declare_parameter("rplidar.scan_mode", "");
    m_param.scan_frequency   = this->declare_parameter("rplidar.scan_frequency", CHANNEL_TYPE == "udp" ? 20.0 : 10.0);
    m_param.offset_theta     = this->declare_parameter("rplidar.offset.theta", -180.0) * TO_RAD;
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize lidar
    RCLCPP_INFO(this->get_logger(), "Initialize lidar...");
    RCLCPP_INFO(this->get_logger(), "RPLIDAR running on ROS package rplidar_ros, SDK Version:%d.%d.%d",
        SL_LIDAR_SDK_VERSION_MAJOR,
        SL_LIDAR_SDK_VERSION_MINOR,
        SL_LIDAR_SDK_VERSION_PATCH
    );
    m_driver = *sl::createLidarDriver();
    sl::IChannel *channel;
    if(CHANNEL_TYPE == "tcp")      channel = *sl::createTcpChannel(TCP_IP, TCP_PORT);
    else if(CHANNEL_TYPE == "udp") channel = *sl::createUdpChannel(UDP_IP, UDP_PORT);
    else                                 channel = *sl::createSerialPortChannel(SERIAL_PORT, SERIAL_BAUDRATE);
    if(SL_IS_FAIL(m_driver->connect(channel))){
        if(CHANNEL_TYPE == "tcp")      RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the ip addr %s with the tcp port %d", TCP_IP.c_str(), TCP_PORT);
        else if(CHANNEL_TYPE == "udp") RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the ip addr %s with the udp port %d", UDP_IP.c_str(), UDP_PORT);
        else                           RCLCPP_ERROR(this->get_logger(), "Error, cannot bind to the specified serial port %s.", SERIAL_PORT.c_str());
        return;
    }
    if(!this->getRPLIDARDeviceInfo(m_driver)) return;
    if(!this->checkRPLIDARHealth(m_driver)) return;
    m_driver->setMotorSpeed();
    RCLCPP_INFO(this->get_logger(), "Complete! lidar was initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("~/scan", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    m_set_active_service = this->create_service<std_srvs::srv::SetBool>("~/set_active", std::bind(&RPLiDAR::serviceSetActive, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&RPLiDAR::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
RPLiDAR::~RPLiDAR()
{
    m_driver->setMotorSpeed(0);
    m_driver->stop();
    delete m_driver;
    m_thread.release();
}

void RPLiDAR::serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                                const std_srvs::srv::SetBool::Request::SharedPtr request,
                                const std_srvs::srv::SetBool::Response::SharedPtr response)
{
    if(m_driver) response->message = "failure";
    uint32_t ans;
    if(request->data){
        if(m_driver->isConnected()){
            ans = m_driver->setMotorSpeed();
            ans |= m_driver->startScan(0, true);
        }
    }
    else m_driver->setMotorSpeed(0);
    response->success = SL_IS_OK(ans);
    response->message = "ok";
}

void RPLiDAR::toROS2LaserScan(sensor_msgs::msg::LaserScan *laser, 
        sl_lidar_response_measurement_node_hq_t *nodes, const size_t &node_count, 
        const rclcpp::Time &stamp, const double &scan_time,
        const float &angle_min, const float &angle_max,
        const float &distance_max)
{
    laser->header.stamp = stamp;
    laser->header.frame_id = m_param.frame_id;
    bool reversed = (angle_max > angle_min);
    if(reversed){
        laser->angle_min = M_PI - angle_max;
        laser->angle_max = M_PI - angle_min;
    }
    else{
        laser->angle_min = M_PI - angle_min;
        laser->angle_max = M_PI - angle_max;
    }
    // laser->angle_min = angle_min;
    // laser->angle_max = angle_max;

    laser->angle_increment = (laser->angle_max - laser->angle_min)/(double)(node_count-1);
    laser->scan_time = scan_time;
    laser->time_increment = scan_time / (double)(node_count-1);
    laser->range_min = 0.15;
    laser->range_max = distance_max;
    laser->intensities.resize(node_count);
    laser->ranges.resize(node_count);

    if( (!m_param.inverted && reversed) || (m_param.inverted && !reversed)){
        for(size_t i=0; i<node_count; i++){
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if(read_value == 0.0) laser->ranges[i] = std::numeric_limits<float>::infinity();
            else                  laser->ranges[i] = read_value;
            laser->intensities[i] = (float)(nodes[i].quality >> 2);
        }
    }
    else{
        for(size_t i=0; i<node_count; i++){
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if(read_value == 0.0) laser->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else                  laser->ranges[node_count-1-i] = read_value;
            laser->intensities[node_count-1-i] = (float)(nodes[i].quality >> 2);
        }
    }
}

bool RPLiDAR::getRPLIDARDeviceInfo(sl::ILidarDriver *driver)
{
    uint32_t op_result;
    sl_lidar_response_device_info_t devinfo;

    op_result = driver->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        else                                          RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: %x",op_result);
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    char sn_str[35] = {0}; 
    char mode_str[16] = {0};
    for (int pos = 0; pos < 16 ;++pos) std::sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
    if((devinfo.model>>4) <= LIDAR_S_SERIES_MINUM_MAJOR_ID)      std::sprintf(mode_str,"A%dM%d", (devinfo.model>>4), (devinfo.model&0xf));
    else if((devinfo.model>>4) <= LIDAR_T_SERIES_MINUM_MAJOR_ID) std::sprintf(mode_str,"S%dM%d", (devinfo.model>>4)-LIDAR_S_SERIES_MINUM_MAJOR_ID, (devinfo.model&0xf));
    else                                                         std::sprintf(mode_str,"T%dM%d", (devinfo.model>>4)-LIDAR_T_SERIES_MINUM_MAJOR_ID, (devinfo.model&0xf));

    RCLCPP_INFO(this->get_logger(), "RPLIDAR MODE:%s",mode_str);
    RCLCPP_INFO(this->get_logger(), "RPLIDAR S/N: %s",sn_str);
    RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}

bool RPLiDAR::checkRPLIDARHealth(sl::ILidarDriver *driver)
{
    bool ret = true;
    uint32_t op_result;
    sl_lidar_response_device_health_t health_info;

    op_result = driver->getHealth(health_info);
    if (SL_IS_OK(op_result)) { 
        RCLCPP_INFO(this->get_logger(), "RPLidar health status : %d", health_info.status);
        if(health_info.status == SL_LIDAR_STATUS_OK)           RCLCPP_INFO(this->get_logger(), "RPLidar health status : OK.");
        else if(health_info.status == SL_LIDAR_STATUS_WARNING) RCLCPP_INFO(this->get_logger(), "RPLidar health status : Warning.");
        else if(health_info.status == SL_LIDAR_STATUS_ERROR){
            RCLCPP_ERROR(this->get_logger(), "Error, rplidar internal error detected. Please reboot the device to retry.");
            ret = false;
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Error, Unknown internal error detected. Please reboot the device to retry.");
            ret = false;
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve rplidar health code: %x", op_result);
        ret = false;
    }
    return ret;
}

/**
 * @brief Execute method
 * 
 */
void RPLiDAR::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());

    uint32_t op_result;
    sl::LidarScanMode current_scan_mode;
    if(m_param.scan_mode.empty()) op_result = m_driver->startScan(false, true, 0, &current_scan_mode); // not force scan, use_typical scan mode
    else{
        std::vector<sl::LidarScanMode> allSupportedScanModes;
        op_result = m_driver->getAllSupportedScanModes(allSupportedScanModes);
        if(SL_IS_OK(op_result)){
            uint16_t selectedScanMode = uint16_t(-1);
            for (auto iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == m_param.scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == uint16_t(-1)) {
                RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", m_param.scan_mode.c_str());
                for (auto iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    RCLCPP_ERROR(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", 
                        iter->scan_mode, 
                        iter->max_distance, 
                        (1000/iter->us_per_sample)
                    );
                }
                op_result = SL_RESULT_OPERATION_FAIL;
            } 
            else op_result = m_driver->startScanExpress(false , selectedScanMode, 0, &current_scan_mode); // not force scan
        }
    }
    
    int points_per_circle = 0;
    float angle_compensate_multiple = 0;
    float max_distance = 0;
    if(SL_IS_OK(op_result)){
        points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/m_param.scan_frequency);
        angle_compensate_multiple = points_per_circle/360.0 + 1;
        if(angle_compensate_multiple < 1) angle_compensate_multiple = 1.0;
        max_distance = (float)current_scan_mode.max_distance;
        RCLCPP_INFO(this->get_logger(), "current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ", 
            current_scan_mode.scan_mode,
            (int)(1000/current_scan_mode.us_per_sample+0.5), 
            max_distance, 
            m_param.scan_frequency);
    }
    else RCLCPP_ERROR(this->get_logger(), "Can not start scan: %08x", op_result);

    // Main loop
    for(rclcpp::WallRate loop(m_param.scan_frequency * 2); rclcpp::ok(); loop.sleep()){
        sl_lidar_response_measurement_node_hq_t nodes[8192] = {0};
        size_t count = sizeof(nodes)/sizeof(nodes[0]);
        
        const auto stamp = this->now();
        if(m_driver->grabScanDataHq(nodes, count) != SL_RESULT_OK) continue;
        const double scan_time = (this->now() - stamp).seconds();

        float angle_min = 0.0;
        float angle_max = 360.0 * TO_RAD;
        sl_lidar_response_measurement_node_hq_t *corrected_nodes = nodes;
        size_t corrected_node_count = count;
        if(m_driver->ascendScanData(nodes, count) == SL_RESULT_OK){
            if(m_param.angle_compensate){
                const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
                sl_lidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(sl_lidar_response_measurement_node_hq_t));

                int angle_compensate_offset = 0;
                for(size_t i=0; i<count; i++){
                    if(nodes[i].dist_mm_q2 != 0){
                        int angle_value = (int)(this->getAngle(nodes[i]) * angle_compensate_multiple);
                        if((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        for(int j=0; j<angle_compensate_multiple; j++){
                            int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
                            if(angle_compensate_nodes_index >= angle_compensate_nodes_count){
                                angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                            }
                            angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                        }
                    }
                }

                corrected_nodes = angle_compensate_nodes;
                corrected_node_count = angle_compensate_nodes_count;
            }
            else{
                int i=0, j=count-1;
                while(nodes[i++].dist_mm_q2 == 0);
                while(nodes[j--].dist_mm_q2 == 0);

                angle_min = this->getAngle(nodes[i-1]) * TO_RAD;
                angle_max = this->getAngle(nodes[j+1]) * TO_RAD;

                corrected_nodes = &nodes[i];
                corrected_node_count = j-i+1;
            }
        }

        RCLCPP_INFO(this->get_logger(), "angle min: %f", angle_min);
        RCLCPP_INFO(this->get_logger(), "angle max: %f", angle_max);
        RCLCPP_INFO(this->get_logger(), "scan time: %lf", scan_time);


        auto laser_scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();        
        this->toROS2LaserScan(laser_scan_msg.get(), 
            corrected_nodes, corrected_node_count, 
            stamp, scan_time,
            angle_min, angle_max, 
            max_distance
        );

        m_laser_scan_publisher->publish(std::move(laser_scan_msg));
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::RPLiDAR)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
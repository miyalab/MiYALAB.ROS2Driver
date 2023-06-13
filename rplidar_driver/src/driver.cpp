//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "rplidar_driver/driver.hpp"

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
    const std::string SERIAL_PORT     = this->declare_parameter("rplidar.serial_port", "/dev/ttyUSB0");
    const int         SERIAL_BAUDRATE = this->declare_parameter("rplidar.serial_baudrate", 115200);
    this->forceSet(&this->FRAME_ID        , this->declare_parameter("rplidar.frame_id", "laser_frame"));
    this->forceSet(&this->INVERTED        , this->declare_parameter("rplidar.inverted", false));
    this->forceSet(&this->ANGLE_COMPENSATE, this->declare_parameter("rplidar.angle_compensate", false));
    this->forceSet(&this->SCAN_MODE       , this->declare_parameter("rplidar.scan_mode", ""));
    if(CHANNEL_TYPE == "udp") this->forceSet(&this->SCAN_FREQUENCY, this->declare_parameter("rplidar.scan_frequency", 20.0));
    else                      this->forceSet(&this->SCAN_FREQUENCY, this->declare_parameter("rplidar.scan_frequency", 10.0));
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize lidar
    RCLCPP_INFO(this->get_logger(), "Initialize lidar...");
    RCLCPP_INFO(this->get_logger(), "RPLIDAR running on ROS package rplidar_ros, SDK Version:%d.%d.%d",
        SL_LIDAR_SDK_VERSION_MAJOR,
        SL_LIDAR_SDK_VERSION_MINOR,
        SL_LIDAR_SDK_VERSION_PATCH
    );
    this->driver = *sl::createLidarDriver();
    sl::IChannel *channel;
    if(CHANNEL_TYPE == "tcp")      channel = *sl::createTcpChannel(TCP_IP, TCP_PORT);
    else if(CHANNEL_TYPE == "udp") channel = *sl::createUdpChannel(UDP_IP, UDP_PORT);
    else                                 channel = *sl::createSerialPortChannel(SERIAL_PORT, SERIAL_BAUDRATE);
    if(SL_IS_FAIL(this->driver->connect(channel))){
        if(CHANNEL_TYPE == "tcp")      RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the ip addr %s with the tcp port %d", TCP_IP.c_str(), TCP_PORT);
        else if(CHANNEL_TYPE == "udp") RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the ip addr %s with the udp port %d", UDP_IP.c_str(), UDP_PORT);
        else                           RCLCPP_ERROR(this->get_logger(), "Error, cannot bind to the specified serial port %s.", SERIAL_PORT.c_str());
        return;
    }
    if(!this->getRPLIDARDeviceInfo(this->driver)) return;
    if(!this->checkRPLIDARHealth(this->driver)) return;
    RCLCPP_INFO(this->get_logger(), "Complete! lidar was initialized.");

    // Initialize subscriber
    // RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    this->laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("~/scan", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    this->set_active_service = this->create_service<std_srvs::srv::SetBool>("~/set_active", std::bind(&RPLiDAR::serviceSetActive, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    // RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&RPLiDAR::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
RPLiDAR::~RPLiDAR()
{
    this->driver->setMotorSpeed(0);
    this->driver->stop();
    delete this->driver;
    this->thread.release();
}

void RPLiDAR::serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                                const std_srvs::srv::SetBool::Request::SharedPtr request,
                                const std_srvs::srv::SetBool::Response::SharedPtr response)
{
    if(this->driver) response->message = "failure";
    uint32_t ans;
    if(request->data){
        if(this->driver->isConnected()){
            ans = this->driver->setMotorSpeed();
            ans |= driver->startScan(0, true);
        }
    }
    else driver->setMotorSpeed(0);
    response->success = SL_IS_OK(ans);
    response->message = "ok";
}

void RPLiDAR::toROS2LaserScan(sensor_msgs::msg::LaserScan *laser, 
        sl_lidar_response_measurement_node_hq_t *nodes, const size_t &node_count, 
        const rclcpp::Time &start, const double &scan_time,
        const float &angle_min, const float &angle_max,
        const float &distance_max)
{
    laser->header.stamp = start;
    laser->header.frame_id = this->FRAME_ID;
    bool reversed = (angle_max > angle_min);
    if(reversed){
        laser->angle_min = M_PI - angle_max;
        laser->angle_max = M_PI - angle_min;
    }
    else{
        laser->angle_min = M_PI - angle_min;
        laser->angle_max = M_PI - angle_max;
    }

    laser->angle_increment = (laser->angle_max - laser->angle_min)/(double)(node_count-1);
    laser->scan_time = scan_time;
    laser->time_increment = scan_time / (double)(node_count-1);
    laser->range_min = 0.15;
    laser->range_max = distance_max;
    laser->intensities.resize(node_count);
    laser->ranges.resize(node_count);

    bool reverse_data = (!this->INVERTED && reversed) || (this->INVERTED && !reversed);
    if(!reverse_data){
        for(size_t i=0; i<node_count; i++){
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if(read_value == 0.0) laser->ranges[i] = std::numeric_limits<float>::infinity();
            else                  laser->ranges[i] = read_value;
            laser->intensities[i] = (float)(nodes[i].quality >> 2);
        }
    }
    else{
        for(size_t i=0; i<node_count; i++){
            float read_value = (float)nodes[i].dist_mm_q2/4.0/1000;
            if(read_value == 0.0) laser->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else                  laser->ranges[node_count-1-i] = read_value;
            laser->intensities[node_count-1-i] = (float)(nodes[i].quality>>2);
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
    
    sl_lidar_response_device_info_t device_info;
    uint32_t op_result = this->driver->getDeviceInfo(device_info);
    bool scan_frequency_turning_after_scan = false;
    if((device_info.model >> 4) > LIDAR_S_SERIES_MINUM_MAJOR_ID) scan_frequency_turning_after_scan = true;
    if(!scan_frequency_turning_after_scan) driver->setMotorSpeed(600);
    sl::LidarScanMode current_scan_mode;
    if(SCAN_MODE.empty()){
        op_result = this->driver->startScan(false, true, 0, &current_scan_mode); // not force scan, use_typical scan mode
    }
    else{
        std::vector<sl::LidarScanMode> allSupportedScanModes;
        op_result = this->driver->getAllSupportedScanModes(allSupportedScanModes);
        if(SL_IS_OK(op_result)){
            uint16_t selectedScanMode = uint16_t(-1);
            for (std::vector<sl::LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == SCAN_MODE) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == uint16_t(-1)) {
                RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", this->SCAN_MODE.c_str());
                for (std::vector<sl::LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    RCLCPP_ERROR(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", 
                        iter->scan_mode, iter->max_distance, (1000/iter->us_per_sample)
                    );
                }
                op_result = SL_RESULT_OPERATION_FAIL;
            } 
            else op_result = this->driver->startScanExpress(false , selectedScanMode, 0, &current_scan_mode); // not force scan
        }
    }
    
    int points_per_circle = 0;
    float angle_compensate_multiple = 0;
    float max_distance = 0;
    if(SL_IS_OK(op_result)){
        points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/this->SCAN_FREQUENCY);
        angle_compensate_multiple = points_per_circle/360.0 + 1;
        if(angle_compensate_multiple < 1) angle_compensate_multiple = 1.0;
        max_distance = (float)current_scan_mode.max_distance;
        RCLCPP_INFO(this->get_logger(), "current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ", 
            current_scan_mode.scan_mode,
            (int)(1000/current_scan_mode.us_per_sample+0.5), 
            max_distance, 
            this->SCAN_FREQUENCY);
    }
    else RCLCPP_ERROR(this->get_logger(), "Can not start scan: %08x", op_result);

    // Main loop
    for(rclcpp::WallRate loop(this->SCAN_FREQUENCY); rclcpp::ok(); loop.sleep()){
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes)/sizeof(nodes[0]);
        
        const auto start_scan_time = this->now();
        uint32_t op_result = this->driver->grabScanDataHq(nodes, count);
        const auto end_scan_time = this->now();
        const double scan_duration = (long long)(end_scan_time.seconds()*1e9 + end_scan_time.nanoseconds() - start_scan_time.seconds()*1e9 - start_scan_time.nanoseconds()) /1e9;

        if(op_result != SL_RESULT_OK) continue;

        if(scan_frequency_turning_after_scan){
            RCLCPP_INFO(this->get_logger(), "set lidar scan freq %.1f Hz(%.1f rpm)", this->SCAN_FREQUENCY, SCAN_FREQUENCY*60);
            this->driver->setMotorSpeed(this->SCAN_FREQUENCY*60);
            scan_frequency_turning_after_scan = false;            
            continue;
        }
        op_result = this->driver->ascendScanData(nodes, count);

        auto laser_scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        if(op_result == SL_RESULT_OK){
            float angle_min = 0;
            float angle_max = 360 * TO_RAD;
            if(angle_compensate_multiple){
                const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
                int angle_compensate_offset = 0;
                sl_lidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                for(size_t i=0; i<count; i++){
                    if(nodes[i].dist_mm_q2 != 0){
                        int angle_value = (int)(this->getAngle(nodes[i]) * angle_compensate_multiple);
                        if((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        for(int j=0; j<angle_compensate_multiple; j++){
                            int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
                            if(angle_compensate_nodes_index >= angle_compensate_nodes_count){
                                angle_compensate_nodes_index -= 1;
                            }
                            angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                        }
                    }
                }

                this->toROS2LaserScan(laser_scan_msg.get(),
                    angle_compensate_nodes, angle_compensate_nodes_count,
                    start_scan_time, scan_duration,
                    angle_min, angle_max, 
                    max_distance
                );
            }
            else{
                int start_node = 0, end_node = 0;
                int i=0;
                for(i=0; nodes[i].dist_mm_q2 == 0; i++);
                start_node = i;
                for(i=count-1; nodes[i].dist_mm_q2 == 0; i--);
                end_node = i;

                angle_min = this->getAngle(nodes[start_node]) * TO_RAD;
                angle_max = this->getAngle(nodes[end_node])   * TO_RAD;

                this->toROS2LaserScan(laser_scan_msg.get(),
                    &nodes[start_node], end_node-start_node+1,
                    start_scan_time, scan_duration,
                    angle_min, angle_max, 
                    max_distance);
            }
        }
        else if(op_result == SL_RESULT_OPERATION_FAIL){
            float angle_min = 0;
            float angle_max = 359.0 * TO_RAD;

            this->toROS2LaserScan(laser_scan_msg.get(), 
                nodes, count, 
                start_scan_time, scan_duration,
                angle_min, angle_max, 
                max_distance
            );
        }
        laser_scan_publisher->publish(std::move(laser_scan_msg));
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
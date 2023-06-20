/*
 * MIT License
 * 
 * Copyright (c) 2023 MiYA LAB(K.Miyauchi)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

//-----------------------------
// include
//-----------------------------
// STL
#include <iostream>
#include <string>
#include <memory>
#include <future>

// Boost
#include <boost/array.hpp>
#include <boost/asio/ip/udp.hpp>

// MiYALAB
#include "MiYALAB/Sensor/RFansLiDAR/driver.hpp"
#include "MiYALAB/Sensor/RFansLiDAR/parameter_define.hpp"

//-----------------------------
// Namespace & using
//-----------------------------
using namespace boost::asio;
using namespace boost::asio::ip;    

//-----------------------------
// Const value
//-----------------------------
constexpr double TO_RAD = M_PI / 180;

constexpr unsigned char CMD_SETTING_SET[18] = {
    0xA5, 0x72, 0x00, 0x70, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};
constexpr unsigned char CMD_SCAN_START[][18] = {
    {0xA5, 0x42, 0x00, 0x40, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xA5, 0x43, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xA5, 0x93, 0x00, 0x40, 0x00, 0x00, 0x00, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xA5, 0x33, 0x00, 0x40, 0x00, 0x00, 0x00, 0xF3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};
constexpr unsigned char CMD_SCAN_STOP[18] = {
    0xA5, 0x40, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//-----------------------------
// Method
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace Sensor{

RFansDriver::RFansDriver(const std::string &ip_address, const int &status_port, const std::string &model)
{
    // 型番チェック
    for(int i=0; i<4; i++){
        if(model == RFansParams::MODEL_NAME[i]) this->forceSet(&this->MODEL, i);
    }
    if(MODEL == -1) throw "The input model name does not match: " + model;

    // デバイス情報ポート接続
    io_service io;
    this->status_socket = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status_port));

    // デバイス情報取得
    RFansDeviceStatus status;
    for(int i=0; i<10 || !this->getDeviceInfo(&status); i++);
    if(status.mac_address == "") return;

    // 点群ポート,　コマンドポート接続
    this->forceSet(&this->IP_ADDRESS, *((std::string*)&ip_address));
    this->forceSet(&this->COMMAND_PORT, status.command_port);
    this->points_socket  = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status.points_port));
    this->command_socket = std::make_shared<udp::socket>(io, udp::endpoint(udp::v4(), status.command_port));
}

RFansDriver::~RFansDriver()
{
    this->scanStop();
    status_socket = nullptr;
    points_socket = nullptr;
    command_socket = nullptr;
}

bool RFansDriver::scanStart(const int &hz)
{
    // パラメータチェック
    int command_num = -1;
    if(hz == 0)       command_num = 0;
    else if(hz == 5 ) command_num = 1;
    else if(hz == 10) command_num = 2;
    else if(hz == 20) command_num = 3;
    if(command_num == -1) return false;
    
    // パラメータラッチ
    this->forceSet(&this->HZ, hz);
    this->forceSet(&this->ANGULAR_VEL, this->HZ * 360 / 1e6); // [deg/us]
    
    // コマンド送信
    command_socket->send_to(
        boost::asio::buffer(CMD_SETTING_SET),
        udp::endpoint(address::from_string(this->IP_ADDRESS), this->COMMAND_PORT)
    );
    command_socket->send_to(
        boost::asio::buffer(CMD_SCAN_START[command_num]),
        udp::endpoint(address::from_string(this->IP_ADDRESS), this->COMMAND_PORT)
    );

    // 目標スキャン速度チェック
    int i;
    for(i=0; i<10; i++){
        RFansDeviceStatus status;
        this->getDeviceInfo(&status);
        if(hz -1.0 <= status.motor_speed && status.motor_speed <= hz + 1.0) break;
    }
    if(i==10) return false;
    return true;
}

bool RFansDriver::scanStop()
{
    this->forceSet(&this->HZ, 0);
    
    // コマンド送信
    command_socket->send_to(
        boost::asio::buffer(CMD_SETTING_SET),
        udp::endpoint(address::from_string(this->IP_ADDRESS), this->COMMAND_PORT)
    );
    command_socket->send_to(
        boost::asio::buffer(CMD_SCAN_STOP),
        udp::endpoint(address::from_string(this->IP_ADDRESS), this->COMMAND_PORT)
    );
    return true;
}

bool RFansDriver::getDeviceInfo(RFansDeviceStatus *status)
{
    char buf[32];
    boost::array<char, 256> recv_data;
    udp::endpoint endpoint;
    size_t len = status_socket->receive_from(boost::asio::buffer(recv_data), endpoint);
    if(len < 256) return false;
    status->header = (recv_data[0] & 0xff) << 24 | (recv_data[1] & 0xff) << 16 | (recv_data[2] & 0xff) << 8 | (recv_data[3] & 0xff);
    status->id     = (recv_data[4] & 0xff) << 24 | (recv_data[5] & 0xff) << 16 | (recv_data[6] & 0xff) << 8 | (recv_data[7] & 0xff);
    status->year   = recv_data[8] & 0xff;
    status->month  = recv_data[9]  & 0xff;
    status->day    = recv_data[10] & 0xff;
    status->hour   = recv_data[11] & 0xff;
    status->minute = recv_data[12] & 0xff;
    status->second = recv_data[13] & 0xff;
    std::snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x", recv_data[14]&0xff, recv_data[15]&0xff, recv_data[16]&0xff, recv_data[17]&0xff, recv_data[18]&0xff, recv_data[19]&0xff);
    status->mac_address  = buf;
    status->points_port  = (recv_data[20] & 0xff) << 8 | (recv_data[21] & 0xff);
    status->command_port = (recv_data[22] & 0xff) << 8 | (recv_data[23] & 0xff);
    status->motor_speed  = (recv_data[24] & 0xff) / 10.0;
    status->device_info  = (recv_data[25] & 0xff) << 24 | (recv_data[26] & 0xff) << 16 | (recv_data[27] & 0xff) << 8 | (recv_data[28] & 0xff);
    status->pps_encode   = (recv_data[29] & 0xff) << 8 | (recv_data[30] & 0xff);
    status->device_id    = (recv_data[31] & 0xff) << 8 | (recv_data[32] & 0xff);
    status->temperature  =((recv_data[33] & 0xff) << 8 | (recv_data[34] & 0xff)) / 100.0;
    status->check_sum    = (recv_data[35] & 0xff) << 24 | (recv_data[36] & 0xff) << 16 | (recv_data[37] & 0xff) << 8 | (recv_data[38] & 0xff);
    return true;
}

bool RFansDriver::getPoints(MiYALAB::Sensor::PointCloudPolar *polars)
{
    double divide = 12.0;
    if(this->MODEL <= 1) divide *= 2;   // R-Fans-16 or R-Fans-32
    const double loop_count = 360.0 / (0.09 * this->HZ / 5.0) / divide;
    double angle_sum = 0;
    double angle_before = 0;

    double theta_time_offset[32];
    for(int i=0; i<32; i++) theta_time_offset[i] = (this->ANGULAR_VEL) * RFansParams::DELTA_TIME_US[this->MODEL][i];
    
    for(int k=0; k<loop_count && angle_sum < 360; k++){
        char recv_data[1206];
        udp::endpoint endpoint;
        size_t len = points_socket->receive_from(boost::asio::buffer(recv_data), endpoint);
        if(len < 1206) continue;
        for(int i=0; i<12; i++){
            auto *group = &recv_data[i*100];
            double angle = ((group[3] & 0xff) << 8 | (group[2] & 0xff)) / 100.0 - 180.0;
            for(int j=0; j<32; j++){
                auto *point = &group[3*j+4];
                double range = ((point[1] & 0xff) << 8 | (point[0] & 0xff)) * 0.004;
                if(range > RFansParams::RANGE_MAX || range == 0) continue;
                polars->polars.emplace_back(
                    range,
                    -(angle + RFansParams::HORIZONTAL_THETA[this->MODEL][j] + theta_time_offset[j]) * TO_RAD,
                    RFansParams::VERTICAL_THETA[this->MODEL][j] * TO_RAD
                );
                polars->intensity.emplace_back((point[2] & 0xff) / 255.0);
            }
            double diff = angle - angle_before;
            diff += 360.0 * (diff<0);
            diff *= diff<1;
            angle_sum += diff;
            angle_before = angle;
        }
    }

    // 角度の正規化
    for(int i=0, size=polars->polars.size(); i<size; i++){
        if(polars->polars[i].theta < -M_PI)      polars->polars[i].theta += 2.0 * M_PI;
        else if(polars->polars[i].theta >  M_PI) polars->polars[i].theta -= 2.0 * M_PI;
    }

    // std::cout << "diff: " << k << ", " << angle_sum << std::endl;
    return true;
}
}
}

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------
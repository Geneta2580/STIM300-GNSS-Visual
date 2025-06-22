#ifndef GNSS_DRIVER_H
#define GNSS_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/NavSatFix.h> 
#include <string>
#include <vector>
#include <cstdint> // For uint8_t, int32_t, uint32_t etc.
#include <array>

// For serial port communication (POSIX)
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

class GnssSerialProcessor {
public:
    GnssSerialProcessor();  // 构造函数
    ~GnssSerialProcessor(); // 析构函数

    bool init(); // 初始化节点，加载参数，打开串口
    void run();  // 主循环，读取和发布数据

private:
    // ROS句柄和发布器
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher gnss_pub_;
    ros::Rate* loop_rate_;

    // 串口参数和文件描述符
    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    double publish_rate_hz_;
    int serial_fd_;
    bool initialized_;

    // 串口读取缓冲区 (用于二进制数据)
    std::vector<uint8_t> byte_buffer_;

    // UBX数据包常量
    static const uint8_t UBX_SYNC_CHAR1 = 0xB5; // UBX 协议同步字符1
    static const uint8_t UBX_SYNC_CHAR2 = 0x62; // UBX 协议同步字符2
    static const uint8_t UBX_CLASS_NAV = 0x01;
    static const uint8_t UBX_ID_NAV_PVT = 0x07;
    static const uint8_t UBX_ID_NAV_COV = 0x36; // UBX-NAV-COV 消息 ID

    // 私有辅助函数
    int configure_serial_port(const std::string& port_name, int baud_rate);

    // 新的二进制解析函数
    bool process_byte_buffer(); // 在缓冲区中查找并处理数据包
    bool parse_ubx_nav_pvt_packet(const uint8_t* payload_data, uint16_t payload_length, sensor_msgs::NavSatFix& msg); // 用于 NAV-PVT
    bool parse_ubx_nav_cov_packet(const uint8_t* payload_data, uint16_t payload_length); // 用于 NAV-COV
    bool verify_ubx_checksum(const uint8_t* packet_data, size_t length_without_checksum); 

    // 用于存储来自NAV-COV的最新协方差矩阵的成员
    std::array<double, 9> current_covariance_matrix_;
    bool has_fresh_covariance_;
    uint32_t last_covariance_itow_; // 用于可能与NAV-PVT的iTOW匹配
};

#endif
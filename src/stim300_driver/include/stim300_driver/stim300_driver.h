#ifndef STIM300_DRIVER_H
#define STIM300_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <string>
#include <vector>
#include <cstdint> // For uint8_t, int32_t, uint32_t etc.

// For serial port communication (POSIX)
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

class ImuSerialProcessor {
public:
    ImuSerialProcessor();  // 构造函数
    ~ImuSerialProcessor(); // 析构函数

    bool init(); // 初始化节点，加载参数，打开串口
    void run();  // 主循环，读取和发布数据

private:
    // ROS句柄和发布器
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher imu_pub_;
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

    // STIM300 数据包常量
    static const uint8_t FRAME_HEADER = 0x93;
    static const size_t PACKET_LENGTH = 38; // 根据描述计算的总包长

    const double GYRO_SCALE_FACTOR = 0.0000476837; //  (400 deg/s / 2^23)
    const double ACCEL_SCALE_FACTOR = 0.00003505302;   // (30g / 2^23) * 9.801535 m/s^2/g 北京
    // const double INCLI_SCALE_FACTOR = 0.001; // 如果需要处理倾角计数据

    // CRC32 计算相关
    static uint32_t crc32_table[256]; // CRC查找表
    static bool crc_table_initialized;
    static void initialize_crc32_table(); // 初始化查找表的函数
    static uint32_t reflect_bits(uint32_t data, uint8_t num_bits); // 位反转辅助函数
    uint32_t compute_crc32_for_stim300(const uint8_t* data, size_t length); // 实际的CRC计算函数，由 calculate_crc32 调用

    // 私有辅助函数
    int configure_serial_port(const std::string& port_name, int baud_rate);

    // 新的二进制解析函数
    bool process_byte_buffer(); // 在缓冲区中查找并处理数据包
    bool parse_stim300_packet(const uint8_t* packet_data, sensor_msgs::Imu& msg); // 解析单个数据包
    uint32_t calculate_crc32(const uint8_t* data, size_t length); // CRC32校验函数 (占位符)
    int32_t convert_24bit_signed(const uint8_t* bytes); // 转换3字节有符号整数
};

#endif
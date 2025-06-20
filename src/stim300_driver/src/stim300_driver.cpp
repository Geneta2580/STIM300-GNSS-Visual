#include <stim300_driver/stim300_driver.h>
#include <iomanip> // 用于调试时打印16进制 (std::hex, std::setw, std::setfill)
#include <cstring> // For strerror

// uint8_t就是一个字节的数据
// STIM300 CRC计算
uint32_t ImuSerialProcessor::calculate_crc32(const uint8_t* data, size_t length) {
    // 1. 计算需要填充的虚拟字节数量
    size_t num_dummy_bytes = (4 - (length % 4)) % 4;
    size_t total_length_for_crc = length + num_dummy_bytes;

    // 2. 创建一个新的数据缓冲区，包含原始数据和填充字节
    std::vector<uint8_t> data_with_padding;
    data_with_padding.reserve(total_length_for_crc);
    data_with_padding.insert(data_with_padding.end(), data, data + length);
    for (size_t i = 0; i < num_dummy_bytes; ++i) {
        data_with_padding.push_back(0x00);
    }

    // 3. 对包含填充字节的完整数据块进行CRC计算 (MSB优先逻辑)
    uint32_t crc = 0xFFFFFFFF; // Initial CRC value (通常的标准初始值)
    const uint32_t polynomial = 0x04C11DB7; // CRC32正常形式多项式
    const uint8_t* current_byte_ptr = data_with_padding.data(); // 指向新数据块的开头

    for (size_t i = 0; i < total_length_for_crc; ++i) {
        // 将当前字节与CRC的高8位异或
        crc ^= (static_cast<uint32_t>(*current_byte_ptr++) << 24);  // 将字节移到32位字的最高8位

        for (int j = 0; j < 8; ++j) { // 处理字节中的每一位 (从MSB开始)
            if (crc & 0x80000000) { // 如果CRC的最高位 (MSB) 是1
                crc = (crc << 1) ^ polynomial; // 左移一位，并与多项式异或
            } else {
                crc = (crc << 1); // 左移一位
            }
        }
    }

    return crc ^ 0x00000000; // 相当于直接返回 crc
}

// Constructor
ImuSerialProcessor::ImuSerialProcessor() :
    private_nh_("~"),
    serial_fd_(-1),
    initialized_(false),
    loop_rate_(nullptr)
{
    ROS_INFO("ImuSerialProcessor constructor.");
}

// Destructor
ImuSerialProcessor::~ImuSerialProcessor() {
    if (serial_fd_ != -1) {
        close(serial_fd_);
        ROS_INFO("Serial port closed.");
    }
    if (loop_rate_) {
        delete loop_rate_;
    }
    ROS_INFO("ImuSerialProcessor destructor.");
}

bool ImuSerialProcessor::init() {
    // 修改参数名 "serial_port_fd" 为 "serial_port"
    private_nh_.param<std::string>("serial_port", this->port_name_, "/dev/ttyUSB0");
    private_nh_.param<int>("baud_rate", this->baud_rate_, 921600);
    private_nh_.param<std::string>("frame_id", this->frame_id_, "imu_link");
    private_nh_.param<double>("publish_rate", this->publish_rate_hz_, 125.0);

    ROS_INFO("Parameters: Port: %s, Baud: %d, Frame ID: %s, Rate: %.1f Hz",
            port_name_.c_str(), baud_rate_, frame_id_.c_str(), publish_rate_hz_);
    
    serial_fd_ = configure_serial_port(port_name_, baud_rate_);
    if (serial_fd_ < 0) {
        ROS_ERROR("Failed to configure or open serial port.");
        return false;
    }
    ROS_INFO("Serial port opened successfully with fd: %d", serial_fd_);

    // nh_ = ros::NodeHandle(); // 可以在构造函数中初始化，或者在这里
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 50); // 发布到 imu/data_raw
    ROS_INFO("IMU data publisher initialized on topic 'imu/data_raw'.");

    if (publish_rate_hz_ > 0) {
        loop_rate_ = new ros::Rate(publish_rate_hz_);
    } else {
        ROS_WARN("Publish rate is <= 0, defaulting to 125 Hz for loop rate.");
        loop_rate_ = new ros::Rate(125.0);
    }

    initialized_ = true;
    ROS_INFO("ImuSerialProcessor initialized successfully.");
    return true;
}

int ImuSerialProcessor::configure_serial_port(const std::string& port_name, int baud_rate) {
    int serial_port_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port_fd < 0) {
        ROS_ERROR("Error %i from open(%s): %s", errno, port_name.c_str(), strerror(errno)); // 添加串口名
        return -1;
    }

    termios tty;
    if (tcgetattr(serial_port_fd, &tty) != 0) {
        ROS_ERROR("Error %i from tcgetattr for %s: %s", errno, port_name.c_str(), strerror(errno)); // 添加串口名
        close(serial_port_fd);
        return -1;
    }

    speed_t speed;
    switch (baud_rate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 921600: speed = B921600; break;
        default:
            ROS_ERROR("Unsupported baud rate for %s: %d", port_name.c_str(), baud_rate); // 添加串口名
            close(serial_port_fd);
            return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error %i from tcsetattr for %s: %s", errno, port_name.c_str(), strerror(errno)); // 添加串口名
        close(serial_port_fd);
        return -1;
    }
    // fcntl(serial_port_fd, F_SETFL, 0); // 这一行使读取操作根据VMIN/VTIME阻塞或超时
    ROS_INFO("Serial port %s configured successfully at %d baud.", port_name.c_str(), baud_rate);
    return serial_port_fd;
}

// 将3字节有符号整数（大端字节序）转换为int32_t
int32_t ImuSerialProcessor::convert_24bit_signed(const uint8_t* bytes) {
    uint32_t u_val = (static_cast<uint32_t>(bytes[0]) << 16) |
                     (static_cast<uint32_t>(bytes[1]) << 8)  |
                     (static_cast<uint32_t>(bytes[2]));

    // 如果最高位（第24位）是1，则进行符号扩展
    if (u_val & 0x00800000) {
        return static_cast<int32_t>(u_val | 0xFF000000);
    } else {
        return static_cast<int32_t>(u_val);
    }
}

bool ImuSerialProcessor::parse_stim300_packet(const uint8_t* packet, sensor_msgs::Imu& msg) {
    // packet 指向一个长度为 PACKET_LENGTH (38字节) 的数据包的起始位置
    // 假设CRC已经校验通过

    // 根据你提供的数据格式提取数据:
    // 偏移量:
    // 0: 帧头 (0x93)
    // 1-3: 陀螺仪X (00 02 81)
    // 4-6: 陀螺仪Y (00 04 89)
    // 7-9: 陀螺仪Z (00 04 D9)
    // 10: 陀螺仪状态 (00)
    // 11-13: 加速度计X (FF F2 72)
    // 14-16: 加速度计Y (00 11 43)
    // 17-19: 加速度计Z (07 F7 AF)
    // 20: 加速度计状态 (00)
    // 21-23: 倾角计X (00 56 4B)
    // 24-26: 倾角计Y (00 7F FE)
    // 27-29: 倾角计Z (40 35 5B)
    // 30: 倾角计状态 (00)
    // 31: 自增计数器 (40)
    // 32-33: 数据延迟 (02 04)
    // 34-37: CRC (62 C4 63 99)

    int32_t raw_gyro_x = convert_24bit_signed(&packet[1]); // 拼接三个字节的数据
    int32_t raw_gyro_y = convert_24bit_signed(&packet[4]);
    int32_t raw_gyro_z = convert_24bit_signed(&packet[7]);
    // uint8_t gyro_status = packet[10]; // 可以用于检查传感器状态

    int32_t raw_accel_x = convert_24bit_signed(&packet[11]);
    int32_t raw_accel_y = convert_24bit_signed(&packet[14]);
    int32_t raw_accel_z = convert_24bit_signed(&packet[17]);
    // uint8_t accel_status = packet[20];

    // 倾角计数据 (如果需要，可以提取并用自定义消息发布，或用于姿态估计)
    // int32_t raw_incli_x = convert_24bit_signed(&packet[21]);
    // int32_t raw_incli_y = convert_24bit_signed(&packet[24]);
    // int32_t raw_incli_z = convert_24bit_signed(&packet[27]);
    // uint8_t incli_status = packet[30];

    // uint8_t counter = packet[31];
    // uint16_t latency = (static_cast<uint16_t>(packet[32]) << 8) | packet[33]; // 假设大端

    // 应用比例因子 (!!! 使用手册中的正确值 !!!)
    msg.angular_velocity.x = static_cast<double>(raw_gyro_x) * GYRO_SCALE_FACTOR;
    msg.angular_velocity.y = static_cast<double>(raw_gyro_y) * GYRO_SCALE_FACTOR;
    msg.angular_velocity.z = static_cast<double>(raw_gyro_z) * GYRO_SCALE_FACTOR;

    msg.linear_acceleration.x = static_cast<double>(raw_accel_x) * ACCEL_SCALE_FACTOR;
    msg.linear_acceleration.y = static_cast<double>(raw_accel_y) * ACCEL_SCALE_FACTOR;
    msg.linear_acceleration.z = static_cast<double>(raw_accel_z) * ACCEL_SCALE_FACTOR;

    // 方向数据 (四元数)
    // 这个数据包格式不直接提供四元数。你需要通过传感器融合算法（如EKF, Madgwick）
    // 从陀螺仪、加速度计（和磁力计，如果可用）计算姿态。
    // 或者，如果倾角计数据用于姿态，也需要转换。
    // 目前，我们将其设置为一个无效/默认值。
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0; // 表示单位四元数（无旋转）
    msg.orientation_covariance[0] = -1; // 表示方向数据不可用或未计算

    // 设置协方差 (示例值，应根据传感器手册或标定结果设置)
    // 如果不确定，可以先设为0，或者根据手册中的噪声特性估算。
    // 对角线元素表示对应轴的方差。
    msg.angular_velocity_covariance[0] = 0.0001; // x-axis variance
    msg.angular_velocity_covariance[4] = 0.0001; // y-axis variance
    msg.angular_velocity_covariance[8] = 0.0001; // z-axis variance

    msg.linear_acceleration_covariance[0] = 0.001; // x-axis variance
    msg.linear_acceleration_covariance[4] = 0.001; // y-axis variance
    msg.linear_acceleration_covariance[8] = 0.001; // z-axis variance

    return true;
}

bool ImuSerialProcessor::process_byte_buffer() {
    // 在缓冲区中查找帧头并尝试解析一个完整的数据包
    for (size_t i = 0; (i + PACKET_LENGTH) <= byte_buffer_.size(); ++i) {
        if (byte_buffer_[i] == FRAME_HEADER) { // 查找帧头
            const uint8_t* candidate_packet_start = &byte_buffer_[i];

            // 提取数据包中的CRC值 (假设大端字节序，位于包的最后4字节)
            uint32_t received_crc = (static_cast<uint32_t>(candidate_packet_start[PACKET_LENGTH - 4]) << 24) |
                                    (static_cast<uint32_t>(candidate_packet_start[PACKET_LENGTH - 3]) << 16) |
                                    (static_cast<uint32_t>(candidate_packet_start[PACKET_LENGTH - 2]) << 8)  |
                                    (static_cast<uint32_t>(candidate_packet_start[PACKET_LENGTH - 1]));

            // 计算数据部分的CRC (从帧头开始，长度为 PACKET_LENGTH - 4)
            uint32_t calculated_crc = calculate_crc32(candidate_packet_start, PACKET_LENGTH - 4);

            /*********************************** 测试原始数据CRC校验格式 ***********************************/
            // // 4. 使用 ROS_INFO 显示原始数据和CRC信息
            // std::stringstream ss_raw_data;
            // ss_raw_data << "Raw Packet (Hex): [ ";
            // for(size_t k=0; k < PACKET_LENGTH; ++k) {
            //     ss_raw_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(candidate_packet_start[k]) << " ";
            // }
            // ss_raw_data << "]";
            
            // ROS_INFO_STREAM(ss_raw_data.str());
            // ROS_INFO("Received CRC: 0x%08X, Calculated CRC: 0x%08X", received_crc, calculated_crc);
            /*********************************** 测试原始数据CRC校验格式 ***********************************/

            if (received_crc == calculated_crc) {
                // CRC校验通过
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = this->frame_id_;

                if (parse_stim300_packet(candidate_packet_start, imu_msg)) {
                    imu_pub_.publish(imu_msg);
                } else {
                    ROS_WARN("STIM300 packet content parsing failed after CRC check.");
                }
                // 从缓冲区移除已处理的数据包 (从找到的帧头开始，长度为PACKET_LENGTH)
                byte_buffer_.erase(byte_buffer_.begin() + i, byte_buffer_.begin() + i + PACKET_LENGTH);
                return true; // 成功处理一个包，让外层循环继续检查缓冲区
            } else {
                // CRC校验失败
                ROS_WARN_STREAM("CRC mismatch! Received: 0x" << std::hex << std::setw(8) << std::setfill('0') << received_crc
                                << ", Calculated: 0x" << std::hex << std::setw(8) << std::setfill('0') << calculated_crc
                                << ". Discarding byte at index " << std::dec << i << " (0x" << std::hex << (unsigned int)byte_buffer_[i] << std::dec << ")");
                // 丢弃这个错误的帧头，然后从下一个字节开始继续搜索
                // 注意：这里只移除了导致CRC错误的帧头所在的那个字节，
                // 而不是整个包，因为我们不知道这个包的真实结束位置。
                // 更稳健的做法可能是只移除byte_buffer_[i]，或者移除到i+1。
                // 为避免死循环（如果CRC实现错误或数据流持续损坏），我们至少移除一个字节。
                byte_buffer_.erase(byte_buffer_.begin() + i);
                return false; // 缓冲区已修改，但未成功处理包，让外层循环从头开始检查修改后的缓冲区
            }
        }
    }

    // 如果缓冲区过大但没有找到有效数据包，可以考虑清理一部分旧数据以防无限增长
    if (byte_buffer_.size() > PACKET_LENGTH * 20) { // 示例阈值：20个包的长度
        ROS_WARN("Byte buffer too large without valid packet, clearing half to prevent overflow.");
        byte_buffer_.erase(byte_buffer_.begin(), byte_buffer_.begin() + byte_buffer_.size() / 2);
    }

    return false; // 没有在当前缓冲区中找到或处理完整的数据包
}

void ImuSerialProcessor::run() {
    if (!initialized_ || !loop_rate_) {
        ROS_ERROR("ImuSerialProcessor not initialized or loop_rate not set. Cannot run.");
        return;
    }
    ROS_INFO("IMU node started. Reading from %s. Publishing to %s",
             port_name_.c_str(), imu_pub_.getTopic().c_str());

    uint8_t read_temp_buf[256]; // 用于read()系统调用的临时缓冲区

    while (ros::ok()) {
        int bytes_read = read(serial_fd_, read_temp_buf, sizeof(read_temp_buf));

        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 非阻塞读取或超时，没有数据是正常的
            } else {
                ROS_ERROR_THROTTLE(1.0, "Error reading from serial port %s: %s", port_name_.c_str(), strerror(errno));
            }
        } else if (bytes_read > 0) {
            // 将读取到的数据追加到内部缓冲区
            byte_buffer_.insert(byte_buffer_.end(), read_temp_buf, read_temp_buf + bytes_read);
        }

        // 循环处理缓冲区中的数据，直到没有完整的数据包可以处理
        while(process_byte_buffer()) {
            // 如果 process_byte_buffer 返回 true，说明成功处理了一个包，继续尝试处理下一个
        }

        ros::spinOnce();
        loop_rate_->sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stim300_node"); // 节点名
    ImuSerialProcessor imu_driver;

    if (imu_driver.init()) {
        imu_driver.run(); // **取消注释，运行主循环**
    } else {
        ROS_FATAL("Failed to initialize STIM300 driver. Shutting down.");
        return 1;
    }

    ROS_INFO("STIM300 node shutting down.");
    return 0;
}
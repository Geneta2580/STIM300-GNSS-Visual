#include <gnss_driver/gnss_driver.h>
#include <iomanip> // 用于调试时打印16进制 (std::hex, std::setw, std::setfill)
#include <cstring> // For strerror

// Constructor
GnssSerialProcessor::GnssSerialProcessor() :
    private_nh_("~"),
    serial_fd_(-1),
    initialized_(false),
    loop_rate_(nullptr),
    has_fresh_covariance_(false),
    last_covariance_itow_(0)      
{
    ROS_INFO("GnssSerialProcessor constructor.");
}

// Destructor
GnssSerialProcessor::~GnssSerialProcessor() {
    if (serial_fd_ != -1) {
        close(serial_fd_);
        ROS_INFO("Serial port closed.");
    }
    if (loop_rate_) {
        delete loop_rate_;
    }
    ROS_INFO("GnssSerialProcessor destructor.");
}

bool GnssSerialProcessor::init() {
    // 修改参数名 "serial_port_fd" 为 "serial_port"
    private_nh_.param<std::string>("serial_port", this->port_name_, "/dev/ttyUSB0");
    private_nh_.param<int>("baud_rate", this->baud_rate_, 38400);
    private_nh_.param<std::string>("frame_id", this->frame_id_, "gnss_link");
    private_nh_.param<double>("publish_rate", this->publish_rate_hz_, 1.0);

    ROS_INFO("Parameters: Port: %s, Baud: %d, Frame ID: %s, Rate: %.1f Hz",
            port_name_.c_str(), baud_rate_, frame_id_.c_str(), publish_rate_hz_);
    
    serial_fd_ = configure_serial_port(port_name_, baud_rate_);
    if (serial_fd_ < 0) {
        ROS_ERROR("Failed to configure or open serial port.");
        return false;
    }
    ROS_INFO("Serial port opened successfully with fd: %d", serial_fd_);

    gnss_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("gnss/fix", 50);
    ROS_INFO("GNSS NavSatFix data publisher initialized on topic 'gnss/fix'.");

    if (publish_rate_hz_ > 0) {
        loop_rate_ = new ros::Rate(publish_rate_hz_);
    } else {
        ROS_WARN("Publish rate is <= 0, defaulting to 1 Hz for loop rate.");
        loop_rate_ = new ros::Rate(1.0);
    }

    initialized_ = true;
    ROS_INFO("GnssSerialProcessor initialized successfully.");
    return true;
}

int GnssSerialProcessor::configure_serial_port(const std::string& port_name, int baud_rate) {
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

bool GnssSerialProcessor::verify_ubx_checksum(const uint8_t* packet_data_for_checksum, size_t length_of_data_for_checksum) {
    // packet_data_for_checksum 应该指向 UBX 消息的 Class 字段。
    // length_of_data_for_checksum 是 Class, ID, Length, 和 Payload 的总长度。
    // UBX 校验和 (CK_A, CK_B) 紧跟在这部分数据之后。

    if (packet_data_for_checksum == nullptr || length_of_data_for_checksum == 0) {
        ROS_WARN("UBX checksum verification called with null or zero length data.");
        return false;
    }

    const uint8_t received_ck_a = packet_data_for_checksum[length_of_data_for_checksum];
    const uint8_t received_ck_b = packet_data_for_checksum[length_of_data_for_checksum + 1];

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 0; i < length_of_data_for_checksum; ++i) {
        ck_a = ck_a + packet_data_for_checksum[i];
        ck_b = ck_b + ck_a;
    }

    if (ck_a == received_ck_a && ck_b == received_ck_b) {
        return true;
    } else {
        ROS_DEBUG_THROTTLE(1.0, "UBX Checksum mismatch: Calculated CK_A=0x%02X, CK_B=0x%02X. Received CK_A=0x%02X, CK_B=0x%02X",
               ck_a, ck_b, received_ck_a, received_ck_b);
        return false;
    }
}

bool GnssSerialProcessor::parse_ubx_nav_pvt_packet(const uint8_t* payload_data, uint16_t payload_length, sensor_msgs::NavSatFix& msg) {
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = this->frame_id_;

    // --- UBX NAV-PVT (0x01 0x07) 消息解析示例 ---
    // 请参考您的接收器协议版本的UBX协议规范以获取确切的偏移量和类型
    if (payload_length < 92) { // NAV-PVT 的示例长度
        ROS_WARN("UBX NAV-PVT payload too short. Length: %d", payload_length);
        return false;
    }

    // 提取整个数据包字段
    uint32_t iTOW; // ms, GPS week time
    memcpy(&iTOW, &payload_data[0], sizeof(uint32_t));
    // ... (解析其他 NAV-PVT 字段，如 lat, lon, alt, fixType, hAcc, vAcc) ...

    int32_t lon_raw, lat_raw, height_raw, hMSL_raw;
    uint32_t hAcc_raw, vAcc_raw;
    uint8_t fix_type_raw;

    memcpy(&lon_raw, &payload_data[24], sizeof(int32_t));
    memcpy(&lat_raw, &payload_data[28], sizeof(int32_t));
    memcpy(&height_raw, &payload_data[32], sizeof(int32_t)); // 椭球高度height
    memcpy(&hMSL_raw, &payload_data[36], sizeof(int32_t));  // 海平面高度
    memcpy(&hAcc_raw, &payload_data[40], sizeof(uint32_t)); // 水平精度
    memcpy(&vAcc_raw, &payload_data[44], sizeof(uint32_t)); // 垂直精度
    fix_type_raw = payload_data[20]; // GNSS数据类型，2为2D定位，3为3D定位

    msg.latitude = lat_raw * 1e-7; // 转换为deg单位
    msg.longitude = lon_raw * 1e-7;
    msg.altitude = hMSL_raw * 1e-3; // 将毫米转换为米

    // ... (根据 fix_type_raw 设置 msg.status.status) ...
    msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // 根据需要调整

    // 协方差处理
    if (has_fresh_covariance_ && last_covariance_itow_ == iTOW) { // 匹配 时间戳
        msg.position_covariance[0] = current_covariance_matrix_[0]; // C_nn (cov_lat_lat)
        msg.position_covariance[1] = current_covariance_matrix_[1]; // C_ne (cov_lat_lon)
        msg.position_covariance[2] = current_covariance_matrix_[2]; // C_nd (cov_lat_alt)
        msg.position_covariance[3] = current_covariance_matrix_[3]; // C_en (cov_lon_lat)
        msg.position_covariance[4] = current_covariance_matrix_[4]; // C_ee (cov_lon_lon)
        msg.position_covariance[5] = current_covariance_matrix_[5]; // C_ed (cov_lon_alt)
        msg.position_covariance[6] = current_covariance_matrix_[6]; // C_dn (cov_alt_lat)
        msg.position_covariance[7] = current_covariance_matrix_[7]; // C_dn (cov_alt_lon)
        msg.position_covariance[8] = current_covariance_matrix_[8]; // C_dd (cov_alt_alt)
        msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
        has_fresh_covariance_ = false; // 释放
    } else {
        // 回退到使用 NAV-PVT 中的 hAcc 和 vAcc 计算对角协方差
        double h_acc_m = hAcc_raw * 1e-3; // 毫米转米
        double v_acc_m = vAcc_raw * 1e-3; // 毫米转米
        msg.position_covariance[0] = h_acc_m * h_acc_m; // 方差 N
        msg.position_covariance[4] = h_acc_m * h_acc_m; // 方差 E
        msg.position_covariance[8] = v_acc_m * v_acc_m; // 方差 D (U)
        msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        for(int k=0; k<9; ++k) if(k!=0 && k!=4 && k!=8) msg.position_covariance[k] = 0.0; // 将非对角线元素置零
    }
    return true;
}

// 解析 UBX-NAV-COV 的新函数
bool GnssSerialProcessor::parse_ubx_nav_cov_packet(const uint8_t* payload_data, uint16_t payload_length) {
    // UBX-NAV-COV (0x01 0x36)
    // 请参考UBX协议规范以获取载荷结构。
    // 它包含 iTOW, version, posCovValid, velCovValid, 以及协方差矩阵元素。
    // 示例针对具有6x6位置和速度协方差（例如60字节载荷）或仅3x3位置协方差的版本。请检查您的接收器规格。
    // 假设载荷至少包含 iTOW 和 3x3 位置协方差部分。

    if (payload_length < (4 + 1 + 1 + 1 + 1 + 6*4) ) { // iTOW, version, flags, 6个float协方差项的最小长度
       ROS_WARN("NAV-COV payload too short: %d bytes", payload_length);
       return false;
    }

    uint32_t iTOW;
    memcpy(&iTOW, &payload_data[0], sizeof(uint32_t));
    uint8_t version = payload_data[4];
    uint8_t posCovValid = payload_data[5];
    uint8_t velCovValid = payload_data[6]; // cov有效性

    if (!(posCovValid & 0x01)) { // 检查位置协方差是否有效
        ROS_DEBUG("NAV-COV received, but position covariance is not valid.");
        has_fresh_covariance_ = false; // 如果无效，则标记为不是最新的
        return false;
    }

    // 偏移量取决于 NAV-COV 消息的版本。
    // 假设版本0或1，其中posCovar矩阵元素在一些头部字节之后开始。
    // 例如，如果它们是float类型并且从偏移量8开始（示例）：32位单精度数（float）
    float covNN, covNE, covND, covEE, covED, covDD;
    size_t offset = 16; // 占位符 - 请检查您的文档
    memcpy(&covNN, &payload_data[offset + 0*sizeof(float)], sizeof(float));
    memcpy(&covNE, &payload_data[offset + 1*sizeof(float)], sizeof(float));
    memcpy(&covND, &payload_data[offset + 2*sizeof(float)], sizeof(float));
    memcpy(&covEE, &payload_data[offset + 3*sizeof(float)], sizeof(float));
    memcpy(&covED, &payload_data[offset + 4*sizeof(float)], sizeof(float));
    memcpy(&covDD, &payload_data[offset + 5*sizeof(float)], sizeof(float));

    // 按 NavSatFix 顺序存储 (行主序):
    // [ C_NN, C_NE, C_ND ]
    // [ C_EN, C_EE, C_ED ]
    // [ C_DN, C_DE, C_DD ]
    current_covariance_matrix_[0] = static_cast<double>(covNN);
    current_covariance_matrix_[1] = static_cast<double>(covNE);
    current_covariance_matrix_[2] = static_cast<double>(covND);
    current_covariance_matrix_[3] = static_cast<double>(covNE); // C_EN = C_NE
    current_covariance_matrix_[4] = static_cast<double>(covEE);
    current_covariance_matrix_[5] = static_cast<double>(covED);
    current_covariance_matrix_[6] = static_cast<double>(covND); // C_DN = C_ND
    current_covariance_matrix_[7] = static_cast<double>(covED); // C_DE = C_ED
    current_covariance_matrix_[8] = static_cast<double>(covDD);

    has_fresh_covariance_ = true;
    last_covariance_itow_ = iTOW;
    ROS_DEBUG("Parsed NAV-COV data. iTOW: %u", iTOW);
    return true;
}

bool GnssSerialProcessor::process_byte_buffer() {
    // --- 完整的UBX帧查找逻辑的占位符 ---
    const size_t MIN_UBX_PACKET_LEN = 8; // UBX头(6) + 校验和(2)
    for (size_t i = 0; (i + MIN_UBX_PACKET_LEN) <= byte_buffer_.size(); ++i) {
        if (byte_buffer_[i] == UBX_SYNC_CHAR1 && byte_buffer_[i+1] == UBX_SYNC_CHAR2) {
            uint8_t msg_class = byte_buffer_[i+2];
            uint8_t msg_id = byte_buffer_[i+3];
            uint16_t payload_length = 0;
            memcpy(&payload_length, &byte_buffer_[i+4], sizeof(uint16_t)); // UBX是小端字节序，提取数据包长度
            size_t total_packet_length = 6 + payload_length + 2; // 头 + 载荷 + 校验和

            if ((i + total_packet_length) > byte_buffer_.size()) {
                ROS_DEBUG("Incomplete UBX packet, waiting for more data. Needed: %zu, Have: %zu", total_packet_length, byte_buffer_.size() - i);
                return false; // 包不完整，等待更多数据
            }

            // 指针指向各个起始
            const uint8_t* packet_start = &byte_buffer_[i];
            const uint8_t* payload_start = &byte_buffer_[i+6];

            // 校验和从Class字段开始，到载荷结束
            if (!verify_ubx_checksum(packet_start + 2, 4 + payload_length)) { // 传入 Class, ID, Length, Payload
                ROS_WARN_THROTTLE(1.0, "UBX checksum failed. Discarding packet.");
                byte_buffer_.erase(byte_buffer_.begin() + i, byte_buffer_.begin() + i + 1); // 丢弃错误的同步字符，然后重试
                continue; 
            }

            if (msg_class == UBX_CLASS_NAV) {
                if (msg_id == UBX_ID_NAV_PVT) {
                    sensor_msgs::NavSatFix nav_fix_msg;
                    if (parse_ubx_nav_pvt_packet(payload_start, payload_length, nav_fix_msg)) {
                        gnss_pub_.publish(nav_fix_msg);
                    } else {
                        ROS_WARN_THROTTLE(1.0, "UBX NAV-PVT content parsing failed.");
                    }
                } else if (msg_id == UBX_ID_NAV_COV) {
                    if (parse_ubx_nav_cov_packet(payload_start, payload_length)) {
                        ROS_DEBUG_THROTTLE(5.0,"Successfully parsed UBX-NAV-COV packet.");
                    } else {
                        ROS_WARN_THROTTLE(1.0, "UBX NAV-COV content parsing failed.");
                    }
                } else {
                    ROS_DEBUG_THROTTLE(5.0, "Unhandled UBX NAV message: ID 0x%02X", msg_id);
                }
            } else {
                ROS_DEBUG_THROTTLE(5.0, "Unhandled UBX message: Class 0x%02X, ID 0x%02X", msg_class, msg_id);
            }
            
            byte_buffer_.erase(byte_buffer_.begin() + i, byte_buffer_.begin() + i + total_packet_length);
            return true; // 成功处理一个包
        }
    }
    // --- 结束占位符 ---

    if (byte_buffer_.size() > (6 + 200 + 2) * 10) { // 示例：10个最大尺寸的UBX数据包 (假设最大载荷200字节)
        ROS_WARN("Byte buffer very large, clearing half.");
        byte_buffer_.erase(byte_buffer_.begin(), byte_buffer_.begin() + byte_buffer_.size() / 2);
    }
    return false; 
}

void GnssSerialProcessor::run() {
    if (!initialized_ || !loop_rate_) {
        ROS_ERROR("GnssSerialProcessor not initialized or loop_rate not set. Cannot run.");
        return;
    }
    ROS_INFO("GNSS node started. Reading from %s. Publishing to %s",
             port_name_.c_str(), gnss_pub_.getTopic().c_str());

    uint8_t read_temp_buf[512]; // 用于read()系统调用的临时缓冲区

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
    ros::init(argc, argv, "gnss_node"); // 节点名
    GnssSerialProcessor gnss_driver;

    if (gnss_driver.init()) {
        gnss_driver.run(); // **取消注释，运行主循环**
    } else {
        ROS_FATAL("Failed to initialize GNSS driver. Shutting down.");
        return 1;
    }

    ROS_INFO("GNSS node shutting down.");
    return 0;
}
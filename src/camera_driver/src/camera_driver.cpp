#include "camera_driver/camera_driver.h"
#include <std_msgs/Header.h>

// If you need std::filesystem and your environment supports C++17
// #include <filesystem>
// namespace fs = std::filesystem;

StereoCam::StereoCam(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
    nh_(nh),
    private_nh_(private_nh),
    it_(nh) // Initialize ImageTransport with the public NodeHandle
{
    // 参数设置
    private_nh_.param("device_id", device_id_, 0);
    private_nh_.param("frame_width", frame_width_, 1280);  // Default for combined stereo
    private_nh_.param("frame_height", frame_height_, 480);
    private_nh_.param("fps", fps_, 30.0);
    private_nh_.param<std::string>("left_camera_name", left_camera_name_, "left_camera");
    private_nh_.param<std::string>("right_camera_name", right_camera_name_, "right_camera");
    private_nh_.param<std::string>("left_camera_info_url", left_camera_info_url_, "package://camera_driver/config/left_camera.yaml"); // 指定相机配置文件yaml
    private_nh_.param<std::string>("right_camera_info_url", right_camera_info_url_, "package://camera_driver/config/right_camera.yaml");


    ROS_INFO("Start collect images. Device ID: %d, Resolution: %dx%d @ %.1f FPS",
             device_id_, frame_width_, frame_height_, fps_);

    // Setup Camera Info Managers
    ros::NodeHandle left_nh(nh_, left_camera_name_);
    ros::NodeHandle right_nh(nh_, right_camera_name_);

    // 使用不同的nh节点注册句柄
    left_cinfo_manager_.reset(new camera_info_manager::CameraInfoManager(left_nh, left_camera_name_, left_camera_info_url_));
    right_cinfo_manager_.reset(new camera_info_manager::CameraInfoManager(right_nh, right_camera_name_, right_camera_info_url_));

    // Create publishers创建发布者
    // The topic names "Geneta_Image_left" and "Geneta_Image_right" will be relative to the node's namespace
    // or absolute if they start with "/"
    left_pub_ = it_.advertiseCamera(left_camera_name_ + "/image_raw", 1);
    right_pub_ = it_.advertiseCamera(right_camera_name_ + "/image_raw", 1);

    // 初始化相机
    initCamera();
}

StereoCam::~StereoCam() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}


void StereoCam::initCamera() {
    cap_.open(device_id_, cv::CAP_V4L2); // Or just cap_.open(device_id_);

    if(!cap_.isOpened()) {
        ROS_ERROR("Failed to open camera device %d", device_id_);
        ros::shutdown();
        return;
    }

    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    ROS_INFO("Camera opened. Actual FPS: %.1f", actual_fps);
    if (fps_ > 0) { // 设置定时器控制帧率
         timer_ = nh_.createTimer(ros::Duration(1.0/fps_), &StereoCam::captureAndPublish, this);
    } else {
         ROS_WARN("FPS set to 0 or negative, timer not started. Will rely on camera's own rate if supported by grab/retrieve.");
    }
}

void StereoCam::captureAndPublish(const ros::TimerEvent& event) {
    cv::Mat frame;
    if (!cap_.read(frame)) { // Use read for better error checking
        ROS_WARN_THROTTLE(1.0, "Failed to capture frame");
        return;
    }
    if(frame.empty()) {
        ROS_WARN_THROTTLE(1.0, "Captured empty frame");
        return;
    }

    // ***** 添加这行来调试 *****
    cv::imshow("Raw Full Frame from Camera", frame);
    cv::waitKey(1);

    // Assuming the camera provides a side-by-side stereo image
    if (frame.cols < 2 || frame.cols % 2 != 0) {
        ROS_ERROR_THROTTLE(1.0, "Frame width is not suitable for stereo splitting (width: %d)", frame.cols);
        return;
    }
    int single_frame_width = frame.cols / 2; // 分隔图像
    cv::Mat left_frame = frame(cv::Rect(0, 0, single_frame_width, frame.rows)); 
    cv::Mat right_frame = frame(cv::Rect(single_frame_width, 0, single_frame_width, frame.rows));

    ros::Time stamp = ros::Time::now(); // Or use event.current_real if more appropriate and available

    if (left_cinfo_manager_ && right_cinfo_manager_) {
         publishImage(left_frame, left_camera_name_, stamp, left_pub_, *left_cinfo_manager_);
         publishImage(right_frame, right_camera_name_, stamp, right_pub_, *right_cinfo_manager_);
    } else {
        ROS_ERROR_ONCE("Camera info managers not initialized!");
    }
}

void StereoCam::publishImage(cv::Mat& img, const std::string& frame_id,
    const ros::Time& stamp, image_transport::CameraPublisher& pub,
    camera_info_manager::CameraInfoManager& cinfo_manager) {

    if (img.empty()) {
        ROS_WARN("Attempting to publish an empty image for frame_id: %s", frame_id.c_str());
        return;
    }

    // Get CameraInfo first to use its frame_id
    sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(cinfo_manager.getCameraInfo()));
    info_msg->header.stamp = stamp;

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = info_msg->header.frame_id; // Use frame_id from CameraInfo for consistency

    // Convert OpenCV image to ROS image message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

    pub.publish(msg, info_msg);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "stereo_cam_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // Private NodeHandle for parameters

    try {
        StereoCam stereo_cam_node(nh, private_nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in stereo_cam_node: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown exception in stereo_cam_node");
        return 1;
    }

    return 0;
}
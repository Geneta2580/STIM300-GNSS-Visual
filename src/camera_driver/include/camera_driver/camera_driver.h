#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h> // For camera calibration

// Forward declaration if needed, or include specific headers
// #include <filesystem> // std::filesystem is C++17, ensure your ROS 1 environment supports it or use alternatives

class StereoCam {
public:
    StereoCam(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~StereoCam();

private:
    void initCamera();
    // ROS 1 timer callback takes const ros::TimerEvent&
    void captureAndPublish(const ros::TimerEvent& event);
    void publishImage(cv::Mat& img, const std::string& frame_id,
                      const ros::Time& stamp, image_transport::CameraPublisher& pub,
                      camera_info_manager::CameraInfoManager& cinfo_manager);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    cv::VideoCapture cap_;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher left_pub_, right_pub_;
    ros::Timer timer_;

    // Camera Info Managers
    boost::shared_ptr<camera_info_manager::CameraInfoManager> left_cinfo_manager_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> right_cinfo_manager_;

    // Parameters
    int device_id_;
    int frame_width_;
    int frame_height_;
    double fps_;
    std::string left_camera_name_;
    std::string right_camera_name_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
};

#endif // CAMERA_DRIVER_H
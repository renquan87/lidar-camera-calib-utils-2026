#include "hnurm_camera/camera_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace hnurm
{
void CameraNode::run()
{
    RCLCPP_INFO(logger_, "CameraNode is running");
    std::string camera_img_topic = this->declare_parameter("camera_img_topic", "image");
    camera_info_url_ = this->declare_parameter("camera_info_url", "");

    cam_ = std::make_shared<HKcam>(shared_from_this());

    // using best effort
    pub_img_  = image_transport::create_publisher(this, camera_img_topic, rmw_qos_profile_sensor_data);

    // camera_info 发布者
    pub_cam_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS());

    // 初始化 camera_info_manager，同时注册 set_camera_info 服务
    cam_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "camera");
    if (!camera_info_url_.empty()) {
        if (cam_info_->loadCameraInfo(camera_info_url_)) {
            RCLCPP_INFO(logger_, "Loaded camera calibration from: %s", camera_info_url_.c_str());
        } else {
            RCLCPP_WARN(logger_, "Failed to load camera calibration from: %s", camera_info_url_.c_str());
        }
    }
    cam_info_msg_ = cam_info_->getCameraInfo();

    capture_thread_ = std::thread([this]() {
        while(rclcpp::ok())
        {
            timer_callback();
        }
    });
}

void CameraNode::timer_callback()
{
    std::vector<uint8_t> img;
    static auto          prev    = this->now();
    if(cam_->GetFrame(img))
    {
        img_msg_.data            = img;
        img_msg_.header.stamp    = now();
        img_msg_.header.frame_id = "camera";
        img_msg_.encoding        = sensor_msgs::image_encodings::BGR8;
        img_msg_.height          = cam_->_nHeight;
        img_msg_.width           = cam_->_nWidth;
        img_msg_.is_bigendian    = 0;
        img_msg_.step            = cam_->_nWidth * 3;

        pub_img_.publish(img_msg_);

        // 同步发布 camera_info
        cam_info_msg_.header.stamp    = img_msg_.header.stamp;
        cam_info_msg_.header.frame_id = img_msg_.header.frame_id;
        pub_cam_info_->publish(cam_info_msg_);

        auto now = this->now();
        RCLCPP_INFO(this->get_logger(), "Capture FPS: %f", 1.0 / (now - prev).seconds());
        prev = now;
    }
}
}
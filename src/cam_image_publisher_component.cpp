// Copyright 2020 amsl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "amsl_usb_cam_ros2/cam_image_publisher_component.hpp"

namespace amsl_usb_cam_ros2
{
CamImagePublisherComponent::CamImagePublisherComponent(const rclcpp::NodeOptions & options)
: Node("cam_image_publisher", options),
  clock_(RCL_ROS_TIME)
{
  declare_parameter("camera_device_name", "/dev/video0");
  get_parameter("camera_device_name", camera_device_name_);
  declare_parameter("image_frame_id", "camera_link");
  get_parameter("image_frame_id", image_frame_id_);
  declare_parameter("width", 1280);
  get_parameter("width", width_);
  declare_parameter("height", 720);
  get_parameter("height", height_);
  declare_parameter("fps", 30);
  get_parameter("fps", fps_);
  declare_parameter("enable_gui", true);
  get_parameter("enable_gui", enable_gui_);

  RCLCPP_INFO_STREAM(get_logger(), "camera_device_name: " << camera_device_name_);
  RCLCPP_INFO_STREAM(get_logger(), "image_frame_id: " << image_frame_id_);
  RCLCPP_INFO_STREAM(get_logger(), "width_: " << width_);
  RCLCPP_INFO_STREAM(get_logger(), "height: " << height_);
  RCLCPP_INFO_STREAM(get_logger(), "fps: " << fps_);
  RCLCPP_INFO_STREAM(get_logger(), "enable_gui: " << enable_gui_);

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image", 1);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int>(1.0 / fps_ * 1e3)),
    std::bind(&CamImagePublisherComponent::timer_callback, this));

  cap.open(camera_device_name_);

  if (!cap.isOpened()) {
    RCLCPP_FATAL(get_logger(), "Failed to open device: " + camera_device_name_);
    exit(-1);
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap.set(cv::CAP_PROP_FPS, fps_);
}

void CamImagePublisherComponent::timer_callback(void)
{
  cv::Mat image;
  if (!cap.read(image)) {
    RCLCPP_ERROR(get_logger(), "Failed to read image from " + camera_device_name_);
    return;
  }
  cv::resize(
    image, image,
    cv::Size(), width_ / static_cast<double>(image.cols),
    height_ / static_cast<double>(image.rows));

  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = clock_.now();
  image_msg.is_bigendian = false;
  convert_frame_to_message(image, image_msg);
  image_pub_->publish(image_msg);

  if (enable_gui_) {
    cv::imshow("window", image);
    cv::waitKey(1);
  }
}

// this function is borrowed from https://github.com/ros2/demos/blob/b17664c8ae849ef3b8eaa51453bcca2c95f08d7f/image_tools/src/cam2image.cpp#L253-L267
std::string CamImagePublisherComponent::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

// this function is borrowed from https://github.com/ros2/demos/blob/b17664c8ae849ef3b8eaa51453bcca2c95f08d7f/image_tools/src/cam2image.cpp#L276-L288
// and modified
void CamImagePublisherComponent::convert_frame_to_message(
  const cv::Mat & frame, sensor_msgs::msg::Image & msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = image_frame_id_;
}

}  //  namespace amsl_usb_cam_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(amsl_usb_cam_ros2::CamImagePublisherComponent)

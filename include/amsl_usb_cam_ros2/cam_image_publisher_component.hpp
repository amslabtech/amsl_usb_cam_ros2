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

#ifndef AMSL_USB_CAM_ROS2__CAM_IMAGE_PUBLISHER_COMPONENT_HPP_
#define AMSL_USB_CAM_ROS2__CAM_IMAGE_PUBLISHER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_EXPORT __attribute__ ((dllexport))
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_IMPORT __attribute__ ((dllimport))
  #else
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_EXPORT __declspec(dllexport)
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_IMPORT __declspec(dllimport)
  #endif
  #ifdef AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_BUILDING_DLL
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC \
  AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_EXPORT
  #else
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC \
  AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_IMPORT
  #endif
  #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC_TYPE \
  AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC
  #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_LOCAL
#else
  #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_EXPORT __attribute__ ((visibility("default")))
  #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_IMPORT
  #if __GNUC__ >= 4
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC __attribute__ ((visibility("default")))
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC
    #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_LOCAL
  #endif
  #define AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


namespace amsl_usb_cam_ros2
{
class CamImagePublisherComponent : public rclcpp::Node
{
public:
  AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_PUBLIC
  explicit CamImagePublisherComponent(const rclcpp::NodeOptions & options);

private:
  void timer_callback(void);
  std::string mat_type2encoding(int mat_type);
  void convert_frame_to_message(const cv::Mat & frame, sensor_msgs::msg::Image & msg);

  std::string camera_device_name_;
  std::string image_frame_id_;
  int width_;
  int height_;
  int fps_;
  bool enable_gui_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap;
  rclcpp::Clock clock_;
};
}  //  namespace amsl_usb_cam_ros2

#endif  // AMSL_USB_CAM_ROS2__CAM_IMAGE_PUBLISHER_COMPONENT_HPP_

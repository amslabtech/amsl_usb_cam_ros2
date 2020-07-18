add_library(cam_image_publisher_component SHARED
  cam_image_publisher_component.cpp
)

target_compile_definitions(cam_image_publisher_component PRIVATE "AMSL_USB_CAM_ROS2_CAM_IMAGE_PUBLISHER_BUILDING_DLL")

target_link_libraries(cam_image_publisher_component
)

ament_target_dependencies(cam_image_publisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
)

add_executable(cam_image_publisher
  cam_image_publisher_node.cpp
)

target_link_libraries(cam_image_publisher
  cam_image_publisher_component
)

ament_target_dependencies(cam_image_publisher
  rclcpp
  rclcpp_components
  sensor_msgs
)

install(TARGETS
  cam_image_publisher
  DESTINATION lib/amsl_usb_cam_ros2
)

install(TARGETS
  cam_image_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(cam_image_publisher_component)

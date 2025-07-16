#pragma once

#include <rclcpp/rclcpp.hpp>     
#include <sensor_msgs/msg/image.hpp>  
#include <memory>                     
#include <string> 
#include <filesystem> 
#include <chrono>
#include <iomanip>
#include <sstream>  
#include <pylon/PylonIncludes.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "basler_camera_interfaces/srv/change_filename.hpp"
#include "basler_camera_interfaces/srv/take_pictures.hpp"
#include "basler_camera_interfaces/srv/take_a_picture.hpp"

using namespace Pylon;

namespace basler_camera
{

class BaslerCameraNode : public rclcpp::Node
{
public:
  explicit BaslerCameraNode(const rclcpp::NodeOptions & options);
  ~BaslerCameraNode();

private:
  void grabLoop();
  void changeFilenameCB(const std::shared_ptr<basler_camera_interfaces::srv::ChangeFilename::Request> request, 
    std::shared_ptr<basler_camera_interfaces::srv::ChangeFilename::Response> response);
  void takeAPictureCB(const std::shared_ptr<basler_camera_interfaces::srv::TakeAPicture::Request> request, 
    std::shared_ptr<basler_camera_interfaces::srv::TakeAPicture::Response> response);

  // member vars
  std::string camera_id_;
  std::string camera_ip_;
  std::unique_ptr<CInstantCamera>  camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::thread grab_thread_;
  const std::string file_directory = "images";
  std::string file_name;
  std::filesystem::path full_path;
  rclcpp::Service<basler_camera_interfaces::srv::ChangeFilename>::SharedPtr change_filename_server_;
  rclcpp::Service<basler_camera_interfaces::srv::TakeAPicture>::SharedPtr take_a_picture_server_;
};

}  // namespace basler_camera
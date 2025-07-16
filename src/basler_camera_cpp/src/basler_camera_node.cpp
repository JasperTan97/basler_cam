#include "basler_camera_cpp/basler_camera_node.hpp"

using namespace Pylon;
using namespace std::chrono_literals;


namespace basler_camera {

BaslerCameraNode::BaslerCameraNode(
    const rclcpp::NodeOptions & options
  )
  : Node("basler_camera_node", options)
{
  this->declare_parameter<int>("camera_id", 0);
  int camera_id_int;
  this->get_parameter("camera_id", camera_id_int);
  const std::string subnet = "192.168.1.";
  if (camera_id_int < 1 || camera_id_int > 254) {
    RCLCPP_ERROR(get_logger(), "camera_id must be 1 to 254, got %d", camera_id_int);
    throw std::out_of_range("camera_id out of range");
  }
  camera_id_ = std::to_string(camera_id_int);
  camera_ip_   = "192.168.1." + camera_id_;
  RCLCPP_INFO(get_logger(), "Using camera at IP: %s", camera_ip_.c_str());

  PylonInitialize();

  Pylon::CTlFactory & factory = Pylon::CTlFactory::GetInstance();
  Pylon::DeviceInfoList_t devices;
  factory.EnumerateDevices(devices);

  for (auto &devInfo : devices) {
    if (camera_ip_ == devInfo.GetIpAddress().c_str()) {
      camera_ = std::make_unique<Pylon::CInstantCamera>(
        factory.CreateDevice(devInfo)
      );
      break;
    }
  }

  if (!camera_) {
    RCLCPP_ERROR(get_logger(),
                 "Camera with IP %s not connected", camera_ip_.c_str());
    throw std::runtime_error("Camera not found");
  }

  std::filesystem::create_directories(file_directory);

  auto now = std::chrono::system_clock::now();
  auto epoch_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                           now.time_since_epoch()
                         ).count();

  std::ostringstream oss;
  oss << "camera_" << camera_id_ << "_" << epoch_seconds;
  file_name = oss.str();

  full_path = std::filesystem::path(file_directory) / file_name;

  camera_->Open();

  // publisher
  std::stringstream topic_name;
  topic_name << "camera_" << camera_id_ << "/image_raw";
  pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    topic_name.str(), rclcpp::QoS(10)
  );

  // services
  std::stringstream change_filename_service_name;
  change_filename_service_name << "camera_" << camera_id_ << "/change_filename";
  change_filename_server_ = this->create_service<basler_camera_interfaces::srv::ChangeFilename>(
    change_filename_service_name.str(),
    std::bind(
      &BaslerCameraNode::changeFilenameCB,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  std::stringstream take_a_picture_service_name;
  take_a_picture_service_name << "camera_" << camera_id_ << "/take_a_picture";
  take_a_picture_server_ = this->create_service<basler_camera_interfaces::srv::TakeAPicture>(
    take_a_picture_service_name.str(),
    std::bind(
      &BaslerCameraNode::takeAPictureCB,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
}

BaslerCameraNode::~BaslerCameraNode()
{
  camera_->StopGrabbing();
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  camera_->Close();
  PylonTerminate();
}

void BaslerCameraNode::changeFilenameCB(
  const std::shared_ptr<basler_camera_interfaces::srv::ChangeFilename::Request> request, 
  std::shared_ptr<basler_camera_interfaces::srv::ChangeFilename::Response> response){
  file_name = request->file_name;
  full_path = std::filesystem::path(file_directory) / file_name;
  response->success = true;
}

void BaslerCameraNode::takeAPictureCB(const std::shared_ptr<basler_camera_interfaces::srv::TakeAPicture::Request> request, 
    std::shared_ptr<basler_camera_interfaces::srv::TakeAPicture::Response> response) {
  std::string image_name = request->image_name;
  camera_->StartGrabbing();

  Pylon::CGrabResultPtr result;
  if (!camera_->RetrieveResult(5000, result, Pylon::TimeoutHandling_ThrowException))
  {
    response->success = false;
    return;
  }

  if (!result->GrabSucceeded())
  {
    RCLCPP_ERROR(get_logger(), "Frame grab failed");
    response->success = false;
    return;
  }

  CImageFormatConverter converter;
  converter.OutputPixelFormat = PixelType_BGR8packed;
  CPylonImage pylonImage;
  converter.Convert(pylonImage, result);

  cv::Mat raw(
    result->GetHeight(),
    result->GetWidth(),
    CV_8UC3,
    reinterpret_cast<uint8_t*>(pylonImage.GetBuffer())
  );

  std::filesystem::create_directories(full_path);
  std::ostringstream oss;
  oss << image_name << ".png";
  auto filename = full_path / oss.str();

  if (!cv::imwrite(filename.string(), raw))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write image %s", filename.c_str());
    response->success = false;
    return;
  }

  camera_->StopGrabbing();

  response->success = true;
}

void BaslerCameraNode::grabLoop()
{
  CImageFormatConverter format_converter;
  format_converter.OutputPixelFormat = PixelType_BGR8packed;
  CPylonImage pylon_image;

  while (rclcpp::ok() && camera_->IsGrabbing()) {
    CGrabResultPtr result;
    if (!camera_->RetrieveResult(
          5000, result, TimeoutHandling_ThrowException
        )) {
      continue;
    }

    format_converter.Convert(pylon_image, result);

    cv::Mat frame(
      result->GetHeight(),
      result->GetWidth(),
      CV_8UC3,
      reinterpret_cast<uint8_t*>(pylon_image.GetBuffer())
    );

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame
    ).toImageMsg();
    msg->header.stamp = this->now();

    pub_->publish(*msg);
  }
}

}  // namespace basler_camera

int main(int argc, char *argv[])
{
  std::string cam_id = argv[1];
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<basler_camera::BaslerCameraNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

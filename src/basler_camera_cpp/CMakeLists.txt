cmake_minimum_required(VERSION 3.8)
project(basler_camera_cpp)

set(PYLON_ROOT "/opt/pylon" CACHE PATH "Path to Pylon installation")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(basler_camera_interfaces REQUIRED)

include_directories(
  include
  ${PYLON_ROOT}/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PYLON_ROOT}/lib)

add_executable(basler_camera_node src/basler_camera_node.cpp)

ament_target_dependencies(basler_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  basler_camera_interfaces
)

target_link_libraries(basler_camera_node
    pylonbase
    pylonutility
    pylon_TL_gige 
    GCBase_gcc_v3_1_Basler_pylon_v3
)

install(TARGETS basler_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

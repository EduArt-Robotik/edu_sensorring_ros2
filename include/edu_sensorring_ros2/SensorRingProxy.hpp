#pragma once

#include <memory>
#include <sensorring/logger/Logger.hpp>
#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/measurement/DepthMeasurement.hpp>
#include <sensorring/measurement/ThermalMeasurement.hpp>
#include <sensorring/subscription/Subscription.hpp>
#include <string>
#include <vector>

#include "edu_sensorring_ros2/srv/start_thermal_calibration.hpp"
#include "edu_sensorring_ros2/srv/stop_thermal_calibration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace eduart {

namespace sensorring {

class SensorRingProxy : public rclcpp::Node {
public:
  SensorRingProxy(std::string node_name);

  ~SensorRingProxy();

  bool run(std::unique_ptr<manager::MeasurementManager> manager, std::string tf_name);

private:
  void onStateChange(const manager::ManagerState state);

  void onDepthFrame(const std::vector<measurement::DepthMeasurement>& frame);

  void onThermalFrame(const std::vector<measurement::ThermalMeasurement>& frame);

  void onLogMessage(const logger::LogVerbosity verbosity, const std::string& msg);

  void stopThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Request> request, std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Response> response);
  void startThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Request> request, std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Response> response);

  std::uint8_t* packPointData(const measurement::DepthMeasurement& src, std::uint8_t* dst);

  std::unique_ptr<manager::MeasurementManager> _manager;
  std::vector<subscription::Subscription> _subscriptions;

  // Combined point cloud publishers
  sensor_msgs::msg::PointCloud2 _pc2_msg_raw;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > _pointcloud_pub_raw;

  sensor_msgs::msg::PointCloud2 _pc2_msg_transformed;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > _pointcloud_pub_transformed;

  // Individual sensor point cloud publishers
  std::vector<sensor_msgs::msg::PointCloud2> _pc2_msg_individual_vec;
  std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > > _pointcloud_pub_individual_vec;

  // Thermal image publishers
  std::vector<std::shared_ptr<sensor_msgs::msg::Image> > _img_msg_vec;
  std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > > _img_pub_vec;

  std::vector<std::shared_ptr<sensor_msgs::msg::Image> > _colorimg_msg_vec;
  std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > > _colorimg_pub_vec;

  std::string _tf_name;
};

} // namespace sensorring

} // namespace eduart
#include "SensorRingProxy.hpp"

#include <algorithm>
#include <sstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace eduart {

namespace sensorring {

// Size of a packed point in the PointCloud2 message (5 floats + 1 int32)
static constexpr std::size_t PACKED_POINT_SIZE = 5 * sizeof(float) + sizeof(int32_t);

SensorRingProxy::SensorRingProxy(std::string node_name)
    : Node(node_name) {

  // Subscribe to logger first to capture all messages
  _subscriptions.emplace_back(logger::Logger::getInstance()->subscribe(std::bind(&SensorRingProxy::onLogMessage, this, std::placeholders::_1, std::placeholders::_2)));
}

SensorRingProxy::~SensorRingProxy() {
  // Subscriptions are automatically cancelled when _subscriptions vector is destroyed
}

bool SensorRingProxy::run(std::unique_ptr<manager::MeasurementManager> manager, std::string tf_name, bool enable_individual_publish, bool enable_combined_publish, bool enable_raw_publish) {

  _manager                   = std::move(manager);
  _tf_name                   = tf_name;
  _enable_individual_publish = enable_individual_publish;
  _enable_combined_publish   = enable_combined_publish;
  _enable_raw_publish        = enable_raw_publish;

  // Subscribe to state changes
  _subscriptions.emplace_back(_manager->subscribeToStateChanges(std::bind(&SensorRingProxy::onStateChange, this, std::placeholders::_1)));

  // Subscribe to depth sensors (synchronized frame delivery)
  _subscriptions.emplace_back(_manager->depthSensors().subscribeAll(std::bind(&SensorRingProxy::onDepthFrame, this, std::placeholders::_1)));

  // Subscribe to thermal sensors (synchronized frame delivery)
  _subscriptions.emplace_back(_manager->thermalSensors().subscribeAll(std::bind(&SensorRingProxy::onThermalFrame, this, std::placeholders::_1)));

  // Prepare PointCloud2 message template
  sensor_msgs::msg::PointCloud2 pc2_msg;
  pc2_msg.header.frame_id = tf_name;
  pc2_msg.height          = 1;
  pc2_msg.point_step      = PACKED_POINT_SIZE;
  pc2_msg.is_bigendian    = false;
  pc2_msg.is_dense        = true;

  sensor_msgs::msg::PointField field_x;
  field_x.name     = "x";
  field_x.offset   = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count    = 1;

  sensor_msgs::msg::PointField field_y;
  field_y.name     = "y";
  field_y.offset   = 4;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count    = 1;

  sensor_msgs::msg::PointField field_z;
  field_z.name     = "z";
  field_z.offset   = 8;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count    = 1;

  sensor_msgs::msg::PointField field_dist;
  field_dist.name     = "raw_distance";
  field_dist.offset   = 12;
  field_dist.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_dist.count    = 1;

  sensor_msgs::msg::PointField field_sigma;
  field_sigma.name     = "sigma";
  field_sigma.offset   = 16;
  field_sigma.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_sigma.count    = 1;

  sensor_msgs::msg::PointField field_idx;
  field_idx.name     = "sensor_idx";
  field_idx.offset   = 20;
  field_idx.datatype = sensor_msgs::msg::PointField::INT32;
  field_idx.count    = 1;

  pc2_msg.fields = { field_x, field_y, field_z, field_dist, field_sigma, field_idx };

  _pc2_msg_raw         = pc2_msg;
  _pc2_msg_transformed = pc2_msg;

  // Create combined pointCloud2 publishers
  if (_enable_raw_publish) {
    _pointcloud_pub_raw = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_raw", 1);
  }
  if (_enable_combined_publish) {
    _pointcloud_pub_transformed = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_transformed", 1);
  }

  // Prepare individual sensor publishers and static transforms using depth sensor poses from the header
  std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster> > tf_broadcasters;
  std::vector<std::string> published_tf_frames;

  const auto& depth_sensors = _manager->depthSensors();
  for (std::size_t board_idx = 0; board_idx < depth_sensors.size(); ++board_idx) {
    const auto& sensor = depth_sensors[board_idx];
    const auto& pose   = sensor.getGlobalPose();
    const std::string board_name = "board_" + std::to_string(board_idx);

    auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcasters.push_back(tf_broadcaster);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->now();
    t.header.frame_id = tf_name;
    t.child_frame_id  = board_name;

    t.transform.translation.x = pose.translation.x();
    t.transform.translation.y = pose.translation.y();
    t.transform.translation.z = pose.translation.z();
    tf2::Quaternion q;
    q.setRPY(pose.orientation.x() * M_PI / 180.0, pose.orientation.y() * M_PI / 180.0, pose.orientation.z() * M_PI / 180.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(t);
    published_tf_frames.push_back(t.child_frame_id);

    if (_enable_individual_publish) {
      sensor_msgs::msg::PointCloud2 individual_msg = pc2_msg;
      individual_msg.header.frame_id               = t.child_frame_id;
      _pc2_msg_individual_vec.push_back(individual_msg);
      _pointcloud_pub_individual_vec.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_individual/" + board_name, 1));
    }
  }

  if (!published_tf_frames.empty()) {
    std::ostringstream tf_stream;
    tf_stream << "Publishing static TF frames: ";
    for (std::size_t i = 0; i < published_tf_frames.size(); ++i) {
      if (i > 0) {
        tf_stream << ", ";
      }
      tf_stream << published_tf_frames[i];
    }
    RCLCPP_INFO_STREAM(this->get_logger(), tf_stream.str());
  }

  // Prepare thermal image publishers
  std::size_t thermal_sensor_count = _manager->thermalSensors().size();
  for (std::size_t i = 0; i < thermal_sensor_count; i++) {
    // Grayscale image
    std::string sensor_name  = "thermal_sensor_" + std::to_string(i) + "/grayscale";
    auto img_msg             = std::make_shared<sensor_msgs::msg::Image>();
    img_msg->header.frame_id = sensor_name;
    img_msg->height          = 32;
    img_msg->width           = 32;
    img_msg->encoding        = "mono8";
    img_msg->is_bigendian    = false;
    img_msg->step            = img_msg->width * 1;
    auto img_pub             = this->create_publisher<sensor_msgs::msg::Image>("/sensors/" + sensor_name, 1);
    _img_msg_vec.push_back(img_msg);
    _img_pub_vec.push_back(img_pub);

    // False color image
    sensor_name                   = "thermal_sensor_" + std::to_string(i) + "/falsecolor";
    auto colorimg_msg             = std::make_shared<sensor_msgs::msg::Image>();
    colorimg_msg->header.frame_id = sensor_name;
    colorimg_msg->height          = 32;
    colorimg_msg->width           = 32;
    colorimg_msg->encoding        = "rgb8";
    colorimg_msg->is_bigendian    = false;
    colorimg_msg->step            = colorimg_msg->width * 3;
    auto colorimg_pub             = this->create_publisher<sensor_msgs::msg::Image>("/sensors/" + sensor_name, 1);
    _colorimg_msg_vec.push_back(colorimg_msg);
    _colorimg_pub_vec.push_back(colorimg_pub);
  }

  // Set up light color subscriber
  _light_sub = this->create_subscription<std_msgs::msg::ColorRGBA>("/lights/set_color", 1, std::bind(&SensorRingProxy::onLightColor, this, std::placeholders::_1));

  // Set up ROS services
  auto stop_cali_srv = this->create_service<edu_sensorring_ros2::srv::StopThermalCalibration>(
      std::string(this->get_name()) + "/stopThermalCalibration", std::bind(&SensorRingProxy::stopThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));
  auto start_cali_srv = this->create_service<edu_sensorring_ros2::srv::StartThermalCalibration>(
      std::string(this->get_name()) + "/startThermalCalibration", std::bind(&SensorRingProxy::startThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));

  // Force first state update
  onStateChange(_manager->getManagerState());

  bool success = _manager->startMeasuring();

  if (success) {
    while (_manager->isMeasuring() && rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
    }

    success = _manager->stopMeasuring();
  }

  return success;
}

void SensorRingProxy::onLightColor(std_msgs::msg::ColorRGBA::SharedPtr msg) {
  // Set all lights to the received color
  for (auto& light : _manager->lights()) {
    const auto r = static_cast<std::uint8_t>(std::clamp(msg->r, 0.0f, 1.0f) * 255);
    const auto g = static_cast<std::uint8_t>(std::clamp(msg->g, 0.0f, 1.0f) * 255);
    const auto b = static_cast<std::uint8_t>(std::clamp(msg->b, 0.0f, 1.0f) * 255);
    light.setLight(device::LightMode::FixedColor, r, g, b);
  }
}

void SensorRingProxy::onStateChange(const manager::ManagerState state) {
  if (state < manager::ManagerState::Error) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "New MeasurementManager state: " << state);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "New MeasurementManager state: " << state);
  }
}

void SensorRingProxy::onDepthFrame(const std::vector<measurement::DepthMeasurement>& frame) {
  auto now = this->now();

  // Publish individual sensor point clouds
  if (_enable_individual_publish) {
    for (std::size_t idx = 0; idx < frame.size(); idx++) {
      if (idx < _pc2_msg_individual_vec.size()) {
        auto& msg        = _pc2_msg_individual_vec.at(idx);
        msg.header.stamp = now;
        msg.width        = frame[idx].point_cloud.data.size();
        msg.row_step     = msg.width * msg.point_step;
        msg.data.resize(msg.row_step);
        packPointData(frame[idx], msg.data.data());
        _pointcloud_pub_individual_vec.at(idx)->publish(msg);
      }
    }
  }

  if (!_enable_raw_publish && !_enable_combined_publish) {
    return;
  }

  std::size_t total_points = 0;
  for (const auto& m : frame) {
    total_points += m.point_cloud.data.size();
  }

  // Publish combined raw point cloud
  if (_enable_raw_publish) {
    _pc2_msg_raw.header.stamp = now;
    _pc2_msg_raw.width        = total_points;
    _pc2_msg_raw.row_step     = _pc2_msg_raw.width * _pc2_msg_raw.point_step;
    _pc2_msg_raw.data.resize(_pc2_msg_raw.row_step);

    std::uint8_t* raw_ptr = _pc2_msg_raw.data.data();
    for (const auto& m : frame) {
      raw_ptr = packPointData(m, raw_ptr);
    }
    _pointcloud_pub_raw->publish(_pc2_msg_raw);
  }

  // Publish combined transformed point cloud
  if (_enable_combined_publish) {
    _pc2_msg_transformed.header.stamp = now;
    _pc2_msg_transformed.width        = total_points;
    _pc2_msg_transformed.row_step     = _pc2_msg_transformed.width * _pc2_msg_transformed.point_step;
    _pc2_msg_transformed.data.resize(_pc2_msg_transformed.row_step);

    std::uint8_t* transformed_ptr = _pc2_msg_transformed.data.data();
    for (const auto& m : frame) {
      auto transformed_cloud = m.transformToGlobalFrame();
      measurement::DepthMeasurement transformed_meas;
      transformed_meas.header          = m.header;
      transformed_meas.point_cloud     = transformed_cloud;
      transformed_meas.nr_valid_points = m.nr_valid_points;
      transformed_ptr                  = packPointData(transformed_meas, transformed_ptr);
    }
    _pointcloud_pub_transformed->publish(_pc2_msg_transformed);
  }
}

void SensorRingProxy::onThermalFrame(const std::vector<measurement::ThermalMeasurement>& frame) {
  auto now_stamp = this->now();

  for (std::size_t idx = 0; idx < frame.size(); idx++) {
    if (idx >= _img_msg_vec.size()) {
      continue;
    }

    const auto& meas = frame[idx];

    // Convert temperature image to grayscale and publish
    auto grayscale        = meas.temperatures.toGrayscale();
    auto img_msg          = _img_msg_vec.at(idx);
    std::size_t size      = img_msg->width * img_msg->height;
    img_msg->header.stamp = now_stamp;
    img_msg->data.resize(size);
    std::copy_n(grayscale.data.begin(), size, img_msg->data.data());
    _img_pub_vec[idx]->publish(*img_msg);

    // Convert temperature image to false color and publish
    auto falsecolor            = meas.temperatures.toFalseColor();
    auto colorimg_msg          = _colorimg_msg_vec.at(idx);
    std::size_t color_size     = colorimg_msg->width * colorimg_msg->height * 3;
    colorimg_msg->header.stamp = now_stamp;
    colorimg_msg->data.resize(color_size);
    const std::uint8_t* color_data_ptr = falsecolor.data.begin()->begin();
    std::copy_n(color_data_ptr, color_size, colorimg_msg->data.data());
    _colorimg_pub_vec[idx]->publish(*colorimg_msg);
  }
}

void SensorRingProxy::onLogMessage(const logger::LogVerbosity verbosity, const std::string& msg) {
  switch (verbosity) {
  case logger::LogVerbosity::Debug:
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    break;
  case logger::LogVerbosity::Info:
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
    break;
  case logger::LogVerbosity::Warning:
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    break;
  case logger::LogVerbosity::Error:
    RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
    break;
  case logger::LogVerbosity::Exception:
    RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
    break;
  }
}

void SensorRingProxy::stopThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Request> request, std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Response> response) {
  if (request->stop) {
    bool success = true;
    for (auto& sensor : _manager->thermalSensors()) {
      success &= sensor.stopCalibration();
    }
    response->output = success;
  } else {
    response->output = false;
  }
}

void SensorRingProxy::startThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Request> request, std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Response> response) {
  bool success = true;
  for (auto& sensor : _manager->thermalSensors()) {
    success &= sensor.startCalibration(static_cast<unsigned int>(request->window));
  }
  response->output = success;
}

std::uint8_t* SensorRingProxy::packPointData(const measurement::DepthMeasurement& src, std::uint8_t* dst) {
  for (const auto& p : src.point_cloud.data) {
    float* f                                               = reinterpret_cast<float*>(dst);
    f[0]                                                   = static_cast<float>(p.point.data[0]);
    f[1]                                                   = static_cast<float>(p.point.data[1]);
    f[2]                                                   = static_cast<float>(p.point.data[2]);
    f[3]                                                   = static_cast<float>(p.raw_distance);
    f[4]                                                   = static_cast<float>(p.sigma);
    reinterpret_cast<int32_t*>(dst + 5 * sizeof(float))[0] = static_cast<int32_t>(p.sensor_index);
    dst += PACKED_POINT_SIZE;
  }

  return dst;
}

} // namespace sensorring

} // namespace eduart
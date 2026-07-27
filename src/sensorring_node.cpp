#include <sensorring/SensorRingFactory.hpp>
#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/device/light/ws2812b/WS2812b_Device.hpp>

#include "rclcpp/rclcpp.hpp"

#include "NodeSetup.hpp"
#include "SensorRingProxy.hpp"

using namespace eduart::sensorring;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SensorRingProxy>("edu_sensorring_ros2");
  RCLCPP_INFO(node->get_logger(), "Starting the sensorring node");

  const std::string ns = "pointcloud_sensor";

  // Read all parameters
  const auto base              = readBaseSetup(*node, ns);
  const auto htpa32_defaults   = readHtpa32Params(*node, ns);
  const auto vl53l8cx_defaults = readVl53l8cxParams(*node, ns);
  const auto tmf8829_defaults  = readTmf8829Params(*node, ns);
  const auto light             = readLightSetup(*node, ns);
  const auto depth_publish     = readDepthPublishSetup(*node, ns);

  // Build factory and configure topology
  const ValidationMode validation_mode = base.enforce_topology ? ValidationMode::Strict : ValidationMode::Relaxed;
  SensorRingFactory factory(validation_mode);
  factory.setDefaultDeviceParams(htpa32_defaults);
  factory.setDefaultDeviceParams(vl53l8cx_defaults);
  factory.setDefaultDeviceParams(tmf8829_defaults);

  if (base.auto_discover) {
    RCLCPP_INFO(node->get_logger(), "Auto-discover mode enabled. Discovering hardware on configured interfaces...");
  }
  configureTopology(*node, ns, factory, base.auto_discover);

  // Start manager and set initial light state
  auto manager = std::make_unique<manager::MeasurementManager>(base.manager_params, factory);
  device::WS2812b_Device::setAllLights(manager->lights().get(), light.mode, light.color[0], light.color[1], light.color[2]);

  return node->run(std::move(manager), base.tf_name, depth_publish.enable_individual, depth_publish.enable_combined, depth_publish.enable_raw) ? 0 : 1;
}
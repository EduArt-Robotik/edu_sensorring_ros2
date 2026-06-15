#pragma once

#include <sensorring/SensorRingFactory.hpp>
#include <sensorring/device/depth/tmf8829/TMF8829_Params.hpp>
#include <sensorring/device/depth/vl53l8cx/VL53L8CX_Params.hpp>
#include <sensorring/device/light/LightMode.hpp>
#include <sensorring/device/thermal/htpa32/HTPA32_Params.hpp>
#include <sensorring/interface/ComInterfaceID.hpp>
#include <sensorring/interface/InterfaceParams.hpp>
#include <sensorring/manager/ManagerParams.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace eduart::sensorring {

/**
 * @brief Holds the result of reading the base/manager ROS parameters.
 */
struct BaseSetup {
  manager::ManagerParams manager_params;
  std::string tf_name;
  bool enforce_topology;
  bool auto_discover;
};

/**
 * @brief Holds the result of reading the LED/light ROS parameters.
 */
struct LightSetup {
  device::LightMode mode;
  std::vector<long> color; ///< RGB values, each in [0, 255]
};

/**
 * @brief Holds depth point cloud publish behavior flags.
 */
struct DepthPublishSetup {
  bool enable_individual;
  bool enable_combined;
  bool enable_raw;
};

/// Declares and reads base manager parameters from @p node under @p ns.
BaseSetup readBaseSetup(rclcpp::Node& node, const std::string& ns);

/// Declares and reads HTPA32 thermal sensor parameters from @p node under @p ns.
device::HTPA32_Params readHtpa32Params(rclcpp::Node& node, const std::string& ns);

/// Declares and reads VL53L8CX depth sensor parameters from @p node under @p ns.
device::VL53L8CX_Params readVl53l8cxParams(rclcpp::Node& node, const std::string& ns);

/// Declares and reads TMF8829 depth sensor parameters from @p node under @p ns.
device::TMF8829_Params readTmf8829Params(rclcpp::Node& node, const std::string& ns);

/// Declares and reads LED configuration from @p node under @p ns. Validates color range.
LightSetup readLightSetup(rclcpp::Node& node, const std::string& ns);

/// Declares and reads depth point cloud publish flags from @p node under @p ns.
DepthPublishSetup readDepthPublishSetup(rclcpp::Node& node, const std::string& ns);

/// Maps an interface type string ("socketcan", "usbtingo") to a @c com::InterfaceType.
com::InterfaceType parseInterfaceType(const std::string& type_str);

/**
 * @brief Declares topology parameters, then configures @p factory with interfaces and
 *        (in non-auto-discover mode) expected sensor boards.
 *
 * @param htpa32_defaults  Default HTPA32 params used as base for per-interface orientation overrides.
 */
void configureTopology(rclcpp::Node& node, const std::string& ns, SensorRingFactory& factory, bool auto_discover, const device::HTPA32_Params& htpa32_defaults);

} // namespace eduart::sensorring

#include "NodeSetup.hpp"

#include <chrono>
#include <sensorring/board/SensorBoardParams.hpp>
#include <stdexcept>

namespace eduart::sensorring {

BaseSetup readBaseSetup(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".base_setup.timeout_ms", 1000);
  node.declare_parameter(ns + ".base_setup.repair_errors", true);
  node.declare_parameter(ns + ".base_setup.tf_name", "base_sensorring");
  node.declare_parameter(ns + ".base_setup.enforce_topology", false);
  node.declare_parameter(ns + ".base_setup.auto_discover", false);

  BaseSetup result;
  result.manager_params.timeout       = std::chrono::milliseconds(node.get_parameter(ns + ".base_setup.timeout_ms").as_int());
  result.manager_params.repair_errors = node.get_parameter(ns + ".base_setup.repair_errors").as_bool();
  result.tf_name                      = node.get_parameter(ns + ".base_setup.tf_name").as_string();
  result.enforce_topology             = node.get_parameter(ns + ".base_setup.enforce_topology").as_bool();
  result.auto_discover                = node.get_parameter(ns + ".base_setup.auto_discover").as_bool();
  return result;
}

device::HTPA32_Params readHtpa32Params(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".htpa32_config.auto_min_max", true);
  node.declare_parameter(ns + ".htpa32_config.use_eeprom_file", false);
  node.declare_parameter(ns + ".htpa32_config.use_calibration_file", false);
  node.declare_parameter(ns + ".htpa32_config.eeprom_file_dir", "");
  node.declare_parameter(ns + ".htpa32_config.calibration_file_dir", "");
  node.declare_parameter(ns + ".htpa32_config.scale_t_min_deg", 20.0);
  node.declare_parameter(ns + ".htpa32_config.scale_t_max_deg", 30.0);
  node.declare_parameter(ns + ".htpa32_config.max_rate_hz", 5.0);

  device::HTPA32_Params params;
  params.auto_min_max         = node.get_parameter(ns + ".htpa32_config.auto_min_max").as_bool();
  params.use_eeprom_file      = node.get_parameter(ns + ".htpa32_config.use_eeprom_file").as_bool();
  params.use_calibration_file = node.get_parameter(ns + ".htpa32_config.use_calibration_file").as_bool();
  params.eeprom_dir           = node.get_parameter(ns + ".htpa32_config.eeprom_file_dir").as_string();
  params.calibration_dir      = node.get_parameter(ns + ".htpa32_config.calibration_file_dir").as_string();
  params.t_min_deg_c          = node.get_parameter(ns + ".htpa32_config.scale_t_min_deg").as_double();
  params.t_max_deg_c          = node.get_parameter(ns + ".htpa32_config.scale_t_max_deg").as_double();
  params.max_rate_hz          = node.get_parameter(ns + ".htpa32_config.max_rate_hz").as_double();
  return params;
}

device::VL53L8CX_Params readVl53l8cxParams(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".vl53l8cx_config.max_rate_hz", 15.0);

  device::VL53L8CX_Params params;
  params.max_rate_hz = node.get_parameter(ns + ".vl53l8cx_config.max_rate_hz").as_double();
  return params;
}

device::TMF8829_Params readTmf8829Params(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".tmf8829_config.resolution_mode", 3); // 0=8x8 … 8=48x32HA
  node.declare_parameter(ns + ".tmf8829_config.k_iterations", 0);
  node.declare_parameter(ns + ".tmf8829_config.max_rate_hz", 30.0);

  device::TMF8829_Params params;
  params.resolution_mode = static_cast<device::ResolutionMode>(node.get_parameter(ns + ".tmf8829_config.resolution_mode").as_int());
  params.k_iterations    = static_cast<std::uint16_t>(node.get_parameter(ns + ".tmf8829_config.k_iterations").as_int());
  params.max_rate_hz     = node.get_parameter(ns + ".tmf8829_config.max_rate_hz").as_double();
  return params;
}

LightSetup readLightSetup(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".led_config.initial_mode", 0);
  node.declare_parameter(ns + ".led_config.initial_color", std::vector<int>{ 0, 0, 0 });

  LightSetup result;
  result.mode  = static_cast<device::LightMode>(node.get_parameter(ns + ".led_config.initial_mode").as_int());
  result.color = node.get_parameter(ns + ".led_config.initial_color").as_integer_array();

  if (result.color.size() != 3) {
    throw std::runtime_error("Light color vector has wrong length! Expected 3 values for RGB color.");
  }
  for (const auto& v : result.color) {
    if (v < 0 || v > 255) {
      throw std::runtime_error("Light color values must be in the range [0, 255]!");
    }
  }
  return result;
}

DepthPublishSetup readDepthPublishSetup(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".base_setup.publishers.depth_individual", true);
  node.declare_parameter(ns + ".base_setup.publishers.depth_combined", true);
  node.declare_parameter(ns + ".base_setup.publishers.depth_raw", true);

  DepthPublishSetup result;
  result.enable_individual = node.get_parameter(ns + ".base_setup.publishers.depth_individual").as_bool();
  result.enable_combined   = node.get_parameter(ns + ".base_setup.publishers.depth_combined").as_bool();
  result.enable_raw        = node.get_parameter(ns + ".base_setup.publishers.depth_raw").as_bool();
  return result;
}

com::InterfaceType parseInterfaceType(const std::string& type_str) {
  if (type_str == "socketcan")
    return com::InterfaceType::SocketCan;
  if (type_str == "usbtingo")
    return com::InterfaceType::UsbTingo;
  return com::InterfaceType::Undefined;
}

void configureTopology(rclcpp::Node& node, const std::string& ns, SensorRingFactory& factory, bool auto_discover, const device::HTPA32_Params& htpa32_defaults) {
  node.declare_parameter(ns + ".topology.nr_of_interfaces", 1);
  const int nr_of_interfaces     = node.get_parameter(ns + ".topology.nr_of_interfaces").as_int();
  std::string topology_namespace = ns + ".topology.interfaces";

  for (int i = 0; i < nr_of_interfaces; i++) {
    const std::string iface_prefix = topology_namespace + ".interface_" + std::to_string(i);

    node.declare_parameter(iface_prefix + ".interface_type", "undefined");
    node.declare_parameter(iface_prefix + ".interface_name", "can0");
    node.declare_parameter(iface_prefix + ".enable_brs", false);
    node.declare_parameter(iface_prefix + ".data_baudrate", 0);
    node.declare_parameter(iface_prefix + ".sample_point", 0.0);

    const std::string iface_name        = node.get_parameter(iface_prefix + ".interface_name").as_string();
    const std::string iface_type_str    = node.get_parameter(iface_prefix + ".interface_type").as_string();
    const bool iface_brs                = node.get_parameter(iface_prefix + ".enable_brs").as_bool();
    const unsigned int data_baudrate    = node.get_parameter(iface_prefix + ".data_baudrate").as_int();
    const double sample_point           = node.get_parameter(iface_prefix + ".sample_point").as_double();
    const com::InterfaceType iface_type = parseInterfaceType(iface_type_str);

    if (iface_type == com::InterfaceType::SocketCan) {
      factory.addInterface(com::SocketCanParams(iface_name, iface_brs, data_baudrate, sample_point));
    } else if (iface_type == com::InterfaceType::UsbTingo) {
      factory.addInterface(com::UsbTingoParams(iface_name, iface_brs, data_baudrate, sample_point));
    } else {
      factory.addInterface(com::SocketCanParams(iface_name, iface_brs, data_baudrate, sample_point));
    }

    if (auto_discover)
      continue;

    node.declare_parameter(iface_prefix + ".nr_of_sensors", 1);

    const int nr_of_sensors = node.get_parameter(iface_prefix + ".nr_of_sensors").as_int();

    const std::string sensors_prefix = iface_prefix + ".sensors";
    for (int j = 0; j < nr_of_sensors; j++) {
      const std::string sensor_prefix = sensors_prefix + ".sensor_" + std::to_string(j);

      node.declare_parameter(sensor_prefix + ".enable_tof", true);
      node.declare_parameter(sensor_prefix + ".enable_thermal", false);
      node.declare_parameter(sensor_prefix + ".enable_light", false);
      node.declare_parameter(sensor_prefix + ".orientation", "none");
      node.declare_parameter(sensor_prefix + ".rotation", std::vector<double>{ 0.0, 0.0, 0.0 });
      node.declare_parameter(sensor_prefix + ".translation", std::vector<double>{ 0.0, 0.0, 0.0 });

      const bool enable_tof                 = node.get_parameter(sensor_prefix + ".enable_tof").as_bool();
      const bool enable_thermal             = node.get_parameter(sensor_prefix + ".enable_thermal").as_bool();
      const bool enable_light               = node.get_parameter(sensor_prefix + ".enable_light").as_bool();
      const std::string orientation_str     = node.get_parameter(sensor_prefix + ".orientation").as_string();
      const std::vector<double> rotation    = node.get_parameter(sensor_prefix + ".rotation").as_double_array();
      const std::vector<double> translation = node.get_parameter(sensor_prefix + ".translation").as_double_array();

      board::Orientation orientation = board::Orientation::None;
      if (orientation_str == "left")
        orientation = board::Orientation::Left;
      if (orientation_str == "right")
        orientation = board::Orientation::Right;

      if (rotation.size() != 3) {
        throw std::invalid_argument("Rotation vector of sensor " + std::to_string(j) + " on interface " + iface_name + " has wrong length!");
      }
      if (translation.size() != 3) {
        throw std::invalid_argument("Translation vector of sensor " + std::to_string(j) + " on interface " + iface_name + " has wrong length!");
      }

      board::SensorBoardParams board_params;
      board_params.rotation    = { rotation[0], rotation[1], rotation[2] };
      board_params.translation = { translation[0], translation[1], translation[2] };
      board_params.orientation = orientation;

      factory.expectBoard(board_params);

      if (enable_tof) {
        factory.expectDevice(device::DepthSensorParams{});
      }

      device::HTPA32_Params thermal_params = htpa32_defaults;
      if (enable_thermal) {
        factory.expectDevice(thermal_params);
      }

      if (enable_light) {
        factory.expectDevice(device::LightParams{});
      }
    }
  }
}

} // namespace eduart::sensorring

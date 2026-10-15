#include "NodeSetup.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sensorring/board/SensorBoardParams.hpp>
#include <stdexcept>

namespace eduart::sensorring {

namespace {

std::string normalizeToken(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    if (c == '-' || c == ' ') {
      return static_cast<char>('_');
    }
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

board::SensorBoardType parseBoardType(const std::string& board_type_str) {
  const std::string normalized = normalizeToken(board_type_str);

  if (normalized.empty() || normalized == "undefined" || normalized == "none") {
    return board::SensorBoardType::Undefined;
  }
  if (normalized == "sidepanel") {
    return board::SensorBoardType::Sidepanel;
  }
  if (normalized == "headlight") {
    return board::SensorBoardType::Headlight;
  }
  if (normalized == "taillight") {
    return board::SensorBoardType::Taillight;
  }
  if (normalized == "minipanel" || normalized == "mini_panel") {
    return board::SensorBoardType::Minipanel;
  }

  throw std::invalid_argument("Unsupported board_type value '" + board_type_str + "'.");
}

device::DeviceType parseExpectedDeviceType(const std::string& device_type_str) {
  const std::string normalized = normalizeToken(device_type_str);

  if (normalized == "vl53l8cx") {
    return device::DeviceType::VL53L8CX;
  }
  if (normalized == "tmf8829") {
    return device::DeviceType::TMF8829;
  }
  if (normalized == "htpa32") {
    return device::DeviceType::HTPA32;
  }
  if (normalized == "ws2812b") {
    return device::DeviceType::WS2812b;
  }
  if (normalized == "any_depth" || normalized == "depth" || normalized == "tof") {
    return device::DeviceType::AnyDepth;
  }
  if (normalized == "any_thermal" || normalized == "thermal") {
    return device::DeviceType::AnyThermal;
  }
  if (normalized == "any_light" || normalized == "light" || normalized == "led") {
    return device::DeviceType::AnyLight;
  }

  throw std::invalid_argument("Unsupported expected_devices entry '" + device_type_str + "'.");
}

} // namespace

BaseSetup readBaseSetup(rclcpp::Node& node, const std::string& ns) {
  const manager::ManagerParams defaults;
  node.declare_parameter(ns + ".base_setup.timeout_ms", defaults.timeout.count());
  node.declare_parameter(ns + ".base_setup.repair_errors", defaults.repair_errors);
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
  const device::HTPA32_Params defaults;
  node.declare_parameter(ns + ".htpa32_config.enable", defaults.enable);
  node.declare_parameter(ns + ".htpa32_config.auto_min_max", defaults.auto_min_max);
  node.declare_parameter(ns + ".htpa32_config.use_eeprom_file", defaults.use_eeprom_file);
  node.declare_parameter(ns + ".htpa32_config.use_calibration_file", defaults.use_calibration_file);
  node.declare_parameter(ns + ".htpa32_config.eeprom_file_dir", defaults.eeprom_dir);
  node.declare_parameter(ns + ".htpa32_config.calibration_file_dir", defaults.calibration_dir);
  node.declare_parameter(ns + ".htpa32_config.scale_t_min_deg", defaults.t_min_deg_c);
  node.declare_parameter(ns + ".htpa32_config.scale_t_max_deg", defaults.t_max_deg_c);
  node.declare_parameter(ns + ".htpa32_config.max_rate_hz", defaults.max_rate_hz);

  device::HTPA32_Params params;
  params.enable               = node.get_parameter(ns + ".htpa32_config.enable").as_bool();
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
  const device::VL53L8CX_Params defaults;
  node.declare_parameter(ns + ".vl53l8cx_config.enable", defaults.enable);
  node.declare_parameter(ns + ".vl53l8cx_config.max_rate_hz", defaults.max_rate_hz);

  device::VL53L8CX_Params params;
  params.enable      = node.get_parameter(ns + ".vl53l8cx_config.enable").as_bool();
  params.max_rate_hz = node.get_parameter(ns + ".vl53l8cx_config.max_rate_hz").as_double();
  return params;
}

device::TMF8829_Params readTmf8829Params(rclcpp::Node& node, const std::string& ns) {
  const device::TMF8829_Params defaults;
  node.declare_parameter(ns + ".tmf8829_config.enable", defaults.enable);
  node.declare_parameter(ns + ".tmf8829_config.resolution_mode", static_cast<int>(defaults.resolution_mode));
  node.declare_parameter(ns + ".tmf8829_config.k_iterations", static_cast<int>(defaults.k_iterations));
  node.declare_parameter(ns + ".tmf8829_config.max_rate_hz", defaults.max_rate_hz);

  device::TMF8829_Params params;
  params.enable          = node.get_parameter(ns + ".tmf8829_config.enable").as_bool();
  params.resolution_mode = static_cast<device::ResolutionMode>(node.get_parameter(ns + ".tmf8829_config.resolution_mode").as_int());
  params.k_iterations    = static_cast<std::uint16_t>(node.get_parameter(ns + ".tmf8829_config.k_iterations").as_int());
  params.max_rate_hz     = node.get_parameter(ns + ".tmf8829_config.max_rate_hz").as_double();
  return params;
}

LightSetup readLightSetup(rclcpp::Node& node, const std::string& ns) {
  node.declare_parameter(ns + ".led_config.initial_mode", static_cast<int>(device::LightMode::Off) - 2);
  node.declare_parameter(ns + ".led_config.initial_color", std::vector<int>{ 0, 0, 0 });
  const device::LightParams defaults;
  node.declare_parameter(ns + ".led_config.distance_map_lower_m", defaults.distance_map_lower_m);
  node.declare_parameter(ns + ".led_config.distance_map_upper_m", defaults.distance_map_upper_m);
  node.declare_parameter(ns + ".led_config.max_brightness", defaults.max_brightness);

  LightSetup result;
  result.params.distance_map_lower_m = node.get_parameter(ns + ".led_config.distance_map_lower_m").as_double();
  result.params.distance_map_upper_m = node.get_parameter(ns + ".led_config.distance_map_upper_m").as_double();
  result.params.max_brightness = node.get_parameter(ns + ".led_config.max_brightness").as_int();
  const int initial_mode = node.get_parameter(ns + ".led_config.initial_mode").as_int();
  constexpr int min_user_mode = 0;
  constexpr int max_user_mode = 11; // 0=Off ... 11=PulsationColor
  if (initial_mode < min_user_mode || initial_mode > max_user_mode) {
    throw std::runtime_error("Light mode is out of range! Expected values in [0, 11].");
  }
  // ROS params use a compact 0-based mode index, while transport mode bytes start at 0x02.
  result.mode  = static_cast<device::LightMode>(initial_mode + 2);
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
  node.declare_parameter(ns + ".publishers.depth_individual", true);
  node.declare_parameter(ns + ".publishers.depth_combined", true);
  node.declare_parameter(ns + ".publishers.depth_raw", true);

  DepthPublishSetup result;
  result.enable_individual = node.get_parameter(ns + ".publishers.depth_individual").as_bool();
  result.enable_combined   = node.get_parameter(ns + ".publishers.depth_combined").as_bool();
  result.enable_raw        = node.get_parameter(ns + ".publishers.depth_raw").as_bool();
  return result;
}

com::InterfaceType parseInterfaceType(const std::string& type_str) {
  const std::string normalized = normalizeToken(type_str);
  if (normalized == "socketcan")
    return com::InterfaceType::SocketCan;
  if (normalized == "usbtingo")
    return com::InterfaceType::UsbTingo;
  return com::InterfaceType::Undefined;
}

void configureTopology(rclcpp::Node& node, const std::string& ns, SensorRingFactory& factory, bool auto_discover) {
  node.declare_parameter(ns + ".topology.nr_of_interfaces", 1);
  const int nr_of_interfaces     = node.get_parameter(ns + ".topology.nr_of_interfaces").as_int();
  std::string topology_namespace = ns + ".topology.interfaces";

  for (int i = 0; i < nr_of_interfaces; i++) {
    const std::string iface_prefix = topology_namespace + ".interface_" + std::to_string(i);
    const com::SocketCanParams interface_defaults;

    node.declare_parameter(iface_prefix + ".interface_type", com::toString(com::InterfaceType::Undefined));
    node.declare_parameter(iface_prefix + ".interface_name", interface_defaults.name);
    node.declare_parameter(iface_prefix + ".enable_brs", interface_defaults.respond_with_brs);
    node.declare_parameter(iface_prefix + ".data_baudrate", static_cast<int>(interface_defaults.data_bitrate));
    node.declare_parameter(iface_prefix + ".sample_point", static_cast<double>(interface_defaults.data_sample_point));

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

    node.declare_parameter(iface_prefix + ".nr_of_boards", 1);

    const int nr_of_boards = node.get_parameter(iface_prefix + ".nr_of_boards").as_int();

    const std::string boards_prefix = iface_prefix + ".boards";
    for (int j = 0; j < nr_of_boards; j++) {
      const std::string board_prefix = boards_prefix + ".board_" + std::to_string(j);
      const board::SensorBoardParams board_defaults;

      node.declare_parameter(board_prefix + ".board_type", board::toString(board_defaults.board_type));
      node.declare_parameter(board_prefix + ".expected_devices", std::vector<std::string>{});
      node.declare_parameter(board_prefix + ".orientation", board::toString(board_defaults.orientation));
      node.declare_parameter(board_prefix + ".rotation", std::vector<double>{ board_defaults.rotation[0], board_defaults.rotation[1], board_defaults.rotation[2] });
      node.declare_parameter(board_prefix + ".translation", std::vector<double>{ board_defaults.translation[0], board_defaults.translation[1], board_defaults.translation[2] });

      const std::string board_type_str           = node.get_parameter(board_prefix + ".board_type").as_string();
      const std::vector<std::string> expected_devices = node.get_parameter(board_prefix + ".expected_devices").as_string_array();
      const std::string orientation_str          = node.get_parameter(board_prefix + ".orientation").as_string();
      const std::vector<double> rotation         = node.get_parameter(board_prefix + ".rotation").as_double_array();
      const std::vector<double> translation      = node.get_parameter(board_prefix + ".translation").as_double_array();

      board::Orientation orientation = board::Orientation::None;
      if (orientation_str == "left")
        orientation = board::Orientation::Left;
      if (orientation_str == "right")
        orientation = board::Orientation::Right;

      if (rotation.size() != 3) {
        throw std::invalid_argument("Rotation vector of board " + std::to_string(j) + " on interface " + iface_name + " has wrong length!");
      }
      if (translation.size() != 3) {
        throw std::invalid_argument("Translation vector of board " + std::to_string(j) + " on interface " + iface_name + " has wrong length!");
      }

      board::SensorBoardParams board_params;
      board_params.board_type = parseBoardType(board_type_str);
      board_params.rotation    = { rotation[0], rotation[1], rotation[2] };
      board_params.translation = { translation[0], translation[1], translation[2] };
      board_params.orientation = orientation;

      factory.expectBoard(board_params);

      for (const auto& expected_device : expected_devices) {
        factory.expectDevice(parseExpectedDeviceType(expected_device));
      }
    }
  }
}

} // namespace eduart::sensorring

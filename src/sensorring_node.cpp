#include <sensorring/SensorRingFactory.hpp>
#include <sensorring/board/SensorBoardParams.hpp>
#include <sensorring/device/light/LightMode.hpp>
#include <sensorring/device/thermal/htpa32/HTPA32_Params.hpp>
#include <sensorring/interface/ComInterfaceID.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "SensorRingProxy.hpp"

using namespace eduart;
using namespace eduart::sensorring;

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  std::string tf_name;
  manager::ManagerParams manager_params;

  // Create SensorRing Node
  auto measurement_node = std::make_shared<sensorring::SensorRingProxy>("edu_sensorring_ros2");
  RCLCPP_INFO(measurement_node->get_logger(), "Starting the sensorring node");

  // Get SensorRing parameters
  std::string param_namespace = "pointcloud_sensor";
  measurement_node->declare_parameter(param_namespace + ".base_setup.timeout_ms", 1000);
  measurement_node->declare_parameter(param_namespace + ".base_setup.enable_brs", false);
  measurement_node->declare_parameter(param_namespace + ".base_setup.repair_errors", true);
  measurement_node->declare_parameter(param_namespace + ".base_setup.tf_name", "base_sensorring");
  measurement_node->declare_parameter(param_namespace + ".base_setup.enforce_topology", false);
  measurement_node->declare_parameter(param_namespace + ".base_setup.auto_discover", false);
  measurement_node->declare_parameter(param_namespace + ".base_setup.frequency_tof_hz", 0.0);
  measurement_node->declare_parameter(param_namespace + ".base_setup.frequency_thermal_hz", 5.0);
  measurement_node->declare_parameter(param_namespace + ".thermal_config.auto_min_max", true);
  measurement_node->declare_parameter(param_namespace + ".thermal_config.use_eeprom_file", false);
  measurement_node->declare_parameter(param_namespace + ".thermal_config.use_calibration_file", false);
  measurement_node->declare_parameter(param_namespace + ".thermal_config.eeprom_file_dir", "");
  measurement_node->declare_parameter(param_namespace + ".thermal_config.calibration_file_dir", "");
  measurement_node->declare_parameter(param_namespace + ".thermal_config.scale_t_min_deg", 15.0);
  measurement_node->declare_parameter(param_namespace + ".thermal_config.scale_t_max_deg", 25.0);
  measurement_node->declare_parameter(param_namespace + ".topology.nr_of_interfaces", 1);

  manager_params.timeout              = std::chrono::milliseconds(measurement_node->get_parameter(param_namespace + ".base_setup.timeout_ms").as_int());
  tf_name                             = measurement_node->get_parameter(param_namespace + ".base_setup.tf_name").as_string();
  manager_params.enable_brs           = measurement_node->get_parameter(param_namespace + ".base_setup.enable_brs").as_bool();
  manager_params.repair_errors        = measurement_node->get_parameter(param_namespace + ".base_setup.repair_errors").as_bool();
  bool enforce_topology               = measurement_node->get_parameter(param_namespace + ".base_setup.enforce_topology").as_bool();
  bool auto_discover                  = measurement_node->get_parameter(param_namespace + ".base_setup.auto_discover").as_bool();
  manager_params.frequency_tof_hz     = measurement_node->get_parameter(param_namespace + ".base_setup.frequency_tof_hz").as_double();
  manager_params.frequency_thermal_hz = measurement_node->get_parameter(param_namespace + ".base_setup.frequency_thermal_hz").as_double();
  bool thermal_auto_min_max           = measurement_node->get_parameter(param_namespace + ".thermal_config.auto_min_max").as_bool();
  bool thermal_use_eeprom_file        = measurement_node->get_parameter(param_namespace + ".thermal_config.use_eeprom_file").as_bool();
  bool thermal_use_calibration_file   = measurement_node->get_parameter(param_namespace + ".thermal_config.use_calibration_file").as_bool();
  std::string thermal_eeprom_dir      = measurement_node->get_parameter(param_namespace + ".thermal_config.eeprom_file_dir").as_string();
  std::string thermal_calibration_dir = measurement_node->get_parameter(param_namespace + ".thermal_config.calibration_file_dir").as_string();
  double thermal_t_min                = measurement_node->get_parameter(param_namespace + ".thermal_config.scale_t_min_deg").as_double();
  double thermal_t_max                = measurement_node->get_parameter(param_namespace + ".thermal_config.scale_t_max_deg").as_double();

  measurement_node->declare_parameter(param_namespace + ".led_config.initial_mode", 0);
  measurement_node->declare_parameter(param_namespace + ".led_config.initial_color", std::vector<int>{ 0, 0, 0 });

  int light_initial_mode_code          = measurement_node->get_parameter(param_namespace + ".led_config.initial_mode").as_int();
  std::vector<long> light_color        = measurement_node->get_parameter(param_namespace + ".led_config.initial_color").as_integer_array();
  device::LightMode light_initial_mode = static_cast<device::LightMode>(light_initial_mode_code);

  if (light_color.size() != 3) {
    throw std::runtime_error("Light color vector has wrong length! Expected 3 values for RGB color.");
  }

  for (const auto& color_value : light_color) {
    if (color_value < 0 || color_value > 255) {
      throw std::runtime_error("Light color values must be in the range [0, 255]!");
    }
  }

  // Configure default thermal sensor parameters
  device::HTPA32_Params htpa32_defaults;
  htpa32_defaults.auto_min_max         = thermal_auto_min_max;
  htpa32_defaults.use_eeprom_file      = thermal_use_eeprom_file;
  htpa32_defaults.use_calibration_file = thermal_use_calibration_file;
  htpa32_defaults.eeprom_dir           = thermal_eeprom_dir;
  htpa32_defaults.calibration_dir      = thermal_calibration_dir;
  htpa32_defaults.t_min_deg_c          = thermal_t_min;
  htpa32_defaults.t_max_deg_c          = thermal_t_max;

  // Create factory with validation mode based on enforce_topology parameter
  ValidationMode validation_mode = enforce_topology ? ValidationMode::Strict : ValidationMode::Relaxed;
  SensorRingFactory factory(validation_mode);
  factory.setDefaultDeviceParams(htpa32_defaults);

  if (auto_discover) {
    // Auto-discover mode: only add interfaces, no board expectations.
    // The factory discovers all connected hardware in relaxed mode.
    // All sensor poses default to identity (zero rotation/translation).
    RCLCPP_INFO(measurement_node->get_logger(), "Auto-discover mode enabled. Discovering hardware on configured interfaces...");

    int nr_of_can_interfaces       = measurement_node->get_parameter(param_namespace + ".topology.nr_of_interfaces").as_int();
    std::string topology_namespace = param_namespace + ".topology.can_interfaces";

    for (int i = 0; i < nr_of_can_interfaces; i++) {
      std::string interface_param_name = ".can_interface_" + std::to_string(i);
      measurement_node->declare_parameter(topology_namespace + interface_param_name + ".interface_type", "undefined");
      measurement_node->declare_parameter(topology_namespace + interface_param_name + ".interface_name", "can0");

      std::string interface_type = measurement_node->get_parameter(topology_namespace + interface_param_name + ".interface_type").as_string();
      std::string interface_name = measurement_node->get_parameter(topology_namespace + interface_param_name + ".interface_name").as_string();

      com::ComInterfaceID com_interface;
      com_interface.name = interface_name;
      if (interface_type == "socketcan") {
        com_interface.type = com::InterfaceType::SocketCan;
      } else if (interface_type == "usbtingo") {
        com_interface.type = com::InterfaceType::UsbTingo;
      } else {
        com_interface.type = com::InterfaceType::Undefined;
      }
      factory.addInterface(com_interface);
    }
  } else {
    // Configured mode: read full topology from parameters
    int nr_of_can_interfaces       = measurement_node->get_parameter(param_namespace + ".topology.nr_of_interfaces").as_int();
    std::string topology_namespace = param_namespace + ".topology.can_interfaces";

    for (int i = 0; i < nr_of_can_interfaces; i++) {

      std::string interface_param_name = ".can_interface_" + std::to_string(i);
      measurement_node->declare_parameter(topology_namespace + interface_param_name + ".interface_type", "undefined");
      measurement_node->declare_parameter(topology_namespace + interface_param_name + ".interface_name", "can0");
      measurement_node->declare_parameter(topology_namespace + interface_param_name + ".orientation", "none");
      measurement_node->declare_parameter(topology_namespace + interface_param_name + ".nr_of_sensors", 1);

      int nr_of_sensors           = measurement_node->get_parameter(topology_namespace + interface_param_name + ".nr_of_sensors").as_int();
      std::string orientation_str = measurement_node->get_parameter(topology_namespace + interface_param_name + ".orientation").as_string();
      std::string interface_type  = measurement_node->get_parameter(topology_namespace + interface_param_name + ".interface_type").as_string();
      std::string interface_name  = measurement_node->get_parameter(topology_namespace + interface_param_name + ".interface_name").as_string();

      // Add interface to factory
      com::ComInterfaceID com_interface;
      com_interface.name = interface_name;
      if (interface_type == "socketcan") {
        com_interface.type = com::InterfaceType::SocketCan;
      } else if (interface_type == "usbtingo") {
        com_interface.type = com::InterfaceType::UsbTingo;
      } else {
        com_interface.type = com::InterfaceType::Undefined;
      }
      factory.addInterface(com_interface);

      // Determine orientation for thermal sensors on this bus
      device::Orientation orientation = device::Orientation::None;
      if (orientation_str == "left")
        orientation = device::Orientation::Left;
      if (orientation_str == "right")
        orientation = device::Orientation::Right;

      // Get parameters for every sensor board on the current interface
      std::string sensors_namespace = topology_namespace + interface_param_name + ".sensors";
      for (int j = 0; j < nr_of_sensors; j++) {

        std::string sensor_param_name = ".sensor_" + std::to_string(j);
        measurement_node->declare_parameter(sensors_namespace + sensor_param_name + ".rotation", std::vector<double>{ 0.0, 0.0, 0.0 });
        measurement_node->declare_parameter(sensors_namespace + sensor_param_name + ".translation", std::vector<double>{ 0.0, 0.0, 0.0 });

        std::vector<double> rotation    = measurement_node->get_parameter(sensors_namespace + sensor_param_name + ".rotation").as_double_array();
        std::vector<double> translation = measurement_node->get_parameter(sensors_namespace + sensor_param_name + ".translation").as_double_array();

        board::SensorBoardParams board_params;

        if (rotation.size() == 3) {
          board_params.rotation = { rotation[0], rotation[1], rotation[2] };
        } else {
          throw std::invalid_argument("Rotation vector of sensor " + std::to_string(j) + " on interface " + interface_name + " has wrong length!");
        }

        if (translation.size() == 3) {
          board_params.translation = { translation[0], translation[1], translation[2] };
        } else {
          throw std::invalid_argument("Translation vector of sensor " + std::to_string(j) + " on interface " + interface_name + " has wrong length!");
        }

        // Set thermal orientation for this board's thermal sensor
        device::HTPA32_Params thermal_params = htpa32_defaults;
        thermal_params.orientation           = orientation;

        factory.expectBoard(board_params, { thermal_params });
      }
    }
  }

  // Build the MeasurementManager using the factory
  auto manager = std::make_unique<manager::MeasurementManager>(manager_params, factory);

  // Set initial light mode on all lights
  for (auto& light : manager->lights()) {
    light.setLight(light_initial_mode, light_color[0], light_color[1], light_color[2]);
  }

  bool success = measurement_node->run(std::move(manager), tf_name);
  rclcpp::shutdown();

  return success ? 0 : 1;
}
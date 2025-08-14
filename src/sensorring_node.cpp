#include <memory>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "SensorRingProxy.hpp"

using namespace eduart;

int main (int argc, char* argv[]){
	
	rclcpp::init(argc, argv);
	
	std::string tf_name;
	ring::RingParams ring_params;
	manager::ManagerParams manager_params;

	// Create SensorRing Node
	auto measurement_node = std::make_shared<sensorring::SensorRingProxy>("edu_sensorring_ros2");
	RCLCPP_INFO(measurement_node->get_logger(), "Starting the sensorring node");

	// Get SensorRing parameters
	std::string param_namespace = "pointcloud_sensor";
	measurement_node->declare_parameter(param_namespace + ".base_setup.timeout_ms", 1000);
	measurement_node->declare_parameter(param_namespace + ".base_setup.tf_name", "base_sensorring");
	measurement_node->declare_parameter(param_namespace + ".base_setup.print_topology", true);
	measurement_node->declare_parameter(param_namespace + ".base_setup.frequency_tof_hz", 5.0);
	measurement_node->declare_parameter(param_namespace + ".base_setup.frequency_thermal_hz", 1.0);
	measurement_node->declare_parameter(param_namespace + ".thermal_config.auto_min_max", true);
	measurement_node->declare_parameter(param_namespace + ".thermal_config.use_eeprom_file", false);
	measurement_node->declare_parameter(param_namespace + ".thermal_config.use_calibration_file", false);
	measurement_node->declare_parameter(param_namespace + ".thermal_config.eeprom_file_dir", "");
	measurement_node->declare_parameter(param_namespace + ".thermal_config.calibration_file_dir", "");
	measurement_node->declare_parameter(param_namespace + ".thermal_config.scale_t_min_deg", 15.0);
	measurement_node->declare_parameter(param_namespace + ".thermal_config.scale_t_max_deg", 25.0);
	measurement_node->declare_parameter(param_namespace + ".topology.nr_of_interfaces", 1);

	ring_params.timeout					= std::chrono::milliseconds(measurement_node->get_parameter(param_namespace + ".base_setup.timeout_ms").as_int());
	tf_name								= measurement_node->get_parameter(param_namespace + ".base_setup.tf_name").as_string();
	manager_params.print_topology	    = measurement_node->get_parameter(param_namespace + ".base_setup.print_topology").as_bool();
	manager_params.frequency_tof_hz     = measurement_node->get_parameter(param_namespace + ".base_setup.frequency_tof_hz").as_double();
	manager_params.frequency_thermal_hz = measurement_node->get_parameter(param_namespace + ".base_setup.frequency_thermal_hz").as_double();
	bool thermal_auto_min_max			= measurement_node->get_parameter(param_namespace + ".thermal_config.auto_min_max").as_bool();
	bool thermal_use_eeprom_file		= measurement_node->get_parameter(param_namespace + ".thermal_config.use_eeprom_file").as_bool();
	bool thermal_use_calibration_file	= measurement_node->get_parameter(param_namespace + ".thermal_config.use_calibration_file").as_bool();
	std::string thermal_eeprom_dir		= measurement_node->get_parameter(param_namespace + ".thermal_config.eeprom_file_dir").as_string();
	std::string thermal_calibration_dir = measurement_node->get_parameter(param_namespace + ".thermal_config.calibration_file_dir").as_string();
	double thermal_t_min				= measurement_node->get_parameter(param_namespace + ".thermal_config.scale_t_min_deg").as_double();
	double thermal_t_max				= measurement_node->get_parameter(param_namespace + ".thermal_config.scale_t_max_deg").as_double();
	int nr_of_can_interfaces            = measurement_node->get_parameter(param_namespace + ".topology.nr_of_interfaces").as_int();

	measurement_node->declare_parameter(param_namespace + ".led_config.initial_mode", 0);
	measurement_node->declare_parameter(param_namespace + ".led_config.initial_color", std::vector<int>{0, 0, 0});

	int light_initial_mode_code			= measurement_node->get_parameter(param_namespace + ".led_config.initial_mode").as_int();
	std::vector<long> light_color		= measurement_node->get_parameter(param_namespace + ".led_config.initial_color").as_integer_array();
	light::LightMode light_initial_mode = static_cast<light::LightMode>(light_initial_mode_code);
	
	if(light_color.size() != 3){
		throw std::runtime_error("Light color vector has wrong length! Expected 3 values for RGB color.");
	}

	for(const auto& color_value : light_color){
		if(color_value < 0 || color_value > 255){
			throw std::runtime_error("Light color values must be in the range [0, 255]!");
		}
	}
	
	// Get parameters for every can interface
	param_namespace += ".topology.can_interfaces";
	int sensor_idx = 0;
	for(int i=0; i<nr_of_can_interfaces; i++){

		bus::BusParams bus_params;

		// Get SensorBus parameters
		std::string interface_param_name = ".can_interface_" + std::to_string(i);
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".interface_type", "undefined");
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".interface_name", "can0");
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".orientation",    "none");
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".nr_of_sensors",   1);

		int nr_of_sensors           = measurement_node->get_parameter(param_namespace + interface_param_name + ".nr_of_sensors").as_int();
		std::string orientation_str = measurement_node->get_parameter(param_namespace + interface_param_name + ".orientation").as_string();
		std::string interface_type	= measurement_node->get_parameter(param_namespace + interface_param_name + ".interface_type").as_string();
		bus_params.interface_name	= measurement_node->get_parameter(param_namespace + interface_param_name + ".interface_name").as_string();
		
		if(interface_type == "socketcan"){
			bus_params.type = com::DeviceType::SOCKETCAN;
		}else if(interface_type == "usbtingo"){
			bus_params.type = com::DeviceType::USBTINGO;
		}else{
			bus_params.type = com::DeviceType::UNDEFINED;
		}

		sensor::Orientation orientation		= sensor::Orientation::none;
		if (orientation_str == "left")  orientation	= sensor::Orientation::left;
		if (orientation_str == "right") orientation	= sensor::Orientation::right;

		// Get parameters for every sensor on the current can interface
		interface_param_name += ".sensors";
		for(int j=0; j<nr_of_sensors; j++){
			
			// Get parameters for the current sensor board
			std::string sensor_param_name = ".sensor_" + std::to_string(j);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".enable_tof", true);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".enable_thermal", false);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".enable_light", false);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".rotation", std::vector<double>{0.0,0.0,0.0});
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".translation", std::vector<double>{0.0,0.0,0.0});

			bool enable_tof		= measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".enable_tof").as_bool();
			bool enable_thermal	= measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".enable_thermal").as_bool();
			bool enable_light	= measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".enable_light").as_bool();

			std::vector<double> rotation    = measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".rotation").as_double_array();
			std::vector<double> translation = measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".translation").as_double_array();

			sensor::TofSensorParams tof_params;
			tof_params.enable		= enable_tof;
			tof_params.user_idx		= sensor_idx;

			if(rotation.size() == 3){
				std::copy_n(rotation.begin(), 3, tof_params.rotation.data.begin());
			}else{
				throw std::invalid_argument("Rotation vector of sensor " + std::to_string(j) + " on interface " + bus_params.interface_name + " has wrong length!");
			}

			if(translation.size() == 3){
				std::copy_n(translation.begin(), 3, tof_params.translation.data.begin());
			}else{
				throw std::invalid_argument("Translation vector of sensor " + std::to_string(j) + " on interface " + bus_params.interface_name + " has wrong length!");
			}

			sensor::ThermalSensorParams thermal_params;
			thermal_params.enable				= enable_thermal;
			thermal_params.user_idx				= sensor_idx;
			thermal_params.rotation     		= tof_params.rotation; // ToDo: The pose of the thermal sensor is not the pose of the tof sensor. Need to add an offset.
			thermal_params.translation  		= tof_params.translation;
			thermal_params.orientation			= orientation;
			thermal_params.auto_min_max 		= thermal_auto_min_max;
			thermal_params.use_eeprom_file		= thermal_use_eeprom_file;
			thermal_params.use_calibration_file = thermal_use_calibration_file;
			thermal_params.eeprom_dir			= thermal_eeprom_dir;
			thermal_params.calibration_dir		= thermal_calibration_dir;
			thermal_params.t_min_deg_c			= thermal_t_min;
			thermal_params.t_max_deg_c			= thermal_t_max;

			sensor::LightParams led_params;
			led_params.enable = enable_light;
			led_params.orientation = orientation;
					
			// Create sensor board
			sensor::SensorBoardParams board_params;
			board_params.tof_params     = tof_params;
			board_params.thermal_params = thermal_params;
			board_params.led_params     = led_params;

			bus_params.board_param_vec.push_back(board_params);
			sensor_idx++;
		}
		ring_params.bus_param_vec.push_back(bus_params);
	}
	manager_params.ring_params = ring_params;

	// Run ros node
	bool success = measurement_node->run(manager_params, tf_name, light_initial_mode, light_color[0], light_color[1], light_color[2]);
	rclcpp::shutdown();
	
	return success ? 0 : 1;
}
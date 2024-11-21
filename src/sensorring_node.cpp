#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "SensorRingProxy.hpp"


int main (int argc, char* argv[]){

	// Init ROS
	rclcpp::init(argc, argv);
	RCLCPP_INFO(rclcpp::get_logger("sensor_ring"), "Starting the sensor ring node");

	// Create SensorRing Node
	sensorring::SensorRingParams ring_params;
	auto measurement_node = std::make_shared<sensorring::SensorRingProxy>("sensor_ring");

	// Get SensorRing parameters
	measurementmanager::MeasurementManagerParams manager_params;
	std::string param_namespace = "point_cloud_sensor";
	measurement_node->declare_parameter(param_namespace + ".base_setup.tf_name", "base_sensor_ring");
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

	std::string tf_name					= measurement_node->get_parameter(param_namespace + ".base_setup.tf_name").as_string();
	manager_params.print_topology       = measurement_node->get_parameter(param_namespace + ".base_setup.print_topology").as_bool();
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

	// Get parameters for every can interface
	param_namespace += ".topology.can_interfaces";
	for(int i=0; i<nr_of_can_interfaces; i++){

		sensorbus::SensorBusParams bus_params;

		// Get SensorBus parameters
		std::string interface_param_name = ".can_interface_" + std::to_string(i);
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".interface_name", "can0");
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".orientation",    "none");
		measurement_node->declare_parameter(param_namespace + interface_param_name + ".nr_of_sensors",   1);

		int nr_of_sensors           = measurement_node->get_parameter(param_namespace + interface_param_name + ".nr_of_sensors").as_int();
		std::string orientation_str = measurement_node->get_parameter(param_namespace + interface_param_name + ".orientation").as_string();
		bus_params.interface_name	= measurement_node->get_parameter(param_namespace + interface_param_name + ".interface_name").as_string();

		sensor::SensorOrientation orientation		= sensor::SensorOrientation::none;
		if (orientation_str == "left")  orientation = sensor::SensorOrientation::left;
		if (orientation_str == "right") orientation = sensor::SensorOrientation::right;

		// Get parameters for every sensor on the current can interface
		interface_param_name += ".sensors";
		for(int j=0; j<nr_of_sensors; j++){
			
			// Get parameters for the current sensor board
			std::string sensor_param_name = ".sensor_" + std::to_string(j);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".get_tof", true);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".get_thermal", false);
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".rotation", std::vector<double>{0.0,0.0,0.0});
			measurement_node->declare_parameter(param_namespace + interface_param_name + sensor_param_name + ".translation", std::vector<double>{0.0,0.0,0.0});

			bool enable_tof          = measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".get_tof").as_bool();
			bool enable_thermal      = measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".get_thermal").as_bool();

			sensor::TofSensorParams tof_params;
			tof_params.idx                  = j;
			std::vector<double> rotation    = measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".rotation").as_double_array();
			std::vector<double> translation = measurement_node->get_parameter(param_namespace + interface_param_name + sensor_param_name + ".translation").as_double_array();

			if(rotation.size() == 3){
				std::copy_n(rotation.begin(), 3, tof_params.rotation.data.begin());
			}else{
				RCLCPP_ERROR_STREAM(rclcpp::get_logger("sensor_ring"), "Rotation vector of sensor " << j << " on interface " << bus_params.interface_name << " has wrong length!");
			}

			if(translation.size() == 3){
				std::copy_n(translation.begin(), 3, tof_params.translation.data.begin());
			}else{
				RCLCPP_ERROR_STREAM(rclcpp::get_logger("sensor_ring"), "Translation vector of sensor " << j << " on interface " << bus_params.interface_name << " has wrong length!");
			}
			
			sensor::ThermalSensorParams thermal_params;
			thermal_params.idx          		= j;
			thermal_params.rotation     		= tof_params.rotation; // ToDo: The pose of the thermal sensor is not the pose of the tof sensor. Need to add an offset.
			thermal_params.translation  		= tof_params.translation;
			thermal_params.orientation			= orientation;
			thermal_params.auto_min_max 		= thermal_auto_min_max;
			thermal_params.use_eeprom_file		= thermal_use_eeprom_file;
			thermal_params.use_calibration_file = thermal_use_calibration_file;
			thermal_params.eeprom_dir			= thermal_eeprom_dir;
			thermal_params.calibration_dir		= thermal_calibration_dir;
			thermal_params.t_min				= thermal_t_min;
			thermal_params.t_max				= thermal_t_max;

			sensor::LedLightParams led_params;
			led_params.orientation = orientation;
					
			// Create sensor board
			sensor::SensorBoardParams board_params;
			board_params.idx			= j;
			board_params.enable_tof     = enable_tof;
			board_params.enable_thermal = enable_thermal;
			board_params.tof_params     = tof_params;
			board_params.thermal_params = thermal_params;
			board_params.led_params     = led_params;

			bus_params.board_param_vec.push_back(board_params);
		}

		ring_params.bus_param_vec.push_back(bus_params);
	}

	// Run ros node
	manager_params.ring_params = ring_params;
	int success = measurement_node->run(manager_params);
	
	if(success == 0){
		while(rclcpp::ok() && !measurement_node->isShutdown()){
			rclcpp::spin_some(measurement_node);
		}
		rclcpp::shutdown();
	}
	
	return success;
}
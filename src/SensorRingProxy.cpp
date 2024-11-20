#include "SensorRingProxy.hpp"

#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <algorithm>

namespace sensorring{

SensorRingProxy::SensorRingProxy(std::string node_name) :
Node(node_name), TofObserver(), ThermalObserver(), LogObserver() {

};
    
SensorRingProxy::~SensorRingProxy(){

};

int SensorRingProxy::run(measurementmanager::MeasurementManagerParams params){
	_params = params;
	_manager = std::make_unique<measurementmanager::MeasurementManager>(_params);

	// set up services
	auto stop_cali_srv	= this->create_service<sensorring_ros2::srv::StopThermalCalibration>(std::string(this->get_name()) + "/stopThermalCalibration", std::bind(&SensorRingProxy::stopThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));
	auto start_cali_srv	= this->create_service<sensorring_ros2::srv::StartThermalCalibration>(std::string(this->get_name()) + "/startThermalCalibration", std::bind(&SensorRingProxy::startThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));

	return 1;
};

void SensorRingProxy::onTofMeasurement(const measurement::TofSensorMeasurement measurement){

};

void SensorRingProxy::onThermalMeasurement(const measurement::ThermalSensorMeasurement measurement){

};

void SensorRingProxy::onOutputLog(const LogVerbosity verbosity, const std::string msg){

};

void SensorRingProxy::stopThermalCalibration(	const std::shared_ptr<sensorring_ros2::srv::StopThermalCalibration::Request> request,
												std::shared_ptr<sensorring_ros2::srv::StopThermalCalibration::Response> response){
	if(request->stop){
		response->output = _manager->stopThermalCalibration();
	}else{
		response->output = false;
	}
};

void SensorRingProxy::startThermalCalibration(	const std::shared_ptr<sensorring_ros2::srv::StartThermalCalibration::Request> request,
                                				std::shared_ptr<sensorring_ros2::srv::StartThermalCalibration::Response> response){
	response->output = _manager->startThermalCalibration((std::size_t)request->window);
};

}; // namespace sensorring
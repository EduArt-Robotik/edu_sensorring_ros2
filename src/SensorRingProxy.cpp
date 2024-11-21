#include "SensorRingProxy.hpp"

#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <algorithm>

namespace sensorring{

SensorRingProxy::SensorRingProxy(std::string node_name) :
Node(node_name), MeasurementObserver(), _shutdown(false) {

};
    
SensorRingProxy::~SensorRingProxy(){
	_manager->stopMeasuring();
};

int SensorRingProxy::run(measurementmanager::MeasurementManagerParams params){
	_params = params;
	_manager = std::make_unique<measurementmanager::MeasurementManager>(_params);

	// set up ros services
	auto stop_cali_srv	= this->create_service<sensorring_ros2::srv::StopThermalCalibration>(std::string(this->get_name()) + "/stopThermalCalibration", std::bind(&SensorRingProxy::stopThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));
	auto start_cali_srv	= this->create_service<sensorring_ros2::srv::StartThermalCalibration>(std::string(this->get_name()) + "/startThermalCalibration", std::bind(&SensorRingProxy::startThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));

	// set up ros publisher

	// register this observer at the MeasurementManager
	_manager->registerObserver(this);

	// force first state update
	onStateChange(_manager->getWorkerState());

	return _manager->startMeasuring();
};

void SensorRingProxy::onStateChange(const WorkerState state){
	switch(state){
		case WorkerState::Initialized:
			RCLCPP_DEBUG(this->get_logger(), "New MeasurementManager state: initialized");
			break;
		case WorkerState::Running:
			RCLCPP_DEBUG(this->get_logger(), "New MeasurementManager state: running");
			break;
		case WorkerState::Shutdown:
			RCLCPP_INFO(this->get_logger(), "New MeasurementManager state: shutdown");
			_shutdown = true;
			break;
		case WorkerState::Error:
			RCLCPP_INFO(this->get_logger(), "New MeasurementManager state: error");
			_shutdown = true;
			break;
	}
};

bool SensorRingProxy::isShutdown(){
	return _shutdown;
}

void SensorRingProxy::onTofMeasurement(const measurement::TofSensorMeasurement measurement){

};

void SensorRingProxy::onThermalMeasurement(const measurement::ThermalSensorMeasurement measurement){

};

void SensorRingProxy::onOutputLog(const LogVerbosity verbosity, const std::string msg){
	switch(verbosity){
		case LogVerbosity::Debug:
			RCLCPP_DEBUG(this->get_logger(), msg.c_str());
			break;
		case LogVerbosity::Info:
			RCLCPP_INFO(this->get_logger(), msg.c_str());
			break;
		case LogVerbosity::Warning:
			RCLCPP_WARN(this->get_logger(), msg.c_str());
			break;
		case LogVerbosity::Error:
			RCLCPP_ERROR(this->get_logger(), msg.c_str());
			break;
	}
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
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
	_manager = std::make_unique<measurementmanager::MeasurementManager>(params, static_cast<MeasurementObserver*>(this));

	// prepare pointCloud2 message
	_pc2_msg = sensor_msgs::msg::PointCloud2();
	_pc2_msg.header.frame_id = _manager->getParams().ring_params.tf_name;
	_pc2_msg.height        = 1;
	_pc2_msg.point_step    = 4 * 4; // Dimensions per zone * bytes per dimension (float32 -> 4 bytes) // ToDo: remove magic number for float/double size
	_pc2_msg.is_bigendian  = true;
	_pc2_msg.is_dense      = true;

	sensor_msgs::msg::PointField field_x;
	field_x.name = "x";
	field_x.offset = 0;
	field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_x.count = 1;

	sensor_msgs::msg::PointField field_y;
	field_y.name = "y";
	field_y.offset = 4;
	field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_y.count = 1;

	sensor_msgs::msg::PointField field_z;
	field_z.name = "z";
	field_z.offset = 8;
	field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_z.count = 1;

	sensor_msgs::msg::PointField field_sigma;
	field_sigma.name = "sigma";
	field_sigma.offset = 12;
	field_sigma.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_sigma.count = 1;

	_pc2_msg.fields = {field_x, field_y, field_z, field_sigma};

	// create pointCloud2 publisher
	_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_raw", 1);

	// prepare one image publisher for each active thermal sensor
	int i = 0;
	for(auto sensor_bus : _manager->getParams().ring_params.bus_param_vec){
		for(auto sensor_board : sensor_bus.board_param_vec){
			if(sensor_board.enable_thermal){

				// =================== Temperature image ===================
				std::string sensor_name = "thermal_sensor_" + std::to_string(i) + "/grayscale";

				// ToDo: Use BoardManager for these infos
				// prepare image message
				auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
				img_msg->header.frame_id = sensor_name;
				img_msg->height			= 32;
				img_msg->width			= 32;
				img_msg->encoding		= "mono8";
				img_msg->is_bigendian	= false;
				img_msg->step			= img_msg->width * 1; // ToDo: remove magic number for uint8_t/uint16_t size

				auto img_pub = this->create_publisher<sensor_msgs::msg::Image>("/sensors/" + sensor_name, 1);

				_img_msg_vec.push_back(img_msg);
				_img_pub_vec.push_back(img_pub);

				// =================== False color image ===================
				sensor_name = "thermal_sensor_" + std::to_string(i) + "/falsecolor";

				// ToDo: Use BoardManager for these infos
				// prepare image message
				auto colorimg_msg = std::make_shared<sensor_msgs::msg::Image>();
				colorimg_msg->header.frame_id = sensor_name;
				colorimg_msg->height	    = 32;
				colorimg_msg->width			= 32;
				colorimg_msg->encoding		= "rgb8";
				colorimg_msg->is_bigendian	= false;
				colorimg_msg->step			= img_msg->width * 3; // ToDo: remove magic number for uint8_t/uint16_t size

				auto colorimg_pub = this->create_publisher<sensor_msgs::msg::Image>("/sensors/" + sensor_name, 1);

				_colorimg_msg_vec.push_back(colorimg_msg);
				_colorimg_pub_vec.push_back(colorimg_pub);
			}
			i++;
		}
	}

	// send static transforms for each sensor (mainly a visual aid for visualization)
	std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> tf_braodcasters;
	
	i = 0;
	for(auto sensor_bus : _manager->getParams().ring_params.bus_param_vec){
		for (auto sensor_board : sensor_bus.board_param_vec){
			auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
			tf_braodcasters.push_back(tf_broadcaster);

			geometry_msgs::msg::TransformStamped t;
			t.header.stamp = this->now();
			t.header.frame_id = _manager->getParams().ring_params.tf_name;
			t.child_frame_id = "sensor_" + std::to_string(i);

			t.transform.translation.x = sensor_board.tof_params.translation.x();
			t.transform.translation.y = sensor_board.tof_params.translation.y();
			t.transform.translation.z = sensor_board.tof_params.translation.z();
			tf2::Quaternion q;
			q.setRPY(
				sensor_board.tof_params.rotation.x() * M_PI / 180.0,
				sensor_board.tof_params.rotation.y() * M_PI / 180.0,
				sensor_board.tof_params.rotation.z() * M_PI / 180.0);
			t.transform.rotation.x = q.x();
			t.transform.rotation.y = q.y();
			t.transform.rotation.z = q.z();
			t.transform.rotation.w = q.w();

			tf_broadcaster->sendTransform(t);
			i++;
		}
	}

	// set up ros services
	auto stop_cali_srv	= this->create_service<sensorring_ros2::srv::StopThermalCalibration>(std::string(this->get_name()) + "/stopThermalCalibration", std::bind(&SensorRingProxy::stopThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));
	auto start_cali_srv	= this->create_service<sensorring_ros2::srv::StartThermalCalibration>(std::string(this->get_name()) + "/startThermalCalibration", std::bind(&SensorRingProxy::startThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));

	// force first state update
	onStateChange(_manager->getWorkerState());

	int error = _manager->startMeasuring();

	if(error == 0){
		while(!_shutdown && rclcpp::ok()){
			rclcpp::spin_some(shared_from_this());
		}

		error = _manager->stopMeasuring();
	}

	return error;
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
	if(measurement.size > 0){

		_pc2_msg.header.stamp  = this->now();
		_pc2_msg.width         = measurement.size;
		_pc2_msg.row_step      = _pc2_msg.width * _pc2_msg.point_step;
		_pc2_msg.data.resize(_pc2_msg.row_step * _pc2_msg.height);

		float* data_ptr = reinterpret_cast<float*>(_pc2_msg.data.data());

		// Populate the point cloud with the measurement data
		for (size_t i = 0; i < _pc2_msg.width; i++) {
				data_ptr[i * 4 + 0] = static_cast<float>(measurement.point_data_transformed[i].x()); // x
				data_ptr[i * 4 + 1] = static_cast<float>(measurement.point_data_transformed[i].y()); // y
				data_ptr[i * 4 + 2] = static_cast<float>(measurement.point_data_transformed[i].z()); // z
				data_ptr[i * 4 + 3] = static_cast<float>(measurement.point_sigma[i]);  				 // sigma
		}

		_pointcloud_pub->publish(_pc2_msg);
	}
};

void SensorRingProxy::onThermalMeasurement(const std::size_t idx, const measurement::ThermalSensorMeasurement measurement){
	// prepare and publish grayscale image
	const std::uint8_t* temp_data_ptr  = measurement.grayscale_img.data.begin();
	auto img_msg = _img_msg_vec.at(idx);
	size_t size = img_msg->width * img_msg->height * sizeof(decltype(img_msg->data)::value_type);
	img_msg->header.stamp = this->now();
	img_msg->data.resize(size);
	std::copy_n(temp_data_ptr, size, img_msg->data.data());					
	_img_pub_vec[idx]->publish(*img_msg);

	// prepare and publish false color image
	const std::uint8_t* color_data_ptr = measurement.falsecolor_img.data.begin()->begin();
	auto colorimg_msg = _colorimg_msg_vec.at(idx);
	size *= 3; // ToDO: remove magic number
	colorimg_msg->header.stamp = this->now();
	colorimg_msg->data.resize(size);
	std::copy_n(color_data_ptr, size, colorimg_msg->data.data());
	_colorimg_pub_vec[idx]->publish(*colorimg_msg);
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
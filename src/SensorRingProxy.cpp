#include "SensorRingProxy.hpp"

#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <algorithm>

namespace eduart{

namespace sensorring{

SensorRingProxy::SensorRingProxy(std::string node_name) :
Node(node_name), MeasurementObserver(), _shutdown(false) {

};
    
SensorRingProxy::~SensorRingProxy(){
	if(_manager)
		_manager->unregisterObserver(this);
}

bool SensorRingProxy::run(manager::ManagerParams params, std::string tf_name, light::LightMode initial_light_mode, std::uint8_t red, std::uint8_t green, std::uint8_t blue){

	// create MeasurementManager
	_manager = std::make_unique<manager::MeasurementManager>(params, static_cast<MeasurementObserver*>(this));
	_manager->setLight(initial_light_mode, red, green, blue);

	// prepare pointCloud2 message
	sensor_msgs::msg::PointCloud2 pc2_msg;
	pc2_msg.header.frame_id = tf_name;
	pc2_msg.height        = 1;
	pc2_msg.point_step    = sizeof(measurement::PointData);
	pc2_msg.is_bigendian  = true;
	pc2_msg.is_dense      = true;

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
	field_sigma.offset = 16;
	field_sigma.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_sigma.count = 1;

	sensor_msgs::msg::PointField field_dist;
	field_dist.name = "raw_distance";
	field_dist.offset = 12;
	field_dist.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_dist.count = 1;

	sensor_msgs::msg::PointField field_idx;
	field_idx.name = "sensor_idx";
	field_idx.offset = 20;
	field_idx.datatype = sensor_msgs::msg::PointField::INT32;
	field_idx.count = 1;

	pc2_msg.fields = {field_x, field_y, field_z, field_sigma, field_dist, field_idx};

	_pc2_msg_raw = pc2_msg;
	_pc2_msg_transformed = pc2_msg;
	
	// create pointCloud2 publisher
	_pointcloud_pub_raw			= this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_raw", 1);
	_pointcloud_pub_transformed	= this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_transformed", 1);

	// prepare one image publisher for each active thermal sensor
	int i = 0;
	for(auto sensor_bus : _manager->getParams().ring_params.bus_param_vec){
		for(auto sensor_board : sensor_bus.board_param_vec){
			if(sensor_board.thermal_params.enable){

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

	// send static transforms for each sensor
	i = 0;
	std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> tf_braodcasters;
	for(auto sensor_bus : _manager->getParams().ring_params.bus_param_vec){

		for (auto sensor_board : sensor_bus.board_param_vec){
			auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
			tf_braodcasters.push_back(tf_broadcaster);

			geometry_msgs::msg::TransformStamped t;
			t.header.stamp = this->now();
			t.header.frame_id = tf_name;
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

			// prepare publisher for individual sensor pointclouds
			pc2_msg.header.frame_id = t.child_frame_id;
			_pc2_msg_individual_vec.push_back(pc2_msg);
			_pointcloud_pub_individual_vec.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/tof_sensors/pcl_individual/sensor_" + std::to_string(i), 1));

			i++;
		}
	}

	// set up ros services
	auto stop_cali_srv	= this->create_service<edu_sensorring_ros2::srv::StopThermalCalibration>(std::string(this->get_name()) + "/stopThermalCalibration", std::bind(&SensorRingProxy::stopThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));
	auto start_cali_srv	= this->create_service<edu_sensorring_ros2::srv::StartThermalCalibration>(std::string(this->get_name()) + "/startThermalCalibration", std::bind(&SensorRingProxy::startThermalCalibration, this, std::placeholders::_1, std::placeholders::_2));

	// force first state update
	onStateChange(_manager->getWorkerState());

	bool success = _manager->startMeasuring();

	if(success){
		while(!_shutdown && rclcpp::ok()){
			rclcpp::spin_some(shared_from_this());
		}

		success = _manager->stopMeasuring();
	}

	return static_cast<int>(success);
};

void SensorRingProxy::onStateChange(const manager::WorkerState state){
	switch(state){
		case manager::WorkerState::Initialized:
			RCLCPP_DEBUG(this->get_logger(), "New MeasurementManager state: initialized");
			break;
		case manager::WorkerState::Running:
			RCLCPP_DEBUG(this->get_logger(), "New MeasurementManager state: running");
			break;
		case manager::WorkerState::Shutdown:
			RCLCPP_INFO(this->get_logger(), "New MeasurementManager state: shutdown");
			_shutdown = true;
			break;
		case manager::WorkerState::Error:
			RCLCPP_ERROR(this->get_logger(), "New MeasurementManager state: error");
			_shutdown = true;
			break;
	}
};

bool SensorRingProxy::isShutdown(){
	return _shutdown;
}

void SensorRingProxy::onRawTofMeasurement(std::vector<measurement::TofMeasurement> measurement_vec){
	if(!measurement_vec.empty()){

		int idx = 0;
		auto now = this->now();

		std::size_t point_count = 0;
		for(const auto& measurement : measurement_vec){
			point_count += measurement.point_cloud.size();

			// prepare individual pc2 messages
			auto& msg = _pc2_msg_individual_vec.at(idx);
			msg.header.stamp	= now;
			msg.width			= measurement.point_cloud.size();
			msg.row_step		= msg.width * msg.point_step;
			msg.data.resize(msg.row_step);
			idx++;
		}

		// prepare combined pc2 message
		_pc2_msg_raw.header.stamp  = now;
		_pc2_msg_raw.width         = point_count;
		_pc2_msg_raw.row_step      = _pc2_msg_raw.width * _pc2_msg_raw.point_step;
		_pc2_msg_raw.data.resize(_pc2_msg_raw.row_step);

		auto data_ptr = reinterpret_cast<measurement::PointData*>(_pc2_msg_raw.data.data());

		idx = 0;
		for(const auto& measurement : measurement_vec){
			// prepare combined measurement
			std::copy(measurement.point_cloud.begin(), measurement.point_cloud.end(), data_ptr);
			data_ptr += measurement.point_cloud.size();

			// prepare and publish individual measurement
			auto& msg = _pc2_msg_individual_vec.at(idx);
			auto data_ptr_individual = reinterpret_cast<measurement::PointData*>(msg.data.data());
			std::copy(measurement.point_cloud.begin(), measurement.point_cloud.end(), data_ptr_individual);
			_pointcloud_pub_individual_vec.at(idx)->publish(msg);
			idx++;
		}

		// publish combined measurement
		_pointcloud_pub_raw->publish(_pc2_msg_raw);
	}
};

void SensorRingProxy::onTransformedTofMeasurement(std::vector<measurement::TofMeasurement> measurement_vec){
	if(!measurement_vec.empty()){

		std::size_t point_count = 0;
		for(const auto& measurement : measurement_vec){
			point_count += measurement.point_cloud.size();
		}

		_pc2_msg_transformed.header.stamp  = this->now();
		_pc2_msg_transformed.width         = point_count;
		_pc2_msg_transformed.row_step      = _pc2_msg_transformed.width * _pc2_msg_transformed.point_step;
		_pc2_msg_transformed.data.resize(_pc2_msg_transformed.row_step);

		auto data_ptr = reinterpret_cast<measurement::PointData*>(_pc2_msg_transformed.data.data());

		for(const auto& measurement : measurement_vec){
			std::copy(measurement.point_cloud.begin(), measurement.point_cloud.end(), data_ptr);
			data_ptr += measurement.point_cloud.size();
		}

		_pointcloud_pub_transformed->publish(_pc2_msg_transformed);
	}
};

void SensorRingProxy::onThermalMeasurement(std::vector<measurement::ThermalMeasurement> measurement_vec){
	int idx = 0;
	for(const auto& measurement : measurement_vec){
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

		idx++;
	}
};

void SensorRingProxy::onOutputLog(const logger::LogVerbosity verbosity, const std::string msg){
	switch(verbosity){
		case logger::LogVerbosity::Debug:
			RCLCPP_DEBUG(this->get_logger(), msg.c_str());
			break;
		case logger::LogVerbosity::Info:
			RCLCPP_INFO(this->get_logger(), msg.c_str());
			break;
		case logger::LogVerbosity::Warning:
			RCLCPP_WARN(this->get_logger(), msg.c_str());
			break;
		case logger::LogVerbosity::Error:
			RCLCPP_ERROR(this->get_logger(), msg.c_str());
			break;
	}
};

void SensorRingProxy::stopThermalCalibration(	const std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Request> request,
												std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Response> response){
	if(request->stop){
		response->output = _manager->stopThermalCalibration();
	}else{
		response->output = false;
	}
};

void SensorRingProxy::startThermalCalibration(	const std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Request> request,
                                				std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Response> response){
	response->output = _manager->startThermalCalibration((std::size_t)request->window);
};

};

};
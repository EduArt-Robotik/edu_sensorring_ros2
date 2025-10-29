


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "edu_sensorring_ros2/srv/start_thermal_calibration.hpp"
#include "edu_sensorring_ros2/srv/stop_thermal_calibration.hpp"

#include <sensorring/Logger.hpp>
#include <sensorring/LoggerClient.hpp>
#include <sensorring/MeasurementManager.hpp>

#include <vector>
#include <memory>
#include <string>

namespace eduart{

namespace sensorring{

    class SensorRingProxy : public rclcpp::Node, manager::MeasurementClient, logger::LoggerClient{
    public:
        SensorRingProxy(std::string node_name);

        ~SensorRingProxy();

        bool run(std::unique_ptr<manager::MeasurementManager> manager, std::string tf_name, light::LightMode initial_light_mode = light::LightMode::Off, std::uint8_t red = 0, std::uint8_t green = 0, std::uint8_t blue = 0);

        void onStateChange(manager::ManagerState state) override;

        void onRawTofMeasurement(std::vector<measurement::TofMeasurement> measurement_vec) override;

        void onTransformedTofMeasurement(std::vector<measurement::TofMeasurement> measurement_vec) override;

        void onThermalMeasurement(std::vector<measurement::ThermalMeasurement> measurement_vec) override;

        void onOutputLog(const logger::LogVerbosity verbosity, const std::string msg) override;

    private:

        void stopThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Request> request,
                                    std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Response> response);
        void startThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Request> request,
                                    std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Response> response);
        
        std::unique_ptr<manager::MeasurementManager> _manager;

        sensor_msgs::msg::PointCloud2 _pc2_msg_raw;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _pointcloud_pub_raw;

        sensor_msgs::msg::PointCloud2 _pc2_msg_transformed;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _pointcloud_pub_transformed;

        std::vector<sensor_msgs::msg::PointCloud2> _pc2_msg_individual_vec;
        std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>> _pointcloud_pub_individual_vec;

        std::vector<std::shared_ptr<sensor_msgs::msg::Image>> _img_msg_vec;
        std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> _img_pub_vec;

        std::vector<std::shared_ptr<sensor_msgs::msg::Image>> _colorimg_msg_vec;
        std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> _colorimg_pub_vec;

    };
};

};
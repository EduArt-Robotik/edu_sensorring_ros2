


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "edu_sensorring_ros2/srv/start_thermal_calibration.hpp"
#include "edu_sensorring_ros2/srv/stop_thermal_calibration.hpp"

#include <sensorring/MeasurementManager.hpp>

#include <vector>
#include <memory>
#include <string>


namespace sensorring{

    class SensorRingProxy : public rclcpp::Node, eduart::manager::MeasurementObserver{
    public:
        SensorRingProxy(std::string node_name);

        ~SensorRingProxy();

        bool run(eduart::manager::ManagerParams params, std::string tf_name);

        bool isShutdown();

        void onStateChange(const eduart::manager::WorkerState state) override;

        void onTofMeasurement(const eduart::measurement::TofMeasurement measurement) override;

        void onThermalMeasurement(const std::size_t idx, const eduart::measurement::ThermalMeasurement measurement) override;

        void onOutputLog(const eduart::logger::LogVerbosity verbosity, const std::string msg) override;

    private:

        void stopThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Request> request,
                                    std::shared_ptr<edu_sensorring_ros2::srv::StopThermalCalibration::Response> response);
        void startThermalCalibration(const std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Request> request,
                                    std::shared_ptr<edu_sensorring_ros2::srv::StartThermalCalibration::Response> response);
        
        bool _shutdown;
        std::unique_ptr<eduart::manager::MeasurementManager> _manager;

        sensor_msgs::msg::PointCloud2 _pc2_msg;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _pointcloud_pub;

        std::vector<std::shared_ptr<sensor_msgs::msg::Image>> _img_msg_vec;
        std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> _img_pub_vec;

        std::vector<std::shared_ptr<sensor_msgs::msg::Image>> _colorimg_msg_vec;
        std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> _colorimg_pub_vec;

    };
};
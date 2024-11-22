


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensorring_ros2/srv/start_thermal_calibration.hpp"
#include "sensorring_ros2/srv/stop_thermal_calibration.hpp"

#include <sensorring/MeasurementManager.hpp>

#include <vector>
#include <memory>
#include <string>


namespace sensorring{

class SensorRingProxy : public rclcpp::Node, MeasurementObserver{
public:
    SensorRingProxy(std::string node_name);

    ~SensorRingProxy();

    int run(manager::ManagerParams params);

    bool isShutdown();

    void onStateChange(const WorkerState state) override;

    void onTofMeasurement(const measurement::TofMeasurement measurement) override;

    void onThermalMeasurement(const std::size_t idx, const measurement::ThermalMeasurement measurement) override;

    void onOutputLog(const LogVerbosity verbosity, const std::string msg) override;

private:

    void stopThermalCalibration(const std::shared_ptr<sensorring_ros2::srv::StopThermalCalibration::Request> request,
                                std::shared_ptr<sensorring_ros2::srv::StopThermalCalibration::Response> response);
    void startThermalCalibration(const std::shared_ptr<sensorring_ros2::srv::StartThermalCalibration::Request> request,
                                std::shared_ptr<sensorring_ros2::srv::StartThermalCalibration::Response> response);
    
    bool _shutdown;
    std::unique_ptr<manager::MeasurementManager> _manager;

    sensor_msgs::msg::PointCloud2 _pc2_msg;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _pointcloud_pub;

    std::vector<std::shared_ptr<sensor_msgs::msg::Image>> _img_msg_vec;
    std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> _img_pub_vec;

    std::vector<std::shared_ptr<sensor_msgs::msg::Image>> _colorimg_msg_vec;
    std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> _colorimg_pub_vec;

}; // class MeasurementManager

}; // namespace MeasurementManager
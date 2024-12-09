cmake_minimum_required(VERSION 3.8)

#########################################################
# build type (has to be done before project() call)
IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
ELSE()
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#########################################################
# project
set(CMAKE_PROJECT_VERSION_MAJOR 1)
set(CMAKE_PROJECT_VERSION_MINOR 0)
set(CMAKE_PROJECT_VERSION_PATCH 0)
set(CMAKE_PROJECT_VERSION "${CMAKE_PROJECT_VERSION_MAJOR}.${CMAKE_PROJECT_VERSION_MINOR}.${CMAKE_PROJECT_VERSION_PATCH}")

project(edu_sensorring_ros2
  LANGUAGES CXX C
  VERSION ${CMAKE_PROJECT_VERSION}
  HOMEPAGE_URL "https://github.com/EduArt-Robotik/edu_sensorring_ros2"
)

#########################################################
# build settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#########################################################
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensorring 1.0.0 REQUIRED)

#########################################################
# generate services
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartThermalCalibration.srv"
  "srv/StopThermalCalibration.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

#########################################################
# generate node
add_executable(${PROJECT_NAME}_node
  src/sensorring_node.cpp
  src/SensorRingProxy.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  include/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  ${cpp_typesupport_target}
  sensorring::sensorring
)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs geometry_msgs tf2_ros tf2 sensorring)

#########################################################
# install node
install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
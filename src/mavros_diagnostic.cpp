#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <map>

void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
  for (const auto& status : msg->status) {
    std::string name = status.name;
    std::string message = status.message;
    std::string hardware_id = status.hardware_id;
    int level = status.level;

    std::map<std::string, std::string> values;
    for (const auto& value : status.values) {
      values[value.key] = value.value;
    }

    if (name.find("FCU connection") != std::string::npos) {
      ROS_INFO("FCU state: %s", message.c_str());
    }

    if (name.find("Heartbeat") != std::string::npos) {
      ROS_INFO("Hearbeat Frequency: %s", values["Frequency (Hz)"].c_str());
      /* auto it = values.find("Frequency"); */
      /* if (it != values.end()) { */
      /*   std::string frequency = it->second; */
      /*   ROS_INFO("Heartbeat Frequency: %s", frequency.c_str()); */
      /* } */
    }

    if (name.find("System") != std::string::npos) {
      /* auto it = values["3D gyro"]; */
      /* std::cout<< it << std::endl; */
      ROS_INFO("3D gyro: %s", values["3D gyro"].c_str());
      ROS_INFO("3D accelerometer: %s", values["3D accelerometer"].c_str());
      ROS_INFO("3D magnetometer: %s", values["3D magnetometer"].c_str());
      ROS_INFO("GPS: %s", values["GPS"].c_str());
      ROS_INFO("rc receiver: %s", values["rc receiver"].c_str());
      ROS_INFO("AHRS: %s", values["AHRS subsystem health"].c_str());
      ROS_INFO("Battery: %s", values["Battery"].c_str());
      ROS_INFO("pre-arm check: %s", values["pre-arm check status. Always healthy when armed"].c_str());
      ROS_INFO("CPU Load: %s", values["CPU Load (%)"].c_str());
    }

    if (name.find("GPS") != std::string::npos) {
      ROS_INFO("GPS Satellites Visible: %s", values["Satellites visible"].c_str());
    }

    if (name.find("Battery") != std::string::npos) {
      ROS_INFO("Voltage battery: %s", values["Voltage"].c_str());
      ROS_INFO("Current battery: %s", values["Current"].c_str());
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavros_diagnostic");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/uav49/diagnostics", 10, callback);
  ros::spin();
  return 0;
}


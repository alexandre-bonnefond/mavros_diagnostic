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
      print("FCU state: %s", message.c_str());
    }

    if (name.find("Heartbeat") != std::string::npos) {
      print("Heartbeat Frequency: %s", values["Frequency (Hz)"].c_str());
      /* auto it = values.find("Frequency"); */
      /* if (it != values.end()) { */
      /*   std::string frequency = it->second; */
      /*   print("Heartbeat Frequency: %s", frequency.c_str()); */
      /* } */
    }

    if (name.find("System") != std::string::npos) {
      /* auto it = values["3D gyro"]; */
      /* std::cout<< it << std::endl; */
      print("3D gyro: %s", values["3D gyro"].c_str());
      print("3D accelerometer: %s", values["3D accelerometer"].c_str());
      print("3D magnetometer: %s", values["3D magnetometer"].c_str());
      print("GPS: %s", values["GPS"].c_str());
      print("rc receiver: %s", values["rc receiver"].c_str());
      print("AHRS: %s", values["AHRS subsystem health"].c_str());
      print("Battery: %s", values["Battery"].c_str());
      print("pre-arm check: %s", values["pre-arm check status. Always healthy when armed"].c_str());
      print("CPU Load: %s", values["CPU Load (%)"].c_str());
    }

    if (name.find("GPS") != std::string::npos) {
      print("GPS Satellites Visible: %s", values["Satellites visible"].c_str());
    }

    if (name.find("Battery") != std::string::npos) {
      print("Voltage battery: %s", values["Voltage"].c_str());
      print("Current battery: %s", values["Current"].c_str());
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


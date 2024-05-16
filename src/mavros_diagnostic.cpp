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

    if (name.find("heartbeat") != std::string::npos || name.find("GPS") != std::string::npos) {
      ROS_INFO("Name: %s", name.c_str());
      ROS_INFO("Message: %s", message.c_str());
      ROS_INFO("Hardware ID: %s", hardware_id.c_str());
      ROS_INFO("Level: %d", level);

      std::stringstream ss;
      for (const auto& kv : values) {
        ss << kv.first << ": " << kv.second << ", ";
      }
      ROS_INFO("Values: %s", ss.str().c_str());
    }

    // Example: Specific handling for heartbeat and GPS
    if (name.find("heartbeat") != std::string::npos) {
      auto it = values.find("Frequency");
      if (it != values.end()) {
        std::string frequency = it->second;
        ROS_INFO("Heartbeat Frequency: %s", frequency.c_str());
      }
    }

    if (name.find("GPS") != std::string::npos) {
      auto it = values.find("Satellites visible");
      if (it != values.end()) {
        std::string satellites_visible = it->second;
        ROS_INFO("GPS Satellites Visible: %s", satellites_visible.c_str());
      }
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


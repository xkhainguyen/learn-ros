#include <string>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

ros::ServiceClient client{};
float accel_z{0.0};

void stop_call_back(const std_msgs::Float32 &msg)
{
  std_srvs::SetBool service;
  float range{msg.data};
  ROS_INFO_STREAM("accel_z: " << accel_z);
  if (range < 1.0 || range > 50.0)
    if (accel_z > 10.0)
    {
      service.request.data = false;
      client.call(service);
    }
}

void imu_call_back(const sensor_msgs::Imu &msg)
{
  accel_z = msg.linear_acceleration.z;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "stop_node");
  ros::NodeHandle nodeHandle("~");

  client = 
    nodeHandle.serviceClient<std_srvs::SetBool>("/smb_highlevel_controller/e_switch");
  ros::Subscriber sub = nodeHandle.subscribe("/smb_highlevel_controller/current_range", 1, stop_call_back);  
  ros::Subscriber sub2 = nodeHandle.subscribe("/imu0", 1, imu_call_back);  
  ros::spin();
  return 0;
}

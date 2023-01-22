#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  enabled = true;

  std::string topic_name{};
	int queue_size{};
  std::string sv_name{};

  nodeHandle_.getParam("topic_name", topic_name);
  nodeHandle_.getParam("queue_size", queue_size);
  nodeHandle_.getParam("gain", gain);		
  nodeHandle_.getParam("sv_name", sv_name);	

  sub = nodeHandle_.subscribe(topic_name, queue_size, &SmbHighlevelController::ScanCallback, this);
  pub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pub_range = nodeHandle_.advertise<std_msgs::Float32>("current_range", 1);
  vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  service = nodeHandle_.advertiseService(sv_name, &SmbHighlevelController::SwitchEmergency, this);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::ScanCallback(const sensor_msgs::LaserScan& msg)
{
  auto it{std::min_element(msg.ranges.begin(), msg.ranges.end())};
  long int index{std::distance(msg.ranges.begin(), it)};
  float angle{index * msg.angle_increment + msg.angle_min};
  angle = std::max(msg.angle_min, std::min(angle, msg.angle_max));
  float range{*it};

  // ROS_INFO_STREAM("Range: " << range << "\n");
  std_msgs::Float32 range_msg{};
  range_msg.data = range;
  pub_range.publish(range_msg);

  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.linear.x = 0.0;
  vel.angular.z = 0.0;  

  if (enabled)
    if (range > 0.0)
    {
      vel.linear.x = 5.0;
      vel.angular.z = gain * angle;
    }

  pub.publish(vel);
  SmbHighlevelController::PubMarker(range, angle);
}  

bool SmbHighlevelController::SwitchEmergency(std_srvs::SetBool::Request &request,
						 std_srvs::SetBool::Response &response)
{
  enabled = request.data;
  response.success = true;
  response.message = "No error";
  return true;
}

void SmbHighlevelController::PubMarker(const float& range, const float& angle)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "pillar";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = range * cos(angle);
  marker.pose.position.y = range * sin(angle);
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  vis_pub.publish(marker);
}

} /* namespace */

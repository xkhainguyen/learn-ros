#ifndef SMBHIGHLEVELCONTROLLER_HPP
#define SMBHIGHLEVELCONTROLLER_HPP

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);
	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

	/*!
	 * Callback and Control.
	 */
	void ScanCallback(const sensor_msgs::LaserScan& msg);
	void PubMarker(const float& range, const float& angle);
	bool SwitchEmergency(std_srvs::SetBool::Request &request,
						 std_srvs::SetBool::Response &response);
private:
	ros::NodeHandle nodeHandle_;
	float gain;	
	bool enabled;
	geometry_msgs::Twist vel;
	ros::Publisher pub; 
	ros::Publisher pub_range; 
	ros::Publisher vis_pub;
	ros::Subscriber sub;	
	ros::ServiceServer service;
};
} /* namespace */

#endif
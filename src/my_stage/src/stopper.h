#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class stopper {
public:
	const static double FORWARD_SPEED_MPS = 0.5;
	const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
	const static float MIN_PROXIMITY_RANGE_M = 0.5;    

	stopper();
	void startMoving();

private:
	ros::NodeHandle node;
	ros::Publisher commandPub;
	ros::Subscriber laserSub;
	bool keepMoving;
 
	void moveForward();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

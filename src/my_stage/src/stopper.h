#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class stopper {
public:
	const static double FORWARD_SPEED_MPS = 0.5;
	const static double TURN_SPEED_MPS = 1.57; //90 degrees equals 1.57 rad
	const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
	const static float MIN_PROXIMITY_RANGE_M = 0.5;    
	float current_angle = 0;
	const static float final_angle = 1.8; //..............FIX THIS FOR TURN
	int t1, t0;

	stopper();
	void startMoving();

private:
	ros::NodeHandle node;
	ros::Publisher commandPub;
	ros::Subscriber laserSub;
	bool keepMoving;
	int edgeTraveled;
	int counter;
	float distanceOfWall;
 
	void moveForward();
	void turnLeft();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

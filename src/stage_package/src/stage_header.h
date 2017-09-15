#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class stage_class {
public:
	const static double FORWARD_SPEED_MPS = 0.5;
	const static double TURN_SPEED_MPS = 1.57; //90 degrees equals 1.57 rad calculated by speed *2*PI/360, 0.0087
	const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
	const static float MIN_PROXIMITY_RANGE_M = 0.5;    
	float current_angle;
	const static float final_angle = 1.8; //..............FIX THIS FOR TURN
	int t1, t0;

	stage_class();
	void start();

private:
	ros::NodeHandle nodeHandle;
	ros::Publisher messagePublisher;
	ros::Subscriber laserSubscriber;
	bool flagToMove;
	int edgeTraveled;
	int counter;
	float distanceOfWall;
 
	void runForward();
	void turnLeft();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);
};

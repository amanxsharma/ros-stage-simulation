#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class stage_class {
public:
	const static double FORWARD_SPEED_MPS = 0.5;
	const static double TURN_SPEED_MPS = 0.1; //9 0.0087
    
	float current_angle;
	const static float final_angle = 1.57; //90 degrees equals 1.57 rad
	double t1, t0;

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

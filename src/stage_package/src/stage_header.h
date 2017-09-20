#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class stage_class {
public:
	const static double FORWARD_SPEED_MPS = 0.1;
	const static double TURN_SPEED_MPS = -0.045; //-0.005 with 1.75
    
	float current_angle;
	const static float final_angle = 1.75; //-90 degrees equals -1.57 rad
	double t1, t0, dt0, dt1;

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
	float distanceTraveled;
 
	void runForward();
	void turn();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);
};

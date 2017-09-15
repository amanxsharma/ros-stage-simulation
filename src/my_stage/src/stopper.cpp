#include "stopper.h"
#include "geometry_msgs/Twist.h"

stopper::stopper()
{
	keepMoving = true;
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	laserSub = node.subscribe("base_scan", 1, &stopper::scanCallback, this);
}

void stopper::moveForward(){
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
}

void stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	
	for(int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++){
		if(scan->ranges[currIndex] < closestRange){
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if(closestRange < MIN_PROXIMITY_RANGE_M){
		ROS_INFO("Stop!");
		keepMoving = false;
	}
}

void stopper::startMoving(){
	ros::Rate rate(10);
	ROS_INFO("Start Moving");

	while(ros::ok() && keepMoving){
		moveForward();
		ros::spinOnce();

		rate.sleep();
	}
}


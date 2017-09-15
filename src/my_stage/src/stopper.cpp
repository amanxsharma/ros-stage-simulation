#include "stopper.h"
#include "geometry_msgs/Twist.h"

stopper::stopper()
{
	keepMoving = true;
	edgeTraveled = 0;
	counter = 0;
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	laserSub = node.subscribe("base_scan", 1, &stopper::scanCallback, this);
}

void stopper::moveForward(){
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
}

void stopper::turnLeft(){
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0; msg.linear.y = 0.0;
	msg.angular.z = TURN_SPEED_MPS;
	t0 = ros::Time::now().toSec();
	
	while(current_angle < final_angle){
		commandPub.publish(msg);
		t1 = ros::Time::now().toSec();	
		current_angle = TURN_SPEED_MPS * (t1 - t0);
		//ROS_INFO_STREAM("t0 = "<<t0<<" t1 = "<<t1);
		//ROS_INFO_STREAM("Current Angle = "<<current_angle);
	}
	edgeTraveled++;
	keepMoving = true;
	counter = 0;

}

void stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	
	if(counter == 0){
		distanceOfWall = scan->ranges[minIndex]; //......
		counter++;
	}
	float closestRange = scan->ranges[minIndex];
	
	for(int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++){
		if(scan->ranges[currIndex] < closestRange){
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange <<" distanceOfWall: "<<distanceOfWall);

	//if(closestRange < MIN_PROXIMITY_RANGE_M){
	if(closestRange < distanceOfWall / 2){
		ROS_INFO("Stop!");
		keepMoving = false;
		ROS_INFO("Turn Left!");
		turnLeft();
		
	}
}

void stopper::startMoving(){
	ros::Rate rate(10);
	ROS_INFO("Start Moving");

	while(ros::ok() && keepMoving && edgeTraveled < 4){
		moveForward();
		ros::spinOnce();

		rate.sleep();
	}
}


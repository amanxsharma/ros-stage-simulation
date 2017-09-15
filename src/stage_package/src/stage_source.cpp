#include "stage_header.h"
#include "geometry_msgs/Twist.h"

stage_class::stage_class(){
	flagToMove = true;
	edgeTraveled = 0;
	counter = 0;
	current_angle = 0;
	messagePublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	laserSubscriber = nodeHandle.subscribe("base_scan", 1, &stage_class::scanCallback, this);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "stage_node"); //create a node

	stage_class stg;
	stg.start();
	return 0;
}

void stage_class::start(){
	ros::Rate rate(10);
	ROS_INFO("Measurement done, Start Moving Forward");

	while(ros::ok() && flagToMove && edgeTraveled < 4){
		runForward();
		ros::spinOnce();

		rate.sleep();
	}
}

void stage_class::runForward(){
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	messagePublisher.publish(msg);
}

void stage_class::turnLeft(){
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0; msg.linear.y = 0.0;
	msg.angular.z = TURN_SPEED_MPS;
	t0 = ros::Time::now().toSec();
	
	while(current_angle < final_angle){
		messagePublisher.publish(msg);
		t1 = ros::Time::now().toSec();	
		current_angle = TURN_SPEED_MPS * (t1 - t0);
		//ROS_INFO_STREAM("t0 = "<<t0<<" t1 = "<<t1);
		//ROS_INFO_STREAM("Current Angle = "<<current_angle);
	}
	edgeTraveled++;
	flagToMove = true;
	counter = 0;
	current_angle = 0;

}

void stage_class::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan) {

	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - laserScan->angle_min) / laserScan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - laserScan->angle_min) / laserScan->angle_increment);
	
	if(counter == 0){
		distanceOfWall = laserScan->ranges[minIndex]; //get distance from wall, only once for each wall
		counter++;
	}
	float closestRange = laserScan->ranges[minIndex];
	
	for(int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++){
		if(laserScan->ranges[currIndex] < closestRange){
			closestRange = laserScan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange <<" distanceOfWall: "<<distanceOfWall);

	//if(closestRange < MIN_PROXIMITY_RANGE_M){
	if(closestRange < distanceOfWall / 2){
		ROS_INFO("Stop!");
		flagToMove = false;
		ROS_INFO("Turn Left!");
		turnLeft();
		
	}
}





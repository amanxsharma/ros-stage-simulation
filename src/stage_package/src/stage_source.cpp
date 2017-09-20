#include "stage_header.h"
#include "geometry_msgs/Twist.h"

stage_class::stage_class(){  //Class Constructor- All definitions of variables, publisher and subscriber is done here
	flagToMove = true;
	edgeTraveled = 0;
	counter = 0;
	current_angle = 0;
	
	laserSubscriber = nodeHandle.subscribe("base_scan", 1, &stage_class::scanCallback, this);
	messagePublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	
	
}

int main(int argc, char **argv){  //initializes a node ‘stage_node’ and call start()
	ros::init(argc, argv, "stage_node"); //create a node

	stage_class stg;	
	
	stg.start();
	
	return 0;
}

void stage_class::start(){ //call runForward() if finds conditions satisfied

	ros::Rate rate(10);
	ROS_INFO("Measurement done, Start Moving Forward");

	while(ros::ok() && flagToMove && edgeTraveled < 4){
				
		runForward();
		ros::spinOnce();

		rate.sleep();
	}
}

void stage_class::runForward(){ //Publishes command to move robot forward
	//ROS_INFO("runForward()");	
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	messagePublisher.publish(msg);
}

void stage_class::turn(){ //to turn robot by 90 degrees in clockwise direction
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0; msg.linear.y = 0.0; msg.angular.z = TURN_SPEED_MPS;

	t0 = ros::Time::now().toSec();
	
	while(-1*current_angle < final_angle){
		//ROS_INFO_STREAM("current: "<<current_angle);
		messagePublisher.publish(msg);
		t1 = ros::Time::now().toSec();	
		current_angle = TURN_SPEED_MPS * (t1 - t0);
		//ROS_INFO_STREAM("current: "<<current_angle);
	}
	msg.angular.z = 0.0; msg.linear.x = 0.0; msg.linear.y = 0.0;
	messagePublisher.publish(msg);

	flagToMove = true;
	counter = 0;
	current_angle = 0;

}

//function as a callback for laserScan to check if the robot has travelled half of the distance from the wall
void stage_class::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan) {
	
	int size = laserScan->ranges.size();
	if(counter == 0){		
		distanceOfWall = laserScan->ranges[size/2]; //get distance from wall, only once for each wall
		dt0 = ros::Time::now().toSec();
		counter++;
	}	

	ROS_INFO_STREAM("distanceOfWall: "<<distanceOfWall <<" distanceTraveled: "<<distanceTraveled);
	distanceTraveled = 0;
	dt1 = ros::Time::now().toSec();
	distanceTraveled = FORWARD_SPEED_MPS * (dt1 - dt0);
	
	if(distanceTraveled > distanceOfWall/2){
		
		ROS_INFO("Stop!");
		flagToMove = false;
		edgeTraveled++;
		if(edgeTraveled < 4) {
		ROS_INFO("Turn Left!");
		turn();
		
		}
	
	}
}

